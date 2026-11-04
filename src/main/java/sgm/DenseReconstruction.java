package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import java.util.concurrent.*;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

/** Rectification, Java SGM, world-space triangulation and depth-buffer consistency fusion. */
final class DenseReconstruction {
  private final Options options;
  private final List<ImageFrame> frames;
  private final DepthFusion fusion;
  private long fusionRejected, accepted, angleRejected, candidates;
  private long consensusRejected, uncertaintyRejected, refinedPoints;
  private long visibilityRejected, matchingConfidenceRejected;
  private long surfaceMerged, surfaceMoved, beforeSurface, voxelRejected, confidenceRejected;
  private long twoViewExtraRejected;
  private long rawCandidates, provisionalRejected;
  // --on-pair-failure skip bookkeeping; candidatesWritten guards the ordered candidate stream.
  private boolean candidatesWritten;
  private int attemptedPairs;
  private final List<Object> skippedPairs = new ArrayList<>();
  private final List<Long> pairStarts = new ArrayList<>();
  private final List<String> quality = new ArrayList<>();
  private final Project project;
  private final Crs crs;
  private final List<FeatureTracks.Track> tracks;
  private WindowedRasters windowedStore;
  private ExecutorService pairPreparer;
  private Future<PairPixels> pendingPair;
  private int pendingA=-1,pendingB=-1,pairIndex;
  private List<FeatureTracks.Pair> stereoPairs;
  private long preparedPairs,serialPairs,retriedPairs;
  private final LinkedHashMap<Integer, WindowedRasters.Image> windowedSources =
      new LinkedHashMap<>(8, .75f, true);

  DenseReconstruction(Options options, List<ImageFrame> frames) {
    this(options, frames, null);
  }

  DenseReconstruction(Options options, List<ImageFrame> frames, Project project) {
    this(options, frames, project, null);
  }

  DenseReconstruction(Options options, List<ImageFrame> frames, Project project, Crs crs) {
    this(options, frames, project, crs, Collections.emptyList());
  }

  DenseReconstruction(
      Options options,
      List<ImageFrame> frames,
      Project project,
      Crs crs,
      List<FeatureTracks.Track> tracks) {
    this.options = options;
    this.frames = frames;
    this.project = project;
    this.crs = crs;
    this.tracks = tracks;
    Camera[] cameras = new Camera[frames.size()];
    int[] widths = new int[frames.size()], heights = new int[frames.size()];
    for (ImageFrame f : frames) {
      cameras[f.id] = f.camera;
      widths[f.id] = f.width;
      heights[f.id] = f.height;
    }
    fusion = new DepthFusion(cameras, widths, heights, options);
  }

  long run(List<FeatureTracks.Pair> pairs) throws IOException {
    pairs = DensePairSelection.select(frames, pairs, tracks, options);
    stereoPairs = pairs;
    if(options.stereoPipeline && !options.diagnostics && options.cores>1)
      pairPreparer=Executors.newSingleThreadExecutor(r -> new Thread(r,"stereo-prepare"));
    Path candidates = Files.createTempFile(options.output, "candidates-", ".bin");
    Path preparedResults = null;
    long count = 0;
    boolean completed = false;
    try (IncrementalFusion incremental =
        options.fusionMode.equals("incremental") ? new IncrementalFusion(options, fusion) : null) {
      if (incremental != null)
        preparedResults = Files.createTempFile(options.output, "prepared-fusion-", ".bin");
      try (DataOutputStream out =
          new DataOutputStream(
              new BufferedOutputStream(
                  incremental == null
                      ? Files.newOutputStream(candidates)
                      : incremental.stream()))) {
        if (options.denseMethod.equals("patchmatch"))
          count =
              new PatchMatchReconstruction(
                      options, frames, tracks, fusion, project == null ? null : project.data)
                  .run(pairs, out);
        else {
          // Decode the non-windowed pairs' images in parallel up front (identical rasters).
          List<ImageFrame> needed = new ArrayList<>();
          for (FeatureTracks.Pair pair : pairs) {
            ImageFrame a = frames.get(pair.a), b = frames.get(pair.b);
            if (!a.camera.registered || !b.camera.registered || windowed(a, b)) continue;
            if (a.camera.denseExcluded || b.camera.denseExcluded) continue;
            needed.add(a);
            needed.add(b);
          }
          long preloadStarted = System.nanoTime();
          int preloaded = RasterCache.preload(needed, options.cores);
          if (preloaded > 0) RunTelemetry.record("Parallel image preload (" + preloaded + ")", preloadStarted);
        }
        if (!options.denseMethod.equals("patchmatch"))
          try (WorkProgress densePairs = WorkProgress.start("Dense matching pairs", pairs.size(), "pairs")) {
          int inFlight = pairsInFlight(pairs);
          if (inFlight > 1) count = parallelPairs(pairs, out, densePairs, count, inFlight);
          else
          for (pairIndex=0;pairIndex<pairs.size();pairIndex++) {
            FeatureTracks.Pair pair=pairs.get(pairIndex);
            densePairs.update(pairIndex);
            densePairs.detail("pair " + pair.a + "-" + pair.b);
            if (!frames.get(pair.a).camera.registered || !frames.get(pair.b).camera.registered)
              continue;
            if (frames.get(pair.a).camera.denseExcluded || frames.get(pair.b).camera.denseExcluded)
              continue;
            int source = fusion.addPair(pair.a, pair.b);
            // Candidates are streamed pair by pair: pair `source` owns [start, next start).
            while (pairStarts.size() <= source) pairStarts.add(count);
            long n;
            candidatesWritten = false;
            attemptedPairs++;
            try (WorkProgress work =
                WorkProgress.start(
                    "Rectify/match/triangulate pair " + pair.a + "-" + pair.b, 0, "pixels")) {
              n = pair(frames.get(pair.a), frames.get(pair.b), source, count, out);
              work.done();
            } catch (RuntimeException | IOException failure) {
              // A pair that already streamed candidates cannot be skipped without corrupting the
              // ordered candidate stream; its partial evidence would also remain.
              if (!options.onPairFailure.equals("skip")
                  || candidatesWritten
                  || Thread.currentThread().isInterrupted()) throw failure;
              skippedPairs.add(
                  Json.object(
                      "fusion_pair_id", source, "image_a", pair.a, "image_b", pair.b,
                      "error", String.valueOf(failure.getMessage()),
                      "type", failure.getClass().getSimpleName()));
              System.err.printf(
                  Locale.ROOT, "Warning: skipping failed pair %d-%d: %s%n", pair.a, pair.b, failure);
              if (project != null)
                project.event(
                    "stereo_pair_failed",
                    Json.object("fusion_pair_id", source, "image_a", pair.a, "image_b", pair.b,
                        "error", String.valueOf(failure.getMessage())));
              continue;
            }
            count += n;
            if (project != null)
              project.event(
                  "stereo_pair_complete",
                  Json.object(
                      "fusion_pair_id",
                      source,
                      "image_a",
                      pair.a,
                      "image_b",
                      pair.b,
                      "sampled_candidates",
                      n));
            System.out.printf(Locale.ROOT, "Dense pair %d-%d: %,d candidates%n", pair.a, pair.b, n);
          }
          densePairs.done();
          }
      }
      if (!skippedPairs.isEmpty()) {
        Files.createDirectories(options.output.resolve("05_stereo"));
        Json.write(
            options.output.resolve("05_stereo/skipped_pairs.json"),
            Json.object("policy", options.onPairFailure, "attempted", attemptedPairs, "skipped", skippedPairs));
        if (skippedPairs.size() >= attemptedPairs)
          throw new IOException("Every stereo pair failed; see 05_stereo/skipped_pairs.json");
      }
      this.candidates = count;
      rawCandidates = count;
      Files.write(options.output.resolve("stereo-quality.csv"), quality);
      if (project != null) {
        project.stage(
            "05_stereo",
            "complete",
            "Rectification, filtering stages and confidence rasters exported");
        project.stage("06_fusion", "running", "Independent stereo-pair depth corroboration");
        long exportStarted = System.nanoTime();
        fusion.export(project.data);
        RunTelemetry.record("Fusion depth evidence export", exportStarted);
      }
      if (count == 0)
        throw new IOException(
            "No dense candidates; check disparity range, camera calibration, and baseline");
      boolean imageFusion = options.fusionMode.equals("image");
      // Image-space fusion reads candidates directly; the per-candidate evidence backend is unused.
      if (!imageFusion)
        try (WorkProgress work = WorkProgress.start("Prepare fusion evidence/backend", 0, "pixels")) {
          fusion.prepare(count);
          work.done();
        }
      if (incremental != null) {
        count = incremental.finish(candidates, preparedResults);
        provisionalRejected = incremental.rejectedHypotheses();
        System.out.printf(
            Locale.ROOT,
            "Incremental fusion: %,d final representatives; all pair evidence considered before"
                + " publication%n",
            count);
      }
      System.out.printf(
          Locale.ROOT, "Fusion: processing %,d candidates from stereo depth evidence%n", count);
      long fusionProgress = System.nanoTime();
      long kept;
      long fusionOutputNanos = 0, outputStarted = 0;
      long normalNanos = 0;
      var twoViewDiagnostics = new TwoViewConfidence.Diagnostics(options);
      if (imageFusion) {
        try (WorkProgress fusionWork = WorkProgress.start("Image-space fusion", 0, "pixels");
            FusionOutput output = new FusionOutput(options, fusion, project, crs)) {
          long[] starts = new long[fusion.pairCount() + 1];
          for (int k = 0; k < starts.length; k++)
            starts[k] = k < pairStarts.size() ? pairStarts.get(k) : count;
          ImageFusion.Counters c =
              ImageFusion.run(options, fusion, candidates, starts, output, twoViewDiagnostics);
          fusionWork.done();
          long finishStarted = System.nanoTime();
          output.finish();
          RunTelemetry.record(
              "Fusion output finish (deferred normals, consolidation, LAS/PLY)", finishStarted);
          kept = output.count();
          beforeSurface = output.stagedCount();
          surfaceMerged = output.mergedCount();
          surfaceMoved = output.movedCount();
          voxelRejected = output.voxelRejected();
          confidenceRejected = output.confidenceRejected();
          fusionRejected = c.rejected();
          consensusRejected = c.ambiguous;
          visibilityRejected = c.visibility;
          uncertaintyRejected = c.uncertainty;
          matchingConfidenceRejected = c.twoView;
          refinedPoints = c.refined;
          Json.write(options.output.resolve("06_fusion/image_fusion.json"), c.json());
          System.out.println("Image-space fusion: " + Json.string(c.json()));
        }
      } else
      try (DataInputStream in =
              new DataInputStream(new BufferedInputStream(Files.newInputStream(candidates)));
          DataInputStream prepared =
              preparedResults == null
                  ? null
                  : new DataInputStream(
                      new BufferedInputStream(Files.newInputStream(preparedResults)));
          WorkProgress fusionWork =
              WorkProgress.start("Fusion candidates / normal estimation", count, "candidates");
          FusionOutput output = new FusionOutput(options, fusion, project, crs);
          FusionPipeline pipeline =
              new FusionPipeline(
                  in,
                  count,
                  fusion,
                  options.fusionPipeline,
                  (options.fusionEmission.equals("pair")
                      && (options.surfaceConsolidation.equals("normal")
                          || options.pointAttributes.equals("surface"))),
                  true,
                  prepared)) {
        {
          int capacity = pipeline.capacity();
          FusionPipeline.Batch batch = null;
          for (long i = 0; i < count; i++) {
            int offset = (int) (i % capacity);
            if ((i & 4095) == 0) fusionWork.update(i);
            if (offset == 0) {
              if (batch != null) fusionOutputNanos += System.nanoTime() - outputStarted;
              batch = pipeline.next();
              outputStarted = System.nanoTime();
            }
            if ((i & 4095) == 0 && System.nanoTime() - fusionProgress >= 5_000_000_000L) {
              System.out.printf(
                  Locale.ROOT,
                  "Fusion: %.1f%% (%,d/%,d), %,d retained, %,d rejected%n",
                  100.0 * i / count,
                  i,
                  count,
                  output.count(),
                  fusionRejected);
              if (PointConfidence.bidirectional(options))
                System.out.printf(
                    Locale.ROOT,
                    "Experimental two-view gate: %,d additional rejections; final confidence"
                        + " cutoff: %,d rejected%n",
                    twoViewDiagnostics.additionalRejected(),
                    output.confidenceRejected());
              fusionProgress = System.nanoTime();
            }
            double[] p = batch.points()[offset];
            int r = batch.rgb()[offset] >>> 16,
                g = batch.rgb()[offset] >>> 8 & 255,
                b = batch.rgb()[offset] & 255,
                source = batch.source()[offset];
            int pixel = batch.pixel()[offset];
            DepthFusion.Result result = batch.results()[offset];
            long candidateId = batch.candidateIds()[offset];
            twoViewDiagnostics.record(result);
            int views = result.views;
            if (project != null) project.data.fusionRefinement(candidateId, source, p, result);
            if (!result.accepted()) {
              fusionRejected++;
              if (result.status.equals("depth_consensus_rejected")) consensusRejected++;
              if (result.status.equals("depth_uncertainty_rejected")) uncertaintyRejected++;
              if (result.status.equals("visibility_contradiction_rejected")) visibilityRejected++;
              if (result.status.startsWith("two_view_")) matchingConfidenceRejected++;
              if (project != null)
                project.data.decision(candidateId, source, pixel, views, result.status);
              continue;
            }
            if (result.shift > 1e-8) refinedPoints++;
            output.add(
                candidateId,
                source,
                pixel,
                (r << 16) | (g << 8) | b,
                result,
                batch.surface()[offset]);
          }
          if (batch != null) fusionOutputNanos += System.nanoTime() - outputStarted;
        }
        fusionWork.update(count);
        fusionWork.done();
        fusionWork.close();
        pipeline.close();
        pipeline.recordTimings();
        RunTelemetry.recordTotal("Fusion batches: candidate output staging", fusionOutputNanos, 0);
        normalNanos = pipeline.normalNanos();
        System.out.printf(
            Locale.ROOT,
            "Fusion: 100.0%% (%,d/%,d), %,d retained, %,d rejected; candidate processing"
                + " complete%n",
            count,
            count,
            output.count(),
            fusionRejected);
        long finishStarted = System.nanoTime();
        output.finish();
        RunTelemetry.record("Fusion output finish (deferred normals, consolidation, LAS/PLY)", finishStarted);
        normalNanos += output.deferredNormalNanos();
        kept = output.count();
        beforeSurface = output.stagedCount();
        surfaceMerged = output.mergedCount();
        surfaceMoved = output.movedCount();
        voxelRejected = output.voxelRejected();
        confidenceRejected = output.confidenceRejected();
      }
      accepted = kept;
      twoViewExtraRejected = twoViewDiagnostics.additionalRejected();
      twoViewDiagnostics.write(options.output);
      if (!imageFusion)
        Json.write(options.output.resolve("06_fusion/backend.json"), fusion.backendReport());
      Json.write(
          options.output.resolve("06_fusion/output_profile.json"),
          Json.object(
              "candidate_output_nanoseconds",
              fusionOutputNanos,
              "normal_preprocessing_nanoseconds",
              normalNanos,
              "compression_level",
              options.outputCompression,
              "pipeline",
              options.fusionPipeline,
              "scope",
              "CPU candidate staging/metadata work; excludes surface consolidation and final"
                  + " LAS/PLY publication; overlaps prefetched validation when enabled"));
      this.candidates = count;
      Files.write(options.output.resolve("stereo-quality.csv"), quality);
      Files.writeString(options.output.resolve("fusion-quality.txt"), summary());
      System.out.println(summary());
      if (project != null) {
        project.data.finishFusion();
        project.stage(
            "06_fusion",
            "complete",
            kept
                + " points retained; "
                + surfaceMerged
                + " redundant observations consolidated; "
                + fusionRejected
                + " rejected by fusion geometry/uncertainty");
        project.stage("07_results", "running", "Publishing final clouds and report");
      }
      completed = true;
      return kept;
    } finally {
      try {
        closePreparation();
      } finally {
        try {
          fusion.close();
          if (completed && !(options.diagnostics && options.storage.equals("debug")))
            fusion.discardStorage();
        } finally {
          try {
            if (windowedStore != null) windowedStore.close();
          } finally {
            Files.deleteIfExists(candidates);
            if (preparedResults != null) Files.deleteIfExists(preparedResults);
          }
        }
      }
    }
  }

  String summary() {
    return "Dense candidates="
        + candidates
        + "\nPairs retried serially after a parallel failure=" + retriedPairs
        + "\nPairs prepared ahead=" + preparedPairs
        + "\nPairs prepared serially=" + serialPairs
        + "\nRaw stereo observations="
        + rawCandidates
        + "\nRejected provisional hypotheses="
        + provisionalRejected
        + "\nFusion rejected="
        + fusionRejected
        + "\nPer-ray depth consensus rejected="
        + consensusRejected
        + "\nDepth uncertainty rejected="
        + uncertaintyRejected
        + "\nVisibility contradiction rejected="
        + visibilityRejected
        + "\nTwo-view matching/conditioning rejected="
        + matchingConfidenceRejected
        + "\nMulti-pair depth refined="
        + refinedPoints
        + "\nAccepted before surface consolidation="
        + beforeSurface
        + "\nSurface observations consolidated="
        + surfaceMerged
        + "\nSurface positions refined="
        + surfaceMoved
        + "\nVoxel duplicates="
        + voxelRejected
        + "\nConfidence threshold rejected="
        + confidenceRejected
        + "\nConfidence threshold="
        + options.confidenceThreshold
        + "\nConfidence model="
        + PointConfidence.MODEL
        + (PointConfidence.bidirectional(options)
            ? "\nExperimental two-view threshold="
                + TwoViewConfidence.effective(options.twoViewConfidenceThreshold)
                + "\nAdditional two-view gate rejected="
                + twoViewExtraRejected
            : "")
        + "\nDense method="
        + options.denseMethod
        + "\nWindowed mode="
        + options.windowedMode
        + "\nFusion mode="
        + options.fusionMode
        + "\nCamera outlier model="
        + options.fusionOutliers
        + "\nMinimum supporting views="
        + fusion.requiredViews()
        + "\nTriangulation angle rejected="
        + angleRejected
        + "\nExported points="
        + accepted
        + "\n";
  }

  /**
   * S1 (--stereo-pairs-in-flight): pairs computed concurrently. Windowed stereo, diagnostics
   * exports and PatchMatch keep the serial path. auto = up to 4, one per 4 cores.
   */
  private int pairsInFlight(List<FeatureTracks.Pair> pairs) {
    int requested =
        options.stereoPairsInFlight > 0 ? options.stereoPairsInFlight : Math.max(1, Math.min(4, options.cores / 4));
    if (requested <= 1 || options.diagnostics || options.cores < 2) return 1;
    for (FeatureTracks.Pair pair : pairs)
      if (windowed(frames.get(pair.a), frames.get(pair.b))) return 1;
    return Math.min(requested, Math.max(1, pairs.size()));
  }

  /**
   * Computes up to {@code inFlight} pairs at once on worker threads (each with its own share of
   * cores and GPU memory, and its own OpenCL session), and publishes them strictly in pair order:
   * fusion evidence, candidate stream, exports, counters and reports are applied exactly as the
   * serial loop applies them. Pairs are registered with the fusion up front, in the serial order.
   */
  private long parallelPairs(
      List<FeatureTracks.Pair> pairs,
      DataOutputStream out,
      WorkProgress progress,
      long count,
      int inFlight)
      throws IOException {
    List<int[]> work = new ArrayList<>();
    for (int i = 0; i < pairs.size(); i++) {
      FeatureTracks.Pair pair = pairs.get(i);
      Camera a = frames.get(pair.a).camera, b = frames.get(pair.b).camera;
      if (!a.registered || !b.registered || a.denseExcluded || b.denseExcluded) continue;
      work.add(new int[] {i, fusion.addPair(pair.a, pair.b)});
    }
    Options base = options.copy();
    base.cores = Math.max(1, options.cores / inFlight);
    try {
      long budget = GpuSgm.budget(GpuSgm.selectedDevice(options), options);
      base.gpuMemoryMb = (int) Math.max(64, budget / 1048576 / inFlight);
    } catch (RuntimeException noGpu) {
      // CPU only: nothing to share.
    }
    System.out.printf(
        Locale.ROOT,
        "Dense matching: %d pairs in flight (%d cores and %d MiB GPU memory each)%n",
        inFlight,
        base.cores,
        base.gpuMemoryMb);
    ExecutorService workers =
        Executors.newFixedThreadPool(
            inFlight,
            r -> {
              Thread t =
                  new Thread(
                      () -> {
                        try {
                          r.run();
                        } finally {
                          GpuSgm.closeSession();
                        }
                      },
                      "stereo-pair");
              t.setDaemon(true);
              return t;
            });
    ArrayDeque<Future<PairCompute>> queue = new ArrayDeque<>();
    int next = 0;
    try {
      for (int done = 0; done < work.size(); done++) {
        while (next < work.size() && queue.size() < inFlight) {
          int[] item = work.get(next++);
          FeatureTracks.Pair pair = pairs.get(item[0]);
          int source = item[1];
          queue.add(
              workers.submit(
                  () -> {
                    ImageFrame a = frames.get(pair.a), b = frames.get(pair.b);
                    if ((a.id + "-" + b.id).equals(System.getProperty("sgm.test.failPair")))
                      throw new IllegalStateException("Injected test failure for pair " + a.id + "-" + b.id);
                    try (WorkProgress task =
                        WorkProgress.start("Rectify/match/triangulate pair " + pair.a + "-" + pair.b, 0, "pixels")) {
                      PairPixels pixels;
                      synchronized (RasterCache.class) {
                        pixels = new PairPixels(rectifyLocked(a, b, false, -1, false, false), options);
                      }
                      try {
                        PairCompute computed = compute(pixels, source, base, false);
                        task.done();
                        return computed;
                      } catch (RuntimeException | IOException | Error e) {
                        pixels.close();
                        throw e;
                      }
                    }
                  }));
        }
        int[] item = work.get(done);
        FeatureTracks.Pair pair = pairs.get(item[0]);
        int source = item[1];
        progress.update(item[0]);
        progress.detail("pair " + pair.a + "-" + pair.b);
        while (pairStarts.size() <= source) pairStarts.add(count);
        candidatesWritten = false;
        attemptedPairs++;
        long n;
        try {
          PairCompute computed;
          try {
            computed = joinPair(queue.poll());
          } catch (RuntimeException | IOException | Error workerFailure) {
            // compute() has no side effects: redo this pair serially on the coordinator.
            System.err.printf(
                Locale.ROOT,
                "Warning: parallel compute of pair %d-%d failed (%s); retrying it serially%n",
                pair.a,
                pair.b,
                workerFailure);
            retriedPairs++;
            PairPixels pixels;
            synchronized (RasterCache.class) {
              pixels = new PairPixels(rectifyLocked(frames.get(pair.a), frames.get(pair.b), false, -1, false, false), options);
            }
            try {
              computed = compute(pixels, source, options, false);
            } catch (RuntimeException | IOException | Error e) {
              pixels.close();
              throw e;
            }
          }
          try {
            // Rectification metadata in pair order, as the one-ahead preparation does.
            if (project != null)
              try (Rectified geometry = rectify(frames.get(pair.a), frames.get(pair.b), false, source, true)) {}
            n = publish(computed, source, count, out);
          } finally {
            computed.pixels().close();
          }
        } catch (RuntimeException | IOException failure) {
          if (!options.onPairFailure.equals("skip")
              || candidatesWritten
              || Thread.currentThread().isInterrupted()) throw failure;
          skippedPairs.add(
              Json.object(
                  "fusion_pair_id", source, "image_a", pair.a, "image_b", pair.b,
                  "error", String.valueOf(failure.getMessage()),
                  "type", failure.getClass().getSimpleName()));
          System.err.printf(Locale.ROOT, "Warning: skipping failed pair %d-%d: %s%n", pair.a, pair.b, failure);
          if (project != null)
            project.event(
                "stereo_pair_failed",
                Json.object("fusion_pair_id", source, "image_a", pair.a, "image_b", pair.b,
                    "error", String.valueOf(failure.getMessage())));
          continue;
        }
        count += n;
        if (project != null)
          project.event(
              "stereo_pair_complete",
              Json.object(
                  "fusion_pair_id", source, "image_a", pair.a, "image_b", pair.b,
                  "sampled_candidates", n));
        System.out.printf(Locale.ROOT, "Dense pair %d-%d: %,d candidates%n", pair.a, pair.b, n);
      }
    } finally {
      workers.shutdownNow();
      for (Future<PairCompute> pending : queue)
        try {
          PairCompute c = pending.get();
          if (c != null) c.pixels().close();
        } catch (Exception ignored) {
        }
      try {
        workers.awaitTermination(10, TimeUnit.MINUTES);
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
      }
    }
    pairIndex = pairs.size();
    return count;
  }

  private static PairCompute joinPair(Future<PairCompute> task) throws IOException {
    try {
      return task.get();
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      throw new IOException("Pair matching interrupted", e);
    } catch (ExecutionException e) {
      Throwable cause = e.getCause();
      // Worker failures lose their stack in the rethrow; keep it in the log.
      System.err.println("Stereo pair worker failed: " + cause);
      cause.printStackTrace(System.err);
      if (cause instanceof IOException io) throw io;
      if (cause instanceof RuntimeException r) throw r;
      if (cause instanceof Error error) throw error;
      throw new IOException(cause);
    }
  }

  private long pair(
      ImageFrame a, ImageFrame b, int source, long firstCandidate, DataOutputStream out)
      throws IOException {
    // Test hook for --on-pair-failure: -Dsgm.test.failPair=A-B fails that pair before matching.
    if ((a.id + "-" + b.id).equals(System.getProperty("sgm.test.failPair")))
      throw new IllegalStateException("Injected test failure for pair " + a.id + "-" + b.id);
    if (windowed(a, b)) return windowedPair(a, b, source, firstCandidate, out);
    long rectificationStarted = System.nanoTime();
    try (PairPixels pixels = takePixels(a,b,source)) {
      RunTelemetry.record("Rectification pair " + a.id + "-" + b.id, rectificationStarted);
      return publish(compute(pixels, source, options, true), source, firstCandidate, out);
    }
  }

  /** One computed pair (S1): disparities and triangulated rows, before any shared side effect. */
  private record PairCompute(
      PairPixels pixels,
      String prefix,
      String qualityRow,
      String warning,
      int w,
      int h,
      int border,
      CensusSgm.Result result,
      List<TriangulatedRow[]> rows) {}

  /**
   * Rectified pixels to triangulated rows. Shared state is only read here (frames, cameras, fusion
   * plans), so pairs can be computed concurrently; {@link #publish} applies every side effect in
   * pair order. {@code base} carries a worker's share of cores and GPU memory; range decisions use
   * the full options so results do not depend on the number of pairs in flight. {@code serial}
   * enables the serial path's one-pair-ahead preparation.
   */
  private PairCompute compute(PairPixels pixels, int source, Options base, boolean serial)
      throws IOException {
    Options options = base;
    // OpenCV's RNG is per thread; seeding it per pair makes a pair's result independent of what ran
    // before it on this thread (earlier pairs, prefetching) and of --stereo-pairs-in-flight.
    org.opencv.core.Core.setRNGSeed(0x5EED + source);
    {
      Rectified rectified=pixels.rectified;
      int w = rectified.leftGray.cols(), h = rectified.leftGray.rows();
      byte[] left=pixels.left,right=pixels.right,colors=pixels.colors,rightColors=pixels.rightColors;
      byte[] mask1=pixels.mask1,mask2=pixels.mask2;
      String prefix = String.format(Locale.ROOT, "pair_%04d_%04d", rectified.a.id, rectified.b.id);
      long rangeStarted = System.nanoTime();
      Options rangeOptions = options.copy();
      rangeOptions.reprojection *=
          Math.max(rectified.a.keypointPixelScale, rectified.b.keypointPixelScale);
      DisparityRange range =
          DisparityRange.estimate(
              rectified.leftGray,
              rectified.rightGray,
              rectified.leftMask,
              rectified.rightMask,
              rangeOptions,
              rangeSamples(rectified, w, h, mask1, mask2));
      Options matching = options.copy();
      matching.minDisparity = range.minimum;
      matching.disparities = range.count;
      if (options.pairwiseSearch.equals("tsgm")) {
        double[] seeds = new double[3 * range.samples.size()];
        int at = 0;
        for (DisparityRange.Sample s : range.samples) {
          seeds[at++] = s.x();
          seeds[at++] = s.y();
          seeds[at++] = s.disparity();
        }
        matching.tsgmSeeds = seeds;
      }
      int memoryLimit = DisparityRange.memoryLimit(w, h, this.options);
      Path rangeDirectory =
          project == null ? options.output : project.data.directory("05_stereo/" + prefix);
      matching.stereoReportDirectory = rangeDirectory;
      range.evidence(rangeDirectory, options);
      List<Object> attempts = new ArrayList<>();
      if (!options.disparitiesExplicit && matching.disparities > memoryLimit) {
        rangeReport(
            rangeDirectory,
            prefix,
            range,
            matching,
            w,
            h,
            memoryLimit,
            attempts,
            "initial_memory_limit");
        throw new IllegalArgumentException(
            "Automatic disparity range for "
                + prefix
                + " needs "
                + matching.disparities
                + " disparities, but memory permits "
                + memoryLimit
                + ". Increase --memory-mb/heap or lower --max-dimension; the estimated range was"
                + " not silently truncated.");
      }
      rangeReport(rangeDirectory, prefix, range, matching, w, h, memoryLimit, attempts, "planned");
      RunTelemetry.record("Disparity range estimate " + prefix, rangeStarted);
      CensusSgm.Result result;
      int attempt = 0;
      String stopReason;
      if (serial) scheduleNext(w,h,matching);
      long descriptorsStarted = System.nanoTime();
      boolean prefetched = pixels.descriptors != null;
      if(pixels.descriptors==null)pixels.prepareDescriptors(matching);
      RunTelemetry.record(
          "Census/texture descriptors " + prefix + (prefetched ? " (prefetched)" : ""),
          descriptorsStarted);
      try(var descriptors=CensusSgm.use(pixels.descriptors)) {
      while (true) {
        String folder =
            options.disparitiesExplicit
                ? "disparity"
                : String.format(Locale.ROOT, "disparity_attempt_%02d", attempt);
        System.out.println(
            "Disparity search "
                + prefix
                + ": min="
                + matching.minDisparity
                + ", count="
                + matching.disparities
                + " ("
                + range.method
                + ", attempt "
                + attempt
                + ")");
        long matchingStarted = System.nanoTime();
        result =
            ReuseCache.stereo(
                matching,
                left,
                right,
                colors,
                rightColors,
                w,
                h,
                mask1,
                mask2,
                project == null || !(options.diagnostics && options.storage.equals("debug"))
                    ? null
                    : project.data.stereoObserver(source, w, h, matching, folder),
                project == null || !(options.diagnostics && options.storage.equals("debug"))
                    ? null
                    : project.data.directory("05_stereo/" + prefix + "/" + folder));
        RunTelemetry.record(
            "Stereo matching/filtering " + prefix + " attempt " + attempt, matchingStarted);
        System.out.printf(
            Locale.ROOT,
            "Stereo %s: matching/filtering complete in %.2f s; %,d valid depth pixels%n",
            prefix,
            (System.nanoTime() - matchingStarted) / 1e9,
            result.stats.valid);
        attempts.add(
            Json.object(
                "attempt",
                attempt,
                "min_disparity",
                matching.minDisparity,
                "disparities",
                matching.disparities,
                "max_disparity",
                matching.minDisparity + matching.disparities - 1,
                "matching_filtering_seconds",
                (System.nanoTime() - matchingStarted) / 1e9,
                "lower_boundary_rejected",
                result.stats.lowerBoundaryRejected,
                "upper_boundary_rejected",
                result.stats.upperBoundaryRejected,
                "full_interval_pixels",
                result.stats.fullIntervalPixels,
                "lower_pressure",
                result.stats.lowerPressure(),
                "upper_pressure",
                result.stats.upperPressure(),
                "coarse_lower_boundary_rejected",
                result.stats.coarseLowerBoundaryRejected,
                "coarse_upper_boundary_rejected",
                result.stats.coarseUpperBoundaryRejected,
                "coarse_full_interval_pixels",
                result.stats.coarseFullIntervalPixels,
                "final_valid",
                result.stats.valid));
        boolean clipped = result.stats.lowerClipped() || result.stats.upperClipped();
        DisparityRange.Bounds next =
            DisparityRange.expansion(options, matching, result.stats, w, attempt);
        if (options.disparitiesExplicit
            || next == null
            || attempt >= 4
            || next.count() > memoryLimit) {
          stopReason =
              options.disparitiesExplicit
                  ? "explicit"
                  : !clipped
                      ? "no_boundary_pressure"
                      : next == null
                          ? "physical_or_user_boundary"
                          : next.count() > memoryLimit ? "memory_limit" : "retry_limit";
          if (project != null
              && (options.diagnostics && options.storage.equals("debug"))
              && !options.disparitiesExplicit) project.data.finalDisparityAttempt(source, folder);
          break;
        }
        rangeReport(
            rangeDirectory, prefix, range, matching, w, h, memoryLimit, attempts, "expanding");
        matching.minDisparity = next.minimum();
        matching.disparities = next.count();
        attempt++;
      }
      }
      pixels.descriptors = null;
      rangeReport(rangeDirectory, prefix, range, matching, w, h, memoryLimit, attempts, stopReason);
      subpixelHistogram(rangeDirectory, result.disparity);
      if (project == null || (options.diagnostics && options.storage.equals("debug")))
        writeDisparity(options.output.resolve(prefix + ".pfm"), result);
      if (project != null && options.diagnostics && options.storage.equals("standard"))
        project.data.finalDisparity(source, result, matching);
      String qualityRow = prefix + "," + result.stats.csv();
      String warning =
          !(result.stats.lowerClipped() || result.stats.upperClipped())
              ? null
              : "Warning: "
                  + prefix
                  + " rejected "
                  + result.stats.lowerBoundaryRejected
                  + " lower and "
                  + result.stats.upperBoundaryRejected
                  + " upper search-boundary matches with a full search interval. The disparity range"
                  + (options.disparitiesExplicit
                      ? " may be truncated; consider increasing --disparities."
                      : " still touches the automatic search limit. Inspect disparity_range.json;"
                          + " memory, image width or the retry limit prevented further"
                          + " expansion.");
      if (project == null || (options.diagnostics && options.storage.equals("debug"))) {
        Mat preview = new Mat(h, w, CvType.CV_8U);
        byte[] display = new byte[w * h];
        for (int i = 0; i < display.length; i++)
          if (Float.isFinite(result.disparity[i]))
            display[i] =
                (byte)
                    Math.max(
                        1,
                        Math.min(
                            255,
                            255
                                * (result.disparity[i] - matching.minDisparity)
                                / matching.disparities));
        preview.put(0, 0, display);
        try {
          if (!Imgcodecs.imwrite(options.output.resolve(prefix + ".png").toString(), preview))
            throw new IOException("Cannot write disparity preview");
        } finally {
          preview.release();
        }
      }
      double[] q = rectified.q,
          rInv = Math3.transpose(rectified.r1),
          worldRotation = Math3.transpose(rectified.a.camera.rotation);
      int border = options.census / 2;
      double[] centerA = rectified.a.camera.center(), centerB = rectified.b.camera.center();
      // Geometry is computed for blocks of rows in parallel; every side effect (depth evidence,
      // candidate stream, counters) is applied by publish() in the original row-major order.
      final int rowsPerTask = 32;
      final float[] finalDisparity = result.disparity;
      List<Future<TriangulatedRow[]>> blocks = new ArrayList<>();
      ExecutorService triangulators =
          options.cores > 1
              ? Executors.newFixedThreadPool(
                  Math.min(options.cores, 16),
                  r -> {
                    Thread t = new Thread(r, "triangulate");
                    t.setDaemon(true);
                    return t;
                  })
              : null;
      List<TriangulatedRow[]> rows = new ArrayList<>();
      try {
        for (int y0 = border; y0 < h - border; y0 += rowsPerTask) {
          int first = y0, last = Math.min(h - border, y0 + rowsPerTask);
          Callable<TriangulatedRow[]> block =
              () -> {
                TriangulatedRow[] part = new TriangulatedRow[last - first];
                for (int y = first; y < last; y++)
                  part[y - first] =
                      triangulateRow(
                          y, w, border, finalDisparity, mask1, mask2, q, rInv, worldRotation,
                          rectified, centerA, centerB, source);
                return part;
              };
          if (triangulators == null) {
            var done = new CompletableFuture<TriangulatedRow[]>();
            try {
              done.complete(block.call());
            } catch (Exception e) {
              done.completeExceptionally(e);
            }
            blocks.add(done);
          } else blocks.add(triangulators.submit(block));
        }
        for (Future<TriangulatedRow[]> block : blocks) {
          try {
            rows.add(block.get());
          } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new IOException("Triangulation interrupted", e);
          } catch (ExecutionException e) {
            if (e.getCause() instanceof RuntimeException r) throw r;
            if (e.getCause() instanceof Error error) throw error;
            throw new IOException(e.getCause());
          }
        }
      } finally {
        if (triangulators != null) triangulators.shutdownNow();
      }
      return new PairCompute(pixels, prefix, qualityRow, warning, w, h, border, result, rows);
    }
  }

  /** Applies one computed pair's side effects, in pair order: evidence, candidates, reports. */
  private long publish(PairCompute c, int source, long firstCandidate, DataOutputStream out)
      throws IOException {
    PairPixels pixels = c.pixels();
    Rectified rectified = pixels.rectified;
    String prefix = c.prefix();
    int w = c.w(), h = c.h(), border = c.border();
    CensusSgm.Result result = c.result();
    byte[] colors = pixels.colors;
    long exportStarted = System.nanoTime();
    if (project != null)
      project.data.rectifiedImages(
          source,
          rectified.a,
          rectified.b,
          rectified.leftColor,
          rectified.rightColor,
          rectified.leftGray,
          rectified.rightGray,
          rectified.leftMask,
          rectified.rightMask);
    RunTelemetry.record("Rectified image export " + prefix, exportStarted);
    if (quality.isEmpty())
      quality.add(
          "pair,boundary_rejected,upper_boundary_full_search_rejected,ambiguous_rejected,selected,lr_rejected,post_median_lr_rejected,photometric_rejected,speckle_rejected,final_valid");
    quality.add(c.qualityRow());
    Files.write(options.output.resolve("stereo-quality.csv"), quality);
    if (c.warning() != null) System.err.println(c.warning());
    long count = 0;
    float[]
        depths =
            project == null || !(options.diagnostics && options.storage.equals("debug"))
                ? null
                : new float[w * h],
        angles = depths == null ? null : new float[w * h];
    byte[] geometryMask = depths == null ? null : new byte[w * h];
    if (depths != null) {
      Arrays.fill(depths, Float.NaN);
      Arrays.fill(angles, Float.NaN);
    }
    System.out.printf(
        Locale.ROOT, "Triangulating %s: %dx%d raster; recording depth evidence%n", prefix, w, h);
    // From here on, failures leave partial evidence/candidates and cannot be skipped.
    candidatesWritten = true;
    long triangulationStarted = System.nanoTime(), triangulationProgress = triangulationStarted;
    java.nio.ByteBuffer record = java.nio.ByteBuffer.allocate(64);
    boolean bidirectional = PointConfidence.bidirectional(options);
    for (TriangulatedRow[] rows : c.rows()) {
      for (TriangulatedRow row : rows) {
        int y = row.y();
        for (int k = 0; k < row.count(); k++) {
          int x = row.x()[k], i = y * w + x;
          float d = result.disparity[i];
          double[] world = {row.world()[3 * k], row.world()[3 * k + 1], row.world()[3 * k + 2]};
          double angle = row.angle()[k];
          if (depths != null) {
            depths[i] = row.depth()[k];
            angles[i] = (float) angle;
          }
          if (angle < options.minAngle) {
            angleRejected++;
            continue;
          }
          fusion.addPlanned(source, world, row.plan(), 6 * k);
          if (geometryMask != null) geometryMask[i] = (byte) 255;
          // Evidence uses every valid pixel. Sampling only controls the exported candidates.
          if ((x - border) % options.step != 0 || (y - border) % options.step != 0) continue;
          // Same big-endian bytes as the former DataOutputStream field writes, in one call.
          record.clear();
          record.putDouble(world[0]).putDouble(world[1]).putDouble(world[2]);
          record.put(colors[3 * i + 2]).put(colors[3 * i + 1]).put(colors[3 * i]);
          record.putInt(source).putInt(i);
          record.putFloat(result.confidence == null ? Float.NaN : result.confidence[i]);
          record.putFloat(result.costMargin == null ? Float.NaN : result.costMargin[i]);
          record.putFloat(result.costSharpness == null ? Float.NaN : result.costSharpness[i]);
          record.putFloat(result.correlation == null ? Float.NaN : result.correlation[i]);
          record.putFloat(result.leftRightError == null ? Float.NaN : result.leftRightError[i]);
          if (bidirectional)
            record.putFloat(result.rightConfidence == null ? Float.NaN : result.rightConfidence[i]);
          out.write(record.array(), 0, record.position());
          if (project != null)
            project.data.candidate(
                firstCandidate + count,
                source,
                x,
                y,
                d,
                world,
                colors[3 * i + 2] & 255,
                colors[3 * i + 1] & 255,
                colors[3 * i] & 255,
                angle,
                result.correlation == null ? Double.NaN : result.correlation[i]);
          count++;
        }
        if (System.nanoTime() - triangulationProgress >= 5_000_000_000L) {
          System.out.printf(
              Locale.ROOT,
              "Triangulating %s: %.1f%% of rows, %,d candidates%n",
              prefix,
              100.0 * (y - border + 1) / (h - 2 * border),
              count);
          triangulationProgress = System.nanoTime();
        }
      }
    }
    RunTelemetry.record("Triangulation/depth evidence " + prefix, triangulationStarted);
    System.out.printf(
        Locale.ROOT,
        "Triangulating %s: complete in %.2f s; %,d candidates%n",
        prefix,
        (System.nanoTime() - triangulationStarted) / 1e9,
        count);
    if (depths != null) project.data.triangulation(source, depths, angles, geometryMask, w, h);
    return count;
  }

  static double triangulationAngle(double[] point, double[] centerA, double[] centerB) {
    double[] a = Math3.sub(point, centerA), b = Math3.sub(point, centerB);
    double denominator = Math3.norm(a) * Math3.norm(b);
    if (denominator == 0) return 0;
    double crossX = a[1] * b[2] - a[2] * b[1],
        crossY = a[2] * b[0] - a[0] * b[2],
        crossZ = a[0] * b[1] - a[1] * b[0];
    double sine = Math.sqrt(crossX * crossX + crossY * crossY + crossZ * crossZ) / denominator;
    return Math.toDegrees(Math.asin(Math.min(1, sine)));
  }

  private boolean windowed(ImageFrame a, ImageFrame b) {
    return WindowedStereo.shouldUse(a.width, a.height, options);
  }

  private long windowedPair(
      ImageFrame a, ImageFrame b, int source, long firstCandidate, DataOutputStream out)
      throws IOException {
    if (options.pairwiseSearch.equals("tsgm"))
      System.out.println(
          "Windowed pair " + a.id + "-" + b.id + ": tsgm bands are not implemented for windowed"
              + " stereo; using full-range search");
    // Windowed pairs interleave matching and evidence output; treat any failure as unskippable.
    candidatesWritten = true;
    if (windowedStore == null) {
      System.out.println(
          "Windowed stereo uses source-region decoding, bounded processed-image disk caching and"
              + " disk aggregates; resolution, disparities and path directions are preserved");
      RasterCache.clear();
      windowedStore = new WindowedRasters(options);
    }
    WindowedRasters store = windowedStore;
    try (Rectified r = rectify(a, b, false, source, true)) {
      // Evict earlier managed full rasters before constructing the bounded working set.
      RasterCache.forget(a);
      RasterCache.forget(b);
      a.releaseRasters();
      b.releaseRasters();
      var sa = windowedSource(r.a);
      var sb = windowedSource(r.b);
      WindowedRasters.Image left =
          store.warp(sa, r.a.camera, new double[5], r.r1, r.p1, r.vertical, false);
      WindowedRasters.Image right =
          store.warp(sb, r.b.camera, new double[5], r.r2, r.p2, r.vertical, false);
      store.erode(left);
      store.erode(right);
      int w = left.width, h = left.height;
      var la = left;
      var rb = right;
      Options matching = options.copy();
      matching.reprojection *= Math.max(r.a.keypointPixelScale, r.b.keypointPixelScale);
      DisparityRange range =
          DisparityRange.estimate(
              left,
              right,
              matching,
              rangeSamples(r, w, h, p -> la.mask.get(p) != 0, p -> rb.mask.get(p) != 0));
      matching.minDisparity = range.minimum;
      matching.disparities = range.count;
      String prefix = String.format(Locale.ROOT, "pair_%04d_%04d", r.a.id, r.b.id);
      Path directory =
          project == null ? options.output : project.data.directory("05_stereo/" + prefix);
      matching.stereoReportDirectory = directory;
      range.evidence(directory, options);
      List<Object> attempts = new ArrayList<>();
      WindowedStereo stereo = null;
      try {
        int attempt = 0;
        String stop;
        while (true) {
          stereo = new WindowedStereo(store, left, right, matching);
          stereo.compute();
          attempts.add(
              Json.object(
                  "attempt",
                  attempt,
                  "min_disparity",
                  matching.minDisparity,
                  "disparities",
                  matching.disparities,
                  "final_valid",
                  stereo.stats.valid,
                  "windowed",
                  true));
          var next = DisparityRange.expansion(options, matching, stereo.stats, w, attempt);
          if (options.disparitiesExplicit || next == null || attempt >= 4) {
            stop =
                options.disparitiesExplicit
                    ? "explicit"
                    : next == null ? "no_boundary_pressure" : "retry_limit";
            break;
          }
          stereo.close();
          stereo.result.close();
          store.files.remove(stereo.result);
          matching.minDisparity = next.minimum();
          matching.disparities = next.count();
          attempt++;
        }
        rangeReport(directory, prefix, range, matching, w, h, 4096, attempts, stop);
        if (quality.isEmpty())
          quality.add(
              "pair,boundary_rejected,upper_boundary_full_search_rejected,ambiguous_rejected,selected,lr_rejected,post_median_lr_rejected,photometric_rejected,speckle_rejected,final_valid");
        quality.add(prefix + "," + stereo.stats.csv());
        double[] q = r.q,
            rinv = Math3.transpose(r.r1),
            worldRotation = Math3.transpose(r.a.camera.rotation);
        double[] ca = r.a.camera.center(), cb = r.b.camera.center();
        int border = options.census / 2;
        long count = 0;
        try (var work = WorkProgress.start("Windowed triangulation " + prefix, h, "rows")) {
          for (int y = border; y < h - border; y++) {
            for (int x = border; x < w - border; x++) {
              int i = y * w + x;
              float d = stereo.get(i, WindowedStereo.FINAL);
              if (!Float.isFinite(d) || Math.abs(d) < 1e-6) continue;
              int xr = Math.round(x - d);
              if (xr < border
                  || xr >= w - border
                  || left.mask.get(i) == 0
                  || right.mask.get((long) y * w + xr) == 0) continue;
              double xx = r.vertical ? y : x,
                  yy = r.vertical ? x : y,
                  den = q[12] * xx + q[13] * yy + q[14] * d + q[15];
              if (Math.abs(den) < 1e-12) continue;
              double[] rectPoint = {
                (q[0] * xx + q[1] * yy + q[2] * d + q[3]) / den,
                (q[4] * xx + q[5] * yy + q[6] * d + q[7]) / den,
                (q[8] * xx + q[9] * yy + q[10] * d + q[11]) / den
              };
              if (!Double.isFinite(rectPoint[0] + rectPoint[1] + rectPoint[2])
                  || rectPoint[2] <= 0
                  || rectPoint[2] > options.maxDepth) continue;
              double[] cameraPoint = Math3.mv(rinv, rectPoint),
                  world = Math3.mv(worldRotation, Math3.sub(cameraPoint, r.a.camera.translation));
              if (!positiveInBoth(r.a, r.b, world)) continue;
              double angle = triangulationAngle(world, ca, cb);
              if (angle < options.minAngle) {
                angleRejected++;
                continue;
              }
              fusion.add(source, world);
              if ((x - border) % options.step != 0 || (y - border) % options.step != 0) continue;
              int color = left.color.get(i),
                  red = color >>> 16 & 255,
                  green = color >>> 8 & 255,
                  blue = color & 255;
              out.writeDouble(world[0]);
              out.writeDouble(world[1]);
              out.writeDouble(world[2]);
              out.writeByte(red);
              out.writeByte(green);
              out.writeByte(blue);
              out.writeInt(source);
              out.writeInt(i);
              for (int field :
                  new int[] {
                    WindowedStereo.CONFIDENCE,
                    WindowedStereo.LM,
                    WindowedStereo.LS,
                    WindowedStereo.PHOTO,
                    WindowedStereo.LR
                  }) out.writeFloat(stereo.get(i, field));
              if (PointConfidence.bidirectional(options))
                out.writeFloat(stereo.get(i, WindowedStereo.REVERSE));
              if (project != null)
                project.data.candidate(
                    firstCandidate + count,
                    source,
                    x,
                    y,
                    d,
                    world,
                    red,
                    green,
                    blue,
                    angle,
                    stereo.get(i, WindowedStereo.PHOTO));
              count++;
            }
            work.update(y + 1);
          }
          work.done();
        }
        return count;
      } finally {
        if (stereo != null) stereo.close();
      }
    } finally {
      Set<PagedVolume> retained = Collections.newSetFromMap(new IdentityHashMap<>());
      for (var image : windowedSources.values()) {
        retained.add(image.color);
        retained.add(image.gray);
        retained.add(image.mask);
      }
      for (var file : new ArrayList<>(store.files))
        if (!retained.contains(file)) {
          file.close();
          store.files.remove(file);
        }
    }
  }

  private WindowedRasters.Image windowedSource(ImageFrame frame) throws IOException {
    var found = windowedSources.get(frame.id);
    if (found != null) return found;
    long bytes = 12L * frame.width * frame.height;
    long limit =
        Math.min(8L * 1024 * 1048576L, Files.getFileStore(options.output).getUsableSpace() / 8);
    long used = 0;
    for (var image : windowedSources.values()) used += 12L * image.width * image.height;
    var iterator = windowedSources.entrySet().iterator();
    while (iterator.hasNext() && (windowedSources.size() >= 4 || used + bytes > limit)) {
      var entry = iterator.next();
      var image = entry.getValue();
      // Keep the most recently used source for the other endpoint of the current pair.
      if (windowedSources.size() == 1) break;
      used -= 12L * image.width * image.height;
      image.discard();
      iterator.remove();
    }
    found = windowedStore.source(frame);
    windowedSources.put(frame.id, found);
    return found;
  }

  /**
   * P4 diagnostic: histogram of fractional disparities (20 bins). Pixel-locking shows as excess
   * mass near 0 and 1; the index is the share within 0.1 of an integer divided by uniform (0.2).
   */
  private static void subpixelHistogram(Path directory, float[] disparity) throws IOException {
    long[] bins = new long[20];
    long valid = 0;
    for (float d : disparity) {
      if (!Float.isFinite(d)) continue;
      double fraction = d - Math.floor(d);
      bins[Math.min(19, (int) (fraction * 20))]++;
      valid++;
    }
    long near = bins[0] + bins[1] + bins[18] + bins[19];
    Json.write(
        directory.resolve("subpixel_histogram.json"),
        Json.object(
            "valid_pixels", valid,
            "bin_width", .05,
            "fraction_counts", bins,
            "pixel_locking_index", valid == 0 ? null : near / (double) valid / .2,
            "interpretation", "1 = uniform fractions; >1 = mass near integers (pixel-locking)"));
  }

  /** Pixels of one rectified row that triangulate in front of both cameras, in x order. */
  private record TriangulatedRow(
      int y, int count, int[] x, double[] world, float[] depth, double[] angle, double[] plan) {}

  /** The pure geometric part of triangulation for one row; identical arithmetic to before. */
  private TriangulatedRow triangulateRow(
      int y,
      int w,
      int border,
      float[] disparity,
      byte[] mask1,
      byte[] mask2,
      double[] q,
      double[] rInv,
      double[] worldRotation,
      Rectified rectified,
      double[] centerA,
      double[] centerB,
      int source) {
    int capacity = Math.max(0, w - 2 * border), n = 0;
    int[] xs = new int[capacity];
    double[] worlds = new double[3 * capacity], angles = new double[capacity], plans = new double[6 * capacity];
    float[] depthValues = new float[capacity];
    for (int x = border; x < w - border; x++) {
      int i = y * w + x;
      float d = disparity[i];
      if (!Float.isFinite(d) || Math.abs(d) < 1e-6) continue;
      int xr = Math.round(x - d);
      if (xr < border || xr >= w - border || mask1[i] == 0 || mask2[y * w + xr] == 0) continue;
      double xx = rectified.vertical ? y : x, yy = rectified.vertical ? x : y;
      double denominator = q[12] * xx + q[13] * yy + q[14] * d + q[15];
      if (Math.abs(denominator) < 1e-12) continue;
      double[] rectPoint = {
        (q[0] * xx + q[1] * yy + q[2] * d + q[3]) / denominator,
        (q[4] * xx + q[5] * yy + q[6] * d + q[7]) / denominator,
        (q[8] * xx + q[9] * yy + q[10] * d + q[11]) / denominator
      };
      if (!Double.isFinite(rectPoint[0] + rectPoint[1] + rectPoint[2])
          || rectPoint[2] <= 0
          || rectPoint[2] > options.maxDepth) continue;
      double[] cameraPoint = Math3.mv(rInv, rectPoint);
      double[] world =
          Math3.mv(worldRotation, Math3.sub(cameraPoint, rectified.a.camera.translation));
      if (!positiveInBoth(rectified.a, rectified.b, world)) continue;
      xs[n] = x;
      worlds[3 * n] = world[0];
      worlds[3 * n + 1] = world[1];
      worlds[3 * n + 2] = world[2];
      depthValues[n] = (float) cameraPoint[2];
      angles[n] = triangulationAngle(world, centerA, centerB);
      fusion.plan(source, world, plans, 6 * n);
      n++;
    }
    return new TriangulatedRow(y, n, xs, worlds, depthValues, angles, plans);
  }

  private static boolean positiveInBoth(ImageFrame a, ImageFrame b, double[] p) {
    return a.camera.view(p)[2] > 0 && b.camera.view(p)[2] > 0;
  }

  private List<DisparityRange.Sample> rangeSamples(
      Rectified r, int width, int height, byte[] ma, byte[] mb) {
    return rangeSamples(r, width, height, p -> ma[p] != 0, p -> mb[p] != 0);
  }

  private List<DisparityRange.Sample> rangeSamples(
      Rectified r,
      int width,
      int height,
      java.util.function.IntPredicate ma,
      java.util.function.IntPredicate mb) {
    List<DisparityRange.Sample> samples = new ArrayList<>();
    if (options.disparitiesExplicit) return samples;
    double tolerance =
        Math.max(
            2,
            1.5 * options.reprojection * Math.max(r.a.keypointPixelScale, r.b.keypointPixelScale));
    for (FeatureTracks.Track track : tracks) {
      var oa = Geometry.observation(track, r.a.id);
      var ob = Geometry.observation(track, r.b.id);
      if (oa == null || ob == null) continue;
      Point measuredA = oa.feature < r.a.keypoints.length ? r.a.keypoints[oa.feature].pt : null;
      Point measuredB = ob.feature < r.b.keypoints.length ? r.b.keypoints[ob.feature].pt : null;
      if (measuredA != null && measuredB != null) {
        Point a = DisparityRange.rectifyPoint(r.a.camera, measuredA, r.r1, r.p1, r.vertical),
            b = DisparityRange.rectifyPoint(r.b.camera, measuredB, r.r2, r.p2, r.vertical);
        if (inMask(a, width, height, ma)
            && inMask(b, width, height, mb)
            && Math.abs(a.y - b.y) <= tolerance)
          samples.add(new DisparityRange.Sample(a.x - b.x, a.x, a.y, "verified_keypoints", false));
      }
      if (track.point == null) continue;
      double[] pa = r.a.camera.project(track.point), pb = r.b.camera.project(track.point);
      if (pa == null || pb == null) continue;
      if (measuredA != null && Math.hypot(pa[0] - measuredA.x, pa[1] - measuredA.y) > 2 * tolerance)
        continue;
      if (measuredB != null && Math.hypot(pb[0] - measuredB.x, pb[1] - measuredB.y) > 2 * tolerance)
        continue;
      Point
          a =
              DisparityRange.rectifyPoint(
                  r.a.camera, new Point(pa[0], pa[1]), r.r1, r.p1, r.vertical),
          b =
              DisparityRange.rectifyPoint(
                  r.b.camera, new Point(pb[0], pb[1]), r.r2, r.p2, r.vertical);
      if (inMask(a, width, height, ma) && inMask(b, width, height, mb)) {
        int views = 0;
        for (var observation : track.observations)
          if (frames.get(observation.image).camera.registered) views++;
        samples.add(new DisparityRange.Sample(a.x - b.x, a.x, a.y, "sparse_geometry", views >= 3));
      }
    }
    return samples;
  }

  private static boolean inMask(Point p, int width, int height, byte[] mask) {
    return inMask(p, width, height, i -> mask[i] != 0);
  }

  private static boolean inMask(
      Point p, int width, int height, java.util.function.IntPredicate mask) {
    if (p == null || !Double.isFinite(p.x) || !Double.isFinite(p.y)) return false;
    int x = (int) Math.round(p.x), y = (int) Math.round(p.y);
    return x >= 0 && x < width && y >= 0 && y < height && mask.test(y * width + x);
  }

  private void rangeReport(
      Path directory,
      String prefix,
      DisparityRange range,
      Options matching,
      int w,
      int h,
      int limit,
      List<Object> attempts,
      String stop)
      throws IOException {
    Json.write(
        directory.resolve(
            project == null ? prefix + "_disparity_range.json" : "disparity_range.json"),
        Json.object(
            "method",
            range.method,
            "automatic",
            !options.disparitiesExplicit,
            "automatic_minimum",
            !options.disparitiesExplicit && !DisparityRange.pinnedMinimum(options),
            "sample_count",
            range.sampleCount,
            "sparse_geometry_sample_count",
            range.samples.stream().filter(s -> s.source().equals("sparse_geometry")).count(),
            "observed_p01_disparity",
            range.observedLow,
            "observed_p99_disparity",
            range.observedHigh,
            "evidence",
            range.diagnostics,
            "initial_min_disparity",
            range.minimum,
            "initial_disparities",
            range.count,
            "memory_disparity_limit",
            limit,
            "stop_reason",
            stop,
            "effective_min_disparity",
            matching.minDisparity,
            "effective_disparities",
            matching.disparities,
            "effective_max_disparity",
            matching.minDisparity + matching.disparities - 1,
            "processed_matching_width",
            w,
            "processed_matching_height",
            h,
            "sgm_estimated_bytes",
            CensusSgm.memoryEstimate(w, h, matching.disparities, matching),
            "requested_compute_mode",
            options.compute,
            "gpu_estimated_bytes",
            GpuSgm.memoryEstimate(w, h, matching.disparities, matching),
            "gpu_memory_plan",
            GpuStereo.report(w, h, matching),
            "attempts",
            attempts));
  }

  private static final class PairPixels implements AutoCloseable {
    final Rectified rectified;
    final byte[] left,right,colors,rightColors,mask1,mask2;
    CensusSgm.Prepared descriptors;
    PairPixels(Rectified r,Options o) {
      rectified=r;int n=(int)r.leftGray.total();
      left=new byte[n];right=new byte[n];colors=new byte[n*3];
      rightColors=o.censusMode.equals("color-distance")?new byte[n*3]:null;
      mask1=new byte[n];mask2=new byte[n];
      r.leftGray.get(0,0,left);r.rightGray.get(0,0,right);r.leftColor.get(0,0,colors);
      if(rightColors!=null)r.rightColor.get(0,0,rightColors);
      r.leftMask.get(0,0,mask1);r.rightMask.get(0,0,mask2);
    }
    void prepareDescriptors(Options o) {
      descriptors=CensusSgm.prepare(o,left,right,colors,rightColors,rectified.leftGray.cols(),rectified.leftGray.rows(),mask1,mask2);
    }
    public void close(){rectified.close();}
  }

  private PairPixels takePixels(ImageFrame a,ImageFrame b,int source) throws IOException {
    if(pendingPair!=null && pendingA==a.id && pendingB==b.id) {
      Future<PairPixels> task=pendingPair;
      try {
        PairPixels ready=task.get();
        pendingPair=null;
        // Publish metadata in original pair order; workers write no project artifacts/events.
        try { if(project!=null)try(Rectified geometry=rectify(a,b,false,source,true)) {} }
        catch(RuntimeException|IOException failure){ready.close();throw failure;}
        preparedPairs++;return ready;
      } catch(InterruptedException e){Thread.currentThread().interrupt();throw new IOException("Pair preparation interrupted",e);}
      catch(ExecutionException e){if(e.getCause() instanceof IOException io)throw io;if(e.getCause() instanceof RuntimeException r)throw r;throw new IOException(e.getCause());}
    }
    serialPairs++;return new PairPixels(rectify(a,b,false,source),options);
  }

  private void scheduleNext(int w,int h,Options matching) {
    if(pairPreparer==null || pendingPair!=null)return;
    for(int next=pairIndex+1;next<stereoPairs.size();next++) {
      var pair=stereoPairs.get(next);var a=frames.get(pair.a);var b=frames.get(pair.b);
      if(!a.camera.registered||!b.camera.registered)continue;
      if(windowed(a,b))return;
      long pixels=(long)a.width*a.height;
      long allowance=Math.min(options.memoryMb*1048576L,(long)(Runtime.getRuntime().maxMemory()*.60));
      long combined=CensusSgm.memoryEstimate(w,h,matching.disparities,matching)
          + pixels*(96+32L*((options.census*options.census-1+63)/64))
          + RasterCache.limit(options)+128*1048576L;
      if(combined>allowance)return;
      pendingA=pair.a;pendingB=pair.b;
      pendingPair=pairPreparer.submit(() -> {
        PairPixels prepared;
        synchronized(RasterCache.class){prepared=new PairPixels(rectifyLocked(a,b,false,-1,false,false),options);}
        try {Options work=options.copy();work.cores=Math.max(1,options.cores/2);prepared.prepareDescriptors(work);return prepared;}
        catch(RuntimeException|Error e){prepared.close();throw e;}
      });
      return;
    }
  }

  private void closePreparation() throws IOException {
    boolean interrupted = false;
    try {
      if(pendingPair!=null) {
        while (true) {
          try { pendingPair.get().close(); break; }
          catch (InterruptedException e) { interrupted = true; }
          catch (ExecutionException ignored) { break; }
        }
        pendingPair=null;
      }
    } finally {
      try { if(pairPreparer!=null){pairPreparer.close();pairPreparer=null;} }
      finally { if(interrupted)Thread.currentThread().interrupt(); }
    }
  }

  private Rectified rectify(ImageFrame a, ImageFrame b, boolean swapped, int source)
      throws IOException {
    return rectify(a, b, swapped, source, false);
  }

  private Rectified rectify(
      ImageFrame a, ImageFrame b, boolean swapped, int source, boolean geometryOnly)
      throws IOException {
    synchronized(RasterCache.class) { return rectifyLocked(a,b,swapped,source,geometryOnly,true); }
  }

  private Rectified rectifyLocked(ImageFrame a,ImageFrame b,boolean swapped,int source,boolean geometryOnly,boolean report) throws IOException {
    long loadStarted = System.nanoTime();
    if (!geometryOnly) RasterCache.pair(a, b);
    if (!geometryOnly) RunTelemetry.record("Rectification image load " + a.id + "-" + b.id, loadStarted);
    if (a.width != b.width || a.height != b.height)
      throw new IllegalArgumentException(
          "Dense pairs currently require equal processed image dimensions");
    Rectified out = new Rectified(a, b);
    Mat k1 = a.camera.intrinsic(), k2 = b.camera.intrinsic(), d = new MatOfDouble(0, 0, 0, 0, 0);
    Mat r = new Mat(),
        t = new Mat(),
        r1 = new Mat(),
        r2 = new Mat(),
        p1 = new Mat(),
        p2 = new Mat(),
        q = new Mat();
    Mat mapX = new Mat(), mapY = new Mat(), ones = new Mat();
    try {
      if (options.rectified) {
        out.r1 = Math3.identity();
        out.r2 = Math3.identity();
        out.p1 =
            new double[] {
              a.camera.fx, 0, a.camera.cx, 0, 0, a.camera.fy, a.camera.cy, 0, 0, 0, 1, 0
            };
        out.p2 = out.p1.clone();
        out.p2[3] = -a.camera.fx * options.baseline;
        double f = a.camera.fx, B = options.baseline;
        out.q =
            new double[] {
              1,
              0,
              0,
              -a.camera.cx,
              0,
              f / a.camera.fy,
              0,
              -a.camera.cy * f / a.camera.fy,
              0,
              0,
              0,
              f,
              0,
              0,
              1 / B,
              0
            };
        if (!geometryOnly) {
          a.gray.copyTo(out.leftGray);
          b.gray.copyTo(out.rightGray);
          a.color.copyTo(out.leftColor);
          b.color.copyTo(out.rightColor);
          a.validMask.copyTo(out.leftMask);
          b.validMask.copyTo(out.rightMask);
          maskFootprint(out);
        }
        if (report && project != null) {
          double[]
              pLeft = {a.camera.fx, 0, a.camera.cx, 0, 0, a.camera.fy, a.camera.cy, 0, 0, 0, 1, 0},
              pRight = pLeft.clone();
          pRight[3] = -a.camera.fx * B;
          project.data.rectification(
              source,
              a,
              b,
              false,
              out.r1,
              Math3.identity(),
              pLeft,
              pRight,
              out.q,
              Math3.identity(),
              new double[] {-B, 0, 0},
              true);
        }
        return out;
      }
      double[] relativeR = Math3.mul(b.camera.rotation, Math3.transpose(a.camera.rotation));
      double[] relativeT =
          Math3.sub(b.camera.translation, Math3.mv(relativeR, a.camera.translation));
      if (Math3.norm(relativeT) < 1e-8)
        throw new IllegalArgumentException("Coincident cameras cannot define a stereo baseline");
      r.release();
      t.release();
      r = Math3.mat(relativeR, 3, 3);
      t = Math3.mat(relativeT, 3, 1);
      Calib3d.stereoRectify(
          k1,
          d,
          k2,
          d,
          new Size(a.width, a.height),
          r,
          t,
          r1,
          r2,
          p1,
          p2,
          q,
          Calib3d.CALIB_ZERO_DISPARITY,
          options.rectificationAlpha.equals("1") ? 1 : 0,
          new Size(a.width, a.height));
      // Zero-border cropping can magnify convergent close-range pairs by an order of
      // magnitude. That magnifies residual pose error and discards most shared scene content.
      double magnification = p1.get(0, 0)[0] / Math.max(a.camera.fx, b.camera.fx);
      if (options.rectificationAlpha.equals("auto") && magnification > 1.5) {
        Calib3d.stereoRectify(
            k1,
            d,
            k2,
            d,
            new Size(a.width, a.height),
            r,
            t,
            r1,
            r2,
            p1,
            p2,
            q,
            Calib3d.CALIB_ZERO_DISPARITY,
            1,
            new Size(a.width, a.height));
        if(report)System.out.printf(
            Locale.ROOT,
            "Rectification pair %d-%d: alpha=0 magnified %.2fx; preserving field of view with"
                + " alpha=1%n",
            a.id,
            b.id,
            magnification);
        if (report && project != null)
          project.event(
              "rectification_field_of_view",
              Json.object(
                  "a",
                  a.id,
                  "b",
                  b.id,
                  "cropped_magnification",
                  magnification,
                  "chosen_alpha",
                  1,
                  "chosen_focal_pixels",
                  p1.get(0, 0)[0]));
      }
      out.vertical = Math.abs(p2.get(1, 3)[0]) > Math.abs(p2.get(0, 3)[0]);
      double signed = p2.get(out.vertical ? 1 : 0, 3)[0];
      if (signed > 0 && !swapped) {
        out.close();
        return rectifyLocked(b, a, true, source, geometryOnly, report);
      }
      if (signed >= 0)
        throw new IllegalArgumentException("Could not orient stereo pair for positive disparity");
      out.r1 = Math3.array(r1);
      out.r2 = Math3.array(r2);
      out.p1 = Math3.array(p1);
      out.p2 = Math3.array(p2);
      out.q = Math3.array(q);
      if (report && project != null)
        project.data.rectification(
            source,
            a,
            b,
            out.vertical,
            out.r1,
            Math3.array(r2),
            Math3.array(p1),
            Math3.array(p2),
            out.q,
            relativeR,
            relativeT,
            false);
      if (geometryOnly) return out;
      long remapNanos = 0, mapExportNanos = 0, clock = System.nanoTime();
      Calib3d.initUndistortRectifyMap(k1, d, r1, p1, a.gray.size(), CvType.CV_32FC1, mapX, mapY);
      Imgproc.remap(a.gray, out.leftGray, mapX, mapY, Imgproc.INTER_LINEAR);
      Imgproc.remap(a.color, out.leftColor, mapX, mapY, Imgproc.INTER_LINEAR);
      Imgproc.remap(a.validMask, out.leftMask, mapX, mapY, Imgproc.INTER_LINEAR);
      remapNanos += System.nanoTime() - clock;
      clock = System.nanoTime();
      if (report && project != null) project.data.rectificationMaps(source, "left", mapX, mapY, out.vertical);
      mapExportNanos += System.nanoTime() - clock;
      clock = System.nanoTime();
      Calib3d.initUndistortRectifyMap(k2, d, r2, p2, b.gray.size(), CvType.CV_32FC1, mapX, mapY);
      Imgproc.remap(b.gray, out.rightGray, mapX, mapY, Imgproc.INTER_LINEAR);
      Imgproc.remap(b.color, out.rightColor, mapX, mapY, Imgproc.INTER_LINEAR);
      Imgproc.remap(b.validMask, out.rightMask, mapX, mapY, Imgproc.INTER_LINEAR);
      remapNanos += System.nanoTime() - clock;
      clock = System.nanoTime();
      if (report && project != null)
        project.data.rectificationMaps(source, "right", mapX, mapY, out.vertical);
      mapExportNanos += System.nanoTime() - clock;
      RunTelemetry.recordTotal("Rectification maps/remap " + a.id + "-" + b.id, remapNanos, 2);
      RunTelemetry.recordTotal("Rectification map export " + a.id + "-" + b.id, mapExportNanos, 2);
      // Transpose vertical epipolar lines; Q still receives original (x,y,disparity).
      if (out.vertical) {
        transpose(out.leftGray);
        transpose(out.rightGray);
        transpose(out.leftColor);
        transpose(out.rightColor);
        transpose(out.leftMask);
        transpose(out.rightMask);
      }
      maskFootprint(out);
      return out;
    } catch (RuntimeException | IOException e) {
      out.close();
      throw e;
    } finally {
      k1.release();
      k2.release();
      d.release();
      r.release();
      t.release();
      r1.release();
      r2.release();
      p1.release();
      p2.release();
      q.release();
      mapX.release();
      mapY.release();
      ones.release();
    }
  }

  private void maskFootprint(Rectified rectified) {
    // One extra pixel covers bilinear sampling at a fractional right-image coordinate.
    int size = Math.max(options.census, options.correlationWindow) + 2;
    Mat kernel = Mat.ones(size, size, CvType.CV_8U);
    try {
      for (Mat mask : new Mat[] {rectified.leftMask, rectified.rightMask}) {
        Imgproc.threshold(mask, mask, 254, 255, Imgproc.THRESH_BINARY);
        Imgproc.erode(
            mask, mask, kernel, new Point(-1, -1), 1, Core.BORDER_CONSTANT, new Scalar(0));
      }
    } finally {
      kernel.release();
    }
  }

  private static void transpose(Mat m) {
    Mat tmp = new Mat();
    try {
      Core.transpose(m, tmp);
      tmp.copyTo(m);
    } finally {
      tmp.release();
    }
  }

  static void writeDisparity(Path path, CensusSgm.Result result) throws IOException {
    try (OutputStream out = new BufferedOutputStream(Files.newOutputStream(path))) {
      out.write(
          ("Pf\n" + result.width + " " + result.height + "\n-1.0\n")
              .getBytes(java.nio.charset.StandardCharsets.US_ASCII));
      java.nio.ByteBuffer row =
          java.nio.ByteBuffer.allocate(result.width * 4).order(java.nio.ByteOrder.LITTLE_ENDIAN);
      for (int y = result.height - 1; y >= 0; y--) {
        row.clear();
        for (int x = 0; x < result.width; x++) row.putFloat(result.disparity[y * result.width + x]);
        out.write(row.array());
      }
    }
  }

  private static final class Rectified implements AutoCloseable {
    final ImageFrame a, b;
    boolean vertical;
    double[] r1, r2, p1, p2, q;
    final Mat leftGray = new Mat(),
        rightGray = new Mat(),
        leftColor = new Mat(),
        rightColor = new Mat(),
        leftMask = new Mat(),
        rightMask = new Mat();

    Rectified(ImageFrame a, ImageFrame b) {
      this.a = a;
      this.b = b;
    }

    public void close() {
      leftGray.release();
      rightGray.release();
      leftColor.release();
      rightColor.release();
      leftMask.release();
      rightMask.release();
    }
  }
}
