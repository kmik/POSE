package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;
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
  private long surfaceMerged, surfaceMoved, beforeSurface, voxelRejected;
  private final List<String> quality = new ArrayList<>();
  private final Project project;
  private final Crs crs;
  private final List<FeatureTracks.Track> tracks;

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
    Path candidates = Files.createTempFile(options.output, "candidates-", ".bin");
    long count = 0;
    boolean completed = false;
    try {
      try (DataOutputStream out =
          new DataOutputStream(new BufferedOutputStream(Files.newOutputStream(candidates)))) {
        for (FeatureTracks.Pair pair : pairs) {
          if (!frames.get(pair.a).camera.registered || !frames.get(pair.b).camera.registered)
            continue;
          int source = fusion.addPair(pair.a, pair.b);
          long n;
          try (WorkProgress work =
              WorkProgress.start(
                  "Rectify/match/triangulate pair " + pair.a + "-" + pair.b, 0, "pixels")) {
            n = pair(frames.get(pair.a), frames.get(pair.b), source, count, out);
            work.done();
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
      }
      this.candidates = count;
      Files.write(options.output.resolve("stereo-quality.csv"), quality);
      if (project != null) {
        project.stage(
            "05_stereo",
            "complete",
            "Rectification, filtering stages and confidence rasters exported");
        project.stage("06_fusion", "running", "Independent stereo-pair depth corroboration");
        fusion.export(project.data);
      }
      if (count == 0)
        throw new IOException(
            "No dense candidates; check disparity range, camera calibration, and baseline");
      try (WorkProgress work = WorkProgress.start("Prepare fusion evidence/backend", 0, "pixels")) {
        fusion.prepare(count);
        work.done();
      }
      System.out.printf(
          Locale.ROOT, "Fusion: processing %,d candidates from stereo depth evidence%n", count);
      long fusionProgress = System.nanoTime();
      long kept;
      long fusionOutputNanos = 0, outputStarted = 0;
      long normalNanos = 0;
      try (DataInputStream in =
              new DataInputStream(new BufferedInputStream(Files.newInputStream(candidates)));
          WorkProgress fusionWork =
              WorkProgress.start("Fusion candidates / normal estimation", count, "candidates");
          FusionOutput output = new FusionOutput(options, fusion, project, crs);
          FusionPipeline pipeline =
              new FusionPipeline(
                  in,
                  count,
                  fusion,
                  options.fusionPipeline,
                  (options.surfaceConsolidation.equals("normal")
                      || options.pointAttributes.equals("surface")))) {
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
              fusionProgress = System.nanoTime();
            }
            double[] p = batch.points()[offset];
            int r = batch.rgb()[offset] >>> 16,
                g = batch.rgb()[offset] >>> 8 & 255,
                b = batch.rgb()[offset] & 255,
                source = batch.source()[offset];
            int pixel = batch.pixel()[offset];
            DepthFusion.Result result = batch.results()[offset];
            int views = result.views;
            if (project != null) project.data.fusionRefinement(i, source, p, result);
            if (!result.accepted()) {
              fusionRejected++;
              if (result.status.equals("depth_consensus_rejected")) consensusRejected++;
              if (result.status.equals("depth_uncertainty_rejected")) uncertaintyRejected++;
              if (project != null) project.data.decision(i, source, pixel, views, result.status);
              continue;
            }
            if (result.shift > 1e-8) refinedPoints++;
            output.add(i, source, pixel, (r << 16) | (g << 8) | b, result, batch.surface()[offset]);
          }
          if (batch != null) fusionOutputNanos += System.nanoTime() - outputStarted;
        }
        fusionWork.update(count);
        fusionWork.done();
        fusionWork.close();
        pipeline.close();
        normalNanos = pipeline.normalNanos();
        System.out.printf(
            Locale.ROOT,
            "Fusion: 100.0%% (%,d/%,d), %,d retained, %,d rejected; candidate processing"
                + " complete%n",
            count,
            count,
            output.count(),
            fusionRejected);
        output.finish();
        kept = output.count();
        beforeSurface = output.stagedCount();
        surfaceMerged = output.mergedCount();
        surfaceMoved = output.movedCount();
        voxelRejected = output.voxelRejected();
      }
      accepted = kept;
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
        fusion.close();
        if (completed && !options.storage.equals("debug")) fusion.discardStorage();
      } finally {
        Files.deleteIfExists(candidates);
      }
    }
  }

  String summary() {
    return "Dense candidates="
        + candidates
        + "\nFusion rejected="
        + fusionRejected
        + "\nPer-ray depth consensus rejected="
        + consensusRejected
        + "\nDepth uncertainty rejected="
        + uncertaintyRejected
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
        + "\nMinimum supporting views="
        + fusion.requiredViews()
        + "\nTriangulation angle rejected="
        + angleRejected
        + "\nExported points="
        + accepted
        + "\n";
  }

  private long pair(
      ImageFrame a, ImageFrame b, int source, long firstCandidate, DataOutputStream out)
      throws IOException {
    long rectificationStarted = System.nanoTime();
    try (Rectified rectified = rectify(a, b, false, source)) {
      RunTelemetry.record("Rectification pair " + a.id + "-" + b.id, rectificationStarted);
      int w = rectified.leftGray.cols(), h = rectified.leftGray.rows();
      byte[] left = new byte[w * h], right = new byte[w * h];
      rectified.leftGray.get(0, 0, left);
      rectified.rightGray.get(0, 0, right);
      byte[] colors = new byte[w * h * 3],
          rightColors = options.censusMode.equals("color-distance") ? new byte[w * h * 3] : null;
      rectified.leftColor.get(0, 0, colors);
      if (rightColors != null) rectified.rightColor.get(0, 0, rightColors);
      byte[] mask1 = new byte[w * h], mask2 = new byte[w * h];
      rectified.leftMask.get(0, 0, mask1);
      rectified.rightMask.get(0, 0, mask2);
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
      String prefix = String.format(Locale.ROOT, "pair_%04d_%04d", rectified.a.id, rectified.b.id);
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
      int memoryLimit = DisparityRange.memoryLimit(w, h, options);
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
      CensusSgm.Result result;
      int attempt = 0;
      String stopReason;
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
                project == null || !options.storage.equals("debug")
                    ? null
                    : project.data.stereoObserver(source, w, h, matching, folder),
                project == null || !options.storage.equals("debug")
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
          if (project != null && options.storage.equals("debug") && !options.disparitiesExplicit)
            project.data.finalDisparityAttempt(source, folder);
          break;
        }
        rangeReport(
            rangeDirectory, prefix, range, matching, w, h, memoryLimit, attempts, "expanding");
        matching.minDisparity = next.minimum();
        matching.disparities = next.count();
        attempt++;
      }
      rangeReport(rangeDirectory, prefix, range, matching, w, h, memoryLimit, attempts, stopReason);
      if (project == null || options.storage.equals("debug"))
        writeDisparity(options.output.resolve(prefix + ".pfm"), result);
      if (project != null && options.storage.equals("standard"))
        project.data.finalDisparity(source, result, matching);
      if (quality.isEmpty())
        quality.add(
            "pair,boundary_rejected,upper_boundary_full_search_rejected,ambiguous_rejected,selected,lr_rejected,post_median_lr_rejected,photometric_rejected,speckle_rejected,final_valid");
      quality.add(prefix + "," + result.stats.csv());
      Files.write(options.output.resolve("stereo-quality.csv"), quality);
      if (result.stats.lowerClipped() || result.stats.upperClipped())
        System.err.println(
            "Warning: "
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
                        + " expansion."));
      if (project == null || options.storage.equals("debug")) {
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
      long count = 0;
      int border = options.census / 2;
      double[] centerA = rectified.a.camera.center(), centerB = rectified.b.camera.center();
      float[]
          depths = project == null || !options.storage.equals("debug") ? null : new float[w * h],
          angles = depths == null ? null : new float[w * h];
      byte[] geometryMask = depths == null ? null : new byte[w * h];
      if (depths != null) {
        Arrays.fill(depths, Float.NaN);
        Arrays.fill(angles, Float.NaN);
      }
      System.out.printf(
          Locale.ROOT, "Triangulating %s: %dx%d raster; recording depth evidence%n", prefix, w, h);
      long triangulationStarted = System.nanoTime(), triangulationProgress = triangulationStarted;
      for (int y = border; y < h - border; y++) {
        for (int x = border; x < w - border; x++) {
          int i = y * w + x;
          float d = result.disparity[i];
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
          double angle = triangulationAngle(world, centerA, centerB);
          if (depths != null) {
            depths[i] = (float) cameraPoint[2];
            angles[i] = (float) angle;
          }
          if (angle < options.minAngle) {
            angleRejected++;
            continue;
          }
          fusion.add(source, world);
          if (geometryMask != null) geometryMask[i] = (byte) 255;
          // Evidence uses every valid pixel. Sampling only controls the exported candidates.
          if ((x - border) % options.step != 0 || (y - border) % options.step != 0) continue;
          out.writeDouble(world[0]);
          out.writeDouble(world[1]);
          out.writeDouble(world[2]);
          out.writeByte(colors[3 * i + 2] & 255);
          out.writeByte(colors[3 * i + 1] & 255);
          out.writeByte(colors[3 * i] & 255);
          out.writeInt(source);
          out.writeInt(i);
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

  private static boolean positiveInBoth(ImageFrame a, ImageFrame b, double[] p) {
    return a.camera.view(p)[2] > 0 && b.camera.view(p)[2] > 0;
  }

  private List<DisparityRange.Sample> rangeSamples(
      Rectified r, int width, int height, byte[] ma, byte[] mb) {
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
    if (p == null || !Double.isFinite(p.x) || !Double.isFinite(p.y)) return false;
    int x = (int) Math.round(p.x), y = (int) Math.round(p.y);
    return x >= 0 && x < width && y >= 0 && y < height && mask[y * width + x] != 0;
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

  private Rectified rectify(ImageFrame a, ImageFrame b, boolean swapped, int source)
      throws IOException {
    RasterCache.pair(a, b);
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
    Mat mapX = new Mat(),
        mapY = new Mat(),
        ones = new Mat(a.gray.size(), CvType.CV_8U, new Scalar(255));
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
        a.gray.copyTo(out.leftGray);
        b.gray.copyTo(out.rightGray);
        a.color.copyTo(out.leftColor);
        b.color.copyTo(out.rightColor);
        a.validMask.copyTo(out.leftMask);
        b.validMask.copyTo(out.rightMask);
        maskFootprint(out);
        if (project != null) {
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
          a.gray.size(),
          r,
          t,
          r1,
          r2,
          p1,
          p2,
          q,
          Calib3d.CALIB_ZERO_DISPARITY,
          options.rectificationAlpha.equals("1") ? 1 : 0,
          a.gray.size());
      // Zero-border cropping can magnify convergent close-range pairs by an order of
      // magnitude. That magnifies residual pose error and discards most shared scene content.
      double magnification = p1.get(0, 0)[0] / Math.max(a.camera.fx, b.camera.fx);
      if (options.rectificationAlpha.equals("auto") && magnification > 1.5) {
        Calib3d.stereoRectify(
            k1,
            d,
            k2,
            d,
            a.gray.size(),
            r,
            t,
            r1,
            r2,
            p1,
            p2,
            q,
            Calib3d.CALIB_ZERO_DISPARITY,
            1,
            a.gray.size());
        System.out.printf(
            Locale.ROOT,
            "Rectification pair %d-%d: alpha=0 magnified %.2fx; preserving field of view with"
                + " alpha=1%n",
            a.id,
            b.id,
            magnification);
        if (project != null)
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
        return rectify(b, a, true, source);
      }
      if (signed >= 0)
        throw new IllegalArgumentException("Could not orient stereo pair for positive disparity");
      out.r1 = Math3.array(r1);
      out.r2 = Math3.array(r2);
      out.p1 = Math3.array(p1);
      out.p2 = Math3.array(p2);
      out.q = Math3.array(q);
      if (project != null)
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
      Calib3d.initUndistortRectifyMap(k1, d, r1, p1, a.gray.size(), CvType.CV_32FC1, mapX, mapY);
      Imgproc.remap(a.gray, out.leftGray, mapX, mapY, Imgproc.INTER_LINEAR);
      Imgproc.remap(a.color, out.leftColor, mapX, mapY, Imgproc.INTER_LINEAR);
      Imgproc.remap(a.validMask, out.leftMask, mapX, mapY, Imgproc.INTER_LINEAR);
      if (project != null) project.data.rectificationMaps(source, "left", mapX, mapY, out.vertical);
      Calib3d.initUndistortRectifyMap(k2, d, r2, p2, b.gray.size(), CvType.CV_32FC1, mapX, mapY);
      Imgproc.remap(b.gray, out.rightGray, mapX, mapY, Imgproc.INTER_LINEAR);
      Imgproc.remap(b.color, out.rightColor, mapX, mapY, Imgproc.INTER_LINEAR);
      Imgproc.remap(b.validMask, out.rightMask, mapX, mapY, Imgproc.INTER_LINEAR);
      if (project != null)
        project.data.rectificationMaps(source, "right", mapX, mapY, out.vertical);
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
