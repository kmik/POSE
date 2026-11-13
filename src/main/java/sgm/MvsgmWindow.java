package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import java.util.concurrent.*;

/** Demand-driven stereo/consistency producer for image fusion, with unchanged pair/candidate IDs. */
final class MvsgmWindow implements AutoCloseable {
  final CameraWindow graph;
  final boolean[] eligible;
  final boolean retainFusion;
  final long[] planningStarts;
  final long estimatedPeakBytes;
  final int[] pairOf;
  private final MultiViewSgm mvs;
  private final Options o;
  private final List<ImageFrame> frames;
  private final List<FeatureTracks.Track> tracks;
  private final Project project;
  private final MultiViewSgm.Plan[] headers;
  private final Object[] reports;
  private final boolean[] raw, consistent, fusionFinished;
  private final int[] rawReaders;
  private final long[] counts, sourceBytes;
  private final ExecutorService preparer;
  private final GpuMemoryPool.Scope gpu;
  private final WorkProgress stereoWork;
  private Future<MultiViewSgm.Plan> next;
  private int nextCamera = -1, completed, depthCompleted, cameraCount;
  private long considered, kept, depthNanos, consistencyNanos, currentSourceBytes, peakSourceBytes;
  private boolean reported, fusionStarted, closed;

  static boolean enabled(Options o) {
    return o.denseMethod.equals("mvsgm") && o.fusionMode.equals("image")
        && !o.streamedProcessing.equals("off")
        && Boolean.parseBoolean(System.getProperty("sgm.mvsgm.window", "true"));
  }

  MvsgmWindow(MultiViewSgm mvs, List<ImageFrame> frames, List<FeatureTracks.Track> tracks,
      DepthFusion fusion, Project project) throws IOException {
    this.mvs = mvs;
    o = mvs.o;
    this.frames = frames;
    this.tracks = tracks;
    this.project = project;
    if (o.step > 1)
      System.out.println("Note: --step " + o.step + " does not thin --dense-method mvsgm depth maps; every consistent pixel goes to fusion");
    int n = frames.size();
    headers = new MultiViewSgm.Plan[n];
    reports = new Object[n];
    raw = new boolean[n];
    consistent = new boolean[n];
    fusionFinished = new boolean[n];
    rawReaders = new int[n];
    counts = new long[n];
    sourceBytes = new long[n];
    pairOf = new int[n];
    Arrays.fill(pairOf, -1);
    Camera[] cameras = new Camera[n];
    int[] widths = new int[n], heights = new int[n];
    mvs.handoff = new MvsgmMaps(o.output.resolve("05_stereo/mvsgm"),
        o.diagnostics || Boolean.getBoolean("sgm.mvsgm.keepMaps"));
    if (tracks.stream().filter(t -> t.point != null).count() < 12)
      throw new IllegalArgumentException("mvsgm needs at least 12 triangulated sparse points; rerun the match step");
    long pixels = 0;
    for (ImageFrame frame : frames) {
      int c = frame.id;
      cameras[c] = frame.camera;
      widths[c] = frame.width;
      heights[c] = frame.height;
      if (!MultiViewSgm.usable(frame.camera)) continue;
      mvs.handoff.add(new MvsgmMaps.Camera(c, -1, 0, 0, null, false));
      headers[c] = mvs.headerPlan(frames, tracks, c);
      if (headers[c] == null) {
        reports[c] = Json.object("image", c, "skipped", mvs.skipped.get(c));
        continue;
      }
      cameraCount++;
      pixels = Math.addExact(pixels, (long) frame.width * frame.height);
      var h = headers[c];
      mvs.info.put(c, new MultiViewSgm.CameraInfo(c, h.neighbours.stream().mapToInt(MultiViewSgm.Neighbor::id).toArray(),
          h.labels.step(), 1 / h.labels.q(h.labels.count() - 1), 1 / h.labels.low(), mvs.tileSize(frame) > 0));
    }
    if (cameraCount == 0) throw new IOException("mvsgm produced no camera plan; check sparse points and calibration");
    retainFusion = o.fusionSeedOrder.equals("precision") || Boolean.getBoolean("sgm.fusion.keepFinishedMaps");
    boolean retainSource = retainFusion || o.diagnostics || Boolean.getBoolean("sgm.mvsgm.keepMaps");
    graph = new CameraWindow(cameras, widths, heights, headers, retainSource);
    eligible = graph.eligible;
    long[] fusionBytes = new long[n];
    for (int c = 0; c < n; c++) fusionBytes[c] = 13L * widths[c] * heights[c];
    long mapDisk = Math.addExact(graph.sourcePeakBytes, graph.peakBytes(fusionBytes, retainFusion));
    int attributes = o.pointAttributes.equals("surface") ? SurfaceAttributes.BYTES : 0;
    long spool = Math.multiplyExact(pixels, 27L + attributes);
    long records = o.writePointConfidence ? Math.multiplyExact(pixels, 224L + Math.ceilDiv(n, 8)) : 0;
    if (o.fusionEmission.equals("camera") || o.surfaceConsolidation.equals("normal"))
      records = Math.addExact(records, Math.multiplyExact(pixels, 256L + Math.ceilDiv(n, 8)));
    long exports = Math.multiplyExact(pixels, 26L + attributes + (o.writePly ? 27L + attributes : 0));
    long retained = retainSource ? graph.sourcePeakBytes : 0;
    if (o.diagnostics) retained = Math.addExact(retained, Math.multiplyExact(pixels, 2048L));
    long need = Math.addExact(Math.max(Math.addExact(mapDisk, spool), Math.addExact(retained, Math.addExact(spool, exports))), records);
    estimatedPeakBytes = Math.addExact(need, Math.max(256L << 20, need / 20));
    checkDisk(o.output, estimatedPeakBytes, mapDisk, graph.peakCameras, cameraCount);
    planningStarts = new long[cameraCount + 1];
    int p = 0;
    for (int c = 0; c < n; c++) if (eligible[c]) {
      var ci = mvs.info.get(c);
      int pair = fusion.addPair(c, ci.neighbours()[0]);
      fusion.markSingleView(pair);
      pairOf[c] = pair;
      planningStarts[++p] = Math.addExact(planningStarts[p - 1], (long) widths[c] * heights[c]);
      mvs.handoff.add(new MvsgmMaps.Camera(c, pair, widths[c], heights[c], null, ci.tiled()));
      rawReaders[c]++;
      for (int j : ci.neighbours()) if (eligible[j]) rawReaders[j]++;
    }
    gpu = o.compute.equals("cpu") ? null : GpuMemoryPool.install(mvs.gpuBudget());
    preparer = Executors.newSingleThreadExecutor(r -> {
      Thread t = new Thread(r, "mvsgm-window-prepare");
      t.setDaemon(true);
      return t;
    });
    stereoWork = WorkProgress.start("Multi-view stereo window", 2L * cameraCount, "camera passes");
    mvs.handoff.window = this;
  }

  static void checkDisk(Path output, long need, long maps, int window, int total) throws IOException {
    long available = Files.getFileStore(output).getUsableSpace();
    Long override = Long.getLong("sgm.mvsgm.diskAvailableMb");
    if (override != null) available = Math.min(available, Math.max(0, override) << 20);
    System.out.printf(Locale.ROOT,
        "mvsgm overlap window: up to %d/%d cameras; peak temporary maps estimate %,d MiB;"
            + " peak additional disk including cloud staging/exports estimate %,d MiB; available %,d MiB%n",
        window, total, maps >> 20, need >> 20, available >> 20);
    if (need > available)
      throw new IOException("mvsgm overlap window needs approximately " + (need >> 20)
          + " MiB additional disk including cloud staging/exports, available " + (available >> 20)
          + " MiB; free disk or lower --quality (no temporary maps were created)");
  }

  private void schedule(int camera) {
    if (next != null) return;
    nextCamera = camera;
    next = preparer.submit(() -> {
      try (var rows = StereoRows.open(Math.max(1, o.cores / 2))) {
        return mvs.preparePlan(frames, headers[camera]);
      }
    });
  }

  private void rawCameras(SortedSet<Integer> requested) throws IOException {
    int[] missing = requested.stream().filter(c -> eligible[c] && !raw[c]).mapToInt(Integer::intValue).toArray();
    for (int i = 0; i < missing.length; i++) {
      int camera = missing[i];
      long clock = System.nanoTime();
      stereoWork.detail("depth camera " + camera);
      schedule(camera);
      MultiViewSgm.Plan plan;
      try {
        if (nextCamera != camera) throw new IllegalStateException("Stereo preparation order changed");
        plan = next.get();
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
        throw new IOException("mvsgm window interrupted", e);
      } catch (ExecutionException e) {
        if (e.getCause() instanceof IOException io) throw io;
        if (e.getCause() instanceof RuntimeException r) throw r;
        if (e.getCause() instanceof Error error) throw error;
        throw new IOException(e.getCause());
      }
      next = null;
      if (i + 1 < missing.length) schedule(missing[i + 1]);
      try (var rows = StereoRows.open(Math.max(1, o.cores / 2))) {
        reports[camera] = mvs.computeCamera(frames, tracks, camera, plan, clock);
      }
      depthNanos += System.nanoTime() - clock;
      raw[camera] = true;
      stereoWork.update(++depthCompleted + completed);
      sourceBytes[camera] = 16L * frames.get(camera).width * frames.get(camera).height + 4L * DepthMapFile.HEADER;
      currentSourceBytes += sourceBytes[camera];
      peakSourceBytes = Math.max(peakSourceBytes, currentSourceBytes);
    }
  }

  void prepare(int c) throws IOException {
    if (consistent[c] || !eligible[c]) return;
    var ci = mvs.info.get(c);
    SortedSet<Integer> requested = new TreeSet<>();
    requested.add(c);
    for (int j : ci.neighbours()) if (eligible[j]) requested.add(j);
    rawCameras(requested);
    long clock = System.nanoTime();
    stereoWork.detail("consistency camera " + c);
    MultiViewSgm.Consistency result = mvs.consistentCamera(frames, ci, pairOf[c]);
    consistencyNanos += System.nanoTime() - clock;
    counts[c] = result.kept();
    considered += result.considered();
    kept += result.kept();
    consistent[c] = true;
    completed++;
    stereoWork.update(depthCompleted + completed);
    long extra = 8L * frames.get(c).width * frames.get(c).height + 2L * DepthMapFile.HEADER;
    sourceBytes[c] += extra;
    currentSourceBytes += extra;
    peakSourceBytes = Math.max(peakSourceBytes, currentSourceBytes);
    consumed(c);
    for (int j : ci.neighbours()) if (eligible[j]) consumed(j);
    if (completed == cameraCount) finishStereo();
  }

  /** Fix each emitted candidate's global prefix after its earlier cameras' counts exist. */
  void beforePassTwo(int c, long[] starts) throws IOException {
    long prefix = 0;
    for (int j = 0; j <= c; j++) if (eligible[j]) {
      prepare(j);
      starts[pairOf[j]] = prefix;
      prefix = Math.addExact(prefix, counts[j]);
      starts[pairOf[j] + 1] = prefix;
    }
  }

  void fusionStarting() throws IOException {
    if (fusionStarted) return;
    fusionStarted = true;
    if (project != null) project.stage("06_fusion", "running", "Overlap-window image fusion; stereo may still be running");
  }

  private void consumed(int c) throws IOException {
    if (--rawReaders[c] < 0) throw new IllegalStateException("Raw depth reader counted twice");
    releaseIfFinished(c);
  }

  void fusionFinished(int c) throws IOException {
    fusionFinished[c] = true;
    releaseIfFinished(c);
  }

  private void releaseIfFinished(int c) throws IOException {
    if (!fusionFinished[c] || rawReaders[c] != 0) return;
    mvs.handoff.releaseCamera(c);
    if (!mvs.handoff.keepsFiles()) {
      currentSourceBytes -= sourceBytes[c];
      sourceBytes[c] = 0;
    }
  }

  long count() {
    return kept;
  }

  private void finishStereo() throws IOException {
    if (reported) return;
    reported = true;
    List<Object> cameras = new ArrayList<>();
    for (Object report : reports) if (report != null) cameras.add(report);
    mvs.phaseTwo.putAll(Json.object("consistent_views_required", o.mvsgmConsistentViews,
        "tolerance_labels", o.mvsgmConsistencyLabels, "depths", considered, "consistent", kept,
        "inconsistent_rejected", considered - kept, "candidates", kept, "handoff", "maps"));
    Json.write(o.output.resolve("05_stereo/mvsgm/mvsgm.json"), Json.object(
        "census_window", mvs.census, "best_k", mvs.k, "neighbours", mvs.neighbors,
        "label_pixels", mvs.labelPixels, "levels", mvs.levels(), "consistency", mvs.phaseTwo,
        "cameras", cameras, "window", report()));
    RunTelemetry.recordTotal("Multi-view SGM depth maps", depthNanos, cameraCount);
    RunTelemetry.recordTotal("Multi-view consistency", consistencyNanos, cameraCount);
    stereoWork.done();
    stereoWork.close();
    if (project != null) project.stage("05_stereo", "complete", "Overlap-window stereo and consistency complete");
  }

  Object report() {
    return Json.object("enabled", true, "peak_camera_bound", graph.peakCameras,
        "camera_count", cameraCount, "peak_additional_disk_estimate", estimatedPeakBytes,
        "peak_source_map_bytes", peakSourceBytes, "remaining_source_map_bytes", currentSourceBytes,
        "stereo_depth_seconds", depthNanos / 1e9, "stereo_consistency_seconds", consistencyNanos / 1e9,
        "precision_or_retention_requires_all_maps", retainFusion, "stage_times_overlap", true);
  }

  @Override public void close() {
    if (closed) return;
    closed = true;
    preparer.shutdownNow();
    boolean interrupted = false;
    for (;;) try {
      if (preparer.awaitTermination(1, TimeUnit.SECONDS)) break;
    } catch (InterruptedException e) {
      interrupted = true;
    }
    if (gpu != null) gpu.close();
    stereoWork.close();
    if (interrupted) Thread.currentThread().interrupt();
  }
}
