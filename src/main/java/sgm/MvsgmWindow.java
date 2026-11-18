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
  final MvsgmDiskPlan diskPlan;
  final FusionHostMemory hostMemory;
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
  private final MvsgmCameraPool pool;
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
        o.diagnostics || Boolean.getBoolean("sgm.mvsgm.keepMaps"), o.fusionSourceMaps.equals("scoped"));
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
    // Cameras in flight: up to that many raw maps (16 bytes per pixel) exist ahead of their first reader.
    int inFlight = mvs.camerasInFlight(frames, cameraCount);
    if (inFlight > 1) {
      long largest = 0;
      for (int c = 0; c < n; c++)
        if (eligible[c]) largest = Math.max(largest, 16L * widths[c] * heights[c] + 4L * DepthMapFile.HEADER);
      mapDisk = Math.addExact(mapDisk, Math.multiplyExact(largest, (long) inFlight));
    }
    long retained = retainSource ? graph.sourcePeakBytes : 0;
    diskPlan = MvsgmDiskPlan.of(pixels, n, mapDisk, retained, o);
    estimatedPeakBytes = diskPlan.worstCase();
    checkDisk(o.output, diskPlan, graph.peakCameras, cameraCount);
    o.outputDiskSpace = new OutputDiskSpace(o.output, diskPlan.headroom());
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
    long futureMetadata = 0;
    if (o.fusionHostMemoryMb > 0) futureMetadata += 8L * n * Math.ceilDiv(n, 64);
    if (o.fusionHostMemoryMb > 0) for (ImageFrame f : frames)
      futureMetadata += 8L * (f.height + 1) + (64L << 10)
          + 16L * Math.ceilDiv(40L * f.width * f.height, HostMapCache.PAGE);
    hostMemory = o.fusionHostMemoryMb > 0 ? new FusionHostMemory(o, futureMetadata) : null;
    mvs.hostMemory = hostMemory;
    mvs.handoff.hostMemory = hostMemory;
    gpu = o.compute.equals("cpu") ? null : GpuMemoryPool.install(mvs.gpuBudget());
    preparer = Executors.newSingleThreadExecutor(r -> {
      Thread t = new Thread(r, "mvsgm-window-prepare");
      t.setDaemon(true);
      return t;
    });
    if (inFlight > 1) {
      // Predicted request order of fusion: camera c with its matching neighbours, then the same
      // for every camera that may overlap c. A wrong guess costs time only (see MvsgmCameraPool).
      List<Integer> order = new ArrayList<>();
      boolean[] listed = new boolean[n];
      for (int c = 0; c < n; c++) {
        if (!eligible[c]) continue;
        predict(c, listed, order);
        for (int j : graph.possible[c]) predict(j, listed, order);
      }
      pool = new MvsgmCameraPool(inFlight, o.cores, order.stream().mapToInt(Integer::intValue).toArray(), camera -> {
        long clock = System.nanoTime();
        MultiViewSgm.Plan plan = mvs.preparePlan(frames, headers[camera]);
        OutputDiskSpace.require(o, 16L * frames.get(camera).width * frames.get(camera).height
            + 4L * DepthMapFile.HEADER, "raw MVSGM maps for camera " + camera);
        return mvs.computeCamera(frames, tracks, camera, plan, clock);
      });
      System.out.printf(Locale.ROOT, "mvsgm: %d cameras in flight (--mvsgm-cameras-in-flight), %d row workers each%n",
          pool.inFlight, pool.rowCores);
    } else pool = null;
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

  static void checkDisk(Path output, MvsgmDiskPlan plan, int window, int total) throws IOException {
    long available = OutputDiskSpace.available(output);
    Json.write(output.resolve("05_stereo/mvsgm/disk_plan.json"), Json.object(
        "map_working_set_upper_bytes", plan.maps(), "all_pixels_cloud_staging_upper_bytes", plan.spoolUpper(),
        "all_pixels_exports_upper_bytes", plan.exportsUpper(), "all_pixels_records_upper_bytes", plan.recordsUpper(),
        "deferred_records_reservation_bytes", plan.deferredUpper(), "diagnostics_reservation_bytes", plan.diagnosticsUpper(),
        "all_pixels_peak_upper_bytes", plan.worstCase(), "startup_required_bytes", plan.startup(),
        "safety_reserve_bytes", plan.headroom(), "available_bytes", available, "window_cameras", window,
        "total_cameras", total, "actual_point_count_guards", true,
        "scope", "Cloud upper bound assumes every processed pixel becomes a final point; startup admission reserves maps and fixed/deferred work, then guards actual staging and export growth"));
    System.out.printf(Locale.ROOT,
        "mvsgm overlap window: up to %d/%d cameras; maps upper estimate %,d MiB;"
            + " startup disk reservation %,d MiB; available %,d MiB%n"
            + "mvsgm cloud disk: all-pixels staging+exports upper bound %,d MiB"
            + " (not a required output size); actual point-count checks enabled%n",
        window, total, plan.maps() >> 20, plan.startup() >> 20, available >> 20,
        (plan.spoolUpper() + plan.exportsUpper()) >> 20);
    if (plan.startup() > available)
      throw new IOException("mvsgm map/deferred-work startup reservation needs approximately "
          + (plan.startup() >> 20) + " MiB, available " + (available >> 20)
          + " MiB (no temporary maps were created; see 05_stereo/mvsgm/disk_plan.json)");
  }

  private void predict(int c, boolean[] listed, List<Integer> order) {
    if (!eligible[c]) return;
    SortedSet<Integer> requested = new TreeSet<>();
    requested.add(c);
    for (int j : mvs.info.get(c).neighbours()) if (eligible[j]) requested.add(j);
    for (int j : requested) if (!listed[j]) {
      listed[j] = true;
      order.add(j);
    }
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
    if (pool != null) {
      for (int camera : missing) {
        long clock = System.nanoTime();
        stereoWork.detail("depth camera " + camera);
        reports[camera] = pool.await(camera);
        depthNanos += System.nanoTime() - clock;
        raw[camera] = true;
        stereoWork.update(++depthCompleted + completed);
        sourceBytes[camera] = 16L * frames.get(camera).width * frames.get(camera).height + 4L * DepthMapFile.HEADER;
        currentSourceBytes += sourceBytes[camera];
        peakSourceBytes = Math.max(peakSourceBytes, currentSourceBytes);
      }
      return;
    }
    for (int i = 0; i < missing.length; i++) {
      int camera = missing[i];
      long clock = System.nanoTime();
      stereoWork.detail("depth camera " + camera);
      MultiViewSgm.Plan plan;
      if (hostMemory != null) {
        // No prepared replacement/prefetch alongside the previous camera's working set.
        hostMemory.require(mvs.hostWorkspace(frames, headers[camera]), "stereo camera " + camera);
        try (var rows = StereoRows.open(Math.max(1, o.cores / 2))) {
          plan = mvs.preparePlan(frames, headers[camera]);
        }
      } else {
        schedule(camera);
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
      }
      OutputDiskSpace.require(o, 16L * frames.get(camera).width * frames.get(camera).height
          + 4L * DepthMapFile.HEADER, "raw MVSGM maps for camera " + camera);
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
    Map<String, Object> out = new LinkedHashMap<>(Json.object("enabled", true, "peak_camera_bound", graph.peakCameras,
        "camera_count", cameraCount, "peak_additional_disk_estimate", estimatedPeakBytes,
        "peak_additional_disk_estimate_scope", "All-pixels cloud upper bound; streaming startup uses a separate reservation",
        "startup_disk_reservation_bytes", diskPlan.startup(), "actual_point_count_disk_checks", true,
        "peak_source_map_bytes", peakSourceBytes, "remaining_source_map_bytes", currentSourceBytes,
        "stereo_depth_seconds", depthNanos / 1e9, "stereo_consistency_seconds", consistencyNanos / 1e9,
        "precision_or_retention_requires_all_maps", retainFusion, "stage_times_overlap", true));
    if (pool != null) out.put("cameras_in_flight", pool.report());
    if (hostMemory != null) out.put("host_memory", hostMemory.report());
    return out;
  }

  @Override public void close() {
    if (closed) return;
    closed = true;
    if (pool != null) pool.close();
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
