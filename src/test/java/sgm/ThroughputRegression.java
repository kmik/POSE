package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;

/** Exact CPU controls and optional real-device transfer/resident-matching controls. */
public final class ThroughputRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Core.setNumThreads(1);
    Path root = Files.createTempDirectory(Path.of("target"), "throughput-");
    run(root);
    if (args.length > 0 && args[0].equals("gpu")) gpu();
    System.out.println("PASS throughput: " + checks + " checks; " + root);
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Files.createDirectories(root);
    Options parsed = Options.parse(new String[] {"-i", ".", "--image-fusion-cache-mb", "0",
        "--gpu-transfer-mode", "blocking"});
    check(parsed.imageFusionCacheMb == 0 && !parsed.gpuQueuedUploads, "Reference controls parse");
    for (String[] args : new String[][] {{"--image-fusion-cache-mb", "-2"},
        {"--gpu-transfer-mode", "invalid"}}) {
      boolean rejected = false;
      try { Options.parse(new String[] {"-i", ".", args[0], args[1]}); }
      catch (IllegalArgumentException expected) { rejected = true; }
      check(rejected, "Reject invalid throughput option");
    }
    validation(root.resolve("validation"));
    telemetry(root.resolve("telemetry"));
    for (String strategy : List.of("owner", "owner-consume", "graph-average")) {
      byte[][] reference = fusion(root.resolve(strategy + "-off"), strategy, 0);
      for (int cache : new int[] {1, -1, 1024}) {
        byte[][] actual = fusion(root.resolve(strategy + "-" + cache), strategy, cache);
        check(Arrays.equals(reference[0], actual[0]), strategy + ": cached PLY is byte-identical");
        check(Arrays.equals(reference[1], actual[1]), strategy + ": cached confidence is byte-identical");
      }
    }
    Path selection = Files.createDirectories(root.resolve("selection"));
    PerformanceRegression.selection(selection);
    var selected = ProcessingReport.map(selection.resolve("pair-selection/05_stereo/dense_pair_selection.json"));
    check(selected.get("required_sparse_cells").equals(selected.get("covered_sparse_cells")),
        "Coverage selection preserves every requested sparse-cell witness target");
    check(((Number) selected.get("required_pair_witnesses_per_cell")).intValue() == 2,
        "Three-view fusion asks for two distinct pair witnesses per sparse cell");
    return checks;
  }

  private static void telemetry(Path root) throws Exception {
    Files.createDirectories(root.resolve("logs"));
    Options o = TestSupport.options();
    Path file = root.resolve("logs/processing.json");
    try (var t = new RunTelemetry(root, o)) {
      RunTelemetry.gpuProfile("fixture", Json.object("device_kernel_nanoseconds", Json.object("test", 7),
          "queued_uploaded_bytes", 13));
      t.persist();
    }
    try (var t = new RunTelemetry(root, o)) {
      t.continueFrom(file, 1);
      RunTelemetry.gpuProfile("fixture", Json.object("device_kernel_nanoseconds", Json.object("test", 11),
          "queued_uploaded_bytes", 17));
      t.persist();
      t.persist();
    }
    var saved = ProcessingReport.map(file);
    var totals = ProcessingReport.object(saved.get("opencl_profile_totals"));
    check(((List<?>) saved.get("opencl_profiles")).size() == 2, "Step continuation preserves GPU profiles without duplication");
    check(((Number) totals.get("kernel_test_nanoseconds")).longValue() == 18,
        "Step continuation adds kernel durations once");
    check(((Number) totals.get("queued_uploaded_bytes")).longValue() == 30,
        "Step continuation preserves queued-transfer measurements");
  }

  private static void validation(Path root) throws Exception {
    Files.createDirectories(root);
    Options o = TestSupport.options();
    o.output = root;
    Calibration c = new Calibration(650, 650, 319.5, 239.5,
        new double[] {-.12, .04, .002, -.003, 0}, 640, 480);
    var frames = SharedCalibrationRegression.frames(root, c, o);
    try {
      var tracks = SharedCalibrationRegression.scene(frames, c);
      for (var t : tracks) t.point = Math3.add(t.point, new double[] {.01, -.02, .03});
      SharedCalibration lens = new SharedCalibration(frames);
      o.cores = 1;
      double serial = lens.validationRms(frames, tracks);
      double[][] points = tracks.stream().map(t -> t.point.clone()).toArray(double[][]::new);
      o.cores = 4;
      double parallel = lens.validationRms(frames, tracks);
      check(Double.doubleToRawLongBits(serial) == Double.doubleToRawLongBits(parallel),
          "Held-out residual reduction remains bit-identical across thread counts");
      for (int i = 0; i < tracks.size(); i++)
        check(Arrays.equals(points[i], tracks.get(i).point), "Validation never commits held-out structure");
      check(DenseReconstruction.pairSubmissionWindow(o, frames, 3) == 3,
          "CPU execution keeps original pair admission");
      o.compute = "gpu";
      o.memoryMb = 128;
      check(DenseReconstruction.pairSubmissionWindow(o, frames, 3) == 3,
          "Low-memory execution does not add look-ahead results");
    } finally { for (var f : frames) f.close(); }
  }

  private static byte[][] fusion(Path root, String strategy, int cache) throws Exception {
    Files.createDirectories(root);
    Options o = TestSupport.options();
    o.output = root;
    o.memoryMb = 256;
    o.imageFusionCacheMb = cache;
    o.cores = 2;
    o.minViews = 2;
    o.fusionMode = "image";
    o.fusionStrategy = strategy;
    o.fusionSpikeFilter = false;
    o.fusionDepthOutliers = false;
    o.speckleSize = 0;
    o.fusionFreeSpace = "off";
    int w = 112, h = 80;
    Camera[] cameras = new Camera[3];
    int[] widths = {w, w, w}, heights = {h, h, h};
    for (int i = 0; i < cameras.length; i++) {
      cameras[i] = new Camera(100, 100, (w - 1) / 2., (h - 1) / 2.);
      cameras[i].translation = new double[] {-.4 * i, 0, 0};
    }
    try (var depth = new DepthFusion(cameras, widths, heights, o)) {
      int[][] pairs = {{0, 1}, {0, 2}, {1, 2}};
      long[] starts = new long[4];
      int fields = Double.isFinite(depth.twoViewConfidenceThreshold()) ? 6 : 5;
      Path file = root.resolve("candidates.bin");
      long count = 0;
      try (var out = new DataOutputStream(new BufferedOutputStream(Files.newOutputStream(file)))) {
        for (int k = 0; k < pairs.length; k++) {
          starts[k] = count;
          depth.addPair(pairs[k][0], pairs[k][1]);
          for (int y = 0; y < h; y++) for (int x = 0; x < w; x++) {
            double z = 10 + (x >= w / 2 ? .5 : 0);
            double[] p = {.4 * pairs[k][0] + (x - (w - 1) / 2.) * z / 100,
                (y - (h - 1) / 2.) * z / 100, z};
            for (double v : p) out.writeDouble(v);
            out.write(new byte[] {10, 20, 30});
            out.writeInt(k);
            out.writeInt(y * w + x);
            for (int f = 0; f < fields; f++) out.writeFloat(.9f);
            depth.add(k, p);
            count++;
          }
        }
        starts[3] = count;
      }
      try (var output = new FusionOutput(o, depth, null, null)) {
        var counters = ImageFusion.run(o, depth, file, starts, output, new TwoViewConfidence.Diagnostics(o));
        output.finish();
        check(counters.emitted > 0, "Fusion fixture emits points");
        check(counters.cachePeakBytes <= counters.cacheLimitBytes, "Cache remains within its admitted bound");
        check(counters.cacheLimitBytes <= o.memoryMb * 1048576L / 4,
            "Explicit cache request cannot exceed the processing-memory share");
        if (cache == 0) check(counters.pairReads == 12 && counters.cacheHits == 0, "Uncached reference replays all pairs");
        else if (cache < 0 || cache == 1024) check(counters.pairReads == 3 && counters.cacheHits == 9,
            "Resident candidate cache reads each pair once");
        else check(counters.cacheHits > 0 && counters.pairReads > 3,
            "Small cache exercises eviction and reuse");
      }
    }
    return new byte[][] {Files.readAllBytes(root.resolve("dense.ply")),
        Files.readAllBytes(root.resolve("07_results/point-confidence.bin"))};
  }

  private static void gpu() throws Exception {
    int device = Integer.getInteger("sgm.test.gpu", 0);
    Options cpu = TestSupport.options();
    cpu.ratio = .8;
    Options gpu = cpu.copy();
    gpu.compute = "gpu";
    gpu.gpuDevice = device;
    for (int bytes : new int[] {32, 64}) for (double ratio : new double[] {0, .8, .99, 1}) {
      Mat a = new Mat(257, bytes, CvType.CV_8U), b = new Mat(193, bytes, CvType.CV_8U);
      byte[] ad = new byte[257 * bytes], bd = new byte[193 * bytes];
      new Random(72 + bytes).nextBytes(ad);
      new Random(18 + bytes).nextBytes(bd);
      for (int i = 0; i < 35; i++) System.arraycopy(ad, i * bytes, bd, i * bytes, bytes);
      // Duplicate/tied descriptors exercise stable nearest-neighbour insertion order.
      System.arraycopy(ad, 0, ad, bytes, bytes);
      System.arraycopy(bd, 0, bd, bytes, bytes);
      a.put(0, 0, ad);
      b.put(0, 0, bd);
      List<MatOfDMatch> expected = new ArrayList<>(), actual = new ArrayList<>();
      Map<Integer, Integer> er = new HashMap<>(), ar = new HashMap<>();
      try {
        cpu.ratio = gpu.ratio = ratio;
        int queries = FeatureAlgorithms.reciprocal(cpu, a, b, expected, er, s -> {});
        Integer searched = GpuHamming.reciprocal(gpu, a, b, actual, ar, s -> {});
        check(searched != null, "Real GPU reciprocal execution is required");
        check(queries == searched && er.equals(ar), "GPU reciprocal decisions and query IDs match CPU");
        check(expected.size() == actual.size(), "GPU forward query count");
        for (int i = 0; i < expected.size(); i++) {
          DMatch[] e = expected.get(i).toArray(), r = actual.get(i).toArray();
          for (int k = 0; k < e.length; k++)
            check(e[k].queryIdx == r[k].queryIdx && e[k].trainIdx == r[k].trainIdx
                && e[k].distance == r[k].distance, "GPU exact Hamming neighbors and distances");
        }
      } finally {
        for (Mat row : expected) row.release();
        for (Mat row : actual) row.release();
        a.release(); b.release();
      }
    }
    GpuHamming.close();
    for (boolean queued : new boolean[] {false, true}) {
      Options o = TsgmRegression.options("gpu", "census", 8, "16");
      o.gpuDevice = device;
      o.gpuQueuedUploads = queued;
      int w = 120, h = 72;
      byte[][] images = TsgmRegression.scene(w, h, 11, new float[w * h]);
      long[][] a = CensusSgm.census(images[0], w, h, o.census),
          b = CensusSgm.census(images[1], w, h, o.census);
      boolean[] ta = CensusSgm.textureMask(images[0], w, h, o.census),
          tb = CensusSgm.textureMask(images[1], w, h, o.census);
      try (var session = new GpuBandedStereo.Session(o, a, b, images[0], images[1], ta, tb, w, h)) {
        for (int labels : new int[] {4, 10, 40, 8, 40}) {
          o.disparities = labels;
          var bands = DisparityBands.full(w, h, labels, ta);
          int[] expected = BandedSgm.cpuSummary(o, a, b, images[0], images[1], ta, tb, w, h, false, bands);
          check(Arrays.equals(expected, session.summary(o, false, bands)),
              "Queued/blocking uploads and growing/reused volumes match the CPU exactly");
        }
      }
    }
    GpuSgm.closeSession();
  }
}
