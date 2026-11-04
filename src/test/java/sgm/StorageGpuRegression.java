package sgm;

import java.io.*;
import java.nio.*;
import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;

/** Real GPU/CPU agreement and independent storage-lifecycle integration checks. */
final class StorageGpuRegression {
  private static int checks;

  private static void check(boolean condition, String message) {
    checks++;
    if (!condition) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Core.setNumThreads(1);
    Path root = Files.createTempDirectory(Path.of("target"), "storage gpu ");
    checks = 0;
    boolean gpu = !GpuSgm.devices().isEmpty();
    if (Arrays.asList(args).contains("--require-gpu") && !gpu)
      throw new AssertionError("A real GPU is required");
    gpu(root, gpu);
    if (!Arrays.asList(args).contains("--gpu-only")) storage(root);
    GpuSgm.closeSession();
    System.out.println(
        "PASS storage/GPU checks=" + checks + "; GPU tested=" + gpu + "; fixtures=" + root);
  }

  static int run(Path root) throws Exception {
    checks = 0;
    boolean gpu = false;
    try {
      gpu = !GpuSgm.devices().isEmpty();
    } catch (RuntimeException e) {
      System.out.println("GPU regression skipped: " + e.getMessage());
    }
    gpu(root, gpu);
    storage(root);
    GpuSgm.closeSession();
    return checks;
  }

  private static void gpu(Path root, boolean available) throws Exception {
    if (!available) {
      System.out.println("No OpenCL GPU: real-GPU comparisons skipped");
      return;
    }
    int w = 160, h = 96, n = w * h;
    byte[] a = new byte[n], b = new byte[n], mask = new byte[n];
    new Random(124).nextBytes(a);
    Arrays.fill(mask, (byte) 255);
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) {
        b[y * w + x] = a[y * w + Math.min(w - 1, x + 8)];
        if (x > 70 && x < 84 && y > 20 && y < 60) mask[y * w + x] = 0;
      }
    byte[] ca = new byte[n * 3], cb = new byte[n * 3];
    for (int i = 0; i < n; i++)
      for (int k = 0; k < 3; k++) {
        ca[i * 3 + k] = a[i];
        cb[i * 3 + k] = b[i];
      }
    for (int directions : new int[] {4, 8})
      for (int census : new int[] {5, 11})
        for (String mode : List.of("gray", "color-distance")) {
          Options cpu = new Options();
          cpu.compute = "cpu";
          cpu.census = census;
          cpu.censusMode = mode;
          cpu.gpuDevice = Integer.getInteger("sgm.test.gpu", 0);
          cpu.directions = directions;
          cpu.disparities = census == 11 ? 177 : 37;
          cpu.speckleSize = 0;
          cpu.output = root;
          var expected = new CensusSgm(cpu).compute(a, b, ca, cb, w, h, mask, mask);
          Options gpu = cpu.copy();
          gpu.compute = "gpu";
          var actual = new CensusSgm(gpu).compute(a, b, ca, cb, w, h, mask, mask);
          check(
              Arrays.equals(expected.disparity, actual.disparity),
              "Exact GPU disparities: " + directions + " paths, window " + census + ", " + mode);
          check(
              expected.stats.csv().equals(actual.stats.csv()),
              "Exact GPU rejection counts: " + directions + "/" + census + "/" + mode);
        }
    Options signed = new Options();
    signed.compute = "cpu";
    signed.disparities = 29;
    signed.minDisparity = -16;
    signed.output = root;
    signed.gpuDevice = Integer.getInteger("sgm.test.gpu", 0);
    var expected = new CensusSgm(signed).compute(b, a, w, h);
    signed.compute = "gpu";
    signed.tiled = true;
    var actual = new CensusSgm(signed).compute(b, a, w, h);
    check(
        Arrays.equals(expected.disparity, actual.disparity),
        "Signed GPU disparity and tiled-option GPU execution are exact");
    check(expected.stats.csv().equals(actual.stats.csv()), "Signed GPU stats are exact");
    Options pyramid = new Options();
    pyramid.compute = "cpu";
    pyramid.disparities = 32;
    pyramid.pyramidLevels = 3;
    pyramid.tileSize = 64;
    pyramid.tileOverlap = 16;
    pyramid.pyramidRadius = 8;
    pyramid.speckleSize = 0;
    pyramid.output = root;
    pyramid.gpuDevice = Integer.getInteger("sgm.test.gpu", 0);
    var pyramidCpu = DenseMatcher.compute(pyramid, a, b, null, null, w, h, mask, mask, null);
    pyramid.compute = "gpu";
    var pyramidGpu = DenseMatcher.compute(pyramid, a, b, null, null, w, h, mask, mask, null);
    check(
        Arrays.equals(pyramidCpu.disparity, pyramidGpu.disparity),
        "GPU coarse-to-fine tile seams and fallback intervals match CPU exactly");
    check(
        pyramidCpu.stats.csv().equals(pyramidGpu.stats.csv()),
        "GPU pyramid rejection counts match CPU exactly");
    int valid = 0;
    double error = 0;
    for (int y = 16; y < h - 16; y++)
      for (int x = 20; x < w - 16; x++)
        if (Float.isFinite(pyramidGpu.disparity[y * w + x])) {
          valid++;
          error += Math.abs(pyramidGpu.disparity[y * w + x] - 8);
        }
    check(
        valid > 3000 && error / valid < .2,
        "GPU overlapping pyramid tiles recover the independent eight-pixel shift");
    Options limited = signed.copy();
    limited.gpuSgm = "resident";
    limited.gpuMemoryMb = 1;
    boolean rejected = false;
    try {
      new CensusSgm(limited).compute(b, a, w, h);
    } catch (IllegalArgumentException e) {
      rejected = true;
    }
    check(rejected, "Required GPU rejects insufficient VRAM budget before allocation");
    limited.compute = "auto";
    limited.disparities = 128;
    var fallback = new CensusSgm(limited).compute(b, a, w, h);
    limited.compute = "cpu";
    check(
        Arrays.equals(fallback.disparity, new CensusSgm(limited).compute(b, a, w, h).disparity),
        "Auto mode falls back exactly when GPU budget is too small");
    Options invalid = limited.copy();
    invalid.compute = "gpu";
    invalid.gpuDevice = 9999;
    rejected = false;
    try {
      new CensusSgm(invalid).compute(b, a, w, h);
    } catch (IllegalArgumentException e) {
      rejected = true;
    }
    check(rejected, "Required GPU rejects a missing device");
    invalid.compute = "auto";
    check(
        Arrays.equals(fallback.disparity, new CensusSgm(invalid).compute(b, a, w, h).disparity),
        "Auto mode handles missing selected GPU with exact CPU fallback");
    GpuSgm.closeSession();
    // Moderate known-shift benchmark includes census, transfers and CPU post-filtering.
    int bw = 640, bh = 384;
    byte[] ba = new byte[bw * bh], bb = new byte[ba.length];
    new Random(762).nextBytes(ba);
    for (int y = 0; y < bh; y++)
      for (int x = 0; x < bw; x++) bb[y * bw + x] = ba[y * bw + Math.min(bw - 1, x + 12)];
    Options bench = new Options();
    bench.compute = "gpu";
    bench.disparities = 128;
    bench.speckleSize = 0;
    bench.output = root;
    bench.gpuDevice = Integer.getInteger("sgm.test.gpu", 0);
    new CensusSgm(bench).compute(a, b, w, h); // initialize/compile outside timing
    long start = System.nanoTime();
    var g = new CensusSgm(bench).compute(ba, bb, bw, bh);
    double gpuSeconds = (System.nanoTime() - start) / 1e9;
    bench.compute = "cpu";
    start = System.nanoTime();
    var c = new CensusSgm(bench).compute(ba, bb, bw, bh);
    double cpuSeconds = (System.nanoTime() - start) / 1e9;
    check(
        Arrays.equals(g.disparity, c.disparity) && g.stats.csv().equals(c.stats.csv()),
        "Benchmark GPU/CPU results are exact");
    Json.write(
        root.resolve("gpu-benchmark.json"),
        Json.object(
            "width",
            bw,
            "height",
            bh,
            "disparities",
            128,
            "gpu_seconds",
            gpuSeconds,
            "cpu_seconds",
            cpuSeconds,
            "speedup",
            cpuSeconds / gpuSeconds,
            "includes",
            "census, cost construction, 8-path SGM, transfers and CPU post-filtering; excludes"
                + " first context/kernel compilation"));
    System.out.printf(
        Locale.ROOT,
        "GPU benchmark: %.3f s GPU vs %.3f s CPU (%.2fx)%n",
        gpuSeconds,
        cpuSeconds,
        cpuSeconds / gpuSeconds);
  }

  private static Path latest(Path project) throws IOException {
    try (var paths = Files.list(project.resolve("runs"))) {
      return paths
          .filter(Files::isDirectory)
          .max(Comparator.comparing(Path::toString))
          .orElseThrow();
    }
  }

  private static long bytes(Path root) throws IOException {
    try (var files = Files.walk(root)) {
      long sum = 0;
      for (Path p : files.filter(Files::isRegularFile).toList()) sum += Files.size(p);
      return sum;
    }
  }

  private static void storage(Path root) throws Exception {
    Path images = Files.createDirectories(root.resolve("storage images"));
    int w = 192, h = 128;
    byte[] left = new byte[w * h], right = new byte[left.length];
    new Random(9122).nextBytes(left);
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) right[y * w + x] = left[y * w + Math.min(w - 1, x + 8)];
    Mat m = new Mat(h, w, CvType.CV_8U);
    try {
      m.put(0, 0, left);
      Imgcodecs.imwrite(images.resolve("0.png").toString(), m);
      m.put(0, 0, right);
      Imgcodecs.imwrite(images.resolve("1.png").toString(), m);
    } finally {
      m.release();
    }
    Path calib = root.resolve("storage-camera.txt");
    new Calibration(190, 190, 95.5, 63.5, new double[5], w, h).write(calib, "Measured fixture");
    byte[] original = null;
    long debugBytes = 0, standardBytes = 0;
    for (String mode : List.of("debug", "standard", "minimal")) {
      Path project = root.resolve("storage-" + mode);
      List<String> args =
          new ArrayList<>(
              List.of(
                  "-i",
                  images.toString(),
                  "-o",
                  project.toString(),
                  "--calibration",
                  calib.toString(),
                  "--rectified",
                  "--baseline",
                  "0.25",
                  "--disparities",
                  "24",
                  "--speckle-size",
                  "0",
                  "--no-gps",
                  "--compute",
                  "cpu",
                  "--diagnostics",
                  "--compatibility-copies"));
      // Exercise all legacy export profiles explicitly; compact defaults have separate controls.
      args.addAll(List.of("--storage", mode));
      check(Main.run(args.toArray(String[]::new)) == 0, "Storage " + mode + " completes");
      Path run = latest(project);
      byte[] las = Files.readAllBytes(project.resolve("dense.las"));
      if (original == null) original = las;
      else check(Arrays.equals(original, las), "Storage mode does not change final cloud: " + mode);
      check(!Files.exists(project.resolve("cache")), "Cache is absent by default: " + mode);
      try (var files = Files.walk(run.resolve("06_fusion"))) {
        boolean found = files.anyMatch(p -> p.toString().endsWith(".tiles"));
        check(found == mode.equals("debug"), "Successful fusion working-depth lifecycle: " + mode);
      }
      check(
          Files.exists(run.resolve("07_results/point-confidence.csv.gz")),
          "Final confidence is retained: " + mode);
      check(
          Files.exists(run.resolve("06_fusion/candidates.csv.gz")) == mode.equals("debug"),
          "Exhaustive candidate table opt-in: " + mode);
      check(
          Files.exists(run.resolve("00_inputs/image_0000/undistort_source_x.pfm")) == false,
          "Rectified fixture has no undistort maps");
      if (mode.equals("debug")) debugBytes = bytes(project);
      if (mode.equals("standard")) {
        standardBytes = bytes(project);
        check(
            Files.exists(run.resolve("05_stereo/pair_0000_0001/disparity/05_final_stereo.pfm")),
            "Standard preserves final numerical disparity");
        check(
            !Files.exists(run.resolve("05_stereo/pair_0000_0001/left_source_x.pfm")),
            "Standard does not generate rectification-map rasters");
        args.add("--cache");
        check(Main.run(args.toArray(String[]::new)) == 0, "Opt-in stage caching succeeds");
        args.addAll(List.of("--rerun-from", "stereo"));
        check(Main.run(args.toArray(String[]::new)) == 0, "Selective cache refresh succeeds");
        Path cache = project.resolve("cache/v3/standard-diagnostics/stereo");
        try (var keys = Files.list(cache)) {
          for (Path key : keys.filter(Files::isDirectory).toList())
            try (var generations = Files.list(key)) {
              check(
                  generations.filter(Files::isDirectory).count() == 1,
                  "Superseded same-key cache generations are pruned");
            }
        }
      }
      if (mode.equals("minimal"))
        check(
            !Files.exists(run.resolve("05_stereo/pair_0000_0001/disparity")),
            "Minimal omits disparity rasters");
    }
    check(
        standardBytes < debugBytes * .6,
        "Standard storage materially reduces disk without changing cloud");
    Json.write(
        root.resolve("storage-comparison.json"),
        Json.object(
            "debug_bytes",
            debugBytes,
            "standard_bytes",
            standardBytes,
            "ratio",
            standardBytes / (double) debugBytes,
            "clouds_bitwise_identical",
            true));
    Options defaults = Options.parse(new String[] {"-i", images.toString(), "--no-gps"});
    check(
        defaults.storage.equals("standard") && !defaults.cache && defaults.compute.equals("auto"),
        "CLI defaults are standard storage, opt-in cache and auto GPU");
    boolean conflict = false;
    try {
      Options.parse(new String[] {"-i", images.toString(), "--resume", "--no-cache"});
    } catch (IllegalArgumentException e) {
      conflict = true;
    }
    check(conflict, "Resume cannot silently ignore an explicit no-cache constraint");
    boolean available = false;
    try {
      available = !GpuSgm.devices().isEmpty();
    } catch (RuntimeException ignored) {
    }
    if (available) {
      Path gpuProject = root.resolve("storage-gpu-pipeline");
      check(
          Main.run(
                  new String[] {
                    "-i",
                    images.toString(),
                    "-o",
                    gpuProject.toString(),
                    "--calibration",
                    calib.toString(),
                    "--rectified",
                    "--baseline",
                    "0.25",
                    "--disparities",
                    "24",
                    "--speckle-size",
                    "0",
                    "--no-gps",
                    "--compute",
                    "gpu"
                  })
              == 0,
          "Full standard-storage pipeline runs on the GPU");
      check(
          Arrays.equals(original, Files.readAllBytes(gpuProject.resolve("dense.las"))),
          "GPU pipeline final LAS is bitwise identical to all three CPU storage modes");
      try (var files = Files.walk(gpuProject)) {
        check(
            files.noneMatch(p -> p.toString().endsWith(".tiles")),
            "Successful GPU pipeline leaves no working depth/cost volumes");
      }
    }
  }
}
