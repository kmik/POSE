package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;

public final class BoundedFusionRegression {
  static int checks;

  static void check(boolean ok, String label) {
    checks++;
    if (!ok) throw new AssertionError(label);
  }

  static Camera[] cameras(int count) {
    Camera[] out = new Camera[count];
    for (int i = 0; i < count; i++) {
      var c = out[i] = new Camera(100, 100, 79.5, 59.5);
      c.translation[0] = -(i - 1) * .3;
      c.registered = true;
    }
    return out;
  }

  static Options options(Path path) throws Exception {
    Options o = new Options();
    o.output = Files.createDirectories(path);
    o.projectDirectory = o.output;
    o.diskDepth = true;
    o.fusionEvidence = "pair";
    o.fusionCompute = "cpu";
    o.minViews = 2;
    o.cores = 2;
    o.memoryMb = 256;
    o.fusionCacheMb = 8;
    o.fusionBatchSize = 128;
    o.fusionOutliers = "robust";
    o.gpuDevice = Integer.getInteger("sgm.test.gpu", 0);
    o.gpuMemoryMb = 256;
    return o;
  }

  static void evidence(DepthFusion f, int count) {
    for (int a = 0; a < count; a++)
      for (int b = a + 1; b < count; b++) {
        int pair = f.addPair(a, b);
        double z = b == count - 1 ? 12 : 10;
        for (int y = -48; y <= 48; y++)
          for (int x = -65; x <= 65; x++) f.add(pair, new double[] {x * .1, y * .1, z});
      }
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    org.opencv.core.Core.setNumThreads(1);
    Path root = Files.createDirectories(Path.of("target/windowed-revision/fusion-controls"));
    if (args.length > 0 && args[0].equals("benchmark")) {
      benchmark(root);
      GpuSgm.closeSession();
      return;
    }
    robust(root);
    incremental(root);
    if (args.length > 0 && args[0].equals("gpu")) {
      for (String layout : new String[] {"reference", "coalesced"}) regional(root, layout);
      many(root);
      unloaded(root);
    }
    System.out.println("PASS bounded fusion: " + checks);
    GpuSgm.closeSession();
  }

  static void benchmark(Path root) throws Exception {
    Options gpu = options(root.resolve("benchmark-gpu"));
    gpu.fusionCompute = "gpu";
    gpu.gpuMemoryMb = 16;
    gpu.fusionCacheMb = 64;
    gpu.cores = 4;
    gpu.fusionBatchSize = 1024;
    Options cpu = gpu.copy();
    cpu.fusionCompute = "cpu";
    cpu.output = Files.createDirectories(root.resolve("benchmark-cpu"));
    cpu.projectDirectory = cpu.output;
    int count = 40, n = 4096;
    Camera[] c = cameras(count);
    int[] w = new int[count], h = new int[count];
    Arrays.fill(w, 320);
    Arrays.fill(h, 200);
    for (var camera : c) {
      camera.cx = 159.5;
      camera.cy = 99.5;
    }
    try (var a = new DepthFusion(c, w, h, cpu);
        var b = new DepthFusion(c, w, h, gpu)) {
      for (int first = 0; first < count; first++)
        for (int second = first + 1; second < Math.min(count, first + 4); second++) {
          int pair = a.addPair(first, second);
          b.addPair(first, second);
          for (int y = -12; y <= 12; y++)
            for (int x = -12; x <= 12; x++) {
              double[] p = {x * .05, y * .05, 10};
              a.add(pair, p);
              b.add(pair, p);
            }
        }
      a.prepare(n);
      b.prepare(n);
      int[] sources = new int[n];
      double[][] p = new double[n][];
      Random random = new Random(361);
      for (int i = 0; i < n; i++)
        p[i] = new double[] {random.nextDouble() * .6 - .3, random.nextDouble() * .6 - .3, 10};
      a.fuseBatch(sources, p, 256);
      b.fuseBatch(sources, p, 256);
      long began = System.nanoTime();
      var reference = a.fuseBatch(sources, p, n);
      long cpuNanos = System.nanoTime() - began;
      began = System.nanoTime();
      var actual = b.fuseBatch(sources, p, n);
      long gpuNanos = System.nanoTime() - began;
      int retained = 0;
      for (int i = 0; i < n; i++) {
        check(DepthFusion.equivalent(reference[i], actual[i]), "Many-camera throughput parity");
        if (actual[i].accepted()) retained++;
      }
      Json.write(
          root.resolve("benchmark.json"),
          Json.object(
              "cameras",
              count,
              "pairs",
              a.pairCount(),
              "image_size",
              new int[] {320, 200},
              "candidates",
              n,
              "retained",
              retained,
              "cache_mib",
              64,
              "cores",
              4,
              "gpu_budget_mib",
              16,
              "cpu_seconds",
              cpuNanos / 1e9,
              "regional_gpu_seconds",
              gpuNanos / 1e9,
              "speedup",
              cpuNanos / (double) gpuNanos,
              "gpu",
              b.backendReport(),
              "cpu",
              a.backendReport(),
              "scope",
              "Reduced synthetic plane, warmed caches and compiled kernels; preparation separate."
                  + " Not an end-to-end forest or native 89-camera speed claim."));
      System.out.printf(
          Locale.ROOT,
          "PASS 40-camera benchmark: CPU %.3f s, regional GPU %.3f s, %.2fx; %d/%d retained%n",
          cpuNanos / 1e9,
          gpuNanos / 1e9,
          cpuNanos / (double) gpuNanos,
          retained,
          n);
      a.close();
      a.discardStorage();
      b.close();
      b.discardStorage();
    }
  }

  static void many(Path root) throws Exception {
    Options o = options(root.resolve("many"));
    o.fusionCompute = "gpu";
    o.fusionGpuEvidence = "auto";
    o.gpuMemoryMb = 16;
    int count = 40;
    Camera[] cameras = cameras(count);
    int[] w = new int[count], h = new int[count];
    Arrays.fill(w, 320);
    Arrays.fill(h, 200);
    for (var c : cameras) {
      c.cx = 159.5;
      c.cy = 99.5;
    }
    try (var f = new DepthFusion(cameras, w, h, o)) {
      for (int a = 0; a < count; a++)
        for (int b = a + 1; b < Math.min(count, a + 4); b++) {
          int pair = f.addPair(a, b);
          for (int y = -12; y <= 12; y++)
            for (int x = -12; x <= 12; x++) f.add(pair, new double[] {x * .05, y * .05, 10});
        }
      f.prepare(100000);
      int[] sources = new int[32];
      double[][] points = new double[32][];
      for (int i = 0; i < 32; i++)
        points[i] = new double[] {.013 + (i % 8) * .01, .017 + (i / 8) * .01, 10};
      var results = f.fuseBatch(sources, points, 32);
      for (int i = 0; i < 32; i++)
        check(
            DepthFusion.equivalent(f.fuse(0, points[i]), results[i]),
            "Many-camera regional parity");
      var report = (Map<?, ?>) f.backendReport();
      var device = (Map<?, ?>) report.get("gpu_resources");
      check(
          device.get("evidence_layout").equals("regional_pair_windows"),
          "Evidence exceeding simulated VRAM uses regional GPU automatically");
      check(
          ((Number) device.get("resident_evidence_bytes")).longValue() <= 8 * 1048576L,
          "Bounded GPU evidence reserve");
      Json.write(o.output.resolve("backend.json"), report);
    }
  }

  static void unloaded(Path root) throws Exception {
    Options o = options(root.resolve("unloaded"));
    o.fusionCompute = "gpu";
    o.fusionGpuEvidence = "windowed";
    o.fusionWindowMb = 2;
    o.fusionOutliers = "reference";
    o.fusionRelative = .1;
    o.depthRefinementMethod = "weighted";
    Camera[] cameras = cameras(4);
    cameras[0].translation[0] = 0;
    cameras[1].translation[0] = -1;
    cameras[2].translation[0] = -2;
    cameras[3].translation[0] = -400;
    cameras[3].cx = 5000;
    cameras[3].cy = 31.5;
    int[] w = {160, 160, 160, 8192}, h = {120, 120, 120, 64};
    try (var f = new DepthFusion(cameras, w, h, o)) {
      int ab = f.addPair(0, 1), ac = f.addPair(0, 2), bc = f.addPair(1, 2);
      for (int y = -50; y <= 50; y++)
        for (int x = -50; x <= 50; x++) {
          f.add(ab, new double[] {x * .025, y * .025, 10});
          f.add(ac, new double[] {x * .025, y * .025, 9.5});
          f.add(bc, new double[] {x * .025, y * .025, 9.5});
        }
      for (int image = 0; image < 3; image++) {
        int pair = f.addPair(image, 3);
        for (int y = 0; y < 64; y++)
          for (int x = 600; x < 1300; x++)
            f.add(pair, new double[] {400 + (x - 5000) / 100. * 16, (y - 31.5) / 100. * 16, 16});
      }
      f.prepare(128);
      int[] source = new int[32];
      double[][] points = new double[32][];
      for (int i = 0; i < 32; i++) points[i] = new double[] {.013 + i * .0001, .017, 10};
      var result = f.fuseBatch(source, points, 32);
      for (int i = 0; i < 32; i++)
        check(
            DepthFusion.equivalent(f.fuse(0, points[i]), result[i]),
            "Refinement outside loaded regions replays on CPU");
      var report = (Map<?, ?>) f.backendReport();
      var device = (Map<?, ?>) report.get("gpu_resources");
      Json.write(o.output.resolve("backend.json"), report);
      check(
          ((Number) device.get("window_cpu_guards")).longValue() > 0,
          "Unloaded visibility queries detected, never treated as missing depth");
      check(
          !result[0].accepted(),
          "Actual free-space contradiction outside initial window preserved");
      Json.write(o.output.resolve("backend.json"), report);
    }
  }

  static void robust(Path root) throws Exception {
    int[] w = {160, 160, 160, 160}, h = {120, 120, 120, 120};
    Camera[] cameras = cameras(4);
    Options o = options(root.resolve("robust"));
    Options old = o.copy();
    old.output = Files.createDirectories(root.resolve("old"));
    old.fusionOutliers = "reference";
    try (var robust = new DepthFusion(cameras, w, h, o);
        var reference = new DepthFusion(cameras, w, h, old)) {
      evidence(robust, 4);
      evidence(reference, 4);
      robust.prepare(1);
      reference.prepare(1);
      double[] p = {.013, .017, 10};
      var result = robust.fuse(0, p);
      var before = reference.fuse(0, p);
      check(
          before.status.equals("visibility_contradiction_rejected"),
          "Reference isolated-camera veto fixture");
      check(
          result.accepted() && result.views == 3,
          "Three good cameras survive one coherently bad camera");
      check(!result.supportingCameras.get(3), "Bad camera excluded from point provenance");
      double[] bad = {.013, .017, 12};
      check(
          !robust.fuse(0, bad).accepted(),
          "False producing-pair depth not rescued by a vote count");
    }
    // A genuine additional-camera free-space contradiction remains effective for a two-view point.
    o = options(root.resolve("free-space"));
    Camera[] three = cameras(3);
    int[] sizes = {160, 160, 160}, heights = {120, 120, 120};
    try (var f = new DepthFusion(three, sizes, heights, o)) {
      int a = f.addPair(0, 1), reversed = f.addPair(1, 0), b = f.addPair(0, 2), c = f.addPair(1, 2);
      for (int y = -48; y <= 48; y++)
        for (int x = -65; x <= 65; x++) {
          f.add(a, new double[] {x * .1, y * .1, 10});
          f.add(reversed, new double[] {x * .1, y * .1, 10});
          f.add(b, new double[] {x * .1, y * .1, 12});
          f.add(c, new double[] {x * .1, y * .1, 12});
        }
      f.prepare(1);
      var result = f.fuse(0, new double[] {.013, .017, 10});
      check(!result.accepted(), "Real free-space contradiction retained");
      check(result.views <= 2, "Reversed pair invents no third view");
    }
  }

  static void regional(Path root, String layout) throws Exception {
    Options o = options(root.resolve("regional-" + layout));
    o.fusionCompute = "gpu";
    o.fusionGpuEvidence = "windowed";
    o.fusionWindowMb = 1;
    o.fusionLayout = layout;
    int[] w = {160, 160, 160, 160}, h = {120, 120, 120, 120};
    try (var f = new DepthFusion(cameras(4), w, h, o)) {
      evidence(f, 4);
      f.prepare(256);
      int[] sources = new int[128];
      double[][] points = new double[128][];
      for (int i = 0; i < points.length; i++)
        points[i] = new double[] {(i % 16 - 8) * .021 + .013, (i / 16 - 4) * .021 + .017, 10};
      var result = f.fuseBatch(sources, points, points.length);
      for (int i = 0; i < points.length; i++)
        check(
            DepthFusion.equivalent(f.fuse(0, points[i]), result[i]),
            "Regional GPU exact support/numerical parity " + i);
      var report = (Map<?, ?>) f.backendReport();
      check(((Number) report.get("gpu_candidates")).longValue() == 128, "Regional GPU executed");
      Json.write(o.output.resolve("backend.json"), report);
    }
  }

  static void observation(OutputStream output, int source, int pixel, double[] p, float score)
      throws IOException {
    var out = new DataOutputStream(output);
    for (double x : p) out.writeDouble(x);
    out.write(new byte[] {10, 20, 30});
    out.writeInt(source);
    out.writeInt(pixel);
    out.writeFloat(score);
    for (int i = 0; i < 4; i++) out.writeFloat(.9f);
    out.writeFloat(score);
  }

  static void incremental(Path root) throws Exception {
    Options o = options(root.resolve("incremental"));
    o.twoViewConfidenceThreshold = .85;
    int[] w = {160, 160, 160}, h = {120, 120, 120};
    try (var f = new DepthFusion(cameras(3), w, h, o);
        var provisional = new IncrementalFusion(o, f)) {
      int ab = f.addPair(0, 1), ac = f.addPair(0, 2);
      double[] p = {.013, .017, 10};
      f.add(ab, p);
      // Weak two-view confidence is provisional, not irreversibly discarded.
      try (var out = provisional.stream()) {
        observation(out, ab, 42, p, .4f);
        observation(out, ac, 43, p, .3f);
      }
      f.add(ac, p);
      f.prepare(2);
      Path candidates = o.output.resolve("representatives.bin"),
          prepared = o.output.resolve("results.bin");
      long count = provisional.finish(candidates, prepared);
      check(count == 1, "Compatible observations finalized once after delayed support");
      try (var in = new DataInputStream(Files.newInputStream(prepared))) {
        check(in.readLong() == 0, "Original stereo candidate ID retained");
        var result = IncrementalFusion.readResult(in);
        check(
            result.accepted() && result.views == 3,
            "Third camera rescues weak provisional two-view match");
        check(result.supportingCameras.cardinality() == 3, "Exact distinct-camera provenance");
      }
    }
  }
}
