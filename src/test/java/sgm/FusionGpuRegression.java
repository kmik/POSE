package sgm;

import java.nio.file.*;
import java.util.*;
import org.opencv.core.Core;

/**
 * Differential checks against the frozen pre-acceleration algorithm and independent scene geometry.
 */
final class FusionGpuRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Core.setNumThreads(1);
    Path root = Files.createTempDirectory(Path.of("target"), "fusion gpu ");
    if (Arrays.asList(args).contains("--packed-banks")) {
      checks = 0;
      packedBanks(root);
      System.out.println("PASS packed bank checks=" + checks + "; fixtures=" + root);
    } else System.out.println("PASS fusion checks=" + run(root) + "; fixtures=" + root);
    GpuSgm.closeSession();
  }

  static int run(Path root) throws Exception {
    return run(root, false);
  }

  static int run(Path root, boolean camera) throws Exception {
    checks = 0;
    boolean gpu = false;
    try {
      gpu = !GpuSgm.devices().isEmpty();
    } catch (RuntimeException ignored) {
    }
    for (String fixture :
        List.of(
            "plane",
            "layers",
            "conflict",
            "uncertain",
            "large coordinates",
            "no refinement",
            "disk",
            "many pairs")) {
      compare(root, fixture, gpu, camera);
    }
    check(Options.parse(new String[] {"--help"}).help, "Help remains available");
    check(Options.help().contains("--fusion-compute"), "Fusion selector is documented");
    Options parsed = Options.parse(new String[] {"-i", root.toString(), "--fusion-compute", "cpu"});
    check(
        parsed.copy().fusionCompute.equals("cpu"),
        "Independent fusion override survives option copying");
    boolean invalid = false;
    try {
      Options.parse(new String[] {"-i", root.toString(), "--fusion-compute", "invalid"});
    } catch (IllegalArgumentException expected) {
      invalid = true;
    }
    check(invalid, "Invalid fusion backend is rejected");
    if (gpu) {
      banks(root);
      if (camera) packedBanks(root);
      runtimeFallback(root);
    }
    System.out.println("Fusion GPU hardware tested=" + gpu);
    return checks;
  }

  private static void compare(Path root, String fixture, boolean hardware, boolean camera)
      throws Exception {
    int C = fixture.equals("many pairs") ? 32 : 7,
        width = C == 32 ? 320 : 160,
        height = C == 32 ? 200 : 100;
    Camera[] cameras = new Camera[C];
    int[] w = new int[C], h = new int[C];
    double[] worldOffset =
        fixture.equals("large coordinates") ? new double[] {490000, 6710000, 150} : new double[3];
    for (int i = 0; i < C; i++) {
      Camera c = cameras[i] = new Camera(120, 120, (width - 1) * .5, (height - 1) * .5);
      double[] r = Math3.rotation(new double[] {.005 * (i - 3), .003 * (i - 3), .002 * (i - 3)});
      c.rotation = r;
      double spacing = fixture.equals("uncertain") ? .0005 : .35;
      c.translation =
          Math3.times(
              Math3.mv(
                  r,
                  new double[] {
                    worldOffset[0] + (i - 3) * spacing, worldOffset[1], worldOffset[2]
                  }),
              -1);
      c.registered = true;
      w[i] = width;
      h[i] = height;
    }
    Options o = TestSupport.options();
    o.output = Files.createDirectories(root.resolve("fusion-" + fixture));
    o.depthRefinement = !fixture.equals("no refinement");
    // This differential fixture deliberately freezes the earlier weighted-depth estimator.
    o.depthRefinementMethod = "weighted";
    o.gpuDevice = Integer.getInteger("sgm.test.gpu", 0);
    if (fixture.equals("disk")) {
      o.projectDirectory = o.output;
      o.diskDepth = true;
    }
    List<double[]> points = new ArrayList<>();
    List<Integer> source = new ArrayList<>();
    Options device = o.copy();
    if (camera) device.fusionEvidence = "camera";
    device.fusionCompute = hardware ? "gpu" : "cpu";
    try (ReferenceDepthFusion reference = new ReferenceDepthFusion(cameras, w, h, o);
        DepthFusion cpu = new DepthFusion(cameras, w, h, o);
        DepthFusion accelerated = new DepthFusion(cameras, w, h, device)) {
      int pairCount = 0;
      for (int a = 0; a < C; a++)
        for (int b = a + 1; b < C; b++) {
          if (C == 32 && pairCount >= 204) continue;
          pairCount++;
          int pair = cpu.addPair(a, b);
          reference.addPair(a, b);
          accelerated.addPair(a, b);
          for (int y = -6; y <= 6; y++)
            for (int x = -12; x <= 12; x++) {
              double z = 10 + ((a + b) % 3 - 1) * .003;
              if (fixture.equals("conflict") && a < 3 && b < 3) z += .25;
              double[] p = {x * .13 + worldOffset[0], y * .13 + worldOffset[1], z + worldOffset[2]};
              cpu.add(pair, p);
              reference.add(pair, p);
              accelerated.add(pair, p);
              points.add(p);
              source.add(pair);
              if (fixture.equals("layers")) {
                double[] q = {p[0], p[1], worldOffset[2] + 7};
                cpu.add(pair, q);
                reference.add(pair, q);
                accelerated.add(pair, q);
                points.add(q);
                source.add(pair);
              }
            }
        }
      // Off-image, behind-camera, and a non-finite candidate exercise conservative CPU guards.
      points.add(new double[] {worldOffset[0] + 1000, worldOffset[1], worldOffset[2] + 10});
      source.add(0);
      points.add(new double[] {worldOffset[0], worldOffset[1], worldOffset[2] - 1});
      source.add(0);
      points.add(new double[] {Double.NaN, 0, 10});
      source.add(0);
      accelerated.prepare(points.size());
      int accepted = 0;
      double referenceNanos = 0, cpuNanos = 0, gpuNanos = 0, maxError = 0;
      for (int begin = 0; begin < points.size(); begin += GpuFusion.MAX_BATCH) {
        int n = Math.min(GpuFusion.MAX_BATCH, points.size() - begin);
        int[] ids = new int[n];
        double[][] p = new double[n][];
        for (int i = 0; i < n; i++) {
          ids[i] = source.get(begin + i);
          p[i] = points.get(begin + i);
        }
        long start = System.nanoTime();
        DepthFusion.Result[] fast = accelerated.fuseBatch(ids, p, n);
        gpuNanos += System.nanoTime() - start;
        for (int i = 0; i < n; i++) {
          start = System.nanoTime();
          ReferenceDepthFusion.Result old = reference.fuse(ids[i], p[i]);
          referenceNanos += System.nanoTime() - start;
          start = System.nanoTime();
          DepthFusion.Result indexed = cpu.fuse(ids[i], p[i]);
          cpuNanos += System.nanoTime() - start;
          if (!old.status.equals(indexed.status)
              || old.views != indexed.views
              || old.independentPairs != indexed.independentPairs)
            throw new AssertionError(fixture + ": indexed CPU decisions differ from reference");
          for (int j = 0; j < 3; j++)
            if (Double.doubleToLongBits(old.point[j]) != Double.doubleToLongBits(indexed.point[j])
                && Math.abs(old.point[j] - indexed.point[j])
                    > 1e-11 * Math.max(1, Math.abs(old.point[j])))
              throw new AssertionError(fixture + ": indexed CPU coordinates changed");
          // Direct cross-product weights remove the old inverse-trigonometry round trip.
          // Decisions remain exact; certify the intended numeric change against the old oracle.
          if ((Double.doubleToLongBits(old.relativeSigma)
                      != Double.doubleToLongBits(indexed.relativeSigma)
                  && !(Math.abs(old.relativeSigma - indexed.relativeSigma)
                      <= 1e-12 * Math.max(1, Math.abs(old.relativeSigma))))
              || Math.abs(old.shift - indexed.shift) > 1e-11)
            throw new AssertionError(fixture + ": indexed CPU confidence changed");
          if (!DepthFusion.equivalent(indexed, fast[i]))
            throw new AssertionError(
                fixture
                    + ": GPU/CPU result differs, candidate="
                    + (begin + i)
                    + ", CPU="
                    + indexed.status
                    + ", GPU="
                    + fast[i].status);
          if (indexed.accepted()) {
            accepted++;
            for (int j = 0; j < 3; j++)
              maxError = Math.max(maxError, Math.abs(indexed.point[j] - fast[i].point[j]));
            if (fixture.equals("plane") && Math.abs(fast[i].point[2] - 10) >= .015)
              throw new AssertionError("Independent ground-plane depth changed");
          }
        }
      }
      check(true, fixture + ": all indexed CPU decisions/support match frozen reference");
      check(true, fixture + ": all indexed CPU coordinates/confidence remain bitwise identical");
      check(true, fixture + ": all GPU decisions/support/coordinates agree with CPU");
      check(fixture.equals("uncertain") || accepted > 100, fixture + ": meaningful valid coverage");
      if (hardware) {
        Map<?, ?> backend = (Map<?, ?>) accelerated.backendReport();
        check(
            ((Number) backend.get("gpu_candidates")).longValue() == points.size(),
            fixture + ": all batches execute on GPU");
        check(
            ((Number) backend.get("numerically_guarded_cpu_candidates")).longValue()
                < points.size() / 2,
            fixture + ": CPU guards do not conceal a CPU-only execution");
      }
      if (fixture.equals("uncertain")) check(accepted == 0, "Negligible parallax remains rejected");
      Json.write(
          o.output.resolve("comparison.json"),
          Json.object(
              "fixture",
              fixture,
              "candidates",
              points.size(),
              "accepted",
              accepted,
              "reference_seconds",
              referenceNanos / 1e9,
              "indexed_cpu_seconds",
              cpuNanos / 1e9,
              "gpu_seconds",
              gpuNanos / 1e9,
              "maximum_coordinate_difference",
              maxError,
              "backend",
              accelerated.backendReport()));
      System.out.printf(
          Locale.ROOT,
          "Fusion %s: %,d candidates, %,d accepted; reference %.3fs, CPU %.3fs, GPU %.3fs; max"
              + " coordinate error %.3g; %s%n",
          fixture,
          points.size(),
          accepted,
          referenceNanos / 1e9,
          cpuNanos / 1e9,
          gpuNanos / 1e9,
          maxError,
          Json.string(accelerated.backendReport()));
    }
    if (hardware && fixture.equals("plane")) {
      Options tiny = device.copy();
      tiny.gpuMemoryMb = 1;
      tiny.fusionCompute = "auto";
      try (DepthFusion fallback = new DepthFusion(cameras, w, h, tiny)) {
        int id = fallback.addPair(0, 1);
        fallback.add(id, points.getFirst());
        fallback.prepare(2000000);
        check(
            fallback.fuseBatch(new int[] {id}, new double[][] {points.getFirst()}, 1).length == 1,
            "Auto safely falls back with insufficient VRAM");
      }
      tiny.fusionCompute = "gpu";
      boolean rejected = false;
      try (DepthFusion strict = new DepthFusion(cameras, w, h, tiny)) {
        strict.addPair(0, 1);
        strict.prepare(2000000);
      } catch (IllegalArgumentException expected) {
        rejected = expected.getMessage().contains("GPU fusion required");
      }
      check(rejected, "Required GPU mode fails clearly when memory is insufficient");
    }
  }

  private static void banks(Path root) throws Exception {
    Options o = TestSupport.options();
    o.output = Files.createDirectories(root.resolve("multi-bank"));
    o.diskDepth = false;
    o.gpuDevice = Integer.getInteger("sgm.test.gpu", 0);
    o.minViews = 2;
    Camera a = new Camera(120, 120, 31.3, 15.2), b = a.copy();
    b.translation[0] = -1;
    Camera[] cameras = {a, b};
    int[] w = {64, 64}, h = {32, 32};
    double[] p = {.01, .02, 10};
    try (DepthRaster ma = new DepthRaster(2048, o);
        DepthRaster mb = new DepthRaster(2048, o);
        DepthFusion cpu = new DepthFusion(cameras, w, h, o)) {
      DepthRaster[] maps = {ma, mb};
      for (int i = 0; i < 2; i++) {
        double[] q = cameras[i].project(p);
        int x = (int) Math.round(q[0]), y = (int) Math.round(q[1]);
        maps[i].put(y * 64 + x, 10, DepthFusion.pack(q[0] - x, q[1] - y));
      }
      int source = cpu.addPair(0, 1);
      cpu.add(source, p);
      try (GpuFusion gpu =
          new GpuFusion(
              cameras,
              w,
              h,
              List.of(new DepthFusion.Endpoint(0, 0, ma), new DepthFusion.Endpoint(0, 1, mb)),
              new int[][] {{0, 1}},
              o,
              2,
              8192)) {
        DepthFusion.Result[] result = gpu.fuse(new int[] {0}, new double[][] {p}, 1);
        check(
            result[0] != null && DepthFusion.equivalent(cpu.fuse(source, p), result[0]),
            "Evidence addressing crosses four allocation banks without changing fusion");
      }
    }
  }

  private static void packedBanks(Path root) throws Exception {
    Options o = TestSupport.options();
    o.output = Files.createDirectories(root.resolve("packed-banks"));
    o.diskDepth = false;
    o.gpuDevice = Integer.getInteger("sgm.test.gpu", 0);
    o.minViews = 2;
    Camera a = new Camera(120, 120, 31.3, 15.2), b = a.copy();
    b.translation[0] = -1;
    Camera[] cameras = {a, b};
    int[] w = {64, 64}, h = {32, 32};
    double[] p = {.01, .02, 10};
    try (DepthRaster ma = new DepthRaster(2048, o);
        DepthRaster mb = new DepthRaster(2048, o);
        DepthFusion cpu = new DepthFusion(cameras, w, h, o)) {
      DepthRaster[] maps = {ma, mb};
      for (int i = 0; i < 2; i++) {
        double[] q = cameras[i].project(p);
        int x = (int) Math.round(q[0]), y = (int) Math.round(q[1]);
        maps[i].put(y * 64 + x, 10, DepthFusion.pack(q[0] - x, q[1] - y));
      }
      List<List<DepthFusion.Endpoint>> byCamera = List.of(new ArrayList<>(), new ArrayList<>());
      int[][] pairs = new int[32][];
      for (int i = 0; i < 32; i++) {
        pairs[i] = new int[] {0, 1};
        int id = cpu.addPair(0, 1);
        cpu.add(id, p);
        byCamera.get(0).add(new DepthFusion.Endpoint(i, 0, ma));
        byCamera.get(1).add(new DepthFusion.Endpoint(i, 1, mb));
      }
      CameraEvidence atlas = CameraEvidence.build(byCamera, w, h, 1 << 20, 1 << 20, false);
      check(atlas != null, "32-layer packed fixture fits its budget");
      List<DepthFusion.Endpoint> endpoints = new ArrayList<>(byCamera.get(0));
      endpoints.addAll(byCamera.get(1));
      try {
        for (String layout : List.of("coalesced", "reference")) {
          o.fusionLayout = layout;
          try (GpuFusion gpu = new GpuFusion(cameras, w, h, endpoints, pairs, o, 2, 8192, atlas)) {
            var results = gpu.fuse(new int[] {0, 31}, new double[][] {p, p}, 2);
            for (int i = 0; i < 2; i++)
              check(
                  results[i] != null
                      && DepthFusion.equivalent(cpu.fuse(i == 0 ? 0 : 31, p), results[i]),
                  "Packed high-bit layer and five GPU allocation banks preserve fusion");
          }
        }
      } finally {
        atlas.clear();
      }
    }
  }

  private static void runtimeFallback(Path root) throws Exception {
    Options o = TestSupport.options();
    o.output = Files.createDirectories(root.resolve("runtime-fallback"));
    o.fusionCompute = "auto";
    o.gpuDevice = Integer.getInteger("sgm.test.gpu", 0);
    o.minViews = 2;
    Camera a = new Camera(120, 120, 31.3, 15.2), b = a.copy();
    b.translation[0] = -1;
    try (DepthFusion fusion =
        new DepthFusion(new Camera[] {a, b}, new int[] {64, 64}, new int[] {32, 32}, o)) {
      int source = fusion.addPair(0, 1);
      double[] p = {.01, .02, 10};
      fusion.add(source, p);
      fusion.prepare(2000000);
      // A null OpenCL buffer is a defined error, exercising recovery without corrupting native
      // memory.
      var field = DepthFusion.class.getDeclaredField("gpu");
      field.setAccessible(true);
      GpuFusion gpu = (GpuFusion) field.get(fusion);
      check(gpu != null, "Runtime fallback fixture starts on GPU");
      var input = GpuFusion.class.getDeclaredField("sources");
      input.setAccessible(true);
      input.set(gpu, java.lang.foreign.MemorySegment.NULL);
      DepthFusion.Result expected = fusion.fuse(source, p);
      DepthFusion.Result result = fusion.fuseBatch(new int[] {source}, new double[][] {p}, 1)[0];
      check(
          DepthFusion.equivalent(expected, result),
          "Failed GPU batch is replayed on CPU with identical result");
      check(
          ((Number) ((Map<?, ?>) fusion.backendReport()).get("cpu_candidates")).longValue() == 1,
          "Fallback reports CPU execution");
      check(
          fusion.fuseBatch(new int[] {source}, new double[][] {p}, 1)[0].status.equals(
              expected.status),
          "Subsequent batches continue safely on CPU");
    }
  }
}
