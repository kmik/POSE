package sgm;

import java.nio.file.*;
import java.util.*;

/** Exact storage parity and resource bounds, including long-indexed native aggregate segments. */
final class ScalingRegression {
  private static int checks;

  private static void check(boolean ok, String why) {
    checks++;
    if (!ok) throw new AssertionError(why);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Path root = Files.createDirectories(Path.of("target/scaling/evidence"));
    System.out.println("PASS scaling checks=" + run(root));
    GpuSgm.closeSession();
  }

  static int run(Path root) throws Exception {
    checks = 0;
    // Greater than int-indexed cells, tiny transfer at a high offset: no Java-array addressing.
    try (var arena = java.lang.foreign.Arena.ofConfined();
        var volume = new GpuAggregate(64, false, root)) {
      var bytes = arena.allocate(16);
      bytes.set(java.lang.foreign.ValueLayout.JAVA_LONG, 0, 0x1020304050607080L);
      volume.transfer(48, bytes, true);
      bytes.fill((byte) 0);
      volume.transfer(48, bytes, false);
      check(
          bytes.get(java.lang.foreign.ValueLayout.JAVA_LONG, 0) == 0x1020304050607080L,
          "RAM aggregate exact bytes");
      boolean rejected = false;
      try {
        volume.transfer(60, bytes, false);
      } catch (IllegalArgumentException e) {
        rejected = true;
      }
      check(rejected, "Aggregate bounds checked");
    }
    int w = 160, h = 47, n = w * h;
    byte[] a = new byte[n], b = new byte[n], ma = new byte[n], mb = new byte[n];
    new Random(219).nextBytes(a);
    Arrays.fill(ma, (byte) 255);
    Arrays.fill(mb, (byte) 255);
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) b[y * w + x] = a[y * w + Math.min(w - 1, x + 11)];
    for (int y = 10; y < 20; y++) for (int x = 23; x < 32; x++) ma[y * w + x] = 0;
    boolean hardware = false;
    try {
      hardware = !GpuSgm.devices().isEmpty();
    } catch (RuntimeException absent) {
    }
    if (hardware)
      for (String cost : List.of("census", "ad-census", "census-gradient", "zncc"))
        for (int dirs : new int[] {4, 8})
          for (String bits : List.of("16", "32")) {
            Options o = TestSupport.options();
            o.output = root;
            o.compute = "gpu";
            o.gpuDevice = Integer.getInteger("sgm.test.gpu", 0);
            o.disparities = 33;
            o.minDisparity = -5;
            o.matchingCost = cost;
            o.directions = dirs;
            o.sgmAggregate = bits;
            o.gpuSgm = "resident";
            o.median = 1;
            o.speckleSize = 0;
            o.memoryMb = 128;
            var expected = new CensusSgm(o).compute(a, b, null, null, w, h, ma, mb);
            for (String storage : List.of("ram", "disk", "recompute")) {
              Options actual = o.copy();
              actual.gpuSgm = "streamed";
              actual.gpuAggregate = storage;
              actual.gpuStripRows = 7;
              // Force several verification blocks without needing a huge fixture or allocation.
              if (storage.equals("recompute")) actual.memoryMb = 1;
              var plan =
                  GpuStereo.plan(
                      GpuSgm.selectedDevice(actual),
                      GpuSgm.selectedGroup(actual),
                      w,
                      h,
                      actual.disparities,
                      actual);
              check(plan != null, "Feasible exact streamed plan");
              if (storage.equals("recompute"))
                check(plan.blockRows() < h, "Test exercises multiple global-context blocks");
              var result = new CensusSgm(actual).compute(a, b, null, null, w, h, ma, mb);
              check(
                  Arrays.equals(result.disparity, expected.disparity),
                  cost + dirs + bits + storage + ": exact disparities");
              check(
                  result.stats.csv().equals(expected.stats.csv()),
                  "Exact selection/filtering counters");
              check(
                  plan.hostBytes() <= actual.memoryMb * 1048576L,
                  "Aggregate plus workspace within host budget");
              try (var files = Files.list(root)) {
                check(
                    files.noneMatch(
                        p -> p.getFileName().toString().startsWith("gpu-sgm-aggregate-")),
                    "Temporary aggregate removed");
              }
            }
          }
    return checks;
  }
}
