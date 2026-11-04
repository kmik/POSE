package sgm;

import java.lang.foreign.MemorySegment;
import java.nio.file.*;
import java.util.*;

final class HybridSgmRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Path root = Files.createTempDirectory(Path.of("target"), "hybrid-sgm-");
    System.out.println("PASS hybrid SGM: " + run(root));
    GpuSgm.closeSession();
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Options planOptions = TestSupport.options();
    planOptions.memoryMb = 16000;
    planOptions.gpuMemoryMb = 8000;
    planOptions.gpuSgm = "auto";
    planOptions.gpuAggregate = "auto";
    var simulated =
        new GpuSgm.Device(
            MemorySegment.NULL, "8GiB control", "control", 8L << 30, 2L << 30, 64L << 10, 256);
    var p = GpuStereo.plan(simulated, 128, 3000, 4000, 174, planOptions);
    check(
        p != null && p.aggregateStorage().equals("gpu"),
        "logged 12 MP / 174-label pair uses hybrid on 8 GiB control");
    check(
        p.deviceBytes() <= GpuSgm.budget(simulated, planOptions),
        "whole hybrid working set fits budget");
    check(p.aggregateBytes() == 3000L * 4000 * 174 * 2, "all hypotheses retained on device");
    check(p.hostBytes() < p.aggregateBytes(), "no host aggregate copy");
    Options tiny = planOptions.copy();
    tiny.gpuMemoryMb = 1024;
    p = GpuStereo.plan(simulated, 128, 3000, 4000, 174, tiny);
    check(
        p != null && !p.aggregateStorage().equals("gpu"),
        "small GPU budget falls back to exact streaming");
    tiny.gpuSgm = "hybrid";
    check(
        GpuStereo.plan(simulated, 128, 3000, 4000, 174, tiny) == null,
        "explicit infeasible hybrid is not silently substituted");
    int w = 112, h = 39, n = w * h;
    byte[] a = new byte[n], b = new byte[n], ma = new byte[n], mb = new byte[n];
    new Random(9841).nextBytes(a);
    Arrays.fill(ma, (byte) 255);
    Arrays.fill(mb, (byte) 255);
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) b[y * w + x] = a[y * w + Math.min(w - 1, x + 7)];
    for (int y = 8; y < 21; y++) for (int x = 30; x < 35; x++) ma[y * w + x] = 0;
    for (int device = 0; device < GpuSgm.devices().size(); device++)
      for (String cost : List.of("census", "ad-census", "census-gradient", "zncc"))
        for (String bits : List.of("16", "32")) {
          Options o = TestSupport.options();
          o.output = root;
          o.compute = "cpu";
          o.matchingCost = cost;
          o.disparities = 25;
          o.minDisparity = -4;
          o.sgmAggregate = bits;
          o.median = 1;
          o.speckleSize = 0;
          o.directions = bits.equals("16") ? 8 : 4;
          var reference = new CensusSgm(o).compute(a, b, null, null, w, h, ma, mb);
          o.compute = "gpu";
          o.gpuDevice = device;
          o.gpuSgm = "hybrid";
          o.gpuStripRows = 7;
          var actual = new CensusSgm(o).compute(a, b, null, null, w, h, ma, mb);
          check(
              Arrays.equals(reference.disparity, actual.disparity),
              "CPU/hybrid disparity parity " + device + "/" + cost + "/" + bits);
          check(reference.stats.csv().equals(actual.stats.csv()), "all rejection counters exact");
          check(
              Arrays.equals(reference.costMargin, actual.costMargin),
              "aggregate competitor margins exact");
          check(
              Arrays.equals(reference.costSharpness, actual.costSharpness),
              "aggregate curvature exact");
          var report = ProcessingReport.object(GpuStereo.report(w, h, o));
          check("hybrid_gpu_aggregate".equals(report.get("storage")), "effective mode reported");
          check(
              ((Number) report.get("aggregate_volume_reads")).longValue() == 0
                  && ((Number) report.get("aggregate_volume_writes")).longValue() == 0,
              "no host aggregate transfers");
        }
    return checks;
  }
}
