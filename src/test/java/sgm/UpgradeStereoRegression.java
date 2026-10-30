package sgm;

import java.nio.file.*;
import java.util.*;

final class UpgradeStereoRegression {
  private static int checks;

  private static void check(boolean yes, String text) {
    checks++;
    if (!yes) throw new AssertionError(text);
  }

  public static void main(String[] args) throws Exception {
    Path root = Files.createTempDirectory(Path.of("target"), "upgrade stereo ");
    System.out.println("PASS upgrade stereo checks=" + run(root) + "; fixtures=" + root);
    GpuSgm.closeSession();
  }

  static int run(Path root) throws Exception {
    checks = 0;
    int w = 192, h = 79, n = w * h, shift = 13;
    byte[] a = new byte[n], b = new byte[n], ma = new byte[n], mb = new byte[n];
    new Random(120).nextBytes(a);
    Arrays.fill(ma, (byte) 255);
    Arrays.fill(mb, (byte) 255);
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) {
        b[y * w + x] = a[y * w + Math.min(w - 1, x + shift)];
        if (x > 85 && x < 97 && y > 20 && y < 55) ma[y * w + x] = 0;
        if (x > 52 && x < 59 && y > 5 && y < 18) mb[y * w + x] = 0;
      }
    int gpuIndex = Integer.getInteger("sgm.test.gpu", 0);
    boolean hardware = !GpuSgm.devices().isEmpty();
    for (String cost : List.of("census", "ad-census", "census-gradient", "zncc"))
      for (int directions : new int[] {4, 8}) {
        Options cpu = TestSupport.options();
        cpu.output = Files.createDirectories(root.resolve(cost + directions));
        cpu.disparities = 41;
        cpu.minDisparity = -3;
        cpu.matchingCost = cost;
        cpu.directions = directions;
        cpu.speckleSize = 0;
        cpu.correlation = .3;
        cpu.median = 1;
        cpu.gpuDevice = gpuIndex;
        CensusSgm.Result expected = new CensusSgm(cpu).compute(a, b, null, null, w, h, ma, mb);
        int good = 0, valid = 0;
        for (float d : expected.disparity)
          if (Float.isFinite(d)) {
            valid++;
            if (Math.abs(d - shift) < .6) good++;
          }
        check(valid > n / 2 && good > valid * .98, cost + ": known shift and meaningful coverage");
        if (hardware)
          for (String mode : List.of("resident", "streamed"))
            for (int rows : mode.equals("streamed") ? new int[] {1, 7, 32} : new int[] {h}) {
              Options gpu = cpu.copy();
              gpu.compute = "gpu";
              gpu.gpuSgm = mode;
              gpu.gpuStripRows = rows;
              gpu.gpuBankMb = 1;
              CensusSgm.Result actual = new CensusSgm(gpu).compute(a, b, null, null, w, h, ma, mb);
              check(
                  Arrays.equals(expected.disparity, actual.disparity),
                  cost
                      + "/"
                      + directions
                      + "/"
                      + mode
                      + "/"
                      + rows
                      + ": exact CPU/GPU disparities and seams");
              check(
                  expected.stats.csv().equals(actual.stats.csv())
                      && expected.stats.lowerBoundaryRejected == actual.stats.lowerBoundaryRejected
                      && expected.stats.fullIntervalPixels == actual.stats.fullIntervalPixels,
                  cost + ": exact rejection and boundary counters");
            }
      }
    Options c = TestSupport.options();
    c.census = 5;
    check(
        StereoCost.penalty(c.p1, c) == 4 && StereoCost.p2(c) == 24,
        "Revised default cost penalty units");
    c.census = 11;
    check(
        StereoCost.penalty(c.p1, c) == 20 && StereoCost.p2(c) == 120,
        "Larger descriptor uses consistent penalty scaling");
    check(
        MemoryLauncher.requestedBudget(new String[] {"--memory-mb=1024"}) == 1024,
        "Inline heap budget parsed");
    check(
        MemoryLauncher.requestedBudget(new String[] {"--memory-mb", "2048", "-h"}) == 0,
        "Help does not restart a larger JVM");
    check(MemoryLauncher.heapMiB(1024) * .65 > 1024, "Heap includes budget headroom");
    // Hundreds of hypotheses: several cost/aggregate banks and continuous strip carries.
    if (hardware) {
      int ww = 800, hh = 64, dd = 693;
      byte[] first = new byte[ww * hh], second = new byte[first.length];
      new Random(93).nextBytes(first);
      for (int y = 0; y < hh; y++)
        for (int x = 0; x < ww; x++) second[y * ww + x] = first[y * ww + Math.min(ww - 1, x + 130)];
      Options large = TestSupport.options();
      large.output = Files.createDirectories(root.resolve("wide-range"));
      large.disparities = dd;
      large.gpuDevice = gpuIndex;
      large.median = 1;
      large.speckleSize = 0;
      var expected = new CensusSgm(large).compute(first, second, ww, hh);
      for (String mode : List.of("resident", "streamed")) {
        Options go = large.copy();
        go.compute = "gpu";
        go.gpuSgm = mode;
        go.gpuBankMb = mode.equals("resident") ? 32 : 1;
        go.gpuStripRows = 7;
        var actual = new CensusSgm(go).compute(first, second, ww, hh);
        check(
            Arrays.equals(expected.disparity, actual.disparity),
            "693 hypotheses: exact CPU/GPU " + mode);
        check(
            expected.stats.csv().equals(actual.stats.csv()),
            "693 hypotheses: same confidence/rejection counts " + mode);
      }
    }
    return checks;
  }
}
