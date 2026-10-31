package sgm;

import java.nio.file.*;
import java.util.*;

final class EsgmProductionRegression {
  static int run(Path root) throws Exception {
    int checks = 0;
    boolean hardware = false;
    try {
      hardware = !GpuSgm.devices().isEmpty();
    } catch (RuntimeException absent) {
    }
    if (!hardware) return 0;
    for (int fixture = 0; fixture < 3; fixture++)
      for (int dirs : new int[] {4, 8}) {
        int w = 160, h = 47, n = w * h;
        byte[] a = new byte[n], b = new byte[n], ma = new byte[n], mb = new byte[n];
        new Random(174).nextBytes(a);
        Arrays.fill(ma, (byte) 255);
        Arrays.fill(mb, (byte) 255);
        if (fixture == 1)
          for (int y = 0; y < h; y++)
            for (int x = 0; x < w; x++) a[y * w + x] = (byte) ((x % 12) * 19 + y % 3 * 7);
        int shift = fixture == 2 ? -11 : 11;
        for (int y = 0; y < h; y++)
          for (int x = 0; x < w; x++)
            b[y * w + x] = a[y * w + Math.max(0, Math.min(w - 1, x + shift))];
        for (int y = 8; y < 13; y++) for (int x = 29; x < 36; x++) ma[y * w + x] = 0;
        Options o = TestSupport.options();
        o.output = root;
        o.compute = "gpu";
        o.gpuDevice = Integer.getInteger("sgm.test.gpu", 0);
        o.minDisparity = -15;
        o.disparities = 129;
        o.memoryMb = 2;
        o.median = 1;
        o.speckleSize = 0;
        o.gpuSgm = "resident";
        o.directions = dirs;
        var expected = new CensusSgm(o).compute(a, b, null, null, w, h, ma, mb);
        for (int pool : new int[] {1, 2, 4}) {
          Options test = o.copy();
          test.gpuSgm = "esgm";
          test.esgmCandidates = pool;
          test.gpuStripRows = 7;
          var result = new CensusSgm(test).compute(a, b, null, null, w, h, ma, mb);
          if (!Arrays.equals(expected.disparity, result.disparity))
            throw new AssertionError(
                "Compact plus sparse exact verification differs: "
                    + fixture
                    + "/"
                    + dirs
                    + "/"
                    + pool);
          checks++;
          if (!expected.stats.csv().equals(result.stats.csv()))
            throw new AssertionError(
                "Compact filtering counters differ: "
                    + expected.stats.csv()
                    + " vs "
                    + result.stats.csv());
          checks++;
        }
      }
    return checks;
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Path root = Files.createDirectories(Path.of("target/scaling/esgm"));
    System.out.println("PASS production compact checks=" + run(root));
    GpuSgm.closeSession();
  }
}
