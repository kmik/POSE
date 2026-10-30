package sgm;

import java.nio.file.*;
import java.util.*;

/**
 * Explicit hardware stress run, kept outside the routine suite to avoid reserving 6 GB on every
 * build.
 */
final class LargeGpuRegression {
  public static void main(String[] args) throws Exception {
    int w = 900, h = 1600, D = 693, shift = 130;
    Options o = TestSupport.options();
    o.compute = "gpu";
    o.gpuSgm = "resident";
    o.gpuMemoryMb = 7600;
    o.gpuDevice = Integer.getInteger("sgm.test.gpu", 0);
    o.disparities = D;
    o.output = Files.createTempDirectory(Path.of("target"), "large banked sgm ");
    o.median = 1;
    o.speckleSize = 0;
    byte[] a = new byte[w * h], b = new byte[a.length];
    new Random(34).nextBytes(a);
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) b[y * w + x] = a[y * w + Math.min(w - 1, x + shift)];
    Object plan = GpuStereo.report(w, h, o);
    System.out.println("Large banked plan: " + Json.string(plan));
    long start = System.nanoTime();
    CensusSgm.Result r = new CensusSgm(o).compute(a, b, w, h);
    long valid = 0, correct = 0;
    for (int y = 10; y < h - 10; y++)
      for (int x = shift + 12; x < w - 12; x++) {
        float d = r.disparity[y * w + x];
        if (Float.isFinite(d)) {
          valid++;
          if (Math.abs(d - shift) < .6) correct++;
        }
      }
    if (valid < (long) (w - shift - 24) * (h - 20) * .8 || correct < valid * .99)
      throw new AssertionError("Large-volume independent shift/coverage failure");
    System.out.printf(
        Locale.ROOT,
        "PASS resident 900x1600x693: %,d valid, %.6f correct fraction, %.3f seconds; %s%n",
        valid,
        correct / (double) valid,
        (System.nanoTime() - start) / 1e9,
        o.output);
    GpuSgm.closeSession();
  }
}
