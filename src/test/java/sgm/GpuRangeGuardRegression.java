package sgm;

import java.util.*;

/**
 * The disparity-range guard's small full-range solve on the GPU (DisparityRange.guardSolve with
 * gpuSmallSolves) against the CPU solve: disparities bit for bit and all statistics, on portrait
 * and landscape probes with depth steps, slopes, flat areas, noise and masks. Skipped without GPU.
 */
final class GpuRangeGuardRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) {
    nu.pattern.OpenCV.loadLocally();
    System.out.println("PASS GPU range guard: " + run());
  }

  static int run() {
    checks = 0;
    for (int seed = 0; seed < 12; seed++) {
      Random r = new Random(seed);
      int pw = seed % 2 == 0 ? 72 : 128, ph = seed % 2 == 0 ? 128 : 72;
      byte[] a = new byte[pw * ph], b = new byte[pw * ph], ma = new byte[pw * ph], mb = new byte[pw * ph];
      double shift = 3 + r.nextDouble() * 20, slope = r.nextDouble() * .1;
      double[] tex = new double[(pw + 80) * ph];
      for (int i = 0; i < tex.length; i++) tex[i] = r.nextDouble();
      for (int y = 0; y < ph; y++)
        for (int x = 0; x < pw; x++) {
          int p = y * pw + x;
          double d = shift + slope * y + (x > pw / 2 ? 6 : 0), xs = x + d;
          int xi = (int) xs;
          double f = xs - xi;
          a[p] = (byte) (255 * (seed % 5 == 0 && y > ph / 2 ? .5 : tex[y * (pw + 80) + x + 40]));
          double t0 = tex[y * (pw + 80) + Math.min(pw + 79, xi + 40)], t1 = tex[y * (pw + 80) + Math.min(pw + 79, xi + 41)];
          b[p] = (byte) Math.max(0, Math.min(255, 255 * ((1 - f) * t0 + f * t1) + r.nextGaussian() * (seed % 3) * 4));
          ma[p] = (byte) (x < 3 && seed % 4 == 1 ? 0 : 1);
          mb[p] = (byte) (y < 4 && seed % 4 == 2 ? 0 : 1);
        }
      Options o = TestSupport.options();
      o.compute = "auto";
      o.pyramidLevels = 1;
      o.pairwiseSearch = "full";
      o.tiled = false;
      o.censusMode = "gray";
      o.census = Math.min(5, o.census);
      o.memoryMb = 128;
      o.speckleSize = 8;
      o.speckleRange = 1.5;
      o.uniqueness = Math.max(.05, o.uniqueness);
      o.correlation = Math.max(.5, o.correlation);
      o.minDisparity = 0;
      o.disparities = pw;
      Options gpu = o.copy();
      gpu.gpuSmallSolves = true;
      if (!GpuSgm.use(pw, ph, gpu)) {
        System.out.println("GpuRangeGuardRegression skipped: no usable OpenCL device");
        return checks;
      }
      Options cpu = o.copy();
      cpu.compute = "cpu";
      CensusSgm.Result rc = new CensusSgm(cpu).compute(a, b, null, null, pw, ph, ma, mb);
      CensusSgm.Result rg = DisparityRange.guardSolve(o, a, b, pw, ph, ma, mb);
      int differ = 0;
      for (int i = 0; i < rc.disparity.length; i++)
        if (Float.floatToRawIntBits(rc.disparity[i]) != Float.floatToRawIntBits(rg.disparity[i])) differ++;
      check(differ == 0, differ + " guard disparities differ (seed " + seed + ")");
      check(rc.stats.csv().equals(rg.stats.csv()), "guard statistics differ (seed " + seed + ")");
      check(rc.stats.valid > 0, "guard fixture without valid pixels (seed " + seed + ")");
    }
    return checks;
  }
}
