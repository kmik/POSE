package sgm;

import java.util.*;

/**
 * TSGM planning on the GPU against the CPU code: band derivation (GpuTsgm vs
 * TsgmSearch.deriveCpu) on parent maps with holes of every fallback size, an empty parent, both
 * directions and signed minimum disparities; and small coarse banded solves (gpuSmallSolves) vs
 * BandedSgm.cpuSummary. Skipped without a usable OpenCL device.
 */
final class GpuTsgmRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) {
    nu.pattern.OpenCV.loadLocally();
    System.out.println("PASS GPU TSGM planning: " + run());
  }

  static int run() {
    checks = 0;
    Options o = TestSupport.options();
    o.compute = "auto";
    int w = 600, h = 340;
    if (!GpuTsgm.applies(o, w, h)) {
      System.out.println("GpuTsgmRegression skipped: GPU TSGM planning not applicable");
      return checks;
    }
    long fallbacks = 0;
    for (int seed = 0; seed < 8; seed++) {
      Random r = new Random(seed);
      int pw = w / 2, ph = h / 2;
      float[] parent = new float[pw * ph];
      for (int y = 0; y < ph; y++)
        for (int x = 0; x < pw; x++) {
          int p = y * pw + x;
          float d = (float) (10 + .05 * x + (x > pw / 2 ? 7 : 0) + r.nextGaussian() * .7);
          if (seed % 4 == 1 && Math.hypot(x - 80, y - 60) < 50) d = Float.NaN;      // wide hole
          if (seed % 4 == 2 && x > 150) d = Float.NaN;                                // half empty
          if (seed == 3) d = Float.NaN;                                               // all empty
          if (r.nextDouble() < .2) d = Float.NaN;
          if (seed == 5 && r.nextDouble() < .05) d = r.nextBoolean() ? -0.0f : 0.0f;
          parent[p] = d;
        }
      boolean[] valid = new boolean[w * h];
      for (int i = 0; i < valid.length; i++) valid[i] = r.nextDouble() > .05;
      int labels = 30 + 10 * seed, lmin = seed % 3 == 0 ? -5 : seed;
      for (int reverse = 0; reverse < 2; reverse++) {
        Options level = o.copy();
        level.tsgmMargin = 2 + seed % 3;
        DisparityBands gpu = new DisparityBands(w, h, labels), cpu = new DisparityBands(w, h, labels);
        long gpuFull = GpuTsgm.derive(level, parent, pw, ph, gpu, valid, lmin, reverse == 1);
        if (gpuFull < 0) {
          System.out.println("GpuTsgmRegression skipped: GPU derive not available");
          return checks;
        }
        long cpuFull = TsgmSearch.deriveCpu(level, parent, pw, ph, cpu, valid, lmin, 1, reverse == 1);
        fallbacks += cpuFull;
        check(gpuFull == cpuFull, "full-range count differs (seed " + seed + ")");
        check(Arrays.equals(gpu.low, cpu.low), "band low differs (seed " + seed + ")");
        check(Arrays.equals(gpu.count, cpu.count), "band count differs (seed " + seed + ")");
      }
    }
    check(fallbacks > 0, "full-range fallback not exercised");
    // Small coarse solves: GPU (gpuSmallSolves) vs the CPU reference, both directions.
    for (int seed = 0; seed < 4; seed++) {
      Random r = new Random(100 + seed);
      int lw = 140, lh = 90, labels = 8 + 2 * seed;
      byte[] a = new byte[lw * lh], b = new byte[lw * lh];
      for (int i = 0; i < a.length; i++) a[i] = (byte) r.nextInt(256);
      for (int y = 0; y < lh; y++)
        for (int x = 0; x < lw; x++) b[y * lw + x] = a[y * lw + Math.min(lw - 1, x + 3 + (x > 70 ? 2 : 0))];
      boolean[] ta = new boolean[a.length], tb = new boolean[a.length];
      Arrays.fill(ta, true);
      Arrays.fill(tb, true);
      Options level = o.copy();
      level.minDisparity = 0;
      level.disparities = labels;
      level.matchingCost = "census";
      level.censusMode = "gray";
      level.gpuSmallSolves = true;
      DisparityBands bands = DisparityBands.full(lw, lh, labels, ta);
      check(bands.total() < 200_000, "fixture must be a small solve");
      if (!GpuBandedStereo.use(level, lw, lh, bands)) {
        System.out.println("GpuTsgmRegression: small banded GPU solve not applicable; skipped");
        return checks;
      }
      long[][] da = CensusSgm.census(a, lw, lh, level.census), db = CensusSgm.census(b, lw, lh, level.census);
      for (int reverse = 0; reverse < 2; reverse++) {
        int[] gpu;
        String[] backend = new String[1];
        try (BandedSgm.Pair pair = new BandedSgm.Pair(da, db, a, b, ta, tb, lw, lh)) {
          gpu = pair.summary(level, reverse == 1, bands, backend);
        }
        check("gpu".equals(backend[0]), "small solve did not use the GPU: " + backend[0]);
        int[] cpu =
            reverse == 1
                ? BandedSgm.cpuSummary(level, db, da, b, a, tb, ta, lw, lh, true, bands)
                : BandedSgm.cpuSummary(level, da, db, a, b, ta, tb, lw, lh, false, bands);
        check(Arrays.equals(gpu, cpu), "small banded GPU solve differs (seed " + seed + ")");
      }
    }
    return checks;
  }
}
