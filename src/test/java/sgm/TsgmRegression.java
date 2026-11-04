package sgm;

import java.util.*;

/**
 * P1 controls. (1) --tsgm-levels 0 (banded engine, full bands) reproduces ordinary SGM bit for
 * bit; (2) GPU banded summaries equal the CPU reference on random bands; (3) tsgm agrees with full
 * search on a synthetic slanted scene with a depth step. Pass "gpu" to run the GPU cases.
 */
public final class TsgmRegression {
  static long checks;

  static void check(boolean condition, String message) {
    checks++;
    if (!condition) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    boolean gpu = args.length > 0 && args[0].equals("gpu");
    String[] costs = {"census", "ad-census", "census-gradient", "zncc"};
    for (String compute : gpu ? new String[] {"cpu", "gpu"} : new String[] {"cpu"})
      for (String cost : costs)
        for (int directions : new int[] {4, 8})
          for (String aggregate : new String[] {"16", "32"})
            fullBands(compute, cost, directions, aggregate, 7);
    fullBands(gpu ? "gpu" : "cpu", "census", 8, "16", 23);
    if (gpu) randomBands();
    accuracy(gpu ? "gpu" : "cpu");
    strips(gpu ? "gpu" : "cpu");
    extremes();
    System.out.println("PASS tsgm controls: " + checks);
    GpuSgm.closeSession();
  }

  static Options options(String compute, String cost, int directions, String aggregate) {
    Options o = new Options();
    o.compute = compute;
    o.gpuDevice = Integer.getInteger("sgm.test.gpu", 0);
    o.matchingCost = cost;
    o.directions = directions;
    o.sgmAggregate = aggregate;
    o.census = 5;
    o.cores = 2;
    o.memoryMb = 512;
    o.gpuMemoryMb = 256;
    o.minDisparity = 2;
    o.disparities = 40;
    o.median = 3;
    o.speckleSize = 20;
    o.output = java.nio.file.Path.of("target/tsgm-controls");
    return o;
  }

  /** Random texture; right image built from a slanted plane with a raised block (occlusion step). */
  static byte[][] scene(int w, int h, long seed, float[] truth) {
    Random random = new Random(seed);
    byte[] texture = new byte[(w + 96) * h];
    for (int i = 0; i < texture.length; i++) texture[i] = (byte) random.nextInt(256);
    byte[] left = new byte[w * h], right = new byte[w * h];
    // Smooth slightly so subpixel content exists, keep strong Census texture.
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) {
        int s = 0, c = 0;
        for (int dx = 0; dx <= 1; dx++) {
          s += texture[y * (w + 96) + x + 48 + dx] & 255;
          c++;
        }
        left[y * w + x] = (byte) (s / c);
      }
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) {
        float d = 10 + 6f * x / w + 4f * y / h;
        if (x > w / 3 && x < w / 2 && y > h / 4 && y < 3 * h / 4) d += 9;
        truth[y * w + x] = d;
      }
    for (int y = 0; y < h; y++)
      for (int xr = 0; xr < w; xr++) {
        // Right pixel xr shows the left pixel x with x - d(x) = xr; take the nearest (largest d).
        int best = -1;
        float bestD = -1;
        for (int x = xr; x < Math.min(w, xr + 40); x++) {
          float d = truth[y * w + x];
          if (Math.abs(x - d - xr) < .5f && d > bestD) {
            bestD = d;
            best = x;
          }
        }
        right[y * w + xr] = best >= 0 ? left[y * w + best] : (byte) random.nextInt(256);
      }
    return new byte[][] {left, right};
  }

  static void fullBands(String compute, String cost, int directions, String aggregate, int seed)
      throws Exception {
    int w = 120, h = 72;
    float[] truth = new float[w * h];
    byte[][] images = scene(w, h, seed, truth);
    Options full = options(compute, cost, directions, aggregate);
    Options banded = full.copy();
    banded.pairwiseSearch = "tsgm";
    banded.tsgmLevels = 0;
    CensusSgm.Result a = new CensusSgm(full).compute(images[0], images[1], w, h);
    CensusSgm.Result b = new CensusSgm(banded).compute(images[0], images[1], w, h);
    String label = compute + "/" + cost + "/" + directions + "/" + aggregate + "/" + seed;
    check(Arrays.equals(bits(a.disparity), bits(b.disparity)), label + ": disparity identical");
    check(a.stats.csv().equals(b.stats.csv()), label + ": stats identical " + a.stats.csv() + " vs " + b.stats.csv());
    check(Arrays.equals(bits(a.costMargin), bits(b.costMargin)), label + ": margin identical");
    check(Arrays.equals(bits(a.costSharpness), bits(b.costSharpness)), label + ": sharpness identical");
    check(Arrays.equals(bits(a.correlation), bits(b.correlation)), label + ": correlation identical");
    check(a.stats.valid > w * h / 3, label + ": dense result " + a.stats.valid);
  }

  static int[] bits(float[] values) {
    if (values == null) return null;
    int[] out = new int[values.length];
    for (int i = 0; i < values.length; i++) out[i] = Float.floatToRawIntBits(values[i]);
    return out;
  }

  static void randomBands() {
    int w = 101, h = 67;
    float[] truth = new float[w * h];
    byte[][] images = scene(w, h, 3, truth);
    for (String cost : new String[] {"census", "ad-census", "census-gradient", "zncc"})
      for (int directions : new int[] {4, 8})
        for (boolean reverse : new boolean[] {false, true}) {
          Options o = options("gpu", cost, directions, "16");
          long[][] a = CensusSgm.census(images[0], w, h, o.census), b = CensusSgm.census(images[1], w, h, o.census);
          boolean[] ta = CensusSgm.textureMask(images[0], w, h, o.census),
              tb = CensusSgm.textureMask(images[1], w, h, o.census);
          Random random = new Random(directions * 31 + cost.length() + (reverse ? 7 : 0));
          DisparityBands bands = new DisparityBands(w, h, o.disparities);
          boolean[] valid = reverse ? tb : ta;
          for (int p = 0; p < w * h; p++) {
            if (!valid[p] || random.nextInt(10) == 0) continue;
            bands.low[p] = random.nextInt(o.disparities);
            bands.count[p] = 1 + random.nextInt(o.disparities - bands.low[p]);
          }
          bands.changed();
          long[][] da = reverse ? b : a, db = reverse ? a : b;
          byte[] ga = reverse ? images[1] : images[0], gb = reverse ? images[0] : images[1];
          boolean[] ma = reverse ? tb : ta, mb = reverse ? ta : tb;
          int[] cpu = BandedSgm.cpuSummary(o, da, db, ga, gb, ma, mb, w, h, reverse, bands);
          int[] gpuSummary = GpuBandedStereo.summary(o, da, db, ga, gb, ma, mb, w, h, reverse, bands);
          // Resident session: forward then reverse then forward again from one upload.
          try (GpuBandedStereo.Session s = new GpuBandedStereo.Session(o, a, b, images[0], images[1], ta, tb, w, h)) {
            int[] again = s.summary(o, reverse, bands);
            check(Arrays.equals(again, cpu), cost + "/" + directions + "/" + reverse + ": session summary");
            s.summary(o, !reverse, DisparityBands.full(w, h, o.disparities, reverse ? ta : tb));
            check(Arrays.equals(s.summary(o, reverse, bands), cpu), "session reuse after other direction");
          }
          int different = 0;
          for (int i = 0; i < cpu.length; i++) if (cpu[i] != gpuSummary[i]) different++;
          check(different == 0, cost + "/" + directions + "/" + reverse + ": GPU/CPU banded summaries differ in " + different);
        }
  }

  /** van Herk window extremes equal a brute-force scan (NaN gaps, borders, radius > size). */
  static void extremes() {
    Random random = new Random(5);
    for (int[] size : new int[][] {{1, 1}, {7, 3}, {37, 23}, {130, 90}})
      for (int radius : new int[] {1, 2, 4, 16, 64}) {
        int w = size[0], h = size[1];
        float[] v = new float[w * h];
        for (int i = 0; i < v.length; i++) v[i] = random.nextInt(5) == 0 ? Float.NaN : random.nextFloat() * 100 - 20;
        float[][] fast = TsgmSearch.extremes(v, w, h, radius);
        boolean same = true;
        for (int y = 0; y < h && same; y++)
          for (int x = 0; x < w; x++) {
            float lo = Float.POSITIVE_INFINITY, hi = Float.NEGATIVE_INFINITY;
            for (int yy = Math.max(0, y - radius); yy <= Math.min(h - 1, y + radius); yy++)
              for (int xx = Math.max(0, x - radius); xx <= Math.min(w - 1, x + radius); xx++) {
                float q = v[yy * w + xx];
                if (!Float.isFinite(q)) continue;
                lo = Math.min(lo, q);
                hi = Math.max(hi, q);
              }
            float eMin = lo <= hi ? lo : Float.NaN, eMax = lo <= hi ? hi : Float.NaN;
            if (Float.floatToRawIntBits(eMin) != Float.floatToRawIntBits(fast[0][y * w + x])
                || Float.floatToRawIntBits(eMax) != Float.floatToRawIntBits(fast[1][y * w + x])) same = false;
          }
        check(same, "window extremes exact for " + w + "x" + h + " radius " + radius);
      }
  }

  /** Oversized volumes (D1): forced strips agree with the whole solve; a high limit is exact. */
  static void strips(String compute) throws Exception {
    int w = 320, h = 260;
    float[] truth = new float[w * h];
    byte[][] images = scene(w, h, 17, truth);
    Options o = options(compute, "census", 8, "16");
    o.pairwiseSearch = "tsgm";
    o.tsgmLevels = 2;
    CensusSgm.Result whole = new CensusSgm(o).compute(images[0], images[1], w, h);
    String old = System.getProperty("sgm.test.stripCells");
    java.io.PrintStream out = System.out;
    java.io.ByteArrayOutputStream log = new java.io.ByteArrayOutputStream();
    try {
      System.setProperty("sgm.test.stripCells", Long.toString(Long.MAX_VALUE / 4));
      CensusSgm.Result same = new CensusSgm(o).compute(images[0], images[1], w, h);
      check(Arrays.equals(bits(whole.disparity), bits(same.disparity)), "strip limit above the volume is exact");
      System.setProperty("sgm.test.stripCells", "400000");
      System.setOut(new java.io.PrintStream(log, true));
      CensusSgm.Result split = new CensusSgm(o).compute(images[0], images[1], w, h);
      System.setOut(out);
      check(log.toString().contains("overlapping strips"), "strip path was exercised");
      long both = 0, agree = 0, validWhole = 0, validSplit = 0;
      for (int i = 0; i < w * h; i++) {
        boolean a = Float.isFinite(whole.disparity[i]), b = Float.isFinite(split.disparity[i]);
        if (a) validWhole++;
        if (b) validSplit++;
        if (a && b) {
          both++;
          if (Math.abs(whole.disparity[i] - split.disparity[i]) <= 1) agree++;
        }
      }
      System.out.printf(Locale.ROOT, "tsgm strips (%s): agreement %.3f%% of %d, density %d vs %d%n", compute, 100. * agree / Math.max(1, both), both, validSplit, validWhole);
      check(agree >= .99 * both, "strips agree with the whole solve on 99% of shared pixels");
      check(validSplit >= .98 * validWhole, "strips keep 98% of the whole solve's density");
    } finally {
      System.setOut(out);
      if (old == null) System.clearProperty("sgm.test.stripCells");
      else System.setProperty("sgm.test.stripCells", old);
    }
  }

  static void accuracy(String compute) throws Exception {
    int w = 320, h = 200;
    float[] truth = new float[w * h];
    byte[][] images = scene(w, h, 11, truth);
    Options full = options(compute, "census", 8, "16");
    Options tsgm = full.copy();
    tsgm.pairwiseSearch = "tsgm";
    tsgm.tsgmLevels = 2;
    CensusSgm.Result a = new CensusSgm(full).compute(images[0], images[1], w, h);
    CensusSgm.Result b = new CensusSgm(tsgm).compute(images[0], images[1], w, h);
    long both = 0, agree = 0, fullValid = 0, tsgmValid = 0, fullTruth = 0, tsgmTruth = 0;
    for (int i = 0; i < w * h; i++) {
      boolean fa = Float.isFinite(a.disparity[i]), fb = Float.isFinite(b.disparity[i]);
      if (fa) {
        fullValid++;
        if (Math.abs(a.disparity[i] - truth[i]) <= 1) fullTruth++;
      }
      if (fb) {
        tsgmValid++;
        if (Math.abs(b.disparity[i] - truth[i]) <= 1) tsgmTruth++;
      }
      if (fa && fb) {
        both++;
        if (Math.abs(a.disparity[i] - b.disparity[i]) <= 1) agree++;
      }
    }
    System.out.printf(
        Locale.ROOT,
        "tsgm accuracy (%s): full valid=%d (%.2f%% within 1px of truth), tsgm valid=%d (%.2f%%),"
            + " agreement %.3f%% of %d%n",
        compute,
        fullValid,
        100. * fullTruth / Math.max(1, fullValid),
        tsgmValid,
        100. * tsgmTruth / Math.max(1, tsgmValid),
        100. * agree / Math.max(1, both),
        both);
    check(agree >= .99 * both, "tsgm agrees with full search on 99% of shared pixels");
    check(tsgmValid >= .95 * fullValid, "tsgm keeps at least 95% of full-search density");
    check(tsgmTruth >= .98 * tsgmValid, "tsgm disparities are within 1px of truth");
  }
}
