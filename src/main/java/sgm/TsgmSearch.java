package sgm;

import java.io.*;
import java.util.*;

/**
 * Coarse-to-fine band planning for {@code --pairwise-search tsgm}: a 2x box pyramid, full-range
 * banded SGM at the coarsest level, then per-pixel bands from LR-checked parent disparities with a
 * margin, guarded by sparse range samples. See P1_TSGM.md.
 */
final class TsgmSearch {
  /**
   * Bands for the full-resolution solve, plus a JSON-ready report. Right bands come from the right
   * image's own checked parents; left winners are added after the left solve.
   */
  record Plan(
      DisparityBands left, DisparityBands right, List<Object> levels, long fullRangePixels, long seededPixels) {}

  private TsgmSearch() {}

  static Plan plan(
      Options o,
      byte[] left,
      byte[] right,
      byte[] maskLeft,
      byte[] maskRight,
      boolean[] texturedLeft,
      boolean[] texturedRight,
      int w,
      int h) {
    int D = o.disparities, md = o.minDisparity;
    List<Object> reports = new ArrayList<>();
    if (o.tsgmLevels == 0) {
      DisparityBands bands = DisparityBands.full(w, h, D, texturedLeft);
      reports.add(Json.object("level", 0, "width", w, "height", h, "bands", bands.histogram(), "mode", "full_bands"));
      return new Plan(bands, DisparityBands.full(w, h, D, texturedRight), reports, 0, 0);
    }
    int levels = 0;
    while (levels < o.tsgmLevels) {
      int f = 1 << (levels + 1);
      int lw = w / f, lh = h / f;
      int lmin = Math.floorDiv(md, f), lmax = Math.ceilDiv(md + D - 1, f);
      if (Math.min(lw, lh) < 48 || lmax - lmin + 1 < 8 || Math.min(lw, lh) < o.census + 2) break;
      levels++;
    }
    // Level images, from full resolution down.
    byte[][] grayA = new byte[levels + 1][], grayB = new byte[levels + 1][];
    byte[][] validA = new byte[levels + 1][], validB = new byte[levels + 1][];
    int[] widths = new int[levels + 1], heights = new int[levels + 1];
    grayA[0] = left;
    grayB[0] = right;
    validA[0] = maskLeft == null ? ones(w * h) : maskLeft;
    validB[0] = maskRight == null ? ones(w * h) : maskRight;
    widths[0] = w;
    heights[0] = h;
    for (int l = 1; l <= levels; l++) {
      int pw = widths[l - 1], ph = heights[l - 1], lw = pw / 2, lh = ph / 2;
      widths[l] = lw;
      heights[l] = lh;
      grayA[l] = new byte[lw * lh];
      grayB[l] = new byte[lw * lh];
      validA[l] = new byte[lw * lh];
      validB[l] = new byte[lw * lh];
      downsample(grayA[l - 1], validA[l - 1], pw, lw, lh, grayA[l], validA[l]);
      downsample(grayB[l - 1], validB[l - 1], pw, lw, lh, grayB[l], validB[l]);
    }
    float[] parentLeft = null, parentRight = null;
    long fullRange = 0, seeded = 0;
    DisparityBands bands = null, rightBands = null;
    int r = o.census / 2;
    for (int l = levels; l >= 0; l--) {
      long started = System.nanoTime();
      int f = 1 << l, lw = widths[l], lh = heights[l];
      int lmin = Math.floorDiv(md, f), lmax = Math.ceilDiv(md + D - 1, f), labels = l == 0 ? D : lmax - lmin + 1;
      if (l == 0) lmin = md;
      boolean[] ta, tb;
      if (l == 0) {
        ta = texturedLeft;
        tb = texturedRight;
      } else {
        ta = CensusSgm.textureMask(grayA[l], lw, lh, o.census);
        tb = CensusSgm.textureMask(grayB[l], lw, lh, o.census);
        for (int i = 0; i < ta.length; i++) {
          ta[i] &= validA[l][i] != 0;
          tb[i] &= validB[l][i] != 0;
        }
      }
      long levelFull = 0, levelSeeded;
      if (parentLeft == null) {
        bands = DisparityBands.full(lw, lh, labels, ta);
        rightBands = DisparityBands.full(lw, lh, labels, tb);
      } else {
        bands = new DisparityBands(lw, lh, labels);
        rightBands = new DisparityBands(lw, lh, labels);
        levelFull = derive(o, parentLeft, widths[l + 1], heights[l + 1], bands, ta, lmin, 1, false);
        levelFull += derive(o, parentRight, widths[l + 1], heights[l + 1], rightBands, tb, lmin, 1, true);
      }
      levelSeeded = seed(o, bands, ta, f, lmin, false) + seed(o, rightBands, tb, f, lmin, true);
      long unmatchable = bands.clampToImage(lmin, false, r) + rightBands.clampToImage(lmin, true, r);
      fullRange += levelFull;
      seeded += levelSeeded;
      Map<String, Object> report = new LinkedHashMap<>();
      report.put("level", l);
      report.put("width", lw);
      report.put("height", lh);
      report.put("min_disparity", lmin);
      report.put("labels", labels);
      report.put("full_range_fallback_pixels", levelFull);
      report.put("unmatchable_pixels", unmatchable);
      report.put("seed_widened_pixels", levelSeeded);
      report.put("bands", bands.histogram());
      report.put("right_bands", rightBands.histogram());
      if (l > 0) {
        Options level = o.copy();
        level.minDisparity = lmin;
        level.disparities = labels;
        level.matchingCost = "census";
        level.censusMode = "gray";
        level.stereoSelection = null;
        long[][] da = CensusSgm.census(grayA[l], lw, lh, o.census),
            db = CensusSgm.census(grayB[l], lw, lh, o.census);
        BandedSgm.Outcome forward, reverse;
        try (BandedSgm.Pair pair = new BandedSgm.Pair(da, db, grayA[l], grayB[l], ta, tb, lw, lh)) {
          forward = BandedSgm.match(level, pair, false, bands, false, new CensusSgm.Stats(), null, null);
          includeWinners(rightBands, forward.disparity(), lmin, tb);
          reverse = BandedSgm.match(level, pair, true, rightBands, false, new CensusSgm.Stats(), null, null);
        }
        parentLeft = leftRight(forward.disparity(), reverse.disparity(), lw, lh, r, false);
        parentRight = leftRight(reverse.disparity(), forward.disparity(), lw, lh, r, true);
        long valid = 0;
        for (float v : parentLeft) if (Float.isFinite(v)) valid++;
        report.put("backend", forward.backend());
        report.put("lr_valid_pixels", valid);
      }
      report.put("seconds", (System.nanoTime() - started) / 1e9);
      reports.add(report);
    }
    return new Plan(bands, rightBands, reports, fullRange, seeded);
  }

  /**
   * Adds every left winner d at x to the band of right pixel x-d, so the right search can always
   * represent the left answer and LR checks are not limited by the right band.
   */
  static void includeWinners(DisparityBands right, float[] left, int minDisparity, boolean[] validRight) {
    int w = right.width;
    StereoRows.rows(0, right.height, y -> {
      for (int x = 0; x < w; x++) {
        float d = left[y * w + x];
        if (!Float.isFinite(d)) continue;
        int xr = Math.round(x - d);
        if (xr < 0 || xr >= w || !validRight[y * w + xr]) continue;
        int q = y * w + xr;
        right.include(q, (int) Math.floor(d) - minDisparity);
        right.include(q, (int) Math.ceil(d) - minDisparity);
      }
    });
    right.changed();
  }

  private static byte[] ones(int n) {
    byte[] out = new byte[n];
    Arrays.fill(out, (byte) 1);
    return out;
  }

  /** 2x2 box mean; valid only where all four children are valid. */
  static void downsample(byte[] gray, byte[] valid, int pw, int lw, int lh, byte[] outGray, byte[] outValid) {
    for (int y = 0; y < lh; y++)
      for (int x = 0; x < lw; x++) {
        int p = 2 * y * pw + 2 * x;
        int sum = (gray[p] & 255) + (gray[p + 1] & 255) + (gray[p + pw] & 255) + (gray[p + pw + 1] & 255);
        outGray[y * lw + x] = (byte) ((sum + 2) / 4);
        outValid[y * lw + x] =
            (byte) (valid[p] != 0 && valid[p + 1] != 0 && valid[p + pw] != 0 && valid[p + pw + 1] != 0 ? 1 : 0);
      }
  }

  /**
   * Disparities of {@code first} that pass a 1-pixel check against {@code second}; others NaN.
   * Forward: first is left (match at x-d); reverse: first is right (match at x+d).
   */
  static float[] leftRight(float[] first, float[] second, int w, int h, int r, boolean reverse) {
    float[] out = new float[first.length];
    Arrays.fill(out, Float.NaN);
    StereoRows.rows(r, h - r, y -> {
      for (int x = r; x < w - r; x++) {
        int p = y * w + x;
        if (!Float.isFinite(first[p])) continue;
        int other = Math.round(reverse ? x + first[p] : x - first[p]);
        if (other < r || other >= w - r) continue;
        float value = second[y * w + other];
        if (Float.isFinite(value) && Math.abs(first[p] - value) <= 1) out[p] = first[p];
      }
    });
    return out;
  }

  /**
   * Bands from the parent level's checked disparities: min/max over a (2*radius+1)^2 parent window,
   * doubled, plus margin (doubled at depth edges); 9x9 fallback window, then the full range.
   * Returns the number of full-range fallback pixels.
   */
  static long derive(
      Options o,
      float[] parent,
      int pw,
      int ph,
      DisparityBands bands,
      boolean[] valid,
      int lmin,
      int radius,
      boolean reverse) {
    int w = bands.width, h = bands.height, D = bands.labels, k = o.tsgmMargin;
    // Windowed min/max of valid parent disparities for each fallback radius, computed once.
    // A 129x129 parent window comes before the full range: with ranges of 1000+ disparities, full
    // range fallbacks dominate the packed volume.
    float[][] near = extremes(parent, pw, ph, radius),
        wider = extremes(parent, pw, ph, 4),
        wide = extremes(parent, pw, ph, 16),
        widest = extremes(parent, pw, ph, 64);
    long[] full = new long[h];
    StereoRows.rows(0, h, y -> {
      for (int x = 0; x < w; x++) {
        int p = y * w + x;
        if (!valid[p]) continue;
        // Pixels whose every target falls outside the other image have nothing to search.
        int r = o.census / 2;
        if (reverse
            ? (long) w - r - 1 - x - lmin < 0 || (long) r - x - lmin > D - 1
            : (long) x - lmin - r < 0 || (long) x - lmin - (w - r - 1) > D - 1) continue;
        int c = Math.min(ph - 1, y / 2) * pw + Math.min(pw - 1, x / 2);
        float[][] source =
            !Float.isNaN(near[0][c])
                ? near
                : !Float.isNaN(wider[0][c]) ? wider : !Float.isNaN(wide[0][c]) ? wide : widest;
        if (Float.isNaN(source[0][c])) {
          bands.low[p] = 0;
          bands.count[p] = D;
          full[y]++;
          continue;
        }
        float minimum = source[0][c], maximum = source[1][c];
        int margin = (maximum - minimum > 2 ? 2 * k : k) + 1;
        int lo = (int) Math.floor(2 * minimum) - margin - lmin,
            hi = (int) Math.ceil(2 * maximum) + margin - lmin;
        lo = Math.clamp(lo, 0, D - 1);
        hi = Math.clamp(hi, lo, D - 1);
        bands.low[p] = lo;
        bands.count[p] = hi - lo + 1;
      }
    });
    long total = 0;
    for (long f : full) total += f;
    return total;
  }

  /**
   * Separable (2*radius+1)^2 minimum and maximum of finite values; NaN where a window has none.
   * Equivalent to scanning each square window, in O(n * radius).
   */
  /**
   * Min/max of the finite values in each (2r+1)^2 window, clipped at the borders (NaN where the
   * window has none). Separable van Herk/Gil-Werman passes: about 3 comparisons per pixel per pass
   * whatever the radius (the 129-wide fallback windows dominated band planning before), with
   * exactly the same results as scanning each window.
   */
  static float[][] extremes(float[] parent, int pw, int ph, int radius) {
    float[] rowMin = new float[parent.length], rowMax = new float[parent.length];
    StereoRows.rows(0, ph, y -> {
      float[] g = new float[pw + 2 * radius], h = new float[pw + 2 * radius];
      sliding(parent, y * pw, 1, pw, radius, true, rowMin, g, h);
      sliding(parent, y * pw, 1, pw, radius, false, rowMax, g, h);
    });
    float[] min = new float[parent.length], max = new float[parent.length];
    StereoRows.rows(0, pw, x -> {
      float[] g = new float[ph + 2 * radius], h = new float[ph + 2 * radius];
      sliding(rowMin, x, pw, ph, radius, true, min, g, h);
      sliding(rowMax, x, pw, ph, radius, false, max, g, h);
    });
    for (int p = 0; p < min.length; p++)
      if (!(min[p] <= max[p])) {
        min[p] = Float.NaN;
        max[p] = Float.NaN;
      }
    return new float[][] {min, max};
  }

  /**
   * One 1D pass over n values at in[from + i * stride]: out = min (or max) of the finite values
   * within +/- radius (+/-infinity where none). Blocks of length 2r+1 hold prefix (g) and suffix
   * (h) extremes of the padded line; a window starting at padded index i spans h[i] and g[i+2r].
   */
  static void sliding(
      float[] in, int from, int stride, int n, int radius, boolean minimum, float[] out, float[] g, float[] h) {
    int k = 2 * radius + 1, length = n + 2 * radius;
    float empty = minimum ? Float.POSITIVE_INFINITY : Float.NEGATIVE_INFINITY;
    for (int i = 0; i < length; i++) {
      int j = i - radius;
      float v = j >= 0 && j < n ? in[from + j * stride] : empty;
      if (!Float.isFinite(v)) v = empty;
      g[i] = i % k == 0 ? v : minimum ? Math.min(g[i - 1], v) : Math.max(g[i - 1], v);
    }
    for (int i = length - 1; i >= 0; i--) {
      int j = i - radius;
      float v = j >= 0 && j < n ? in[from + j * stride] : empty;
      if (!Float.isFinite(v)) v = empty;
      h[i] = i == length - 1 || (i + 1) % k == 0 ? v : minimum ? Math.min(h[i + 1], v) : Math.max(h[i + 1], v);
    }
    for (int x = 0; x < n; x++) {
      float a = h[x], b = g[x + 2 * radius];
      out[from + x * stride] = minimum ? Math.min(a, b) : Math.max(a, b);
    }
  }

  /**
   * Widens bands near sparse range samples to include sample +/- (margin+1). Samples are in left
   * rectified coordinates; right bands use the corresponding right position x - d.
   */
  static long seed(
      Options o, DisparityBands bands, boolean[] valid, int factor, int lmin, boolean reverse) {
    double[] seeds = o.tsgmSeeds;
    if (seeds == null) return 0;
    int w = bands.width, h = bands.height, D = bands.labels, m = o.tsgmMargin + 1, radius = 4;
    double lowLimit = o.minDisparity, highLimit = o.minDisparity + o.disparities - 1;
    long widened = 0;
    for (int s = 0; s + 2 < seeds.length; s += 3) {
      double d = seeds[s + 2];
      if (!(d >= lowLimit && d <= highLimit)) continue;
      int cx = (int) Math.round((reverse ? seeds[s] - d : seeds[s]) / factor),
          cy = (int) Math.round(seeds[s + 1] / factor);
      int lo = (int) Math.floor(d / factor) - m - lmin, hi = (int) Math.ceil(d / factor) + m - lmin;
      lo = Math.clamp(lo, 0, D - 1);
      hi = Math.clamp(hi, 0, D - 1);
      for (int y = Math.max(0, cy - radius); y <= Math.min(h - 1, cy + radius); y++)
        for (int x = Math.max(0, cx - radius); x <= Math.min(w - 1, cx + radius); x++) {
          int p = y * w + x;
          if (!valid[p]) continue;
          int before = bands.count[p], beforeLow = bands.low[p];
          bands.include(p, lo);
          bands.include(p, hi);
          if (bands.count[p] != before || bands.low[p] != beforeLow) widened++;
        }
    }
    return widened;
  }

  static void write(Options o, String side, Plan plan, BandedSgm.Outcome outcome, double seconds) {
    if (o.stereoReportDirectory == null) return;
    try {
      java.nio.file.Files.createDirectories(o.stereoReportDirectory);
      Json.write(
          o.stereoReportDirectory.resolve("tsgm_" + side + ".json"),
          Json.object(
              "pairwise_search",
              "tsgm",
              "levels",
              plan == null ? List.of() : plan.levels(),
              "full_range_fallback_pixels",
              plan == null ? 0 : plan.fullRangePixels(),
              "seed_widened_pixels",
              plan == null ? 0 : plan.seededPixels(),
              "final_bands",
              outcome.bands().histogram(),
              "band_edge_retry_pixels",
              outcome.retried(),
              "band_edge_rejected_pixels",
              outcome.edgeRejected(),
              "backend",
              outcome.backend(),
              "final_solve_seconds",
              seconds));
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }
}
