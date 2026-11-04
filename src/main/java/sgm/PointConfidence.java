package sgm;

/** Evidence scores are bounded ranking heuristics, not probabilities or accuracy estimates. */
final class PointConfidence {
  static final String MODEL = "matching-margin-curvature-lr-geometry-v3";
  static final double TWO_VIEW_MINIMUM = .35;

  static boolean bidirectional(Options o) {
    return Double.isFinite(o.twoViewConfidenceThreshold) && o.denseMethod.equals("sgm");
  }

  static long extraHostBytes(Options o, long pixels) {
    return bidirectional(o) ? pixels * 12L : 0;
  }

  /**
   * Right-side evidence aligned to the final left raster; aggregate evidence is from its SGM
   * winner.
   */
  static void reverse(
      CensusSgm.Result result,
      byte[] left,
      byte[] right,
      byte[] photoLeft,
      byte[] photoRight,
      int channels,
      float[] dr,
      Selection selected,
      byte[] maskLeft,
      byte[] maskRight,
      Options o) {
    int w = result.width,
        h = result.height,
        n = w * h,
        radius = Math.max(3, o.correlationWindow / 2);
    result.rightConfidence = new float[n];
    java.util.Arrays.fill(result.rightConfidence, Float.NaN);
    StereoRows.rows(radius, h - radius, y -> {

      for (int x = 0; x < w; x++) {
        int p = y * w + x;
        float d = result.disparity[p];
        if (!Float.isFinite(d)) continue;
        int rx = Math.round(x - d);
        if (rx < radius || rx + radius >= w) continue;
        int q = y * w + rx;
        float rd = dr[q];
        if (!Float.isFinite(rd) || !valid(selected.margin[q]) || !valid(selected.sharpness[q]))
          continue;
        int back = Math.round(rx + rd);
        if (back < 0 || back >= w || !Float.isFinite(result.disparity[y * w + back])) continue;
        double lr = Math.abs(rd - result.disparity[y * w + back]);
        if (lr > o.lrThreshold
            || !patch(maskRight, w, h, rx, y, radius)
            || !patch(maskLeft, w, h, (int) Math.floor(rx + rd), y, radius)
            || !patch(maskLeft, w, h, (int) Math.floor(rx + rd) + 1, y, radius)) continue;
        double photo =
            CensusSgm.zncc(
                photoRight, photoLeft, w, h, rx, y, -rd, o.correlationWindow / 2, channels);
        if (photo < o.correlation) {
          result.rightConfidence[p] = 0;
          continue;
        }
        double center = CensusSgm.zncc(right, left, w, h, rx, y, -rd, 1, 1),
            minus = CensusSgm.zncc(right, left, w, h, rx, y, -rd - 1, 1, 1),
            plus = CensusSgm.zncc(right, left, w, h, rx, y, -rd + 1, 1, 1),
            competitor =
                Math.max(
                    CensusSgm.zncc(right, left, w, h, rx, y, -rd - 2, 1, 1),
                    CensusSgm.zncc(right, left, w, h, rx, y, -rd + 2, 1, 1));
        double margin = clamp((center - competitor) / Math.max(.1, 1 - competitor)),
            sharpness = clamp((2 * center - minus - plus) / Math.max(.1, 2 - minus - plus)),
            strength =
                Math.sqrt(clamp(selected.margin[q] / .1) * clamp(selected.sharpness[q] / .1));
        result.rightConfidence[p] = (float) (score(photo, margin, sharpness, lr) * strength);
      }
    });
  }

  private static boolean patch(byte[] mask, int w, int h, int x, int y, int radius) {
    if (x - radius < 0 || x + radius >= w || y - radius < 0 || y + radius >= h) return false;
    if (mask != null)
      for (int yy = y - radius; yy <= y + radius; yy++)
        for (int xx = x - radius; xx <= x + radius; xx++) if (mask[yy * w + xx] == 0) return false;
    return true;
  }

  static double score(double photo, double margin, double sharpness, double lr) {
    if (!Double.isFinite(photo)
        || !Double.isFinite(margin)
        || !Double.isFinite(sharpness)
        || !Double.isFinite(lr)
        || lr < 0) return Double.NaN;
    return Math.sqrt(clamp(photo)) * Math.sqrt(clamp(margin) * clamp(sharpness)) / (1 + lr);
  }

  static double finalScore(double matching, double sigma) {
    if (!valid(matching) || !Double.isFinite(sigma) || sigma < 0) return Double.NaN;
    return matching * (.5 + .5 / (1 + sigma / .02));
  }

  static boolean valid(double score) {
    return Double.isFinite(score) && score >= 0 && score <= 1;
  }

  static boolean passes(double score, double threshold) {
    return valid(score) ? score >= threshold : threshold == 0;
  }

  static double clamp(double value) {
    return Math.clamp(value, 0, 1);
  }

  static final class Selection {
    final float[] margin, sharpness;

    Selection(int n) {
      margin = new float[n];
      sharpness = new float[n];
      java.util.Arrays.fill(margin, Float.NaN);
      java.util.Arrays.fill(sharpness, Float.NaN);
    }

    void put(int p, double winner, double second, double before, double after) {
      if (second != Integer.MAX_VALUE)
        margin[p] = (float) clamp((second - winner) / Math.max(1, second));
      if (before >= 0 && after >= 0)
        sharpness[p] = (float) clamp((before + after - 2 * winner) / Math.max(1, before + after));
    }
  }

  static void stereo(CensusSgm.Result result, byte[] left, byte[] right, Options options) {
    try (var rows = StereoRows.open(options.stereoPipeline ? Math.max(1,options.cores/2) : options.cores)) { stereo(result, left, right); }
  }

  static void stereo(CensusSgm.Result result, byte[] left, byte[] right) {
    int w = result.width, h = result.height, n = w * h;
    result.confidence = new float[n];
    boolean aggregate = result.costMargin != null && result.costSharpness != null;
    if (!aggregate) {
      result.costMargin = new float[n];
      result.costSharpness = new float[n];
    }
    java.util.Arrays.fill(result.confidence, Float.NaN);
    if (!aggregate) {
      java.util.Arrays.fill(result.costMargin, Float.NaN);
      java.util.Arrays.fill(result.costSharpness, Float.NaN);
    }
    StereoRows.rows(1, h - 1, y -> {

      for (int x = 1; x < w - 1; x++) {
        int p = y * w + x;
        float d = result.disparity[p];
        if (!Float.isFinite(d)) continue;
        // Independent local photometric alternatives, not the SGM path aggregate uniqueness.
        double center = CensusSgm.zncc(left, right, w, h, x, y, d, 1, 1);
        double minus = CensusSgm.zncc(left, right, w, h, x, y, d - 1, 1, 1);
        double plus = CensusSgm.zncc(left, right, w, h, x, y, d + 1, 1, 1);
        double competitor =
            Math.max(
                CensusSgm.zncc(left, right, w, h, x, y, d - 2, 1, 1),
                CensusSgm.zncc(left, right, w, h, x, y, d + 2, 1, 1));
        double margin = clamp((center - competitor) / Math.max(.1, 1 - competitor));
        double sharpness = clamp((2 * center - minus - plus) / Math.max(.1, 2 - minus - plus));
        if (!aggregate) {
          result.costMargin[p] = (float) margin;
          result.costSharpness[p] = (float) sharpness;
        }
        double photo = result.correlation == null ? center : result.correlation[p];
        double lr = result.leftRightError == null ? Double.NaN : result.leftRightError[p];
        double strength =
            aggregate
                ? Math.sqrt(clamp(result.costMargin[p] / .1) * clamp(result.costSharpness[p] / .1))
                : 1;
        result.confidence[p] = (float) (score(photo, margin, sharpness, lr) * strength);
      }
    });
  }
}
