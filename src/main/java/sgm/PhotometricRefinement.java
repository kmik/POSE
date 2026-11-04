package sgm;

/** Local subpixel photometric optimization; no hole filling or smoothing across depth edges. */
final class PhotometricRefinement {
  static long refine(
      float[] disparity, byte[] a, byte[] b, int w, int h, boolean reverse, Options o) {
    float[] original = disparity.clone();
    long[] changed = new long[h];
    int r = o.correlationWindow / 2;
    StereoRows.rows(r, h - r, y -> {

      for (int x = r; x < w - r; x++) {
        int p = y * w + x;
        double d = original[p];
        if (!Double.isFinite(d)) continue;
        boolean edge = false;
        for (int dy = -1; dy <= 1; dy++)
          for (int dx = -1; dx <= 1; dx++) {
            float q = original[p + dy * w + dx];
            if (Float.isFinite(q) && Math.abs(q - d) > 1.5) edge = true;
          }
        if (edge) continue;
        double initial = CensusSgm.zncc(a, b, w, h, x, y, reverse ? -d : d, r, 1),
            best = initial,
            chosen = d;
        int slot = -1;
        double[] scores = new double[7];
        for (int k = 0; k < 7; k++) {
          double candidate = d + (k - 3) * .25;
          if (candidate <= o.minDisparity || candidate >= o.minDisparity + o.disparities - 1) {
            scores[k] = -1;
            continue;
          }
          scores[k] = CensusSgm.zncc(a, b, w, h, x, y, reverse ? -candidate : candidate, r, 1);
          if (scores[k] > best) {
            best = scores[k];
            chosen = candidate;
            slot = k;
          }
        }
        if (slot > 0 && slot < 6 && scores[slot - 1] > -1 && scores[slot + 1] > -1) {
          double curvature = 2 * best - scores[slot - 1] - scores[slot + 1];
          if (curvature > 1e-8) {
            double offset = .125 * (scores[slot + 1] - scores[slot - 1]) / curvature;
            offset = Math.max(-.125, Math.min(.125, offset));
            double candidate = chosen + offset,
                score = CensusSgm.zncc(a, b, w, h, x, y, reverse ? -candidate : candidate, r, 1);
            if (score > best) {
              best = score;
              chosen = candidate;
            }
          }
        }
        if (best >= Math.max(.5, o.correlation)
            && best > initial + .002
            && Math.abs(chosen - d) <= .75) {
          disparity[p] = (float) chosen;
          changed[y]++;
        }
      }
    });
    long total = 0;
    for (long value : changed) total += value;
    return total;
  }
}
