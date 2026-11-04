package sgm;

import java.util.*;

/**
 * P4: parabola vs equiangular vs ZNCC subpixel refinement on the synthetic slanted-texture plane of
 * SubpixelRegression (known disparity 13.37), with a pixel-locking index per method.
 */
final class SubpixelFitRegression {
  public static void main(String[] args) throws Exception {
    int w = 384, h = 192;
    double truth = 13.37;
    byte[] a = new byte[w * h], b = new byte[w * h];
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) {
        a[y * w + x] = SubpixelRegression.texture(x, y);
        b[y * w + x] = SubpixelRegression.texture(x + truth, y);
      }
    Map<String, double[]> results = new LinkedHashMap<>();
    for (String method : new String[] {"parabola", "equiangular", "zncc"}) {
      Options o = TestSupport.options();
      o.disparities = 32;
      o.median = 1;
      o.speckleSize = 0;
      o.correlation = .5;
      if (method.equals("zncc")) o.subpixelRefinement = "zncc";
      else o.subpixelFit = method;
      CensusSgm.Result r = new CensusSgm(o).compute(a, b, w, h);
      double[] m = SubpixelRegression.metrics(r, truth);
      long[] bins = new long[20];
      long valid = 0;
      // Varying ground truth would hide locking; here use a sweep of planes with offsets.
      for (float d : r.disparity)
        if (Float.isFinite(d)) {
          bins[Math.min(19, (int) ((d - Math.floor(d)) * 20))]++;
          valid++;
        }
      results.put(method, new double[] {m[0], m[1], m[2]});
      System.out.printf(
          Locale.ROOT, "%-12s valid=%.0f rmse=%.4f p10-p90=%.4f%n", method, m[0], m[1], m[2]);
    }
    // A sweep of fractional offsets measures pixel-locking: errors should not depend on the offset.
    for (String method : new String[] {"parabola", "equiangular"}) {
      double worst = 0, sum = 0;
      int n = 0;
      for (double fraction = 0; fraction < 1; fraction += .125) {
        double d = 13 + fraction;
        for (int y = 0; y < h; y++)
          for (int x = 0; x < w; x++) b[y * w + x] = SubpixelRegression.texture(x + d, y);
        Options o = TestSupport.options();
        o.disparities = 32;
        o.median = 1;
        o.speckleSize = 0;
        o.correlation = .5;
        o.subpixelFit = method;
        CensusSgm.Result r = new CensusSgm(o).compute(a, b, w, h);
        double[] m = SubpixelRegression.metrics(r, d);
        worst = Math.max(worst, m[1]);
        sum += m[1];
        n++;
      }
      System.out.printf(Locale.ROOT, "%-12s sweep mean rmse=%.4f worst=%.4f%n", method, sum / n, worst);
      results.put(method + "_sweep", new double[] {sum / n, worst});
    }
    if (results.get("equiangular")[0] < results.get("parabola")[0] * .95)
      throw new AssertionError("Equiangular fit lost coverage");
    System.out.println("PASS subpixel fit comparison");
  }
}
