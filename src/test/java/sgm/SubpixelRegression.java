package sgm;

import java.nio.file.*;
import java.util.*;

final class SubpixelRegression {
  public static void main(String[] args) throws Exception {
    System.out.println("PASS subpixel numerical checks=" + run());
  }

  static int run() throws Exception {
    int w = 384, h = 192;
    double d = 13.37;
    byte[] a = new byte[w * h], b = new byte[w * h];
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) {
        a[y * w + x] = texture(x, y);
        b[y * w + x] = texture(x + d, y);
      }
    Options o = TestSupport.options();
    o.disparities = 32;
    o.median = 1;
    o.speckleSize = 0;
    o.correlation = .5;
    CensusSgm.Result original = new CensusSgm(o).compute(a, b, w, h);
    o.subpixelRefinement = "zncc";
    CensusSgm.Result refined = new CensusSgm(o).compute(a, b, w, h);
    double[] old = metrics(original, d), now = metrics(refined, d);
    System.out.println(
        "Subpixel plane: before=" + Arrays.toString(old) + " after=" + Arrays.toString(now));
    if (now[0] < old[0] * .8) throw new AssertionError("Coverage collapsed");
    if (now[1] >= old[1] * .8)
      throw new AssertionError(
          "Subpixel photometric refinement did not reduce known disparity error");
    return 2;
  }

  static byte texture(double x, double y) {
    double v =
        128
            + 27 * Math.sin(.71 * x + .23 * y)
            + 23 * Math.cos(.43 * x - .69 * y)
            + 21 * Math.sin(.29 * x + .91 * y)
            + 14 * Math.cos(1.11 * x + .17 * y);
    return (byte) Math.round(Math.max(0, Math.min(255, v)));
  }

  static double[] metrics(CensusSgm.Result r, double truth) {
    List<Double> errors = new ArrayList<>();
    double sum = 0;
    for (int y = 12; y < r.height - 12; y++)
      for (int x = 45; x < r.width - 12; x++) {
        float d = r.disparity[y * r.width + x];
        if (Float.isFinite(d)) {
          double e = d - truth;
          errors.add(e);
          sum += e * e;
        }
      }
    Collections.sort(errors);
    return new double[] {
      errors.size(),
      Math.sqrt(sum / errors.size()),
      errors.get(errors.size() * 9 / 10) - errors.get(errors.size() / 10)
    };
  }
}
