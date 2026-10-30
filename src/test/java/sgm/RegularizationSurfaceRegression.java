package sgm;

import java.nio.file.*;
import java.util.*;

final class RegularizationSurfaceRegression {
  public static void main(String[] args) throws Exception {
    if (args.length == 2) {
      run(Integer.parseInt(args[0]), Integer.parseInt(args[1]));
      return;
    }
    int w = 160, h = 96;
    byte[] left = new byte[w * h], right = new byte[left.length];
    new Random(981).nextBytes(left);
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) right[y * w + x] = left[y * w + Math.min(w - 1, x + 8)];
    for (String version : List.of("previous", "revised")) {
      Options o = TestSupport.options();
      o.disparities = 24;
      o.minDisparity = -16;
      o.speckleSize = 0;
      if (version.equals("previous")) o = FenceRegression.previous(o);
      var r = new CensusSgm(o).compute(right, left, w, h);
      int n = 0;
      double sum = 0, square = 0;
      for (int y = 8; y < h - 8; y++)
        for (int x = 20; x < w - 20; x++)
          if (Float.isFinite(r.disparity[y * w + x])) {
            n++;
            double e = r.disparity[y * w + x] + 8;
            sum += Math.abs(e);
            square += e * e;
          }
      System.out.println(
          version
              + " signed center="
              + r.disparity[40 * w + 60]
              + " valid="
              + n
              + " MAE="
              + sum / n
              + " RMS="
              + Math.sqrt(square / n));
    }
    run();
  }

  static int run() {
    return run((int) new Options().p2, new Options().sgmEdgeScale);
  }

  static int run(int penalty, int edgeScale) {
    List<Object> rows = new ArrayList<>();
    int checks = 0;
    int w = 336, h = 128;
    for (double slope : new double[] {0, .015})
      for (double noise : new double[] {0, 2, 4}) {
        byte[] a = new byte[w * h], b = new byte[a.length];
        Random rng = new Random(62);
        double base = 8.37;
        for (int y = 0; y < h; y++)
          for (int x = 0; x < w; x++) {
            a[y * w + x] =
                (byte)
                    Math.round(
                        Math.max(
                            0,
                            Math.min(
                                255,
                                (SubpixelRegression.texture(x, y) & 255)
                                    + rng.nextGaussian() * noise)));
            b[y * w + x] =
                (byte)
                    Math.round(
                        Math.max(
                            0,
                            Math.min(
                                255,
                                (SubpixelRegression.texture((x + base) / (1 - slope), y) & 255)
                                    + rng.nextGaussian() * noise)));
          }
        double oldRms = 0, oldValid = 0;
        for (String version : List.of("previous", "revised")) {
          Options o = TestSupport.options();
          o.disparities = 32;
          o.speckleSize = 0;
          o.p2 = penalty;
          o.sgmEdgeScale = edgeScale;
          if (version.equals("previous")) o = FenceRegression.previous(o);
          var result = new CensusSgm(o).compute(a, b, w, h);
          int n = 0;
          double square = 0;
          List<Double> errors = new ArrayList<>();
          for (int y = 10; y < h - 10; y++)
            for (int x = 48; x < w - 12; x++)
              if (Float.isFinite(result.disparity[y * w + x])) {
                n++;
                double e = result.disparity[y * w + x] - (base + slope * x);
                square += e * e;
                errors.add(e);
              }
          Collections.sort(errors);
          double rms = Math.sqrt(square / n),
              spread = errors.get(errors.size() * 9 / 10) - errors.get(errors.size() / 10);
          rows.add(
              Json.object(
                  "slope",
                  slope,
                  "noise_sigma",
                  noise,
                  "version",
                  version,
                  "valid",
                  n,
                  "rms",
                  rms,
                  "p90_p10_error_spread",
                  spread));
          if (version.equals("previous")) {
            oldRms = rms;
            oldValid = n;
          } else {
            checks += 2;
            if (n < oldValid * .95 || rms > oldRms * 1.25 + .02 || rms > .4)
              throw new AssertionError("Surface coverage/error regressed: " + Json.string(rows));
          }
        }
      }
    try {
      Json.write(
          Path.of("target/fence-surface-controls-p" + penalty + "-e" + edgeScale + ".json"), rows);
    } catch (java.io.IOException e) {
      throw new java.io.UncheckedIOException(e);
    }
    System.out.println(Json.string(rows));
    return checks;
  }
}
