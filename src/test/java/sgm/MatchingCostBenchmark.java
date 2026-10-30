package sgm;

import java.nio.file.*;
import java.util.*;

/** Controlled continuous textured plane; timings include CPU matching and post-filtering. */
final class MatchingCostBenchmark {
  public static void main(String[] args) throws Exception {
    int w = 384, h = 192;
    double truth = 13.37;
    byte[] a = new byte[w * h], b = new byte[a.length];
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) {
        a[y * w + x] = SubpixelRegression.texture(x, y);
        b[y * w + x] = SubpixelRegression.texture(x + truth, y);
      }
    List<Object> rows = new ArrayList<>();
    for (String cost : List.of("census", "ad-census", "census-gradient", "zncc")) {
      Options o = TestSupport.options();
      o.disparities = 32;
      o.median = 1;
      o.speckleSize = 0;
      o.correlation = .5;
      o.matchingCost = cost;
      CensusSgm.Result result = null;
      double[] times = new double[3];
      for (int trial = -1; trial < 3; trial++) {
        long start = System.nanoTime();
        result = new CensusSgm(o).compute(a, b, w, h);
        if (trial >= 0) times[trial] = (System.nanoTime() - start) / 1e9;
      }
      double[] metrics = SubpixelRegression.metrics(result, truth);
      if (metrics[0] < 40000 || metrics[1] > .5)
        throw new AssertionError("Cost failed independent fractional plane: " + cost);
      double[] sorted = times.clone();
      Arrays.sort(sorted);
      rows.add(
          Json.object(
              "cost",
              cost,
              "valid_pixels_in_test_region",
              metrics[0],
              "rms_disparity_error_pixels",
              metrics[1],
              "p90_p10_disparity_spread",
              metrics[2],
              "median_cpu_seconds",
              sorted[1],
              "cpu_trials_seconds",
              times));
    }
    Object report =
        Json.object(
            "width",
            w,
            "height",
            h,
            "disparities",
            32,
            "true_disparity",
            truth,
            "scope",
            "Single controlled continuous-texture plane; CPU times include filters, one discarded"
                + " warm-up and three timed trials; no general accuracy/performance ranking"
                + " implied",
            "results",
            rows);
    Json.write(Path.of("target/matching-cost-benchmark.json"), report);
    System.out.println(Json.string(report));
  }
}
