package sgm;

import java.nio.file.*;
import java.util.*;

/**
 * Small synthetic controls only: unchanged disparity/legacy scores, right evidence and two-view
 * gating.
 */
final class TwoViewConfidenceRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Path root = Files.createDirectories(Path.of("target/two-view-experiment/regression"));
    System.out.println("PASS two-view confidence: " + run(root));
    GpuSgm.closeSession();
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Files.createDirectories(root);
    for (String bad : List.of("NaN", "Infinity", "-Infinity", "-.01", "1.01")) {
      boolean rejected = false;
      try {
        Options.parse(new String[] {"-i", root.toString(), "--two-view-confidence-threshold", bad});
      } catch (IllegalArgumentException expected) {
        rejected = true;
      }
      check(rejected, "invalid experimental threshold rejected");
    }
    for (double t : new double[] {0, .35, .5, 1}) {
      Options parsed =
          Options.parse(
              new String[] {
                "-i", root.toString(), "--two-view-confidence-threshold", Double.toString(t)
              });
      check(parsed.copy().twoViewConfidenceThreshold == t, "threshold copied");
    }
    check(
        Double.isNaN(new Options().twoViewConfidenceThreshold),
        "experimental gate disabled by default");
    check(
        TwoViewConfidence.apply(result(2), .9, .1, .5)
            .status
            .equals("two_view_pair_confidence_rejected"),
        "weak reverse match rejected");
    check(
        TwoViewConfidence.apply(result(2), .4, .9, .5)
            .status
            .equals("two_view_pair_confidence_rejected"),
        "stricter left threshold applied too");
    check(TwoViewConfidence.apply(result(2), .5, .5, .5).accepted(), "threshold equality passes");
    check(
        TwoViewConfidence.apply(result(2), .9, Double.NaN, .5)
            .status
            .equals("two_view_right_confidence_unavailable"),
        "unavailable right evidence rejected");
    check(
        TwoViewConfidence.apply(result(2), .9, Double.NaN, Double.NaN).accepted(),
        "omitted option preserves legacy behavior");
    check(
        !TwoViewConfidence.apply(result(2), .3, .9, 0).accepted(),
        "legacy floor not weakened by zero threshold");
    for (int views : new int[] {3, 4, 12}) {
      var original = result(views);
      check(
          TwoViewConfidence.apply(original, .01, Double.NaN, 1) == original,
          "3+ view result untouched");
    }
    int w = 112, h = 48, n = w * h;
    byte[] a = new byte[n], b = new byte[n], ma = new byte[n], mb = new byte[n];
    new Random(8832).nextBytes(a);
    Arrays.fill(ma, (byte) 255);
    Arrays.fill(mb, (byte) 255);
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) b[y * w + x] = a[y * w + Math.min(w - 1, x + 7)];
    for (int y = 12; y < 28; y++) for (int x = 37; x < 40; x++) mb[y * w + x] = 0;
    for (int sign : new int[] {1, -1}) {
      byte[] left = sign == 1 ? a : b,
          right = sign == 1 ? b : a,
          mLeft = sign == 1 ? ma : mb,
          mRight = sign == 1 ? mb : ma;
      Options o = TestSupport.options();
      o.output = root;
      o.minDisparity = -12;
      o.disparities = 31;
      o.median = 3;
      o.speckleSize = 0;
      var legacy = new CensusSgm(o).compute(left, right, null, null, w, h, mLeft, mRight);
      PointConfidence.stereo(legacy, left, right);
      o.twoViewConfidenceThreshold = .5;
      var experimental = new CensusSgm(o).compute(left, right, null, null, w, h, mLeft, mRight);
      PointConfidence.stereo(experimental, left, right);
      check(
          Arrays.equals(legacy.disparity, experimental.disparity),
          "experimental metadata does not change CPU disparities");
      check(
          Arrays.equals(legacy.confidence, experimental.confidence), "legacy confidence unchanged");
      check(
          legacy.stats.csv().equals(experimental.stats.csv()), "stereo rejection counts unchanged");
      int known = 0, strong = 0;
      for (int i = 0; i < n; i++)
        if (Float.isFinite(experimental.rightConfidence[i])) {
          known++;
          check(
              PointConfidence.valid(experimental.rightConfidence[i]),
              "finite reverse score in 0..1");
          if (experimental.rightConfidence[i] > .35) strong++;
        }
      check(
          known > 1500 && strong > 1000,
          "textured signed-shift plane retains strong right evidence");
      for (int device = 0; device < GpuSgm.devices().size(); device++)
        for (String mode : List.of("resident", "hybrid", "streamed", "esgm")) {
          Options gpu = o.copy();
          gpu.compute = "gpu";
          gpu.gpuDevice = device;
          gpu.gpuSgm = mode;
          gpu.gpuStripRows = 7;
          var actual = new CensusSgm(gpu).compute(left, right, null, null, w, h, mLeft, mRight);
          PointConfidence.stereo(actual, left, right);
          check(
              Arrays.equals(experimental.disparity, actual.disparity),
              "GPU disparity exact: " + device + "/" + mode);
          check(experimental.stats.csv().equals(actual.stats.csv()), "GPU rejection counts exact");
          if (!mode.equals("esgm"))
            check(
                Arrays.equals(experimental.rightConfidence, actual.rightConfidence),
                "CPU/GPU reverse score exact");
          else {
            for (int i = 0; i < n; i++)
              if (Float.isFinite(actual.rightConfidence[i])
                  && Float.isFinite(experimental.rightConfidence[i]))
                check(
                    actual.rightConfidence[i] <= experimental.rightConfidence[i] + 1e-6,
                    "compact reverse confidence conservative");
          }
        }
      o.cacheDirectory = root.resolve("cache-" + sign);
      o.resume = true;
      o.twoViewConfidenceThreshold = Double.NaN;
      var legacyCache =
          ReuseCache.stereo(
              o,
              left,
              right,
              null,
              null,
              w,
              h,
              mLeft,
              mRight,
              null,
              root.resolve("legacy-cache-" + sign));
      check(legacyCache.rightConfidence == null, "legacy cache contains no reverse metadata");
      o.twoViewConfidenceThreshold = .5;
      var stored =
          ReuseCache.stereo(
              o, left, right, null, null, w, h, mLeft, mRight, null, root.resolve("write-" + sign));
      o.twoViewConfidenceThreshold = .65;
      var loaded =
          ReuseCache.stereo(
              o, left, right, null, null, w, h, mLeft, mRight, null, root.resolve("read-" + sign));
      check(
          Arrays.equals(stored.rightConfidence, loaded.rightConfidence),
          "cache round-trips reverse evidence across thresholds");
      check(Arrays.equals(stored.confidence, loaded.confidence), "cache preserves legacy score");
    }
    for (boolean color : new boolean[] {false, true}) {
      Options o = TestSupport.options();
      o.output = root;
      o.minDisparity = -12;
      o.disparities = 31;
      o.twoViewConfidenceThreshold = .5;
      o.median = 3;
      o.speckleSize = 0;
      o.sgmAggregate = "32";
      o.directions = 4;
      o.subpixelRefinement = "zncc";
      o.matchingCost = color ? "census" : "ad-census";
      o.censusMode = color ? "color-distance" : "gray";
      byte[] ca = new byte[n * 3], cb = new byte[n * 3];
      for (int i = 0; i < n; i++)
        for (int c = 0; c < 3; c++) {
          ca[i * 3 + c] = (byte) ((a[i] & 255) ^ (c * 83));
          cb[i * 3 + c] = (byte) ((b[i] & 255) ^ (c * 83));
        }
      var expected = new CensusSgm(o).compute(a, b, ca, cb, w, h, ma, mb);
      var tiled = o.copy();
      tiled.tiled = true;
      var onDisk = new CensusSgm(tiled).compute(a, b, ca, cb, w, h, ma, mb);
      check(
          Arrays.equals(expected.rightConfidence, onDisk.rightConfidence),
          "tiled reverse evidence exact with refinement/color/32-bit controls");
      if (!GpuSgm.devices().isEmpty()) {
        o.compute = "gpu";
        o.gpuSgm = "hybrid";
        var actual = new CensusSgm(o).compute(a, b, ca, cb, w, h, ma, mb);
        check(
            Arrays.equals(expected.disparity, actual.disparity),
            "refined/color/32-bit GPU disparity exact");
        check(
            Arrays.equals(expected.rightConfidence, actual.rightConfidence),
            "refined/color/32-bit reverse score exact");
      }
    }
    return checks;
  }

  private static DepthFusion.Result result(int views) {
    return new DepthFusion.Result(
        new double[] {0, 0, 4}, views, Math.max(0, views - 2), .01, 0, "accepted");
  }
}
