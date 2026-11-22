package sgm;

import java.nio.file.*;
import java.util.*;

/**
 * --dense-method mvsgm hands each camera's map to fusion as pair (camera, best neighbour). The
 * neighbour only weights the triangulation angle, so it must not count as a supporting view where
 * it does not see the point (2026-11-21: very_large edge bands seen by two cameras were recorded as
 * three views and passed --min-views 3, leaving stacked sheets one depth label apart).
 *
 * Three cameras in a row over a plane at depth 10, footprints [-4.75, 4.75], [1.25, 10.75],
 * [7.25, 16.75]; partners 0->1, 1->2, 2->0. With --min-views 2 only the two overlaps may emit.
 */
final class MvsgmPartnerViewsRegression {
  private static int checks;

  private static void check(boolean value, String why) {
    checks++;
    if (!value) throw new AssertionError(why);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Path root = Files.createTempDirectory(Path.of("target"), "partner-views-");
    try {
      System.out.println("PASS mvsgm partner views: " + run(root));
    } finally {
      LargeFormatRegression.delete(root);
    }
  }

  static int run(Path root) throws Exception {
    checks = 0;
    List<double[]> points = fuse(root.resolve("step1"), 1);
    int inOverlap = 0, single = 0;
    for (double[] p : points) {
      double x = p[0];
      // Overlaps [1.25, 4.75] and [7.25, 10.75], widened by one 0.1 m pixel.
      if (x >= 1.15 && x <= 4.85 || x >= 7.15 && x <= 10.85) inOverlap++;
      else single++;
    }
    check(inOverlap > 0, "Overlaps seen by two cameras emit points");
    check(single == 0, "Pixels seen only by one camera (partner outside) are rejected at --min-views 2: " + single);

    // --step 2 with mvsgm: fusion keeps the dense maps (nothing falls to the speckle filter) and
    // each camera emits only its even pixels: about a quarter of the points, over the same area.
    List<double[]> thinned = fuse(root.resolve("step2"), 2);
    double ratio = thinned.size() / (double) points.size();
    check(ratio > .2 && ratio < .3, "--step 2 emits about a quarter of the points: " + thinned.size() + " of " + points.size());
    double[] span = span(points), spanThin = span(thinned);
    check(Math.abs(span[0] - spanThin[0]) < .25 && Math.abs(span[1] - spanThin[1]) < .25,
        "--step 2 covers the same area: " + Arrays.toString(span) + " vs " + Arrays.toString(spanThin));


    // --point-attributes all: fusion's evidence reaches the LAS. Two cameras 6 m apart see the
    // overlap points 10 m away at 31-34 degrees; the uncertainty report is written (identical
    // depth maps: no scatter, so the model sigma is kept with factor 1).
    Path evidenceDir = root.resolve("evidence");
    List<double[]> withFields = fuse(evidenceDir, 1, "all");
    check(withFields.size() == points.size(), "point attributes do not change the points: " + withFields.size() + " vs " + points.size());
    java.nio.ByteBuffer las = java.nio.ByteBuffer.wrap(Files.readAllBytes(evidenceDir.resolve("dense.las"))).order(java.nio.ByteOrder.LITTLE_ENDIAN);
    int data = las.getInt(96), len = las.getShort(105) & 0xffff;
    var all = PointFields.parse("all");
    int angleAt = 26, sigmaAt = 26, viewsAt = 26;
    for (var f : all) {
      if (f.ordinal() < PointFields.Field.ANGLE.ordinal()) angleAt += f.bytes();
      if (f.ordinal() < PointFields.Field.HEIGHT_SIGMA.ordinal()) sigmaAt += f.bytes();
      if (f.ordinal() < PointFields.Field.VIEWS.ordinal()) viewsAt += f.bytes();
    }
    check(len == 26 + PointFields.bytes(all), "record length with all fields: " + len);
    int odd = 0;
    for (int i = 0; i < withFields.size(); i++) {
      int at = data + i * len;
      float angle = las.getFloat(at + angleAt), sigma = las.getFloat(at + sigmaAt);
      int views = las.get(at + viewsAt) & 255;
      if (!(angle > 25 && angle < 40) || !(sigma > 0) || views < 2) odd++;
    }
    check(odd == 0, "angles 25-40 deg, positive height sigma, 2+ views for every point: " + odd + " not");
    String report = Files.readString(evidenceDir.resolve("07_results/point-uncertainty.json"));
    check(report.contains("\"factor\":1") && report.contains("\"calibrated\":false"), "identical maps: no scatter, model sigma kept: " + report);
    // --min-angle (MultiViewSgm.wideEnough): a point 10 m in front of camera 0 (centre at the
    // origin), seen by neighbours with baselines 0.5 m (2.9 deg) and 1.0 m (5.7 deg).
    Camera a = new Camera(100, 100, 47.5, 19.5), narrow = new Camera(100, 100, 47.5, 19.5), wide = new Camera(100, 100, 47.5, 19.5);
    narrow.translation[0] = -.5;
    wide.translation[0] = -1;
    double[] X = {0, 0, 10}, v = new double[3];
    double cos4 = Math.cos(Math.toRadians(4));
    check(!MultiViewSgm.wideEnough(X, a.center(), new Camera[] {narrow}, new int[] {96}, new int[] {40},
        new double[][] {narrow.center()}, cos4, v), "A 2.9 deg neighbour does not reach --min-angle 4");
    check(MultiViewSgm.wideEnough(X, a.center(), new Camera[] {narrow, wide}, new int[] {96, 96}, new int[] {40, 40},
        new double[][] {narrow.center(), wide.center()}, cos4, v), "A 5.7 deg neighbour does");
    // The wide neighbour does not count when the point falls outside its image (x = 37.5, width 30).
    check(!MultiViewSgm.wideEnough(X, a.center(), new Camera[] {narrow, wide}, new int[] {96, 30}, new int[] {40, 40},
        new double[][] {narrow.center(), wide.center()}, cos4, v), "A neighbour that does not see the point does not count");
    check(!MultiViewSgm.wideEnough(X, a.center(), new Camera[] {null, null}, new int[2], new int[2], new double[2][],
        cos4, v), "Neighbours without depth do not count");
    return checks;
  }

  static double[] span(List<double[]> pts) {
    double lo = Double.MAX_VALUE, hi = -Double.MAX_VALUE;
    for (double[] p : pts) {
      lo = Math.min(lo, p[0]);
      hi = Math.max(hi, p[0]);
    }
    return new double[] {lo, hi};
  }

  /** The three-camera fixture fused at --step {@code step}; the emitted points (LAS). */
  static List<double[]> fuse(Path dir, int step) throws Exception {
    return fuse(dir, step, "surface");
  }

  static List<double[]> fuse(Path dir, int step, String attributes) throws Exception {
    Options o = new Options();
    o.output = Files.createDirectories(dir);
    o.projectDirectory = o.output;
    o.denseMethod = "mvsgm";
    o.fusionMode = "image";
    o.minViews = 2;
    o.step = step;
    o.pointAttributes = attributes;
    o.compute = "cpu";
    o.cores = 2;
    int n = 3, w = 96, h = 40;
    Camera[] cameras = new Camera[n];
    int[] widths = new int[n], heights = new int[n];
    for (int c = 0; c < n; c++) {
      widths[c] = w;
      heights[c] = h;
      cameras[c] = new Camera(100, 100, (w - 1) / 2., (h - 1) / 2.);
      cameras[c].translation[0] = -6 * c; // centre at x = 6c
      cameras[c].registered = true;
    }
    try (var maps = new MvsgmMaps(Files.createDirectories(o.output.resolve("source")), false, false);
        var fusion = new DepthFusion(cameras, widths, heights, o)) {
      long[] starts = new long[n + 1];
      for (int c = 0; c < n; c++) {
        int pair = fusion.addPair(c, (c + 1) % n);
        fusion.markSingleView(pair);
        long[] rows = new long[h + 1];
        for (int y = 0; y <= h; y++) rows[y] = (long) y * w;
        maps.add(new MvsgmMaps.Camera(c, pair, w, h, rows, false));
        starts[c + 1] = starts[c] + (long) w * h;
        for (int k = 0; k < MvsgmMaps.KINDS.length; k++) {
          float[] values = new float[w * h];
          Arrays.fill(values, k == MvsgmMaps.DEPTH ? 10 : k == MvsgmMaps.RGB ? Float.intBitsToFloat(0x2277AA) : k == MvsgmMaps.LR ? 0 : .9f);
          DepthMapFile.write(maps.file(c, k), cameras[c], w, h, values);
        }
      }
      try (FusionOutput output = new FusionOutput(o, fusion, null, null)) {
        Path candidates = Files.createFile(o.output.resolve("empty-candidates"));
        ImageFusion.run(o, fusion, candidates, starts, output, new TwoViewConfidence.Diagnostics(o), maps);
        output.finish();
      }
    }
    return CloudDiagnostics.readLas(o.output.resolve("dense.las"));
  }
}
