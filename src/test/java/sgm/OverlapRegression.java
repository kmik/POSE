package sgm;

import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;

final class OverlapRegression {
  static int checks;

  static void check(boolean yes, String text) {
    checks++;
    if (!yes) throw new AssertionError(text);
  }

  static void near(double a, double b, String text) {
    check(Math.abs(a - b) < 1e-8, text + ": " + a + " != " + b);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Path p = Files.createTempDirectory(Path.of("target"), "overlap test ");
    System.out.println("PASS overlap=" + run(p));
  }

  static Camera down(double x, double y) {
    Camera c = new Camera(100, 100, 49.5, 49.5);
    c.rotation = new double[] {1, 0, 0, 0, -1, 0, 0, 0, -1};
    c.translation = Math3.times(Math3.mv(c.rotation, new double[] {x, y, 10}), -1);
    c.registered = true;
    return c;
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Options o = TestSupport.options();
    o.output = Files.createDirectories(root.resolve("overlap"));
    o.overlapGroundZ = 0;
    o.flightHeading = 90;
    Camera a = down(0, 0), b = down(2, 3);
    var plane = OverlapSelection.plane(new Camera[] {a, b}, List.of(), o);
    var overlap = OverlapSelection.estimate(a, 100, 100, b, 100, 100, plane);
    near(overlap.forward(), 80, "Forward common extent");
    near(overlap.side(), 70, "Side common extent");
    near(overlap.area(), 56, "Actual shared polygon area");
    Camera ga = down(390000, 7200000), gb = down(390002, 7200003);
    var gp = OverlapSelection.plane(new Camera[] {ga, gb}, List.of(), o);
    var ge = OverlapSelection.estimate(ga, 100, 100, gb, 100, 100, gp);
    near(
        ge.area(),
        56,
        "Projected-coordinate footprint intersection avoids large-offset area cancellation");
    var disjoint = OverlapSelection.estimate(a, 100, 100, down(20, 0), 100, 100, plane);
    near(disjoint.area(), 0, "Disjoint footprints");
    near(disjoint.forward(), 0, "Disjoint forward overlap");
    near(
        OverlapSelection.estimate(a, 100, 100, a, 100, 100, plane).area(),
        100,
        "Identical footprint");
    Options uncertain = o.copy();
    uncertain.overlapGroundZ = Double.NaN;
    check(
        OverlapSelection.plane(new Camera[] {a, b}, List.of(), uncertain) == null,
        "No scene plane is unknown, not fabricated overlap");
    Camera unregistered = down(0, 0);
    unregistered.registered = false;
    check(
        Double.isNaN(OverlapSelection.estimate(a, 100, 100, unregistered, 100, 100, plane).area()),
        "Unregistered camera stays unknown");
    List<ImageFrame> frames = new ArrayList<>();
    Calibration calib = new Calibration(100, 100, 49.5, 49.5, new double[5], 100, 100);
    for (int i = 0; i < 3; i++) {
      Path image = o.output.resolve("image" + i + ".png");
      Mat m = new Mat(100, 100, CvType.CV_8UC3, new Scalar(30, 80, 160));
      try {
        Imgcodecs.imwrite(image.toString(), m);
      } finally {
        m.release();
      }
      ImageFrame f = new ImageFrame(i, image, calib, o);
      f.camera.rotation = down(i == 2 ? 20 : i * 2, 0).rotation;
      f.camera.translation = down(i == 2 ? 20 : i * 2, 0).translation;
      f.camera.registered = true;
      frames.add(f);
    }
    try {
      o.neighbors = 2;
      o.minForwardOverlap = 70;
      o.minSideOverlap = 90;
      List<int[]> selected = PairSelection.select(frames, o, null);
      check(
          selected.size() == 1 && selected.getFirst()[0] == 0 && selected.getFirst()[1] == 1,
          "Thresholds also reject sequential fallback edges");
      o.overlapGroundZ = Double.NaN;
      o.overlapUnknown = "keep";
      check(PairSelection.select(frames, o, null).size() == 3, "Unknown keep is explicit");
      o.overlapUnknown = "reject";
      check(PairSelection.select(frames, o, null).isEmpty(), "Unknown reject is explicit");
      o.minForwardOverlap = 0;
      o.minSideOverlap = 0;
      check(
          PairSelection.select(frames, o, null).size() == 3,
          "Disabled thresholds preserve original graph");
      List<double[]> points = new ArrayList<>();
      for (int y = 0; y < 5; y++)
        for (int x = 0; x < 5; x++) points.add(new double[] {x, y, .01 * Math.sin(x)});
      check(
          OverlapSelection.plane(new Camera[] {a, b}, points, o) != null,
          "Constrained sparse scene plane can estimate overlap");
      o.flightHeading = Double.NaN;
      Camera[] turns = {down(0, 0), down(1, 0), down(1, 1), down(0, 1), down(0, 0)};
      o.overlapGroundZ = 0;
      var noDirection = OverlapSelection.plane(turns, List.of(), o);
      check(
          Double.isNaN(noDirection.heading()),
          "Mixed capture directions do not invent a flight axis");
      check(
          Double.isFinite(OverlapSelection.estimate(a, 100, 100, b, 100, 100, noDirection).area()),
          "Nonflight shared-area estimate remains available");
    } finally {
      for (ImageFrame frame : frames) frame.close();
    }
    return checks;
  }
}
