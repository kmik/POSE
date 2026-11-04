package sgm;

import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;

/** Independent Brown projection, full-block recovery, and coordinate/cache contracts. */
final class SharedCalibrationRegression {
  static int checks;

  static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  static void near(double a, double b, double tolerance, String message) {
    check(Math.abs(a - b) < tolerance, message + ": " + a + " versus " + b);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Core.setNumThreads(1);
    Path root = Files.createTempDirectory(Path.of("target"), "shared-calibration-");
    System.out.println("PASS shared calibration: " + run(root) + " checks; " + root);
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Path dir = Files.createDirectories(root.resolve("shared-calibration"));
    Calibration initial = new Calibration(610, 610, 319.5, 239.5, new double[5], 640, 480);
    Calibration truth =
        new Calibration(650, 650, 327, 234, new double[] {-.16, .045, .004, -.003, 0}, 640, 480);
    Options o = TestSupport.options();
    o.output = dir;
    o.alignmentCalibration = true;
    o.poses = dir.resolve("fixed-poses.csv");
    o.extrinsicsMode = "fixed";
    o.maxDimension = 640;
    List<ImageFrame> frames = frames(dir, initial, o);
    try {
      List<FeatureTracks.Track> tracks = scene(frames, truth);
      SharedCalibration lens = new SharedCalibration(frames);
      Arrays.fill(lens.active, true);
      double[] p = {.7, -.3, 5.4};
      for (int k = 0; k < SharedCalibration.SIZE; k++) {
        double old = lens.values[k], eps = 1e-6;
        double[][] analytic = lens.lensJacobian(frames.get(2), p);
        lens.values[k] = old + eps;
        double[] plus = lens.project(frames.get(2), p);
        lens.values[k] = old - eps;
        double[] minus = lens.project(frames.get(2), p);
        lens.values[k] = old;
        for (int row = 0; row < 2; row++)
          near(
              analytic[row][k],
              (plus[row] - minus[row]) / (2 * eps),
              1e-5,
              "Lens derivative " + k + "/" + row);
      }
      double[][] jq = lens.spatialJacobian(frames.get(2), p);
      double[] q = frames.get(2).camera.view(p);
      for (int k = 0; k < 3; k++) {
        Camera cam = frames.get(2).camera;
        double[] translation = cam.translation.clone();
        double eps = 1e-6;
        cam.translation[k] += eps;
        double[] plus = lens.project(frames.get(2), p);
        cam.translation[k] -= 2 * eps;
        double[] minus = lens.project(frames.get(2), p);
        cam.translation = translation;
        for (int row = 0; row < 2; row++)
          near(jq[row][k], (plus[row] - minus[row]) / (2 * eps), 1e-5, "Spatial derivative");
      }
      List<Point[]> raw = new ArrayList<>();
      List<Camera> poses = new ArrayList<>();
      for (var f : frames) {
        raw.add(
            Arrays.stream(f.keypoints).map(k -> new Point(k.pt.x, k.pt.y)).toArray(Point[]::new));
        poses.add(f.camera.copy());
      }
      BundleAdjuster.optimize(frames, tracks, 35, 2, null, 0, 1);
      Calibration fitted = frames.get(0).calibration;
      System.out.println(
          "RECOVERED "
              + fitted.fx
              + " "
              + fitted.cx
              + " "
              + fitted.cy
              + " "
              + Arrays.toString(fitted.distortion));
      near(fitted.fx, truth.fx, 2, "Recovered focal");
      near(fitted.cx, truth.cx, 1, "Recovered cx");
      near(fitted.cy, truth.cy, 1, "Recovered cy");
      near(fitted.distortion[0], truth.distortion[0], .005, "Recovered k1");
      near(fitted.distortion[1], truth.distortion[1], .01, "Recovered k2");
      near(fitted.distortion[2], truth.distortion[2], .0005, "Recovered p1");
      near(fitted.distortion[3], truth.distortion[3], .0005, "Recovered p2");
      check(
          Files.isRegularFile(
              dir.resolve("04_geometry/bundle/calibration_diagnostics/initial-image-0.svg")),
          "Image-space residual map exported");
      check(
          Files.readString(dir.resolve("04_geometry/bundle/shared_calibration.json"))
              .contains("second_held_out_tracks"),
          "Two separate held-out groups reported");
      for (int i = 0; i < frames.size(); i++) {
        ImageFrame f = frames.get(i);
        near(
            Math3.norm(Math3.sub(f.camera.translation, poses.get(i).translation)),
            0,
            1e-12,
            "Fixed pose translation");
        for (int j = 0; j < f.keypoints.length; j += 31) {
          Point measured =
              SharedCalibration.distorted(f.camera, f.calibration.distortion, f.keypoints[j].pt);
          near(
              Math.hypot(measured.x - raw.get(i)[j].x, measured.y - raw.get(i)[j].y),
              0,
              1e-6,
              "Raw pixel invariant after calibration");
        }
      }
      check(
          Files.isRegularFile(dir.resolve("04_geometry/bundle/shared_calibration.json")),
          "Staged diagnostics written");
      // A supplied fixed calibration must not change, including nonzero k3 and tangential terms.
      o.alignmentCalibrationDone = false;
      o.calibration = dir.resolve("camera.txt");
      o.intrinsicsMode = "fixed";
      Calibration before = frames.get(0).calibration;
      BundleAdjuster.optimize(frames, tracks, 5, 2, null, 0, 1);
      check(
          SharedCalibration.same(before, frames.get(0).calibration),
          "Supplied fixed calibration preserved");
      // Cache geometry restores calibration and remaps existing feature coordinates before use.
      o.cacheDirectory = Files.createDirectories(dir.resolve("cache"));
      o.resume = true;
      FeatureTracks features = new FeatureTracks();
      features.tracks.addAll(tracks);
      features.pairs.add(new FeatureTracks.Pair(0, 1));
      features.seed = features.pairs.get(0);
      ReuseCache.geometry(frames, features, o, "shared-calibration-test", true);
      for (ImageFrame f : frames) f.applyCalibration(initial);
      check(
          ReuseCache.geometry(frames, features, o, "shared-calibration-test", false),
          "Geometry cache loaded");
      check(
          SharedCalibration.same(before, frames.get(0).calibration),
          "Geometry cache restores refined lens");
      // Source-to-dense resolution propagation uses the same source calibration.
      Options dense = o.copy();
      dense.quality = "high";
      try (ImageFrame f =
          new ImageFrame(0, frames.get(0).path, frames.get(0).calibration, dense, true)) {
        near(
            f.camera.cx,
            (before.cx + .5) * .5 - .5,
            1e-12,
            "Dense principal point pixel-centre scaling");
        check(
            Arrays.equals(f.calibration.distortion, before.distortion),
            "Dense distortion propagated");
      }
    } finally {
      for (var f : frames) f.close();
    }
    Options subset = TestSupport.options();
    subset.output = Files.createDirectories(dir.resolve("subset"));
    subset.alignmentCalibration = true;
    subset.maxDimension = 640;
    subset.poses = subset.output.resolve("poses.csv");
    subset.calibration = subset.output.resolve("supplied-camera.txt");
    subset.intrinsicsMode = "prior";
    subset.refineIntrinsics = "cx,cy,p1,p2";
    Calibration supplied =
        new Calibration(650, 680, 319.5, 239.5, new double[] {-.16, .045, 0, 0, .01}, 640, 480);
    Calibration target =
        new Calibration(650, 680, 327, 234, new double[] {-.16, .045, .004, -.003, .01}, 640, 480);
    List<ImageFrame> limited = frames(subset.output, supplied, subset);
    try {
      List<FeatureTracks.Track> tracks = scene(limited, target);
      BundleAdjuster.optimize(limited, tracks, 25, 2, null, 0, 1);
      Calibration c = limited.get(0).calibration;
      check(c.fx == supplied.fx && c.fy == supplied.fy, "Unselected focal values remain exact");
      check(
          c.distortion[0] == supplied.distortion[0]
              && c.distortion[1] == supplied.distortion[1]
              && c.distortion[4] == supplied.distortion[4],
          "Unselected radial and k3 values remain exact");
      near(c.cx, target.cx, 1, "Prior-mode cx");
      near(c.cy, target.cy, 1, "Prior-mode cy");
      near(c.distortion[2], target.distortion[2], .0005, "Prior-mode p1");
      near(c.distortion[3], target.distortion[3], .0005, "Prior-mode p2");
      subset.alignmentCalibrationDone = false;
      subset.selfCalibration = false;
      Calibration retained = c;
      SharedCalibration.refine(limited, tracks, 12, 2, null, 0, 1);
      check(
          SharedCalibration.same(retained, limited.get(0).calibration),
          "No-self-calibration keeps all supplied values");
    } finally {
      for (var f : limited) f.close();
    }
    Options radial = TestSupport.options();
    radial.output = Files.createDirectories(dir.resolve("k3"));
    radial.maxDimension = 640;
    radial.poses = radial.output.resolve("poses.csv");
    radial.refineIntrinsics = "k3";
    Calibration radialTruth =
        new Calibration(650, 650, 319.5, 239.5, new double[] {-.16, .045, 0, 0, -.08}, 640, 480);
    Calibration radialStart =
        new Calibration(650, 650, 319.5, 239.5, new double[] {-.16, .045, 0, 0, 0}, 640, 480);
    List<ImageFrame> radialFrames = frames(radial.output, radialStart, radial);
    try {
      var observations = scene(radialFrames, radialTruth);
      var lens = new SharedCalibration(radialFrames);
      lens.active[7] = true;
      BundleAdjuster.optimizeImpl(radialFrames, observations, 35, 2, null, 0, 1, lens);
      near(lens.calibration().distortion[4], -.08, .002, "Independent nonzero k3 recovery");
      Arrays.fill(lens.active, false);
      lens.active[2] = true;
      double[] uncertainty = new double[SharedCalibration.SIZE];
      double cornerR2 = (320. * 320 + 240. * 240) / (650. * 650);
      uncertainty[2] = .025 / (cornerR2 * cornerR2);
      check(
          lens.constrained(uncertainty, .7) && !lens.constrained(uncertainty, 1),
          "Observed-radius fit distinguished from unsupported corner extrapolation");
      check(!lens.constrained(uncertainty, .4), "Central-only calibration still rejected");
      uncertainty[2] = Double.NaN;
      check(!lens.constrained(uncertainty, .7), "Nonfinite uncertainty rejected");
      Arrays.fill(uncertainty, 0);
      Arrays.fill(lens.active, false);
      lens.active[0] = true;
      uncertainty[0] = .09;
      check(!lens.constrained(uncertainty, .7), "Weak focal geometry remains rejected");
      check(
          lens.calibration().fx == radialStart.fx
              && lens.calibration().distortion[0] == radialStart.distortion[0],
          "Explicit k3 selection preserves other parameters exactly");
      var first = CalibrationDiagnostics.split(radialFrames, observations);
      var again = CalibrationDiagnostics.split(radialFrames, observations);
      check(
          first.train().equals(again.train())
              && first.first().equals(again.first())
              && first.second().equals(again.second()),
          "Balanced splits repeat exactly");
      Set<FeatureTracks.Track> heldSet = new HashSet<>(first.first());
      heldSet.addAll(first.second());
      check(
          Collections.disjoint(first.train(), heldSet)
              && Collections.disjoint(first.first(), first.second()),
          "No calibration track leaks across training and validation groups");
    } finally {
      for (var frame : radialFrames) frame.close();
    }
    // Exercise the actual joint pose/lens Schur coupling, not only known-pose fitting.
    Options joint = TestSupport.options();
    joint.output = Files.createDirectories(dir.resolve("joint"));
    joint.alignmentCalibration = true;
    joint.maxDimension = 640;
    List<ImageFrame> moving = frames(joint.output, initial, joint);
    try {
      List<FeatureTracks.Track> tracks = scene(moving, truth);
      double[] reference = moving.get(0).camera.center();
      double baseline = Math3.norm(Math3.sub(moving.get(1).camera.center(), reference));
      BundleAdjuster.optimize(moving, tracks, 35, 2, null, 0, 1);
      Calibration c = moving.get(0).calibration;
      near(c.fx, truth.fx, 4, "Joint focal recovery");
      near(c.cx, truth.cx, 2, "Joint cx recovery");
      near(c.cy, truth.cy, 2, "Joint cy recovery");
      near(c.distortion[0], truth.distortion[0], .01, "Joint k1 recovery");
      near(c.distortion[1], truth.distortion[1], .02, "Joint k2 recovery");
      near(c.distortion[2], truth.distortion[2], .001, "Joint p1 recovery");
      near(c.distortion[3], truth.distortion[3], .001, "Joint p2 recovery");
      near(
          Math3.norm(Math3.sub(moving.get(0).camera.center(), reference)),
          0,
          1e-10,
          "Joint anchor preserved");
      near(
          Math3.norm(Math3.sub(moving.get(1).camera.center(), reference)),
          baseline,
          1e-9,
          "Joint baseline gauge preserved");
      // At the exact optimum, attempts must be rejected and keep the previous solution intact.
      joint.alignmentCalibrationDone = false;
      Calibration before = moving.get(0).calibration;
      SharedCalibration.refine(moving, tracks, 12, 2, null, 0, 1);
      check(
          SharedCalibration.same(before, moving.get(0).calibration),
          "No-improvement stages preserve accepted calibration");
      SharedCalibration model = new SharedCalibration(moving);
      Arrays.fill(model.active, true);
      model.values[5] = .5;
      check(!model.physical(), "Extreme tangential distortion rejected");
      model.values = model.prior.clone();
      model.values[3] = 1.2;
      check(!model.physical(), "Extreme principal point rejected");
    } finally {
      for (var f : moving) f.close();
    }
    return checks;
  }

  static List<ImageFrame> frames(Path dir, Calibration c, Options o) throws Exception {
    List<ImageFrame> frames = new ArrayList<>();
    Mat image = new Mat(480, 640, CvType.CV_8UC3, new Scalar(100, 100, 100));
    try {
      for (int i = 0; i < 8; i++) {
        Path path = dir.resolve("view" + i + ".png");
        Imgcodecs.imwrite(path.toString(), image);
        ImageFrame f = new ImageFrame(i, path, c, o, true);
        frames.add(f);
        f.camera.rotation =
            Math3.rotation(
                new double[] {.14 * Math.sin(i * 1.7), .18 * Math.cos(i * .8), .12 * Math.sin(i)});
        double[] center = {1.2 * Math.cos(i * .9), .9 * Math.sin(i * .9), .3 * Math.sin(i * 1.6)};
        f.camera.translation = Math3.times(Math3.mv(f.camera.rotation, center), -1);
        f.camera.registered = true;
      }
    } finally {
      image.release();
    }
    return frames;
  }

  static List<FeatureTracks.Track> scene(List<ImageFrame> frames, Calibration truth) {
    Random random = new Random(6741);
    List<FeatureTracks.Track> tracks = new ArrayList<>();
    List<List<KeyPoint>> keys = new ArrayList<>();
    for (var f : frames) keys.add(new ArrayList<>());
    for (int i = 0; i < 900; i++) {
      double[] p = {
        (random.nextDouble() - .5) * 5, (random.nextDouble() - .5) * 4, 4 + random.nextDouble() * 5
      };
      FeatureTracks.Track t = new FeatureTracks.Track();
      t.id = i;
      t.point = p.clone();
      for (ImageFrame f : frames) {
        double[] uv = brown(f.camera, p, truth);
        if (uv == null || uv[0] < 5 || uv[0] > 634 || uv[1] < 5 || uv[1] > 474) continue;
        List<KeyPoint> k = keys.get(f.id);
        t.observations.add(new FeatureTracks.Observation(f.id, k.size()));
        k.add(new KeyPoint((float) uv[0], (float) uv[1], 1));
      }
      if (t.observations.size() >= 3) tracks.add(t);
    }
    for (ImageFrame f : frames) {
      f.keypoints = keys.get(f.id).toArray(KeyPoint[]::new);
      Calibration supplied = f.calibration;
      f.calibration =
          new Calibration(
              supplied.fx,
              supplied.fy,
              supplied.cx,
              supplied.cy,
              new double[5],
              supplied.width,
              supplied.height);
      f.applyCalibration(supplied);
    }
    return tracks;
  }

  static double[] brown(Camera pose, double[] point, Calibration c) {
    double[] q = pose.view(point);
    if (q[2] <= 0) return null;
    double x = q[0] / q[2], y = q[1] / q[2], rr = x * x + y * y;
    double scale =
        1 + c.distortion[0] * rr + c.distortion[1] * rr * rr + c.distortion[4] * rr * rr * rr;
    double u = x * scale + 2 * c.distortion[2] * x * y + c.distortion[3] * (3 * x * x + y * y);
    double v = y * scale + c.distortion[2] * (x * x + 3 * y * y) + 2 * c.distortion[3] * x * y;
    return new double[] {c.fx * u + c.cx, c.fy * v + c.cy};
  }
}
