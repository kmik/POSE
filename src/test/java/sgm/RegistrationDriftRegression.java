package sgm;

import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;

/**
 * Sparse point creation with outlier observations (--triangulation-outliers) and the option
 * checks of --registration-bundle-growth.
 */
final class RegistrationDriftRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Path root = Files.createTempDirectory(Path.of("target"), "registration drift ");
    System.out.println("PASS registration/drift checks=" + run(root) + "; fixtures=" + root);
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Files.createDirectories(root);
    Path image = root.resolve("0.png");
    Imgcodecs.imwrite(image.toString(), new Mat(384, 512, CvType.CV_8UC1, new Scalar(128)));
    Options o = TestSupport.options();
    o.input = root;
    o.output = root.resolve("out");
    o.maxDimension = 512;
    Calibration calibration = new Calibration(360, 360, 255.5, 191.5, new double[5], 512, 384);
    List<ImageFrame> frames = new ArrayList<>();
    int n = 6;
    double[] truth = {3.4, -2.1, 8};
    try {
      for (int i = 0; i < n; i++) {
        ImageFrame f = new ImageFrame(i, image, calibration, o);
        frames.add(f);
        f.camera.registered = true;
        f.camera.rotation = Math3.rotation(new double[] {.01 * i, -.008 * i, .004});
        double[] center = {3 + .25 * i, -2 + .05 * Math.sin(i), .03 * i};
        f.camera.translation = Math3.times(Math3.mv(f.camera.rotation, center), -1);
        f.keypoints = new KeyPoint[4];
        double[] uv = f.camera.project(truth);
        for (int k = 0; k < 4; k++) f.keypoints[k] = new KeyPoint((float) uv[0], (float) uv[1], 1);
      }
      // Feature 1: wrong in the widest-baseline end camera (5); feature 2: wrong in camera 2;
      // feature 3: wrong in two cameras out of six.
      frames.get(5).keypoints[1].pt.x += 30;
      frames.get(2).keypoints[2].pt.y -= 25;
      frames.get(0).keypoints[3].pt.x += 20;
      frames.get(3).keypoints[3].pt.y += 40;

      // All observations agree: every mode gives the same point and keeps all observations.
      double[][] clean = new double[3][];
      for (int mode = 0; mode < 3; mode++) {
        FeatureTracks.Track t = track(n, 0);
        check(Geometry.triangulateTrack(frames, t, 2, mode), "clean track triangulates, mode " + mode);
        check(t.observations.size() == n, "clean track keeps its observations, mode " + mode);
        clean[mode] = t.point;
      }
      check(
          Arrays.equals(clean[0], clean[1]) && Arrays.equals(clean[0], clean[2]),
          "clean track: identical point in every mode");
      check(Math3.norm(Math3.sub(clean[0], truth)) < 1e-3, "clean track point is correct");

      for (int feature = 1; feature <= 3; feature++) {
        int bad = feature == 3 ? 2 : 1;
        FeatureTracks.Track strict = track(n, feature);
        check(
            !Geometry.triangulateTrack(frames, strict, 2, Geometry.TRIANGULATE_STRICT)
                && strict.point == null,
            "strict mode rejects a track with an outlier, feature " + feature);
        check(
            !Geometry.triangulateTrack(frames, strict, 2) && strict.point == null,
            "the three-argument form is still strict");
        FeatureTracks.Track keep = track(n, feature);
        check(
            Geometry.triangulateTrack(frames, keep, 2, Geometry.TRIANGULATE_KEEP),
            "keep mode accepts, feature " + feature);
        check(keep.observations.size() == n, "keep mode leaves observations, feature " + feature);
        FeatureTracks.Track drop = track(n, feature);
        check(
            Geometry.triangulateTrack(frames, drop, 2, Geometry.TRIANGULATE_DROP),
            "drop mode accepts, feature " + feature);
        check(
            drop.observations.size() == n - bad,
            "drop mode removes exactly the outliers, feature " + feature);
        check(
            Math3.norm(Math3.sub(drop.point, truth)) < 1e-3
                && Arrays.equals(drop.point, keep.point),
            "drop/keep point is the inlier point, feature " + feature);
        check(
            Geometry.valid(frames, drop, drop.point, 2),
            "the remaining observations all fit, feature " + feature);
      }
      // Unregistered cameras are neither used nor dropped.
      frames.get(4).camera.registered = false;
      FeatureTracks.Track partial = track(n, 1);
      check(
          Geometry.triangulateTrack(frames, partial, 2, Geometry.TRIANGULATE_DROP)
              && partial.observations.stream().anyMatch(ob -> ob.image == 4)
              && partial.observations.stream().noneMatch(ob -> ob.image == 5),
          "only registered outliers are dropped");
      frames.get(4).camera.registered = true;

      // Two observations that disagree cannot be repaired.
      FeatureTracks.Track two = new FeatureTracks.Track();
      two.observations.add(new FeatureTracks.Observation(0, 2));
      two.observations.add(new FeatureTracks.Observation(2, 2));
      check(
          !Geometry.triangulateTrack(frames, two, 2, Geometry.TRIANGULATE_DROP)
              && two.observations.size() == 2,
          "a disagreeing two-view track stays rejected");
    } finally {
      for (ImageFrame f : frames) f.close();
    }

    // Options.
    check(TestSupport.options().registrationBundleGrowth == 15, "default growth 15 %");
    check(TestSupport.options().triangulationOutliers.equals("drop"), "default outliers drop");
    Options parsed =
        Options.parse(
            new String[] {
              "-i", root.toString(), "-o", root.resolve("p").toString(),
              "--registration-bundle-growth", "7.5", "--triangulation-outliers", "reject"
            });
    check(
        parsed.registrationBundleGrowth == 7.5 && parsed.triangulationOutliers.equals("reject"),
        "options parse");
    check(Geometry.outlierMode(parsed) == Geometry.TRIANGULATE_STRICT, "reject maps to strict");
    for (String[] bad :
        new String[][] {
          {"--registration-bundle-growth", "-1"},
          {"--registration-bundle-growth", "NaN"},
          {"--triangulation-outliers", "maybe"}
        }) {
      boolean failed = false;
      try {
        Options.parse(
            new String[] {"-i", root.toString(), "-o", root.resolve("p").toString(), bad[0], bad[1]});
      } catch (IllegalArgumentException e) {
        failed = true;
      }
      check(failed, "rejects " + bad[0] + " " + bad[1]);
    }
    check(
        Options.help().contains("--registration-bundle-growth")
            && Options.help().contains("--triangulation-outliers"),
        "help lists the options");
    return checks;
  }

  private static FeatureTracks.Track track(int n, int feature) {
    FeatureTracks.Track t = new FeatureTracks.Track();
    for (int i = 0; i < n; i++) t.observations.add(new FeatureTracks.Observation(i, feature));
    return t;
  }
}
