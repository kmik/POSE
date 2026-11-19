package sgm;

import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;

/** Two-view recovery, data-only degeneracy, rollback and explicit calibration contracts. */
final class AdaptiveCalibrationRegression {
  private static int checks;

  private static void check(boolean condition, String message) {
    checks++;
    if (!condition) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Core.setNumThreads(1);
    Path root = Files.createTempDirectory(Path.of("target"), "adaptive-calibration-");
    System.out.println("PASS adaptive calibration: " + run(root) + " checks; " + root);
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Path dir = Files.createDirectories(root.resolve("two-view"));
    Calibration initial = new Calibration(650, 650, 319.5, 239.5, new double[5], 640, 480);
    Options o = TestSupport.options();
    o.output = dir;
    o.alignmentCalibration = true;
    o.maxDimension = 640;
    List<ImageFrame> frames = stereo(dir, initial, o);
    try {
      Calibration truth = new Calibration(650, 650, 319.5, 239.5,
          new double[] {-.10, 0, 0, 0, 0}, 640, 480);
      var tracks = scene(frames, truth);
      FeatureTracks.Track rejectedTrack = new FeatureTracks.Track();
      rejectedTrack.id = tracks.size();
      tracks.add(rejectedTrack);
      var split = CalibrationDiagnostics.split(frames, tracks);
      check(split.train().size() >= 80 && split.first().size() >= 20 && split.second().size() >= 20,
          "Two-view tracks are eligible for disjoint calibration folds");
      check(Collections.disjoint(split.train(), split.first())
          && Collections.disjoint(split.train(), split.second())
          && Collections.disjoint(split.first(), split.second()), "Calibration folds do not leak tracks");
      SharedCalibration lens = new SharedCalibration(frames);
      // Held-out scoring with gross mismatches (a large block before tie-point cleanup): the
      // unweighted RMS is dominated by them, the capped score still sees the lens correction.
      List<FeatureTracks.Track> scored = new ArrayList<>();
      for (var t : split.first()) if (t.point != null) scored.add(t);
      int clean = scored.size();
      for (int i = 0; i < clean / 4; i++) {
        FeatureTracks.Track bad = new FeatureTracks.Track();
        bad.id = -1 - i;
        var a = scored.get(i).observations.get(0);
        var b = scored.get((i + clean / 2) % clean).observations.get(1);
        bad.observations.add(a);
        bad.observations.add(b);
        bad.point = scored.get(i).point.clone();
        scored.add(bad);
      }
      double[] start = lens.heldOutRms(frames, scored, -1);
      check(start[1] == lens.validationRms(frames, scored),
          "The unweighted held-out score is the existing validation RMS");
      double[] startValues = lens.values.clone();
      lens.values[1] = -.10;
      double[] corrected = lens.heldOutRms(frames, scored, -1);
      lens.values = startValues;
      System.out.println("HELD-OUT SCORE: start " + Arrays.toString(start) + " corrected " + Arrays.toString(corrected));
      check(corrected[2] == start[2], "The gate is fixed at the starting model");
      check(corrected[0] < start[0] * .995 && start[0] - corrected[0] > .002,
          "The true lens passes the capped held-out test despite 20% gross mismatches");
      check((start[0] - corrected[0]) / start[0] > 2 * (start[1] - corrected[1]) / start[1],
          "Capping makes the relative lens gain visible behind gross mismatches");
      check(lens.heldOutRms(frames, scored, -1)[0] == start[0], "The capped score is repeatable");
      frames.get(0).camera.fixed = true;
      lens.active[0] = true;
      lens.adaptivePriors = true;
      lens.focalPriors[0] = Math.log(650);
      lens.focalPriorSigma = 1e-9;
      var focal = CalibrationObservability.assess(frames, tracks, 2, lens, .35);
      check(focal.covariance() == null,
          "Pure-translation focal degeneracy rejected even with an arbitrarily strong focal prior");
      lens.active[0] = false;
      lens.active[1] = true;
      var radial = CalibrationObservability.assess(frames, tracks, 2, lens, .35);
      System.out.println("RADIAL INFORMATION: " + radial);
      check(radial.covariance() != null,
          "The same two cameras have data information for radial distortion");
      // A tie point behind both cameras (before the post-bundle cleanup, possible on a large
      // block) has no usable observation: its undamped point block is singular. It is left out
      // instead of failing the whole assessment.
      FeatureTracks.Track behind = new FeatureTracks.Track();
      behind.id = tracks.size();
      for (var t : tracks)
        if (t.point != null && t.observations.size() >= 2) {
          behind.observations.addAll(t.observations);
          break;
        }
      behind.point = new double[] {0, 0, -50};
      tracks.add(behind);
      var robust = CalibrationObservability.assess(frames, tracks, 2, lens, .35);
      check(robust.covariance() != null && robust.reason().contains("1 degenerate tracks"),
          "A singular tie point is left out of the assessment: " + robust.reason());
      for (int k = 0; k < radial.sigma().length; k++)
        check(Math.abs(robust.sigma()[k] - radial.sigma()[k]) <= 1e-9 * Math.max(1, Math.abs(radial.sigma()[k])),
            "Leaving out the degenerate track keeps the information of the others");
      tracks.remove(behind);
      Geometry.triangulateTracks(frames, tracks, 8);
      double baseline = Math3.norm(frames.get(1).camera.center());
      BundleAdjuster.optimize(frames, tracks, 25, 2, null, 0, 1);
      Calibration fitted = frames.get(0).calibration;
      check(rejectedTrack.point == null, "Conditional structure refit skips rejected/untriangulated tracks");
      System.out.println("TWO VIEW RECOVERY: " + fitted.fx + " " + Arrays.toString(fitted.distortion));
      check(Math.abs(fitted.distortion[0] + .1) < .015, "Supported two-view k1 recovered");
      check(fitted.fx == initial.fx && fitted.cx == initial.cx && fitted.cy == initial.cy,
          "Unsupported focal and principal point retained exactly");
      check(Math.abs(fitted.distortion[1]) < .01 && Math.abs(fitted.distortion[4]) < .01,
          "Higher-order parameters do not replace the supported simple model");
      check(Math3.norm(frames.get(0).camera.center()) < 1e-12
          && Math.abs(Math3.norm(frames.get(1).camera.center()) - baseline) < 1e-9,
          "Adaptive calibration preserves the anchor and baseline gauge");
      String report = Files.readString(dir.resolve("04_geometry/bundle/shared_calibration.json"));
      check(report.contains("data_only_standard_deviations") && report.contains("minimum_relative_information")
          && report.contains("accepted_parameters"), "Parameter-level decisions and data uncertainty reported");

      // At a correct model, rejecting every trial must restore poses and points too.
      o.alignmentCalibrationDone = false;
      tracks = scene(frames, fitted);
      Camera[] poses = frames.stream().map(f -> f.camera.copy()).toArray(Camera[]::new);
      double[][] points = tracks.stream().map(t -> t.point.clone()).toArray(double[][]::new);
      o.calibrationTrialIterations = 0;
      o.bundleIterationsExplicit = false;
      SharedCalibration.refine(frames, tracks, 15, 2, null, 0, 1);
      var unchanged = ProcessingReport.map(dir.resolve("04_geometry/bundle/shared_calibration.json"));
      check(((List<?>) unchanged.get("stages")).stream().map(s -> (Map<?, ?>) s)
          .allMatch(s -> s.get("iteration_budget_per_trial") == null
              && "legacy automatic".equals(s.get("iteration_mode"))),
          "Legacy automatic proposals report an unbounded iteration budget without unboxing null");
      o.calibrationTrialIterations = 12;
      check(((Number) unchanged.get("unchanged_model_trials_skipped")).intValue() > 0,
          "A rejected model does not repeat identical parameter proposals");
      check(((List<?>) unchanged.get("stages")).size() < 15,
          "No-change calibration executes fewer full solves");
      check(SharedCalibration.same(fitted, frames.get(0).calibration), "Correct model retained without evidence gain");
      for (int i = 0; i < frames.size(); i++)
        check(Arrays.equals(poses[i].rotation, frames.get(i).camera.rotation)
            && Arrays.equals(poses[i].translation, frames.get(i).camera.translation),
            "Rejected trials restore camera " + i + " exactly");
      for (int i = 0; i < tracks.size(); i++)
        check(Arrays.equals(points[i], tracks.get(i).point), "Rejected trials restore structure exactly");

      o.alignmentCalibrationDone = false;
      o.calibration = dir.resolve("fixed-camera.txt");
      o.intrinsicsMode = "fixed";
      check(!SharedCalibration.enabled(o), "Explicit fixed calibration remains fixed");
      o.calibration = null;
      o.selfCalibration = false;
      check(!SharedCalibration.enabled(o), "Explicit no-self-calibration remains disabled");
      check(!SharedCalibration.validationEvidence(.01, .005, 100, .35),
          "Tiny noise-level improvements do not justify another parameter");
      check(lens.options == o, "The lens reads the run options");
      check(!lens.relaxed() && !lens.heldOutImproved(1, .996, 1, .9)
          && lens.heldOutImproved(1, .99, 1, .99), "strict: both held-out groups improve by 0.5%");
      o.calibrationAcceptance = "relaxed";
      check(lens.relaxed() && lens.heldOutImproved(1, 1, 1, .999) && lens.heldOutImproved(1, .996, 1, .9),
          "relaxed: neither held-out group may get worse");
      check(!lens.heldOutImproved(1, 1.0001, 1, .5) && !lens.heldOutImproved(1, .5, 1, 1.0001)
          && !lens.heldOutImproved(1, Double.POSITIVE_INFINITY, 1, .5)
          && !lens.heldOutImproved(1, .5, 1, Double.NaN), "relaxed: a worse or invalid group rejects");
      o.calibrationAcceptance = "strict";
    } finally {
      for (var f : frames) f.close();
    }
    return checks;
  }

  private static List<ImageFrame> stereo(Path dir, Calibration c, Options o) throws Exception {
    List<ImageFrame> all = SharedCalibrationRegression.frames(dir, c, o);
    for (int i = 2; i < all.size(); i++) all.get(i).close();
    List<ImageFrame> frames = new ArrayList<>(all.subList(0, 2));
    for (int i = 0; i < 2; i++) {
      frames.get(i).camera.rotation = Math3.identity();
      frames.get(i).camera.translation = new double[] {-i, 0, 0};
      frames.get(i).camera.fixed = i == 0;
    }
    return frames;
  }

  private static List<FeatureTracks.Track> scene(List<ImageFrame> frames, Calibration truth) {
    Random rng = new Random(9581);
    List<FeatureTracks.Track> tracks = new ArrayList<>();
    List<List<KeyPoint>> keys = List.of(new ArrayList<>(), new ArrayList<>());
    for (int i = 0; i < 2400; i++) {
      double[] p = {(rng.nextDouble() - .4) * 6, (rng.nextDouble() - .5) * 4.5,
          4 + rng.nextDouble() * 4};
      double[][] uv = new double[2][];
      for (int j = 0; j < 2; j++) uv[j] = SharedCalibrationRegression.brown(frames.get(j).camera, p, truth);
      if (Arrays.stream(uv).anyMatch(q -> q == null || q[0] < 5 || q[0] > 634 || q[1] < 5 || q[1] > 474)) continue;
      FeatureTracks.Track t = new FeatureTracks.Track();
      t.id = tracks.size();
      t.point = p;
      for (int j = 0; j < 2; j++) {
        t.observations.add(new FeatureTracks.Observation(j, keys.get(j).size()));
        keys.get(j).add(new KeyPoint((float) uv[j][0], (float) uv[j][1], 1));
      }
      tracks.add(t);
    }
    for (int j = 0; j < 2; j++) {
      ImageFrame f = frames.get(j);
      Calibration c = f.calibration;
      f.keypoints = keys.get(j).toArray(KeyPoint[]::new);
      f.calibration = new Calibration(c.fx, c.fy, c.cx, c.cy, new double[5], c.width, c.height);
      f.applyCalibration(c);
    }
    return tracks;
  }
}

