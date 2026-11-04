package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;

/**
 * Shared Brown camera block in the Schur pose/structure bundle solve. Observations stay in the
 * original distorted pixel frame throughout each staged fit.
 */
final class SharedCalibration {
  // Beyond the lens (0-7): 8 rolling-shutter readout time (s, positive when rows are read top to
  // bottom), 9 GPS time offset (s), 10-12 GPS lever arm (camera axes, m). They are never part of
  // the staged lens refinement; only the GPS-weighted adjustment activates them.
  static final int SIZE = 13, LENS = 8, READOUT = 8, TIME = 9, LEVER = 10;
  static final String[] NAMES = {
    "f", "k1", "k2", "cx", "cy", "p1", "p2", "k3", "rs", "dt", "lx", "ly", "lz"
  };
  final Calibration initial;
  final Options options;
  final Point[][] observed;
  final boolean fixedPoses;
  final boolean[] active = new boolean[SIZE];
  double[] values;
  final double[] prior;
  // GPS-weighted adjustment priors: log focal (NaN = none) with log sigma; readout sigma (s).
  double focalPrior = Double.NaN, focalPriorSigma = .03, readoutSigma = .1;

  SharedCalibration(List<ImageFrame> frames) {
    initial = frames.get(0).calibration;
    options = frames.get(0).options;
    fixedPoses = options.poses != null && options.extrinsicsMode.equals("fixed");
    values =
        new double[] {
          Math.log(initial.fx),
          initial.distortion[0],
          initial.distortion[1],
          initial.cx / initial.width,
          initial.cy / initial.height,
          initial.distortion[2],
          initial.distortion[3],
          initial.distortion[4],
          0,
          0,
          0,
          0,
          0
        };
    prior = values.clone();
    observed = new Point[frames.size()][];
    for (ImageFrame f : frames) {
      observed[f.id] = new Point[f.keypoints.length];
      for (int i = 0; i < f.keypoints.length; i++)
        observed[f.id][i] = distorted(f.camera, f.calibration.distortion, f.keypoints[i].pt);
    }
  }

  static Point distorted(Camera c, double[] d, Point pixel) {
    double x = (pixel.x - c.cx) / c.fx, y = (pixel.y - c.cy) / c.fy, r = x * x + y * y;
    double radial = 1 + d[0] * r + d[1] * r * r + d[4] * r * r * r;
    return new Point(
        c.fx * (x * radial + 2 * d[2] * x * y + d[3] * (r + 2 * x * x)) + c.cx,
        c.fy * (y * radial + d[2] * (r + 2 * y * y) + 2 * d[3] * x * y) + c.cy);
  }

  Calibration calibration() {
    double f = values[0] == prior[0] ? initial.fx : Math.exp(values[0]);
    return new Calibration(
        f,
        values[0] == prior[0] ? initial.fy : f * initial.fy / initial.fx,
        values[3] == prior[3] ? initial.cx : values[3] * initial.width,
        values[4] == prior[4] ? initial.cy : values[4] * initial.height,
        new double[] {values[1], values[2], values[5], values[6], values[7]},
        initial.width,
        initial.height);
  }

  /** True when camera position priors are GPS antenna fixes: C + R^T L + V dt (indices 9-12). */
  boolean antennaModel() {
    for (int k = TIME; k < SIZE; k++) if (active[k] || values[k] != 0) return true;
    return false;
  }

  /** GPS antenna position of a camera: C + R^T L + V dt. */
  double[] antenna(Camera c) {
    double[] p =
        Math3.add(
            c.center(),
            Math3.mv(
                Math3.transpose(c.rotation),
                new double[] {values[LEVER], values[LEVER + 1], values[LEVER + 2]}));
    return c.velocity == null ? p : Math3.add(p, Math3.times(c.velocity, values[TIME]));
  }

  /** Signed offset of an observed row from the mid-frame row, in frame heights; 0 if unmodelled. */
  private static double rowFraction(ImageFrame frame, double row) {
    return frame.camera.velocity == null || !Double.isFinite(row) ? 0 : row / frame.height - .5;
  }

  /**
   * Camera-frame point at the exposure time of the observed row. Rows are read sequentially, so
   * row y sees the camera at C + V s with s = readout * (y / height - 1/2): q = R (X - C - V s).
   * Translation only; the gimbal keeps rotation rates small between turns.
   */
  double[] view(ImageFrame frame, double[] point, double row) {
    double[] q = frame.camera.view(point);
    double s = values[8] * rowFraction(frame, row);
    if (s != 0) {
      double[] rv = Math3.mv(frame.camera.rotation, frame.camera.velocity);
      for (int k = 0; k < 3; k++) q[k] -= s * rv[k];
    }
    return q;
  }

  double[] project(ImageFrame frame, double[] point) {
    return project(frame, point, Double.NaN);
  }

  double[] project(ImageFrame frame, double[] point, double row) {
    double[] q = view(frame, point, row);
    return q[2] > 1e-8 ? projectView(frame, q) : null;
  }

  private double[] projectView(ImageFrame frame, double[] q) {
    double fx = Math.exp(values[0]) * frame.width / initial.width;
    double fy = Math.exp(values[0]) * initial.fy / initial.fx * frame.height / initial.height;
    double x = q[0] / q[2], y = q[1] / q[2], r = x * x + y * y;
    double radial = 1 + values[1] * r + values[2] * r * r + values[7] * r * r * r;
    return new double[] {
      fx * (x * radial + 2 * values[5] * x * y + values[6] * (r + 2 * x * x))
          + (values[3] * initial.width + .5) * frame.width / initial.width
          - .5,
      fy * (y * radial + values[5] * (r + 2 * y * y) + 2 * values[6] * x * y)
          + (values[4] * initial.height + .5) * frame.height / initial.height
          - .5
    };
  }

  double[][] lensJacobian(ImageFrame frame, double[] point) {
    return lensJacobian(frame, point, Double.NaN);
  }

  double[][] lensJacobian(ImageFrame frame, double[] point, double row) {
    double[] q = view(frame, point, row), uv = projectView(frame, q);
    double x = q[0] / q[2], y = q[1] / q[2], r = x * x + y * y;
    double fx = Math.exp(values[0]) * frame.width / initial.width;
    double fy = Math.exp(values[0]) * initial.fy / initial.fx * frame.height / initial.height;
    double cx = (values[3] * initial.width + .5) * frame.width / initial.width - .5;
    double cy = (values[4] * initial.height + .5) * frame.height / initial.height - .5;
    double[][] j = new double[2][SIZE];
    double[][] lens = {
      {
        uv[0] - cx,
        fx * x * r,
        fx * x * r * r,
        frame.width,
        0,
        2 * fx * x * y,
        fx * (r + 2 * x * x),
        fx * x * r * r * r
      },
      {
        uv[1] - cy,
        fy * y * r,
        fy * y * r * r,
        0,
        frame.height,
        fy * (r + 2 * y * y),
        2 * fy * x * y,
        fy * y * r * r * r
      }
    };
    for (int i = 0; i < 2; i++) System.arraycopy(lens[i], 0, j[i], 0, LENS);
    double fraction = rowFraction(frame, row);
    if (active[8] && fraction != 0) {
      // d q / d readout = -fraction * R V.
      double[] rv = Math3.mv(frame.camera.rotation, frame.camera.velocity);
      double[][] jq = spatialView(frame, q);
      for (int i = 0; i < 2; i++)
        for (int k = 0; k < 3; k++) j[i][8] -= jq[i][k] * fraction * rv[k];
    }
    for (int k = 0; k < SIZE; k++)
      if (!active[k]) {
        j[0][k] = 0;
        j[1][k] = 0;
      }
    return j;
  }

  double[][] spatialJacobian(ImageFrame frame, double[] point) {
    return spatialJacobian(frame, point, Double.NaN);
  }

  /** Derivative of the projection with respect to the (rolling-shutter) camera-frame point. */
  double[][] spatialJacobian(ImageFrame frame, double[] point, double row) {
    return spatialView(frame, view(frame, point, row));
  }

  private double[][] spatialView(ImageFrame frame, double[] q) {
    double x = q[0] / q[2], y = q[1] / q[2], r = x * x + y * y;
    double fx = Math.exp(values[0]) * frame.width / initial.width;
    double fy = Math.exp(values[0]) * initial.fy / initial.fx * frame.height / initial.height;
    double a = 1 + values[1] * r + values[2] * r * r + values[7] * r * r * r;
    double slope = values[1] + 2 * values[2] * r + 3 * values[7] * r * r;
    double ux = fx * (a + 2 * x * x * slope + 2 * values[5] * y + 6 * values[6] * x);
    double uy = fx * (2 * x * y * slope + 2 * values[5] * x + 2 * values[6] * y);
    double vx = fy * (2 * x * y * slope + 2 * values[5] * x + 2 * values[6] * y);
    double vy = fy * (a + 2 * y * y * slope + 6 * values[5] * y + 2 * values[6] * x);
    return new double[][] {
      {ux / q[2], uy / q[2], -(ux * x + uy * y) / q[2]},
      {vx / q[2], vy / q[2], -(vx * x + vy * y) / q[2]}
    };
  }

  void move(double[] delta, int offset) {
    for (int k = 0; k < SIZE; k++) if (active[k]) values[k] += delta[offset + k];
  }

  private double sigma(int k) {
    return k == 0 ? options.focalSigma : k == 3 || k == 4 ? .02 : options.radialSigma;
  }

  /** Prior {target, sigma} of an active parameter, or null when it has none. */
  private double[] priorOf(int k) {
    if (k == READOUT) return new double[] {0, readoutSigma};
    if (k == TIME) return new double[] {0, GpsAdjustment.TIME_SIGMA};
    if (k >= LEVER) return new double[] {0, GpsAdjustment.LEVER_SIGMA};
    if (k == 0 && Double.isFinite(focalPrior)) return new double[] {focalPrior, focalPriorSigma};
    return options.calibration != null && options.intrinsicsMode.equals("prior")
        ? new double[] {prior[k], sigma(k)}
        : null;
  }

  double priorCost() {
    double cost = 0;
    for (int k = 0; k < SIZE; k++) {
      double[] p = active[k] ? priorOf(k) : null;
      if (p != null) cost += .5 * Math.pow((values[k] - p[0]) / p[1], 2);
    }
    return cost;
  }

  void priorNormal(double[] normal, double[] rhs) {
    for (int k = 0; k < SIZE; k++) {
      if (!active[k]) {
        normal[k * SIZE + k] += 1;
        rhs[k] = 0;
        continue;
      }
      double[] p = priorOf(k);
      if (p != null) {
        double weight = 1 / (p[1] * p[1]);
        normal[k * SIZE + k] += weight;
        rhs[k] -= weight * (values[k] - p[0]);
      }
    }
  }

  boolean physical() {
    for (double x : values) if (!Double.isFinite(x)) return false;
    if (Math.abs(values[0] - prior[0]) > Math.log(2)
        || Math.abs(values[1]) > 2
        || Math.abs(values[2]) > 3
        || Math.abs(values[7]) > 5
        || Math.abs(values[3] - .5) > .25
        || Math.abs(values[4] - .5) > .25
        || Math.abs(values[5]) > .1
        || Math.abs(values[6]) > .1
        || Math.abs(values[8]) > .25
        || Math.abs(values[TIME]) > 2
        || Math.abs(values[LEVER]) > 2
        || Math.abs(values[LEVER + 1]) > 2
        || Math.abs(values[LEVER + 2]) > 2) return false;
    // Reject folding anywhere on a conservative grid spanning the source image.
    double f = Math.exp(values[0]), fy = f * initial.fy / initial.fx;
    for (int iy = 0; iy <= 8; iy++)
      for (int ix = 0; ix <= 8; ix++) {
        double x = (ix * initial.width / 8. - values[3] * initial.width) / f;
        double y = (iy * initial.height / 8. - values[4] * initial.height) / fy;
        double r = x * x + y * y, a = 1 + values[1] * r + values[2] * r * r + values[7] * r * r * r;
        double slope = values[1] + 2 * values[2] * r + 3 * values[7] * r * r;
        double xx = a + 2 * x * x * slope + 2 * values[5] * y + 6 * values[6] * x;
        double xy = 2 * x * y * slope + 2 * values[5] * x + 2 * values[6] * y;
        double yy = a + 2 * y * y * slope + 6 * values[5] * y + 2 * values[6] * x;
        if (xx < .2 || yy < .2 || xx * yy - xy * xy < .04) return false;
      }
    return true;
  }

  boolean constrained(double[] sigma, double coverage) {
    if (sigma == null || !Double.isFinite(coverage) || coverage < .55) return false;
    // Judge supported radial corrections over the measured P90 radius. Applying the same bound
    // at unobserved corners conflates fit identifiability with extrapolation. Full-image physical
    // checks still apply, and both observed and extrapolated uncertainty are exported separately.
    double r2 =
        (Math.pow(initial.width / 2., 2) + Math.pow(initial.height / 2., 2))
            / Math.exp(2 * values[0])
            * Math.pow(Math.min(1, coverage), 2);
    for (int k = 0; k < LENS; k++)
      if (active[k]) {
        if (!Double.isFinite(sigma[k]) || sigma[k] < 0) return false;
        double effect =
            k == 0 || k == 3 || k == 4
                ? sigma[k]
                : k == 7 ? sigma[k] * r2 * r2 * r2 : k == 2 ? sigma[k] * r2 * r2 : sigma[k] * r2;
        if (effect >= (k == 0 ? .08 : .02)) return false;
      }
    return true;
  }

  static boolean enabled(Options o) {
    return o.alignmentCalibration
        && o.selfCalibration
        && !o.rectified
        && (o.calibration == null || !o.intrinsicsMode.equals("fixed"));
  }

  static void refine(
      List<ImageFrame> frames,
      List<FeatureTracks.Track> tracks,
      int iterations,
      double huber,
      Project project,
      int anchor,
      int scaleCamera)
      throws IOException {
    if (frames.isEmpty()) return;
    Options o = frames.get(0).options;
    if (!enabled(o) || o.alignmentCalibrationDone) return;
    o.alignmentCalibrationDone = true;
    Path report = o.output.resolve("04_geometry/bundle/shared_calibration.json");
    Files.createDirectories(report.getParent());
    Calibration base = frames.get(0).calibration;
    for (ImageFrame f : frames)
      if (!same(base, f.calibration)) {
        Json.write(
            report,
            Json.object(
                "accepted",
                false,
                "reason",
                "Shared alignment refinement requires one calibration group; heterogeneous imported"
                    + " cameras retained"));
        return;
      }
    var split = CalibrationDiagnostics.split(frames, tracks);
    List<FeatureTracks.Track> train = split.train(), held = split.first(), second = split.second();
    Set<Integer> views = new HashSet<>();
    for (var t : train)
      for (var ob : t.observations) if (frames.get(ob.image).camera.registered) views.add(ob.image);
    if (views.size() < 4 || train.size() < 80 || held.size() < 20 || second.size() < 20) {
      Json.write(
          report,
          Json.object(
              "accepted",
              false,
              "reason",
              "Need four registered views, 80 training and two groups of 20 held-out multi-view"
                  + " tracks"));
      return;
    }
    SharedCalibration lens = new SharedCalibration(frames);
    // A repeatable track split; validation points may move, but never influence cameras/lens.
    Set<String> selected = new HashSet<>(Arrays.asList(o.refineIntrinsics.split(",")));
    List<Object> stages = new ArrayList<>();
    boolean acceptedAny = false;
    List<Double> radii = new ArrayList<>();
    double corner = Math.hypot(base.width / 2., base.height / 2.);
    for (var t : train)
      for (var ob : t.observations) {
        ImageFrame f = frames.get(ob.image);
        if (!f.camera.registered) continue;
        Point p = lens.observed[f.id][ob.feature];
        radii.add(
            Math.hypot(
                    (p.x - f.camera.cx) * base.width / f.width,
                    (p.y - f.camera.cy) * base.height / f.height)
                / corner);
      }
    Collections.sort(radii);
    double coverage = radii.isEmpty() ? 0 : radii.get((radii.size() - 1) * 9 / 10);
    if (coverage < .55) {
      Json.write(
          report,
          Json.object(
              "accepted",
              false,
              "reason",
              "Insufficient image-radius coverage",
              "radial_coverage_p90_fraction",
              coverage));
      return;
    }
    Path diagnostics = report.getParent().resolve("calibration_diagnostics");
    if (lens.options.diagnostics)
      CalibrationDiagnostics.residuals(diagnostics, "initial", lens, frames, train, held, second);
    double initialRms = lens.validationRms(frames, held);
    for (int stage = 0; stage < 5; stage++) {
      long stageStarted = System.nanoTime();
      boolean added = false;
      Arrays.fill(lens.active, false);
      for (int k = 0; k < SIZE; k++) {
        int group = group(k);
        if (group <= stage && selected.contains(NAMES[k])) lens.active[k] = true;
        if (group == stage && selected.contains(NAMES[k])) added = true;
      }
      if (!added) continue;
      Camera[] saved = new Camera[frames.size()];
      for (ImageFrame f : frames) saved[f.id] = f.camera.copy();
      double[][] points = new double[tracks.size()][];
      for (int i = 0; i < tracks.size(); i++) points[i] = tracks.get(i).point;
      double[] beforeLens = lens.values.clone();
      double before = lens.validationRms(frames, held);
      double secondBefore = lens.validationRms(frames, second);
      if (stage == 0 && lens.active[0])
        lens.focalStarts(
            frames, tracks, train, held, second, iterations, huber, anchor, scaleCamera);
      else
        BundleAdjuster.optimizeImpl(
            frames, train, Math.max(12, iterations), huber, null, anchor, scaleCamera, lens);
      double after = lens.validationRms(frames, held);
      double secondAfter = lens.validationRms(frames, second);
      if (lens.options.diagnostics)
        CalibrationDiagnostics.residuals(
            diagnostics, "stage-" + stage, lens, frames, train, held, second);
      boolean improvement =
          lens.physical()
              && Double.isFinite(after)
              && after < before * .995
              && before - after > .002
              && secondAfter < secondBefore * .995
              && secondBefore - secondAfter > .002;
      double[][] covariance =
          improvement
              ? BundleAdjuster.calibrationCovariance(
                  frames, train, huber, lens, Math.max(.35, Math.max(after, secondAfter)))
              : null;
      double[] uncertainty = covariance == null ? null : new double[SIZE];
      if (covariance != null)
        for (int k = 0; k < SIZE; k++) uncertainty[k] = Math.sqrt(covariance[k][k]);
      boolean constrained = lens.constrained(uncertainty, coverage);
      boolean accept = improvement && constrained;
      stages.add(
          Json.object(
              "parameters",
              lens.parameters(),
              "elapsed_seconds",
              (System.nanoTime() - stageStarted) / 1e9,
              "iteration_budget_per_trial",
              Math.max(12, iterations),
              "focal_start_scales",
              stage == 0 && lens.active[0] ? new double[] {1, .9, 1.1} : new double[] {1},
              "accepted",
              accept,
              "validation_rms_before_pixels",
              before,
              "validation_rms_after_pixels",
              after,
              "second_validation_rms_before_pixels",
              secondBefore,
              "second_validation_rms_after_pixels",
              secondAfter,
              "pixel_correction_uncertainty",
              CalibrationDiagnostics.correctionUncertainty(lens, covariance, coverage),
              "candidate",
              lens.calibrationValues(),
              "conditional_standard_deviations",
              uncertainty,
              "reason",
              accept
                  ? "Held-out improvement with constrained shared parameters"
                  : improvement
                      ? "Shared parameters poorly constrained after marginalizing poses and"
                          + " structure"
                      : "Retained preceding calibration; insufficient held-out improvement"));
      if (accept) acceptedAny = true;
      else {
        lens.values = beforeLens;
        for (ImageFrame f : frames) {
          f.camera.rotation = saved[f.id].rotation;
          f.camera.translation = saved[f.id].translation;
          f.camera.fixed = saved[f.id].fixed;
        }
        for (int i = 0; i < tracks.size(); i++) tracks.get(i).point = points[i];
        // Restore the preceding model; the next stage can retry earlier parameters jointly.
        for (int k = 0; k < SIZE; k++) if ((group(k)) == stage) lens.active[k] = false;
      }
    }
    if (acceptedAny) {
      // Finish held-out structure with the retained calibration, then update all pinhole
      // coordinates.
      lens.validationRms(frames, held, true);
      lens.validationRms(frames, second, true);
      Calibration result = lens.calibration();
      for (ImageFrame frame : frames) frame.applyCalibration(result);
      result.write(
          o.output.resolve("camera.txt"),
          "Staged shared alignment calibration from full-block tie points");
      if (!o.orientationCalibrations.isEmpty()) {
        Map<String, Calibration> updated = new LinkedHashMap<>();
        for (ImageFrame f : frames) updated.put(f.path.getFileName().toString(), result);
        o.orientationCalibrations = updated;
      }
      if (!lens.fixedPoses)
        BundleAdjuster.optimizeImpl(
            frames, tracks, iterations, huber, project, anchor, scaleCamera, null);
    }
    Json.write(
        report,
        Json.object(
            "accepted",
            acceptedAny,
            "model",
            "Shared Brown f,cx,cy,k1,k2,k3,p1,p2; fixed focal aspect ratio",
            "selected_parameters",
            o.refineIntrinsics,
            "parameter_order",
            NAMES,
            "solver_parameter_units",
            new String[] {
              "log_pixels",
              "dimensionless",
              "dimensionless",
              "fraction_of_source_width",
              "fraction_of_source_height",
              "dimensionless",
              "dimensionless",
              "dimensionless"
            },
            "uncertainty_scope",
            "Damped local conditional diagnostics; excludes model error",
            "uncertainty_acceptance_radius",
            "Training observation P90 radius; radial/tangential effects below 0.02, log-focal sigma"
                + " below 0.08, principal point fractions below 0.02. Uncovered image corners are"
                + " reported as extrapolation; physical map checks cover the full image.",
            "sampling",
            split.coverage(),
            "validation_policy",
            "Two disjoint spatially stratified holdouts; neither fits poses or intrinsics. Both"
                + " must improve by 0.5% and 0.002 processed pixels. Focal trials selected by"
                + " combined holdout RMS; these sets perform model selection and are not an"
                + " independent accuracy test.",
            "second_held_out_tracks",
            second.size(),
            "training_tracks",
            train.size(),
            "held_out_tracks",
            held.size(),
            "registered_views",
            views.size(),
            "radial_coverage_p90_fraction",
            coverage,
            "initial_validation_rms_pixels",
            initialRms,
            "calibration",
            lens.calibrationValues(),
            "stages",
            stages,
            "minimum_relative_held_out_improvement",
            .005,
            "minimum_pixel_held_out_improvement",
            .002,
            "principal_point_prior_sigma_fraction",
            .02));
    System.out.println(
        "Shared alignment calibration: "
            + (acceptedAny ? "accepted staged refinement" : "retained initial calibration")
            + "; "
            + report);
  }

  private void focalStarts(
      List<ImageFrame> frames,
      List<FeatureTracks.Track> tracks,
      List<FeatureTracks.Track> train,
      List<FeatureTracks.Track> first,
      List<FeatureTracks.Track> second,
      int iterations,
      double huber,
      int anchor,
      int scaleCamera)
      throws IOException {
    Camera[] original = frames.stream().map(f -> f.camera.copy()).toArray(Camera[]::new);
    double[][] originalPoints = tracks.stream().map(t -> t.point).toArray(double[][]::new);
    double[] originalLens = values.clone(), bestLens = values.clone();
    Camera[] best = original;
    double[][] bestPoints = originalPoints;
    double bestScore = Double.POSITIVE_INFINITY;
    for (double scale : new double[] {1, .9, 1.1}) {
      try (WorkProgress work =
          WorkProgress.start("Calibration focal start " + scale, 0, "trials")) {
        values = originalLens.clone();
        values[0] += Math.log(scale);
        restore(frames, tracks, original, originalPoints);
        BundleAdjuster.optimizeImpl(
            frames, train, Math.max(12, iterations), huber, null, anchor, scaleCamera, this);
        double score = validationRms(frames, first) + validationRms(frames, second);
        if (physical() && score < bestScore) {
          bestScore = score;
          bestLens = values.clone();
          best = frames.stream().map(f -> f.camera.copy()).toArray(Camera[]::new);
          bestPoints = tracks.stream().map(t -> t.point).toArray(double[][]::new);
        }
        work.done();
      }
    }
    values = bestLens;
    restore(frames, tracks, best, bestPoints);
  }

  private static void restore(
      List<ImageFrame> frames,
      List<FeatureTracks.Track> tracks,
      Camera[] cameras,
      double[][] points) {
    for (var f : frames) {
      f.camera.rotation = cameras[f.id].rotation.clone();
      f.camera.translation = cameras[f.id].translation.clone();
      f.camera.fixed = cameras[f.id].fixed;
    }
    for (int i = 0; i < tracks.size(); i++) tracks.get(i).point = points[i];
  }

  private static int group(int k) {
    return k == 0 ? 0 : k == 1 ? 1 : k == 2 ? 2 : k == 7 ? 3 : k >= 8 ? 99 : 4;
  }

  List<String> parameters() {
    List<String> names = new ArrayList<>();
    for (int k = 0; k < SIZE; k++) if (active[k]) names.add(NAMES[k]);
    return names;
  }

  private Object calibrationValues() {
    Calibration c = calibration();
    return Json.object(
        "fx", c.fx, "fy", c.fy, "cx", c.cx, "cy", c.cy, "k1", values[1], "k2", values[2], "p1",
        values[5], "p2", values[6], "k3", values[7], "rolling_shutter_readout_seconds",
        values[8]);
  }

  static boolean same(Calibration a, Calibration b) {
    return a.width == b.width
        && a.height == b.height
        && a.fx == b.fx
        && a.fy == b.fy
        && a.cx == b.cx
        && a.cy == b.cy
        && Arrays.equals(a.distortion, b.distortion);
  }

  double validationRms(List<ImageFrame> frames, List<FeatureTracks.Track> held) {
    return validationRms(frames, held, false);
  }

  double validationRms(List<ImageFrame> frames, List<FeatureTracks.Track> held, boolean commit) {
    double sum = 0;
    long n = 0;
    for (var t : held) {
      double[] p = t.point.clone();
      for (int iter = 0; iter < 25; iter++) {
        double[][] h = new double[3][3];
        double[] b = new double[3];
        for (var ob : t.observations) {
          ImageFrame frame = frames.get(ob.image);
          if (!frame.camera.registered) continue;
          Point measured = observed[frame.id][ob.feature];
          double[] uv = project(frame, p, measured.y);
          if (uv == null) return Double.POSITIVE_INFINITY;
          double[] r = {uv[0] - measured.x, uv[1] - measured.y};
          double weight = Math.min(1, 2 / Math.max(1e-12, Math.hypot(r[0], r[1])));
          double[][] jq = spatialJacobian(frame, p, measured.y), jp = new double[2][3];
          for (int row = 0; row < 2; row++)
            for (int j = 0; j < 3; j++)
              for (int k = 0; k < 3; k++)
                jp[row][j] += jq[row][k] * frame.camera.rotation[k * 3 + j];
          for (int j = 0; j < 3; j++) {
            for (int row = 0; row < 2; row++) b[j] -= weight * jp[row][j] * r[row];
            for (int k = 0; k < 3; k++)
              for (int row = 0; row < 2; row++) h[j][k] += weight * jp[row][j] * jp[row][k];
          }
        }
        for (int j = 0; j < 3; j++) h[j][j] += 1e-6 * Math.max(1, h[j][j]);
        double[] delta = Math3.solve(h, b);
        if (delta == null) break;
        double previous = pointCost(frames, t, p), factor = 1;
        boolean moved = false;
        while (factor >= 1. / 128) {
          double[] trial = Math3.add(p, Math3.times(delta, factor));
          if (pointCost(frames, t, trial) < previous) {
            p = trial;
            moved = true;
            break;
          }
          factor *= .5;
        }
        if (!moved || Math3.norm(delta) * factor < 1e-9) break;
      }
      if (commit) t.point = p;
      for (var ob : t.observations) {
        ImageFrame frame = frames.get(ob.image);
        if (!frame.camera.registered) continue;
        Point measured = observed[frame.id][ob.feature];
        double[] uv = project(frame, p, measured.y);
        if (uv == null) return Double.POSITIVE_INFINITY;
        sum += Math.pow(uv[0] - measured.x, 2) + Math.pow(uv[1] - measured.y, 2);
        n++;
      }
    }
    return Math.sqrt(sum / Math.max(1, n));
  }

  private double pointCost(List<ImageFrame> frames, FeatureTracks.Track t, double[] p) {
    double cost = 0;
    for (var ob : t.observations) {
      ImageFrame f = frames.get(ob.image);
      if (!f.camera.registered) continue;
      Point o = observed[f.id][ob.feature];
      double[] uv = project(f, p, o.y);
      if (uv == null) return Double.POSITIVE_INFINITY;
      double e = Math.hypot(uv[0] - o.x, uv[1] - o.y);
      cost += e <= 2 ? .5 * e * e : 2 * (e - 1);
    }
    return cost;
  }
}
