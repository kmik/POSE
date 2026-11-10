package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;

/**
 * GPS-weighted orientation adjustment after the GPS similarity fit (G1 and its extensions G3-G5).
 *
 * <p>The inlier GPS camera centres become weighted position priors in a final bundle adjustment in
 * the projected frame. Within that adjustment, by default:
 *
 * <ul>
 *   <li>the shared lens (f, k1, k2) is refitted, with the EXIF/XMP focal length as a prior. In a
 *       near-nadir block focal length and flying height are almost interchangeable; GPS altitudes
 *       fix the cameras but not the ground, so only the prior resolves that direction;
 *   <li>the GPS antenna model gps = C + V dt + R^T L is fitted: a GPS time offset dt (along-track
 *       residuals that change sign with flight direction) and a lever arm L in the camera frame;
 *   <li>a rolling-shutter readout time is fitted: row y is exposed at s = readout (y/h - 1/2)
 *       relative to the mid-frame row, while the camera moves with velocity V.
 * </ul>
 *
 * Velocities come from DJI XMP flight speeds (checked against the camera track) or from camera
 * centres and capture times. The GPS model and the bundle alternate; the model's block translation
 * is applied to the whole block, which leaves reprojection unchanged. A guard reverts to the
 * poses-only adjustment if the result is unphysical or fits the images or GPS clearly worse.
 */
final class GpsAdjustment {
  private GpsAdjustment() {}

  static final double TIME_SIGMA = 1, LEVER_SIGMA = .5;

  /** GPS antenna model: time offset (s) and lever arm (camera frame, m). */
  static final class Model {
    double dt;
    double[] lever = new double[3];
    boolean timeFree, leverFree;

    double priorCost() {
      double cost = timeFree ? .5 * Math.pow(dt / TIME_SIGMA, 2) : 0;
      if (leverFree) for (double l : lever) cost += .5 * Math.pow(l / LEVER_SIGMA, 2);
      return cost;
    }

    /** Where the GPS antenna sits for camera c: C + V dt + R^T L. */
    double[] antenna(Camera c) {
      double[] p = Math3.add(c.center(), Math3.mv(Math3.transpose(c.rotation), lever));
      if (c.velocity != null) p = Math3.add(p, Math3.times(c.velocity, dt));
      return p;
    }
  }

  record Outcome(double rms, String summary) {}

  private record State(
      Camera[] cameras, double[][] points, Calibration calibration, double[] lens, Model model) {}

  // Set by the GCP step: the GCPs fix the datum, so the block is never moved onto GPS; GPS gets an
  // offset instead (accumulated in gpsOffset). Single-threaded use only.
  static boolean datumFixed;
  static double[] gpsOffset = new double[3];

  static Outcome run(
      List<ImageFrame> frames,
      List<FeatureTracks.Track> tracks,
      Options o,
      Crs crs,
      Map<Integer, double[]> controls,
      Path dir,
      double similarityRms)
      throws IOException {
    if (datumFixed) {
      Map<Integer, double[]> copy = new LinkedHashMap<>();
      for (var e : controls.entrySet()) copy.put(e.getKey(), e.getValue().clone());
      controls = copy;
      gpsOffset = new double[3];
    }
    Map<String, Object> report = new LinkedHashMap<>();
    if (o.poses != null && o.extrinsicsMode.equals("fixed")) {
      report.put("status", "skipped");
      report.put("reason", "Supplied exterior orientations are fixed");
      Json.write(dir.resolve("gps_bundle.json"), report);
      return new Outcome(similarityRms, "");
    }
    String motion = velocities(frames, crs, report);
    int registered = 0, moving = 0;
    for (ImageFrame f : frames)
      if (f.camera.registered) {
        registered++;
        if (f.camera.velocity != null) moving++;
      }
    boolean hasMotion = moving >= Math.max(2, registered / 2);
    List<String> notes = new ArrayList<>();
    // DJI heights are barometer-aided: relative heights hold to decimetres while horizontal GPS
    // wanders by metres. A tight height prior keeps radial distortion from doming the block.
    int barometric = 0;
    for (ImageFrame f : frames)
      if (f.camera.registered && Double.isFinite(Exif.read(f.path).relativeAltitude)) barometric++;
    String heightSource;
    if (!o.gpsHeightSigma.equals("auto")) {
      o.gpsHeightSigmaValue = Double.parseDouble(o.gpsHeightSigma);
      heightSource = "user";
    } else if (barometric >= Math.max(2, registered / 2)) {
      o.gpsHeightSigmaValue = Math.min(.2, o.gpsSigma);
      heightSource = "dji_barometric_relative_altitude";
    } else {
      o.gpsHeightSigmaValue = o.gpsSigma;
      heightSource = "same_as_horizontal";
    }
    report.put("height_sigma_metres", o.gpsHeightSigmaValue);
    report.put("height_sigma_source", heightSource);

    // Shared parameters: lens (G3) and rolling shutter (G5) in one shared block.
    Set<Integer> lensParameters = new TreeSet<>();
    if (!o.gpsIntrinsics.equals("none"))
      for (String name : o.gpsIntrinsics.split(","))
        lensParameters.add(Arrays.asList(SharedCalibration.NAMES).indexOf(name.trim()));
    String lensReason = null;
    if (o.rectified) lensReason = "rectified input";
    else if (o.calibration != null && o.intrinsicsMode.equals("fixed"))
      lensReason = "supplied calibration is fixed (--intrinsics-mode fixed)";
    else
      for (ImageFrame f : frames)
        if (!SharedCalibration.same(frames.get(0).calibration, f.calibration))
          lensReason = "images do not share one calibration";
    if (lensReason != null && !lensParameters.isEmpty()) {
      notes.add("lens refit skipped: " + lensReason);
      lensParameters.clear();
    }
    boolean shutterFree = o.rollingShutter.equals("estimate") && hasMotion;
    double shutterFixed =
        o.rollingShutter.equals("estimate") || o.rollingShutter.equals("off")
            ? 0
            : Double.parseDouble(o.rollingShutter);
    if (!hasMotion && !o.rollingShutter.equals("off"))
      notes.add("rolling shutter not modelled: no camera velocities");
    if (lensReason != null && (shutterFree || shutterFixed != 0)) {
      notes.add("rolling shutter not modelled: " + lensReason);
      shutterFree = false;
      shutterFixed = 0;
    }
    if (!hasMotion) shutterFixed = 0;

    Model model = new Model();
    model.timeFree = o.gpsTimeOffset.equals("estimate") && hasMotion;
    if (!o.gpsTimeOffset.equals("estimate") && !o.gpsTimeOffset.equals("off"))
      model.dt = hasMotion ? Double.parseDouble(o.gpsTimeOffset) : 0;
    if (!hasMotion && !o.gpsTimeOffset.equals("off"))
      notes.add("GPS time offset not modelled: no camera velocities");
    model.leverFree = o.gpsLeverArm.equals("estimate");
    if (!o.gpsLeverArm.equals("estimate") && !o.gpsLeverArm.equals("off")) {
      String[] parts = o.gpsLeverArm.split(",");
      for (int k = 0; k < 3; k++) model.lever[k] = Double.parseDouble(parts[k].trim());
    }
    if (lensReason != null && (model.timeFree || model.leverFree || model.dt != 0)) {
      // The antenna model lives in the shared block, which needs one shared calibration.
      notes.add("GPS time offset and lever arm not modelled: " + lensReason);
      model = new Model();
    }
    boolean shared =
        !lensParameters.isEmpty()
            || shutterFree
            || shutterFixed != 0
            || model.timeFree
            || model.leverFree
            || model.dt != 0
            || Math3.norm(model.lever) != 0;

    double focalPrior = Double.NaN;
    String focalSource = "none";
    if (lensParameters.contains(0) && o.calibration == null) {
      double[] prior = focalPrior(frames, o);
      focalPrior = prior[0];
      focalSource = prior[0] > 0 ? (prior[1] == 1 ? "xmp_calibrated_focal" : prior[1] == 2 ? "exif_35mm_equivalent" : "user") : "none";
      if (!(focalPrior > 0)) focalPrior = Double.NaN;
    }

    int anchor = -1, scale = -1;
    for (ImageFrame f : frames)
      if (f.camera.registered && controls.containsKey(f.id)) {
        if (anchor < 0) anchor = f.id;
        else if (scale < 0
            && Math3.norm(Math3.sub(f.camera.center(), frames.get(anchor).camera.center()))
                > 1e-6) scale = f.id;
      }
    if (anchor < 0 || scale < 0) {
      report.put("status", "skipped");
      report.put("reason", "Fewer than two distinct GPS camera controls");
      Json.write(dir.resolve("gps_bundle.json"), report);
      return new Outcome(similarityRms, "");
    }

    State start = save(frames, tracks, null, model);
    double rmsBefore = reprojectionRms(frames, tracks, null);
    double currentFocal = frames.get(0).calibration.fx;
    // Focal starts: the current lens first; the prior focal only if that fit ends far from the
    // prior (the valley along focal, distortion and flying height is flat and slow to cross).
    List<Double> starts = new ArrayList<>(List.of(1.));
    List<Object> trials = new ArrayList<>();
    State best = null;
    SharedCalibration bestLens = null;
    double bestObjective = Double.POSITIVE_INFINITY;
    for (int trial = 0; trial < starts.size(); trial++) {
      double factor = starts.get(trial);
      restore(frames, tracks, start, null);
      if (factor != 1) rescaleFocal(frames, tracks, factor);
      Model m = copy(start.model());
      SharedCalibration lens = null;
      if (shared) {
        lens = new SharedCalibration(frames);
        for (int k : lensParameters) lens.active[k] = true;
        lens.active[SharedCalibration.READOUT] = shutterFree;
        lens.values[SharedCalibration.READOUT] = shutterFixed;
        lens.active[SharedCalibration.TIME] = m.timeFree;
        for (int k = 0; k < 3; k++) lens.active[SharedCalibration.LEVER + k] = m.leverFree;
        toLens(m, lens);
        if (Double.isFinite(focalPrior)) {
          lens.focalPrior = Math.log(focalPrior);
          lens.focalPriorSigma = o.gpsFocalSigma;
        }
      }
      double objective;
      try (WorkProgress work =
          WorkProgress.start(
              String.format(Locale.ROOT, "GPS-weighted adjustment (focal start %.0f px)", currentFocal * factor),
              o.gpsAdjustmentRounds,
              "rounds")) {
        objective = rounds(frames, tracks, o, controls, lens, m, anchor, scale, work);
      }
      boolean physical = lens == null || lens.physical();
      trials.add(
          Json.object(
              "start_focal_pixels", currentFocal * factor,
              "objective", objective,
              "physical", physical,
              "focal_pixels", lens == null ? currentFocal : Math.exp(lens.values[0]),
              "time_offset_seconds", m.dt,
              "lever_arm_camera_metres", m.lever,
              "rolling_shutter_readout_seconds", lens == null ? 0 : lens.values[8]));
      if (physical && objective < bestObjective) {
        bestObjective = objective;
        best = save(frames, tracks, lens, m);
        bestLens = lens;
      }
      // --quick-calibration: keep the first (current-lens) start only.
      if (trial == 0
          && !o.quickCalibration
          && lens != null
          && lens.active[0]
          && Double.isFinite(focalPrior)
          && Math.abs(lens.values[0] - Math.log(focalPrior)) > 2 * o.gpsFocalSigma)
        starts.add(focalPrior / currentFocal);
      else if (trial == 0
          && o.quickCalibration
          && lens != null
          && lens.active[0]
          && Double.isFinite(focalPrior)
          && Math.abs(lens.values[0] - Math.log(focalPrior)) > 2 * o.gpsFocalSigma)
        notes.add("--quick-calibration: second focal start (at the EXIF/XMP prior) skipped");
    }
    boolean accepted = best != null;
    double rmsAfter = Double.NaN, gpsAfter = Double.NaN;
    if (accepted) {
      restore(frames, tracks, best, bestLens);
      model = best.model();
      rmsAfter = reprojectionRms(frames, tracks, bestLens);
      gpsAfter = gpsRms(frames, controls, model);
      // Sanity bounds only: with consumer GPS (sigma 2 m) the images dominate, so GPS residuals
      // may legitimately grow a little while the block converges; blow-ups are rejected.
      accepted = rmsAfter <= rmsBefore * 1.15 + .05 && gpsAfter <= similarityRms * 1.5 + .5;
    }
    if (!accepted) {
      // Fall back to the plain G1 adjustment: poses only, no antenna model.
      notes.add(
          String.format(
              Locale.ROOT,
              "extended adjustment rejected (reprojection RMS %.3f -> %.3f px, GPS RMS %.3f -> %.3f"
                  + " m); poses-only GPS adjustment applied",
              rmsBefore,
              rmsAfter,
              similarityRms,
              gpsAfter));
      restore(frames, tracks, start, null);
      model = new Model();
      bestLens = null;
      try (WorkProgress work = WorkProgress.start("GPS-weighted adjustment (poses only)", 1, "rounds")) {
        rounds(frames, tracks, o, controls, null, model, anchor, scale, work);
      }
      rmsAfter = reprojectionRms(frames, tracks, null);
      gpsAfter = gpsRms(frames, controls, model);
    }
    Calibration before = frames.get(0).calibration, result = before;
    double focalBefore = start.calibration().fx;
    if (bestLens != null) {
      result = bestLens.calibration();
      for (ImageFrame f : frames) f.applyCalibration(result);
    }
    double readout = bestLens == null ? 0 : bestLens.values[8];
    o.rollingShutterReadout = readout;

    List<Object> cameras = new ArrayList<>();
    double[][] split = new double[2][3];
    int[] counts = new int[2];
    for (var e : controls.entrySet()) {
      Camera c = frames.get(e.getKey()).camera;
      double[] raw = Math3.sub(c.center(), e.getValue()),
          modelled = Math3.sub(model.antenna(c), e.getValue());
      double[] along = trackFrame(c, raw), alongModel = trackFrame(c, modelled);
      if (along != null) {
        for (int k = 0; k < 3; k++) {
          split[0][k] += along[k];
          split[1][k] += alongModel[k];
        }
        counts[0]++;
      }
      cameras.add(
          Json.object(
              "image_id", e.getKey(),
              "residual_metres", Math3.norm(modelled),
              "residual_enu", modelled,
              "camera_centre_minus_gps_enu", raw,
              "along_cross_up_metres", alongModel,
              "velocity_enu_metres_per_second", c.velocity));
    }
    for (int k = 0; k < 3; k++) {
      split[0][k] /= Math.max(1, counts[0]);
      split[1][k] /= Math.max(1, counts[0]);
    }
    report.put("status", "applied");
    report.put("extended_model_accepted", accepted);
    report.put("position_sigma_metres", o.gpsSigma);
    report.put("controls", controls.size());
    report.put("similarity_rms_metres", similarityRms);
    report.put("bundle_rms_metres", gpsAfter);
    report.put("camera_centre_rms_metres", gpsRms(frames, controls, new Model()));
    double[] axes = new double[3];
    for (var e : controls.entrySet()) {
      double[] r = Math3.sub(model.antenna(frames.get(e.getKey()).camera), e.getValue());
      for (int k = 0; k < 3; k++) axes[k] += r[k] * r[k] / Math.max(1, controls.size());
    }
    report.put("bundle_horizontal_rms_metres", Math.sqrt(axes[0] + axes[1]));
    report.put("bundle_height_rms_metres", Math.sqrt(axes[2]));
    report.put("reprojection_rms_before_pixels", rmsBefore);
    report.put("reprojection_rms_after_pixels", rmsAfter);
    report.put("velocity_source", motion);
    report.put("cameras_with_velocity", moving);
    report.put(
        "intrinsics",
        Json.object(
            "refitted",
            bestLens == null
                ? List.of()
                : bestLens.parameters().stream()
                    .filter(p -> Arrays.asList(SharedCalibration.NAMES).indexOf(p) < SharedCalibration.LENS)
                    .toList(),
            "shared_block_parameters",
            bestLens == null ? List.of() : bestLens.parameters(),
            "focal_before_pixels", focalBefore,
            "focal_after_pixels", result.fx,
            "focal_prior_pixels", Double.isFinite(focalPrior) ? focalPrior : null,
            "focal_prior_source", focalSource,
            "focal_prior_sigma_log", o.gpsFocalSigma,
            "distortion_after", result.distortion,
            "note",
            "Near-nadir blocks barely constrain focal length against flying height; the prior"
                + " decides that direction. A focal change moves the ground by about"
                + " (flying height) x (relative focal change)."));
    report.put(
        "gps_model",
        Json.object(
            "time_offset_seconds", model.dt,
            "time_offset_mode", o.gpsTimeOffset,
            "lever_arm_camera_metres", model.lever,
            "lever_arm_mode", o.gpsLeverArm,
            "time_sigma_seconds", TIME_SIGMA,
            "lever_sigma_metres", LEVER_SIGMA,
            "mean_along_cross_up_before_metres", split[0],
            "mean_along_cross_up_after_metres", split[1],
            "note",
            "gps = C + V dt + R^T L. Along-track is the camera's own direction of travel, so a"
                + " timing offset shows as a nonzero mean along-track residual before the fit."));
    report.put(
        "rolling_shutter",
        Json.object(
            "mode", o.rollingShutter,
            "readout_seconds", readout,
            "model",
            "row y exposed at readout*(y/height-1/2) from mid-frame; translation only;"
                + " dense matching uses mid-exposure poses"));
    report.put("trials", trials);
    report.put("notes", notes);
    report.put("cameras", cameras);
    Json.write(dir.resolve("gps_bundle.json"), report);
    String summary =
        String.format(
            Locale.ROOT,
            "GPS-weighted adjustment: GPS RMS %.3f -> %.3f m, reprojection %.3f -> %.3f px; f %.1f"
                + " -> %.1f px (prior %s), GPS time offset %.3f s, lever arm [%.2f, %.2f, %.2f] m,"
                + " rolling-shutter readout %.4f s%s",
            similarityRms,
            gpsAfter,
            rmsBefore,
            rmsAfter,
            focalBefore,
            result.fx,
            Double.isFinite(focalPrior) ? String.format(Locale.ROOT, "%.1f", focalPrior) : "none",
            model.dt,
            model.lever[0],
            model.lever[1],
            model.lever[2],
            readout,
            notes.isEmpty() ? "" : "; " + String.join("; ", notes));
    System.out.println(summary);
    return new Outcome(gpsAfter, summary);
  }

  /** Alternates the GPS antenna model and the prior-weighted bundle; returns the full objective. */
  private static double rounds(
      List<ImageFrame> frames,
      List<FeatureTracks.Track> tracks,
      Options o,
      Map<Integer, double[]> controls,
      SharedCalibration lens,
      Model model,
      int anchor,
      int scale,
      WorkProgress work)
      throws IOException {
    int rounds = lens == null && !model.timeFree && !model.leverFree ? 1 : o.gpsAdjustmentRounds;
    try {
      for (int round = 0; round < rounds; round++) {
        double dt = model.dt;
        if (!datumFixed) alignBlock(frames, tracks, controls, model);
        // With the cameras held, the linear antenna fit is exact; it starts (or, without a shared
        // block, replaces) the joint estimate in the bundle.
        double[] lever = model.lever.clone(), shift = fitModel(frames, tracks, controls, model, o);
        if (lens != null) toLens(model, lens);
        references(frames, controls, model, lens, o);
        BundleAdjuster.optimizeImpl(
            frames,
            tracks,
            round == 0 && lens != null ? Math.max(o.iterations, 20) : o.iterations,
            o.reprojection,
            null,
            anchor,
            scale,
            lens);
        if (lens != null) fromLens(lens, model);
        if (work != null) work.update(round + 1);
        if (round > 0
            && Math.abs(model.dt - dt) < 1e-3
            && Math3.norm(Math3.sub(model.lever, lever)) < 1e-3
            && Math3.norm(shift) < 1e-3) break;
      }
      if (!datumFixed) alignBlock(frames, tracks, controls, model);
      if (lens == null) {
        fitModel(frames, tracks, controls, model, o);
        references(frames, controls, model, null, o);
        return BundleAdjuster.objective(frames, tracks, o.reprojection, null) + model.priorCost();
      }
      // The shared block carries the antenna model and its priors.
      references(frames, controls, model, lens, o);
      return BundleAdjuster.objective(frames, tracks, o.reprojection, lens);
    } finally {
      for (ImageFrame f : frames) {
        f.camera.reference = null;
        f.camera.positionSigma = 10;
        f.camera.heightSigma = Double.NaN;
        f.camera.rotationSigma = Math.toRadians(10);
      }
    }
  }

  /**
   * Least squares for the block translation tau, time offset and lever arm with the cameras held:
   * gps_i = C_i + tau + V_i dt + R_i^T L. Applies tau to cameras and points (reprojection
   * unchanged); returns tau.
   */
  static double[] fitModel(
      List<ImageFrame> frames,
      List<FeatureTracks.Track> tracks,
      Map<Integer, double[]> controls,
      Model model,
      Options o) {
    double[][] a = new double[7][7];
    double[] b = new double[7], sigma = {o.gpsSigma, o.gpsSigma, o.gpsHeightSigmaValue};
    for (var e : controls.entrySet()) {
      Camera c = frames.get(e.getKey()).camera;
      if (!c.registered) continue;
      double[] residual = Math3.sub(e.getValue(), c.center());
      double[] rt = Math3.transpose(c.rotation);
      for (int axis = 0; axis < 3; axis++) {
        double[] row = new double[7];
        row[axis] = 1;
        row[3] = c.velocity == null ? 0 : c.velocity[axis];
        for (int k = 0; k < 3; k++) row[4 + k] = rt[axis * 3 + k];
        for (int i = 0; i < 7; i++) {
          b[i] += row[i] * residual[axis] / (sigma[axis] * sigma[axis]);
          for (int j = 0; j < 7; j++) a[i][j] += row[i] * row[j] / (sigma[axis] * sigma[axis]);
        }
      }
    }
    double big = 1e12;
    if (model.timeFree) a[3][3] += 1 / (TIME_SIGMA * TIME_SIGMA);
    else {
      a[3][3] += big;
      b[3] += big * model.dt;
    }
    for (int k = 0; k < 3; k++)
      if (model.leverFree) a[4 + k][4 + k] += 1 / (LEVER_SIGMA * LEVER_SIGMA);
      else {
        a[4 + k][4 + k] += big;
        b[4 + k] += big * model.lever[k];
      }
    double[] x = Math3.solve(a, b);
    if (x == null) return new double[3];
    // Bounded: a runaway estimate means the motion data cannot support the model.
    if (model.timeFree) model.dt = Math.max(-2, Math.min(2, x[3]));
    if (model.leverFree)
      for (int k = 0; k < 3; k++) model.lever[k] = Math.max(-2, Math.min(2, x[4 + k]));
    double[] shift = {x[0], x[1], x[2]};
    if (datumFixed) {
      // GCP datum: the GPS fixes take the offset, the block stays.
      for (double[] v : controls.values()) for (int k = 0; k < 3; k++) v[k] -= shift[k];
      for (int k = 0; k < 3; k++) gpsOffset[k] += shift[k];
      return shift;
    }
    for (ImageFrame f : frames) {
      Camera c = f.camera;
      c.translation = Math3.sub(c.translation, Math3.mv(c.rotation, shift));
    }
    shiftPoints(tracks, shift);
    return shift;
  }

  /**
   * Exact step along the seven directions the images cannot see: fit the block's similarity
   * (scale, rotation, translation) to the GPS-implied camera centres and apply it to cameras and
   * points. The bundle alone crawls along these directions: with a free lens, scale couples to the
   * flat focal/height valley and the GPS priors are weak next to 300k image residuals.
   */
  static void alignBlock(
      List<ImageFrame> frames,
      List<FeatureTracks.Track> tracks,
      Map<Integer, double[]> controls,
      Model model) {
    List<double[]> source = new ArrayList<>(), target = new ArrayList<>();
    for (var e : controls.entrySet()) {
      Camera c = frames.get(e.getKey()).camera;
      if (!c.registered) continue;
      source.add(c.center());
      target.add(Math3.sub(e.getValue(), Math3.sub(model.antenna(c), c.center())));
    }
    if (source.size() < 3) return;
    Georeference.Similarity s;
    try {
      s = Georeference.fit(source, target);
    } catch (IllegalArgumentException e) {
      return;
    }
    if (!(Math.abs(Math.log(s.scale)) < .2)) return;
    for (ImageFrame f : frames) {
      Camera c = f.camera;
      c.rotation = Math3.mul(c.rotation, Math3.transpose(s.rotation));
      c.translation =
          Math3.sub(Math3.times(c.translation, s.scale), Math3.mv(c.rotation, s.translation));
    }
    for (FeatureTracks.Track t : tracks) if (t.point != null) t.point = s.apply(t.point);
  }

  private static void shiftPoints(List<FeatureTracks.Track> tracks, double[] shift) {
    for (FeatureTracks.Track t : tracks) if (t.point != null) t.point = Math3.add(t.point, shift);
  }

  static void toLens(Model m, SharedCalibration lens) {
    lens.values[SharedCalibration.TIME] = m.dt;
    for (int k = 0; k < 3; k++) lens.values[SharedCalibration.LEVER + k] = m.lever[k];
  }

  static void fromLens(SharedCalibration lens, Model m) {
    m.dt = lens.values[SharedCalibration.TIME];
    for (int k = 0; k < 3; k++) m.lever[k] = lens.values[SharedCalibration.LEVER + k];
  }

  /**
   * Camera position priors. With the antenna model in the shared block the prior compares the
   * modelled antenna to the raw GPS fix; otherwise it targets the centre that would put the
   * (fixed) antenna on the fix.
   */
  private static void references(
      List<ImageFrame> frames,
      Map<Integer, double[]> controls,
      Model model,
      SharedCalibration lens,
      Options o) {
    boolean antenna = lens != null && lens.antennaModel();
    for (var e : controls.entrySet()) {
      Camera c = frames.get(e.getKey()).camera;
      if (!c.registered) continue;
      double[] centre =
          antenna
              ? e.getValue()
              : Math3.sub(e.getValue(), Math3.sub(model.antenna(c), c.center()));
      Camera reference = c.copy();
      reference.reference = null;
      reference.translation = Math3.times(Math3.mv(reference.rotation, centre), -1);
      c.reference = reference;
      c.positionSigma = o.gpsSigma;
      c.heightSigma = o.gpsHeightSigmaValue;
      c.rotationSigma = Math.PI;
    }
  }

  static double gpsRms(List<ImageFrame> frames, Map<Integer, double[]> controls, Model model) {
    double square = 0;
    for (var e : controls.entrySet())
      square +=
          Math.pow(
              Math3.norm(Math3.sub(model.antenna(frames.get(e.getKey()).camera), e.getValue())), 2);
    return Math.sqrt(square / Math.max(1, controls.size()));
  }

  /** RMS reprojection error (processed pixels) of all triangulated registered observations. */
  static double reprojectionRms(
      List<ImageFrame> frames, List<FeatureTracks.Track> tracks, SharedCalibration lens) {
    double sum = 0;
    long n = 0;
    for (FeatureTracks.Track t : tracks)
      if (t.point != null)
        for (FeatureTracks.Observation ob : t.observations) {
          ImageFrame f = frames.get(ob.image);
          if (!f.camera.registered) continue;
          org.opencv.core.Point p =
              lens == null ? f.keypoints[ob.feature].pt : lens.observed[f.id][ob.feature];
          double[] uv = lens == null ? f.camera.project(t.point) : lens.project(f, t.point, p.y);
          if (uv == null) continue;
          sum += Math.pow(uv[0] - p.x, 2) + Math.pow(uv[1] - p.y, 2);
          n++;
        }
    return Math.sqrt(sum / Math.max(1, n));
  }

  /** Residual in the camera's horizontal track frame: along-track, cross-track (left), up. */
  private static double[] trackFrame(Camera c, double[] r) {
    if (c.velocity == null) return null;
    double vx = c.velocity[0], vy = c.velocity[1], v = Math.hypot(vx, vy);
    if (v < .5) return null;
    vx /= v;
    vy /= v;
    return new double[] {r[0] * vx + r[1] * vy, -r[0] * vy + r[1] * vx, r[2]};
  }

  /**
   * Focal prior in source pixels and its source (1 = DJI XMP calibrated focal, 2 = EXIF 35 mm
   * equivalent, 3 = --gps-focal-prior value), or {NaN, 0}. XMP values refer to the original sensor
   * width, so they are used only when they agree with the resolution-independent 35 mm value.
   */
  static double[] focalPrior(List<ImageFrame> frames, Options o) throws IOException {
    if (o.gpsFocalPrior.equals("none")) return new double[] {Double.NaN, 0};
    if (!o.gpsFocalPrior.equals("auto")) return new double[] {Double.parseDouble(o.gpsFocalPrior), 3};
    Calibration c = frames.get(0).calibration;
    List<Double> xmp = new ArrayList<>(), equivalent = new ArrayList<>();
    for (ImageFrame f : frames) {
      Exif.Metadata m = Exif.read(f.path);
      if (m.calibratedFocal > 0) xmp.add(m.calibratedFocal);
      if (m.focal35Mm > 0) equivalent.add(m.focal35Mm / Math.hypot(36, 24) * Math.hypot(c.width, c.height));
    }
    double fromXmp = median(xmp), from35 = median(equivalent);
    if (fromXmp > 0 && (!(from35 > 0) || Math.abs(Math.log(fromXmp / from35)) < .1))
      return new double[] {fromXmp, 1};
    if (from35 > 0) return new double[] {from35, 2};
    return new double[] {Double.NaN, 0};
  }

  private static double median(List<Double> values) {
    if (values.isEmpty()) return Double.NaN;
    Collections.sort(values);
    return values.get(values.size() / 2);
  }

  /**
   * World velocities (m/s) of the registered cameras, stored in Camera.velocity. Prefers DJI XMP
   * flight speeds (north, east, down), accepted only if they point along the camera track; falls
   * back to camera-centre differences over (smoothed) capture times. Returns the source.
   */
  static String velocities(List<ImageFrame> frames, Crs crs, Map<String, Object> report)
      throws IOException {
    List<ImageFrame> order = new ArrayList<>();
    Map<Integer, Exif.Metadata> metadata = new HashMap<>();
    Map<Integer, double[]> supplied = new HashMap<>();
    for (ImageFrame f : frames) {
      if (f.camera.velocity != null) supplied.put(f.id, f.camera.velocity);
      f.camera.velocity = null;
      if (!f.camera.registered) continue;
      order.add(f);
      metadata.put(f.id, Exif.read(f.path));
    }
    Comparator<ImageFrame> byTime =
        Comparator.comparingDouble(
            (ImageFrame f) -> {
              double t = metadata.get(f.id).captureSeconds();
              return Double.isFinite(t) ? t : Double.MAX_VALUE;
            });
    order.sort(byTime.thenComparing(f -> f.path.getFileName().toString()));
    // DJI XMP: FlightXSpeed north, FlightYSpeed east, FlightZSpeed down (m/s).
    Map<Integer, double[]> xmp = new HashMap<>();
    for (ImageFrame f : order) {
      Exif.Metadata m = metadata.get(f.id);
      if (crs == null || f.gps == null || !f.gps.usable() || !Double.isFinite(m.speedNorth + m.speedEast))
        continue;
      double[] here = crs.project(f.gps);
      double[] north =
          unit(Math3.sub(crs.project(new Exif.Gps(f.gps.latitude + 1e-4, f.gps.longitude, f.gps.altitude, f.gps.datum)), here));
      double[] east =
          unit(Math3.sub(crs.project(new Exif.Gps(f.gps.latitude, f.gps.longitude + 1e-4, f.gps.altitude, f.gps.datum)), here));
      double down = Double.isFinite(m.speedDown) ? m.speedDown : 0;
      xmp.put(
          f.id,
          new double[] {
            m.speedNorth * north[0] + m.speedEast * east[0],
            m.speedNorth * north[1] + m.speedEast * east[1],
            -down
          });
    }
    if (xmp.size() >= Math.max(2, order.size() / 2)) {
      List<Double> cosines = new ArrayList<>();
      for (int i = 0; i + 1 < order.size(); i++) {
        double[] a = xmp.get(order.get(i).id), b = xmp.get(order.get(i + 1).id);
        double[] d = Math3.sub(order.get(i + 1).camera.center(), order.get(i).camera.center());
        if (a == null || b == null) continue;
        double[] v = Math3.add(a, b);
        if (Math3.norm(d) > 1 && Math3.norm(v) > 1)
          cosines.add((d[0] * v[0] + d[1] * v[1] + d[2] * v[2]) / Math3.norm(d) / Math3.norm(v));
      }
      double agreement = median(cosines);
      report.put("xmp_velocity_track_cosine_median", agreement);
      if (agreement > .8) {
        for (ImageFrame f : order) f.camera.velocity = xmp.get(f.id);
        return "dji_xmp_flight_speed";
      }
    }
    // Capture times: whole seconds are common, so fit a local line through neighbouring times.
    List<ImageFrame> timed = new ArrayList<>();
    for (ImageFrame f : order) if (Double.isFinite(metadata.get(f.id).captureSeconds())) timed.add(f);
    if (timed.size() < Math.max(3, order.size() / 2)) return supplied(frames, supplied);
    double[] t = new double[timed.size()];
    for (int i = 0; i < t.length; i++) t[i] = metadata.get(timed.get(i).id).captureSeconds();
    List<Double> gaps = new ArrayList<>();
    for (int i = 1; i < t.length; i++) if (t[i] > t[i - 1]) gaps.add(t[i] - t[i - 1]);
    double interval = median(gaps);
    if (!(interval > 0)) return supplied(frames, supplied);
    int[] segment = new int[t.length];
    for (int i = 1; i < t.length; i++)
      segment[i] = segment[i - 1] + (t[i] - t[i - 1] > 3 * interval + 1 ? 1 : 0);
    double[] smooth = new double[t.length];
    for (int i = 0; i < t.length; i++) {
      double sx = 0, sy = 0, sxx = 0, sxy = 0;
      int n = 0;
      for (int j = Math.max(0, i - 4); j <= Math.min(t.length - 1, i + 4); j++)
        if (segment[j] == segment[i]) {
          sx += j;
          sy += t[j];
          sxx += (double) j * j;
          sxy += j * t[j];
          n++;
        }
      double slope = n > 1 && n * sxx - sx * sx > 0 ? (n * sxy - sx * sy) / (n * sxx - sx * sx) : 0;
      smooth[i] = (sy - slope * sx) / n + slope * i;
    }
    int count = 0;
    for (int i = 0; i < t.length; i++) {
      int a = i > 0 && segment[i - 1] == segment[i] ? i - 1 : i;
      int b = i + 1 < t.length && segment[i + 1] == segment[i] ? i + 1 : i;
      if (a == b || smooth[b] - smooth[a] < .05) continue;
      timed.get(i).camera.velocity =
          Math3.times(
              Math3.sub(timed.get(b).camera.center(), timed.get(a).camera.center()),
              1 / (smooth[b] - smooth[a]));
      count++;
    }
    report.put("capture_interval_seconds", interval);
    return count > 0 ? "camera_centres_and_capture_times" : supplied(frames, supplied);
  }

  /** Without metadata, keeps velocities the caller already attached to the cameras. */
  private static String supplied(List<ImageFrame> frames, Map<Integer, double[]> supplied) {
    for (ImageFrame f : frames) f.camera.velocity = supplied.get(f.id);
    return supplied.isEmpty() ? "none" : "supplied";
  }

  private static double[] unit(double[] v) {
    return Math3.times(v, 1 / Math3.norm(v));
  }

  /**
   * Re-expresses the structure for focal length f*k at fixed camera centres: each point keeps its
   * lateral offset in an observing camera while its depth scales by k (averaged over observers).
   */
  static void rescaleFocal(List<ImageFrame> frames, List<FeatureTracks.Track> tracks, double k) {
    Calibration c = frames.get(0).calibration;
    Calibration next =
        new Calibration(c.fx * k, c.fy * k, c.cx, c.cy, c.distortion.clone(), c.width, c.height);
    for (FeatureTracks.Track t : tracks) {
      if (t.point == null) continue;
      double[] sum = new double[3];
      int n = 0;
      for (FeatureTracks.Observation ob : t.observations) {
        Camera cam = frames.get(ob.image).camera;
        if (!cam.registered) continue;
        double[] q = cam.view(t.point);
        if (!(q[2] > 0)) continue;
        q[2] *= k;
        sum = Math3.add(sum, Math3.mv(Math3.transpose(cam.rotation), Math3.sub(q, cam.translation)));
        n++;
      }
      if (n > 0) t.point = Math3.times(sum, 1. / n);
    }
    for (ImageFrame f : frames) f.applyCalibration(next);
  }

  private static Model copy(Model m) {
    Model out = new Model();
    out.dt = m.dt;
    out.lever = m.lever.clone();
    out.timeFree = m.timeFree;
    out.leverFree = m.leverFree;
    return out;
  }

  private static State save(
      List<ImageFrame> frames, List<FeatureTracks.Track> tracks, SharedCalibration lens, Model m) {
    Camera[] cameras = new Camera[frames.size()];
    for (ImageFrame f : frames) cameras[f.id] = f.camera.copy();
    double[][] points = new double[tracks.size()][];
    for (int i = 0; i < points.length; i++) points[i] = tracks.get(i).point;
    return new State(
        cameras,
        points,
        frames.get(0).calibration,
        lens == null ? null : lens.values.clone(),
        copy(m));
  }

  private static void restore(
      List<ImageFrame> frames, List<FeatureTracks.Track> tracks, State s, SharedCalibration lens) {
    for (ImageFrame f : frames) f.applyCalibration(s.calibration());
    for (ImageFrame f : frames) {
      Camera c = s.cameras()[f.id];
      f.camera.rotation = c.rotation.clone();
      f.camera.translation = c.translation.clone();
      f.camera.fixed = c.fixed;
    }
    for (int i = 0; i < tracks.size(); i++) tracks.get(i).point = s.points()[i];
    if (lens != null && s.lens() != null) lens.values = s.lens().clone();
  }
}
