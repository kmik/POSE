package sgm;

import java.util.*;
import java.util.function.Predicate;

/**
 * {@code --registration-pose-check on}: approves cameras solved by PnP during incremental
 * registration. Over flat ground with few, clustered tie points, PnP can fit a camera tilted by
 * tens of degrees and shifted sideways by about height x tan(tilt) while still looking at the same
 * patch; its reprojection error stays low and bundle adjustment keeps it. Two checks catch it:
 *
 * <ul>
 *   <li>GPS: a similarity fit of the registered camera centres to GPS (the same robust fit as
 *       {@link RegistrationRecovery#toGps}) predicts where the camera is; it is rejected when it is
 *       more than max({@link RegistrationRecovery#GPS_FACTOR} x fit RMS, {@link
 *       RegistrationRecovery#GPS_FLOOR} m) from its GPS position horizontally (twice that plus 5 m
 *       vertically).
 *   <li>Tilt: when the registered cameras look in nearly the same direction (90th percentile
 *       angle to their mean axis at most {@link #NADIR_P90} degrees), a camera whose axis is more
 *       than max({@link #TILT_FLOOR}, {@link #TILT_FACTOR} x that percentile) degrees off is
 *       rejected. Sets with varied view directions skip this check.
 * </ul>
 *
 * Both fits wait for {@link #MIN_CAMERAS} registered cameras and are refreshed when the model has
 * grown by 5%. A rejected camera stays unregistered and is tried again later, when more of its
 * neighbours are in the model.
 */
final class RegistrationPoseCheck {
  static final int MIN_CAMERAS = 5;
  static final double NADIR_P90 = 5, TILT_FLOOR = 10, TILT_FACTOR = 3;

  private final List<ImageFrame> frames;
  private final List<FeatureTracks.Track> tracks;
  private final double[][] enu;
  private int fittedAt = -1;
  private RegistrationRecovery.Similarity toEnu;
  private double tolerance = Double.NaN, fitRms = Double.NaN;
  private double[] axis;
  private double tiltLimit = Double.NaN, tiltP90 = Double.NaN;
  /** A camera removed after this many bundles is not registered again. */
  static final int MAX_SCREENED = 2;
  /** screen() removes nothing when more than this fraction of the cameras fail. */
  static final double MAX_SCREEN_FRACTION = 0.1;
  private final int[] screenedOut;
  int checked, rejectedGps, rejectedTilt, refits, screens, screensSkipped, removed;
  double largestGpsError, largestTilt;

  RegistrationPoseCheck(List<ImageFrame> frames, List<FeatureTracks.Track> tracks, double[][] enu) {
    this.frames = frames;
    this.tracks = tracks;
    this.enu = enu;
    this.screenedOut = new int[frames.size()];
  }

  /** The check, or null when the option is off or some image lacks usable GPS. */
  static RegistrationPoseCheck create(
      List<ImageFrame> frames, List<FeatureTracks.Track> tracks, Options o) {
    if (!o.registrationPoseCheck.equals("on")) return null;
    if (frames.size() < MIN_CAMERAS
        || !frames.stream().allMatch(f -> f.gps != null && f.gps.usable())) {
      System.out.println("Registration pose check: skipped (needs GPS on every image)");
      return null;
    }
    return new RegistrationPoseCheck(frames, tracks, MatchRepair.positions(frames));
  }

  Predicate<Camera> forImage(int image) {
    return camera -> accept(image, camera);
  }

  static double[] axis(Camera c) {
    double[] r = c.rotation;
    return new double[] {r[6], r[7], r[8]};
  }

  static double angle(double[] a, double[] b) {
    double cos = RegistrationRecovery.dot(a, b) / (Math3.norm(a) * Math3.norm(b));
    return Math.toDegrees(Math.acos(Math.max(-1, Math.min(1, cos))));
  }

  boolean accept(int image, Camera camera) {
    refresh();
    checked++;
    String why = screenedOut[image] >= MAX_SCREENED
        ? "removed after " + MAX_SCREENED + " bundles; not registered again"
        : problem(image, camera);
    if (why == null) return true;
    if (why.startsWith("gps")) rejectedGps++;
    else if (why.startsWith("tilt")) rejectedTilt++;
    if (Geometry.LOG != null) Geometry.LOG.poseRejected(image, why);
    System.out.printf(
        Locale.ROOT, "Registration pose check rejected %s: %s%n", frames.get(image).path.getFileName(), why);
    return false;
  }

  /**
   * After a bundle adjustment: un-registers cameras the bundle moved beyond the limits (the seed
   * cameras stay), then clears the points left with fewer than two registered views, so the
   * cameras are solved again from the rest of the block. Nothing is removed when more than
   * {@link #MAX_SCREEN_FRACTION} of the cameras fail (a poor fit rather than bad cameras).
   * Returns the number of cameras removed.
   */
  int screen(String when) {
    fittedAt = -1;
    refresh();
    if (toEnu == null && axis == null) return 0;
    List<Integer> drop = new ArrayList<>();
    List<String> reasons = new ArrayList<>();
    int registered = 0;
    for (ImageFrame f : frames) {
      if (!f.camera.registered) continue;
      registered++;
      if (f.camera.fixed) continue;
      String why = problem(f.id, f.camera);
      if (why != null) {
        drop.add(f.id);
        reasons.add(why);
      }
    }
    screens++;
    if (drop.isEmpty()) return 0;
    if (tooMany(drop.size(), registered)) {
      screensSkipped++;
      System.out.printf(Locale.ROOT,
          "Registration pose check after %s: %d of %d cameras off; not removed (fit unreliable)%n",
          when, drop.size(), registered);
      return 0;
    }
    for (int k = 0; k < drop.size(); k++) {
      int image = drop.get(k);
      frames.get(image).camera.registered = false;
      screenedOut[image]++;
      removed++;
      String why = "after " + when + ": " + reasons.get(k);
      if (Geometry.LOG != null) {
        Geometry.LOG.poseRejected(image, why);
        Geometry.LOG.method[image] = null;
      }
      System.out.printf(Locale.ROOT, "Registration pose check removed %s %s%n",
          frames.get(image).path.getFileName(), why);
    }
    for (FeatureTracks.Track track : tracks) {
      if (track.point == null) continue;
      int views = 0, last = -1;
      for (FeatureTracks.Observation ob : track.observations)
        if (ob.image != last && frames.get(ob.image).camera.registered) {
          views++;
          last = ob.image;
        }
      if (views < 2) track.point = null;
    }
    fittedAt = -1;
    return drop.size();
  }

  /** More failing cameras than a few bad ones explain: max(2, 10% of the registered). */
  static boolean tooMany(int failing, int registered) {
    return failing > Math.max(2, MAX_SCREEN_FRACTION * registered);
  }

  /** The reason the pose fails a check, or null. */
  private String problem(int image, Camera camera) {
    String why = null;
    if (toEnu != null) {
      double[] at = toEnu.apply(camera.center()), want = enu[image];
      double horizontal = Math.hypot(at[0] - want[0], at[1] - want[1]);
      double vertical = Math.abs(at[2] - want[2]);
      if (horizontal > tolerance || vertical > 2 * tolerance + 5) {
        largestGpsError = Math.max(largestGpsError, Math.hypot(horizontal, vertical));
        why =
            String.format(
                Locale.ROOT,
                "gps: %.1f m horizontal, %.1f m vertical from GPS (limit %.1f m)",
                horizontal,
                vertical,
                tolerance);
      }
    }
    if (why == null && axis != null) {
      double tilt = angle(axis(camera), axis);
      if (tilt > tiltLimit) {
        largestTilt = Math.max(largestTilt, tilt);
        why =
            String.format(
                Locale.ROOT,
                "tilt: %.1f deg from the block's mean view axis (limit %.1f deg)",
                tilt,
                tiltLimit);
      }
    }
    return why;
  }

  private void refresh() {
    List<Integer> cameras = new ArrayList<>();
    boolean[] member = new boolean[frames.size()];
    for (ImageFrame f : frames)
      if (f.camera.registered) {
        cameras.add(f.id);
        member[f.id] = true;
      }
    int count = cameras.size();
    if (fittedAt >= 0 && count >= fittedAt && count < fittedAt + Math.max(1, fittedAt / 20)) return;
    fittedAt = count;
    toEnu = null;
    axis = null;
    if (count < MIN_CAMERAS) return;
    refits++;
    RegistrationRecovery.Fit fit = RegistrationRecovery.toGps(frames, cameras, enu, tracks, member);
    if (fit != null && fit.rms() <= RegistrationRecovery.GPS_UNUSABLE) {
      toEnu = fit.similarity();
      fitRms = fit.rms();
      tolerance =
          Math.max(RegistrationRecovery.GPS_FACTOR * fit.rms(), RegistrationRecovery.GPS_FLOOR);
    }
    double[] mean = new double[3];
    for (int i : cameras) {
      double[] a = axis(frames.get(i).camera);
      double length = Math3.norm(a);
      for (int k = 0; k < 3; k++) mean[k] += a[k] / length;
    }
    if (!(Math3.norm(mean) > 0)) return;
    double[] angles = new double[count];
    for (int j = 0; j < count; j++) angles[j] = angle(axis(frames.get(cameras.get(j)).camera), mean);
    Arrays.sort(angles);
    double p90 = angles[Math.min(count - 1, (int) Math.ceil(0.9 * count) - 1)];
    tiltP90 = p90;
    if (p90 <= NADIR_P90) {
      axis = mean;
      tiltLimit = Math.max(TILT_FLOOR, TILT_FACTOR * p90);
    }
  }

  /** Summary for registration_summary.json. */
  Map<String, Object> summary() {
    Map<String, Object> m = new LinkedHashMap<>();
    m.put("checked_poses", checked);
    m.put("rejected_gps", rejectedGps);
    m.put("rejected_tilt", rejectedTilt);
    m.put("refits", refits);
    m.put("post_bundle_screens", screens);
    m.put("post_bundle_screens_skipped", screensSkipped);
    m.put("removed_after_bundle", removed);
    m.put("last_gps_fit_rms_metres", Double.isNaN(fitRms) ? null : fitRms);
    m.put("last_gps_tolerance_metres", Double.isNaN(tolerance) ? null : tolerance);
    m.put("last_tilt_p90_degrees", Double.isNaN(tiltP90) ? null : tiltP90);
    m.put("last_tilt_limit_degrees", axis == null ? null : tiltLimit);
    m.put("largest_rejected_gps_error_metres", rejectedGps == 0 ? null : largestGpsError);
    m.put("largest_rejected_tilt_degrees", rejectedTilt == 0 ? null : largestTilt);
    return m;
  }
}
