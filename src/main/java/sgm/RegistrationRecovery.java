package sgm;

import java.io.IOException;
import java.util.*;
import org.opencv.core.*;

/**
 * --registration-recovery on: images still unregistered after the main model's retry pass.
 *
 * <ol>
 *   <li>Further models: the best seed pair among unregistered images grows a separate model
 *       (the main model's cameras and points are set aside meanwhile; the new model may take in
 *       up to {@link #COMMON_LIMIT} of the main model's images as anchors). It is merged into the
 *       main model by a similarity transform from the images both models contain (camera centres
 *       and viewing axes), or, with fewer than two of those, from GPS positions (both models
 *       fitted to local east/north/up; a nearly straight strip of cameras gets its roll from the
 *       ground-plane normal of its tie points). A merge whose fit is poor is rejected. After each
 *       merge a joint bundle and an ordinary PnP retry pass run.
 *   <li>GPS-checked PnP: remaining images are tried with a lower inlier minimum
 *       ({@link #RELAXED_INLIERS}), accepted only when the solved camera lands where its GPS
 *       position (mapped into the model) says.
 * </ol>
 *
 * Outcomes are recorded in {@link Geometry#LOG} (registration.json / registration_summary.json).
 */
final class RegistrationRecovery {
  private RegistrationRecovery() {}

  static final int COMMON_LIMIT = 6, MAX_MODELS = 50, MIN_MODEL_IMAGES = 3, RELAXED_INLIERS = 6;
  /** GPS fits: residual limit max(GPS_FACTOR x main-model RMS, GPS_FLOOR metres). */
  static final double GPS_FACTOR = 3, GPS_FLOOR = 3, GPS_UNUSABLE = 15;

  /** x' = s Q x + u. */
  record Similarity(double s, double[] q, double[] u) {
    double[] apply(double[] x) {
      return Math3.add(Math3.times(Math3.mv(q, x), s), u);
    }

    Similarity inverse() {
      double[] qt = Math3.transpose(q);
      return new Similarity(1 / s, qt, Math3.times(Math3.mv(qt, u), -1 / s));
    }

    /** this after other: x -> this(other(x)). */
    Similarity after(Similarity other) {
      return new Similarity(
          s * other.s, Math3.mul(q, other.q), apply(other.u));
    }

    /** The same camera in the target frame. */
    void transform(Camera c) {
      double[] center = apply(c.center());
      c.rotation = Math3.mul(c.rotation, Math3.transpose(q));
      c.translation = Math3.times(Math3.mv(c.rotation, center), -1);
    }
  }

  record Fit(Similarity similarity, double rms, int points) {}

  /** Umeyama least-squares similarity src -> dst; rms over the first {@code scored} pairs. */
  static Fit umeyama(List<double[]> src, List<double[]> dst, int scored) {
    int n = src.size();
    if (n < 3) return null;
    double[] ms = new double[3], md = new double[3];
    for (int i = 0; i < n; i++)
      for (int j = 0; j < 3; j++) {
        ms[j] += src.get(i)[j] / n;
        md[j] += dst.get(i)[j] / n;
      }
    double[] sigma = new double[9];
    double var = 0;
    for (int i = 0; i < n; i++) {
      double[] a = Math3.sub(src.get(i), ms), b = Math3.sub(dst.get(i), md);
      var += dot(a, a) / n;
      for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) sigma[3 * r + c] += b[r] * a[c] / n;
    }
    if (!(var > 1e-18)) return null;
    Mat m = new Mat(3, 3, CvType.CV_64F), w = new Mat(), u = new Mat(), vt = new Mat();
    try {
      m.put(0, 0, sigma);
      Core.SVDecomp(m, w, u, vt);
      double[] U = new double[9], Vt = new double[9], W = new double[3];
      u.get(0, 0, U);
      vt.get(0, 0, Vt);
      w.get(0, 0, W);
      double det = det(U) * det(Vt);
      double[] S = {1, 0, 0, 0, 1, 0, 0, 0, det < 0 ? -1 : 1};
      double[] q = Math3.mul(Math3.mul(U, S), Vt);
      double s = (W[0] + W[1] + (det < 0 ? -W[2] : W[2])) / var;
      if (!(s > 0) || !Double.isFinite(s)) return null;
      double[] t = Math3.sub(md, Math3.times(Math3.mv(q, ms), s));
      Similarity sim = new Similarity(s, q, t);
      double sum = 0;
      for (int i = 0; i < scored; i++) {
        double[] e = Math3.sub(sim.apply(src.get(i)), dst.get(i));
        sum += dot(e, e);
      }
      return new Fit(sim, Math.sqrt(sum / Math.max(1, scored)), n);
    } finally {
      m.release();
      w.release();
      u.release();
      vt.release();
    }
  }

  /** Camera centres (and ground normal when they are nearly collinear) of a model to ENU. */
  static Fit toGps(
      List<ImageFrame> frames, List<Integer> cameras, double[][] enu, List<FeatureTracks.Track> tracks,
      boolean[] member) {
    List<double[]> src = new ArrayList<>(), dst = new ArrayList<>();
    for (int i : cameras) {
      src.add(frames.get(i).camera.center());
      dst.add(enu[i]);
    }
    if (src.size() < 3) return null;
    int centres = src.size();
    double spreadModel = spread(src), spreadGps = spread(dst);
    if (!(spreadModel > 0) || !(spreadGps > 0)) return null;
    double[] normal = null;
    if (straightness(src) < .1) {
      normal = groundNormal(frames, cameras, tracks, member);
      if (normal == null) return null;
    }
    // Robust: refit without cameras whose GPS residual is over 3 x the median (at least 1 m);
    // a few bad GPS positions would otherwise inflate the fit and loosen every limit.
    boolean[] keep = new boolean[centres];
    Arrays.fill(keep, true);
    Fit fit = null;
    for (int round = 0; round < 3; round++) {
      List<double[]> s = new ArrayList<>(), d = new ArrayList<>();
      for (int k = 0; k < centres; k++)
        if (keep[k]) {
          s.add(src.get(k));
          d.add(dst.get(k));
        }
      int scored = s.size();
      if (scored < 3) return fit;
      if (normal != null) {
        // Offsets along the normal / up in proportion to the kept cameras' spreads (the scale).
        double sm = spread(s), sg = spread(d);
        if (!(sm > 0) || !(sg > 0)) return fit;
        for (int k = 0; k < scored; k++) {
          s.add(Math3.add(s.get(k), Math3.times(normal, sm)));
          d.add(Math3.add(d.get(k), new double[] {0, 0, sg}));
        }
      }
      Fit next = umeyama(s, d, scored);
      if (next == null) return fit;
      fit = next;
      double[] residual = new double[centres];
      for (int k = 0; k < centres; k++)
        residual[k] = Math3.norm(Math3.sub(fit.similarity().apply(src.get(k)), dst.get(k)));
      double[] sorted = residual.clone();
      Arrays.sort(sorted);
      double cut = Math.max(3 * sorted[centres / 2], 1);
      boolean changed = false;
      for (int k = 0; k < centres; k++) {
        boolean in = residual[k] <= cut;
        if (in != keep[k]) changed = true;
        keep[k] = in;
      }
      if (!changed) break;
    }
    return fit;
  }

  /** RMS distance to the centroid. */
  static double spread(List<double[]> points) {
    double[] m = new double[3];
    for (double[] p : points) for (int j = 0; j < 3; j++) m[j] += p[j] / points.size();
    double sum = 0;
    for (double[] p : points) {
      double[] d = Math3.sub(p, m);
      sum += dot(d, d);
    }
    return Math.sqrt(sum / points.size());
  }

  /** Second / first principal spread of the points (0 = on a line). */
  static double straightness(List<double[]> points) {
    double[] m = new double[3];
    for (double[] p : points) for (int j = 0; j < 3; j++) m[j] += p[j] / points.size();
    double[] c = new double[9];
    for (double[] p : points) {
      double[] d = Math3.sub(p, m);
      for (int r = 0; r < 3; r++) for (int k = 0; k < 3; k++) c[3 * r + k] += d[r] * d[k];
    }
    double[] values = eigen(c, null);
    return values[0] > 0 ? Math.sqrt(Math.max(0, values[1]) / values[0]) : 0;
  }

  /** Eigenvalues (descending) of a symmetric 3x3; vectors (rows) into {@code vectors} when given. */
  static double[] eigen(double[] c, double[] vectors) {
    Mat m = new Mat(3, 3, CvType.CV_64F), values = new Mat(), v = new Mat();
    try {
      m.put(0, 0, c);
      Core.eigen(m, values, v);
      double[] out = new double[3];
      values.get(0, 0, out);
      if (vectors != null) v.get(0, 0, vectors);
      return out;
    } finally {
      m.release();
      values.release();
      v.release();
    }
  }

  /** Normal of the tie points seen by the model's cameras, pointing towards the cameras. */
  static double[] groundNormal(
      List<ImageFrame> frames, List<Integer> cameras, List<FeatureTracks.Track> tracks, boolean[] member) {
    List<double[]> points = new ArrayList<>();
    int step = Math.max(1, tracks.size() / 50000);
    for (int k = 0; k < tracks.size(); k += step) {
      FeatureTracks.Track t = tracks.get(k);
      if (t.point == null) continue;
      for (FeatureTracks.Observation ob : t.observations)
        if (member[ob.image]) {
          points.add(t.point);
          break;
        }
    }
    if (points.size() < 20) return null;
    double[] m = new double[3];
    for (double[] p : points) for (int j = 0; j < 3; j++) m[j] += p[j] / points.size();
    double[] c = new double[9];
    for (double[] p : points) {
      double[] d = Math3.sub(p, m);
      for (int r = 0; r < 3; r++) for (int k = 0; k < 3; k++) c[3 * r + k] += d[r] * d[k];
    }
    double[] vectors = new double[9];
    eigen(c, vectors);
    double[] normal = {vectors[6], vectors[7], vectors[8]};
    double[] centre = new double[3];
    for (int i : cameras) {
      double[] x = frames.get(i).camera.center();
      for (int j = 0; j < 3; j++) centre[j] += x[j] / cameras.size();
    }
    if (dot(normal, Math3.sub(centre, m)) < 0) normal = Math3.times(normal, -1);
    double length = Math3.norm(normal);
    return length > 0 ? Math3.times(normal, 1 / length) : null;
  }

  /** Similarity from the images both models contain: centres plus points along their axes. */
  static Fit fromCommon(List<ImageFrame> frames, List<Integer> common, Camera[] other) {
    List<double[]> src = new ArrayList<>(), dst = new ArrayList<>();
    for (int i : common) {
      src.add(other[i].center());
      dst.add(frames.get(i).camera.center());
    }
    int centres = src.size();
    if (centres < 2) return null;
    double spreadSrc = spread(src), spreadDst = spread(dst);
    if (!(spreadSrc > 0) || !(spreadDst > 0)) return null;
    for (int k = 0; k < centres; k++) {
      int i = common.get(k);
      double[] rs = other[i].rotation, rd = frames.get(i).camera.rotation;
      for (int axis = 0; axis < 2; axis++) {
        double[] as = {rs[3 * axis], rs[3 * axis + 1], rs[3 * axis + 2]};
        double[] ad = {rd[3 * axis], rd[3 * axis + 1], rd[3 * axis + 2]};
        src.add(Math3.add(src.get(k), Math3.times(as, spreadSrc)));
        dst.add(Math3.add(dst.get(k), Math3.times(ad, spreadDst)));
      }
    }
    return umeyama(src, dst, centres);
  }

  /** Median distance from each camera to its nearest other camera (model units). */
  static double spacing(List<ImageFrame> frames, List<Integer> cameras) {
    double[] nearest = new double[cameras.size()];
    for (int a = 0; a < cameras.size(); a++) {
      double best = Double.MAX_VALUE;
      double[] ca = frames.get(cameras.get(a)).camera.center();
      for (int b = 0; b < cameras.size(); b++)
        if (a != b) best = Math.min(best, Math3.norm(Math3.sub(ca, frames.get(cameras.get(b)).camera.center())));
      nearest[a] = best;
    }
    Arrays.sort(nearest);
    return nearest.length == 0 ? Double.NaN : nearest[nearest.length / 2];
  }

  /** Saved poses, flags and points of all cameras/tracks. */
  private static final class State {
    final double[][] rotation, translation, points;
    final boolean[] registered, fixed;

    State(List<ImageFrame> frames, List<FeatureTracks.Track> tracks) {
      int n = frames.size();
      rotation = new double[n][];
      translation = new double[n][];
      registered = new boolean[n];
      fixed = new boolean[n];
      for (int i = 0; i < n; i++) {
        Camera c = frames.get(i).camera;
        rotation[i] = c.rotation.clone();
        translation[i] = c.translation.clone();
        registered[i] = c.registered;
        fixed[i] = c.fixed;
      }
      points = new double[tracks.size()][];
      for (int k = 0; k < tracks.size(); k++)
        points[k] = tracks.get(k).point == null ? null : tracks.get(k).point.clone();
    }

    void restore(List<ImageFrame> frames, List<FeatureTracks.Track> tracks) {
      for (int i = 0; i < frames.size(); i++) {
        Camera c = frames.get(i).camera;
        c.rotation = rotation[i].clone();
        c.translation = translation[i].clone();
        c.registered = registered[i];
        c.fixed = fixed[i];
      }
      for (int k = 0; k < tracks.size(); k++)
        tracks.get(k).point = points[k] == null ? null : points[k].clone();
    }
  }

  static boolean run(
      List<ImageFrame> frames,
      FeatureTracks features,
      Options o,
      Project project,
      int anchor,
      int scale,
      int triangulation)
      throws IOException {
    RegistrationDiagnostics log = Geometry.LOG;
    List<FeatureTracks.Track> tracks = features.tracks;
    int n = frames.size();
    long started = System.nanoTime();
    boolean gps = frames.stream().allMatch(f -> f.gps != null && f.gps.usable());
    double[][] enu = gps ? MatchRepair.positions(frames) : null;
    int before = registeredCount(frames);
    Set<Long> tried = new HashSet<>();
    boolean changed = false;
    int modelId = 0, rejected = 0;
    System.out.printf(
        Locale.ROOT, "Registration recovery: %d of %d images unregistered%n", n - before, n);
    for (int round = 0; round < MAX_MODELS * 4 && modelId < MAX_MODELS; round++) {
      FeatureTracks.Pair seed = null;
      double best = 0;
      for (FeatureTracks.Pair p : features.pairs) {
        if (p.rotation == null || p.translation == null) continue;
        if (frames.get(p.a).camera.registered || frames.get(p.b).camera.registered) continue;
        if (tried.contains(PairSelection.key(p.a, p.b))) continue;
        double score = Geometry.seedScore(p, frames);
        if (score > best) {
          best = score;
          seed = p;
        }
      }
      if (seed == null) break;
      tried.add(PairSelection.key(seed.a, seed.b));
      // Grow a separate model with the main one set aside.
      State main = new State(frames, tracks);
      boolean[] inMain = main.registered.clone();
      for (ImageFrame f : frames) {
        f.camera.registered = false;
        f.camera.fixed = false;
      }
      for (FeatureTracks.Track t : tracks) t.point = null;
      Camera a = frames.get(seed.a).camera, b = frames.get(seed.b).camera;
      a.rotation = new double[] {1, 0, 0, 0, 1, 0, 0, 0, 1};
      a.translation = new double[3];
      b.rotation = seed.rotation.clone();
      b.translation = Math3.times(seed.translation, o.baseline / Math3.norm(seed.translation));
      a.registered = b.registered = a.fixed = b.fixed = true;
      String previousStage = log == null ? null : log.stage;
      if (log != null) log.stage = null; // a trial model: not recorded
      int[] common = {0};
      Geometry.TrackIndex index = new Geometry.TrackIndex(frames, tracks, o);
      index.triangulation = triangulation;
      Geometry.triangulateAll(frames, index, o.reprojection);
      Geometry.Gate gate =
          new Geometry.Gate() {
            @Override
            public boolean allowed(int image) {
              return !inMain[image] || common[0] < COMMON_LIMIT;
            }

            @Override
            public void registered(int image) {
              if (inMain[image]) common[0]++;
            }
          };
      Geometry.grow(frames, features, index, o, null, seed.a, seed.b, gate);
      index = null;
      List<Integer> members = new ArrayList<>();
      for (ImageFrame f : frames) if (f.camera.registered) members.add(f.id);
      if (members.size() >= MIN_MODEL_IMAGES)
        BundleAdjuster.optimizeImpl(
            frames, tracks, o.iterations, o.reprojection, null, seed.a, seed.b, null);
      Camera[] model = new Camera[n];
      for (int i : members) model[i] = frames.get(i).camera.copy();
      double[][] modelPoints = new double[tracks.size()][];
      for (int k = 0; k < tracks.size(); k++) modelPoints[k] = tracks.get(k).point;
      boolean[] member = new boolean[n];
      for (int i : members) member[i] = true;
      Fit modelToGps = null;
      if (gps && members.size() >= MIN_MODEL_IMAGES)
        modelToGps = toGps(frames, members, enu, tracks, member);
      main.restore(frames, tracks);
      if (log != null) log.stage = previousStage;
      List<Integer> fresh = new ArrayList<>(), shared = new ArrayList<>();
      for (int i : members) (inMain[i] ? shared : fresh).add(i);
      if (members.size() < MIN_MODEL_IMAGES || fresh.isEmpty()) continue;
      // Merge: through shared images, else through GPS.
      Fit fit = null;
      String via = null;
      double limit = Double.NaN;
      List<Integer> mainCameras = new ArrayList<>();
      for (ImageFrame f : frames) if (f.camera.registered) mainCameras.add(f.id);
      if (shared.size() >= 2) {
        fit = fromCommon(frames, shared, model);
        via = "shared images";
        limit = .25 * spacing(frames, mainCameras);
      }
      Fit mainToGps = null;
      if ((fit == null || !(fit.rms() <= limit)) && gps && modelToGps != null) {
        mainToGps = toGps(frames, mainCameras, enu, tracks, inMain);
        if (mainToGps != null && mainToGps.rms() <= GPS_UNUSABLE) {
          double gpsLimit = Math.max(GPS_FACTOR * mainToGps.rms(), GPS_FLOOR);
          if (modelToGps.rms() <= gpsLimit) {
            fit =
                new Fit(
                    mainToGps.similarity().inverse().after(modelToGps.similarity()),
                    modelToGps.rms(),
                    modelToGps.points());
            via = "GPS";
            limit = gpsLimit;
          } else {
            fit = modelToGps;
            via = "GPS";
            limit = gpsLimit;
          }
        }
      }
      boolean accepted = fit != null && fit.rms() <= limit;
      Map<String, Object> entry = new LinkedHashMap<>();
      entry.put("model", modelId + 2);
      entry.put("seed", List.of(frames.get(seed.a).path.getFileName().toString(),
          frames.get(seed.b).path.getFileName().toString()));
      entry.put("images", members.size());
      entry.put("new_images", fresh.size());
      entry.put("shared_with_main", shared.size());
      entry.put("merged_via", via);
      entry.put("fit_rms", fit == null ? null : fit.rms());
      entry.put("fit_limit", Double.isFinite(limit) ? limit : null);
      entry.put("fit_units", "GPS".equals(via) ? "metres" : "model units");
      entry.put("accepted", accepted);
      if (log != null) log.models.add(entry);
      if (!accepted) {
        rejected++;
        System.out.printf(
            Locale.ROOT,
            "Registration recovery: model from %s/%s (%d images, %d new) not merged: %s%n",
            frames.get(seed.a).path.getFileName(),
            frames.get(seed.b).path.getFileName(),
            members.size(),
            fresh.size(),
            fit == null ? "no usable shared images or GPS fit"
                : String.format(Locale.ROOT, "%s fit RMS %.3g above %.3g", via, fit.rms(), limit));
        continue;
      }
      modelId++;
      Similarity sim = fit.similarity();
      for (int i : fresh) {
        Camera c = frames.get(i).camera;
        c.rotation = model[i].rotation.clone();
        c.translation = model[i].translation.clone();
        sim.transform(c);
        c.registered = true;
        c.fixed = false;
        if (log != null) log.registered(i, "model " + (modelId + 1) + " (" + via + ")", modelId);
      }
      int points = 0;
      for (int k = 0; k < tracks.size(); k++)
        if (tracks.get(k).point == null && modelPoints[k] != null) {
          boolean seen = false;
          for (FeatureTracks.Observation ob : tracks.get(k).observations)
            if (member[ob.image] && !inMain[ob.image]) seen = true;
          if (!seen) continue;
          tracks.get(k).point = sim.apply(modelPoints[k]);
          points++;
        }
      System.out.printf(
          Locale.ROOT,
          "Registration recovery: model %d merged via %s (%d new images, %d shared, fit RMS %.3g,"
              + " limit %.3g); %d points added%n",
          modelId + 1,
          via,
          fresh.size(),
          shared.size(),
          fit.rms(),
          limit,
          points);
      changed = true;
      BundleAdjuster.optimizeImpl(
          frames, tracks, o.iterations, o.reprojection, null, anchor, scale, null);
      retry(frames, features, o, project, triangulation, "retry after model " + (modelId + 1), null, 0);
    }
    // GPS-checked PnP with a lower inlier minimum.
    int gpsRegistered = 0;
    if (gps && registeredCount(frames) < n) {
      List<Integer> mainCameras = new ArrayList<>();
      boolean[] inMain = new boolean[n];
      for (ImageFrame f : frames)
        if (f.camera.registered) {
          mainCameras.add(f.id);
          inMain[f.id] = true;
        }
      Fit mainToGps = toGps(frames, mainCameras, enu, tracks, inMain);
      if (mainToGps != null && mainToGps.rms() <= GPS_UNUSABLE) {
        double tolerance = Math.max(GPS_FACTOR * mainToGps.rms(), GPS_FLOOR);
        Similarity toEnu = mainToGps.similarity();
        java.util.function.IntFunction<java.util.function.Predicate<Camera>> check =
            image ->
                camera -> {
                  double[] at = toEnu.apply(camera.center());
                  double[] want = enu[image];
                  return Math.hypot(at[0] - want[0], at[1] - want[1]) <= tolerance
                      && Math.abs(at[2] - want[2]) <= 2 * tolerance + 5;
                };
        gpsRegistered =
            retry(frames, features, o, project, triangulation, "gps_checked_pnp", check, RELAXED_INLIERS);
        if (log != null) {
          log.recovery.put("gps_fit_rms_metres", mainToGps.rms());
          log.recovery.put("gps_check_tolerance_metres", tolerance);
        }
        changed |= gpsRegistered > 0;
        System.out.printf(
            Locale.ROOT,
            "Registration recovery: GPS-checked PnP (>= %d inliers, within %.1f m of GPS) added %d"
                + " images%n",
            RELAXED_INLIERS,
            tolerance,
            gpsRegistered);
      } else
        System.out.println(
            "Registration recovery: GPS-checked PnP skipped (main model does not fit GPS well enough)");
    }
    int after = registeredCount(frames);
    double seconds = (System.nanoTime() - started) / 1e9;
    if (log != null) {
      log.recovery.put("registered_before", before);
      log.recovery.put("registered_after", after);
      log.recovery.put("models_merged", modelId);
      log.recovery.put("models_rejected", rejected);
      log.recovery.put("gps_checked_pnp", gpsRegistered);
      log.recovery.put("seconds", seconds);
    }
    System.out.printf(
        Locale.ROOT,
        "Registration recovery: %d -> %d of %d images registered (%d models merged, %d rejected;"
            + " %.1f s)%n",
        before,
        after,
        n,
        modelId,
        rejected,
        seconds);
    return changed;
  }

  /**
   * PnP passes over unregistered images until none is added; with {@code check}, the inlier
   * minimum is {@code minimum} and each camera must pass its check. Returns the number added.
   */
  private static int retry(
      List<ImageFrame> frames,
      FeatureTracks features,
      Options o,
      Project project,
      int triangulation,
      String stage,
      java.util.function.IntFunction<java.util.function.Predicate<Camera>> check,
      int minimum)
      throws IOException {
    RegistrationDiagnostics log = Geometry.LOG;
    String previous = log == null ? null : log.stage;
    if (log != null) log.stage = stage;
    Geometry.TrackIndex index = new Geometry.TrackIndex(frames, features.tracks, o);
    index.triangulation = triangulation;
    Geometry.triangulateAll(frames, index, o.reprojection);
    int added = 0;
    try {
      for (int pass = 0; pass < frames.size(); pass++) {
        boolean progress = false;
        for (ImageFrame f : frames) {
          if (f.camera.registered) continue;
          boolean ok =
              check == null
                  ? Geometry.register(
                      f, features.tracks, index, o, project, false,
                      Math.max(8, o.minMatches / 2), null)
                  : Geometry.register(
                      f, features.tracks, index, o, project, false, minimum, check.apply(f.id));
          if (ok) {
            Geometry.triangulateNew(frames, index, f.id, o.reprojection);
            added++;
            progress = true;
          }
        }
        if (!progress) break;
      }
    } finally {
      if (log != null) log.stage = previous;
    }
    return added;
  }

  static double dot(double[] a, double[] b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  }

  static double det(double[] m) {
    return m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6])
        + m[2] * (m[3] * m[7] - m[4] * m[6]);
  }

  static int registeredCount(List<ImageFrame> frames) {
    int n = 0;
    for (ImageFrame f : frames) if (f.camera.registered) n++;
    return n;
  }
}
