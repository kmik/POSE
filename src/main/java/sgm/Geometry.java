package sgm;

import java.io.*;
import java.nio.file.Path;
import java.util.*;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;

final class Geometry {
  private Geometry() {}

  static void reconstruct(List<ImageFrame> frames, FeatureTracks features, Options o) {
    try {
      reconstruct(frames, features, o, null);
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  static void reconstruct(
      List<ImageFrame> frames, FeatureTracks features, Options o, Project project)
      throws IOException {
    FeatureTracks.Pair seed =
        features.pairs.stream()
            .max(Comparator.comparingDouble(p -> seedScore(p, frames)))
            .orElseThrow(
                () ->
                    new IllegalArgumentException(
                        "No viable stereo pair. Check overlap, texture, calibration, and"
                            + " --neighbors."));
    if (o.seedPair != null) {
      List<String> names = PoseIO.csv(o.seedPair);
      if (names.size() != 2)
        throw new IllegalArgumentException(
            "--seed-pair must contain two comma-separated filenames");
      seed = null;
      for (FeatureTracks.Pair pair : features.pairs)
        if (frames.get(pair.a).path.getFileName().toString().equals(names.get(0))
            && frames.get(pair.b).path.getFileName().toString().equals(names.get(1))) seed = pair;
      if (seed == null)
        throw new IllegalArgumentException(
            "Seed pair has no viable matches; list filenames in sorted order and increase"
                + " --neighbors if needed");
    }
    if (o.seedPair == null && !o.calibrationPoses.isEmpty()) {
      seed =
          features.pairs.stream()
              .filter(
                  p ->
                      o.calibrationPoses.containsKey(frames.get(p.a).path.getFileName().toString())
                          && o.calibrationPoses.containsKey(
                              frames.get(p.b).path.getFileName().toString()))
              .max(Comparator.comparingDouble(p -> seedScore(p, frames)))
              .orElse(seed);
    }
    if (o.seedPair == null && o.calibrationPoses.isEmpty() && frames.size() > 2)
      seed = selectSeed(frames, features, o, seed, project);
    features.seed = seed;
    System.out.printf(
        Locale.ROOT,
        "Seed pair: %s / %s; baseline %.6f%n",
        frames.get(seed.a).path.getFileName(),
        frames.get(seed.b).path.getFileName(),
        o.baseline);
    Camera a = frames.get(seed.a).camera, b = frames.get(seed.b).camera;
    a.registered = true;
    a.fixed = true;
    b.rotation = seed.rotation.clone();
    b.translation = Math3.times(seed.translation, o.baseline / Math3.norm(seed.translation));
    Camera calibratedA = o.calibrationPoses.get(frames.get(seed.a).path.getFileName().toString()),
        calibratedB = o.calibrationPoses.get(frames.get(seed.b).path.getFileName().toString());
    if (calibratedA != null && calibratedB != null) {
      b.rotation = Math3.mul(calibratedB.rotation, Math3.transpose(calibratedA.rotation));
      double[] translation =
          Math3.sub(calibratedB.translation, Math3.mv(b.rotation, calibratedA.translation));
      b.translation = Math3.times(translation, o.baseline / Math3.norm(translation));
      System.out.println(
          "Seed exterior orientation initialized from accepted joint self-calibration.");
    }
    b.registered = true;
    b.fixed = true;
    triangulateTracks(frames, features.tracks, o.reprojection);
    if (project != null) {
      project.event(
          "seed_pair",
          Json.object(
              "image_a",
              seed.a,
              "image_b",
              seed.b,
              "baseline",
              o.baseline,
              "orientation_source",
              calibratedA != null && calibratedB != null
                  ? "accepted_self_calibration"
                  : "pair_essential_matrix"));
      project.data.snapshot(
          "seed", frames, features.tracks, "seed-camera frame; baseline=" + o.baseline);
    }
    while (true) {
      int best = -1, most = 0;
      for (ImageFrame f : frames)
        if (!f.camera.registered) {
          int n = 0;
          for (FeatureTracks.Track track : features.tracks)
            if (track.point != null && observation(track, f.id) != null) n++;
          if (n > most) {
            most = n;
            best = f.id;
          }
        }
      if (best < 0 || most < Math.max(8, o.minMatches / 2)) break;
      ImageFrame next = frames.get(best);
      if (!register(next, features.tracks, o, project)) {
        // Try all currently registerable views before declaring the graph disconnected.
        boolean registered = false;
        for (ImageFrame candidate : frames)
          if (!candidate.camera.registered
              && candidate.id != best
              && register(candidate, features.tracks, o, project)) {
            registered = true;
            break;
          }
        if (!registered) break;
      }
      triangulateTracks(frames, features.tracks, o.reprojection);
      if (project != null)
        project.data.snapshot(
            "registration_" + frames.stream().filter(f -> f.camera.registered).count(),
            frames,
            features.tracks,
            "seed-camera frame");
    }
    long registered = frames.stream().filter(f -> f.camera.registered).count();
    if (registered != frames.size()) {
      List<String> missing = new ArrayList<>();
      for (ImageFrame f : frames)
        if (!f.camera.registered) missing.add(f.path.getFileName().toString());
      System.err.println(
          "Some images could not be registered ("
              + registered
              + "/"
              + frames.size()
              + "). Missing: "
              + missing
              + ". They will be excluded from dense reconstruction.");
    }
    if (registered < 2) throw new IllegalArgumentException("Fewer than two cameras registered");
    List<Object> status = new ArrayList<>();
    for (ImageFrame frame : frames)
      status.add(
          Json.object(
              "image_id",
              frame.id,
              "filename",
              frame.path.getFileName().toString(),
              "registered",
              frame.camera.registered));
    java.nio.file.Path statusFile = o.output.resolve("04_geometry/registration.json");
    java.nio.file.Files.createDirectories(statusFile.getParent());
    Json.write(statusFile, status);
    if (project != null)
      project.data.snapshot("pre_bundle", frames, features.tracks, "seed-camera frame");
    BundleAdjuster.optimize(
        frames, features.tracks, o.iterations, o.reprojection, project, seed.a, seed.b);
    SparseCleanup.run(frames, features.tracks, o, project, seed.a, seed.b);
    boolean recovered = false;
    for (int pass = 0; pass < frames.size(); pass++) {
      boolean changed = false;
      for (ImageFrame frame : frames)
        if (!frame.camera.registered && register(frame, features.tracks, o, project)) {
          changed = true;
          recovered = true;
          triangulateTracks(frames, features.tracks, o.reprojection);
        }
      if (!changed) break;
    }
    if (recovered) {
      BundleAdjuster.optimize(
          frames, features.tracks, o.iterations, o.reprojection, null, seed.a, seed.b);
      SparseCleanup.run(frames, features.tracks, o, null, seed.a, seed.b);
      if (project != null)
        project.data.snapshot(
            "registration_retry",
            frames,
            features.tracks,
            "after registration recovery and bundle refinement");
    }
    status.clear();
    for (ImageFrame frame : frames)
      status.add(
          Json.object(
              "image_id",
              frame.id,
              "filename",
              frame.path.getFileName().toString(),
              "registered",
              frame.camera.registered));
    Json.write(statusFile, status);
    if (project != null)
      project.data.snapshot(
          "post_bundle_before_track_filter", frames, features.tracks, "seed-camera frame");
    for (FeatureTracks.Track track : features.tracks) {
      boolean keep = track.point != null && valid(frames, track, track.point, o.reprojection * 2);
      if (project != null) project.data.trackFilter(track, frames, keep);
      if (track.point != null && !keep) track.point = null;
    }
  }

  static double seedScore(FeatureTracks.Pair pair, List<ImageFrame> frames) {
    if (pair.rotation == null || pair.translation == null) return 0;
    List<Double> angles = new ArrayList<>();
    double[] rt = Math3.transpose(pair.rotation);
    for (int k = 0; k < pair.matches.size(); k += Math.max(1, pair.matches.size() / 200)) {
      var m = pair.matches.get(k);
      Point
          a =
              FeatureTracks.normalize(
                  frames.get(pair.a).camera, frames.get(pair.a).keypoints[m.a].pt),
          b =
              FeatureTracks.normalize(
                  frames.get(pair.b).camera, frames.get(pair.b).keypoints[m.b].pt);
      double[] x = {a.x, a.y, 1}, y = Math3.mv(rt, new double[] {b.x, b.y, 1});
      double cosine = (x[0] * y[0] + x[1] * y[1] + x[2] * y[2]) / (Math3.norm(x) * Math3.norm(y));
      angles.add(Math.toDegrees(Math.acos(Math.max(-1, Math.min(1, cosine)))));
    }
    if (angles.isEmpty()) return 0;
    Collections.sort(angles);
    double angle = angles.get(angles.size() / 2);
    return pair.matches.size() * Math.min(1, angle / 5);
  }

  private static FeatureTracks.Pair selectSeed(
      List<ImageFrame> frames,
      FeatureTracks features,
      Options o,
      FeatureTracks.Pair fallback,
      Project project)
      throws IOException {
    Core.setRNGSeed(1);
    List<FeatureTracks.Pair> candidates = new ArrayList<>(features.pairs);
    candidates.sort(
        Comparator.comparingDouble((FeatureTracks.Pair p) -> seedScore(p, frames)).reversed());
    Camera[] original = new Camera[frames.size()];
    for (ImageFrame f : frames) original[f.id] = f.camera.copy();
    List<double[]> points = new ArrayList<>();
    for (var t : features.tracks) points.add(t.point);
    FeatureTracks.Pair best = fallback;
    double bestScore = -Double.MAX_VALUE;
    List<Object> audit = new ArrayList<>();
    try {
      for (var pair : candidates.subList(0, Math.min(8, candidates.size()))) {
        for (ImageFrame f : frames) {
          Camera c = original[f.id];
          f.camera.rotation = c.rotation.clone();
          f.camera.translation = c.translation.clone();
          f.camera.registered = false;
          f.camera.fixed = false;
        }
        for (var t : features.tracks) t.point = null;
        Camera a = frames.get(pair.a).camera, b = frames.get(pair.b).camera;
        a.registered = true;
        b.registered = true;
        b.rotation = pair.rotation.clone();
        b.translation = Math3.times(pair.translation, o.baseline / Math3.norm(pair.translation));
        triangulateTracks(frames, features.tracks, o.reprojection);
        for (int pass = 0; pass < frames.size(); pass++) {
          boolean changed = false;
          for (ImageFrame f : frames)
            if (!f.camera.registered && register(f, features.tracks, o, null, true)) {
              changed = true;
              triangulateTracks(frames, features.tracks, o.reprojection);
            }
          if (!changed) break;
        }
        long registered = frames.stream().filter(f -> f.camera.registered).count(),
            triangulated = features.tracks.stream().filter(t -> t.point != null).count();
        double objective = BundleAdjuster.error(frames, features.tracks, o.reprojection);
        double score = registered * 1e9 + triangulated * 1000 - objective;
        audit.add(
            Json.object(
                "a",
                pair.a,
                "b",
                pair.b,
                "registered_cameras",
                registered,
                "triangulated_tracks",
                triangulated,
                "robust_objective",
                objective,
                "score",
                score));
        if (score > bestScore) {
          bestScore = score;
          best = pair;
        }
      }
    } finally {
      for (ImageFrame f : frames) {
        Camera c = original[f.id];
        f.camera.rotation = c.rotation;
        f.camera.translation = c.translation;
        f.camera.registered = c.registered;
        f.camera.fixed = c.fixed;
      }
      for (int i = 0; i < points.size(); i++) features.tracks.get(i).point = points.get(i);
      Core.setRNGSeed(1);
    }
    Path file = o.output.resolve("04_geometry/seed_trials.json");
    java.nio.file.Files.createDirectories(file.getParent());
    Json.write(
        file,
        Json.object(
            "trials",
            audit,
            "chosen_a",
            best.a,
            "chosen_b",
            best.b,
            "selection",
            "prefer global registration support over raw pair match count"));
    return best;
  }

  private static boolean register(
      ImageFrame frame, List<FeatureTracks.Track> tracks, Options o, Project project)
      throws IOException {
    return register(frame, tracks, o, project, false);
  }

  private static boolean register(
      ImageFrame frame, List<FeatureTracks.Track> tracks, Options o, Project project, boolean quiet)
      throws IOException {
    List<Point3> xyz = new ArrayList<>();
    List<Point> uv = new ArrayList<>();
    List<FeatureTracks.Track> usedTracks = new ArrayList<>();
    for (FeatureTracks.Track track : tracks) {
      FeatureTracks.Observation observation = observation(track, frame.id);
      if (track.point != null && observation != null) {
        double[] p = track.point;
        xyz.add(new Point3(p[0], p[1], p[2]));
        uv.add(frame.keypoints[observation.feature].pt);
        usedTracks.add(track);
      }
    }
    if (xyz.size() < 8) {
      if (project != null) project.data.pnp(frame, usedTracks, uv, new int[0], null, null, false);
      return false;
    }
    MatOfPoint3f points = new MatOfPoint3f();
    points.fromList(xyz);
    MatOfPoint2f pixels = new MatOfPoint2f();
    pixels.fromList(uv);
    Mat k = frame.camera.intrinsic(), r = new Mat(), t = new Mat(), inliers = new Mat();
    MatOfDouble d = new MatOfDouble();
    try {
      if (!Calib3d.solvePnPRansac(
              points,
              pixels,
              k,
              d,
              r,
              t,
              false,
              1000,
              (float) o.reprojection,
              .999,
              inliers,
              Calib3d.SOLVEPNP_EPNP)
          || inliers.total() < Math.max(8, o.minMatches / 2)) return false;
      List<Point3> good3 = new ArrayList<>();
      List<Point> good2 = new ArrayList<>();
      int[] indices = new int[(int) inliers.total()];
      inliers.get(0, 0, indices);
      for (int index : indices) {
        good3.add(xyz.get(index));
        good2.add(uv.get(index));
      }
      points.fromList(good3);
      pixels.fromList(good2);
      Calib3d.solvePnPRefineLM(points, pixels, k, d, r, t);
      Camera candidate = frame.camera.copy();
      candidate.rotation = Math3.rotation(Math3.array(r));
      candidate.translation = Math3.array(t);
      for (double value : candidate.translation) if (!Double.isFinite(value)) return false;
      for (double value : candidate.rotation) if (!Double.isFinite(value)) return false;
      int consistent = 0;
      for (int i = 0; i < good3.size(); i++) {
        Point3 p = good3.get(i);
        double[] predicted = candidate.project(new double[] {p.x, p.y, p.z});
        Point observed = good2.get(i);
        if (predicted != null
            && Math.hypot(predicted[0] - observed.x, predicted[1] - observed.y)
                <= o.reprojection * 2) consistent++;
      }
      if (consistent < Math.max(8, o.minMatches / 2)) return false;
      frame.camera.rotation = candidate.rotation;
      frame.camera.translation = candidate.translation;
      frame.camera.registered = true;
      if (!quiet)
        System.out.printf(
            Locale.ROOT,
            "Registered %s: %d PnP inliers%n",
            frame.path.getFileName(),
            indices.length);
      return true;
    } catch (org.opencv.core.CvException e) {
      if (project != null)
        project.event(
            "registration_numerical_failure",
            Json.object("image_id", frame.id, "reason", e.getMessage()));
      return false;
    } finally {
      try {
        if (project != null) {
          int[] selected = new int[(int) inliers.total()];
          if (selected.length > 0) inliers.get(0, 0, selected);
          project.data.pnp(
              frame,
              usedTracks,
              uv,
              selected,
              r.empty() ? null : Math3.array(r),
              t.empty() ? null : Math3.array(t),
              frame.camera.registered);
        }
      } finally {
        points.release();
        pixels.release();
        k.release();
        r.release();
        t.release();
        inliers.release();
        d.release();
      }
    }
  }

  static FeatureTracks.Observation observation(FeatureTracks.Track track, int image) {
    for (FeatureTracks.Observation o : track.observations) if (o.image == image) return o;
    return null;
  }

  static void triangulateTracks(
      List<ImageFrame> frames, List<FeatureTracks.Track> tracks, double maxError) {
    for (FeatureTracks.Track track : tracks) {
      if (track.point != null) continue;
      FeatureTracks.Observation first = null, second = null;
      double baseline = 0;
      for (FeatureTracks.Observation a : track.observations)
        if (frames.get(a.image).camera.registered)
          for (FeatureTracks.Observation b : track.observations)
            if (b.image > a.image && frames.get(b.image).camera.registered) {
              double distance =
                  Math3.norm(
                      Math3.sub(
                          frames.get(a.image).camera.center(),
                          frames.get(b.image).camera.center()));
              if (distance > baseline) {
                baseline = distance;
                first = a;
                second = b;
              }
            }
      if (first == null) continue;
      ImageFrame fa = frames.get(first.image), fb = frames.get(second.image);
      Point ua = FeatureTracks.normalize(fa.camera, fa.keypoints[first.feature].pt),
          ub = FeatureTracks.normalize(fb.camera, fb.keypoints[second.feature].pt);
      double[] point = triangulate(fa.camera, fb.camera, ua, ub);
      if (point != null
          && baseline / Math.max(1e-9, Math3.norm(Math3.sub(point, fa.camera.center()))) > .002
          && valid(frames, track, point, maxError)) track.point = point;
    }
  }

  static double[] triangulate(Camera a, Camera b, Point u, Point v) {
    Mat p = a.projection(),
        q = b.projection(),
        matrix = new Mat(4, 4, CvType.CV_64F),
        w = new Mat(),
        left = new Mat(),
        vt = new Mat();
    try {
      double[] pa = Math3.array(p), pb = Math3.array(q), data = new double[16];
      for (int j = 0; j < 4; j++) {
        data[j] = u.x * pa[8 + j] - pa[j];
        data[4 + j] = u.y * pa[8 + j] - pa[4 + j];
        data[8 + j] = v.x * pb[8 + j] - pb[j];
        data[12 + j] = v.y * pb[8 + j] - pb[4 + j];
      }
      matrix.put(0, 0, data);
      Core.SVDecomp(matrix, w, left, vt);
      double[] h = new double[4];
      vt.get(3, 0, h);
      if (Math.abs(h[3]) < 1e-12) return null;
      double[] out = {h[0] / h[3], h[1] / h[3], h[2] / h[3]};
      return Double.isFinite(out[0] + out[1] + out[2]) ? out : null;
    } finally {
      p.release();
      q.release();
      matrix.release();
      w.release();
      left.release();
      vt.release();
    }
  }

  static boolean valid(
      List<ImageFrame> frames, FeatureTracks.Track track, double[] p, double threshold) {
    int count = 0;
    for (FeatureTracks.Observation observation : track.observations) {
      ImageFrame frame = frames.get(observation.image);
      if (!frame.camera.registered) continue;
      double[] predicted = frame.camera.project(p);
      Point actual = frame.keypoints[observation.feature].pt;
      if (predicted == null
          || Math.hypot(predicted[0] - actual.x, predicted[1] - actual.y) > threshold) return false;
      count++;
    }
    return count >= 2;
  }
}
