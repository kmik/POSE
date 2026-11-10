package sgm;

import java.io.*;
import java.nio.file.Path;
import java.util.*;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;

final class Geometry {
  private Geometry() {}

  private static final ThreadLocal<TriangulationWorkspace> TRIANGULATION = new ThreadLocal<>();
  private static final ThreadLocal<RegistrationWorkspace> REGISTRATION = new ThreadLocal<>();

  static final class TriangulationWorkspace {
    final Mat matrix = new Mat(4, 4, CvType.CV_64F), w = new Mat(), left = new Mat(), vt = new Mat();
    final double[] pa = new double[12], pb = new double[12], data = new double[16], h = new double[4];
    void release() { matrix.release(); w.release(); left.release(); vt.release(); }
  }

  private static final class RegistrationWorkspace {
    final MatOfPoint3f points = new MatOfPoint3f();
    final MatOfPoint2f pixels = new MatOfPoint2f();
    final Mat k = new Mat(3, 3, CvType.CV_64F), r = new Mat(), t = new Mat(), inliers = new Mat();
    final MatOfDouble d = new MatOfDouble();
    void release() { points.release(); pixels.release(); k.release(); r.release(); t.release(); inliers.release(); d.release(); }
  }

  static TriangulationWorkspace triangulator() {
    TriangulationWorkspace workspace = TRIANGULATION.get();
    if (workspace == null) { workspace = new TriangulationWorkspace(); TRIANGULATION.set(workspace); }
    return workspace;
  }

  static void releaseWorkspaces() {
    var tri = TRIANGULATION.get();
    if (tri != null) { tri.release(); TRIANGULATION.remove(); }
    var pnp = REGISTRATION.get();
    if (pnp != null) { pnp.release(); REGISTRATION.remove(); }
  }

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
    try { reconstructImpl(frames, features, o, project); }
    finally { releaseWorkspaces(); }
  }

  private static void reconstructImpl(
      List<ImageFrame> frames, FeatureTracks features, Options o, Project project)
      throws IOException {
    AlignmentMemory.checkGraph(frames, features.tracks, features.pairs, o);
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
    // Per-image track lists and triangulated-track counts (see TrackIndex): same choices as the
    // former whole-track scans per step, without images x tracks work per registration.
    TrackIndex index = new TrackIndex(frames, features.tracks, o);
    while (true) {
      int best = -1, most = 0;
      for (ImageFrame f : frames)
        if (!f.camera.registered) {
          int n = index.triangulated[f.id];
          if (n > most) {
            most = n;
            best = f.id;
          }
        }
      if (best < 0 || most < Math.max(8, o.minMatches / 2)) break;
      ImageFrame next = frames.get(best);
      int added = best;
      if (!register(next, index, o, project)) {
        // Try all currently registerable views before declaring the graph disconnected.
        boolean registered = false;
        for (ImageFrame candidate : frames)
          if (!candidate.camera.registered
              && candidate.id != best
              && register(candidate, index, o, project)) {
            registered = true;
            added = candidate.id;
            break;
          }
        if (!registered) break;
      }
      // Only the new camera changed since the previous triangulation (no pose moved).
      triangulateNew(frames, index, added, o.reprojection);
      if (project != null)
        project.data.snapshot(
            "registration_" + frames.stream().filter(f -> f.camera.registered).count(),
            frames,
            features.tracks,
            "seed-camera frame");
    }
    index = null; // released before the bundle adjustment
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
    // Rebuilt after the bundle and cleanup (they edit observations and move poses): the first
    // triangulation after them covers all tracks, later ones only the newly registered camera.
    TrackIndex retry = null;
    for (int pass = 0; pass < frames.size(); pass++) {
      boolean changed = false;
      for (ImageFrame frame : frames) {
        if (frame.camera.registered) continue;
        if (retry == null) retry = new TrackIndex(frames, features.tracks, o);
        if (register(frame, retry, o, project)) {
          if (!recovered) triangulateAll(frames, retry, o.reprojection);
          else triangulateNew(frames, retry, frame.id, o.reprojection);
          changed = true;
          recovered = true;
        }
      }
      if (!changed) break;
    }
    retry = null;
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
    // Observations do not change between trials (only points and poses are reset).
    TrackIndex trial = new TrackIndex(frames, features.tracks, o);
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
        trial.recount();
        Camera a = frames.get(pair.a).camera, b = frames.get(pair.b).camera;
        a.registered = true;
        b.registered = true;
        b.rotation = pair.rotation.clone();
        b.translation = Math3.times(pair.translation, o.baseline / Math3.norm(pair.translation));
        triangulateAll(frames, trial, o.reprojection);
        for (int pass = 0; pass < frames.size(); pass++) {
          boolean changed = false;
          for (ImageFrame f : frames)
            if (!f.camera.registered && register(f, features.tracks, trial, o, null, true)) {
              changed = true;
              // Only this camera changed since the previous triangulation (see triangulateNew).
              triangulateNew(frames, trial, f.id, o.reprojection);
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
    return register(frame, tracks, null, o, project, false);
  }

  private static boolean register(
      ImageFrame frame, TrackIndex index, Options o, Project project) throws IOException {
    return register(frame, index.tracks, index, o, project, false);
  }

  private static boolean register(
      ImageFrame frame, List<FeatureTracks.Track> tracks, TrackIndex index, Options o,
      Project project, boolean quiet)
      throws IOException {
    List<Point3> xyz = new ArrayList<>();
    List<Point> uv = new ArrayList<>();
    List<FeatureTracks.Track> usedTracks = new ArrayList<>();
    if (index != null) {
      // The image's own tracks, in the same (ascending) order as the full scan below.
      for (int t : index.byImage[frame.id]) {
        FeatureTracks.Track track = index.tracks.get(t);
        if (track.point == null) continue;
        FeatureTracks.Observation observation = observation(track, frame.id);
        double[] p = track.point;
        xyz.add(new Point3(p[0], p[1], p[2]));
        uv.add(frame.keypoints[observation.feature].pt);
        usedTracks.add(track);
      }
    } else
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
    RegistrationWorkspace workspace = REGISTRATION.get();
    if (workspace == null) { workspace = new RegistrationWorkspace(); REGISTRATION.set(workspace); }
    MatOfPoint3f points = workspace.points;
    points.fromList(xyz);
    MatOfPoint2f pixels = workspace.pixels;
    pixels.fromList(uv);
    Mat k = workspace.k, r = workspace.r, t = workspace.t, inliers = workspace.inliers;
    k.put(0, 0, new double[] {frame.camera.fx, 0, frame.camera.cx, 0, frame.camera.fy, frame.camera.cy, 0, 0, 1});
    // Failed solves must not expose a previous camera's outputs to diagnostics.
    r.release(); t.release(); inliers.release();
    MatOfDouble d = workspace.d;
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
      for (int inlier : indices) {
        good3.add(xyz.get(inlier));
        good2.add(uv.get(inlier));
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
      // Reuse native Mat headers across registrations. releaseWorkspaces() owns cleanup.
    }
  }

  /**
   * Tracks observed by each image (ascending track order, each track once per image) and, for the
   * incremental registration loops, how many of them are triangulated. Built from the current
   * observations; it must be rebuilt after anything that edits observations (bundle cleanup).
   *
   * <p>It replaces whole-track-list scans per registration step (choosing the next camera scanned
   * every unregistered image against every track: images^2 x tracks for a block) with the same
   * decisions from per-image lists: identical counts, track order and tie-breaking.
   */
  static final class TrackIndex {
    final List<FeatureTracks.Track> tracks;
    final int[][] byImage;
    final int[] triangulated;

    TrackIndex(List<ImageFrame> frames, List<FeatureTracks.Track> tracks, Options o) {
      this.tracks = tracks;
      int images = frames.size();
      int[] sizes = new int[images];
      long entries = 0;
      for (FeatureTracks.Track track : tracks)
        for (int i = 0; i < track.observations.size(); i++)
          if (first(track, i)) {
            sizes[track.observations.get(i).image]++;
            entries++;
          }
      MemorySafety.requireHeap(
          Math.addExact(Math.multiplyExact(entries, 4L), 40L * images), o, "Registration track index");
      byImage = new int[images][];
      for (int i = 0; i < images; i++) byImage[i] = new int[sizes[i]];
      int[] fill = new int[images];
      for (int t = 0; t < tracks.size(); t++) {
        FeatureTracks.Track track = tracks.get(t);
        for (int i = 0; i < track.observations.size(); i++)
          if (first(track, i)) {
            int image = track.observations.get(i).image;
            byImage[image][fill[image]++] = t;
          }
      }
      triangulated = new int[images];
      for (FeatureTracks.Track track : tracks) if (track.point != null) added(track);
    }

    /** Whether observation i is the track's first observation of its image. */
    private static boolean first(FeatureTracks.Track track, int i) {
      int image = track.observations.get(i).image;
      for (int j = 0; j < i; j++) if (track.observations.get(j).image == image) return false;
      return true;
    }

    /** A track just received its point: one more triangulated track for each image observing it. */
    void added(FeatureTracks.Track track) {
      for (int i = 0; i < track.observations.size(); i++)
        if (first(track, i)) triangulated[track.observations.get(i).image]++;
    }

    /** Points were cleared or restored outside the index: recount. */
    void recount() {
      java.util.Arrays.fill(triangulated, 0);
      for (FeatureTracks.Track track : tracks) if (track.point != null) added(track);
    }
  }

  /**
   * triangulateTracks restricted to the tracks observed by a camera that was just registered,
   * when no other pose or registration changed since the previous triangulation: a track not seen
   * by that camera has the same registered observations and poses as before, so it fails again
   * exactly as it did. Same per-track computation, same (ascending) order.
   */
  static void triangulateNew(
      List<ImageFrame> frames, TrackIndex index, int image, double maxError) {
    for (int t : index.byImage[image]) {
      FeatureTracks.Track track = index.tracks.get(t);
      if (track.point == null && triangulateTrack(frames, track, maxError)) index.added(track);
    }
  }

  /** triangulateTracks over all tracks, keeping the index counts. */
  static void triangulateAll(List<ImageFrame> frames, TrackIndex index, double maxError) {
    for (FeatureTracks.Track track : index.tracks)
      if (track.point == null && triangulateTrack(frames, track, maxError)) index.added(track);
  }

  static FeatureTracks.Observation observation(FeatureTracks.Track track, int image) {
    for (FeatureTracks.Observation o : track.observations) if (o.image == image) return o;
    return null;
  }

  static void triangulateTracks(
      List<ImageFrame> frames, List<FeatureTracks.Track> tracks, double maxError) {
    for (FeatureTracks.Track track : tracks)
      if (track.point == null) triangulateTrack(frames, track, maxError);
  }

  /** One track of triangulateTracks (point == null); true when it received a point. */
  static boolean triangulateTrack(
      List<ImageFrame> frames, FeatureTracks.Track track, double maxError) {
    {
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
      if (first == null) return false;
      ImageFrame fa = frames.get(first.image), fb = frames.get(second.image);
      Point ua = FeatureTracks.normalize(fa.camera, fa.keypoints[first.feature].pt),
          ub = FeatureTracks.normalize(fb.camera, fb.keypoints[second.feature].pt);
      double[] point = triangulate(fa.camera, fb.camera, ua, ub);
      if (point != null
          && baseline / Math.max(1e-9, Math3.norm(Math3.sub(point, fa.camera.center()))) > .002
          && valid(frames, track, point, maxError)) {
        track.point = point;
        return true;
      }
      return false;
    }
  }

  static double[] triangulate(Camera a, Camera b, Point u, Point v) {
    TriangulationWorkspace workspace = triangulator();
    double[] pa = workspace.pa, pb = workspace.pb, data = workspace.data;
    projection(a, pa);
    projection(b, pb);
    for (int j = 0; j < 4; j++) {
      data[j] = u.x * pa[8 + j] - pa[j];
      data[4 + j] = u.y * pa[8 + j] - pa[4 + j];
      data[8 + j] = v.x * pb[8 + j] - pb[j];
      data[12 + j] = v.y * pb[8 + j] - pb[4 + j];
    }
    workspace.matrix.put(0, 0, data);
    Core.SVDecomp(workspace.matrix, workspace.w, workspace.left, workspace.vt);
    double[] h = workspace.h;
    workspace.vt.get(3, 0, h);
    if (Math.abs(h[3]) < 1e-12) return null;
    double[] out = {h[0] / h[3], h[1] / h[3], h[2] / h[3]};
    return Double.isFinite(out[0] + out[1] + out[2]) ? out : null;
  }

  private static void projection(Camera camera, double[] out) {
    for (int i = 0; i < 3; i++) {
      System.arraycopy(camera.rotation, 3 * i, out, 4 * i, 3);
      out[4 * i + 3] = camera.translation[i];
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
