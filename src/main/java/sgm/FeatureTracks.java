package sgm;

import java.io.*;
import java.util.*;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;

final class FeatureTracks {
  static final class Match {
    final int a, b;

    Match(int a, int b) {
      this.a = a;
      this.b = b;
    }
  }

  static final class Pair {
    final int a, b;
    final List<Match> matches = new ArrayList<>();
    double[] rotation, translation;
    double[] essential;
    String reason = "not_estimated";

    Pair(int a, int b) {
      this.a = a;
      this.b = b;
    }
  }

  static final class Observation {
    final int image, feature;

    Observation(int image, int feature) {
      this.image = image;
      this.feature = feature;
    }
  }

  static final class Track {
    final List<Observation> observations = new ArrayList<>();
    double[] point;
    int id = -1;
    // Optional known position (ground control point) and its per-axis sigma, in world units.
    double[] prior, priorSigma;
  }

  final List<Pair> pairs = new ArrayList<>();
  final List<Track> tracks = new ArrayList<>();
  Pair seed;

  void build(List<ImageFrame> frames, Options o) {
    try {
      build(frames, o, null);
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  void build(List<ImageFrame> frames, Options o, Project project) throws IOException {
    if (project != null)
      project.stage(
          "02_features",
          "running",
          o.matchingAlgorithm + " extraction on processed undistorted images");
    int[] offset = new int[frames.size() + 1];
    int chunk = Math.max(1, o.cores);
    WorkProgress extraction = WorkProgress.start("Feature extraction", frames.size(), "images");
    for (int i = 0; i < frames.size(); i++) {
      extraction.update(i);
      // F1: decode the next chunk's rasters in parallel (identical pixels). Fingerprinting, cache
      // reuse, extraction and output below stay serial and in the original order.
      if (i % chunk == 0 && chunk > 1) {
        long preloadStarted = System.nanoTime();
        int decoded =
            RasterCache.preload(frames.subList(i, Math.min(frames.size(), i + chunk)), chunk);
        if (decoded > 0)
          RunTelemetry.record("Parallel feature image preload (" + decoded + ")", preloadStarted);
      }
      try (WorkProgress work =
          WorkProgress.start(
              "Feature extraction " + frames.get(i).path.getFileName(), 0, "keypoints")) {
        ReuseCache.features(frames.get(i), o);
        work.done();
      }
      frames.get(i).captureFeatureColors();
      if (project != null) project.data.features(frames.get(i));
      DescriptorCache.retain(frames.get(i));
      offset[i + 1] = Math.addExact(offset[i], frames.get(i).keypoints.length);
      System.out.printf(
          Locale.ROOT,
          "Features %d/%d: %d%n",
          i + 1,
          frames.size(),
          frames.get(i).keypoints.length);
    }
    extraction.done();
    extraction.close();
    if (project != null && ReuseCache.matches(frames, this, o, false)) {
      PairMatching.clear();
      if (project != null) {
        project.stage(
            "02_features",
            "complete",
            "Keypoints exported; compatible feature caches reused where available");
        project.stage(
            "03_matches",
            "complete",
            "Verified cached matches, tracks and original artifacts reused");
      }
      return;
    }
    pairs.clear();
    tracks.clear();
    UnionFind uf = new UnionFind(offset[frames.size()]);
    if (project != null) {
      project.stage("02_features", "complete", "All keypoints and descriptors exported");
      project.stage("03_matches", "running", "Reciprocal matching, RANSAC and track building");
    }
    int reusable = PairMatching.remembered();
    int[] reused = {0};
    List<int[]> edges = PairSelection.select(frames, o, project);
    WorkProgress matching = WorkProgress.start("Matching image pairs", edges.size(), "pairs");
    long[] consumed = {0};
    PairMatching.ordered(
        edges,
        PairMatching.threads(o),
        edge -> {
          try (WorkProgress work =
              WorkProgress.start("Match/verify pair " + edge[0] + "-" + edge[1], 0, "matches")) {
            Result result = match(frames.get(edge[0]), frames.get(edge[1]), o, work);
            work.done();
            return result;
          }
        },
        (edge, result) -> {
          int a = edge[0], b = edge[1];
          Pair pair = result.pair;
          matching.update(++consumed[0]);
          matching.detail("pair " + a + "-" + b);
          try {
            // Exports and track joins stay on this thread, in the original pair order.
            if (project != null)
              project.data.matches(
                  frames.get(a),
                  frames.get(b),
                  pair,
                  result.forward,
                  result.reverse,
                  result.ransacFlags,
                  result.cheiralityFlags);
          } finally {
            for (Mat m : result.forward) m.release();
          }
          if (result.reused) reused[0]++;
          if (pair.matches.size() < o.minMatches) return;
          pairs.add(pair);
          for (Match m : pair.matches) uf.join(offset[a] + m.a, offset[b] + m.b);
          System.out.printf(
              Locale.ROOT, "Pair %d-%d: %d geometric inliers%n", a, b, pair.matches.size());
        });
    matching.done();
    matching.close();
    if (reusable > 0)
      System.out.printf(
          Locale.ROOT,
          "Pair matching: %d descriptor searches reused from focal initialization (%d available)%n",
          reused[0],
          reusable);
    if (project != null) PairMatching.clear();
    Map<Integer, Track> groups = new TreeMap<>();
    for (int i = 0; i < frames.size(); i++)
      for (int k = 0; k < frames.get(i).keypoints.length; k++) {
        int root = uf.find(offset[i] + k);
        groups.computeIfAbsent(root, r -> new Track()).observations.add(new Observation(i, k));
      }
    int conflicts = 0, singletons = 0;
    for (Track track : groups.values()) {
      Set<Integer> seen = new HashSet<>();
      boolean conflict = false;
      for (Observation observation : track.observations)
        if (!seen.add(observation.image)) {
          conflict = true;
          break;
        }
      if (conflict) conflicts++;
      else if (track.observations.size() < 2) singletons++;
      else {
        track.id = tracks.size();
        tracks.add(track);
      }
    }
    if (project != null) {
      project.data.tracks(tracks, groups, conflicts, singletons);
      project.stage(
          "03_matches",
          "complete",
          pairs.size() + " accepted stereo pairs; " + tracks.size() + " retained tracks");
    }
    if (project != null) ReuseCache.matches(frames, this, o, true);
  }

  /** One verified pair plus the nearest-neighbor decisions its export needs. */
  private static final class Result {
    final Pair pair;
    final List<MatOfDMatch> forward = new ArrayList<>();
    final Map<Integer, Integer> reverse = new HashMap<>();
    final Set<Integer> ransacFlags = new HashSet<>(), cheiralityFlags = new HashSet<>();
    final boolean[] reusedSearch = new boolean[1];
    boolean reused;

    Result(Pair pair) {
      this.pair = pair;
    }
  }

  private static Result match(ImageFrame a, ImageFrame b, Options o, WorkProgress work)
      throws IOException {
    DescriptorCache.acquire(a, b);
    Result result = new Result(new Pair(a.id, b.id));
    try {
      verify(a, b, o, work, result);
      result.reused = result.reusedSearch[0];
      return result;
    } catch (RuntimeException | Error e) {
      for (Mat m : result.forward) m.release();
      throw e;
    } finally {
      DescriptorCache.release(a, b);
    }
  }

  private static void verify(ImageFrame a, ImageFrame b, Options o, WorkProgress work, Result res) {
    Pair out = res.pair;
    if (a.descriptors.empty() || b.descriptors.rows() < 2) {
      out.reason = "missing_descriptors";
      return;
    }
    List<MatOfDMatch> forward = res.forward;
    Map<Integer, Integer> reverse = res.reverse;
    Set<Integer> ransacFlags = res.ransacFlags, cheiralityFlags = res.cheiralityFlags;
    Mat p1 = new Mat(),
        p2 = new Mat(),
        mask = new Mat(),
        e = new Mat(),
        r = new Mat(),
        t = new Mat();
    try {
      PairMatching.reciprocal(
          o, a, b, forward, reverse, work::detail, false, res.reusedSearch);
      work.detail("essential RANSAC and cheirality");
      List<Match> accepted = new ArrayList<>();
      List<Point> x = new ArrayList<>(), y = new ArrayList<>();
      for (MatOfDMatch row : forward) {
        DMatch[] m = row.toArray();
        if (m.length < 2 || m[0].distance >= o.ratio * m[1].distance) continue;
        DMatch best = m[0];
        if (!Objects.equals(reverse.get(best.trainIdx), best.queryIdx)) continue;
        accepted.add(new Match(best.queryIdx, best.trainIdx));
        x.add(normalize(a.camera, a.keypoints[best.queryIdx].pt));
        y.add(normalize(b.camera, b.keypoints[best.trainIdx].pt));
      }
      if (accepted.size() < o.minMatches) {
        out.reason = "insufficient_reciprocal_ratio_matches";
        return;
      }
      p1.release();
      p2.release();
      p1 = new MatOfPoint2f();
      p2 = new MatOfPoint2f();
      ((MatOfPoint2f) p1).fromList(x);
      ((MatOfPoint2f) p2).fromList(y);
      double threshold = o.reprojection / Math.max(a.camera.fx, b.camera.fx);
      e.release();
      e =
          Calib3d.findEssentialMat(
              p1, p2, 1.0, new Point(0, 0), Calib3d.RANSAC, .999, threshold, 2000, mask);
      if (e.empty() || e.rows() != 3) {
        out.reason = "essential_estimation_failed";
        return;
      }
      out.essential = Math3.array(e);
      byte[] ransacMask = new byte[(int) mask.total()];
      mask.get(0, 0, ransacMask);
      for (int i = 0; i < accepted.size(); i++)
        if (ransacMask[i] != 0) ransacFlags.add(accepted.get(i).a);
      Calib3d.recoverPose(e, p1, p2, r, t, 1.0, new Point(0, 0), mask);
      byte[] inliers = new byte[(int) mask.total()];
      mask.get(0, 0, inliers);
      for (int i = 0; i < accepted.size(); i++)
        if (inliers[i] != 0) {
          out.matches.add(accepted.get(i));
          cheiralityFlags.add(accepted.get(i).a);
        }
      out.rotation = Math3.array(r);
      out.translation = Math3.array(t);
      out.reason =
          out.matches.size() >= o.minMatches ? "accepted" : "insufficient_geometric_inliers";
    } finally {
      p1.release();
      p2.release();
      mask.release();
      e.release();
      r.release();
      t.release();
    }
  }

  static Point normalize(Camera c, Point p) {
    return new Point((p.x - c.cx) / c.fx, (p.y - c.cy) / c.fy);
  }

  private static final class UnionFind {
    final int[] parent, size;

    UnionFind(int n) {
      parent = new int[n];
      size = new int[n];
      for (int i = 0; i < n; i++) {
        parent[i] = i;
        size[i] = 1;
      }
    }

    int find(int x) {
      while (x != parent[x]) {
        parent[x] = parent[parent[x]];
        x = parent[x];
      }
      return x;
    }

    void join(int a, int b) {
      a = find(a);
      b = find(b);
      if (a == b) return;
      if (size[a] < size[b]) {
        int s = a;
        a = b;
        b = s;
      }
      parent[b] = a;
      size[a] += size[b];
    }
  }
}
