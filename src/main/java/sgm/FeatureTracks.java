package sgm;

import java.io.*;
import java.util.*;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.features2d.DescriptorMatcher;

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
    for (int i = 0; i < frames.size(); i++) {
      ReuseCache.features(frames.get(i), o);
      if (project != null) project.data.features(frames.get(i));
      offset[i + 1] = Math.addExact(offset[i], frames.get(i).keypoints.length);
      System.out.printf(
          Locale.ROOT,
          "Features %d/%d: %d%n",
          i + 1,
          frames.size(),
          frames.get(i).keypoints.length);
    }
    if (project != null && ReuseCache.matches(frames, this, o, false)) {
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
    for (int[] edge : PairSelection.select(frames, o, project)) {
      int a = edge[0], b = edge[1];
      Pair pair = match(frames.get(a), frames.get(b), o, project);
      if (pair.matches.size() < o.minMatches) continue;
      pairs.add(pair);
      for (Match m : pair.matches) uf.join(offset[a] + m.a, offset[b] + m.b);
      System.out.printf(
          Locale.ROOT, "Pair %d-%d: %d geometric inliers%n", a, b, pair.matches.size());
    }
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

  private static Pair match(ImageFrame a, ImageFrame b, Options o, Project project)
      throws IOException {
    Pair out = new Pair(a.id, b.id);
    if (a.descriptors.empty() || b.descriptors.rows() < 2) {
      out.reason = "missing_descriptors";
      if (project != null)
        project.data.matches(
            a,
            b,
            out,
            Collections.emptyList(),
            Collections.emptyMap(),
            Collections.emptySet(),
            Collections.emptySet());
      return out;
    }
    DescriptorMatcher matcher = FeatureAlgorithms.matcher(o, a.descriptors);
    List<MatOfDMatch> forward = new ArrayList<>(), backward = new ArrayList<>();
    Map<Integer, Integer> reverse = new HashMap<>();
    Set<Integer> ransacFlags = new HashSet<>(), cheiralityFlags = new HashSet<>();
    Mat p1 = new Mat(),
        p2 = new Mat(),
        mask = new Mat(),
        e = new Mat(),
        r = new Mat(),
        t = new Mat();
    try {
      matcher.knnMatch(a.descriptors, b.descriptors, forward, 2);
      matcher.knnMatch(b.descriptors, a.descriptors, backward, 2);
      for (MatOfDMatch row : backward) {
        DMatch[] m = row.toArray();
        if (m.length >= 2 && m[0].distance < o.ratio * m[1].distance)
          reverse.put(m[0].queryIdx, m[0].trainIdx);
      }
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
        return out;
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
        return out;
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
      return out;
    } finally {
      try {
        if (project != null)
          project.data.matches(a, b, out, forward, reverse, ransacFlags, cheiralityFlags);
      } finally {
        for (Mat m : forward) m.release();
        for (Mat m : backward) m.release();
        p1.release();
        p2.release();
        mask.release();
        e.release();
        r.release();
        t.release();
      }
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
