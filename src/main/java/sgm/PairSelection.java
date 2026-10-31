package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;
import org.opencv.features2d.*;

/** Deterministic preview shortlist; sequential edges preserve a capture-order fallback graph. */
final class PairSelection {
  static List<int[]> select(List<ImageFrame> frames, Options o, Project project)
      throws IOException {
    int n = frames.size();
    Set<Long> edges = new TreeSet<>();
    List<Object> report = new ArrayList<>();
    for (int a = 0; a < n; a++)
      for (int b = a + 1; b < n && b - a <= o.neighbors; b++) edges.add(key(a, b));
    if (!o.pairSelection.equals("sequential")) {
      double[][] scores = new double[n][n];
      for (int a = 0; a < n; a++)
        for (int b = a + 1; b < n; b++) {
          double score = 0;
          String method = o.pairSelection;
          if (o.pairSelection.equals("all")) score = 1;
          else if (o.pairSelection.equals("gps")
              && frames.get(a).gps != null
              && frames.get(b).gps != null
              && frames.get(a).gps.usable()
              && frames.get(b).gps.usable()) {
            Exif.Gps x = frames.get(a).gps, y = frames.get(b).gps;
            double lat = Math.toRadians((x.latitude + y.latitude) / 2),
                dx = (x.longitude - y.longitude) * Math.cos(lat),
                dy = x.latitude - y.latitude;
            score = 1 / Math.max(1e-12, Math.hypot(dx, dy));
          } else {
            method = "descriptor_preview";
            score = preview(frames.get(a), frames.get(b), o);
          }
          scores[a][b] = scores[b][a] = score;
          report.add(Json.object("a", a, "b", b, "score", score, "method", method));
          if (o.pairSelection.equals("all")) edges.add(key(a, b));
        }
      for (int a = 0; a < n; a++) {
        final int image = a;
        List<Integer> ids = new ArrayList<>();
        for (int b = 0; b < n; b++) if (b != a) ids.add(b);
        ids.sort(
            (x, y) -> {
              int c = Double.compare(scores[image][y], scores[image][x]);
              return c != 0 ? c : Integer.compare(x, y);
            });
        for (int k = 0; k < Math.min(o.pairCandidates, ids.size()); k++)
          if (scores[a][ids.get(k)] > 0) edges.add(key(a, ids.get(k)));
      }
    }
    List<int[]> result = new ArrayList<>();
    for (long e : edges) result.add(new int[] {(int) (e >>> 32), (int) e});
    Path path = o.output.resolve("03_matches/pair_selection.json");
    Files.createDirectories(path.getParent());
    Json.write(
        path,
        Json.object(
            "mode",
            o.pairSelection,
            "sequential_fallback_neighbors",
            o.neighbors,
            "candidate_neighbors",
            o.pairCandidates,
            "selected_pairs",
            result,
            "preview_scores",
            report));
    return OverlapSelection.filter(frames, result, List.of(), o, "alignment");
  }

  private static long key(int a, int b) {
    return ((long) Math.min(a, b) << 32) | (Math.max(a, b) & 0xffffffffL);
  }

  private static int preview(ImageFrame a, ImageFrame b, Options o) throws IOException {
    DescriptorCache.pair(a, b);
    if (a.descriptors.rows() < 2 || b.descriptors.rows() < 2) return 0;
    Mat x = a.descriptors.rowRange(0, Math.min(600, a.descriptors.rows())),
        y = b.descriptors.rowRange(0, Math.min(600, b.descriptors.rows()));
    DescriptorMatcher matcher = FeatureAlgorithms.matcher(o, x);
    List<MatOfDMatch> fw = new ArrayList<>(), bw = new ArrayList<>();
    try {
      matcher.knnMatch(x, y, fw, 2);
      matcher.knnMatch(y, x, bw, 2);
      Map<Integer, Integer> reverse = new HashMap<>();
      for (var row : bw) {
        DMatch[] m = row.toArray();
        if (m.length >= 2 && m[0].distance < o.ratio * m[1].distance)
          reverse.put(m[0].queryIdx, m[0].trainIdx);
      }
      int count = 0;
      for (var row : fw) {
        DMatch[] m = row.toArray();
        if (m.length >= 2
            && m[0].distance < o.ratio * m[1].distance
            && Objects.equals(reverse.get(m[0].trainIdx), m[0].queryIdx)) count++;
      }
      return count;
    } finally {
      x.release();
      y.release();
      for (Mat m : fw) m.release();
      for (Mat m : bw) m.release();
      matcher.clear();
    }
  }
}
