package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;

/** Select each image's strongest estimated neighbours; match the union once. */
final class PairSelection {
  record Candidate(int image, double score, String method) {}
  static final Comparator<Candidate> BEST = Comparator.comparingDouble(Candidate::score)
      .reversed().thenComparingInt(Candidate::image);

  static long maximumPairs(int n, Options o) {
    long all = (long) n * (n - 1) / 2;
    return o.pairSelection.equals("all") ? all : Math.min(all, (long) n * o.neighbors);
  }
  static long key(int a, int b) {
    return ((long) Math.min(a, b) << 32) | (Math.max(a, b) & 0xffffffffL);
  }
  private static void offer(List<PriorityQueue<Candidate>> choices, int a, int b,
      double score, String method, int k) {
    if (!(score > 0) || !Double.isFinite(score)) return;
    PriorityQueue<Candidate> q = choices.get(a);
    q.add(new Candidate(b, score, method));
    if (q.size() > k) q.poll();
  }

  static List<int[]> select(List<ImageFrame> frames, Options o, Project project)
      throws IOException {
    int n = frames.size(), k = Math.min(o.neighbors, Math.max(0, n - 1));
    Set<Long> edges = new TreeSet<>();
    List<PriorityQueue<Candidate>> choices = new ArrayList<>();
    for (int a = 0; a < n; a++) choices.add(new PriorityQueue<>(BEST.reversed()));
    String search = o.pairSelection;
    int evaluated = 0;
    if (search.equals("sequential")) {
      for (int a = 0; a < n; a++)
        for (int b = a + 1; b < n && b - a <= k; b++) edges.add(key(a, b));
    } else if (search.equals("all")) {
      for (int a = 0; a < n; a++)
        for (int b = a + 1; b < n; b++) edges.add(key(a, b));
    } else {
      Camera[] cameras = frames.stream().map(f -> f.camera).toArray(Camera[]::new);
      OverlapSelection.Plane plane = OverlapSelection.plane(cameras, List.of(), o);
      boolean footprints = search.equals("overlap") && plane != null
          && frames.stream().allMatch(f -> f.camera.registered);
      boolean gps = search.equals("gps") && frames.stream()
          .allMatch(f -> f.gps != null && f.gps.usable());
      if (footprints || gps) {
        search = footprints ? "ground_footprints" : "gps_proximity";
        for (int a = 0; a < n; a++) for (int b = a + 1; b < n; b++) {
          double score;
          if (footprints) {
            ImageFrame x = frames.get(a), y = frames.get(b);
            var e = OverlapSelection.estimate(x.camera, x.width, x.height,
                y.camera, y.width, y.height, plane);
            score = OverlapSelection.accepted(e, o) ? e.area() : 0;
          } else score = gpsScore(frames.get(a).gps, frames.get(b).gps);
          evaluated++;
          offer(choices, a, b, score, search, k);
          offer(choices, b, a, score, search, k);
        }
      } else {
        var visual = VisualPairSelection.scores(frames, o);
        search = visual.search();
        evaluated = visual.scores().size();
        for (var entry : visual.scores().entrySet()) {
          int a = (int) (entry.getKey() >>> 32), b = (int) (long) entry.getKey();
          offer(choices, a, b, entry.getValue(), "descriptor_preview", k);
          offer(choices, b, a, entry.getValue(), "descriptor_preview", k);
        }
      }
      for (int a = 0; a < n; a++)
        for (Candidate c : choices.get(a)) edges.add(key(a, c.image));
    }
    List<int[]> result = new ArrayList<>();
    for (long e : edges) result.add(new int[] {(int) (e >>> 32), (int) e});
    result = OverlapSelection.filter(frames, result, List.of(), o, "alignment");
    Set<Long> retained = new HashSet<>();
    for (int[] p : result) retained.add(key(p[0], p[1]));
    List<Object> report = new ArrayList<>();
    for (int a = 0; a < n; a++) {
      List<Candidate> sorted = new ArrayList<>(choices.get(a));
      sorted.sort(BEST);
      int rank = 0;
      for (Candidate c : sorted) report.add(Json.object("a", a, "b", c.image,
          "rank", ++rank, "score", c.score, "method", c.method,
          "retained", retained.contains(key(a, c.image))));
    }
    Path path = o.output.resolve("03_matches/pair_selection.json");
    Files.createDirectories(path.getParent());
    String rule = switch (o.pairSelection) {
      case "all" -> "Every pair";
      case "sequential" -> "Next K filename-sorted images";
      default -> "Union of each image's top K positive scores; incoming choices may increase degree";
    };
    Json.write(path, Json.object("mode", o.pairSelection, "search", search,
        "neighbors_per_image", o.pairSelection.equals("all") ? Math.max(0, n - 1) : k,
        "candidate_neighbors", o.pairCandidates,
        "sequential_fallback_neighbors", o.pairSelection.equals("sequential") ? k : 0,
        "evaluated_pairs", evaluated, "selected_pairs", result, "preview_scores", report,
        "selection_rule", rule,
        "score_units", "ground_footprints: percent; descriptor_preview: visual support and coverage, not percent; gps_proximity: inverse distance"));
    System.out.printf(Locale.ROOT, "Pair selection: %s, %d pairs (%d scored); %s%n",
        search, result.size(), evaluated, rule + " (K=" + k + ")");
    return result;
  }
  private static double gpsScore(Exif.Gps x, Exif.Gps y) {
    double lat = Math.toRadians((x.latitude + y.latitude) / 2);
    return 1 / Math.max(1e-12, Math.hypot((x.longitude - y.longitude) * Math.cos(lat),
        x.latitude - y.latitude));
  }
}
