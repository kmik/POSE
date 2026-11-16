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
      boolean auto = search.equals("auto");
      boolean footprints = (auto || search.equals("overlap")) && plane != null
          && frames.stream().allMatch(f -> f.camera.registered);
      boolean gps = !footprints && (auto || search.equals("gps")) && frames.stream()
          .allMatch(f -> f.gps != null && f.gps.usable());
      // auto: GPS spread within its own noise (close-range, handheld) does not rank neighbours.
      if (gps && auto && gpsSpread(frames) < AUTO_GPS_SPREAD) {
        gps = false;
        System.out.printf(Locale.ROOT, "Pair selection auto: GPS positions spread %.1f m (< %.0f m); using visual search%n",
            gpsSpread(frames), AUTO_GPS_SPREAD);
      }
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
    int gpsAdded = 0;
    if (o.gpsNeighbors > 0 && !o.pairSelection.equals("all")) gpsAdded = addGpsNeighbors(frames, edges, o.gpsNeighbors);
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
        "gps_neighbors", o.gpsNeighbors, "gps_neighbor_pairs_added", gpsAdded,
        "evaluated_pairs", evaluated, "selected_pairs", result, "preview_scores", report,
        "selection_rule", rule,
        "score_units", "ground_footprints: percent; descriptor_preview: visual support and coverage, not percent; gps_proximity: inverse distance"));
    System.out.printf(Locale.ROOT, "Pair selection: %s, %d pairs (%d scored); %s%n",
        search, result.size(), evaluated, rule + " (K=" + k + ")");
    if (o.gpsNeighbors > 0)
      System.out.printf(Locale.ROOT, "Pair selection: --gps-neighbors %d added %d pairs%s%n",
          o.gpsNeighbors, gpsAdded, gpsAdded == 0 && !o.pairSelection.equals("all")
              && !frames.stream().allMatch(f -> f.gps != null && f.gps.usable())
              ? " (not every image has GPS)" : "");
    return result;
  }

  /**
   * Adds each image's {@code count} nearest GPS neighbours (horizontal distance) to {@code edges};
   * returns the number of new pairs. Nothing is added unless every image has usable GPS.
   */
  static int addGpsNeighbors(List<ImageFrame> frames, Set<Long> edges, int count) {
    int n = frames.size();
    if (n < 2 || !frames.stream().allMatch(f -> f.gps != null && f.gps.usable())) return 0;
    return addNearest(MatchRepair.positions(frames), edges, count);
  }

  /** Adds each position's {@code count} nearest others (horizontal); returns the new pairs. */
  static int addNearest(double[][] enu, Set<Long> edges, int count) {
    int n = enu.length, added = 0;
    Integer[] order = new Integer[n];
    for (int a = 0; a < n; a++) {
      for (int b = 0; b < n; b++) order[b] = b;
      final int self = a;
      Arrays.sort(order, Comparator.comparingDouble((Integer b) ->
          Math.hypot(enu[b][0] - enu[self][0], enu[b][1] - enu[self][1])).thenComparingInt(b -> b));
      int taken = 0;
      for (int i = 0; i < n && taken < count; i++) {
        int b = order[i];
        if (b == a) continue;
        taken++;
        if (edges.add(key(a, b))) added++;
      }
    }
    return added;
  }
  /** auto uses GPS ranking only when the positions spread at least this much (RMS, metres). */
  static final double AUTO_GPS_SPREAD = 20;

  /** RMS horizontal distance of the GPS positions to their centroid, in metres. */
  static double gpsSpread(List<ImageFrame> frames) {
    double[][] enu = MatchRepair.positions(frames);
    return spread(enu);
  }

  static double spread(double[][] enu) {
    if (enu.length == 0) return 0;
    double x = 0, y = 0, s = 0;
    for (double[] p : enu) { x += p[0] / enu.length; y += p[1] / enu.length; }
    for (double[] p : enu) s += ((p[0] - x) * (p[0] - x) + (p[1] - y) * (p[1] - y)) / enu.length;
    return Math.sqrt(s);
  }

  private static double gpsScore(Exif.Gps x, Exif.Gps y) {
    double lat = Math.toRadians((x.latitude + y.latitude) / 2);
    return 1 / Math.max(1e-12, Math.hypot((x.longitude - y.longitude) * Math.cos(lat),
        x.latitude - y.latitude));
  }
}
