package sgm;

import java.util.*;

/**
 * --match-repair: after pair matching, images with few accepted pairs and images outside the
 * largest connected part of the match graph are matched against their nearest images that were
 * not tried yet (GPS distance when both images have usable GPS, else filename order). Only the
 * weak part of the graph is touched, so the cost is a few extra pairs per weak image.
 */
final class MatchRepair {
  private MatchRepair() {}

  /** Why an image is weak, and the extra pairs chosen for it. */
  record Weak(int image, String reason, int acceptedPairs, int component, List<Integer> candidates) {}

  record Plan(List<Weak> weak, List<int[]> edges, int components, int largestComponent) {}

  /** Connected components of the accepted-pair graph: component id per image (ids by size, 0 = largest). */
  static int[] components(int images, List<FeatureTracks.Pair> accepted) {
    int[] parent = new int[images];
    for (int i = 0; i < images; i++) parent[i] = i;
    for (FeatureTracks.Pair p : accepted) union(parent, p.a, p.b);
    Map<Integer, List<Integer>> groups = new HashMap<>();
    for (int i = 0; i < images; i++) groups.computeIfAbsent(find(parent, i), k -> new ArrayList<>()).add(i);
    List<List<Integer>> sorted = new ArrayList<>(groups.values());
    // Largest first; ties by lowest image id, so the numbering is deterministic.
    sorted.sort(Comparator.comparingInt((List<Integer> g) -> -g.size()).thenComparingInt(g -> g.get(0)));
    int[] id = new int[images];
    for (int c = 0; c < sorted.size(); c++) for (int i : sorted.get(c)) id[i] = c;
    return id;
  }

  private static int find(int[] parent, int x) {
    while (parent[x] != x) {
      parent[x] = parent[parent[x]];
      x = parent[x];
    }
    return x;
  }

  private static void union(int[] parent, int a, int b) {
    a = find(parent, a);
    b = find(parent, b);
    if (a != b) parent[Math.max(a, b)] = Math.min(a, b);
  }

  static Plan plan(
      List<ImageFrame> frames, List<FeatureTracks.Pair> accepted, List<int[]> evaluated, Options o) {
    int n = frames.size();
    int[] degree = new int[n];
    for (FeatureTracks.Pair p : accepted) {
      degree[p.a]++;
      degree[p.b]++;
    }
    int[] component = components(n, accepted);
    int components = 0, largest = 0;
    for (int c : component) components = Math.max(components, c + 1);
    for (int c : component) if (c == 0) largest++;
    Set<Long> tried = new HashSet<>();
    for (int[] e : evaluated) tried.add(PairSelection.key(e[0], e[1]));
    boolean gps = frames.stream().allMatch(f -> f.gps != null && f.gps.usable());
    double[][] position = gps ? positions(frames) : null;
    List<Weak> weak = new ArrayList<>();
    Set<Long> chosen = new TreeSet<>();
    for (int w = 0; w < n; w++) {
      String reason =
          component[w] != 0
              ? (degree[w] == 0 ? "no_accepted_pairs" : "outside_largest_component")
              : degree[w] < o.matchRepairMinPairs ? "few_accepted_pairs" : null;
      if (reason == null) continue;
      int self = w;
      List<Integer> others = new ArrayList<>();
      for (int i = 0; i < n; i++) if (i != w && !tried.contains(PairSelection.key(w, i))) others.add(i);
      Comparator<Integer> nearest =
          gps
              ? Comparator.comparingDouble((Integer i) -> distance(position[self], position[i]))
              : Comparator.comparingInt((Integer i) -> Math.abs(i - self));
      others.sort(nearest.thenComparingInt(i -> i));
      // Outside the main part: prefer candidates that are in it, so the repair can connect.
      if (component[w] != 0) {
        List<Integer> main = new ArrayList<>(), rest = new ArrayList<>();
        for (int i : others) (component[i] == 0 ? main : rest).add(i);
        int half = Math.max(1, o.matchRepairCandidates / 2);
        List<Integer> merged = new ArrayList<>(main.subList(0, Math.min(half, main.size())));
        for (int i : others) if (merged.size() < o.matchRepairCandidates && !merged.contains(i)) merged.add(i);
        others = merged;
      }
      List<Integer> picked = new ArrayList<>(others.subList(0, Math.min(o.matchRepairCandidates, others.size())));
      for (int i : picked) chosen.add(PairSelection.key(w, i));
      weak.add(new Weak(w, reason, degree[w], component[w], picked));
    }
    List<int[]> edges = new ArrayList<>();
    for (long e : chosen) edges.add(new int[] {(int) (e >>> 32), (int) e});
    return new Plan(weak, edges, components, largest);
  }

  /** Local east/north/up metres from GPS (equirectangular about the block centre). */
  static double[][] positions(List<ImageFrame> frames) {
    double lat = 0, lon = 0;
    for (ImageFrame f : frames) {
      lat += f.gps.latitude;
      lon += f.gps.longitude;
    }
    lat /= frames.size();
    lon /= frames.size();
    double[][] out = new double[frames.size()][];
    for (int i = 0; i < frames.size(); i++) out[i] = enu(frames.get(i).gps, lat, lon);
    return out;
  }

  static double[] enu(Exif.Gps g, double lat0, double lon0) {
    double r = 6371008.8;
    return new double[] {
      Math.toRadians(g.longitude - lon0) * r * Math.cos(Math.toRadians(lat0)),
      Math.toRadians(g.latitude - lat0) * r,
      g.altitude
    };
  }

  static double distance(double[] a, double[] b) {
    // Horizontal distance: barometric heights are noisy and images of one block share a height.
    return Math.hypot(a[0] - b[0], a[1] - b[1]);
  }
}
