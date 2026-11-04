package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;

/** Experimental sparse-coverage selection. Unknown support always retains the original edge. */
final class DensePairSelection {
  private record Candidate(
      FeatureTracks.Pair pair, BitSet cells, double cost, int samples, boolean unknown) {}

  static List<FeatureTracks.Pair> select(
      List<ImageFrame> frames,
      List<FeatureTracks.Pair> pairs,
      List<FeatureTracks.Track> tracks,
      Options options)
      throws IOException {
    if (options.densePairSelection.equals("all")) return pairs;
    List<Candidate> candidates = new ArrayList<>();
    BitSet required = new BitSet();
    for (var pair : pairs) {
      Camera a = frames.get(pair.a).camera, b = frames.get(pair.b).camera;
      double baseline = Math3.norm(Math3.sub(a.center(), b.center()));
      double focal = Math.min(Math.min(a.fx, a.fy), Math.min(b.fx, b.fy));
      BitSet cells = new BitSet();
      List<Double> parallax = new ArrayList<>();
      for (var track : tracks) {
        if (track.point == null) continue;
        boolean seenA = false, seenB = false;
        for (var ob : track.observations) {
          seenA |= ob.image == pair.a;
          seenB |= ob.image == pair.b;
        }
        if (!seenA || !seenB) continue;
        double[] p = a.project(track.point), q = b.project(track.point);
        if (!inside(p, frames.get(pair.a)) || !inside(q, frames.get(pair.b))) continue;
        cells.set(cell(pair.a, p, frames.get(pair.a)));
        cells.set(cell(pair.b, q, frames.get(pair.b)));
        double z = Math.min(a.view(track.point)[2], b.view(track.point)[2]);
        parallax.add(focal * baseline / z);
      }
      parallax.sort(Double::compare);
      boolean unknown = parallax.size() < 12 || cells.cardinality() < 4;
      double cost =
          unknown
              ? Double.POSITIVE_INFINITY
              : 32
                  + parallax.get((int) (parallax.size() * .95))
                  - parallax.get((int) (parallax.size() * .05));
      candidates.add(new Candidate(pair, cells, cost, parallax.size(), unknown));
      required.or(cells);
    }
    boolean[] selected = new boolean[pairs.size()];
    int[] degree = new int[frames.size()], available = new int[frames.size()];
    BitSet covered = new BitSet();
    for (int i = 0; i < candidates.size(); i++) {
      var c = candidates.get(i);
      available[c.pair.a]++;
      available[c.pair.b]++;
      if (c.unknown) take(i, candidates, selected, degree, covered);
    }
    for (; ; ) {
      int best = -1;
      double bestScore = -1;
      for (int i = 0; i < candidates.size(); i++)
        if (!selected[i]) {
          var c = candidates.get(i);
          BitSet added = (BitSet) c.cells.clone();
          added.andNot(covered);
          int degreeGain =
              (degree[c.pair.a] < Math.min(options.densePairNeighbors, available[c.pair.a]) ? 1 : 0)
                  + (degree[c.pair.b] < Math.min(options.densePairNeighbors, available[c.pair.b])
                      ? 1
                      : 0);
          if (added.isEmpty() && degreeGain == 0) continue;
          double score = (added.cardinality() + degreeGain * 64.) / Math.max(1, c.cost);
          if (score > bestScore) {
            best = i;
            bestScore = score;
          }
        }
      if (best < 0) break;
      take(best, candidates, selected, degree, covered);
    }
    // Retain connectivity within every original component, including links between flight strips.
    int[] parent = new int[frames.size()];
    for (int i = 0; i < parent.length; i++) parent[i] = i;
    for (int i = 0; i < candidates.size(); i++)
      if (selected[i]) join(parent, pairs.get(i).a, pairs.get(i).b);
    List<Integer> order = new ArrayList<>();
    for (int i = 0; i < pairs.size(); i++) order.add(i);
    order.sort(Comparator.comparingDouble(i -> candidates.get(i).cost));
    for (int i : order) {
      var p = pairs.get(i);
      if (root(parent, p.a) != root(parent, p.b)) {
        take(i, candidates, selected, degree, covered);
        join(parent, p.a, p.b);
      }
    }
    List<FeatureTracks.Pair> result = new ArrayList<>();
    List<Object> report = new ArrayList<>();
    for (int i = 0; i < candidates.size(); i++) {
      var c = candidates.get(i);
      if (selected[i]) result.add(c.pair);
      report.add(
          Json.object(
              "a",
              c.pair.a,
              "b",
              c.pair.b,
              "selected",
              selected[i],
              "unknown_keep",
              c.unknown,
              "sparse_samples",
              c.samples,
              "coverage_cells",
              c.cells.cardinality(),
              "predicted_range_cost",
              c.cost));
    }
    Path file = options.output.resolve("05_stereo/dense_pair_selection.json");
    Files.createDirectories(file.getParent());
    Json.write(
        file,
        Json.object(
            "experimental",
            true,
            "input_pairs",
            pairs.size(),
            "selected_pairs",
            result.size(),
            "required_sparse_cells",
            required.cardinality(),
            "covered_sparse_cells",
            covered.cardinality(),
            "pairs",
            report,
            "scope",
            "Sparse grid coverage and original connectivity retained; not a guarantee of dense"
                + " thin-structure completeness. Unknown pairs are retained."));
    return result;
  }

  private static boolean inside(double[] p, ImageFrame f) {
    return p != null && p[0] >= 0 && p[1] >= 0 && p[0] < f.width && p[1] < f.height;
  }

  private static int cell(int image, double[] p, ImageFrame f) {
    return image * 64 + (int) (p[1] * 8 / f.height) * 8 + (int) (p[0] * 8 / f.width);
  }

  private static void take(
      int i, List<Candidate> candidates, boolean[] selected, int[] degree, BitSet covered) {
    if (selected[i]) return;
    selected[i] = true;
    var c = candidates.get(i);
    degree[c.pair.a]++;
    degree[c.pair.b]++;
    covered.or(c.cells);
  }

  private static int root(int[] p, int i) {
    while (p[i] != i) {
      p[i] = p[p[i]];
      i = p[i];
    }
    return i;
  }

  private static void join(int[] p, int a, int b) {
    p[root(p, a)] = root(p, b);
  }
}
