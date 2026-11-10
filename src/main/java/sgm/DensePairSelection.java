package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;

/** Experimental sparse-coverage selection. Unknown support always retains the original edge. */
final class DensePairSelection {
  private record Candidate(
      FeatureTracks.Pair pair, BitSet cells, double cost, int samples, int useful,
      double medianAngle, boolean unknown) {}

  static List<FeatureTracks.Pair> select(
      List<ImageFrame> frames,
      List<FeatureTracks.Pair> pairs,
      List<FeatureTracks.Track> tracks,
      Options options)
      throws IOException {
    if (options.densePairSelection.equals("all")) return pairs;
    List<Candidate> candidates = new ArrayList<>();
    BitSet required = new BitSet();
    long observations = 0;
    for (var track : tracks) if (track.point != null) observations += track.observations.size();
    MemorySafety.requireHeap(Math.addExact(Math.multiplyExact(observations, 16L), 64L * frames.size()),
        options, "Dense pair-selection observation index");
    // Index once in original track order; avoid scanning the whole sparse graph for every edge.
    List<List<FeatureTracks.Track>> byImage = new ArrayList<>();
    for (int i = 0; i < frames.size(); i++) byImage.add(new ArrayList<>());
    BitSet seen = new BitSet(frames.size());
    for (var track : tracks) if (track.point != null) {
      seen.clear();
      for (var ob : track.observations) if (!seen.get(ob.image)) {
        seen.set(ob.image);
        byImage.get(ob.image).add(track);
      }
    }
    for (var pair : pairs) {
      Camera a = frames.get(pair.a).camera, b = frames.get(pair.b).camera;
      double baseline = Math3.norm(Math3.sub(a.center(), b.center()));
      double focal = Math.min(Math.min(a.fx, a.fy), Math.min(b.fx, b.fy));
      BitSet cells = new BitSet();
      BitSet sampledCells = new BitSet();
      List<Double> parallax = new ArrayList<>();
      List<Double> angles = new ArrayList<>();
      int useful = 0;
      double[] centerA = a.center(), centerB = b.center();
      List<FeatureTracks.Track> local = byImage.get(pair.a).size() <= byImage.get(pair.b).size()
          ? byImage.get(pair.a) : byImage.get(pair.b);
      for (var track : local) {
        boolean seenA = false, seenB = false;
        for (var ob : track.observations) {
          seenA |= ob.image == pair.a;
          seenB |= ob.image == pair.b;
        }
        if (!seenA || !seenB) continue;
        double[] p = a.project(track.point), q = b.project(track.point);
        if (!inside(p, frames.get(pair.a)) || !inside(q, frames.get(pair.b))) continue;
        sampledCells.set(cell(pair.a, p, frames.get(pair.a)));
        sampledCells.set(cell(pair.b, q, frames.get(pair.b)));
        double z = Math.min(a.view(track.point)[2], b.view(track.point)[2]);
        if (!(z > 0) || !Double.isFinite(z)) continue;
        double[] rayA = Math3.sub(track.point, centerA), rayB = Math3.sub(track.point, centerB);
        double cosine = (rayA[0] * rayB[0] + rayA[1] * rayB[1] + rayA[2] * rayB[2])
            / (Math3.norm(rayA) * Math3.norm(rayB));
        double angle = Math.toDegrees(Math.acos(Math.clamp(cosine, -1, 1)));
        if (!Double.isFinite(angle)) continue;
        angles.add(angle);
        if (angle >= options.minAngle) {
          useful++;
          cells.set(cell(pair.a, p, frames.get(pair.a)));
          cells.set(cell(pair.b, q, frames.get(pair.b)));
        }
        parallax.add(focal * baseline / z);
      }
      parallax.sort(Double::compare);
      angles.sort(Double::compare);
      boolean unknown = parallax.size() < 12 || sampledCells.cardinality() < 4;
      // Missing geometric evidence keeps the original pair and its sampled coverage.
      if (unknown) cells.or(sampledCells);
      double cost =
          unknown
              ? Double.POSITIVE_INFINITY
              : (32
                  + parallax.get((int) (parallax.size() * .95))
                  - parallax.get((int) (parallax.size() * .05)))
                  / Math.max(.05, useful / (double) parallax.size());
      candidates.add(new Candidate(pair, cells, cost, parallax.size(), useful,
          angles.isEmpty() ? Double.NaN : angles.get(angles.size() / 2), unknown));
      required.or(cells);
    }
    boolean[] selected = new boolean[pairs.size()];
    int[] degree = new int[frames.size()], available = new int[frames.size()];
    int[] cellCounts = new int[frames.size() * 64], cellTargets = new int[cellCounts.length];
    for (var c : candidates)
      for (int bit = c.cells.nextSetBit(0); bit >= 0; bit = c.cells.nextSetBit(bit + 1))
        cellTargets[bit]++;
    for (int bit = 0; bit < cellTargets.length; bit++)
      cellTargets[bit] = Math.min(cellTargets[bit], Math.max(1, options.minViews - 1));
    BitSet covered = new BitSet();
    for (int i = 0; i < candidates.size(); i++) {
      var c = candidates.get(i);
      available[c.pair.a]++;
      available[c.pair.b]++;
      if (c.unknown) take(i, candidates, selected, degree, covered, cellCounts, cellTargets);
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
      take(best, candidates, selected, degree, covered, cellCounts, cellTargets);
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
        take(i, candidates, selected, degree, covered, cellCounts, cellTargets);
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
              "samples_above_min_angle", c.useful,
              "median_triangulation_angle_degrees", c.medianAngle,
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
            "required_pair_witnesses_per_cell", Math.max(1, options.minViews - 1),
            "pairs",
            report,
            "scope",
            "Angle-qualified sparse grid coverage with distinct pair witnesses, angle-aware cost and original connectivity retained; not a guarantee of dense"
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
      int i, List<Candidate> candidates, boolean[] selected, int[] degree, BitSet covered,
      int[] cellCounts, int[] cellTargets) {
    if (selected[i]) return;
    selected[i] = true;
    var c = candidates.get(i);
    degree[c.pair.a]++;
    degree[c.pair.b]++;
    for (int bit = c.cells.nextSetBit(0); bit >= 0; bit = c.cells.nextSetBit(bit + 1))
      if (++cellCounts[bit] >= cellTargets[bit]) covered.set(bit);
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
