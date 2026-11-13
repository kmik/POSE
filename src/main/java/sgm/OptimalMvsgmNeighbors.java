package sgm;

import java.util.*;

/** Opt-in greedy coverage/diversity selection; the existing angle score remains the prior. */
final class OptimalMvsgmNeighbors {
  private static final int GRID = 8;
  record Selection(List<MultiViewSgm.Neighbor> neighbors, Map<String, Object> report) {}
  private record Candidate(
      MultiViewSgm.Neighbor neighbor, BitSet cells, double direction, double[] centre) {}

  static Selection select(List<ImageFrame> frames, List<FeatureTracks.Track> tracks,
      int reference, int count, int witnesses) {
    // Same eligibility, scores, ordering and relative score floor as the current selector.
    List<MultiViewSgm.Neighbor> eligible =
        MultiViewSgm.neighbours(frames, tracks, reference, Integer.MAX_VALUE);
    int target = Math.max(1, Math.min(count, witnesses));
    ImageFrame ref = frames.get(reference);
    double[] centre = ref.camera.center();
    Map<Integer, Candidate> byId = new HashMap<>();
    List<Candidate> candidates = new ArrayList<>();
    for (var neighbor : eligible) {
      double[] otherCentre = frames.get(neighbor.id()).camera.center();
      double[] baseline = Math3.mv(ref.camera.rotation, Math3.sub(otherCentre, centre));
      double direction = Math.hypot(baseline[0], baseline[1]) > 1e-10
          ? Math.atan2(baseline[1], baseline[0]) : Double.NaN;
      Candidate c = new Candidate(neighbor, new BitSet(GRID * GRID), direction, otherCentre);
      candidates.add(c);
      byId.put(neighbor.id(), c);
    }
    BitSet supported = new BitSet(GRID * GRID);
    for (var t : tracks) {
      if (t.point == null || Geometry.observation(t, reference) == null) continue;
      double[] p = ref.camera.project(t.point);
      if (!inside(p, ref)) continue;
      int cell = cell(p, ref);
      double[] u = Math3.sub(t.point, centre);
      double normU = Math3.norm(u);
      for (var observation : t.observations) {
        Candidate c = byId.get(observation.image);
        if (c == null) continue;
        ImageFrame other = frames.get(observation.image);
        if (!inside(other.camera.project(t.point), other)) continue;
        double[] v = Math3.sub(t.point, c.centre);
        double angle = Math.toDegrees(Math.acos(Math.clamp(
            (u[0] * v[0] + u[1] * v[1] + u[2] * v[2]) / (normU * Math3.norm(v)), -1, 1)));
        if (!Double.isFinite(angle) || angle < 1) continue;
        c.cells.set(cell);
        supported.set(cell);
      }
    }
    int[] covered = new int[GRID * GRID];
    List<Candidate> selected = new ArrayList<>();
    List<Object> decisions = new ArrayList<>();
    double bestScore = eligible.isEmpty() ? 1 : eligible.getFirst().score();
    while (selected.size() < Math.min(count, candidates.size())) {
      Candidate best = null;
      double bestUtility = -1, bestCoverage = 0, bestDirection = 0;
      for (Candidate c : candidates) {
        if (selected.contains(c)) continue;
        double gain = coverageGain(c, covered, target, supported.cardinality());
        double diversity = directionGain(c, selected);
        double utility = c.neighbor.score() / bestScore * (1 + gain + .25 * diversity);
        // Always keep the strongest raw-score source as the anchor (also used by fusion).
        if (selected.isEmpty()) { best = c; bestUtility = utility; bestCoverage = gain; break; }
        if (utility > bestUtility) {
          best = c; bestUtility = utility; bestCoverage = gain; bestDirection = diversity;
        }
      }
      if (best == null) break;
      selected.add(best);
      for (int bit = best.cells.nextSetBit(0); bit >= 0; bit = best.cells.nextSetBit(bit + 1))
        covered[bit]++;
      decisions.add(Json.object("image", best.neighbor.id(), "rank", selected.size(),
          "angle_score", best.neighbor.score(), "shared_points", best.neighbor.shared(),
          "median_angle_deg", best.neighbor.medianAngle(), "coverage_cells", best.cells.cardinality(),
          "coverage_gain", bestCoverage, "direction_gain", bestDirection, "utility", bestUtility));
    }
    List<Candidate> baseline = candidates.subList(0, Math.min(count, candidates.size()));
    return new Selection(selected.stream().map(Candidate::neighbor).toList(),
        Json.object("mode", "coverage_and_direction", "experimental", true,
            "eligible_candidates", candidates.size(), "supported_cells", supported.cardinality(),
            "grid", new int[] {GRID, GRID}, "witness_target", target,
            "score_only_coverage", coverage(baseline, target),
            "selected_coverage", coverage(selected, target),
            "score_only_direction_sectors", sectors(baseline),
            "selected_direction_sectors", sectors(selected),
            "decisions", decisions,
            "objective", "relative angle score * (1 + diminishing coverage gain + 0.25 * direction gain); strongest source anchored"));
  }

  private static boolean inside(double[] p, ImageFrame f) {
    return p != null && Double.isFinite(p[0]) && Double.isFinite(p[1])
        && p[0] >= 0 && p[1] >= 0 && p[0] < f.width && p[1] < f.height;
  }

  private static int cell(double[] p, ImageFrame f) {
    return Math.min(GRID - 1, (int) (GRID * p[1] / f.height)) * GRID
        + Math.min(GRID - 1, (int) (GRID * p[0] / f.width));
  }

  private static double coverageGain(Candidate c, int[] covered, int target, int supported) {
    double gain = 0;
    for (int bit = c.cells.nextSetBit(0); bit >= 0; bit = c.cells.nextSetBit(bit + 1))
      if (covered[bit] < target) gain += 1.0 / (covered[bit] + 1);
    return gain / Math.max(1, supported);
  }

  private static double directionGain(Candidate c, List<Candidate> selected) {
    if (!Double.isFinite(c.direction) || selected.isEmpty()) return 0;
    double nearest = Math.PI;
    boolean known = false;
    for (Candidate s : selected) if (Double.isFinite(s.direction)) {
      double d = Math.abs(Math.IEEEremainder(c.direction - s.direction, 2 * Math.PI));
      nearest = Math.min(nearest, d); known = true;
    }
    return known ? Math.min(1, nearest / (Math.PI / 2)) : 0;
  }

  private static int sectors(List<Candidate> candidates) {
    BitSet seen = new BitSet(8);
    for (Candidate c : candidates) if (Double.isFinite(c.direction))
      seen.set(Math.floorMod((int) Math.floor((c.direction + Math.PI / 8) / (Math.PI / 4)), 8));
    return seen.cardinality();
  }

  private static List<Integer> coverage(List<Candidate> selected, int target) {
    int[] counts = new int[GRID * GRID];
    for (Candidate c : selected)
      for (int bit = c.cells.nextSetBit(0); bit >= 0; bit = c.cells.nextSetBit(bit + 1))
        counts[bit]++;
    List<Integer> result = new ArrayList<>();
    for (int witnesses = 1; witnesses <= target; witnesses++) {
      int cells = 0;
      for (int n : counts) if (n >= witnesses) cells++;
      result.add(cells);
    }
    return result;
  }
}
