package sgm;

import java.util.*;

/**
 * GS1: vertical offset (and tilt) of every stereo pair's ground against the consensus ground,
 * like LiDAR strip adjustment. Uses each pair's own ground level per 1 m cell from all near-ground
 * points (GroundSheet.levels), so a pair whose ground copy sits outside the ground tolerance is
 * still measured. Each pair gets a robust (Huber) plane a + b (x - cx) + c (y - cy) fitted to its
 * level-minus-consensus residuals (weighted by points), with a tilt only when its cells span at
 * least 10 m both ways. Three rounds with the consensus recomputed from corrected levels; the
 * weighted mean offset is removed so the block height is kept.
 */
final class PairOffsets {
  private PairOffsets() {}

  static final int MIN_CELLS = 20;

  /** Per pair id: {a, b, c, cx, cy, cells, rmsBefore, rmsAfter}. */
  record Fit(
      Map<Integer, double[]> models, double[] twoSheetBefore, double[] twoSheetAfter) {
    int pairs() {
      return models.size();
    }

    double correction(int pair, double x, double y) {
      double[] m = models.get(pair);
      return m == null ? 0 : m[0] + m[1] * (x - m[3]) + m[2] * (y - m[4]);
    }

    Map<String, Object> summary() {
      List<Double> offsets = new ArrayList<>(), tilts = new ArrayList<>();
      double before = 0, after = 0, cells = 0;
      for (double[] m : models.values()) {
        offsets.add(Math.abs(m[0]));
        tilts.add(Math.hypot(m[1], m[2]) * 1000);
        before += m[6] * m[6] * m[5];
        after += m[7] * m[7] * m[5];
        cells += m[5];
      }
      Collections.sort(offsets);
      Collections.sort(tilts);
      return Json.object(
          "pairs_fitted", models.size(),
          "median_abs_offset_metres", GroundClassification.quantile(offsets, .5),
          "p95_abs_offset_metres", GroundClassification.quantile(offsets, .95),
          "max_abs_offset_metres", offsets.isEmpty() ? null : offsets.get(offsets.size() - 1),
          "median_tilt_mm_per_m", GroundClassification.quantile(tilts, .5),
          "pair_level_rms_about_consensus_before_metres", Math.sqrt(before / Math.max(1, cells)),
          "pair_level_rms_about_consensus_after_metres", Math.sqrt(after / Math.max(1, cells)),
          "two_sheet_cells_before", twoSheetBefore[0],
          "two_sheet_cells_after", twoSheetAfter[0],
          "median_pair_spread_before_metres", twoSheetBefore[1],
          "median_pair_spread_after_metres", twoSheetAfter[1],
          "note",
          "Pair levels: each pair's own ground height per 1 m cell. Two-sheet cells: cells seen by"
              + " two or more pairs whose levels differ by more than 0.3 m; 'after' applies the"
              + " fitted corrections to the levels.");
    }

    Map<String, Object> report() {
      Map<String, Object> out = new LinkedHashMap<>(summary());
      List<Object> rows = new ArrayList<>();
      for (var e : new TreeMap<>(models).entrySet()) {
        double[] m = e.getValue();
        rows.add(
            Json.object(
                "pair", e.getKey(),
                "cells", (long) m[5],
                "offset_metres", m[0],
                "tilt_x_per_m", m[1],
                "tilt_y_per_m", m[2],
                "rms_before_metres", m[6],
                "rms_after_metres", m[7]));
      }
      out.put("pairs", rows);
      return out;
    }

    String line() {
      return String.format(
          Locale.ROOT,
          "%d pairs, median |offset| %.3f m (p95 %.3f m), two-sheet cells %.1f%% -> %.1f%%",
          models.size(),
          ((Number) summary().get("median_abs_offset_metres")).doubleValue(),
          ((Number) summary().get("p95_abs_offset_metres")).doubleValue(),
          100 * twoSheetBefore[0],
          100 * twoSheetAfter[0]);
    }
  }

  static Fit fit(GroundClassification.Cloud c, GroundClassification.Grid g, GroundClassification.Core core) {
    GroundSheet.Levels l = GroundSheet.levels(c, g, core);
    Map<Integer, double[]> models = new HashMap<>();
    if (l == null) return new Fit(models, new double[] {0, Double.NaN}, new double[] {0, Double.NaN});
    int n = l.groups.size();
    int[] pair = new int[n], cell = new int[n];
    double[] x = new double[n], y = new double[n], weight = new double[n], level = new double[n];
    Map<Integer, List<Integer>> byPair = new HashMap<>();
    for (int q = 0; q < n; q++) {
      long key = l.groups.get(q)[0];
      pair[q] = (int) (key >>> 32);
      cell[q] = (int) key;
      x[q] = l.centreX(cell[q]);
      y[q] = l.centreY(cell[q]);
      weight[q] = l.counts.get(q);
      level[q] = l.levels.get(q);
      if (l.accepted[q]) byPair.computeIfAbsent(pair[q], p -> new ArrayList<>()).add(q);
    }
    double huber = Math.max(.05, l.tolerance);
    double[] correction = new double[n];
    double[] before = spread(cell, level, correction, l.accepted);
    Map<Integer, double[]> byIndex = new HashMap<>();
    for (int round = 0; round < 3; round++) {
      double[] consensus = consensus(cell, level, correction, weight, l.accepted);
      for (var e : byPair.entrySet()) {
        List<Integer> members = e.getValue();
        if (members.size() < MIN_CELLS) continue;
        double[] m = plane(members, x, y, level, correction, consensus, cell, weight, huber);
        if (m == null) continue;
        double[] previous = byIndex.get(e.getKey());
        if (previous != null) {
          m[0] += previous[0] + previous[1] * (m[3] - previous[3]) + previous[2] * (m[4] - previous[4]);
          m[1] += previous[1];
          m[2] += previous[2];
          m[6] = previous[6];
        }
        byIndex.put(e.getKey(), m);
        for (int q : members) correction[q] = m[0] + m[1] * (x[q] - m[3]) + m[2] * (y[q] - m[4]);
      }
    }
    // Gauge: keep the block's mean ground height.
    double mean = 0, total = 0;
    for (double[] m : byIndex.values()) {
      mean += m[0] * m[5];
      total += m[5];
    }
    mean /= Math.max(1, total);
    for (double[] m : byIndex.values()) m[0] -= mean;
    for (var e : byPair.entrySet()) {
      double[] m = byIndex.get(e.getKey());
      if (m != null) for (int q : e.getValue()) correction[q] = m[0] + m[1] * (x[q] - m[3]) + m[2] * (y[q] - m[4]);
    }
    double[] consensus = consensus(cell, level, correction, weight, l.accepted);
    for (var e : byPair.entrySet()) {
      double[] m = byIndex.get(e.getKey());
      if (m == null) continue;
      double sum = 0, w = 0;
      for (int q : e.getValue())
        if (Double.isFinite(consensus[q])) {
          double r = level[q] - correction[q] - consensus[q];
          sum += weight[q] * r * r;
          w += weight[q];
        }
      m[7] = Math.sqrt(sum / Math.max(1e-9, w));
      models.put(l.pairSource[e.getKey()], m);
    }
    return new Fit(models, before, spread(cell, level, correction, l.accepted));
  }

  /** Corrections for every point of the fitted pairs (canopy too); applied to the cloud's z. */
  static float[] apply(GroundClassification.Cloud c, Fit fit) {
    float[] delta = new float[c.n];
    for (int i = 0; i < c.n; i++) {
      double d = -fit.correction(c.source[i], c.x[i], c.y[i]);
      delta[i] = (float) d;
      c.z[i] += (float) d;
    }
    return delta;
  }

  /** Weighted consensus of corrected levels per cell, returned per group (NaN if alone). */
  private static double[] consensus(int[] cell, double[] level, double[] correction, double[] weight, boolean[] accepted) {
    Map<Integer, double[]> sums = new HashMap<>();
    for (int q = 0; q < cell.length; q++)
      if (accepted[q]) {
        double[] s = sums.computeIfAbsent(cell[q], k -> new double[3]);
        s[0] += weight[q] * (level[q] - correction[q]);
        s[1] += weight[q];
        s[2]++;
      }
    double[] out = new double[cell.length];
    for (int q = 0; q < cell.length; q++) {
      double[] s = sums.get(cell[q]);
      // A pair alone in a cell carries no relative information.
      out[q] = s == null || s[2] < 2 ? Double.NaN : s[0] / s[1];
    }
    return out;
  }

  /** {fraction of multi-pair cells spreading more than 0.3 m, median spread}. */
  private static double[] spread(int[] cell, double[] level, double[] correction, boolean[] accepted) {
    Map<Integer, double[]> range = new HashMap<>();
    for (int q = 0; q < cell.length; q++)
      if (accepted[q]) {
        double v = level[q] - correction[q];
        double[] r = range.computeIfAbsent(cell[q], k -> new double[] {Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY, 0});
        r[0] = Math.min(r[0], v);
        r[1] = Math.max(r[1], v);
        r[2]++;
      }
    int cells = 0, split = 0;
    List<Double> spreads = new ArrayList<>();
    for (double[] r : range.values())
      if (r[2] >= 2) {
        cells++;
        spreads.add(r[1] - r[0]);
        if (r[1] - r[0] > .3) split++;
      }
    Collections.sort(spreads);
    return new double[] {split / (double) Math.max(1, cells), GroundClassification.quantile(spreads, .5)};
  }

  /** Robust weighted plane (or constant) of one pair's residuals; {a, b, c, cx, cy, n, rms, 0}. */
  private static double[] plane(
      List<Integer> members,
      double[] x,
      double[] y,
      double[] level,
      double[] correction,
      double[] consensus,
      int[] cell,
      double[] weight,
      double huber) {
    List<double[]> rows = new ArrayList<>();
    double cx = 0, cy = 0, total = 0;
    for (int q : members) {
      if (!Double.isFinite(consensus[q])) continue;
      rows.add(new double[] {x[q], y[q], level[q] - correction[q] - consensus[q], weight[q]});
      cx += weight[q] * x[q];
      cy += weight[q] * y[q];
      total += weight[q];
    }
    if (rows.size() < MIN_CELLS) return null;
    cx /= total;
    cy /= total;
    double sx = 0, sy = 0, rms = 0;
    for (double[] r : rows) {
      sx += r[3] * Math.pow(r[0] - cx, 2);
      sy += r[3] * Math.pow(r[1] - cy, 2);
      rms += r[3] * r[2] * r[2];
    }
    rms = Math.sqrt(rms / total);
    boolean tilt = Math.sqrt(sx / total) > 10 / Math.sqrt(12) && Math.sqrt(sy / total) > 10 / Math.sqrt(12);
    double[] beta = new double[3];
    for (int iter = 0; iter < 10; iter++) {
      double[][] a = new double[3][3];
      double[] b = new double[3];
      for (double[] r : rows) {
        double[] j = {1, tilt ? r[0] - cx : 0, tilt ? r[1] - cy : 0};
        double e = r[2] - (beta[0] + beta[1] * j[1] + beta[2] * j[2]);
        double w = r[3] * Math.min(1, huber / Math.max(1e-9, Math.abs(e)));
        for (int p = 0; p < 3; p++) {
          b[p] += w * j[p] * r[2];
          for (int s = 0; s < 3; s++) a[p][s] += w * j[p] * j[s];
        }
      }
      if (!tilt) {
        a[1][1] = a[2][2] = 1;
        a[0][1] = a[0][2] = a[1][0] = a[2][0] = a[1][2] = a[2][1] = 0;
        b[1] = b[2] = 0;
      }
      double[] next = Math3.solve(a, b);
      if (next == null) return null;
      double change = Math.abs(next[0] - beta[0]);
      beta = next;
      if (change < 1e-5) break;
    }
    return new double[] {beta[0], beta[1], beta[2], cx, cy, rows.size(), rms, 0};
  }
}
