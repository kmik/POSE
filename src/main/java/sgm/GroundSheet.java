package sgm;

import java.io.*;
import java.nio.*;
import java.nio.channels.FileChannel;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.*;

/**
 * GS2, ground-aware fusion (--ground-aware-fusion): merges the ground copies of all stereo pairs
 * into one sheet after classification.
 *
 * <p>Each pair's own ground level is found per 1 m cell: the median of its points within the
 * ground tolerance of its 10th-percentile height, accepted when near the terrain model. The
 * consensus level is the weighted mean over pairs (pairs more than 1 m from the weighted median
 * are left alone). Every pair's ground copy is shifted onto the consensus (the shift interpolated
 * bilinearly within the pair, so pair-internal relief is kept), then the copies are merged: one
 * point per point-spacing cell at the view-weighted mean height, keeping the best-supported
 * point's colour and metadata. Points above a pair's own ground (shrubs, canopy) are not moved.
 * LAS, PLY and the confidence sidecar are rewritten consistently.
 */
final class GroundSheet {
  private GroundSheet() {}

  static Map<String, Object> run(
      Path las,
      Path sidecar,
      GroundClassification.Cloud c,
      GroundClassification.Grid g,
      GroundClassification.Core core,
      double spacing,
      double fusionRadius)
      throws IOException {
    if (!c.sidecar)
      return Json.object(
          "status", "skipped", "reason", "Needs point-confidence.bin aligned with dense.las (pair ids)");
    long started = System.nanoTime();
    Levels l = levels(c, g, core);
    if (l == null) return Json.object("status", "skipped", "reason", "Too many stereo pairs");
    double tolerance = l.tolerance, band = l.band, coarse = l.coarse;
    GroundClassification.Grid cg = l.cg;
    int[] members = l.members;
    List<long[]> groups = l.groups;
    List<Float> levels = l.levels;
    float[] consensus = l.consensus;
    boolean[] accepted = l.accepted;
    long multiPair = l.multiPair, layered = l.layered;
    List<Double> spreadBefore = l.spread;
    // Shift per (pair, cell), sorted by key for bilinear lookups.
    int accepts = 0;
    for (boolean a : accepted) if (a) accepts++;
    long[] shiftKey = new long[accepts];
    float[] shift = new float[accepts];
    int s = 0;
    for (int q = 0; q < groups.size(); q++)
      if (accepted[q]) {
        shiftKey[s] = groups.get(q)[0];
        shift[s++] = consensus[(int) groups.get(q)[0]] - levels.get(q);
      }
    // groups are already in key order, so shiftKey is sorted.
    float[] newZ = c.z.clone();
    byte[] classes = core.classes.clone();
    boolean[] moved = new boolean[c.n];
    long movedPoints = 0;
    for (int q = 0; q < groups.size(); q++) {
      if (!accepted[q]) continue;
      long[] group = groups.get(q);
      float level = levels.get(q);
      int pair = (int) (group[0] >>> 32);
      for (int j = (int) group[1]; j < group[2]; j++) {
        int i = members[j];
        if (Math.abs(c.z[i] - level) > tolerance) continue;
        newZ[i] = c.z[i] + (float) interpolate(shiftKey, shift, pair, cg, coarse, c.x[i], c.y[i]);
        moved[i] = true;
        classes[i] = 2;
        movedPoints++;
      }
    }
    // Thickness of the near-ground layer before and after (p90 - p10 per coarse cell).
    double thicknessBefore = thickness(c.z, null, c, cg, consensus, band);
    // Merge: one ground point per spacing cell, view-weighted mean height.
    double fine = Math.max(.01, spacing);
    long[] fineKey = new long[c.n];
    int[] order = new int[c.n];
    int f = 0;
    long columns = (long) Math.ceil(g.w() * g.cell() / fine) + 1;
    for (int i = 0; i < c.n; i++)
      if (classes[i] == 2) {
        long col = (long) Math.floor((c.x[i] - g.x0()) / fine), row = (long) Math.floor((g.y1() - c.y[i]) / fine);
        if (col < 0 || row < 0) continue;
        fineKey[f] = row * columns + col;
        order[f++] = i;
      }
    fineKey = Arrays.copyOf(fineKey, f);
    order = Arrays.copyOf(order, f);
    sortTogether(fineKey, order);
    boolean[] keep = new boolean[c.n];
    Arrays.fill(keep, true);
    long removed = 0, mergedCells = 0;
    for (int start = 0; start < f; ) {
      int end = start;
      while (end < f && fineKey[end] == fineKey[start]) end++;
      if (end - start > 1) {
        mergedCells++;
        int best = order[start];
        double sum = 0, weight = 0;
        for (int j = start; j < end; j++) {
          int i = order[j];
          double w = Math.max(1, c.views[i]);
          sum += w * newZ[i];
          weight += w;
          if (c.views[i] > c.views[best]) best = i;
        }
        for (int j = start; j < end; j++) {
          int i = order[j];
          if (i != best) {
            keep[i] = false;
            removed++;
          }
        }
        newZ[best] = (float) (sum / weight);
        moved[best] = true;
      }
      start = end;
    }
    // Fuse the merged copies into one smooth sheet: local robust plane over the ground points.
    double radius = fusionRadius > 0 ? fusionRadius : 2 * g.cell();
    long smoothed = smooth(newZ, keep, classes, c, g, radius, tolerance);
    double[] thickness = {thicknessBefore, thickness(newZ, keep, c, cg, consensus, band)};
    double near = Math.max(.5, tolerance);
    double[] twoSheet = {
      layered(c.z, null, c, g, l.dtm, near), layered(newZ, keep, c, g, l.dtm, near)
    };
    float[] delta = new float[c.n];
    for (int i = 0; i < c.n; i++) delta[i] = newZ[i] - c.z[i];
    long kept = rewrite(las, sidecar, c, keep, delta, classes);
    Collections.sort(spreadBefore);
    Map<String, Object> report =
        Json.object(
            "status", "complete",
            "method",
            "Pair-local ground levels per coarse cell shifted onto their weighted consensus, then"
                + " one point per spacing cell at the view-weighted mean height",
            "coarse_cell_metres", coarse,
            "merge_cell_metres", fine,
            "sheet_fusion_radius_metres", radius,
            "sheet_fused_points", smoothed,
            "ground_tolerance_metres", tolerance,
            "near_ground_band_metres", band,
            "pair_cells", groups.size(),
            "pair_cells_merged", accepts,
            "cells_seen_by_two_or_more_pairs", multiPair,
            "layered_cells_before", multiPair == 0 ? 0 : layered / (double) multiPair,
            "median_pair_level_spread_before_metres", GroundClassification.quantile(spreadBefore, .5),
            "moved_ground_points", movedPoints,
            "merged_cells", mergedCells,
            "removed_duplicate_points", removed,
            "points_before", c.n,
            "points_after", kept,
            "near_ground_thickness_before_metres", thickness[0],
            "near_ground_thickness_after_metres", thickness[1],
            "two_sheet_cells_before", twoSheet[0],
            "two_sheet_cells_after", twoSheet[1],
            "note",
            "Thickness: median over 1 m cells of the p90 - p10 height spread of all points within"
                + " the near-ground band. Shrubs and canopy above a pair's own ground are not moved."
                + " dense.las, dense.ply and point-confidence.bin are rewritten in step.",
            "elapsed_seconds", (System.nanoTime() - started) / 1e9);
    System.out.printf(
        Locale.ROOT,
        "Ground-aware fusion: %,d ground points moved onto one sheet, %,d duplicates merged (%,d ->"
            + " %,d points); two-sheet cells %.1f%% -> %.1f%%%n",
        movedPoints,
        removed,
        c.n,
        kept,
        100 * twoSheet[0],
        100 * twoSheet[1]);
    return report;
  }

  /**
   * Each pair's own ground level per coarse cell (median of its near-ground points within the
   * ground tolerance of their 10th percentile, accepted when near the terrain model), and the
   * consensus per cell (weighted mean over pairs within 1 m of the weighted median). Shared by
   * GS1 (pair offsets) and GS2 (sheet merge).
   */
  static final class Levels {
    double tolerance, band, coarse;
    GroundClassification.Grid cg;
    float[] dtm;
    int[] members, pairSource;
    List<long[]> groups = new ArrayList<>(); // {pair << 32 | cell, start, end} in key order
    List<Float> levels = new ArrayList<>(), counts = new ArrayList<>();
    float[] consensus;
    boolean[] accepted;
    long multiPair, layered;
    List<Double> spread = new ArrayList<>();

    double centreX(int cell) {
      return cg.x0() + (cell % cg.w() + .5) * coarse;
    }

    double centreY(int cell) {
      return cg.y1() - (cell / cg.w() + .5) * coarse;
    }
  }

  static Levels levels(
      GroundClassification.Cloud c, GroundClassification.Grid g, GroundClassification.Core core) {
    Levels out = new Levels();
    double sigma = core.sigma, tolerance = Math.max(.1, 3 * sigma), band = Math.max(1.5, 10 * sigma);
    double coarse = Math.max(1, 4 * g.cell());
    GroundClassification.Grid cg = coarseGrid(g, coarse);
    float[] dtm = core.full.dtm();
    // Near-ground points per (pair, coarse cell).
    Map<Integer, Integer> pairIndex = new HashMap<>();
    for (int i = 0; i < c.n; i++) pairIndex.putIfAbsent(c.source[i], pairIndex.size());
    int pairs = pairIndex.size();
    if (pairs >= 1 << 20) return null;
    int[] pairOf = new int[c.n];
    long near = 0;
    for (int i = 0; i < c.n; i++) {
      pairOf[i] = pairIndex.get(c.source[i]);
      if (nearGround(c, g, dtm, i, core, band)) near++;
    }
    long[] keys = new long[(int) near];
    int[] members = new int[(int) near];
    int m = 0;
    for (int i = 0; i < c.n; i++)
      if (nearGround(c, g, dtm, i, core, band)) {
        int k = cg.index(c.x[i], c.y[i]);
        if (k < 0) continue;
        keys[m] = ((long) pairOf[i] << 32) | k;
        members[m++] = i;
      }
    keys = Arrays.copyOf(keys, m);
    members = Arrays.copyOf(members, m);
    sortTogether(keys, members);
    // Pair-local ground levels.
    List<long[]> groups = new ArrayList<>(); // {key, start, end}
    List<Float> levels = new ArrayList<>(), counts = new ArrayList<>();
    for (int start = 0; start < m; ) {
      int end = start;
      while (end < m && keys[end] == keys[start]) end++;
      if (end - start >= 8) {
        float[] z = new float[end - start];
        for (int j = start; j < end; j++) z[j - start] = c.z[members[j]];
        Arrays.sort(z);
        float q10 = z[(z.length - 1) / 10];
        List<Float> low = new ArrayList<>();
        for (float v : z) if (Math.abs(v - q10) <= tolerance) low.add(v);
        float level = low.get(low.size() / 2);
        int k = (int) keys[start];
        double centreX = cg.x0() + (k % cg.w() + .5) * coarse, centreY = cg.y1() - (k / cg.w() + .5) * coarse;
        double terrain = GroundClassification.sample(dtm, g, centreX, centreY);
        if (Double.isFinite(terrain) && Math.abs(level - terrain) <= band) {
          groups.add(new long[] {keys[start], start, end});
          levels.add(level);
          counts.add((float) low.size());
        }
      }
      start = end;
    }
    // Consensus per coarse cell: weighted mean over pairs after a 1 m weighted-median cut.
    Map<Integer, List<Integer>> byCell = new HashMap<>();
    for (int q = 0; q < groups.size(); q++) byCell.computeIfAbsent((int) groups.get(q)[0], k -> new ArrayList<>()).add(q);
    float[] consensus = new float[cg.w() * cg.h()];
    Arrays.fill(consensus, Float.NaN);
    boolean[] accepted = new boolean[groups.size()];
    long multiPair = 0, layered = 0;
    List<Double> spreadBefore = new ArrayList<>();
    for (var e : byCell.entrySet()) {
      List<Integer> list = e.getValue();
      list.sort(Comparator.comparingDouble(levels::get));
      double total = 0, half = 0;
      for (int q : list) total += counts.get(q);
      float median = levels.get(list.get(0));
      for (int q : list) {
        half += counts.get(q);
        if (half >= total / 2) {
          median = levels.get(q);
          break;
        }
      }
      double sum = 0, weight = 0, lo = Double.POSITIVE_INFINITY, hi = Double.NEGATIVE_INFINITY;
      for (int q : list)
        if (Math.abs(levels.get(q) - median) <= 1) {
          accepted[q] = true;
          sum += levels.get(q) * counts.get(q);
          weight += counts.get(q);
          lo = Math.min(lo, levels.get(q));
          hi = Math.max(hi, levels.get(q));
        }
      consensus[e.getKey()] = (float) (sum / weight);
      if (list.size() >= 2) {
        multiPair++;
        spreadBefore.add(hi - lo);
        if (hi - lo > .3) layered++;
      }
    }
    out.tolerance = tolerance;
    out.band = band;
    out.coarse = coarse;
    out.cg = cg;
    out.dtm = dtm;
    out.members = members;
    out.pairSource = new int[pairs];
    for (var e : pairIndex.entrySet()) out.pairSource[e.getValue()] = e.getKey();
    out.groups = groups;
    out.levels = levels;
    out.counts = counts;
    out.consensus = consensus;
    out.accepted = accepted;
    out.multiPair = multiPair;
    out.layered = layered;
    out.spread = spreadBefore;
    return out;
  }

  private static boolean nearGround(
      GroundClassification.Cloud c,
      GroundClassification.Grid g,
      float[] dtm,
      int i,
      GroundClassification.Core core,
      double band) {
    if (core.classes[i] == 7) return false;
    double t = GroundClassification.sample(dtm, g, c.x[i], c.y[i]);
    return Double.isFinite(t) && c.z[i] - t <= band && c.z[i] - t >= -band;
  }

  static GroundClassification.Grid coarseGrid(GroundClassification.Grid g, double cell) {
    int w = (int) Math.ceil(g.w() * g.cell() / cell) + 1, h = (int) Math.ceil(g.h() * g.cell() / cell) + 1;
    return new GroundClassification.Grid(w, h, cell, g.x0(), g.y1());
  }

  /** Bilinear shift of one pair at (x, y) from its coarse-cell shifts (own cell if alone). */
  static double interpolate(
      long[] keys, float[] shift, int pair, GroundClassification.Grid cg, double cell, double x, double y) {
    double fx = (x - cg.x0()) / cell - .5, fy = (cg.y1() - y) / cell - .5;
    int c0 = (int) Math.floor(fx), r0 = (int) Math.floor(fy);
    double ax = fx - c0, ay = fy - r0, sum = 0, weight = 0;
    for (int dr = 0; dr < 2; dr++)
      for (int dc = 0; dc < 2; dc++) {
        int r = r0 + dr, c = c0 + dc;
        if (r < 0 || c < 0 || r >= cg.h() || c >= cg.w()) continue;
        int at = Arrays.binarySearch(keys, ((long) pair << 32) | (r * cg.w() + c));
        if (at < 0) continue;
        double w = (dc == 0 ? 1 - ax : ax) * (dr == 0 ? 1 - ay : ay);
        sum += w * shift[at];
        weight += w;
      }
    if (weight > 1e-9) return sum / weight;
    int own = Arrays.binarySearch(keys, ((long) pair << 32) | cg.index(x, y));
    return own >= 0 ? shift[own] : 0;
  }

  /**
   * Two-sheet ground cells: 0.5 m cells whose lowest point lies on the terrain and whose points
   * within 1.5 m of it
   * split at a vertical gap of more than 0.25 m with at least 25% of them on each side. Measured
   * on the cloud as written, whatever the classes (shrubs count both before and after).
   */
  static double layered(
      float[] z, boolean[] keep, GroundClassification.Cloud c, GroundClassification.Grid g, float[] dtm, double near) {
    GroundClassification.Grid half = new GroundClassification.Grid(
        (int) Math.ceil(g.w() * g.cell() / .5) + 1, (int) Math.ceil(g.h() * g.cell() / .5) + 1, .5, g.x0(), g.y1());
    int stride = Math.max(1, c.n / 8_000_000);
    long[] packed = new long[c.n / stride + 1];
    int n = 0;
    for (int i = 0; i < c.n; i += stride) {
      if (keep != null && !keep[i]) continue;
      int k = half.index(c.x[i], c.y[i]);
      if (k >= 0) packed[n++] = ((long) k << 32) | i;
    }
    packed = Arrays.copyOf(packed, n);
    Arrays.sort(packed);
    long cells = 0, split = 0;
    for (int start = 0; start < n; ) {
      int end = start;
      long cell = packed[start] >>> 32;
      while (end < n && packed[end] >>> 32 == cell) end++;
      if (end - start >= 8) {
        float[] v = new float[end - start];
        for (int j = start; j < end; j++) v[j - start] = z[(int) packed[j]];
        Arrays.sort(v);
        int top = 0;
        while (top < v.length && v[top] <= v[0] + 1.5) top++;
        // Ground cells only: the lowest point must lie on the terrain (crowns have their own spread).
        int any = (int) packed[start];
        double terrain = GroundClassification.sample(dtm, g, c.x[any], c.y[any]);
        if (top >= 8 && Math.abs(v[0] - terrain) <= near) {
          cells++;
          for (int j = Math.max(1, top / 4); j <= top - top / 4 && j < top; j++)
            if (v[j] - v[j - 1] > .25) {
              split++;
              break;
            }
        }
      }
      start = end;
    }
    return split / (double) Math.max(1, cells);
  }

  /**
   * Sheet fusion: every kept ground point takes the height of a weighted plane fitted to the
   * ground points within radius (Gaussian weights x views; one Huber reweighting at the ground
   * tolerance), so per-pair depth noise averages out across all pairs. Only class 2 moves;
   * relief larger than the radius is kept. Returns the number of points fused.
   */
  static long smooth(
      float[] z,
      boolean[] keep,
      byte[] classes,
      GroundClassification.Cloud c,
      GroundClassification.Grid g,
      double radius,
      double tolerance) {
    GroundClassification.Grid bins =
        new GroundClassification.Grid(
            (int) Math.ceil(g.w() * g.cell() / radius) + 1,
            (int) Math.ceil(g.h() * g.cell() / radius) + 1,
            radius,
            g.x0(),
            g.y1());
    int count = 0;
    for (int i = 0; i < c.n; i++) if (keep[i] && classes[i] == 2) count++;
    long[] packed = new long[count];
    int n = 0;
    for (int i = 0; i < c.n; i++)
      if (keep[i] && classes[i] == 2) {
        int k = bins.index(c.x[i], c.y[i]);
        if (k >= 0) packed[n++] = ((long) k << 32) | i;
      }
    final long[] sorted = Arrays.copyOf(packed, n);
    Arrays.sort(sorted);
    // One robust plane per occupied bin, anchored at the bin centre.
    int[] starts = java.util.stream.IntStream.range(0, n)
        .filter(j -> j == 0 || (sorted[j] >>> 32) != (sorted[j - 1] >>> 32))
        .toArray();
    int bins2 = starts.length;
    int[] binId = new int[bins2];
    double[][] planes = new double[bins2][];
    for (int b = 0; b < bins2; b++) binId[b] = (int) (sorted[starts[b]] >>> 32);
    java.util.stream.IntStream.range(0, bins2).parallel().forEach(b -> {
      int bin = binId[b], row = bin / bins.w(), col = bin % bins.w();
      double cx = bins.x0() + (col + .5) * radius, cy = bins.y1() - (row + .5) * radius;
      int[] near = new int[64];
      int k = 0;
      for (int dr = -1; dr <= 1; dr++)
        for (int dc = -1; dc <= 1; dc++) {
          int r = row + dr, cc = col + dc;
          if (r < 0 || cc < 0 || r >= bins.h() || cc >= bins.w()) continue;
          long key = (long) (r * bins.w() + cc) << 32;
          int from = lowerBound(sorted, key), to = lowerBound(sorted, key + (1L << 32));
          if (k + to - from > near.length) near = Arrays.copyOf(near, Math.max(2 * near.length, k + to - from));
          for (int j = from; j < to; j++) near[k++] = (int) sorted[j];
        }
      planes[b] = plane(c, z, near, k, cx, cy, 1.5 * radius, radius / 2, tolerance);
    });
    // Each point: bilinear blend of the planes of the four nearest bin centres.
    Map<Integer, Integer> slot = new HashMap<>(bins2 * 2);
    for (int b = 0; b < bins2; b++) slot.put(binId[b], b);
    float[] out = z.clone();
    long fused = 0;
    for (int j = 0; j < n; j++) {
      int i = (int) sorted[j];
      double fx = (c.x[i] - bins.x0()) / radius - .5, fy = (bins.y1() - c.y[i]) / radius - .5;
      int c0 = (int) Math.floor(fx), r0 = (int) Math.floor(fy);
      double ax = fx - c0, ay = fy - r0, sum = 0, weight = 0;
      for (int dr = 0; dr < 2; dr++)
        for (int dc = 0; dc < 2; dc++) {
          int r = r0 + dr, cc = c0 + dc;
          if (r < 0 || cc < 0 || r >= bins.h() || cc >= bins.w()) continue;
          Integer b = slot.get(r * bins.w() + cc);
          if (b == null || planes[b] == null) continue;
          double w = (dc == 0 ? 1 - ax : ax) * (dr == 0 ? 1 - ay : ay);
          double px = bins.x0() + (cc + .5) * radius, py = bins.y1() - (r + .5) * radius;
          double[] p = planes[b];
          sum += w * (p[0] + p[1] * (c.x[i] - px) + p[2] * (c.y[i] - py));
          weight += w;
        }
      if (weight > 1e-9) {
        out[i] = (float) (sum / weight);
        fused++;
      }
    }
    System.arraycopy(out, 0, z, 0, z.length);
    return fused;
  }

  private static int lowerBound(long[] a, long key) {
    int lo = 0, hi = a.length;
    while (lo < hi) {
      int mid = (lo + hi) >>> 1;
      if (a[mid] < key) lo = mid + 1;
      else hi = mid;
    }
    return lo;
  }

  /**
   * Robust weighted plane {a, b, c} about (x0, y0) through the points within reach (Gaussian
   * weights x views; one Huber reweighting at the ground tolerance), or null with too few points.
   */
  private static double[] plane(
      GroundClassification.Cloud c,
      float[] z,
      int[] near,
      int count,
      double x0,
      double y0,
      double reach,
      double sigma,
      double tolerance) {
    double[] plane = null;
    for (int pass = 0; pass < 2; pass++) {
      double[][] a = new double[3][3];
      double[] b = new double[3];
      int used = 0;
      for (int s = 0; s < count; s++) {
        int j = near[s];
        double dx = c.x[j] - x0, dy = c.y[j] - y0, d2 = dx * dx + dy * dy;
        if (d2 > reach * reach) continue;
        double w = Math.exp(-d2 / (2 * sigma * sigma)) * Math.max(1, c.views[j]);
        if (plane != null) {
          double e = z[j] - (plane[0] + plane[1] * dx + plane[2] * dy);
          w *= Math.min(1, tolerance / Math.max(1e-9, Math.abs(e)));
        }
        double[] row = {1, dx, dy};
        for (int p = 0; p < 3; p++) {
          b[p] += w * row[p] * z[j];
          for (int q = 0; q < 3; q++) a[p][q] += w * row[p] * row[q];
        }
        used++;
      }
      if (used < 6) return null;
      for (int p = 1; p < 3; p++) a[p][p] += 1e-6;
      double[] next = Math3.solve(a, b);
      if (next == null) return null;
      plane = next;
    }
    return plane;
  }

  /** Median over coarse cells of p90 - p10 of heights within the band around the consensus. */
  static double thickness(
      float[] z,
      boolean[] keep,
      GroundClassification.Cloud c,
      GroundClassification.Grid cg,
      float[] consensus,
      double band) {
    Map<Integer, List<Integer>> cells = new HashMap<>();
    int stride = Math.max(1, c.n / 4_000_000);
    for (int i = 0; i < c.n; i += stride) {
      int k = cg.index(c.x[i], c.y[i]);
      if (keep != null && !keep[i]) continue;
      if (k < 0 || !Float.isFinite(consensus[k]) || Math.abs(z[i] - consensus[k]) > band) continue;
      cells.computeIfAbsent(k, q -> new ArrayList<>()).add(i);
    }
    List<Double> spreads = new ArrayList<>();
    for (List<Integer> v : cells.values()) {
      if (v.size() < 10) continue;
      // Residuals about the cell's least-squares plane, so slope is not counted as thickness.
      double[][] a = new double[3][3];
      double[] b = new double[3];
      double mx = 0, my = 0;
      for (int i : v) {
        mx += c.x[i] / v.size();
        my += c.y[i] / v.size();
      }
      for (int i : v) {
        double[] j = {1, c.x[i] - mx, c.y[i] - my};
        for (int p = 0; p < 3; p++) {
          b[p] += j[p] * z[i];
          for (int q = 0; q < 3; q++) a[p][q] += j[p] * j[q];
        }
      }
      double[] plane = Math3.solve(a, b);
      if (plane == null) continue;
      float[] r = new float[v.size()];
      for (int n = 0; n < r.length; n++) {
        int i = v.get(n);
        r[n] = (float) (z[i] - plane[0] - plane[1] * (c.x[i] - mx) - plane[2] * (c.y[i] - my));
      }
      Arrays.sort(r);
      spreads.add((double) (r[(r.length - 1) * 9 / 10] - r[(r.length - 1) / 10]));
    }
    Collections.sort(spreads);
    return GroundClassification.quantile(spreads, .5);
  }

  /** Sorts keys ascending and permutes values alongside (in-place merge via index sort). */
  static void sortTogether(long[] keys, int[] values) {
    int n = keys.length;
    if (n < 2) return;
    // Keys use more than 32 bits: sort positions by key with a primitive merge sort.
    int[] pos = new int[n], tmp = new int[n];
    for (int i = 0; i < n; i++) pos[i] = i;
    mergeSort(pos, tmp, keys, 0, n);
    long[] k2 = new long[n];
    int[] v2 = new int[n];
    for (int i = 0; i < n; i++) {
      k2[i] = keys[pos[i]];
      v2[i] = values[pos[i]];
    }
    System.arraycopy(k2, 0, keys, 0, n);
    System.arraycopy(v2, 0, values, 0, n);
  }

  private static void mergeSort(int[] a, int[] tmp, long[] keys, int lo, int hi) {
    if (hi - lo < 32) {
      for (int i = lo + 1; i < hi; i++) {
        int v = a[i], j = i - 1;
        while (j >= lo && keys[a[j]] > keys[v]) {
          a[j + 1] = a[j];
          j--;
        }
        a[j + 1] = v;
      }
      return;
    }
    int mid = (lo + hi) >>> 1;
    mergeSort(a, tmp, keys, lo, mid);
    mergeSort(a, tmp, keys, mid, hi);
    if (keys[a[mid - 1]] <= keys[a[mid]]) return;
    System.arraycopy(a, lo, tmp, lo, hi - lo);
    int i = lo, j = mid, k = lo;
    while (i < mid && j < hi) a[k++] = keys[tmp[i]] <= keys[tmp[j]] ? tmp[i++] : tmp[j++];
    while (i < mid) a[k++] = tmp[i++];
    while (j < hi) a[k++] = tmp[j++];
  }

  /**
   * Writes the kept points with adjusted heights and classes: dense.las (header count, by-return
   * count and bounds patched), dense.ply (vertex count) and point-confidence.bin (ids renumbered).
   * Each goes to a temporary file first and replaces the original when complete.
   */
  static long rewrite(
      Path las, Path sidecar, GroundClassification.Cloud c, boolean[] keep, float[] delta, byte[] classes)
      throws IOException {
    long kept = 0;
    for (boolean k : keep) if (k) kept++;
    Path lasTemp = las.resolveSibling("dense.las.sheet");
    double[] min = {Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY};
    double[] max = {Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY};
    double[] scale = new double[3], offset = new double[3];
    try (FileChannel in = FileChannel.open(las, StandardOpenOption.READ);
        FileChannel out =
            FileChannel.open(lasTemp, StandardOpenOption.CREATE, StandardOpenOption.TRUNCATE_EXISTING, StandardOpenOption.WRITE)) {
      ByteBuffer header = ByteBuffer.allocate((int) c.dataOffset).order(ByteOrder.LITTLE_ENDIAN);
      while (header.hasRemaining()) if (in.read(header, header.position()) < 0) throw new EOFException();
      for (int k = 0; k < 3; k++) {
        scale[k] = header.getDouble(131 + 8 * k);
        offset[k] = header.getDouble(155 + 8 * k);
      }
      out.position(c.dataOffset);
      ByteBuffer block = ByteBuffer.allocate(c.recordLength * 65536).order(ByteOrder.LITTLE_ENDIAN);
      ByteBuffer write = ByteBuffer.allocate(c.recordLength * 65536).order(ByteOrder.LITTLE_ENDIAN);
      for (long start = 0; start < c.n; start += 65536) {
        int records = (int) Math.min(65536, c.n - start);
        block.clear().limit(records * c.recordLength);
        long position = c.dataOffset + start * c.recordLength;
        while (block.hasRemaining()) if (in.read(block, position + block.position()) < 0) throw new EOFException();
        write.clear();
        for (int r = 0; r < records; r++) {
          int i = (int) (start + r), p = r * c.recordLength;
          if (!keep[i]) continue;
          double z = block.getInt(p + 8) * scale[2] + offset[2] + delta[i];
          block.putInt(p + 8, (int) Math.round((z - offset[2]) / scale[2]));
          block.put(p + 15, classes[i]);
          double[] v = {block.getInt(p) * scale[0] + offset[0], block.getInt(p + 4) * scale[1] + offset[1], z};
          for (int k = 0; k < 3; k++) {
            min[k] = Math.min(min[k], v[k]);
            max[k] = Math.max(max[k], v[k]);
          }
          write.put(block.array(), p, c.recordLength);
        }
        write.flip();
        while (write.hasRemaining()) out.write(write);
      }
      header.putInt(107, (int) kept).putInt(111, (int) kept);
      for (int k = 0; k < 3; k++) header.putDouble(179 + 16 * k, max[k]).putDouble(187 + 16 * k, min[k]);
      header.clear();
      while (header.hasRemaining()) out.write(header, header.position());
    }
    Path ply = las.resolveSibling("dense.ply"), plyTemp = las.resolveSibling("dense.ply.sheet");
    if (Files.isRegularFile(ply)) {
      try (FileChannel in = FileChannel.open(ply, StandardOpenOption.READ);
          FileChannel out =
              FileChannel.open(plyTemp, StandardOpenOption.CREATE, StandardOpenOption.TRUNCATE_EXISTING, StandardOpenOption.WRITE)) {
        long headerLength = GroundClassification.plyHeaderLength(in);
        ByteBuffer header = ByteBuffer.allocate((int) headerLength);
        in.read(header, 0);
        String text = new String(header.array(), StandardCharsets.ISO_8859_1)
            .replaceFirst("element vertex \\d+", "element vertex " + kept);
        out.write(ByteBuffer.wrap(text.getBytes(StandardCharsets.ISO_8859_1)));
        long record = (in.size() - headerLength) / c.n;
        ByteBuffer block = ByteBuffer.allocate((int) record * 16384).order(ByteOrder.LITTLE_ENDIAN);
        ByteBuffer write = ByteBuffer.allocate((int) record * 16384).order(ByteOrder.LITTLE_ENDIAN);
        for (long start = 0; start < c.n; start += 16384) {
          int records = (int) Math.min(16384, c.n - start);
          long position = headerLength + start * record;
          block.clear().limit((int) (records * record));
          while (block.hasRemaining()) if (in.read(block, position + block.position()) < 0) throw new EOFException();
          write.clear();
          for (int r = 0; r < records; r++) {
            int i = (int) (start + r), p = (int) (r * record);
            if (!keep[i]) continue;
            block.putDouble(p + 16, block.getDouble(p + 16) + delta[i]);
            write.put(block.array(), p, (int) record);
          }
          write.flip();
          while (write.hasRemaining()) out.write(write);
        }
      }
    }
    Path sideTemp = sidecar.resolveSibling("point-confidence.bin.sheet");
    try (PointRecords.Reader reader = new PointRecords.Reader(sidecar);
        PointRecords.Writer writer = new PointRecords.Writer(sideTemp)) {
      long id = 0;
      int i = 0;
      for (PointRecords.Row row; (row = reader.next()) != null; i++) {
        if (!keep[i]) continue;
        var sample = row.sample();
        sample.point[2] += delta[i];
        writer.point(id++, sample, 1, row.score());
      }
    }
    CloudWriter.publish(lasTemp, las);
    if (Files.isRegularFile(plyTemp)) CloudWriter.publish(plyTemp, ply);
    CloudWriter.publish(sideTemp, sidecar);
    return kept;
  }
}
