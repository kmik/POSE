package sgm;

import java.nio.file.*;
import java.util.*;

/**
 * Read-only quality comparison of a candidate LAS against a reference LAS from the same pipeline,
 * and optionally against an independent cloud (e.g. Metashape). Not ground truth.
 *
 * <p>Usage: CloudQuality candidate.las reference.las out.json [independent.las]
 *
 * <p>Distances use the reference's median nearest-neighbour spacing s: "matched" means within 3s,
 * "unsupported" means farther than 5s from every reference point, "detached" means fewer than 4
 * candidate neighbours within 5s. Height layers are relative to the 2nd percentile of reference Z
 * in each 5 m column. Grid comparisons use 1 m cells: canopy = max Z, ground = min Z.
 */
public final class CloudQuality {
  public static void main(String[] args) throws Exception {
    List<double[]> candidate = CloudDiagnostics.readLas(Path.of(args[0]));
    List<double[]> reference = CloudDiagnostics.readLas(Path.of(args[1]));
    Map<String, Object> report = new LinkedHashMap<>();
    report.put("candidate", args[0]);
    report.put("reference", args[1]);
    report.put("candidate_points", candidate.size());
    report.put("reference_points", reference.size());
    Grid refGrid = new Grid(reference, 0.05);
    double spacing = medianSpacing(reference, refGrid);
    Grid refIndex = new Grid(reference, 3 * spacing);
    Grid candIndex = new Grid(candidate, 3 * spacing);
    report.put("reference_median_spacing", spacing);
    double[] toReference = distances(candidate, reference, refIndex, 5 * spacing, 200_000);
    double[] toCandidate = distances(reference, candidate, candIndex, 5 * spacing, 200_000);
    report.put("candidate_to_reference", quantiles(toReference, 3 * spacing, 5 * spacing));
    report.put("reference_to_candidate", quantiles(toCandidate, 3 * spacing, 5 * spacing));
    report.put("completeness_within_3s", fraction(toCandidate, 3 * spacing));
    report.put("accuracy_within_3s", fraction(toReference, 3 * spacing));
    report.put("unsupported_fraction_beyond_5s", 1 - fraction(toReference, 5 * spacing));
    // Detached points use each cloud's own spacing, so sparser clouds are not penalized for density.
    double ownSpacing = medianSpacing(candidate, candIndex);
    report.put("candidate_median_spacing", ownSpacing);
    report.put(
        "candidate_detached_fraction",
        detached(candidate, new Grid(candidate, 3 * ownSpacing), 5 * ownSpacing, 4, 200_000));
    report.put("reference_detached_fraction", detached(reference, refIndex, 5 * spacing, 4, 200_000));
    report.put("completeness_by_height_layer", layers(reference, candidate, candIndex, 3 * spacing));
    report.put("grid_vs_reference", grids(candidate, reference));
    if (args.length > 3) {
      List<double[]> independent = CloudDiagnostics.readLas(Path.of(args[3]));
      Map<String, Object> other = new LinkedHashMap<>();
      other.put("independent", args[3]);
      other.put("independent_points", independent.size());
      other.put("candidate_grid", grids(candidate, independent));
      other.put("reference_grid", grids(reference, independent));
      report.put("vs_independent", other);
    }
    Json.write(Path.of(args[2]), report);
    System.out.println(Json.string(report));
  }

  /** Uniform hash grid over XYZ. */
  static final class Grid {
    final double cell;
    final HashMap<Long, int[]> cells = new HashMap<>();
    final List<double[]> points;

    Grid(List<double[]> points, double cell) {
      this.cell = cell;
      this.points = points;
      HashMap<Long, Integer> counts = new HashMap<>();
      for (double[] p : points) counts.merge(key(p[0], p[1], p[2]), 1, Integer::sum);
      HashMap<Long, Integer> fill = new HashMap<>();
      for (var e : counts.entrySet()) {
        cells.put(e.getKey(), new int[e.getValue()]);
        fill.put(e.getKey(), 0);
      }
      for (int i = 0; i < points.size(); i++) {
        double[] p = points.get(i);
        long k = key(p[0], p[1], p[2]);
        int at = fill.merge(k, 1, Integer::sum) - 1;
        cells.get(k)[at] = i;
      }
    }

    long key(double x, double y, double z) {
      long a = (long) Math.floor(x / cell) & 0x1FFFFF,
          b = (long) Math.floor(y / cell) & 0x1FFFFF,
          c = (long) Math.floor(z / cell) & 0x1FFFFF;
      return a << 42 | b << 21 | c;
    }

    /** Nearest distance within limit (limit <= cell * reach), or +inf; skips index skip. */
    double nearest(double[] q, double limit, int skip) {
      int reach = (int) Math.ceil(limit / cell);
      double best = limit * limit;
      boolean found = false;
      long cx = (long) Math.floor(q[0] / cell), cy = (long) Math.floor(q[1] / cell), cz = (long) Math.floor(q[2] / cell);
      for (long dx = -reach; dx <= reach; dx++)
        for (long dy = -reach; dy <= reach; dy++)
          for (long dz = -reach; dz <= reach; dz++) {
            int[] members = cells.get(((cx + dx) & 0x1FFFFF) << 42 | ((cy + dy) & 0x1FFFFF) << 21 | ((cz + dz) & 0x1FFFFF));
            if (members == null) continue;
            for (int i : members) {
              if (i == skip) continue;
              double[] p = points.get(i);
              double ex = p[0] - q[0], ey = p[1] - q[1], ez = p[2] - q[2], d = ex * ex + ey * ey + ez * ez;
              if (d <= best) {
                best = d;
                found = true;
              }
            }
          }
      return found ? Math.sqrt(best) : Double.POSITIVE_INFINITY;
    }

    int neighbours(double[] q, double radius, int skip, int enough) {
      int reach = (int) Math.ceil(radius / cell), count = 0;
      long cx = (long) Math.floor(q[0] / cell), cy = (long) Math.floor(q[1] / cell), cz = (long) Math.floor(q[2] / cell);
      double r2 = radius * radius;
      for (long dx = -reach; dx <= reach; dx++)
        for (long dy = -reach; dy <= reach; dy++)
          for (long dz = -reach; dz <= reach; dz++) {
            int[] members = cells.get(((cx + dx) & 0x1FFFFF) << 42 | ((cy + dy) & 0x1FFFFF) << 21 | ((cz + dz) & 0x1FFFFF));
            if (members == null) continue;
            for (int i : members) {
              if (i == skip) continue;
              double[] p = points.get(i);
              double ex = p[0] - q[0], ey = p[1] - q[1], ez = p[2] - q[2];
              if (ex * ex + ey * ey + ez * ez <= r2 && ++count >= enough) return count;
            }
          }
      return count;
    }
  }

  static int[] sample(int size, int limit) {
    int n = Math.min(size, limit);
    int[] out = new int[n];
    for (int i = 0; i < n; i++) out[i] = (int) ((long) i * size / n);
    return out;
  }

  static double medianSpacing(List<double[]> points, Grid grid) {
    int[] indices = sample(points.size(), 20_000);
    double[] d = new double[indices.length];
    Grid coarse = new Grid(points, 0.25);
    for (int i = 0; i < indices.length; i++) d[i] = coarse.nearest(points.get(indices[i]), 0.25, indices[i]);
    Arrays.sort(d);
    return Math.max(1e-4, d[d.length / 2]);
  }

  static double[] distances(List<double[]> from, List<double[]> to, Grid index, double limit, int samples) {
    int[] indices = sample(from.size(), samples);
    double[] out = new double[indices.length];
    java.util.stream.IntStream.range(0, indices.length).parallel()
        .forEach(i -> out[i] = index.nearest(from.get(indices[i]), limit, -1));
    Arrays.sort(out);
    return out;
  }

  static double fraction(double[] sorted, double limit) {
    int n = 0;
    for (double d : sorted) if (d <= limit) n++;
    return n / (double) Math.max(1, sorted.length);
  }

  static Object quantiles(double[] sorted, double matched, double far) {
    double[] finite = Arrays.stream(sorted).filter(Double::isFinite).toArray();
    Map<String, Object> m = new LinkedHashMap<>();
    m.put("samples", sorted.length);
    m.put("within_3s", fraction(sorted, matched));
    m.put("within_5s", fraction(sorted, far));
    if (finite.length > 0) {
      m.put("p50", finite[finite.length / 2]);
      m.put("p90", finite[(int) (finite.length * .9)]);
      m.put("rms_within_5s", Math.sqrt(Arrays.stream(finite).map(d -> d * d).average().orElse(0)));
    }
    return m;
  }

  static double detached(List<double[]> points, Grid index, double radius, int minimum, int samples) {
    int[] indices = sample(points.size(), samples);
    long isolated = java.util.stream.IntStream.range(0, indices.length).parallel()
        .filter(i -> index.neighbours(points.get(indices[i]), radius, indices[i], minimum) < minimum)
        .count();
    return isolated / (double) Math.max(1, indices.length);
  }

  static Object layers(List<double[]> reference, List<double[]> candidate, Grid candIndex, double limit) {
    HashMap<Long, List<Double>> columns = new HashMap<>();
    for (double[] p : reference) columns.computeIfAbsent(column(p, 5), k -> new ArrayList<>()).add(p[2]);
    HashMap<Long, Double> ground = new HashMap<>();
    for (var e : columns.entrySet()) {
      List<Double> z = e.getValue();
      Collections.sort(z);
      ground.put(e.getKey(), z.get((int) (z.size() * .02)));
    }
    String[] names = {"0-2m", "2-10m", "10m+"};
    long[] total = new long[3], found = new long[3];
    int[] indices = sample(reference.size(), 200_000);
    for (int i : indices) {
      double[] p = reference.get(i);
      double above = p[2] - ground.get(column(p, 5));
      int layer = above < 2 ? 0 : above < 10 ? 1 : 2;
      total[layer]++;
      if (candIndex.nearest(p, limit, -1) <= limit) found[layer]++;
    }
    Map<String, Object> out = new LinkedHashMap<>();
    for (int l = 0; l < 3; l++)
      out.put(names[l], Json.object("reference_samples", total[l], "completeness", found[l] / (double) Math.max(1, total[l])));
    return out;
  }

  static long column(double[] p, double size) {
    return ((long) Math.floor(p[0] / size) & 0xFFFFFFFL) << 32 | ((long) Math.floor(p[1] / size) & 0xFFFFFFFL);
  }

  /** 1 m columns: canopy max Z and ground min Z differences (candidate - other) over common cells. */
  static Object grids(List<double[]> candidate, List<double[]> other) {
    HashMap<Long, double[]> a = extremes(candidate), b = extremes(other);
    List<Double> canopy = new ArrayList<>(), groundDiff = new ArrayList<>();
    for (var e : a.entrySet()) {
      double[] q = b.get(e.getKey());
      if (q == null) continue;
      canopy.add(e.getValue()[1] - q[1]);
      groundDiff.add(e.getValue()[0] - q[0]);
    }
    long onlyCandidate = a.keySet().stream().filter(k -> !b.containsKey(k)).count(),
        onlyOther = b.keySet().stream().filter(k -> !a.containsKey(k)).count();
    return Json.object(
        "common_cells", canopy.size(),
        "candidate_only_cells", onlyCandidate,
        "other_only_cells", onlyOther,
        "canopy_max_difference", stats(canopy),
        "ground_min_difference", stats(groundDiff));
  }

  static HashMap<Long, double[]> extremes(List<double[]> points) {
    HashMap<Long, double[]> out = new HashMap<>();
    for (double[] p : points) {
      double[] e = out.computeIfAbsent(column(p, 1), k -> new double[] {Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY});
      e[0] = Math.min(e[0], p[2]);
      e[1] = Math.max(e[1], p[2]);
    }
    return out;
  }

  static Object stats(List<Double> values) {
    if (values.isEmpty()) return null;
    double[] v = values.stream().mapToDouble(Double::doubleValue).sorted().toArray();
    double median = v[v.length / 2];
    double[] centred = Arrays.stream(v).map(x -> Math.abs(x - median)).sorted().toArray();
    return Json.object(
        "median", median,
        "p10", v[(int) (v.length * .1)],
        "p90", v[(int) (v.length * .9)],
        "mad_about_median", centred[centred.length / 2],
        "within_0_5m_of_median", Arrays.stream(v).filter(x -> Math.abs(x - median) <= .5).count() / (double) v.length);
  }
}
