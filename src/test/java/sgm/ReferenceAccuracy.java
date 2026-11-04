package sgm;

import java.nio.file.*;
import java.util.*;

/**
 * Accuracy of a candidate LAS against an independent reference LAS of the same area (for example
 * forest_tiny's Metashape cloud with the frozen cameras). Not ground truth: the reference has its
 * own errors, and a constant vertical frame offset is removed first (median difference of 1 m
 * ground cells, i.e. per-cell minimum Z).
 *
 * <p>Reports, after that shift: candidate-to-reference nearest-neighbour distances (accuracy),
 * reference-to-candidate distances (completeness), and per-cell ground differences. Usage:
 * ReferenceAccuracy candidate.las reference.las [out.json].
 */
final class ReferenceAccuracy {
  public static void main(String[] args) throws Exception {
    List<double[]> cand = CloudDiagnostics.readLas(Path.of(args[0]));
    List<double[]> ref = CloudDiagnostics.readLas(Path.of(args[1]));
    Map<Long, Double> cg = ground(cand), rg = ground(ref);
    List<Double> diffs = new ArrayList<>();
    for (var e : cg.entrySet()) if (rg.containsKey(e.getKey())) diffs.add(e.getValue() - rg.get(e.getKey()));
    double dz = quantile(diffs, .5);
    List<Double> groundAbs = new ArrayList<>();
    for (double d : diffs) groundAbs.add(Math.abs(d - dz));
    Map<Long, List<double[]>> refGrid = grid(ref, .5);
    // Refine to a full 3D translation: median nearest-neighbour offset, iterated (frames may also
    // differ horizontally). Only a translation is removed, never scale or rotation.
    double[] t = {0, 0, dz};
    for (int iteration = 0; iteration < 6; iteration++) {
      double[] step = medianOffset(shift(cand, t), refGrid, 100_000);
      for (int k = 0; k < 3; k++) t[k] += step[k];
    }
    List<double[]> moved = shift(cand, t);
    Map<Long, List<double[]>> candGrid = grid(moved, .5);
    double[] acc = distances(moved, refGrid, 300_000);
    double[] comp = distances(ref, candGrid, 300_000);
    Map<String, Object> out = new LinkedHashMap<>();
    out.put("candidate", args[0]);
    out.put("candidate_points", cand.size());
    out.put("reference_points", ref.size());
    out.put("vertical_frame_offset_from_ground_m", dz);
    out.put("translation_removed_m", t);
    out.put("ground_cells_compared", diffs.size());
    out.put("ground_abs_diff_median_m", quantile(groundAbs, .5));
    out.put("ground_abs_diff_p90_m", quantile(groundAbs, .9));
    out.put("accuracy_nn_median_m", median(acc));
    out.put("accuracy_nn_p90_m", percentile(acc, .9));
    out.put("accuracy_within_0_10m", within(acc, .1));
    out.put("accuracy_within_0_25m", within(acc, .25));
    out.put("completeness_within_0_10m", within(comp, .1));
    out.put("completeness_within_0_25m", within(comp, .25));
    String json = Json.string(out);
    if (args.length > 2) Files.writeString(Path.of(args[2]), json);
    System.out.println(json);
  }

  private static Map<Long, Double> ground(List<double[]> pts) {
    Map<Long, Double> g = new HashMap<>();
    for (double[] p : pts) g.merge(key(p[0], p[1], 1), p[2], Math::min);
    return g;
  }

  private static List<double[]> shift(List<double[]> pts, double dz) {
    return shift(pts, new double[] {0, 0, dz});
  }

  private static List<double[]> shift(List<double[]> pts, double[] t) {
    List<double[]> out = new ArrayList<>(pts.size());
    for (double[] p : pts) out.add(new double[] {p[0] - t[0], p[1] - t[1], p[2] - t[2]});
    return out;
  }

  /** Per-axis median of (candidate - nearest reference) over pairs closer than 1 m. */
  private static double[] medianOffset(List<double[]> from, Map<Long, List<double[]>> to, int cap) {
    int stride = Math.max(1, from.size() / cap);
    List<Double> dx = new ArrayList<>(), dy = new ArrayList<>(), dzs = new ArrayList<>();
    for (int i = 0; i < from.size(); i += stride) {
      double[] p = from.get(i), q = nearest(p, to);
      if (q == null) continue;
      dx.add(p[0] - q[0]);
      dy.add(p[1] - q[1]);
      dzs.add(p[2] - q[2]);
    }
    return dx.isEmpty() ? new double[3] : new double[] {quantile(dx, .5), quantile(dy, .5), quantile(dzs, .5)};
  }

  private static double[] nearest(double[] p, Map<Long, List<double[]>> to) {
    double best = 1;
    double[] found = null;
    long ix = (long) Math.floor(p[0] / .5), iy = (long) Math.floor(p[1] / .5), iz = (long) Math.floor(p[2] / .5);
    for (long a = -2; a <= 2; a++)
      for (long b = -2; b <= 2; b++)
        for (long c = -2; c <= 2; c++) {
          List<double[]> cell = to.get(pack(ix + a, iy + b, iz + c));
          if (cell == null) continue;
          for (double[] q : cell) {
            double d = Math.sqrt((p[0] - q[0]) * (p[0] - q[0]) + (p[1] - q[1]) * (p[1] - q[1]) + (p[2] - q[2]) * (p[2] - q[2]));
            if (d < best) {
              best = d;
              found = q;
            }
          }
        }
    return found;
  }

  private static Map<Long, List<double[]>> grid(List<double[]> pts, double size) {
    Map<Long, List<double[]>> g = new HashMap<>();
    for (double[] p : pts) g.computeIfAbsent(key3(p, size), k -> new ArrayList<>()).add(p);
    return g;
  }

  /** Nearest-neighbour distances (capped at 1 m; beyond counts as 1 m) for an even sample. */
  private static double[] distances(List<double[]> from, Map<Long, List<double[]>> to, int cap) {
    int stride = Math.max(1, from.size() / cap);
    double[] out = new double[(from.size() + stride - 1) / stride];
    for (int s = 0, i = 0; i < from.size(); i += stride, s++) {
      double[] p = from.get(i);
      double best = 1;
      long ix = (long) Math.floor(p[0] / .5), iy = (long) Math.floor(p[1] / .5), iz = (long) Math.floor(p[2] / .5);
      for (long a = -2; a <= 2; a++)
        for (long b = -2; b <= 2; b++)
          for (long c = -2; c <= 2; c++) {
            List<double[]> cell = to.get(pack(ix + a, iy + b, iz + c));
            if (cell == null) continue;
            for (double[] q : cell) {
              double d = Math.sqrt((p[0] - q[0]) * (p[0] - q[0]) + (p[1] - q[1]) * (p[1] - q[1]) + (p[2] - q[2]) * (p[2] - q[2]));
              if (d < best) best = d;
            }
          }
      out[s] = best;
    }
    return out;
  }

  private static long key(double x, double y, double size) {
    return (long) Math.floor(x / size) * 4_000_003L + (long) Math.floor(y / size);
  }

  private static long key3(double[] p, double size) {
    return pack((long) Math.floor(p[0] / size), (long) Math.floor(p[1] / size), (long) Math.floor(p[2] / size));
  }

  private static long pack(long x, long y, long z) {
    return (x & 0x1FFFFF) << 42 | (y & 0x1FFFFF) << 21 | (z & 0x1FFFFF);
  }

  private static double median(double[] v) {
    return percentile(v, .5);
  }

  private static double percentile(double[] v, double q) {
    double[] a = v.clone();
    Arrays.sort(a);
    return a.length == 0 ? Double.NaN : a[(int) Math.min(a.length - 1, Math.floor(q * (a.length - 1) + .5))];
  }

  private static double within(double[] v, double limit) {
    int n = 0;
    for (double d : v) if (d <= limit) n++;
    return v.length == 0 ? Double.NaN : n / (double) v.length;
  }

  private static double quantile(List<Double> v, double q) {
    double[] a = v.stream().mapToDouble(Double::doubleValue).toArray();
    return percentile(a, q);
  }
}
