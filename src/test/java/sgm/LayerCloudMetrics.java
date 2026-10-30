package sgm;

import java.nio.file.*;
import java.util.*;

/**
 * Read-only local thickness comparison on common bright, neutral-coloured patches; not ground
 * truth.
 */
public final class LayerCloudMetrics {
  private static final double CELL = 1;

  private static Map<String, List<double[]>> cells(List<double[]> cloud) {
    Map<String, List<double[]>> out = new TreeMap<>();
    for (double[] p : cloud) {
      int rgb = (int) p[3], r = (rgb >>> 16) & 255, g = (rgb >>> 8) & 255, b = rgb & 255;
      if (Math.min(r, Math.min(g, b)) < 90
          || Math.max(r, Math.max(g, b)) - Math.min(r, Math.min(g, b)) > 50) continue;
      long x = (long) Math.floor(p[0] / CELL), y = (long) Math.floor(p[1] / CELL);
      out.computeIfAbsent(x + ":" + y, k -> new ArrayList<>()).add(p);
    }
    return out;
  }

  private static double thickness(List<double[]> points) {
    double x0 = 0, y0 = 0, z0 = 0;
    for (double[] p : points) {
      x0 += p[0] / points.size();
      y0 += p[1] / points.size();
      z0 += p[2] / points.size();
    }
    double[][] h = new double[3][3];
    double[] b = new double[3];
    for (double[] p : points) {
      double[] row = {p[0] - x0, p[1] - y0, 1};
      for (int i = 0; i < 3; i++) {
        b[i] += row[i] * (p[2] - z0);
        for (int j = 0; j < 3; j++) h[i][j] += row[i] * row[j];
      }
    }
    double[] plane = Math3.solve(h, b);
    if (plane == null) return Double.NaN;
    double[] residuals = new double[points.size()];
    double norm = Math.sqrt(1 + plane[0] * plane[0] + plane[1] * plane[1]);
    for (int i = 0; i < points.size(); i++) {
      double[] p = points.get(i);
      residuals[i] =
          (p[2] - z0 - plane[0] * (p[0] - x0) - plane[1] * (p[1] - y0) - plane[2]) / norm;
    }
    Arrays.sort(residuals);
    return residuals[(int) ((residuals.length - 1) * .9)]
        - residuals[(int) ((residuals.length - 1) * .1)];
  }

  private static double q(List<Double> values, double fraction) {
    Collections.sort(values);
    return values.isEmpty() ? Double.NaN : values.get((int) ((values.size() - 1) * fraction));
  }

  public static void main(String[] args) throws Exception {
    List<double[]> a = CloudDiagnostics.readLas(Path.of(args[0])),
        b = CloudDiagnostics.readLas(Path.of(args[1]));
    Map<String, List<double[]>> ca = cells(a), cb = cells(b);
    List<Double> ta = new ArrayList<>(), tb = new ArrayList<>();
    List<String> rows = new ArrayList<>();
    rows.add(
        "cell,before_count,after_count,before_normal_thickness_p90_p10,after_normal_thickness_p90_p10");
    for (String key : ca.keySet())
      if (cb.containsKey(key) && ca.get(key).size() >= 8 && cb.get(key).size() >= 8) {
        double x = thickness(ca.get(key)), y = thickness(cb.get(key));
        if (!Double.isFinite(x + y)) continue;
        ta.add(x);
        tb.add(y);
        rows.add(key + "," + ca.get(key).size() + "," + cb.get(key).size() + "," + x + "," + y);
      }
    Path out = Path.of(args[2]);
    Files.createDirectories(out);
    Files.write(out.resolve("common-patch-thickness.csv"), rows);
    Object report =
        Json.object(
            "before_points",
            a.size(),
            "after_points",
            b.size(),
            "retained_fraction",
            b.size() / (double) a.size(),
            "cell_size",
            CELL,
            "before_neutral_cells",
            ca.size(),
            "after_neutral_cells",
            cb.size(),
            "common_cells_min_8_points",
            ta.size(),
            "median_thickness_before",
            q(ta, .5),
            "median_thickness_after",
            q(tb, .5),
            "p95_thickness_before",
            q(ta, .95),
            "p95_thickness_after",
            q(tb, .95),
            "definition",
            "Orthogonal residual p90-p10 after fitting one local plane per one-metre XY cell;"
                + " common bright/neutral patches only. Terrain/material proxy, not surveyed ground"
                + " classification or accuracy.");
    Json.write(out.resolve("metrics.json"), report);
    System.out.println(Json.string(report));
  }
}
