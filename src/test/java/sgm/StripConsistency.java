package sgm;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.*;
import java.util.zip.GZIPInputStream;

/**
 * Layering diagnostics for a finished run, from {@code 07_results/point-confidence.csv.gz} (create
 * it with {@code --export-confidence RUN}) and {@code 04_geometry/final/exterior.csv}.
 *
 * <p>Cameras are split into flight lines from their centre sequence (a new line starts where the
 * heading turns by more than 60 degrees or a step is unusually long). A point is "single-line" when
 * all its supporting cameras belong to one line. Per {@code cell}-metre grid cell, the low surface
 * of each line is the 10th height percentile of its single-line points; differences between
 * adjacent lines in the same cell measure line-specific depth bias, which shows up as layered
 * ground. Ground-band thickness is the 10–90% height spread of points within 2.5 m of each 1 m
 * cell's lowest point. Usage: StripConsistency RUN [cell metres] [json out].
 */
final class StripConsistency {
  public static void main(String[] args) throws Exception {
    if (args[0].toLowerCase(Locale.ROOT).endsWith(".las")) {
      // Cloud only (e.g. a Metashape reference): layering with an assumed depth for the ratio.
      double depth = args.length > 1 ? Double.parseDouble(args[1]) : 100;
      Map<Long, List<double[]>> sheets = new HashMap<>();
      for (double[] p : CloudDiagnostics.readLas(Path.of(args[0])))
        sheets.computeIfAbsent(key(p[0], p[1], .5), k -> new ArrayList<>()).add(new double[] {p[2], 0, depth});
      Map<String, Object> out = new LinkedHashMap<>();
      out.put("cloud", Path.of(args[0]).getFileName().toString());
      layers(sheets, out);
      System.out.println(Json.string(out));
      return;
    }
    Path run = Path.of(args[0]);
    double cell = args.length > 1 ? Double.parseDouble(args[1]) : 2;
    Map<String, Object> report = analyse(run, cell);
    String json = Json.string(report);
    if (args.length > 2) Files.writeString(Path.of(args[2]), json);
    System.out.println(json);
  }

  static Map<String, Object> analyse(Path run, double cell) throws IOException {
    List<double[]> centres = new ArrayList<>();
    try (BufferedReader r = Files.newBufferedReader(run.resolve("04_geometry/final/exterior.csv"))) {
      String[] head = split(r.readLine());
      int cx = index(head, "center_x"), cy = index(head, "center_y"), cz = index(head, "center_z");
      for (String line; (line = r.readLine()) != null; ) {
        String[] f = split(line);
        centres.add(
            new double[] {
              Double.parseDouble(f[cx]), Double.parseDouble(f[cy]), Double.parseDouble(f[cz])
            });
      }
    }
    int[] lineOf = flightLines(centres);
    int lines = Arrays.stream(lineOf).max().orElse(-1) + 1;
    Map<Long, Map<Integer, List<Double>>> low = new HashMap<>();
    Map<Long, List<Double>> fine = new HashMap<>();
    Map<Long, List<double[]>> sheets = new HashMap<>();
    long points = 0, single = 0;
    Path csv = run.resolve("07_results/point-confidence.csv.gz");
    try (BufferedReader r =
        new BufferedReader(
            new InputStreamReader(
                new GZIPInputStream(new BufferedInputStream(Files.newInputStream(csv), 1 << 20)),
                StandardCharsets.UTF_8))) {
      String[] head = r.readLine().split(",");
      int ix = index(head, "x"), iy = index(head, "y"), iz = index(head, "z");
      int ic = index(head, "supporting_camera_ids");
      // -Dsgm.sample=N keeps every Nth point, for comparing clouds of different density.
      int sample = Integer.getInteger("sgm.sample", 1);
      long row = 0;
      for (String line; (line = r.readLine()) != null; ) {
        if (row++ % sample != 0) continue;
        String[] f = line.split(",");
        double x = Double.parseDouble(f[ix]), y = Double.parseDouble(f[iy]), z = Double.parseDouble(f[iz]);
        points++;
        int strip = -2;
        int[] votes = new int[lines];
        for (String id : f[ic].split(";")) {
          int s = lineOf[Integer.parseInt(id)];
          votes[s]++;
          strip = strip == -2 ? s : strip == s ? s : -1;
        }
        if (strip >= 0) single++;
        int major = 0;
        for (int s = 1; s < lines; s++) if (votes[s] > votes[major]) major = s;
        strip = major;
        fine.computeIfAbsent(key(x, y, 1), k -> new ArrayList<>()).add(z);
        double depth = 0;
        String[] ids = f[ic].split(";");
        for (String id : ids) depth += centres.get(Integer.parseInt(id))[2] - z;
        sheets.computeIfAbsent(key(x, y, .5), k -> new ArrayList<>())
            .add(new double[] {z, major, depth / ids.length});
        low.computeIfAbsent(key(x, y, cell), k -> new HashMap<>())
            .computeIfAbsent(strip, k -> new ArrayList<>())
            .add(z);
      }
    }
    List<Double> gaps = new ArrayList<>();
    double[][] signed = new double[Math.max(1, lines)][2];
    for (Map<Integer, List<Double>> byLine : low.values())
      for (var e : byLine.entrySet()) {
        List<Double> next = byLine.get(e.getKey() + 1);
        if (e.getValue().size() < 8 || next == null || next.size() < 8) continue;
        double d = quantile(next, .1) - quantile(e.getValue(), .1);
        gaps.add(Math.abs(d));
        signed[e.getKey()][0] += d;
        signed[e.getKey()][1]++;
      }
    List<Double> thickness = new ArrayList<>();
    for (List<Double> zs : fine.values()) {
      if (zs.size() < 10) continue;
      double min = Collections.min(zs);
      List<Double> band = new ArrayList<>();
      for (double z : zs) if (z <= min + 2.5) band.add(z);
      if (band.size() >= 10) thickness.add(quantile(band, .9) - quantile(band, .1));
    }
    List<Object> perPair = new ArrayList<>();
    for (int s = 0; s + 1 < lines; s++)
      if (signed[s][1] > 0)
        perPair.add(
            Json.object(
                "lines", s + "-" + (s + 1),
                "cells", (long) signed[s][1],
                "mean_signed_gap_m", signed[s][0] / signed[s][1]));
    Map<String, Object> out = new LinkedHashMap<>();
    out.put("run", run.getFileName().toString());
    out.put("points", points);
    out.put("flight_lines", lines);
    out.put("single_line_fraction", single / (double) Math.max(1, points));
    out.put("adjacent_line_cells", gaps.size());
    out.put("adjacent_line_gap_median_m", gaps.isEmpty() ? Double.NaN : quantile(gaps, .5));
    out.put("adjacent_line_gap_p90_m", gaps.isEmpty() ? Double.NaN : quantile(gaps, .9));
    out.put("per_line_pair", perPair);
    out.put("ground_band_cells", thickness.size());
    out.put("ground_band_thickness_median_m", thickness.isEmpty() ? Double.NaN : quantile(thickness, .5));
    out.put("ground_band_thickness_p90_m", thickness.isEmpty() ? Double.NaN : quantile(thickness, .9));
    layers(sheets, out);
    return out;
  }

  /**
   * Two-sheet ground cells: in each 0.5 m cell, the points within 3 m of the lowest are sorted by
   * height; the cell is layered when its largest gap with at least 3 points on each side is 0.3 m
   * or more. Reports the gap relative to depth (compare with --fusion-relative) and whether the
   * two sheets come from different majority flight lines.
   */
  static void layers(Map<Long, List<double[]>> sheets, Map<String, Object> out) {
    long cells = 0, layered = 0, crossLine = 0;
    List<Double> relative = new ArrayList<>(), metres = new ArrayList<>();
    for (List<double[]> all : sheets.values()) {
      if (all.size() < 6) continue;
      double min = Double.POSITIVE_INFINITY;
      for (double[] p : all) min = Math.min(min, p[0]);
      List<double[]> band = new ArrayList<>();
      for (double[] p : all) if (p[0] <= min + 3) band.add(p);
      if (band.size() < 6) continue;
      cells++;
      band.sort(Comparator.comparingDouble(p -> p[0]));
      double best = 0;
      int at = -1;
      for (int i = 3; i <= band.size() - 3; i++) {
        double g = band.get(i)[0] - band.get(i - 1)[0];
        if (g > best) {
          best = g;
          at = i;
        }
      }
      if (best < .3) continue;
      layered++;
      metres.add(best);
      double depth = 0;
      for (double[] p : band) depth += p[2] / band.size();
      relative.add(best / Math.max(1e-6, depth));
      int[] lower = new int[64], upper = new int[64];
      for (int i = 0; i < band.size(); i++) (i < at ? lower : upper)[(int) band.get(i)[1] & 63]++;
      if (argmax(lower) != argmax(upper)) crossLine++;
    }
    out.put("sheet_cells", cells);
    out.put("layered_cell_fraction", layered / (double) Math.max(1, cells));
    out.put("layer_gap_median_m", metres.isEmpty() ? Double.NaN : quantile(metres, .5));
    out.put("layer_gap_relative_median", relative.isEmpty() ? Double.NaN : quantile(relative, .5));
    out.put("layer_gap_relative_p10", relative.isEmpty() ? Double.NaN : quantile(relative, .1));
    out.put("layer_gap_relative_p90", relative.isEmpty() ? Double.NaN : quantile(relative, .9));
    out.put("layered_cells_from_different_lines", crossLine / (double) Math.max(1, layered));
  }

  private static int argmax(int[] v) {
    int best = 0;
    for (int i = 1; i < v.length; i++) if (v[i] > v[best]) best = i;
    return best;
  }

  /**
   * Parallel flight lines: cameras are grouped by their coordinate across the dominant flight
   * direction (principal axis of the centres); a new line starts at a gap far larger than the
   * typical spacing of sorted across-track coordinates.
   */
  static int[] flightLines(List<double[]> c) {
    int n = c.size();
    int[] line = new int[n];
    if (n < 3) return line;
    double mx = 0, my = 0;
    for (double[] p : c) {
      mx += p[0] / n;
      my += p[1] / n;
    }
    double sxx = 0, sxy = 0, syy = 0;
    for (double[] p : c) {
      sxx += (p[0] - mx) * (p[0] - mx);
      sxy += (p[0] - mx) * (p[1] - my);
      syy += (p[1] - my) * (p[1] - my);
    }
    double angle = .5 * Math.atan2(2 * sxy, sxx - syy);
    // Across-track unit vector is perpendicular to the principal (along-track) axis.
    double ux = -Math.sin(angle), uy = Math.cos(angle);
    Integer[] order = new Integer[n];
    double[] across = new double[n];
    for (int i = 0; i < n; i++) {
      order[i] = i;
      across[i] = (c.get(i)[0] - mx) * ux + (c.get(i)[1] - my) * uy;
    }
    Arrays.sort(order, Comparator.comparingDouble(i -> across[i]));
    double[] gaps = new double[n - 1];
    for (int k = 1; k < n; k++) gaps[k - 1] = across[order[k]] - across[order[k - 1]];
    double[] sorted = gaps.clone();
    Arrays.sort(sorted);
    double typical = Math.max(1e-6, sorted[sorted.length / 2]);
    int current = 0;
    line[order[0]] = 0;
    for (int k = 1; k < n; k++) {
      if (gaps[k - 1] > 10 * typical && gaps[k - 1] > 1e-3) current++;
      line[order[k]] = current;
    }
    return line;
  }
  private static long key(double x, double y, double size) {
    long i = (long) Math.floor(x / size), j = (long) Math.floor(y / size);
    return i * 4_000_003L + j;
  }

  private static double quantile(List<Double> v, double q) {
    double[] a = v.stream().mapToDouble(Double::doubleValue).sorted().toArray();
    return a[(int) Math.min(a.length - 1, Math.floor(q * (a.length - 1) + .5))];
  }

  private static int index(String[] head, String name) {
    for (int i = 0; i < head.length; i++) if (head[i].equals(name)) return i;
    throw new IllegalArgumentException("Missing column " + name);
  }

  private static String[] split(String line) {
    return line.split(",(?=(?:[^\"]*\"[^\"]*\")*[^\"]*$)");
  }
}
