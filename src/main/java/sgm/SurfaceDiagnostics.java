package sgm;

import java.io.*;
import java.nio.*;
import java.nio.channels.*;
import java.nio.file.*;
import java.util.*;
import java.util.zip.GZIPInputStream;

/**
 * Read-only sampled surface diagnostics. A dominant plane is a hypothesis, never a terrain filter.
 */
final class SurfaceDiagnostics {
  record Point(long id, double x, double y, double z) {}

  record Plane(double x, double y, double z, double a, double b) {
    double residual(Point p) {
      return p.z - z - a * (p.x - x) - b * (p.y - y);
    }

    double height(double px, double py) {
      return z + a * (px - x) + b * (py - y);
    }
  }

  record Cell(
      int x,
      int y,
      List<Point> points,
      Plane plane,
      double[] quantiles,
      double medianOffset,
      double inliers) {}

  static Path analyze(Options o) throws IOException {
    Path run = o.analyzeRun, ply = run.resolve("07_results/dense.ply");
    if (!Files.exists(ply)) ply = run.resolve("dense.ply");
    Path out =
        o.analysisOutput != null
            ? o.analysisOutput
            : run.resolve("surface_analysis_" + System.currentTimeMillis());
    Files.createDirectories(out);
    List<Point> points = sample(ply, o.analysisSamples, o.analysisRegion);
    if (points.size() < 20)
      throw new IllegalArgumentException("Too few sampled points in the analysis region");
    double[] xs = points.stream().mapToDouble(Point::x).sorted().toArray(),
        ys = points.stream().mapToDouble(Point::y).sorted().toArray();
    double x0 = xs[(int) (xs.length * .001)],
        x1 = xs[(int) ((xs.length - 1) * .999)],
        y0 = ys[(int) (ys.length * .001)],
        y1 = ys[(int) ((ys.length - 1) * .999)];
    double size = o.analysisCellSize > 0 ? o.analysisCellSize : Math.max(x1 - x0, y1 - y0) / 24;
    if (!(size > 0)) throw new IllegalArgumentException("Analysis region has no horizontal extent");
    double tolerance = o.analysisTolerance > 0 ? o.analysisTolerance : size * .01;
    Plane global = fit(points, tolerance, 1001);
    Map<Long, List<Point>> groups = new TreeMap<>();
    for (Point p : points) {
      if (p.x < x0 || p.x > x1 || p.y < y0 || p.y > y1) continue;
      int x = (int) Math.floor((p.x - x0) / size), y = (int) Math.floor((p.y - y0) / size);
      groups.computeIfAbsent(key(x, y), ignored -> new ArrayList<>()).add(p);
    }
    List<Cell> cells = new ArrayList<>();
    Map<Long, Cell> byCell = new HashMap<>();
    int below = 0;
    try (BufferedWriter table = Files.newBufferedWriter(out.resolve("surface_cells.csv"));
        BufferedWriter samples = Files.newBufferedWriter(out.resolve("sampled_residuals.csv"))) {
      table.write(
          "cell_x,cell_y,samples,center_x,center_y,plane_z,plane_slope_x,plane_slope_y,inlier_fraction,residual_p01,residual_p10,residual_p50,residual_p90,residual_p99,p90_p10_thickness,median_height_from_global_plane,below_3_tolerance_fraction\n");
      samples.write(
          "point_id,x,y,z,cell_x,cell_y,local_plane_residual,below_3_tolerance_candidate\n");
      for (var entry : groups.entrySet()) {
        List<Point> group = entry.getValue();
        if (group.size() < 20) continue;
        Plane plane = fit(group, tolerance, entry.getKey());
        if (plane == null) continue;
        double[] residual = group.stream().mapToDouble(plane::residual).sorted().toArray();
        int inliers = 0, negative = 0;
        for (double r : residual) {
          if (Math.abs(r) <= tolerance) inliers++;
          if (r < -3 * tolerance) negative++;
        }
        double[] q = {
          quantile(residual, .01),
          quantile(residual, .1),
          quantile(residual, .5),
          quantile(residual, .9),
          quantile(residual, .99)
        };
        int x = (int) (entry.getKey() >> 32), y = (int) (long) entry.getKey();
        double cx = x0 + (x + .5) * size, cy = y0 + (y + .5) * size;
        double offset =
            global == null ? Double.NaN : plane.height(cx, cy) + q[2] - global.height(cx, cy);
        Cell cell = new Cell(x, y, group, plane, q, offset, inliers / (double) group.size());
        cells.add(cell);
        byCell.put(entry.getKey(), cell);
        below += negative;
        table.write(
            x
                + ","
                + y
                + ","
                + group.size()
                + ","
                + cx
                + ","
                + cy
                + ","
                + plane.height(cx, cy)
                + ","
                + plane.a
                + ","
                + plane.b
                + ","
                + cell.inliers);
        for (double v : q) table.write("," + v);
        table.write(
            "," + (q[3] - q[1]) + "," + offset + "," + negative / (double) group.size() + "\n");
        for (Point p : group) {
          double r = plane.residual(p);
          samples.write(
              p.id
                  + ","
                  + p.x
                  + ","
                  + p.y
                  + ","
                  + p.z
                  + ","
                  + x
                  + ","
                  + y
                  + ","
                  + r
                  + ","
                  + (r < -3 * tolerance)
                  + "\n");
        }
      }
    }
    svg(out.resolve("height_offsets.svg"), cells, false, size, x0, y0);
    svg(out.resolve("surface_thickness.svg"), cells, true, size, x0, y0);
    Map<String, Object> stats =
        Json.object(
            "source_run",
            run.toAbsolutePath().toString(),
            "sampled_points",
            points.size(),
            "cells",
            cells.size(),
            "cell_size",
            size,
            "plane_tolerance",
            tolerance,
            "below_plane_candidate_samples",
            below,
            "analysis_region",
            o.analysisRegion,
            "global_plane",
            global == null
                ? null
                : Json.object(
                    "origin",
                    new double[] {global.x, global.y, global.z},
                    "slope_x",
                    global.a,
                    "slope_y",
                    global.b),
            "median_cell_thickness_p90_p10",
            median(cells.stream().mapToDouble(c -> c.quantiles[3] - c.quantiles[1]).toArray()),
            "median_absolute_cell_offset_from_global_plane",
            median(
                cells.stream()
                    .mapToDouble(c -> Math.abs(c.medianOffset))
                    .filter(Double::isFinite)
                    .toArray()),
            "scope",
            "Dominant-plane surface proxies, not a ground classification or surveyed accuracy"
                + " measurement. Real slopes, steps, stacked surfaces and vegetation can explain"
                + " spread/negative residuals. No points removed.");
    if (o.analysisConfidence)
      stats.put(
          "confidence_attribution", confidence(run, out, points, byCell, x0, y0, size, tolerance));
    Json.write(out.resolve("surface_summary.json"), stats);
    System.out.println(
        "Surface analysis: "
            + out.toAbsolutePath()
            + "; "
            + points.size()
            + " samples, "
            + cells.size()
            + " local planes; no cloud modified");
    return out;
  }

  static double median(double[] data) {
    if (data.length == 0) return Double.NaN;
    Arrays.sort(data);
    return data[data.length / 2];
  }

  private static double quantile(double[] sorted, double fraction) {
    return sorted[(int) Math.floor((sorted.length - 1) * fraction)];
  }

  private static long key(int x, int y) {
    return ((long) x << 32) | (y & 0xffffffffL);
  }

  static Plane fit(List<Point> p, double tolerance, long seed) {
    double x = 0, y = 0, z = 0;
    for (Point q : p) {
      x += q.x / p.size();
      y += q.y / p.size();
      z += q.z / p.size();
    }
    Random random = new Random(seed);
    Plane best = null;
    int bestCount = 0;
    for (int trial = 0; trial < 64; trial++) {
      Point a = p.get(random.nextInt(p.size())),
          b = p.get(random.nextInt(p.size())),
          c = p.get(random.nextInt(p.size()));
      double bx = b.x - a.x,
          by = b.y - a.y,
          cx = c.x - a.x,
          cy = c.y - a.y,
          det = bx * cy - by * cx;
      if (Math.abs(det) < 1e-10) continue;
      double ax = ((b.z - a.z) * cy - (c.z - a.z) * by) / det,
          ay = (bx * (c.z - a.z) - cx * (b.z - a.z)) / det;
      if (!Double.isFinite(ax + ay)) continue;
      Plane model = new Plane(a.x, a.y, a.z, ax, ay);
      int count = 0;
      for (Point q : p) if (Math.abs(model.residual(q)) < tolerance) count++;
      if (count > bestCount) {
        bestCount = count;
        best = model;
      }
    }
    if (best == null || bestCount < Math.max(8, p.size() / 10)) return null;
    for (int iteration = 0; iteration < 3; iteration++) {
      double[][] A = new double[3][3];
      double[] B = new double[3];
      for (Point q : p)
        if (Math.abs(best.residual(q)) <= tolerance) {
          double[] v = {q.x - x, q.y - y, 1};
          for (int i = 0; i < 3; i++) {
            B[i] += v[i] * (q.z - z);
            for (int j = 0; j < 3; j++) A[i][j] += v[i] * v[j];
          }
        }
      double[] solution = Math3.solve(A, B);
      if (solution == null) break;
      best = new Plane(x, y, z + solution[2], solution[0], solution[1]);
    }
    return best;
  }

  static List<Point> sample(Path file, int maximum, String region) throws IOException {
    long count = 0;
    int header = 0, recordBytes = 27;
    try (InputStream in = Files.newInputStream(file)) {
      String line;
      do {
        ByteArrayOutputStream text = new ByteArrayOutputStream();
        int c;
        while ((c = in.read()) >= 0) {
          header++;
          if (c == '\n') break;
          text.write(c);
        }
        line = text.toString(java.nio.charset.StandardCharsets.US_ASCII).strip();
        if (line.equals("property float nx")) recordBytes = 59;
        if (line.startsWith("element vertex ")) count = Long.parseLong(line.substring(15));
        if (line.startsWith("format ") && !line.equals("format binary_little_endian 1.0"))
          throw new IllegalArgumentException("Surface analysis requires binary little-endian PLY");
        if (header > 65536 || c < 0) throw new IOException("Invalid PLY header");
      } while (!line.equals("end_header"));
    }
    long stride = Math.max(1, (count + maximum - 1) / maximum);
    double[] roi = region == null ? null : OrientationIO.numbers(region);
    if (roi != null && (roi.length != 4 || roi[0] >= roi[2] || roi[1] >= roi[3]))
      throw new IllegalArgumentException("Analysis region must be minX,minY,maxX,maxY");
    List<Point> points = new ArrayList<>();
    try (FileChannel in = FileChannel.open(file)) {
      if (in.size() != header + count * recordBytes)
        throw new IllegalArgumentException(
            "Surface analysis currently supports this program's XYZ-double/RGB PLY layout");
      in.position(header);
      ByteBuffer data =
          ByteBuffer.allocateDirect(recordBytes * 8192).order(ByteOrder.LITTLE_ENDIAN);
      for (long start = 0; start < count; start += 8192) {
        int n = (int) Math.min(8192, count - start);
        data.clear();
        data.limit(n * recordBytes);
        while (data.hasRemaining()) if (in.read(data) < 0) throw new EOFException();
        for (int i = 0; i < n; i++)
          if ((start + i) % stride == 0) {
            double x = data.getDouble(i * recordBytes),
                y = data.getDouble(i * recordBytes + 8),
                z = data.getDouble(i * recordBytes + 16);
            if (Double.isFinite(x + y + z)
                && (roi == null || x >= roi[0] && x <= roi[2] && y >= roi[1] && y <= roi[3]))
              points.add(new Point(start + i, x, y, z));
          }
      }
    }
    return points;
  }

  private static Object confidence(
      Path run,
      Path out,
      List<Point> points,
      Map<Long, Cell> cells,
      double x0,
      double y0,
      double size,
      double tolerance)
      throws IOException {
    Path file = run.resolve("07_results/point-confidence.csv.gz");
    Map<Long, Point> ids = new HashMap<>();
    for (Point p : points) ids.put(p.id, p);
    Map<Integer, List<Double>> residuals = new TreeMap<>();
    long rows = 0, matched = 0, progress = System.nanoTime();
    try (BufferedReader reader =
            new BufferedReader(
                new InputStreamReader(new GZIPInputStream(Files.newInputStream(file), 65536)));
        BufferedWriter writer =
            Files.newBufferedWriter(out.resolve("below_plane_attribution.csv"))) {
      reader.readLine();
      writer.write(
          "point_id,fusion_pair_id,x,y,z,local_residual,supporting_views,independent_pairs,relative_sigma,refinement_shift\n");
      String line;
      while ((line = reader.readLine()) != null) {
        if (ids.containsKey(rows)) {
          String[] columns = line.split(",");
          Point p = ids.get(rows);
          Cell cell =
              cells.get(
                  key((int) Math.floor((p.x - x0) / size), (int) Math.floor((p.y - y0) / size)));
          if (cell != null) {
            double residual = cell.plane.residual(p);
            int source = Integer.parseInt(columns[2]);
            residuals.computeIfAbsent(source, ignored -> new ArrayList<>()).add(residual);
            matched++;
            if (residual < -3 * tolerance)
              writer.write(
                  p.id
                      + ","
                      + source
                      + ","
                      + p.x
                      + ","
                      + p.y
                      + ","
                      + p.z
                      + ","
                      + residual
                      + ","
                      + columns[9]
                      + ","
                      + columns[10]
                      + ","
                      + columns[11]
                      + ","
                      + columns[12]
                      + "\n");
          }
        }
        rows++;
        if ((rows & 65535) == 0 && System.nanoTime() - progress > 5_000_000_000L) {
          System.out.printf(
              Locale.ROOT, "Surface attribution: %,d confidence rows scanned%n", rows);
          progress = System.nanoTime();
        }
      }
    }
    List<Object> report = new ArrayList<>();
    for (var entry : residuals.entrySet()) {
      double[] r = entry.getValue().stream().mapToDouble(Double::doubleValue).sorted().toArray();
      report.add(
          Json.object(
              "fusion_pair_id",
              entry.getKey(),
              "samples",
              r.length,
              "median_residual",
              quantile(r, .5),
              "p10",
              quantile(r, .1),
              "p90",
              quantile(r, .9)));
    }
    Json.write(out.resolve("pair_surface_residuals.json"), report);
    return Json.object(
        "confidence_rows_scanned",
        rows,
        "sampled_points_attributed",
        matched,
        "pair_groups",
        report.size());
  }

  private static void svg(
      Path file, List<Cell> cells, boolean thickness, double size, double x0, double y0)
      throws IOException {
    int xmax = 0, ymax = 0;
    for (Cell c : cells) {
      xmax = Math.max(xmax, c.x);
      ymax = Math.max(ymax, c.y);
    }
    double scale = Math.min(850. / Math.max(1, xmax + 1), 650. / Math.max(1, ymax + 1));
    double[] values =
        cells.stream()
            .mapToDouble(
                c -> thickness ? c.quantiles[3] - c.quantiles[1] : Math.abs(c.medianOffset))
            .filter(Double::isFinite)
            .sorted()
            .toArray();
    double range = values.length == 0 ? 1 : Math.max(1e-12, quantile(values, .95));
    StringBuilder s =
        new StringBuilder(
            "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"1000\" height=\"800\" viewBox=\"0 0"
                + " 1000 800\"><rect width=\"1000\" height=\"800\" fill=\"white\"/><text x=\"40\""
                + " y=\"28\" font-family=\"sans-serif\" font-size=\"18\">");
    s.append(
            thickness
                ? "Local surface spread: residual p90 minus p10"
                : "Local median height relative to a global plane")
        .append("</text>");
    for (Cell c : cells) {
      double v = thickness ? c.quantiles[3] - c.quantiles[1] : c.medianOffset,
          t = Math.max(-1, Math.min(1, v / range));
      int red = (int) (255 * (t >= 0 ? 1 : 1 + t)),
          blue = (int) (255 * (t <= 0 ? 1 : 1 - t)),
          green = (int) (255 * (1 - Math.abs(t)));
      s.append("<rect x=\"")
          .append(40 + c.x * scale)
          .append("\" y=\"")
          .append(60 + (ymax - c.y) * scale)
          .append("\" width=\"")
          .append(scale)
          .append("\" height=\"")
          .append(scale)
          .append("\" fill=\"rgb(")
          .append(red)
          .append(',')
          .append(green)
          .append(',')
          .append(blue)
          .append(")\"><title>")
          .append("cell ")
          .append(c.x)
          .append(',')
          .append(c.y)
          .append("; value=")
          .append(v)
          .append("; samples=")
          .append(c.points.size())
          .append("</title></rect>");
    }
    s.append("<text x=\"40\" y=\"750\" font-family=\"sans-serif\" font-size=\"13\">Cell size ")
        .append(size)
        .append("; colour scale ")
        .append(thickness ? "0 to " : "+/- ")
        .append(range)
        .append(" in cloud units. Dominant-plane proxy; not ground truth.</text></svg>");
    Files.writeString(file, s);
  }
}
