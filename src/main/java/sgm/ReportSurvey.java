package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;

/**
 * Survey figures for the processing report, read from the archived run (no reprocessing): camera
 * model, tie points and projections, reprojection error, flying altitude, ground resolution,
 * coverage, image residuals, lens and GPS-model parameters, camera location errors, ground control
 * points and the pipeline steps. Every block is optional: missing files leave it out.
 */
final class ReportSurvey {
  private ReportSurvey() {}

  static final int GRID_COLUMNS = 12;

  static Map<String, Object> collect(
      Path run, Map<String, Object> coordinates, Map<String, Object> maps, Map<String, Object> dsm)
      throws IOException {
    Map<String, Object> s = new LinkedHashMap<>();
    boolean metres = Boolean.TRUE.equals(coordinates.get("georeferenced"));
    s.put("units", metres ? "m" : "units");
    Path geometry = run.resolve("04_geometry"), fin = geometry.resolve("final");
    // Cameras: source lens (orientation model), processed lens, exterior positions.
    Map<String, Object> model = ProcessingReport.map(geometry.resolve("orientation_model.json"));
    List<Object> views = list(ProcessingReport.object(model.get("source")).get("views"));
    List<Object> groups = new ArrayList<>();
    Set<String> seen = new HashSet<>();
    for (Object v : views) {
      var interior = ProcessingReport.object(ProcessingReport.object(v).get("interior"));
      if (!interior.isEmpty() && seen.add(Json.string(interior)) && groups.size() < 4) groups.add(interior);
    }
    List<String[]> interior = csv(fin.resolve("interior.csv"));
    double processedFx = interior.size() > 1 ? number(interior.get(1)[4]) : Double.NaN;
    double processedWidth = interior.size() > 1 ? number(interior.get(1)[2]) : Double.NaN;
    double processedHeight = interior.size() > 1 ? number(interior.get(1)[3]) : Double.NaN;
    var first = groups.isEmpty() ? Map.<String, Object>of() : ProcessingReport.object(groups.get(0));
    double sourceFx = num(first.get("fx"));
    double factor = Double.isFinite(sourceFx) && processedFx > 0 ? sourceFx / processedFx : 1;
    s.put("source_pixels_per_processed_pixel", factor);
    // Camera hardware from EXIF.
    List<Object> exif = list(ProcessingReport.read(run.resolve("01_calibration/exif_intrinsics.json")));
    Map<String, Object> camera = new LinkedHashMap<>();
    if (!exif.isEmpty()) {
      var e = ProcessingReport.object(exif.get(0));
      camera.put("model", (String.valueOf(e.getOrDefault("make", "")) + " " + e.getOrDefault("model", "")).trim());
      camera.put("focal_mm", e.get("focal_mm"));
      camera.put("focal_35mm_mm", e.get("focal_35mm_equivalent_mm"));
      if (e.get("focal_mm") instanceof Number mm && sourceFx > 0)
        camera.put("pixel_size_um", 1000 * mm.doubleValue() / sourceFx);
    }
    if (!first.isEmpty()) camera.put("resolution", new Object[] {first.get("width"), first.get("height")});
    camera.put("groups", groups.size());
    s.put("camera", camera);
    s.put("calibrations", groups);

    // Camera centres.
    Map<Integer, double[]> centres = new LinkedHashMap<>();
    Map<Integer, String> names = new HashMap<>();
    List<String[]> exterior = csv(fin.resolve("exterior.csv"));
    int images = 0;
    for (int i = 1; i < exterior.size(); i++) {
      String[] r = exterior.get(i);
      if (r.length < 19) continue;
      images++;
      int id = (int) number(r[0]);
      names.put(id, r[1]);
      if (r[2].equals("true")) centres.put(id, new double[] {number(r[16]), number(r[17]), number(r[18])});
    }
    if (images > 0) {
      s.put("images", images);
      s.put("aligned", centres.size());
    }

    // Tie points, projections, reprojection error and the image residual grid.
    List<double[]> trackZ = new ArrayList<>();
    List<double[]> trackXY = new ArrayList<>();
    long tiePoints = 0;
    for (String[] r : csv(fin.resolve("tracks.csv")))
      if (r.length >= 5 && r[1].equals("true")) {
        tiePoints++;
        if (trackZ.size() < 400000) {
          trackZ.add(new double[] {number(r[4])});
          trackXY.add(new double[] {number(r[2]), number(r[3])});
        }
      }
    Path observations = fin.resolve("observations.csv");
    if (Files.isRegularFile(observations) && processedWidth > 0) {
      int columns = GRID_COLUMNS, rows = Math.max(1, (int) Math.round(columns * processedHeight / processedWidth));
      double[][] grid = new double[columns * rows][3];
      long projections = 0;
      double square = 0, max = 0;
      try (BufferedReader reader = Files.newBufferedReader(observations)) {
        reader.readLine();
        for (String line; (line = reader.readLine()) != null; ) {
          String[] r = line.split(",", -1);
          if (r.length < 12 || !r[5].equals("true") || r[10].isEmpty()) continue;
          double e = number(r[10]);
          if (!Double.isFinite(e)) continue;
          projections++;
          square += e * e;
          max = Math.max(max, e);
          double u = number(r[3]), v = number(r[4]);
          int cx = (int) Math.floor(u / processedWidth * columns), cy = (int) Math.floor(v / processedHeight * rows);
          if (cx < 0 || cy < 0 || cx >= columns || cy >= rows) continue;
          double[] cell = grid[cy * columns + cx];
          cell[0] += number(r[8]);
          cell[1] += number(r[9]);
          cell[2]++;
        }
      }
      if (projections > 0) {
        s.put("projections", projections);
        s.put("reprojection_rms_pixels", factor * Math.sqrt(square / projections));
        s.put("reprojection_max_pixels", factor * max);
        s.put("reprojection_rms_processed_pixels", Math.sqrt(square / projections));
        List<Object> cells = new ArrayList<>();
        for (double[] c : grid)
          cells.add(c[2] == 0 ? new double[] {0, 0, 0} : new double[] {factor * c[0] / c[2], factor * c[1] / c[2], c[2]});
        s.put(
            "image_residuals",
            Json.object(
                "columns", columns,
                "rows", rows,
                "width", first.getOrDefault("width", processedWidth * factor),
                "height", first.getOrDefault("height", processedHeight * factor),
                "cells", cells,
                "units", "source pixels, mean residual per cell (observed minus predicted)"));
      }
    }
    s.put("tie_points", tiePoints);

    // Flying altitude above the median tie point, ground resolution, coverage, density.
    if (!centres.isEmpty() && !trackZ.isEmpty()) {
      double[] z = trackZ.stream().mapToDouble(a -> a[0]).filter(Double::isFinite).sorted().toArray();
      double ground = z.length == 0 ? Double.NaN : z[z.length / 2], height = 0;
      for (double[] c : centres.values()) height += c[2] / centres.size();
      double altitude = height - ground;
      if (Double.isFinite(altitude) && altitude > 0) {
        s.put("flying_altitude", altitude);
        if (sourceFx > 0) s.put("ground_resolution", altitude / sourceFx);
        if (processedFx > 0) s.put("processed_ground_resolution", altitude / processedFx);
      }
    }
    // Coverage: convex hull of the tie points (the reconstructed area, holes included).
    double area = hullArea(trackXY);
    if (area > 0) {
      s.put("coverage_area", area);
      if (dsm.get("source_points") instanceof Number points && area > 0)
        s.put("point_density", points.doubleValue() / area);
    }

    // Camera location errors against GPS (after the GPS model) and the lens/GPS model.
    boolean gcp = Files.isRegularFile(geometry.resolve("gcp/gcp_report.json"));
    Path bundle = gcp ? geometry.resolve("gcp/gps_bundle.json") : geometry.resolve("georeference/gps_bundle.json");
    Map<String, Object> gps = ProcessingReport.map(bundle);
    if ("applied".equals(gps.get("status"))) {
      s.put("gps_model", gps.get("gps_model"));
      s.put("rolling_shutter", gps.get("rolling_shutter"));
      s.put("gps_intrinsics", gps.get("intrinsics"));
      s.put("gps_position_sigma", gps.get("position_sigma_metres"));
      s.put("gps_height_sigma", gps.get("height_sigma_metres"));
      List<Object> rows = new ArrayList<>();
      double[] sums = new double[5];
      for (Object item : list(gps.get("cameras"))) {
        var c = ProcessingReport.object(item);
        double[] r = nums(c.get("residual_enu"));
        if (r == null || r.length < 3) continue;
        int id = (int) num(c.get("image_id"));
        double[] xyz = centres.get(id);
        rows.add(
            Json.object(
                "image", names.getOrDefault(id, "" + id),
                "position", xyz,
                "error", r));
        sums[0] += r[0] * r[0];
        sums[1] += r[1] * r[1];
        sums[2] += r[2] * r[2];
        sums[3]++;
      }
      if (sums[3] > 0) {
        double n = sums[3];
        s.put(
            "camera_locations",
            Json.object(
                "cameras", rows,
                "rms",
                Json.object(
                    "x", Math.sqrt(sums[0] / n),
                    "y", Math.sqrt(sums[1] / n),
                    "z", Math.sqrt(sums[2] / n),
                    "xy", Math.sqrt((sums[0] + sums[1]) / n),
                    "total", Math.sqrt((sums[0] + sums[1] + sums[2]) / n)),
                "reference",
                gcp
                    ? "GPS fixes after the GPS model and the offset to the GCP datum"
                    : "GPS fixes after the GPS model (time offset, lever arm)"));
      }
    } else if (!gps.isEmpty()) s.put("gps_status", gps.get("status"));
    if (gcp) s.put("gcp", ProcessingReport.read(geometry.resolve("gcp/gcp_report.json")));
    Path dtm = run.resolve("07_results/dtm.png");
    if (Files.isRegularFile(dtm)) s.put("dtm_preview", dtm.toString());
    List<Object> positions = new ArrayList<>();
    for (double[] c : centres.values()) positions.add(c);
    s.put("camera_positions", positions);

    // Pipeline steps (split runs) with their status.
    Map<String, Object> state = ProcessingReport.map(run.resolve("state/state.json"));
    var steps = ProcessingReport.object(state.get("steps"));
    if (!steps.isEmpty()) {
      List<Object> list = new ArrayList<>();
      for (Pipeline.Step step : Pipeline.Step.values()) {
        var e = ProcessingReport.object(steps.get(step.name));
        list.add(Json.object("step", step.name, "status", e.getOrDefault("status", "skipped"), "updated_utc", e.get("updated_utc")));
      }
      s.put("steps", list);
    }
    return s;
  }

  /** Area of the convex hull (monotone chain). */
  static double hullArea(List<double[]> points) {
    List<double[]> p = new ArrayList<>();
    for (double[] q : points) if (Double.isFinite(q[0]) && Double.isFinite(q[1])) p.add(q);
    if (p.size() < 3) return 0;
    p.sort((a, b) -> a[0] != b[0] ? Double.compare(a[0], b[0]) : Double.compare(a[1], b[1]));
    double[][] hull = new double[2 * p.size()][];
    int k = 0;
    for (int pass = 0; pass < 2; pass++) {
      int start = k;
      for (int i = 0; i < p.size(); i++) {
        double[] q = p.get(pass == 0 ? i : p.size() - 1 - i);
        while (k >= start + 2 && cross(hull[k - 2], hull[k - 1], q) <= 0) k--;
        hull[k++] = q;
      }
      k--;
    }
    double area = 0;
    for (int i = 0; i < k; i++) {
      double[] a = hull[i], b = hull[(i + 1) % k];
      area += a[0] * b[1] - b[0] * a[1];
    }
    return Math.abs(area) / 2;
  }

  private static double cross(double[] o, double[] a, double[] b) {
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  }

  private static List<Object> list(Object o) {
    return o instanceof List<?> l ? new ArrayList<>(l) : new ArrayList<>();
  }

  private static double num(Object o) {
    return o instanceof Number n ? n.doubleValue() : Double.NaN;
  }

  private static double[] nums(Object o) {
    if (!(o instanceof List<?> l)) return null;
    double[] v = new double[l.size()];
    for (int i = 0; i < v.length; i++) v[i] = num(l.get(i));
    return v;
  }

  private static double number(String s) {
    try {
      return Double.parseDouble(s.trim());
    } catch (RuntimeException e) {
      return Double.NaN;
    }
  }

  /** Small CSV reader (quoted fields, no embedded newlines); empty if the file is missing. */
  static List<String[]> csv(Path file) throws IOException {
    List<String[]> rows = new ArrayList<>();
    if (!Files.isRegularFile(file)) return rows;
    try (BufferedReader r = Files.newBufferedReader(file)) {
      for (String line; (line = r.readLine()) != null; ) {
        List<String> fields = new ArrayList<>();
        StringBuilder f = new StringBuilder();
        boolean quoted = false;
        for (int i = 0; i < line.length(); i++) {
          char c = line.charAt(i);
          if (c == '"') {
            if (quoted && i + 1 < line.length() && line.charAt(i + 1) == '"') {
              f.append(c);
              i++;
            } else quoted = !quoted;
          } else if (c == ',' && !quoted) {
            fields.add(f.toString());
            f.setLength(0);
          } else f.append(c);
        }
        fields.add(f.toString());
        rows.add(fields.toArray(String[]::new));
      }
    }
    return rows;
  }
}
