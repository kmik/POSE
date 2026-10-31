package sgm;

import java.io.*;
import java.nio.file.*;
import java.time.*;
import java.util.*;

/** Self-contained printable HTML with complete archived parameters and bounded DSM products. */
final class ProcessingReport {
  @SuppressWarnings("unchecked")
  static Map<String, Object> object(Object o) {
    return o instanceof Map<?, ?> ? (Map<String, Object>) o : new LinkedHashMap<>();
  }

  static Object read(Path p) throws IOException {
    return Files.isRegularFile(p) && Files.size(p) <= 32 * 1048576L ? JsonRead.read(p) : null;
  }

  static Map<String, Object> map(Path p) throws IOException {
    return object(read(p));
  }

  static void export(Options o) throws IOException {
    Path run = o.exportReport.toAbsolutePath().normalize();
    Map<String, Object> state = map(run.resolve("run.json"));
    if (state.isEmpty() || "running".equals(state.get("status")))
      throw new IllegalArgumentException(
          "Report export requires an archived, non-running project run");
    Path out =
        o.reportOutput == null
            ? run.resolve("07_results/report")
            : o.reportOutput.toAbsolutePath().normalize();
    if (out.equals(run)
        || out.startsWith(run.resolve("00_inputs"))
        || out.startsWith(run.resolve("logs")))
      throw new IllegalArgumentException(
          "Choose a separate report directory, not run metadata/input storage");
    generate(o, run, out);
  }

  static void generate(Options o, Path run, Path out) throws IOException {
    Files.createDirectories(out);
    System.out.println(
        "Processing report: collecting archived settings, timings and surface overview");
    Map<String, Object> state = map(run.resolve("run.json")),
        coordinate = map(run.resolve("07_results/coordinates.json"));
    if (coordinate.isEmpty()) coordinate = map(run.resolve("coordinates.json"));
    Map<String, Object> dsm;
    try {
      dsm = DsmRaster.create(run, out, o, coordinate);
    } catch (IOException | IllegalArgumentException e) {
      System.err.println("Report DSM unavailable: " + e.getMessage());
      dsm = Json.object("status", "unavailable", "reason", e.getMessage());
      Json.write(out.resolve("dsm.json"), dsm);
    }
    RunTelemetry current = RunTelemetry.current();
    if (current != null) current.persist();
    Map<String, Object> telemetry = map(run.resolve("logs/processing.json"));
    Map<String, Object> parameters = object(telemetry.get("effective_parameters"));
    if (parameters.isEmpty()) {
      parameters = new TreeMap<>();
      Path file = run.resolve("logs/run-settings.properties");
      if (Files.isRegularFile(file)) {
        Properties props = new Properties();
        try (Reader reader = Files.newBufferedReader(file)) {
          props.load(reader);
        }
        for (String key : props.stringPropertyNames()) parameters.put(key, props.getProperty(key));
      }
    }
    Map<String, Object> initial = object(telemetry.get("initial_parameters"));
    Map<String, Object> request = map(run.resolve("logs/request.json"));
    Map<String, Double> recovered = recoveredStageTimes(run);
    List<Object> stages = new ArrayList<>();
    for (var e : object(state.get("stages")).entrySet()) {
      Map<String, Object> stage = new LinkedHashMap<>(object(e.getValue()));
      stage.put("stage", e.getKey());
      if (!stage.containsKey("elapsed_seconds") && recovered.containsKey(e.getKey())) {
        stage.put("elapsed_seconds", recovered.get(e.getKey()));
        stage.put("timing_provenance", "archived stage event timestamps");
      }
      stages.add(stage);
    }
    Map<String, Object> profiles = new LinkedHashMap<>();
    for (String relative :
        List.of(
            "06_fusion/backend.json",
            "06_fusion/output_profile.json",
            "06_fusion/surface_consolidation.json",
            "06_fusion/fusion.json",
            "00_inputs/resolution_plan.json",
            "01_calibration/refinement/result.json",
            "04_geometry/registration.json",
            "04_geometry/bundle/solver.json")) {
      Object value = read(run.resolve(relative));
      if (value != null) profiles.put(relative, value);
    }
    Map<String, Object> camera = map(run.resolve("04_geometry/orientation_model.json"));
    String overview = cameras(camera);
    if (!overview.isEmpty()) Files.writeString(out.resolve("camera-overview.svg"), overview);
    List<Object> pairParameters = new ArrayList<>();
    Path stereo = run.resolve("05_stereo");
    if (Files.isDirectory(stereo))
      try (var dirs = Files.list(stereo)) {
        for (Path p : dirs.filter(Files::isDirectory).sorted().toList()) {
          Object range = read(p.resolve("disparity_range.json"));
          if (range != null)
            pairParameters.add(
                Json.object("pair", p.getFileName().toString(), "resolved_range", range));
        }
      }
    Map<String, Object> data =
        Json.object(
            "schema",
            "collinea-processing-report-1",
            "generated_utc",
            Instant.now().toString(),
            "run_id",
            state.get("run_id"),
            "run_status",
            state.get("status"),
            "processing_started_utc",
            state.get("started_utc"),
            "processing_finished_utc",
            state.get("updated_utc"),
            "arguments",
            request.getOrDefault("arguments", state.get("arguments")),
            "parameter_provenance",
            telemetry.isEmpty()
                ? "Archived properties only; historical missing fields and measured peaks are"
                    + " unavailable"
                : "Complete public Options fields, including defaults, explicit overrides and"
                    + " effective final values; resolved per-image/per-pair settings archived"
                    + " below",
            "parameters",
            parameters,
            "initial_parameters",
            initial,
            "report_generation_parameters",
            RunTelemetry.parameters(o),
            "request_metadata",
            request,
            "stage_timings",
            stages,
            "telemetry",
            telemetry,
            "profiles",
            profiles,
            "coordinates",
            coordinate,
            "camera_orientations",
            camera,
            "pair_parameters",
            pairParameters,
            "dsm",
            dsm);
    Json.write(out.resolve("processing-report.json"), data);
    Files.writeString(out.resolve("processing-report.html"), html(data, overview, out));
    System.out.println("Processing report complete: " + out.resolve("processing-report.html"));
  }

  private static Double archivedDuration(Map<String, Object> report) {
    try {
      return Duration.between(
                  Instant.parse(String.valueOf(report.get("processing_started_utc"))),
                  Instant.parse(String.valueOf(report.get("processing_finished_utc"))))
              .toNanos()
          / 1e9;
    } catch (RuntimeException e) {
      return null;
    }
  }

  private static Map<String, Double> recoveredStageTimes(Path run) throws IOException {
    Map<String, Instant> started = new HashMap<>();
    Map<String, Double> result = new HashMap<>();
    Path events = run.resolve("logs/events.jsonl");
    if (!Files.isRegularFile(events)) return result;
    try (BufferedReader in = Files.newBufferedReader(events)) {
      String line;
      while ((line = in.readLine()) != null) {
        try {
          var event = object(JsonRead.parse(line));
          if (!"stage".equals(event.get("type"))) continue;
          var detail = object(event.get("detail"));
          String stage = String.valueOf(detail.get("stage")),
              status = String.valueOf(detail.get("status"));
          Instant time = Instant.parse(String.valueOf(event.get("utc")));
          if (status.equals("running")) started.putIfAbsent(stage, time);
          else if (started.containsKey(stage)) {
            result.merge(
                stage, Duration.between(started.remove(stage), time).toNanos() / 1e9, Double::sum);
          }
        } catch (IllegalArgumentException ignored) {
        }
      }
    }
    return result;
  }

  private static String html(Map<String, Object> d, String overview, Path out) throws IOException {
    StringBuilder b =
        new StringBuilder(
            "<!doctype html><html lang=\"en\"><head><meta charset=\"utf-8\"><meta name=\"viewport\""
                + " content=\"width=device-width,initial-scale=1\"><title>Collinea processing"
                + " report</title><style>");
    b.append(
        "body{margin:0;background:#eef2f6;color:#182735;font:15px/1.55"
            + " system-ui,Arial,sans-serif}main{max-width:1100px;margin:32px"
            + " auto;background:white;padding:42px;border-radius:12px;box-shadow:0 2px 18px"
            + " #102a4312}h1{font-size:34px;margin:0}h2{font-size:23px;border-bottom:2px solid"
            + " #e5edf3;padding-bottom:8px;margin-top:34px}h3{font-size:17px}.brand{color:#126b83;font-weight:700;letter-spacing:2px}.sub,.note{color:#526777}.cards{display:grid;grid-template-columns:repeat(auto-fit,minmax(190px,1fr));gap:12px;margin:24px"
            + " 0}.card{border:1px solid #dbe4eb;border-radius:8px;padding:16px}.card"
            + " strong{display:block;font-size:22px;color:#145c76}table{border-collapse:collapse;width:100%;font-size:13px}th,td{padding:8px"
            + " 10px;text-align:left;border-bottom:1px solid"
            + " #e3eaf0;vertical-align:top;overflow-wrap:anywhere}th{background:#edf3f7}td:first-child{font-weight:500}pre{white-space:pre-wrap;overflow-wrap:anywhere;background:#f4f7f9;padding:14px;border-radius:6px;font-size:12px}figure{margin:18px"
            + " 0;text-align:center}img,svg{max-width:100%;height:auto}figure"
            + " img{max-height:700px;object-fit:contain}figure"
            + " svg{max-height:520px}figcaption{color:#526777;font-size:13px;margin-top:8px}.legend{height:14px;background:linear-gradient(90deg,#0000ff,#00ffff,#00ff00,#ffff00,#ff0000);max-width:500px;margin:10px"
            + " auto}.legendlabels{display:flex;justify-content:space-between;max-width:500px;margin:auto}.pill{display:inline-block;padding:3px"
            + " 10px;background:#e8f4ef;color:#165a3c;border-radius:20px}summary{cursor:pointer;font-weight:600;padding:10px"
            + " 0}@media(max-width:700px){main{padding:18px;margin:0;border-radius:0}h1{font-size:27px}}@media"
            + " print{body{background:white}main{box-shadow:none;margin:0;padding:0;max-width:none}details{display:block}details>summary{display:block}details>div{display:block}table{font-size:10px}h2{break-after:avoid}figure{break-inside:avoid}pre{font-size:9px}}</style></head><body><main>");
    b.append(
            "<div class=\"brand\">COLLINEA</div><h1>Photogrammetry processing report</h1><p"
                + " class=\"sub\">Run ")
        .append(esc(d.get("run_id")))
        .append(" &middot; <span class=\"pill\">")
        .append(esc(d.get("run_status")))
        .append("</span><br>Generated ")
        .append(esc(d.get("generated_utc")))
        .append(" (UTC)</p>");
    Map<String, Object> t = object(d.get("telemetry"));
    Map<String, Object> surface =
        object(object(d.get("profiles")).get("06_fusion/surface_consolidation.json"));
    b.append("<div class=\"cards\">");
    card(
        b,
        "Processing wall time",
        seconds(
            t.containsKey("elapsed_wall_seconds")
                ? t.get("elapsed_wall_seconds")
                : archivedDuration(d)));
    card(b, "Peak sampled process RAM", memory(t.get("peak_sampled_process_resident_bytes")));
    card(b, "Peak tracked OpenCL buffers", memory(t.get("peak_tracked_opencl_buffer_bytes")));
    card(b, "Final surface samples", surface.getOrDefault("output_samples", "Unavailable"));
    b.append("</div>");
    b.append("<h2>Digital surface model</h2>");
    Map<String, Object> dsm = object(d.get("dsm"));
    if ("complete".equals(dsm.get("status"))) {
      byte[] png = Files.readAllBytes(out.resolve("dsm.png"));
      b.append(
              "<figure><img alt=\"Digital surface model colored by elevation; gray cells have no"
                  + " data\" src=\"data:image/png;base64,")
          .append(Base64.getEncoder().encodeToString(png))
          .append("\"><div class=\"legend\"></div><div class=\"legendlabels\"><span>")
          .append(esc(dsm.get("minimum_elevation")))
          .append("</span><span>")
          .append(esc(dsm.get("maximum_elevation")))
          .append("</span></div><figcaption>Surface elevations in ")
          .append(esc(dsm.get("units")))
          .append(
              ". "
                  + (dsm.get("epsg") == null ? "Local +Y at top" : "North at top")
                  + ". Gray: nodata; no gap filling. Includes roofs and"
                  + " vegetation.</figcaption></figure>");
    }
    table(b, dsm);
    b.append("<h3>Coordinate reference</h3>");
    table(b, object(d.get("coordinates")));
    if (!overview.isEmpty())
      b.append("<h2>Camera centers</h2><figure>")
          .append(overview)
          .append(
              "<figcaption>Archived camera centers projected onto XY; image footprints are not"
                  + " inferred.</figcaption></figure>");
    b.append("<h2>Processing setup</h2>");
    table(b, object(d.get("request_metadata")));
    b.append("<h3>Supplied arguments</h3><pre>")
        .append(esc(Json.string(d.get("arguments"))))
        .append("</pre>");
    b.append(
        "<h2>Stage times</h2><p class=\"note\">Elapsed wall times. Nested and overlapping work must"
            + " not be summed. Older runs may not contain individual timing"
            + " measurements.</p><table><thead><tr><th>Stage</th><th>Status</th><th>Wall"
            + " time</th><th>Details</th></tr></thead><tbody>");
    for (Object value : list(d.get("stage_timings"))) {
      Map<String, Object> stage = object(value);
      b.append("<tr><td>")
          .append(esc(stage.get("stage")))
          .append("</td><td>")
          .append(esc(stage.get("status")))
          .append("</td><td>")
          .append(esc(seconds(stage.get("elapsed_seconds"))))
          .append("</td><td>")
          .append(esc(stage.get("detail")))
          .append("</td></tr>");
    }
    b.append("</tbody></table>");
    b.append(
        "<details><summary>Detailed measured"
            + " tasks</summary><div><table><thead><tr><th>Task</th><th>Wall time</th><th>Status /"
            + " thread</th></tr></thead><tbody>");
    for (Object v : list(t.get("tasks"))) {
      var task = object(v);
      b.append("<tr><td>")
          .append(esc(task.get("task")))
          .append("</td><td>")
          .append(esc(seconds(task.get("wall_seconds"))))
          .append("</td><td>")
          .append(esc(task.get("status")))
          .append(" / ")
          .append(esc(task.get("thread")))
          .append("</td></tr>");
    }
    b.append("</tbody></table></div></details>");
    b.append(
        "<h3>Fusion and consolidation substages</h3><table><thead><tr><th>Profile /"
            + " substage</th><th>Measured time</th></tr></thead><tbody>");
    for (var e : object(d.get("profiles")).entrySet()) timings(b, e.getKey(), object(e.getValue()));
    b.append(
        "</tbody></table><p class=\"note\">Profile counters can be cumulative across fusion and"
            + " surface revalidation, nested or overlapping. Blocking GPU reads include time"
            + " waiting for queued kernels; do not add these to kernel times.</p>");
    b.append("<h2>Resources</h2>");
    Map<String, Object> resources = new LinkedHashMap<>(t);
    resources.remove("tasks");
    resources.remove("initial_parameters");
    resources.remove("effective_parameters");
    table(b, resources);
    b.append(
        "<p class=\"note\">Process RAM includes heap and native allocations. Private bytes and"
            + " resident memory are different metrics. OpenCL figures exclude driver/display"
            + " allocations; they are not total GPU usage. Peaks are sampled unless explicitly"
            + " tracked.</p>");
    b.append("<h2>All processing parameter values</h2><p class=\"note\">")
        .append(esc(d.get("parameter_provenance")))
        .append(
            ". Null means unset/not applicable; automatic/unset marks a non-finite initialization"
                + " value. Per-image intrinsics/poses and per-pair resolved ranges are included in"
                + " the archived detail below.</p>");
    table(b, object(d.get("parameters")));
    b.append("<h2>Resolved geometry, pair settings and diagnostics</h2>");
    details(b, "Source and processed camera orientations", d.get("camera_orientations"));
    details(b, "Resolved per-pair disparity settings", d.get("pair_parameters"));
    details(b, "Stage profiles, calibration, resolution and quality", d.get("profiles"));
    details(b, "Initial effective parameters", d.get("initial_parameters"));
    details(
        b,
        "Report-generation parameters (separate from reconstruction)",
        d.get("report_generation_parameters"));
    b.append(
        "<p class=\"note\">This report records computation and data provenance. It does not"
            + " establish surveyed accuracy. Stereo uncertainty excludes camera/GPS errors. DSM"
            + " outliers and an unspecified vertical datum remain relevant to"
            + " interpretation.</p></main></body></html>");
    return b.toString();
  }

  private static void timings(StringBuilder b, String prefix, Map<String, Object> map) {
    for (var e : map.entrySet()) {
      if (e.getValue() instanceof Map<?, ?>) {
        timings(b, prefix + " / " + e.getKey(), object(e.getValue()));
        continue;
      }
      if (e.getValue() instanceof Number n
          && (e.getKey().endsWith("_seconds")
              || e.getKey().endsWith("_nanoseconds")
              || e.getKey().equals("seconds"))) {
        double value = n.doubleValue() / (e.getKey().endsWith("_nanoseconds") ? 1e9 : 1);
        b.append("<tr><td>")
            .append(esc(prefix + " / " + e.getKey()))
            .append("</td><td>")
            .append(esc(seconds(value)))
            .append("</td></tr>");
      }
    }
  }

  private static void card(StringBuilder b, String label, Object value) {
    b.append("<div class=\"card\"><strong>")
        .append(esc(value))
        .append("</strong>")
        .append(esc(label))
        .append("</div>");
  }

  private static String seconds(Object value) {
    return value instanceof Number n
        ? String.format(Locale.ROOT, "%.2f s", n.doubleValue())
        : "Unavailable";
  }

  private static String memory(Object value) {
    return value instanceof Number n
        ? String.format(Locale.ROOT, "%.1f MiB", n.doubleValue() / 1048576)
        : "Unavailable";
  }

  private static List<?> list(Object o) {
    return o instanceof List<?> l ? l : List.of();
  }

  private static void table(StringBuilder b, Map<String, Object> values) {
    b.append("<table><thead><tr><th>Parameter / metric</th><th>Value</th></tr></thead><tbody>");
    for (var e : values.entrySet())
      b.append("<tr><td>")
          .append(esc(e.getKey()))
          .append("</td><td>")
          .append(
              esc(
                  e.getValue() instanceof Map || e.getValue() instanceof List
                      ? Json.string(e.getValue())
                      : e.getValue()))
          .append("</td></tr>");
    b.append("</tbody></table>");
  }

  private static void details(StringBuilder b, String label, Object value) {
    b.append("<details><summary>")
        .append(esc(label))
        .append("</summary><div><pre>")
        .append(esc(Json.string(value)))
        .append("</pre></div></details>");
  }

  static String esc(Object s) {
    return s == null
        ? "Unavailable / unset"
        : String.valueOf(s)
            .replace("&", "&amp;")
            .replace("<", "&lt;")
            .replace(">", "&gt;")
            .replace("\"", "&quot;")
            .replace("'", "&#39;");
  }

  private static String cameras(Map<String, Object> archive) {
    Map<String, Object> model = object(archive.get("processed"));
    if (model.isEmpty()) model = object(archive.get("source"));
    List<double[]> centers = new ArrayList<>();
    List<String> names = new ArrayList<>();
    for (Object value : list(model.get("views"))) {
      var v = object(value);
      List<?> rotation = list(v.get("R")), translation = list(v.get("t"));
      if (rotation.size() != 9 || translation.size() != 3) continue;
      double[] c = new double[3];
      for (int k = 0; k < 3; k++)
        for (int j = 0; j < 3; j++)
          c[k] -=
              ((Number) rotation.get(j * 3 + k)).doubleValue()
                  * ((Number) translation.get(j)).doubleValue();
      if (Double.isFinite(c[0] + c[1])) {
        centers.add(c);
        names.add(String.valueOf(v.get("image")));
      }
    }
    if (centers.isEmpty()) return "";
    double minx = Double.POSITIVE_INFINITY, miny = minx, maxx = -minx, maxy = -minx;
    for (double[] c : centers) {
      minx = Math.min(minx, c[0]);
      maxx = Math.max(maxx, c[0]);
      miny = Math.min(miny, c[1]);
      maxy = Math.max(maxy, c[1]);
    }
    double scale = Math.min(760 / Math.max(1e-12, maxx - minx), 440 / Math.max(1e-12, maxy - miny));
    StringBuilder b =
        new StringBuilder(
            "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 850 550\" role=\"img\""
                + " aria-label=\"Camera centers in archived XY coordinates\"><rect width=\"850\""
                + " height=\"550\" fill=\"#f4f7fa\"/><text x=\"20\" y=\"25\" fill=\"#526777\">Y /"
                + " north ↑</text>");
    for (int i = 0; i < centers.size(); i++) {
      double[] c = centers.get(i);
      double x = 45 + (c[0] - minx) * scale, y = 500 - (c[1] - miny) * scale;
      b.append(
              String.format(
                  Locale.ROOT,
                  "<circle cx=\"%.2f\" cy=\"%.2f\" r=\"4\" fill=\"#147e99\"><title>",
                  x,
                  y))
          .append(esc(names.get(i)))
          .append("</title></circle>");
      if (centers.size() <= 100)
        b.append(
            String.format(
                Locale.ROOT,
                "<text x=\"%.2f\" y=\"%.2f\" font-size=\"10\">%d</text>",
                x + 6,
                y - 4,
                i + 1));
    }
    return b.append("<text x=\"750\" y=\"535\" fill=\"#526777\">X →</text></svg>").toString();
  }
}
