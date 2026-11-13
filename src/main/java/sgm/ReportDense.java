package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;

/**
 * Dense-cloud figures for the processing report, read from the archived run (no reprocessing):
 * the candidate-to-point counts, points by number of supporting cameras, depth-map completeness
 * per camera, stereo matching totals, skipped pairs, and where the work ran on the CPU or on disk
 * instead of the GPU or memory. Every block is optional: a missing file leaves it out, so runs made
 * before a statistic was recorded simply do not show it.
 */
final class ReportDense {
  private ReportDense() {}

  static Map<String, Object> collect(Path run, Map<String, Object> telemetry) throws IOException {
    Map<String, Object> d = new LinkedHashMap<>();
    Path stereo = run.resolve("05_stereo"), fusion = run.resolve("06_fusion");
    Map<String, Object> settings = ProcessingReport.map(fusion.resolve("fusion.json"));
    if (settings.get("dense_method") != null) d.put("method", settings.get("dense_method"));
    if (settings.get("fusion_mode") != null) d.put("fusion_mode", settings.get("fusion_mode"));
    if (settings.get("minimum_distinct_views") != null)
      d.put("minimum_views", settings.get("minimum_distinct_views"));

    // The run's own summary lines (label=value), numbers where they parse.
    Map<String, Object> summary = new LinkedHashMap<>();
    Path quality = fusion.resolve("fusion-quality.txt");
    if (!Files.isRegularFile(quality)) quality = run.resolve("fusion-quality.txt");
    if (Files.isRegularFile(quality))
      for (String line : Files.readAllLines(quality)) {
        int at = line.indexOf('=');
        if (at <= 0) continue;
        String value = line.substring(at + 1).trim();
        Object parsed = value;
        try {
          parsed = Long.parseLong(value);
        } catch (NumberFormatException text) {
          // Text values (method names, thresholds) stay as written.
        }
        summary.put(line.substring(0, at).trim(), parsed);
      }
    if (!summary.isEmpty()) d.put("summary", summary);

    Map<String, Object> image = ProcessingReport.map(fusion.resolve("image_fusion.json"));
    if (!image.isEmpty()) {
      d.put("image_fusion", image);
      views(d, image);
      cameras(d, image, run);
      memory(d, image);
    }
    pairs(d, stereo);
    multiView(d, stereo);

    Map<String, Object> skipped = ProcessingReport.map(stereo.resolve("skipped_pairs.json"));
    if (!skipped.isEmpty())
      d.put(
          "skipped_pairs",
          Json.object(
              "attempted", skipped.get("attempted"),
              "skipped", skipped.get("skipped") instanceof List<?> l ? l.size() : 0,
              "policy", skipped.get("policy"),
              "file", "05_stereo/skipped_pairs.json"));

    fallbacks(d, run, telemetry);
    return d;
  }

  /** Points by number of supporting cameras. */
  private static void views(Map<String, Object> d, Map<String, Object> image) {
    if (!(image.get("views_per_point_histogram") instanceof List<?> histogram)) return;
    long total = 0;
    for (Object n : histogram) total += count(n);
    if (total == 0) return;
    d.put(
        "views_per_point",
        Json.object(
            "histogram", histogram,
            "last_bucket", image.getOrDefault("views_per_point_last_bucket", "or more"),
            "points", total));
  }

  /** Depth-map completeness per camera: the share of the image with a filtered merged depth. */
  private static void cameras(Map<String, Object> d, Map<String, Object> image, Path run)
      throws IOException {
    if (!(image.get("cameras") instanceof List<?> list) || list.isEmpty()) return;
    Map<Integer, String> names = new HashMap<>();
    List<String[]> exterior = ReportSurvey.csv(run.resolve("04_geometry/final/exterior.csv"));
    for (int i = 1; i < exterior.size(); i++)
      if (exterior.get(i).length > 1)
        try {
          names.put(Integer.parseInt(exterior.get(i)[0].trim()), exterior.get(i)[1]);
        } catch (NumberFormatException header) {
          // Not a camera row.
        }
    List<Object> cameras = new ArrayList<>();
    List<Double> shares = new ArrayList<>();
    long pixels = 0, valid = 0;
    for (Object item : list) {
      var c = ProcessingReport.object(item);
      long p = count(c.get("pixels")), v = count(c.get("valid_depth_pixels"));
      if (p <= 0) continue;
      int id = (int) count(c.get("camera"));
      // A camera without any depth map took no part in dense matching (not registered, screened
      // out or without a stereo partner): it is listed but kept out of the completeness figures.
      boolean used = v > 0;
      cameras.add(
          Json.object(
              "camera", id,
              "image", names.getOrDefault(id, Integer.toString(id)),
              "pixels", p,
              "valid_depth_pixels", v,
              "emitted_points", count(c.get("emitted_points")),
              "completeness", v / (double) p,
              "used", used));
      if (!used) continue;
      pixels += p;
      valid += v;
      shares.add(v / (double) p);
    }
    if (cameras.isEmpty()) return;
    d.put("cameras", cameras);
    if (shares.isEmpty()) return;
    Collections.sort(shares);
    d.put(
        "completeness",
        Json.object(
            "definition",
            "Share of a camera's pixels with a merged depth after the per-camera filters",
            "cameras_with_depth", shares.size(),
            "cameras_without_depth", cameras.size() - shares.size(),
            "overall", valid / (double) pixels,
            "median", shares.get(shares.size() / 2),
            "minimum", shares.get(0),
            "maximum", shares.get(shares.size() - 1)));
  }

  /** Fusion that ran from disk instead of memory. The result is the same; it is slower. */
  private static void memory(Map<String, Object> d, Map<String, Object> image) {
    var memory = ProcessingReport.object(image.get("memory"));
    if (memory.isEmpty()) return;
    long spilled = count(memory.get("spill_bytes_written")), strips = count(memory.get("strip_rows"));
    if (spilled <= 0 && strips <= 0) return;
    d.put(
        "fusion_on_disk",
        Json.object(
            "spill_bytes_written", spilled,
            "map_evictions", count(memory.get("map_evictions")),
            "strip_rows", strips,
            "mapped_bytes", count(memory.get("mapped_bytes")),
            "plan", memory.get("plan")));
  }

  /** Pair SGM: matches selected and kept, summed over the pairs. */
  private static void pairs(Map<String, Object> d, Path stereo) throws IOException {
    Path file = stereo.resolve("stereo-quality.csv");
    List<String[]> rows = ReportSurvey.csv(file);
    if (rows.size() < 2) return;
    List<String> header = Arrays.asList(rows.get(0));
    int selected = header.indexOf("selected"), kept = header.indexOf("final_valid");
    if (selected < 0 || kept < 0) return;
    long sumSelected = 0, sumKept = 0;
    List<Object[]> shares = new ArrayList<>();
    for (int i = 1; i < rows.size(); i++) {
      String[] r = rows.get(i);
      if (r.length <= Math.max(selected, kept)) continue;
      long s, k;
      try {
        s = Long.parseLong(r[selected].trim());
        k = Long.parseLong(r[kept].trim());
      } catch (NumberFormatException skip) {
        continue;
      }
      sumSelected += s;
      sumKept += k;
      if (s > 0) shares.add(new Object[] {r[0], k / (double) s});
    }
    if (shares.isEmpty()) return;
    shares.sort(Comparator.comparingDouble(a -> (double) a[1]));
    List<Object> lowest = new ArrayList<>();
    for (Object[] s : shares.subList(0, Math.min(5, shares.size())))
      lowest.add(Json.object("pair", s[0], "kept_share", s[1]));
    d.put(
        "pair_matching",
        Json.object(
            "pairs", shares.size(),
            "selected_matches", sumSelected,
            "kept_matches", sumKept,
            "kept_share", sumSelected > 0 ? sumKept / (double) sumSelected : null,
            "median_pair_kept_share", shares.get(shares.size() / 2)[1],
            "lowest_pairs", lowest));
  }

  /** Multi-view SGM: depths before and after the consistency check. */
  private static void multiView(Map<String, Object> d, Path stereo) throws IOException {
    Map<String, Object> mv = ProcessingReport.map(stereo.resolve("mvsgm/mvsgm.json"));
    var consistency = ProcessingReport.object(mv.get("consistency"));
    if (consistency.isEmpty()) return;
    d.put(
        "multi_view_matching",
        Json.object(
            "depths", consistency.get("depths"),
            "consistent", consistency.get("consistent"),
            "inconsistent_rejected", consistency.get("inconsistent_rejected"),
            "consistent_views_required", consistency.get("consistent_views_required")));
  }

  /**
   * Work that ran on the CPU although a GPU device was selected: multi-view cameras and pair
   * search levels by backend, and the GPU fusion fallbacks recorded in the telemetry.
   */
  private static void fallbacks(Map<String, Object> d, Path run, Map<String, Object> telemetry)
      throws IOException {
    var device = ProcessingReport.object(telemetry.get("selected_opencl_device"));
    boolean gpu = device.get("name") != null;
    Map<String, Object> out = new LinkedHashMap<>();
    out.put("gpu_device_selected", gpu);
    Path stereo = run.resolve("05_stereo");
    Map<String, Object> mv = ProcessingReport.map(stereo.resolve("mvsgm/mvsgm.json"));
    if (mv.get("cameras") instanceof List<?> cameras) {
      int cpu = 0, all = 0;
      for (Object item : cameras) {
        Object backend = ProcessingReport.object(item).get("backend");
        if (backend == null) continue;
        all++;
        if (!"gpu".equals(backend)) cpu++;
      }
      out.put("multi_view_cameras", all);
      out.put("multi_view_cameras_on_cpu", cpu);
    }
    if (Files.isDirectory(stereo)) {
      long levels = 0, cpuLevels = 0;
      int pairsOnCpu = 0, pairs = 0;
      try (var dirs = Files.list(stereo)) {
        for (Path p : dirs.filter(Files::isDirectory).sorted().toList()) {
          boolean any = false, cpu = false;
          for (String side : new String[] {"tsgm_left.json", "tsgm_right.json"})
            for (Object item :
                ProcessingReport.map(p.resolve(side)).get("levels") instanceof List<?> l ? l : List.of()) {
              Object backend = ProcessingReport.object(item).get("backend");
              if (backend == null || "cache".equals(backend)) continue;
              any = true;
              levels++;
              if (!"gpu".equals(backend)) {
                cpuLevels++;
                cpu = true;
              }
            }
          if (any) pairs++;
          if (cpu) pairsOnCpu++;
        }
      }
      if (levels > 0) {
        out.put("pair_search_pairs", pairs);
        out.put("pair_search_pairs_with_cpu_levels", pairsOnCpu);
        out.put("pair_search_levels", levels);
        out.put("pair_search_levels_on_cpu", cpuLevels);
      }
    }
    long fusion = 0;
    for (Object item : telemetry.get("tasks") instanceof List<?> l ? l : List.of()) {
      var task = ProcessingReport.object(item);
      if (String.valueOf(task.get("task")).startsWith("GPU fusion fallback")) fusion += Math.max(1, count(task.get("events")));
    }
    out.put("gpu_fusion_fallback_events", fusion);
    d.put("backends", out);
  }

  static long count(Object o) {
    return o instanceof Number n ? n.longValue() : 0;
  }
}
