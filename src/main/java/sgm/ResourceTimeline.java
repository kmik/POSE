package sgm;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.nio.file.*;
import java.util.*;
import java.util.List;
import javax.imageio.ImageIO;

/** Bounded report chart; unavailable utilization is never replaced by configured budgets. */
final class ResourceTimeline {
  private static final Color[] COLORS = {
    new Color(0, 113, 132), new Color(222, 130, 28), new Color(117, 80, 159), new Color(40, 135, 70)
  };

  /** Stage attribution uses monotonic task offsets, not reconstructed wall-clock timestamps. */
  static Map<String, Object> stageUsage(Map<String, Object> telemetry, String stage) {
    Object tasks = telemetry.get("tasks"), samples = telemetry.get("resource_timeline");
    if (!(tasks instanceof List<?> events) || !(samples instanceof List<?> rows)) return Map.of();
    List<double[]> intervals = new ArrayList<>();
    for (Object item : events) {
      var event = ProcessingReport.object(item);
      if (!String.valueOf(event.get("task")).startsWith("Stage " + stage + " ")) continue;
      if (event.get("started_offset_seconds") instanceof Number start
          && event.get("wall_seconds") instanceof Number length
          && Double.isFinite(start.doubleValue() + length.doubleValue())
          && length.doubleValue() >= 0)
        intervals.add(
            new double[] {start.doubleValue(), start.doubleValue() + length.doubleValue()});
    }
    if (intervals.isEmpty()) return Map.of();
    Map<String, Object> result = new LinkedHashMap<>();
    int count = 0, cpuCount = 0, gpuCount = 0;
    double cpu = 0, gpu = 0;
    for (Object item : rows) {
      var row = ProcessingReport.object(item);
      if (!(row.get("seconds") instanceof Number at)) continue;
      double time = at.doubleValue();
      if (intervals.stream().noneMatch(range -> time >= range[0] && time <= range[1])) continue;
      count++;
      if (row.get("cpu_percent") instanceof Number n && Double.isFinite(n.doubleValue())) {
        cpu += n.doubleValue();
        cpuCount++;
      }
      if (row.get("gpu_percent") instanceof Number n && Double.isFinite(n.doubleValue())) {
        gpu += n.doubleValue();
        gpuCount++;
      }
      for (String key :
          List.of("heap_bytes", "rss_bytes", "tracked_opencl_bytes", "device_vram_bytes"))
        if (row.get(key) instanceof Number n && n.longValue() >= 0)
          result.put(
              "peak_" + key,
              Math.max(
                  ((Number) result.getOrDefault("peak_" + key, 0L)).longValue(), n.longValue()));
    }
    result.put("samples", count);
    result.put("mean_sampled_cpu_percent", cpuCount == 0 ? null : cpu / cpuCount);
    result.put("mean_sampled_device_gpu_percent", gpuCount == 0 ? null : gpu / gpuCount);
    result.put(
        "scope",
        "Samples inside completed stage intervals; CPU normalized to all processors, GPU"
            + " device-wide. Short/unsampled stages remain unavailable; overlapping stages are not"
            + " additive.");
    return result;
  }

  static void create(Map<String, Object> telemetry, Path file) throws IOException {
    Object value = telemetry.get("resource_timeline");
    boolean available = value instanceof List<?> samples && !samples.isEmpty();
    BufferedImage image =
        new BufferedImage(1100, available ? 590 : 160, BufferedImage.TYPE_INT_RGB);
    Graphics2D g = image.createGraphics();
    g.setColor(Color.WHITE);
    g.fillRect(0, 0, 1100, 590);
    g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
    g.setFont(new Font(Font.SANS_SERIF, Font.PLAIN, 18));
    g.setColor(new Color(30, 55, 70));
    if (!(value instanceof List<?> list) || list.isEmpty()) {
      g.drawString("Resource timeline unavailable in this archived run.", 65, 100);
      g.drawString("Measured scalar peaks, where available, are listed separately.", 65, 135);
    } else {
      List<Map<String, Object>> rows = new ArrayList<>();
      for (Object item : list) rows.add(ProcessingReport.object(item));
      plot(g, rows, "CPU capacity used (%)", List.of("cpu_percent"), 100, 60);
      double ram = 0;
      for (var row : rows)
        for (String key :
            List.of("rss_bytes", "heap_bytes", "tracked_opencl_bytes", "device_vram_bytes"))
          if (row.get(key) instanceof Number n) ram = Math.max(ram, n.doubleValue() / 1048576.);
      plot(
          g,
          rows,
          "Memory (MiB): process / heap / OpenCL buffers / device VRAM",
          List.of("rss_bytes", "heap_bytes", "tracked_opencl_bytes", "device_vram_bytes"),
          Math.max(1, ram),
          235);
      String[] names = {"Process RAM", "Heap", "OpenCL buffers", "Device VRAM"};
      g.setFont(new Font(Font.SANS_SERIF, Font.PLAIN, 16));
      for (int k = 0; k < names.length; k++) {
        g.setColor(COLORS[k]);
        g.fillRect(120 + k * 235, 370, 22, 3);
        g.drawString(names[k], 150 + k * 235, 377);
      }
      g.setFont(new Font(Font.SANS_SERIF, Font.PLAIN, 18));
      plot(
          g,
          rows,
          "GPU device utilization (%) - other programs may contribute",
          List.of("gpu_percent"),
          100,
          410);
    }
    g.dispose();
    ImageIO.write(image, "png", file.toFile());
    image.flush();
  }

  private static void plot(
      Graphics2D g,
      List<Map<String, Object>> rows,
      String label,
      List<String> keys,
      double maximum,
      int top) {
    int left = 105, right = 1040, height = 105;
    g.setColor(new Color(30, 55, 70));
    g.drawString(label, 60, top - 12);
    for (int i = 0; i <= 4; i++) {
      int y = top + height - i * height / 4;
      g.setColor(new Color(220, 228, 233));
      g.drawLine(left, y, right, y);
      g.setColor(Color.DARK_GRAY);
      g.drawString(String.format(Locale.ROOT, "%.1f", maximum * i / 4), 20, y + 5);
    }
    double span = number(rows.get(rows.size() - 1).get("seconds"));
    boolean any = false;
    for (int k = 0; k < keys.size(); k++) {
      String key = keys.get(k);
      g.setColor(COLORS[k % COLORS.length]);
      g.setStroke(new BasicStroke(2));
      int px = -1, py = -1;
      for (var row : rows) {
        Object value = row.get(key);
        if (!(value instanceof Number n)) {
          px = -1;
          continue;
        }
        double v = n.doubleValue();
        if (key.endsWith("_bytes")) v /= 1048576.;
        int x = left + (int) (number(row.get("seconds")) / Math.max(1e-9, span) * (right - left));
        int y = top + height - (int) (Math.clamp(v / maximum, 0, 1) * height);
        if (px >= 0) g.drawLine(px, py, x, y);
        px = x;
        py = y;
        any = true;
      }
    }
    g.setColor(Color.DARK_GRAY);
    g.drawString("0 s", left, top + height + 25);
    g.drawString(String.format(Locale.ROOT, "%.1f s", span), right - 75, top + height + 25);
    if (!any)
      g.drawString("Unavailable - no hardware measurement for this device", left + 100, top + 55);
  }

  private static double number(Object value) {
    return value instanceof Number n ? n.doubleValue() : 0;
  }
}
