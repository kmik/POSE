package sgm;

import java.awt.image.*;
import java.io.*;
import java.nio.*;
import java.nio.file.*;
import java.util.*;
import javax.imageio.ImageIO;

/** Bounded-grid surface raster. Max/mean per cell; unsupported areas remain nodata. */
final class DsmRaster {
  static Map<String, Object> create(Path run, Path out, Options o, Map<String, Object> coordinates)
      throws IOException {
    Object epsgValue = coordinates.get("epsg");
    int epsg = epsgValue instanceof Number n ? n.intValue() : 0;
    boolean projected = Boolean.TRUE.equals(coordinates.get("georeferenced")) && epsg > 0;
    if (!projected && !o.reportLocalDsm)
      return Json.object(
          "status",
          "unavailable",
          "reason",
          "No established projected elevation frame; use --report-local-dsm only when local XY/Z is"
              + " meaningful");
    Path path = run.resolve("07_results/dense.ply");
    if (!Files.isRegularFile(path)) path = run.resolve("dense.ply");
    PlyStream cloud;
    if (Files.isRegularFile(path)) cloud = new PlyStream(path);
    else {
      // --ply off: the LAS cloud (same points, coordinates quantized to the LAS scale).
      Path las = run.resolve("07_results/dense.las");
      if (!Files.isRegularFile(las)) las = run.resolve("dense.las");
      if (!Files.isRegularFile(las))
        return Json.object("status", "unavailable", "reason", "No final PLY/LAS cloud");
      cloud = PlyStream.las(las);
    }
    double[] bounds = {
      Double.POSITIVE_INFINITY,
      Double.POSITIVE_INFINITY,
      Double.POSITIVE_INFINITY,
      Double.NEGATIVE_INFINITY,
      Double.NEGATIVE_INFINITY,
      Double.NEGATIVE_INFINITY
    };
    cloud.scan(
        "Report DSM extent",
        (x, y, z) -> {
          if (!Double.isFinite(x + y + z)) return;
          bounds[0] = Math.min(bounds[0], x);
          bounds[1] = Math.min(bounds[1], y);
          bounds[2] = Math.min(bounds[2], z);
          bounds[3] = Math.max(bounds[3], x);
          bounds[4] = Math.max(bounds[4], y);
          bounds[5] = Math.max(bounds[5], z);
        });
    if (!Double.isFinite(bounds[0] + bounds[3]))
      return Json.object("status", "unavailable", "reason", "No finite cloud points");
    long allowance = Math.max(1048576L, Math.min(128 * 1048576L, o.memoryMb * 1048576L / 8));
    int cap = (int) Math.min(o.reportDsmMaxDimension, Math.floor(Math.sqrt(allowance / 20.)));
    double cell =
        o.reportDsmResolution > 0
            ? o.reportDsmResolution
            : Math.max(bounds[3] - bounds[0], bounds[4] - bounds[1]) / cap;
    if (!(cell > 0)) cell = 1;
    long wl = Math.max(1, (long) Math.ceil((bounds[3] - bounds[0]) / cell)),
        hl = Math.max(1, (long) Math.ceil((bounds[4] - bounds[1]) / cell));
    if (wl > cap || hl > cap)
      throw new IOException(
          "Requested DSM resolution exceeds report dimension/RAM cap; increase"
              + " --report-dsm-resolution or the allowed cap/budget");
    int w = (int) wl, h = (int) hl;
    double[] statistic = new double[w * h];
    long[] counts = new long[w * h];
    if (o.reportDsmStatistic.equals("max")) Arrays.fill(statistic, Double.NEGATIVE_INFINITY);
    final double size = cell;
    cloud.scan(
        "Report DSM rasterization",
        (x, y, z) -> {
          if (!Double.isFinite(x + y + z)) return;
          int col = (int) Math.max(0, Math.min(w - 1, Math.floor((x - bounds[0]) / size))),
              row = (int) Math.max(0, Math.min(h - 1, Math.floor((bounds[4] - y) / size)));
          int p = row * w + col;
          counts[p]++;
          if (o.reportDsmStatistic.equals("max")) statistic[p] = Math.max(statistic[p], z);
          else statistic[p] += z;
        });
    float[] heights = new float[w * h];
    long occupied = 0;
    double low = Double.POSITIVE_INFINITY, high = Double.NEGATIVE_INFINITY;
    for (int p = 0; p < heights.length; p++) {
      double value =
          counts[p] >= o.reportDsmMinPoints
              ? (o.reportDsmStatistic.equals("mean") ? statistic[p] / counts[p] : statistic[p])
              : Double.NaN;
      heights[p] = (float) value;
      if (Float.isFinite(heights[p])) {
        occupied++;
        low = Math.min(low, heights[p]);
        high = Math.max(high, heights[p]);
      }
    }
    writeTiff(
        out.resolve("dsm.tif"), w, h, heights, bounds[0], bounds[4], cell, projected ? epsg : 0);
    int pw = Math.min(1024, w), ph = Math.max(1, (int) Math.round(h * (pw / (double) w)));
    if (ph > 1024) {
      pw = Math.max(1, (int) Math.round(pw * (1024. / ph)));
      ph = 1024;
    }
    BufferedImage preview = new BufferedImage(pw, ph, BufferedImage.TYPE_INT_RGB);
    for (int y = 0; y < ph; y++)
      for (int x = 0; x < pw; x++) {
        int ix = Math.min(w - 1, x * w / pw), iy = Math.min(h - 1, y * h / ph);
        float value = heights[iy * w + ix];
        int rgb = 0xe5e7eb;
        if (Float.isFinite(value)) {
          float t = (float) ((value - low) / Math.max(1e-20, high - low));
          rgb = java.awt.Color.HSBtoRGB((1 - t) * .67f, 1, 1);
        }
        preview.setRGB(x, y, rgb);
      }
    ImageIO.write(preview, "png", out.resolve("dsm.png").toFile());
    preview.flush();
    Files.writeString(
        out.resolve("dsm.tfw"),
        String.format(
            Locale.ROOT,
            "%.12f%n0%n0%n%.12f%n%.12f%n%.12f%n",
            cell,
            -cell,
            bounds[0] + cell / 2,
            bounds[4] - cell / 2));
    Map<String, Object> result =
        Json.object(
            "status",
            "complete",
            "file",
            "dsm.tif",
            "preview",
            "dsm.png",
            "width",
            w,
            "height",
            h,
            "cell_size",
            cell,
            "configured_max_dimension",
            o.reportDsmMaxDimension,
            "effective_max_dimension",
            cap,
            "estimated_grid_bytes",
            20L * w * h,
            "extent",
            new double[] {bounds[0], bounds[4] - h * cell, bounds[0] + w * cell, bounds[4]},
            "minimum_elevation",
            Double.isFinite(low) ? low : null,
            "maximum_elevation",
            Double.isFinite(high) ? high : null,
            "occupied_cells",
            occupied,
            "source_points",
            cloud.count,
            "nodata_cells",
            (long) w * h - occupied,
            "nodata",
            "NaN",
            "gap_fill",
            "none",
            "statistic",
            o.reportDsmStatistic,
            "minimum_points_per_cell",
            o.reportDsmMinPoints,
            "epsg",
            projected ? epsg : null,
            "units",
            coordinates.getOrDefault("xyz_units", "local output units"),
            "vertical_datum",
            coordinates.get("vertical_crs"),
            "height_note",
            coordinates.getOrDefault(
                "height_note", "Local Z projection; elevation meaning supplied by user"),
            "caution",
            "Surface model includes vegetation/roofs; not classified bare-earth terrain. Max cells"
                + " can retain remaining outliers. Preview uses nearest-cell sampling.");
    Json.write(out.resolve("dsm.json"), result);
    return result;
  }

  // GeoTIFF pixel-is-area, north-up; Float32 single strip and explicit NaN nodata.
  static void writeTiff(
      Path path, int w, int h, float[] z, double x, double y, double cell, int epsg)
      throws IOException {
    record Tag(int id, int type, int count, byte[] value) {}
    List<Tag> tags = new ArrayList<>();
    java.util.function.IntFunction<byte[]> integer =
        n -> ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN).putInt(n).array();
    java.util.function.IntFunction<byte[]> small =
        n -> ByteBuffer.allocate(2).order(ByteOrder.LITTLE_ENDIAN).putShort((short) n).array();
    tags.add(new Tag(256, 4, 1, integer.apply(w)));
    tags.add(new Tag(257, 4, 1, integer.apply(h)));
    tags.add(new Tag(258, 3, 1, small.apply(32)));
    tags.add(new Tag(259, 3, 1, small.apply(1)));
    tags.add(new Tag(262, 3, 1, small.apply(1)));
    tags.add(new Tag(273, 4, 1, integer.apply(0)));
    tags.add(new Tag(277, 3, 1, small.apply(1)));
    tags.add(new Tag(278, 4, 1, integer.apply(h)));
    tags.add(new Tag(279, 4, 1, integer.apply(Math.multiplyExact(z.length, 4))));
    tags.add(new Tag(339, 3, 1, small.apply(3)));
    tags.add(
        new Tag(
            33550,
            12,
            3,
            ByteBuffer.allocate(24)
                .order(ByteOrder.LITTLE_ENDIAN)
                .putDouble(cell)
                .putDouble(cell)
                .putDouble(0)
                .array()));
    tags.add(
        new Tag(
            33922,
            12,
            6,
            ByteBuffer.allocate(48)
                .order(ByteOrder.LITTLE_ENDIAN)
                .putDouble(0)
                .putDouble(0)
                .putDouble(0)
                .putDouble(x)
                .putDouble(y)
                .putDouble(0)
                .array()));
    if (epsg > 0) {
      int[] keys = {1, 1, 0, 4, 1024, 0, 1, 1, 1025, 0, 1, 1, 3072, 0, 1, epsg, 3076, 0, 1, 9001};
      ByteBuffer b = ByteBuffer.allocate(keys.length * 2).order(ByteOrder.LITTLE_ENDIAN);
      for (int value : keys) b.putShort((short) value);
      tags.add(new Tag(34735, 3, keys.length, b.array()));
    }
    tags.add(new Tag(42113, 2, 4, new byte[] {'n', 'a', 'n', 0}));
    tags.sort(Comparator.comparingInt(Tag::id));
    int start = 8 + 2 + tags.size() * 12 + 4, extra = 0;
    for (Tag t : tags) if (t.value.length > 4) extra += t.value.length + (t.value.length & 1);
    int pixels = start + extra;
    ByteBuffer header = ByteBuffer.allocate(pixels).order(ByteOrder.LITTLE_ENDIAN);
    header
        .put((byte) 'I')
        .put((byte) 'I')
        .putShort((short) 42)
        .putInt(8)
        .putShort((short) tags.size());
    int cursor = start;
    for (Tag t : tags) {
      header.putShort((short) t.id).putShort((short) t.type).putInt(t.count);
      if (t.id == 273) header.putInt(pixels);
      else if (t.value.length <= 4) {
        header.put(t.value);
        for (int i = t.value.length; i < 4; i++) header.put((byte) 0);
      } else {
        header.putInt(cursor);
        int save = header.position();
        header.position(cursor);
        header.put(t.value);
        cursor += t.value.length + (t.value.length & 1);
        header.position(save);
      }
    }
    header.putInt(0);
    try (OutputStream out = new BufferedOutputStream(Files.newOutputStream(path))) {
      out.write(header.array());
      ByteBuffer row = ByteBuffer.allocate(w * 4).order(ByteOrder.LITTLE_ENDIAN);
      for (int j = 0; j < h; j++) {
        row.clear();
        for (int i = 0; i < w; i++) row.putFloat(z[j * w + i]);
        out.write(row.array());
      }
    }
  }
}
