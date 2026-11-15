package sgm;

import java.awt.image.BufferedImage;
import java.io.*;
import java.nio.file.*;
import java.util.*;
import javax.imageio.ImageIO;

/**
 * Report comparison of a lidar-referenced run with its ALS reference: the photogrammetric DSM
 * minus the lidar DSM (and the photogrammetric DTM of the ground step minus the lidar DTM) on the
 * lidar grid, by surface kind (open ground, low vegetation, canopy, roofs), with a difference map.
 * Uses the rasters the lidar step saved (04_geometry/lidar/als_*.tif) and the report DSM; nothing
 * is read from the LAS/LAZ files again. Windowless.
 */
final class LidarComparison {
  private LidarComparison() {}

  /** A float raster with its world file: west/north edges and square cell. */
  record Raster(int w, int h, float[] z, double x0, double y0, double cell) {
    float at(double x, double y) {
      int c = (int) Math.floor((x - x0) / cell), r = (int) Math.floor((y0 - y) / cell);
      return c < 0 || r < 0 || c >= w || r >= h ? Float.NaN : z[r * w + c];
    }
  }

  /** Reads a float GeoTIFF written by DsmRaster.writeTiff and its .tfw world file (or the given grid). */
  static Raster raster(Path tif) throws IOException {
    DsmRaster.Heights h = DsmRaster.readTiff(tif);
    Path tfw = tif.resolveSibling(tif.getFileName().toString().replaceAll("\\.tif$", ".tfw"));
    if (!Files.isRegularFile(tfw)) throw new IOException("No world file next to " + tif);
    List<String> lines = Files.readAllLines(tfw);
    double cell = Double.parseDouble(lines.get(0).trim()), cx = Double.parseDouble(lines.get(4).trim()), cy = Double.parseDouble(lines.get(5).trim());
    return new Raster(h.width(), h.height(), h.z(), cx - cell / 2, cy + cell / 2, cell);
  }

  /** The lidar grids of the lidar step (grid from als_grid.json). */
  static Raster lidarRaster(Path dir, String name) throws IOException {
    Map<String, Object> g = JsonRead.object(JsonRead.read(dir.resolve("als_grid.json")));
    DsmRaster.Heights h = DsmRaster.readTiff(dir.resolve(name));
    return new Raster(h.width(), h.height(), h.z(), num(g.get("west")), num(g.get("north")), num(g.get("cell_metres")));
  }

  private static double num(Object o) {
    return o instanceof Number n ? n.doubleValue() : Double.NaN;
  }

  /**
   * Differences on the lidar grid. The photogrammetric DSM is taken as the highest of its cells
   * within each lidar cell (both are then top-of-surface samples). Returns null when the run has no
   * lidar reference (the report then has no lidar section).
   */
  static Map<String, Object> collect(Path run, Path out) throws IOException {
    Path dir = run.resolve(LidarReference.FOLDER);
    Path json = dir.resolve("lidar_reference.json");
    if (!Files.isRegularFile(json)) return null;
    Map<String, Object> reference = JsonRead.object(JsonRead.read(json));
    Map<String, Object> result = new LinkedHashMap<>();
    result.put("reference", reference);
    if (!Files.isRegularFile(dir.resolve("als_grid.json"))) {
      result.put("comparison", Json.object("status", "unavailable", "reason", "the lidar step saved no lidar grids"));
      return result;
    }
    Raster dsm = lidarRaster(dir, "als_dsm.tif"), dtm = lidarRaster(dir, "als_dtm.tif"), kind = lidarRaster(dir, "als_kind.tif");
    Path photoDsm = out.resolve("dsm.tif");
    if (!Files.isRegularFile(photoDsm)) photoDsm = run.resolve("07_results/dsm.tif");
    Map<String, Object> comparison = new LinkedHashMap<>();
    if (Files.isRegularFile(photoDsm)) {
      Raster photo = raster(photoDsm);
      float[] top = medianOnGrid(photo, dsm);
      float[] diff = new float[dsm.z().length];
      for (int k = 0; k < diff.length; k++) diff[k] = top[k] - dsm.z()[k];
      comparison.put("dsm", byKind(diff, kind, "photogrammetric DSM (median of its cells in each lidar cell) minus lidar DSM (highest lidar point per cell; on canopy the lidar is then the higher by construction)"));
      image(diff, dsm, out.resolve("lidar-dsm-difference.png"), 1.5);
      comparison.put("dsm_map", "lidar-dsm-difference.png");
      comparison.put("photogrammetric_dsm_cell_metres", photo.cell());
    } else comparison.put("dsm", Json.object("status", "unavailable", "reason", "no report DSM (dsm.tif)"));
    Path photoDtm = run.resolve("07_results/dtm.tif");
    if (Files.isRegularFile(photoDtm)) {
      Raster photo = raster(photoDtm);
      float[] diff = new float[dtm.z().length];
      for (int r = 0; r < dtm.h(); r++)
        for (int c = 0; c < dtm.w(); c++) {
          int k = r * dtm.w() + c;
          diff[k] = photo.at(dtm.x0() + (c + .5) * dtm.cell(), dtm.y0() - (r + .5) * dtm.cell()) - dtm.z()[k];
        }
      comparison.put("dtm", byKind(diff, kind, "photogrammetric DTM (ground step) minus lidar DTM, at lidar cell centres"));
      image(diff, dtm, out.resolve("lidar-dtm-difference.png"), 1.5);
      comparison.put("dtm_map", "lidar-dtm-difference.png");
    } else comparison.put("dtm", Json.object("status", "unavailable", "reason", "no ground step DTM (07_results/dtm.tif); run with --classify-ground"));
    comparison.put("colour_scale", "blue: photogrammetry below the lidar, red: above, white: equal; saturated at +-1.5 m; grey: no data");
    result.put("comparison", comparison);
    Json.write(out.resolve("lidar_comparison.json"), result);
    return result;
  }

  /**
   * Median of the photogrammetric cells inside each lidar cell (at least a quarter of the cell
   * covered). The maximum would compare ~100 dense samples against ~3 lidar points per cell and
   * put the photogrammetric surface metres too high even on open ground.
   */
  static float[] medianOnGrid(Raster photo, Raster grid) {
    int n = grid.w() * grid.h();
    int[] count = new int[n];
    float[] out = new float[n];
    Arrays.fill(out, Float.NaN);
    int[] index = new int[photo.w() * photo.h()];
    for (int r = 0; r < photo.h(); r++)
      for (int c = 0; c < photo.w(); c++) {
        int p = r * photo.w() + c;
        index[p] = -1;
        if (Float.isNaN(photo.z()[p])) continue;
        double x = photo.x0() + (c + .5) * photo.cell(), y = photo.y0() - (r + .5) * photo.cell();
        int gc = (int) Math.floor((x - grid.x0()) / grid.cell()), gr = (int) Math.floor((grid.y0() - y) / grid.cell());
        if (gc < 0 || gr < 0 || gc >= grid.w() || gr >= grid.h()) continue;
        index[p] = gr * grid.w() + gc;
        count[index[p]]++;
      }
    int[] start = new int[n + 1];
    for (int k = 0; k < n; k++) start[k + 1] = start[k] + count[k];
    float[] values = new float[start[n]];
    int[] fill = start.clone();
    for (int p = 0; p < index.length; p++) if (index[p] >= 0) values[fill[index[p]]++] = photo.z()[p];
    double expected = grid.cell() * grid.cell() / (photo.cell() * photo.cell());
    for (int k = 0; k < n; k++) {
      int m = count[k];
      if (m == 0 || m < expected / 4) continue;
      Arrays.sort(values, start[k], start[k + 1]);
      out[k] = values[start[k] + m / 2];
    }
    return out;
  }

  /** Statistics of a difference raster in total, on stable areas (open ground, roofs) and by kind. */
  static Map<String, Object> byKind(float[] diff, Raster kind, String definition) {
    Map<String, List<Double>> groups = new LinkedHashMap<>();
    for (String name : new String[] {"stable", "open_ground", "low_vegetation", "canopy", "roof", "total"}) groups.put(name, new ArrayList<>());
    for (int k = 0; k < diff.length; k++) {
      if (Float.isNaN(diff[k])) continue;
      int t = Math.round(kind.z()[k]);
      if (t == LidarReference.NODATA || t == LidarReference.WATER) continue;
      double d = diff[k];
      groups.get("total").add(d);
      if (t == LidarReference.OPEN) groups.get("open_ground").add(d);
      if (t == LidarReference.LOW) groups.get("low_vegetation").add(d);
      if (t == LidarReference.CANOPY) groups.get("canopy").add(d);
      if (t == LidarReference.ROOF) groups.get("roof").add(d);
      if (t == LidarReference.OPEN || t == LidarReference.ROOF) groups.get("stable").add(d);
    }
    Map<String, Object> out = new LinkedHashMap<>();
    out.put("definition", definition);
    out.put("cell_metres", kind.cell());
    for (var e : groups.entrySet()) {
      LidarReference.Stats s = LidarReference.Stats.of(e.getValue());
      Map<String, Object> m = new LinkedHashMap<>(s.json());
      m.put("area_m2", s.count() * kind.cell() * kind.cell());
      out.put(e.getKey(), m);
    }
    return out;
  }

  /** Diverging colour map: blue (negative) - white - red (positive), saturated at +-limit; grey: no data. */
  static void image(float[] diff, Raster grid, Path file, double limit) throws IOException {
    // Cropped to the cells with a difference (plus 5 cells).
    int c0 = grid.w(), c1 = -1, r0 = grid.h(), r1 = -1;
    for (int r = 0; r < grid.h(); r++)
      for (int c = 0; c < grid.w(); c++)
        if (!Float.isNaN(diff[r * grid.w() + c])) {
          c0 = Math.min(c0, c);
          c1 = Math.max(c1, c);
          r0 = Math.min(r0, r);
          r1 = Math.max(r1, r);
        }
    if (c1 < 0) {
      c0 = r0 = 0;
      c1 = grid.w() - 1;
      r1 = grid.h() - 1;
    }
    c0 = Math.max(0, c0 - 5);
    r0 = Math.max(0, r0 - 5);
    c1 = Math.min(grid.w() - 1, c1 + 5);
    r1 = Math.min(grid.h() - 1, r1 + 5);
    int w = c1 - c0 + 1, h = r1 - r0 + 1;
    int scale = Math.max(1, Math.min(6, 600 / Math.max(w, h)));
    BufferedImage im = new BufferedImage(w * scale, h * scale, BufferedImage.TYPE_INT_RGB);
    for (int r = 0; r < h; r++)
      for (int c = 0; c < w; c++) {
        float d = diff[(r + r0) * grid.w() + c + c0];
        int rgb = Float.isNaN(d) ? 0xd9d9d9 : diverging(d / limit);
        for (int i = 0; i < scale; i++) for (int j = 0; j < scale; j++) im.setRGB(c * scale + j, r * scale + i, rgb);
      }
    ImageIO.write(im, "png", file.toFile());
  }

  static int diverging(double t) {
    t = Math.max(-1, Math.min(1, t));
    int[] lo = {0x21, 0x66, 0xac}, mid = {0xf7, 0xf7, 0xf7}, hi = {0xb2, 0x18, 0x2b};
    int[] a = t < 0 ? lo : hi;
    double f = Math.abs(t);
    int r = (int) Math.round(mid[0] + (a[0] - mid[0]) * f), g = (int) Math.round(mid[1] + (a[1] - mid[1]) * f), b = (int) Math.round(mid[2] + (a[2] - mid[2]) * f);
    return r << 16 | g << 8 | b;
  }
}
