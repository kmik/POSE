package sgm;

import java.awt.image.BufferedImage;
import java.io.*;
import java.nio.file.*;
import java.util.*;
import java.util.function.Consumer;
import javax.imageio.ImageIO;

/**
 * Lidar-referenced georeferencing (the lidar step): the GPS-georeferenced block is moved onto an
 * airborne laser scanning (ALS) surface by a rigid transformation (three rotations about the
 * block centre and three shifts; scale optional), estimated from the tie points.
 *
 * <ol>
 *   <li>Sources: LAS/LAZ headers, CRS (GeoTIFF keys or WKT) against --crs, vertical datum.
 *   <li>Area: the tie points (stray ones dropped) on a 5 m grid plus --lidar-margin.
 *   <li>Lidar grids: one streaming pass over the points near the area: DSM (highest point), DTM
 *       (ground class 2, gaps interpolated), cell kinds (open ground, roof, low vegetation,
 *       canopy, water, no data) and the 5 m coverage grid.
 *   <li>Coverage: every area cell must hold lidar points or be a gap enclosed by them (water,
 *       no returns) inside a file's extent; otherwise the step stops and says what is missing.
 *   <li>Registration: tie points against the surface their cell calls for (open ground and gaps
 *       under canopy: DTM, roofs and canopy: DSM), vertical residuals turned into surface-normal
 *       distances. Coarse search from the GPS solution (shifts within --lidar-search-shift,
 *       heading within --lidar-search-angle, height by a weighted median), then robust
 *       Gauss-Newton on 4x, 2x and 1x cells (Tukey weights, residual limit, canopy weighted
 *       --lidar-canopy-weight, 20 m patches that disagree with the block are rejected as changed).
 *   <li>Uncertainty: the larger of the formal and a spatial block jackknife standard deviation.
 *       Parameters whose effect at the block edge is above --lidar-weak-limit are fixed at the
 *       GPS values and the fit is repeated with fewer parameters.
 * </ol>
 *
 * Windowless; the pipeline step (Pipeline.lidar) applies the result to cameras and tie points.
 */
final class LidarReference {
  private LidarReference() {}

  static final String FOLDER = "04_geometry/lidar";

  // Cell kinds of the lidar grid.
  static final byte NODATA = 0, OPEN = 1, ROOF = 2, LOW = 3, CANOPY = 4, WATER = 5;
  static final String[] KIND_NAMES = {"no data", "open ground", "roof", "low vegetation", "canopy", "water"};
  /** Parameter order: shifts east, north, up; roll (about east), pitch (about north), heading (about up); scale. */
  static final String[] PARAMETERS = {"shift_east", "shift_north", "shift_up", "roll", "pitch", "heading", "scale"};

  record Settings(
      Path source, boolean scale, double searchShift, double searchAngle, double cell, double canopyWeight,
      double outlier, double changeLimit, double weakLimit, double margin, String lidarCrs, String vertical) {
    static Settings of(Options o) {
      return new Settings(o.lidarReference, o.lidarScale, o.lidarSearchShift, o.lidarSearchAngle, o.lidarCell, o.lidarCanopyWeight,
          o.lidarOutlier, o.lidarChangeLimit, o.lidarWeakLimit, o.lidarMargin, o.lidarCrs, o.lidarVerticalDatum);
    }
  }

  // ---- Sources and coordinate systems.

  record Source(List<LasFile.Header> files, int epsg, String crsNote, String vertical, String verticalSource, List<String> warnings) {}

  /**
   * Reads the headers and checks their CRS against the run's. Refuses: a declared EPSG code other
   * than --crs; a WKT without an EPSG code (unless --lidar-crs); mixed declarations.
   */
  static Source sources(Path reference, Crs run, Settings s) throws IOException {
    List<Path> paths = LasFile.files(reference);
    List<LasFile.Header> headers = new ArrayList<>();
    List<String> warnings = new ArrayList<>();
    Set<Integer> verticals = new TreeSet<>();
    int undeclared = 0;
    for (Path p : paths) {
      LasFile.Header h;
      try {
        h = LasFile.header(p);
      } catch (IOException e) {
        throw new IllegalArgumentException("Cannot read lidar file " + p.getFileName() + ": " + e.getMessage(), e);
      }
      if (h.count() == 0) {
        warnings.add(p.getFileName() + " has no points; ignored");
        continue;
      }
      int code = h.crs().horizontal();
      if (code == 0 && s.lidarCrs() != null) code = new Crs(s.lidarCrs()).epsg;
      if (code == 0 && h.crs().wkt() != null && !h.crs().wkt().isBlank())
        throw new IllegalArgumentException("Lidar file " + p.getFileName() + " declares a coordinate system without an EPSG code ("
            + abbreviate(h.crs().wkt()) + "). If it is " + run.name + ", say so with --lidar-crs " + run.name
            + "; otherwise reproject the lidar to " + run.name + " first.");
      if (code == 0) undeclared++;
      else if (code != run.epsg)
        throw new IllegalArgumentException("Lidar file " + p.getFileName() + " is in EPSG:" + code + " but the run is in " + run.name
            + ". Reproject the lidar to " + run.name + " (e.g. las2las -target_epsg " + run.epsg + "), or run the whole block with --crs EPSG:"
            + code + ".");
      if (h.crs().vertical() > 0) verticals.add(h.crs().vertical());
      headers.add(h);
    }
    if (headers.isEmpty()) throw new IllegalArgumentException("The lidar reference has no points: " + reference);
    if (s.lidarCrs() != null && !s.lidarCrs().equals(run.name))
      throw new IllegalArgumentException("--lidar-crs " + s.lidarCrs() + " differs from the run's --crs " + run.name + "; reproject the lidar first");
    String crsNote;
    if (undeclared == headers.size()) {
      crsNote = "not declared in the lidar files; taken to be " + run.name + (s.lidarCrs() != null ? " (--lidar-crs)" : "");
      if (s.lidarCrs() == null) warnings.add("The lidar files declare no coordinate system; they are taken to be in " + run.name + " (--crs).");
    } else if (undeclared > 0) {
      crsNote = run.name + " (declared in " + (headers.size() - undeclared) + " of " + headers.size() + " files, the others taken to match)";
      warnings.add(undeclared + " lidar files declare no coordinate system; they are taken to be in " + run.name + ".");
    } else crsNote = run.name + " (declared in the lidar files, " + headers.get(0).crs().source() + ")";
    String vertical, verticalSource;
    if (s.vertical() != null) {
      vertical = s.vertical();
      verticalSource = "--lidar-vertical-datum";
    } else if (verticals.size() == 1) {
      vertical = LasFile.verticalName(verticals.iterator().next());
      verticalSource = "declared in the lidar files";
    } else if (verticals.size() > 1) {
      throw new IllegalArgumentException("The lidar files declare different height datums " + verticals
          + "; use tiles with one height datum");
    } else {
      vertical = "the lidar reference's heights (not declared in its files"
          + (run.epsg == 3067 ? "; Finnish national ALS (NLS) uses N2000, EPSG:3900" : "") + ")";
      verticalSource = "not declared";
      warnings.add("The lidar files do not declare their height datum" + (run.epsg == 3067 ? " (Finnish national ALS uses N2000)" : "")
          + "; name it with --lidar-vertical-datum.");
    }
    return new Source(headers, run.epsg, crsNote, vertical, verticalSource, warnings);
  }

  private static String abbreviate(String wkt) {
    String one = wkt.replaceAll("\\s+", " ");
    return one.length() > 90 ? one.substring(0, 90) + "..." : one;
  }

  // ---- Grids.

  /** North-up raster: west edge x0, north edge y0, square cells; row 0 is the northern row. */
  static final class Grid {
    final double x0, y0, cell;
    final int w, h;

    Grid(double x0, double y0, double cell, int w, int h) {
      this.x0 = x0;
      this.y0 = y0;
      this.cell = cell;
      this.w = w;
      this.h = h;
    }

    /** A grid aligned to multiples of the cell covering [minX,maxX] x [minY,maxY]. */
    static Grid covering(double minX, double minY, double maxX, double maxY, double cell) {
      double x0 = Math.floor(minX / cell) * cell, y0 = Math.ceil(maxY / cell) * cell;
      int w = (int) Math.max(1, Math.ceil((maxX - x0) / cell)), h = (int) Math.max(1, Math.ceil((y0 - minY) / cell));
      if ((long) w * h > 120_000_000L)
        throw new IllegalArgumentException(String.format(Locale.ROOT,
            "The lidar area is too large for a %.2f m grid (%d x %d cells); use a larger --lidar-cell", cell, w, h));
      return new Grid(x0, y0, cell, w, h);
    }

    int col(double x) {
      return (int) Math.floor((x - x0) / cell);
    }

    int row(double y) {
      return (int) Math.floor((y0 - y) / cell);
    }

    boolean inside(int c, int r) {
      return c >= 0 && r >= 0 && c < w && r < h;
    }

    double cx(int c) {
      return x0 + (c + .5) * cell;
    }

    double cy(int r) {
      return y0 - (r + .5) * cell;
    }

    Map<String, Object> json() {
      return Json.object("west", x0, "north", y0, "cell_metres", cell, "columns", w, "rows", h);
    }
  }

  // ---- Area of interest.

  /** Tie points (x, y, z) in the run's projected frame. */
  record Points(double[] x, double[] y, double[] z) {
    int size() {
      return x.length;
    }
  }

  /** Area grid (5 m): tie-point cells, the area with margin, and the centre used for rotations. */
  record Area(Grid grid, boolean[] core, boolean[] inside, double[] centre, double radius, int used, int stray, double coreM2, double insideM2) {}

  static final double AREA_CELL = 5;

  /**
   * The photogrammetric area: 5 m cells with tie points, stray points (a cell whose 7x7
   * neighbourhood holds fewer than 3 tie points) dropped, closed over 10 m, plus the margin.
   */
  static Area area(Points p, double margin) {
    if (p.size() < 20) throw new IllegalArgumentException("Too few tie points (" + p.size() + ") for lidar referencing");
    double[] xs = p.x().clone(), ys = p.y().clone();
    Arrays.sort(xs);
    Arrays.sort(ys);
    // Bounds from the 0.5..99.5% quantiles widened by a quarter of the span: far strays do not blow up the grid.
    double qx0 = xs[(int) (.005 * (xs.length - 1))], qx1 = xs[(int) Math.ceil(.995 * (xs.length - 1))];
    double qy0 = ys[(int) (.005 * (ys.length - 1))], qy1 = ys[(int) Math.ceil(.995 * (ys.length - 1))];
    double padX = (qx1 - qx0) * .25 + 50, padY = (qy1 - qy0) * .25 + 50;
    double pad = margin + 30;
    Grid g = Grid.covering(qx0 - padX - pad, qy0 - padY - pad, qx1 + padX + pad, qy1 + padY + pad, AREA_CELL);
    int[] count = new int[g.w * g.h];
    for (int i = 0; i < p.size(); i++) {
      int c = g.col(p.x()[i]), r = g.row(p.y()[i]);
      if (g.inside(c, r)) count[r * g.w + c]++;
    }
    boolean[] cells = new boolean[count.length];
    int stray = 0, used = 0;
    for (int r = 0; r < g.h; r++)
      for (int c = 0; c < g.w; c++) {
        int n = count[r * g.w + c];
        if (n == 0) continue;
        int around = 0;
        for (int dr = -3; dr <= 3; dr++)
          for (int dc = -3; dc <= 3; dc++) if (g.inside(c + dc, r + dr)) around += count[(r + dr) * g.w + c + dc];
        if (around >= 3) {
          cells[r * g.w + c] = true;
          used += n;
        } else stray += n;
      }
    stray += p.size() - Arrays.stream(count).sum();
    if (used < 20) throw new IllegalArgumentException("Too few clustered tie points (" + used + ") for lidar referencing");
    // Close gaps between sparse tie points (dilate then erode by 10 m).
    boolean[] core = erode(dilate(cells, g, 2), g, 2);
    for (int i = 0; i < core.length; i++) core[i] |= cells[i];
    boolean[] inside = dilate(core, g, (int) Math.ceil(margin / AREA_CELL));
    double sx = 0, sy = 0;
    List<Double> zs = new ArrayList<>();
    int n = 0;
    for (int i = 0; i < p.size(); i++) {
      int c = g.col(p.x()[i]), r = g.row(p.y()[i]);
      if (!g.inside(c, r) || !cells[r * g.w + c]) continue;
      sx += p.x()[i];
      sy += p.y()[i];
      zs.add(p.z()[i]);
      n++;
    }
    Collections.sort(zs);
    double[] centre = {sx / n, sy / n, zs.get(zs.size() / 2)};
    // Lever for "effect at the block edge": 90th percentile distance of tie points from the centre.
    double[] d = new double[n];
    int k = 0;
    for (int i = 0; i < p.size(); i++) {
      int c = g.col(p.x()[i]), r = g.row(p.y()[i]);
      if (!g.inside(c, r) || !cells[r * g.w + c]) continue;
      d[k++] = Math.hypot(p.x()[i] - centre[0], p.y()[i] - centre[1]);
    }
    Arrays.sort(d);
    double radius = Math.max(1, d[(int) (.9 * (n - 1))]);
    int coreCells = 0, insideCells = 0;
    for (int i = 0; i < core.length; i++) {
      if (core[i]) coreCells++;
      if (inside[i]) insideCells++;
    }
    return new Area(g, core, inside, centre, radius, used, stray, coreCells * AREA_CELL * AREA_CELL, insideCells * AREA_CELL * AREA_CELL);
  }

  static boolean[] dilate(boolean[] in, Grid g, int radius) {
    boolean[] out = new boolean[in.length];
    for (int r = 0; r < g.h; r++)
      for (int c = 0; c < g.w; c++) {
        if (!in[r * g.w + c]) continue;
        for (int dr = -radius; dr <= radius; dr++)
          for (int dc = -radius; dc <= radius; dc++)
            if (dr * dr + dc * dc <= radius * radius && g.inside(c + dc, r + dr)) out[(r + dr) * g.w + c + dc] = true;
      }
    return out;
  }

  static boolean[] erode(boolean[] in, Grid g, int radius) {
    boolean[] inverse = new boolean[in.length];
    for (int i = 0; i < in.length; i++) inverse[i] = !in[i];
    boolean[] grown = dilate(inverse, g, radius);
    boolean[] out = new boolean[in.length];
    for (int i = 0; i < in.length; i++) out[i] = !grown[i];
    return out;
  }

  // ---- Lidar surfaces.

  /** Lidar grids over the area: base cell surfaces and the 5 m coverage of the area grid. */
  static final class Surfaces {
    Grid grid;
    float[] dsm, dtm;
    byte[] kind;
    int[] points;
    boolean[] occupied; // area grid (5 m): cells holding lidar points
    boolean[] inBox; // area grid: cell centre inside a file's header extent
    long read, used, noise, withheld, ground, building, water;
    int filesRead;
    boolean groundClassified;
    double seconds;
    // Acquisition dates from the points' GPS times (adjusted standard GPS time only): median, first, last.
    java.time.LocalDate acquired, acquiredFirst, acquiredLast;
  }

  /** Streams every file overlapping the area grid; nothing but the grids stays in memory. */
  static Surfaces read(Source src, Area area, double cell, Consumer<String> progress) throws IOException {
    long t0 = System.nanoTime();
    Grid a = area.grid();
    Surfaces s = new Surfaces();
    Grid g = Grid.covering(a.x0, a.y0 - a.h * a.cell, a.x0 + a.w * a.cell, a.y0, cell);
    s.grid = g;
    int n = g.w * g.h;
    float[] max = new float[n], groundSum = new float[n];
    int[] all = new int[n], groundN = new int[n], buildingN = new int[n], waterN = new int[n];
    Arrays.fill(max, Float.NEGATIVE_INFINITY);
    s.occupied = new boolean[a.w * a.h];
    s.inBox = new boolean[a.w * a.h];
    double minX = a.x0, maxX = a.x0 + a.w * a.cell, maxY = a.y0, minY = a.y0 - a.h * a.cell;
    List<Double> times = new ArrayList<>();
    for (LasFile.Header h : src.files()) {
      for (int r = 0; r < a.h; r++)
        for (int c = 0; c < a.w; c++) {
          double x = a.cx(c), y = a.cy(r);
          if (x >= h.min()[0] && x <= h.max()[0] && y >= h.min()[1] && y <= h.max()[1]) s.inBox[r * a.w + c] = true;
        }
      if (h.max()[0] < minX || h.min()[0] > maxX || h.max()[1] < minY || h.min()[1] > maxY) continue;
      s.filesRead++;
      try (LasFile.Reader reader = LasFile.open(h.path())) {
        int rl = h.recordLength(), f = h.format(), block = 65536;
        byte[] b = new byte[rl * block];
        long done = 0, lastReport = System.nanoTime();
        for (int k; (k = reader.read(b, block)) > 0; ) {
          for (int i = 0; i < k; i++) {
            int o = i * rl;
            s.read++;
            double x = h.x(LasFile.rawX(b, o)), y = h.y(LasFile.rawY(b, o));
            int c = g.col(x), r = g.row(y);
            if (!g.inside(c, r)) continue;
            if (LasFile.withheld(f, b, o)) {
              s.withheld++;
              continue;
            }
            int cls = LasFile.classification(f, b, o);
            if (cls == 7 || cls == 18) {
              s.noise++;
              continue;
            }
            float z = (float) h.z(LasFile.rawZ(b, o));
            int p = r * g.w + c;
            s.used++;
            int gt = LasFile.gpsTimeOffset(f);
            if (gt >= 0 && (h.globalEncoding() & 1) != 0 && s.used % 997 == 1) times.add(Double.longBitsToDouble(LazDecoder.i64(b, o + gt)));
            all[p]++;
            if (z > max[p]) max[p] = z;
            if (cls == 2) {
              groundSum[p] += z;
              groundN[p]++;
              s.ground++;
            } else if (cls == 6) {
              buildingN[p]++;
              s.building++;
            } else if (cls == 9) {
              waterN[p]++;
              s.water++;
            }
            int ac = a.col(x), ar = a.row(y);
            if (a.inside(ac, ar)) s.occupied[ar * a.w + ac] = true;
          }
          done += k;
          if (progress != null && System.nanoTime() - lastReport > 2e9) {
            lastReport = System.nanoTime();
            progress.accept(String.format(Locale.ROOT, "Reading %s: %.0f%%", h.path().getFileName(), 100.0 * done / Math.max(1, h.count())));
          }
        }
      }
    }
    s.groundClassified = s.ground > 0;
    if (!times.isEmpty()) {
      Collections.sort(times);
      s.acquired = LasFile.gpsDate(times.get(times.size() / 2), 1);
      s.acquiredFirst = LasFile.gpsDate(times.get(0), 1);
      s.acquiredLast = LasFile.gpsDate(times.get(times.size() - 1), 1);
    }
    s.points = all;
    s.dsm = new float[n];
    s.dtm = new float[n];
    for (int p = 0; p < n; p++) {
      s.dsm[p] = all[p] > 0 ? max[p] : Float.NaN;
      s.dtm[p] = groundN[p] > 0 ? groundSum[p] / groundN[p] : Float.NaN;
    }
    if (!s.groundClassified) {
      // No ground class: the lowest of the highest points over 7x7 cells as a crude terrain.
      float[] low = new float[n];
      for (int r = 0; r < g.h; r++)
        for (int c = 0; c < g.w; c++) {
          float m = Float.NaN;
          for (int dr = -3; dr <= 3; dr++)
            for (int dc = -3; dc <= 3; dc++)
              if (g.inside(c + dc, r + dr)) {
                float v = s.dsm[(r + dr) * g.w + c + dc];
                if (!Float.isNaN(v) && (Float.isNaN(m) || v < m)) m = v;
              }
          low[r * g.w + c] = m;
        }
      s.dtm = low;
    }
    fill(s.dtm, g, 12);
    // DSM: single empty cells take the highest neighbour (sparse ALS leaves holes at 2 m).
    float[] dsm = s.dsm.clone();
    for (int r = 0; r < g.h; r++)
      for (int c = 0; c < g.w; c++) {
        int p = r * g.w + c;
        if (!Float.isNaN(dsm[p])) continue;
        float m = Float.NaN;
        for (int dr = -1; dr <= 1; dr++)
          for (int dc = -1; dc <= 1; dc++)
            if (g.inside(c + dc, r + dr)) {
              float v = s.dsm[(r + dr) * g.w + c + dc];
              if (!Float.isNaN(v) && (Float.isNaN(m) || v > m)) m = v;
            }
        dsm[p] = m;
      }
    s.dsm = dsm;
    // Kinds from the canopy height (DSM - DTM) over 3x3 cells and the classes.
    s.kind = new byte[n];
    for (int r = 0; r < g.h; r++)
      for (int c = 0; c < g.w; c++) {
        int p = r * g.w + c;
        if (Float.isNaN(s.dsm[p]) || Float.isNaN(s.dtm[p])) continue;
        if (all[p] > 0 && waterN[p] * 2 > all[p]) {
          s.kind[p] = WATER;
          continue;
        }
        float chm = Float.NEGATIVE_INFINITY, lo = Float.POSITIVE_INFINITY, hi = Float.NEGATIVE_INFINITY;
        for (int dr = -1; dr <= 1; dr++)
          for (int dc = -1; dc <= 1; dc++)
            if (g.inside(c + dc, r + dr)) {
              int q = (r + dr) * g.w + c + dc;
              if (Float.isNaN(s.dsm[q]) || Float.isNaN(s.dtm[q])) continue;
              chm = Math.max(chm, s.dsm[q] - s.dtm[q]);
              lo = Math.min(lo, s.dsm[q]);
              hi = Math.max(hi, s.dsm[q]);
            }
        float own = s.dsm[p] - s.dtm[p];
        if (all[p] > 0 && buildingN[p] * 2 > all[p]) s.kind[p] = hi - lo < 1.5f ? ROOF : CANOPY;
        else if (chm < .5f) s.kind[p] = OPEN;
        else if (own < 2f && chm < 2f) s.kind[p] = LOW;
        else s.kind[p] = CANOPY;
      }
    s.seconds = (System.nanoTime() - t0) / 1e9;
    return s;
  }

  /** Fills NaN cells within {@code reach} cells of data by repeated neighbour averaging. */
  static void fill(float[] z, Grid g, int reach) {
    for (int pass = 0; pass < reach; pass++) {
      float[] next = z.clone();
      boolean changed = false;
      for (int r = 0; r < g.h; r++)
        for (int c = 0; c < g.w; c++) {
          int p = r * g.w + c;
          if (!Float.isNaN(z[p])) continue;
          double sum = 0;
          int k = 0;
          for (int dr = -1; dr <= 1; dr++)
            for (int dc = -1; dc <= 1; dc++)
              if (g.inside(c + dc, r + dr) && !Float.isNaN(z[(r + dr) * g.w + c + dc])) {
                sum += z[(r + dr) * g.w + c + dc];
                k++;
              }
          if (k >= 2) {
            next[p] = (float) (sum / k);
            changed = true;
          }
        }
      System.arraycopy(next, 0, z, 0, z.length);
      if (!changed) break;
    }
  }

  // ---- Coverage.

  record Coverage(boolean complete, double areaM2, double missingM2, double gapM2, double[] missingBox, String where, String message, byte[] map) {}

  /**
   * Area cells (with margin) must hold lidar points, or be empty cells enclosed by lidar data
   * inside a file's extent (water, no returns). Empty cells reachable from outside every file
   * extent through empty cells are missing coverage.
   */
  static Coverage coverage(Area area, Surfaces s, Source src, double margin) {
    Grid g = area.grid();
    int n = g.w * g.h;
    boolean[] outside = new boolean[n];
    ArrayDeque<Integer> queue = new ArrayDeque<>();
    for (int p = 0; p < n; p++)
      if (!s.occupied[p] && !s.inBox[p]) {
        outside[p] = true;
        queue.add(p);
      }
    while (!queue.isEmpty()) {
      int p = queue.poll(), r = p / g.w, c = p % g.w;
      int[][] next = {{c + 1, r}, {c - 1, r}, {c, r + 1}, {c, r - 1}};
      for (int[] q : next)
        if (g.inside(q[0], q[1])) {
          int k = q[1] * g.w + q[0];
          if (!outside[k] && !s.occupied[k]) {
            outside[k] = true;
            queue.add(k);
          }
        }
    }
    // Map: 0 outside area, 1 covered, 2 enclosed gap, 3 missing, 4 covered outside the area.
    byte[] map = new byte[n];
    double cellM2 = g.cell * g.cell;
    double areaM2 = 0, missing = 0, gaps = 0;
    double[] box = {Double.MAX_VALUE, Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE};
    double mx = 0, my = 0;
    int missingCells = 0;
    for (int r = 0; r < g.h; r++)
      for (int c = 0; c < g.w; c++) {
        int p = r * g.w + c;
        if (!area.inside()[p]) {
          map[p] = (byte) (s.occupied[p] ? 4 : 0);
          continue;
        }
        areaM2 += cellM2;
        if (s.occupied[p]) map[p] = 1;
        else if (!outside[p]) {
          map[p] = 2;
          gaps += cellM2;
        } else {
          map[p] = 3;
          missing += cellM2;
          missingCells++;
          double x = g.cx(c), y = g.cy(r);
          box[0] = Math.min(box[0], x - g.cell / 2);
          box[1] = Math.min(box[1], y - g.cell / 2);
          box[2] = Math.max(box[2], x + g.cell / 2);
          box[3] = Math.max(box[3], y + g.cell / 2);
          mx += x;
          my += y;
        }
      }
    if (missingCells == 0)
      return new Coverage(true, areaM2, 0, gaps, null, null,
          String.format(Locale.ROOT, "The lidar covers the whole block area with its %.0f m margin (%.2f ha)%s.",
              margin, areaM2 / 1e4,
              gaps > 0 ? String.format(Locale.ROOT, "; %.1f%% has no lidar returns inside the tiles (water or gaps)", 100 * gaps / areaM2) : ""),
          map);
    String where = direction(mx / missingCells - area.centre()[0], my / missingCells - area.centre()[1]);
    StringBuilder extents = new StringBuilder();
    for (LasFile.Header h : src.files())
      extents.append(String.format(Locale.ROOT, "%n  %s: E %.0f..%.0f, N %.0f..%.0f", h.path().getFileName(), h.min()[0], h.max()[0], h.min()[1], h.max()[1]));
    String message = String.format(Locale.ROOT,
        "The lidar reference does not cover the whole block: %.0f m2 (%.1f%%) of the tie-point area with its %.0f m margin has no lidar"
            + " points, mostly in the %s part (missing cells span E %.0f..%.0f, N %.0f..%.0f). Add the lidar tiles covering that"
            + " area (several files or a folder are allowed). Lidar files read:%s",
        missing, 100 * missing / areaM2, margin, where, box[0], box[2], box[1], box[3], extents);
    return new Coverage(false, areaM2, missing, gaps, box, where, message, map);
  }

  static String direction(double dx, double dy) {
    if (Math.hypot(dx, dy) < 1e-9) return "central";
    String[] names = {"eastern", "north-eastern", "northern", "north-western", "western", "south-western", "southern", "south-eastern"};
    int k = (int) Math.floorMod(Math.round(Math.atan2(dy, dx) / (Math.PI / 4)), 8);
    return names[k];
  }

  // ---- Transformation.

  /** X' = c + A (X - c) + t, A = (1 + s) R; R = Rz(heading) Ry(pitch) Rx(roll). */
  static final class Transform {
    final double[] c;
    double[] a = {1, 0, 0, 0, 1, 0, 0, 0, 1}, t = new double[3];

    Transform(double[] centre) {
      c = centre.clone();
    }

    Transform copy() {
      Transform o = new Transform(c);
      o.a = a.clone();
      o.t = t.clone();
      return o;
    }

    double[] apply(double x, double y, double z) {
      double dx = x - c[0], dy = y - c[1], dz = z - c[2];
      return new double[] {
        c[0] + a[0] * dx + a[1] * dy + a[2] * dz + t[0],
        c[1] + a[3] * dx + a[4] * dy + a[5] * dz + t[1],
        c[2] + a[6] * dx + a[7] * dy + a[8] * dz + t[2]
      };
    }

    /** Applies an increment: shifts d[0..2], rotation vector d[3..5], scale d[6]. */
    void update(double[] d) {
      double[] r = rodrigues(d[3], d[4], d[5]);
      double k = 1 + d[6];
      double[] ra = mul(r, a);
      double[] rt = mv(r, t);
      for (int i = 0; i < 9; i++) a[i] = k * ra[i];
      for (int i = 0; i < 3; i++) t[i] = k * rt[i] + d[i];
    }

    double scale() {
      return Math.cbrt(det(a));
    }

    double[] rotation() {
      double s = scale();
      double[] r = a.clone();
      for (int i = 0; i < 9; i++) r[i] /= s;
      return r;
    }

    /** {roll, pitch, heading} in radians. */
    double[] angles() {
      double[] r = rotation();
      return new double[] {Math.atan2(r[7], r[8]), Math.asin(Math.max(-1, Math.min(1, -r[6]))), Math.atan2(r[3], r[0])};
    }

    /** The same transformation as X' = scale * R * X + T (for Georeference.Similarity). */
    Georeference.Similarity similarity() {
      double s = scale();
      double[] ac = mv(a, c);
      return new Georeference.Similarity(s, rotation(), new double[] {c[0] + t[0] - ac[0], c[1] + t[1] - ac[1], c[2] + t[2] - ac[2]});
    }

    /** Parameter values in report units: metres, radians, scale - 1. */
    double[] values() {
      double[] ang = angles();
      return new double[] {t[0], t[1], t[2], ang[0], ang[1], ang[2], scale() - 1};
    }
  }

  static double[] rodrigues(double x, double y, double z) {
    double th = Math.sqrt(x * x + y * y + z * z);
    if (th < 1e-15) return new double[] {1, -z, y, z, 1, -x, -y, x, 1};
    double kx = x / th, ky = y / th, kz = z / th, s = Math.sin(th), cc = 1 - Math.cos(th);
    return new double[] {
      1 - cc * (ky * ky + kz * kz), -s * kz + cc * kx * ky, s * ky + cc * kx * kz,
      s * kz + cc * kx * ky, 1 - cc * (kx * kx + kz * kz), -s * kx + cc * ky * kz,
      -s * ky + cc * kx * kz, s * kx + cc * ky * kz, 1 - cc * (kx * kx + ky * ky)
    };
  }

  static double[] mul(double[] a, double[] b) {
    double[] o = new double[9];
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) o[i * 3 + j] = a[i * 3] * b[j] + a[i * 3 + 1] * b[3 + j] + a[i * 3 + 2] * b[6 + j];
    return o;
  }

  static double[] mv(double[] a, double[] v) {
    return new double[] {
      a[0] * v[0] + a[1] * v[1] + a[2] * v[2], a[3] * v[0] + a[4] * v[1] + a[5] * v[2], a[6] * v[0] + a[7] * v[1] + a[8] * v[2]
    };
  }

  static double det(double[] a) {
    return a[0] * (a[4] * a[8] - a[5] * a[7]) - a[1] * (a[3] * a[8] - a[5] * a[6]) + a[2] * (a[3] * a[7] - a[4] * a[6]);
  }

  // ---- Surface levels.

  /** One pyramid level: DSM, DTM and their bilinear lookup with gradients. */
  static final class Level {
    final Grid g;
    final float[] dsm, dtm;

    Level(Grid g, float[] dsm, float[] dtm) {
      this.g = g;
      this.dsm = dsm;
      this.dtm = dtm;
    }

    /** Coarser level: mean DTM, max DSM of factor x factor base cells. */
    static Level of(Surfaces s, int factor) {
      Grid b = s.grid;
      if (factor == 1) return new Level(b, s.dsm, s.dtm);
      int w = (b.w + factor - 1) / factor, h = (b.h + factor - 1) / factor;
      Grid g = new Grid(b.x0, b.y0, b.cell * factor, w, h);
      float[] dsm = new float[w * h], dtm = new float[w * h];
      for (int r = 0; r < h; r++)
        for (int c = 0; c < w; c++) {
          float m = Float.NaN;
          double sum = 0;
          int k = 0;
          for (int i = 0; i < factor; i++)
            for (int j = 0; j < factor; j++) {
              int br = r * factor + i, bc = c * factor + j;
              if (br >= b.h || bc >= b.w) continue;
              float v = s.dsm[br * b.w + bc], t = s.dtm[br * b.w + bc];
              if (!Float.isNaN(v)) m = Float.isNaN(m) ? v : Math.max(m, v);
              if (!Float.isNaN(t)) {
                sum += t;
                k++;
              }
            }
          dsm[r * w + c] = m;
          dtm[r * w + c] = k > 0 ? (float) (sum / k) : Float.NaN;
        }
      return new Level(g, dsm, dtm);
    }

    /** {height, dh/dx, dh/dy} of the surface at (x, y), or null outside / no data. */
    double[] sample(float[] z, double x, double y) {
      double fx = (x - g.x0) / g.cell - .5, fy = (g.y0 - y) / g.cell - .5;
      int c = (int) Math.floor(fx), r = (int) Math.floor(fy);
      if (c < 0 || r < 0 || c + 1 >= g.w || r + 1 >= g.h) return null;
      float z00 = z[r * g.w + c], z10 = z[r * g.w + c + 1], z01 = z[(r + 1) * g.w + c], z11 = z[(r + 1) * g.w + c + 1];
      if (Float.isNaN(z00) || Float.isNaN(z10) || Float.isNaN(z01) || Float.isNaN(z11)) return null;
      double u = fx - c, v = fy - r;
      double hgt = z00 * (1 - u) * (1 - v) + z10 * u * (1 - v) + z01 * (1 - u) * v + z11 * u * v;
      double dx = ((z10 - z00) * (1 - v) + (z11 - z01) * v) / g.cell;
      double dyDown = ((z01 - z00) * (1 - u) + (z11 - z10) * u) / g.cell;
      return new double[] {hgt, dx, -dyDown};
    }
  }

  // ---- Registration.

  /** Observation categories of a tie point against the lidar. */
  static final int SKIP = 0, STABLE = 1, GAP = 2, CANOPY_OBS = 3, REGROWTH = 4, UNDERSTOREY = 5, BELOW_GROUND = 6;
  static final double BELOW_GROUND_LIMIT = 3, CANOPY_CHANGE_LIMIT = 10;
  static final String[] OBS_NAMES = {"outside", "stable", "ground under canopy", "canopy", "vegetation on lidar open ground", "below the crown tops", "below the lidar ground"};

  record Stats(int count, double mean, double median, double rms, double nmad) {
    Map<String, Object> json() {
      return Json.object("points", count, "mean_metres", mean, "median_metres", median, "rms_metres", rms, "nmad_metres", nmad);
    }

    static Stats of(List<Double> v) {
      if (v.isEmpty()) return new Stats(0, Double.NaN, Double.NaN, Double.NaN, Double.NaN);
      double[] a = v.stream().mapToDouble(Double::doubleValue).sorted().toArray();
      double sum = 0, sq = 0;
      for (double x : a) {
        sum += x;
        sq += x * x;
      }
      double med = a[a.length / 2];
      double[] dev = new double[a.length];
      for (int i = 0; i < a.length; i++) dev[i] = Math.abs(a[i] - med);
      Arrays.sort(dev);
      return new Stats(a.length, sum / a.length, med, Math.sqrt(sq / a.length), 1.4826 * dev[dev.length / 2]);
    }
  }

  /** Result of one registration with a given parameter set. */
  static final class Fit {
    Transform transform;
    boolean[] active = new boolean[7];
    double[] sigmaFormal = new double[7], sigmaJackknife = new double[7], sigma = new double[7];
    double sigma0;
    int used, outliers, changedPatches, patches, observations, iterations;
    double[] weakRatio = new double[7];
    double[] searchBest;
    Search search;
    boolean atSearchEdge;
    int[] category; // per tie point
    double[] residual, weight;
    boolean[] changed;
    List<Double> canopyOffsets = List.of();
    int belowCrownTops;
    final List<Object> groups = new ArrayList<>();
  }

  /** Coarse search from the GPS solution, then robust coarse-to-fine refinement. */
  static Fit register(Points p, Surfaces s, Area area, Settings set, boolean[] active, Consumer<String> log) {
    Level[] levels = {Level.of(s, 4), Level.of(s, 2), Level.of(s, 1)};
    Search found = search(p, s, levels[1], area, set);
    Transform start = found.transform().copy();
    // Parameters that are not estimated stay at the GPS values (the search moves shifts and heading).
    if (!active[5]) start.a = new double[] {1, 0, 0, 0, 1, 0, 0, 0, 1};
    if (!active[0]) start.t[0] = 0;
    if (!active[1]) start.t[1] = 0;
    double[] sb = start.values();
    Fit fit = new Fit();
    fit.search = found;
    fit.searchBest = new double[] {sb[0], sb[1], sb[2], sb[5]};
    fit.atSearchEdge = set.searchShift() > 0 && (Math.abs(sb[0]) >= set.searchShift() - 1e-9 || Math.abs(sb[1]) >= set.searchShift() - 1e-9)
        || set.searchAngle() > 0 && Math.abs(Math.toDegrees(sb[5])) >= set.searchAngle() - 1e-9;
    if (log != null)
      log.accept(String.format(Locale.ROOT, "Lidar coarse search: shift %.2f / %.2f / %.2f m, heading %.2f deg; correlation %.3f (contrast %.1f)",
          sb[0], sb[1], sb[2], Math.toDegrees(sb[5]), found.ncc(), found.contrast()));
    Transform t = start.copy();
    boolean[] changed = new boolean[p.size()];
    for (int li = 0; li < levels.length; li++) {
      double limit = Math.max(set.outlier(), 2 * levels[li].g.cell);
      fit.iterations += refine(p, s, levels[li], t, active, set, limit, changed, 15, null);
    }
    // Changed areas: 20 m patches whose median residual departs from the block's.
    Level base = levels[2];
    for (int round = 0; round < 2; round++) {
      boolean any = markChanged(p, s, base, t, set, changed, fit);
      if (!any) break;
      fit.iterations += refine(p, s, base, t, active, set, set.outlier(), changed, 15, null);
    }
    fit.transform = t;
    fit.active = active.clone();
    fit.changed = changed;
    // Final residuals, weights and the formal covariance.
    double[][] normal = new double[7][7];
    double[] stats = new double[3];
    evaluate(p, s, base, t, active, set, set.outlier(), changed, normal, null, stats, fit);
    fit.sigma0 = stats[2];
    double[] diag = covarianceDiagonal(normal, active, fit.sigma0);
    fit.sigmaFormal = diag;
    // Spatial block jackknife (4 x 4 blocks over the tie-point area).
    fit.sigmaJackknife = jackknife(p, s, base, t, active, set, changed, area, fit);
    for (int k = 0; k < 7; k++) fit.sigma[k] = active[k] ? Math.max(fit.sigmaFormal[k], fit.sigmaJackknife[k]) : 0;
    for (int k = 0; k < 7; k++) {
      double lever = k <= 2 ? 1 : area.radius();
      fit.weakRatio[k] = active[k] ? fit.sigma[k] * lever / set.weakLimit() : 0;
    }
    return fit;
  }

  /** Coarse search diagnostics: best correlation, its contrast to the rest of the search window. */
  record Search(Transform transform, double ncc, double background, double contrast, int overlap, boolean ambiguous) {}

  static final int HIGH_PASS_CELLS = 5;
  static final double HIGH_PASS_CLIP = 2.5;

  /**
   * Coarse search: normalised cross-correlation of the high-passed surfaces (the tie points' top
   * surface and the lidar DSM, each minus its 11 x 11 cell mean, so terrain slope does not trade
   * horizontal shift for height) over whole-cell shifts within --lidar-search-shift and headings
   * within --lidar-search-angle (0.5 degree steps). Canopy texture, roofs and terrain breaks
   * carry the signal. The height comes from a weighted median of the surface distances.
   */
  static Search search(Points p, Surfaces s, Level heightLevel, Area area, Settings set) {
    Grid g = s.grid;
    float[] als = highPass(s.dsm, g, HIGH_PASS_CELLS);
    int ns = (int) Math.ceil(set.searchShift() / g.cell - 1e-9);
    double angleStep = .5;
    int na = (int) Math.round(set.searchAngle() / angleStep);
    double best = Double.NEGATIVE_INFINITY;
    int bx = 0, by = 0, ba = 0, bestOverlap = 0;
    List<Double> all = new ArrayList<>();
    double[][] bestMap = null;
    for (int ia = -na; ia <= na; ia++) {
      Transform t = new Transform(area.centre());
      t.a = rodrigues(0, 0, Math.toRadians(ia * angleStep));
      float[] tp = highPass(topSurface(p, g, t), g, HIGH_PASS_CELLS);
      double[][] map = new double[2 * ns + 1][2 * ns + 1];
      for (int dy = -ns; dy <= ns; dy++)
        for (int dx = -ns; dx <= ns; dx++) {
          double[] r = ncc(tp, als, g, dx, dy);
          map[dy + ns][dx + ns] = r[0];
          if (Double.isNaN(r[0])) continue;
          all.add(r[0]);
          // Prefer the GPS solution among equal correlations.
          double score = r[0] - 1e-9 * (Math.abs(dx) + Math.abs(dy) + Math.abs(ia));
          if (score > best) {
            best = score;
            bx = dx;
            by = dy;
            ba = ia;
            bestOverlap = (int) r[1];
            bestMap = map;
          }
        }
    }
    if (bestMap == null) throw new IllegalArgumentException("No tie points fall on the lidar surface; check the CRS and the lidar coverage");
    // Background: correlations at the best heading more than 2 cells from the peak.
    List<Double> rest = new ArrayList<>();
    for (int dy = -ns; dy <= ns; dy++)
      for (int dx = -ns; dx <= ns; dx++)
        if (Math.max(Math.abs(dx - bx), Math.abs(dy - by)) > 2 && !Double.isNaN(bestMap[dy + ns][dx + ns])) rest.add(bestMap[dy + ns][dx + ns]);
    Stats bg = Stats.of(rest);
    double background = rest.isEmpty() ? 0 : bg.median();
    double spread = rest.size() < 5 ? Double.NaN : Math.max(1e-6, bg.nmad());
    double contrast = Double.isNaN(spread) ? Double.NaN : (best - background) / spread;
    // Sub-cell peak by parabolas through the neighbours (when inside the window).
    double fx = bx, fy = by;
    if (Math.abs(bx) < ns) fx += vertex(bestMap[by + ns][bx + ns - 1], bestMap[by + ns][bx + ns], bestMap[by + ns][bx + ns + 1]);
    if (Math.abs(by) < ns) fy += vertex(bestMap[by + ns - 1][bx + ns], bestMap[by + ns][bx + ns], bestMap[by + ns + 1][bx + ns]);
    boolean ambiguous = !(contrast >= 4);
    Transform t = new Transform(area.centre());
    // A weak peak is no evidence: the refinement then starts from the GPS position and heading.
    if (!ambiguous) {
      t.a = rodrigues(0, 0, Math.toRadians(ba * angleStep));
      t.t = new double[] {fx * g.cell, fy * g.cell, 0};
    }
    List<Integer> sample = new ArrayList<>();
    for (int i = 0; i < p.size(); i += Math.max(1, p.size() / 20000)) sample.add(i);
    double[] cost = searchCost(p, s, heightLevel, t, sample, set);
    if (cost == null) throw new IllegalArgumentException("No tie points fall on the lidar surface; check the CRS and the lidar coverage");
    t.t[2] = -cost[1]; // residuals are tie point minus surface: shift down by their median
    return new Search(t, best, background, contrast, bestOverlap, ambiguous);
  }

  private static double vertex(double a, double b, double c) {
    double d = a - 2 * b + c;
    if (!(d < 0) || Double.isNaN(a + c)) return 0;
    return Math.max(-.5, Math.min(.5, .5 * (a - c) / d));
  }

  /** Highest transformed tie point per cell (NaN: none). */
  static float[] topSurface(Points p, Grid g, Transform t) {
    float[] z = new float[g.w * g.h];
    Arrays.fill(z, Float.NaN);
    for (int i = 0; i < p.size(); i++) {
      double[] q = t.apply(p.x()[i], p.y()[i], p.z()[i]);
      int c = g.col(q[0]), r = g.row(q[1]);
      if (!g.inside(c, r)) continue;
      int k = r * g.w + c;
      if (Float.isNaN(z[k]) || q[2] > z[k]) z[k] = (float) q[2];
    }
    return z;
  }

  /** Value minus the mean of the valid cells in its (2 rad + 1)^2 window (integral images); NaN where fewer than 5. */
  static float[] highPass(float[] z, Grid g, int rad) {
    int w = g.w, h = g.h;
    double[] sum = new double[(w + 1) * (h + 1)];
    int[] count = new int[(w + 1) * (h + 1)];
    for (int r = 0; r < h; r++)
      for (int c = 0; c < w; c++) {
        float v = z[r * w + c];
        boolean ok = !Float.isNaN(v);
        int k = (r + 1) * (w + 1) + c + 1;
        sum[k] = (ok ? v : 0) + sum[k - 1] + sum[k - (w + 1)] - sum[k - (w + 1) - 1];
        count[k] = (ok ? 1 : 0) + count[k - 1] + count[k - (w + 1)] - count[k - (w + 1) - 1];
      }
    float[] out = new float[z.length];
    for (int r = 0; r < h; r++)
      for (int c = 0; c < w; c++) {
        int k = r * w + c;
        if (Float.isNaN(z[k])) {
          out[k] = Float.NaN;
          continue;
        }
        int r0 = Math.max(0, r - rad), r1 = Math.min(h, r + rad + 1), c0 = Math.max(0, c - rad), c1 = Math.min(w, c + rad + 1);
        int a = r0 * (w + 1) + c0, b = r0 * (w + 1) + c1, d = r1 * (w + 1) + c0, e = r1 * (w + 1) + c1;
        int n = count[e] - count[b] - count[d] + count[a];
        double s = sum[e] - sum[b] - sum[d] + sum[a];
        // Clipped: structure that changed since the lidar (a felled stand, a new building) must not
        // dominate the correlation; ordinary crown and terrain texture stays within the clip.
        out[k] = n < 5 ? Float.NaN : (float) Math.max(-HIGH_PASS_CLIP, Math.min(HIGH_PASS_CLIP, z[k] - s / n));
      }
    return out;
  }

  /** {NCC, overlap} of tie surface {@code a} shifted by (dx east, dy north) cells against {@code b}; NaN below 50 cells. */
  static double[] ncc(float[] a, float[] b, Grid g, int dx, int dy) {
    double sa = 0, sb = 0, saa = 0, sbb = 0, sab = 0;
    int n = 0;
    for (int r = 0; r < g.h; r++) {
      int rb = r - dy;
      if (rb < 0 || rb >= g.h) continue;
      for (int c = 0; c < g.w; c++) {
        float va = a[r * g.w + c];
        if (Float.isNaN(va)) continue;
        int cb = c + dx;
        if (cb < 0 || cb >= g.w) continue;
        float vb = b[rb * g.w + cb];
        if (Float.isNaN(vb)) continue;
        sa += va;
        sb += vb;
        saa += va * va;
        sbb += vb * vb;
        sab += va * vb;
        n++;
      }
    }
    if (n < 50) return new double[] {Double.NaN, n};
    double den = Math.sqrt(Math.max(0, saa - sa * sa / n) * Math.max(0, sbb - sb * sb / n));
    return new double[] {den > 0 ? (sab - sa * sb / n) / den : Double.NaN, n};
  }

  /** {robust cost, height offset} for a candidate, or null with too few points on the lidar. */
  private static double[] searchCost(Points p, Surfaces s, Level level, Transform t, List<Integer> sample, Settings set) {
    List<double[]> stable = new ArrayList<>(), all = new ArrayList<>();
    for (int i : sample) {
      double[] q = t.apply(p.x()[i], p.y()[i], p.z()[i]);
      byte kind = kindAt(s, q[0], q[1]);
      if (kind == NODATA || kind == WATER) continue;
      double[] dsm = level.sample(level.dsm, q[0], q[1]), dtm = level.sample(level.dtm, q[0], q[1]);
      if (dsm == null || dtm == null) continue;
      if (kind == OPEN) stable.add(new double[] {q[2] - dtm[0], 1});
      else if (kind == ROOF) stable.add(new double[] {q[2] - dsm[0], 1});
      all.add(new double[] {q[2] - (kind == OPEN ? dtm[0] : dsm[0]), kind == OPEN || kind == ROOF ? 1 : Math.max(.05, set.canopyWeight()), q[2] - dtm[0]});
    }
    if (all.size() < 20) return null;
    double dz = weightedMedian(stable.size() >= 20 ? stable : all);
    double cost = 0, wsum = 0;
    for (double[] o : all) {
      double r = o[0] - dz;
      // Under canopy a point near the ground counts against the ground.
      if (o.length > 2 && Math.abs(o[2] - dz) < Math.abs(r)) r = o[2] - dz;
      cost += o[1] * Math.min(Math.abs(r), 3);
      wsum += o[1];
    }
    return new double[] {cost / wsum, dz};
  }

  static double weightedMedian(List<double[]> values) {
    List<double[]> v = new ArrayList<>(values);
    v.sort(Comparator.comparingDouble(a -> a[0]));
    double total = 0;
    for (double[] a : v) total += a[1];
    double acc = 0;
    for (double[] a : v) {
      acc += a[1];
      if (acc >= total / 2) return a[0];
    }
    return v.get(v.size() / 2)[0];
  }

  static byte kindAt(Surfaces s, double x, double y) {
    int c = s.grid.col(x), r = s.grid.row(y);
    return s.grid.inside(c, r) ? s.kind[r * s.grid.w + c] : NODATA;
  }

  /**
   * Residuals of all tie points: fills the weighted normal equations (if given), the per-point
   * category/residual/weight of {@code fit} (if given) and stats {sum w, count used, sigma0}.
   */
  private static void evaluate(
      Points p, Surfaces s, Level level, Transform t, boolean[] active, Settings set, double limit, boolean[] changed,
      double[][] normal, double[] rhs, double[] stats, Fit fit) {
    int n = p.size();
    double[] res = new double[n], base = new double[n];
    double[][] jac = new double[n][];
    int[] cat = new int[n];
    double[][] moved = new double[n][];
    for (int i = 0; i < n; i++) moved[i] = t.apply(p.x()[i], p.y()[i], p.z()[i]);
    // Ground test allowance grows with the terrain slope (lidar DTM): on a slope the lowest tie point
    // of the 7.5 m window lies lower than a ground point by up to the slope times ~5 m.
    double[] allowance = new double[n];
    for (int i = 0; i < n; i++) {
      double[] d = level.sample(level.dtm, moved[i][0], moved[i][1]);
      allowance[i] = GROUND_BAND + (d == null ? 0 : GROUND_SLOPE_REACH * Math.hypot(d[1], d[2]));
    }
    int[] rank = localRank(moved, allowance);
    long[] patch = new long[n];
    for (int i = 0; i < n; i++) {
      double[] q = moved[i];
      byte kind = kindAt(s, q[0], q[1]);
      if (kind == NODATA || kind == WATER) continue;
      double[] dsm = level.sample(level.dsm, q[0], q[1]), dtm = level.sample(level.dtm, q[0], q[1]);
      if (dsm == null || dtm == null) continue;
      double[] surface;
      // More than 3 m below the lidar terrain cannot be a real surface point (gross blunder).
      if (q[2] < dtm[0] - BELOW_GROUND_LIMIT) {
        cat[i] = BELOW_GROUND;
        continue;
      }
      // The lidar may be years older than the images: ground counts only where the tie point is
      // also ground in the photogrammetric cloud (locally lowest); an open lidar cell with tie
      // points above the local ground has grown since (regrowth) and is left out.
      if (kind == OPEN) {
        if ((rank[i] & GROUND_LIKE) == 0) {
          cat[i] = REGROWTH;
          continue;
        }
        surface = dtm;
        cat[i] = STABLE;
        base[i] = 1;
      } else if (kind == ROOF) {
        surface = dsm;
        cat[i] = STABLE;
        base[i] = 1;
      } else if ((rank[i] & GROUND_LIKE) != 0 && Math.abs(q[2] - dtm[0]) < 1.5 && q[2] < dsm[0] - 1.5 && Math.abs(q[2] - dtm[0]) < Math.abs(q[2] - dsm[0])) {
        // Ground under the lidar's canopy: a gap, or the stand was felled since.
        surface = dtm;
        cat[i] = GAP;
        base[i] = .5;
      } else {
        surface = dsm;
        cat[i] = CANOPY_OBS;
        base[i] = set.canopyWeight();
      }
      double norm = Math.sqrt(1 + surface[1] * surface[1] + surface[2] * surface[2]);
      res[i] = (q[2] - surface[0]) / norm;
      double[] a = {-surface[1] / norm, -surface[2] / norm, 1 / norm};
      double dx = q[0] - t.c[0], dy = q[1] - t.c[1], dz = q[2] - t.c[2];
      // d(r)/d(rotation vector) = d x a; d(r)/d(scale) = a . d.
      jac[i] = new double[] {a[0], a[1], a[2], dy * a[2] - dz * a[1], dz * a[0] - dx * a[2], dx * a[1] - dy * a[0], a[0] * dx + a[1] * dy + a[2] * dz};
      patch[i] = patchKey(q[0], q[1]);
    }
    // Canopy height is free per 20 m patch (growth since the lidar, stand by stand): canopy points
    // inform the fit only through the shape of the crowns, never through their absolute height.
    Map<Long, List<double[]>> canopy = new HashMap<>();
    List<double[]> allCanopy = new ArrayList<>();
    for (int i = 0; i < n; i++) if (cat[i] == CANOPY_OBS && !changed[i]) allCanopy.add(new double[] {res[i], 1});
    double global = allCanopy.isEmpty() ? 0 : weightedMedian(allCanopy);
    // Canopy change beyond 10 m from the block's median is not growth but a blunder or a felled stand.
    for (int i = 0; i < n; i++)
      if (cat[i] == CANOPY_OBS && !changed[i] && Math.abs(res[i] - global) <= CANOPY_CHANGE_LIMIT)
        canopy.computeIfAbsent(patch[i], k -> new ArrayList<>()).add(new double[] {res[i], 1});
    Map<Long, Double> offset = new HashMap<>();
    for (var e : canopy.entrySet()) offset.put(e.getKey(), e.getValue().size() >= 5 ? weightedMedian(e.getValue()) : global);
    // For the report: canopy change from the crown tops only (understorey and crown sides would read
    // as shrinking), before the offsets are taken off.
    Map<Long, List<double[]>> tops = new HashMap<>();
    int below = 0;
    for (int i = 0; i < n; i++) {
      if (cat[i] != CANOPY_OBS) continue;
      if ((rank[i] & TOP_LIKE) == 0) below++;
      else if (!changed[i] && Math.abs(res[i] - global) <= CANOPY_CHANGE_LIMIT) tops.computeIfAbsent(patch[i], k -> new ArrayList<>()).add(new double[] {res[i], 1});
    }
    for (int i = 0; i < n; i++) if (cat[i] == CANOPY_OBS) res[i] -= offset.getOrDefault(patch[i], global);
    if (fit != null) {
      List<Double> growth = new ArrayList<>();
      for (var e : tops.values()) if (e.size() >= 5) growth.add(weightedMedian(e));
      fit.canopyOffsets = growth;
      fit.belowCrownTops = below;
    }
    // Robust scale from the stable points (all if too few), Tukey biweight.
    List<Double> sr = new ArrayList<>(), ar = new ArrayList<>();
    for (int i = 0; i < n; i++) {
      if (cat[i] == SKIP || changed[i] || base[i] <= 0) continue;
      if (Math.abs(res[i]) > limit) continue;
      ar.add(res[i]);
      if (cat[i] != CANOPY_OBS) sr.add(res[i]);
    }
    Stats st = Stats.of(sr.size() >= 30 ? sr : ar);
    double scale = Math.max(.03, Double.isNaN(st.nmad()) ? limit : st.nmad());
    double tukey = 4.685 * scale;
    double sw = 0, swr = 0;
    int used = 0, outliers = 0, inside = 0;
    double[] weights = new double[n];
    for (int i = 0; i < n; i++) {
      if (cat[i] == SKIP) continue;
      inside++;
      if (changed[i] || base[i] <= 0) continue;
      double r = res[i] - (Double.isNaN(st.median()) ? 0 : 0);
      if (Math.abs(r) > limit || Math.abs(r) > tukey) {
        outliers++;
        continue;
      }
      double u = r / tukey, w = base[i] * (1 - u * u) * (1 - u * u);
      weights[i] = w;
      used++;
      sw += w;
      swr += w * r * r;
      if (normal != null)
        for (int a = 0; a < 7; a++) {
          if (!active[a]) continue;
          if (rhs != null) rhs[a] -= w * jac[i][a] * r;
          for (int b = 0; b < 7; b++) if (active[b]) normal[a][b] += w * jac[i][a] * jac[i][b];
        }
    }
    int np = 0;
    for (boolean b : active) if (b) np++;
    if (stats != null) {
      stats[0] = sw;
      stats[1] = used;
      // sigma0 of unit weight: weighted residual RMS, normalised by the mean weight.
      stats[2] = used > np ? Math.sqrt(swr / sw * used / (used - np)) : Double.NaN;
    }
    if (fit != null) {
      fit.category = cat;
      fit.residual = res;
      fit.weight = weights;
      fit.used = used;
      fit.outliers = outliers;
      fit.observations = inside;
    }
  }

  static long patchKey(double x, double y) {
    return Math.floorDiv((long) Math.floor(x), 20L) << 32 ^ Math.floorMod(Math.floorDiv((long) Math.floor(y), 20L), 1L << 32);
  }

  static final double GROUND_CELL = 2.5, GROUND_BAND = .7, GROUND_SLOPE_REACH = 2.5;

  /**
   * Tie points that are ground in the photogrammetric cloud: within the allowance (0.7 m plus the
   * slope term) of the lowest tie point in their 7.5 m neighbourhood (3 x 3 cells of 2.5 m).
   * Crowns, saplings and roofs are not.
   */
  static final int GROUND_LIKE = 1, TOP_LIKE = 2;

  /**
   * Local rank of each tie point in its 7.5 m neighbourhood (3 x 3 cells of 2.5 m): GROUND_LIKE
   * within the allowance of the second-lowest tie point there (one point below the ground, noise or
   * a blunder, does not decide), TOP_LIKE within 1.5 m plus the slope term of the second-highest.
   */
  static int[] localRank(double[][] q, double[] allowance) {
    Map<Long, double[]> cells = new HashMap<>(); // two lowest, two highest per cell
    for (double[] p : q) {
      long k = cellKey(p);
      double[] c = cells.computeIfAbsent(k, x -> new double[] {Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY});
      double z = p[2];
      if (z < c[0]) {
        c[1] = c[0];
        c[0] = z;
      } else if (z < c[1]) c[1] = z;
      if (z > c[2]) {
        c[3] = c[2];
        c[2] = z;
      } else if (z > c[3]) c[3] = z;
    }
    int[] out = new int[q.length];
    for (int i = 0; i < q.length; i++) {
      long cx = (long) Math.floor(q[i][0] / GROUND_CELL), cy = (long) Math.floor(q[i][1] / GROUND_CELL);
      double l1 = Double.POSITIVE_INFINITY, l2 = Double.POSITIVE_INFINITY, h1 = Double.NEGATIVE_INFINITY, h2 = Double.NEGATIVE_INFINITY;
      for (long dx = -1; dx <= 1; dx++)
        for (long dy = -1; dy <= 1; dy++) {
          double[] c = cells.get((cx + dx) << 32 ^ Math.floorMod(cy + dy, 1L << 32));
          if (c == null) continue;
          for (int k = 0; k < 2; k++) {
            double v = c[k];
            if (v < l1) {
              l2 = l1;
              l1 = v;
            } else if (v < l2) l2 = v;
            double u = c[2 + k];
            if (u > h1) {
              h2 = h1;
              h1 = u;
            } else if (u > h2) h2 = u;
          }
        }
      double low = Double.isFinite(l2) ? l2 : l1, high = Double.isFinite(h2) ? h2 : h1;
      if (q[i][2] - low < allowance[i]) out[i] |= GROUND_LIKE;
      if (high - q[i][2] < allowance[i] - GROUND_BAND + 1.5) out[i] |= TOP_LIKE;
    }
    return out;
  }

  private static long cellKey(double[] p) {
    long cx = (long) Math.floor(p[0] / GROUND_CELL), cy = (long) Math.floor(p[1] / GROUND_CELL);
    return cx << 32 ^ Math.floorMod(cy, 1L << 32);
  }

  /** Gauss-Newton iterations at one level; returns the number of iterations. */
  private static int refine(Points p, Surfaces s, Level level, Transform t, boolean[] active, Settings set, double limit, boolean[] changed, int max, Fit fit) {
    int it = 0;
    for (; it < max; it++) {
      double[][] normal = new double[7][7];
      double[] rhs = new double[7], stats = new double[3];
      evaluate(p, s, level, t, active, set, limit, changed, normal, rhs, stats, fit);
      if (stats[1] < 10) throw new IllegalArgumentException("Too few tie points on the lidar surface (" + (int) stats[1] + ") to fit the block");
      double[] d = solve(normal, rhs, active);
      t.update(d);
      double move = Math.abs(d[0]) + Math.abs(d[1]) + Math.abs(d[2]) + 300 * (Math.abs(d[3]) + Math.abs(d[4]) + Math.abs(d[5]) + Math.abs(d[6]));
      if (move < 1e-4) {
        it++;
        break;
      }
    }
    return it;
  }

  /** Solves the active part of N d = b (others zero); a tiny ridge keeps near-singular systems finite. */
  static double[] solve(double[][] normal, double[] rhs, boolean[] active) {
    int[] idx = activeIndex(active);
    int m = idx.length;
    double[][] a = new double[m][m + 1];
    for (int i = 0; i < m; i++) {
      for (int j = 0; j < m; j++) a[i][j] = normal[idx[i]][idx[j]];
      a[i][i] += 1e-9 * Math.max(1, normal[idx[i]][idx[i]]);
      a[i][m] = rhs[idx[i]];
    }
    double[] x = gauss(a, m);
    double[] d = new double[7];
    for (int i = 0; i < m; i++) d[idx[i]] = x[i];
    return d;
  }

  private static int[] activeIndex(boolean[] active) {
    int m = 0;
    for (boolean b : active) if (b) m++;
    int[] idx = new int[m];
    m = 0;
    for (int k = 0; k < active.length; k++) if (active[k]) idx[m++] = k;
    return idx;
  }

  private static double[] gauss(double[][] a, int m) {
    for (int col = 0; col < m; col++) {
      int pivot = col;
      for (int r = col + 1; r < m; r++) if (Math.abs(a[r][col]) > Math.abs(a[pivot][col])) pivot = r;
      double[] tmp = a[col];
      a[col] = a[pivot];
      a[pivot] = tmp;
      if (Math.abs(a[col][col]) < 1e-300) continue;
      for (int r = 0; r < m; r++) {
        if (r == col) continue;
        double f = a[r][col] / a[col][col];
        for (int k = col; k <= m; k++) a[r][k] -= f * a[col][k];
      }
    }
    double[] x = new double[m];
    for (int i = 0; i < m; i++) x[i] = Math.abs(a[i][i]) < 1e-300 ? 0 : a[i][m] / a[i][i];
    return x;
  }

  /** Standard deviations sqrt(sigma0^2 N^-1)_kk of the active parameters. */
  static double[] covarianceDiagonal(double[][] normal, boolean[] active, double sigma0) {
    int[] idx = activeIndex(active);
    int m = idx.length;
    double[] out = new double[7];
    for (int k = 0; k < m; k++) {
      double[][] a = new double[m][m + 1];
      for (int i = 0; i < m; i++) {
        for (int j = 0; j < m; j++) a[i][j] = normal[idx[i]][idx[j]];
        a[i][i] += 1e-12 * Math.max(1, normal[idx[i]][idx[i]]);
        a[i][m] = i == k ? 1 : 0;
      }
      double[] col = gauss(a, m);
      out[idx[k]] = sigma0 * Math.sqrt(Math.max(0, col[k]));
    }
    return out;
  }

  /** Spatial groups for the jackknife: 4 equal-count slabs along easting, each cut into 4 equal-count parts along northing. */
  static final int GROUP_SLABS = 4;

  /**
   * Delete-a-group jackknife standard deviations over 16 spatially compact groups of equal size
   * (tie points on the lidar in the tie-point area; changed patches stay out). Spatially
   * correlated errors (canopy, terrain model, block deformation) show up here, not in the formal
   * covariance. The group estimates are kept in the report.
   */
  private static double[] jackknife(Points p, Surfaces s, Level base, Transform t, boolean[] active, Settings set, boolean[] changed, Area area, Fit fit) {
    List<Integer> members = new ArrayList<>();
    Grid ag = area.grid();
    for (int i = 0; i < p.size(); i++) {
      if (changed[i] || fit.category == null || fit.category[i] == SKIP) continue;
      int c = ag.col(p.x()[i]), r = ag.row(p.y()[i]);
      if (ag.inside(c, r) && area.core()[r * ag.w + c]) members.add(i);
    }
    int[] group = new int[p.size()];
    Arrays.fill(group, -1);
    members.sort(Comparator.comparingDouble(i -> p.x()[i]));
    int n = members.size();
    for (int sl = 0; sl < GROUP_SLABS; sl++) {
      List<Integer> slab = new ArrayList<>(members.subList(sl * n / GROUP_SLABS, (sl + 1) * n / GROUP_SLABS));
      slab.sort(Comparator.comparingDouble(i -> p.y()[i]));
      int m = slab.size();
      for (int part = 0; part < GROUP_SLABS; part++)
        for (int j = part * m / GROUP_SLABS; j < (part + 1) * m / GROUP_SLABS; j++) group[slab.get(j)] = sl * GROUP_SLABS + part;
    }
    List<double[]> estimates = new ArrayList<>();
    for (int g = 0; g < GROUP_SLABS * GROUP_SLABS; g++) {
      boolean[] out = changed.clone();
      int left = 0;
      double sx = 0, sy = 0;
      for (int i = 0; i < p.size(); i++)
        if (group[i] == g) {
          out[i] = true;
          left++;
          sx += p.x()[i];
          sy += p.y()[i];
        }
      if (left < 5) continue;
      Transform tj = t.copy();
      try {
        refine(p, s, base, tj, active, set, set.outlier(), out, 6, null);
      } catch (IllegalArgumentException e) {
        continue;
      }
      double[] v = tj.values();
      estimates.add(v);
      fit.groups.add(Json.object("group", g, "centre", new double[] {sx / left, sy / left}, "tie_points", left,
          "leave_out_estimate", Json.object("shift_east", v[0], "shift_north", v[1], "shift_up", v[2], "roll_degrees", Math.toDegrees(v[3]),
              "pitch_degrees", Math.toDegrees(v[4]), "heading_degrees", Math.toDegrees(v[5]), "scale_ppm", v[6] * 1e6)));
    }
    double[] sd = new double[7];
    int k = estimates.size();
    if (k < 3) return sd;
    for (int j = 0; j < 7; j++) {
      double mean = 0;
      for (double[] e : estimates) mean += e[j] / k;
      double ss = 0;
      for (double[] e : estimates) ss += (e[j] - mean) * (e[j] - mean);
      sd[j] = Math.sqrt((k - 1.0) / k * ss);
    }
    return sd;
  }

  /** Marks tie points in 20 m patches whose median residual departs from the block's; true if any new. */
  private static boolean markChanged(Points p, Surfaces s, Level base, Transform t, Settings set, boolean[] changed, Fit fit) {
    Fit f = new Fit();
    evaluate(p, s, base, t, new boolean[7], set, Double.POSITIVE_INFINITY, new boolean[p.size()], null, null, null, f);
    Map<Long, List<double[]>> patches = new HashMap<>();
    for (int i = 0; i < p.size(); i++) {
      if (f.category[i] != STABLE && f.category[i] != GAP) continue; // ground and roofs: canopy growth is free per patch anyway
      double[] q = t.apply(p.x()[i], p.y()[i], p.z()[i]);
      long key = (long) Math.floor(q[0] / 20) << 32 ^ (Math.floorMod((long) Math.floor(q[1] / 20), 1L << 32));
      double w = 1;
      patches.computeIfAbsent(key, k -> new ArrayList<>()).add(new double[] {f.residual[i], w, i});
    }
    List<double[]> medians = new ArrayList<>();
    Map<Long, Double> med = new HashMap<>();
    for (var e : patches.entrySet()) {
      if (e.getValue().size() < 3) continue;
      double m = weightedMedian(e.getValue());
      med.put(e.getKey(), m);
      medians.add(new double[] {m, 1});
    }
    if (medians.size() < 4) return false;
    double global = weightedMedian(medians);
    List<Double> dev = new ArrayList<>();
    for (double[] m : medians) dev.add(m[0] - global);
    double spread = Stats.of(dev).nmad();
    double limit = Math.max(set.changeLimit(), 3 * spread);
    boolean any = false;
    int flagged = 0;
    for (var e : med.entrySet()) {
      if (Math.abs(e.getValue() - global) <= limit) continue;
      flagged++;
      for (double[] o : patches.get(e.getKey())) {
        int i = (int) o[2];
        if (!changed[i]) {
          changed[i] = true;
          any = true;
        }
      }
    }
    fit.changedPatches = flagged;
    fit.patches = med.size();
    return any;
  }

  // ---- Fallback to fewer parameters.

  /** Parameter groups that are dropped together: shifts east/north, heading, tilts, scale. */
  static final int[][] GROUPS = {{0, 1}, {5}, {3, 4}, {6}};
  static final String[] GROUP_NAMES = {"horizontal shift", "heading", "roll and pitch", "scale"};

  record Outcome(Fit fit, List<String> fixed, List<Fit> attempts) {}

  /**
   * Fits all requested parameters; while a group is weakly determined (ratio above 1), the weakest
   * group is fixed at its GPS value and the fit repeated. The height shift is always estimated; if
   * it is weak the registration fails.
   */
  static Outcome estimate(Points p, Surfaces s, Area area, Settings set, Consumer<String> log) {
    boolean[] active = {true, true, true, true, true, true, set.scale()};
    List<String> fixed = new ArrayList<>();
    List<Fit> attempts = new ArrayList<>();
    if (!set.scale()) fixed.add("scale: not requested (--lidar-scale off)");
    while (true) {
      Fit fit = register(p, s, area, set, active, log);
      attempts.add(fit);
      if (log != null) log.accept(attemptText(fit, area.radius()));
      // A group is dropped only when it is weak AND none of its parameters differs significantly
      // (3 sigma) from the GPS value: fixing a clearly non-zero parameter would be worse than its
      // uncertain estimate (that case is kept and reported as a warning instead).
      // The bounded search: a horizontal shift or heading that ends outside its search range (plus
      // one cell / one degree) was not found by the search but drifted there; it is not trusted.
      // Horizontal shift and heading need horizontal evidence: a distinct correlation peak.
      if (fit.search.ambiguous() && (active[0] || active[5])) {
        for (int g : new int[] {0, 1}) for (int k : GROUPS[g]) active[k] = false;
        String why = String.format(Locale.ROOT, "horizontal shift and heading: the coarse search found no distinct horizontal match (contrast %.1f,"
            + " below 4: too little texture); kept at the GPS values", fit.search.contrast());
        fixed.add(why);
        if (log != null) log.accept("Lidar referencing: " + why);
        continue;
      }
      double[] v = fit.transform.values();
      boolean shiftOut = active[0] && Math.hypot(v[0], v[1]) > set.searchShift() + 2 * set.cell();
      boolean headingOut = active[5] && Math.abs(Math.toDegrees(v[5])) > set.searchAngle() + 1;
      if (shiftOut || headingOut) {
        int g = shiftOut ? 0 : 1;
        for (int k : GROUPS[g]) active[k] = false;
        String why = String.format(Locale.ROOT, "%s: the fit ended outside the search range (%s), so it is not trusted; kept at the GPS value",
            GROUP_NAMES[g], shiftOut ? String.format(Locale.ROOT, "%.1f m > %.1f m", Math.hypot(v[0], v[1]), set.searchShift())
                : String.format(Locale.ROOT, "%.2f deg > %.1f deg", Math.toDegrees(v[5]), set.searchAngle()));
        fixed.add(why);
        if (log != null) log.accept("Lidar referencing: " + why);
        continue;
      }
      int worst = -1;
      double worstRatio = 1;
      for (int g = 0; g < GROUPS.length; g++) {
        double ratio = 0;
        boolean on = false, significant = false;
        for (int k : GROUPS[g]) {
          ratio = Math.max(ratio, fit.weakRatio[k]);
          on |= active[k];
          significant |= active[k] && significant(fit, k);
        }
        if (on && !significant && ratio > worstRatio) {
          worst = g;
          worstRatio = ratio;
        }
      }
      if (worst < 0) {
        // The height shift is always estimated; only a height uncertain beyond 4x the limit fails.
        if (fit.weakRatio[2] > 4)
          throw new IllegalArgumentException(String.format(Locale.ROOT,
              "The lidar registration does not determine even the height shift (standard deviation %.2f m > 4 x --lidar-weak-limit %.2f m"
                  + " with only %s estimated; %d tie points used). The block may lie mostly outside the lidar or over changed areas.",
              fit.sigma[2], set.weakLimit(), String.join(", ", estimatedNames(active)), fit.used));
        return new Outcome(fit, fixed, attempts);
      }
      for (int k : GROUPS[worst]) active[k] = false;
      String why = String.format(Locale.ROOT, "%s: weakly determined (standard deviation %s, %.1fx the limit); kept at the GPS value",
          GROUP_NAMES[worst], sigmaText(fit, GROUPS[worst], area.radius()), worstRatio);
      fixed.add(why);
      if (log != null) log.accept("Lidar referencing: " + why);
    }
  }

  static String groupName(int k) {
    for (int g = 0; g < GROUPS.length; g++) for (int m : GROUPS[g]) if (m == k) return GROUP_NAMES[g];
    return PARAMETERS[k];
  }

  /** |value| above three standard deviations: clearly different from the GPS solution. */
  static boolean significant(Fit fit, int k) {
    double v = fit.transform.values()[k];
    return fit.sigma[k] > 0 && Math.abs(v) > 3 * fit.sigma[k];
  }

  /** One line per fit attempt: value +- sigma (formal / jackknife) of each estimated parameter. */
  static String attemptText(Fit fit, double radius) {
    double[] v = fit.transform.values();
    StringBuilder b = new StringBuilder("Lidar fit (" + fit.used + " points, sigma0 " + String.format(Locale.ROOT, "%.3f m", fit.sigma0) + "):");
    for (int k = 0; k < 7; k++) {
      if (!fit.active[k]) continue;
      double f = k == 6 ? 1e6 : k >= 3 ? Math.toDegrees(1) : 1;
      b.append(String.format(Locale.ROOT, " %s %+.4f +- %.4f (%.4f/%.4f, weak %.1f)", PARAMETERS[k], v[k] * f, fit.sigma[k] * f,
          fit.sigmaFormal[k] * f, fit.sigmaJackknife[k] * f, fit.weakRatio[k]));
    }
    return b.toString();
  }

  private static String sigmaText(Fit fit, int[] group, double radius) {
    StringBuilder b = new StringBuilder();
    for (int k : group) {
      if (b.length() > 0) b.append(", ");
      if (k <= 2) b.append(String.format(Locale.ROOT, "%.3f m", fit.sigma[k]));
      else if (k == 6) b.append(String.format(Locale.ROOT, "%.0f ppm = %.3f m at %.0f m", fit.sigma[k] * 1e6, fit.sigma[k] * radius, radius));
      else b.append(String.format(Locale.ROOT, "%.4f deg = %.3f m at %.0f m", Math.toDegrees(fit.sigma[k]), fit.sigma[k] * radius, radius));
    }
    return b.toString();
  }

  // ---- Report pieces.

  /** Residual statistics of the tie points against the lidar with transform t at the base level. */
  static Map<String, Object> residuals(Points p, Surfaces s, Transform t, Settings set, boolean[] changed) {
    Level base = Level.of(s, 1);
    Fit f = new Fit();
    evaluate(p, s, base, t, new boolean[7], set, Double.POSITIVE_INFINITY, new boolean[p.size()], null, null, null, f);
    List<Double> stable = new ArrayList<>(), all = new ArrayList<>(), canopy = new ArrayList<>(), stableUsed = new ArrayList<>(), allUsed = new ArrayList<>();
    for (int i = 0; i < p.size(); i++) {
      if (f.category[i] == SKIP || f.category[i] >= REGROWTH) continue;
      double r = f.residual[i];
      all.add(r);
      if (f.category[i] == CANOPY_OBS) canopy.add(r);
      else stable.add(r);
      if (changed != null && changed[i] || Math.abs(r) > set.outlier()) continue;
      allUsed.add(r);
      if (f.category[i] != CANOPY_OBS) stableUsed.add(r);
    }
    return Json.object(
        "stable_all", Stats.of(stable).json(),
        "canopy_all", Stats.of(canopy).json(),
        "total_all", Stats.of(all).json(),
        "stable_within_limit", Stats.of(stableUsed).json(),
        "total_within_limit", Stats.of(allUsed).json(),
        "note", "Surface-normal distance of tie points to the lidar surface (positive: tie point above). 'within_limit' leaves out"
            + " changed patches and residuals beyond --lidar-outlier.");
  }

  /**
   * Block deformation left after the rigid fit: residuals of the stable points (open ground,
   * ground under canopy, roofs; within --lidar-outlier, outside changed patches) on a regular
   * grid, and a quadratic surface a + bx + cy + dx^2 + exy + fy^2 fitted to them (Huber weights).
   * The quadratic part's range over the tie-point area is the bending a rigid transformation
   * cannot remove (a bundle adjustment with the lidar surface as a constraint could).
   */
  static Map<String, Object> deformation(Points p, Fit fit, Area area, Settings set) {
    List<double[]> obs = new ArrayList<>();
    double[] c = area.centre();
    for (int i = 0; i < p.size(); i++) {
      if (fit.category[i] == SKIP || fit.category[i] == CANOPY_OBS || fit.category[i] >= REGROWTH || fit.changed[i] || Math.abs(fit.residual[i]) > set.outlier()) continue;
      double[] q = fit.transform.apply(p.x()[i], p.y()[i], p.z()[i]);
      obs.add(new double[] {q[0] - c[0], q[1] - c[1], fit.residual[i]});
    }
    Map<String, Object> out = new LinkedHashMap<>();
    out.put("stable_points", obs.size());
    if (obs.size() < 30) {
      out.put("status", "too few stable points");
      return out;
    }
    double r = area.radius();
    double[] coef = new double[6];
    double[] w = new double[obs.size()];
    Arrays.fill(w, 1);
    for (int it = 0; it < 8; it++) {
      double[][] n = new double[6][7];
      for (int i = 0; i < obs.size(); i++) {
        double x = obs.get(i)[0] / r, y = obs.get(i)[1] / r;
        double[] f = {1, x, y, x * x, x * y, y * y};
        for (int a = 0; a < 6; a++) {
          for (int b = 0; b < 6; b++) n[a][b] += w[i] * f[a] * f[b];
          n[a][6] += w[i] * f[a] * obs.get(i)[2];
        }
      }
      coef = gauss(n, 6);
      List<Double> res = new ArrayList<>();
      for (double[] o : obs) res.add(o[2] - quadratic(coef, o[0] / r, o[1] / r));
      double k = 1.345 * Math.max(.02, Stats.of(res).nmad());
      for (int i = 0; i < obs.size(); i++) w[i] = Math.abs(res.get(i)) <= k ? 1 : k / Math.abs(res.get(i));
    }
    // Regular residual grid (cells of about a fifth of the block diameter).
    double cell = Math.max(20, Math.round(2 * r / 5 / 5) * 5.0);
    Map<String, List<Double>> cells = new TreeMap<>();
    for (double[] o : obs) cells.computeIfAbsent((int) Math.floor(o[0] / cell) + "," + (int) Math.floor(o[1] / cell), k -> new ArrayList<>()).add(o[2]);
    List<Object> grid = new ArrayList<>();
    // Ranges at the grid cells that hold data: of the cell medians and of the quadratic's curvature part.
    double lo = Double.MAX_VALUE, hi = -Double.MAX_VALUE, mlo = Double.MAX_VALUE, mhi = -Double.MAX_VALUE;
    for (var e : cells.entrySet()) {
      if (e.getValue().size() < 10) continue;
      String[] ij = e.getKey().split(",");
      Stats st = Stats.of(e.getValue());
      double x = (Integer.parseInt(ij[0]) + .5) * cell / r, y = (Integer.parseInt(ij[1]) + .5) * cell / r;
      double curved = coef[3] * x * x + coef[4] * x * y + coef[5] * y * y;
      lo = Math.min(lo, curved);
      hi = Math.max(hi, curved);
      mlo = Math.min(mlo, st.median());
      mhi = Math.max(mhi, st.median());
      grid.add(Json.object("east", c[0] + (Integer.parseInt(ij[0]) + .5) * cell, "north", c[1] + (Integer.parseInt(ij[1]) + .5) * cell,
          "points", st.count(), "median_metres", st.median(), "nmad_metres", st.nmad()));
    }
    out.put("status", "measured");
    out.put("quadratic_coefficients_metres", coef);
    out.put("coordinates", "x = (easting - centre) / radius, y = (northing - centre) / radius, radius = " + String.format(Locale.ROOT, "%.1f m", r));
    out.put("curvature_range_metres", grid.isEmpty() ? null : hi - lo);
    out.put("cell_median_range_metres", grid.isEmpty() ? null : mhi - mlo);
    out.put("grid_cell_metres", cell);
    out.put("grid", grid);
    out.put("note", "curvature_range_metres is the height range of the quadratic (bowl / saddle) part of the stable residuals at the grid"
        + " cells with data (10 or more stable points) after the rigid fit; cell_median_range_metres the range of their median residuals."
        + " Both measure block bending that one rigid transformation cannot remove.");
    return out;
  }

  private static double quadratic(double[] k, double x, double y) {
    return k[0] + k[1] * x + k[2] * y + k[3] * x * x + k[4] * x * y + k[5] * y * y;
  }

  /** Coverage / kind map PNG of the area grid (green covered, blue gaps, red missing, grey other lidar). */
  static void coverageImage(Coverage c, Area a, Path file) throws IOException {
    Grid g = a.grid();
    BufferedImage im = new BufferedImage(g.w, g.h, BufferedImage.TYPE_INT_RGB);
    int[] colours = {0xffffff, 0x4caf50, 0x64b5f6, 0xe53935, 0xd0d0d0};
    for (int r = 0; r < g.h; r++)
      for (int col = 0; col < g.w; col++) {
        int p = r * g.w + col;
        int rgb = colours[c.map()[p]];
        if (a.core()[p] && c.map()[p] == 1) rgb = 0x2e7d32;
        im.setRGB(col, r, rgb);
      }
    ImageIO.write(im, "png", file.toFile());
  }

  // ---- The step.

  record Result(Map<String, Object> report, String summary, Georeference.Similarity similarity, Source source) {}

  /**
   * The whole lidar step on the loaded block: reads, checks, registers, applies to cameras and
   * tie points and writes FOLDER. Throws IllegalArgumentException with a clear message when the
   * lidar does not cover the block or cannot determine the transformation.
   */
  static Result run(List<ImageFrame> frames, List<FeatureTracks.Track> tracks, Options o, Crs crs, Path dir, Consumer<String> log) throws IOException {
    Files.createDirectories(dir);
    Settings set = Settings.of(o);
    Source src = sources(set.source(), crs, set);
    for (String w : src.warnings()) log.accept("Warning: " + w);
    List<double[]> pts = new ArrayList<>();
    for (FeatureTracks.Track t : tracks) if (t.point != null && t.prior == null && t.observations.size() >= 2) pts.add(t.point);
    Points p = points(pts);
    Area area = area(p, set.margin());
    long t0 = System.nanoTime();
    Surfaces s = read(src, area, set.cell(), log);
    Coverage coverage = coverage(area, s, src, set.margin());
    coverageImage(coverage, area, dir.resolve("coverage.png"));
    Map<String, Object> report = new LinkedHashMap<>();
    report.put("status", "running");
    report.put("settings", Json.object(
        "source", set.source().toAbsolutePath().toString(), "estimated", set.scale() ? "rotation, shift and scale" : "rotation and shift",
        "search_shift_metres", set.searchShift(), "search_heading_degrees", set.searchAngle(), "cell_metres", set.cell(),
        "canopy_weight", set.canopyWeight(), "outlier_metres", set.outlier(), "change_limit_metres", set.changeLimit(),
        "weak_limit_metres", set.weakLimit(), "margin_metres", set.margin()));
    List<Object> files = new ArrayList<>();
    for (LasFile.Header h : src.files())
      files.add(Json.object("file", h.path().getFileName().toString(), "las_version", h.version(), "point_format", h.format(),
          "compressed", h.compressed(), "points", h.count(), "extent", new double[] {h.min()[0], h.min()[1], h.max()[0], h.max()[1]},
          "horizontal_epsg", h.crs().horizontal() == 0 ? null : h.crs().horizontal(), "vertical_epsg", h.crs().vertical() == 0 ? null : h.crs().vertical(),
          "crs_source", h.crs().source(), "software", h.software()));
    report.put("lidar", Json.object("files", files, "horizontal_crs", src.crsNote(), "vertical_datum", src.vertical(),
        "vertical_datum_source", src.verticalSource(), "files_read", s.filesRead, "points_read", s.read, "points_in_area", s.used + s.noise + s.withheld,
        "points_used", s.used, "noise_points", s.noise, "withheld_points", s.withheld, "ground_classified", s.groundClassified,
        "read_seconds", s.seconds, "warnings", src.warnings()));
    report.put("area", Json.object("tie_points", p.size(), "tie_points_in_area", area.used(), "stray_tie_points", area.stray(),
        "tie_point_area_m2", area.coreM2(), "area_with_margin_m2", area.insideM2(), "centre", area.centre(), "radius_90_metres", area.radius(),
        "grid", area.grid().json()));
    report.put("coverage", Json.object("complete", coverage.complete(), "area_m2", coverage.areaM2(), "missing_m2", coverage.missingM2(),
        "missing_share", coverage.missingM2() / Math.max(1, coverage.areaM2()), "no_returns_inside_tiles_m2", coverage.gapM2(),
        "missing_extent", coverage.missingBox(), "missing_where", coverage.where(), "map", "coverage.png", "message", coverage.message()));
    if (!coverage.complete()) {
      report.put("status", "failed: incomplete lidar coverage");
      Json.write(dir.resolve("lidar_reference.json"), report);
      throw new IllegalArgumentException(coverage.message());
    }
    log.accept(coverage.message());
    // Lidar grids for the report (difference maps).
    writeGrids(s, dir, crs.epsg);
    int[] kindCount = new int[6];
    for (byte k : s.kind) kindCount[k]++;
    Map<String, Object> kinds = new LinkedHashMap<>();
    for (int k = 0; k < 6; k++) kinds.put(KIND_NAMES[k].replace(' ', '_') + "_share", kindCount[k] / (double) s.kind.length);
    report.put("surfaces", Json.object("grid", s.grid.json(), "kinds", kinds, "files", List.of("als_dsm.tif", "als_dtm.tif", "als_kind.tif", "als_grid.json")));
    // Registration.
    Outcome outcome = estimate(p, s, area, set, log);
    Fit fit = outcome.fit();
    Transform tr = fit.transform;
    double[] values = tr.values();
    List<Object> params = new ArrayList<>();
    List<String> weakWarnings = new ArrayList<>();
    Fit full = outcome.attempts().get(0);
    for (int k = 0; k < 7; k++) {
      boolean degrees = k >= 3 && k <= 5;
      double value = k == 6 ? values[k] * 1e6 : degrees ? Math.toDegrees(values[k]) : values[k];
      double sd = k == 6 ? fit.sigma[k] * 1e6 : degrees ? Math.toDegrees(fit.sigma[k]) : fit.sigma[k];
      boolean on = fit.active[k];
      Double z = on && fit.sigma[k] > 0 ? Math.abs(values[k]) / fit.sigma[k] : null;
      params.add(Json.object(
          "name", PARAMETERS[k], "unit", k == 6 ? "ppm" : degrees ? "degrees" : "metres", "estimated", on,
          "value", on ? value : 0.0, "sigma", on ? sd : null,
          "sigma_formal", on ? (k == 6 ? fit.sigmaFormal[k] * 1e6 : degrees ? Math.toDegrees(fit.sigmaFormal[k]) : fit.sigmaFormal[k]) : null,
          "sigma_jackknife", on ? (k == 6 ? fit.sigmaJackknife[k] * 1e6 : degrees ? Math.toDegrees(fit.sigmaJackknife[k]) : fit.sigmaJackknife[k]) : null,
          "effect_at_edge_metres", on ? (k <= 2 ? fit.sigma[k] : fit.sigma[k] * area.radius()) : null,
          "significant", z == null ? null : z > 3, "value_over_sigma", z,
          "full_fit_value", full.active[k] ? (k == 6 ? full.transform.values()[k] * 1e6 : degrees ? Math.toDegrees(full.transform.values()[k]) : full.transform.values()[k]) : null,
          "full_fit_weak_ratio", full.active[k] ? full.weakRatio[k] : null));
    }
    for (String f : outcome.fixed()) if (!f.startsWith("scale: not requested")) weakWarnings.add(f);
    for (int k = 0; k < 7; k++)
      if (fit.active[k] && fit.weakRatio[k] > 1)
        weakWarnings.add(String.format(Locale.ROOT, "%s: weakly determined (%s, %.1fx the limit) %s",
            PARAMETERS[k], sigmaText(fit, new int[] {k}, area.radius()), fit.weakRatio[k],
            significant(fit, k)
                ? "but clearly different from the GPS solution, so it is estimated, not fixed"
                : "and not significant; it is estimated because the other parameter of its group (" + groupName(k) + ") is clearly different from"
                    + " the GPS solution"));
    double before = GpsAdjustment.reprojectionRms(frames, tracks, null);
    Georeference.Similarity sim = tr.similarity();
    for (ImageFrame f : frames) {
      Camera c = f.camera;
      c.rotation = Math3.mul(c.rotation, Math3.transpose(sim.rotation));
      c.translation = Math3.sub(Math3.times(c.translation, sim.scale), Math3.mv(c.rotation, sim.translation));
    }
    for (FeatureTracks.Track t : tracks) if (t.point != null) t.point = sim.apply(t.point);
    double after = GpsAdjustment.reprojectionRms(frames, tracks, null);
    if (Math.abs(after - before) > 1e-6 * Math.max(1, before))
      throw new IllegalStateException(String.format(Locale.ROOT, "Lidar transformation changed the reprojection RMS (%.9f -> %.9f px)", before, after));
    // Residuals before (GPS) and after, on the moved tie points.
    Transform identity = new Transform(area.centre());
    Map<String, Object> resBefore = residuals(p, s, identity, set, null);
    Map<String, Object> resAfter = residuals(p, s, tr, set, fit.changed);
    int changedPoints = 0, gap = 0, stable = 0, canopy = 0, regrowth = 0, understorey = fit.belowCrownTops;
    for (int i = 0; i < p.size(); i++) {
      if (fit.changed[i]) changedPoints++;
      if (fit.category[i] == STABLE) stable++;
      else if (fit.category[i] == GAP) gap++;
      else if (fit.category[i] == CANOPY_OBS) canopy++;
      else if (fit.category[i] == REGROWTH) regrowth++;
    }
    double stableWeight = 0, totalWeight = 0;
    for (int i = 0; i < p.size(); i++) {
      totalWeight += fit.weight[i];
      if (fit.category[i] != CANOPY_OBS) stableWeight += fit.weight[i];
    }
    report.put("parameters", params);
    report.put("rotation_centre", area.centre());
    report.put("convention", "X_lidar = c + (1 + scale) R (X_gps - c) + shift, R = Rz(heading) Ry(pitch) Rx(roll); roll about the"
        + " easting axis, pitch about the northing axis, heading about the vertical axis, counter-clockwise positive seen from"
        + " the positive axis; c = rotation_centre (block centre).");
    report.put("estimated", estimatedNames(fit.active));
    report.put("fixed", outcome.fixed());
    report.put("fallback", !weakWarnings.isEmpty());
    report.put("warnings", weakWarnings);
    report.put("coarse_search", Json.object("shift_east", fit.searchBest[0], "shift_north", fit.searchBest[1], "shift_up", fit.searchBest[2],
        "heading_degrees", Math.toDegrees(fit.searchBest[3]), "at_search_edge", fit.atSearchEdge,
        "correlation", fit.search.ncc(), "background_correlation", fit.search.background(), "contrast", fit.search.contrast(),
        "overlap_cells", fit.search.overlap(), "ambiguous", fit.search.ambiguous(),
        "method", "normalised cross-correlation of the high-passed tie-point top surface and lidar DSM (cell minus its 11 x 11 cell"
            + " mean), whole-cell shifts and 0.5 degree headings, sub-cell peak; height by a weighted median"));
    if (fit.search.ambiguous())
      weakWarnings.add(String.format(Locale.ROOT, "The coarse search peak is weak (contrast %.1f, below 4): the surface has little texture or the"
          + " block has changed since the lidar; check the residual map.", fit.search.contrast()));
    report.put("fit", Json.object("iterations", fit.iterations, "sigma0_metres", fit.sigma0, "tie_points_on_lidar", fit.observations,
        "used", fit.used, "used_share", fit.used / (double) Math.max(1, fit.observations), "outliers", fit.outliers,
        "outlier_share", fit.outliers / (double) Math.max(1, fit.observations), "changed_points", changedPoints,
        "changed_patches", fit.changedPatches, "patches", fit.patches, "stable_points", stable, "ground_under_canopy_points", gap,
        "canopy_points", canopy, "below_crown_top_points", understorey, "vegetation_on_lidar_open_ground_points", regrowth, "stable_weight_share", stableWeight / Math.max(1e-12, totalWeight)));
    report.put("change_since_lidar", changes(frames, s, fit, gap, regrowth, changedPoints, weakWarnings));
    report.put("jackknife_groups", fit.groups);
    report.put("deformation", deformation(p, fit, area, set));
    report.put("residuals_before", resBefore);
    report.put("residuals_after", resAfter);
    report.put("reprojection_rms_pixels", Json.object("before", before, "after", after,
        "note", "A rigid or similarity transformation must leave reprojection errors unchanged; checked."));
    report.put("similarity", Json.object("scale", sim.scale, "R_row_major", sim.rotation, "translation", sim.translation,
        "operation", "X_lidar = scale * R * X_gps + translation"));
    report.put("height_datum", src.vertical());
    if (fit.atSearchEdge) weakWarnings.add("The coarse search ended at the edge of its range (--lidar-search-shift / --lidar-search-angle); widen it if the GPS may be further off.");
    report.put("status", "applied");
    report.put("seconds", (System.nanoTime() - t0) / 1e9);
    writeResiduals(dir.resolve("residuals.csv"), p, fit);
    Json.write(dir.resolve("lidar_reference.json"), report);
    String summary = summary(values, fit, outcome, resAfter);
    log.accept(summary);
    return new Result(report, summary, sim, src);
  }

  /** Capture date of the images (median of the EXIF dates of up to 9 images), or null. */
  static java.time.LocalDate imageDate(List<ImageFrame> frames) {
    List<java.time.LocalDate> dates = new ArrayList<>();
    for (int i = 0; i < frames.size() && dates.size() < 9; i += Math.max(1, frames.size() / 9))
      try {
        double sec = Exif.read(frames.get(i).path).captureSeconds();
        if (Double.isFinite(sec)) dates.add(java.time.LocalDate.ofEpochDay((long) Math.floor(sec / 86400)));
      } catch (IOException | RuntimeException e) {
        // no date in this image
      }
    if (dates.isEmpty()) return null;
    Collections.sort(dates);
    return dates.get(dates.size() / 2);
  }

  /**
   * What changed between the lidar and the images: dates and age, the canopy height change per
   * 20 m patch (the free canopy offsets: photogrammetric crowns minus the lidar's highest point per
   * cell, so growth plus a sampling bias of a few decimetres), ground where the lidar had canopy
   * (gaps or felling), vegetation on the lidar's open ground (regrowth), changed patches.
   */
  static Map<String, Object> changes(List<ImageFrame> frames, Surfaces s, Fit fit, int groundUnderCanopy, int regrowth, int changedPoints, List<String> warnings) {
    java.time.LocalDate images = imageDate(frames);
    Double age = images != null && s.acquired != null ? (images.toEpochDay() - s.acquired.toEpochDay()) / 365.25 : null;
    List<Double> offsets = new ArrayList<>(fit.canopyOffsets);
    Collections.sort(offsets);
    Map<String, Object> out = new LinkedHashMap<>();
    out.put("lidar_acquired", s.acquired == null ? null : s.acquired.toString());
    out.put("lidar_acquired_first", s.acquiredFirst == null ? null : s.acquiredFirst.toString());
    out.put("lidar_acquired_last", s.acquiredLast == null ? null : s.acquiredLast.toString());
    out.put("lidar_date_source", s.acquired == null ? "unknown (the points carry GPS week time, not a date)" : "GPS time of the lidar points");
    out.put("images_taken", images == null ? null : images.toString());
    out.put("lidar_age_years", age);
    out.put("canopy_patches", offsets.size());
    out.put("canopy_height_change_metres", offsets.isEmpty() ? null : Json.object(
        "median", offsets.get(offsets.size() / 2), "p10", offsets.get((int) (.1 * (offsets.size() - 1))), "p90", offsets.get((int) Math.ceil(.9 * (offsets.size() - 1)))));
    out.put("ground_under_lidar_canopy_points", groundUnderCanopy);
    out.put("vegetation_on_lidar_open_ground_points", regrowth);
    out.put("changed_patch_points", changedPoints);
    out.put("handling", "Canopy height is free per 20 m patch (only crown shape and position count); ground counts only where the tie"
        + " point is also locally lowest in the photogrammetric cloud; ground and roof patches that depart from the block are left out;"
        + " the coarse search clips structure differences at 2.5 m.");
    out.put("note", "canopy_height_change is the photogrammetric crown surface minus the lidar's highest point per cell, per 20 m"
        + " patch: growth (or thinning when negative) plus a sampling difference of a few decimetres; not a forest inventory measure.");
    if (age != null && age >= 3)
      warnings.add(String.format(Locale.ROOT, "The lidar (%s) is %.1f years older than the images (%s): canopy heights were not used for"
          + " the height; ground and roofs carry it (canopy changed by %s m per patch, median).", s.acquired, age, images,
          offsets.isEmpty() ? "?" : String.format(Locale.ROOT, "%+.1f", offsets.get(offsets.size() / 2))));
    return out;
  }

  static List<String> estimatedNames(boolean[] active) {
    List<String> out = new ArrayList<>();
    for (int k = 0; k < 7; k++) if (active[k]) out.add(PARAMETERS[k]);
    return out;
  }

  static String summary(double[] v, Fit fit, Outcome outcome, Map<String, Object> after) {
    Object stable = JsonRead.object(after.get("stable_within_limit")).get("rms_metres");
    return String.format(Locale.ROOT,
        "Lidar referencing: shift E %+.2f N %+.2f U %+.2f m, roll %+.3f pitch %+.3f heading %+.3f deg%s; %d of %d tie points used"
            + " (%.0f%% outliers), stable-area RMS after %.3f m%s",
        v[0], v[1], v[2], Math.toDegrees(v[3]), Math.toDegrees(v[4]), Math.toDegrees(v[5]),
        fit.active[6] ? String.format(Locale.ROOT, ", scale %+.0f ppm", v[6] * 1e6) : "", fit.used, fit.observations,
        100.0 * fit.outliers / Math.max(1, fit.observations), stable instanceof Number n ? n.doubleValue() : Double.NaN,
        outcome.fixed().stream().anyMatch(f -> !f.startsWith("scale: not requested")) ? "; fewer parameters: " + String.join("; ", outcome.fixed()) : "");
  }

  static Points points(List<double[]> pts) {
    double[] x = new double[pts.size()], y = new double[pts.size()], z = new double[pts.size()];
    for (int i = 0; i < pts.size(); i++) {
      x[i] = pts.get(i)[0];
      y[i] = pts.get(i)[1];
      z[i] = pts.get(i)[2];
    }
    return new Points(x, y, z);
  }

  private static void writeResiduals(Path file, Points p, Fit fit) throws IOException {
    try (BufferedWriter w = Files.newBufferedWriter(file)) {
      w.write("x_gps,y_gps,z_gps,category,residual_after_metres,weight,changed_patch\n");
      for (int i = 0; i < p.size(); i++) {
        if (fit.category[i] == SKIP) continue;
        w.write(String.format(Locale.ROOT, "%.3f,%.3f,%.3f,%s,%.4f,%.4f,%b%n", p.x()[i], p.y()[i], p.z()[i], OBS_NAMES[fit.category[i]].replace(' ', '_'),
            fit.residual[i], fit.weight[i], fit.changed[i]));
      }
    }
  }

  /** ALS DSM, DTM and kinds as float GeoTIFFs plus their grid description. */
  static void writeGrids(Surfaces s, Path dir, int epsg) throws IOException {
    Grid g = s.grid;
    DsmRaster.writeTiff(dir.resolve("als_dsm.tif"), g.w, g.h, s.dsm, g.x0, g.y0, g.cell, epsg);
    DsmRaster.writeTiff(dir.resolve("als_dtm.tif"), g.w, g.h, s.dtm, g.x0, g.y0, g.cell, epsg);
    float[] kind = new float[s.kind.length];
    for (int i = 0; i < kind.length; i++) kind[i] = s.kind[i];
    DsmRaster.writeTiff(dir.resolve("als_kind.tif"), g.w, g.h, kind, g.x0, g.y0, g.cell, epsg);
    Map<String, Object> grid = new LinkedHashMap<>(g.json());
    grid.put("epsg", epsg);
    grid.put("kinds", Arrays.asList(KIND_NAMES));
    Json.write(dir.resolve("als_grid.json"), grid);
  }
}
