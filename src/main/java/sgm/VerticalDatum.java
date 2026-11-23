package sgm;

import java.io.*;
import java.nio.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.*;

/**
 * Camera GPS height conversion between vertical datums (--source-geoid, --target-geoid).
 *
 * <p>Each side is either {@code ellipsoid} (heights above the WGS84/ETRS89 ellipsoid) or a geoid
 * grid file giving the geoid height N above the ellipsoid. A camera height H_source becomes
 * h = H_source + N_source(lat, lon), then H_target = h - N_target(lat, lon). A side that is not
 * given is the ellipsoid; with neither option the EXIF heights are used unchanged.
 *
 * <p>Grid files: NOAA/PROJ .gtx (binary), or text with one "latitude longitude N" node per line
 * (degrees, metres; spaces, commas or semicolons; non-numeric lines are skipped) on a regular
 * latitude/longitude grid. Heights are interpolated bilinearly.
 */
final class VerticalDatum {
  static final String ELLIPSOID = "ellipsoid";

  private VerticalDatum() {}

  /** Normalised option value: "ellipsoid" or the grid path as given. */
  static String normalize(String value, String option) {
    String v = value.trim();
    if (v.isEmpty()) throw new IllegalArgumentException(option + " needs ellipsoid or a geoid grid file");
    String lower = v.toLowerCase(Locale.ROOT);
    if (lower.equals("ellipsoid") || lower.equals("none") || lower.equals("ellipsoidal")) return ELLIPSOID;
    if (!Files.isRegularFile(Paths.get(v)))
      throw new IllegalArgumentException(option + ": geoid grid file not found: " + v);
    return v;
  }

  static boolean active(Options o) {
    return o.sourceGeoid != null || o.targetGeoid != null;
  }

  static String source(Options o) {
    return o.sourceGeoid == null ? ELLIPSOID : o.sourceGeoid;
  }

  static String target(Options o) {
    return o.targetGeoid == null ? ELLIPSOID : o.targetGeoid;
  }

  /** Height added to a source height at (lat, lon): N_source - N_target, metres. */
  static double shift(double latitude, double longitude, Options o) throws IOException {
    return undulation(source(o), latitude, longitude, "--source-geoid")
        - undulation(target(o), latitude, longitude, "--target-geoid");
  }

  /** The GPS position with its height converted from the source to the target datum. */
  static Exif.Gps convert(Exif.Gps gps, Options o) throws IOException {
    if (gps == null || !active(o) || !Double.isFinite(gps.latitude + gps.longitude + gps.altitude))
      return gps;
    return new Exif.Gps(
        gps.latitude, gps.longitude, gps.altitude + shift(gps.latitude, gps.longitude, o), gps.datum);
  }

  /** Whether imported exteriors are converted: projected (--orientation-crs) poses with the options. */
  static boolean convertsPoses(Options o) {
    return active(o) && o.orientationCrs != null;
  }

  /**
   * Moves an imported camera centre (in --orientation-crs, metres) by the datum shift at its
   * position; the rotation is kept. Local exteriors without a CRS are left alone (their heights
   * are not georeferenced; the EXIF GPS conversion covers them).
   */
  static void shiftPose(Camera camera, Options o) throws IOException {
    if (!convertsPoses(o)) return;
    double[] c = camera.center();
    double[] latLon = new Crs(o.orientationCrs).geographic(c[0], c[1]);
    double dz = shift(latLon[0], latLon[1], o);
    // t = -R C, so C + dz e3 gives t - dz R e3 (third column of R).
    double[] r = camera.rotation;
    camera.translation[0] -= dz * r[2];
    camera.translation[1] -= dz * r[5];
    camera.translation[2] -= dz * r[8];
  }

  /** Short label of one side: "ellipsoid" or the grid file name. */
  static String label(String side) {
    return side.equals(ELLIPSOID) ? "ellipsoidal heights" : "geoid " + Paths.get(side).getFileName();
  }

  /** The height statement for supplied projected exteriors converted by the options. */
  static String poseHeightNote(Options o) {
    return "Z is the supplied exterior heights converted from "
        + label(source(o))
        + " to "
        + label(target(o))
        + " (--source-geoid/--target-geoid), metres.";
  }

  /** The height statement for reports and coordinates.json. */
  static String heightNote(Options o) {
    int vertical = Crs.verticalOf(o.crs);
    if (!active(o) && vertical > 0)
      return "Z retains EXIF GPS altitude in metres, labelled "
          + Crs.verticalName(vertical)
          + " by --crs without a conversion. Horizontal and vertical CRS in the file tags.";
    if (!active(o)) return Georeference.HEIGHT_NOTE;
    if (vertical > 0)
      return "Z is EXIF GPS altitude converted from "
          + label(source(o))
          + " to "
          + label(target(o))
          + " (--source-geoid/--target-geoid), metres, labelled "
          + Crs.verticalName(vertical)
          + ". The conversion changes the datum only: GPS height errors remain.";
    return "Z is EXIF GPS altitude converted from "
        + label(source(o))
        + " to "
        + label(target(o))
        + " (--source-geoid/--target-geoid), metres. The conversion changes the datum only:"
        + " GPS height errors remain. Horizontal CRS only in the file tags.";
  }

  /** Short vertical datum name for coordinates.json, or null when no conversion was requested. */
  static String datumName(Options o) {
    if (!active(o)) return null;
    int vertical = Crs.verticalOf(o.crs);
    return vertical > 0 ? Crs.verticalName(vertical) : label(target(o));
  }

  private static double undulation(String side, double latitude, double longitude, String option)
      throws IOException {
    if (side.equals(ELLIPSOID)) return 0;
    Grid g = grid(Paths.get(side));
    double n = g.at(latitude, longitude);
    if (Double.isNaN(n)) {
      String hint = Double.isNaN(g.at(longitude, latitude)) ? "" : " (the file may list longitude before latitude; it must be latitude longitude N)";
      throw new IllegalArgumentException(
          String.format(
              Locale.ROOT,
              "%s: camera at latitude %.6f, longitude %.6f lies outside geoid grid %s (latitude %.4f..%.4f, longitude %.4f..%.4f)%s",
              option, latitude, longitude, side, g.lat0, g.lat0 + (g.rows - 1) * g.dlat,
              g.lon0, g.lon0 + (g.cols - 1) * g.dlon, hint));
    }
    return n;
  }

  private static final Map<Path, Grid> CACHE = new HashMap<>();

  static Grid grid(Path path) throws IOException {
    Path key = path.toAbsolutePath().normalize();
    synchronized (CACHE) {
      Grid g = CACHE.get(key);
      if (g == null) {
        g = path.getFileName().toString().toLowerCase(Locale.ROOT).endsWith(".gtx") ? gtx(path) : text(path);
        CACHE.put(key, g);
      }
      return g;
    }
  }

  /** Regular latitude/longitude grid of geoid heights; row 0 is the southernmost. */
  static final class Grid {
    final double lat0, lon0, dlat, dlon;
    final int rows, cols;
    final float[] n;

    Grid(double lat0, double lon0, double dlat, double dlon, int rows, int cols, float[] n) {
      if (rows < 2 || cols < 2 || !(dlat > 0) || !(dlon > 0))
        throw new IllegalArgumentException("geoid grid needs at least 2x2 nodes with positive spacing");
      this.lat0 = lat0;
      this.lon0 = lon0;
      this.dlat = dlat;
      this.dlon = dlon;
      this.rows = rows;
      this.cols = cols;
      this.n = n;
    }

    /** Bilinear geoid height, or NaN outside the grid or next to a missing node. */
    double at(double latitude, double longitude) {
      double r = (latitude - lat0) / dlat;
      if (!(r >= -1e-9 && r <= rows - 1 + 1e-9)) return Double.NaN;
      double c = Double.NaN;
      for (double shift : new double[] {0, 360, -360}) {
        double t = (longitude + shift - lon0) / dlon;
        if (t >= -1e-9 && t <= cols - 1 + 1e-9) {
          c = t;
          break;
        }
      }
      if (Double.isNaN(c)) return Double.NaN;
      int i = Math.min(rows - 2, Math.max(0, (int) Math.floor(r)));
      int j = Math.min(cols - 2, Math.max(0, (int) Math.floor(c)));
      double fr = Math.min(1, Math.max(0, r - i)), fc = Math.min(1, Math.max(0, c - j));
      double a = n[i * cols + j], b = n[i * cols + j + 1], d = n[(i + 1) * cols + j], e = n[(i + 1) * cols + j + 1];
      double v = (a * (1 - fc) + b * fc) * (1 - fr) + (d * (1 - fc) + e * fc) * fr;
      return Double.isFinite(v) ? v : Double.NaN;
    }
  }

  /** NOAA .gtx: big-endian lat0, lon0, dlat, dlon (doubles), rows, cols (ints), float32 rows from the south. */
  static Grid gtx(Path path) throws IOException {
    byte[] bytes = Files.readAllBytes(path);
    if (bytes.length < 40) throw new IllegalArgumentException("not a .gtx geoid grid: " + path);
    ByteBuffer b = ByteBuffer.wrap(bytes).order(ByteOrder.BIG_ENDIAN);
    double lat0 = b.getDouble(), lon0 = b.getDouble(), dlat = b.getDouble(), dlon = b.getDouble();
    int rows = b.getInt(), cols = b.getInt();
    if (rows < 2 || cols < 2 || 40L + 4L * rows * cols != bytes.length)
      throw new IllegalArgumentException("not a .gtx geoid grid (size does not match its header): " + path);
    float[] n = new float[rows * cols];
    for (int k = 0; k < n.length; k++) {
      float v = b.getFloat();
      // -88.8888 marks a missing node in .gtx files.
      n[k] = Math.abs(v + 88.8888f) < 1e-3f ? Float.NaN : v;
    }
    return new Grid(lat0, lon0, dlat, dlon, rows, cols, n);
  }

  /** Text grid: "latitude longitude N" per line on a regular grid; missing nodes stay NaN. */
  static Grid text(Path path) throws IOException {
    List<double[]> nodes = new ArrayList<>();
    try (BufferedReader r = Files.newBufferedReader(path, StandardCharsets.ISO_8859_1)) {
      for (String line; (line = r.readLine()) != null; ) {
        String[] parts = line.trim().split("[\\s,;]+");
        if (parts.length < 3) continue;
        try {
          double lat = Double.parseDouble(parts[0]), lon = Double.parseDouble(parts[1]), n = Double.parseDouble(parts[2]);
          if (Double.isFinite(lat + lon + n)) nodes.add(new double[] {lat, lon, n});
        } catch (NumberFormatException header) {
          // header or comment line
        }
      }
    }
    if (nodes.size() < 4) throw new IllegalArgumentException("geoid grid has fewer than 4 nodes: " + path);
    double[] lats = axis(nodes, 0, path), lons = axis(nodes, 1, path);
    double lat0 = lats[0], dlat = lats[1], lon0 = lons[0], dlon = lons[1];
    int rows = (int) lats[2], cols = (int) lons[2];
    float[] n = new float[rows * cols];
    Arrays.fill(n, Float.NaN);
    for (double[] p : nodes) {
      int i = (int) Math.round((p[0] - lat0) / dlat), j = (int) Math.round((p[1] - lon0) / dlon);
      n[i * cols + j] = (float) p[2];
    }
    return new Grid(lat0, lon0, dlat, dlon, rows, cols, n);
  }

  /** {first, spacing, count} of one coordinate column; refuses irregular spacing. */
  private static double[] axis(List<double[]> nodes, int column, Path path) {
    TreeSet<Double> values = new TreeSet<>();
    for (double[] p : nodes) values.add(p[column]);
    if (values.size() < 2) throw new IllegalArgumentException("geoid grid needs at least 2 distinct " + (column == 0 ? "latitudes" : "longitudes") + ": " + path);
    double first = values.first(), step = Double.POSITIVE_INFINITY, previous = Double.NaN;
    for (double v : values) {
      if (!Double.isNaN(previous)) step = Math.min(step, v - previous);
      previous = v;
    }
    double count = Math.round((values.last() - first) / step) + 1;
    for (double v : values) {
      double k = (v - first) / step;
      if (Math.abs(k - Math.round(k)) > 1e-3)
        throw new IllegalArgumentException("geoid grid " + path + " is not regular in " + (column == 0 ? "latitude" : "longitude"));
    }
    return new double[] {first, step, count};
  }
}
