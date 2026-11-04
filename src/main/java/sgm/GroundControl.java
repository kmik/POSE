package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import org.opencv.core.KeyPoint;

/**
 * Ground control points (the gcp step and the marker editor's data).
 *
 * <p>GCP file (--gcp): CSV with {@code gcp_id, x, y, z} and optionally {@code sx, sy, sz} (metres,
 * one standard deviation per axis) and {@code type} ({@code control} or {@code check}); comma,
 * semicolon or tab separated, header optional. Coordinates are in the output CRS (--crs).
 *
 * <p>Markers ({@code RUN/04_geometry/gcp/markers.json}): one entry per GCP and image, in original
 * image pixels (distorted, as in the photograph), with status {@code predicted}, {@code pinned}
 * (placed by the user) or {@code rejected}. Predictions come from the cameras: from the GCP
 * coordinates when the block is georeferenced, from the pinned markers once a GCP has two, and
 * from a GCP similarity once three GCPs have two pins each in a local block.
 */
final class GroundControl {
  private GroundControl() {}

  static final String FORMAT = "msgm-gcp-markers-1";

  static final class Point3 {
    final String id;
    final double[] xyz, sigma;
    final boolean check;

    Point3(String id, double[] xyz, double[] sigma, boolean check) {
      this.id = id;
      this.xyz = xyz;
      this.sigma = sigma;
      this.check = check;
    }
  }

  static final class Marker {
    final String gcp;
    final int image;
    double u = Double.NaN, v = Double.NaN, predictedU = Double.NaN, predictedV = Double.NaN;
    String status = "predicted";

    Marker(String gcp, int image) {
      this.gcp = gcp;
      this.image = image;
    }

    boolean pinned() {
      return status.equals("pinned");
    }
  }

  static final class Markers {
    final List<Point3> gcps = new ArrayList<>();
    final List<Marker> markers = new ArrayList<>();
    String crs;

    Point3 gcp(String id) {
      for (Point3 p : gcps) if (p.id.equals(id)) return p;
      return null;
    }

    Marker marker(String gcp, int image) {
      for (Marker m : markers) if (m.gcp.equals(gcp) && m.image == image) return m;
      return null;
    }

    List<Marker> of(String gcp) {
      List<Marker> out = new ArrayList<>();
      for (Marker m : markers) if (m.gcp.equals(gcp)) out.add(m);
      return out;
    }

    int pins(String gcp) {
      int n = 0;
      for (Marker m : of(gcp)) if (m.pinned()) n++;
      return n;
    }
  }

  // ---- GCP file.

  static List<Point3> read(Path file, double horizontalSigma, double heightSigma) throws IOException {
    List<Point3> out = new ArrayList<>();
    int line = 0;
    Map<String, Integer> column = null;
    for (String raw : Files.readAllLines(file)) {
      line++;
      String text = raw.replace("﻿", "").trim();
      if (text.isEmpty() || text.startsWith("#")) continue;
      String[] f = text.split("\\s*[,;\\t]\\s*");
      if (column == null && !isNumber(f.length > 1 ? f[1] : "")) {
        column = new HashMap<>();
        for (int i = 0; i < f.length; i++) column.put(f[i].toLowerCase(Locale.ROOT).replace("\"", ""), i);
        continue;
      }
      if (column == null) {
        column = new HashMap<>();
        String[] names = {"gcp_id", "x", "y", "z", "sx", "sy", "sz", "type"};
        for (int i = 0; i < names.length; i++) column.put(names[i], i);
      }
      try {
        String id = field(f, column, "gcp_id", "id", "label", "name");
        double x = Double.parseDouble(field(f, column, "x", "e", "easting")),
            y = Double.parseDouble(field(f, column, "y", "n", "northing")),
            z = Double.parseDouble(field(f, column, "z", "h", "height", "elevation"));
        double sx = optional(f, column, "sx", horizontalSigma), sy = optional(f, column, "sy", horizontalSigma);
        double sz = optional(f, column, "sz", heightSigma);
        String type = Optional.ofNullable(field(f, column, "type")).orElse("control").toLowerCase(Locale.ROOT);
        if (id == null || id.isBlank()) throw new IllegalArgumentException("missing gcp_id");
        if (!type.equals("control") && !type.equals("check"))
          throw new IllegalArgumentException("type must be control or check");
        if (!(sx > 0 && sy > 0 && sz > 0)) throw new IllegalArgumentException("sigmas must be positive");
        out.add(new Point3(id.replace("\"", ""), new double[] {x, y, z}, new double[] {sx, sy, sz}, type.equals("check")));
      } catch (RuntimeException e) {
        throw new IllegalArgumentException("GCP file " + file + " line " + line + ": " + e.getMessage(), e);
      }
    }
    if (out.isEmpty()) throw new IllegalArgumentException("No ground control points in " + file);
    Set<String> ids = new HashSet<>();
    for (Point3 p : out) if (!ids.add(p.id)) throw new IllegalArgumentException("Duplicate GCP id " + p.id);
    return out;
  }

  private static boolean isNumber(String s) {
    try {
      Double.parseDouble(s.trim());
      return true;
    } catch (NumberFormatException e) {
      return false;
    }
  }

  private static String field(String[] f, Map<String, Integer> column, String... names) {
    for (String n : names) {
      Integer i = column.get(n);
      if (i != null && i < f.length && !f[i].isBlank()) return f[i].trim();
    }
    return null;
  }

  private static double optional(String[] f, Map<String, Integer> column, String name, double fallback) {
    String v = field(f, column, name);
    return v == null ? fallback : Double.parseDouble(v);
  }

  // ---- Markers file.

  static Path markersFile(Path run) {
    return run.resolve("04_geometry/gcp/markers.json");
  }

  static Markers load(Path file) throws IOException {
    Map<String, Object> json = JsonRead.object(JsonRead.read(file));
    if (!FORMAT.equals(json.get("format"))) throw new IOException("Not a markers file: " + file);
    Markers out = new Markers();
    out.crs = json.get("crs") == null ? null : String.valueOf(json.get("crs"));
    for (Object o : (List<?>) json.get("gcps")) {
      var g = JsonRead.object(o);
      out.gcps.add(
          new Point3(
              String.valueOf(g.get("id")),
              numbers(g.get("xyz")),
              numbers(g.get("sigma")),
              "check".equals(g.get("type"))));
    }
    for (Object o : (List<?>) json.get("markers")) {
      var m = JsonRead.object(o);
      Marker marker = new Marker(String.valueOf(m.get("gcp")), ((Number) m.get("image")).intValue());
      marker.status = String.valueOf(m.get("status"));
      marker.u = number(m.get("u"));
      marker.v = number(m.get("v"));
      marker.predictedU = number(m.get("predicted_u"));
      marker.predictedV = number(m.get("predicted_v"));
      out.markers.add(marker);
    }
    return out;
  }

  static void save(Markers m, Path file, List<ImageFrame> frames) throws IOException {
    Files.createDirectories(file.getParent());
    List<Object> gcps = new ArrayList<>(), markers = new ArrayList<>();
    for (Point3 p : m.gcps)
      gcps.add(Json.object("id", p.id, "xyz", p.xyz, "sigma", p.sigma, "type", p.check ? "check" : "control"));
    for (Marker k : m.markers)
      markers.add(
          Json.object(
              "gcp", k.gcp,
              "image", k.image,
              "filename", frames == null || k.image >= frames.size() ? null : frames.get(k.image).path.getFileName().toString(),
              "status", k.status,
              "u", finite(k.u),
              "v", finite(k.v),
              "predicted_u", finite(k.predictedU),
              "predicted_v", finite(k.predictedV)));
    Path temp = file.resolveSibling(file.getFileName() + ".part");
    Json.write(
        temp,
        Json.object(
            "format", FORMAT,
            "crs", m.crs,
            "pixel_frame", "original image pixels, distorted (as in the photograph), origin at the top-left pixel centre",
            "gcps", gcps,
            "markers", markers));
    CloudWriter.publish(temp, file);
  }

  private static Double finite(double v) {
    return Double.isFinite(v) ? v : null;
  }

  private static double number(Object o) {
    return o instanceof Number n ? n.doubleValue() : Double.NaN;
  }

  private static double[] numbers(Object o) {
    List<?> l = (List<?>) o;
    double[] out = new double[l.size()];
    for (int i = 0; i < out.length; i++) out[i] = ((Number) l.get(i)).doubleValue();
    return out;
  }

  // ---- Geometry in original image pixels.

  /** World point to original (distorted) image pixels, or null behind the camera. */
  static double[] project(ImageFrame f, double[] world) {
    Camera c = f.camera;
    double[] q = c.view(world);
    if (!(q[2] > 1e-9)) return null;
    Calibration k = f.calibration;
    double x = q[0] / q[2], y = q[1] / q[2], r = x * x + y * y;
    double[] d = k.distortion;
    double radial = 1 + d[0] * r + d[1] * r * r + d[4] * r * r * r;
    double xd = x * radial + 2 * d[2] * x * y + d[3] * (r + 2 * x * x);
    double yd = y * radial + d[2] * (r + 2 * y * y) + 2 * d[3] * x * y;
    return new double[] {k.fx * xd + k.cx, k.fy * yd + k.cy};
  }

  /** Original pixel to an undistorted normalized ray direction in camera coordinates. */
  static double[] normalized(ImageFrame f, double u, double v) {
    Calibration k = f.calibration;
    double xd = (u - k.cx) / k.fx, yd = (v - k.cy) / k.fy, x = xd, y = yd;
    double[] d = k.distortion;
    for (int iter = 0; iter < 30; iter++) {
      double r = x * x + y * y, a = 1 + d[0] * r + d[1] * r * r + d[4] * r * r * r;
      double ex = x * a + 2 * d[2] * x * y + d[3] * (r + 2 * x * x) - xd;
      double ey = y * a + d[2] * (r + 2 * y * y) + 2 * d[3] * x * y - yd;
      x -= ex / Math.max(.2, a);
      y -= ey / Math.max(.2, a);
      if (Math.hypot(ex, ey) < 1e-13) break;
    }
    return new double[] {x, y, 1};
  }

  /** The same observation as a keypoint of the processed (undistorted) frame. */
  static KeyPoint keypoint(ImageFrame f, double u, double v) {
    double[] n = normalized(f, u, v);
    Camera c = f.camera;
    return new KeyPoint((float) (c.fx * n[0] + c.cx), (float) (c.fy * n[1] + c.cy), 4);
  }

  static boolean inside(ImageFrame f, double[] uv) {
    return uv != null && uv[0] >= 0 && uv[1] >= 0 && uv[0] <= f.sourceWidth - 1 && uv[1] <= f.sourceHeight - 1;
  }

  /** Least-squares intersection of the rays of a GCP's pinned markers (null if fewer than two). */
  static double[] triangulate(Markers m, String gcp, List<ImageFrame> frames) {
    double[][] a = new double[3][3];
    double[] b = new double[3];
    int rays = 0;
    for (Marker k : m.of(gcp)) {
      if (!k.pinned() || k.image >= frames.size()) continue;
      ImageFrame f = frames.get(k.image);
      if (!f.camera.registered) continue;
      double[] n = normalized(f, k.u, k.v), d = Math3.mv(Math3.transpose(f.camera.rotation), n), c = f.camera.center();
      double norm = Math3.norm(d);
      for (int i = 0; i < 3; i++) d[i] /= norm;
      for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
          double p = (i == j ? 1 : 0) - d[i] * d[j];
          a[i][j] += p;
          b[i] += p * c[j];
        }
      rays++;
    }
    return rays < 2 ? null : Math3.solve(a, b);
  }

  /** Pixel residual of a pinned marker against a world point (original pixels). */
  static double residual(ImageFrame f, Marker k, double[] world) {
    double[] uv = project(f, world);
    return uv == null ? Double.NaN : Math.hypot(uv[0] - k.u, uv[1] - k.v);
  }

  /**
   * Refreshes the predicted positions of every GCP in every registered image: from its pins when
   * it has two, else from its coordinates (georeferenced block) or through a similarity fitted
   * to the GCPs that have two pins (local block). New markers are added for images that see the
   * point; predicted-only markers that no longer fall in the image are dropped.
   */
  static void predict(Markers m, List<ImageFrame> frames, boolean georeferenced) {
    Georeference.Similarity toLocal = null;
    if (!georeferenced) {
      List<double[]> world = new ArrayList<>(), local = new ArrayList<>();
      for (Point3 p : m.gcps) {
        double[] t = triangulate(m, p.id, frames);
        if (t != null) {
          world.add(p.xyz);
          local.add(t);
        }
      }
      if (world.size() >= 3)
        try {
          toLocal = Georeference.fit(world, local);
        } catch (IllegalArgumentException e) {
          toLocal = null;
        }
    }
    for (Point3 p : m.gcps) {
      double[] where = triangulate(m, p.id, frames);
      if (where == null) where = georeferenced ? p.xyz : toLocal == null ? null : toLocal.apply(p.xyz);
      for (ImageFrame f : frames) {
        Marker k = m.marker(p.id, f.id);
        double[] uv = where == null || !f.camera.registered ? null : project(f, where);
        boolean visible = inside(f, uv);
        if (k == null) {
          if (!visible) continue;
          k = new Marker(p.id, f.id);
          m.markers.add(k);
        }
        k.predictedU = visible ? uv[0] : Double.NaN;
        k.predictedV = visible ? uv[1] : Double.NaN;
        if (k.status.equals("predicted")) {
          k.u = k.predictedU;
          k.v = k.predictedV;
        }
      }
    }
    m.markers.removeIf(k -> k.status.equals("predicted") && !Double.isFinite(k.u));
    m.markers.sort(Comparator.comparing((Marker k) -> k.gcp).thenComparingInt(k -> k.image));
  }

  /** Ready for the adjustment: enough pins to fix the datum. */
  static String readiness(Markers m, boolean georeferenced) {
    int control = 0;
    for (Point3 p : m.gcps) if (!p.check && m.pins(p.id) >= 2) control++;
    int needed = georeferenced ? 1 : 3;
    return control >= needed
        ? null
        : String.format(
            Locale.ROOT,
            "%d of the needed %d control points have markers pinned in at least two images",
            control,
            needed);
  }
}
