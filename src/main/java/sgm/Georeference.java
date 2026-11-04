package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;

/** Robust camera-center similarity alignment to projected WGS84 EXIF GPS controls. */
final class Georeference {
  static final String HEIGHT_NOTE =
      "Z retains EXIF GPS altitude in metres. Its vertical datum is unspecified; no geoid/N2000"
          + " conversion is applied. Horizontal CRS only.";

  static final class Result {
    final Crs crs;
    final String description;

    Result(Crs crs, String description) {
      this.crs = crs;
      this.description = description;
    }
  }

  static Result align(List<ImageFrame> frames, List<FeatureTracks.Track> tracks, Options o)
      throws IOException {
    Path dir = o.output.resolve("04_geometry/georeference");
    Files.createDirectories(dir);
    if (o.orientationCrs != null && o.poses != null) {
      Crs supplied = new Crs(o.orientationCrs);
      if (!Set.of("auto", "m").contains(o.orientationUnits))
        throw new IllegalArgumentException(
            "Projected supplied orientations require metre coordinates; convert coordinates before"
                + " declaring their CRS");
      if (o.scale != 1)
        throw new IllegalArgumentException(
            "Projected supplied orientations require --scale 1 to preserve metre coordinates");
      if (o.gps)
        throw new IllegalArgumentException(
            "--gps cannot realign poses with an explicit --orientation-crs; remove the orientation"
                + " CRS to request a GPS fit");
      Json.write(
          o.output.resolve("coordinates.json"),
          Json.object(
              "georeferenced",
              true,
              "horizontal_crs",
              supplied.name,
              "reason",
              "Supplied projected camera coordinates; no transformation applied",
              "vertical_datum",
              "unspecified"));
      return new Result(
          supplied,
          "Supplied orientations in "
              + supplied.name
              + "; metre coordinates, no GPS similarity applied; vertical datum unspecified");
    }
    List<double[]> source = new ArrayList<>(), target = new ArrayList<>(), views = new ArrayList<>();
    List<Integer> ids = new ArrayList<>();
    Crs crs = o.noGps ? null : new Crs(o.crs);
    List<Object> inventory = new ArrayList<>();
    for (ImageFrame f : frames) {
      Exif.Gps g = f.gps;
      inventory.add(
          Json.object(
              "image_id",
              f.id,
              "filename",
              f.path.getFileName().toString(),
              "latitude_deg",
              g == null ? null : g.latitude,
              "longitude_deg",
              g == null ? null : g.longitude,
              "altitude_metres",
              g == null ? null : g.altitude,
              "gps_datum",
              g == null ? null : g.datum,
              "usable_3d_control",
              g != null && g.usable() && f.camera.registered));
      if (!o.noGps && g != null && g.usable() && f.camera.registered) {
        source.add(f.camera.center());
        target.add(crs.project(g));
        // Viewing direction (camera z axis) in the reconstruction frame: third row of R.
        views.add(Arrays.copyOfRange(f.camera.rotation, 6, 9));
        ids.add(f.id);
      }
    }
    Json.write(dir.resolve("gps_inventory.json"), inventory);
    Similarity transform = null;
    String reason =
        o.noGps
            ? "GPS alignment disabled by --no-gps"
            : "GPS alignment needs at least 3 registered images with WGS84 latitude, longitude and"
                + " altitude";
    String method = "similarity";
    if (source.size() >= 3) {
      try {
        if (collinear(target)) {
          // One straight flight line: GPS cannot fix the roll about the line; the cameras can.
          transform = lineFit(source, target, views, o.gpsThreshold);
          method = "single_line_with_downward_view";
        } else transform = robustFit(source, target, o.gpsThreshold);
      } catch (IllegalArgumentException e) {
        reason = e.getMessage();
      }
    }
    if (transform == null) {
      Json.write(
          dir.resolve("transform.json"),
          Json.object(
              "status",
              "not_applied",
              "reason",
              reason,
              "usable_controls",
              source.size(),
              "requested_crs",
              o.crs));
      Json.write(
          o.output.resolve("coordinates.json"),
          Json.object("georeferenced", false, "horizontal_crs", null, "reason", reason));
      if (o.gps) throw new IllegalArgumentException("Required GPS alignment failed: " + reason);
      System.out.println(
          "Georeference: " + reason + "; output remains in its reconstruction frame.");
      return new Result(null, reason);
    }
    if (o.scale != 1)
      throw new IllegalArgumentException(
          "GPS-georeferenced output requires --scale 1. Use --no-gps to export arbitrarily scaled"
              + " local coordinates.");
    double square = 0;
    int inliers = 0;
    try (BufferedWriter w = Files.newBufferedWriter(dir.resolve("camera_control.csv"))) {
      w.write(
          "image_id,sfm_x,sfm_y,sfm_z,target_easting,target_northing,target_altitude,fitted_easting,fitted_northing,fitted_altitude,residual_metres,inlier\n");
      for (int i = 0; i < source.size(); i++) {
        double[] a = source.get(i), b = target.get(i), predicted = transform.apply(a);
        double e = Math3.norm(Math3.sub(predicted, b));
        boolean used = e <= o.gpsThreshold;
        if (used) {
          square += e * e;
          inliers++;
        }
        w.write(
            ids.get(i)
                + ","
                + a[0]
                + ","
                + a[1]
                + ","
                + a[2]
                + ","
                + b[0]
                + ","
                + b[1]
                + ","
                + b[2]
                + ","
                + predicted[0]
                + ","
                + predicted[1]
                + ","
                + predicted[2]
                + ","
                + e
                + ","
                + used
                + "\n");
      }
    }
    double rms = Math.sqrt(square / inliers);
    Json.write(
        dir.resolve("transform.json"),
        Json.object(
            "status",
            "applied",
            "operation",
            "X_projected = scale * R * X_reconstruction + translation",
            "method",
            method,
            "method_note",
            method.equals("similarity")
                ? "Robust 3D similarity to the GPS camera centres"
                : "GPS camera centres lie on one line: scale, line direction and translation from"
                    + " GPS; roll about the line chosen so the mean camera viewing direction points"
                    + " straight down (survey imagery). Tilt across the line is not measured.",
            "scale",
            transform.scale,
            "R_row_major",
            transform.rotation,
            "translation",
            transform.translation,
            "horizontal_crs",
            crs.name,
            "proj_parameters",
            crs.parameters,
            "xy_order",
            "easting,northing",
            "units",
            "metres",
            "height_note",
            HEIGHT_NOTE,
            "gps_controls",
            source.size(),
            "gps_inliers",
            inliers,
            "inlier_rms_metres",
            rms,
            "ransac_threshold_metres",
            o.gpsThreshold,
            "accuracy_note",
            "Camera GPS accuracy, scene geometry and static datum transformations limit absolute"
                + " accuracy. This similarity has no epoch, geoid or ground control adjustment; GPS time"
                + " offset, lever arm and lens refit: gps_bundle.json."));
    Json.write(
        o.output.resolve("coordinates.json"),
        Json.object(
            "georeferenced",
            true,
            "horizontal_crs",
            crs.name,
            "epsg",
            crs.epsg,
            "xy_order",
            "easting,northing",
            "xyz_units",
            "metres",
            "vertical_crs",
            null,
            "height_note",
            HEIGHT_NOTE,
            "inlier_rms_metres",
            rms,
            "gps_inliers",
            inliers,
            "gps_controls",
            source.size(),
            "coordinate_multiplier",
            1));
    for (ImageFrame f : frames) {
      Camera c = f.camera;
      c.rotation = Math3.mul(c.rotation, Math3.transpose(transform.rotation));
      c.translation =
          Math3.sub(
              Math3.times(c.translation, transform.scale),
              Math3.mv(c.rotation, transform.translation));
    }
    for (FeatureTracks.Track track : tracks)
      if (track.point != null) track.point = transform.apply(track.point);
    String bundled = "";
    if (o.gpsBundle.equals("prior")) {
      Map<Integer, double[]> controls = new LinkedHashMap<>();
      for (int i = 0; i < source.size(); i++)
        if (Math3.norm(Math3.sub(transform.apply(source.get(i)), target.get(i))) <= o.gpsThreshold)
          controls.put(ids.get(i), target.get(i));
      // G1, extended: lens, GPS time offset/lever arm and rolling shutter (GpsAdjustment).
      GpsAdjustment.Outcome after = GpsAdjustment.run(frames, tracks, o, crs, controls, dir, rms);
      if (!after.summary().isEmpty())
        bundled =
            String.format(
                Locale.ROOT,
                "; GPS-weighted bundle (sigma %.2f m) RMS=%.3f m",
                o.gpsSigma,
                after.rms());
    }
    String description =
        String.format(
            Locale.ROOT,
            "%s easting/northing, metres; GPS camera fit RMS=%.3f m (%d/%d inliers)%s. %s",
            crs.name,
            rms,
            inliers,
            source.size(),
            bundled,
            HEIGHT_NOTE);
    System.out.println("Georeference: " + description);
    return new Result(crs, description);
  }

  /**
   * True when the points spread along one line: the second principal spread is below 3% of the
   * first (standard deviations). GPS noise across a single flight line would otherwise decide the
   * roll of the whole block.
   */
  static boolean collinear(List<double[]> points) {
    double[] spread = principal(points);
    return spread[1] < .03 * spread[0];
  }

  /** {std along first axis, std along second axis, axis x, y, z} of the point cloud. */
  private static double[] principal(List<double[]> points) {
    int n = points.size();
    double[] mean = new double[3], c = new double[9];
    for (double[] p : points) for (int k = 0; k < 3; k++) mean[k] += p[k] / n;
    for (double[] p : points)
      for (int r = 0; r < 3; r++)
        for (int k = 0; k < 3; k++) c[r * 3 + k] += (p[r] - mean[r]) * (p[k] - mean[k]) / n;
    Mat m = Math3.mat(c, 3, 3), w = new Mat(), u = new Mat(), vt = new Mat();
    try {
      Core.SVDecomp(m, w, u, vt);
      double[] s = Math3.array(w), axes = Math3.array(u);
      return new double[] {
        Math.sqrt(Math.max(0, s[0])), Math.sqrt(Math.max(0, s[1])), axes[0], axes[3], axes[6]
      };
    } finally {
      m.release();
      w.release();
      u.release();
      vt.release();
    }
  }

  /**
   * Similarity for camera centres on one line. Scale, line direction and translation come from
   * GPS; the roll about the line, which GPS cannot see, turns the mean camera viewing direction
   * as close to straight down as possible. Outliers beyond the threshold are dropped iteratively.
   */
  static Similarity lineFit(
      List<double[]> source, List<double[]> target, List<double[]> views, double threshold) {
    int n = source.size();
    boolean[] use = new boolean[n];
    Arrays.fill(use, true);
    Similarity best = null;
    for (int iteration = 0; iteration < 5; iteration++) {
      List<double[]> s = new ArrayList<>(), t = new ArrayList<>(), v = new ArrayList<>();
      for (int i = 0; i < n; i++)
        if (use[i]) {
          s.add(source.get(i));
          t.add(target.get(i));
          v.add(views.get(i));
        }
      if (s.size() < Math.max(3, n / 2 + 1))
        throw new IllegalArgumentException("Single-line GPS alignment has no inlier majority");
      best = lineSimilarity(s, t, v);
      boolean changed = false;
      for (int i = 0; i < n; i++) {
        boolean inlier =
            Math3.norm(Math3.sub(best.apply(source.get(i)), target.get(i))) <= threshold;
        changed |= inlier != use[i];
        use[i] = inlier;
      }
      if (!changed) break;
    }
    return best;
  }

  private static Similarity lineSimilarity(
      List<double[]> source, List<double[]> target, List<double[]> views) {
    double[] ps = principal(source), pt = principal(target);
    double[] us = {ps[2], ps[3], ps[4]}, ut = {pt[2], pt[3], pt[4]};
    int n = source.size();
    double[] ms = new double[3], mt = new double[3];
    for (int i = 0; i < n; i++)
      for (int k = 0; k < 3; k++) {
        ms[k] += source.get(i)[k] / n;
        mt[k] += target.get(i)[k] / n;
      }
    double cross = 0, ss = 0, tt = 0;
    for (int i = 0; i < n; i++) {
      double a = dot(Math3.sub(source.get(i), ms), us), b = dot(Math3.sub(target.get(i), mt), ut);
      cross += a * b;
      ss += a * a;
      tt += b * b;
    }
    if (!(ss > 1e-12) || !(tt > 1e-12))
      throw new IllegalArgumentException("GPS alignment requires distinct camera positions");
    if (cross < 0) ut = Math3.times(ut, -1);
    double scale = Math.sqrt(tt / ss);
    double[] r0 = turn(us, ut);
    // Roll about the line: bring the mean viewing direction to straight down.
    double[] view = new double[3];
    for (double[] v : views) view = Math3.add(view, v);
    double[] w = Math3.mv(r0, view), down = {0, 0, -1};
    double[] wp = Math3.sub(w, Math3.times(ut, dot(w, ut))),
        dp = Math3.sub(down, Math3.times(ut, dot(down, ut)));
    double[] rotation = r0;
    if (Math3.norm(wp) > 1e-9 && Math3.norm(dp) > 1e-9) {
      double angle =
          Math.atan2(dot(ut, cross(wp, dp)), dot(wp, dp));
      rotation = Math3.mul(Math3.rotation(Math3.times(ut, angle)), r0);
    }
    return new Similarity(
        scale, rotation, Math3.sub(mt, Math3.times(Math3.mv(rotation, ms), scale)));
  }

  /** Smallest rotation taking unit vector a to unit vector b. */
  private static double[] turn(double[] a, double[] b) {
    double[] axis = cross(a, b);
    double sin = Math3.norm(axis), cos = dot(a, b);
    if (sin < 1e-12) {
      if (cos > 0) return Math3.identity();
      // Opposite: half turn about any axis perpendicular to a.
      double[] p = Math.abs(a[0]) < .9 ? new double[] {1, 0, 0} : new double[] {0, 1, 0};
      double[] q = cross(a, p);
      return Math3.rotation(Math3.times(q, Math.PI / Math3.norm(q)));
    }
    return Math3.rotation(Math3.times(axis, Math.atan2(sin, cos) / sin));
  }

  private static double dot(double[] a, double[] b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  }

  private static double[] cross(double[] a, double[] b) {
    return new double[] {
      a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]
    };
  }

  static Similarity robustFit(List<double[]> source, List<double[]> target, double threshold) {
    if (source.size() != target.size() || source.size() < 3)
      throw new IllegalArgumentException("At least 3 GPS camera controls required");
    int n = source.size(), bestCount = 0;
    double bestError = Double.POSITIVE_INFINITY;
    Similarity best = null;
    Random random = new Random(57291);
    for (int trial = 0; trial < (n == 3 ? 1 : 600); trial++) {
      List<double[]> x = new ArrayList<>(), y = new ArrayList<>();
      Set<Integer> indices = new LinkedHashSet<>();
      if (trial == 0) for (int i = 0; i < n; i++) indices.add(i);
      else while (indices.size() < 3) indices.add(random.nextInt(n));
      for (int i : indices) {
        x.add(source.get(i));
        y.add(target.get(i));
      }
      Similarity candidate;
      try {
        candidate = fit(x, y);
      } catch (IllegalArgumentException e) {
        continue;
      }
      int count = 0;
      double error = 0;
      for (int i = 0; i < n; i++) {
        double d = Math3.norm(Math3.sub(candidate.apply(source.get(i)), target.get(i)));
        if (d <= threshold) {
          count++;
          error += d * d;
        }
      }
      if (count > bestCount || count == bestCount && error < bestError) {
        best = candidate;
        bestCount = count;
        bestError = error;
      }
    }
    if (best == null || bestCount < Math.max(3, n / 2 + 1))
      throw new IllegalArgumentException(
          "GPS alignment is underconstrained, collinear, or has no majority within"
              + " --gps-threshold");
    for (int iteration = 0; iteration < 5; iteration++) {
      List<double[]> x = new ArrayList<>(), y = new ArrayList<>();
      for (int i = 0; i < n; i++)
        if (Math3.norm(Math3.sub(best.apply(source.get(i)), target.get(i))) <= threshold) {
          x.add(source.get(i));
          y.add(target.get(i));
        }
      if (x.size() < Math.max(3, n / 2 + 1))
        throw new IllegalArgumentException("GPS inlier fit became unstable");
      best = fit(x, y);
    }
    int finalCount = 0;
    for (int i = 0; i < n; i++)
      if (Math3.norm(Math3.sub(best.apply(source.get(i)), target.get(i))) <= threshold)
        finalCount++;
    if (finalCount < Math.max(3, n / 2 + 1))
      throw new IllegalArgumentException("GPS fit failed final residual validation");
    return best;
  }

  static Similarity fit(List<double[]> source, List<double[]> target) {
    if (source.size() != target.size() || source.size() < 3)
      throw new IllegalArgumentException("At least 3 camera pairs needed");
    int n = source.size();
    double[] meanX = new double[3], meanY = new double[3];
    for (int i = 0; i < n; i++)
      for (int k = 0; k < 3; k++) {
        meanX[k] += source.get(i)[k] / n;
        meanY[k] += target.get(i)[k] / n;
      }
    double[] covariance = new double[9];
    double variance = 0;
    for (int i = 0; i < n; i++) {
      double[] x = Math3.sub(source.get(i), meanX), y = Math3.sub(target.get(i), meanY);
      variance += Math.pow(Math3.norm(x), 2) / n;
      for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++) covariance[r * 3 + c] += y[r] * x[c] / n;
    }
    Mat m = Math3.mat(covariance, 3, 3), w = new Mat(), u = new Mat(), vt = new Mat();
    try {
      Core.SVDecomp(m, w, u, vt);
      double[] singular = Math3.array(w);
      if (variance < 1e-12 || singular[0] < 1e-12 || singular[1] / singular[0] < 1e-4)
        throw new IllegalArgumentException("GPS alignment requires non-collinear camera positions");
      double sign = Core.determinant(u) * Core.determinant(vt) < 0 ? -1 : 1;
      double[] ua = Math3.array(u);
      for (int i = 0; i < 3; i++) ua[i * 3 + 2] *= sign;
      double[] rotation = Math3.mul(ua, Math3.array(vt));
      double scale = (singular[0] + singular[1] + sign * singular[2]) / variance;
      return new Similarity(
          scale, rotation, Math3.sub(meanY, Math3.times(Math3.mv(rotation, meanX), scale)));
    } finally {
      m.release();
      w.release();
      u.release();
      vt.release();
    }
  }

  static double[] ecef(Exif.Gps gps) {
    double lat = Math.toRadians(gps.latitude),
        lon = Math.toRadians(gps.longitude),
        e2 = 6.69437999014e-3;
    double radius = 6378137 / Math.sqrt(1 - e2 * Math.sin(lat) * Math.sin(lat));
    return new double[] {
      (radius + gps.altitude) * Math.cos(lat) * Math.cos(lon),
      (radius + gps.altitude) * Math.cos(lat) * Math.sin(lon),
      (radius * (1 - e2) + gps.altitude) * Math.sin(lat)
    };
  }

  static final class Similarity {
    final double scale;
    final double[] rotation, translation;

    Similarity(double scale, double[] rotation, double[] translation) {
      this.scale = scale;
      this.rotation = rotation;
      this.translation = translation;
    }

    double[] apply(double[] point) {
      return Math3.add(Math3.times(Math3.mv(rotation, point), scale), translation);
    }
  }
}
