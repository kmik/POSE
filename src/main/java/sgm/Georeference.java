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
    List<double[]> source = new ArrayList<>(), target = new ArrayList<>();
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
    if (source.size() >= 3) {
      try {
        transform = robustFit(source, target, o.gpsThreshold);
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
                + " accuracy. No epoch, antenna lever arm, geoid or ground control adjustment."));
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
    String description =
        String.format(
            Locale.ROOT,
            "%s easting/northing, metres; GPS camera fit RMS=%.3f m (%d/%d inliers). %s",
            crs.name,
            rms,
            inliers,
            source.size(),
            HEIGHT_NOTE);
    System.out.println("Georeference: " + description);
    return new Result(crs, description);
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
