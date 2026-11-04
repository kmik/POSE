package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import org.opencv.core.Point;

/**
 * T1: flags cameras whose orientation is likely poor, after the final geometry. A camera is flagged
 * when its sparse reprojection RMS or its GPS residual is far above the block's (more than 1.6x
 * the median, or more than 4 robust deviations above it), or when the heading between consecutive
 * camera centres turns by more than 45 degrees (frames taken in turns: fast yaw/roll, rolling
 * shutter, blur). {@code --camera-screening report} (default) writes
 * 04_geometry/camera_screening.json only; {@code exclude} also keeps flagged cameras out of dense
 * matching. The sparse geometry is unchanged in both modes.
 */
final class CameraScreening {
  private CameraScreening() {}

  static List<Integer> run(List<ImageFrame> frames, List<FeatureTracks.Track> tracks, Options o, Crs crs)
      throws IOException {
    if (o.cameraScreening.equals("off")) return List.of();
    int n = frames.size();
    double[] rms = new double[n];
    int[] observations = new int[n];
    double[] square = new double[n];
    for (FeatureTracks.Track track : tracks)
      if (track.point != null)
        for (FeatureTracks.Observation ob : track.observations) {
          ImageFrame f = frames.get(ob.image);
          if (!f.camera.registered) continue;
          double[] predicted = f.camera.project(track.point);
          if (predicted == null) continue;
          Point actual = f.keypoints[ob.feature].pt;
          double e = Math.hypot(predicted[0] - actual.x, predicted[1] - actual.y);
          square[ob.image] += e * e;
          observations[ob.image]++;
        }
    List<Double> rmsValues = new ArrayList<>(), gpsValues = new ArrayList<>();
    double[] gps = new double[n];
    Arrays.fill(gps, Double.NaN);
    for (ImageFrame f : frames) {
      if (!f.camera.registered) continue;
      if (observations[f.id] > 0) {
        rms[f.id] = Math.sqrt(square[f.id] / observations[f.id]);
        rmsValues.add(rms[f.id]);
      }
      if (crs != null && f.gps != null && f.gps.usable()) {
        gps[f.id] = Math3.norm(Math3.sub(f.camera.center(), crs.project(f.gps)));
        gpsValues.add(gps[f.id]);
      }
    }
    double rmsLimit = limit(rmsValues), gpsLimit = limit(gpsValues);
    double[] turn = turns(frames);
    List<Object> rows = new ArrayList<>();
    List<Integer> flagged = new ArrayList<>();
    int byResidual = 0, byGps = 0, byTurn = 0;
    for (ImageFrame f : frames) {
      if (!f.camera.registered) continue;
      List<String> reasons = new ArrayList<>();
      if (observations[f.id] > 0 && rms[f.id] > rmsLimit) {
        reasons.add("reprojection");
        byResidual++;
      }
      if (Double.isFinite(gps[f.id]) && gps[f.id] > gpsLimit) {
        reasons.add("gps_residual");
        byGps++;
      }
      if (turn[f.id] > 45) {
        reasons.add("turn");
        byTurn++;
      }
      if (!reasons.isEmpty()) flagged.add(f.id);
      rows.add(
          Json.object(
              "image_id", f.id,
              "image", f.path.getFileName().toString(),
              "observations", observations[f.id],
              "reprojection_rms_pixels", observations[f.id] > 0 ? rms[f.id] : null,
              "gps_residual_metres", Double.isFinite(gps[f.id]) ? gps[f.id] : null,
              "heading_change_degrees", turn[f.id],
              "flags", reasons));
    }
    boolean exclude = o.cameraScreening.equals("exclude");
    if (exclude) for (int id : flagged) frames.get(id).camera.denseExcluded = true;
    Path dir = o.output.resolve("04_geometry");
    Files.createDirectories(dir);
    Json.write(
        dir.resolve("camera_screening.json"),
        Json.object(
            "mode", o.cameraScreening,
            "reprojection_rms_limit_pixels", rmsLimit,
            "gps_residual_limit_metres", Double.isFinite(gpsLimit) ? gpsLimit : null,
            "turn_limit_degrees", 45,
            "flagged", flagged.size(),
            "flagged_reprojection", byResidual,
            "flagged_gps_residual", byGps,
            "flagged_turn", byTurn,
            "excluded_from_dense_matching", exclude ? flagged : List.of(),
            "cameras", rows));
    System.out.printf(
        Locale.ROOT,
        "Camera screening: %d of %d cameras flagged (reprojection %d, GPS residual %d, turn %d)%s%n",
        flagged.size(),
        frames.size(),
        byResidual,
        byGps,
        byTurn,
        exclude ? "; excluded from dense matching" : "; see 04_geometry/camera_screening.json");
    return flagged;
  }

  /** max(1.6 x median, median + 4 x 1.4826 MAD): far above the block, robust to a few outliers. */
  static double limit(List<Double> values) {
    if (values.size() < 5) return Double.POSITIVE_INFINITY;
    double[] v = values.stream().mapToDouble(Double::doubleValue).sorted().toArray();
    double median = v[v.length / 2];
    double[] dev = new double[v.length];
    for (int i = 0; i < v.length; i++) dev[i] = Math.abs(v[i] - median);
    Arrays.sort(dev);
    return Math.max(1.6 * median, median + 4 * 1.4826 * dev[dev.length / 2]);
  }

  /** Largest heading change (degrees) at each camera between its neighbouring centre steps. */
  static double[] turns(List<ImageFrame> frames) {
    int n = frames.size();
    double[] out = new double[n];
    for (int i = 1; i + 1 < n; i++) {
      Camera a = frames.get(i - 1).camera, b = frames.get(i).camera, c = frames.get(i + 1).camera;
      if (!a.registered || !b.registered || !c.registered) continue;
      double[] p = a.center(), q = b.center(), r = c.center();
      double h1 = Math.atan2(q[1] - p[1], q[0] - p[0]), h2 = Math.atan2(r[1] - q[1], r[0] - q[0]);
      double d = Math.toDegrees(Math.abs(Math.atan2(Math.sin(h2 - h1), Math.cos(h2 - h1))));
      out[i] = Math.max(out[i], d);
      out[i - 1] = Math.max(out[i - 1], d / 2);
      out[i + 1] = Math.max(out[i + 1], d / 2);
    }
    return out;
  }
}
