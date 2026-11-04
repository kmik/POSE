package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import org.opencv.core.KeyPoint;

/**
 * Bundle adjustment with ground control points (the gcp step).
 *
 * <ol>
 *   <li>Datum: the control points are triangulated from their pinned markers and the block is
 *       moved onto them by a similarity (three or more control points) or a shift (fewer, when GPS
 *       already georeferences the block). A local block becomes georeferenced in --crs here.
 *   <li>Each control point becomes a track with a position prior (its sigmas) observed by its pins.
 *   <li>With GPS: the GPS-weighted adjustment runs again with the datum fixed by the GCPs (the GPS
 *       fixes get an offset, e.g. a different height datum, instead of moving the block). Without
 *       GPS: a bundle adjustment with the shared lens refit (--gps-intrinsics).
 *   <li>Residuals: control points (adjusted minus given), check points (triangulated from their
 *       pins with the final cameras, not used in the adjustment), pixel residual of every pin.
 * </ol>
 */
final class GcpAdjustment {
  private GcpAdjustment() {}

  record Result(Crs crs, Map<String, Object> report, String summary) {}

  static Result run(
      List<ImageFrame> frames, List<FeatureTracks.Track> tracks, Options o, Crs crs, GroundControl.Markers markers, Path dir)
      throws IOException {
    Files.createDirectories(dir);
    boolean georeferenced = crs != null;
    List<GroundControl.Point3> control = new ArrayList<>();
    List<double[]> local = new ArrayList<>(), world = new ArrayList<>();
    for (GroundControl.Point3 p : markers.gcps) {
      if (p.check) continue;
      double[] t = GroundControl.triangulate(markers, p.id, frames);
      if (t == null) continue;
      control.add(p);
      local.add(t);
      world.add(p.xyz);
    }
    String reason = GroundControl.readiness(markers, georeferenced);
    if (reason != null) throw new IllegalArgumentException("Ground control not ready: " + reason);
    // 1. Datum from the GCPs.
    Georeference.Similarity datum;
    String datumMethod;
    if (control.size() >= 3) {
      datum = Georeference.fit(local, world);
      datumMethod = "similarity to " + control.size() + " control points";
    } else {
      double[] shift = new double[3];
      for (int i = 0; i < local.size(); i++)
        for (int k = 0; k < 3; k++) shift[k] += (world.get(i)[k] - local.get(i)[k]) / local.size();
      datum = new Georeference.Similarity(1, Math3.identity(), shift);
      datumMethod = "shift to " + control.size() + " control point(s); GPS keeps scale and rotation";
    }
    for (ImageFrame f : frames) {
      Camera c = f.camera;
      c.rotation = Math3.mul(c.rotation, Math3.transpose(datum.rotation));
      c.translation = Math3.sub(Math3.times(c.translation, datum.scale), Math3.mv(c.rotation, datum.translation));
    }
    for (FeatureTracks.Track t : tracks) if (t.point != null) t.point = datum.apply(t.point);
    if (crs == null) crs = new Crs(o.crs);
    // 2. Control points as tracks with priors; their pins become extra keypoints.
    int[] originalKeypoints = new int[frames.size()];
    for (ImageFrame f : frames) originalKeypoints[f.id] = f.keypoints.length;
    Map<Integer, List<KeyPoint>> added = new HashMap<>();
    List<FeatureTracks.Track> gcpTracks = new ArrayList<>();
    for (GroundControl.Point3 p : control) {
      FeatureTracks.Track t = new FeatureTracks.Track();
      t.id = -1 - gcpTracks.size();
      t.prior = p.xyz.clone();
      t.priorSigma = p.sigma.clone();
      for (GroundControl.Marker k : markers.of(p.id)) {
        if (!k.pinned()) continue;
        ImageFrame f = frames.get(k.image);
        List<KeyPoint> list = added.computeIfAbsent(f.id, q -> new ArrayList<>());
        t.observations.add(new FeatureTracks.Observation(f.id, originalKeypoints[f.id] + list.size()));
        list.add(GroundControl.keypoint(f, k.u, k.v));
      }
      t.point = GroundControl.triangulate(markers, p.id, frames);
      gcpTracks.add(t);
    }
    for (var e : added.entrySet()) {
      ImageFrame f = frames.get(e.getKey());
      KeyPoint[] grown = Arrays.copyOf(f.keypoints, f.keypoints.length + e.getValue().size());
      for (int i = 0; i < e.getValue().size(); i++) grown[f.keypoints.length + i] = e.getValue().get(i);
      f.keypoints = grown;
    }
    List<FeatureTracks.Track> all = new ArrayList<>(tracks);
    all.addAll(gcpTracks);
    Map<String, Object> report = new LinkedHashMap<>();
    report.put("datum", datumMethod);
    report.put("datum_scale", datum.scale);
    String adjustment;
    try {
      // 3. Adjustment.
      Map<Integer, double[]> controls = new LinkedHashMap<>();
      for (ImageFrame f : frames)
        if (f.camera.registered && f.gps != null && f.gps.usable() && !o.noGps) {
          double[] g = crs.project(f.gps);
          controls.put(f.id, g);
        }
      if (controls.size() >= 3) {
        // Start the GPS fixes at the GCP datum: remove their mean offset first.
        double[] offset = new double[3];
        for (var e : controls.entrySet())
          for (int k = 0; k < 3; k++)
            offset[k] += (e.getValue()[k] - frames.get(e.getKey()).camera.center()[k]) / controls.size();
        double square = 0;
        for (var e : controls.entrySet()) {
          for (int k = 0; k < 3; k++) e.getValue()[k] -= offset[k];
          square += Math.pow(Math3.norm(Math3.sub(frames.get(e.getKey()).camera.center(), e.getValue())), 2);
        }
        GpsAdjustment.datumFixed = true;
        try {
          GpsAdjustment.Outcome outcome =
              GpsAdjustment.run(frames, all, o, crs, controls, dir, Math.sqrt(square / controls.size()));
          adjustment = "datum fixed by the GCPs; " + outcome.summary();
        } finally {
          GpsAdjustment.datumFixed = false;
        }
        double[] total = Math3.add(offset, GpsAdjustment.gpsOffset);
        report.put("gps_offset_to_gcp_datum_metres", total);
      } else {
        SharedCalibration lens = null;
        boolean shared = true;
        for (ImageFrame f : frames) shared &= SharedCalibration.same(frames.get(0).calibration, f.calibration);
        if (shared && !o.rectified && !(o.calibration != null && o.intrinsicsMode.equals("fixed")) && !o.gpsIntrinsics.equals("none")) {
          lens = new SharedCalibration(frames);
          for (String name : o.gpsIntrinsics.split(",")) {
            int k = Arrays.asList(SharedCalibration.NAMES).indexOf(name.trim());
            if (k >= 0) lens.active[k] = true;
          }
        }
        int anchor = -1, second = -1;
        for (ImageFrame f : frames)
          if (f.camera.registered) {
            if (anchor < 0) anchor = f.id;
            else if (second < 0) second = f.id;
          }
        BundleAdjuster.optimizeImpl(frames, all, Math.max(o.iterations, 20), o.reprojection, null, anchor, second, lens);
        if (lens != null && lens.physical()) {
          Calibration result = lens.calibration();
          for (ImageFrame f : frames) f.applyCalibration(result);
        }
        adjustment = "bundle adjustment with GCP priors" + (lens == null ? "" : " and shared lens " + String.join(",", lens.parameters()));
      }
    } finally {
      // Back to tie points only (keypoints were remapped together if the lens changed).
      for (ImageFrame f : frames)
        if (f.keypoints.length > originalKeypoints[f.id]) f.keypoints = Arrays.copyOf(f.keypoints, originalKeypoints[f.id]);
    }
    report.put("adjustment", adjustment);
    // 4. Residuals.
    List<Object> rows = new ArrayList<>();
    double[][] sums = new double[2][4];
    int[] counts = new int[2];
    List<Double> pixels = new ArrayList<>();
    for (GroundControl.Point3 p : markers.gcps) {
      double[] estimate = GroundControl.triangulate(markers, p.id, frames);
      if (!p.check)
        for (FeatureTracks.Track t : gcpTracks)
          if (Arrays.equals(t.prior, p.xyz)) estimate = t.point;
      List<Object> pins = new ArrayList<>();
      double pixelSum = 0;
      int pinCount = 0;
      for (GroundControl.Marker k : markers.of(p.id))
        if (k.pinned() && estimate != null) {
          double r = GroundControl.residual(frames.get(k.image), k, estimate);
          pins.add(Json.object("image", frames.get(k.image).path.getFileName().toString(), "pixel_residual", r));
          if (Double.isFinite(r)) {
            pixelSum += r * r;
            pinCount++;
            pixels.add(r);
          }
        }
      double[] error = estimate == null ? null : Math3.sub(estimate, p.xyz);
      if (error != null) {
        int s = p.check ? 1 : 0;
        for (int k = 0; k < 3; k++) sums[s][k] += error[k] * error[k];
        sums[s][3] += Math3.norm(error) * Math3.norm(error);
        counts[s]++;
      }
      rows.add(
          Json.object(
              "id", p.id,
              "type", p.check ? "check" : "control",
              "given", p.xyz,
              "sigma", p.sigma,
              "estimated", estimate,
              "error_xyz_metres", error,
              "error_metres", error == null ? null : Math3.norm(error),
              "pinned_markers", markers.pins(p.id),
              "pixel_rms", pinCount == 0 ? null : Math.sqrt(pixelSum / pinCount),
              "pins", pins));
    }
    String[] kinds = {"control", "check"};
    for (int s = 0; s < 2; s++)
      if (counts[s] > 0)
        report.put(
            kinds[s] + "_rms_metres",
            Json.object(
                "x", Math.sqrt(sums[s][0] / counts[s]),
                "y", Math.sqrt(sums[s][1] / counts[s]),
                "z", Math.sqrt(sums[s][2] / counts[s]),
                "xy", Math.sqrt((sums[s][0] + sums[s][1]) / counts[s]),
                "total", Math.sqrt(sums[s][3] / counts[s]),
                "points", counts[s]));
    Collections.sort(pixels);
    double pixelRms = 0;
    for (double p : pixels) pixelRms += p * p;
    report.put("marker_pixel_rms", pixels.isEmpty() ? null : Math.sqrt(pixelRms / pixels.size()));
    report.put("crs", crs.name);
    report.put("points", rows);
    report.put(
        "note",
        "Control points are adjusted with their given sigmas; check points are triangulated from their"
            + " pins with the final cameras and never used in the adjustment, so their errors are an"
            + " independent accuracy measure. Pixel residuals are in original image pixels.");
    Json.write(dir.resolve("gcp_report.json"), report);
    try (BufferedWriter w = Files.newBufferedWriter(dir.resolve("gcp_residuals.csv"))) {
      w.write("gcp_id,type,dx,dy,dz,error_m,pinned_markers\n");
      for (Object r : rows) {
        var m = JsonRead.object(r);
        double[] e = m.get("error_xyz_metres") instanceof double[] d ? d : null;
        w.write(m.get("id") + "," + m.get("type") + "," + (e == null ? ",," : e[0] + "," + e[1] + "," + e[2]) + "," + (m.get("error_metres") == null ? "" : m.get("error_metres")) + "," + m.get("pinned_markers") + "\n");
      }
    }
    String summary =
        String.format(
            Locale.ROOT,
            "Ground control: %d control, %d check points; control RMS %s, check RMS %s, marker RMS %.2f px (%s)",
            counts[0],
            counts[1],
            counts[0] == 0 ? "-" : String.format(Locale.ROOT, "%.3f m", Math.sqrt(sums[0][3] / counts[0])),
            counts[1] == 0 ? "-" : String.format(Locale.ROOT, "%.3f m", Math.sqrt(sums[1][3] / counts[1])),
            pixels.isEmpty() ? Double.NaN : Math.sqrt(pixelRms / pixels.size()),
            datumMethod);
    System.out.println(summary);
    return new Result(crs, report, summary);
  }
}
