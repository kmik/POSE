package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import org.opencv.core.Point;

/** Deterministic, spatially stratified calibration sampling and image-space diagnostics. */
final class CalibrationDiagnostics {
  /** Compare complete forward projections at common rays in the reference pinhole frame. */
  static void compare(Path directory, Calibration candidate, Calibration reference)
      throws IOException {
    if (candidate.width != reference.width || candidate.height != reference.height)
      throw new IllegalArgumentException(
          "Calibration comparison requires the same reference image size");
    Files.createDirectories(directory);
    Camera a = candidate.camera(candidate.width, candidate.height),
        b = reference.camera(reference.width, reference.height);
    double sum = 0, maximum = 0;
    int count = 0;
    try (var out = Files.newBufferedWriter(directory.resolve("projection-field.csv"))) {
      out.write(
          "reference_pinhole_x,reference_pinhole_y,candidate_distorted_x,candidate_distorted_y,reference_distorted_x,reference_distorted_y,difference_pixels\n");
      for (int iy = 0; iy <= 30; iy++)
        for (int ix = 0; ix <= 40; ix++) {
          double u = ix * (reference.width - 1.) / 40, v = iy * (reference.height - 1.) / 30;
          double x = (u - b.cx) / b.fx, y = (v - b.cy) / b.fy;
          Point p =
              SharedCalibration.distorted(
                  a, candidate.distortion, new Point(a.fx * x + a.cx, a.fy * y + a.cy));
          Point q = SharedCalibration.distorted(b, reference.distortion, new Point(u, v));
          double delta = Math.hypot(p.x - q.x, p.y - q.y);
          sum += delta * delta;
          maximum = Math.max(maximum, delta);
          count++;
          out.write(
              u + "," + v + "," + p.x + "," + p.y + "," + q.x + "," + q.y + "," + delta + "\n");
        }
    }
    Json.write(
        directory.resolve("projection-comparison.json"),
        Json.object(
            "samples",
            count,
            "rms_forward_projection_difference_pixels",
            Math.sqrt(sum / count),
            "maximum_difference_pixels",
            maximum,
            "reference_frame",
            "Common rays defined by reference pinhole K; compare complete K+Brown projections,"
                + " including k3 and tangential terms",
            "scope",
            "Model-to-model disagreement, not accuracy against ground truth; uncovered corners are"
                + " extrapolation"));
  }

  record Split(
      List<FeatureTracks.Track> train,
      List<FeatureTracks.Track> first,
      List<FeatureTracks.Track> second,
      Object coverage) {}

  static Split split(List<ImageFrame> frames, List<FeatureTracks.Track> tracks) {
    Map<Integer, List<FeatureTracks.Track>> cells = new TreeMap<>();
    int[][] support = new int[frames.size()][48];
    for (var t : tracks) {
      if (t.point == null) continue;
      int registered = 0, chosen = -1, count = Integer.MAX_VALUE;
      for (var ob : t.observations) {
        var f = frames.get(ob.image);
        if (!f.camera.registered) continue;
        registered++;
        Point p = f.keypoints[ob.feature].pt;
        int x = Math.clamp((int) (p.x * 8 / f.width), 0, 7);
        int y = Math.clamp((int) (p.y * 6 / f.height), 0, 5);
        int cell = y * 8 + x;
        if (support[f.id][cell] < count) {
          chosen = f.id * 48 + cell;
          count = support[f.id][cell];
        }
      }
      // Two-view tracks still constrain some lens parameters. Observability is assessed after
      // eliminating their free 3D positions, rather than by excluding them here.
      if (registered < 2) continue;
      support[chosen / 48][chosen % 48]++;
      cells.computeIfAbsent(chosen, k -> new ArrayList<>()).add(t);
    }
    List<FeatureTracks.Track> train = new ArrayList<>(),
        a = new ArrayList<>(),
        b = new ArrayList<>();
    for (var entry : cells.entrySet()) {
      // A fixed cap prevents central texture-rich cells from dominating the lens fit.
      var list = entry.getValue();
      int n = Math.min(200, list.size());
      for (int i = 0; i < n; i++) {
        var t = list.get((int) ((long) i * list.size() / n));
        int fold = (i + entry.getKey() * 7) % 10;
        (fold == 0 ? a : fold == 1 ? b : train).add(t);
      }
    }
    return new Split(
        train,
        a,
        b,
        Json.object(
            "grid_columns",
            8,
            "grid_rows",
            6,
            "maximum_tracks_per_cell",
            200,
            "eligible_tracks_per_camera_cell",
            support,
            "assignment",
            "least-populated observed camera/cell; deterministic 80/10/10 split"));
  }

  static void residuals(
      Path directory,
      String name,
      SharedCalibration lens,
      List<ImageFrame> frames,
      List<FeatureTracks.Track> train,
      List<FeatureTracks.Track> first,
      List<FeatureTracks.Track> second)
      throws IOException {
    Files.createDirectories(directory);
    // Held-out points are fitted conditionally in a private copy; never fit poses or intrinsics.
    List<FeatureTracks.Track> all = new ArrayList<>(train);
    all.addAll(first);
    all.addAll(second);
    Set<FeatureTracks.Track> held = new HashSet<>(first);
    held.addAll(second);
    double[][] saved = all.stream().map(t -> t.point).toArray(double[][]::new);
    double[][][] grid = new double[frames.size()][48][3];
    double[][][] radial = new double[2][6][4];
    try (var out = Files.newBufferedWriter(directory.resolve(name + "-residuals.csv"))) {
      lens.validationRms(frames, new ArrayList<>(held), true);
      out.write("track_id,image_id,split,x_source,y_source,dx_source,dy_source,radius_fraction\n");
      for (int i = 0; i < all.size(); i++) {
        var t = all.get(i);
        for (var ob : t.observations) {
          var f = frames.get(ob.image);
          if (!f.camera.registered) continue;
          double[] uv = lens.project(f, t.point);
          if (uv == null) continue;
          Point p = lens.observed[f.id][ob.feature];
          double x = (p.x + .5) * lens.initial.width / f.width - .5;
          double y = (p.y + .5) * lens.initial.height / f.height - .5;
          double dx = (uv[0] - p.x) * lens.initial.width / f.width;
          double dy = (uv[1] - p.y) * lens.initial.height / f.height;
          double radius =
              Math.hypot(x - lens.initial.cx, y - lens.initial.cy)
                  / Math.hypot(lens.initial.width / 2., lens.initial.height / 2.);
          double[] bin = radial[held.contains(t) ? 1 : 0][Math.clamp((int) (radius * 6), 0, 5)];
          bin[0]++;
          bin[1] += dx;
          bin[2] += dy;
          bin[3] += dx * dx + dy * dy;
          out.write(
              i
                  + ","
                  + f.id
                  + ","
                  + (held.contains(t) ? "held_out" : "training")
                  + ","
                  + x
                  + ","
                  + y
                  + ","
                  + dx
                  + ","
                  + dy
                  + ","
                  + radius
                  + "\n");
          int cell =
              Math.clamp((int) (p.y * 6 / f.height), 0, 5) * 8
                  + Math.clamp((int) (p.x * 8 / f.width), 0, 7);
          grid[f.id][cell][0] += dx;
          grid[f.id][cell][1] += dy;
          grid[f.id][cell][2]++;
        }
      }
    } finally {
      for (int i = 0; i < all.size(); i++) all.get(i).point = saved[i];
    }
    List<Object> summaries = new ArrayList<>();
    for (int split = 0; split < 2; split++)
      for (int band = 0; band < 6; band++) {
        double[] b = radial[split][band];
        summaries.add(
            Json.object(
                "split",
                split == 0 ? "training" : "held_out",
                "minimum_radius_fraction",
                band / 6.,
                "maximum_radius_fraction",
                band == 5 ? null : (band + 1) / 6.,
                "observations",
                (long) b[0],
                "mean_dx_source_pixels",
                b[0] > 0 ? b[1] / b[0] : null,
                "mean_dy_source_pixels",
                b[0] > 0 ? b[2] / b[0] : null,
                "rms_source_pixels",
                b[0] > 0 ? Math.sqrt(b[3] / b[0]) : null));
      }
    Json.write(directory.resolve(name + "-radial-summary.json"), summaries);
    for (var f : frames) {
      var svg =
          new StringBuilder(
                  "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 800 650\"><rect"
                      + " width=\"800\" height=\"650\" fill=\"white\"/><text x=\"10\""
                      + " y=\"25\">Image ")
              .append(f.id)
              .append(" mean residual vectors; 20 display units per source pixel</text>");
      for (int i = 0; i < 48; i++) {
        double[] v = grid[f.id][i];
        if (v[2] == 0) continue;
        double x = (i % 8 + .5) * 100, y = (i / 8 + .5) * 95 + 55;
        svg.append(
            String.format(
                Locale.ROOT,
                "<circle cx=\"%.2f\" cy=\"%.2f\" r=\"2\"/><path d=\"M %.2f %.2f l %.2f %.2f\""
                    + " stroke=\"#176db0\"/><text x=\"%.2f\" y=\"%.2f\""
                    + " font-size=\"12\">n=%.0f</text>",
                x,
                y,
                x,
                y,
                20 * v[0] / v[2],
                20 * v[1] / v[2],
                x - 25,
                y + 20,
                v[2]));
      }
      Files.writeString(directory.resolve(name + "-image-" + f.id + ".svg"), svg.append("</svg>"));
    }
  }

  static Object correctionUncertainty(
      SharedCalibration lens, double[][] covariance, double coverage) {
    if (covariance == null) return null;
    double observed = 0, corners = 0;
    Calibration c = lens.calibration();
    for (int iy = 0; iy <= 12; iy++)
      for (int ix = 0; ix <= 16; ix++) {
        double u = ix * (c.width - 1.) / 16, v = iy * (c.height - 1.) / 12;
        double x = (u - c.cx) / c.fx, y = (v - c.cy) / c.fy, r = x * x + y * y;
        double radial =
            1 + c.distortion[0] * r + c.distortion[1] * r * r + c.distortion[4] * r * r * r;
        double[][] j = {
          {
            c.fx * (x * radial + 2 * c.distortion[2] * x * y + c.distortion[3] * (r + 2 * x * x)),
            c.fx * x * r,
            c.fx * x * r * r,
            c.width,
            0,
            2 * c.fx * x * y,
            c.fx * (r + 2 * x * x),
            c.fx * x * r * r * r
          },
          {
            c.fy * (y * radial + c.distortion[2] * (r + 2 * y * y) + 2 * c.distortion[3] * x * y),
            c.fy * y * r,
            c.fy * y * r * r,
            0,
            c.height,
            c.fy * (r + 2 * y * y),
            2 * c.fy * x * y,
            c.fy * y * r * r * r
          }
        };
        double variance = 0;
        for (int row = 0; row < 2; row++)
          for (int a = 0; a < SharedCalibration.LENS; a++)
            for (int b = 0; b < SharedCalibration.LENS; b++)
              variance += j[row][a] * covariance[a][b] * j[row][b];
        double sigma = Math.sqrt(Math.max(0, variance));
        corners = Math.max(corners, sigma);
        if (Math.hypot(u - c.cx, v - c.cy) <= coverage * Math.hypot(c.width / 2., c.height / 2.))
          observed = Math.max(observed, sigma);
      }
    double[][] correlations = new double[SharedCalibration.SIZE][SharedCalibration.SIZE];
    for (int i = 0; i < correlations.length; i++)
      for (int j = 0; j < correlations.length; j++) {
        double d = Math.sqrt(covariance[i][i] * covariance[j][j]);
        correlations[i][j] = d > 0 ? covariance[i][j] / d : 0;
      }
    return Json.object(
        "covariance_solver_units",
        covariance,
        "correlation",
        correlations,
        "maximum_combined_pixel_sigma_within_p90_radius",
        observed,
        "maximum_combined_pixel_sigma_full_image_grid",
        corners,
        "scope",
        "Conditional forward pixel-correction uncertainty at fixed rays; includes lens"
            + " correlations; excludes model error. P90 radius is a coverage proxy, not support at"
            + " every grid location.");
  }
}
