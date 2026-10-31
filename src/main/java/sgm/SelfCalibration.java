package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;

/** Joint shared-focal/radial, pose and structure LM with point Schur elimination. */
final class SelfCalibration {
  static final String MODEL =
      "Shared f,k1,k2; square source pixels; principal point fixed at raster center; zero skew,"
          + " p1,p2,k3 fixed to zero";

  static final class View {
    Camera camera;
    final double sx, sy;

    View(Camera camera, double sx, double sy) {
      this.camera = camera.copy();
      this.sx = sx;
      this.sy = sy;
    }
  }

  static final class Seen {
    final int view;
    final double u, v;

    Seen(int view, double u, double v) {
      this.view = view;
      this.u = u;
      this.v = v;
    }
  }

  static final class Sample {
    final int id;
    final boolean validation;
    double[] point;
    final List<Seen> seen = new ArrayList<>();

    Sample(int id, double[] point, boolean validation) {
      this.id = id;
      this.point = point.clone();
      this.validation = validation;
    }
  }

  static final class Data {
    final List<View> views = new ArrayList<>();
    final List<Sample> samples = new ArrayList<>();
    double[] lens; // log(reference focal pixels), k1, k2
    final int width, height;
    final boolean fixedPoses;
    Options controls;
    double[] priorLens, extra = new double[3]; // fixed p1,p2,k3 from supplied calibration

    Data(Calibration c, boolean fixedPoses) {
      lens = new double[] {Math.log(c.fx), c.distortion[0], c.distortion[1]};
      width = c.width;
      height = c.height;
      this.fixedPoses = fixedPoses;
      extra = new double[] {c.distortion[2], c.distortion[3], c.distortion[4]};
    }

    int dimension() {
      return fixedPoses ? 3 : 3 + 6 * (views.size() - (hasPosePriors() ? 0 : 1));
    }

    boolean hasPosePriors() {
      return views.stream().anyMatch(v -> v.camera.reference != null);
    }

    int poseOffset(int view) {
      return fixedPoses || view == 0 && !hasPosePriors()
          ? -1
          : 3 + 6 * (view - (hasPosePriors() ? 0 : 1));
    }

    Data copy() {
      Data d =
          new Data(
              new Calibration(
                  Math.exp(lens[0]),
                  Math.exp(lens[0]),
                  (width - 1) / 2.,
                  (height - 1) / 2.,
                  new double[] {lens[1], lens[2], 0, 0, 0},
                  width,
                  height),
              fixedPoses);
      d.controls = controls;
      d.priorLens = priorLens == null ? null : priorLens.clone();
      d.extra = extra.clone();
      for (View v : views) d.views.add(new View(v.camera, v.sx, v.sy));
      for (Sample s : samples) {
        Sample t = new Sample(s.id, s.point, s.validation);
        t.seen.addAll(s.seen);
        d.samples.add(t);
      }
      return d;
    }
  }

  static final class Result {
    final Data fitted;
    Data evaluatedBaseline;
    boolean accepted;
    String reason;
    double before = Double.NaN, after = Double.NaN, coverage = Double.NaN, condition = Double.NaN;
    double[] standardDeviations = {Double.NaN, Double.NaN, Double.NaN};
    final List<Object> iterations = new ArrayList<>();

    Result(Data fitted) {
      this.fitted = fitted;
    }

    Object report() {
      return Json.object(
          "accepted",
          accepted,
          "reason",
          reason,
          "model",
          fitted.controls == null
              ? MODEL
              : "Selected shared f,k1,k2 refinement; supplied principal point, focal aspect ratio,"
                  + " p1,p2,k3 preserved",
          "intrinsics_mode",
          fitted.controls == null ? "automatic" : fitted.controls.intrinsicsMode,
          "selected_parameters",
          fitted.controls == null ? "f,k1,k2" : fitted.controls.refineIntrinsics,
          "prior_lens_log_f_k1_k2",
          fitted.priorLens,
          "focal_relative_sigma",
          fitted.controls == null ? null : fitted.controls.focalSigma,
          "radial_coefficient_sigma",
          fitted.controls == null ? null : fitted.controls.radialSigma,
          "focal_pixels",
          Math.exp(fitted.lens[0]),
          "k1",
          fitted.lens[1],
          "k2",
          fitted.lens[2],
          "views",
          fitted.views.size(),
          "tracks",
          fitted.samples.size(),
          "validation_rms_before_pixels",
          before,
          "validation_rms_after_pixels",
          after,
          "radial_coverage_p90_fraction",
          coverage,
          "marginal_information_eigenvalue_ratio",
          condition,
          "standard_deviations_log_f_k1_k2",
          standardDeviations,
          "iterations",
          iterations,
          "solver",
          "Huber(2 pixels) Levenberg-Marquardt with 3D point Schur elimination",
          "gauge",
          fitted.fixedPoses
              ? "supplied poses fixed"
              : "first pose fixed; reference baseline norm normalized without freezing second"
                  + " rotation",
          "acceptance_thresholds",
          Json.object(
              "minimum_views",
              4,
              "minimum_training_tracks",
              80,
              "minimum_held_out_tracks",
              20,
              "minimum_radial_coverage_fraction",
              .55,
              "minimum_information_ratio",
              1e-7,
              "maximum_log_f_standard_deviation",
              .08,
              "maximum_radial_standard_deviation_at_corner",
              .02,
              "minimum_held_out_relative_improvement",
              .05,
              "minimum_held_out_pixel_improvement",
              .03));
    }
  }

  static Calibration refine(Calibration initial, List<Path> paths, Options options, Project project)
      throws IOException {
    Path dir = options.output.resolve("01_calibration/refinement");
    Files.createDirectories(dir);
    options.calibrationPoses = Collections.emptyMap();
    if (project != null)
      project.stage(
          "01_calibration", "running", "Guarded joint focal/radial, pose and structure refinement");
    String skip =
        !options.selfCalibration
            ? "Disabled by --no-self-calibration"
            : options.rectified
                ? "Already-rectified inputs do not retain the original lens distortion"
                : paths.size() < 4
                    ? "At least four overlapping views required for guarded radial self-calibration"
                    : null;
    if (skip != null) {
      Json.write(
          dir.resolve("result.json"),
          Json.object("accepted", false, "reason", skip, "model", MODEL));
      return initial;
    }
    Options bootstrap = options.copy();
    bootstrap.seedPair = null;
    int[] finalSize = options.alignmentOptions().processedSize(initial.width, initial.height);
    bootstrap.quality = "custom";
    bootstrap.maxDimension = Math.min(Math.max(finalSize[0], finalSize[1]), 1200);
    bootstrap.reprojection = Math.max(4, options.reprojection);
    bootstrap.iterations = Math.max(8, options.iterations);
    List<ImageFrame> frames = new ArrayList<>();
    try {
      for (Path path : paths.subList(0, Math.min(12, paths.size())))
        frames.add(new ImageFrame(frames.size(), path, initial, bootstrap, true));
      FeatureTracks features = new FeatureTracks();
      features.build(frames, bootstrap);
      if (options.poses != null) {
        PoseIO.read(options.poses, frames, true);
        Geometry.triangulateTracks(frames, features.tracks, bootstrap.reprojection * 2);
      } else Geometry.reconstruct(frames, features, bootstrap);
      if (frames.stream().anyMatch(f -> !f.camera.registered))
        throw new IllegalArgumentException(
            "Incomplete calibration bootstrap; use the retained focal initialization for final"
                + " reconstruction");
      Data data =
          new Data(initial, options.poses != null && options.extrinsicsMode.equals("fixed"));
      data.controls = options.copy();
      if (options.calibration == null) data.controls.intrinsicsMode = "initial";
      data.priorLens = data.lens.clone();
      for (ImageFrame f : frames) {
        if (options.poses != null && options.extrinsicsMode.equals("prior")) {
          f.camera.reference = f.camera.copy();
          f.camera.positionSigma = options.positionSigma;
          f.camera.rotationSigma = Math.toRadians(options.rotationSigma);
        }
        data.views.add(new View(f.camera, f.camera.fx / initial.fx, f.camera.fy / initial.fx));
      }
      List<FeatureTracks.Track> usable = new ArrayList<>();
      for (FeatureTracks.Track t : features.tracks)
        if (t.point != null && t.observations.size() >= 3) usable.add(t);
      // Shuffle before capping, so selection does not privilege low keypoint IDs or image centers.
      Collections.shuffle(usable, new Random(91571));
      int id = 0;
      for (FeatureTracks.Track t : usable.subList(0, Math.min(1800, usable.size()))) {
        Sample sample = new Sample(t.id, t.point, id++ % 5 == 0);
        for (FeatureTracks.Observation ob : t.observations) {
          Point p = frames.get(ob.image).keypoints[ob.feature].pt;
          Camera cam = frames.get(ob.image).camera;
          double x = (p.x - cam.cx) / cam.fx, y = (p.y - cam.cy) / cam.fy, r = x * x + y * y;
          double radial =
              1
                  + initial.distortion[0] * r
                  + initial.distortion[1] * r * r
                  + initial.distortion[4] * r * r * r;
          double
              xd =
                  x * radial
                      + 2 * initial.distortion[2] * x * y
                      + initial.distortion[3] * (r + 2 * x * x),
              yd =
                  y * radial
                      + initial.distortion[2] * (r + 2 * y * y)
                      + 2 * initial.distortion[3] * x * y;
          sample.seen.add(new Seen(ob.image, cam.fx * xd + cam.cx, cam.fy * yd + cam.cy));
        }
        data.samples.add(sample);
      }
      Result result = optimize(data, 50);
      Json.write(dir.resolve("result.json"), result.report());
      evidence(
          dir, result.evaluatedBaseline == null ? data : result.evaluatedBaseline, result.fitted);
      if (!result.accepted) {
        System.out.println("Self-calibration retained initial intrinsics: " + result.reason);
        return initial;
      }
      double[] p = result.fitted.lens;
      double f = Math.exp(p[0]);
      Calibration calibrated =
          new Calibration(
              f,
              f * initial.fy / initial.fx,
              initial.cx,
              initial.cy,
              new double[] {
                p[1], p[2], initial.distortion[2], initial.distortion[3], initial.distortion[4]
              },
              initial.width,
              initial.height);
      options.calibrationPoses = new LinkedHashMap<>();
      for (ImageFrame frame : frames)
        options.calibrationPoses.put(
            frame.path.getFileName().toString(), result.fitted.views.get(frame.id).camera.copy());
      calibrated.write(
          options.output.resolve("camera.txt"),
          "Accepted joint f,k1,k2 self-calibration; " + MODEL);
      System.out.printf(
          Locale.ROOT,
          "Self-calibration accepted: f=%.3f px, k1=%.7f, k2=%.7f; held-out RMS %.4f -> %.4f px%n",
          f,
          p[1],
          p[2],
          result.before,
          result.after);
      return calibrated;
    } catch (IllegalArgumentException e) {
      Json.write(
          dir.resolve("result.json"),
          Json.object(
              "accepted",
              false,
              "reason",
              "Sparse bootstrap failed: " + e.getMessage(),
              "model",
              MODEL));
      System.out.println("Self-calibration retained initial intrinsics: " + e.getMessage());
      return initial;
    } finally {
      for (ImageFrame f : frames) f.close();
    }
  }

  static Result optimize(Data initial, int maxIterations) {
    Data fit = initial.copy();
    Result result = new Result(fit);
    int train = 0, validation = 0;
    List<Double> radii = new ArrayList<>();
    double corner = Math.hypot((initial.width - 1) / 2., (initial.height - 1) / 2.);
    for (Sample s : fit.samples) {
      if (s.validation) validation++;
      else train++;
      for (Seen ob : s.seen) {
        View v = fit.views.get(ob.view);
        radii.add(Math.hypot((ob.u - v.camera.cx) / v.sx, (ob.v - v.camera.cy) / v.sy) / corner);
      }
    }
    Collections.sort(radii);
    result.coverage = radii.isEmpty() ? 0 : radii.get((int) ((radii.size() - 1) * .9));
    if (fit.views.size() < 4 || train < 80 || validation < 20) {
      result.reason =
          "Insufficient independent multi-view tracks (need 80 training and 20 held-out tracks,"
              + " four views)";
      return result;
    }
    if (result.coverage < .55) {
      result.reason = "Features do not cover enough of the image radius";
      return result;
    }
    Data baseline = initial.copy();
    refineValidation(baseline);
    result.evaluatedBaseline = baseline;
    result.before = rms(baseline, true);
    double old = cost(fit, false), lambda = 1e-3;
    double baselineLength =
        fit.fixedPoses
            ? 1
            : Math3.norm(
                Math3.sub(fit.views.get(0).camera.center(), fit.views.get(1).camera.center()));
    if (!fit.fixedPoses && baselineLength < 1e-8) {
      result.reason = "Coincident reference camera centers";
      return result;
    }
    for (int iteration = 0; iteration < maxIterations; iteration++) {
      boolean improved = false;
      double improvement = 0;
      for (int trial = 0; trial < 8; trial++) {
        Normal normal = normal(fit, lambda);
        if (normal == null) {
          lambda *= 10;
          continue;
        }
        double[] delta = Math3.solve(normal.h, normal.b);
        if (delta == null) {
          lambda *= 10;
          continue;
        }
        Data candidate = fit.copy();
        apply(candidate, normal, delta, baselineLength);
        double value =
            physical(candidate, initial) ? cost(candidate, false) : Double.POSITIVE_INFINITY;
        if (value < old) {
          improvement = old - value;
          old = value;
          fit = candidate;
          lambda = Math.max(1e-10, lambda / 3);
          improved = true;
          result.iterations.add(
              Json.object(
                  "iteration",
                  iteration + 1,
                  "robust_training_objective",
                  old,
                  "focal_pixels",
                  Math.exp(fit.lens[0]),
                  "k1",
                  fit.lens[1],
                  "k2",
                  fit.lens[2],
                  "lambda",
                  lambda));
          break;
        }
        lambda = Math.min(1e12, lambda * 10);
      }
      if (!improved || improvement < 1e-8 * Math.max(1, old)) break;
    }
    // Result.fitted holds a stable object, so all consumers see the accepted candidate state.
    if (fit != result.fitted) {
      result.fitted.lens = fit.lens;
      result.fitted.views.clear();
      result.fitted.views.addAll(fit.views);
      result.fitted.samples.clear();
      result.fitted.samples.addAll(fit.samples);
    }
    fit = result.fitted;
    refineValidation(fit);
    result.after = rms(fit, true);
    if (!information(fit, result)) {
      result.reason =
          "Joint focal/distortion estimates are poorly constrained after marginalizing poses and 3D"
              + " points";
      return result;
    }
    if (!(result.after < result.before * .95 && result.before - result.after > .03)) {
      result.reason =
          "Held-out tracks do not improve by both 5 percent and 0.03 pixels; distortion refinement"
              + " not justified";
      return result;
    }
    if (!physical(fit, initial)) {
      result.reason = "Non-physical radial map or focal estimate outside admissible bounds";
      return result;
    }
    result.accepted = true;
    result.reason = "Observable joint fit with independent held-out improvement";
    return result;
  }

  private static void evidence(Path dir, Data before, Data after) throws IOException {
    try (BufferedWriter w = Files.newBufferedWriter(dir.resolve("observations.csv"))) {
      w.write(
          "track_id,held_out,image_id,observed_x,observed_y,initial_projected_x,initial_projected_y,refined_projected_x,refined_projected_y,initial_world_x,initial_world_y,initial_world_z,refined_world_x,refined_world_y,refined_world_z\n");
      for (int i = 0; i < before.samples.size(); i++) {
        Sample a = before.samples.get(i), b = after.samples.get(i);
        for (Seen ob : a.seen) {
          double[] p = project(before, ob.view, a.point), q = project(after, ob.view, b.point);
          w.write(
              a.id
                  + ","
                  + a.validation
                  + ","
                  + ob.view
                  + ","
                  + ob.u
                  + ","
                  + ob.v
                  + ","
                  + (p == null ? "," : p[0] + "," + p[1])
                  + ","
                  + (q == null ? "," : q[0] + "," + q[1])
                  + ","
                  + a.point[0]
                  + ","
                  + a.point[1]
                  + ","
                  + a.point[2]
                  + ","
                  + b.point[0]
                  + ","
                  + b.point[1]
                  + ","
                  + b.point[2]
                  + "\n");
        }
      }
    }
    List<Object> poses = new ArrayList<>();
    for (int i = 0; i < after.views.size(); i++) {
      View view = after.views.get(i);
      Camera c = view.camera;
      Camera original = before.views.get(i).camera;
      poses.add(
          Json.object(
              "image_id",
              i,
              "R",
              c.rotation,
              "t",
              c.translation,
              "center",
              c.center(),
              "initial_R",
              original.rotation,
              "initial_t",
              original.translation,
              "initial_lens_log_f_k1_k2",
              before.lens,
              "refined_lens_log_f_k1_k2",
              after.lens,
              "reference_to_processed_scale_xy",
              new double[] {view.sx, view.sy},
              "principal_point",
              new double[] {c.cx, c.cy},
              "image_coordinates",
              "processed distorted raster; before the final undistortion"));
    }
    Json.write(dir.resolve("bootstrap_exterior.json"), poses);
  }

  static double[] project(Data d, int view, double[] point) {
    View v = d.views.get(view);
    double[] q = v.camera.view(point);
    if (q[2] <= 1e-8) return null;
    double x = q[0] / q[2],
        y = q[1] / q[2],
        r2 = x * x + y * y,
        radial = 1 + d.lens[1] * r2 + d.lens[2] * r2 * r2 + d.extra[2] * r2 * r2 * r2,
        f = Math.exp(d.lens[0]);
    return new double[] {
      f * v.sx * (x * radial + 2 * d.extra[0] * x * y + d.extra[1] * (r2 + 2 * x * x))
          + v.camera.cx,
      f * v.sy * (y * radial + d.extra[0] * (r2 + 2 * y * y) + 2 * d.extra[1] * x * y) + v.camera.cy
    };
  }

  private static double[][] jacobian(Data d, int view, double[] point) {
    View v = d.views.get(view);
    double[] q = v.camera.view(point);
    double x = q[0] / q[2],
        y = q[1] / q[2],
        r = x * x + y * y,
        f = Math.exp(d.lens[0]),
        fx = f * v.sx,
        fy = f * v.sy;
    double radial = 1 + d.lens[1] * r + d.lens[2] * r * r + d.extra[2] * r * r * r,
        dx = 2 * x * (d.lens[1] + 2 * d.lens[2] * r + 3 * d.extra[2] * r * r),
        dy = 2 * y * (d.lens[1] + 2 * d.lens[2] * r + 3 * d.extra[2] * r * r);
    double ux = fx * (radial + x * dx + 2 * d.extra[0] * y + 6 * d.extra[1] * x),
        uy = fx * (x * dy + 2 * d.extra[0] * x + 2 * d.extra[1] * y),
        vx = fy * (y * dx + 2 * d.extra[0] * x + 2 * d.extra[1] * y),
        vy = fy * (radial + y * dy + 6 * d.extra[0] * y + 2 * d.extra[1] * x);
    return new double[][] {
      {ux / q[2], uy / q[2], -(ux * x + uy * y) / q[2]},
      {vx / q[2], vy / q[2], -(vx * x + vy * y) / q[2]}
    };
  }

  private static double loss(double e) {
    return e <= 2 ? .5 * e * e : 2 * (e - 1);
  }

  private static boolean free(Data d, int k) {
    return d.controls == null
        || !d.controls.intrinsicsMode.equals("fixed")
            && d.controls.refineIntrinsics.contains(k == 0 ? "f" : k == 1 ? "k1" : "k2");
  }

  private static double cost(Data d, boolean validation) {
    double sum = 0;
    for (Sample s : d.samples)
      if (s.validation == validation)
        for (Seen ob : s.seen) {
          double[] p = project(d, ob.view, s.point);
          if (p == null) return 1e20;
          sum += loss(Math.hypot(p[0] - ob.u, p[1] - ob.v));
        }
    if (!validation) {
      if (d.controls != null && d.controls.intrinsicsMode.equals("prior"))
        for (int k = 0; k < 3; k++)
          if (free(d, k)) {
            double sigma = k == 0 ? d.controls.focalSigma : d.controls.radialSigma,
                delta = (d.lens[k] - d.priorLens[k]) / sigma;
            sum += .5 * delta * delta;
          }
      for (View view : d.views) sum += BundleAdjuster.priorCost(view.camera);
    }
    return sum;
  }

  private static double rms(Data d, boolean validation) {
    long n = 0;
    for (Sample s : d.samples) if (s.validation == validation) n += s.seen.size();
    return Math.sqrt(2 * cost(d, validation) / Math.max(1, n));
  }

  private static final class Block {
    final Sample sample;
    final double[][] c, inverse;
    final double[] b;

    Block(Sample sample, double[][] c, double[][] inverse, double[] b) {
      this.sample = sample;
      this.c = c;
      this.inverse = inverse;
      this.b = b;
    }
  }

  private static final class Normal {
    final double[][] h;
    final double[] b;
    final List<Block> blocks = new ArrayList<>();

    Normal(int size) {
      h = new double[size][size];
      b = new double[size];
    }
  }

  private static Normal normal(Data d, double lambda) {
    int size = d.dimension();
    Normal out = new Normal(size);
    double[] diagonal = new double[size];
    for (Sample s : d.samples)
      if (!s.validation) {
        double[][] b = new double[3][3], c = new double[size][3];
        double[] bp = new double[3];
        for (Seen ob : s.seen) {
          View view = d.views.get(ob.view);
          Camera cam = view.camera;
          double[] prediction = project(d, ob.view, s.point);
          if (prediction == null) continue;
          double[] residual = {prediction[0] - ob.u, prediction[1] - ob.v}, q = cam.view(s.point);
          double weight = Math.min(1, 2 / Math.max(1e-10, Math.hypot(residual[0], residual[1])));
          double[][] jq = jacobian(d, ob.view, s.point),
              jp = new double[2][3],
              jg = new double[2][size];
          double x = q[0] / q[2], y = q[1] / q[2], r2 = x * x + y * y, f = Math.exp(d.lens[0]);
          jg[0][0] = prediction[0] - cam.cx;
          jg[1][0] = prediction[1] - cam.cy;
          jg[0][1] = f * view.sx * x * r2;
          jg[1][1] = f * view.sy * y * r2;
          jg[0][2] = jg[0][1] * r2;
          jg[1][2] = jg[1][1] * r2;
          for (int k = 0; k < 3; k++)
            if (!free(d, k)) {
              jg[0][k] = 0;
              jg[1][k] = 0;
            }
          int offset = d.poseOffset(ob.view);
          for (int row = 0; row < 2; row++) {
            for (int k = 0; k < 3; k++)
              for (int axis = 0; axis < 3; axis++)
                jp[row][k] += jq[row][axis] * cam.rotation[axis * 3 + k];
            if (offset >= 0) {
              jg[row][offset] = -jq[row][1] * q[2] + jq[row][2] * q[1];
              jg[row][offset + 1] = jq[row][0] * q[2] - jq[row][2] * q[0];
              jg[row][offset + 2] = -jq[row][0] * q[1] + jq[row][1] * q[0];
              for (int k = 0; k < 3; k++) jg[row][offset + 3 + k] = jq[row][k];
            }
          }
          for (int row = 0; row < 2; row++) {
            for (int i = 0; i < 3; i++) {
              bp[i] -= weight * jp[row][i] * residual[row];
              for (int j = 0; j < 3; j++) b[i][j] += weight * jp[row][i] * jp[row][j];
            }
            for (int i = 0; i < size; i++)
              if (jg[row][i] != 0) {
                out.b[i] -= weight * jg[row][i] * residual[row];
                diagonal[i] += weight * jg[row][i] * jg[row][i];
                for (int k = 0; k < 3; k++) c[i][k] += weight * jg[row][i] * jp[row][k];
                for (int j = 0; j < size; j++)
                  if (jg[row][j] != 0) out.h[i][j] += weight * jg[row][i] * jg[row][j];
              }
          }
        }
        for (int i = 0; i < 3; i++) b[i][i] += lambda * Math.max(1, b[i][i]);
        double[][] inv = inverse(b);
        if (inv == null) return null;
        double[][] ci = new double[size][3];
        for (int i = 0; i < size; i++)
          for (int j = 0; j < 3; j++) for (int k = 0; k < 3; k++) ci[i][j] += c[i][k] * inv[k][j];
        for (int i = 0; i < size; i++) {
          for (int k = 0; k < 3; k++) out.b[i] -= ci[i][k] * bp[k];
          for (int j = 0; j < size; j++)
            for (int k = 0; k < 3; k++) out.h[i][j] -= ci[i][k] * c[j][k];
        }
        out.blocks.add(new Block(s, c, inv, bp));
      }
    for (int k = 0; k < 3; k++) {
      if (!free(d, k)) {
        out.h[k][k] += 1;
        out.b[k] = 0;
      } else if (d.controls != null && d.controls.intrinsicsMode.equals("prior")) {
        double sigma = k == 0 ? d.controls.focalSigma : d.controls.radialSigma,
            w = 1 / (sigma * sigma);
        out.h[k][k] += w;
        out.b[k] -= w * (d.lens[k] - d.priorLens[k]);
      }
    }
    if (!d.fixedPoses)
      for (int i = d.hasPosePriors() ? 0 : 1; i < d.views.size(); i++) {
        double[] a = new double[36], b = new double[6];
        BundleAdjuster.priorNormal(d.views.get(i).camera, a, b);
        int offset = d.poseOffset(i);
        for (int j = 0; j < 6; j++) {
          out.b[offset + j] += b[j];
          for (int k = 0; k < 6; k++) out.h[offset + j][offset + k] += a[j * 6 + k];
        }
      }
    for (int i = 0; i < size; i++) out.h[i][i] += lambda * Math.max(1, diagonal[i]);
    return out;
  }

  private static double[][] inverse(double[][] matrix) {
    int n = matrix.length;
    double[][] inv = new double[n][n];
    for (int j = 0; j < n; j++) {
      double[] b = new double[n];
      b[j] = 1;
      double[] x = Math3.solve(matrix, b);
      if (x == null) return null;
      for (int i = 0; i < n; i++) inv[i][j] = x[i];
    }
    return inv;
  }

  private static void apply(Data d, Normal normal, double[] delta, double baseline) {
    for (int k = 0; k < 3; k++) if (free(d, k)) d.lens[k] += delta[k];
    if (!d.fixedPoses)
      for (int i = d.hasPosePriors() ? 0 : 1; i < d.views.size(); i++) {
        int offset = d.poseOffset(i);
        Camera c = d.views.get(i).camera;
        double[] r = Math3.rotation(Arrays.copyOfRange(delta, offset, offset + 3));
        c.rotation = Math3.mul(r, c.rotation);
        c.translation =
            Math3.add(
                Math3.mv(r, c.translation), Arrays.copyOfRange(delta, offset + 3, offset + 6));
      }
    Map<Integer, Sample> samples = new HashMap<>();
    for (Sample s : d.samples) samples.put(s.id, s);
    for (Block block : normal.blocks) {
      Sample sample = samples.get(block.sample.id);
      double[] b = block.b.clone();
      for (int k = 0; k < 3; k++)
        for (int j = 0; j < delta.length; j++) b[k] -= block.c[j][k] * delta[j];
      for (int k = 0; k < 3; k++)
        for (int j = 0; j < 3; j++) sample.point[k] += block.inverse[k][j] * b[j];
    }
    if (!d.fixedPoses && d.views.stream().noneMatch(v -> v.camera.reference != null)) {
      double[] origin = d.views.get(0).camera.center();
      double scale = baseline / Math3.norm(Math3.sub(d.views.get(1).camera.center(), origin));
      for (View view : d.views) {
        Camera c = view.camera;
        double[] center = Math3.add(origin, Math3.times(Math3.sub(c.center(), origin), scale));
        c.translation = Math3.times(Math3.mv(c.rotation, center), -1);
      }
      for (Sample s : d.samples)
        s.point = Math3.add(origin, Math3.times(Math3.sub(s.point, origin), scale));
    }
  }

  private static void refineValidation(Data d) {
    for (Sample s : d.samples)
      if (s.validation)
        for (int iter = 0; iter < 20; iter++) {
          double[][] h = new double[3][3];
          double[] b = new double[3];
          for (Seen ob : s.seen) {
            View view = d.views.get(ob.view);
            double[] p = project(d, ob.view, s.point);
            if (p == null) continue;
            double[][] jq = jacobian(d, ob.view, s.point), jp = new double[2][3];
            for (int row = 0; row < 2; row++)
              for (int k = 0; k < 3; k++)
                for (int j = 0; j < 3; j++)
                  jp[row][k] += jq[row][j] * view.camera.rotation[3 * j + k];
            double[] residual = {p[0] - ob.u, p[1] - ob.v};
            double weight = Math.min(1, 2 / Math.max(1e-10, Math.hypot(residual[0], residual[1])));
            for (int i = 0; i < 3; i++) {
              for (int row = 0; row < 2; row++) b[i] -= weight * jp[row][i] * residual[row];
              for (int j = 0; j < 3; j++)
                for (int row = 0; row < 2; row++) h[i][j] += weight * jp[row][i] * jp[row][j];
            }
          }
          for (int i = 0; i < 3; i++) h[i][i] += 1e-5 * Math.max(1, h[i][i]);
          double[] step = Math3.solve(h, b);
          if (step == null || Math3.norm(step) < 1e-9) break;
          s.point = Math3.add(s.point, step);
        }
  }

  private static boolean physical(Data d, Data initial) {
    double f = Math.exp(d.lens[0]);
    if (!Double.isFinite(f + d.lens[1] + d.lens[2])
        || f < Math.exp(initial.lens[0]) * .5
        || f > Math.exp(initial.lens[0]) * 2
        || Math.abs(d.lens[1]) > 2
        || Math.abs(d.lens[2]) > 3) return false;
    double corner2 = (Math.pow((d.width - 1) / 2., 2) + Math.pow((d.height - 1) / 2., 2)) / (f * f);
    for (int i = 0; i <= 50; i++) {
      double r2 = corner2 * i / 50.;
      if (1 + d.lens[1] * r2 + d.lens[2] * r2 * r2 + d.extra[2] * r2 * r2 * r2 < .3
          || 1 + 3 * d.lens[1] * r2 + 5 * d.lens[2] * r2 * r2 + 7 * d.extra[2] * r2 * r2 * r2 < .2)
        return false;
    }
    return true;
  }

  private static boolean information(Data d, Result result) {
    Normal normal = normal(d, 0);
    if (normal == null) return false;
    int size = d.dimension();
    double[][] info = new double[3][3];
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) info[i][j] = normal.h[i][j];
    if (size > 3) {
      Mat c = new Mat(size - 3, size - 3, CvType.CV_64F), inv = new Mat();
      try {
        double[] values = new double[(size - 3) * (size - 3)];
        for (int i = 3; i < size; i++)
          for (int j = 3; j < size; j++) values[(i - 3) * (size - 3) + j - 3] = normal.h[i][j];
        c.put(0, 0, values);
        Core.invert(c, inv, Core.DECOMP_SVD);
        double[] inverse = Math3.array(inv);
        for (int i = 0; i < 3; i++)
          for (int j = 0; j < 3; j++)
            for (int a = 3; a < size; a++)
              for (int b = 3; b < size; b++)
                info[i][j] -=
                    normal.h[i][a] * inverse[(a - 3) * (size - 3) + b - 3] * normal.h[b][j];
      } finally {
        c.release();
        inv.release();
      }
    }
    double f = Math.exp(d.lens[0]),
        r2 = (Math.pow((d.width - 1) / 2., 2) + Math.pow((d.height - 1) / 2., 2)) / (f * f);
    double[] scale = {1, 1 / r2, 1 / (r2 * r2)};
    List<Integer> active = new ArrayList<>();
    for (int i = 0; i < 3; i++) if (free(d, i)) active.add(i);
    if (active.isEmpty()) return false;
    int dimensions = active.size();
    double[] a = new double[dimensions * dimensions];
    for (int i = 0; i < dimensions; i++)
      for (int j = 0; j < dimensions; j++) {
        int x = active.get(i), y = active.get(j);
        a[dimensions * i + j] = (info[x][y] + info[y][x]) * .5 * scale[x] * scale[y];
      }
    Mat m = Math3.mat(a, dimensions, dimensions), eigen = new Mat();
    try {
      Core.eigen(m, eigen);
      double[] e = Math3.array(eigen);
      result.condition = e[dimensions - 1] / Math.max(1e-30, e[0]);
      if (!(result.condition > 1e-7)) return false;
    } finally {
      m.release();
      eigen.release();
    }
    double[][] covariance = inverse(info);
    if (covariance == null) return false;
    double sigma = Math.max(.35, rms(d, false));
    for (int i = 0; i < 3; i++)
      result.standardDeviations[i] =
          free(d, i) ? sigma * Math.sqrt(Math.max(0, covariance[i][i])) : 0;
    return (!free(d, 0) || result.standardDeviations[0] < .08)
        && (!free(d, 1) || result.standardDeviations[1] * r2 < .02)
        && (!free(d, 2) || result.standardDeviations[2] * r2 * r2 < .02);
  }
}
