package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import org.opencv.core.Point;

/** Joint robust LM with point elimination and a matrix-free, preconditioned Schur solve. */
final class BundleAdjuster {
  private BundleAdjuster() {}

  static void optimize(
      List<ImageFrame> frames, List<FeatureTracks.Track> tracks, int iterations, double huber) {
    try {
      optimize(frames, tracks, iterations, huber, null);
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  static void optimize(
      List<ImageFrame> frames,
      List<FeatureTracks.Track> tracks,
      int iterations,
      double huber,
      Project project)
      throws IOException {
    int anchor = -1, scale = -1;
    for (ImageFrame f : frames)
      if (f.camera.registered && f.camera.fixed) {
        if (anchor < 0) anchor = f.id;
        else if (scale < 0) scale = f.id;
      }
    if (anchor < 0)
      for (ImageFrame f : frames)
        if (f.camera.registered) {
          anchor = f.id;
          break;
        }
    if (scale < 0 && anchor >= 0)
      for (ImageFrame f : frames)
        if (f.camera.registered
            && f.id != anchor
            && Math3.norm(Math3.sub(f.camera.center(), frames.get(anchor).camera.center()))
                > 1e-9) {
          scale = f.id;
          break;
        }
    if (anchor < 0 || scale < 0) return;
    optimize(frames, tracks, iterations, huber, project, anchor, scale);
  }

  static void optimize(
      List<ImageFrame> frames,
      List<FeatureTracks.Track> tracks,
      int iterations,
      double huber,
      Project project,
      int anchor,
      int scaleCamera)
      throws IOException {
    try (WorkProgress work = WorkProgress.start("Joint bundle adjustment", 0, "iterations")) {
      optimizeImpl(frames, tracks, iterations, huber, project, anchor, scaleCamera);
      work.done();
    }
  }

  private static void optimizeImpl(
      List<ImageFrame> frames,
      List<FeatureTracks.Track> tracks,
      int iterations,
      double huber,
      Project project,
      int anchor,
      int scaleCamera)
      throws IOException {
    double[] origin = frames.get(anchor).camera.center();
    double baseline = Math3.norm(Math3.sub(frames.get(scaleCamera).camera.center(), origin));
    if (!(baseline > 1e-9) || !Double.isFinite(baseline))
      throw new IllegalArgumentException(
          "Bundle adjustment requires a nonzero finite seed baseline");
    int[] index = new int[frames.size()];
    Arrays.fill(index, -1);
    int count = 0;
    boolean absolutePriors =
        frames.stream().anyMatch(f -> f.camera.registered && f.camera.reference != null);
    for (ImageFrame f : frames) {
      if (f.camera.registered) {
        f.camera.fixed = !absolutePriors && f.id == anchor;
        if (!f.camera.fixed) index[f.id] = count++;
      }
    }
    double before = error(frames, tracks, huber), lambda = 1e-3;
    Path diagnostics = null;
    if (project != null) {
      Path dir = project.data.directory("04_geometry/bundle");
      diagnostics = dir.resolve("linear_solver.csv");
      Files.writeString(
          diagnostics,
          "iteration,trial,lambda,pcg_iterations,relative_linear_residual,objective_before,objective_trial,accepted\n");
      Json.write(
          dir.resolve("solver.json"),
          Json.object(
              "solver",
              "joint Levenberg-Marquardt; matrix-free Schur complement; block-Jacobi PCG",
              "requested_iterations",
              iterations,
              "huber_pixels",
              huber,
              "intrinsics_fixed",
              true,
              "anchor_image_id",
              anchor,
              "scale_image_id",
              scaleCamera,
              "seed_baseline",
              baseline,
              "variable_cameras",
              count,
              "gauge",
              absolutePriors
                  ? "supplied camera center/rotation uncertainty priors anchor the world frame"
                  : "first seed pose fixed; seed baseline length preserved by similarity"
                      + " normalization; second seed orientation and direction free",
              "linear_relative_tolerance",
              1e-6,
              "maximum_linear_iterations",
              Math.min(500, Math.max(30, count * 12)),
              "track_limit",
              null,
              "note",
              "All triangulated tracks and registered observations participate. Huber weights are"
                  + " recomputed each outer iteration; existing post-bundle track rejection follows"
                  + " this solve. Workspace scales with observations and cameras, not camera count"
                  + " squared."));
      project.data.bundle(0, before, before, frames, tracks);
    }
    for (int iteration = 0; iteration < iterations; iteration++) {
      boolean accepted = false;
      double after = before;
      Linear system = null;
      for (int trial = 0; trial < 8; trial++) {
        system = linearize(frames, tracks, index, count, huber, lambda);
        Result result = system.solve();
        Camera[] old = new Camera[frames.size()];
        for (ImageFrame f : frames) old[f.id] = f.camera.copy();
        List<double[]> oldPoints = new ArrayList<>(system.points.size());
        for (Block p : system.points) oldPoints.add(p.track.point);
        if (result != null) {
          for (ImageFrame f : frames)
            if (index[f.id] >= 0) move(f.camera, result.x, index[f.id] * 6);
          for (Block p : system.points) {
            double[] rhs = p.b.clone();
            for (Edge e : p.edges)
              for (int k = 0; k < 3; k++)
                for (int j = 0; j < 6; j++)
                  rhs[k] -= e.cross[j * 3 + k] * result.x[e.camera * 6 + j];
            p.track.point = Math3.add(p.track.point, mv(p.inverse, rhs, 3));
          }
          double newLength = Math3.norm(Math3.sub(frames.get(scaleCamera).camera.center(), origin));
          if (newLength > 1e-12 && Double.isFinite(newLength)) {
            double factor = absolutePriors ? 1 : baseline / newLength;
            for (ImageFrame f : frames)
              if (f.camera.registered && (absolutePriors || f.id != anchor)) {
                double[] center =
                    Math3.add(origin, Math3.times(Math3.sub(f.camera.center(), origin), factor));
                f.camera.translation = Math3.times(Math3.mv(f.camera.rotation, center), -1);
              }
            for (Block p : system.points)
              p.track.point =
                  Math3.add(origin, Math3.times(Math3.sub(p.track.point, origin), factor));
            after = error(frames, tracks, huber);
          } else after = Double.POSITIVE_INFINITY;
          accepted = Double.isFinite(after) && after < before;
        } else after = Double.POSITIVE_INFINITY;
        if (diagnostics != null)
          Files.writeString(
              diagnostics,
              (iteration + 1)
                  + ","
                  + trial
                  + ","
                  + lambda
                  + ","
                  + (result == null ? 0 : result.iterations)
                  + ","
                  + (result == null ? Double.NaN : result.relative)
                  + ","
                  + before
                  + ","
                  + after
                  + ","
                  + accepted
                  + "\n",
              StandardOpenOption.APPEND);
        if (accepted) {
          lambda = Math.max(1e-10, lambda / 3);
          break;
        }
        for (ImageFrame f : frames) {
          f.camera.rotation = old[f.id].rotation;
          f.camera.translation = old[f.id].translation;
        }
        for (int p = 0; p < system.points.size(); p++)
          system.points.get(p).track.point = oldPoints.get(p);
        lambda = Math.min(1e12, lambda * 10);
      }
      if (!accepted) {
        System.out.println("Bundle: no improving joint step; stopping.");
        break;
      }
      if (project != null) project.data.bundle(iteration + 1, before, after, frames, tracks);
      System.out.printf(
          Locale.ROOT,
          "Bundle iteration %d: robust objective %.5f -> %.5f; joint cameras=%d, points=%d%n",
          iteration + 1,
          before,
          after,
          count,
          system.points.size());
      if (before - after < 1e-7 * Math.max(1, before)) break;
      before = after;
    }
  }

  static double error(List<ImageFrame> frames, List<FeatureTracks.Track> tracks, double huber) {
    double total = 0;
    for (FeatureTracks.Track t : tracks)
      if (t.point != null)
        for (FeatureTracks.Observation o : t.observations) {
          ImageFrame f = frames.get(o.image);
          if (!f.camera.registered) continue;
          double[] uv = f.camera.project(t.point);
          Point observed = f.keypoints[o.feature].pt;
          if (uv == null || !Double.isFinite(uv[0]) || !Double.isFinite(uv[1]))
            return Double.POSITIVE_INFINITY;
          double e = Math.hypot(uv[0] - observed.x, uv[1] - observed.y);
          total += e <= huber ? .5 * e * e : huber * (e - .5 * huber);
        }
    for (ImageFrame f : frames) if (f.camera.registered) total += priorCost(f.camera);
    return total;
  }

  static double priorCost(Camera c) {
    if (c.reference == null) return 0;
    double position = Math3.norm(Math3.sub(c.center(), c.reference.center())),
        rotation =
            Math3.norm(
                Math3.rotationVector(Math3.mul(c.rotation, Math3.transpose(c.reference.rotation))));
    return .5
        * (position * position / (c.positionSigma * c.positionSigma)
            + rotation * rotation / (c.rotationSigma * c.rotationSigma));
  }

  static void priorNormal(Camera c, double[] a, double[] b) {
    if (c.reference == null) return;
    double[] position = Math3.sub(c.center(), c.reference.center()),
        rotation =
            Math3.rotationVector(Math3.mul(c.rotation, Math3.transpose(c.reference.rotation)));
    double[][] jp = new double[3][6], jr = new double[3][6];
    double eps = 1e-6;
    for (int axis = 0; axis < 3; axis++) {
      for (int k = 0; k < 3; k++) jp[axis][k + 3] = -c.rotation[k * 3 + axis] / c.positionSigma;
      double[] w = new double[3];
      w[axis] = eps;
      double[] r =
          Math3.rotationVector(
              Math3.mul(
                  Math3.mul(Math3.rotation(w), c.rotation), Math3.transpose(c.reference.rotation)));
      for (int k = 0; k < 3; k++) jr[k][axis] = (r[k] - rotation[k]) / eps / c.rotationSigma;
    }
    for (int i = 0; i < 6; i++)
      for (int axis = 0; axis < 3; axis++) {
        b[i] -=
            jp[axis][i] * position[axis] / c.positionSigma
                + jr[axis][i] * rotation[axis] / c.rotationSigma;
        for (int j = 0; j < 6; j++)
          a[i * 6 + j] += jp[axis][i] * jp[axis][j] + jr[axis][i] * jr[axis][j];
      }
  }

  private static void move(Camera c, double[] delta, int offset) {
    double[] r = Math3.rotation(Arrays.copyOfRange(delta, offset, offset + 3));
    c.rotation = Math3.mul(r, c.rotation);
    c.translation =
        Math3.add(Math3.mv(r, c.translation), Arrays.copyOfRange(delta, offset + 3, offset + 6));
  }

  private static final class Edge {
    final int camera;
    final double[] cross = new double[18];

    Edge(int camera) {
      this.camera = camera;
    }
  }

  private static final class Block {
    final FeatureTracks.Track track;
    final double[] normal = new double[9], b = new double[3];
    double[] inverse;
    final List<Edge> edges = new ArrayList<>();

    Block(FeatureTracks.Track track) {
      this.track = track;
    }
  }

  private record Result(double[] x, int iterations, double relative) {}

  private static Linear linearize(
      List<ImageFrame> frames,
      List<FeatureTracks.Track> tracks,
      int[] index,
      int count,
      double huber,
      double lambda) {
    Linear s = new Linear(count);
    for (FeatureTracks.Track t : tracks)
      if (t.point != null) {
        Block block = new Block(t);
        for (FeatureTracks.Observation o : t.observations) {
          ImageFrame f = frames.get(o.image);
          Camera c = f.camera;
          if (!c.registered) continue;
          double[] q = c.view(t.point);
          if (q[2] <= 1e-8) continue;
          Point obs = f.keypoints[o.feature].pt;
          double[] residual = {
            c.fx * q[0] / q[2] + c.cx - obs.x, c.fy * q[1] / q[2] + c.cy - obs.y
          };
          double weight =
              Math.min(1, huber / Math.max(1e-12, Math.hypot(residual[0], residual[1])));
          double[][] jq = {
            {c.fx / q[2], 0, -c.fx * q[0] / (q[2] * q[2])},
            {0, c.fy / q[2], -c.fy * q[1] / (q[2] * q[2])}
          };
          double[][] jp = new double[2][3], jc = new double[2][6];
          for (int row = 0; row < 2; row++) {
            for (int j = 0; j < 3; j++)
              for (int k = 0; k < 3; k++) jp[row][j] += jq[row][k] * c.rotation[k * 3 + j];
            jc[row][0] = -jq[row][1] * q[2] + jq[row][2] * q[1];
            jc[row][1] = jq[row][0] * q[2] - jq[row][2] * q[0];
            jc[row][2] = -jq[row][0] * q[1] + jq[row][1] * q[0];
            for (int j = 0; j < 3; j++) jc[row][j + 3] = jq[row][j];
          }
          accumulate(block.normal, block.b, jp, residual, weight);
          int ci = index[o.image];
          if (ci < 0) continue;
          accumulate(s.cameraNormal[ci], s.cameraRhs[ci], jc, residual, weight);
          Edge e = new Edge(ci);
          for (int j = 0; j < 6; j++)
            for (int k = 0; k < 3; k++)
              e.cross[j * 3 + k] = weight * (jc[0][j] * jp[0][k] + jc[1][j] * jp[1][k]);
          block.edges.add(e);
        }
        damp(block.normal, 3, lambda);
        block.inverse = inverse(block.normal, 3);
        if (block.inverse == null) throw new IllegalStateException("Singular damped point block");
        s.points.add(block);
      }
    for (ImageFrame f : frames)
      if (index[f.id] >= 0)
        priorNormal(f.camera, s.cameraNormal[index[f.id]], s.cameraRhs[index[f.id]]);
    for (int i = 0; i < count; i++) damp(s.cameraNormal[i], 6, lambda);
    return s;
  }

  private static void accumulate(double[] a, double[] b, double[][] j, double[] r, double w) {
    int n = b.length;
    for (int x = 0; x < n; x++) {
      b[x] -= w * (j[0][x] * r[0] + j[1][x] * r[1]);
      for (int y = 0; y < n; y++) a[x * n + y] += w * (j[0][x] * j[0][y] + j[1][x] * j[1][y]);
    }
  }

  private static void damp(double[] a, int n, double lambda) {
    for (int i = 0; i < n; i++) a[i * n + i] += lambda * Math.max(1, a[i * n + i]);
  }

  private static double[] inverse(double[] a, int n) {
    double[][] matrix = new double[n][n];
    for (int i = 0; i < n; i++) System.arraycopy(a, i * n, matrix[i], 0, n);
    double[] out = new double[n * n];
    for (int j = 0; j < n; j++) {
      double[] b = new double[n];
      b[j] = 1;
      double[] x = Math3.solve(matrix, b);
      if (x == null) return null;
      for (int i = 0; i < n; i++) out[i * n + j] = x[i];
    }
    return out;
  }

  private static double[] mv(double[] a, double[] x, int n) {
    double[] out = new double[n];
    for (int i = 0; i < n; i++) for (int j = 0; j < n; j++) out[i] += a[i * n + j] * x[j];
    return out;
  }

  private static double dot(double[] a, double[] b) {
    double v = 0;
    for (int i = 0; i < a.length; i++) v += a[i] * b[i];
    return v;
  }

  private static final class Linear {
    final double[][] cameraNormal, cameraRhs;
    final List<Block> points = new ArrayList<>();
    final int count;

    Linear(int n) {
      count = n;
      cameraNormal = new double[n][36];
      cameraRhs = new double[n][6];
    }

    double[] product(double[] x) {
      double[] out = new double[x.length];
      for (int i = 0; i < count; i++)
        for (int j = 0; j < 6; j++)
          for (int k = 0; k < 6; k++) out[i * 6 + j] += cameraNormal[i][j * 6 + k] * x[i * 6 + k];
      for (Block b : points) {
        double[] v = new double[3];
        for (Edge e : b.edges)
          for (int k = 0; k < 3; k++)
            for (int j = 0; j < 6; j++) v[k] += e.cross[j * 3 + k] * x[e.camera * 6 + j];
        double[] u = mv(b.inverse, v, 3);
        for (Edge e : b.edges)
          for (int j = 0; j < 6; j++)
            for (int k = 0; k < 3; k++) out[e.camera * 6 + j] -= e.cross[j * 3 + k] * u[k];
      }
      return out;
    }

    Result solve() {
      int n = count * 6;
      double[] rhs = new double[n];
      double[][] diag = new double[count][];
      for (int c = 0; c < count; c++) {
        System.arraycopy(cameraRhs[c], 0, rhs, c * 6, 6);
        diag[c] = cameraNormal[c].clone();
      }
      for (Block b : points) {
        double[] v = mv(b.inverse, b.b, 3);
        for (Edge e : b.edges)
          for (int j = 0; j < 6; j++) {
            for (int k = 0; k < 3; k++) rhs[e.camera * 6 + j] -= e.cross[j * 3 + k] * v[k];
            for (int l = 0; l < 6; l++)
              for (int k = 0; k < 3; k++)
                for (int m = 0; m < 3; m++)
                  diag[e.camera][j * 6 + l] -=
                      e.cross[j * 3 + k] * b.inverse[k * 3 + m] * e.cross[l * 3 + m];
          }
      }
      for (int c = 0; c < count; c++) {
        diag[c] = inverse(diag[c], 6);
        if (diag[c] == null) return null;
      }
      double[] x = new double[n], r = rhs.clone(), z = precondition(diag, r), p = z.clone();
      double norm = Math.sqrt(dot(rhs, rhs));
      if (norm < 1e-15) return new Result(x, 0, 0);
      double rz = dot(r, z), relative = 1;
      int limit = Math.min(500, Math.max(30, n * 2));
      for (int iter = 0; iter < limit; iter++) {
        double[] ap = product(p);
        double curvature = dot(p, ap);
        if (!(curvature > 0) || !Double.isFinite(curvature)) return null;
        double alpha = rz / curvature;
        for (int i = 0; i < n; i++) {
          x[i] += alpha * p[i];
          r[i] -= alpha * ap[i];
        }
        relative = Math.sqrt(dot(r, r)) / norm;
        if (!Double.isFinite(relative)) return null;
        if (relative < 1e-6) return new Result(x, iter + 1, relative);
        z = precondition(diag, r);
        double next = dot(r, z), beta = next / rz;
        for (int i = 0; i < n; i++) p[i] = z[i] + beta * p[i];
        rz = next;
      }
      // An inexact step is still checked against the full robust objective before acceptance.
      return new Result(x, limit, relative);
    }

    private double[] precondition(double[][] inverse, double[] r) {
      double[] z = new double[r.length];
      for (int c = 0; c < count; c++)
        for (int j = 0; j < 6; j++)
          for (int k = 0; k < 6; k++) z[c * 6 + j] += inverse[c][j * 6 + k] * r[c * 6 + k];
      return z;
    }
  }
}
