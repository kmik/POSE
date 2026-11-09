package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;

/** Joint robust LM with point elimination and bounded direct / matrix-free Schur solves. */
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
      boolean fixedPoses =
          frames.get(0).options.alignmentCalibration
              && frames.get(0).options.poses != null
              && frames.get(0).options.extrinsicsMode.equals("fixed");
      if (!fixedPoses)
        optimizeImpl(frames, tracks, iterations, huber, project, anchor, scaleCamera, null);
      SharedCalibration.refine(frames, tracks, iterations, huber, project, anchor, scaleCamera);
      work.done();
    }
  }

  static void optimizeImpl(
      List<ImageFrame> frames,
      List<FeatureTracks.Track> tracks,
      int iterations,
      double huber,
      Project project,
      int anchor,
      int scaleCamera,
      SharedCalibration lens)
      throws IOException {
    String label = lens == null ? "poses/points" : "shared " + String.join(",", lens.parameters());
    try (WorkProgress work = WorkProgress.start("Bundle fit " + label, 0, "iterations")) {
      optimizeFit(frames, tracks, iterations, huber, project, anchor, scaleCamera, lens, work);
      work.done();
    }
  }

  private static void optimizeFit(
      List<ImageFrame> frames,
      List<FeatureTracks.Track> tracks,
      int iterations,
      double huber,
      Project project,
      int anchor,
      int scaleCamera,
      SharedCalibration lens,
      WorkProgress work)
      throws IOException {
    double[] origin = frames.get(anchor).camera.center();
    double baseline = Math3.norm(Math3.sub(frames.get(scaleCamera).camera.center(), origin));
    if (!(baseline > 1e-9) || !Double.isFinite(baseline))
      throw new IllegalArgumentException(
          "Bundle adjustment requires a nonzero finite seed baseline");
    int[] index = new int[frames.size()];
    Arrays.fill(index, -1);
    int count = 0;
    // Camera position priors (GPS) or known track positions (GCPs) fix the world frame.
    boolean absolutePriors =
        frames.stream().anyMatch(f -> f.camera.registered && f.camera.reference != null)
            || tracks.stream().anyMatch(t -> t.prior != null && t.point != null);
    for (ImageFrame f : frames) {
      if (f.camera.registered) {
        f.camera.fixed = lens != null && lens.fixedPoses || !absolutePriors && f.id == anchor;
        if (!f.camera.fixed) index[f.id] = count++;
      }
    }
    AlignmentMemory.checkBundle(frames, tracks, index, count, lens != null);
    double before = error(frames, tracks, huber, lens), lambda = 1e-3;
    Path diagnostics = null;
    if (project != null) {
      Path dir = project.data.directory("04_geometry/bundle");
      diagnostics = dir.resolve("linear_solver.csv");
      Files.writeString(
          diagnostics,
          "iteration,trial,lambda,pcg_iterations,relative_linear_residual,objective_before,objective_trial,accepted,linear_method\n");
      Json.write(
          dir.resolve("solver.json"),
          Json.object(
              "solver",
              "joint Levenberg-Marquardt; bounded direct Schur Cholesky; matrix-free block-Jacobi"
                  + " PCG fallback",
              "requested_iterations",
              iterations,
              "iteration_mode",
              frames.get(0).options.bundleIterationsExplicit
                  ? "fixed budget (--bundle-iterations)"
                  : "until two successive gains < 1e-6 relative, at most " + AUTO_LIMIT,
              "huber_pixels",
              huber,
              "intrinsics_fixed",
              lens == null,
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
                  + " this solve. Direct assembly is restricted to at most 768 reduced variables"
                  + " and a bounded workspace; larger blocks retain matrix-free PCG."));
      project.data.bundle(0, before, before, frames, tracks);
    }
    // Without --bundle-iterations: iterate until two successive accepted iterations each gain
    // less than CONVERGED (relative), capped at AUTO_LIMIT.
    boolean converge = !frames.get(0).options.bundleIterationsExplicit;
    int limit = converge ? Math.max(iterations, AUTO_LIMIT) : iterations, small = 0;
    for (int iteration = 0; iteration < limit; iteration++) {
      boolean accepted = false;
      double after = before;
      Linear system = null;
      // Rejected trials restore the exact previous parameters, so one linearization serves every
      // damping trial of this iteration.
      work.detail("iteration " + (iteration + 1) + "; linearizing " + count + " cameras");
      Linear undamped = linearize(frames, tracks, index, count, huber, lens);
      for (int trial = 0; trial < 8; trial++) {
        system = undamped.damped(lambda);
        work.detail(
            "iteration "
                + (iteration + 1)
                + ", trial "
                + (trial + 1)
                + "; solving "
                + system.points.size()
                + " point blocks");
        Result result = system.solve();
        work.detail(
            "iteration "
                + (iteration + 1)
                + "; "
                + (result == null
                    ? "linear solve failed"
                    : result.method + ", residual " + result.relative)
                + "; checking objective");
        Camera[] old = new Camera[frames.size()];
        for (ImageFrame f : frames) old[f.id] = f.camera.copy();
        List<double[]> oldPoints = new ArrayList<>(system.points.size());
        for (Block p : system.points) oldPoints.add(p.track.point);
        double[] oldLens = lens == null ? null : lens.values.clone();
        if (result != null) {
          if (lens != null) lens.move(result.x, count * 6);
          for (ImageFrame f : frames)
            if (index[f.id] >= 0) move(f.camera, result.x, index[f.id] * 6);
          // Independent per point (identical arithmetic, any order).
          List<Block> blocks = system.points;
          double[] step = result.x;
          int cameras = count;
          int pointChunks = chunks(blocks.size(), 0);
          parallel(pointChunks, chunk -> {
            int from = (int) ((long) blocks.size() * chunk / pointChunks), to = (int) ((long) blocks.size() * (chunk + 1) / pointChunks);
            for (int i = from; i < to; i++) {
              Block p = blocks.get(i);
              double[] rhs = p.b.clone();
              for (Edge e : p.edges)
                for (int k = 0; k < 3; k++)
                  for (int j = 0; j < 6; j++)
                    rhs[k] -= e.cross[j * 3 + k] * step[e.camera * 6 + j];
              if (lens != null)
                for (int k = 0; k < 3; k++)
                  for (int j = 0; j < SharedCalibration.SIZE; j++)
                    rhs[k] -= p.lensCross[j * 3 + k] * step[cameras * 6 + j];
              p.track.point = Math3.add(p.track.point, mv(p.inverse, rhs, 3));
            }
          });
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
            after =
                lens != null && !lens.physical()
                    ? Double.POSITIVE_INFINITY
                    : error(frames, tracks, huber, lens);
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
                  + ","
                  + (result == null ? "failed" : result.method)
                  + "\n",
              StandardOpenOption.APPEND);
        if (accepted) {
          lambda = Math.max(1e-10, lambda / 3);
          break;
        }
        if (lens != null) lens.values = oldLens;
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
      work.update(iteration + 1);
      if (converge) {
        small = before - after < CONVERGED * Math.max(1, before) ? small + 1 : 0;
        if (small >= 2) {
          System.out.printf(
              Locale.ROOT, "Bundle converged after %d iterations.%n", iteration + 1);
          break;
        }
        if (iteration + 1 == limit)
          System.out.printf(
              Locale.ROOT,
              "Bundle: iteration limit %d reached while still improving; stopping.%n",
              limit);
      } else if (before - after < 1e-7 * Math.max(1, before)) break;
      before = after;
    }
  }

  static final int AUTO_LIMIT = 100;
  static final double CONVERGED = 1e-6;

  static double error(List<ImageFrame> frames, List<FeatureTracks.Track> tracks, double huber) {
    return error(frames, tracks, huber, null);
  }

  /** Robust reprojection objective plus camera and shared-parameter priors. */
  static double objective(
      List<ImageFrame> frames,
      List<FeatureTracks.Track> tracks,
      double huber,
      SharedCalibration lens) {
    return error(frames, tracks, huber, lens);
  }

  private static double error(
      List<ImageFrame> frames,
      List<FeatureTracks.Track> tracks,
      double huber,
      SharedCalibration lens) {
    double total = lens == null ? 0 : lens.priorCost();
    // Per-observation robust costs in parallel (pure), then added serially in the original order:
    // the result is bit-identical to the serial loop.
    for (int begin = 0; begin < tracks.size(); ) {
      int end = linearizationBatchEnd(tracks, begin, linearizationObservationLimit(frames.get(0).options, lens != null));
      List<FeatureTracks.Track> batch = tracks.subList(begin, end);
      int n = batch.size();
      int[] start = observationOffsets(batch);
      double[] costs = new double[start[n]];
      int chunks = chunks(n, 0);
      parallel(chunks, chunk -> {
        int from = (int) ((long) n * chunk / chunks), to = (int) ((long) n * (chunk + 1) / chunks);
        for (int i = from; i < to; i++) {
          FeatureTracks.Track t = batch.get(i);
          if (t.point == null) continue;
          int at = start[i];
          for (FeatureTracks.Observation o : t.observations) {
            ImageFrame f = frames.get(o.image);
            if (!f.camera.registered) {
              costs[at++] = Double.NaN; // skipped
              continue;
            }
            Point observed =
                lens == null ? f.keypoints[o.feature].pt : lens.observed[f.id][o.feature];
            double[] uv =
                lens == null ? f.camera.project(t.point) : lens.project(f, t.point, observed.y);
            if (uv == null || !Double.isFinite(uv[0]) || !Double.isFinite(uv[1])) {
              costs[at++] = Double.POSITIVE_INFINITY;
              continue;
            }
            double e = Math.hypot(uv[0] - observed.x, uv[1] - observed.y);
            costs[at++] = e <= huber ? .5 * e * e : huber * (e - .5 * huber);
          }
        }
      });
      for (double c : costs) {
        if (Double.isNaN(c)) continue;
        if (c == Double.POSITIVE_INFINITY) return Double.POSITIVE_INFINITY;
        total += c;
      }
      begin = end;
    }
    boolean antenna = lens != null && lens.antennaModel();
    for (ImageFrame f : frames)
      if (f.camera.registered)
        total += antenna ? antennaPriorCost(f.camera, lens) : priorCost(f.camera);
    for (FeatureTracks.Track t : tracks)
      if (t.point != null && t.prior != null)
        for (int k = 0; k < 3; k++) total += .5 * Math.pow((t.point[k] - t.prior[k]) / t.priorSigma[k], 2);
    return total;
  }

  /** Camera prior whose position term compares the GPS antenna (C + R^T L + V dt) to the fix. */
  static double antennaPriorCost(Camera c, SharedCalibration lens) {
    if (c.reference == null) return 0;
    double[] position = Math3.sub(lens.antenna(c), c.reference.center()), sigma = positionSigmas(c);
    double rotation =
        Math3.norm(
            Math3.rotationVector(Math3.mul(c.rotation, Math3.transpose(c.reference.rotation))));
    double cost = rotation * rotation / (c.rotationSigma * c.rotationSigma);
    for (int axis = 0; axis < 3; axis++) cost += Math.pow(position[axis] / sigma[axis], 2);
    return .5 * cost;
  }

  /**
   * Normal-equation terms of the antenna prior. Under the left pose update (R <- exp(w) R, t <-
   * exp(w) t + d): dC/dd = -R^T, dC/dw = 0 and d(R^T L)/dw = R^T [L]x; d/d dt = V, d/dL = R^T.
   */
  private static void antennaPriorNormal(Camera c, int ci, Linear s, SharedCalibration lens) {
    if (c.reference == null) return;
    int n = SharedCalibration.SIZE, time = SharedCalibration.TIME, lever = SharedCalibration.LEVER;
    double[] residual = Math3.sub(lens.antenna(c), c.reference.center()), sigma = positionSigmas(c);
    double[] rt = Math3.transpose(c.rotation);
    double lx = lens.values[lever], ly = lens.values[lever + 1], lz = lens.values[lever + 2];
    double[] cross = {0, -lz, ly, lz, 0, -lx, -ly, lx, 0};
    for (int axis = 0; axis < 3; axis++) {
      double[] jc = new double[6], jl = new double[n];
      for (int col = 0; col < 3; col++)
        for (int k = 0; k < 3; k++) jc[col] += rt[axis * 3 + k] * cross[k * 3 + col];
      for (int k = 0; k < 3; k++) jc[3 + k] = -c.rotation[k * 3 + axis];
      if (lens.active[time]) jl[time] = c.velocity == null ? 0 : c.velocity[axis];
      for (int k = 0; k < 3; k++) if (lens.active[lever + k]) jl[lever + k] = rt[axis * 3 + k];
      double r = residual[axis] / sigma[axis];
      for (int i = 0; i < 6; i++) jc[i] /= sigma[axis];
      for (int j = 0; j < n; j++) jl[j] /= sigma[axis];
      for (int j = 0; j < n; j++) {
        s.lensRhs[j] -= jl[j] * r;
        for (int k = 0; k < n; k++) s.lensNormal[j * n + k] += jl[j] * jl[k];
      }
      if (ci < 0) continue;
      for (int i = 0; i < 6; i++) {
        s.cameraRhs[ci][i] -= jc[i] * r;
        for (int k = 0; k < 6; k++) s.cameraNormal[ci][i * 6 + k] += jc[i] * jc[k];
        for (int j = 0; j < n; j++) s.coupling[ci][i * n + j] += jc[i] * jl[j];
      }
    }
    if (ci >= 0) {
      // Rotation term only (the position term above replaces the plain one).
      Camera rotationOnly = c.copy();
      rotationOnly.positionSigma = rotationOnly.heightSigma = Double.POSITIVE_INFINITY;
      priorNormal(rotationOnly, s.cameraNormal[ci], s.cameraRhs[ci]);
    }
  }

  static double priorCost(Camera c) {
    if (c.reference == null) return 0;
    double[] position = Math3.sub(c.center(), c.reference.center()), sigma = positionSigmas(c);
    double rotation =
        Math3.norm(
            Math3.rotationVector(Math3.mul(c.rotation, Math3.transpose(c.reference.rotation))));
    double cost = rotation * rotation / (c.rotationSigma * c.rotationSigma);
    for (int axis = 0; axis < 3; axis++) cost += Math.pow(position[axis] / sigma[axis], 2);
    return .5 * cost;
  }

  /** Per-axis world sigma of the camera-centre prior (z may differ, e.g. barometric heights). */
  private static double[] positionSigmas(Camera c) {
    double z = Double.isFinite(c.heightSigma) ? c.heightSigma : c.positionSigma;
    return new double[] {c.positionSigma, c.positionSigma, z};
  }

  static void priorNormal(Camera c, double[] a, double[] b) {
    if (c.reference == null) return;
    double[] position = Math3.sub(c.center(), c.reference.center()),
        rotation =
            Math3.rotationVector(Math3.mul(c.rotation, Math3.transpose(c.reference.rotation)));
    double[][] jp = new double[3][6], jr = new double[3][6];
    double[] sigma = positionSigmas(c);
    double eps = 1e-6;
    for (int axis = 0; axis < 3; axis++) {
      for (int k = 0; k < 3; k++) jp[axis][k + 3] = -c.rotation[k * 3 + axis] / sigma[axis];
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
            jp[axis][i] * position[axis] / sigma[axis]
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
    final double[] normal, b;
    double[] inverse;
    final double[] lensCross;
    final List<Edge> edges;

    Block(FeatureTracks.Track track, boolean shared) {
      this.track = track;
      normal = new double[9];
      b = new double[3];
      lensCross = shared ? new double[SharedCalibration.SIZE * 3] : null;
      edges = new ArrayList<>(track.observations.size());
    }

    /** Damped copy: own point normal and inverse; Jacobian products are shared read-only. */
    Block(Block undamped, double lambda) {
      track = undamped.track;
      normal = undamped.normal.clone();
      b = undamped.b;
      lensCross = undamped.lensCross;
      edges = undamped.edges;
      damp(normal, 3, lambda);
      inverse = BundleAdjuster.inverse(normal, 3);
      if (inverse == null) throw new IllegalStateException("Singular damped point block");
    }
  }

  private record Result(double[] x, int iterations, double relative, String method) {}

  /** Undamped system, then damped by lambda (the previous single-step behaviour). */
  private static Linear linearize(
      List<ImageFrame> frames,
      List<FeatureTracks.Track> tracks,
      int[] index,
      int count,
      double huber,
      double lambda,
      SharedCalibration lens) {
    return linearize(frames, tracks, index, count, huber, lens).damped(lambda);
  }

  /** One observation's Jacobians for the shared camera/lens sums. */
  private record ObservationTerms(int camera, double[][] jc, double[][] jl, double[] residual, double weight) {}

  /** A track's point block plus its observations' terms, in observation order. */
  private record TrackTerms(Block block, ObservationTerms[] terms) {}

  /**
   * Gauss-Newton normal equations without damping. Jacobians and residuals do not depend on the
   * LM damping, so one linearization serves every damping trial of an iteration ({@link
   * Linear#damped}). Per-track work (projections, Jacobians, point blocks) runs in parallel; the
   * shared camera/lens sums are then accumulated serially in the original track/observation
   * order, so the system is bit-identical to the serial loop.
   */
  private static Linear linearize(
      List<ImageFrame> frames,
      List<FeatureTracks.Track> tracks,
      int[] index,
      int count,
      double huber,
      SharedCalibration lens) {
    Linear s = new Linear(count, lens != null, frames.get(0).options.memoryMb * 1048576L);
    int n = tracks.size();
    int maxObservations = linearizationObservationLimit(frames.get(0).options, lens != null);
    for (int begin = 0; begin < n; ) {
      int end = linearizationBatchEnd(tracks, begin, maxObservations);
      if (end == begin + 1 && tracks.get(begin).point != null
          && tracks.get(begin).observations.size() > maxObservations)
        MemorySafety.requireHeap((long) tracks.get(begin).observations.size() * (lens == null ? 512 : 1024),
            frames.get(0).options, "Single long-track Jacobian batch");
      final int first = begin, size = end - begin, chunks = chunks(size, 0);
      TrackTerms[] terms = new TrackTerms[size];
      parallel(chunks, chunk -> {
        int from = (int) ((long) size * chunk / chunks), to = (int) ((long) size * (chunk + 1) / chunks);
        for (int i = from; i < to; i++) {
          FeatureTracks.Track t = tracks.get(first + i);
          if (t.point != null) terms[i] = linearizeTrack(frames, t, index, huber, lens);
        }
      });
      // Shared sums, parallel by destination: the lens block and each camera's normal/coupling are
      // owned by one task that visits its observations in the original track/observation order, so
      // every element receives the same additions in the same order (bit-identical).
      List<List<ObservationTerms>> byCamera = new ArrayList<>();
      for (int c = 0; c < count; c++) byCamera.add(null);
      List<ObservationTerms> all = new ArrayList<>();
      for (TrackTerms track : terms) {
        if (track == null) continue;
        for (ObservationTerms o : track.terms()) {
          all.add(o);
          if (o.camera() >= 0) {
            if (byCamera.get(o.camera()) == null) byCamera.set(o.camera(), new ArrayList<>());
            byCamera.get(o.camera()).add(o);
          }
        }
        s.points.add(track.block());
      }
      // One task per destination row (6 per camera, one per lens parameter).
      // One task per camera (its 6 rows, one pass) and one per lens row (the heaviest destination).
      int[] touched = java.util.stream.IntStream.range(0, count).filter(c -> byCamera.get(c) != null).toArray();
      parallel(touched.length + (lens != null ? SharedCalibration.SIZE : 0), task -> {
        if (task >= touched.length) {
          int x = task - touched.length;
          for (ObservationTerms o : all) accumulateRow(s.lensNormal, s.lensRhs, o.jl(), o.residual(), o.weight(), x);
          return;
        }
        int c = touched[task];
        for (ObservationTerms o : byCamera.get(c)) {
          if (lens != null)
            for (int j = 0; j < SharedCalibration.SIZE; j++)
              for (int k = 0; k < 6; k++)
                s.coupling[c][k * SharedCalibration.SIZE + j] +=
                    o.weight() * (o.jc()[0][k] * o.jl()[0][j] + o.jc()[1][k] * o.jl()[1][j]);
          accumulate(s.cameraNormal[c], s.cameraRhs[c], o.jc(), o.residual(), o.weight());
        }
      });
      begin = end;
    } // per-observation Jacobians become collectible before the next bounded batch
    boolean antenna = lens != null && lens.antennaModel();
    for (ImageFrame f : frames)
      if (antenna && f.camera.registered) antennaPriorNormal(f.camera, index[f.id], s, lens);
      else if (index[f.id] >= 0)
        priorNormal(f.camera, s.cameraNormal[index[f.id]], s.cameraRhs[index[f.id]]);
    if (lens != null) lens.priorNormal(s.lensNormal, s.lensRhs);
    return s;
  }

  private static TrackTerms linearizeTrack(
      List<ImageFrame> frames,
      FeatureTracks.Track t,
      int[] index,
      double huber,
      SharedCalibration lens) {
    Block block = new Block(t, lens != null);
    List<ObservationTerms> terms = new ArrayList<>(t.observations.size());
    for (FeatureTracks.Observation o : t.observations) {
      ImageFrame f = frames.get(o.image);
      Camera c = f.camera;
      if (!c.registered) continue;
      Point obs = lens == null ? f.keypoints[o.feature].pt : lens.observed[f.id][o.feature];
      // With rolling shutter, q is the camera-frame point at the row's exposure time. It moves
      // with pose updates exactly like the plain view, so the pose Jacobian below is unchanged.
      double[] q = lens == null ? c.view(t.point) : lens.view(f, t.point, obs.y);
      if (q[2] <= 1e-8) continue;
      double[] uv = lens == null ? c.projectView(q) : lens.project(f, t.point, obs.y);
      double[] residual = {uv[0] - obs.x, uv[1] - obs.y};
      double weight =
          Math.min(1, huber / Math.max(1e-12, Math.hypot(residual[0], residual[1])));
      double[][] jq = {
        {c.fx / q[2], 0, -c.fx * q[0] / (q[2] * q[2])},
        {0, c.fy / q[2], -c.fy * q[1] / (q[2] * q[2])}
      };
      if (lens != null) jq = lens.spatialJacobian(f, t.point, obs.y);
      double[][] jl = lens == null ? null : lens.lensJacobian(f, t.point, obs.y);
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
      if (lens != null)
        for (int j = 0; j < SharedCalibration.SIZE; j++)
          for (int k = 0; k < 3; k++)
            block.lensCross[j * 3 + k] += weight * (jl[0][j] * jp[0][k] + jl[1][j] * jp[1][k]);
      terms.add(new ObservationTerms(ci, jc, jl, residual, weight));
      if (ci < 0) continue;
      Edge e = new Edge(ci);
      for (int j = 0; j < 6; j++)
        for (int k = 0; k < 3; k++)
          e.cross[j * 3 + k] = weight * (jc[0][j] * jp[0][k] + jc[1][j] * jp[1][k]);
      block.edges.add(e);
    }
    if (t.prior != null)
      // Known position (GCP): r = (X - prior) / sigma, J = I / sigma.
      for (int k = 0; k < 3; k++) {
        double w = 1 / (t.priorSigma[k] * t.priorSigma[k]);
        block.normal[k * 3 + k] += w;
        block.b[k] -= w * (t.point[k] - t.prior[k]);
      }
    return new TrackTerms(block, terms.toArray(ObservationTerms[]::new));
  }

  /** Start of each track's observations in a flat per-observation array (length tracks + 1). */
  private static int[] observationOffsets(List<FeatureTracks.Track> tracks) {
    int[] start = new int[tracks.size() + 1];
    for (int i = 0; i < tracks.size(); i++) {
      FeatureTracks.Track t = tracks.get(i);
      start[i + 1] = start[i] + (t.point == null ? 0 : t.observations.size());
    }
    return start;
  }

  /** Number of work chunks: depends only on the problem size, never on the thread count. */
  static int chunks(int items, int unused) {
    return (int) Math.max(1, Math.min(64, items / 256 + 1));
  }

  static int linearizationObservationLimit(Options options, boolean shared) {
    long scratch = Math.min(32L * MemorySafety.MIB, options.memoryMb * MemorySafety.MIB / 64);
    int limit = (int) Math.max(1, scratch / (shared ? 1024 : 512));
    return Math.max(1, Math.min(limit, Integer.getInteger("sgm.test.bundleBatchObservations", limit)));
  }

  static int linearizationBatchEnd(List<FeatureTracks.Track> tracks, int first, int maximum) {
    long observations = 0;
    int end = first;
    while (end < tracks.size()) {
      var track = tracks.get(end);
      int count = track.point == null ? 0 : track.observations.size();
      if (end > first && observations + count > maximum) break;
      observations += count;
      end++;
      // Bound both the observation payload and the array of empty/untriangulated tracks.
      if (observations >= maximum || end - first >= 16384) break;
    }
    return end;
  }

  static void parallel(int n, java.util.function.IntConsumer body) {
    if (n <= 1) {
      for (int i = 0; i < n; i++) body.accept(i);
      return;
    }
    java.util.stream.IntStream.range(0, n).parallel().forEach(body);
  }

  /**
   * Marginal lens uncertainty includes nuisance poses and structure through Schur solves. Tiny
   * damping regularizes the scale gauge; these are conditional local diagnostics.
   */
  static double[] calibrationUncertainty(
      List<ImageFrame> frames,
      List<FeatureTracks.Track> tracks,
      double huber,
      SharedCalibration lens,
      double noise) {
    double[][] covariance = calibrationCovariance(frames, tracks, huber, lens, noise);
    if (covariance == null) return null;
    double[] sigma = new double[SharedCalibration.SIZE];
    for (int k = 0; k < sigma.length; k++) sigma[k] = Math.sqrt(covariance[k][k]);
    return sigma;
  }

  static double[][] calibrationCovariance(
      List<ImageFrame> frames,
      List<FeatureTracks.Track> tracks,
      double huber,
      SharedCalibration lens,
      double noise) {
    int[] index = new int[frames.size()];
    Arrays.fill(index, -1);
    int count = 0;
    for (ImageFrame f : frames) if (f.camera.registered && !f.camera.fixed) index[f.id] = count++;
    Linear system = linearize(frames, tracks, index, count, huber, 1e-10, lens);
    double[][] covariance = new double[SharedCalibration.SIZE][SharedCalibration.SIZE];
    for (int k = 0; k < SharedCalibration.SIZE; k++)
      if (lens.active[k]) {
        double[] rhs = new double[count * 6 + SharedCalibration.SIZE];
        rhs[count * 6 + k] = 1;
        Result fit = system.solve(rhs);
        if (fit == null || fit.relative > 1e-3 || !(fit.x[count * 6 + k] > 0)) return null;
        for (int j = 0; j < SharedCalibration.SIZE; j++)
          if (lens.active[j]) covariance[j][k] = noise * noise * fit.x[count * 6 + j];
      }
    for (int j = 0; j < SharedCalibration.SIZE; j++)
      for (int k = j + 1; k < SharedCalibration.SIZE; k++)
        covariance[j][k] = covariance[k][j] = (covariance[j][k] + covariance[k][j]) * .5;
    return covariance;
  }

  /** Undamped observation information, with lens priors excluded. Points are eliminated exactly;
   * the caller must marginalize the remaining camera freedoms, including the scale nullspace. */
  record CalibrationInformation(double[] matrix, double[] observationDiagonal) {}

  static CalibrationInformation calibrationDataInformation(
      List<ImageFrame> frames, List<FeatureTracks.Track> tracks, double huber,
      SharedCalibration lens) {
    int[] index = new int[frames.size()];
    Arrays.fill(index, -1);
    int count = 0;
    for (ImageFrame f : frames) if (f.camera.registered && !f.camera.fixed) index[f.id] = count++;
    int n = count * 6 + SharedCalibration.SIZE;
    // Keep the same bounded workspace as the bundle solver. Unsupported oversized assessments
    // retain the previous calibration instead of substituting a damped/prior-driven certificate.
    if (n > 768 || 8L * n * n > frames.get(0).options.memoryMb * 1048576L / 16) return null;
    boolean before = lens.dataOnly;
    lens.dataOnly = true;
    try {
      Linear system = linearize(frames, tracks, index, count, huber, 0, lens);
      double[] diagonal = new double[n];
      for (int c = 0; c < count; c++)
        for (int k = 0; k < 6; k++) diagonal[c * 6 + k] = system.cameraNormal[c][k * 6 + k];
      for (int k = 0; k < SharedCalibration.SIZE; k++)
        diagonal[count * 6 + k] = system.lensNormal[k * SharedCalibration.SIZE + k];
      return new CalibrationInformation(system.reducedMatrix(n), diagonal);
    } catch (IllegalStateException singularPoint) {
      return null;
    } finally {
      lens.dataOnly = before;
    }
  }

  private static void accumulate(double[] a, double[] b, double[][] j, double[] r, double w) {
    int n = b.length;
    for (int x = 0; x < n; x++) {
      b[x] -= w * (j[0][x] * r[0] + j[1][x] * r[1]);
      for (int y = 0; y < n; y++) a[x * n + y] += w * (j[0][x] * j[0][y] + j[1][x] * j[1][y]);
    }
  }

  /** Row x of accumulate (rows are independent; same per-element arithmetic). */
  private static void accumulateRow(double[] a, double[] b, double[][] j, double[] r, double w, int x) {
    int n = b.length;
    b[x] -= w * (j[0][x] * r[0] + j[1][x] * r[1]);
    for (int y = 0; y < n; y++) a[x * n + y] += w * (j[0][x] * j[0][y] + j[1][x] * j[1][y]);
  }

  private static void damp(double[] a, int n, double lambda) {
    for (int i = 0; i < n; i++) a[i * n + i] += lambda * Math.max(1, a[i * n + i]);
  }

  private static double[] inverse(double[] a, int n) {
    double[] factored = SmallMatrices.inverse(a, n);
    if (factored != null) return factored;
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
    final boolean shared;
    final double[] lensNormal = new double[SharedCalibration.SIZE * SharedCalibration.SIZE],
        lensRhs = new double[SharedCalibration.SIZE];
    final double[][] coupling;
    final long workspaceBudget, memoryBudget;
    double[] reduced;

    Linear(int n, boolean shared, long memoryBudget) {
      this.memoryBudget = memoryBudget;
      long directBudget = Math.min(32L * 1048576, memoryBudget / 16);
      workspaceBudget = Math.max(0, Math.min(directBudget,
          Long.getLong("sgm.test.bundleDirectWorkspaceBytes", directBudget)));
      this.shared = shared;
      coupling = new double[n][6 * SharedCalibration.SIZE];
      count = n;
      cameraNormal = new double[n][36];
      cameraRhs = new double[n][6];
    }

    /**
     * This undamped system with LM damping applied, as the single-pass linearization produced it:
     * the same sums plus lambda on the diagonals, point blocks inverted after damping.
     */
    Linear damped(double lambda) {
      Linear d = new Linear(count, shared, memoryBudget);
      for (int c = 0; c < count; c++) {
        System.arraycopy(cameraNormal[c], 0, d.cameraNormal[c], 0, 36);
        System.arraycopy(cameraRhs[c], 0, d.cameraRhs[c], 0, 6);
        System.arraycopy(coupling[c], 0, d.coupling[c], 0, coupling[c].length);
        damp(d.cameraNormal[c], 6, lambda);
      }
      System.arraycopy(lensNormal, 0, d.lensNormal, 0, lensNormal.length);
      System.arraycopy(lensRhs, 0, d.lensRhs, 0, lensRhs.length);
      if (shared) damp(d.lensNormal, SharedCalibration.SIZE, lambda);
      Block[] blocks = new Block[points.size()];
      int chunks = chunks(blocks.length, 0);
      parallel(chunks, chunk -> {
        int from = (int) ((long) blocks.length * chunk / chunks), to = (int) ((long) blocks.length * (chunk + 1) / chunks);
        for (int i = from; i < to; i++) blocks[i] = new Block(points.get(i), lambda);
      });
      d.points.addAll(Arrays.asList(blocks));
      d.edgeLists = edgesByCamera(); // same point/edge structure
      return d;
    }

    /** Assemble only the small camera/lens system; points are still eliminated analytically. */
    private double[] reducedMatrix(int n) {
      if (reduced != null) return reduced;
      double[] h = new double[n * n];
      for (int c = 0; c < count; c++)
        for (int j = 0; j < 6; j++)
          for (int k = 0; k < 6; k++) h[(c * 6 + j) * n + c * 6 + k] = cameraNormal[c][j * 6 + k];
      if (shared) {
        int off = count * 6;
        for (int j = 0; j < SharedCalibration.SIZE; j++)
          for (int k = 0; k < SharedCalibration.SIZE; k++)
            h[(off + j) * n + off + k] = lensNormal[j * SharedCalibration.SIZE + k];
        for (int c = 0; c < count; c++)
          for (int j = 0; j < 6; j++)
            for (int k = 0; k < SharedCalibration.SIZE; k++)
              h[(c * 6 + j) * n + off + k] =
                  h[(off + k) * n + c * 6 + j] = coupling[c][j * SharedCalibration.SIZE + k];
      }
      // Point eliminations, parallel by destination rows: each camera's (and the lens') rows are
      // written by one task that visits the points in their original order, so every element
      // receives the same subtractions in the same order as the serial loop (bit-identical).
      List<List<int[]>> byCamera = edgesByCamera();
      parallel(count + (shared ? SharedCalibration.SIZE : 0), task -> {
        if (task >= count) {
          for (Block b : points) subtractRow(h, n, b, b.lensCross, count * 6, task - count);
          return;
        }
        for (int[] pe : byCamera.get(task)) {
          Block b = points.get(pe[0]);
          subtractRows(h, n, b, b.edges.get(pe[1]).cross, task * 6, 6);
        }
      });
      // The mathematical Schur matrix is symmetric. Remove only round-off asymmetry.
      for (int i = 0; i < n; i++)
        for (int j = 0; j < i; j++)
          h[i * n + j] = h[j * n + i] = .5 * (h[i * n + j] + h[j * n + i]);
      return reduced = h;
    }

    private List<List<int[]>> edgeLists;

    /** Per camera, its (point, edge) index pairs in point/edge order; built once per system. */
    List<List<int[]>> edgesByCamera() {
      if (edgeLists != null) return edgeLists;
      List<List<int[]>> lists = new ArrayList<>();
      for (int c = 0; c < count; c++) lists.add(new ArrayList<>());
      for (int p = 0; p < points.size(); p++) {
        List<Edge> edges = points.get(p).edges;
        for (int e = 0; e < edges.size(); e++) lists.get(edges.get(e).camera).add(new int[] {p, e});
      }
      return edgeLists = lists;
    }

    /**
     * target[lens rows] -= lensCross * w[p] and target[camera rows] -= cross * w[p] for every point
     * p (w holds 3 values per point). Parallel by destination rows; each owner visits points in
     * their original order, so the result is bit-identical to the serial point loop.
     */
    private void scatter(double[] target, double[] w) {
      List<List<int[]>> byCamera = edgesByCamera();
      parallel(count + (shared ? SharedCalibration.SIZE : 0), task -> {
        if (task >= count) {
          int j = task - count, row = count * 6 + j;
          for (int p = 0; p < points.size(); p++) {
            Block b = points.get(p);
            for (int k = 0; k < 3; k++) target[row] -= b.lensCross[j * 3 + k] * w[3 * p + k];
          }
          return;
        }
        for (int[] pe : byCamera.get(task)) {
          Edge e = points.get(pe[0]).edges.get(pe[1]);
          for (int j = 0; j < 6; j++)
            for (int k = 0; k < 3; k++) target[task * 6 + j] -= e.cross[j * 3 + k] * w[3 * pe[0] + k];
        }
      });
    }

    private void subtractRows(double[] h, int n, Block b, double[] left, int offset, int rows) {
      for (int j = 0; j < rows; j++) subtractRow(h, n, b, left, offset, j);
    }

    /** Row j of subtractRows (rows are independent). */
    private void subtractRow(double[] h, int n, Block b, double[] left, int offset, int j) {
      {
        double v0 = 0, v1 = 0, v2 = 0;
        for (int k = 0; k < 3; k++) {
          v0 += left[j * 3 + k] * b.inverse[k * 3];
          v1 += left[j * 3 + k] * b.inverse[k * 3 + 1];
          v2 += left[j * 3 + k] * b.inverse[k * 3 + 2];
        }
        int row = (offset + j) * n;
        if (shared)
          for (int k = 0; k < SharedCalibration.SIZE; k++)
            h[row + count * 6 + k] -=
                v0 * b.lensCross[k * 3] + v1 * b.lensCross[k * 3 + 1] + v2 * b.lensCross[k * 3 + 2];
        for (Edge e : b.edges)
          for (int k = 0; k < 6; k++)
            h[row + e.camera * 6 + k] -=
                v0 * e.cross[k * 3] + v1 * e.cross[k * 3 + 1] + v2 * e.cross[k * 3 + 2];
      }
    }

    private Result direct(double[] rhs) {
      int n = rhs.length;
      // Includes Java matrix, scaled matrix, native matrix/factor and temporary solve space.
      if (n == 0 || n > 768 || 4L * n * n * Double.BYTES > workspaceBudget) return null;
      double[] h = reducedMatrix(n),
          scaled = new double[h.length],
          scale = new double[n],
          b = new double[n];
      for (int i = 0; i < n; i++) {
        if (!(h[i * n + i] > 0) || !Double.isFinite(h[i * n + i])) return null;
        scale[i] = Math.sqrt(h[i * n + i]);
        b[i] = rhs[i] / scale[i];
      }
      for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++) scaled[i * n + j] = h[i * n + j] / scale[i] / scale[j];
      Mat a = Math3.mat(scaled, n, n), y = Math3.mat(b, n, 1), solution = new Mat();
      try {
        if (!Core.solve(a, y, solution, Core.DECOMP_CHOLESKY)) return null;
        double[] x = Math3.array(solution);
        for (int i = 0; i < n; i++) x[i] /= scale[i];
        // Certify against the original matrix-free operator, not the assembled matrix itself.
        double[] residual = product(x);
        for (int i = 0; i < n; i++) residual[i] -= rhs[i];
        double norm = Math.sqrt(dot(rhs, rhs));
        double relative = Math.sqrt(dot(residual, residual)) / Math.max(1e-15, norm);
        return Double.isFinite(relative) && relative < 1e-6
            ? new Result(x, 0, relative, "direct_schur_cholesky")
            : null;
      } finally {
        a.release();
        y.release();
        solution.release();
      }
    }

    double[] product(double[] x) {
      double[] out = new double[x.length];
      for (int i = 0; i < count; i++)
        for (int j = 0; j < 6; j++)
          for (int k = 0; k < 6; k++) out[i * 6 + j] += cameraNormal[i][j * 6 + k] * x[i * 6 + k];
      if (shared) {
        int off = count * 6;
        for (int j = 0; j < SharedCalibration.SIZE; j++)
          for (int k = 0; k < SharedCalibration.SIZE; k++)
            out[off + j] += lensNormal[j * SharedCalibration.SIZE + k] * x[off + k];
        for (int c = 0; c < count; c++)
          for (int j = 0; j < 6; j++)
            for (int k = 0; k < SharedCalibration.SIZE; k++) {
              out[c * 6 + j] += coupling[c][j * SharedCalibration.SIZE + k] * x[off + k];
              out[off + k] += coupling[c][j * SharedCalibration.SIZE + k] * x[c * 6 + j];
            }
      }
      // Per-point u = inverse * (cross^T x) in parallel (pure), then subtracted serially in point
      // order: bit-identical to the serial loop.
      int pointCount = points.size(), chunks = chunks(pointCount, 0);
      double[] us = new double[3 * pointCount];
      parallel(chunks, chunk -> {
        double[] v = new double[3];
        int from = (int) ((long) pointCount * chunk / chunks), to = (int) ((long) pointCount * (chunk + 1) / chunks);
        for (int p = from; p < to; p++) {
          Block b = points.get(p);
          Arrays.fill(v, 0);
          if (shared)
            for (int k = 0; k < 3; k++)
              for (int j = 0; j < SharedCalibration.SIZE; j++)
                v[k] += b.lensCross[j * 3 + k] * x[count * 6 + j];
          for (Edge e : b.edges)
            for (int k = 0; k < 3; k++)
              for (int j = 0; j < 6; j++) v[k] += e.cross[j * 3 + k] * x[e.camera * 6 + j];
          for (int i = 0; i < 3; i++) {
            double u = 0;
            for (int j = 0; j < 3; j++) u += b.inverse[i * 3 + j] * v[j];
            us[3 * p + i] = u;
          }
        }
      });
      scatter(out, us);
      return out;
    }

    Result solve() {
      return solve(null);
    }

    Result solve(double[] suppliedRhs) {
      int n = count * 6 + (shared ? SharedCalibration.SIZE : 0);
      double[] rhs = new double[n];
      double[][] diag = new double[count][];
      double[] lensDiag = lensNormal.clone();
      if (shared) System.arraycopy(lensRhs, 0, rhs, count * 6, SharedCalibration.SIZE);
      for (int c = 0; c < count; c++) {
        System.arraycopy(cameraRhs[c], 0, rhs, c * 6, 6);
        diag[c] = cameraNormal[c].clone();
      }
      // Reduced right-hand side: per-point v in parallel, ordered scatter (bit-identical).
      int pointCount = points.size(), chunks = chunks(pointCount, 0);
      double[] vs = new double[3 * pointCount];
      parallel(chunks, chunk -> {
        int from = (int) ((long) pointCount * chunk / chunks), to = (int) ((long) pointCount * (chunk + 1) / chunks);
        for (int p = from; p < to; p++) {
          Block b = points.get(p);
          System.arraycopy(mv(b.inverse, b.b, 3), 0, vs, 3 * p, 3);
        }
      });
      scatter(rhs, vs);
      double[] solveRhs = suppliedRhs == null ? rhs : suppliedRhs;
      Result direct = direct(solveRhs);
      if (direct != null) return direct;
      // Block-Jacobi preconditioner: only the PCG fallback needs it.
      for (Block b : points) {
        if (shared)
          for (int j = 0; j < SharedCalibration.SIZE; j++)
            for (int l = 0; l < SharedCalibration.SIZE; l++)
              for (int k = 0; k < 3; k++)
                for (int m = 0; m < 3; m++)
                  lensDiag[j * SharedCalibration.SIZE + l] -=
                      b.lensCross[j * 3 + k] * b.inverse[k * 3 + m] * b.lensCross[l * 3 + m];
        for (Edge e : b.edges)
          for (int j = 0; j < 6; j++)
            for (int l = 0; l < 6; l++)
              for (int k = 0; k < 3; k++)
                for (int m = 0; m < 3; m++)
                  diag[e.camera][j * 6 + l] -=
                      e.cross[j * 3 + k] * b.inverse[k * 3 + m] * e.cross[l * 3 + m];
      }
      for (int c = 0; c < count; c++) {
        diag[c] = inverse(diag[c], 6);
        if (diag[c] == null) return null;
      }
      double[] lensInverse = shared ? inverse(lensDiag, SharedCalibration.SIZE) : null;
      if (shared && lensInverse == null) return null;
      if (suppliedRhs != null) rhs = suppliedRhs.clone();
      double[] x = new double[n],
          r = rhs.clone(),
          z = precondition(diag, lensInverse, r),
          p = z.clone();
      double norm = Math.sqrt(dot(rhs, rhs));
      if (norm < 1e-15) return new Result(x, 0, 0, "matrix_free_pcg");
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
        if (relative < 1e-6) return new Result(x, iter + 1, relative, "matrix_free_pcg");
        z = precondition(diag, lensInverse, r);
        double next = dot(r, z), beta = next / rz;
        for (int i = 0; i < n; i++) p[i] = z[i] + beta * p[i];
        rz = next;
      }
      // An inexact step is still checked against the full robust objective before acceptance.
      return new Result(x, limit, relative, "matrix_free_pcg");
    }

    private double[] precondition(double[][] inverse, double[] lensInverse, double[] r) {
      double[] z = new double[r.length];
      for (int c = 0; c < count; c++)
        for (int j = 0; j < 6; j++)
          for (int k = 0; k < 6; k++) z[c * 6 + j] += inverse[c][j * 6 + k] * r[c * 6 + k];
      if (shared)
        for (int j = 0; j < SharedCalibration.SIZE; j++)
          for (int k = 0; k < SharedCalibration.SIZE; k++)
            z[count * 6 + j] += lensInverse[j * SharedCalibration.SIZE + k] * r[count * 6 + k];
      return z;
    }
  }
}
