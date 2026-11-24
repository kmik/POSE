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
      optimizeFit(frames, tracks, iterations, huber, project, anchor, scaleCamera, lens, work, false, null);
      work.done();
    }
  }

  /**
   * --registration-local-bundles: poses of the cameras with variable[id] true and the points of
   * the given tracks (those the caller selected as seen by them); every other registered camera
   * is held fixed and defines the frame, so no similarity normalization is applied. At most
   * `iterations` accepted iterations, stopping early when the gain is negligible.
   */
  static void optimizeLocal(List<ImageFrame> frames, List<FeatureTracks.Track> tracks,
      int iterations, double huber, int anchor, int scaleCamera, boolean[] variable)
      throws IOException {
    boolean[] fixed = new boolean[frames.size()];
    for (ImageFrame f : frames) fixed[f.id] = f.camera.fixed;
    try (WorkProgress work = WorkProgress.start("Local bundle", iterations, "iterations")) {
      optimizeFit(frames, tracks, iterations, huber, null, anchor, scaleCamera, null, work, true, variable);
      work.done();
    } finally {
      // The flags of the surrounding global fit, not the local frame.
      for (ImageFrame f : frames) f.camera.fixed = fixed[f.id];
    }
  }

  /** Model-selection proposals have a separate hard cap; final bundles retain convergence mode. */
  static void optimizeTrial(List<ImageFrame> frames, List<FeatureTracks.Track> tracks,
      int iterations, double huber, int anchor, int scaleCamera, SharedCalibration lens)
      throws IOException {
    int cap = frames.get(0).options.calibrationTrialIterations;
    if (cap == 0) {
      optimizeImpl(frames, tracks, Math.max(12, iterations), huber, null, anchor, scaleCamera, lens);
      return;
    }
    try (WorkProgress work = WorkProgress.start("Calibration trial " + String.join(",", lens.parameters()),
        cap, "iterations")) {
      optimizeFit(frames, tracks, cap, huber, null, anchor, scaleCamera, lens, work, true, null);
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
      WorkProgress work, boolean hardCap, boolean[] local)
      throws IOException {
    Options options = frames.get(0).options;
    boolean legacy = options.bundleSolverLegacy();
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
        if (local != null && !local[f.id]) f.camera.fixed = true;
        if (!f.camera.fixed) index[f.id] = count++;
      }
    }
    if (local != null && count == 0) return;
    AlignmentMemory.checkBundle(frames, tracks, index, count, lens == null ? 0 : lens.size());
    double before = error(frames, tracks, huber, lens), lambda = 1e-3;
    Path diagnostics = null;
    if (project != null) {
      Path dir = project.data.directory("04_geometry/bundle");
      diagnostics = dir.resolve("linear_solver.csv");
      Files.writeString(
          diagnostics,
          "iteration,trial,lambda,pcg_iterations,relative_linear_residual,objective_before,objective_trial,accepted,linear_method"
              + (legacy ? "" : ",linear_tolerance") + "\n");
      Map<String, Object> solverJson =
          Json.object(
              "solver",
              "joint Levenberg-Marquardt; bounded direct Schur Cholesky; matrix-free block-Jacobi"
                  + " PCG fallback",
              "requested_iterations",
              iterations,
              "iteration_mode",
              frames.get(0).options.bundleIterationsExplicit
                  ? "fixed budget (--bundle-iterations)"
                  : "until two successive gains < " + (frames.get(0).options.quickCalibration ? "1e-4" : "1e-6") + " relative, at most " + AUTO_LIMIT,
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
                  + " and a bounded workspace; larger blocks retain matrix-free PCG.");
      if (!legacy) {
        SolverConfig config = SolverConfig.of(options);
        long n = 6L * count + (lens == null ? 0 : lens.size());
        solverJson.put("solver", "joint Levenberg-Marquardt; "
            + (config.directLimit < 0 ? "bounded direct Schur Cholesky (legacy)" : "parallel dense Schur Cholesky")
            + "; matrix-free " + options.bundlePreconditioner + " PCG fallback");
        solverJson.put("bundle_solver", options.bundleSolver);
        solverJson.put("direct_limit", config.directLimit < 0 ? "legacy" : config.directLimit);
        solverJson.put("dense_memory_budget_bytes", config.directLimit < 0 ? null : config.denseBudget);
        solverJson.put("reduced_variables", n);
        solverJson.put("direct_eligible", config.directLimit < 0 ? n <= 768 : config.dense(n));
        solverJson.put("preconditioner", options.bundlePreconditioner);
        solverJson.put("inexact_steps", options.bundleInexactSteps);
        solverJson.put("linear_relative_tolerance", options.bundleInexactSteps
            ? "adaptive: 0.1 at the start, 0.9 (gradient ratio)^2 per iteration, at least 1e-6" : 1e-6);
        solverJson.put("lens_fit_stop", options.lensFitStop);
        solverJson.put("registration_local_bundles", options.registrationLocalBundles);
        solverJson.put("note", "All triangulated tracks and registered observations participate. Huber"
            + " weights are recomputed each outer iteration; existing post-bundle track rejection"
            + " follows this solve. linear_solver.csv names the method of every step (dense_cholesky,"
            + " direct_schur_cholesky, matrix_free_pcg or cluster_jacobi_pcg).");
      }
      Json.write(dir.resolve("solver.json"), solverJson);
      project.data.bundle(0, before, before, frames, tracks);
    }
    // Without --bundle-iterations: iterate until two successive accepted iterations each gain
    // less than CONVERGED (relative), capped at AUTO_LIMIT.
    boolean converge = !hardCap && !frames.get(0).options.bundleIterationsExplicit;
    int limit = converge ? Math.max(iterations, AUTO_LIMIT) : iterations, small = 0;
    // This fit's reusable systems and scratch: one undamped system refilled each iteration, one
    // damped trial system refilled each trial. Dropped when the fit returns or throws.
    Workspace workspace = new Workspace();
    FitStats stats = new FitStats();
    stats.cameras = count;
    stats.objectiveBefore = before;
    LAST_FIT.set(stats);
    // Progress display only: a fixed budget is an exact upper bound; a converging fit gets an
    // estimate from its recent gains (newest first), refreshed every iteration.
    if (!converge) work.total(limit, false);
    double[] gains = new double[4];
    int gainCount = 0;
    long estimate = 0;
    // Fast path only. --bundle-inexact-steps: linear tolerance eta from the forcing sequence.
    boolean inexact = options.bundleInexactSteps;
    double eta = INEXACT_MAX_TOLERANCE, previousGradient = Double.NaN;
    // --lens-fit-stop parameters: successive accepted iterations whose lens step stayed below
    // LENS_STOP_SIGMA standard deviations; redundancy counts for the a-posteriori sigma.
    boolean lensStop = lens != null && options.lensFitStop.equals("parameters")
        && !options.bundleIterationsExplicit;
    double[] previousSteps = null;
    long lensObservations = 0, lensPoints = 0;
    if (lensStop)
      for (FeatureTracks.Track t : tracks)
        if (t.point != null) {
          lensPoints++;
          for (FeatureTracks.Observation o : t.observations)
            if (frames.get(o.image).camera.registered) lensObservations++;
        }
    for (int iteration = 0; iteration < limit; iteration++) {
      boolean accepted = false;
      double after = before;
      double[] acceptedStep = null;
      // Rejected trials restore the exact previous parameters, so one linearization serves every
      // damping trial of this iteration.
      work.detail("iteration " + (iteration + 1) + "; linearizing " + count + " cameras");
      Linear undamped = linearize(frames, tracks, index, count, huber, lens, workspace);
      Damped system = workspace.damped(undamped);
      if (inexact) {
        // Eisenstat-Walker choice 2 on the gradient norm, safeguarded, within [1e-6, 0.1].
        double gradient = gradientNorm(undamped);
        if (Double.isFinite(previousGradient) && previousGradient > 0) {
          double next = 0.9 * Math.pow(gradient / previousGradient, 2), floor = 0.9 * eta * eta;
          if (floor > 0.1) next = Math.max(next, floor);
          eta = Math.max(1e-6, Math.min(INEXACT_MAX_TOLERANCE, next));
        }
        previousGradient = gradient;
        system.tolerance = eta;
      }
      for (int trial = 0; trial < 8; trial++) {
        system.refill(lambda);
        work.detail(
            "iteration "
                + (iteration + 1)
                + ", trial "
                + (trial + 1)
                + "; solving "
                + system.points()
                + " point blocks");
        long solveStart = System.nanoTime();
        Result result = system.solve();
        if (!legacy || STATS) {
          stats.solveNanos += System.nanoTime() - solveStart;
          if (result != null && result.method.endsWith("cholesky")) stats.denseSolves++;
          else if (result != null) stats.iterativeSolves++;
        }
        stats.trials++;
        if (result == null) stats.failedSolves++;
        else {
          stats.lastMethod = result.method;
          stats.lastRelative = result.relative;
        }
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
        // References only: point updates always install new arrays, so these stay exact.
        int pointCount = system.points();
        double[][] oldPoints = workspace.oldPoints(pointCount);
        for (int p = 0; p < pointCount; p++) oldPoints[p] = undamped.owners[p].point;
        double[] oldLens = lens == null ? null : lens.values.clone();
        if (result != null) {
          if (lens != null) lens.move(result.x, count * 6);
          for (ImageFrame f : frames)
            if (index[f.id] >= 0) move(f.camera, result.x, index[f.id] * 6);
          // The scale depends on the cameras only. Points are moved and rescaled in one pass
          // (identical arithmetic per point, any order); when the scale is unusable the trial is
          // rejected and every point restored, so the points need no unscaled update.
          double newLength = Math3.norm(Math3.sub(frames.get(scaleCamera).camera.center(), origin));
          if (local != null) {
            // Local bundle: the fixed cameras hold the frame; no similarity normalization.
            system.movePoints(result.x, origin, 1);
            after = error(frames, tracks, huber, lens);
          } else if (newLength > 1e-12 && Double.isFinite(newLength)) {
            double factor = absolutePriors ? 1 : baseline / newLength;
            for (ImageFrame f : frames)
              if (f.camera.registered && (absolutePriors || f.id != anchor)) {
                double[] center =
                    Math3.add(origin, Math3.times(Math3.sub(f.camera.center(), origin), factor));
                f.camera.translation = Math3.times(Math3.mv(f.camera.rotation, center), -1);
              }
            system.movePoints(result.x, origin, factor);
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
                  + (legacy ? "" : "," + system.tolerance)
                  + "\n",
              StandardOpenOption.APPEND);
        if (accepted) {
          // The replaced point arrays are not kept alive by the saved references.
          Arrays.fill(oldPoints, 0, pointCount, null);
          lambda = Math.max(1e-10, lambda / 3);
          acceptedStep = result.x;
          break;
        }
        // A rejected step that was solved loosely: the next trial solves it more accurately.
        if (inexact && result != null && result.relative > 1e-6) {
          eta = Math.max(1e-6, Math.min(eta, result.relative) * .1);
          system.tolerance = eta;
        }
        if (lens != null) lens.values = oldLens;
        for (ImageFrame f : frames) {
          f.camera.rotation = old[f.id].rotation;
          f.camera.translation = old[f.id].translation;
        }
        for (int p = 0; p < pointCount; p++) undamped.owners[p].point = oldPoints[p];
        lambda = Math.min(1e12, lambda * 10);
      }
      if (!accepted) {
        stats.stop = "no improving step (" + stats.failedSolves + " of " + stats.trials
            + " linear solves failed)";
        if (local == null) System.out.println("Bundle: no improving joint step; stopping.");
        break;
      }
      stats.acceptedIterations++;
      stats.objectiveAfter = after;
      if (project != null) project.data.bundle(iteration + 1, before, after, frames, tracks);
      if (local == null) System.out.printf(
          Locale.ROOT,
          "Bundle iteration %d: robust objective %.5f -> %.5f; joint cameras=%d, points=%d%n",
          iteration + 1,
          before,
          after,
          count,
          system.points());
      work.update(iteration + 1);
      if (lensStop) {
        double[] steps = lensStepSigmas(system, lens, acceptedStep, count, after,
            lensObservations, lensPoints);
        // A converging fit's steps shrink about geometrically: the movement still to come is
        // step r / (1 - r), r the ratio to the previous step. Settled when, for every active
        // lens parameter, both this step and that remainder are below LENS_STOP_SIGMA.
        double worstStep = 0, worstRemaining = 0;
        if (steps == null || previousSteps == null) worstRemaining = Double.POSITIVE_INFINITY;
        else
          for (int k = 0; k < steps.length; k++) {
            if (k >= count * 6 && !lens.active[k - count * 6]) continue;
            double ratio = steps[k] / previousSteps[k];
            double remaining = steps[k] == 0 ? 0
                : ratio < 1 ? steps[k] * ratio / (1 - ratio) : Double.POSITIVE_INFINITY;
            worstStep = Math.max(worstStep, steps[k]);
            worstRemaining = Math.max(worstRemaining, remaining);
          }
        previousSteps = steps;
        System.out.printf(Locale.ROOT, "Bundle lens step: largest %.3g sigma, still to come %.3g sigma%n",
            worstStep, worstRemaining);
        if (worstStep < LENS_STOP_SIGMA && worstRemaining < LENS_STOP_SIGMA) {
          stats.stop = "lens parameters settled";
          System.out.printf(Locale.ROOT,
              "Bundle: lens parameters settled (step %.3g sigma, %.3g sigma still to come) after %d iterations.%n",
              worstStep, worstRemaining, iteration + 1);
          break;
        }
      }
      if (converge) {
        small = before - after < converged(frames.get(0).options) * Math.max(1, before) ? small + 1 : 0;
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
        System.arraycopy(gains, 0, gains, 1, gains.length - 1);
        gains[0] = before - after;
        gainCount++;
        long next = estimatedIterations(iteration + 1, gains, gainCount, small,
            converged(frames.get(0).options) * Math.max(1, after), limit);
        // Unknown now (gains not shrinking): keep the previous estimate, never below one more.
        if (next == 0 && estimate > 0) next = Math.min(limit, Math.max(estimate, iteration + 2));
        if (next > 0) work.total(estimate = next, true);
      } else if (before - after < 1e-7 * Math.max(1, before)) break;
      before = after;
    }
    if ((!legacy || STATS) && local == null)
      System.out.printf(Locale.ROOT,
          "Bundle linear solves: %d (%d direct, %d iterative, %d failed), %.1f s; %d accepted iterations, %d variables%n",
          stats.trials, stats.denseSolves, stats.iterativeSolves, stats.failedSolves,
          stats.solveNanos / 1e9, stats.acceptedIterations, 6 * count + (lens == null ? 0 : lens.size()));
  }

  /** -Dsgm.bundle.stats=true: per-fit linear solve summary also for the legacy solver (benchmarks). */
  static final boolean STATS = Boolean.getBoolean("sgm.bundle.stats");

  /** Largest linear tolerance of --bundle-inexact-steps (the first iteration's). */
  static final double INEXACT_MAX_TOLERANCE = .1;

  /** --lens-fit-stop parameters: a lens step below this many standard deviations is negligible. */
  static final double LENS_STOP_SIGMA = .1;

  /** Norm of the undamped gradient (camera, lens and point right-hand sides), summed in order. */
  private static double gradientNorm(Linear s) {
    double sum = 0;
    for (int c = 0; c < s.count; c++) for (double v : s.cameraRhs[c]) sum += v * v;
    if (s.shared) for (double v : s.lensRhs) sum += v * v;
    for (int p = 0; p < s.points; p++) {
      double[] page = s.rhs[p >>> s.shift];
      int o = (p & s.mask) * 3;
      for (int k = 0; k < 3; k++) sum += page[o + k] * page[o + k];
    }
    return Math.sqrt(sum);
  }

  /**
   * |step| / sigma of every reduced variable (each camera's six pose parameters, then the lens;
   * 0 for inactive lens parameters) of an accepted step. sigma: the dense factor's marginal
   * variance scaled by the a-posteriori variance of unit weight (2 x objective / redundancy).
   * Null when no dense factor is available (the objective rule then decides alone).
   */
  private static double[] lensStepSigmas(Damped system, SharedCalibration lens, double[] step,
      int cameras, double objective, long observations, long points) {
    if (step == null) return null;
    double[] variance = system.marginalVariances();
    if (variance == null) return null;
    int active = 0;
    for (boolean a : lens.active) if (a) active++;
    double redundancy = Math.max(1, 2.0 * observations - 6.0 * cameras - active - 3.0 * points);
    double unit = 2 * objective / redundancy;
    double[] out = new double[variance.length];
    for (int k = 0; k < out.length; k++) {
      if (k >= cameras * 6 && !lens.active[k - cameras * 6]) continue;
      double sigma = Math.sqrt(unit * variance[k]);
      if (!(sigma > 0) || !Double.isFinite(sigma)) return null;
      out[k] = Math.abs(step[k]) / sigma;
    }
    return out;
  }

  /**
   * Progress-bar estimate (never used by the solve) of the total iterations of a converging fit,
   * after `done` iterations: the gains (newest first, `known` recorded) are assumed to shrink
   * geometrically at their mean recent rate; the fit stops after two successive gains below the
   * threshold. 0 when unknown (fewer than two gains, or gains not shrinking).
   */
  static long estimatedIterations(int done, double[] recent, int known, int small, double threshold, int limit) {
    if (done >= limit) return limit;
    if (small >= 1) return Math.min(limit, done + 1);
    int n = Math.min(known, recent.length);
    if (n < 2) return 0;
    double logRatio = 0;
    for (int i = 0; i + 1 < n; i++) {
      if (!(recent[i] > 0 && recent[i + 1] > 0)) return 0;
      logRatio += Math.log(recent[i] / recent[i + 1]);
    }
    logRatio /= n - 1;
    if (!(logRatio < 0) || !Double.isFinite(logRatio)) return 0;
    // Iterations until a gain falls below the threshold, then one more below it.
    double steps = Math.log(threshold / recent[0]) / logRatio;
    if (!Double.isFinite(steps)) return 0;
    long remaining = (long) Math.ceil(Math.max(1, Math.min(limit, steps))) + 1;
    return Math.min(limit, done + remaining);
  }

  static final int AUTO_LIMIT = 100;
  static final double CONVERGED = 1e-6;
  // --quick-calibration: stop once two successive iterations each gain < 1e-4 (relative). The
  // default crawls along flat focal/height directions for up to 100 iterations of < 0.01%.
  static final double QUICK_CONVERGED = 1e-4;

  static double converged(Options o) {
    return o.quickCalibration ? QUICK_CONVERGED : CONVERGED;
  }

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
    // the result is bit-identical to the serial loop. One offset and one cost buffer serve every
    // batch of this call; each batch writes every slot it later reads.
    int[] offsets = new int[0];
    double[] buffer = new double[0];
    for (int begin = 0; begin < tracks.size(); ) {
      int end = linearizationBatchEnd(tracks, begin, linearizationObservationLimit(frames.get(0).options, lens != null));
      List<FeatureTracks.Track> batch = tracks.subList(begin, end);
      int n = batch.size();
      if (offsets.length < n + 1) offsets = new int[n + 1];
      int[] start = offsets;
      for (int i = 0; i < n; i++) {
        FeatureTracks.Track t = batch.get(i);
        start[i + 1] = start[i] + (t.point == null ? 0 : t.observations.size());
      }
      int observations = start[n];
      if (buffer.length < observations) {
        buffer = null;
        buffer = new double[observations];
      }
      double[] costs = buffer;
      int chunks = chunks(n, 0);
      parallel(chunks, chunk -> {
        double[] q = new double[3], uv = new double[2]; // this task's projection scratch
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
            double ox, oy;
            if (lens == null) {
              Point measured = f.keypoints[o.feature].pt;
              ox = measured.x;
              oy = measured.y;
            } else {
              double[] measured = lens.observed[f.id];
              ox = measured[2 * o.feature];
              oy = measured[2 * o.feature + 1];
            }
            // Camera.project / SharedCalibration.project without their result arrays; "projected"
            // is false exactly where they returned null.
            boolean projected;
            if (lens == null) {
              Camera c = f.camera;
              c.viewInto(t.point[0], t.point[1], t.point[2], q);
              projected = !(q[2] <= 1e-8);
              if (projected) {
                uv[0] = c.fx * q[0] / q[2] + c.cx;
                uv[1] = c.fy * q[1] / q[2] + c.cy;
              }
            } else projected = lens.projectInto(f, t.point, oy, q, uv);
            if (!projected || !Double.isFinite(uv[0]) || !Double.isFinite(uv[1])) {
              costs[at++] = Double.POSITIVE_INFINITY;
              continue;
            }
            double e = Math.hypot(uv[0] - ox, uv[1] - oy);
            costs[at++] = e <= huber ? .5 * e * e : huber * (e - .5 * huber);
          }
        }
      });
      for (int i = 0; i < observations; i++) {
        double c = costs[i];
        if (Double.isNaN(c)) continue;
        if (c == Double.POSITIVE_INFINITY) return Double.POSITIVE_INFINITY;
        total += c;
      }
      begin = end;
    }
    boolean antenna = lens != null && lens.antennaModel();
    for (ImageFrame f : frames)
      if (f.camera.registered)
        total += antenna ? antennaPriorCost(f.camera, lens, f.group) : priorCost(f.camera);
    for (FeatureTracks.Track t : tracks)
      if (t.point != null && t.prior != null)
        for (int k = 0; k < 3; k++) total += .5 * Math.pow((t.point[k] - t.prior[k]) / t.priorSigma[k], 2);
    return total;
  }

  /** Camera prior whose position term compares the GPS antenna (C + R^T L + V dt) to the fix. */
  static double antennaPriorCost(Camera c, SharedCalibration lens, int group) {
    if (c.reference == null) return 0;
    double[] position = Math3.sub(lens.antenna(c, group), c.reference.center()), sigma = positionSigmas(c);
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
  private static void antennaPriorNormal(Camera c, int ci, Linear s, SharedCalibration lens, int group) {
    if (c.reference == null) return;
    int n = lens.size(), o = group * SharedCalibration.SIZE, time = o + SharedCalibration.TIME,
        lever = o + SharedCalibration.LEVER;
    double[] residual = Math3.sub(lens.antenna(c, group), c.reference.center()), sigma = positionSigmas(c);
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

  /** Result of one linear solve. x is always a new array owned by the caller (never scratch). */
  private record Result(double[] x, int iterations, double relative, String method) {}

  /**
   * Linear solver switches of one fit (--bundle-solver and its switches). LEGACY is the previous
   * solver exactly: OpenCV direct solve up to 768 variables, block-Jacobi PCG above.
   * directLimit >= 0: the parallel dense Cholesky up to that many reduced variables, bounded by
   * denseBudget bytes for the n^2 matrix. clusterJacobi: dense cluster blocks precondition PCG.
   */
  record SolverConfig(int directLimit, long denseBudget, boolean clusterJacobi) {
    static final SolverConfig LEGACY = new SolverConfig(-1, 0, false);

    static SolverConfig of(Options o) {
      boolean cluster = o.bundlePreconditioner.equals("cluster-jacobi");
      if (o.bundleDirectLimit < 0 && !cluster) return LEGACY;
      long budget = o.memoryMb * MemorySafety.MIB / 8;
      budget = Math.max(0, Math.min(budget, Long.getLong("sgm.test.bundleDenseBytes", budget)));
      return new SolverConfig(o.bundleDirectLimit, budget, cluster);
    }

    boolean legacy() {
      return directLimit < 0 && !clusterJacobi;
    }

    /** The reduced system of n variables is solved by the dense Cholesky. */
    boolean dense(long n) {
      return directLimit >= 0 && n > 0 && n <= directLimit && n <= DenseCholesky.MAX_ORDER
          && 8 * n * n <= denseBudget;
    }
  }

  /** Cameras per dense block of the cluster-Jacobi preconditioner (consecutive variable cameras). */
  static final int CLUSTER_CAMERAS = 64;

  /** What the last fit of this thread did (diagnostics only; never read by the solve). */
  static final class FitStats {
    int cameras, trials, failedSolves, acceptedIterations;
    // Fast path only (never in json(), so legacy reports are unchanged).
    int denseSolves, iterativeSolves;
    long solveNanos;
    double objectiveBefore = Double.NaN, objectiveAfter = Double.NaN, lastRelative = Double.NaN;
    String lastMethod = "none", stop = "iteration limit or converged";

    Map<String, Object> json() {
      return Json.object(
          "variable_cameras", cameras,
          "accepted_iterations", acceptedIterations,
          "linear_solve_trials", trials,
          "failed_linear_solves", failedSolves,
          "last_linear_method", lastMethod,
          "last_relative_linear_residual", Double.isFinite(lastRelative) ? lastRelative : null,
          "objective_before", Double.isFinite(objectiveBefore) ? objectiveBefore : null,
          "objective_after", Double.isFinite(objectiveAfter) ? objectiveAfter : null,
          "stop", stop);
    }
  }

  static final ThreadLocal<FitStats> LAST_FIT = new ThreadLocal<>();

  /** log2 of the points (or edges) per storage page: 8192 by default; tests may lower it. */
  private static int pageShift() {
    return Math.max(1, Math.min(20, Integer.getInteger("sgm.test.bundlePageShift", 13)));
  }

  /** Pages holding items x width doubles: every page full except an exact-size last one. */
  private static double[][] pages(int items, int width, int shift) {
    int page = 1 << shift, count = (int) (((long) items + page - 1) >>> shift);
    double[][] pages = new double[count][];
    for (int i = 0; i < count; i++) pages[i] = new double[Math.min(page, items - (i << shift)) * width];
    return pages;
  }

  /**
   * Undamped Gauss-Newton system of one fit, with primitive point/observation storage instead of
   * one object per point and per edge. Point p's values are in page p >>> shift at (p & mask) *
   * width; edges (observations of variable cameras) likewise. Pages are sized exactly from upper
   * bounds counted before linearizing, filled batch by batch in track order and never repacked.
   * The system belongs to one {@link Workspace}: the next linearization of the same fit refills it
   * in place when its shape is unchanged. Read-only while the damped trials of an iteration run.
   */
  private static final class Linear {
    final int count, shift, mask, pointCapacity, edgeCapacity;
    final boolean shared;
    // Lens block width: 13 per camera group (SharedCalibration.size()); 13 without a lens.
    final int L;
    final double[][] cameraNormal, cameraRhs, coupling;
    final double[] lensNormal, lensRhs;
    final long memoryBudget;
    final SolverConfig config;
    /** Track of each point, in linearization (track) order. */
    final FeatureTracks.Track[] owners;
    /** Per point: undamped 3 x 3 normal (9), right-hand side (3), lens cross products (3 L). */
    final double[][] normal, rhs, lensCross;
    /** Point p's edges are [edgeStart[p], edgeStart[p + 1]), in observation order. */
    final int[] edgeStart, edgeCamera;
    /** Per edge: the 6 x 3 camera/point cross product. */
    final double[][] cross;
    /** Camera c's (point, edge) pairs are [cameraStart[c], cameraStart[c + 1]), in point/edge order. */
    final int[] cameraStart, cameraPoint, cameraEdge;
    int points, edges;

    Linear(int n, int lensWidth, long memoryBudget, int points, int edges, SolverConfig config) {
      L = lensWidth > 0 ? lensWidth : SharedCalibration.SIZE;
      shared = lensWidth > 0;
      count = n;
      this.memoryBudget = memoryBudget;
      this.config = config;
      shift = pageShift();
      mask = (1 << shift) - 1;
      pointCapacity = points;
      edgeCapacity = edges;
      lensNormal = new double[L * L];
      lensRhs = new double[L];
      coupling = new double[n][6 * L];
      cameraNormal = new double[n][36];
      cameraRhs = new double[n][6];
      owners = new FeatureTracks.Track[points];
      normal = pages(points, 9, shift);
      rhs = pages(points, 3, shift);
      lensCross = shared ? pages(points, 3 * L, shift) : null;
      edgeStart = new int[points + 1];
      edgeCamera = new int[edges];
      cross = pages(edges, 18, shift);
      cameraStart = new int[n + 1];
      cameraPoint = new int[edges];
      cameraEdge = new int[edges];
    }

    boolean fits(int n, int lensWidth, long memoryBudget, int points, int edges, SolverConfig config) {
      return count == n && shared == lensWidth > 0 && (!shared || L == lensWidth)
          && this.memoryBudget == memoryBudget && pointCapacity == points && edgeCapacity == edges
          && shift == pageShift() && this.config.equals(config);
    }

    /** The state of a new system: every accumulated sum +0.0, no points or edges. */
    void clear() {
      for (int c = 0; c < count; c++) {
        Arrays.fill(cameraNormal[c], 0);
        Arrays.fill(cameraRhs[c], 0);
        Arrays.fill(coupling[c], 0);
      }
      Arrays.fill(lensNormal, 0);
      Arrays.fill(lensRhs, 0);
      for (double[] page : normal) Arrays.fill(page, 0);
      for (double[] page : rhs) Arrays.fill(page, 0);
      if (lensCross != null) for (double[] page : lensCross) Arrays.fill(page, 0);
      Arrays.fill(owners, null);
      points = edges = 0;
    }
  }

  /**
   * Linearization scratch of one bounded batch, reused by the following batches of the same
   * workspace; nothing in it is carried from one batch to the next. Per observation slot (the
   * track's observation offset plus the term/edge number): camera index, the two 6-wide camera
   * Jacobian rows, the two L-wide lens Jacobian rows, residual and weight, and edge products.
   */
  private static final class Batch {
    final int lensWidth;
    int[] start = new int[1], point = new int[0], terms = new int[0], edges = new int[0];
    int[] termCamera = new int[0], edgeCamera = new int[0], order = new int[0];
    double[] jc = new double[0], jl = new double[0], rw = new double[0], cross = new double[0];
    int[] cameraStart = new int[1], cursor = new int[0];

    Batch(int lensWidth) {
      this.lensWidth = lensWidth;
    }

    void ensureTracks(int n) {
      if (point.length >= n) return;
      start = point = terms = edges = null;
      start = new int[n + 1];
      point = new int[n];
      terms = new int[n];
      edges = new int[n];
    }

    void ensureObservations(int n) {
      if (termCamera.length >= n) return;
      termCamera = edgeCamera = order = null;
      jc = jl = rw = cross = null;
      termCamera = new int[n];
      edgeCamera = new int[n];
      order = new int[n];
      jc = new double[12 * n];
      jl = new double[2 * lensWidth * n];
      rw = new double[3 * n];
      cross = new double[18 * n];
    }

    void ensureCameras(int n) {
      if (cursor.length >= n) return;
      cameraStart = new int[n + 1];
      cursor = new int[n];
    }
  }

  /**
   * Reusable state of one active fit (or one stand-alone solve): its undamped system, the damped
   * trial system, the batch scratch and the saved point references of a trial. Created by that
   * fit and dropped with it, never shared between fits or threads, so nested, repeated and failed
   * invocations each start from their own.
   */
  private static final class Workspace {
    Linear undamped;
    Damped damped;
    Batch batch;
    double[][] oldPoints;

    Damped damped(Linear system) {
      if (damped == null || damped.u != system) {
        damped = null;
        damped = new Damped(system);
      }
      return damped;
    }

    Batch batch(int lensWidth, int tracks, int observations) {
      if (batch == null || batch.lensWidth != lensWidth) batch = new Batch(lensWidth);
      batch.ensureTracks(tracks);
      batch.ensureObservations(observations);
      return batch;
    }

    double[][] oldPoints(int points) {
      if (oldPoints == null || oldPoints.length < points) {
        oldPoints = null;
        oldPoints = new double[points][];
      }
      return oldPoints;
    }
  }

  /** One-off undamped system damped by lambda (a stand-alone workspace). */
  private static Damped linearize(
      List<ImageFrame> frames,
      List<FeatureTracks.Track> tracks,
      int[] index,
      int count,
      double huber,
      double lambda,
      SharedCalibration lens) {
    Workspace workspace = new Workspace();
    Damped system = workspace.damped(linearize(frames, tracks, index, count, huber, lens, workspace));
    system.refill(lambda);
    return system;
  }

  /**
   * Gauss-Newton normal equations without damping. Jacobians and residuals do not depend on the
   * LM damping, so one linearization serves every damping trial of an iteration ({@link
   * Damped#refill}). Per-track work (projections, Jacobians, point blocks) runs in parallel; the
   * shared camera/lens sums are then accumulated in the original track/observation order, so the
   * system is bit-identical to the serial loop. The workspace's previous system is refilled in
   * place when the shape is unchanged (the usual case between iterations of one fit).
   */
  private static Linear linearize(
      List<ImageFrame> frames,
      List<FeatureTracks.Track> tracks,
      int[] index,
      int count,
      double huber,
      SharedCalibration lens,
      Workspace workspace) {
    int lensWidth = lens == null ? 0 : lens.size();
    long pointCount = 0, edgeBound = 0, observationCount = 0;
    for (FeatureTracks.Track t : tracks)
      if (t.point != null) {
        pointCount++;
        observationCount += t.observations.size();
        for (FeatureTracks.Observation o : t.observations)
          if (index[o.image] >= 0 && frames.get(o.image).camera.registered) edgeBound++;
      }
    int points = Math.toIntExact(pointCount), edgeCapacity = Math.toIntExact(edgeBound);
    long memoryBudget = frames.get(0).options.memoryMb * 1048576L;
    SolverConfig config = SolverConfig.of(frames.get(0).options);
    if (workspace.undamped != null
        && workspace.undamped.fits(count, lensWidth, memoryBudget, points, edgeCapacity, config))
      workspace.undamped.clear();
    else {
      // Drop the previous system before allocating its replacement (no overlapping copies).
      workspace.undamped = null;
      workspace.damped = null;
      workspace.undamped = new Linear(count, lensWidth, memoryBudget, points, edgeCapacity, config);
    }
    if (workspace.damped != null) workspace.damped.invalidate();
    final Linear s = workspace.undamped;
    int n = tracks.size();
    int maxObservations = linearizationObservationLimit(frames.get(0).options, lens != null);
    final Batch batch = workspace.batch(lensWidth, Math.min(16384, Math.max(1, n)),
        (int) Math.max(1, Math.min(maxObservations, observationCount)));
    batch.ensureCameras(count);
    final int L = lens == null ? SharedCalibration.SIZE : lens.size();
    for (int begin = 0; begin < n; ) {
      int end = linearizationBatchEnd(tracks, begin, maxObservations);
      if (end == begin + 1 && tracks.get(begin).point != null
          && tracks.get(begin).observations.size() > maxObservations)
        MemorySafety.requireHeap((long) tracks.get(begin).observations.size() * (lens == null ? 512 : 1024),
            frames.get(0).options, "Single long-track Jacobian batch");
      final int first = begin, size = end - begin, chunks = chunks(size, 0);
      batch.ensureTracks(size);
      int observations = 0;
      for (int i = 0; i < size; i++) {
        FeatureTracks.Track t = tracks.get(first + i);
        batch.start[i] = observations;
        if (t.point == null) batch.point[i] = -1;
        else {
          s.owners[s.points] = t;
          batch.point[i] = s.points++;
          observations += t.observations.size();
        }
      }
      batch.start[size] = observations;
      batch.ensureObservations(observations);
      parallel(chunks, chunk -> {
        double[] q = new double[3], uv = new double[2], jq = new double[6], jp = new double[6], jc = new double[12];
        int from = (int) ((long) size * chunk / chunks), to = (int) ((long) size * (chunk + 1) / chunks);
        for (int i = from; i < to; i++)
          if (batch.point[i] >= 0)
            linearizeTrack(frames, tracks.get(first + i), index, huber, lens, s, batch, i, q, uv, jq, jp, jc);
      });
      // Edges into the system in point/observation order; terms compacted (gaps of skipped
      // observations removed) so they keep the original track/observation order.
      int terms = 0;
      for (int i = 0; i < size; i++) {
        int p = batch.point[i];
        if (p < 0) continue;
        int at = batch.start[i];
        s.edgeStart[p] = s.edges;
        for (int e = 0; e < batch.edges[i]; e++) {
          int edge = s.edges++;
          s.edgeCamera[edge] = batch.edgeCamera[at + e];
          System.arraycopy(batch.cross, (at + e) * 18, s.cross[edge >>> s.shift], (edge & s.mask) * 18, 18);
        }
        s.edgeStart[p + 1] = s.edges;
        int count_i = batch.terms[i];
        if (terms != at) {
          System.arraycopy(batch.termCamera, at, batch.termCamera, terms, count_i);
          System.arraycopy(batch.jc, 12 * at, batch.jc, 12 * terms, 12 * count_i);
          if (lens != null) System.arraycopy(batch.jl, 2 * L * at, batch.jl, 2 * L * terms, 2 * L * count_i);
          System.arraycopy(batch.rw, 3 * at, batch.rw, 3 * terms, 3 * count_i);
        }
        terms += count_i;
      }
      // Each camera's terms in their original order (stable counting sort).
      int[] cameraStart = batch.cameraStart, cursor = batch.cursor, order = batch.order;
      Arrays.fill(cameraStart, 0, count + 1, 0);
      for (int t = 0; t < terms; t++) if (batch.termCamera[t] >= 0) cameraStart[batch.termCamera[t] + 1]++;
      int touchedCount = 0;
      for (int c = 0; c < count; c++) {
        if (cameraStart[c + 1] > 0) touchedCount++;
        cameraStart[c + 1] += cameraStart[c];
      }
      System.arraycopy(cameraStart, 0, cursor, 0, count);
      for (int t = 0; t < terms; t++) if (batch.termCamera[t] >= 0) order[cursor[batch.termCamera[t]]++] = t;
      int[] touched = new int[touchedCount];
      for (int c = 0, k = 0; c < count; c++) if (cameraStart[c + 1] > cameraStart[c]) touched[k++] = c;
      // Shared sums, parallel by destination: the lens block and each camera's normal/coupling are
      // owned by one task that visits its observations in the original track/observation order, so
      // every element receives the same additions in the same order (bit-identical). Each lens row
      // fills its upper-triangle part only; element (y, x) receives exactly the same products in
      // the same order as (x, y), so the mirrored lower triangle (below) is bit-identical.
      final int termCount = terms;
      final double[] jc = batch.jc, jl = batch.jl, rw = batch.rw;
      parallel(touched.length + (lens != null ? L : 0), task -> {
        if (task >= touched.length) {
          lensUpperRow(s.lensNormal, s.lensRhs, jl, rw, task - touched.length, termCount);
          return;
        }
        int c = touched[task];
        double[] normal = s.cameraNormal[c], rhs = s.cameraRhs[c], coupling = s.coupling[c];
        for (int at = cameraStart[c]; at < cameraStart[c + 1]; at++) {
          int t = order[at], j0 = 12 * t;
          double w = rw[3 * t + 2];
          if (lens != null) {
            int l0 = 2 * L * t;
            for (int j = 0; j < L; j++)
              for (int k = 0; k < 6; k++)
                coupling[k * L + j] += w * (jc[j0 + k] * jl[l0 + j] + jc[j0 + 6 + k] * jl[l0 + L + j]);
          }
          accumulate(normal, 0, rhs, 0, jc, j0, 6, rw[3 * t], rw[3 * t + 1], w);
        }
      });
      begin = end;
    }
    if (lens != null) {
      for (int x = 0; x < L; x++)
        for (int y = 0; y < x; y++) s.lensNormal[x * L + y] = s.lensNormal[y * L + x];
    }
    boolean antenna = lens != null && lens.antennaModel();
    for (ImageFrame f : frames)
      if (antenna && f.camera.registered) antennaPriorNormal(f.camera, index[f.id], s, lens, f.group);
      else if (index[f.id] >= 0)
        priorNormal(f.camera, s.cameraNormal[index[f.id]], s.cameraRhs[index[f.id]]);
    if (lens != null) lens.priorNormal(s.lensNormal, s.lensRhs);
    // Per camera, its (point, edge) pairs in point/edge order (stable counting sort).
    int[] cs = s.cameraStart;
    Arrays.fill(cs, 0);
    for (int e = 0; e < s.edges; e++) cs[s.edgeCamera[e] + 1]++;
    for (int c = 0; c < count; c++) cs[c + 1] += cs[c];
    int[] cursor = batch.cursor;
    System.arraycopy(cs, 0, cursor, 0, count);
    for (int p = 0; p < s.points; p++)
      for (int e = s.edgeStart[p]; e < s.edgeStart[p + 1]; e++) {
        int at = cursor[s.edgeCamera[e]]++;
        s.cameraPoint[at] = p;
        s.cameraEdge[at] = e;
      }
    return s;
  }

  /**
   * One track's point block (into its storage slot) and its observations' terms and edges (into
   * the batch scratch at the track's observation offset), in observation order. q, uv, jq, jp and jc are
   * the calling task's 3, 2, 2 x 3, 2 x 3 and 2 x 6 scratch; each observation overwrites them.
   */
  private static void linearizeTrack(
      List<ImageFrame> frames,
      FeatureTracks.Track t,
      int[] index,
      double huber,
      SharedCalibration lens,
      Linear s,
      Batch batch,
      int slot,
      double[] q,
      double[] uv,
      double[] jq,
      double[] jp,
      double[] jc) {
    int p = batch.point[slot], at = batch.start[slot], L = s.L, page = p >>> s.shift, local = p & s.mask;
    double[] normal = s.normal[page], rhs = s.rhs[page], lensCross = lens == null ? null : s.lensCross[page];
    int no = local * 9, ro = local * 3, lo = local * 3 * L;
    int terms = 0, edges = 0;
    for (FeatureTracks.Observation o : t.observations) {
      ImageFrame f = frames.get(o.image);
      Camera c = f.camera;
      if (!c.registered) continue;
      double ox, oy;
      if (lens == null) {
        Point measured = f.keypoints[o.feature].pt;
        ox = measured.x;
        oy = measured.y;
      } else {
        double[] measured = lens.observed[f.id];
        ox = measured[2 * o.feature];
        oy = measured[2 * o.feature + 1];
      }
      // With rolling shutter, q is the camera-frame point at the row's exposure time. It moves
      // with pose updates exactly like the plain view, so the pose Jacobian below is unchanged.
      // q is computed once: projection, spatial and lens Jacobians all used this same point.
      if (lens == null) c.viewInto(t.point[0], t.point[1], t.point[2], q);
      else lens.viewInto(f, t.point, oy, q);
      if (q[2] <= 1e-8) continue;
      int term = at + terms;
      double u0, u1;
      if (lens == null) {
        u0 = c.fx * q[0] / q[2] + c.cx;
        u1 = c.fy * q[1] / q[2] + c.cy;
      } else {
        // A NaN depth passes the test above; the former lens.project(...)[0] then failed.
        if (!(q[2] > 1e-8)) throw new NullPointerException("Projection of a point with undefined depth");
        lens.projectViewInto(f, q, uv);
        u0 = uv[0];
        u1 = uv[1];
      }
      double r0 = u0 - ox, r1 = u1 - oy;
      double weight = Math.min(1, huber / Math.max(1e-12, Math.hypot(r0, r1)));
      if (lens == null) {
        jq[0] = c.fx / q[2];
        jq[1] = 0;
        jq[2] = -c.fx * q[0] / (q[2] * q[2]);
        jq[3] = 0;
        jq[4] = c.fy / q[2];
        jq[5] = -c.fy * q[1] / (q[2] * q[2]);
      } else {
        lens.spatialViewInto(f, q, jq);
        // Lens Jacobian rows straight into this term's batch slot (row 0, then row 1).
        lens.lensJacobianInto(f, q, uv, oy, jq, batch.jl, 2 * L * term);
      }
      for (int row = 0; row < 2; row++) {
        int a = row * 3, b = row * 6;
        for (int j = 0; j < 3; j++) {
          jp[a + j] = 0;
          for (int k = 0; k < 3; k++) jp[a + j] += jq[a + k] * c.rotation[k * 3 + j];
        }
        jc[b] = -jq[a + 1] * q[2] + jq[a + 2] * q[1];
        jc[b + 1] = jq[a] * q[2] - jq[a + 2] * q[0];
        jc[b + 2] = -jq[a] * q[1] + jq[a + 1] * q[0];
        for (int j = 0; j < 3; j++) jc[b + j + 3] = jq[a + j];
      }
      accumulate(normal, no, rhs, ro, jp, 0, 3, r0, r1, weight);
      int ci = index[o.image];
      if (lens != null) {
        double[] jl = batch.jl;
        int j0 = 2 * L * term, j1 = j0 + L;
        for (int j = 0; j < L; j++)
          for (int k = 0; k < 3; k++)
            lensCross[lo + j * 3 + k] += weight * (jl[j0 + j] * jp[k] + jl[j1 + j] * jp[3 + k]);
      }
      terms++;
      batch.termCamera[term] = ci;
      System.arraycopy(jc, 0, batch.jc, 12 * term, 12);
      batch.rw[3 * term] = r0;
      batch.rw[3 * term + 1] = r1;
      batch.rw[3 * term + 2] = weight;
      if (ci < 0) continue;
      int edge = at + edges++;
      batch.edgeCamera[edge] = ci;
      for (int j = 0; j < 6; j++)
        for (int k = 0; k < 3; k++)
          batch.cross[edge * 18 + j * 3 + k] = weight * (jc[j] * jp[k] + jc[6 + j] * jp[3 + k]);
    }
    if (t.prior != null)
      // Known position (GCP): r = (X - prior) / sigma, J = I / sigma.
      for (int k = 0; k < 3; k++) {
        double w = 1 / (t.priorSigma[k] * t.priorSigma[k]);
        normal[no + k * 3 + k] += w;
        rhs[ro + k] -= w * (t.point[k] - t.prior[k]);
      }
    batch.terms[slot] = terms;
    batch.edges[slot] = edges;
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
    double[] sigma = new double[lens.size()];
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
    Damped system = linearize(frames, tracks, index, count, huber, 1e-10, lens);
    double[][] covariance = new double[lens.size()][lens.size()];
    for (int k = 0; k < lens.size(); k++)
      if (lens.active[k]) {
        double[] rhs = new double[count * 6 + lens.size()];
        rhs[count * 6 + k] = 1;
        Result fit = system.solve(rhs);
        if (fit == null || fit.relative > 1e-3 || !(fit.x[count * 6 + k] > 0)) return null;
        for (int j = 0; j < lens.size(); j++)
          if (lens.active[j]) covariance[j][k] = noise * noise * fit.x[count * 6 + j];
      }
    for (int j = 0; j < lens.size(); j++)
      for (int k = j + 1; k < lens.size(); k++)
        covariance[j][k] = covariance[k][j] = (covariance[j][k] + covariance[k][j]) * .5;
    return covariance;
  }

  /** Undamped observation information, with lens priors excluded. Points are eliminated exactly;
   * the caller must marginalize the remaining camera freedoms, including the scale nullspace. */
  /**
   * matrix/observationDiagonal are null when the assessment failed; failure then says why.
   * excludedTracks: degenerate tracks left out after a singular point block (0 normally).
   */
  record CalibrationInformation(double[] matrix, double[] observationDiagonal, int excludedTracks, String failure) {
    CalibrationInformation(double[] matrix, double[] observationDiagonal) {
      this(matrix, observationDiagonal, 0, null);
    }
  }

  /** Smallest ray angle (radians) a track keeps in the degenerate-track retry below. */
  static final double ASSESSMENT_MIN_RAY_ANGLE = 1e-3;

  /** The information, or null when it cannot be formed (see {@link #calibrationDataAssessment}). */
  static CalibrationInformation calibrationDataInformation(
      List<ImageFrame> frames, List<FeatureTracks.Track> tracks, double huber,
      SharedCalibration lens) {
    var info = calibrationDataAssessment(frames, tracks, huber, lens);
    return info.failure() == null ? info : null;
  }

  /**
   * Undamped lens/camera information. Without damping, one tie point whose rays do not intersect
   * (fewer than two observations in front of registered cameras, or parallel rays: possible
   * before the post-bundle cleanup on a large block) has a singular 3 x 3 block, which made the
   * whole assessment fail. Such a failure is retried once without the degenerate tracks; a
   * successful first attempt is unchanged.
   */
  static CalibrationInformation calibrationDataAssessment(
      List<ImageFrame> frames, List<FeatureTracks.Track> tracks, double huber,
      SharedCalibration lens) {
    int[] index = new int[frames.size()];
    Arrays.fill(index, -1);
    int count = 0;
    for (ImageFrame f : frames) if (f.camera.registered && !f.camera.fixed) index[f.id] = count++;
    int n = count * 6 + lens.size();
    // Keep the same bounded workspace as the bundle solver. Unsupported oversized assessments
    // retain the previous calibration instead of substituting a damped/prior-driven certificate.
    if (n > CalibrationSubset.OBSERVABILITY_UNKNOWNS)
      return new CalibrationInformation(null, null, 0, "Observability workspace limit: " + count
          + " free cameras (" + n + " unknowns > " + CalibrationSubset.OBSERVABILITY_UNKNOWNS + ")");
    if (8L * n * n > frames.get(0).options.memoryMb * 1048576L / 16)
      return new CalibrationInformation(null, null, 0, "Observability workspace exceeds the memory budget");
    boolean before = lens.dataOnly;
    lens.dataOnly = true;
    try {
      try {
        return information(frames, tracks, index, count, n, huber, lens, 0);
      } catch (IllegalStateException singularPoint) {
        List<FeatureTracks.Track> usable = new ArrayList<>(tracks.size());
        double[] q = new double[3];
        for (FeatureTracks.Track t : tracks)
          if (t.point != null && intersecting(frames, t, lens, q)) usable.add(t);
        int excluded = (int) tracks.stream().filter(t -> t.point != null).count() - usable.size();
        if (excluded == 0)
          return new CalibrationInformation(null, null, 0, "Singular structure (no degenerate track found)");
        try {
          return information(frames, usable, index, count, n, huber, lens, excluded);
        } catch (IllegalStateException again) {
          return new CalibrationInformation(null, null, excluded,
              "Singular structure after leaving out " + excluded + " degenerate tracks");
        }
      }
    } finally {
      lens.dataOnly = before;
    }
  }

  private static CalibrationInformation information(List<ImageFrame> frames,
      List<FeatureTracks.Track> tracks, int[] index, int count, int n, double huber,
      SharedCalibration lens, int excluded) {
    // A stand-alone system: the returned reduced matrix is never refilled after this.
    Damped system = linearize(frames, tracks, index, count, huber, 0, lens);
    double[] diagonal = new double[n];
    for (int c = 0; c < count; c++)
      for (int k = 0; k < 6; k++) diagonal[c * 6 + k] = system.cameraNormal[c][k * 6 + k];
    for (int k = 0; k < lens.size(); k++)
      diagonal[count * 6 + k] = system.lensNormal[k * lens.size() + k];
    return new CalibrationInformation(system.reducedMatrix(n), diagonal, excluded, null);
  }

  /**
   * At least two observations in front of registered cameras (the depth test of the
   * linearization) whose rays span at least {@link #ASSESSMENT_MIN_RAY_ANGLE}.
   */
  private static boolean intersecting(List<ImageFrame> frames, FeatureTracks.Track t,
      SharedCalibration lens, double[] q) {
    double[] first = null;
    double minSin = Math.sin(ASSESSMENT_MIN_RAY_ANGLE);
    for (FeatureTracks.Observation o : t.observations) {
      ImageFrame f = frames.get(o.image);
      if (!f.camera.registered) continue;
      lens.viewInto(f, t.point, lens.observed[f.id][2 * o.feature + 1], q);
      if (!(q[2] > 1e-8)) continue;
      double[] ray = Math3.sub(t.point, f.camera.center());
      double length = Math3.norm(ray);
      if (!(length > 0)) continue;
      ray = Math3.times(ray, 1 / length);
      if (first == null) {
        first = ray;
        continue;
      }
      double[] cross = {first[1] * ray[2] - first[2] * ray[1], first[2] * ray[0] - first[0] * ray[2],
          first[0] * ray[1] - first[1] * ray[0]};
      if (Math3.norm(cross) >= minSin) return true;
    }
    return false;
  }

  /**
   * a += w J^T J and b -= w J^T r for one observation with Jacobian rows j[at, at + n) and
   * j[at + n, at + 2n), residual (r0, r1) and weight w: the expressions of the original per-array
   * accumulation, at offsets ao and bo.
   */
  private static void accumulate(
      double[] a, int ao, double[] b, int bo, double[] j, int at, int n, double r0, double r1, double w) {
    for (int x = 0; x < n; x++) {
      b[bo + x] -= w * (j[at + x] * r0 + j[at + n + x] * r1);
      for (int y = 0; y < n; y++)
        a[ao + x * n + y] += w * (j[at + x] * j[at + y] + j[at + n + x] * j[at + n + y]);
    }
  }

  /**
   * accumulateRow for lens row x over a batch's first `observations` flat lens Jacobians (2 rows
   * of SIZE per observation) and residual/weight triples, columns y >= x only: the same
   * expressions, in observation order.
   */
  private static void lensUpperRow(double[] a, double[] b, double[] j, double[] rw, int x, int observations) {
    int n = b.length;
    for (int i = 0; i < observations; i++) {
      int base = i * 2 * n;
      double r0 = rw[3 * i], r1 = rw[3 * i + 1], w = rw[3 * i + 2];
      double j0x = j[base + x], j1x = j[base + n + x];
      b[x] -= w * (j0x * r0 + j1x * r1);
      for (int y = x; y < n; y++) a[x * n + y] += w * (j0x * j[base + y] + j1x * j[base + n + y]);
    }
  }

  private static void damp(double[] a, int n, double lambda) {
    for (int i = 0; i < n; i++) a[i * n + i] += lambda * Math.max(1, a[i * n + i]);
  }

  /**
   * Inverse of the n x n matrix at a[ao..] into out[oo..] (Cholesky, then the general solver, as
   * before); l (n * n) and work (n) are caller-owned scratch. False when singular.
   */
  private static boolean inverseInto(
      double[] a, int ao, int n, double[] out, int oo, double[] l, double[] work) {
    if (SmallMatrices.inverseInto(a, ao, n, out, oo, l, work)) return true;
    double[][] matrix = new double[n][n];
    for (int i = 0; i < n; i++) System.arraycopy(a, ao + i * n, matrix[i], 0, n);
    for (int j = 0; j < n; j++) {
      double[] b = new double[n];
      b[j] = 1;
      double[] x = Math3.solve(matrix, b);
      if (x == null) return false;
      for (int i = 0; i < n; i++) out[oo + i * n + j] = x[i];
    }
    return true;
  }

  private static double dot(double[] a, double[] b) {
    double v = 0;
    for (int i = 0; i < a.length; i++) v += a[i] * b[i];
    return v;
  }

  /**
   * The damped trial system of one fit. It owns only what damping changes (camera and lens
   * normals, point inverses) and shares the undamped system's right-hand sides, couplings, lens
   * cross products and edges read-only. {@link #refill} overwrites every damped value for the next
   * lambda (an exception leaves it unusable until the next refill); the solver scratch below is
   * reused by every solve and product of this workspace and never escapes: Result.x is new.
   */
  private static final class Damped {
    final Linear u;
    final int count, L, n;
    final boolean shared;
    final double[][] cameraNormal;
    final double[] lensNormal;
    /** Per point: inverse of the damped 3 x 3 normal (9), pages as the undamped system's. */
    final double[][] inverse;
    final long workspaceBudget;
    private boolean ready, reducedValid;
    private double[] reduced;
    // Solver scratch. Vectors are exactly n long (dot() runs over whole arrays).
    private final double[] rhs, r, z, p, ap, pointValues;
    private double[] scaled, scale, scaledRhs, lensDiag, lensInverse, lensL, lensWork;
    private double[][] diag, diagInverse;
    private final double[] l6 = new double[36], work6 = new double[6];
    // Fast path only (SolverConfig not legacy). tolerance: relative residual each solve must
    // reach (the legacy 1e-6 unless --bundle-inexact-steps sets it before a solve).
    final SolverConfig config;
    double tolerance = 1e-6;
    /** The dense factor (in `reduced`) belongs to the current refill. */
    private boolean factorValid;
    private double[] factorScale, refine;
    /** Cluster-Jacobi blocks: Jacobi scale and dense factor per cluster of consecutive cameras. */
    private double[][] clusterFactor, clusterScale;
    private boolean clusterReady;

    Damped(Linear u) {
      this.u = u;
      count = u.count;
      L = u.L;
      shared = u.shared;
      n = count * 6 + (shared ? L : 0);
      cameraNormal = new double[count][36];
      lensNormal = new double[L * L];
      inverse = pages(u.pointCapacity, 9, u.shift);
      long directBudget = Math.min(32L * 1048576, u.memoryBudget / 16);
      workspaceBudget = Math.max(0, Math.min(directBudget,
          Long.getLong("sgm.test.bundleDirectWorkspaceBytes", directBudget)));
      rhs = new double[n];
      r = new double[n];
      z = new double[n];
      p = new double[n];
      ap = new double[n];
      pointValues = new double[3 * u.pointCapacity];
      config = u.config;
    }

    int points() {
      return u.points;
    }

    /** The undamped system changed: unusable until the next refill. */
    void invalidate() {
      ready = reducedValid = false;
      factorValid = clusterReady = false;
    }

    /**
     * This workspace's undamped system with LM damping applied, as the single-pass linearization
     * produced it: the same sums plus lambda on the diagonals, point blocks inverted after damping.
     */
    void refill(double lambda) {
      invalidate();
      for (int c = 0; c < count; c++) {
        System.arraycopy(u.cameraNormal[c], 0, cameraNormal[c], 0, 36);
        damp(cameraNormal[c], 6, lambda);
      }
      System.arraycopy(u.lensNormal, 0, lensNormal, 0, lensNormal.length);
      if (shared) damp(lensNormal, L, lambda);
      int points = u.points, chunks = chunks(points, 0);
      parallel(chunks, chunk -> {
        double[] a = new double[9], l = new double[9], work = new double[3];
        int from = (int) ((long) points * chunk / chunks), to = (int) ((long) points * (chunk + 1) / chunks);
        for (int i = from; i < to; i++) {
          int page = i >>> u.shift, local = i & u.mask;
          System.arraycopy(u.normal[page], local * 9, a, 0, 9);
          damp(a, 3, lambda);
          if (!inverseInto(a, 0, 3, inverse[page], local * 9, l, work))
            throw new IllegalStateException("Singular damped point block");
        }
      });
      ready = true;
    }

    private void requireReady() {
      if (!ready) throw new IllegalStateException("Damped bundle system used before refill");
    }

    /** Assemble only the small camera/lens system; points are still eliminated analytically. */
    private double[] reducedMatrix(int n) {
      requireReady();
      if (reducedValid && reduced.length == n * n) return reduced;
      if (reduced == null || reduced.length != n * n) {
        reduced = null;
        reduced = new double[n * n];
      } else Arrays.fill(reduced, 0);
      double[] h = reduced;
      for (int c = 0; c < count; c++)
        for (int j = 0; j < 6; j++)
          for (int k = 0; k < 6; k++) h[(c * 6 + j) * n + c * 6 + k] = cameraNormal[c][j * 6 + k];
      if (shared) {
        int off = count * 6;
        for (int j = 0; j < L; j++)
          for (int k = 0; k < L; k++)
            h[(off + j) * n + off + k] = lensNormal[j * L + k];
        for (int c = 0; c < count; c++)
          for (int j = 0; j < 6; j++)
            for (int k = 0; k < L; k++)
              h[(c * 6 + j) * n + off + k] =
                  h[(off + k) * n + c * 6 + j] = u.coupling[c][j * L + k];
      }
      // Point eliminations, parallel by destination rows: each camera's (and the lens') rows are
      // written by one task that visits the points in their original order, so every element
      // receives the same subtractions in the same order as the serial loop (bit-identical).
      // Lens rows are partitioned by destination columns: one task owns lens rows x lens columns
      // (all points in order); each camera task also owns lens rows x its 6 columns, visiting its
      // (point, edge) list in point/edge order. Every element receives the same subtractions, with
      // the same values, in the same order as one task per lens row over all points (which made
      // 13 tasks traverse every point and edge).
      final int off = count * 6;
      parallel(count + (shared ? 1 : 0), task -> {
        if (task >= count) {
          double[] v = new double[3];
          for (int point = 0; point < u.points; point++) {
            double[] lc = u.lensCross[point >>> u.shift];
            int lo = (point & u.mask) * 3 * L;
            for (int j = 0; j < L; j++) {
              lensRowTimesInverse(point, j, v);
              int row = (off + j) * n + off;
              for (int k = 0; k < L; k++)
                h[row + k] -= v[0] * lc[lo + k * 3] + v[1] * lc[lo + k * 3 + 1] + v[2] * lc[lo + k * 3 + 2];
            }
          }
          return;
        }
        for (int at = u.cameraStart[task]; at < u.cameraStart[task + 1]; at++) {
          int e = u.cameraEdge[at];
          subtractRows(h, n, u.cameraPoint[at], u.cross[e >>> u.shift], (e & u.mask) * 18, task * 6, 6);
        }
        if (shared) {
          double[] v = new double[3];
          for (int at = u.cameraStart[task]; at < u.cameraStart[task + 1]; at++) {
            int e = u.cameraEdge[at], co = (e & u.mask) * 18;
            double[] cross = u.cross[e >>> u.shift];
            for (int j = 0; j < L; j++) {
              lensRowTimesInverse(u.cameraPoint[at], j, v);
              int row = (off + j) * n + task * 6;
              for (int k = 0; k < 6; k++)
                h[row + k] -= v[0] * cross[co + k * 3] + v[1] * cross[co + k * 3 + 1] + v[2] * cross[co + k * 3 + 2];
            }
          }
        }
      });
      // The mathematical Schur matrix is symmetric. Remove only round-off asymmetry.
      for (int i = 0; i < n; i++)
        for (int j = 0; j < i; j++)
          h[i * n + j] = h[j * n + i] = .5 * (h[i * n + j] + h[j * n + i]);
      reducedValid = true;
      return h;
    }

    /**
     * target[lens rows] -= lensCross * w[p] and target[camera rows] -= cross * w[p] for every point
     * p (w holds 3 values per point). Parallel by destination rows; each owner visits points in
     * their original order, so the result is bit-identical to the serial point loop.
     */
    private void scatter(double[] target, double[] w) {
      // All lens rows in one pass over the points: each lens row element still receives its
      // subtractions in point order, then k order (identical to one task per lens row).
      parallel(count + (shared ? 1 : 0), task -> {
        if (task >= count) {
          for (int point = 0; point < u.points; point++) {
            double[] lc = u.lensCross[point >>> u.shift];
            int lo = (point & u.mask) * 3 * L;
            for (int j = 0; j < L; j++) {
              int row = count * 6 + j;
              for (int k = 0; k < 3; k++) target[row] -= lc[lo + j * 3 + k] * w[3 * point + k];
            }
          }
          return;
        }
        for (int at = u.cameraStart[task]; at < u.cameraStart[task + 1]; at++) {
          int e = u.cameraEdge[at], point = u.cameraPoint[at], co = (e & u.mask) * 18;
          double[] cross = u.cross[e >>> u.shift];
          for (int j = 0; j < 6; j++)
            for (int k = 0; k < 3; k++) target[task * 6 + j] -= cross[co + j * 3 + k] * w[3 * point + k];
        }
      });
    }

    /** v = (lens row j of the point's lens cross) * inverse, as subtractRow computes it. */
    private void lensRowTimesInverse(int point, int j, double[] v) {
      double[] left = u.lensCross[point >>> u.shift], inv = inverse[point >>> u.shift];
      int lo = (point & u.mask) * 3 * L, io = (point & u.mask) * 9;
      double v0 = 0, v1 = 0, v2 = 0;
      for (int k = 0; k < 3; k++) {
        v0 += left[lo + j * 3 + k] * inv[io + k * 3];
        v1 += left[lo + j * 3 + k] * inv[io + k * 3 + 1];
        v2 += left[lo + j * 3 + k] * inv[io + k * 3 + 2];
      }
      v[0] = v0;
      v[1] = v1;
      v[2] = v2;
    }

    private void subtractRows(double[] h, int n, int point, double[] left, int lo, int offset, int rows) {
      for (int j = 0; j < rows; j++) subtractRow(h, n, point, left, lo, offset, j);
    }

    /** Row j of subtractRows (rows are independent). */
    private void subtractRow(double[] h, int n, int point, double[] left, int lo, int offset, int j) {
      double[] inv = inverse[point >>> u.shift];
      int io = (point & u.mask) * 9;
      double v0 = 0, v1 = 0, v2 = 0;
      for (int k = 0; k < 3; k++) {
        v0 += left[lo + j * 3 + k] * inv[io + k * 3];
        v1 += left[lo + j * 3 + k] * inv[io + k * 3 + 1];
        v2 += left[lo + j * 3 + k] * inv[io + k * 3 + 2];
      }
      int row = (offset + j) * n;
      if (shared) {
        double[] lc = u.lensCross[point >>> u.shift];
        int pl = (point & u.mask) * 3 * L;
        for (int k = 0; k < L; k++)
          h[row + count * 6 + k] -= v0 * lc[pl + k * 3] + v1 * lc[pl + k * 3 + 1] + v2 * lc[pl + k * 3 + 2];
      }
      for (int e = u.edgeStart[point]; e < u.edgeStart[point + 1]; e++) {
        double[] cross = u.cross[e >>> u.shift];
        int co = (e & u.mask) * 18, camera = u.edgeCamera[e];
        for (int k = 0; k < 6; k++)
          h[row + camera * 6 + k] -= v0 * cross[co + k * 3] + v1 * cross[co + k * 3 + 1] + v2 * cross[co + k * 3 + 2];
      }
    }

    private Result direct(double[] rhs) {
      int n = rhs.length;
      // Includes Java matrix, scaled matrix, native matrix/factor and temporary solve space.
      if (n == 0 || n > 768 || 4L * n * n * Double.BYTES > workspaceBudget) return null;
      double[] h = reducedMatrix(n);
      if (scaled == null || scaled.length != n * n) {
        scaled = null;
        scaled = new double[n * n];
        scale = new double[n];
        scaledRhs = new double[n];
      }
      double[] scaled = this.scaled, scale = this.scale, b = scaledRhs;
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
        double[] residual = ap;
        productInto(x, residual);
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

    /** The reduced operator applied to x, as a new array (tests and diagnostics). */
    double[] product(double[] x) {
      double[] out = new double[x.length];
      productInto(x, out);
      return out;
    }

    /** out = reduced operator * x. Uses the point scratch; out must not be x. */
    private void productInto(double[] x, double[] out) {
      requireReady();
      Arrays.fill(out, 0);
      for (int i = 0; i < count; i++)
        for (int j = 0; j < 6; j++)
          for (int k = 0; k < 6; k++) out[i * 6 + j] += cameraNormal[i][j * 6 + k] * x[i * 6 + k];
      if (shared) {
        int off = count * 6;
        for (int j = 0; j < L; j++)
          for (int k = 0; k < L; k++)
            out[off + j] += lensNormal[j * L + k] * x[off + k];
        for (int c = 0; c < count; c++)
          for (int j = 0; j < 6; j++)
            for (int k = 0; k < L; k++) {
              out[c * 6 + j] += u.coupling[c][j * L + k] * x[off + k];
              out[off + k] += u.coupling[c][j * L + k] * x[c * 6 + j];
            }
      }
      // Per-point u = inverse * (cross^T x) in parallel (pure), then subtracted serially in point
      // order: bit-identical to the serial loop. Every point writes its three values.
      int pointCount = u.points, chunks = chunks(pointCount, 0);
      double[] us = pointValues;
      parallel(chunks, chunk -> {
        double[] v = new double[3];
        int from = (int) ((long) pointCount * chunk / chunks), to = (int) ((long) pointCount * (chunk + 1) / chunks);
        for (int point = from; point < to; point++) {
          int page = point >>> u.shift, local = point & u.mask;
          Arrays.fill(v, 0);
          if (shared) {
            double[] lc = u.lensCross[page];
            int lo = local * 3 * L;
            for (int k = 0; k < 3; k++)
              for (int j = 0; j < L; j++)
                v[k] += lc[lo + j * 3 + k] * x[count * 6 + j];
          }
          for (int e = u.edgeStart[point]; e < u.edgeStart[point + 1]; e++) {
            double[] cross = u.cross[e >>> u.shift];
            int co = (e & u.mask) * 18, camera = u.edgeCamera[e];
            for (int k = 0; k < 3; k++)
              for (int j = 0; j < 6; j++) v[k] += cross[co + j * 3 + k] * x[camera * 6 + j];
          }
          double[] inv = inverse[page];
          int io = local * 9;
          for (int i = 0; i < 3; i++) {
            double s = 0;
            for (int j = 0; j < 3; j++) s += inv[io + i * 3 + j] * v[j];
            us[3 * point + i] = s;
          }
        }
      });
      scatter(out, us);
    }

    Result solve() {
      return solve(null);
    }

    Result solve(double[] suppliedRhs) {
      requireReady();
      int n = this.n;
      double[] rhs = this.rhs;
      // Every element is assigned: camera rows, then the lens rows when shared.
      if (shared) System.arraycopy(u.lensRhs, 0, rhs, count * 6, L);
      for (int c = 0; c < count; c++) System.arraycopy(u.cameraRhs[c], 0, rhs, c * 6, 6);
      // Reduced right-hand side: per-point v in parallel, ordered scatter (bit-identical).
      int pointCount = u.points, chunks = chunks(pointCount, 0);
      double[] vs = pointValues;
      parallel(chunks, chunk -> {
        int from = (int) ((long) pointCount * chunk / chunks), to = (int) ((long) pointCount * (chunk + 1) / chunks);
        for (int point = from; point < to; point++) {
          int page = point >>> u.shift, local = point & u.mask;
          double[] inv = inverse[page], b = u.rhs[page];
          int io = local * 9, bo = local * 3;
          for (int i = 0; i < 3; i++) {
            double s = 0;
            for (int j = 0; j < 3; j++) s += inv[io + i * 3 + j] * b[bo + j];
            vs[3 * point + i] = s;
          }
        }
      });
      scatter(rhs, vs);
      double[] solveRhs = suppliedRhs == null ? rhs : suppliedRhs;
      if (config.directLimit < 0) {
        Result direct = direct(solveRhs);
        if (direct != null) return direct;
      } else if (config.dense(n)) {
        Result dense = denseDirect(solveRhs);
        if (dense != null) return dense;
      }
      String method = config.clusterJacobi ? "cluster_jacobi_pcg" : "matrix_free_pcg";
      if (config.clusterJacobi) {
        if (!clusterPreconditioner()) return null;
      } else {
      // Block-Jacobi preconditioner: only the PCG fallback needs it.
      if (diag == null) {
        diag = new double[count][36];
        diagInverse = new double[count][36];
        lensDiag = new double[L * L];
        lensInverse = new double[L * L];
        lensL = new double[L * L];
        lensWork = new double[L];
      }
      for (int c = 0; c < count; c++) System.arraycopy(cameraNormal[c], 0, diag[c], 0, 36);
      System.arraycopy(lensNormal, 0, lensDiag, 0, lensNormal.length);
      for (int point = 0; point < pointCount; point++) {
        int page = point >>> u.shift, local = point & u.mask;
        double[] inv = inverse[page];
        int io = local * 9;
        if (shared) {
          double[] lc = u.lensCross[page];
          int lo = local * 3 * L;
          for (int j = 0; j < L; j++)
            for (int l = 0; l < L; l++)
              for (int k = 0; k < 3; k++)
                for (int m = 0; m < 3; m++)
                  lensDiag[j * L + l] -= lc[lo + j * 3 + k] * inv[io + k * 3 + m] * lc[lo + l * 3 + m];
        }
        for (int e = u.edgeStart[point]; e < u.edgeStart[point + 1]; e++) {
          double[] cross = u.cross[e >>> u.shift], d = diag[u.edgeCamera[e]];
          int co = (e & u.mask) * 18;
          for (int j = 0; j < 6; j++)
            for (int l = 0; l < 6; l++)
              for (int k = 0; k < 3; k++)
                for (int m = 0; m < 3; m++)
                  d[j * 6 + l] -= cross[co + j * 3 + k] * inv[io + k * 3 + m] * cross[co + l * 3 + m];
        }
      }
      for (int c = 0; c < count; c++)
        if (!inverseInto(diag[c], 0, 6, diagInverse[c], 0, l6, work6)) return null;
      if (shared && !inverseInto(lensDiag, 0, L, lensInverse, 0, lensL, lensWork)) return null;
      }
      // The right-hand side is only read from here on (the original cloned a supplied one).
      double[] b = solveRhs;
      double[] x = new double[n], r = this.r, z = this.z, p = this.p, ap = this.ap;
      System.arraycopy(b, 0, r, 0, n);
      preconditionInto(r, z);
      System.arraycopy(z, 0, p, 0, n);
      double norm = Math.sqrt(dot(b, b));
      if (norm < 1e-15) return new Result(x, 0, 0, method);
      double rz = dot(r, z), relative = 1;
      int limit = Math.min(500, Math.max(30, n * 2));
      for (int iter = 0; iter < limit; iter++) {
        productInto(p, ap);
        double curvature = dot(p, ap);
        if (!(curvature > 0) || !Double.isFinite(curvature)) return null;
        double alpha = rz / curvature;
        for (int i = 0; i < n; i++) {
          x[i] += alpha * p[i];
          r[i] -= alpha * ap[i];
        }
        relative = Math.sqrt(dot(r, r)) / norm;
        if (!Double.isFinite(relative)) return null;
        if (relative < tolerance) return new Result(x, iter + 1, relative, method);
        preconditionInto(r, z);
        double next = dot(r, z), beta = next / rz;
        for (int i = 0; i < n; i++) p[i] = z[i] + beta * p[i];
        rz = next;
      }
      // An inexact step is still checked against the full robust objective before acceptance.
      return new Result(x, limit, relative, method);
    }

    /**
     * Fast path: the reduced matrix (assembled as for the legacy direct solve), Jacobi-scaled and
     * factored in place by the parallel dense Cholesky, kept for every solve of this refill.
     * Each solution is certified against the matrix-free operator and refined (at most three
     * times) until its relative residual meets the tolerance; null when the factorization fails
     * or the tolerance is not met (the caller then uses PCG).
     */
    private Result denseDirect(double[] rhs) {
      int n = rhs.length;
      if (!factorValid) {
        double[] h = reducedMatrix(n);
        // The buffer holds the factor from here on, not the reduced matrix.
        reducedValid = false;
        if (factorScale == null || factorScale.length != n) factorScale = new double[n];
        double[] scale = factorScale;
        for (int i = 0; i < n; i++) {
          if (!(h[i * n + i] > 0) || !Double.isFinite(h[i * n + i])) return null;
          scale[i] = Math.sqrt(h[i * n + i]);
        }
        for (int i = 0; i < n; i++)
          for (int j = 0, row = i * n; j <= i; j++) h[row + j] = h[row + j] / scale[i] / scale[j];
        if (!DenseCholesky.factor(h, n, true)) return null;
        factorValid = true;
      }
      double norm = Math.sqrt(dot(rhs, rhs));
      double[] x = new double[n];
      if (norm < 1e-15) return new Result(x, 0, 0, "dense_cholesky");
      if (refine == null || refine.length != n) refine = new double[n];
      double[] residual = ap, correction = refine;
      System.arraycopy(rhs, 0, residual, 0, n);
      double relative = Double.NaN;
      int refinements = 0;
      for (int pass = 0; pass <= 3; pass++) {
        factorSolve(residual, correction);
        for (int i = 0; i < n; i++) x[i] += correction[i];
        // Certify against the original matrix-free operator, not the factored matrix.
        productInto(x, residual);
        for (int i = 0; i < n; i++) residual[i] = rhs[i] - residual[i];
        relative = Math.sqrt(dot(residual, residual)) / norm;
        if (!Double.isFinite(relative)) return null;
        refinements = pass;
        if (relative < tolerance) break;
      }
      return relative < tolerance ? new Result(x, refinements, relative, "dense_cholesky") : null;
    }

    /** out = (scaled factor)^-1 b in the unscaled variables. */
    private void factorSolve(double[] b, double[] out) {
      int n = b.length;
      double[] scale = factorScale;
      for (int i = 0; i < n; i++) out[i] = b[i] / scale[i];
      DenseCholesky.solve(reduced, n, out, out);
      for (int i = 0; i < n; i++) out[i] /= scale[i];
    }

    /**
     * Marginal covariance diagonal of the lens variables of this refill's damped system (in the
     * normal-equation units), from the dense factor: the lens rows are the trailing block. Null
     * without a valid dense factor or a shared lens.
     */
    double[] lensMarginalVariance() {
      if (!shared || !factorValid) return null;
      double[] inverse = DenseCholesky.trailingInverse(reduced, n, L);
      if (inverse == null) return null;
      double[] variance = new double[L];
      for (int k = 0; k < L; k++) {
        double s = factorScale[count * 6 + k];
        variance[k] = inverse[k * L + k] / (s * s);
      }
      return variance;
    }

    /**
     * Marginal variance of every reduced variable (cameras, then lens) of this refill's damped
     * system, in normal-equation units, from the dense factor. Null without a valid factor.
     */
    double[] marginalVariances() {
      if (!factorValid) return null;
      double[] variance = DenseCholesky.inverseDiagonal(reduced, n, true);
      for (int i = 0; i < n; i++) variance[i] /= factorScale[i] * factorScale[i];
      return variance;
    }

    /**
     * Cluster-Jacobi preconditioner: the reduced matrix restricted to blocks of
     * {@link #CLUSTER_CAMERAS} consecutive cameras (each camera's own block plus the point
     * coupling between cameras of the same cluster), each Jacobi-scaled and Cholesky-factored;
     * the lens block as in block-Jacobi. One task per cluster visits its cameras' (point, edge)
     * pairs in order, so the blocks do not depend on the thread count. False when singular.
     */
    private boolean clusterPreconditioner() {
      if (clusterReady) return true;
      int clusters = (count + CLUSTER_CAMERAS - 1) / CLUSTER_CAMERAS;
      if (clusterFactor == null || clusterFactor.length != clusters) {
        clusterFactor = new double[clusters][];
        clusterScale = new double[clusters][];
        for (int g = 0; g < clusters; g++) {
          int m = 6 * (Math.min(count, (g + 1) * CLUSTER_CAMERAS) - g * CLUSTER_CAMERAS);
          clusterFactor[g] = new double[m * m];
          clusterScale[g] = new double[m];
        }
      }
      boolean[] failed = new boolean[clusters];
      parallel(clusters, g -> {
        int c0 = g * CLUSTER_CAMERAS, c1 = Math.min(count, c0 + CLUSTER_CAMERAS), m = 6 * (c1 - c0);
        double[] block = clusterFactor[g], scale = clusterScale[g], v = new double[18];
        Arrays.fill(block, 0);
        for (int c = c0; c < c1; c++)
          for (int j = 0; j < 6; j++)
            for (int k = 0; k < 6; k++)
              block[((c - c0) * 6 + j) * m + (c - c0) * 6 + k] = cameraNormal[c][j * 6 + k];
        for (int c = c0; c < c1; c++)
          for (int at = u.cameraStart[c]; at < u.cameraStart[c + 1]; at++) {
            int e = u.cameraEdge[at], point = u.cameraPoint[at], co = (e & u.mask) * 18;
            double[] cross = u.cross[e >>> u.shift], inv = inverse[point >>> u.shift];
            int io = (point & u.mask) * 9;
            for (int j = 0; j < 6; j++)
              for (int q = 0; q < 3; q++) {
                double s = 0;
                for (int k = 0; k < 3; k++) s += cross[co + j * 3 + k] * inv[io + k * 3 + q];
                v[j * 3 + q] = s;
              }
            for (int e2 = u.edgeStart[point]; e2 < u.edgeStart[point + 1]; e2++) {
              int c2 = u.edgeCamera[e2];
              if (c2 < c0 || c2 >= c1) continue;
              double[] cross2 = u.cross[e2 >>> u.shift];
              int o2 = (e2 & u.mask) * 18;
              for (int j = 0; j < 6; j++) {
                int row = ((c - c0) * 6 + j) * m + (c2 - c0) * 6;
                for (int k = 0; k < 6; k++)
                  block[row + k] -= v[j * 3] * cross2[o2 + k * 3] + v[j * 3 + 1] * cross2[o2 + k * 3 + 1]
                      + v[j * 3 + 2] * cross2[o2 + k * 3 + 2];
              }
            }
          }
        for (int i = 0; i < m; i++) {
          double d = block[i * m + i];
          if (!(d > 0) || !Double.isFinite(d)) {
            failed[g] = true;
            return;
          }
          scale[i] = Math.sqrt(d);
        }
        for (int i = 0; i < m; i++)
          for (int j = 0; j <= i; j++) block[i * m + j] = block[i * m + j] / scale[i] / scale[j];
        if (!DenseCholesky.factor(block, m, false)) failed[g] = true;
      });
      for (boolean f : failed) if (f) return false;
      if (shared) {
        if (lensDiag == null) {
          lensDiag = new double[L * L];
          lensInverse = new double[L * L];
          lensL = new double[L * L];
          lensWork = new double[L];
        }
        System.arraycopy(lensNormal, 0, lensDiag, 0, lensNormal.length);
        for (int point = 0; point < u.points; point++) {
          int page = point >>> u.shift, local = point & u.mask;
          double[] inv = inverse[page], lc = u.lensCross[page];
          int io = local * 9, lo = local * 3 * L;
          for (int j = 0; j < L; j++)
            for (int l = 0; l < L; l++)
              for (int k = 0; k < 3; k++)
                for (int m = 0; m < 3; m++)
                  lensDiag[j * L + l] -= lc[lo + j * 3 + k] * inv[io + k * 3 + m] * lc[lo + l * 3 + m];
        }
        if (!inverseInto(lensDiag, 0, L, lensInverse, 0, lensL, lensWork)) return false;
      }
      clusterReady = true;
      return true;
    }

    /** z = preconditioner applied to r (z fully rewritten). */
    private void preconditionInto(double[] r, double[] z) {
      if (config.clusterJacobi) {
        clusterPreconditionInto(r, z);
        return;
      }
      Arrays.fill(z, 0);
      for (int c = 0; c < count; c++)
        for (int j = 0; j < 6; j++)
          for (int k = 0; k < 6; k++) z[c * 6 + j] += diagInverse[c][j * 6 + k] * r[c * 6 + k];
      if (shared)
        for (int j = 0; j < L; j++)
          for (int k = 0; k < L; k++)
            z[count * 6 + j] += lensInverse[j * L + k] * r[count * 6 + k];
    }

    /** Cluster-Jacobi: z = blockdiag(clusters, lens)^-1 r, serial in cluster order. */
    private void clusterPreconditionInto(double[] r, double[] z) {
      for (int g = 0; g < clusterFactor.length; g++) {
        int o = 6 * g * CLUSTER_CAMERAS;
        double[] scale = clusterScale[g];
        int m = scale.length;
        double[] block = clusterFactor[g], w = work(m);
        for (int i = 0; i < m; i++) w[i] = r[o + i] / scale[i];
        DenseCholesky.solve(block, m, w, w);
        for (int i = 0; i < m; i++) z[o + i] = w[i] / scale[i];
      }
      if (shared) {
        int off = count * 6;
        for (int j = 0; j < L; j++) {
          z[off + j] = 0;
          for (int k = 0; k < L; k++) z[off + j] += lensInverse[j * L + k] * r[off + k];
        }
      }
    }

    private double[] clusterWork;

    private double[] work(int m) {
      if (clusterWork == null || clusterWork.length < m) clusterWork = new double[m];
      return clusterWork;
    }

    /**
     * Every point to a new array: X + inverse (b - cross^T step - lensCross^T step), then the
     * similarity origin + (X' - origin) * factor. The same operations, in the same order, as the
     * former separate update and rescale (each of which allocated new arrays). The previous arrays
     * are left untouched for an exact restore.
     */
    void movePoints(double[] step, double[] origin, double factor) {
      requireReady();
      int cameras = count, pointCount = u.points, chunks = chunks(pointCount, 0);
      parallel(chunks, chunk -> {
        double[] rhs = new double[3];
        int from = (int) ((long) pointCount * chunk / chunks), to = (int) ((long) pointCount * (chunk + 1) / chunks);
        for (int point = from; point < to; point++) {
          int page = point >>> u.shift, local = point & u.mask;
          System.arraycopy(u.rhs[page], local * 3, rhs, 0, 3);
          for (int e = u.edgeStart[point]; e < u.edgeStart[point + 1]; e++) {
            double[] cross = u.cross[e >>> u.shift];
            int co = (e & u.mask) * 18, camera = u.edgeCamera[e];
            for (int k = 0; k < 3; k++)
              for (int j = 0; j < 6; j++) rhs[k] -= cross[co + j * 3 + k] * step[camera * 6 + j];
          }
          if (shared) {
            double[] lc = u.lensCross[page];
            int lo = local * 3 * L;
            for (int k = 0; k < 3; k++)
              for (int j = 0; j < L; j++) rhs[k] -= lc[lo + j * 3 + k] * step[cameras * 6 + j];
          }
          double[] inv = inverse[page], old = u.owners[point].point, moved = new double[3];
          int io = local * 9;
          for (int i = 0; i < 3; i++) {
            double d = 0;
            for (int j = 0; j < 3; j++) d += inv[io + i * 3 + j] * rhs[j];
            moved[i] = origin[i] + (old[i] + d - origin[i]) * factor;
          }
          u.owners[point].point = moved;
        }
      });
    }
  }
}
