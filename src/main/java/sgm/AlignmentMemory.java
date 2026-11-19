package sgm;

import java.util.*;

/** Conservative 64-bit-reference estimates; large sparse graphs must be admitted, not blindly loaded. */
final class AlignmentMemory {
  static long graphBytes(long keypoints, long tracks, long observations, long pairs, long matches) {
    return Math.addExact(Math.addExact(Math.multiplyExact(keypoints, 128), Math.multiplyExact(tracks, 160)),
        Math.addExact(Math.multiplyExact(observations, 48),
            Math.addExact(Math.multiplyExact(pairs, 512), Math.multiplyExact(matches, 32))));
  }

  static void checkGraph(List<ImageFrame> frames, List<FeatureTracks.Track> tracks,
      List<FeatureTracks.Pair> pairs, Options options) {
    long keys = 0, observations = 0, matches = 0;
    for (var frame : frames) keys += frame.keypoints.length;
    for (var track : tracks) observations += track.observations.size();
    for (var pair : pairs) matches += pair.matches.size();
    long estimate = graphBytes(keys, tracks.size(), observations, pairs.size(), matches);
    if (estimate > options.memoryMb * MemorySafety.MIB)
      throw new IllegalArgumentException("Sparse alignment graph needs about " + estimate / MemorySafety.MIB
          + " MiB before bundle workspace, above --memory-mb " + options.memoryMb
          + ". Reduce --features or split the project into blocks; increasing -Xmx alone does not bound native RAM.");
  }

  /**
   * Bundle bytes per triangulated point with lens width L (0 without shared calibration, 13 per
   * camera group with it). Undamped system: normal, right-hand side and lens cross products
   * ((12 + 3 L) doubles), edge offset (int) and track reference. Damped trial system: inverse (9
   * doubles) and solver point scratch (3). Trial: the saved point reference, plus the new point
   * array (40 bytes) that coexists with the saved one until the trial is decided.
   */
  static long bundlePointBytes(int lensWidth) {
    return 252 + 24L * lensWidth;
  }

  /** Per variable-camera observation (edge): 6 x 3 cross product, camera index, camera-order point and edge indices. */
  static final long BUNDLE_EDGE_BYTES = 18 * 8 + 3 * 4;

  /**
   * Per variable camera: undamped normal, right-hand side and lens coupling (6 L doubles), the
   * damped normal, preconditioner block and its inverse, five PCG vector entries, the solution,
   * and the per-trial camera copy, with array headers.
   */
  static long bundleCameraBytes(int lensWidth) {
    return 2048 + 48L * lensWidth;
  }

  static long bundleBytes(long points, long variableObservations, int cameras, int lensWidth) {
    return Math.addExact(Math.multiplyExact(points, bundlePointBytes(lensWidth)),
        Math.addExact(Math.multiplyExact(variableObservations, BUNDLE_EDGE_BYTES),
            Math.multiplyExact((long) cameras, bundleCameraBytes(lensWidth))));
  }

  /**
   * Bundle workspaces whose size does not grow with the block: the lens matrices (undamped,
   * damped, preconditioner, inverse, factor scratch), one bounded Jacobian batch (camera and lens
   * Jacobian rows, residual/weight and edge products per observation slot, plus per-track
   * offsets) sized as allocated (at most the batch limit, at most the observations), the
   * objective's cost batch, and the direct Schur workspace (reduced and scaled matrices plus their
   * native copies, 4 n^2 doubles) when the direct solve is eligible.
   */
  static long bundleWorkspaceBytes(Options options, int lensWidth, int cameras, long observations) {
    long L = lensWidth > 0 ? lensWidth : SharedCalibration.SIZE;
    long slots = Math.max(1, Math.min(observations, BundleAdjuster.linearizationObservationLimit(options, lensWidth > 0)));
    long batch = Math.addExact(Math.multiplyExact(slots, (33 + 2L * lensWidth) * 8 + 3 * 4 + 8),
        16384L * 16);
    long n = 6L * cameras + lensWidth, budget = Math.min(32L * MemorySafety.MIB, options.memoryMb * MemorySafety.MIB / 16);
    long direct = n <= 768 && 4 * n * n * Double.BYTES <= budget ? 4 * n * n * Double.BYTES : 0;
    return Math.addExact(Math.addExact(40 * L * L, batch), direct);
  }

  static void checkBundle(List<ImageFrame> frames, List<FeatureTracks.Track> tracks, int[] index,
      int cameras, int lensWidth) {
    long points = 0, observations = 0, all = 0;
    for (var track : tracks) if (track.point != null) {
      points++;
      all += track.observations.size();
      for (var ob : track.observations) if (index[ob.image] >= 0) observations++;
    }
    Options options = frames.get(0).options;
    MemorySafety.requireHeap(
        Math.addExact(bundleBytes(points, observations, cameras, lensWidth),
            bundleWorkspaceBytes(options, lensWidth, cameras, all)),
        options,
        "Joint bundle adjustment (" + points + " points, " + observations + " variable-camera observations)");
  }
}
