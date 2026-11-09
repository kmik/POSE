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

  static long bundleBytes(long points, long variableObservations, int cameras, boolean shared) {
    // Undamped and damped point blocks, edge products, trial state and solver scratch.
    return Math.addExact(Math.multiplyExact(points, shared ? 1152 : 768),
        Math.addExact(Math.multiplyExact(variableObservations, 256), Math.multiplyExact((long) cameras, 4096)));
  }

  static void checkBundle(List<ImageFrame> frames, List<FeatureTracks.Track> tracks, int[] index,
      int cameras, boolean shared) {
    long points = 0, observations = 0;
    for (var track : tracks) if (track.point != null) {
      points++;
      for (var ob : track.observations) if (index[ob.image] >= 0) observations++;
    }
    MemorySafety.requireHeap(bundleBytes(points, observations, cameras, shared), frames.get(0).options,
        "Joint bundle adjustment (" + points + " points, " + observations + " variable-camera observations)");
  }
}
