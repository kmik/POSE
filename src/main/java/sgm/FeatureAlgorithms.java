package sgm;

import java.util.*;
import java.util.function.Consumer;
import org.opencv.core.*;
import org.opencv.features2d.*;

/** Descriptor metric is selected by actual descriptor storage, including calibration previews. */
final class FeatureAlgorithms {
  private static final ThreadLocal<Workspace> WORK = new ThreadLocal<>();

  private static final class Workspace implements AutoCloseable {
    final OpenCvKnn nativeSearch = new OpenCvKnn();
    final KnnMatches backward = new KnnMatches();
    final Mat queries = new Mat();
    final BitSet needed = new BitSet();
    final Map<String, DescriptorMatcher> matchers = new HashMap<>();
    float[] floatRow = new float[0];
    byte[] byteRow = new byte[0];

    DescriptorMatcher matcher(Options o, Mat descriptors) {
      String key = descriptors.depth() == CvType.CV_8U ? "hamming" : o.descriptorMatcher;
      return matchers.computeIfAbsent(key, k -> FeatureAlgorithms.matcher(o, descriptors));
    }

    void gather(Mat source, int[] ids) {
      queries.create(ids.length, source.cols(), source.type());
      if (source.depth() == CvType.CV_8U) {
        if (byteRow.length != source.cols()) byteRow = new byte[source.cols()];
        for (int i = 0; i < ids.length; i++) { source.get(ids[i], 0, byteRow); queries.put(i, 0, byteRow); }
      } else {
        if (floatRow.length != source.cols()) floatRow = new float[source.cols()];
        for (int i = 0; i < ids.length; i++) { source.get(ids[i], 0, floatRow); queries.put(i, 0, floatRow); }
      }
    }

    @Override public void close() {
      queries.release(); nativeSearch.close();
      for (DescriptorMatcher matcher : matchers.values()) matcher.clear();
      matchers.clear();
    }
  }

  static void closeWorkspace() {
    Workspace workspace = WORK.get();
    WORK.remove();
    if (workspace != null) workspace.close();
  }

  private static Workspace workspace() {
    Workspace workspace = WORK.get();
    if (workspace == null) { workspace = new Workspace(); WORK.set(workspace); }
    return workspace;
  }
  static Feature2D detector(Options o, int maximum) {
    return switch (o.matchingAlgorithm) {
      case "orb" -> {
        ORB detector = ORB.create(maximum);
        detector.setFastThreshold(o.orbFastThreshold);
        yield detector;
      }
      case "akaze" -> {
        AKAZE detector = AKAZE.create();
        detector.setThreshold(o.akazeThreshold);
        yield detector;
      }
      case "kaze" -> {
        KAZE detector = KAZE.create();
        detector.setThreshold(o.kazeThreshold);
        yield detector;
      }
      case "brisk" -> BRISK.create(o.briskThreshold);
      default -> SIFT.create(maximum, 3, o.siftContrast, o.siftEdgeThreshold, 1.6);
    };
  }

  static DescriptorMatcher matcher(Options o, Mat descriptors) {
    boolean binary = descriptors.depth() == CvType.CV_8U;
    // OpenCV's default FLANN index is KD-tree and accepts float descriptors only.
    if (o.descriptorMatcher.equals("flann") && !binary)
      return DescriptorMatcher.create(DescriptorMatcher.FLANNBASED);
    return BFMatcher.create(binary ? Core.NORM_HAMMING : Core.NORM_L2, false);
  }

  /**
   * Search backwards only for descriptors that can survive the forward ratio test. The training
   * set, metric, two-neighbor search and output ordering are unchanged. In particular FLANN still
   * builds both indexes in the original order; skipping an index would change its RNG stream.
   */
  static int reciprocal(
      Options o,
      Mat a,
      Mat b,
      List<MatOfDMatch> forward,
      Map<Integer, Integer> reverse,
      Consumer<String> progress) {
    KnnMatches flat = new KnnMatches();
    int searched = reciprocal(o, a, b, flat, reverse, progress);
    flat.toLegacy(forward);
    return searched;
  }

  static int reciprocal(
      Options o, Mat a, Mat b, KnnMatches forward, Map<Integer, Integer> reverse,
      Consumer<String> progress) {
    Integer resident = GpuHamming.reciprocal(o, a, b, forward, reverse, progress);
    if (resident != null) return resident;
    Workspace work = workspace();
    DescriptorMatcher matcher = work.matcher(o, a);
    KnnMatches backward = work.backward;
    try {
      progress.accept("forward nearest neighbors; " + a.rows() + " queries");
      work.nativeSearch.search(matcher, a, b, forward);
      BitSet needed = work.needed;
      needed.clear();
      for (int row = 0; row < forward.size(); row++)
        if (forward.count(row) >= 2 && forward.distance(row) < o.ratio * forward.secondDistance(row))
          needed.set(forward.train(row));
      int count = needed.cardinality();
      boolean flann = o.descriptorMatcher.equals("flann") && a.depth() != CvType.CV_8U;
      // Dense selections do not justify gathering rows. Preserve FLANN index construction even
      // when no forward match passes, so later geometric RANSAC receives the same RNG state.
      boolean full = count * 5L >= b.rows() * 4L || count == 0 && flann;
      int[] ids = full ? null : needed.stream().toArray();
      if (!full && count == 0) return 0;
      if (!full) work.gather(b, ids);
      int searched = full ? b.rows() : count;
      progress.accept("reverse nearest neighbors; " + searched + "/" + b.rows() + " queries");
      work.nativeSearch.search(matcher, full ? b : work.queries, a, backward);
      for (int row = 0; row < backward.size(); row++) {
        if (backward.count(row) >= 2 && backward.distance(row) < o.ratio * backward.secondDistance(row)) {
          int id = full ? backward.query(row) : ids[backward.query(row)];
          reverse.put(id, backward.train(row));
        }
      }
      return searched;
    } finally {
      matcher.clear();
    }
  }
}
