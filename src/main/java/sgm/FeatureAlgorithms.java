package sgm;

import java.util.*;
import java.util.function.Consumer;
import org.opencv.core.*;
import org.opencv.features2d.*;

/** Descriptor metric is selected by actual descriptor storage, including calibration previews. */
final class FeatureAlgorithms {
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
    DescriptorMatcher matcher = matcher(o, a);
    List<MatOfDMatch> backward = new ArrayList<>();
    Mat queries = new Mat();
    try {
      progress.accept("forward nearest neighbors; " + a.rows() + " queries");
      matcher.knnMatch(a, b, forward, 2);
      BitSet needed = new BitSet(b.rows());
      for (MatOfDMatch row : forward) {
        DMatch[] m = row.toArray();
        if (m.length >= 2 && m[0].distance < o.ratio * m[1].distance) needed.set(m[0].trainIdx);
      }
      int count = needed.cardinality();
      boolean flann = o.descriptorMatcher.equals("flann") && a.depth() != CvType.CV_8U;
      // Dense selections do not justify gathering rows. Preserve FLANN index construction even
      // when no forward match passes, so later geometric RANSAC receives the same RNG state.
      boolean full = count * 5L >= b.rows() * 4L || count == 0 && flann;
      int[] ids = full ? null : needed.stream().toArray();
      if (!full && count == 0) return 0;
      if (!full) {
        queries.create(count, b.cols(), b.type());
        for (int i = 0; i < count; i++) {
          Mat from = b.row(ids[i]), to = queries.row(i);
          try {
            from.copyTo(to);
          } finally {
            from.release();
            to.release();
          }
        }
      }
      int searched = full ? b.rows() : count;
      progress.accept("reverse nearest neighbors; " + searched + "/" + b.rows() + " queries");
      matcher.knnMatch(full ? b : queries, a, backward, 2);
      for (MatOfDMatch row : backward) {
        DMatch[] m = row.toArray();
        if (m.length >= 2 && m[0].distance < o.ratio * m[1].distance) {
          int id = full ? m[0].queryIdx : ids[m[0].queryIdx];
          reverse.put(id, m[0].trainIdx);
        }
      }
      return searched;
    } finally {
      for (Mat m : backward) m.release();
      queries.release();
      matcher.clear();
    }
  }
}
