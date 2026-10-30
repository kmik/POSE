package sgm;

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
}
