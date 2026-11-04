package sgm;

import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;
import org.opencv.features2d.DescriptorMatcher;
import org.opencv.imgcodecs.Imgcodecs;

/** Compare the optimized query schedule to the original two full kNN searches. */
final class ReciprocalMatchingRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Core.setNumThreads(1);
    if (args.length > 0) benchmark(Path.of(args[0]));
    else System.out.println("PASS reciprocal scheduling: " + run());
  }

  static int run() throws Exception {
    checks = 0;
    for (boolean binary : new boolean[] {false, true})
      for (String method : new String[] {"bruteforce", "flann"})
        for (int seed = 1; seed <= 5; seed++) {
          Mat a = descriptors(257, binary, seed), b = descriptors(193, binary, seed + 80);
          try {
            // A mixture of exact matches, collisions and unrelated descriptors.
            for (int i = 0; i < 35; i++) copyRow(a, i * 3, b, i * 2);
            copyRow(a, 0, a, 1);
            copyRow(b, 0, b, 1);
            compare(a, b, method, .8, seed);
            compare(a, b, method, 0, seed); // No candidates; preserve the FLANN RNG stream.
            compare(a, b, method, 1, seed);
          } finally {
            a.release();
            b.release();
          }
        }
    Mat a = descriptors(23, false, 8), b = a.clone();
    try {
      compare(a, b, "flann", .8, 4);
      compare(a, b, "bruteforce", .8, 4);
    } finally {
      a.release();
      b.release();
    }
    featureReuse();
    return checks;
  }

  private static void featureReuse() throws Exception {
    Options o = TestSupport.options();
    o.output = Files.createTempDirectory(Path.of("target"), "focal feature reuse ");
    o.cacheDirectory = o.output.resolve("cache");
    o.resume = true;
    o.quality = "custom";
    o.maxDimension = 200;
    o.features = 100;
    Mat pixels = new Mat(150, 200, CvType.CV_8UC3);
    byte[] data = new byte[150 * 200 * 3];
    new Random(80).nextBytes(data);
    pixels.put(0, 0, data);
    Path image = o.output.resolve("image.png");
    try {
      check(Imgcodecs.imwrite(image.toString(), pixels), "write raster fixture");
    } finally {
      pixels.release();
    }
    Calibration c = new Calibration(170, 170, 99.5, 74.5, new double[5], 200, 150);
    Calibration changed = new Calibration(340, 340, 99.5, 74.5, new double[5], 200, 150);
    try (ImageFrame first = new ImageFrame(0, image, c, o, true);
        ImageFrame next = new ImageFrame(7, image, changed, o, true)) {
      check(!ReuseCache.features(first, o), "cold feature extraction");
      check(
          ReuseCache.features(next, o),
          "identical pixels reuse features across focal guesses and IDs");
      check(
          Core.norm(first.descriptors, next.descriptors, Core.NORM_INF) == 0,
          "cached descriptor values are exact");
      check(
          !ReuseCache.frame(first).equals(ReuseCache.frame(next)),
          "matching/geometry keys still distinguish cameras");
      double originalX = next.keypoints[0].pt.x;
      check(originalX == first.keypoints[0].pt.x, "cached keypoints retain measured pixels");
    }
    Calibration distortion =
        new Calibration(170, 170, 99.5, 74.5, new double[] {-.15, 0, 0, 0, 0}, 200, 150);
    try (ImageFrame frame = new ImageFrame(0, image, distortion, o, true)) {
      check(!ReuseCache.features(frame, o), "changed distortion/raster invalidates feature reuse");
    }
    Options resized = o.copy();
    resized.maxDimension = 160;
    try (ImageFrame frame = new ImageFrame(0, image, c, resized, true)) {
      check(!ReuseCache.features(frame, resized), "changed resolution invalidates feature reuse");
    }
    Options detector = o.copy();
    detector.siftContrast *= 2;
    try (ImageFrame frame = new ImageFrame(0, image, c, detector, true)) {
      check(
          !ReuseCache.features(frame, detector),
          "changed detector settings invalidate feature reuse");
    }
    Options rerun = o.copy();
    rerun.rerunFrom = "features";
    try (ImageFrame frame = new ImageFrame(0, image, c, rerun, true)) {
      check(!ReuseCache.features(frame, rerun), "explicit feature rerun remains authoritative");
    }
  }

  private static Mat descriptors(int n, boolean binary, int seed) {
    int cols = binary ? 32 : 128;
    Mat m = new Mat(n, cols, binary ? CvType.CV_8U : CvType.CV_32F);
    Random rng = new Random(seed);
    if (binary) {
      byte[] data = new byte[n * cols];
      rng.nextBytes(data);
      m.put(0, 0, data);
    } else {
      float[] data = new float[n * cols];
      for (int i = 0; i < data.length; i++) data[i] = rng.nextFloat();
      m.put(0, 0, data);
    }
    return m;
  }

  private static void copyRow(Mat a, int ai, Mat b, int bi) {
    Mat from = a.row(ai), to = b.row(bi);
    try {
      from.copyTo(to);
    } finally {
      from.release();
      to.release();
    }
  }

  private static void original(
      Options o, Mat a, Mat b, List<MatOfDMatch> fw, Map<Integer, Integer> reverse) {
    DescriptorMatcher matcher = FeatureAlgorithms.matcher(o, a);
    List<MatOfDMatch> bw = new ArrayList<>();
    try {
      matcher.knnMatch(a, b, fw, 2);
      matcher.knnMatch(b, a, bw, 2);
      for (MatOfDMatch row : bw) {
        DMatch[] m = row.toArray();
        if (m.length >= 2 && m[0].distance < o.ratio * m[1].distance)
          reverse.put(m[0].queryIdx, m[0].trainIdx);
      }
    } finally {
      for (Mat row : bw) row.release();
      matcher.clear();
    }
  }

  private static int[] rngSuffix() {
    Mat m = new Mat(1, 64, CvType.CV_32S);
    try {
      Core.randu(m, 0, 1000000000);
      int[] v = new int[64];
      m.get(0, 0, v);
      return v;
    } finally {
      m.release();
    }
  }

  private static double compare(Mat a, Mat b, String method, double ratio, int seed) {
    Options o = TestSupport.options();
    o.descriptorMatcher = method;
    o.ratio = ratio;
    List<MatOfDMatch> old = new ArrayList<>(), fast = new ArrayList<>();
    Map<Integer, Integer> expected = new HashMap<>(), actual = new HashMap<>();
    try {
      Core.setRNGSeed(seed);
      long t = System.nanoTime();
      original(o, a, b, old, expected);
      double before = (System.nanoTime() - t) / 1e9;
      int[] state = rngSuffix();
      Core.setRNGSeed(seed);
      t = System.nanoTime();
      int queries = FeatureAlgorithms.reciprocal(o, a, b, fast, actual, s -> {});
      double after = (System.nanoTime() - t) / 1e9;
      check(Arrays.equals(state, rngSuffix()), method + " preserves native RNG state");
      check(old.size() == fast.size(), method + " preserves all forward queries");
      int reciprocal = 0;
      for (int i = 0; i < old.size(); i++) {
        DMatch[] p = old.get(i).toArray(), q = fast.get(i).toArray();
        check(p.length == q.length, "same neighbor count");
        for (int j = 0; j < p.length; j++)
          check(
              p[j].queryIdx == q[j].queryIdx
                  && p[j].trainIdx == q[j].trainIdx
                  && Float.floatToRawIntBits(p[j].distance)
                      == Float.floatToRawIntBits(q[j].distance),
              method + " bit-identical neighbors and distances");
        if (p.length >= 2 && p[0].distance < ratio * p[1].distance) {
          check(
              Objects.equals(expected.get(p[0].trainIdx), actual.get(p[0].trainIdx)),
              method + " identical reciprocal decision");
          if (Objects.equals(expected.get(p[0].trainIdx), p[0].queryIdx)) reciprocal++;
        }
      }
      if (a.rows() > 1000)
        System.out.printf(
            Locale.ROOT,
            "%s %d x %d: original %.4f s; revised %.4f s; reverse %d/%d; reciprocal %d%n",
            method,
            a.rows(),
            b.rows(),
            before,
            after,
            queries,
            b.rows(),
            reciprocal);
      return before / Math.max(1e-9, after);
    } finally {
      for (Mat m : old) m.release();
      for (Mat m : fast) m.release();
    }
  }

  private static void benchmark(Path input) throws Exception {
    Options o = TestSupport.options();
    o.input = input;
    o.features = 56000;
    o.alignmentMaxDimension = 1600;
    o.cores = 12;
    o.output = Files.createTempDirectory(Path.of("target"), "focal matching bench ");
    Core.setNumThreads(o.cores);
    List<Path> files = Main.images(o);
    int[] size = ResolutionPlan.originalSize(files.get(0));
    double d = Math.hypot(size[0], size[1]);
    Calibration c =
        new Calibration(
            d, d, (size[0] - 1) / 2., (size[1] - 1) / 2., new double[5], size[0], size[1]);
    List<ImageFrame> frames = new ArrayList<>();
    try {
      for (int i = 0; i < Math.min(4, files.size()); i++) {
        ImageFrame f = new ImageFrame(i, files.get(i), c, o.alignmentOptions(), true);
        frames.add(f);
        f.extract(o.features);
        DescriptorCache.retain(f);
      }
      for (String method : new String[] {"flann", "bruteforce"})
        for (int i = 1; i < frames.size(); i++) {
          ImageFrame a = frames.get(0), b = frames.get(i);
          DescriptorCache.pair(a, b);
          compare(a.descriptors, b.descriptors, method, .8, i);
        }
      System.out.println("PASS forest descriptor parity: " + checks);
    } finally {
      for (ImageFrame f : frames) f.close();
    }
  }
}
