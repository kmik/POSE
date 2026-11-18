package sgm;

import java.lang.management.ManagementFactory;
import java.util.*;
import org.opencv.core.*;
import org.opencv.features2d.DescriptorMatcher;

/** Exact JNI ownership/reuse checks and a repeatable, separate-JVM before/after benchmark. */
final class MatchingBuffersRegression {
  private static int checks;
  private static void check(boolean value, String message) {
    checks++;
    if (!value) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Core.setNumThreads(1);
    if (args.length > 0 && args[0].equals("benchmark")) {
      benchmark(args[1].equals("flat"), Integer.parseInt(args[2]), Integer.parseInt(args[3]));
      return;
    }
    System.out.println("PASS matching buffers: " + run());
  }

  static int run() {
    checks = 0;
    Options o = new Options();
    o.compute = "cpu";
    try (OpenCvKnn adapter = new OpenCvKnn()) {
      // Reuse the adapter across changing sizes/types, empty searches and one-neighbour rows.
      for (boolean binary : new boolean[] {false, true})
        for (String method : new String[] {"bruteforce", "flann"}) {
          o.descriptorMatcher = method;
          for (int[] sizes : new int[][] {{257, 193}, {3, 2}, {0, 5}, {5, 0}, {67, 23}, {1, 2}}) {
            // FLANN itself rejects an empty float training set (its native Mat has type 0).
            if (!binary && method.equals("flann") && sizes[1] == 0) continue;
            Mat a = random(sizes[0], binary, 17), b = random(sizes[1], binary, 23);
            DescriptorMatcher matcher = FeatureAlgorithms.matcher(o, a);
            List<MatOfDMatch> expected = new ArrayList<>();
            try {
              Core.setRNGSeed(111);
              matcher.knnMatch(a, b, expected, 2);
              int[] state = rng();
              matcher.clear();
              Core.setRNGSeed(111);
              KnnMatches actual = new KnnMatches();
              adapter.search(matcher, a, b, actual);
              check(Arrays.equals(state, rng()), "native RNG is unchanged by the adapter");
              equal(expected, actual);
              check(adapter.copiedRows == adapter.deletedRows, "every owned native row was deleted");
            } finally {
              for (Mat row : expected) row.release();
              matcher.clear(); a.release(); b.release();
            }
          }
        }
      o.descriptorMatcher = "bruteforce";
      Mat a = random(25, false, 2), one = random(1, false, 3), wrong = new Mat(3, 17, CvType.CV_32F);
      DescriptorMatcher matcher = FeatureAlgorithms.matcher(o, a);
      try {
        KnnMatches flat = new KnnMatches();
        adapter.search(matcher, a, one, flat);
        check(flat.size() == 25 && flat.count(0) == 1, "one-neighbour rows are supported");
        boolean rejected = false;
        try { adapter.search(matcher, a, wrong, flat); }
        catch (RuntimeException e) { rejected = true; }
        check(rejected, "native errors propagate");
        adapter.search(matcher, a, one, flat);
        check(flat.size() == 25 && flat.count(24) == 1, "adapter recovers after a native error");
        check(adapter.copiedRows == adapter.deletedRows, "error cleanup preserves native ownership");
      } finally { matcher.clear(); a.release(); one.release(); wrong.release(); }
    } finally { FeatureAlgorithms.closeWorkspace(); }
    return checks;
  }

  private static void equal(List<MatOfDMatch> expected, KnnMatches actual) {
    check(expected.size() == actual.size(), "same query rows: " + expected.size() + " / " + actual.size());
    for (int i = 0; i < expected.size(); i++) {
      DMatch[] a = expected.get(i).toArray(), b = actual.rowArray(i);
      check(a.length == b.length, "same neighbour count");
      for (int j = 0; j < a.length; j++)
        check(a[j].queryIdx == b[j].queryIdx && a[j].trainIdx == b[j].trainIdx
            && a[j].imgIdx == b[j].imgIdx
            && Float.floatToRawIntBits(a[j].distance) == Float.floatToRawIntBits(b[j].distance),
            "bit-identical query/train/image indices and distances");
    }
  }

  private static Mat random(int rows, boolean binary, int seed) {
    Mat result = new Mat(rows, binary ? 32 : 128, binary ? CvType.CV_8U : CvType.CV_32F);
    Core.setRNGSeed(seed);
    if (!result.empty()) Core.randu(result, 0, binary ? 256 : 64);
    return result;
  }

  private static int[] rng() {
    Mat m = new Mat(1, 16, CvType.CV_32S);
    try { Core.randu(m, 0, 1000000); int[] result = new int[16]; m.get(0, 0, result); return result; }
    finally { m.release(); }
  }

  private static void benchmark(boolean flat, int rows, int rounds) {
    Options o = new Options();
    o.compute = "cpu"; o.descriptorMatcher = "flann"; o.ratio = .85;
    Mat a = random(rows, false, 41), b = random(rows, false, 42);
    // Half exact correspondences plus unrelated descriptors exercises the gathered reverse path.
    float[] row = new float[128];
    for (int i = 0; i < rows / 2; i++) { a.get(i, 0, row); b.put(i, 0, row); }
    var bean = (com.sun.management.ThreadMXBean) ManagementFactory.getThreadMXBean();
    long allocated = bean.getThreadAllocatedBytes(Thread.currentThread().threadId());
    long started = System.nanoTime(), digest = 1, peakRss = 0, peakPrivate = 0;
    try {
      for (int pair = 0; pair < rounds; pair++) {
        Core.setRNGSeed(700 + pair);
        Map<Integer, Integer> reverse = new HashMap<>();
        if (flat) {
          KnnMatches matches = new KnnMatches();
          FeatureAlgorithms.reciprocal(o, a, b, matches, reverse, s -> {});
          for (int i = 0; i < matches.size(); i++)
            for (DMatch d : matches.rowArray(i)) digest = hash(digest, d);
        } else {
          List<MatOfDMatch> matches = new ArrayList<>();
          try {
            FeatureAlgorithms.reciprocal(o, a, b, matches, reverse, s -> {});
            for (MatOfDMatch match : matches) for (DMatch d : match.toArray()) digest = hash(digest, d);
          } finally { for (Mat match : matches) match.release(); }
        }
        for (var entry : new TreeMap<>(reverse).entrySet()) digest = digest * 31 + entry.getKey() * 7L + entry.getValue();
        long[] process = RunTelemetry.processMemoryBytes();
        peakRss = Math.max(peakRss, process[0]); peakPrivate = Math.max(peakPrivate, process[1]);
      }
      System.out.printf(Locale.ROOT,
          "{\"mode\":\"%s\",\"rows\":%d,\"pairs\":%d,\"seconds\":%.6f,\"allocated_mib\":%.3f,"
          + "\"peak_rss_mib\":%.3f,\"peak_private_mib\":%.3f,\"digest\":\"%s\"}%n",
          flat ? "flat" : "legacy", rows, rounds, (System.nanoTime() - started) / 1e9,
          (bean.getThreadAllocatedBytes(Thread.currentThread().threadId()) - allocated) / 1048576.,
          peakRss / 1048576., peakPrivate / 1048576., Long.toHexString(digest));
    } finally { a.release(); b.release(); if (flat) FeatureAlgorithms.closeWorkspace(); }
  }

  private static long hash(long value, DMatch d) {
    return value * 31 + d.queryIdx * 131L + d.trainIdx * 7L + d.imgIdx + Float.floatToRawIntBits(d.distance);
  }
}
