package sgm;

import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;

/** Tests designed to fail on the original self-confirming fusion/no-confidence implementation. */
final class NoiseRegression {
  private static int checks;

  private static void check(boolean condition, String description) {
    checks++;
    if (!condition) throw new AssertionError(description);
  }

  static int run(Path root) throws Exception {
    checks = 0;
    fusion();
    confidence();
    mask(root);
    AdversarialSceneTest.main(
        new String[] {root.resolve("adversarial regression").toString(), "strict"});
    checks += 3;
    return checks;
  }

  private static void fusion() {
    Options o = TestSupport.options();
    Camera[] cameras = {
      new Camera(200, 200, 100, 100), new Camera(200, 200, 100, 100), new Camera(200, 200, 100, 100)
    };
    cameras[1].translation[0] = -.25;
    cameras[2].translation[0] = -.5;
    int[] dimensions = {200, 200, 200};
    DepthFusion fusion = new DepthFusion(cameras, dimensions, dimensions, o);
    int source = fusion.addPair(0, 1);
    double[] ghost = {0, 0, 50}, truePoint = {0, 0, 5};
    fusion.add(source, ghost);
    check(
        !fusion.consistent(source, ghost),
        "A point cannot manufacture independent support by projecting itself into its two source"
            + " cameras");
    int duplicate = fusion.addPair(0, 1);
    fusion.add(duplicate, ghost);
    check(
        !fusion.consistent(source, ghost),
        "Repeated evidence from the same two camera IDs does not count as a third view");
    int independent = fusion.addPair(0, 2);
    fusion.add(independent, truePoint);
    check(
        !fusion.consistent(source, ghost),
        "An independently measured nearer surface cannot corroborate a far ghost");
    fusion.add(source, truePoint);
    check(
        fusion.consistent(source, truePoint),
        "Separate stereo pairs corroborate a real three-view point");
    Camera[] stereo = {cameras[0], cameras[1]};
    int[] two = {200, 200};
    DepthFusion fallback = new DepthFusion(stereo, two, two, o);
    int pair = fallback.addPair(0, 1);
    fallback.add(pair, truePoint);
    check(
        fallback.consistent(pair, truePoint) && fallback.requiredViews() == 2,
        "Two-image inputs retain validated stereo without pretending to have three-view evidence");
    check(
        DenseReconstruction.triangulationAngle(
                new double[] {0, 0, 1000}, new double[] {0, 0, 0}, new double[] {.25, 0, 0})
            < .1,
        "Almost parallel rays have unreliable depth");
    check(
        DenseReconstruction.triangulationAngle(
                truePoint, new double[] {0, 0, 0}, new double[] {.25, 0, 0})
            > 2,
        "Ordinary stereo geometry survives the parallax guard");
  }

  private static void confidence() {
    int w = 96, h = 48;
    byte[] left = new byte[w * h], right = new byte[w * h], unrelated = new byte[w * h];
    new Random(73).nextBytes(left);
    new Random(98).nextBytes(unrelated);
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) right[y * w + x] = left[y * w + Math.min(w - 1, x + 6)];
    check(
        CensusSgm.zncc(left, right, w, h, 40, 24, 6, 3, 1) > .99,
        "Photometric verification accepts a known correspondence");
    check(
        CensusSgm.zncc(left, unrelated, w, h, 40, 24, 6, 3, 1) < .5,
        "Photometric verification rejects unrelated texture");
    byte[] black = new byte[w * h];
    check(
        CensusSgm.zncc(black, black, w, h, 40, 24, 6, 3, 1) < 0,
        "Zero-variance patches do not produce confident correlations");
    float[] a = new float[w * h], reciprocal = new float[w * h];
    Arrays.fill(a, 12);
    Arrays.fill(reciprocal, Float.NaN);
    a[24 * w + 40] = 10;
    reciprocal[24 * w + 30] = 10;
    check(
        CensusSgm.reciprocal(reciprocal, w, h, 40, 24, a[24 * w + 40], .75, 2),
        "Pre-median correspondence passes reciprocity");
    float[] median = CensusSgm.medianValid(a, w, h, 3);
    check(
        !CensusSgm.reciprocal(reciprocal, w, h, 40, 24, median[24 * w + 40], .75, 2),
        "A median-modified correspondence must be revalidated");
    float[] speckles = new float[30 * 20];
    Arrays.fill(speckles, Float.NaN);
    for (int y = 2; y < 18; y++) for (int x = 2; x < 12; x++) speckles[y * 30 + x] = 8;
    speckles[5 * 30 + 20] = 20;
    speckles[5 * 30 + 21] = 20;
    speckles[6 * 30 + 20] = 20;
    check(
        CensusSgm.removeSpeckles(speckles, 30, 20, 20, 1) == 3,
        "Small disconnected disparity islands are removed");
    check(
        Float.isFinite(speckles[10 * 30 + 5]),
        "The large valid surface is preserved by speckle filtering");
    Options o = TestSupport.options();
    o.disparities = 16;
    o.median = 1;
    byte[] shifted = new byte[w * h];
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) shifted[y * w + x] = left[y * w + Math.min(w - 1, x + 15)];
    CensusSgm.Result boundary = new CensusSgm(o).compute(left, shifted, w, h);
    check(
        Float.isNaN(boundary.disparity[24 * w + 40]),
        "An optimum on the search boundary is not a trustworthy depth");
    CensusSgm.Result noise = new CensusSgm(o).compute(left, unrelated, w, h);
    int valid = 0;
    for (float d : noise.disparity) if (Float.isFinite(d)) valid++;
    check(valid < .01 * w * h, "Unrelated stereo textures do not form a noisy cloud");
    byte[] mask = new byte[w * h];
    Arrays.fill(mask, (byte) 255);
    for (int y = 0; y < h; y++) for (int x = 30; x < 50; x++) mask[y * w + x] = 0;
    CensusSgm.Result masked = new CensusSgm(o).compute(left, right, null, null, w, h, mask, mask);
    check(
        Float.isNaN(masked.disparity[24 * w + 40]), "Invalid remapped pixels never become matches");
  }

  private static void mask(Path root) throws Exception {
    Path path = root.resolve("black but valid.png");
    Mat black = new Mat(80, 80, CvType.CV_8UC3, new Scalar(0));
    try {
      Imgcodecs.imwrite(path.toString(), black);
    } finally {
      black.release();
    }
    Calibration calibration =
        new Calibration(45, 45, 39.5, 39.5, new double[] {.3, 0, 0, 0, 0}, 80, 80);
    Options o = TestSupport.options();
    o.maxDimension = 80;
    try (ImageFrame frame = new ImageFrame(0, path, calibration, o)) {
      check(
          frame.validMask.get(40, 40)[0] == 255,
          "Legitimate black scene content retains a valid image mask");
      check(
          frame.validMask.get(0, 0)[0] == 0,
          "Undistortion outside the source raster is marked invalid");
    }
  }
}
