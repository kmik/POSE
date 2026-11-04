package sgm;

import java.awt.image.BufferedImage;
import java.nio.file.*;
import java.util.*;
import javax.imageio.ImageIO;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

/** Exact raster and cache-disabled controls for the additional scheduling/IO shortcuts. */
final class ExactReuseRegression {
  private static int checks;

  private static void check(boolean ok, String why) {
    checks++;
    if (!ok) throw new AssertionError(why);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    System.out.println(
        "PASS exact reuse: " + run(Files.createTempDirectory(Path.of("target"), "exact-reuse-")));
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Files.createDirectories(root);
    BufferedImage image = new BufferedImage(137, 91, BufferedImage.TYPE_3BYTE_BGR);
    Random random = new Random(781);
    for (int y = 0; y < 91; y++) for (int x = 0; x < 137; x++) image.setRGB(x, y, random.nextInt());
    Path file = root.resolve("pixels.png");
    ImageIO.write(image, "PNG", file.toFile());
    image.flush();
    for (int dimension : new int[] {137, 92}) {
      Options o = TestSupport.options();
      o.output = root;
      o.cacheDirectory = null;
      o.maxDimension = dimension;
      o.maxDimensionExplicit = true;
      o.quality = "custom";
      Calibration c = new Calibration(123.456, 127.890, 61.32, 48.76, new double[5], 137, 91);
      try (ImageFrame frame = new ImageFrame(0, file, c, o, true)) {
        check(frame.gray.empty(), "managed frame starts without raster decode");
        check(
            !ReuseCache.matches(List.of(frame), null, o, false),
            "disabled match cache returns immediately");
        check(
            frame.gray.empty() && frame.pixelFingerprint == null,
            "disabled cache does not decode/hash pixels");
        // A missing file must not be read merely to produce an unused calibration cache key.
        check(
            ReuseCache.calibration(List.of(root.resolve("missing.jpg")), o, c) == c,
            "disabled calibration cache preserves the exact result without hashing sources");
        frame.ensureRasters();
        Mat original = Imgcodecs.imread(file.toString()),
            resized = new Mat(),
            expected = new Mat(),
            mask = new Mat(frame.height, frame.width, CvType.CV_8U, new Scalar(255)),
            expectedMask = new Mat(),
            k = frame.camera.intrinsic();
        MatOfDouble d = new MatOfDouble(c.distortion);
        try {
          Imgproc.resize(
              original, resized, new Size(frame.width, frame.height), 0, 0, Imgproc.INTER_AREA);
          Calib3d.undistort(resized, expected, k, d);
          Calib3d.undistort(mask, expectedMask, k, d);
          check(
              Core.norm(expected, frame.color, Core.NORM_INF) == 0,
              "identity undistortion bypass has identical colors");
          check(
              Core.norm(expectedMask, frame.validMask, Core.NORM_INF) == 0,
              "identity mask is identical");
        } finally {
          original.release();
          resized.release();
          expected.release();
          mask.release();
          expectedMask.release();
          k.release();
          d.release();
        }
      }
    }
    int w = 96, h = 40, n = w * h;
    byte[] a = new byte[n], b = new byte[n];
    random.nextBytes(a);
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) b[y * w + x] = a[y * w + Math.min(w - 1, x + 7)];
    Options o = TestSupport.options();
    o.output = root;
    o.disparities = 19;
    o.median = 1;
    o.speckleSize = 0;
    o.cacheDirectory = null;
    var plain =
        ReuseCache.stereo(o, a, b, null, null, w, h, null, null, null, root.resolve("plain"));
    o.cacheDirectory = root.resolve("cache");
    o.resume = true;
    var stored =
        ReuseCache.stereo(o, a, b, null, null, w, h, null, null, null, root.resolve("stored"));
    var loaded =
        ReuseCache.stereo(o, a, b, null, null, w, h, null, null, null, root.resolve("loaded"));
    for (var result : List.of(stored, loaded)) {
      check(Arrays.equals(plain.disparity, result.disparity), "cached/uncached disparity exact");
      check(Arrays.equals(plain.costMargin, result.costMargin), "cached/uncached uniqueness exact");
      check(
          Arrays.equals(plain.costSharpness, result.costSharpness),
          "cached/uncached curvature exact");
      check(Arrays.equals(plain.confidence, result.confidence), "cached/uncached confidence exact");
      check(plain.stats.csv().equals(result.stats.csv()), "cached/uncached rejection counts exact");
    }
    return checks;
  }
}
