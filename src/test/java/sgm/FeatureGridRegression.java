package sgm;

import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

/** Spread feature selection (--feature-grid). */
final class FeatureGridRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Path root = Files.createTempDirectory(Path.of("target"), "feature grid ");
    System.out.println("PASS feature/grid checks=" + run(root) + "; fixtures=" + root);
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Files.createDirectories(root);
    // Selection: strong keypoints crowd the top-left quarter, weak ones cover the whole image.
    check(Arrays.equals(FeatureGrid.cells(400, 300, 16), new int[] {16, 12}), "landscape cells");
    check(Arrays.equals(FeatureGrid.cells(300, 400, 16), new int[] {12, 16}), "portrait cells");
    List<KeyPoint> list = new ArrayList<>();
    Random random = new Random(7);
    for (int i = 0; i < 2000; i++)
      list.add(new KeyPoint(random.nextFloat() * 200, random.nextFloat() * 150, 2, 0, 10 + random.nextFloat()));
    for (int i = 0; i < 2000; i++)
      list.add(new KeyPoint(random.nextFloat() * 400, random.nextFloat() * 300, 2, 0, random.nextFloat()));
    KeyPoint[] all = list.toArray(KeyPoint[]::new);
    KeyPoint[] grid = FeatureGrid.select(all, 400, 300, 4, 800);
    check(grid.length == 800, "selects exactly the maximum");
    check(new HashSet<>(Arrays.asList(grid)).size() == 800, "no keypoint twice");
    for (int i = 1; i < grid.length; i++)
      check(grid[i - 1].response >= grid[i].response, "strongest first");
    int outside = 0;
    for (KeyPoint p : grid) if (p.pt.x >= 200 || p.pt.y >= 150) outside++;
    check(outside >= 300, "weak-texture quadrants get their share: " + outside);
    KeyPoint[] strongest = FeatureGrid.select(all, 400, 300, 1, 800);
    int strongestOutside = 0;
    for (KeyPoint p : strongest) if (p.pt.x >= 200 || p.pt.y >= 150) strongestOutside++;
    check(strongestOutside == 0, "one cell = strongest N overall");
    check(
        Arrays.equals(FeatureGrid.select(all, 400, 300, 4, 800), grid), "deterministic selection");
    KeyPoint[] few = FeatureGrid.select(Arrays.copyOf(all, 50), 400, 300, 4, 800);
    check(few.length == 50, "fewer candidates than the maximum: all kept");

    // Extraction on an image whose left half is strongly textured and right half faint.
    Path image = root.resolve("texture.png");
    Mat m = new Mat(300, 400, CvType.CV_8UC1);
    Random noise = new Random(3);
    byte[] pixels = new byte[400 * 300];
    for (int y = 0; y < 300; y++)
      for (int x = 0; x < 400; x++) {
        double amplitude = x < 200 ? 110 : 30;
        pixels[y * 400 + x] = (byte) Math.max(0, Math.min(255, 128 + amplitude * (noise.nextDouble() - .5)));
      }
    m.put(0, 0, pixels);
    Imgproc.GaussianBlur(m, m, new Size(3, 3), 0);
    Imgcodecs.imwrite(image.toString(), m);
    m.release();
    Calibration calibration = new Calibration(400, 400, 199.5, 149.5, new double[5], 400, 300);
    int[] right = new int[2];
    for (int mode = 0; mode < 2; mode++) {
      Options o = TestSupport.options();
      o.input = root;
      o.output = root.resolve("out");
      o.maxDimension = 400;
      o.featureGrid = mode == 0 ? 0 : 8;
      o.siftContrast = .01; // at the default 0.04 the faint half has no candidates at all
      try (ImageFrame f = new ImageFrame(0, image, calibration, o)) {
        f.extract(600);
        check(f.keypoints.length <= 600 && f.keypoints.length > 100, "feature count, mode " + mode);
        check(f.descriptors.rows() == f.keypoints.length, "one descriptor per keypoint, mode " + mode);
        for (KeyPoint p : f.keypoints) if (p.pt.x >= 200) right[mode]++;
      }
    }
    check(right[1] > right[0] * 2 && right[1] >= 150,
        "grid spreads into the faint half: " + right[0] + " -> " + right[1]);

    // Options.
    check(TestSupport.options().featureGrid == 0, "off by default");
    Options parsed =
        Options.parse(new String[] {"-i", root.toString(), "-o", root.resolve("p").toString(),
          "--feature-grid", "16"});
    check(parsed.featureGrid == 16, "parses");
    for (String bad : new String[] {"1", "-2", "257"}) {
      boolean failed = false;
      try {
        Options.parse(new String[] {"-i", root.toString(), "-o", root.resolve("p").toString(),
          "--feature-grid", bad});
      } catch (IllegalArgumentException e) {
        failed = true;
      }
      check(failed, "rejects --feature-grid " + bad);
    }
    check(Options.help().contains("--feature-grid"), "help lists the option");
    Options a = TestSupport.options(), b = TestSupport.options();
    b.featureGrid = 16;
    check(!ReuseCache.settings(a, "featureGrid").equals(ReuseCache.settings(b, "featureGrid")),
        "cache settings differ");
    return checks;
  }
}
