package sgm;

import java.io.*;
import java.nio.*;
import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

/** Deterministic numerical and end-to-end checks; no JUnit installation required. */
public final class SelfTest {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  private static void near(double actual, double expected, double tolerance, String message) {
    check(
        Math.abs(actual - expected) <= tolerance,
        message + ": actual=" + actual + ", expected=" + expected);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Core.setNumThreads(1);
    Core.setRNGSeed(1);
    // A shorter artifact root avoids native OpenCV's Windows MAX_PATH limit in nested fixtures.
    Path testRoot = Paths.get(System.getProperty("sgm.test.root", "target"));
    Files.createDirectories(testRoot);
    Path root = Files.createTempDirectory(testRoot, "self test ");
    testSgm();
    testGeometry();
    testLas(root);
    testCalibration(root);
    Path images = scene(root, 3, false);
    Path calibration = root.resolve("camera.txt");
    Files.writeString(calibration, "width=400\nheight=300\nfx=240\nfy=240\ncx=199.5\ncy=149.5\n");
    testRectified(root, images, calibration);
    testKnownPoses(root, images, calibration);
    testVertical(root, calibration);
    testReversed(root, images, calibration);
    testSfm(root, images, calibration);
    testBundle(root, images, calibration);
    testBadInputs(root);
    checks += NoiseRegression.run(root);
    checks += ProjectRegression.run(root);
    checks += GeoCalibrationRegression.run(root);
    checks += GpsAdjustmentRegression.run(root.resolve("gps-adjustment"));
    checks += GroundClassificationRegression.run(root.resolve("ground"));
    checks += GroundStuffRegression.run(root.resolve("ground-stuff"));
    checks += GcpRegression.run(root.resolve("gcp"));
    checks += GpuStereoFiltersRegression.run();
    checks += GpuDepthOutlierRegression.run();
    checks += GpuRangeGuardRegression.run();
    checks += GpuTsgmRegression.run();
    checks += MultiViewSgmRegression.run();
    checks += HelpSearchRegression.run();
    checks += BlockProcessingRegression.run(root.resolve("block-processing"));
    checks += DepthEdgeRegression.run();
    checks += DepthOutlierRegression.run(root.resolve("depth-outlier"));
    checks += MultiViewDisparityRegression.run(root.resolve("multiview-disparity"));
    checks += ReciprocalMatchingRegression.run();
    checks += SelfCalibrationRegression.run(root);
    checks += LayeringRegression.run();
    checks += FilteringPresetRegression.run(root);
    checks += BundleQualityRegression.run(root);
    checks += BundleSchurRegression.run(root);
    checks += SharedCalibrationRegression.run(root);
    checks += AdaptiveCalibrationRegression.run(root.resolve("adaptive-calibration"));
    checks += ThroughputRegression.run(root.resolve("throughput"));
    checks += AlignmentMemoryRegression.run(root.resolve("alignment-memory"));
    checks += ConfidencePatchMatchRegression.run(root);
    checks += PatchMatchValidityRegression.run(root);
    checks += HybridSgmRegression.run(root);
    checks += ExactReuseRegression.run(root.resolve("exact-reuse"));
    checks += TwoViewConfidenceRegression.run(root.resolve("two-view-confidence"));
    checks += PdfReportRegression.run(root.resolve("pdf-report"));
    checks += WorkflowRegression.run(root);
    checks += StorageGpuRegression.run(root);
    checks += DepthIoRegression.run(root);
    checks += PairRangeRegression.run(root);
    checks += FusionGpuRegression.run(root);
    checks += UpgradeStereoRegression.run(root);
    checks += OrientationRegression.run(root);
    checks += OverlapRegression.run(root);
    checks += SurfaceRegression.run(root);
    checks += SubpixelRegression.run();
    checks += MeasuredReprojectionRegression.run();
    checks += LauncherRegression.run(root);
    checks += FenceRegression.run(root);
    checks += RegularizationSurfaceRegression.run();
    checks += SurfaceFusionRegression.run(root);
    checks += ConsolidationSpeedRegression.run(root);
    checks += FollowupRegression.run(root);
    checks += FollowupThroughputRegression.run(root.resolve("throughput-report"));
    checks += ScalingRegression.run(root);
    checks += EsgmProductionRegression.run(root);
    checks += ImageResourceRegression.run(root.resolve("image-resources"));
    checks += PreDenseSpeedRegression.run(root.resolve("pre-dense-speed"));
    System.out.printf(
        Locale.ROOT,
        "PASS: %d checks; OpenCV %s on %s; fixtures: %s%n",
        checks,
        Core.VERSION,
        System.getProperty("os.name"),
        root);
  }

  private static void testSgm() {
    int w = 96, h = 48;
    byte[] left = new byte[w * h], right = new byte[w * h];
    new Random(4).nextBytes(left);
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) right[y * w + x] = left[y * w + Math.min(w - 1, x + 6)];
    Options o = TestSupport.options();
    o.disparities = 16;
    o.census = 5;
    o.median = 1;
    o.uniqueness = .02;
    CensusSgm.Result r = new CensusSgm(o).compute(left, right, w, h);
    int valid = 0;
    double error = 0;
    for (int y = 6; y < h - 6; y++)
      for (int x = 16; x < w - 8; x++)
        if (Float.isFinite(r.disparity[y * w + x])) {
          valid++;
          error += Math.abs(r.disparity[y * w + x] - 6);
        }
    check(valid > 1500, "Textured stereo has enough consistent disparities");
    check(error / valid < .2, "Known 6-pixel disparity recovered");
    o.minDisparity = -10;
    o.disparities = 16;
    CensusSgm.Result reversed = new CensusSgm(o).compute(right, left, w, h);
    near(reversed.disparity[24 * w + 40], -6, .3, "Signed disparity range");
    o.census = 11;
    o.minDisparity = 0;
    o.directions = 4;
    r = new CensusSgm(o).compute(left, right, w, h);
    near(r.disparity[24 * w + 40], 6, .3, "Multiword 11x11 census and four paths");
    byte[] constant = new byte[w * h];
    o.census = 5;
    r = new CensusSgm(o).compute(constant, constant, w, h);
    check(Float.isNaN(r.disparity[24 * w + 40]), "Ambiguous constant texture rejected");
    o.memoryMb = 16;
    o.disparities = 4096;
    boolean rejected = false;
    try {
      new CensusSgm(o).compute(left, right, w, h);
    } catch (IllegalArgumentException e) {
      rejected = true;
    }
    check(rejected, "Memory limit enforced before volume allocation");
    Options colorOptions = TestSupport.options();
    colorOptions.censusMode = "color-distance";
    colorOptions.disparities = 16;
    colorOptions.median = 1;
    byte[] leftColor = new byte[w * h * 3], rightColor = new byte[w * h * 3];
    new Random(81).nextBytes(leftColor);
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++)
        System.arraycopy(
            leftColor, (y * w + Math.min(w - 1, x + 6)) * 3, rightColor, (y * w + x) * 3, 3);
    CensusSgm.Result color =
        new CensusSgm(colorOptions).compute(constant, constant, leftColor, rightColor, w, h);
    near(
        color.disparity[24 * w + 40],
        6,
        .3,
        "RGB-distance census uses chromatic texture even when grayscale is flat");
    Arrays.fill(leftColor, (byte) 100);
    Arrays.fill(rightColor, (byte) 100);
    color = new CensusSgm(colorOptions).compute(constant, constant, leftColor, rightColor, w, h);
    check(Float.isNaN(color.disparity[24 * w + 40]), "Flat RGB texture rejected");
  }

  private static void testGeometry() {
    Camera a = new Camera(240, 240, 199.5, 149.5), b = new Camera(240, 240, 199.5, 149.5);
    b.translation = new double[] {-.25, 0, 0};
    double[] p = {.3, .2, 5}, ua = a.project(p), ub = b.project(p);
    double[] recovered =
        Geometry.triangulate(
            a,
            b,
            FeatureTracks.normalize(a, new Point(ua[0], ua[1])),
            FeatureTracks.normalize(b, new Point(ub[0], ub[1])));
    near(Math3.norm(Math3.sub(recovered, p)), 0, 1e-9, "DLT triangulation");
    List<double[]> source =
        Arrays.asList(
            new double[] {0, 0, 0},
            new double[] {1, 0, 0},
            new double[] {0, 1, 0},
            new double[] {1, 1, .1});
    double[] rotation = Math3.rotation(new double[] {.1, -.2, .3}), translation = {10, 20, 30};
    List<double[]> target = new ArrayList<>();
    for (double[] x : source)
      target.add(Math3.add(Math3.times(Math3.mv(rotation, x), 2.5), translation));
    Georeference.Similarity fit = Georeference.fit(source, target);
    near(fit.scale, 2.5, 1e-9, "Similarity scale");
    for (int i = 0; i < source.size(); i++)
      near(
          Math3.norm(Math3.sub(fit.apply(source.get(i)), target.get(i))),
          0,
          1e-8,
          "Similarity fit");
    near(Georeference.ecef(new Exif.Gps(0, 0, 0))[0], 6378137, .001, "WGS84 equatorial radius");
  }

  private static void testLas(Path root) throws IOException {
    Path out = Files.createDirectory(root.resolve("LAS writer"));
    try (CloudWriter writer = new CloudWriter(out, 1)) {
      writer.point(500000.123, 7000000.456, -12.5, 255, 128, 0);
      writer.point(500001.123, 7000002.456, 4, 1, 2, 3);
    }
    byte[] bytes = Files.readAllBytes(out.resolve("dense.las"));
    ByteBuffer b = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN);
    check(bytes.length == 227 + 2 * 26, "LAS record lengths");
    check(
        new String(bytes, 0, 4, java.nio.charset.StandardCharsets.US_ASCII).equals("LASF"),
        "LAS signature");
    check(
        (bytes[24] & 255) == 1 && (bytes[25] & 255) == 2 && (bytes[104] & 255) == 2,
        "LAS version and RGB point format");
    check(b.getInt(107) == 2 && b.getInt(111) == 2, "LAS counts and returns");
    near(
        b.getInt(227) * b.getDouble(131) + b.getDouble(155),
        500000.123,
        .00011,
        "LAS X round trip");
    near(
        b.getInt(231) * b.getDouble(139) + b.getDouble(163),
        7000000.456,
        .00011,
        "LAS Y round trip");
    check(
        (b.getShort(247) & 65535) == 65535 && (b.getShort(249) & 65535) == 32896, "LAS 16-bit RGB");
    check(Files.size(out.resolve("dense.ply")) > 54, "PLY emitted");
  }

  static Path scene(Path root, int cameras, boolean vertical) throws IOException {
    Path directory =
        Files.createDirectory(root.resolve(vertical ? "vertical images" : "input images"));
    int w = 400, h = 300, extra = 100;
    Mat texture = new Mat(h + extra, w + extra, CvType.CV_8UC3);
    byte[] values = new byte[(h + extra) * (w + extra) * 3];
    new Random(33).nextBytes(values);
    texture.put(0, 0, values);
    Imgproc.GaussianBlur(texture, texture, new Size(3, 3), .65);
    for (int k = 0; k < 200; k++) {
      Random r = new Random(100 + k);
      Point p = new Point(r.nextInt(w + extra), r.nextInt(h + extra));
      Imgproc.circle(
          texture,
          p,
          2 + r.nextInt(8),
          new Scalar(r.nextInt(256), r.nextInt(256), r.nextInt(256)),
          -1);
    }
    byte[] pixels = new byte[values.length];
    texture.get(0, 0, pixels);
    texture.release();
    for (int c = 0; c < cameras; c++) {
      byte[] image = new byte[w * h * 3];
      for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
          int disparity = vertical ? 12 : (y < h / 3 ? 15 : y < 2 * h / 3 ? 10 : 6);
          int sourceX = x + (vertical ? 0 : c * disparity),
              sourceY = y + (vertical ? c * disparity : 0);
          int src = (sourceY * (w + extra) + sourceX) * 3, dst = (y * w + x) * 3;
          System.arraycopy(pixels, src, image, dst, 3);
        }
      Mat m = new Mat(h, w, CvType.CV_8UC3);
      try {
        m.put(0, 0, image);
        Imgcodecs.imwrite(
            directory.resolve(String.format(Locale.ROOT, "%02d.png", c)).toString(), m);
      } finally {
        m.release();
      }
    }
    return directory;
  }

  private static void testRectified(Path root, Path images, Path calibration) throws IOException {
    Path pair = Files.createDirectory(root.resolve("rectified images"));
    Files.copy(images.resolve("00.png"), pair.resolve("00.png"));
    Files.copy(images.resolve("01.png"), pair.resolve("01.png"));
    Path out = root.resolve("rectified result");
    int status =
        TestSupport.run(
            new String[] {
              "-i",
              pair.toString(),
              "-o",
              out.toString(),
              "--calibration",
              calibration.toString(),
              "--rectified",
              "--baseline",
              ".25",
              "--disparities",
              "32",
              "--step",
              "3",
              "--cores",
              "1"
            });
    check(status == 0, "Rectified end-to-end CLI");
    check(Files.exists(out.resolve("dense.las")), "Rectified LAS written");
    ByteBuffer b =
        ByteBuffer.wrap(Files.readAllBytes(out.resolve("dense.las")))
            .order(ByteOrder.LITTLE_ENDIAN);
    int n = b.getInt(107);
    check(n > 3000, "Dense reconstruction has expected population");
    double mean = 0;
    int valid = 0;
    for (int i = 0; i < n; i++) {
      int at = b.getInt(96) + Short.toUnsignedInt(b.getShort(105)) * i;
      double z = b.getInt(at + 8) * b.getDouble(147) + b.getDouble(171);
      double expected = Math.min(Math.abs(z - 4), Math.min(Math.abs(z - 6), Math.abs(z - 10)));
      if (expected < .25) valid++;
      mean += expected;
    }
    check(valid > .9 * n, "Known metric depths recovered from disparity");
    check(mean / n < .2, "Dense depth error bounded");
  }

  private static void testKnownPoses(Path root, Path images, Path calibration) throws IOException {
    Path poses = root.resolve("input poses.csv");
    StringBuilder csv = new StringBuilder(PoseIO.HEADER + "\n");
    for (int i = 0; i < 3; i++)
      csv.append(String.format(Locale.ROOT, "%02d.png,1,0,0,0,1,0,0,0,1,%.3f,0,0%n", i, -i * .25));
    Files.writeString(poses, csv);
    Path out = root.resolve("known poses result");
    check(
        TestSupport.run(
                new String[] {
                  "-i",
                  images.toString(),
                  "-o",
                  out.toString(),
                  "--calibration",
                  calibration.toString(),
                  "--poses",
                  poses.toString(),
                  "--disparities",
                  "40",
                  "--census-mode",
                  "color-distance",
                  "--step",
                  "4",
                  "--cores",
                  "1"
                })
            == 0,
        "Known-pose multi-view reconstruction");
    check(Files.size(out.resolve("dense.las")) > 227 + 2000 * 26, "Multi-view fused cloud");
  }

  private static void testVertical(Path root, Path calibration) throws IOException {
    Path images = scene(root, 2, true), poses = root.resolve("vertical.csv");
    // Vertical transposition must preserve the original Q-matrix coordinate convention.
    Files.writeString(
        poses,
        PoseIO.HEADER + "\n00.png,1,0,0,0,1,0,0,0,1,0,0,0\n01.png,1,0,0,0,1,0,0,0,1,0,-0.25,0\n");
    Path out = root.resolve("vertical result");
    check(
        TestSupport.run(
                new String[] {
                  "-i",
                  images.toString(),
                  "-o",
                  out.toString(),
                  "--calibration",
                  calibration.toString(),
                  "--poses",
                  poses.toString(),
                  "--disparities",
                  "32",
                  "--step",
                  "4",
                  "--cores",
                  "1"
                })
            == 0,
        "Vertical stereo reconstruction");
    ByteBuffer las =
        ByteBuffer.wrap(Files.readAllBytes(out.resolve("dense.las")))
            .order(ByteOrder.LITTLE_ENDIAN);
    double z = las.getInt(las.getInt(96) + 8) * las.getDouble(147) + las.getDouble(171);
    near(z, 5, .2, "Vertical stereo world-space depth");
  }

  private static void testSfm(Path root, Path images, Path calibration) throws IOException {
    Path out = root.resolve("SfM result");
    check(
        TestSupport.run(
                new String[] {
                  "-i",
                  images.toString(),
                  "-o",
                  out.toString(),
                  "--calibration",
                  calibration.toString(),
                  "--seed-pair",
                  "00.png,01.png",
                  "--baseline",
                  ".25",
                  "--disparities",
                  "48",
                  "--step",
                  "5",
                  "--cores",
                  "1",
                  "--min-matches",
                  "15",
                  "--bundle-iterations",
                  "4"
                })
            == 0,
        "Complete SIFT/SfM/PnP/bundle/SGM pipeline");
    check(Files.readAllLines(out.resolve("cameras.csv")).size() == 4, "All cameras registered");
    check(
        Files.readAllLines(out.resolve("sparse.csv")).size() > 30,
        "Sparse multi-view tracks reconstructed");
  }

  private static void testReversed(Path root, Path images, Path calibration) throws IOException {
    Path reversed = Files.createDirectory(root.resolve("reversed images"));
    Files.copy(images.resolve("01.png"), reversed.resolve("00.png"));
    Files.copy(images.resolve("00.png"), reversed.resolve("01.png"));
    Path poses = root.resolve("reversed.csv");
    Files.writeString(
        poses,
        PoseIO.HEADER + "\n00.png,1,0,0,0,1,0,0,0,1,-0.25,0,0\n01.png,1,0,0,0,1,0,0,0,1,0,0,0\n");
    Path out = root.resolve("reversed result");
    check(
        TestSupport.run(
                new String[] {
                  "-i",
                  reversed.toString(),
                  "-o",
                  out.toString(),
                  "--calibration",
                  calibration.toString(),
                  "--poses",
                  poses.toString(),
                  "--disparities",
                  "32",
                  "--step",
                  "4",
                  "--cores",
                  "1"
                })
            == 0,
        "Automatic negative-baseline orientation");
    check(
        Files.exists(out.resolve("pair_0001_0000.pfm")),
        "Swapped disparity labels identify actual reference cameras");
  }

  private static void testBundle(Path root, Path images, Path calibration) throws IOException {
    Options o = TestSupport.options();
    o.calibration = calibration;
    o.maxDimension = 400;
    List<ImageFrame> frames = new ArrayList<>();
    Calibration c = Calibration.load(o, 400, 300);
    try {
      for (int i = 0; i < 3; i++) {
        ImageFrame f =
            new ImageFrame(i, images.resolve(String.format(Locale.ROOT, "%02d.png", i)), c, o);
        f.camera.registered = true;
        f.camera.fixed = i < 2;
        f.camera.translation = new double[] {-.25 * i, 0, 0};
        frames.add(f);
      }
      Random random = new Random(42);
      List<FeatureTracks.Track> tracks = new ArrayList<>();
      List<List<KeyPoint>> keys = new ArrayList<>();
      for (int i = 0; i < 3; i++) keys.add(new ArrayList<>());
      for (int i = 0; i < 80; i++) {
        double[] actual = {
          random.nextDouble() * 2 - 1, random.nextDouble() - 0.5, 4 + random.nextDouble() * 5
        };
        FeatureTracks.Track track = new FeatureTracks.Track();
        for (int j = 0; j < 3; j++) {
          double[] uv = frames.get(j).camera.project(actual);
          keys.get(j).add(new KeyPoint((float) uv[0], (float) uv[1], 1));
          track.observations.add(new FeatureTracks.Observation(j, i));
        }
        track.point = new double[] {actual[0] + .03, actual[1] - .02, actual[2] + .15};
        tracks.add(track);
      }
      for (int i = 0; i < 3; i++) frames.get(i).keypoints = keys.get(i).toArray(new KeyPoint[0]);
      frames.get(2).camera.translation[1] = .04;
      double before = BundleAdjuster.error(frames, tracks, 2);
      BundleAdjuster.optimize(frames, tracks, 12, 2);
      double after = BundleAdjuster.error(frames, tracks, 2);
      check(
          after < before * .02,
          "Bundle adjustment reduces independently constructed reprojection residuals");
    } finally {
      for (ImageFrame f : frames) f.close();
    }
  }

  private static void testCalibration(Path root) throws IOException {
    Path images = Files.createDirectory(root.resolve("calibration images"));
    int cols = 7, rows = 5, cell = 50;
    Mat board = new Mat((rows + 1) * cell, (cols + 1) * cell, CvType.CV_8U, new Scalar(255));
    for (int y = 0; y <= rows; y++)
      for (int x = 0; x <= cols; x++)
        if ((x + y) % 2 == 0)
          Imgproc.rectangle(
              board,
              new Point(x * cell, y * cell),
              new Point((x + 1) * cell - 1, (y + 1) * cell - 1),
              new Scalar(0),
              -1);
    MatOfPoint2f src =
        new MatOfPoint2f(
            new Point(0, 0),
            new Point(board.cols() - 1, 0),
            new Point(board.cols() - 1, board.rows() - 1),
            new Point(0, board.rows() - 1));
    try {
      for (int i = 0; i < 8; i++) {
        double[] rotation = Math3.rotation(new double[] {-.25 + .07 * i, .18 - .035 * i, .03 * i});
        double[][] corners = {{-.16, -.12, 0}, {.16, -.12, 0}, {.16, .12, 0}, {-.16, .12, 0}};
        Point[] dst = new Point[4];
        for (int k = 0; k < 4; k++) {
          double[] p =
              Math3.add(
                  Math3.mv(rotation, corners[k]),
                  new double[] {.015 * Math.sin(i), .02 * Math.cos(i), .65 + .02 * i});
          dst[k] = new Point(650 * p[0] / p[2] + 319.5, 650 * p[1] / p[2] + 239.5);
        }
        MatOfPoint2f dest = new MatOfPoint2f(dst);
        Mat h = Imgproc.getPerspectiveTransform(src, dest), image = new Mat();
        try {
          Imgproc.warpPerspective(
              board,
              image,
              h,
              new Size(640, 480),
              Imgproc.INTER_LINEAR,
              Core.BORDER_CONSTANT,
              new Scalar(180));
          Imgcodecs.imwrite(images.resolve(i + ".png").toString(), image);
        } finally {
          dest.release();
          h.release();
          image.release();
        }
      }
    } finally {
      board.release();
      src.release();
    }
    Path out = root.resolve("calibration result");
    check(
        TestSupport.run(
                new String[] {
                  "-i",
                  images.toString(),
                  "-o",
                  out.toString(),
                  "--calib",
                  "--board-columns",
                  "7",
                  "--board-rows",
                  "5",
                  "--square-size",
                  ".04",
                  "--cores",
                  "1"
                })
            == 0,
        "Chessboard calibration workflow");
    Options o = TestSupport.options();
    o.calibration = out.resolve("camera.txt");
    Calibration c = Calibration.load(o, 640, 480);
    near(c.fx, 650, 50, "Calibrated focal length");
  }

  private static void testBadInputs(Path root) {
    check(TestSupport.run(new String[] {"--help"}) == 0, "Help needs no images");
    check(
        TestSupport.run(new String[] {"-i", root.resolve("missing").toString()}) == 2,
        "Missing input fails clearly");
    check(TestSupport.run(new String[] {"--unknown", "1"}) == 2, "Unknown options fail");
  }
}
