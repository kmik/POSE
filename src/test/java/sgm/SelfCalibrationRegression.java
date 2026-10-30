package sgm;

import java.nio.*;
import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

final class SelfCalibrationRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  private static void near(double a, double b, double tolerance, String message) {
    check(Math.abs(a - b) <= tolerance, message + ": " + a + " vs " + b);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Core.setNumThreads(1);
    Core.setRNGSeed(1);
    if (args.length == 2 && args[0].equals("--verify-existing")) {
      verifyCalibratedCloud(Path.of(args[1]));
      System.out.println("PASS existing calibrated cloud checks=" + checks);
      return;
    }
    Path root = Files.createTempDirectory(Paths.get("target"), "self calibration regression ");
    System.out.println(
        "PASS self calibration/disparity checks=" + run(root) + "; fixtures=" + root);
  }

  static int run(Path root) throws Exception {
    checks = 0;
    SelfCalibration.Data d = fixture(false, false);
    SelfCalibration.Result result = SelfCalibration.optimize(d, 50);
    System.out.println(Json.string(result.report()));
    check(
        result.accepted,
        "Observable non-planar multi-view radial self-calibration accepted: " + result.reason);
    near(Math.exp(result.fitted.lens[0]), 800, 5, "Joint focal recovery");
    near(result.fitted.lens[1], -.16, .012, "Joint k1 recovery");
    near(result.fitted.lens[2], .065, .025, "Joint k2 recovery");
    check(
        result.after < result.before * .25,
        "Held-out reprojection improves independently of training tracks");
    near(d.lens[1], 0, 0, "Rejected/accepted candidate fits do not mutate input intrinsics");
    check(
        result.standardDeviations[0] < .08 && result.condition > 1e-7,
        "Marginal observability accounts for pose and structure freedom");
    SelfCalibration.Result weak = SelfCalibration.optimize(fixture(true, false), 35);
    check(!weak.accepted, "Planar parallel-camera degeneracy rejected");
    System.out.println("Weak scene: " + Json.string(weak.report()));
    SelfCalibration.Result correct = SelfCalibration.optimize(fixture(false, true), 35);
    check(
        !correct.accepted,
        "Correct zero-distortion model is retained without meaningful held-out improvement");
    near(correct.fitted.views.get(0).camera.cx, 499.5, 0, "Principal point remains fixed");
    testDisparity(root);
    testPhotographs(root);
    return checks;
  }

  static SelfCalibration.Data fixture(boolean planar, boolean undistorted) {
    Calibration calibration =
        new Calibration(
            undistorted ? 800 : 830,
            undistorted ? 800 : 830,
            499.5,
            399.5,
            new double[5],
            1000,
            800);
    SelfCalibration.Data data = new SelfCalibration.Data(calibration, false);
    List<Camera> truth = new ArrayList<>();
    double[][] centers = {
      {-1.4, -.5, .2},
      {-.6, .6, -.3},
      {.5, -.8, .5},
      {1.3, .3, -.4},
      {.2, 1.1, .6},
      {-.9, -1.1, -.5}
    };
    double[][] rotations = {
      {.08, -.13, .03},
      {-.10, .05, -.04},
      {.12, .15, .02},
      {-.07, -.09, .06},
      {.15, -.03, -.07},
      {-.14, .12, .04}
    };
    for (int i = 0; i < 6; i++) {
      Camera c = new Camera(800, 800, 499.5, 399.5);
      c.rotation = planar ? Math3.identity() : Math3.rotation(rotations[i]);
      double[] center = planar ? new double[] {i * .4, 0, 0} : centers[i];
      c.translation = Math3.times(Math3.mv(c.rotation, center), -1);
      truth.add(c);
      Camera initial = c.copy();
      initial.fx = initial.fy = calibration.fx;
      if (i > 0 && !planar && !undistorted) {
        initial.rotation =
            Math3.mul(Math3.rotation(new double[] {.0015, -.002, .001}), initial.rotation);
        initial.translation = Math3.add(initial.translation, new double[] {.015, -.01, .02});
      }
      data.views.add(new SelfCalibration.View(initial, 1, 1));
    }
    Random random = new Random(991);
    for (int id = 0; id < 650; id++) {
      double[] p = {
        random.nextDouble() * 11 - 5.5,
        random.nextDouble() * 8 - 4,
        planar ? 10 : 6 + random.nextDouble() * 9
      };
      SelfCalibration.Sample sample = new SelfCalibration.Sample(id, p, id % 5 == 0);
      for (int i = 0; i < truth.size(); i++) {
        double[] q = truth.get(i).view(p);
        double x = q[0] / q[2],
            y = q[1] / q[2],
            r = x * x + y * y,
            radial = undistorted ? 1 : 1 - .16 * r + .065 * r * r;
        double u = 800 * x * radial + 499.5 + random.nextGaussian() * .08,
            v = 800 * y * radial + 399.5 + random.nextGaussian() * .08;
        if (u > 8 && u < 992 && v > 8 && v < 792)
          sample.seen.add(new SelfCalibration.Seen(i, u, v));
      }
      if (sample.seen.size() >= 3) data.samples.add(sample);
    }
    return data;
  }

  private static void testPhotographs(Path root) throws Exception {
    Path dir = Files.createDirectory(root.resolve("distorted photographs"));
    Path original = GeoCalibrationRegression.render(dir, 6),
        distorted = Files.createDirectory(dir.resolve("distorted"));
    int w = 640, h = 480;
    float[] mx = new float[w * h], my = new float[w * h];
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) {
        double xd = (x - 319.5) / 420, yd = (y - 239.5) / 420, rd = Math.hypot(xd, yd), ru = rd;
        for (int iter = 0; iter < 12; iter++) {
          double r2 = ru * ru;
          ru -= (ru * (1 - .14 * r2 + .07 * r2 * r2) - rd) / (1 - 3 * .14 * r2 + 5 * .07 * r2 * r2);
        }
        double ratio = rd > 0 ? ru / rd : 1;
        mx[y * w + x] = (float) (420 * xd * ratio + 319.5);
        my[y * w + x] = (float) (420 * yd * ratio + 239.5);
      }
    Mat mapX = new Mat(h, w, CvType.CV_32F), mapY = new Mat(h, w, CvType.CV_32F);
    mapX.put(0, 0, mx);
    mapY.put(0, 0, my);
    try {
      for (int i = 0; i < 6; i++) {
        Mat image = Imgcodecs.imread(original.resolve(i + ".png").toString()), warped = new Mat();
        try {
          Imgproc.remap(image, warped, mapX, mapY, Imgproc.INTER_LINEAR);
          Imgcodecs.imwrite(distorted.resolve(i + ".png").toString(), warped);
        } finally {
          image.release();
          warped.release();
        }
      }
    } finally {
      mapX.release();
      mapY.release();
    }
    Options o = TestSupport.options();
    o.input = distorted;
    o.output = Files.createDirectory(dir.resolve("calibration"));
    o.features = 6000;
    o.cores = 1;
    o.maxDimension = 640;
    Calibration initial = new Calibration(425, 425, 319.5, 239.5, new double[5], w, h);
    Calibration result = SelfCalibration.refine(initial, Main.images(o), o, null);
    System.out.println(
        "Photographic calibration: "
            + Files.readString(o.output.resolve("01_calibration/refinement/result.json")));
    check(
        result != initial,
        "Feature extraction, sparse bootstrap and radial calibration accept a distorted six-view"
            + " scene");
    near(result.fx, 420, 12, "Photographic joint focal recovery");
    near(result.distortion[0], -.14, .035, "Photographic k1 recovery");
    near(result.distortion[1], .07, .06, "Photographic k2 recovery");
    Path jpegImages = Files.createDirectory(dir.resolve("EXIF photographs"));
    for (int i = 0; i < 6; i++) {
      Mat image = Imgcodecs.imread(distorted.resolve(i + ".png").toString());
      Path jpg = jpegImages.resolve(i + ".jpg");
      try {
        Imgcodecs.imwrite(jpg.toString(), image);
      } finally {
        image.release();
      }
      GeoCalibrationRegression.inject(
          jpg, GeoCalibrationRegression.tiff(new Exif.Gps(60, 25, 100), 23, true, true));
    }
    Path project = dir.resolve("automatic calibrated project");
    int status =
        TestSupport.run(
            new String[] {
              "-i",
              jpegImages.toString(),
              "-o",
              project.toString(),
              "--max-dimension",
              "480",
              "--features",
              "6000",
              "--neighbors",
              "2",
              "--step",
              "8",
              "--cores",
              "1",
              "--no-gps"
            });
    check(
        status == 0,
        "No calibration, focal or disparity arguments: full distorted-image pipeline completes");
    Options reload = TestSupport.options();
    reload.calibration = project.resolve("camera.txt");
    Calibration applied = Calibration.load(reload, w, h);
    near(
        applied.distortion[0],
        -.14,
        .04,
        "Automatic pipeline exports and applies recovered distortion");
    check(
        Files.size(project.resolve("dense.las")) > 227 + 100 * 26,
        "Automatic calibrated stereo exports a substantial final cloud");
    verifyCalibratedCloud(project);
  }

  private static void verifyCalibratedCloud(Path project) throws Exception {
    double[][] trueCenters = {
      {-.9, 0, 0}, {0, .28, .15}, {.8, -.35, .28}, {.2, .7, -.15}, {.4, -.7, .1}, {-.6, .6, .3}
    };
    List<double[]> source = new ArrayList<>(), target = new ArrayList<>();
    List<String> rows = Files.readAllLines(project.resolve("cameras.csv"));
    for (int i = 1; i < rows.size(); i++) {
      List<String> row = PoseIO.csv(rows.get(i));
      double[] r = new double[9], t = new double[3];
      for (int k = 0; k < 9; k++) r[k] = Double.parseDouble(row.get(k + 1));
      for (int k = 0; k < 3; k++) t[k] = Double.parseDouble(row.get(k + 10));
      source.add(Math3.times(Math3.mv(Math3.transpose(r), t), -1));
      target.add(trueCenters[i - 1]);
    }
    Georeference.Similarity alignment = Georeference.fit(source, target);
    double square = 0;
    for (int i = 0; i < source.size(); i++)
      square += Math.pow(Math3.norm(Math3.sub(alignment.apply(source.get(i)), target.get(i))), 2);
    check(
        Math.sqrt(square / source.size()) < .02,
        "Final camera-center shape agrees with independent synthetic controls");
    ByteBuffer las =
        ByteBuffer.wrap(Files.readAllBytes(project.resolve("dense.las")))
            .order(ByteOrder.LITTLE_ENDIAN);
    int count = las.getInt(107), offset = las.getInt(96), within = 0;
    for (int i = 0; i < count; i++) {
      double[] point = new double[3];
      for (int k = 0; k < 3; k++)
        point[k] =
            las.getInt(offset + i * Short.toUnsignedInt(las.getShort(105)) + k * 4)
                    * las.getDouble(131 + k * 8)
                + las.getDouble(155 + k * 8);
      point = alignment.apply(point);
      double distance =
          Math.min(
              Math.abs(point[2] - 4), Math.min(Math.abs(point[2] - 7), Math.abs(point[2] - 11)));
      if (distance < .25) within++;
    }
    check(
        within > count * .9,
        "Final calibrated cloud agrees with known depth surfaces, not just reprojection"
            + " statistics");
    Properties marker = new Properties();
    try (java.io.Reader reader = Files.newBufferedReader(project.resolve("project.properties"))) {
      marker.load(reader);
    }
    String events =
        Files.readString(
            project
                .resolve("runs")
                .resolve(marker.getProperty("latest_successful_run"))
                .resolve("logs/events.jsonl"));
    check(
        events.contains("\"orientation_source\":\"accepted_self_calibration\""),
        "Final SfM preserves accepted seed orientation instead of discarding its refinement");
  }

  private static void testDisparity(Path root) throws Exception {
    Options o = TestSupport.options();
    o.memoryMb = 512;
    List<Double> samples = new ArrayList<>();
    for (int i = 0; i < 200; i++) samples.add(190 + i % 40.0);
    samples.add(450.);
    DisparityRange estimated = DisparityRange.fromSamples(samples, 480, o);
    check(
        estimated.count > 230 && estimated.count < 400,
        "Wide disparity interval includes margin and resists a sparse outlier");
    check(
        DisparityRange.memoryLimit(1600, 1200, o) < 4096, "Automatic range respects memory budget");
    Path images = Files.createDirectory(root.resolve("wide stereo")),
        project = root.resolve("wide stereo project");
    int w = 480, h = 128, shift = 180;
    Mat texture = new Mat(h, w + shift, CvType.CV_8UC3);
    byte[] values = new byte[h * (w + shift) * 3];
    new Random(572).nextBytes(values);
    texture.put(0, 0, values);
    Imgproc.GaussianBlur(texture, texture, new Size(3, 3), .65);
    Mat left = texture.submat(0, h, 0, w), right = texture.submat(0, h, shift, w + shift);
    try {
      Imgcodecs.imwrite(images.resolve("0.png").toString(), left);
      Imgcodecs.imwrite(images.resolve("1.png").toString(), right);
    } finally {
      left.release();
      right.release();
      texture.release();
    }
    Path calibration = root.resolve("wide stereo camera.txt");
    new Calibration(320, 320, 239.5, 63.5, new double[5], w, h)
        .write(calibration, "Known rectified camera");
    int status =
        TestSupport.run(
            new String[] {
              "-i",
              images.toString(),
              "-o",
              project.toString(),
              "--calibration",
              calibration.toString(),
              "--rectified",
              "--baseline",
              "0.25",
              "--step",
              "4",
              "--cores",
              "1"
            });
    check(
        status == 0,
        "Automatic disparity pipeline reconstructs a shift beyond the old 128 default");
    Path run;
    try (java.util.stream.Stream<Path> files = Files.list(project.resolve("runs"))) {
      run = files.findFirst().orElseThrow();
    }
    String json = Files.readString(run.resolve("05_stereo/pair_0000_0001/disparity_range.json"));
    check(
        json.contains("\"automatic\":true") && json.contains("effective_disparities"),
        "Chosen per-pair search and attempts exported");
    ByteBuffer las =
        ByteBuffer.wrap(Files.readAllBytes(project.resolve("dense.las")))
            .order(ByteOrder.LITTLE_ENDIAN);
    int count = las.getInt(107), offset = las.getInt(96);
    check(count > 500, "Wide stereo retains substantial valid surface coverage");
    double sum = 0;
    for (int i = 0; i < count; i++) {
      double z =
          las.getInt(offset + Short.toUnsignedInt(las.getShort(105)) * i + 8) * las.getDouble(147)
              + las.getDouble(171);
      sum += Math.abs(z - 320 * .25 / 180);
    }
    check(
        sum / count < .005,
        "Automatically selected wide range recovers independently known metric depth");
    Options explicit =
        Options.parse(
            new String[] {
              "-i",
              images.toString(),
              "--calibration",
              calibration.toString(),
              "--disparities",
              "64"
            });
    check(
        explicit.disparitiesExplicit && explicit.disparities == 64,
        "Explicit user disparity setting overrides automatic estimation");
  }
}
