package sgm;

import java.io.*;
import java.nio.*;
import java.nio.file.*;
import java.util.*;
import java.util.regex.*;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;

/** Two-sided evidence bounds, lower/upper retries, transforms, GPU parity and real CLI reuse. */
final class PairRangeRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Core.setNumThreads(1);
    Core.setRNGSeed(1);
    Path root = Files.createTempDirectory(Path.of("target"), "pair ranges ");
    System.out.println("PASS pair-range checks=" + run(root) + "; fixtures=" + root);
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Path dir = Files.createDirectories(root.resolve("pair range regression"));
    evidence();
    expansion();
    boundary(dir);
    transforms(dir);
    missingLayer();
    pipeline(dir);
    verifiedReuse(dir);
    return checks;
  }

  private static Options cpu() {
    Options o = new Options();
    o.compute = "cpu";
    o.gpuDevice = Integer.getInteger("sgm.test.gpu", 0);
    return o;
  }

  private static List<DisparityRange.Sample> band(int n, double disparity) {
    List<DisparityRange.Sample> s = new ArrayList<>();
    for (int i = 0; i < n; i++)
      s.add(
          new DisparityRange.Sample(
              disparity + (i % 11 - 5) * .5,
              550 + i % 20 * 20,
              50 + (i / 20 % 10) * 70,
              "verified_keypoints",
              false));
    return s;
  }

  private static void evidence() {
    Options o = cpu();
    var base = band(1000, 500);
    var range = DisparityRange.fromEvidence(base, 1000, 750, o);
    check(
        range.minimum > 450 && range.count <= 96,
        "Distributed keypoints lift the minimum and narrow the hypothesis count");
    check(
        range.minimum < 497 && range.minimum + range.count - 1 > 503,
        "Both observed tails retain safety margins");
    var isolated = new ArrayList<>(base);
    isolated.add(new DisparityRange.Sample(50, 200, 400, "rectified_preview", false));
    check(
        DisparityRange.fromEvidence(isolated, 1000, 750, o).minimum > 400,
        "A lone unsupported preview mismatch does not drag the interval to zero");
    var anchor = new ArrayList<>(base);
    anchor.add(new DisparityRange.Sample(50, 200, 400, "sparse_geometry", true));
    check(
        DisparityRange.fromEvidence(anchor, 1000, 750, o).minimum <= 34,
        "A credible rare three-view background point is retained");
    anchor = new ArrayList<>(base);
    anchor.add(new DisparityRange.Sample(800, 950, 400, "sparse_geometry", true));
    var foreground = DisparityRange.fromEvidence(anchor, 1000, 750, o);
    check(
        foreground.minimum + foreground.count - 1 >= 816,
        "A credible rare three-view foreground point is retained");
    var secondary = new ArrayList<>(base);
    for (int i = 0; i < 3; i++)
      secondary.add(
          new DisparityRange.Sample(
              110 + i, 300 + i * 80, 100 + i * 200, "rectified_preview", false));
    check(
        DisparityRange.fromEvidence(secondary, 1000, 750, o).minimum <= 94,
        "A spatially supported minority depth band survives robust quantiles");
    List<DisparityRange.Sample> cluster = new ArrayList<>();
    for (int i = 0; i < 80; i++)
      cluster.add(
          new DisparityRange.Sample(
              500 + i % 3, 750 + i % 2, 350 + i % 2, "verified_keypoints", false));
    check(
        DisparityRange.fromEvidence(cluster, 1000, 750, o).minimum == 0,
        "Clustered keypoints retain a broad zero-based fallback");
    check(
        DisparityRange.fromEvidence(base.subList(0, 5), 1000, 750, o).minimum == 0,
        "Too few keypoints retain a broad fallback");
    List<DisparityRange.Sample> high = new ArrayList<>();
    for (int i = 0; i < 60; i++)
      high.add(
          new DisparityRange.Sample(
              690 + i % 3, 710 + i % 10 * 3, 100 + i / 10 * 140, "verified_keypoints", false));
    var narrow = DisparityRange.fromEvidence(high, 750, 1000, o);
    check(
        narrow.minimum > 650 && narrow.count < 100,
        "Coverage uses feasible overlap for high-parallax narrow-overlap pairs");
    o.minDisparityExplicit = true;
    check(
        DisparityRange.fromEvidence(base, 1000, 750, o).minimum == 0,
        "An explicit zero minimum pins the lower bound");
    o.minDisparity = 100;
    check(
        DisparityRange.fromEvidence(base, 1000, 750, o).minimum == 100,
        "A nonzero user lower bound is preserved");
    o.minDisparity = -64;
    var signed = new ArrayList<DisparityRange.Sample>();
    for (int i = 0; i < 100; i++)
      signed.add(
          new DisparityRange.Sample(
              -10 + i % 3, 20 + i % 10 * 10, 10 + i / 10 * 7, "verified_keypoints", false));
    check(
        DisparityRange.fromEvidence(signed, 160, 96, o).minimum == -64,
        "Signed user minima remain fixed");
    o = cpu();
    var dirty = new ArrayList<>(base);
    dirty.add(new DisparityRange.Sample(Double.NaN, 600, 200, "bad", false));
    dirty.add(new DisparityRange.Sample(300, Double.POSITIVE_INFINITY, 200, "bad", false));
    check(
        DisparityRange.fromEvidence(dirty, 1000, 750, o).sampleCount == base.size(),
        "Nonfinite evidence is excluded");
    o.minDisparity = 999;
    boolean failed = false;
    try {
      DisparityRange.bounds(999, 999, 1000, o);
    } catch (IllegalArgumentException e) {
      failed = true;
    }
    check(failed, "An unusable physical minimum fails rather than overflowing");
    o = cpu();
    failed = false;
    try {
      DisparityRange.bounds(0, 8000, 10000, o);
    } catch (IllegalArgumentException e) {
      failed = true;
    }
    check(failed, "An observed span over 4096 hypotheses is not silently truncated");
  }

  private static void expansion() {
    Options o = cpu(), current = o.copy();
    current.minDisparity = 450;
    current.disparities = 64;
    CensusSgm.Stats stats = new CensusSgm.Stats();
    stats.fullIntervalPixels = 1000;
    stats.lowerBoundaryRejected = 100;
    var next = DisparityRange.expansion(o, current, stats, 1000, 0);
    check(
        next != null && next.minimum() < 450 && next.maximum() >= 513,
        "Lower endpoint pressure expands toward smaller disparities");
    stats.lowerBoundaryRejected = 0;
    stats.upperBoundaryRejected = 100;
    next = DisparityRange.expansion(o, current, stats, 1000, 0);
    check(
        next.minimum() == 450 && next.maximum() > 513,
        "Upper endpoint pressure expands toward larger disparities");
    stats.lowerBoundaryRejected = 100;
    next = DisparityRange.expansion(o, current, stats, 1000, 0);
    check(
        next.minimum() < 450 && next.maximum() > 513, "Both pressured boundaries expand together");
    next = DisparityRange.expansion(o, current, stats, 1000, 3);
    check(
        next.minimum() == 0 && next.maximum() == 999,
        "Persistent pressure reaches a broad whole-domain safety fallback");
    o.minDisparity = 450;
    o.minDisparityExplicit = true;
    stats.upperBoundaryRejected = 0;
    check(
        DisparityRange.expansion(o, current, stats, 1000, 0) == null,
        "Automatic expansion respects a pinned lower bound");
    o = cpu();
    o.disparitiesExplicit = true;
    stats.upperBoundaryRejected = 100;
    check(
        DisparityRange.expansion(o, current, stats, 1000, 0) == null,
        "Explicit fixed counts never auto-expand");
    o = cpu();
    stats = new CensusSgm.Stats();
    stats.boundaryRejected = 100000;
    check(
        DisparityRange.expansion(o, current, stats, 1000, 0) == null,
        "Incomplete border intervals cannot trigger range expansion");
    stats.fullIntervalPixels = 1000;
    stats.lowerBoundaryRejected = 3;
    stats.upperBoundaryRejected = 3;
    check(
        DisparityRange.expansion(o, current, stats, 1000, 0) == null,
        "Sparse endpoint noise does not cause retries");
  }

  private static void boundary(Path dir) throws Exception {
    int w = 160, h = 96;
    byte[] a = new byte[w * h], b = new byte[a.length];
    new Random(497).nextBytes(a);
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) b[y * w + x] = a[y * w + Math.min(w - 1, x + 8)];
    boolean gpu = false;
    try {
      gpu = !GpuSgm.devices().isEmpty();
    } catch (RuntimeException ignored) {
    }
    for (boolean low : new boolean[] {true, false}) {
      Options o = cpu();
      o.output = dir;
      o.minDisparity = low ? 8 : 3;
      o.disparities = low ? 17 : 6;
      o.speckleSize = 0;
      byte[] flags = new byte[a.length];
      CensusSgm.Observer observer =
          new CensusSgm.Observer() {
            public void raster(String n, float[] v, int w, int h) {}

            public void searchBoundary(byte[] f, int width, int height) {
              System.arraycopy(f, 0, flags, 0, f.length);
            }
          };
      var r = new CensusSgm(o).computeRecorded(a, b, null, null, w, h, null, null, observer);
      check(
          low ? r.stats.lowerClipped() : r.stats.upperClipped(),
          "Actual " + (low ? "lower" : "upper") + " clipping is detected");
      check(
          r.stats.fullIntervalPixels > 5000,
          "Boundary denominator counts complete searchable intervals");
      long hits = 0;
      for (byte f : flags) if ((f & (low ? 1 : 2)) != 0) hits++;
      check(
          hits == (low ? r.stats.lowerBoundaryRejected : r.stats.upperBoundaryRejected),
          "Per-pixel boundary flags match counters");
      if (gpu) {
        o.compute = "gpu";
        var g = new CensusSgm(o).compute(a, b, w, h);
        check(
            g.stats.lowerBoundaryRejected == r.stats.lowerBoundaryRejected
                && g.stats.upperBoundaryRejected == r.stats.upperBoundaryRejected
                && g.stats.fullIntervalPixels == r.stats.fullIntervalPixels,
            "GPU and CPU boundary/eligible counters agree");
      }
    }
    GpuSgm.closeSession();
  }

  private static void transforms(Path dir) throws Exception {
    Camera camera = new Camera(600, 620, 395, 297);
    double angle = .15, c = Math.cos(angle), s = Math.sin(angle);
    double[] rotation = {c, 0, s, 0, 1, 0, -s, 0, c},
        projection = {650, 0, 430, -130, 0, 655, 310, 0, 0, 0, 1, 0};
    MatOfPoint2f
        points = new MatOfPoint2f(new Point(200, 100), new Point(395, 297), new Point(650, 480)),
        out = new MatOfPoint2f();
    Mat k = camera.intrinsic(),
        d = new MatOfDouble(0, 0, 0, 0, 0),
        r = Math3.mat(rotation, 3, 3),
        p = Math3.mat(projection, 3, 4);
    try {
      Calib3d.undistortPoints(points, out, k, d, r, p);
      Point[] expected = out.toArray(), input = points.toArray();
      for (int i = 0; i < input.length; i++) {
        Point actual = DisparityRange.rectifyPoint(camera, input[i], rotation, projection, false),
            vertical = DisparityRange.rectifyPoint(camera, input[i], rotation, projection, true);
        check(
            Math.hypot(actual.x - expected[i].x, actual.y - expected[i].y) < 1e-3,
            "Rectified keypoint coordinates agree with independent OpenCV projection");
        check(
            actual.x == vertical.y && actual.y == vertical.x,
            "Vertical stereo transposes keypoint axes consistently");
      }
    } finally {
      points.release();
      out.release();
      k.release();
      d.release();
      r.release();
      p.release();
    }
    Path image = dir.resolve("scale.png");
    Mat raster = new Mat(300, 400, CvType.CV_8UC3, new Scalar(80));
    try {
      Imgcodecs.imwrite(image.toString(), raster);
    } finally {
      raster.release();
    }
    Options alignment = cpu();
    alignment.maxDimension = 200;
    Options dense = cpu();
    dense.maxDimension = 400;
    Calibration calibration = new Calibration(300, 300, 199.5, 149.5, new double[5], 400, 300);
    try (ImageFrame a = new ImageFrame(0, image, calibration, alignment);
        ImageFrame b = new ImageFrame(0, image, calibration, dense)) {
      a.keypoints = new KeyPoint[] {new KeyPoint(60, 40, 3, 45, 1, 2, 3)};
      b.inheritKeypoints(a);
      check(
          b.keypoints.length == 1 && b.keypoints[0].pt.x == 120.5 && b.keypoints[0].pt.y == 80.5,
          "Alignment keypoint IDs/coordinates survive independent dense resizing");
      check(
          b.keypointPixelScale == 2 && b.descriptors.empty(),
          "Dense keypoint reuse scales uncertainty without duplicating descriptors");
      check(
          Math.abs(
                  (a.keypoints[0].pt.x - a.camera.cx) / a.camera.fx
                      - (b.keypoints[0].pt.x - b.camera.cx) / b.camera.fx)
              < 1e-10,
          "Resized keypoints preserve the calibrated camera ray");
    }
  }

  private static Path latest(Path project) throws IOException {
    try (var files = Files.list(project.resolve("runs"))) {
      return files.max(Comparator.comparing(Path::toString)).orElseThrow();
    }
  }

  private static void missingLayer() {
    int w = 512, h = 192;
    byte[] a = new byte[w * h], b = new byte[a.length];
    new Random(63728).nextBytes(a);
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++)
        b[y * w + x] = a[y * w + Math.min(w - 1, x + (y < h / 2 ? 30 : 180))];
    Mat left = new Mat(h, w, CvType.CV_8U),
        right = new Mat(h, w, CvType.CV_8U),
        mask = new Mat(h, w, CvType.CV_8U, new Scalar(255));
    try {
      left.put(0, 0, a);
      right.put(0, 0, b);
      Options o = cpu();
      o.siftContrast = 10;
      List<DisparityRange.Sample> foreground = new ArrayList<>();
      for (int i = 0; i < 100; i++)
        foreground.add(
            new DisparityRange.Sample(
                180, 220 + i % 10 * 25, 110 + i / 10 * 7, "verified_keypoints", false));
      var range = DisparityRange.estimate(left, right, mask, mask, o, foreground);
      check(
          range.minimum <= 14 && range.minimum + range.count - 1 >= 196,
          "Whole-domain validation protects a background layer absent from foreground keypoints");
      check(
          range.samples.stream()
              .anyMatch(s -> s.source().equals("coarse_guard") && s.disparity() < 50),
          "Out-of-range background samples pass full-resolution reciprocal/unique photometric"
              + " checks");
    } finally {
      left.release();
      right.release();
      mask.release();
    }
  }

  private static double number(String json, String key) {
    Matcher m = Pattern.compile("\\\"" + key + "\\\":(-?[0-9.eE+]+)").matcher(json);
    if (!m.find()) throw new AssertionError("Missing " + key);
    return Double.parseDouble(m.group(1));
  }

  private static void pipeline(Path dir) throws Exception {
    Path images = Files.createDirectories(dir.resolve("wide images"));
    int w = 512, h = 192, shift = 180;
    byte[] a = new byte[w * h], b = new byte[a.length];
    new Random(6739).nextBytes(a);
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) b[y * w + x] = a[y * w + Math.min(w - 1, x + shift)];
    Mat raster = new Mat(h, w, CvType.CV_8U);
    try {
      raster.put(0, 0, a);
      Imgcodecs.imwrite(images.resolve("0.png").toString(), raster);
      raster.put(0, 0, b);
      Imgcodecs.imwrite(images.resolve("1.png").toString(), raster);
    } finally {
      raster.release();
    }
    Path calibration = dir.resolve("wide-camera.txt");
    new Calibration(400, 400, 255.5, 95.5, new double[5], w, h).write(calibration, "Known shift");
    Path auto = dir.resolve("auto range"),
        broad = dir.resolve("explicit broad"),
        pinned = dir.resolve("pinned minimum");
    List<String> base =
        List.of(
            "-i",
            images.toString(),
            "--calibration",
            calibration.toString(),
            "--rectified",
            "--baseline",
            "0.5",
            "--compute",
            "cpu",
            "--no-gps",
            "--cores",
            "1",
            "--step",
            "3",
            "--speckle-size",
            "0");
    List<String> args = new ArrayList<>(base);
    args.addAll(List.of("-o", auto.toString(), "--cache"));
    check(
        Main.run(args.toArray(String[]::new)) == 0,
        "Automatic two-sided interval reconstructs a wide known shift");
    String report =
        Files.readString(latest(auto).resolve("05_stereo/pair_0000_0001/disparity_range.json"));
    check(
        number(report, "effective_min_disparity") > 100
            && number(report, "effective_disparities") < 128,
        "Public default narrows both ends from actual keypoints");
    check(
        report.contains("coarse_guard") && report.contains("overlap_normalized_x_coverage"),
        "Range diagnostics retain confidence and coarse safety validation");
    byte[] original = Files.readAllBytes(latest(auto).resolve("07_results/dense.las"));
    args.add("--resume");
    check(
        Main.run(args.toArray(String[]::new)) == 0,
        "Automatic positive-minimum ranges resume correctly");
    check(
        Arrays.equals(original, Files.readAllBytes(latest(auto).resolve("07_results/dense.las"))),
        "Resumed narrowed stereo preserves LAS exactly");
    check(
        Files.readString(latest(auto).resolve("logs/console.log")).contains("Reused stereo cache"),
        "Both range bounds and new counters participate in compatible stereo reuse");
    args = new ArrayList<>(base);
    args.addAll(List.of("-o", broad.toString(), "--disparities", "256"));
    check(
        Main.run(args.toArray(String[]::new)) == 0,
        "Explicit broad reference reconstruction succeeds");
    report =
        Files.readString(latest(broad).resolve("05_stereo/pair_0000_0001/disparity_range.json"));
    check(
        number(report, "effective_min_disparity") == 0
            && number(report, "effective_disparities") == 256,
        "Explicit counts preserve the original exact interval");
    ByteBuffer compact = ByteBuffer.wrap(original).order(ByteOrder.LITTLE_ENDIAN),
        reference =
            ByteBuffer.wrap(Files.readAllBytes(latest(broad).resolve("07_results/dense.las")))
                .order(ByteOrder.LITTLE_ENDIAN);
    check(
        compact.getInt(107) >= reference.getInt(107) * .98,
        "Narrowing preserves known-plane dense coverage against a broad reference");
    int offset = compact.getInt(96), count = compact.getInt(107);
    double[] depths = new double[count];
    for (int i = 0; i < count; i++)
      depths[i] =
          compact.getInt(offset + i * Short.toUnsignedInt(compact.getShort(105)) + 8)
                  * compact.getDouble(147)
              + compact.getDouble(171);
    Arrays.sort(depths);
    check(
        Math.abs(depths[count / 2] - 400 * .5 / 180) < .005,
        "Narrowed range retains correct known metric depth");
    args = new ArrayList<>(base);
    args.addAll(List.of("-o", pinned.toString(), "--min-disparity", "0"));
    check(
        Main.run(args.toArray(String[]::new)) == 0,
        "User-pinned zero still estimates the upper limit");
    report =
        Files.readString(latest(pinned).resolve("05_stereo/pair_0000_0001/disparity_range.json"));
    check(
        number(report, "effective_min_disparity") == 0
            && report.contains("\"automatic_minimum\":false"),
        "Pinned lower bounds are documented and honored");
  }

  private static void verifiedReuse(Path dir) throws Exception {
    Path images = Files.createDirectories(dir.resolve("alignment images"));
    int w = 320, h = 240;
    byte[] texture = new byte[(w + 80) * h];
    new Random(1392).nextBytes(texture);
    for (int camera = 0; camera < 3; camera++) {
      byte[] data = new byte[w * h];
      for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
          int disparity = y < h / 3 ? 12 : y < 2 * h / 3 ? 8 : 5;
          data[y * w + x] = texture[y * (w + 80) + x + camera * disparity];
        }
      Mat raster = new Mat(h, w, CvType.CV_8U);
      try {
        raster.put(0, 0, data);
        Imgcodecs.imwrite(images.resolve(camera + ".png").toString(), raster);
      } finally {
        raster.release();
      }
    }
    Path calibration = dir.resolve("alignment-camera.txt"),
        poses = dir.resolve("alignment-poses.csv"),
        project = dir.resolve("verified reuse");
    new Calibration(240, 240, 159.5, 119.5, new double[5], w, h)
        .write(calibration, "Known layered fixture");
    Files.writeString(
        poses,
        PoseIO.HEADER
            + "\n"
            + "0.png,1,0,0,0,1,0,0,0,1,0,0,0\n"
            + "1.png,1,0,0,0,1,0,0,0,1,-.25,0,0\n"
            + "2.png,1,0,0,0,1,0,0,0,1,-.5,0,0\n");
    check(
        Main.run(
                new String[] {
                  "-i",
                  images.toString(),
                  "-o",
                  project.toString(),
                  "--calibration",
                  calibration.toString(),
                  "--poses",
                  poses.toString(),
                  "--extrinsics-mode",
                  "initial",
                  "--bundle-iterations",
                  "0",
                  "--cleanup-passes",
                  "0",
                  "--alignment-max-dimension",
                  "240",
                  "--compute",
                  "cpu",
                  "--cores",
                  "1",
                  "--no-gps",
                  "--speckle-size",
                  "0"
                })
            == 0,
        "Independent alignment/dense resolutions reconstruct with reused verified evidence");
    String json =
        Files.readString(latest(project).resolve("05_stereo/pair_0000_0001/disparity_range.json"));
    check(
        number(json, "verified_keypoints") > 20,
        "Automatic range reuses verified alignment keypoints after dense rescaling");
    check(
        number(json, "sparse_geometry") > 20 && number(json, "processed_matching_width") == 320,
        "Automatic range also uses sparse tracks in the dense rectified frame");
    check(
        Math.abs(number(json, "reprojection_tolerance_pixels") - 8. / 3) < 1e-9,
        "Range uncertainty is converted from alignment pixels to actual dense pixels");
  }
}
