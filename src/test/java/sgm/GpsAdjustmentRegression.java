package sgm;

import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;

/**
 * Synthetic two-line nadir block with known GPS time offset, lever arm, rolling-shutter readout
 * and focal length: the GPS-weighted adjustment must recover them (G3-G5), the off switches must
 * leave them untouched, and the options and XMP parsing must validate.
 */
final class GpsAdjustmentRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Path root = Files.createTempDirectory(Path.of("target"), "gps-adjustment-");
    System.out.println("PASS GPS adjustment: " + run(root) + "; " + root);
  }

  static final double FOCAL = 1000, START_FOCAL = 1060, DT = -.2, READOUT = .04;
  static final double[] LEVER = {.1, -.25, .3};

  static int run(Path root) throws Exception {
    checks = 0;
    String xmp =
        "<rdf:Description drone-dji:FlightXSpeed=\"+4.40\" drone-dji:FlightYSpeed=\"-0.20\""
            + " drone-dji:CalibratedFocalLength=\"4545.454590\"/><drone-dji:RelativeAltitude>"
            + "+99.90</drone-dji:RelativeAltitude>";
    check(Exif.xmpNumber(xmp, "FlightXSpeed") == 4.4, "XMP attribute parsed");
    check(Exif.xmpNumber(xmp, "FlightYSpeed") == -.2, "signed XMP attribute parsed");
    check(Exif.xmpNumber(xmp, "RelativeAltitude") == 99.9, "XMP element form parsed");
    check(Double.isNaN(Exif.xmpNumber(xmp, "FlightZSpeed")), "missing XMP value is NaN");
    Exif.Metadata m = new Exif.Metadata();
    m.captureTime = "2020:07:17 14:09:14";
    m.captureSubSec = "25";
    double t = m.captureSeconds();
    m.captureSubSec = "";
    check(Math.abs(t - m.captureSeconds() - .25) < 1e-9, "sub-second capture time");
    for (String[] bad :
        new String[][] {
          {"--gps-intrinsics", "f,q"},
          {"--gps-time-offset", "5"},
          {"--gps-lever-arm", "1,2"},
          {"--rolling-shutter", "0.5"},
          {"--gps-focal-prior", "-3"},
          {"--gps-height-sigma", "0"}
        }) {
      boolean rejected = false;
      try {
        Options.parse(new String[] {"-i", "x", "-o", root.toString(), bad[0], bad[1]});
      } catch (IllegalArgumentException e) {
        rejected = true;
      }
      check(rejected, "invalid " + bad[0] + " rejected");
    }

    // Single flight line: GPS fixes scale, direction and position; the downward view fixes roll.
    {
      double[] truthRotation = Math3.rotation(new double[] {.4, -1.1, 2.3});
      double truthScale = 3.7;
      double[] truthShift = {612000, 6933000, 190};
      List<double[]> source = new ArrayList<>(), target = new ArrayList<>(), views = new ArrayList<>();
      Random random = new Random(5);
      for (int i = 0; i < 20; i++) {
        double[] world = {10 + i * 9.5, 40 + i * 1.2, 100 + random.nextGaussian() * .05};
        // Reconstruction frame = inverse similarity of the world.
        double[] local =
            Math3.times(Math3.mv(Math3.transpose(truthRotation), Math3.sub(world, truthShift)), 1 / truthScale);
        source.add(local);
        target.add(Math3.add(world, new double[] {random.nextGaussian() * .5, random.nextGaussian() * .5, random.nextGaussian() * .3}));
        double[] worldView = Math3.mv(Math3.rotation(new double[] {random.nextGaussian() * .03, random.nextGaussian() * .03, 0}), new double[] {0, 0, -1});
        views.add(Math3.mv(Math3.transpose(truthRotation), worldView));
      }
      check(Georeference.collinear(target), "single line detected as collinear");
      Georeference.Similarity fit = Georeference.lineFit(source, target, views, 10);
      check(Math.abs(fit.scale / truthScale - 1) < .02, "single-line scale: " + fit.scale);
      double[] down = Math3.mv(fit.rotation, Math3.mv(Math3.transpose(truthRotation), new double[] {0, 0, -1}));
      check(down[2] < -Math.cos(Math.toRadians(3)), "single-line roll puts the view down: " + down[2]);
      double worst = 0;
      for (int i = 0; i < 20; i++)
        worst = Math.max(worst, Math3.norm(Math3.sub(fit.apply(source.get(i)), target.get(i))));
      check(worst < 2.5, "single-line camera residuals at GPS noise: " + worst);
    }

    // Default model: everything estimated, focal prior at the truth.
    Block full = new Block(root.resolve("full"));
    full.options.gpsFocalPrior = "" + FOCAL;
    full.options.gpsFocalSigma = .01;
    GpsAdjustment.run(full.frames, full.tracks, full.options, full.crs, full.controls, full.dir, 1);
    Map<String, Object> report =
        JsonRead.object(JsonRead.read(full.dir.resolve("gps_bundle.json")));
    Map<String, Object> model = JsonRead.object(report.get("gps_model"));
    double dt = ((Number) model.get("time_offset_seconds")).doubleValue();
    double readout =
        ((Number) JsonRead.object(report.get("rolling_shutter")).get("readout_seconds"))
            .doubleValue();
    double focal = full.frames.get(0).calibration.fx;
    check(Boolean.TRUE.equals(report.get("extended_model_accepted")), "extended model accepted");
    // Identifiable parts only. With two opposite lines (5 and 8 m/s) the along-track antenna
    // offsets are 5 dt - Ly + tau and -(8 dt - Ly) + tau with a free block shift tau, so only
    // 13 dt - 2 Ly is determined; the vertical lever arm trades against the block height. The
    // cross-track lever arm flips with heading and is determined.
    List<?> lever = (List<?>) model.get("lever_arm_camera_metres");
    double ly = ((Number) lever.get(1)).doubleValue();
    check(
        Math.abs(13 * dt - 2 * ly - (13 * DT - 2 * LEVER[1])) < .1,
        "along-track antenna offset recovered: dt " + dt + ", ly " + ly);
    check(
        Math.abs(((Number) lever.get(0)).doubleValue() - LEVER[0]) < .1,
        "cross-track lever arm recovered: " + lever);
    check(Math.abs(readout - READOUT) < .01, "rolling-shutter readout recovered: " + readout);
    check(Math.abs(focal - FOCAL) < 10, "focal moved to the prior/truth: " + focal);
    check(((Number) report.get("bundle_rms_metres")).doubleValue() < .1, "GPS fit at noise level");
    check(
        ((Number) report.get("reprojection_rms_after_pixels")).doubleValue() < .5,
        "image fit at noise level");
    check("supplied".equals(report.get("velocity_source")), "supplied velocities used");
    full.close();

    // Off switches: poses-only G1, nothing else moves.
    Block off = new Block(root.resolve("off"));
    off.options.gpsIntrinsics = "none";
    off.options.gpsTimeOffset = "off";
    off.options.gpsLeverArm = "off";
    off.options.rollingShutter = "off";
    GpsAdjustment.run(off.frames, off.tracks, off.options, off.crs, off.controls, off.dir, 1);
    report = JsonRead.object(JsonRead.read(off.dir.resolve("gps_bundle.json")));
    model = JsonRead.object(report.get("gps_model"));
    check(((Number) model.get("time_offset_seconds")).doubleValue() == 0, "time offset off");
    check(
        ((Number) JsonRead.object(report.get("rolling_shutter")).get("readout_seconds"))
                .doubleValue()
            == 0,
        "rolling shutter off");
    check(off.frames.get(0).calibration.fx == START_FOCAL, "lens untouched when off");
    off.close();

    // Fixed values are honoured, not estimated.
    Block fixed = new Block(root.resolve("fixed"));
    fixed.options.gpsIntrinsics = "none";
    fixed.options.gpsTimeOffset = "-0.2";
    fixed.options.gpsLeverArm = "0.1,-0.25,0.3";
    fixed.options.rollingShutter = "0.04";
    GpsAdjustment.run(
        fixed.frames, fixed.tracks, fixed.options, fixed.crs, fixed.controls, fixed.dir, 1);
    report = JsonRead.object(JsonRead.read(fixed.dir.resolve("gps_bundle.json")));
    model = JsonRead.object(report.get("gps_model"));
    check(((Number) model.get("time_offset_seconds")).doubleValue() == -.2, "fixed time offset");
    check(
        ((Number) JsonRead.object(report.get("rolling_shutter")).get("readout_seconds"))
                .doubleValue()
            == .04,
        "fixed readout");
    fixed.close();
    return checks;
  }

  /** Two opposite nadir lines at 5 and 8 m/s, 100 m above 0-15 m relief, 0.3 px noise. */
  static final class Block implements AutoCloseable {
    final Options options = TestSupport.options();
    final List<ImageFrame> frames = new ArrayList<>();
    final List<FeatureTracks.Track> tracks = new ArrayList<>();
    final Map<Integer, double[]> controls = new LinkedHashMap<>();
    final Crs crs = new Crs("EPSG:3067");
    final Path dir;

    Block(Path dir) throws Exception {
      this.dir = dir;
      Files.createDirectories(dir);
      options.output = dir;
      options.quality = "custom";
      options.maxDimension = 1000;
      options.maxDimensionExplicit = true;
      options.gpsSigma = .05;
      options.iterations = Integer.getInteger("sgm.iterations", 10);
      options.gpsAdjustmentRounds = Integer.getInteger("sgm.rounds", 3);
      Path image = dir.resolve("blank.png");
      Mat blank = new Mat(750, 1000, CvType.CV_8U, new Scalar(0));
      try {
        Imgcodecs.imwrite(image.toString(), blank);
      } finally {
        blank.release();
      }
      Calibration start =
          new Calibration(START_FOCAL, START_FOCAL, 499.5, 374.5, new double[5], 1000, 750);
      Random random = new Random(4711);
      List<Camera> truth = new ArrayList<>();
      for (int line = 0; line < 2; line++)
        for (int i = 0; i < 9; i++) {
          ImageFrame frame = new ImageFrame(frames.size(), image, start, options, true);
          Camera c = frame.camera;
          double speed = line == 0 ? 5 : 8, sign = line == 0 ? 1 : -1;
          double[] axes = {sign, 0, 0, 0, -sign, 0, 0, 0, -1};
          double[] tilt = {
            random.nextGaussian() * .01, random.nextGaussian() * .01, random.nextGaussian() * .01
          };
          c.rotation = Math3.mul(Math3.rotation(tilt), axes);
          double[] centre = {30 * line, sign * (i * 10 - 40), 100 + random.nextGaussian() * .3};
          c.translation = Math3.times(Math3.mv(c.rotation, centre), -1);
          c.registered = true;
          c.velocity = new double[] {0, sign * speed, 0};
          Camera exact = c.copy();
          exact.fx = exact.fy = FOCAL;
          truth.add(exact);
          frames.add(frame);
          controls.put(
              frame.id,
              Math3.add(
                  Math3.add(
                      Math3.add(centre, Math3.times(c.velocity, DT)),
                      Math3.mv(Math3.transpose(c.rotation), LEVER)),
                  new double[] {
                    random.nextGaussian() * .03,
                    random.nextGaussian() * .03,
                    random.nextGaussian() * .03
                  }));
        }
      List<List<KeyPoint>> keys = new ArrayList<>();
      for (int i = 0; i < frames.size(); i++) keys.add(new ArrayList<>());
      for (int p = 0; p < 1500; p++) {
        double[] world = {
          -60 + random.nextDouble() * 150, -90 + random.nextDouble() * 180, random.nextDouble() * 15
        };
        FeatureTracks.Track track = new FeatureTracks.Track();
        track.id = p;
        for (ImageFrame frame : frames) {
          double[] uv = rollingShutter(truth.get(frame.id), world);
          if (uv == null || uv[0] < 5 || uv[1] < 5 || uv[0] > 994 || uv[1] > 744) continue;
          track.observations.add(new FeatureTracks.Observation(frame.id, keys.get(frame.id).size()));
          keys.get(frame.id)
              .add(
                  new KeyPoint(
                      (float) (uv[0] + random.nextGaussian() * .3),
                      (float) (uv[1] + random.nextGaussian() * .3),
                      1));
        }
        if (track.observations.size() < 3) continue;
        track.point =
            new double[] {
              world[0] + random.nextGaussian() * .2,
              world[1] + random.nextGaussian() * .2,
              world[2] + random.nextGaussian() * .2
            };
        tracks.add(track);
      }
      for (ImageFrame frame : frames) {
        frame.keypoints = keys.get(frame.id).toArray(KeyPoint[]::new);
        // Start near, not at, the truth.
        Camera c = frame.camera;
        c.rotation = Math3.mul(Math3.rotation(new double[] {.002, -.002, .001}), c.rotation);
        c.translation = Math3.add(c.translation, new double[] {.2, -.2, .1});
      }
    }

    /** Projection with row-dependent camera position C + V readout (v/h - 1/2). */
    static double[] rollingShutter(Camera c, double[] world) {
      double[] uv = c.project(world);
      for (int iter = 0; uv != null && iter < 6; iter++) {
        Camera moved = c.copy();
        double s = READOUT * (uv[1] / 750 - .5);
        double[] centre = Math3.add(c.center(), Math3.times(c.velocity, s));
        moved.translation = Math3.times(Math3.mv(c.rotation, centre), -1);
        uv = moved.project(world);
      }
      return uv;
    }

    public void close() {
      for (ImageFrame frame : frames) frame.close();
    }
  }
}
