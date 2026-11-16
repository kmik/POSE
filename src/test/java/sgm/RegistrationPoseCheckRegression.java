package sgm;

import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;

/** --gps-neighbors pair additions and --registration-pose-check (GPS distance and tilt). */
final class RegistrationPoseCheckRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Path root = Files.createTempDirectory(Path.of("target"), "pose check ");
    System.out.println("PASS registration pose check checks=" + run(root) + "; fixtures=" + root);
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Files.createDirectories(root);
    nearest();
    poseCheck(root);
    options(root);
    return checks;
  }

  private static void nearest() {
    // Two flight lines 18 m apart, images 10 m apart along each.
    double[][] enu = new double[8][];
    for (int i = 0; i < 4; i++) {
      enu[i] = new double[] {0, 10 * i, 100};
      enu[4 + i] = new double[] {18, 10 * i, 100};
    }
    Set<Long> edges = new TreeSet<>();
    edges.add(PairSelection.key(0, 7)); // a visual pair already chosen
    int added = PairSelection.addNearest(enu, edges, 2);
    for (int i = 0; i < 3; i++) {
      check(edges.contains(PairSelection.key(i, i + 1)), "along-line neighbour " + i);
      check(edges.contains(PairSelection.key(4 + i, 5 + i)), "along-line neighbour " + (4 + i));
    }
    check(edges.contains(PairSelection.key(0, 4)) && edges.contains(PairSelection.key(3, 7)),
        "line ends: the image 18 m across is nearer than the one 20 m along");
    check(!edges.contains(PairSelection.key(0, 2)) && !edges.contains(PairSelection.key(1, 5)), "only the 2 nearest");
    check(edges.contains(PairSelection.key(0, 7)) && added == edges.size() - 1, "existing pairs kept, new ones counted");
    Set<Long> once = new TreeSet<>(edges);
    check(PairSelection.addNearest(enu, once, 2) == 0 && once.equals(edges), "idempotent");
    check(PairSelection.addNearest(new double[][] {{0, 0, 0}}, new TreeSet<>(), 3) == 0, "one image: no pairs");
  }

  private static final double[] NADIR = {1, 0, 0, 0, -1, 0, 0, 0, -1};

  private static void place(Camera c, double[] rotation, double[] centre) {
    c.rotation = rotation.clone();
    c.translation = Math3.times(Math3.mv(rotation, centre), -1);
  }

  private static double[] tilted(double degrees) {
    return Math3.mul(Math3.rotation(new double[] {Math.toRadians(degrees), 0, 0}), NADIR);
  }

  private static void poseCheck(Path root) throws Exception {
    Path image = root.resolve("img.png");
    Imgcodecs.imwrite(image.toString(), new Mat(60, 80, CvType.CV_8UC1, new Scalar(100)));
    Options o = TestSupport.options();
    o.input = root;
    o.output = root.resolve("project");
    o.registrationPoseCheck = "on";
    Calibration calibration = new Calibration(80, 80, 39.5, 29.5, new double[5], 80, 60);
    List<ImageFrame> frames = new ArrayList<>();
    // Model frame: two lines 1.8 apart, 1 apart along, height 9; GPS = 10 x model, rotated, offset.
    var truth = new RegistrationRecovery.Similarity(10, Math3.rotation(new double[] {0, 0, .6}),
        new double[] {500, -300, 40});
    int n = 10;
    double[][] enu = new double[n][];
    double[][] centre = new double[n][];
    try {
      for (int i = 0; i < n; i++) {
        ImageFrame f = new ImageFrame(i, image, calibration, o);
        frames.add(f);
        centre[i] = new double[] {i < 5 ? 0 : 1.8, i % 5, 9};
        enu[i] = truth.apply(centre[i]);
      }
      List<FeatureTracks.Track> tracks = new ArrayList<>();
      RegistrationPoseCheck check = new RegistrationPoseCheck(frames, tracks, enu);
      Camera probe = frames.get(9).camera.copy();
      // Too few registered cameras: no fit, anything passes.
      for (int i = 0; i < 3; i++) {
        place(frames.get(i).camera, NADIR, centre[i]);
        frames.get(i).camera.registered = true;
      }
      place(probe, tilted(25), new double[] {-3, 4, 9});
      check(check.accept(9, probe), "no check before " + RegistrationPoseCheck.MIN_CAMERAS + " cameras");
      for (int i = 3; i < 9; i++) {
        place(frames.get(i).camera, tilted(i % 2 == 0 ? .5 : -.5), centre[i]);
        frames.get(i).camera.registered = true;
      }
      RegistrationDiagnostics log = new RegistrationDiagnostics(n);
      Geometry.LOG = log;
      try {
        place(probe, NADIR, centre[9]);
        check(check.accept(9, probe), "a correct camera passes");
        place(probe, tilted(4), Math3.add(centre[9], new double[] {.1, -.1, .05}));
        check(check.accept(9, probe), "small tilt and 1.4 m offset pass");
        // Tilt-and-shift: 25 deg tilt, 4.2 model units (42 m) sideways.
        place(probe, tilted(25), Math3.add(centre[9], new double[] {0, -4.2, 0}));
        check(!check.accept(9, probe) && check.rejectedGps == 1, "tilt-and-shift is rejected by GPS");
        check(log.poseRejectedLast[9] && log.poseRejection[9].startsWith("gps:"), "rejection recorded: " + log.poseRejection[9]);
        place(probe, tilted(20), centre[9]);
        check(!check.accept(9, probe) && check.rejectedTilt == 1, "a 20 deg tilt at the right place is rejected by tilt");
        check(log.poseRejections[9] == 2 && log.poseRejection[9].startsWith("tilt:"), "second rejection recorded");
        int refits = check.refits;
        frames.get(8).camera.registered = false;
        place(probe, NADIR, centre[9]);
        check(check.accept(9, probe) && check.refits == refits + 1, "fewer registered cameras: refit");
        frames.get(8).camera.registered = true;
        // Post-bundle screen: camera 8 slid 42 m with a tilt; camera 0 is a fixed seed camera.
        double[] keep8r = frames.get(8).camera.rotation.clone(), keep8t = frames.get(8).camera.translation.clone();
        place(frames.get(8).camera, tilted(25), Math3.add(centre[8], new double[] {0, -4.2, 0}));
        frames.get(0).camera.fixed = true;
        FeatureTracks.Track pairOnly = new FeatureTracks.Track(), shared = new FeatureTracks.Track();
        pairOnly.point = new double[] {1, 1, 0};
        pairOnly.observations.add(new FeatureTracks.Observation(7, 0));
        pairOnly.observations.add(new FeatureTracks.Observation(8, 0));
        shared.point = new double[] {1, 2, 0};
        for (int i : new int[] {6, 7, 8}) shared.observations.add(new FeatureTracks.Observation(i, 1));
        tracks.add(pairOnly);
        tracks.add(shared);
        // 1 of 9 cameras off is over 10% but within the floor of 2: removed.
        check(check.screen("test bundle") == 1 && !frames.get(8).camera.registered, "screen removes the slid camera");
        check(pairOnly.point == null && shared.point != null, "points left with one registered view are cleared");
        check(log.poseRejection[8].startsWith("after test bundle: gps:"), "removal recorded: " + log.poseRejection[8]);
        check(check.screen("again") == 0 && frames.get(0).camera.registered, "nothing more to remove; seed stays");
        frames.get(8).camera.rotation = keep8r;
        frames.get(8).camera.translation = keep8t;
        frames.get(8).camera.registered = true;
        check(check.accept(8, frames.get(8).camera), "a removed camera can register again");
        check(!RegistrationPoseCheck.tooMany(2, 9) && RegistrationPoseCheck.tooMany(3, 9)
            && !RegistrationPoseCheck.tooMany(30, 300) && RegistrationPoseCheck.tooMany(31, 300), "removal guard");
        // A camera removed MAX_SCREENED times is not registered again.
        place(frames.get(8).camera, tilted(25), Math3.add(centre[8], new double[] {0, -4.2, 0}));
        check(check.screen("second") == 1, "second removal");
        place(frames.get(8).camera, NADIR, centre[8]);
        check(!check.accept(8, frames.get(8).camera), "after two removals the camera stays out");
        frames.get(8).camera.registered = true;
        frames.get(0).camera.fixed = false;
        tracks.clear();
        log.attempt(9, 40, 30, 30);
        check(!log.poseRejectedLast[9] && log.poseRejections[9] == 2, "a later attempt clears the last-rejection flag");
        var summary = check.summary();
        check(((Number) summary.get("rejected_gps")).intValue() == 1
                && ((Number) summary.get("rejected_tilt")).intValue() == 1
                && ((Number) summary.get("last_tilt_limit_degrees")).doubleValue() == RegistrationPoseCheck.TILT_FLOOR
                && ((Number) summary.get("last_gps_tolerance_metres")).doubleValue() == RegistrationRecovery.GPS_FLOOR,
            "summary: " + summary);
        // Varied view directions (oblique set): the tilt check switches off; GPS still applies.
        o.registrationPoseCheck = "on";
        for (int i = 0; i < 9; i++) place(frames.get(i).camera, tilted(i * 7 - 28), centre[i]);
        RegistrationPoseCheck oblique = new RegistrationPoseCheck(frames, tracks, enu);
        place(probe, tilted(35), centre[9]);
        check(oblique.accept(9, probe), "no tilt check when the block's view directions vary");
        place(probe, tilted(0), Math3.add(centre[9], new double[] {0, -4.2, 0}));
        check(!oblique.accept(9, probe), "GPS check still applies");
      } finally {
        Geometry.LOG = null;
      }
      // Diagnostics: the unregistered image's reason and the summary block.
      FeatureTracks features = new FeatureTracks();
      FeatureTracks.Pair p = new FeatureTracks.Pair(8, 9);
      for (int k = 0; k < 30; k++) p.matches.add(new FeatureTracks.Match(k, k));
      features.pairs.add(p);
      for (int k = 0; k < 20; k++) {
        FeatureTracks.Track t = new FeatureTracks.Track();
        t.point = new double[] {k, 0, 0};
        t.observations.add(new FeatureTracks.Observation(8, k));
        t.observations.add(new FeatureTracks.Observation(9, k));
        features.tracks.add(t);
      }
      for (ImageFrame f : frames) f.keypoints = new KeyPoint[0];
      log.poseRejected(9, "gps: test");
      log.poseCheck = Map.of("rejected_gps", 1);
      log.write(frames, features, o, null);
      Map<Integer, Map<String, Object>> byId = new HashMap<>();
      for (Object item : (List<?>) ProcessingReport.read(o.output.resolve("04_geometry/registration.json"))) {
        var r = ProcessingReport.object(item);
        byId.put(((Number) r.get("image_id")).intValue(), r);
      }
      check("pose_check_rejected".equals(byId.get(9).get("reason")), "reason: " + byId.get(9).get("reason"));
      check(((Number) byId.get(9).get("pose_check_rejections")).intValue() == 4, "rejection count in the file (3 above + 1)");
      var summary = ProcessingReport.map(o.output.resolve("04_geometry/registration_summary.json"));
      check(summary.get("pose_check") != null, "summary has pose_check");
      check(((Number) ProcessingReport.object(summary.get("unregistered_reasons")).get("pose_check_rejected")).intValue() == 1,
          "summary counts the reason");
    } finally {
      for (ImageFrame f : frames) f.close();
    }
  }

  private static void options(Path root) {
    Options d = TestSupport.options();
    check(d.gpsNeighbors == 0 && d.registrationPoseCheck.equals("off"), "both off by default");
    check(RegistrationPoseCheck.create(List.of(), List.of(), d) == null, "off: no check");
    Options p = Options.parse(new String[] {"-i", root.toString(), "-o", root.resolve("p").toString(),
      "--gps-neighbors", "4", "--registration-pose-check", "on"});
    check(p.gpsNeighbors == 4 && p.registrationPoseCheck.equals("on"), "options parse");
    for (String[] bad : new String[][] {{"--gps-neighbors", "-1"}, {"--gps-neighbors", "65"},
        {"--registration-pose-check", "yes"}}) {
      boolean failed = false;
      try {
        Options.parse(new String[] {"-i", root.toString(), "-o", root.resolve("p").toString(), bad[0], bad[1]});
      } catch (IllegalArgumentException e) {
        failed = true;
      }
      check(failed, "rejects " + bad[0] + " " + bad[1]);
    }
    String help = Options.help();
    check(help.contains("--gps-neighbors") && help.contains("--registration-pose-check"), "help lists the options");
    Options a = TestSupport.options(), b = TestSupport.options();
    b.gpsNeighbors = 4;
    b.registrationPoseCheck = "on";
    check(!ReuseCache.settings(a, "gpsNeighbors").equals(ReuseCache.settings(b, "gpsNeighbors")), "matches cache key differs");
    check(!ReuseCache.settings(a, "registrationPoseCheck").equals(ReuseCache.settings(b, "registrationPoseCheck")),
        "geometry cache key differs");
  }
}
