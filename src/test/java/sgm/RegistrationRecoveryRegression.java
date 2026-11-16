package sgm;

import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;

/**
 * Registration diagnostics (reasons per image), match repair planning, model merging maths
 * (Umeyama similarity, camera transform, robust GPS fit, shared-image fit), the GUI text and the
 * options of --registration-recovery / --match-repair.
 */
final class RegistrationRecoveryRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Path root = Files.createTempDirectory(Path.of("target"), "registration recovery ");
    System.out.println("PASS registration/recovery checks=" + run(root) + "; fixtures=" + root);
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Files.createDirectories(root);
    similarity();
    fits();
    diagnosticsAndRepair(root);
    options(root);
    return checks;
  }

  private static double[] rotation(double x, double y, double z) {
    return Math3.rotation(new double[] {x, y, z});
  }

  private static void similarity() {
    var truth = new RegistrationRecovery.Similarity(2.5, rotation(.3, -.2, .9), new double[] {10, -4, 7});
    Random random = new Random(5);
    List<double[]> src = new ArrayList<>(), dst = new ArrayList<>();
    for (int i = 0; i < 12; i++) {
      double[] p = {random.nextGaussian() * 5, random.nextGaussian() * 5, random.nextGaussian()};
      src.add(p);
      dst.add(truth.apply(p));
    }
    var fit = RegistrationRecovery.umeyama(src, dst, src.size());
    check(fit != null && fit.rms() < 1e-9, "Umeyama recovers an exact similarity");
    check(Math.abs(fit.similarity().s() - 2.5) < 1e-9, "scale");
    double[] x = {1, 2, 3};
    check(Math3.norm(Math3.sub(fit.similarity().apply(x), truth.apply(x))) < 1e-9, "maps like the truth");
    check(Math3.norm(Math3.sub(truth.inverse().apply(truth.apply(x)), x)) < 1e-12, "inverse");
    var other = new RegistrationRecovery.Similarity(.5, rotation(-.1, .4, .2), new double[] {1, 1, 1});
    check(Math3.norm(Math3.sub(truth.after(other).apply(x), truth.apply(other.apply(x)))) < 1e-12, "composition");
    // A camera moved by the similarity sees the moved point at the same pixel.
    Camera c = new Camera(300, 300, 100, 80);
    c.rotation = rotation(.05, -.1, .2);
    c.translation = new double[] {.3, -.2, 1};
    double[] point = Math3.add(c.center(), Math3.mv(Math3.transpose(c.rotation), new double[] {.4, -.3, 6}));
    double[] before = c.project(point);
    Camera moved = c.copy();
    truth.transform(moved);
    double[] after = moved.project(truth.apply(point));
    check(before != null && after != null && Math.hypot(before[0] - after[0], before[1] - after[1]) < 1e-6,
        "camera transform keeps the projection");
    check(Math3.norm(Math3.sub(moved.center(), truth.apply(c.center()))) < 1e-9, "camera centre moves");
  }

  private static void fits() throws Exception {
    // A straight strip of cameras over a ground plane, in a model frame; GPS = true similarity of it.
    var truth = new RegistrationRecovery.Similarity(12, rotation(.2, .1, -.7), new double[] {300, -200, 140});
    List<ImageFrame> frames = new ArrayList<>();
    Path image = Files.createTempDirectory(Path.of("target"), "recovery frames ").resolve("0.png");
    Imgcodecs.imwrite(image.toString(), new Mat(60, 80, CvType.CV_8UC1, new Scalar(100)));
    Options o = TestSupport.options();
    o.output = image.getParent().resolve("out");
    Calibration calibration = new Calibration(80, 80, 39.5, 29.5, new double[5], 80, 60);
    int n = 8;
    double[][] enu = new double[n][];
    List<Integer> cameras = new ArrayList<>();
    boolean[] member = new boolean[n];
    try {
      for (int i = 0; i < n; i++) {
        ImageFrame f = new ImageFrame(i, image, calibration, o);
        frames.add(f);
        f.camera.rotation = new double[] {1, 0, 0, 0, -1, 0, 0, 0, -1}; // looking down (-z)
        double[] centre = {i * 1.0, 0, 5};
        f.camera.translation = Math3.times(Math3.mv(f.camera.rotation, centre), -1);
        f.camera.registered = true;
        enu[i] = truth.apply(centre);
        cameras.add(i);
        member[i] = true;
      }
      // Ground tie points on z = 0 seen by the cameras; the GPS "up" is the model's +z after truth.
      List<FeatureTracks.Track> tracks = new ArrayList<>();
      Random random = new Random(9);
      for (int k = 0; k < 200; k++) {
        FeatureTracks.Track t = new FeatureTracks.Track();
        t.point = new double[] {random.nextDouble() * 8, random.nextDouble() * 6 - 3, 0};
        t.observations.add(new FeatureTracks.Observation(k % n, 0));
        tracks.add(t);
      }
      // Make GPS "up" consistent with the truth: rotate the ENU targets so the model's +z maps to +z.
      double[] up = Math3.mv(truth.q(), new double[] {0, 0, 1});
      check(RegistrationRecovery.straightness(cameras.stream().map(i -> frames.get(i).camera.center()).toList()) < .1,
          "the strip is straight");
      double[] normal = RegistrationRecovery.groundNormal(frames, cameras, tracks, member);
      check(normal != null && Math.abs(normal[2]) > .99 && normal[2] > 0, "ground normal points to the cameras");
      // Level truth (no tilt) so that the ground normal agrees with GPS up.
      var level = new RegistrationRecovery.Similarity(12, rotation(0, 0, -.7), new double[] {300, -200, 140});
      for (int i = 0; i < n; i++) enu[i] = level.apply(frames.get(i).camera.center());
      enu[3] = Math3.add(enu[3], new double[] {40, -25, 0}); // one bad GPS position
      var fit = RegistrationRecovery.toGps(frames, cameras, enu, tracks, member);
      check(fit != null && fit.rms() < 1e-6, "robust GPS fit ignores the bad position: " + (fit == null ? null : fit.rms()));
      double[] probe = {2, 1.5, 0};
      check(Math3.norm(Math3.sub(fit.similarity().apply(probe), level.apply(probe))) < 1e-4,
          "a straight strip gets its roll from the ground normal");
      check(up.length == 3, "up computed");
      // Shared-image fit: the same cameras in another frame.
      var other = new RegistrationRecovery.Similarity(.2, rotation(.4, -.3, .1), new double[] {-1, 2, 3});
      Camera[] model = new Camera[n];
      for (int i = 0; i < n; i++) {
        model[i] = frames.get(i).camera.copy();
        other.transform(model[i]);
      }
      var common = RegistrationRecovery.fromCommon(frames, List.of(2, 5), model);
      check(common != null && common.rms() < 1e-9, "two shared cameras fix the similarity");
      double[] q = {3, -1, 2};
      check(Math3.norm(Math3.sub(common.similarity().apply(other.apply(q)), q)) < 1e-6, "shared-image fit maps back");
      check(RegistrationRecovery.fromCommon(frames, List.of(2), model) == null, "one shared camera is not enough");
      check(Math.abs(RegistrationRecovery.spacing(frames, cameras) - 1) < 1e-9, "median camera spacing");
    } finally {
      for (ImageFrame f : frames) f.close();
    }
  }

  private static void diagnosticsAndRepair(Path root) throws Exception {
    Path image = root.resolve("img.png");
    Imgcodecs.imwrite(image.toString(), new Mat(60, 80, CvType.CV_8UC1, new Scalar(100)));
    Options o = TestSupport.options();
    o.input = root;
    o.output = root.resolve("project");
    Calibration calibration = new Calibration(80, 80, 39.5, 29.5, new double[5], 80, 60);
    List<ImageFrame> frames = new ArrayList<>();
    try {
      for (int i = 0; i < 7; i++) {
        ImageFrame f = new ImageFrame(i, image, calibration, o);
        f.keypoints = new KeyPoint[0];
        frames.add(f);
      }
      // Pairs: 0-1, 1-2, 2-3 (main), 4-5 (separate), 6 none. Registered: 0, 1, 2.
      FeatureTracks features = new FeatureTracks();
      for (int[] e : new int[][] {{0, 1}, {1, 2}, {2, 3}, {4, 5}}) {
        FeatureTracks.Pair p = new FeatureTracks.Pair(e[0], e[1]);
        for (int k = 0; k < 30; k++) p.matches.add(new FeatureTracks.Match(k, k));
        features.pairs.add(p);
      }
      for (int i = 0; i < 3; i++) frames.get(i).camera.registered = true;
      // Image 3 sees 5 triangulated tie points (fewer than the PnP minimum 10).
      for (int k = 0; k < 5; k++) {
        FeatureTracks.Track t = new FeatureTracks.Track();
        t.point = new double[] {k, 0, 5};
        t.observations.add(new FeatureTracks.Observation(2, k));
        t.observations.add(new FeatureTracks.Observation(3, k));
        features.tracks.add(t);
      }
      int[] component = MatchRepair.components(7, features.pairs);
      check(component[0] == 0 && component[3] == 0 && component[4] == 1 && component[5] == 1 && component[6] == 2,
          "components numbered by size: " + Arrays.toString(component));
      RegistrationDiagnostics log = new RegistrationDiagnostics(7);
      log.registered(0, "seed", 0);
      log.registered(1, "seed", 0);
      log.registered(2);
      log.attempt(3, 5, 0, 0);
      log.write(frames, features, o, null);
      Object rows = ProcessingReport.read(o.output.resolve("04_geometry/registration.json"));
      Map<Integer, Map<String, Object>> byId = new HashMap<>();
      for (Object item : (List<?>) rows) {
        var r = ProcessingReport.object(item);
        byId.put(((Number) r.get("image_id")).intValue(), r);
      }
      check("seed".equals(byId.get(0).get("registered_by")), "seed recorded");
      check("incremental".equals(byId.get(2).get("registered_by")), "incremental recorded");
      check("too_few_registered_tie_points".equals(byId.get(3).get("reason")), "image 3: too few tie points");
      check(((Number) byId.get(3).get("registered_tie_points")).intValue() == 5, "tie point count");
      check("separate_match_component".equals(byId.get(4).get("reason")), "image 4: separate component");
      check("no_accepted_pairs".equals(byId.get(6).get("reason")), "image 6: no pairs");
      var summary = ProcessingReport.map(o.output.resolve("04_geometry/registration_summary.json"));
      check(((Number) summary.get("registered")).intValue() == 3, "summary count");
      var reasons = ProcessingReport.object(summary.get("unregistered_reasons"));
      check(((Number) reasons.get("separate_match_component")).intValue() == 2
              && ((Number) reasons.get("no_accepted_pairs")).intValue() == 1
              && ((Number) reasons.get("too_few_registered_tie_points")).intValue() == 1,
          "summary reasons: " + reasons);
      // Report data and GUI text from the files.
      var report = ProcessingReport.registration(o.output, root.resolve("report"));
      check(report != null && ((List<?>) report.get("unregistered_images")).size() == 4, "report lists 4 unregistered images");
      String html = RegistrationWindow.html(o.output);
      check(html.contains("3 of 7 images registered") && html.contains("separate match component")
          && html.contains("no accepted pairs"), "GUI text");
      check(RegistrationWindow.html(root.resolve("missing")).contains("No registration details"), "GUI text without files");
      // Repair plan (no GPS: filename order). Evaluated pairs: the accepted ones plus 3-4.
      o.matchRepairMinPairs = 2;
      o.matchRepairCandidates = 2;
      List<int[]> evaluated = new ArrayList<>(List.of(new int[] {0, 1}, new int[] {1, 2}, new int[] {2, 3}, new int[] {4, 5}, new int[] {3, 4}));
      var plan = MatchRepair.plan(frames, features.pairs, evaluated, o);
      Map<Integer, MatchRepair.Weak> weak = new HashMap<>();
      for (var w : plan.weak()) weak.put(w.image(), w);
      check(weak.keySet().equals(Set.of(0, 3, 4, 5, 6)), "weak images: " + weak.keySet());
      check(weak.get(0).reason().equals("few_accepted_pairs") && weak.get(6).reason().equals("no_accepted_pairs")
          && weak.get(4).reason().equals("outside_largest_component"), "weak reasons");
      check(weak.get(0).candidates().equals(List.of(2, 3)), "nearest untried candidates: " + weak.get(0).candidates());
      check(!weak.get(4).candidates().contains(5) && !weak.get(4).candidates().contains(3), "tried pairs are skipped");
      check(weak.get(6).candidates().size() == 2 && plan.components() == 3 && plan.largestComponent() == 4, "plan counts");
      for (int[] e : plan.edges()) check(e[0] < e[1], "edges ordered");
    } finally {
      for (ImageFrame f : frames) f.close();
    }
  }

  private static void options(Path root) {
    Options d = TestSupport.options();
    check(d.registrationRecovery.equals("off") && d.matchRepair.equals("off"), "both off by default");
    check(d.matchRepairCandidates == 8 && d.matchRepairMinPairs == 2, "repair defaults");
    Options p = Options.parse(new String[] {"-i", root.toString(), "-o", root.resolve("p").toString(),
      "--registration-recovery", "on", "--match-repair", "on", "--match-repair-candidates", "12", "--match-repair-min-pairs", "3"});
    check(p.registrationRecovery.equals("on") && p.matchRepair.equals("on") && p.matchRepairCandidates == 12
        && p.matchRepairMinPairs == 3, "options parse");
    for (String[] bad : new String[][] {{"--registration-recovery", "yes"}, {"--match-repair", "1"},
        {"--match-repair-candidates", "0"}, {"--match-repair-min-pairs", "51"}}) {
      boolean failed = false;
      try {
        Options.parse(new String[] {"-i", root.toString(), "-o", root.resolve("p").toString(), bad[0], bad[1]});
      } catch (IllegalArgumentException e) {
        failed = true;
      }
      check(failed, "rejects " + bad[0] + " " + bad[1]);
    }
    String help = Options.help();
    check(help.contains("--registration-recovery") && help.contains("--match-repair-candidates"), "help lists the options");
    Options a = TestSupport.options(), b = TestSupport.options();
    b.registrationRecovery = "on";
    b.matchRepair = "on";
    check(!ReuseCache.settings(a, "registrationRecovery", "matchRepair").equals(ReuseCache.settings(b, "registrationRecovery", "matchRepair")),
        "cache settings differ");
  }
}
