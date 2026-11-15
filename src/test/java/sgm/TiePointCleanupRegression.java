package sgm;

import java.nio.file.*;
import java.util.*;

/**
 * Tie-point cleanup (--tie-point-cleanup, step 'cleanup'): the selection rules on synthetic
 * points, the files the step and the POSE window share, the options, and the step on a real
 * project (forest_tiny): off skips it, pause waits, a choice copied by --branch is ignored, the
 * window's choice gives the same result as auto, skip and off continue without cleanup.
 */
final class TiePointCleanupRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Path root = Files.createTempDirectory(Path.of("target"), "tie-points-");
    System.out.println("PASS tie-point cleanup: " + run(root) + "; " + root);
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Files.createDirectories(root);
    selection();
    files(root.resolve("files ä"));
    options(root);
    realProject(root.resolve("real"));
    return checks;
  }

  static TiePointCleanup.Point point(int track, double error, int views, double angle, int... cameras) {
    return new TiePointCleanup.Point(track, (float) error, (float) (error * .7), views, (float) angle, cameras);
  }

  /** Limits, worst-first order, the share limit and the cameras' minimum support. */
  static void selection() {
    List<TiePointCleanup.Point> pts = new ArrayList<>();
    // 100 good points over cameras 0..3 (each camera well supported).
    for (int i = 0; i < 100; i++) pts.add(point(i, .3, 3, 10, i % 4, (i + 1) % 4, (i + 2) % 4));
    pts.add(point(100, 5.0, 3, 10, 0, 1, 2)); // error only
    pts.add(point(101, 2.0, 2, 0.2, 0, 1)); // error and angle and views (with min views 3)
    pts.add(point(102, .3, 2, 10, 1, 2)); // views only (with min views 3)
    pts.add(point(103, .3, 3, 0.5, 2, 3, 0)); // angle only
    TiePointCleanup.Thresholds none = new TiePointCleanup.Thresholds(0, 2, 0, 100);
    TiePointCleanup.Selection s0 = TiePointCleanup.select(pts, 4, none);
    check(s0.removed() == 0 && s0.byError() == 0, "no limits: nothing removed");
    TiePointCleanup.Thresholds all = new TiePointCleanup.Thresholds(1.0, 3, 1.0, 100);
    TiePointCleanup.Selection s = TiePointCleanup.select(pts, 4, all);
    check(s.byError() == 2 && s.byViews() == 2 && s.byAngle() == 2, "failing counts per limit: " + s);
    check(s.removed() == 4 && s.remove()[100] && s.remove()[101] && s.remove()[102] && s.remove()[103] && !s.remove()[0], "all four bad points removed");
    // Share limit: one point may go; the worst (most limits failed) is chosen.
    TiePointCleanup.Selection one = TiePointCleanup.select(pts, 4, new TiePointCleanup.Thresholds(1.0, 3, 1.0, 1.0));
    check(one.removed() == 1 && one.remove()[101] && one.capped() == 3, "worst first under the share limit: " + one);
    // Two points failing only the error: the larger error goes first.
    List<TiePointCleanup.Point> two = new ArrayList<>(pts.subList(0, 100));
    two.add(point(200, 2.0, 3, 10, 0, 1, 2));
    two.add(point(201, 9.0, 3, 10, 0, 1, 2));
    TiePointCleanup.Selection big = TiePointCleanup.select(two, 4, new TiePointCleanup.Thresholds(1.0, 2, 0, 1.0));
    check(big.removed() == 1 && big.remove()[101] && !big.remove()[100], "the larger error goes first");
    // Camera support: a camera with few points keeps at least MIN_SUPPORT.
    List<TiePointCleanup.Point> thin = new ArrayList<>();
    for (int i = 0; i < 20; i++) thin.add(point(i, 3.0, 2, 10, 0, 1));
    TiePointCleanup.Selection kept = TiePointCleanup.select(thin, 2, new TiePointCleanup.Thresholds(1.0, 2, 0, 100));
    check(kept.removed() == 20 - TiePointCleanup.MIN_SUPPORT && kept.protectedBySupport() == TiePointCleanup.MIN_SUPPORT,
        "each camera keeps " + TiePointCleanup.MIN_SUPPORT + " points: " + kept);
    check(TiePointCleanup.weakestSupport(thin, kept.remove(), 2) == TiePointCleanup.MIN_SUPPORT, "weakest camera support");
    // Deterministic, and RMS without the removed points is lower.
    check(Arrays.equals(TiePointCleanup.select(pts, 4, all).remove(), s.remove()), "repeatable");
    check(TiePointCleanup.rms(pts, s.remove()) < TiePointCleanup.rms(pts, null), "RMS drops without the removed points");
    // Automatic largest error: three times the median, at least 0.5 px.
    check(Math.abs(TiePointCleanup.autoMaxError(pts) - .9) < 1e-9, "auto largest error 3x median: " + TiePointCleanup.autoMaxError(pts));
    check(TiePointCleanup.autoMaxError(List.of(point(0, .05, 2, 5, 0, 1))) == .5, "auto largest error at least 0.5 px");
  }

  /** The table and the choice: written by one side, read by the other. */
  static void files(Path dir) throws Exception {
    Path run = Files.createDirectories(dir.resolve("run"));
    List<TiePointCleanup.Point> pts = List.of(point(7, 1.23456, 3, 4.5678, 0, 2, 5), point(9, .5, 2, 12, 1, 3));
    Path csv = run.resolve(TiePointCleanup.FOLDER).resolve("metrics.csv");
    TiePointCleanup.write(csv, pts);
    List<TiePointCleanup.Point> back = TiePointCleanup.read(csv);
    check(back.size() == 2 && back.get(0).track() == 7 && Math.abs(back.get(0).error() - 1.2346) < 1e-4
        && Arrays.equals(back.get(0).cameras(), new int[] {0, 2, 5}) && back.get(1).views() == 2, "metrics table round trip");
    check(TiePointCleanup.readChoice(run) == null, "no choice yet");
    TiePointCleanup.writeChoice(run, new TiePointCleanup.Choice(true, new TiePointCleanup.Thresholds(.8, 3, 1.5, 25)));
    TiePointCleanup.Choice c = TiePointCleanup.readChoice(run);
    check(c.apply() && c.thresholds().equals(new TiePointCleanup.Thresholds(.8, 3, 1.5, 25)), "apply choice round trip: " + c);
    TiePointCleanup.writeChoice(run, new TiePointCleanup.Choice(false, null));
    check(!TiePointCleanup.readChoice(run).apply(), "skip choice round trip");
    // Positions (for the window's 3D view) round trip; a table without them still reads.
    List<TiePointCleanup.Point> placed = List.of(new TiePointCleanup.Point(3, .5f, .4f, 2, 6f, new int[] {0, 1}, 1.25, -2.5, 10.125));
    TiePointCleanup.write(csv, placed);
    TiePointCleanup.Point pb = TiePointCleanup.read(csv).get(0);
    check(pb.x() == 1.25 && pb.y() == -2.5 && pb.z() == 10.125 && Arrays.equals(pb.cameras(), new int[] {0, 1}), "positions round trip");
    Files.writeString(csv, "track,max_error_px,rms_error_px,views,max_angle_deg,images\n4,0.5,0.4,2,6.0,0;1\n");
    TiePointCleanup.Point old = TiePointCleanup.read(csv).get(0);
    check(old.track() == 4 && Double.isNaN(old.x()) && Arrays.equals(old.cameras(), new int[] {0, 1}), "a table without positions still reads");
    // The 3D view data: an alignment frame camera looks along +z (R = I); turned so it looks down.
    List<TiePointCleanup.Camera> cams = List.of(new TiePointCleanup.Camera("a.jpg", new double[] {0, 0, 0}, new double[] {1, 0, 0, 0, 1, 0, 0, 0, 1}, 1000, 400, 300),
        new TiePointCleanup.Camera("b.jpg", new double[] {2, 0, 0}, new double[] {1, 0, 0, 0, 1, 0, 0, 0, 1}, 1000, 400, 300));
    List<TiePointCleanup.Point> scene = List.of(new TiePointCleanup.Point(0, .2f, .1f, 2, 5f, new int[] {0, 1}, 1, 0, 10),
        new TiePointCleanup.Point(1, 3f, 2f, 2, 5f, new int[] {0, 1}, 1, 1, 12), point(2, .2, 2, 5, 0, 1));
    CloudData view = TiePointCleanup.cloud(scene, cams);
    check(view.sx.length == 3 && view.cameras.size() == 2 && view.cameras.get(0).center()[2] > view.sz[0] + 9, "cameras above their points after turning: "
        + view.cameras.get(0).center()[2] + " vs " + view.sz[0]);
    check(view.sz[1] < view.sz[0], "the farther point is lower");
    double[] apex = CloudRenderer.frustum(view.cameras.get(0), 1)[0], base = CloudRenderer.frustum(view.cameras.get(0), 1)[1];
    check(base[2] < apex[2] - .9, "turned cameras look down");
    TiePointCleanup.colour(view, scene, new boolean[] {false, true, false}, true);
    check(view.scolour[1] == TiePointCleanup.REMOVED_COLOUR && view.scolour[0] == TiePointCleanup.KEPT_COLOUR && view.sbig[1] && !view.shidden[0] && view.shidden[2],
        "removed red and larger, kept grey, no position hidden");
    TiePointCleanup.colour(view, scene, new boolean[] {false, true, false}, false);
    check(view.shidden[0] && !view.shidden[1], "kept points can be hidden");
    CloudRenderer.Frame frame = CloudRenderer.render(new CloudRenderer.Request(view, CloudRenderer.home(view), 120, 90, CloudRenderer.Colour.RGB, -1, 2, false,
        false, true, 1, 0, 1));
    boolean red = false;
    for (int px : frame.argb()) red |= (px & 0xffffff) == TiePointCleanup.REMOVED_COLOUR;
    check(red, "the removed point is drawn red");
    // The window starts from the run's own --tie-point-* values.
    Map<String, String> given = TiePointWindow.given(List.of("-i", "x", "--tie-point-max-error", "0.7", "--tie-point-min-views=3", "--quality", "high"));
    check(given.equals(Map.of("--tie-point-max-error", "0.7", "--tie-point-min-views", "3")), "window reads the run's limits: " + given);
  }

  static void options(Path root) throws Exception {
    Path images = Files.createDirectories(root.resolve("kuvat"));
    String i = images.toString();
    Options d = Options.parse(new String[] {"-i", i});
    check(d.tiePointCleanup.equals("off") && d.tiePointMaxError == 0 && d.tiePointMinViews == 2 && d.tiePointMinAngle == 1
        && d.tiePointMaxRemoved == 20, "defaults: off, auto error, 2 views, 1 degree, 20%");
    Options o = Options.parse(new String[] {"-i", i, "--tie-point-cleanup", "PAUSE", "--tie-point-max-error", "0.6",
        "--tie-point-min-views", "3", "--tie-point-min-angle", "2", "--tie-point-max-removed", "30"});
    check(o.tiePointCleanup.equals("pause") && o.tiePointMaxError == .6 && o.tiePointMinViews == 3 && o.tiePointMinAngle == 2
        && o.tiePointMaxRemoved == 30 && o.copy().tiePointCleanup.equals("pause"), "options parsed and copied");
    check(Options.parse(new String[] {"-i", i, "--tie-point-max-error", "auto"}).tiePointMaxError == 0, "max error auto");
    for (String[] bad : List.of(new String[] {"--tie-point-cleanup", "maybe"}, new String[] {"--tie-point-min-views", "1"},
        new String[] {"--tie-point-min-angle", "90"}, new String[] {"--tie-point-max-removed", "0"}, new String[] {"--tie-point-max-error", "-1"})) {
      boolean refused = false;
      try {
        Options.parse(new String[] {"-i", i, bad[0], bad[1]});
      } catch (IllegalArgumentException e) {
        refused = true;
      }
      check(refused, "refused: " + String.join(" ", bad));
    }
    check(Pipeline.notRequested(Pipeline.Step.CLEANUP, d) && !Pipeline.notRequested(Pipeline.Step.CLEANUP, o), "optional step: asked for unless off");
    // custom: user-defined limits; auto: the automatic ones only (limits with auto are refused).
    check(!d.tiePointLimitsGiven && o.tiePointLimitsGiven, "limits given are noted");
    Options custom = Options.parse(new String[] {"-i", i, "--tie-point-cleanup", "custom", "--tie-point-max-error", "0.5", "--tie-point-max-removed", "10"});
    check(custom.tiePointCleanup.equals("custom") && custom.tiePointMaxError == .5 && custom.tiePointMaxRemoved == 10 && custom.tiePointMinViews == 2,
        "custom with some limits (the rest at their defaults)");
    check(Options.parse(new String[] {"-i", i, "--tie-point-cleanup", "custom"}).tiePointCleanup.equals("custom"), "custom without limits: the defaults");
    check(!Pipeline.notRequested(Pipeline.Step.CLEANUP, custom), "custom asks for the step");
    boolean refused = false;
    try {
      Options.parse(new String[] {"-i", i, "--tie-point-cleanup", "auto", "--tie-point-min-angle", "2"});
    } catch (IllegalArgumentException e) {
      refused = e.getMessage().contains("--tie-point-cleanup custom");
    }
    check(refused, "auto with a limit is refused with a hint to use custom");
    check(Options.help().contains("--tie-point-cleanup off|auto|custom|pause"), "help lists custom");
    // The window: the limit fields go into the command with custom and pause only.
    Map<String, String> v = new LinkedHashMap<>(Map.of("-i", i, "-o", root.resolve("p").toString(), "--tie-point-max-error", "0.6", "--tie-point-min-views", "3"));
    for (String mode : List.of("", "off", "auto", "custom", "pause")) {
      v.put("--tie-point-cleanup", mode);
      List<String> args = GuiOptions.arguments(v, "");
      boolean has = args.contains("--tie-point-max-error") && args.contains("--tie-point-min-views");
      check(has == (mode.equals("custom") || mode.equals("pause")), "limit fields with mode '" + mode + "': " + args);
      check(GuiOptions.validate(args) == null, "the window's command parses with mode '" + mode + "': " + GuiOptions.validate(args));
    }
    for (String flag : GuiOptions.TIE_POINT_LIMITS) check(GuiOptions.FIELDS.stream().anyMatch(f -> f.flag().equals(flag)), "a New run field for " + flag);
    check(TiePointCleanup.automatic(List.of(point(0, .2, 2, 5, 0, 1), point(1, .4, 2, 5, 0, 1), point(2, .3, 2, 5, 0, 1)))
        .equals(new TiePointCleanup.Thresholds(.9, 2, 1, 20)), "automatic limits");
    check(GuiOptions.field("--tie-point-cleanup") != null && GuiOptions.field("--tie-point-max-error") != null, "fields in the window");
    check(Options.help().contains("--tie-point-cleanup off|auto|custom|pause") && Options.help().contains("align, cleanup"), "help text");
  }

  static String status(Path project, String run, Pipeline.Step step) throws Exception {
    return PipelineState.load(project.resolve("runs").resolve(run)).status(step);
  }

  static Map<String, Object> report(Path project, String run) throws Exception {
    return JsonRead.object(JsonRead.read(project.resolve("runs").resolve(run).resolve(TiePointCleanup.FOLDER).resolve("cleanup.json")));
  }

  static void realProject(Path root) throws Exception {
    Path images = Path.of("..", "example_data", "forest", "forest_tiny");
    if (!Files.isDirectory(images)) {
      System.out.println("TiePointCleanupRegression real project skipped: no " + images);
      return;
    }
    Path p = root.resolve("p");
    String o = p.toString();
    check(Main.run(new String[] {"inputs", "-i", images.toString(), "-o", o, "--quality", "low", "--cores", "4"}) == 0, "inputs");
    check(Main.run(new String[] {"calibrate", "match", "align", "-o", o}) == 0, "steps up to align");
    String a = Main.latestRun(p);
    // off (default): a later step skips cleanup.
    check(Main.run(new String[] {"georeference", "-o", o, "--run", a, "--branch"}) == 0, "georeference without cleanup");
    String off = Main.latestRun(p);
    check(status(p, off, Pipeline.Step.CLEANUP).equals("skipped") && status(p, off, Pipeline.Step.GEOREFERENCE).equals("complete"), "off: cleanup skipped");

    // pause: the step writes the table and waits.
    check(Main.run(new String[] {"cleanup", "-o", o, "--run", a, "--branch", "--tie-point-cleanup", "pause"}) == 0, "pause run");
    String w = Main.latestRun(p);
    Path wr = p.resolve("runs").resolve(w);
    check(status(p, w, Pipeline.Step.CLEANUP).equals("waiting"), "pause: waiting");
    check(ProjectFiles.run(p, w).waitingForCleanup() && !ProjectFiles.run(p, w).waitingForGcp(), "the window sees the run waiting for cleanup");
    List<TiePointCleanup.Point> pts = TiePointCleanup.read(wr.resolve(TiePointCleanup.FOLDER).resolve("metrics.csv"));
    check(pts.size() > 100, "tie points measured: " + pts.size());
    check(pts.stream().allMatch(q -> Double.isFinite(q.x()) && Double.isFinite(q.z())), "every tie point has a position");
    List<TiePointCleanup.Camera> runCams = TiePointCleanup.readCameras(wr.resolve(TiePointCleanup.FOLDER).resolve("cameras.csv"));
    check(runCams.size() >= 2 && runCams.stream().allMatch(c -> c.fx() > 0 && c.width() > 0 && c.name().endsWith(".JPG") || c.name().toLowerCase(Locale.ROOT).endsWith(".jpg")),
        "the alignment's cameras are written: " + runCams.size());
    CloudData shown = TiePointCleanup.cloud(pts, runCams);
    double camZ = shown.cameras.stream().mapToDouble(c -> c.center()[2]).average().orElse(0);
    float[] zs = shown.sz.clone();
    Arrays.sort(zs);
    check(camZ > zs[zs.length / 2], "real block turned: cameras above the tie points");
    check(Main.run(new String[] {"cleanup", "-o", o, "--run", w}) == 0 && status(p, w, Pipeline.Step.CLEANUP).equals("waiting"),
        "run again without a choice: still waiting");
    // A choice in the run, then a branch at cleanup: the copied choice is ignored, the new run waits.
    TiePointCleanup.Thresholds limits = new TiePointCleanup.Thresholds(TiePointCleanup.autoMaxError(pts), 2, 1, 20);
    TiePointCleanup.writeChoice(wr, new TiePointCleanup.Choice(true, limits));
    check(Main.run(new String[] {"cleanup", "-o", o, "--run", w, "--branch"}) == 0, "branch of a run with a choice");
    String copy = Main.latestRun(p);
    check(status(p, copy, Pipeline.Step.CLEANUP).equals("waiting")
        && !Files.exists(TiePointCleanup.choiceFile(p.resolve("runs").resolve(copy))), "a copied choice is ignored");
    // The window's choice applies: same as auto with the same limits.
    check(Main.run(new String[] {"cleanup", "georeference", "-o", o, "--run", w}) == 0, "continue with the choice");
    check(status(p, w, Pipeline.Step.CLEANUP).equals("complete") && status(p, w, Pipeline.Step.GEOREFERENCE).equals("complete"), "cleanup applied, georeference done");
    Map<String, Object> chosen = report(p, w);
    int removed = ((Number) chosen.get("removed")).intValue();
    double before = ((Number) chosen.get("reprojection_rms_before_pixels")).doubleValue(), after = ((Number) chosen.get("reprojection_rms_after_pixels")).doubleValue();
    check(removed > 0 && removed <= pts.size() / 5 + 1, "removed within the 20% share: " + removed + " of " + pts.size());
    check(after < before, "reprojection RMS drops: " + before + " -> " + after);
    check(((Number) chosen.get("registered_cameras_after")).intValue() == ((Number) chosen.get("registered_cameras_before")).intValue(), "no camera lost");
    check(Main.run(new String[] {"cleanup", "georeference", "-o", o, "--run", a, "--branch", "--tie-point-cleanup", "auto"}) == 0, "auto run");
    String au = Main.latestRun(p);
    Map<String, Object> auto = report(p, au);
    check(((Number) auto.get("removed")).intValue() == removed && ((Number) auto.get("reprojection_rms_after_pixels")).doubleValue() == after,
        "auto and the window's choice with the same limits give the same result");
    check(Files.readString(p.resolve("runs").resolve(au).resolve("sparse.csv"))
        .equals(Files.readString(wr.resolve("sparse.csv"))), "same sparse points after georeference");
    // custom: the given limits, the rest at their defaults; auto with a limit is refused.
    check(Main.run(new String[] {"cleanup", "-o", o, "--run", a, "--branch", "--tie-point-cleanup", "custom", "--tie-point-max-error", "0.3",
        "--tie-point-max-removed", "10"}) == 0, "custom run");
    String cu = Main.latestRun(p);
    Map<String, Object> customReport = report(p, cu);
    Map<String, Object> customLimits = JsonRead.object(customReport.get("limits"));
    check(((Number) customLimits.get("max_error_pixels")).doubleValue() == .3 && ((Number) customLimits.get("max_removed_percent")).doubleValue() == 10
        && ((Number) customLimits.get("min_views")).intValue() == 2 && ((Number) customLimits.get("min_angle_degrees")).doubleValue() == 1, "custom limits used: " + customLimits);
    int customRemoved = ((Number) customReport.get("removed")).intValue();
    check(customRemoved > 0 && customRemoved <= pts.size() / 10, "custom: at most 10% removed: " + customRemoved);
    check(Main.run(new String[] {"cleanup", "-o", o, "--run", a, "--branch", "--tie-point-cleanup", "auto", "--tie-point-max-error", "0.3"}) != 0,
        "auto with a limit is refused");
    // Skip in the window, and off on the continuation: no cleanup, the run goes on.
    TiePointCleanup.writeChoice(p.resolve("runs").resolve(copy), new TiePointCleanup.Choice(false, null));
    check(Main.run(new String[] {"cleanup", "-o", o, "--run", copy}) == 0 && status(p, copy, Pipeline.Step.CLEANUP).equals("skipped")
        && "skipped by the user".equals(report(p, copy).get("action")), "skip choice: cleanup skipped");
    check(Main.run(new String[] {"cleanup", "-o", o, "--run", a, "--branch", "--tie-point-cleanup", "pause"}) == 0, "another pause run");
    String w2 = Main.latestRun(p);
    check(Main.run(new String[] {"cleanup", "-o", o, "--run", w2, "--tie-point-cleanup", "off"}) == 0
        && status(p, w2, Pipeline.Step.CLEANUP).equals("skipped"), "off on the continuation skips the waiting step");
  }
}
