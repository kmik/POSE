package sgm;

import java.nio.file.*;
import java.util.*;
import java.util.concurrent.*;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;

/** Independent sparse-support scene and rendered ground truth for opt-in view selection. */
final class OptimalMvsgmNeighborsRegression {
  private static int checks;
  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally(); Core.setNumThreads(1);
    Path root = Files.createTempDirectory(Path.of("target"), "optimal-neighbors-");
    System.out.println("PASS optimal MVSGM neighbours=" + run(root));
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Files.createDirectories(root);
    Options o = TestSupport.options();
    o.output = Files.createDirectories(root.resolve("maps"));
    o.compute = "cpu"; o.mvsgmLevels = 0; o.speckleSize = 0;
    o.mvsgmNeighbors = 6; o.mvsgmBestK = 3;
    check(!new Options().mvsgmOptimalNeighbors, "Production default stays score-only");
    Options cli = Options.parse(new String[] {"-i", root.toString(), "--mvsgm-optimal-neighbors"});
    check(cli.mvsgmOptimalNeighbors && cli.copy().mvsgmOptimalNeighbors,
        "Flag and options copy preserve opt-in");
    check(!Options.parse(new String[] {"-i", root.toString(), "--mvsgm-optimal-neighbors",
        "--no-mvsgm-optimal-neighbors"}).mvsgmOptimalNeighbors, "Explicit disable overrides enable");
    List<ImageFrame> frames = new ArrayList<>();
    List<FeatureTracks.Track> tracks = new ArrayList<>();
    float[] far = MultiViewSgmRegression.texture(1), near = MultiViewSgmRegression.texture(2);
    float[] truth = new float[MultiViewSgmRegression.W * MultiViewSgmRegression.H];
    try {
      for (int i = 0; i < 10; i++) {
        double x = i == 0 ? 0 : i <= 6 ? 6 + .02 * (i - 1) : -6 - .02 * (i - 7);
        Camera c = MultiViewSgmRegression.camera(x, 0, i == 8);
        byte[] pixels = MultiViewSgmRegression.render(c, far, near, i == 0 ? truth : null);
        Mat m = new Mat(MultiViewSgmRegression.H, MultiViewSgmRegression.W, CvType.CV_8U);
        Path path = root.resolve("camera" + i + ".png");
        try { m.put(0, 0, pixels); Imgcodecs.imwrite(path.toString(), m); }
        finally { m.release(); }
        ImageFrame f = new ImageFrame(i, path, new Calibration(c.fx, c.fy, c.cx, c.cy,
            new double[5], MultiViewSgmRegression.W, MultiViewSgmRegression.H), o, true);
        f.camera.rotation = c.rotation; f.camera.translation = c.translation;
        f.camera.registered = true;
        frames.add(f);
      }
      for (int side = 0; side < 2; side++) for (int i = 0; i < (side == 0 ? 200 : 194); i++) {
        double u = 100 + i % 8 * 7, v = 35 + i / 8 % 8 * 21;
        if (side == 1) u = 320 - u;
        double x = (u - 160) * 44 / 600, y = (v - 120) * 44 / 600;
        double z = MultiViewSgmRegression.inNear(x, y) ? 44 : 50;
        var t = new FeatureTracks.Track();
        t.point = new double[] {(u - 160) * z / 600, (v - 120) * z / 600, z};
        t.observations.add(new FeatureTracks.Observation(0, i));
        for (int j = side == 0 ? 1 : 7; j <= (side == 0 ? 6 : 9); j++)
          t.observations.add(new FeatureTracks.Observation(j, i));
        tracks.add(t);
      }
      List<MultiViewSgm.Neighbor> old = MultiViewSgm.neighbours(frames, tracks, 0, 6);
      check(old.size() == 6 && old.stream().allMatch(n -> n.id() <= 6),
          "Fixture: raw-score selection favours six redundant baselines");
      MultiViewSgm current = new MultiViewSgm(o);
      check(current.headerPlan(frames, tracks, 0).neighbours.equals(old),
          "Disabled path preserves exact neighbour order, scores and metadata");
      check(current.neighborSelectionReports.isEmpty(), "Disabled path allocates no coverage reports");
      var selected = OptimalMvsgmNeighbors.select(frames, tracks, 0, 6, 3);
      check(selected.neighbors().size() == 6, "Opt-in respects six-neighbour cap");
      check(selected.neighbors().getFirst().equals(old.getFirst()), "Strongest raw-score anchor retained");
      check(selected.neighbors().stream().anyMatch(n -> n.id() >= 7),
          "Complementary baseline replaces a redundant source");
      check(selected.neighbors().stream().map(MultiViewSgm.Neighbor::id).distinct().count() == 6,
          "Selection has no duplicates");
      check(Json.string(selected.report()).equals(Json.string(
          OptimalMvsgmNeighbors.select(frames, tracks, 0, 6, 3).report())), "Selection is repeatable");
      List<?> baselineCoverage = (List<?>) selected.report().get("score_only_coverage");
      List<?> chosenCoverage = (List<?>) selected.report().get("selected_coverage");
      check(((Number) chosenCoverage.getFirst()).intValue() > ((Number) baselineCoverage.getFirst()).intValue(),
          "Selected sources cover more reference-image cells");
      check((int) selected.report().get("selected_direction_sectors")
          > (int) selected.report().get("score_only_direction_sectors"), "Baseline directions are more diverse");
      o.mvsgmOptimalNeighbors = true;
      MultiViewSgm optimal = new MultiViewSgm(o);
      check(optimal.headerPlan(frames, tracks, 0).neighbours.equals(selected.neighbors()),
          "Flag reaches production header planning");
      check(!optimal.neighborSelectionReports.isEmpty(), "Opt-in reports its selection decisions");
      concurrentPlanning(frames, tracks, o);
      quality(frames, tracks, o, truth, root);
      Camera weak = MultiViewSgmRegression.camera(1, 0, false);
      ImageFrame lowScore = new ImageFrame(10, frames.getFirst().path,
          frames.getFirst().calibration, o, true);
      lowScore.camera.rotation = weak.rotation; lowScore.camera.translation = weak.translation;
      lowScore.camera.registered = true; frames.add(lowScore);
      for (int i = 0; i < 12; i++)
        tracks.get(i).observations.add(new FeatureTracks.Observation(10, i));
      check(OptimalMvsgmNeighbors.select(frames, tracks, 0, 8, 3).neighbors().stream()
          .noneMatch(n -> n.id() == 10), "Five-percent score floor prevents weak-source padding");
      frames.get(9).camera.denseExcluded = true;
      frames.get(8).camera.registered = false;
      check(OptimalMvsgmNeighbors.select(frames, tracks, 0, 8, 3).neighbors().stream()
          .noneMatch(n -> n.id() == 8 || n.id() == 9), "Excluded/unregistered cameras stay ineligible");
      check(OptimalMvsgmNeighbors.select(frames, List.of(), 0, 6, 3).neighbors().isEmpty(),
          "No evidence does not fabricate neighbours");
      check(OptimalMvsgmNeighbors.select(frames, tracks.subList(0, 11), 0, 6, 3).neighbors().isEmpty(),
          "Existing twelve-shared-point minimum retained");
      check(OptimalMvsgmNeighbors.select(frames, tracks, 0, 1, 3).neighbors().getFirst()
          .equals(MultiViewSgm.neighbours(frames, tracks, 0, 1).getFirst()), "One-source selection retains anchor");
      o.mvsgmOptimalNeighbors = false;
      check(new MultiViewSgm(o).headerPlan(frames, tracks, 0).neighbours.equals(
          MultiViewSgm.neighbours(frames, tracks, 0, 6)), "Disabling after use restores legacy selection");
    } finally { for (ImageFrame f : frames) f.close(); GpuSgm.closeSession(); }
    return checks;
  }

  private static void concurrentPlanning(List<ImageFrame> frames,
      List<FeatureTracks.Track> tracks, Options o) throws Exception {
    MultiViewSgm mv = new MultiViewSgm(o);
    ExecutorService workers = Executors.newFixedThreadPool(4);
    try {
      List<Future<?>> work = new ArrayList<>();
      for (int c = 0; c < frames.size(); c++) {
        int camera = c;
        work.add(workers.submit(() -> mv.headerPlan(frames, tracks, camera)));
      }
      for (Future<?> task : work) task.get();
      check(mv.neighborSelectionReports.size() == frames.size(),
          "Parallel planning retains every camera's selection diagnostics");
    } finally { workers.shutdownNow(); }
  }

  private static void quality(List<ImageFrame> frames, List<FeatureTracks.Track> tracks,
      Options o, float[] truth, Path root) throws Exception {
    List<Object> results = new ArrayList<>();
    double[] completeness = new double[2], rmse = new double[2];
    for (int mode = 0; mode < 2; mode++) {
      o.mvsgmOptimalNeighbors = mode == 1;
      MultiViewSgm mv = new MultiViewSgm(o);
      var plan = mv.plan(frames, tracks, 0);
      var solved = mv.solve(plan, new double[0]);
      if (mode == 1) {
        Options gpuOptions = o.copy();
        gpuOptions.compute = "auto";
        MultiViewSgm gpu = new MultiViewSgm(gpuOptions);
        if (GpuMultiViewSgm.use(gpuOptions, gpu, plan, solved.bands()))
          try (var session = new GpuMultiViewSgm.Session(gpuOptions, gpu, plan, 0)) {
            check(Arrays.equals(session.summary(solved.bands()), solved.summary()),
                "GPU and CPU summaries agree for the six complementary sources");
          }
        else System.out.println("Optimal neighbour GPU check skipped: no suitable device");
      }
      float[] depth = mv.select(plan, solved.summary()).depth();
      int evaluated = 0, valid = 0, accurate = 0; double squared = 0;
      for (int y = 8; y < MultiViewSgmRegression.H - 8; y++)
        for (int x = 8; x < MultiViewSgmRegression.W - 8; x++) {
          if (MultiViewSgmRegression.nearEdge(truth, x, y)) continue;
          int at = y * MultiViewSgmRegression.W + x; evaluated++;
          if (depth[at] <= 0) continue;
          valid++;
          double error = (depth[at] - truth[at]) / truth[at];
          squared += error * error;
          if (Math.abs(error) <= .01) accurate++;
        }
      completeness[mode] = accurate / (double) evaluated;
      rmse[mode] = Math.sqrt(squared / Math.max(1, valid));
      results.add(Json.object("optimal", mode == 1, "evaluated", evaluated, "valid", valid,
          "accurate_within_one_percent", accurate, "accurate_completeness", completeness[mode],
          "relative_rmse", rmse[mode], "sources", plan.neighbours.stream().map(n -> n.id()).toList()));
    }
    Json.write(root.resolve("selection_quality.json"), results);
    System.out.println("Optimal neighbour quality: " + Json.string(results));
    check(completeness[1] >= completeness[0], "Independent rendered scene: accurate completeness does not decrease");
    check(rmse[1] <= Math.max(.01, rmse[0] * 1.1), "Independent rendered scene: depth error stays bounded");
    o.mvsgmOptimalNeighbors = true;
  }
}
