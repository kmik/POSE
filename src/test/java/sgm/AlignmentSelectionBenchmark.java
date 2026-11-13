package sgm;

import java.nio.file.*;
import java.util.*;
import org.opencv.core.Core;

/** Photographic selector overhead plus complete descriptor matching/track construction on warm features. */
final class AlignmentSelectionBenchmark {
  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally(); Core.setNumThreads(12);
    Path root = Path.of(args[1]);
    Files.createDirectories(root);
    Options o = new Options();
    o.input = Path.of(args[0]); o.output = root;
    o.features = 40000; o.ratio = .85; o.neighbors = 5; o.cores = 12;
    o.maxDimension = Integer.parseInt(args[2]);
    o.descriptorMatcher = "flann"; o.compute = "cpu";
    o.descriptorMemoryMb = 2048; o.memoryMb = 16000;
    o.cacheDirectory = root.resolve("feature-cache");
    o.resume = true;
    List<Path> paths = Main.images(o);
    List<ImageFrame> frames = new ArrayList<>();
    List<Object> rows = new ArrayList<>();
    long start = System.nanoTime();
    try {
      for (int i = 0; i < paths.size(); i++) {
        int[] wh = ResolutionPlan.originalSize(paths.get(i));
        ImageFrame f = new ImageFrame(i, paths.get(i),
            new Calibration(wh[0] * 1.2, wh[0] * 1.2, (wh[0] - 1) / 2.,
                (wh[1] - 1) / 2., new double[5], wh[0], wh[1]), o, true);
        frames.add(f);
        ReuseCache.features(f, o);
        DescriptorCache.retain(f);
        System.out.println("Benchmark features " + (i + 1) + "/" + paths.size() + ": " + f.keypoints.length);
      }
      double featureSeconds = (System.nanoTime() - start) / 1e9;
      long totalFeatures = frames.stream().mapToLong(f -> f.keypoints.length).sum();
      // Counterbalanced order separates search changes from warm-up and cache effects.
      for (int iteration = 0; iteration < 4; iteration++) {
        o.pairSelection = iteration == 0 || iteration == 3 ? "sequential" : "overlap";
        o.output = Files.createDirectories(root.resolve("trial" + iteration));
        PairMatching.clear(); Core.setRNGSeed(1);
        start = System.nanoTime();
        List<int[]> selected = PairSelection.select(frames, o, null);
        double selectionSeconds = (System.nanoTime() - start) / 1e9;
        Core.setRNGSeed(1);
        start = System.nanoTime();
        FeatureTracks features = new FeatureTracks();
        features.build(frames, o, null);
        double matchSeconds = (System.nanoTime() - start) / 1e9;
        Object row = Json.object("mode", o.pairSelection, "iteration", iteration,
            "selection_seconds", selectionSeconds, "matching_and_tracks_seconds", matchSeconds,
            "selected_pairs", selected.size(), "accepted_pairs", features.pairs.size(),
            "tracks", features.tracks.size());
        rows.add(row);
        Json.write(root.resolve("timings.json"), Json.object("images", paths.size(),
            "features", totalFeatures, "max_dimension", o.maxDimension, "feature_seconds", featureSeconds,
            "scope", "Standalone selection and complete feature-cache loading, descriptor matching, verification and track building; no calibration/bundle/dense stages",
            "trials", rows));
        System.out.println("ALIGNMENT BENCHMARK " + Json.string(row));
        Geometry.releaseWorkspaces();
      }
    } finally { for (ImageFrame f : frames) f.close(); PairMatching.clear(); GpuSgm.closeSession(); }
  }
}
