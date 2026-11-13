package sgm;

import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;

/** Overlap ranking, filename independence, retrieval determinism and bounded pair counts. */
final class PairSelectionRegression {
  private static int checks;
  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }
  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally(); Core.setNumThreads(1);
    Path root = Files.createTempDirectory(Path.of("target"), "pair-selection-");
    if (args.length > 0 && args[0].equals("scale")) scale(root);
    else System.out.println("PASS pair selection=" + run(root));
  }
  static int run(Path root) throws Exception {
    checks = 0;
    Files.createDirectories(root);
    check(new Options().pairSelection.equals("overlap"), "Production default selects overlap");
    check(Options.parse(new String[] {"-i", root.toString(), "--neighbors", "5"}).pairSelection.equals("overlap"),
        "CLI neighbors retains overlap default");
    Options o = new Options(); o.output = Files.createDirectories(root.resolve("pairs"));
    o.neighbors = 1; o.pairCandidates = 2; o.memoryMb = 512; o.descriptorMemoryMb = 1;
    Path image = image(root);
    for (boolean binary : new boolean[] {false, true}) {
      List<ImageFrame> frames = synthetic(image, o, 20, binary, 256);
      try {
        for (ImageFrame f : frames) DescriptorCache.retain(f);
        if (!binary) check(frames.stream().anyMatch(f -> f.descriptors.empty()), "Spill fixture evicts descriptors");
        List<int[]> pairs = PairSelection.select(frames, o, null);
        for (int a = 0; a < 10; a++) check(has(pairs, a, a + 10), "Retrieves nonadjacent matching image " + a);
        check(!has(pairs, 0, 1), "No arbitrary filename edge");
        String first = Files.readString(o.output.resolve("03_matches/pair_selection.json"));
        check(first.contains("indexed_descriptor_preview"), "Large shortlist uses global index");
        PairSelection.select(frames, o, null);
        check(first.equals(Files.readString(o.output.resolve("03_matches/pair_selection.json"))),
            "Repeated indexed selection is deterministic, including scores/ranks");
        check(pairs.size() == 10, "Only supported top choices, not search breadth, are matched");
        o.pairCandidates = 100;
        check(Json.string(pairs).equals(Json.string(PairSelection.select(frames, o, null))),
            "Exhaustive retrieval agrees on strongest neighbours");
        o.pairCandidates = 2;
        o.pairSelection = "gps";
        check(Json.string(pairs).equals(Json.string(PairSelection.select(frames, o, null))),
            "Missing GPS uses visual evidence");
        o.pairSelection = "sequential";
        List<int[]> sequential = PairSelection.select(frames, o, null);
        check(sequential.size() == 19 && has(sequential, 0, 1) && !has(sequential, 0, 10),
            "Explicit sequential retains old next-N behavior");
        o.pairSelection = "all";
        check(PairSelection.select(frames, o, null).size() == 190, "Explicit all retains every pair");
        o.pairSelection = "overlap";
      } finally { for (ImageFrame f : frames) f.close(); }
    }
    geometry(root, image, o);
    visualTopFive(image, o);
    previews(root);
    cacheKeys(root);
    o.neighbors = 5;
    check(PairSelection.maximumPairs(3013, o) == 15065, "Planner bounds overlap pairs linearly");
    o.pairSelection = "all";
    check(PairSelection.maximumPairs(3013, o) == 4537578, "All-pairs bound remains explicit");
    return checks;
  }
  private static void visualTopFive(Path image, Options o) throws Exception {
    o.neighbors = 5;
    List<ImageFrame> frames = synthetic(image, o, 7, false, 64);
    try {
      for (int i = 1; i < frames.size(); i++) {
        Mat target = frames.get(i).descriptors;
        float[] data = new float[(int) target.total()];
        Random r = new Random(10000 + i);
        for (int j = 0; j < data.length; j++) data[j] = r.nextFloat() * 255;
        target.put(0, 0, data);
        Mat from = frames.getFirst().descriptors.rowRange(0, i * 8);
        Mat to = target.rowRange(0, i * 8);
        try { from.copyTo(to); } finally { from.release(); to.release(); }
      }
      PairSelection.select(frames, o, null);
      Map<?, ?> report = JsonRead.object(JsonRead.read(o.output.resolve("03_matches/pair_selection.json")));
      List<Integer> ids = new ArrayList<>();
      for (Object row : (List<?>) report.get("preview_scores")) {
        Map<?, ?> c = (Map<?, ?>) row;
        if (((Number) c.get("a")).intValue() == 0) ids.add(((Number) c.get("b")).intValue());
      }
      check(ids.equals(List.of(6, 5, 4, 3, 2)), "Visual mode selects five greatest support/coverage scores");
    } finally { for (ImageFrame f : frames) f.close(); }
    o.neighbors = 1;
    String fusionMode = o.fusionMode;
    o.fusionMode = "image";
    long overlapFusion = BlockProcessing.estimateFusion(o, List.of(image), 8);
    o.pairSelection = "sequential";
    check(overlapFusion > BlockProcessing.estimateFusion(o, List.of(image), 8),
        "Block forecast allows incoming degree greater than twice K");
    o.pairSelection = "overlap";
    o.fusionMode = fusionMode;
  }
  private static void geometry(Path root, Path image, Options o) throws Exception {
    List<ImageFrame> frames = synthetic(image, o, 8, false, 32);
    try {
      double[] x = {0, 7, 6, 5, 4, 3, 2, 1};
      for (int i = 0; i < frames.size(); i++) {
        Camera c = OverlapRegression.down(x[i], 0);
        frames.get(i).camera.rotation = c.rotation;
        frames.get(i).camera.translation = c.translation;
        frames.get(i).camera.registered = true;
      }
      o.overlapGroundZ = 0; o.neighbors = 5;
      PairSelection.select(frames, o, null);
      Map<?, ?> report = JsonRead.object(JsonRead.read(o.output.resolve("03_matches/pair_selection.json")));
      List<?> chosen = (List<?>) report.get("preview_scores");
      List<Integer> first = new ArrayList<>();
      for (Object row : chosen) {
        Map<?, ?> c = (Map<?, ?>) row;
        if (((Number) c.get("a")).intValue() == 0) first.add(((Number) c.get("b")).intValue());
      }
      check(first.equals(List.of(7, 6, 5, 4, 3)), "Exactly five greatest footprint overlaps, irrespective of filenames");
      check(report.get("search").equals("ground_footprints"), "Known geometry reports physical footprint ranking");
      Camera near = OverlapRegression.down(0, .6);
      frames.get(1).camera.rotation = near.rotation;
      frames.get(1).camera.translation = near.translation;
      o.neighbors = 1; o.flightHeading = 90; o.minSideOverlap = 95;
      PairSelection.select(frames, o, null);
      report = JsonRead.object(JsonRead.read(o.output.resolve("03_matches/pair_selection.json")));
      Map<?, ?> top = (Map<?, ?>) ((List<?>) report.get("preview_scores")).getFirst();
      check(((Number) top.get("a")).intValue() == 0 && ((Number) top.get("b")).intValue() == 7,
          "Filter ineligible higher-overlap candidate before choosing next eligible neighbour");
      o.minSideOverlap = 0; o.flightHeading = Double.NaN;
      o.neighbors = 1; o.minPairOverlap = 95;
      check(PairSelection.select(frames, o, null).isEmpty(), "Geometric threshold excludes unsupported choices");
      o.minPairOverlap = 0; o.overlapGroundZ = Double.NaN;
    } finally { for (ImageFrame f : frames) f.close(); }
  }
  private static void previews(Path root) throws Exception {
    Options o = new Options(); o.neighbors = 1;
    o.output = Files.createDirectories(root.resolve("thumbnails"));
    List<Path> paths = new ArrayList<>();
    Random random = new Random(92);
    for (int i = 0; i < 2; i++) {
      Path p = root.resolve("texture" + i + ".png");
      Mat m = new Mat(192, 256, CvType.CV_8UC3);
      byte[] pixels = new byte[192 * 256 * 3]; random.nextBytes(pixels); m.put(0, 0, pixels);
      try { Imgcodecs.imwrite(p.toString(), m); } finally { m.release(); }
      paths.add(p);
    }
    paths.add(Files.copy(paths.getFirst(), root.resolve("far_texture.png")));
    for (String algorithm : List.of("sift", "orb")) {
      o.matchingAlgorithm = algorithm;
      List<ImageFrame> frames = new ArrayList<>();
      try {
        for (int i = 0; i < 3; i++) frames.add(new ImageFrame(i, paths.get(i),
            new Calibration(200, 200, 127.5, 95.5, new double[5], 256, 192), o, true));
        check(has(PairSelection.select(frames, o, null), 0, 2), "Real thumbnail features discover nonadjacent duplicate: " + algorithm);
        check(frames.stream().allMatch(f -> f.descriptors.empty() && f.keypoints.length == 0),
            "Preview extraction leaves alignment feature storage untouched");
      } finally { for (ImageFrame f : frames) f.close(); }
    }
    Path blank = image(o.output);
    List<ImageFrame> frames = new ArrayList<>();
    try {
      for (int i = 0; i < 3; i++) frames.add(new ImageFrame(i, blank,
          new Calibration(100, 100, 49.5, 49.5, new double[5], 100, 100), o, true));
      check(PairSelection.select(frames, o, null).isEmpty(), "No evidence produces no fabricated filename edges");
      check(PairSelection.select(frames.subList(0, 1), o, null).isEmpty(), "Single image has no self edge");
    } finally { for (ImageFrame f : frames) f.close(); }
  }
  private static void cacheKeys(Path root) throws Exception {
    Options o = new Options();
    String base = ReuseCache.matchesKey(List.of(), o);
    o.neighbors++;
    check(!base.equals(ReuseCache.matchesKey(List.of(), o)), "Neighbour count invalidates matches");
    o.neighbors--;
    o.pairCandidates++;
    check(!base.equals(ReuseCache.matchesKey(List.of(), o)), "Retrieval breadth invalidates matches");
    o.pairCandidates--;
    o.pairSelection = "sequential";
    check(!base.equals(ReuseCache.matchesKey(List.of(), o)), "Selection mode invalidates matches");
    o.pairSelection = "overlap";
    o.poses = root.resolve("cache-poses.csv");
    Files.writeString(o.poses, "first");
    String poses = ReuseCache.matchesKey(List.of(), o);
    Files.writeString(o.poses, "second");
    check(!poses.equals(ReuseCache.matchesKey(List.of(), o)), "Changed pose contents invalidate overlap selection");
    o.poses = null;
    o.calibrationPoses = Map.of("image", new Camera(100, 100, 50, 50));
    check(!base.equals(ReuseCache.matchesKey(List.of(), o)), "Imported exterior orientations invalidate overlap selection");
  }
  private static boolean has(List<int[]> pairs, int a, int b) {
    return pairs.stream().anyMatch(p -> p[0] == Math.min(a, b) && p[1] == Math.max(a, b));
  }
  private static Path image(Path root) {
    Path path = root.resolve("preview.png");
    Mat m = new Mat(100, 100, CvType.CV_8UC3, new Scalar(0));
    try { Imgcodecs.imwrite(path.toString(), m); } finally { m.release(); }
    return path;
  }
  private static List<ImageFrame> synthetic(Path image, Options o, int n, boolean binary, int rows)
      throws Exception {
    List<ImageFrame> frames = new ArrayList<>();
    for (int i = 0; i < n; i++) {
      ImageFrame f = new ImageFrame(i, image,
          new Calibration(100, 100, 49.5, 49.5, new double[5], 100, 100), o, true);
      f.descriptors.release();
      Random r = new Random(1907L + i % (n / 2));
      f.descriptors = new Mat(rows, binary ? 32 : 128, binary ? CvType.CV_8U : CvType.CV_32F);
      if (binary) {
        byte[] data = new byte[rows * 32]; r.nextBytes(data); f.descriptors.put(0, 0, data);
      } else {
        float[] data = new float[rows * 128];
        for (int k = 0; k < data.length; k++) data[k] = r.nextFloat() * 255;
        f.descriptors.put(0, 0, data);
      }
      f.keypoints = new KeyPoint[rows];
      for (int k = 0; k < rows; k++) f.keypoints[k] = new KeyPoint(
          (k % 8 + .5f) * 12.5f, (k / 8 % 8 + .5f) * 12.5f, 3, -1, rows - k);
      frames.add(f);
    }
    return frames;
  }
  private static void scale(Path root) throws Exception {
    Options o = new Options(); o.neighbors = 5; o.memoryMb = 4096;
    o.output = root;
    List<ImageFrame> frames = synthetic(image(root), o, 3013, false, 64);
    long start = System.nanoTime();
    try {
      var result = VisualPairSelection.scores(frames, o);
      check(result.scores().size() <= 3013 * (o.neighbors + o.pairCandidates), "Bounded scoring at 3013 images");
      int found = 0;
      for (int a = 1; a < 1506; a++) if (result.scores().getOrDefault(PairSelection.key(a, a + 1506), 0.0) > 0) found++;
      check(found >= 1490, "Retrieval finds distant matching images: " + found);
      System.out.printf(Locale.ROOT, "PASS scale: 3013 images, %d scored pairs, %d/1505 distant true pairs, %.2f s%n",
          result.scores().size(), found, (System.nanoTime() - start) / 1e9);
    } finally { for (ImageFrame f : frames) f.close(); }
  }
}
