package sgm;

import java.io.*;
import java.nio.*;
import java.nio.file.*;
import java.util.*;
import java.util.zip.GZIPInputStream;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;

/**
 * Cross-stage regressions: exact tiled SGM, pyramid depth, priors, caches and partial registration.
 */
final class WorkflowRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Core.setNumThreads(1);
    Path root = Files.createTempDirectory(Path.of("target"), "workflow ");
    System.out.println("PASS workflow checks=" + run(root) + "; fixtures=" + root);
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Path dir = Files.createDirectory(root.resolve("workflow regression"));
    stereo(dir);
    pagedEvidence(dir);
    priors();
    pipeline(dir);
    return checks;
  }

  private static void stereo(Path dir) throws Exception {
    int w = 160, h = 96;
    byte[] left = new byte[w * h], right = new byte[w * h];
    new Random(981).nextBytes(left);
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) right[y * w + x] = left[y * w + Math.min(w - 1, x + 8)];
    for (int directions : new int[] {4, 8})
      for (int census : new int[] {5, 11}) {
        Options o = TestSupport.options();
        o.output = dir;
        o.disparities = 24;
        o.census = census;
        o.directions = directions;
        o.speckleSize = 0;
        o.memoryMb = 16;
        o.tileSize = 32;
        var memory = new CensusSgm(o).compute(left, right, w, h);
        o.tiled = true;
        var disk = new CensusSgm(o).compute(left, right, w, h);
        check(
            Arrays.equals(memory.disparity, disk.disparity),
            "Disk tiles preserve every disparity including NaN, for "
                + directions
                + " paths and census "
                + census);
        check(
            memory.stats.csv().equals(disk.stats.csv()),
            "Tiled and ordinary SGM rejection counts are exact");
      }
    Options signed = TestSupport.options();
    signed.output = dir;
    signed.tiled = true;
    signed.disparities = 24;
    signed.minDisparity = -16;
    signed.speckleSize = 0;
    var reverse = new CensusSgm(signed).compute(right, left, w, h);
    check(
        Math.abs(reverse.disparity[40 * w + 60] + 8) < .2,
        "Tiled SGM preserves signed disparities");
    Options pyramid = TestSupport.options();
    pyramid.output = dir;
    pyramid.disparities = 32;
    pyramid.pyramidLevels = 3;
    pyramid.tileSize = 64;
    pyramid.tileOverlap = 16;
    pyramid.pyramidRadius = 8;
    pyramid.speckleSize = 0;
    byte[] mask = new byte[w * h];
    Arrays.fill(mask, (byte) 255);
    var r = DenseMatcher.compute(pyramid, left, right, null, null, w, h, mask, mask, null);
    int count = 0;
    double error = 0;
    for (int y = 16; y < h - 16; y++)
      for (int x = 20; x < w - 16; x++)
        if (Float.isFinite(r.disparity[y * w + x])) {
          count++;
          error += Math.abs(r.disparity[y * w + x] - 8);
        }
    check(
        count > 4000 && error / count < .2,
        "Coarse-to-fine overlapping tiles recover an independent eight-pixel shift across seams");
    try (var files = Files.list(dir)) {
      check(
          files.noneMatch(
              p ->
                  p.getFileName().toString().startsWith("sgm-") && p.toString().endsWith(".tiles")),
          "Temporary SGM storage is closed and removed");
    }
    Options volumes = TestSupport.options();
    volumes.output = dir;
    volumes.tileSize = 32;
    volumes.disparities = 1;
    volumes.memoryMb = 16;
    try (PagedVolume volume = new PagedVolume(3000000, 4, 17, true, volumes, "regression-")) {
      for (int i = 0; i < 3000000; i += 1024) volume.set(i, i ^ 9182);
      boolean exact = true;
      for (int i = 2999296; i >= 0; i -= 1024) if (volume.get(i) != (i ^ 9182)) exact = false;
      check(exact, "Paged volume values survive repeated eviction and reverse reads");
      check(volume.get(91) == 17, "Sparse unwritten pages preserve the requested default");
    }
  }

  private static void pagedEvidence(Path dir) throws Exception {
    Options o = TestSupport.options();
    o.output = dir;
    o.cacheDirectory = dir;
    o.diskDepth = true;
    PagedVolume.Pool pool = new PagedVolume.Pool(65536);
    List<DepthRaster> maps = new ArrayList<>();
    try {
      for (int map = 0; map < 8; map++) {
        DepthRaster raster = new DepthRaster(10000, o, pool);
        maps.add(raster);
        for (int p = 0; p < 10000; p += 17)
          raster.put(p, map * 100000 + p, DepthFusion.pack(.13, -.22));
      }
      check(
          pool.used <= pool.limit,
          "Depth evidence across all pairs shares one bounded resident page budget");
      boolean exact = true;
      for (int map = 7; map >= 0; map--)
        for (int p = 0; p < 10000; p += 17)
          if (maps.get(map).get(p) != map * 100000 + p
              || maps.get(map).offset(p) != DepthFusion.pack(.13, -.22)) exact = false;
      check(exact, "Depth and packed subpixel offsets survive cross-map eviction exactly");
    } finally {
      for (DepthRaster raster : maps) raster.close();
    }
    check(pool.used == 0, "Closing depth maps releases the shared page budget");
  }

  private static void priors() {

    Calibration c =
        new Calibration(
            850, 850, 499.5, 399.5, new double[] {-.12, .05, .001, -.002, .003}, 1000, 800);
    SelfCalibration.Data initial = new SelfCalibration.Data(c, true),
        truth =
            new SelfCalibration.Data(
                new Calibration(800, 800, 499.5, 399.5, c.distortion, 1000, 800), true);
    for (int i = 0; i < 6; i++) {
      Camera camera = new Camera(850, 850, 499.5, 399.5);
      camera.registered = true;
      camera.rotation =
          Math3.rotation(new double[] {.03 * Math.sin(i), .07 * (i - 2), .02 * Math.cos(i)});
      double[] center = {.45 * i, .2 * Math.sin(i), .12 * Math.cos(i)};
      camera.translation = Math3.times(Math3.mv(camera.rotation, center), -1);
      initial.views.add(new SelfCalibration.View(camera, 1, 1));
      truth.views.add(new SelfCalibration.View(camera, 1, 1));
    }
    Random random = new Random(188);
    for (int k = 0; k < 220; k++) {
      double[] point = {
        random.nextDouble() * 5 - 1.5, random.nextDouble() * 4 - 2, 3 + random.nextDouble() * 4
      };
      var sample = new SelfCalibration.Sample(k, point, k % 5 == 0);
      for (int i = 0; i < 6; i++) {
        double[] uv = SelfCalibration.project(truth, i, point);
        sample.seen.add(new SelfCalibration.Seen(i, uv[0], uv[1]));
      }
      initial.samples.add(sample);
    }
    Options weak = TestSupport.options();
    weak.intrinsicsMode = "initial";
    weak.refineIntrinsics = "f";
    initial.controls = weak;
    initial.priorLens = initial.lens.clone();
    var free = SelfCalibration.optimize(initial, 40);
    check(
        Math.abs(Math.exp(free.fitted.lens[0]) - 800) < 2,
        "Selected focal refinement recovers an independently generated raw distorted scene");
    check(
        free.fitted.lens[1] == initial.lens[1] && free.fitted.lens[2] == initial.lens[2],
        "Unselected radial coefficients are bitwise fixed");
    check(
        Arrays.equals(free.fitted.extra, initial.extra),
        "Supplied tangential and k3 coefficients remain fixed");
    SelfCalibration.Data constrained = initial.copy();
    Options strong = weak.copy();
    strong.intrinsicsMode = "prior";
    strong.focalSigma = 1e-5;
    constrained.controls = strong;
    var prior = SelfCalibration.optimize(constrained, 30);
    check(
        Math.abs(Math.exp(prior.fitted.lens[0]) - 850) < 1,
        "A precise focal prior strongly constrains adjustment");
  }

  private static void pipeline(Path dir) throws Exception {
    Path images = Files.createDirectory(dir.resolve("images"));
    int w = 320, h = 240;
    byte[] texture = new byte[(w + 80) * h];
    new Random(17).nextBytes(texture);
    for (int camera = 0; camera < 3; camera++) {
      byte[] values = new byte[w * h];
      for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
          int disparity = y < h / 3 ? 12 : y < 2 * h / 3 ? 8 : 5;
          values[y * w + x] = texture[y * (w + 80) + x + camera * disparity];
        }
      Mat m = new Mat(h, w, CvType.CV_8U);
      try {
        m.put(0, 0, values);
        Imgcodecs.imwrite(images.resolve(camera + ".png").toString(), m);
      } finally {
        m.release();
      }
    }
    Path calib = dir.resolve("camera.txt");
    new Calibration(240, 240, 159.5, 119.5, new double[5], w, h)
        .write(calib, "Independent fixture");
    Options base = Options.parse(new String[] {"-i", images.toString(), "--no-gps"});
    Options independent =
        Options.parse(
            new String[] {
              "-i",
              images.toString(),
              "--no-gps",
              "--quality",
              "low",
              "--alignment-max-dimension",
              "300"
            });
    check(
        independent.processedSize(320, 240)[0] == 40
            && independent.alignmentOptions().processedSize(320, 240)[0] == 300,
        "An explicit alignment cap is independent of dense downsampling");
    for (String[] invalid :
        new String[][] {
          {"--intrinsics-mode", "wrong"},
          {"--extrinsics-mode", "wrong"},
          {"--matching-algorithm", "surf"},
          {"--pair-selection", "wrong"},
          {"--focal-sigma", "0"},
          {"--pyramid-levels", "0"}
        }) {
      boolean rejected = false;
      try {
        Options.parse(new String[] {"-i", images.toString(), "--no-gps", invalid[0], invalid[1]});
      } catch (IllegalArgumentException e) {
        rejected = true;
      }
      check(rejected, "Invalid workflow option fails clearly: " + invalid[0]);
    }
    for (String algorithm : List.of("sift", "orb", "akaze", "kaze", "brisk")) {
      Options o = base.copy();
      o.matchingAlgorithm = algorithm;
      o.features = 500;
      o.output = dir;
      try (ImageFrame a =
              new ImageFrame(
                  0,
                  images.resolve("0.png"),
                  new Calibration(240, 240, 159.5, 119.5, new double[5], w, h),
                  o);
          ImageFrame b =
              new ImageFrame(
                  1,
                  images.resolve("1.png"),
                  new Calibration(240, 240, 159.5, 119.5, new double[5], w, h),
                  o)) {
        a.extract(500);
        b.extract(500);
        check(
            a.keypoints.length > 20 && a.keypoints.length <= 500,
            "Selectable " + algorithm + " detector produces capped descriptors");
        var matcher = FeatureAlgorithms.matcher(o, a.descriptors);
        List<MatOfDMatch> matches = new ArrayList<>();
        try {
          matcher.knnMatch(a.descriptors, b.descriptors, matches, 2);
          check(
              matches.size() == a.keypoints.length,
              "Compatible descriptor metric for " + algorithm);
        } finally {
          for (Mat m : matches) m.release();
          matcher.clear();
        }
      }
    }
    List<ImageFrame> shortlist = new ArrayList<>();
    Options selection = base.copy();
    selection.output = dir;
    selection.neighbors = 1;
    selection.pairCandidates = 2;
    try {
      for (int i = 0; i < 3; i++) {
        ImageFrame f =
            new ImageFrame(
                i,
                images.resolve(i + ".png"),
                new Calibration(240, 240, 159.5, 119.5, new double[5], w, h),
                selection);
        f.extract(500);
        shortlist.add(f);
      }
      selection.pairSelection = "overlap";
      List<int[]> chosen = PairSelection.select(shortlist, selection, null);
      check(
          chosen.stream().anyMatch(p -> p[0] == 0 && p[1] == 2),
          "Overlap preselection discovers a pair beyond filename-neighbor edges");
      check(
          Json.string(chosen).equals(Json.string(PairSelection.select(shortlist, selection, null))),
          "Overlap shortlist ordering is deterministic");
      selection.pairSelection = "gps";
      PairSelection.select(shortlist, selection, null);
      check(
          Files.readString(dir.resolve("03_matches/pair_selection.json"))
              .contains("descriptor_preview"),
          "GPS preselection falls back to actual descriptor evidence when metadata is absent");
    } finally {
      for (ImageFrame f : shortlist) f.close();
    }
    Path pose = dir.resolve("poses.csv");
    Files.writeString(
        pose,
        PoseIO.HEADER
            + "\n"
            + "0.png,1,0,0,0,1,0,0,0,1,0,0,0\n"
            + "1.png,1,0,0,0,1,0,0,0,1,-.25,0,0\n"
            + "2.png,1,0,0,0,1,0,0,0,1,-.5,0,0\n");
    Path priorProject = dir.resolve("pose prior project");
    check(
        TestSupport.run(
                new String[] {
                  "-i",
                  images.toString(),
                  "-o",
                  priorProject.toString(),
                  "--calibration",
                  calib.toString(),
                  "--poses",
                  pose.toString(),
                  "--extrinsics-mode",
                  "prior",
                  "--position-sigma",
                  ".001",
                  "--rotation-sigma",
                  ".1",
                  "--no-gps",
                  "--disparities",
                  "48",
                  "--cleanup-passes",
                  "0",
                  "--cores",
                  "1"
                })
            == 0,
        "Supplied pose uncertainty priors run through joint optimization and reconstruction");
    String constraints =
        Files.readString(latest(priorProject).resolve("04_geometry/pose_constraints.json"));
    check(
        constraints.contains("\"mode\":\"prior\"") && constraints.contains("reference_center"),
        "Pose-prior reference and adjusted values are audited");
    check(
        Files.readString(priorProject.resolve("report.txt")).contains("joint robust Schur"),
        "Report distinguishes optimized supplied poses from fixed poses");
    check(
        Files.readString(latest(priorProject).resolve("04_geometry/bundle/solver.json"))
            .contains("\"variable_cameras\":3"),
        "Absolute pose priors allow every supplied camera to refine without freezing the first"
            + " pose");
    Path project = dir.resolve("reusable project");
    List<String> args =
        new ArrayList<>(
            List.of(
                "-i",
                images.toString(),
                "-o",
                project.toString(),
                "--calibration",
                calib.toString(),
                "--poses",
                pose.toString(),
                "--disparities",
                "48",
                "--no-gps",
                "--step",
                "3",
                "--tiled",
                "--alignment-max-dimension",
                "240",
                "--cleanup-passes",
                "0",
                "--cores",
                "1"));
    check(
        TestSupport.run(args.toArray(new String[0])) == 0,
        "Independent alignment and dense sizes complete with supplied poses and tiled storage");
    Path first = latest(project);
    byte[] original = Files.readAllBytes(project.resolve("dense.las"));
    check(
        Files.exists(first.resolve("00_inputs/dense/image_0000/image.json")),
        "Dense image calibration/raster provenance is separate from alignment inputs");
    args.add("--resume");
    check(TestSupport.run(args.toArray(new String[0])) == 0, "Compatible project rerun succeeds");
    Path second = latest(project);
    check(
        Files.readString(second.resolve("logs/console.log")).contains("Reused stereo cache"),
        "Compatible stereo results and stage rasters are reused");
    check(
        Arrays.equals(original, Files.readAllBytes(project.resolve("dense.las"))),
        "Cached disparity rerun preserves final LAS exactly");
    try (BufferedReader reader =
        new BufferedReader(
            new InputStreamReader(
                new GZIPInputStream(
                    Files.newInputStream(second.resolve("07_results/point-confidence.csv.gz")))))) {
      String[] header = reader.readLine().split(",");
      check(
          header[0].equals("point_id") && header[9].equals("supporting_views"),
          "Confidence sidecar documents cloud-order indexing and support");
      long rows = reader.lines().count();
      ByteBuffer las = ByteBuffer.wrap(original).order(ByteOrder.LITTLE_ENDIAN);
      check(
          rows == Integer.toUnsignedLong(las.getInt(107)),
          "Every exported LAS point has exactly one confidence row");
    }
    args.addAll(List.of("--rerun-from", "fusion", "--step", "2"));
    check(
        TestSupport.run(args.toArray(new String[0])) == 0,
        "Fusion-only rerun accepts changed candidate sampling without recalculating SGM");
    check(
        Files.readString(latest(project).resolve("logs/console.log"))
            .contains("Reused stereo cache"),
        "Candidate sampling change preserves stereo compatibility");
    Path payload;
    try (var files = Files.walk(project.resolve("cache/v3/debug/stereo"))) {
      payload =
          files
              .filter(p -> p.getFileName().toString().equals("payload.bin"))
              .findFirst()
              .orElseThrow();
    }
    try (RandomAccessFile file = new RandomAccessFile(payload.toFile(), "rw")) {
      file.seek(12);
      int value = file.read();
      file.seek(12);
      file.write(value ^ 1);
    }
    check(
        TestSupport.run(args.toArray(new String[0])) == 0,
        "A corrupted cache is rejected and safely recomputed");
    check(
        Files.readString(latest(project).resolve("logs/console.log"))
            .contains("Cache checksum mismatch"),
        "Checksum rejection is visible in the project log");
    // Feature/match/geometry resume, a floating and a binary descriptor serialization path.
    for (String algorithm : List.of("sift", "orb")) {
      Path out = dir.resolve(algorithm + " feature cache");
      List<String> run =
          new ArrayList<>(
              List.of(
                  "-i",
                  images.toString(),
                  "-o",
                  out.toString(),
                  "--calibration",
                  calib.toString(),
                  "--matching-algorithm",
                  algorithm,
                  "--features",
                  "800",
                  "--ratio",
                  ".85",
                  "--disparities",
                  "48",
                  "--no-gps",
                  "--cleanup-passes",
                  "0",
                  "--cores",
                  "1"));
      int status = TestSupport.run(run.toArray(new String[0]));
      check(
          status == 0,
          "Initial " + algorithm + " reconstruction creates reusable features and geometry");
      run.add("--resume");
      check(
          TestSupport.run(run.toArray(new String[0])) == 0,
          "Resume " + algorithm + " reconstruction");
      String log = Files.readString(latest(out).resolve("logs/console.log"));
      check(
          log.contains("Reused features cache")
              && log.contains("Reused matches cache")
              && log.contains("Reused geometry cache"),
          "Features, matches and geometry cache round-trip for " + algorithm);
    }
    Path partial = Files.createDirectory(dir.resolve("partial images"));
    Files.copy(images.resolve("0.png"), partial.resolve("0.png"));
    Files.copy(images.resolve("1.png"), partial.resolve("1.png"));
    Files.copy(images.resolve("2.png"), partial.resolve("2.png"));
    Mat blank = new Mat(h, w, CvType.CV_8U, new Scalar(128));
    try {
      Imgcodecs.imwrite(partial.resolve("3.png").toString(), blank);
    } finally {
      blank.release();
    }
    Path out = dir.resolve("partial project");
    check(
        TestSupport.run(
                new String[] {
                  "-i",
                  partial.toString(),
                  "-o",
                  out.toString(),
                  "--calibration",
                  calib.toString(),
                  "--no-gps",
                  "--disparities",
                  "48",
                  "--cleanup-passes",
                  "0",
                  "--cores",
                  "1"
                })
            == 0,
        "Unregisterable blank image is excluded while valid pair reconstructs");
    check(
        Files.readString(out.resolve("report.txt"))
            .contains("Registered cameras=3/4; failed images=[3.png]"),
        "Final report names unregistered images");
    check(
        Files.readAllLines(out.resolve("cameras.csv")).size() == 4,
        "Camera CSV excludes unregistered placeholder poses");
  }

  private static Path latest(Path project) throws IOException {
    Properties p = new Properties();
    try (Reader in = Files.newBufferedReader(project.resolve("project.properties"))) {
      p.load(in);
    }
    return project.resolve("runs").resolve(p.getProperty("latest_run"));
  }
}
