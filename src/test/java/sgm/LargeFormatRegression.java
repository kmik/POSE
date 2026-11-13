package sgm;

import java.awt.image.BufferedImage;
import java.nio.file.*;
import java.util.*;
import javax.imageio.ImageIO;

/**
 * Large-format paths (STREAMED_PROCESSING.md), end to end through DenseReconstruction with
 * --dense-method mvsgm and image fusion, on the synthetic scene of MultiViewSgmRegression (a
 * textured far plane and a nearer rectangle seen by five cameras, one rotated 180 degrees):
 *
 * <ol>
 *   <li>mvsgm depth maps handed to fusion directly (no candidate file); points on the true planes;
 *   <li>image fusion forced into strips of 7 rows (-Dsgm.fusion.tileRows; smaller than the 13-row
 *       halo) gives a byte-identical LAS; strip map files are removed;
 *   <li>mvsgm forced into 96-pixel tiles (-Dsgm.mvsgm.tile) agrees with the untiled maps except
 *       near tile borders; streamed previews for tiled images; strips on tiled maps identical too;
 *   <li>--streamed-processing parsing.
 * </ol>
 *
 * Runs on the CPU. Fixtures go to target/large-format* and are deleted afterwards.
 */
final class LargeFormatRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    System.out.println("PASS large-format paths: " + run());
  }

  static final int W = MultiViewSgmRegression.W, H = MultiViewSgmRegression.H;

  static int run() throws Exception {
    checks = 0;
    check(new Options().streamedProcessing.equals("auto"), "--streamed-processing defaults to auto");
    check(Options.parse(new String[] {"-i", ".", "--streamed-processing", "on"}).streamedProcessing.equals("on"),
        "--streamed-processing on parses");
    boolean rejected = false;
    try {
      Options.parse(new String[] {"-i", ".", "--streamed-processing", "always"});
    } catch (IllegalArgumentException e) {
      rejected = true;
    }
    check(rejected, "unknown --streamed-processing value is rejected");

    Path root = Files.createTempDirectory(Files.createDirectories(Path.of("target")), "large-format ");
    try {
      Camera[] cams = {
        MultiViewSgmRegression.camera(0, 0, false), MultiViewSgmRegression.camera(6, 0, false),
        MultiViewSgmRegression.camera(-6, 0, true), MultiViewSgmRegression.camera(0, 5, false),
        MultiViewSgmRegression.camera(0, -5, false)
      };
      float[] far = MultiViewSgmRegression.texture(1), near = MultiViewSgmRegression.texture(2);
      Path images = Files.createDirectories(root.resolve("images"));
      for (int i = 0; i < cams.length; i++) {
        byte[] g = MultiViewSgmRegression.render(cams[i], far, near, null);
        BufferedImage image = new BufferedImage(W, H, BufferedImage.TYPE_INT_RGB);
        for (int y = 0; y < H; y++)
          for (int x = 0; x < W; x++) image.setRGB(x, y, (g[y * W + x] & 255) * 0x010101);
        check(ImageIO.write(image, "TIFF", images.resolve(i + ".tif").toFile()), "fixture image written");
      }
      List<FeatureTracks.Track> tracks = tracks(cams);
      lifetimeFiles(root, cams[0]);
      retirementRuns(root, cams, tracks);
      separatedWindows(root, cams, tracks);
      returnFlightWindow(root, cams, tracks);

      Run untiled = dense(root, "untiled", cams, tracks, null, null);
      check(untiled.points > 20000, "mvsgm + image fusion emits points: " + untiled.points);
      check(onPlanes(untiled.las) >= .95, "points lie on the true planes");
      check(!Files.exists(untiled.dir.resolve("06_fusion/map-spill")), "no strip files without strips");
      check(Files.readString(untiled.dir.resolve("05_stereo/mvsgm/mvsgm.json")).contains("\"handoff\":\"maps\""),
          "mvsgm maps handed to fusion");
      try (var files = Files.list(untiled.dir)) {
        check(files.noneMatch(p -> p.getFileName().toString().startsWith("candidates-")), "candidate file removed");
      }

      Run strips = dense(root, "strips", cams, tracks, null, "7");
      check(Files.readString(strips.dir.resolve("06_fusion/image_fusion.json")).contains("fusion in strips of 7 rows"),
          "fusion ran in strips");
      check(!Files.exists(strips.dir.resolve("06_fusion/map-spill")), "strip map files removed");
      check(sameLas(untiled.las, strips.las), "fusion in strips is byte-identical to untiled fusion");

      Run tiled = dense(root, "tiled", cams, tracks, "96", null);
      check(Files.readString(tiled.dir.resolve("05_stereo/mvsgm/mvsgm.json")).contains("\"tiled\":true"),
          "mvsgm ran in tiles");
      int total = 0, both = 0, agree = 0, onlyOne = 0;
      for (int c = 0; c < cams.length; c++) {
        Path a = untiled.dir.resolve("05_stereo/mvsgm/camera_" + c + ".depth"), b = tiled.dir.resolve("05_stereo/mvsgm/camera_" + c + ".depth");
        if (!Files.exists(a) || !Files.exists(b)) continue;
        float[] za = DepthMapFile.read(a).z(), zb = DepthMapFile.read(b).z();
        for (int p = 0; p < za.length; p++) {
          if (za[p] > 0 || zb[p] > 0) total++;
          if (za[p] > 0 && zb[p] > 0) {
            both++;
            if (Math.abs(za[p] - zb[p]) <= .005 * za[p]) agree++;
          } else if (za[p] > 0 || zb[p] > 0) onlyOne++;
        }
      }
      System.out.printf(Locale.ROOT, "Large-format regression: tiled vs untiled mvsgm: %d of %d depths within 0.5%%,"
          + " %d of %d pixels valid in only one; points %d untiled, %d tiled%n", agree, both, onlyOne, total,
          untiled.points, tiled.points);
      check(both > 0 && agree >= .97 * both, "tiled mvsgm depths agree with untiled: " + agree + "/" + both);
      check(onlyOne <= .05 * total, "tiled mvsgm coverage matches untiled: " + onlyOne + " of " + total + " differ");
      check(onPlanes(tiled.las) >= .95, "tiled mvsgm points lie on the true planes");
      Path preview;
      try (var dirs = Files.list(tiled.dir.resolve("05_stereo/mvsgm"))) {
        preview = dirs.filter(p -> p.getFileName().toString().startsWith("image_0_")).findFirst().orElseThrow();
      }
      for (String kind : new String[] {"depth", "consistent_depth"}) {
        check(Files.size(preview.resolve(kind + ".pfm")) == ("Pf\n" + W + " " + H + "\n-1.0\n").length() + 4L * W * H,
            "streamed " + kind + ".pfm at full resolution");
        check(Files.exists(preview.resolve(kind + ".png")), "streamed " + kind + ".png");
        check(Files.readString(preview.resolve(kind + ".json")).contains("\"preview_scale\":1"), kind + " preview scale");
      }
      Run tiledStrips = dense(root, "tiled-strips", cams, tracks, "96", "5");
      check(sameLas(tiled.las, tiledStrips.las), "strips on tiled mvsgm maps are byte-identical");
      widePreview(root);
    } finally {
      delete(root);
    }
    return checks;
  }

  /** A last camera returns to the first group: raw lookahead must skip unrelated middle groups. */
  private static void returnFlightWindow(Path root, Camera[] cams, List<FeatureTracks.Track> tracks) throws Exception {
    Path block = Files.createDirectories(root.resolve("return-window"));
    Files.createDirectories(block.resolve("images"));
    Camera[] scene = new Camera[16];
    List<FeatureTracks.Track> observations = new ArrayList<>();
    for (int group = 0; group < 3; group++) {
      double offset = 1000 * group;
      for (int i = 0; i < cams.length; i++) {
        int id = group * cams.length + i;
        Camera c = cams[i].copy();
        for (int r = 0; r < 3; r++) c.translation[r] -= c.rotation[3 * r] * offset;
        scene[id] = c;
        Files.copy(root.resolve("images/" + i + ".tif"), block.resolve("images/" + id + ".tif"));
      }
      for (FeatureTracks.Track track : tracks) {
        FeatureTracks.Track copy = new FeatureTracks.Track();
        copy.point = track.point.clone();
        copy.point[0] += offset;
        for (FeatureTracks.Observation ob : track.observations) {
          copy.observations.add(new FeatureTracks.Observation(group * cams.length + ob.image, ob.feature));
          if (group == 0 && ob.image == 0) copy.observations.add(new FeatureTracks.Observation(15, ob.feature));
        }
        observations.add(copy);
      }
    }
    scene[15] = cams[0].copy();
    Files.copy(root.resolve("images/0.tif"), block.resolve("images/15.tif"));
    String before = System.getProperty("sgm.mvsgm.window");
    java.util.function.Consumer<Options> configure = o -> {
      o.diagnostics = false;
      o.writePointConfidence = true;
    };
    try {
      System.setProperty("sgm.mvsgm.window", "false");
      Run sequential = dense(block, "sequential", scene, observations, null, "23", configure);
      System.setProperty("sgm.mvsgm.window", "true");
      Run window = dense(block, "window", scene, observations, null, "23", configure);
      check(sameLas(sequential.las, window.las), "return-to-first-group window preserves the sequential cloud");
      check(Files.mismatch(sequential.dir.resolve("07_results/point-confidence.bin"),
          window.dir.resolve("07_results/point-confidence.bin")) == -1, "out-of-order raw solves preserve global candidate IDs");
      String json = Files.readString(window.dir.resolve("06_fusion/image_fusion.json"));
      check(json.contains("\"peak_live_cameras\":11"), "return flight retains the first group and one current group");
      check(json.contains("\"peak_source_map_bytes\":" + 11 * (24L * W * H + 6L * DepthMapFile.HEADER)),
          "distant raw input does not create all sixteen source maps at startup");
    } finally {
      if (before == null) System.clearProperty("sgm.mvsgm.window");
      else System.setProperty("sgm.mvsgm.window", before);
    }
  }

  /** Two disjoint five-camera blocks: the first retires while the second still needs its maps. */
  private static void separatedWindows(Path root, Camera[] cams, List<FeatureTracks.Track> tracks) throws Exception {
    Path block = Files.createDirectories(root.resolve("separated"));
    Files.createDirectories(block.resolve("images"));
    Camera[] separated = new Camera[2 * cams.length];
    List<FeatureTracks.Track> observations = new ArrayList<>();
    for (int group = 0; group < 2; group++) {
      double offset = 1000 * group;
      for (int i = 0; i < cams.length; i++) {
        int id = group * cams.length + i;
        Camera c = MultiViewSgmRegression.camera(0, 0, false);
        c.rotation = cams[i].rotation.clone();
        c.translation = cams[i].translation.clone();
        for (int r = 0; r < 3; r++) c.translation[r] -= c.rotation[3 * r] * offset;
        separated[id] = c;
        Files.copy(root.resolve("images/" + i + ".tif"), block.resolve("images/" + id + ".tif"));
      }
      for (FeatureTracks.Track track : tracks) {
        FeatureTracks.Track copy = new FeatureTracks.Track();
        copy.point = track.point.clone();
        copy.point[0] += offset;
        for (FeatureTracks.Observation ob : track.observations)
          copy.observations.add(new FeatureTracks.Observation(group * cams.length + ob.image, ob.feature));
        observations.add(copy);
      }
    }
    String prior = System.getProperty("sgm.fusion.keepFinishedMaps");
    String budget = System.getProperty("sgm.fusion.mapBudgetMb");
    String window = System.getProperty("sgm.mvsgm.window");
    try {
      for (String mode : new String[] {"strips", "spill"}) {
        if (mode.equals("spill")) System.setProperty("sgm.fusion.mapBudgetMb", "1");
        else System.clearProperty("sgm.fusion.mapBudgetMb");
        String rows = mode.equals("strips") ? "23" : null;
        System.setProperty("sgm.fusion.keepFinishedMaps", "true");
        System.setProperty("sgm.mvsgm.window", "false");
        Run kept = dense(block, mode + "-kept", separated, observations, null, rows, o -> o.diagnostics = false);
        System.clearProperty("sgm.fusion.keepFinishedMaps");
        System.setProperty("sgm.mvsgm.window", "true");
        Run released = dense(block, mode + "-released", separated, observations, null, rows, o -> o.diagnostics = false);
        check(sameLas(kept.las, released.las), "retiring first window preserves later window output: " + mode);
        String json = Files.readString(released.dir.resolve("06_fusion/image_fusion.json"));
        check(json.contains("\"last_map_use\":[4,4,4,4,4,9,9,9,9,9]"), "two independent retirement windows");
        check(json.contains("\"retired_cameras\":10") && json.contains("\"remaining_map_bytes\":0"),
            "both windows retired with no planned maps remaining");
        if (mode.equals("spill")) check(!json.contains("\"map_evictions\":0"), "separated windows exercise heap spill");
        check(json.contains("\"peak_live_cameras\":5"), "only one separated group is live during fusion");
        long sourceWindow = 5 * (24L * W * H + 6L * DepthMapFile.HEADER);
        check(json.contains("\"peak_source_map_bytes\":" + sourceWindow), "source-map disk peaks at one group too");
        check(json.contains("\"remaining_source_map_bytes\":0"), "source-map reader ledger releases both groups");
      }
    } finally {
      if (prior == null) System.clearProperty("sgm.fusion.keepFinishedMaps");
      else System.setProperty("sgm.fusion.keepFinishedMaps", prior);
      if (budget == null) System.clearProperty("sgm.fusion.mapBudgetMb");
      else System.setProperty("sgm.fusion.mapBudgetMb", budget);
      if (window == null) System.clearProperty("sgm.mvsgm.window");
      else System.setProperty("sgm.mvsgm.window", window);
    }
  }

  /** Directed overlap, including a camera needed only by a later incoming edge. */
  private static void lifetimeFiles(Path root, Camera cam) throws Exception {
    int[][] neighbours = {{1}, {}, {0}, {2}, {}};
    check(Arrays.equals(ImageFusion.lastMapUse(neighbours), new int[] {2, 1, 3, 3, 4}),
        "last use includes own pass and incoming directed edges");
    check(ImageFusion.lastMapUse(new int[0][]).length == 0, "empty overlap graph");
    Path dir = Files.createDirectories(root.resolve("lifetime-maps"));
    try (MvsgmMaps maps = new MvsgmMaps(dir, false)) {
      for (int c = 0; c < 2; c++) {
        maps.add(new MvsgmMaps.Camera(c, c, 2, 2, new long[3], false));
        for (int k = 0; k < MvsgmMaps.KINDS.length; k++)
          DepthMapFile.write(maps.file(c, k), cam, 2, 2, new float[] {1, 2, 3, 4});
      }
      var finished = maps.segment(0, MvsgmMaps.DEPTH);
      var needed = maps.segment(1, MvsgmMaps.DEPTH);
      maps.releaseCamera(0);
      check(!finished.scope().isAlive(), "finished camera is unmapped before deletion");
      for (int k = 0; k < MvsgmMaps.KINDS.length; k++) {
        check(!Files.exists(maps.file(0, k)), "finished camera file gone: " + k);
        check(Files.exists(maps.file(1, k)), "still-needed camera file retained: " + k);
      }
      check(needed.scope().isAlive() && needed.getAtIndex(MappedFiles.FLOAT, 3) == 4,
          "still-needed camera mapping retains its values");
      maps.releaseCamera(0); // Cleanup is idempotent.
    }
    check(!Files.exists(dir.resolve("camera_1.depth")), "failure/close removes remaining camera");
    try (MvsgmMaps maps = new MvsgmMaps(dir, true)) {
      maps.add(new MvsgmMaps.Camera(0, 0, 2, 2, new long[3], false));
      DepthMapFile.write(maps.file(0, MvsgmMaps.DEPTH), cam, 2, 2, new float[4]);
      var kept = maps.segment(0, MvsgmMaps.DEPTH);
      maps.releaseCamera(0);
      check(!kept.scope().isAlive() && Files.exists(maps.file(0, MvsgmMaps.DEPTH)),
          "diagnostic retention unmaps and keeps files");
    }
    try (MvsgmMaps maps = new MvsgmMaps(dir, false)) {
      maps.add(new MvsgmMaps.Camera(7, -1, 0, 0, null, false));
      Files.write(maps.file(7, MvsgmMaps.DEPTH), new byte[9]);
      throw new java.io.IOException("injected partial stereo failure");
    } catch (java.io.IOException expected) {
      check(expected.getMessage().equals("injected partial stereo failure"), "partial stereo failure propagated");
    }
    check(!Files.exists(dir.resolve("camera_7.depth")), "partial stereo file removed on failure");
  }

  /** Early deletion against the old lifecycle, including precision revisits and mapped marks. */
  private static void retirementRuns(Path root, Camera[] cams, List<FeatureTracks.Track> tracks) throws Exception {
    String prior = System.getProperty("sgm.fusion.keepFinishedMaps");
    String beforeWindow = System.getProperty("sgm.mvsgm.window");
    try {
      for (String strategy : new String[] {"graph-average", "owner-consume"}) {
        java.util.function.Consumer<Options> configure = o -> {
          o.diagnostics = false;
          o.fusionStrategy = strategy;
          o.fusionSeedOrder = "precision";
          o.fusionSeedMinAngle = 3;
        };
        System.setProperty("sgm.fusion.keepFinishedMaps", "true");
        System.setProperty("sgm.mvsgm.window", "false");
        Run old = dense(root, strategy + "-kept", cams, tracks, null, "23", configure);
        System.clearProperty("sgm.fusion.keepFinishedMaps");
        System.setProperty("sgm.mvsgm.window", "true");
        Run released = dense(root, strategy + "-released", cams, tracks, null, "23", configure);
        check(sameLas(old.las, released.las), "early deletion preserves " + strategy + " across precision tiers");
        String json = Files.readString(released.dir.resolve("06_fusion/image_fusion.json"));
        check(json.contains("\"retired_cameras\":5"), "all cameras retire during fusion: " + strategy);
        check(json.contains("\"remaining_map_bytes\":0"), "finished cameras leave memory plan: " + strategy);
        check(!Files.exists(released.dir.resolve("06_fusion/map-spill")), "mapped spill files removed: " + strategy);
        try (var files = Files.list(released.dir.resolve("05_stereo/mvsgm"))) {
          check(files.noneMatch(p -> p.getFileName().toString().matches("camera_.*")),
              "source maps removed: " + strategy);
        }
      }
      boolean failed = false;
      try {
        dense(root, "failed-fusion", cams, tracks, null, "23", o -> {
          o.diagnostics = false;
          try {
            Files.createDirectories(o.output.resolve("06_fusion"));
            Files.writeString(o.output.resolve("06_fusion/map-spill"), "block spill creation");
          } catch (java.io.IOException e) {
            throw new java.io.UncheckedIOException(e);
          }
        });
      } catch (java.io.IOException expected) {
        failed = true;
      }
      check(failed, "fusion I/O failure propagated");
      try (var files = Files.list(root.resolve("failed-fusion/05_stereo/mvsgm"))) {
        check(files.noneMatch(p -> p.getFileName().toString().startsWith("camera_")),
            "source maps removed after fusion failure");
      }
      interruptedRun(root, cams, tracks, false);
      interruptedRun(root, cams, tracks, true);
    } finally {
      if (prior == null) System.clearProperty("sgm.fusion.keepFinishedMaps");
      else System.setProperty("sgm.fusion.keepFinishedMaps", prior);
      if (beforeWindow == null) System.clearProperty("sgm.mvsgm.window");
      else System.setProperty("sgm.mvsgm.window", beforeWindow);
    }
  }

  private static void interruptedRun(Path root, Camera[] cams, List<FeatureTracks.Track> tracks, boolean consistencyPhase)
      throws Exception {
    String label = consistencyPhase ? "interrupted-consistency" : "interrupted-fusion";
    var failure = new java.util.concurrent.atomic.AtomicReference<Throwable>();
    Thread worker = new Thread(() -> {
      try {
        dense(root, label, cams, tracks, null, "7", o -> o.diagnostics = false);
      } catch (Throwable e) {
        failure.set(e);
      }
    }, label + "-test");
    worker.start();
    Path dir = root.resolve(label);
    Path watched = dir.resolve(consistencyPhase ? "05_stereo/mvsgm/camera_0.rgb" : "06_fusion/map-spill/camera_0.z");
    long deadline = System.nanoTime() + java.util.concurrent.TimeUnit.SECONDS.toNanos(30);
    boolean started = false;
    while (worker.isAlive() && System.nanoTime() < deadline) {
      started = Files.exists(watched) && (!consistencyPhase || Arrays.stream(worker.getStackTrace())
          .anyMatch(f -> f.getClassName().equals("sgm.MultiViewSgm") && f.getMethodName().equals("consistentCamera")));
      if (started) break;
      Thread.sleep(1);
    }
    worker.interrupt();
    worker.join(30000);
    check(started && !worker.isAlive(), "interrupted mapped fusion terminates");
    check(failure.get() != null, "interrupted fusion reports failure");
    check(!Files.exists(dir.resolve("06_fusion/map-spill")), "interruption unmaps and removes fusion files");
    try (var files = Files.list(dir.resolve("05_stereo/mvsgm"))) {
      check(files.noneMatch(p -> p.getFileName().toString().startsWith("camera_")),
          "interruption removes source maps");
    }
  }

  /**
   * Streamed previews of a frame wider than 4,096 pixels: full-resolution PFM, PNG downscaled by 2
   * (block means of valid depths), JSON with the scale.
   */
  static void widePreview(Path root) throws Exception {
    int w = 4200, h = 50;
    Path image = root.resolve("wide.tif");
    check(ImageIO.write(new BufferedImage(w, h, BufferedImage.TYPE_INT_RGB), "TIFF", image.toFile()), "wide image written");
    Options o = TestSupport.options();
    o.output = Files.createDirectories(root.resolve("wide"));
    o.maxDimension = 8192;
    try (ImageFrame f = new ImageFrame(7, image, new Calibration(1000, 1000, w / 2.0, h / 2.0, new double[5], w, h), o, true);
        MappedFiles mapped = new MappedFiles()) {
      check(f.width == w && f.height == h, "wide frame size");
      float[] z = new float[w * h];
      for (int y = 0; y < h; y++) for (int x = 0; x < w; x++) z[y * w + x] = x % 2 == 0 ? 10 + x / 1000f : 0;
      Path depth = o.output.resolve("camera_7.depth");
      DepthMapFile.write(depth, f.camera, w, h, z);
      new MultiViewSgm(o).exportDepthStreamed(f, "depth", mapped.map(depth, DepthMapFile.HEADER, 4L * w * h, false), null, 10, 15);
      Path dir = o.output.resolve("05_stereo/mvsgm/image_7_wide");
      check(Files.size(dir.resolve("depth.pfm")) == ("Pf\n" + w + " " + h + "\n-1.0\n").length() + 4L * w * h, "wide PFM at full resolution");
      org.opencv.core.Mat png = org.opencv.imgcodecs.Imgcodecs.imread(dir.resolve("depth.png").toString(), org.opencv.imgcodecs.Imgcodecs.IMREAD_GRAYSCALE);
      try {
        check(png.cols() == w / 2 && png.rows() == h / 2, "wide PNG downscaled by 2: " + png.cols() + "x" + png.rows());
        // Block (2x, 2y) holds one valid depth 10 + 2x/1000 (odd columns are empty).
        double expected = 1 + 254 * ((10 + 2 * 1500 / 1000f) - 10) / 5.0;
        check(Math.abs(png.get(5, 1500)[0] - Math.floor(expected)) <= 1, "block mean of valid depths: " + png.get(5, 1500)[0]);
      } finally {
        png.release();
      }
      check(Files.readString(dir.resolve("depth.json")).contains("\"preview_scale\":2"), "wide preview scale recorded");
    }
  }

  record Run(Path dir, long points, byte[] las) {}

  /** Tracks: a grid of points on the true surfaces, observed by every camera that sees them. */
  static List<FeatureTracks.Track> tracks(Camera[] cams) {
    List<FeatureTracks.Track> out = new ArrayList<>();
    for (double x = -7; x <= 7; x += .5)
      for (double y = -5; y <= 5; y += .5) {
        double z = MultiViewSgmRegression.inNear(x, y) ? MultiViewSgmRegression.NEAR : MultiViewSgmRegression.FAR;
        FeatureTracks.Track t = new FeatureTracks.Track();
        t.point = new double[] {x, y, z};
        for (int i = 0; i < cams.length; i++) {
          double[] v = cams[i].view(t.point);
          double u = cams[i].fx * v[0] / v[2] + cams[i].cx, w = cams[i].fy * v[1] / v[2] + cams[i].cy;
          if (v[2] > 0 && u >= 0 && w >= 0 && u < W && w < H) t.observations.add(new FeatureTracks.Observation(i, out.size()));
        }
        if (t.observations.size() >= 2) out.add(t);
      }
    return out;
  }

  static Run dense(Path root, String label, Camera[] cams, List<FeatureTracks.Track> tracks, String tile, String rows)
      throws Exception {
    return dense(root, label, cams, tracks, tile, rows, o -> {});
  }

  static Run dense(Path root, String label, Camera[] cams, List<FeatureTracks.Track> tracks, String tile, String rows,
      java.util.function.Consumer<Options> configure) throws Exception {
    Options o = TestSupport.options();
    o.output = Files.createDirectories(root.resolve(label));
    o.projectDirectory = o.output;
    o.maxDimension = 2048;
    o.denseMethod = "mvsgm";
    o.fusionMode = "image";
    o.fusionStrategy = "graph-average";
    o.fusionOutliers = "robust";
    o.depthOutlierFilter = "edge-aware";
    o.pointAttributes = "surface";
    o.speckleSize = 20;
    o.minViews = 2;
    o.cores = 2;
    o.step = 1;
    configure.accept(o);
    Calibration calibration = new Calibration(600, 600, W / 2.0, H / 2.0, new double[5], W, H);
    List<ImageFrame> frames = new ArrayList<>();
    if (tile != null) System.setProperty("sgm.mvsgm.tile", tile);
    if (rows != null) System.setProperty("sgm.fusion.tileRows", rows);
    try {
      for (int i = 0; i < cams.length; i++) {
        ImageFrame f = new ImageFrame(i, root.resolve("images/" + i + ".tif"), calibration, o, true);
        check(f.width == W && f.height == H && f.camera.fx == 600 && f.camera.cx == W / 2.0, "fixture camera intrinsics");
        f.camera.rotation = cams[i].rotation.clone();
        f.camera.translation = cams[i].translation.clone();
        f.camera.registered = true;
        frames.add(f);
      }
      long points = new DenseReconstruction(o, frames, null, null, tracks).run(List.of());
      return new Run(o.output, points, Files.readAllBytes(o.output.resolve("dense.las")));
    } finally {
      System.clearProperty("sgm.mvsgm.tile");
      System.clearProperty("sgm.fusion.tileRows");
      for (ImageFrame f : frames) f.close();
    }
  }

  /** Equal LAS files apart from the header's creation day and year (bytes 90-93). */
  static boolean sameLas(byte[] a, byte[] b) {
    if (a.length != b.length) return false;
    for (int i = 0; i < a.length; i++) if (a[i] != b[i] && (i < 90 || i > 93)) return false;
    return true;
  }

  /** Fraction of LAS points within 0.5 of the near (z = 44) or far (z = 50) plane. */
  static double onPlanes(byte[] las) {
    var b = java.nio.ByteBuffer.wrap(las).order(java.nio.ByteOrder.LITTLE_ENDIAN);
    int offset = b.getInt(96), record = Short.toUnsignedInt(b.getShort(105));
    long n = Integer.toUnsignedLong(b.getInt(107));
    double scale = b.getDouble(147), shift = b.getDouble(171);
    long good = 0;
    for (long i = 0; i < n; i++) {
      double z = b.getInt((int) (offset + i * record + 8)) * scale + shift;
      if (Math.abs(z - MultiViewSgmRegression.NEAR) < .5 || Math.abs(z - MultiViewSgmRegression.FAR) < .5) good++;
    }
    return n == 0 ? 0 : good / (double) n;
  }

  static void delete(Path root) {
    try (var walk = Files.walk(root)) {
      walk.sorted(Comparator.reverseOrder()).forEach(p -> {
        try {
          Files.deleteIfExists(p);
        } catch (java.io.IOException ignored) {
        }
      });
    } catch (java.io.IOException ignored) {
    }
  }
}
