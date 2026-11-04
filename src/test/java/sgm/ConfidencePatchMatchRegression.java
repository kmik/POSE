package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import java.util.zip.GZIPInputStream;

/** Independent scene controls for bounded confidence, free space and plane-hypothesis stereo. */
final class ConfidencePatchMatchRegression {
  private static int checks;

  private static List<GpuSgm.Device> devices() {
    try {
      return GpuSgm.devices();
    } catch (RuntimeException unavailable) {
      return List.of();
    }
  }

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Path root = Files.createTempDirectory(Path.of("target"), "confidence-patchmatch-");
    System.out.println("PASS confidence/PatchMatch: " + run(root) + " checks; " + root);
    GpuSgm.closeSession();
  }

  static int run(Path root) throws Exception {
    checks = 0;
    for (String value : List.of("-0.01", "1.01", "NaN", "Infinity", "-Infinity")) {
      boolean rejected = false;
      try {
        Options.parse(new String[] {"-i", root.toString(), "--confidence-threshold", value});
      } catch (IllegalArgumentException expected) {
        rejected = true;
      }
      check(rejected, "Invalid confidence threshold: " + value);
    }
    for (double t : new double[] {0, .4, 1}) {
      var o =
          Options.parse(
              new String[] {"-i", root.toString(), "--confidence-threshold", Double.toString(t)});
      check(o.copy().confidenceThreshold == t, "Threshold option copy");
      check(PointConfidence.passes(t, t), "Threshold equality is inclusive");
    }
    check(
        PointConfidence.passes(Double.NaN, 0) && !PointConfidence.passes(Double.NaN, .01),
        "Unknown score policy");
    check(!PointConfidence.passes(.399, .4), "Strictly below threshold removed");
    check(PointConfidence.score(1, 1, 1, 0) == 1, "Ideal matching score endpoint");
    check(PointConfidence.score(1, 0, 1, 0) == 0, "Ambiguity lowers confidence");
    check(
        PointConfidence.score(1, 1, 1, .5) < PointConfidence.score(1, 1, 1, .1),
        "Reciprocal error lowers confidence");
    check(
        PointConfidence.finalScore(.8, .05) < PointConfidence.finalScore(.8, .01),
        "Weak geometry lowers confidence");
    int previous = 101;
    for (int k = 0; k <= 100; k++) {
      int kept = 0;
      for (int i = 0; i <= 100; i++) if (PointConfidence.passes(i / 100., k / 100.)) kept++;
      check(kept <= previous, "Monotonic fixed-score retention");
      previous = kept;
    }
    for (boolean surface : new boolean[] {false, true}) output(root, surface);
    visibility(root, 5, true, -1);
    visibility(root, 2, false, -1);
    for (int device = 0; device < devices().size(); device++) {
      visibility(root, 5, true, device);
      visibility(root, 2, false, device);
    }
    numerical(root);
    stereoConfidence();
    endToEnd(root);
    // This deliberately ambiguous control blocks default promotion: without reliable third-view
    // depth a two-image false surface is indistinguishable from a real occluded surface.
    AdversarialSceneTest.main(
        new String[] {root.resolve("two-view-phantom").toString(), "observe", "2"});
    check(
        new Options().minViews == 3,
        "Two-view default remains gated by unresolved phantom control");
    return checks;
  }

  private static void output(Path root, boolean surface) throws Exception {
    Options o = TestSupport.options();
    o.output = Files.createDirectories(root.resolve("confidence-" + surface));
    o.surfaceConsolidation = surface ? "normal" : "none";
    o.confidenceThreshold = .6;
    o.pointAttributes = "none";
    Camera c = new Camera(100, 100, 20, 20);
    c.registered = true;
    Camera[] cameras = new Camera[80];
    int[] sizes = new int[80];
    Arrays.fill(sizes, 40);
    for (int i = 0; i < cameras.length; i++) cameras[i] = c.copy();
    try (var fusion = new DepthFusion(cameras, sizes, sizes, o);
        var out = new FusionOutput(o, fusion, null, null)) {
      fusion.addPair(0, 79);
      for (int i = 0; i < 4; i++) {
        var r = new DepthFusion.Result(new double[] {i * 10, 0, 5}, 2, 0, .02, 0, "accepted");
        r.matchingConfidence = i == 0 ? 1 : i == 1 ? .8 : i == 2 ? .7 : Double.NaN;
        r.supportingCameras = new BitSet();
        r.supportingCameras.set(0);
        r.supportingCameras.set(79);
        r.costMargin = .9;
        r.costSharpness = .8;
        r.photometric = .99;
        r.lrError = .1;
        var sample =
            new SurfaceConsolidator.Sample(
                i, 0, 0, 0, r, new DepthFusion.SurfaceSample(null, .01, .01));
        var bytes = new ByteArrayOutputStream();
        SurfaceConsolidator.write(new DataOutputStream(bytes), sample);
        var restored =
            SurfaceConsolidator.read(
                new DataInputStream(new ByteArrayInputStream(bytes.toByteArray())));
        check(
            Math.abs(restored.matchingConfidence - r.matchingConfidence) < 1e-6 || i == 3,
            "Confidence surface storage round trip");
        check(
            restored.supportingCameras.equals(r.supportingCameras),
            "Supporting camera IDs survive surface storage");
        out.add(i, 0, 0, 0, r, null);
      }
      out.finish();
      check(
          out.count() == 2 && out.confidenceRejected() == 2,
          "Final threshold identical with/without consolidation");
    }
    try (var in =
        new BufferedReader(
            new InputStreamReader(
                new GZIPInputStream(
                    Files.newInputStream(
                        o.output.resolve("07_results/point-confidence.csv.gz")))))) {
      String header = in.readLine();
      check(header.contains(",confidence,matching_confidence,"), "Confidence columns named");
      int rows = 0, fields = header.split(",").length;
      for (String line; (line = in.readLine()) != null; ) {
        check(line.split(",").length == fields, "Confidence row/cloud schema aligned");
        check(
            line.endsWith(",0;79"),
            "Supporting camera IDs beyond 64 are exported without truncation");
        rows++;
      }
      check(rows == 2, "One confidence row per final point");
    }
    check(
        Files.readString(o.output.resolve("06_fusion/confidence.json"))
            .contains(PointConfidence.MODEL),
        "Score version archived");
  }

  private static void visibility(Path root, double depth, boolean reject, int device)
      throws Exception {
    Options o = TestSupport.options();
    o.output = Files.createDirectories(root.resolve("visibility-" + depth + "-" + device));
    o.minViews = 2;
    o.depthRefinement = false;
    o.fusionCompute = device < 0 ? "cpu" : "gpu";
    o.gpuDevice = Math.max(0, device);
    Camera[] c = new Camera[4];
    for (int i = 0; i < 4; i++) {
      c[i] = new Camera(100, 100, 39.5, 29.5);
      c[i].translation = new double[] {-i * .2, 0, 0};
      c[i].registered = true;
    }
    try (var fusion =
        new DepthFusion(c, new int[] {80, 80, 80, 80}, new int[] {60, 60, 60, 60}, o)) {
      int origin = fusion.addPair(0, 1), a = fusion.addPair(2, 3), b = fusion.addPair(2, 3);
      double[] candidate = {.017, .011, 3};
      fusion.add(origin, candidate);
      int reverse = fusion.addPair(1, 0);
      fusion.add(reverse, candidate);
      for (int y = 0; y < 60; y++)
        for (int x = 0; x < 80; x++) {
          double[] p = PatchMatchReconstruction.world(c[2], x, y, depth);
          fusion.add(a, p);
          fusion.add(b, p);
        }
      var result = fusion.fuse(origin, candidate);
      check(
          result.independentPairs == 0,
          "Reverse map from producing cameras is not independent support");
      check(
          reject ? result.status.equals("visibility_contradiction_rejected") : result.accepted(),
          reject ? "Reliable free space contradicts floating point" : "Nearer occluder is neutral");
      fusion.prepare(1);
      var batch = fusion.fuseBatch(new int[] {origin}, new double[][] {candidate}, 1);
      check(batch[0].status.equals(result.status), "Scalar/batch visibility semantics match");
      if (!reject)
        check(
            batch[0].supportingCameras.equals(result.supportingCameras)
                && result.supportingCameras.cardinality() == result.views,
            "CPU/GPU verified camera identities agree");
    }
  }

  static PatchMatch.Scene planeScene(int views) {
    int w = 96, h = 64, n = w * h;
    byte[] pixels = new byte[n * views], mask = new byte[n * views];
    Arrays.fill(mask, (byte) 255);
    byte[] texture = new byte[(w + 32) * h];
    new Random(613).nextBytes(texture);
    float[] c = new float[views * 20];
    for (int v = 0; v < views; v++) {
      for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) pixels[v * n + y * w + x] = texture[y * (w + 32) + x + 6 * v];
      int at = v * 20;
      c[at] = c[at + 4] = c[at + 8] = 1;
      c[at + 9] = -v * .25f;
      c[at + 12] = c[at + 13] = 96;
      c[at + 14] = 47.5f;
      c[at + 15] = 31.5f;
      c[at + 16] = w;
      c[at + 17] = h;
      c[at + 18] = v * n / 65536;
      c[at + 19] = v * n % 65536;
    }
    return new PatchMatch.Scene(w, h, pixels, mask, c, views, 3, 5);
  }

  private static void numerical(Path root) throws Exception {
    Options o = TestSupport.options();
    o.compute = "cpu";
    o.patchmatchIterations = 6;
    var scene = planeScene(3);
    check(
        PatchMatch.cost(scene, 48, 32, new float[] {0, 0, 1, 4}, 1) < 1e-12,
        "Independent 4-unit plane has zero photometric cost");
    check(
        PatchMatch.cost(scene, 48, 32, new float[] {0, 0, 1, 3.3f}, 1) > .2,
        "Wrong depth has larger cost");
    var cpu = PatchMatch.compute(scene, o, null);
    metrics(cpu, "CPU");
    check(
        Json.string(cpu.backend()).contains("cpu_patchmatch"),
        "Explicit CPU reference backend reported");
    o.output = Files.createDirectories(root.resolve("patchmatch-cache"));
    o.cacheDirectory = o.output.resolve("cache");
    o.denseMethod = "patchmatch";
    o.resume = true;
    var first = ReuseCache.patchmatch(o, scene, null);
    var second = ReuseCache.patchmatch(o, scene, null);
    check(Arrays.equals(first.data(), second.data()), "PatchMatch cache exact round trip");
    check(Json.string(second.backend()).contains("cache"), "PatchMatch cache reuse reported");
    if (!devices().isEmpty()) {
      o.compute = "gpu";
      for (int device = 0; device < devices().size(); device++) {
        o.gpuDevice = device;
        var gpu = PatchMatch.compute(scene, o, null);
        metrics(gpu, "GPU " + device);
        check(
            Json.string(gpu.backend()).contains("opencl_patchmatch"),
            "Actual GPU PatchMatch reported");
        double disagreement = 0;
        int count = 0;
        for (int y = 8; y < 56; y++)
          for (int x = 24; x < 80; x++) {
            int i = y * 96 + x;
            if (cpu.photo(i) > .8 && gpu.photo(i) > .8) {
              disagreement += Math.abs(cpu.depth(i) - gpu.depth(i));
              count++;
            }
          }
        check(
            count > 1500 && disagreement / count < .03,
            "CPU/GPU depth quality parity on textured plane");
      }
    }
  }

  private static void stereoConfidence() {
    var scene = planeScene(2);
    int n = scene.width() * scene.height();
    byte[] a = Arrays.copyOfRange(scene.pixels(), 0, n),
        b = Arrays.copyOfRange(scene.pixels(), n, 2 * n);
    Options o = TestSupport.options();
    o.disparities = 16;
    o.median = 1;
    o.speckleSize = 0;
    var cpu = new CensusSgm(o).compute(a, b, scene.width(), scene.height());
    PointConfidence.stereo(cpu, a, b);
    int finite = 0;
    for (float value : cpu.confidence) if (Float.isFinite(value)) finite++;
    check(finite > 3000, "Production aggregate/photometric confidence available");
    for (int device = 0; device < devices().size(); device++)
      for (String method : List.of("resident", "streamed", "esgm")) {
        Options gpuOptions = o.copy();
        gpuOptions.compute = "gpu";
        gpuOptions.gpuDevice = device;
        gpuOptions.gpuSgm = method;
        var gpu = new CensusSgm(gpuOptions).compute(a, b, scene.width(), scene.height());
        PointConfidence.stereo(gpu, a, b);
        if (!method.equals("esgm")) {
          check(
              Arrays.equals(cpu.costMargin, gpu.costMargin),
              "CPU/GPU exact aggregate margin: " + method);
          check(
              Arrays.equals(cpu.costSharpness, gpu.costSharpness),
              "CPU/GPU aggregate sharpness: " + method);
          check(
              Arrays.equals(cpu.confidence, gpu.confidence),
              "CPU/GPU confidence decisions: " + method);
        } else {
          boolean conservative = true;
          for (int i = 0; i < n; i++)
            if (Float.isFinite(gpu.confidence[i])
                && Float.isFinite(cpu.confidence[i])
                && gpu.confidence[i] > cpu.confidence[i] + 1e-6) conservative = false;
          check(
              conservative,
              "Compact certified confidence never overstates omitted runner-up margin");
        }
      }
  }

  private static void metrics(PatchMatch.Map map, String label) {
    int good = 0, wrong = 0;
    double error = 0;
    for (int y = 8; y < 56; y++)
      for (int x = 24; x < 80; x++) {
        int i = y * 96 + x;
        if (map.photo(i) > .8) {
          good++;
          double e = Math.abs(map.depth(i) - 4);
          error += e;
          if (e > .08) wrong++;
        }
      }
    System.out.println(
        label
            + " plane: good="
            + good
            + " wrong="
            + wrong
            + " mean error="
            + error / Math.max(1, good));
    check(
        good > 2000 && wrong < good * .01 && error / good < .025,
        label + " known planar depth and coverage");
  }

  private static void endToEnd(Path root) throws Exception {
    var scene = planeScene(3);
    Path dir = Files.createDirectories(root.resolve("end-to-end"));
    Path images = Files.createDirectories(dir.resolve("images"));
    for (int v = 0; v < 3; v++) {
      var image = new org.opencv.core.Mat(64, 96, org.opencv.core.CvType.CV_8UC1);
      try {
        image.put(0, 0, Arrays.copyOfRange(scene.pixels(), v * 96 * 64, (v + 1) * 96 * 64));
        org.opencv.imgcodecs.Imgcodecs.imwrite(images.resolve(v + ".png").toString(), image);
      } finally {
        image.release();
      }
    }
    Path calibration = dir.resolve("camera.txt"), poses = dir.resolve("poses.csv");
    Files.writeString(calibration, "width=96\nheight=64\nfx=96\nfy=96\ncx=47.5\ncy=31.5\n");
    StringBuilder pose = new StringBuilder(PoseIO.HEADER + "\n");
    for (int v = 0; v < 3; v++) pose.append(v + ".png,1,0,0,0,1,0,0,0,1," + (-v * .25) + ",0,0\n");
    Files.writeString(poses, pose);
    for (String method : List.of("sgm", "patchmatch")) {
      Path out = dir.resolve(method);
      int status =
          TestSupport.run(
              new String[] {
                "-i",
                images.toString(),
                "-o",
                out.toString(),
                "--calibration",
                calibration.toString(),
                "--poses",
                poses.toString(),
                "--no-gps",
                "--dense-method",
                method,
                "--patchmatch-min-depth",
                "3",
                "--patchmatch-max-depth",
                "5",
                "--patchmatch-iterations",
                "6",
                "--patchmatch-levels",
                "2",
                "--min-views",
                "2",
                "--disparities",
                "24",
                "--step",
                "2",
                "--confidence-threshold",
                "0.2",
                "--speckle-size",
                "0",
                "--storage",
                "minimal",
                "--compute",
                devices().isEmpty() ? "cpu" : "gpu"
              });
      check(status == 0, "End-to-end " + method + " pipeline completes");
      byte[] bytes = Files.readAllBytes(out.resolve("dense.las"));
      var b = java.nio.ByteBuffer.wrap(bytes).order(java.nio.ByteOrder.LITTLE_ENDIAN);
      int n = b.getInt(107),
          offset = b.getInt(96),
          stride = Short.toUnsignedInt(b.getShort(105)),
          wrong = 0;
      for (int i = 0; i < n; i++) {
        double z = b.getInt(offset + i * stride + 8) * b.getDouble(147) + b.getDouble(171);
        if (Math.abs(z - 4) > .1) wrong++;
      }
      System.out.println(method + " final plane: points=" + n + " wrong=" + wrong);
      check(n > 900 && wrong < n * .01, "End-to-end " + method + " known-depth cloud quality");
      if (method.equals("patchmatch"))
        try (var paths = Files.walk(out.resolve("runs"))) {
          check(
              paths.noneMatch(p -> p.getFileName().toString().equals("depth-normal-evidence.bin")),
              "Minimal storage removes temporary PatchMatch maps after reciprocal evidence");
        }
    }
    Path limited = dir.resolve("patchmatch-max-depth");
    int limitedStatus =
        TestSupport.run(
            new String[] {
              "-i",
              images.toString(),
              "-o",
              limited.toString(),
              "--calibration",
              calibration.toString(),
              "--poses",
              poses.toString(),
              "--no-gps",
              "--dense-method",
              "patchmatch",
              "--patchmatch-min-depth",
              "3",
              "--patchmatch-max-depth",
              "5",
              "--patchmatch-levels",
              "2",
              "--patchmatch-iterations",
              "2",
              "--max-depth",
              "1",
              "--storage",
              "minimal",
              "--report",
              "none",
              "--compute",
              devices().isEmpty() ? "cpu" : "gpu"
            });
    check(
        limitedStatus != 0 && !Files.exists(limited.resolve("dense.las")),
        "PatchMatch respects the common maximum depth and publishes no out-of-range cloud");
    Options failed = TestSupport.options();
    failed.maxDimension = 96;
    failed.output = Files.createDirectories(dir.resolve("unregistered-view"));
    List<ImageFrame> frames = new ArrayList<>();
    try {
      Calibration c = new Calibration(96, 96, 47.5, 31.5, new double[5], 96, 64);
      for (int i = 0; i < 3; i++) {
        var f = new ImageFrame(i, images.resolve(i + ".png"), c, failed, true);
        f.camera.registered = i < 2;
        f.camera.translation = new double[] {-i * .25, 0, 0};
        frames.add(f);
      }
      try (var fusion =
          new DepthFusion(
              frames.stream().map(f -> f.camera).toArray(Camera[]::new),
              new int[] {96, 96, 96},
              new int[] {64, 64, 64},
              failed)) {
        new PatchMatchReconstruction(failed, frames, List.of(), fusion, null)
            .run(
                List.of(new FeatureTracks.Pair(0, 2)),
                new DataOutputStream(new ByteArrayOutputStream()));
        check(fusion.pairCount() == 0, "Failed cameras do not allocate unused PatchMatch evidence");
      }
    } finally {
      for (var f : frames) f.close();
    }
  }
}
