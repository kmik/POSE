package sgm;

import java.io.*;
import java.lang.foreign.*;
import java.nio.file.*;
import java.util.*;

/** Small CPU controls for the interrupted revision; GPU admission/parity remain separate tests. */
public final class RevisionControls {
  private static long checks;

  private static void check(boolean value, String description) {
    checks++;
    if (!value) throw new AssertionError(description);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    org.opencv.core.Core.setNumThreads(1);
    arguments();
    cameraBounds();
    preparedRetries();
    Path root = Files.createDirectories(Path.of("target/claude-speed/controls/association"));
    check(association(root.resolve("pair"), "pair") == 2, "Reference owners retain two candidates");
    check(association(root.resolve("camera"), "camera") == 1, "Shared camera validates one compatible representative");
    if (args.length > 0 && args[0].equals("gpu")) {
      for (String evidence : new String[] {"pair", "camera"})
        for (String storage : new String[] {"resident", "windowed"})
          scopedGpu(root.resolve("scoped-" + evidence + "-" + storage), evidence, storage);
    }
    System.out.println("PASS isolated revision "
        + (args.length > 0 && args[0].equals("gpu") ? "CPU/GPU" : "CPU")
        + " controls: " + checks);
  }

  private static void arguments() {
    check(GpuSgm.Compute.sameArgument(7, 7), "Equal integer arguments cached");
    check(!GpuSgm.Compute.sameArgument(7, 8), "Changed integer argument sent");
    check(!GpuSgm.Compute.sameArgument(0f, -0f), "Signed float zero is significant");
    check(!GpuSgm.Compute.sameArgument(Float.intBitsToFloat(0x7fc00001),
        Float.intBitsToFloat(0x7fc00002)), "NaN payload is significant");
    check(!GpuSgm.Compute.sameArgument(new GpuSgm.Local(8), new GpuSgm.Local(16)),
        "Changed local memory size sent");
    try (Arena arena = Arena.ofConfined()) {
      MemorySegment a = arena.allocate(8), b = arena.allocate(8);
      check(GpuSgm.Compute.sameArgument(a, a), "Same live buffer cached");
      check(!GpuSgm.Compute.sameArgument(a, b), "Different buffer sent");
    }
    Options defaults = new Options();
    check(defaults.fusionMode.equals("reference") && defaults.fusionAssociation.equals("pair")
        && !defaults.fusionCameraCulling && !defaults.stereoPipeline, "Defaults preserved");
  }

  private static void cameraBounds() {
    Random random = new Random(10017);
    for (int trial = 0; trial < 40; trial++) {
      Camera[] cameras = new Camera[10];
      int[] widths = new int[10], heights = new int[10];
      Arrays.fill(widths, 160); Arrays.fill(heights, 120);
      for (int i = 0; i < cameras.length; i++) {
        Camera c = cameras[i] = new Camera(100, 105, 79.5, 59.5);
        c.registered = true;
        if (i > 1) {
          c.rotation = Math3.rotation(new double[] {(random.nextDouble() - .5) * 3,
              (random.nextDouble() - .5) * 3, (random.nextDouble() - .5) * 2});
          c.translation = new double[] {(random.nextDouble() - .5) * 100,
              (random.nextDouble() - .5) * 100, (random.nextDouble() - .5) * 40};
        }
      }
      double[] box = {-2, -3, 8, 4, 2, 15};
      double rel = trial % 2 == 0 ? .005 : .1, scale = (1 + rel) * (1 + rel);
      FusionScope scope = new FusionScope(cameras, widths, heights, new int[][] {{0, 1}},
          Collections.singletonList(box), rel);
      for (int sample = 0; sample < 150; sample++) {
        double[] p = new double[3];
        for (int a = 0; a < 3; a++) p[a] = box[a] + random.nextDouble() * (box[a + 3] - box[a]);
        check(scope.contains(0, p), "Input domain retained");
        double factor = 1 / scale + random.nextDouble() * (scale - 1 / scale);
        for (int image = 0; image < cameras.length; image++) {
          double[] refined = Math3.times(p, factor);
          double[] uv = cameras[image].project(refined);
          if (uv != null && uv[0] >= 0 && uv[1] >= 0 && uv[0] <= 159 && uv[1] <= 119)
            check(scope.keeps(0, image), "No potentially observing camera culled");
        }
      }
      check(!scope.contains(0, new double[] {100, 100, 10}), "Moved query requests full replay");
    }
    Camera c = new Camera(100, 100, 79.5, 59.5);
    check(FusionScope.intersects(c, 160, 120, new double[] {-1, -1, -1, 1, 1, 1}),
        "Near-plane crossing is uncertain");
    check(!FusionScope.intersects(c, 160, 120, new double[] {-1, -1, -3, 1, 1, -1}),
        "Entirely behind camera");
    check(!FusionScope.intersects(c, 160, 120, new double[] {100, 100, 8, 110, 110, 10}),
        "Far outside frustum");
  }

  private static void preparedRetries() throws Exception {
    Options o = new Options();
    o.compute = "cpu"; o.cores = 2; o.speckleSize = 0;
    int w = 73, h = 49;
    byte[] a = new byte[w * h], b = new byte[w * h];
    new Random(717).nextBytes(a);
    for (int y = 0; y < h; y++) for (int x = 0; x < w; x++)
      b[y * w + x] = a[y * w + Math.min(w - 1, x + 4)];
    var prepared = CensusSgm.prepare(o, a, b, null, null, w, h, null, null);
    for (int d : new int[] {9, 13, 21}) {
      o.disparities = d;
      var fresh = new CensusSgm(o).compute(a, b, w, h);
      CensusSgm.Result reused;
      try (var scope = CensusSgm.use(prepared)) { reused = new CensusSgm(o).compute(a, b, w, h); }
      for (int p = 0; p < w * h; p++)
        check(Float.floatToRawIntBits(fresh.disparity[p]) == Float.floatToRawIntBits(reused.disparity[p]),
            "Fresh/reused descriptors across expanded ranges");
    }
  }

  private static long association(Path root, String mode) throws Exception {
    Options o = BoundedFusionRegression.options(root);
    o.fusionMode = "incremental"; o.fusionAssociation = mode;
    o.twoViewConfidenceThreshold = .85;
    Camera[] cameras = BoundedFusionRegression.cameras(3);
    for (Camera camera : cameras) camera.translation[0] *= 2;
    int[] w = {160, 160, 160}, h = {120, 120, 120};
    try (var fusion = new DepthFusion(cameras, w, h, o);
        var provisional = new IncrementalFusion(o, fusion)) {
      int ab = fusion.addPair(0, 1), bc = fusion.addPair(1, 2);
      double[] point = {.013, .017, 10};
      fusion.add(ab, point);
      try (var stream = provisional.stream()) {
        BoundedFusionRegression.observation(stream, ab, 1, point, .9f);
        BoundedFusionRegression.observation(stream, bc, 2, point, .8f);
      }
      fusion.add(bc, point);
      fusion.prepare(2);
      Path result = root.resolve("prepared.bin");
      long count = provisional.finish(root.resolve("candidates.bin"), result);
      try (var in = new DataInputStream(Files.newInputStream(result))) {
        for (int i = 0; i < count; i++) {
          in.readLong();
          var r = IncrementalFusion.readResult(in);
          check(r.accepted() && r.views == 3 && r.supportingCameras.cardinality() == 3,
              "Ownership never fabricates support; all evidence considered");
        }
      }
      return count;
    }
  }

  private static void scopedGpu(Path root, String evidence, String storage) throws Exception {
    Options o = BoundedFusionRegression.options(root);
    o.fusionCompute = "gpu";
    o.fusionEvidence = evidence;
    o.fusionGpuEvidence = storage;
    o.fusionWindowMb = storage.equals("windowed") ? 2 : 0;
    o.fusionCameraCulling = true;
    Camera[] cameras = new Camera[6];
    int[] w = new int[6], h = new int[6];
    Arrays.fill(w, 160); Arrays.fill(h, 120);
    for (int image = 0; image < 6; image++) {
      cameras[image] = new Camera(100, 100, 79.5, 59.5);
      cameras[image].registered = true;
      cameras[image].translation[0] = -(image < 3 ? (image - 1) * .6 : 100 + (image - 4) * .6);
    }
    try (var fusion = new DepthFusion(cameras, w, h, o)) {
      for (int group = 0; group < 2; group++)
        for (int a = group * 3; a < group * 3 + 3; a++)
          for (int b = a + 1; b < group * 3 + 3; b++) {
            int pair = fusion.addPair(a, b);
            for (int y = -10; y <= 10; y++) for (int x = -10; x <= 10; x++)
              fusion.add(pair, new double[] {group * 100 + x * .02, y * .02, 10});
          }
      fusion.prepare(32);
      int[] source = {0, 3, 0, 3, 0};
      double[][] points = {{.013, .017, 10}, {100.013, .017, 10},
          {-.031, -.019, 10}, {99.969, -.019, 10}, {1, 0, 10}};
      var results = fusion.fuseBatch(source, points, points.length);
      for (int i = 0; i < points.length; i++)
        check(DepthFusion.equivalent(fusion.fuse(source[i], points[i]), results[i]),
            "Scoped GPU preserves CPU decisions, including source switches and moved queries");
      Map<?, ?> report = (Map<?, ?>) fusion.backendReport();
      Map<?, ?> resources = (Map<?, ?>) report.get("gpu_resources");
      check(resources != null, "Required GPU actually executed");
      check(((Number) resources.get("scratch_endpoint_capacity")).intValue()
          < ((Number) resources.get("full_endpoint_count")).intValue(), "Endpoint scratch is compacted");
      check(((Number) resources.get("scope_cpu_replays")).longValue() > 0,
          "Out-of-domain query replays instead of trusting a shortlist");
      Json.write(root.resolve("backend.json"), report);
    }
  }
}
