package sgm;

import java.io.*;
import java.nio.*;
import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;

/** Analytical planes, independently rendered views, provenance and thin foreground controls. */
public final class MultiViewDisparityRegression {
  private static int checks;
  private static final int W = 96, H = 80, RECORD = 59;

  private static void check(boolean value, String text) {
    checks++;
    if (!value) throw new AssertionError(text);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Core.setNumThreads(1);
    Path root = Files.createTempDirectory(Path.of("target"), "multiview-disparity-");
    System.out.println("PASS multiview disparity checks=" + run(root) + "; fixtures=" + root);
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Files.createDirectories(root);
    Options o = TestSupport.options();
    o.output = root;
    o.quality = "ultra";
    o.step = 1;
    o.pyramidLevels = 1;
    o.fusionMode = "image";
    o.minAngle = 1;
    o.twoViewConfidenceThreshold = 1;
    o.disparityRefinement = "multiview";
    check(new Options().disparityRefinement.equals("off"), "Refinement is opt-in");
    check(
        !MultiViewDisparityRefinement.reliable(12, .1f, .8f, 4),
        "Wide baseline alone cannot certify a poor match");
    check(
        !MultiViewDisparityRefinement.reliable(2, .9f, .9f, 4),
        "Strong photometry alone cannot certify weak depth geometry");
    check(
        MultiViewDisparityRefinement.reliable(8, .8f, .7f, 4),
        "Both precision and matching evidence protect reliable measurements");
    for (String[] args :
        new String[][] {
          {"--disparity-refinement", "wrong"},
          {"--disparity-refinement", "multiview"},
          {"--disparity-anchor-angle", "NaN"},
          {"--disparity-refinement-memory-mb", "0"}
        }) {
      boolean rejected = false;
      try {
        Options.parse(args);
      } catch (IllegalArgumentException expected) {
        rejected = true;
      }
      check(rejected, "Unsupported/invalid settings rejected");
    }
    var parsed =
        Options.parse(
            new String[] {
              "-i",
              root.toString(),
              "--disparity-refinement",
              "multiview",
              "--step",
              "1",
              "--pyramid-levels",
              "1",
              "--fusion-mode",
              "image"
            });
    check(parsed.disparityRefinement.equals("multiview"), "Valid mode parsed");
    check(parsed.copy().disparityRefinement.equals("multiview"), "Mode copied between steps");
    check(
        MultiViewDisparityRefinement.bytes(new int[] {10000, 12000}, new int[] {9000, 8000}, 64)
            <= 128L << 20,
        "Bounded anchor grid accounting");
    var frames = frames(root, o);
    try {
      int[][] identities = {{0, 1}, {2, 3}, {4, 5}};
      Path input = root.resolve("candidates.bin");
      long[] starts = stream(input, frames, false);
      byte[] original = Files.readAllBytes(input);
      var refinement = new MultiViewDisparityRefinement(o, frames, identities, 4, 8L << 20);
      refinement.anchors(input, starts);
      var result =
          refinement.refine(
              input, starts, (a, b, p) -> rectified(frames.get(a), frames.get(b)), null);
      check(result.count() == Files.size(input) / RECORD, "Compacted record count exact");
      byte[] after = Files.readAllBytes(input);
      check(
          Arrays.equals(
              Arrays.copyOf(original, (int) starts[2] * RECORD),
              Arrays.copyOf(after, (int) starts[2] * RECORD)),
          "Reliable pair records stay bitwise unchanged");
      int wire = 0, corrected = 0, weak = 0;
      double squared = 0;
      for (int pos = 0; pos < after.length; pos += RECORD) {
        ByteBuffer row = ByteBuffer.wrap(after, pos, RECORD).slice();
        double x = row.getDouble(), y = row.getDouble(), z = row.getDouble();
        row.position(27);
        int pair = row.getInt(), pixel = row.getInt();
        if (pair != 2) continue;
        weak++;
        if (pixel / W == 40) {
          wire++;
          check(Math.abs(z - 4) < 1e-10, "Thin foreground retained at its own depth");
        } else {
          double error = Math.abs(z - 5);
          squared += error * error;
          if (error < .03) corrected++;
        }
      }
      check(wire == 56, "Thin line including endpoints retained");
      check(
          corrected > 300, "Weak plane disparities recovered using independently rendered images");
      check(
          ((Number) result.report().get("refined")).longValue() > 300,
          "Actual refinement, not just rejection");
      check(
          Math.sqrt(squared / Math.max(1, weak - wire)) < .35,
          "Plane depth error improves from 0.556");
      Path swapped = root.resolve("swapped.bin");
      Files.write(swapped, original);
      var reversed =
          new MultiViewDisparityRefinement(
              o, frames, new int[][] {{0, 1}, {2, 3}, {5, 4}}, 4, 8L << 20);
      reversed.anchors(swapped, starts);
      reversed.refine(
          swapped,
          starts,
          (a, b, p) ->
              p == 2
                  ? rectified(frames.get(4), frames.get(5))
                  : rectified(frames.get(a), frames.get(b)),
          null);
      check(
          Arrays.equals(after, Files.readAllBytes(swapped)),
          "Swapped pair identities use the actual rectified reference pixel frame");
      Path correlated = root.resolve("correlated.bin");
      long[] correlatedStarts = stream(correlated, frames, true);
      byte[] before = Files.readAllBytes(correlated);
      var shared =
          new MultiViewDisparityRefinement(
              o, frames, new int[][] {{0, 1}, {0, 3}, {4, 5}}, 4, 8L << 20);
      shared.anchors(correlated, correlatedStarts);
      var no =
          shared.refine(
              correlated,
              correlatedStarts,
              (a, b, p) -> rectified(frames.get(a), frames.get(b)),
              null);
      check(
          ((Number) no.report().get("refined")).longValue() == 0,
          "Shared anchor camera cannot create independent support");
      check(
          Arrays.equals(before, Files.readAllBytes(correlated)),
          "Missing independent evidence preserves all records exactly");
      o.twoViewConfidenceThreshold = Double.NaN;
      Path single = root.resolve("single-direction.bin");
      long[] singleStarts = stream(single, frames, false);
      var singleRefinement = new MultiViewDisparityRefinement(o, frames, identities, 4, 8L << 20);
      singleRefinement.anchors(single, singleStarts);
      var singleResult =
          singleRefinement.refine(
              single, singleStarts, (a, b, p) -> rectified(frames.get(a), frames.get(b)), null);
      check(
          Files.size(single) == singleResult.count() * 55,
          "Single-direction candidate layout remains valid");
      check(
          ((Number) singleResult.report().get("refined")).longValue() > 300,
          "Refinement also works without bidirectional-confidence records");
      check(
          Boolean.FALSE.equals(
              singleResult.report().get("anchor_bidirectional_confidence_available")),
          "Unavailable bidirectional confidence is reported honestly");
      o.twoViewConfidenceThreshold = 1;
      geometry(frames);
      search();
      smoke(root, frames, o);
    } finally {
      for (var f : frames) f.close();
    }
    return checks;
  }

  private static List<ImageFrame> frames(Path root, Options o) throws Exception {
    var calibration = new Calibration(80, 80, 47.5, 39.5, new double[5], W, H);
    double[] centers = {0, 1, .25, 1.25, .5, .75};
    var frames = new ArrayList<ImageFrame>();
    for (int c = 0; c < centers.length; c++) {
      byte[] rgb = new byte[W * H * 3];
      for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
          int coordinate = x + (int) Math.round((y == 40 ? 20 : 16) * centers[c]);
          int value = texture(coordinate, y);
          for (int k = 0; k < 3; k++) rgb[(y * W + x) * 3 + k] = (byte) value;
        }
      Mat image = new Mat(H, W, CvType.CV_8UC3);
      image.put(0, 0, rgb);
      Path path = root.resolve("view" + c + ".png");
      check(Imgcodecs.imwrite(path.toString(), image), "Synthetic view written");
      image.release();
      ImageFrame f = new ImageFrame(c, path, calibration, o);
      f.camera.translation = new double[] {-centers[c], 0, 0};
      f.camera.registered = true;
      frames.add(f);
    }
    return frames;
  }

  private static int texture(int x, int y) {
    int hash = (x + 331) * 0x9e3779b9 ^ (y + 719) * 0x85ebca6b;
    hash ^= hash >>> 16;
    return 30 + Math.floorMod(hash, 190);
  }

  private static long[] stream(Path file, List<ImageFrame> frames, boolean shared)
      throws Exception {
    long[] starts = new long[4];
    long count = 0;
    try (var out = new DataOutputStream(new BufferedOutputStream(Files.newOutputStream(file)))) {
      for (int p = 0; p < 3; p++) {
        starts[p] = count;
        int camera = p == 0 ? 0 : p == 1 ? (shared ? 0 : 2) : 4;
        for (int y = p == 2 ? 18 : 10; y < (p == 2 ? 65 : 70); y++)
          for (int x = p == 2 ? 20 : 10; x < (p == 2 ? 76 : 86); x++) {
            if (p < 2 && y == 40) continue;
            double z = p < 2 ? 5 : y == 40 ? 4 : 80 * .25 / 3.6;
            Camera c = frames.get(camera).camera;
            double[] point = PatchMatchReconstruction.world(c, x, y, z);
            for (double v : point) out.writeDouble(v);
            out.writeByte(128);
            out.writeByte(128);
            out.writeByte(128);
            out.writeInt(p);
            out.writeInt(y * W + x);
            float[] evidence = {.9f, .8f, .8f, .9f, .1f, .9f};
            for (int k = 0; k < (PointConfidence.bidirectional(frames.get(0).options) ? 6 : 5); k++)
              out.writeFloat(evidence[k]);
            count++;
          }
      }
    }
    starts[3] = count;
    return starts;
  }

  private static DenseReconstruction.Rectified rectified(ImageFrame a, ImageFrame b) {
    var r = new DenseReconstruction.Rectified(a, b);
    r.r1 = Math3.identity();
    r.r2 = Math3.identity();
    double baseline = b.camera.center()[0] - a.camera.center()[0];
    r.q = new double[] {1, 0, 0, -47.5, 0, 1, 0, -39.5, 0, 0, 0, 80, 0, 0, 1 / baseline, 0};
    a.gray.copyTo(r.leftGray);
    b.gray.copyTo(r.rightGray);
    r.leftMask.create(H, W, CvType.CV_8U);
    r.rightMask.create(H, W, CvType.CV_8U);
    r.leftMask.setTo(Scalar.all(255));
    r.rightMask.setTo(Scalar.all(255));
    return r;
  }

  private static void geometry(List<ImageFrame> frames) {
    try (var r = rectified(frames.get(4), frames.get(5))) {
      var p =
          new MultiViewDisparityRefinement.Plane(
              .0001, -.0002, .2, 1e-6, new double[] {0, 0, W, 0, W, H, 0, H});
      double[] d = MultiViewDisparityRefinement.disparityPlane(p, frames.get(4).camera, r);
      for (boolean vertical : new boolean[] {false, true}) {
        r.vertical = vertical;
        double[] dp = MultiViewDisparityRefinement.disparityPlane(p, frames.get(4).camera, r);
        for (int y = 15; y < 65; y += 5)
          for (int x = 15; x < 80; x += 5) {
            double disparity = dp[0] * x + dp[1] * y + dp[2];
            double[] world = MultiViewDisparityRefinement.world(r, x, y, disparity),
                view = frames.get(4).camera.view(world),
                uv = frames.get(4).camera.project(world);
            check(
                Math.abs(p.inverse(uv[0], uv[1]) - 1 / view[2]) < 1e-12,
                "Signed/sloped plane and transposed epipolar geometry agree analytically");
          }
      }
    }
  }

  private static void search() {
    int w = 80, h = 40;
    byte[] left = new byte[w * h], right = new byte[w * h], mask = new byte[w * h];
    Arrays.fill(mask, (byte) 255);
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) {
        left[y * w + x] = (byte) texture(x, y);
        right[y * w + x] = (byte) texture(x + 5, y);
      }
    Options o = new Options();
    var found =
        MultiViewDisparityRefinement.search(left, right, mask, mask, w, h, 40, 20, 5, 5.4, o);
    check(
        found != null && Math.abs(found.disparity() - 5) < .03,
        "Photometric search recovers known correspondence");
    check(
        MultiViewDisparityRefinement.foregroundPreferred(left, right, w, h, 40, 20, 5, 2, o),
        "A photometrically preferred foreground cannot be erased by a background prior");
    check(
        !MultiViewDisparityRefinement.foregroundPreferred(left, right, w, h, 40, 20, 5.4, 5, o),
        "A better anchor-supported correspondence is eligible for refinement");
    byte[] blank = new byte[w * h];
    Arrays.fill(blank, (byte) 128);
    check(
        MultiViewDisparityRefinement.search(blank, blank, mask, mask, w, h, 40, 20, 5, 5.4, o)
            == null,
        "No invented correspondence on a textureless surface");
    mask[20 * w + 35] = 0;
    check(
        MultiViewDisparityRefinement.search(left, right, mask, mask, w, h, 40, 20, 5, 5.4, o)
            == null,
        "Masked correspondence cannot be recovered");
  }

  private static void smoke(Path root, List<ImageFrame> frames, Options o) throws Exception {
    o.cores = 1;
    o.diagnostics = false;
    o.cache = false;
    o.census = 3;
    o.disparities = 24;
    o.disparitiesExplicit = true;
    o.step = 1;
    o.compute = "cpu";
    var pairs =
        List.of(
            new FeatureTracks.Pair(0, 1),
            new FeatureTracks.Pair(2, 3),
            new FeatureTracks.Pair(4, 5));
    byte[] baseline = null;
    for (int run = 0; run < 3; run++) {
      o.output = Files.createDirectories(root.resolve("pipeline-" + run));
      o.disparityRefinement = run == 2 ? "multiview" : "off";
      long count = new DenseReconstruction(o, frames).run(pairs);
      check(count > 0, "Normal reconstruction pipeline produces points");
      if (run == 0) baseline = Files.readAllBytes(o.output.resolve("dense.ply"));
      if (run == 1)
        check(
            Arrays.equals(baseline, Files.readAllBytes(o.output.resolve("dense.ply"))),
            "Disabled mode reproduces the existing cloud bitwise");
      check(
          Files.exists(o.output.resolve("05_stereo/disparity_refinement.json")) == (run == 2),
          "Refinement hook/report enabled only on request");
    }
    Path camera = root.resolve("known-camera.txt"),
        poses = root.resolve("known-poses.csv"),
        project = root.resolve("cli-project");
    Files.writeString(camera, "width=96\nheight=80\nfx=80\nfy=80\ncx=47.5\ncy=39.5\n");
    PoseIO.write(poses, frames);
    int status =
        Main.run(
            new String[] {
              "-i",
              root.toString(),
              "-o",
              project.toString(),
              "--calibration",
              camera.toString(),
              "--poses",
              poses.toString(),
              "--intrinsics-mode",
              "fixed",
              "--extrinsics-mode",
              "fixed",
              "--quality",
              "ultra",
              "--compute",
              "cpu",
              "--cores",
              "1",
              "--memory-mb",
              "512",
              "--fusion-mode",
              "image",
              "--surface-consolidation",
              "none",
              "--point-attributes",
              "none",
              "--step",
              "1",
              "--pyramid-levels",
              "1",
              "--disparities",
              "32",
              "--disparity-refinement",
              "multiview",
              "--two-view-confidence-threshold",
              "1.0"
            });
    check(status == 0, "Enabled mode works through the normal CLI project lifecycle");
    Path run;
    try (var runs = Files.list(project.resolve("runs"))) {
      run = runs.findFirst().orElseThrow();
    }
    check(
        PipelineState.load(run).status(Pipeline.Step.DENSE).equals("complete"),
        "CLI dense state completed");
    check(
        Files.isRegularFile(run.resolve("05_stereo/disparity_refinement.json")),
        "CLI refinement report published");
  }
}
