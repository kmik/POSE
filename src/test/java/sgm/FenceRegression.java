package sgm;

import java.nio.file.*;
import java.util.*;

/** Ray-plane fence renderer with independently specified depths and visibility. */
final class FenceRegression {
  record Scene(
      int w,
      int h,
      int foregroundDisparity,
      int backgroundDisparity,
      byte[] left,
      byte[] right,
      boolean[] fence,
      boolean[] background) {}

  private static int checks;

  private static void check(boolean yes, String message) {
    checks++;
    if (!yes) throw new AssertionError(message);
  }

  static int hash(int x, int y, int seed) {
    int h = x * 374761393 + y * 668265263 + seed * 1442695041;
    h = (h ^ (h >>> 13)) * 1274126177;
    return (h ^ (h >>> 16)) & 0x7fffffff;
  }

  static boolean front(double u, int y, int width) {
    if (u < 60 || u > 282 || y < 15 || y > 113) return false;
    for (int post = 70; post <= 270; post += 50)
      if (u >= post - .5 && u < post + width - .5) return true;
    return y == 42 || y == 86 || Math.abs(y - (20 + .37 * (u - 60))) < .5;
  }

  static Scene scene(int width, int seed, int fd, int bd) {
    int w = 336, h = 128;
    byte[] left = new byte[w * h], right = new byte[left.length];
    boolean[] fence = new boolean[left.length], background = new boolean[left.length];
    for (int view = 0; view < 2; view++)
      for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
          // f*baseline=120. Foreground depth=120/fd; background depth=120/bd.
          // Translate each ray on its own plane. Occlusion uses the first visible hit.
          int fu = x + view * fd, bu = x + view * bd;
          boolean fg = front(fu, y, width);
          (view == 0 ? left : right)[y * w + x] =
              (byte) (fg ? 35 + hash(fu, y, seed) % 55 : 120 + hash(bu, y, seed + 1) % 120);
          if (view == 0) {
            fence[y * w + x] = fg;
            background[y * w + x] =
                !fg
                    && x >= fd + 8
                    && x < w - 12
                    && y >= 10
                    && y < h - 10
                    && !front(x - bd + fd, y, width);
          }
        }
    return new Scene(w, h, fd, bd, left, right, fence, background);
  }

  static Map<String, Object> metrics(Scene s, float[] d) {
    int fg = 0, correct = 0, bump = 0, wrong = 0, bg = 0, bgValid = 0, bgWrong = 0;
    double tolerance = Math.min(.6, (s.foregroundDisparity - s.backgroundDisparity) * .35);
    for (int i = 0; i < d.length; i++) {
      if (s.fence[i]) {
        fg++;
        if (Float.isFinite(d[i])) {
          if (Math.abs(d[i] - s.foregroundDisparity) < tolerance) correct++;
          else {
            wrong++;
            if (d[i] > s.backgroundDisparity + tolerance
                && d[i] < s.foregroundDisparity - tolerance) bump++;
          }
        }
      }
      if (s.background[i]) {
        bg++;
        if (Float.isFinite(d[i])) {
          bgValid++;
          if (Math.abs(d[i] - s.backgroundDisparity) > tolerance) bgWrong++;
        }
      }
    }
    return Json.object(
        "foreground_pixels",
        fg,
        "fence_correct",
        correct,
        "fence_correct_fraction",
        correct / (double) fg,
        "fence_wrong",
        wrong,
        "fence_intermediate_bump",
        bump,
        "background_pixels",
        bg,
        "background_valid",
        bgValid,
        "background_wrong",
        bgWrong);
  }

  static Options previous(Options current) {
    Options o = current.copy();
    o.p1 = 8;
    o.p2 = 32;
    o.sgmEdgeScale = 0;
    o.medianMode = "classic";
    o.speckleConnectivity = 4;
    return o;
  }

  public static void main(String[] args) throws Exception {
    Path root = Files.createTempDirectory(Path.of("target"), "fence test ");
    int count =
        args.length == 2
            ? run(root, Integer.parseInt(args[0]), Integer.parseInt(args[1]))
            : run(root);
    System.out.println("PASS fence=" + count + "; " + root);
    GpuSgm.closeSession();
  }

  static int run(Path root) throws Exception {
    return run(root, (int) new Options().p2, new Options().sgmEdgeScale);
  }

  static int run(Path root, int penalty, int edgeScale) throws Exception {
    checks = 0;
    List<Object> rows = new ArrayList<>();
    for (int[] disparities : new int[][] {{24, 8}, {40, 39}})
      for (int seed : new int[] {19, 31, 93})
        for (int width : new int[] {1, 2, 3, 5}) {
          Scene s = scene(width, seed, disparities[0], disparities[1]);
          Options modern = TestSupport.options();
          modern.disparities = 48;
          modern.p2 = penalty;
          modern.sgmEdgeScale = edgeScale;
          if (seed == 19 && width == 2 && s.foregroundDisparity == 40) {
            java.awt.image.BufferedImage source =
                new java.awt.image.BufferedImage(
                    s.w, s.h, java.awt.image.BufferedImage.TYPE_INT_RGB);
            float[] truth = new float[s.w * s.h];
            for (int p = 0; p < truth.length; p++) {
              int value = s.left[p] & 255;
              source.setRGB(p % s.w, p / s.w, value * 0x010101);
              truth[p] = s.fence[p] ? s.foregroundDisparity : s.backgroundDisparity;
            }
            javax.imageio.ImageIO.write(source, "png", root.resolve("source.png").toFile());
            preview(root.resolve("truth.png"), s, truth);
          }
          Options old = previous(modern);
          for (String version : List.of("previous", "revised")) {
            Options o = version.equals("previous") ? old : modern;
            Map<String, Object> stages = new LinkedHashMap<>();
            new CensusSgm(o)
                .computeRecorded(
                    s.left,
                    s.right,
                    null,
                    null,
                    s.w,
                    s.h,
                    null,
                    null,
                    new CensusSgm.Observer() {
                      public void raster(String name, float[] pixels, int w, int h)
                          throws java.io.IOException {
                        if (name.matches(
                            "00_left_selected|02_left_right_validated|03_median|04_photometric|05_final_stereo"))
                          stages.put(name, metrics(s, pixels));
                        if (seed == 19
                            && width == 2
                            && s.foregroundDisparity == 40
                            && (name.equals("03_median") || name.equals("05_final_stereo")))
                          preview(root.resolve(version + "_" + name + ".png"), s, pixels);
                      }
                    });
            rows.add(
                Json.object(
                    "seed",
                    seed,
                    "post_width",
                    width,
                    "foreground_disparity",
                    s.foregroundDisparity,
                    "background_disparity",
                    s.backgroundDisparity,
                    "version",
                    version,
                    "p1",
                    o.p1,
                    "p2",
                    o.p2,
                    "edge_scale",
                    o.sgmEdgeScale,
                    "median_mode",
                    o.medianMode,
                    "stages",
                    stages));
          }
        }
    Json.write(root.resolve("comparison.json"), rows);
    for (int fd : new int[] {24, 40}) {
      int oldCorrect = 0, newCorrect = 0, oldWrong = 0, newWrong = 0, oldBg = 0, newBg = 0;
      for (Object value : rows) {
        Map<?, ?> row = (Map<?, ?>) value;
        if (((Number) row.get("foreground_disparity")).intValue() != fd) continue;
        Map<?, ?> stages = (Map<?, ?>) row.get("stages"),
            finals = (Map<?, ?>) stages.get("05_final_stereo");
        int correct = ((Number) finals.get("fence_correct")).intValue(),
            wrong =
                ((Number) finals.get("fence_wrong")).intValue()
                    + ((Number) finals.get("background_wrong")).intValue(),
            bg = ((Number) finals.get("background_valid")).intValue();
        if (row.get("version").equals("previous")) {
          oldCorrect += correct;
          oldWrong += wrong;
          oldBg += bg;
        } else {
          newCorrect += correct;
          newWrong += wrong;
          newBg += bg;
        }
      }
      System.out.printf(
          Locale.ROOT,
          "Fence d=%d: correct %d -> %d, incorrect %d -> %d, valid background %d -> %d%n",
          fd,
          oldCorrect,
          newCorrect,
          oldWrong,
          newWrong,
          oldBg,
          newBg);
      check(
          newCorrect > oldCorrect * 1.1,
          "Revised defaults recover more actual fence depth (d=" + fd + ")");
      check(
          newWrong < oldWrong,
          "Revised defaults reduce combined wrong fence/background depths (d=" + fd + ")");
      check(newBg > oldBg * .95, "Retain valid background coverage (d=" + fd + ")");
    }
    medianAndConnectivity();
    Options parsed =
        Options.parse(
            new String[] {
              "-i",
              root.toString(),
              "--sgm-edge-scale",
              "0",
              "--median-mode",
              "classic",
              "--median-max-change",
              "0.25",
              "--median-edge-threshold",
              "0",
              "--speckle-connectivity",
              "4"
            });
    check(parsed.p1 == 4 && parsed.p2 == 24, "CLI uses revised default penalties");
    check(
        parsed.copy().sgmEdgeScale == 0
            && parsed.copy().medianMode.equals("classic")
            && parsed.copy().medianMaxChange == .25
            && parsed.copy().medianEdgeThreshold == 0
            && parsed.copy().speckleConnectivity == 4,
        "Explicit controls survive parsing and option copies");
    for (String[] invalid :
        new String[][] {
          {"--sgm-edge-scale", "-1"},
          {"--median-mode", "blur"},
          {"--median-max-change", "-1"},
          {"--median-edge-threshold", "256"},
          {"--speckle-connectivity", "6"}
        }) {
      boolean rejected = false;
      try {
        Options.parse(new String[] {"-i", root.toString(), invalid[0], invalid[1]});
      } catch (IllegalArgumentException e) {
        rejected = true;
      }
      check(rejected, "Invalid thin-structure control rejected: " + invalid[0]);
    }
    if (!GpuSgm.devices().isEmpty()) {
      Scene s = scene(2, 31, 40, 39);
      Options cpu = TestSupport.options();
      cpu.disparities = 48;
      cpu.p2 = penalty;
      cpu.sgmEdgeScale = edgeScale;
      cpu.gpuDevice = Integer.getInteger("sgm.test.gpu", 0);
      cpu.output = Files.createDirectories(root.resolve("fence-gpu"));
      var expected = new CensusSgm(cpu).compute(s.left, s.right, s.w, s.h);
      for (String mode : List.of("resident", "streamed")) {
        Options go = cpu.copy();
        go.compute = "gpu";
        go.gpuSgm = mode;
        go.gpuStripRows = 5;
        var actual = new CensusSgm(go).compute(s.left, s.right, s.w, s.h);
        check(
            Arrays.equals(expected.disparity, actual.disparity),
            "CPU/GPU fence layers and strip edges agree: " + mode);
        check(
            expected.stats.csv().equals(actual.stats.csv()),
            "CPU/GPU fence rejection counters: " + mode);
      }
      Options historical = previous(cpu), go = historical.copy();
      go.compute = "gpu";
      var reference = new CensusSgm(historical).compute(s.left, s.right, s.w, s.h);
      var restored = new CensusSgm(go).compute(s.left, s.right, s.w, s.h);
      check(
          Arrays.equals(reference.disparity, restored.disparity),
          "Explicit constant-penalty/classic filtering CPU/GPU parity");
      check(
          reference.stats.csv().equals(restored.stats.csv()),
          "Explicit legacy rejection-counter parity");
    }
    return checks;
  }

  private static void medianAndConnectivity() {
    int w = 9, h = 9;
    float[] disparity = new float[w * h];
    byte[] gray = new byte[disparity.length];
    Arrays.fill(disparity, 8);
    Arrays.fill(gray, (byte) 160);
    for (int y = 0; y < h; y++) {
      disparity[y * w + 4] = 9;
      gray[y * w + 4] = 40;
    }
    disparity[0] = Float.NaN;
    float[] before = disparity.clone();
    check(
        CensusSgm.medianValid(disparity, w, h, 3)[4 * w + 4] == 8,
        "Classic median reproduces one-pixel fence flattening");
    float[] modern = CensusSgm.medianEdgePreserving(disparity, w, h, 3, 1, gray, 24);
    check(
        modern[4 * w + 4] == 9,
        "Guided median preserves a one-pixel disparity jump on a thin fence");
    check(
        Float.isNaN(modern[0]) && Arrays.equals(disparity, before),
        "Median neither fills holes nor mutates source raster");
    byte[] pixels = {(byte) 100, (byte) 100, (byte) 200};
    check(StereoCost.pathP2(4, 16, 16, pixels, 0, 1) == 16, "Flat image keeps base P2");
    check(
        StereoCost.pathP2(4, 16, 16, pixels, 1, 2) == 5,
        "Strong image edge reduces P2 but keeps P2>P1");
    check(StereoCost.pathP2(4, 16, 0, pixels, 1, 2) == 16, "Zero edge scale restores constant P2");
    float[] diagonal = new float[70 * 70];
    Arrays.fill(diagonal, Float.NaN);
    for (int i = 5; i < 65; i++) diagonal[i * 70 + i] = 12;
    float[] legacy = diagonal.clone();
    check(
        CensusSgm.removeSpeckles(legacy, 70, 70, 50, 1, 4) == 60,
        "Four-connectivity reproduces diagonal-wire removal");
    check(
        CensusSgm.removeSpeckles(diagonal, 70, 70, 50, 1, 8) == 0,
        "Eight-connectivity preserves a long thin diagonal wire");
    diagonal[0] = 20;
    check(
        CensusSgm.removeSpeckles(diagonal, 70, 70, 50, 1, 8) == 1,
        "Isolated disparity speckles still rejected");
  }

  private static void preview(Path file, Scene s, float[] disparity) throws java.io.IOException {
    java.awt.image.BufferedImage picture =
        new java.awt.image.BufferedImage(s.w, s.h, java.awt.image.BufferedImage.TYPE_INT_RGB);
    for (int p = 0; p < disparity.length; p++) {
      int rgb = 0;
      if (Float.isFinite(disparity[p])) {
        double t =
            Math.max(
                0,
                Math.min(
                    1,
                    (disparity[p] - s.backgroundDisparity)
                        / (s.foregroundDisparity - s.backgroundDisparity)));
        rgb = ((int) Math.round(t * 255) << 16) | 0x2000 | (int) Math.round((1 - t) * 255);
      }
      picture.setRGB(p % s.w, p / s.w, rgb);
    }
    javax.imageio.ImageIO.write(picture, "png", file.toFile());
  }
}
