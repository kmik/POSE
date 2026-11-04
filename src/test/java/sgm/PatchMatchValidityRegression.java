package sgm;

import java.nio.file.*;
import java.util.*;

/**
 * Invalid evidence, coarse seeds, geometric visibility, repeated texture and thin-layer controls.
 */
final class PatchMatchValidityRegression {
  private static int checks;

  private static void check(boolean ok, String text) {
    checks++;
    if (!ok) throw new AssertionError(text);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Path root = Files.createTempDirectory(Path.of("target"), "patchmatch-validity-");
    System.out.println("PASS PatchMatch validity: " + run(root) + "; " + root);
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Options o = TestSupport.options();
    o.patchmatchIterations = 6;
    PatchMatch.Scene scene = ConfidencePatchMatchRegression.planeScene(3);
    int n = scene.width() * scene.height();
    float[] plane = {0, 0, 1, 4};
    check(
        !Double.isFinite(PatchMatch.cost(scene, 1, 30, plane, 1)),
        "out-of-image patch is unavailable, not bad photometry");
    check(
        Double.isFinite(PatchMatch.cost(scene, 48, 32, new float[] {0, 0, 1, 3.3f}, 1)),
        "poor available match remains distinguishable from missing evidence");
    check(
        !Double.isFinite(
            PatchMatch.cost(
                scene, 48, 32, new float[] {20, 0, 1, 4 * (20 * (48 - 47.5f) / 96 + 1)}, 1)),
        "back-facing/degenerate patch warp rejected");
    float[] depths = new float[scene.pixels().length];
    Arrays.fill(depths, n, depths.length, 4);
    PatchMatch.Scene geometry =
        new PatchMatch.Scene(
            scene.width(),
            scene.height(),
            scene.pixels(),
            scene.masks(),
            scene.cameras(),
            scene.views(),
            3,
            5,
            depths,
            .005f);
    check(
        !Double.isFinite(PatchMatch.cost(geometry, 48, 32, new float[] {0, 0, 1, 3.3f}, -1)),
        "two coherent free-space contradictions invalidate false depth");
    check(PatchMatch.cost(geometry, 48, 32, plane, -1) < 1e-10, "known surface remains supported");
    Arrays.fill(depths, 2 * n, depths.length, 0);
    check(
        Double.isFinite(PatchMatch.cost(geometry, 48, 32, new float[] {0, 0, 1, 3.3f}, -1)),
        "one contradiction plus unavailable depth does not invent independent corroboration");
    Arrays.fill(depths, n, 2 * n, 3);
    Arrays.fill(depths, 2 * n, depths.length, 4);
    check(
        Double.isFinite(PatchMatch.cost(geometry, 48, 32, plane, -1)),
        "nearer occluder is neutral when another image sees the surface");
    Arrays.fill(depths, 0);
    check(
        PatchMatch.cost(geometry, 48, 32, plane, -1) == PatchMatch.cost(scene, 48, 32, plane, -1),
        "missing geometric evidence exactly neutral");
    float[] invalid = new float[n * 8];
    for (int i = 0; i < n; i++) {
      invalid[i * 8] = 4;
      invalid[i * 8 + 1] = 100;
      invalid[i * 8 + 3] = -5;
      invalid[i * 8 + 4] = Float.POSITIVE_INFINITY;
      invalid[i * 8 + 5] = -1;
    }
    var cold = PatchMatch.compute(scene, o, null);
    var reseeded =
        PatchMatch.compute(
            scene, o, new PatchMatch.Map(scene.width(), scene.height(), invalid, null));
    check(
        Arrays.equals(cold.data(), reseeded.data()),
        "invalid coarse matches are deterministically reinitialized");
    var narrow =
        new PatchMatch.Scene(
            scene.width(),
            scene.height(),
            scene.pixels(),
            scene.masks(),
            scene.cameras(),
            scene.views(),
            3.999f,
            4.001f);
    var limited = PatchMatch.compute(narrow, o, null);
    int seen = 0;
    for (int i = 0; i < n; i++)
      if (limited.view(i) > 0) {
        seen++;
        check(
            limited.margin(i) == 0 && limited.sharpness(i) == 0,
            "invalid competitors cannot inflate confidence at depth bounds");
      }
    check(seen > 1000, "narrow-bound control contains real photometric matches");
    byte[] missing = scene.masks().clone();
    Arrays.fill(missing, (byte) 0);
    var noMatches =
        new PatchMatch.Scene(
            scene.width(),
            scene.height(),
            scene.pixels(),
            missing,
            scene.cameras(),
            scene.views(),
            3,
            5);
    var unavailable = PatchMatch.compute(noMatches, o, null);
    check(
        allInvalid(unavailable), "unavailable CPU matches export invalid depth/source/confidence");
    extraUnavailable(scene, o);
    for (int device = 0; device < GpuSgm.devices().size(); device++) {
      Options gpu = o.copy();
      gpu.compute = "gpu";
      gpu.gpuDevice = device;
      check(
          allInvalid(PatchMatch.compute(noMatches, gpu, null)),
          "unavailable GPU matches invalid on device " + device);
      extraUnavailable(scene, gpu);
      var repeated = periodic(scene);
      var ambiguous = PatchMatch.compute(repeated, gpu, null);
      int strong = 0, ambiguousCount = 0;
      for (int y = 8; y < 56; y++)
        for (int x = 32; x < 80; x++) {
          int i = y * 96 + x;
          if (ambiguous.photo(i) > .9) {
            strong++;
            if (ambiguous.margin(i) < .15) ambiguousCount++;
          }
        }
      System.out.println(
          "periodic " + device + ": strong=" + strong + " ambiguous=" + ambiguousCount);
      check(
          strong > 1000 && ambiguousCount > strong * .7,
          "wide inverse-depth alternatives expose repeated-texture ambiguity");
    }
    thin(o);
    return checks;
  }

  private static void extraUnavailable(PatchMatch.Scene base, Options o) {
    int n = base.width() * base.height();
    byte[] pixels = base.pixels().clone();
    Arrays.fill(pixels, 2 * n, pixels.length, (byte) 127);
    var pair =
        new PatchMatch.Scene(
            base.width(),
            base.height(),
            Arrays.copyOf(pixels, 2 * n),
            Arrays.copyOf(base.masks(), 2 * n),
            Arrays.copyOf(base.cameras(), 40),
            2,
            base.minimum(),
            base.maximum());
    var extra =
        new PatchMatch.Scene(
            base.width(),
            base.height(),
            pixels,
            base.masks(),
            base.cameras(),
            3,
            base.minimum(),
            base.maximum());
    check(
        Arrays.equals(
            PatchMatch.compute(pair, o, null).data(), PatchMatch.compute(extra, o, null).data()),
        "untextured extra source cannot displace a good producing pair or change its confidence");
  }

  private static boolean allInvalid(PatchMatch.Map map) {
    for (int i = 0; i < map.width() * map.height(); i++)
      if (Float.isFinite(map.depth(i))
          || map.view(i) > 0
          || map.margin(i) != 0
          || map.sharpness(i) != 0) return false;
    return true;
  }

  private static PatchMatch.Scene periodic(PatchMatch.Scene base) {
    int w = base.width(), h = base.height(), n = w * h;
    byte[] pixels = new byte[base.pixels().length];
    for (int v = 0; v < base.views(); v++)
      for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++)
          pixels[v * n + y * w + x] =
              (byte) (128 + 55 * Math.sin((x + 6 * v) * Math.PI / 4) + 35 * Math.cos(y * .7));
    return new PatchMatch.Scene(w, h, pixels, base.masks(), base.cameras(), base.views(), 1, 10);
  }

  private static void thin(Options options) {
    int w = 128, h = 80, n = w * h, views = 3;
    float[] cameras = new float[views * 20];
    byte[] pixels = new byte[n * views], mask = new byte[n * views];
    float[] truth = new float[n * views];
    Arrays.fill(mask, (byte) 255);
    boolean[] branch = new boolean[n];
    for (int v = 0; v < views; v++)
      for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
          double rx = (x - 63.5) / 100,
              ry = (y - 39.5) / 100,
              xx = v * .25 + rx * 3.5,
              yy = ry * 3.5;
          boolean front = Math.abs(xx - .2) < .055 && Math.abs(yy) < .8;
          if (v > 0) truth[v * n + y * w + x] = front ? 3.5f : 6;
          double value;
          if (front) value = 50 + 25 * Math.sin(30 * yy) + 20 * Math.cos(11 * yy + 4 * xx);
          else {
            xx = v * .25 + rx * 6;
            yy = ry * 6;
            value = 180 + 40 * Math.sin(14 * xx + 3 * yy) + 25 * Math.cos(17 * yy - 5 * xx);
          }
          pixels[v * n + y * w + x] = (byte) Math.clamp((int) Math.round(value), 0, 255);
          if (v == 0) branch[y * w + x] = front;
        }
    for (int v = 0; v < views; v++) {
      int c = v * 20;
      cameras[c] = cameras[c + 4] = cameras[c + 8] = 1;
      cameras[c + 9] = -v * .25f;
      cameras[c + 12] = cameras[c + 13] = 100;
      cameras[c + 14] = 63.5f;
      cameras[c + 15] = 39.5f;
      cameras[c + 16] = w;
      cameras[c + 17] = h;
      cameras[c + 18] = v * n / 65536;
      cameras[c + 19] = v * n % 65536;
    }
    PatchMatch.Scene scene = new PatchMatch.Scene(w, h, pixels, mask, cameras, views, 3, 7);
    Options o = options.copy();
    o.patchmatchIterations = 8;
    thinMetrics(scene, o, branch, "CPU photometric");
    var geometric = new PatchMatch.Scene(w, h, pixels, mask, cameras, views, 3, 7, truth, .005f);
    thinMetrics(geometric, o, branch, "CPU geometric");
    for (int device = 0; device < GpuSgm.devices().size(); device++) {
      o.compute = "gpu";
      o.gpuDevice = device;
      thinMetrics(scene, o, branch, "GPU " + device + " photometric");
      thinMetrics(geometric, o, branch, "GPU " + device + " geometric");
    }
  }

  private static void thinMetrics(
      PatchMatch.Scene scene, Options o, boolean[] branch, String label) {
    int w = scene.width(), h = scene.height();
    var result = PatchMatch.compute(scene, o, null);
    int total = 0, retained = 0, bad = 0, background = 0;
    for (int y = 8; y < h - 8; y++)
      for (int x = 32; x < w - 8; x++) {
        int i = y * w + x;
        if (branch[i]) {
          total++;
          if (result.photo(i) > .65 && Math.abs(result.depth(i) - 3.5) < .2) retained++;
        } else if (result.photo(i) > .8) {
          background++;
          if (Math.abs(result.depth(i) - 6) > .3) bad++;
        }
      }
    System.out.println(
        label
            + " thin layer: "
            + retained
            + "/"
            + total
            + " retained; background="
            + background
            + " wrong="
            + bad);
    check(retained > total * .35, "edge-aware patches retain thin foreground layer");
    check(bad < background * .08, "thin foreground does not create a false background surface");
  }
}
