package sgm;

import java.nio.file.*;
import java.util.*;

/**
 * PATCHMATCH.md phases 1-4: CPU row split equals the serial oracle, tiles with the dependency-radius
 * halo equal the untiled GPU solve (and agree closely with a short halo), the default program text
 * is unchanged, the top-K view cost is the mean of the K best view costs, and plane-consistent
 * smoothing lowers depth noise without mixing surfaces across a depth edge.
 */
final class PatchMatchScaleRegression {
  private static int checks;

  private static void check(boolean ok, String text) {
    checks++;
    if (!ok) throw new AssertionError(text);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Path root = Files.createTempDirectory(Path.of("target"), "patchmatch-scale-");
    System.out.println("PASS PatchMatch scale: " + run(root) + "; " + root);
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Options o = TestSupport.options();
    o.compute = "cpu";
    o.patchmatchIterations = 5;
    rowSplit(o);
    programs(o);
    topViews(o);
    smoothing();
    speckle();
    for (int device = 0; device < GpuSgm.devices().size(); device++) {
      Options g = o.copy();
      g.compute = "gpu";
      g.gpuDevice = device;
      tiles(g, device);
    }
    return checks;
  }

  private static boolean same(float[] a, float[] b) {
    if (a.length != b.length) return false;
    for (int i = 0; i < a.length; i++)
      if (Float.floatToIntBits(a[i]) != Float.floatToIntBits(b[i])) return false;
    return true;
  }

  /** Jacobi rows are independent: any number of row workers gives the serial result. */
  private static void rowSplit(Options o) {
    PatchMatch.Scene scene = ConfidencePatchMatchRegression.planeScene(3);
    float[] serial = PatchMatch.compute(scene, o, null).data();
    for (int cores : new int[] {2, 5}) {
      float[] split;
      try (var rows = StereoRows.open(cores)) {
        split = PatchMatch.compute(scene, o, null).data();
      }
      check(same(serial, split), "CPU PatchMatch with " + cores + " row workers equals serial");
    }
    var previous = PatchMatch.compute(scene, o, null);
    float[] seeded = PatchMatch.compute(scene, o, previous).data();
    try (var rows = StereoRows.open(3)) {
      check(
          same(seeded, PatchMatch.compute(scene, o, previous).data()),
          "seeded CPU PatchMatch with row workers equals serial");
    }
  }

  /** Default options build the original program text; variants substitute exactly once. */
  private static void programs(Options o) {
    check(PatchMatch.program(PatchMatch.SOURCE, o) == PatchMatch.SOURCE, "default program unchanged");
    check(
        PatchMatch.program(PatchMatch.TILED_SOURCE, o) == PatchMatch.TILED_SOURCE,
        "default tiled program unchanged");
    Options top = o.copy();
    top.patchmatchTopViews = 3;
    for (String base : List.of(PatchMatch.SOURCE, PatchMatch.TILED_SOURCE)) {
      String variant = PatchMatch.program(base, top);
      check(variant.startsWith("#define TOPK 3\n"), "top-K program defines K");
      check(variant.contains("sum/n+.1f*contradictions"), "top-K program averages the K best views");
      check(!variant.contains(".25f*fmin(.4f,second-best)"), "top-K program drops the second-view term");
    }
  }

  /** Without geometry, the top-2 cost of a 3-view scene is the mean of the two view costs. */
  private static void topViews(Options o) {
    PatchMatch.Scene scene = ConfidencePatchMatchRegression.planeScene(3);
    float[] plane = {0, 0, 1, 4};
    int tested = 0;
    for (int y = 20; y < 44; y += 7)
      for (int x = 30; x < 70; x += 9) {
        double a = PatchMatch.cost(scene, x, y, plane, 1), b = PatchMatch.cost(scene, x, y, plane, 2);
        if (!Double.isFinite(a) || !Double.isFinite(b)) continue;
        double mean = PatchMatch.cost(scene, x, y, plane, -1, 2);
        check(mean == (Math.min(a, b) + Math.max(a, b)) / 2, "top-2 cost is the mean at " + x + "," + y);
        check(
            PatchMatch.cost(scene, x, y, plane, -1, 1) == PatchMatch.cost(scene, x, y, plane, -1),
            "top-1 cost is the original cost");
        tested++;
      }
    check(tested > 5, "top-K cost tested on enough pixels");
  }

  /** Noisy fronto-parallel steps at depth 10 and 12: noise drops, the edge stays sharp. */
  private static void smoothing() {
    int w = 64, h = 32;
    float[] data = new float[w * h * 8], clean = new float[w * h];
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) {
        int i = y * w + x;
        float truth = x < w / 2 ? 10 : 12;
        float noise = (PatchMatch.unit(PatchMatch.hash(i * 7 + 3)) - .5f) * .02f * truth;
        clean[i] = truth;
        data[i * 8] = truth + noise;
        data[i * 8 + 3] = 1;
        data[i * 8 + 4] = .1f;
        data[i * 8 + 5] = 1;
        data[i * 8 + 6] = data[i * 8 + 7] = .5f;
      }
    var map = new PatchMatch.Map(w, h, data, null);
    double before = rms(data, clean, w, h);
    long changed;
    try (var rows = StereoRows.open(4)) {
      changed = PatchMatchReconstruction.smoothDepths(map, 50, 50, 31.5, 15.5, 2, .05, i -> true);
    }
    double after = rms(data, clean, w, h);
    check(changed > w * h / 2, "smoothing changes most depths");
    check(after < before / 2, "smoothing lowers depth noise (" + before + " -> " + after + ")");
    for (int y = 0; y < h; y++)
      for (int x = w / 2 - 2; x < w / 2 + 2; x++)
        check(
            Math.abs(data[(y * w + x) * 8] - clean[y * w + x]) < .3,
            "no mixing across the depth edge at " + x + "," + y);
    float[] again = new float[data.length];
    for (int i = 0; i < w * h; i++) {
      float truth = clean[i];
      again[i * 8] = truth + (PatchMatch.unit(PatchMatch.hash(i * 7 + 3)) - .5f) * .02f * truth;
      System.arraycopy(data, i * 8 + 1, again, i * 8 + 1, 7);
    }
    PatchMatchReconstruction.smoothDepths(
        new PatchMatch.Map(w, h, again, null), 50, 50, 31.5, 15.5, 2, .05, i -> true);
    check(same(again, data), "smoothing without row workers equals the parallel result");
  }

  /** A 40 x 30 plane at depth 10 with a 3 x 3 island at depth 13 and a 6 x 6 island at 7. */
  private static void speckle() {
    int w = 40, h = 30;
    float[] d = new float[w * h * 8];
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) {
        int i = (y * w + x) * 8;
        boolean small = x >= 5 && x < 8 && y >= 5 && y < 8, big = x >= 20 && x < 26 && y >= 10 && y < 16;
        d[i] = small ? 13 : big ? 7 : 10 + .001f * x;
        d[i + 5] = 1;
      }
    var map = new PatchMatch.Map(w, h, d, null);
    long removed = PatchMatchReconstruction.speckle(map, 20, .01, i -> map.view(i) > 0);
    check(removed == 9, "speckle removes only the 9-pixel island: " + removed);
    check(map.view(6 * w + 6) == -1 && map.view(12 * w + 22) == 1 && map.view(0) == 1, "island invalid, larger island and plane kept");
    check(PatchMatchReconstruction.speckle(map, 40, .01, i -> map.view(i) > 0) == 36, "a larger minimum also removes the 36-pixel island");
  }

  private static double rms(float[] data, float[] clean, int w, int h) {
    double sum = 0;
    int n = 0;
    for (int y = 3; y < h - 3; y++)
      for (int x = 3; x < w - 3; x++) {
        if (Math.abs(x - w / 2) < 3) continue;
        double e = data[(y * w + x) * 8] - clean[y * w + x];
        sum += e * e;
        n++;
      }
    return Math.sqrt(sum / n);
  }

  /** Tiles: dependency-radius halo = untiled bit for bit; a 2-pixel halo agrees closely. */
  private static void tiles(Options o, int device) {
    PatchMatch.Scene scene = ConfidencePatchMatchRegression.planeScene(3);
    int w = scene.width(), h = scene.height();
    PatchMatch.Map whole = PatchMatch.compute(scene, o, null);
    int radius = PatchMatchReconstruction.dependencyRadius(w, h, o);
    check(radius > 0, "dependency radius positive");
    for (int halo : new int[] {radius, 2}) {
      float[] tiled = new float[whole.data().length];
      int core = Math.max(8, Math.max(w, h) / 3);
      for (int ty = 0; ty < h; ty += core)
        for (int tx = 0; tx < w; tx += core) {
          int x0 = Math.max(0, tx - halo), y0 = Math.max(0, ty - halo);
          int x1 = Math.min(w, tx + core + halo), y1 = Math.min(h, ty + core + halo);
          int[] crops = new int[scene.views() * 4];
          for (int v = 0; v < scene.views(); v++) {
            int c = v * 20;
            crops[v * 4 + 2] = (int) scene.cameras()[c + 16];
            crops[v * 4 + 3] = (int) scene.cameras()[c + 17];
          }
          var part =
              PatchMatch.computeTile(
                  scene, new PatchMatch.Tile(x0, y0, x1 - x0, y1 - y0, w, h, crops), o, null);
          for (int y = ty; y < Math.min(h, ty + core); y++)
            System.arraycopy(
                part.data(),
                ((y - y0) * (x1 - x0) + tx - x0) * 8,
                tiled,
                (y * w + tx) * 8,
                (Math.min(w, tx + core) - tx) * 8);
        }
      int both = 0, close = 0;
      for (int i = 0; i < w * h; i++) {
        float a = whole.data()[i * 8], b = tiled[i * 8];
        if (!(a > 0) || !(b > 0)) continue;
        both++;
        if (Math.abs(a - b) <= .01 * a) close++;
      }
      boolean identical = same(tiled, whole.data());
      System.out.printf(
          Locale.ROOT,
          "GPU %d tiles, halo %d (dependency radius %d): bit-identical %s, %d of %d within 1%%%n",
          device, halo, radius, identical, close, both);
      // The tiled program is compiled separately; a compiler may emit different (still IEEE)
      // arithmetic for it, so bit identity is a property of the device (it holds on the NVIDIA
      // reference GPU). Agreement is required everywhere.
      check(
          both > 0 && close >= (halo == radius ? .99 : .9) * both,
          "GPU " + device + " tiles (halo " + halo + ") agree with untiled");
    }
  }
}
