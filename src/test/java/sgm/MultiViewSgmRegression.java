package sgm;

import java.util.*;

/**
 * --dense-method mvsgm (MULTIVIEW_SGM.md) on a synthetic scene: a textured far plane (z = 50) and a
 * nearer textured rectangle (z = 44) seen by a reference camera and four neighbours, one of them
 * rotated 180 degrees about its optical axis (opposite flight line). Checks:
 * depth accuracy of the CPU solve; GPU summaries equal to the CPU reference for full and random
 * narrow bands (skipped without a GPU); coarse-to-fine banding against the full-range solve; the
 * pyramid-level mapping table; option validation.
 */
final class MultiViewSgmRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) {
    nu.pattern.OpenCV.loadLocally();
    System.out.println("PASS multi-view SGM: " + run());
  }

  static final int W = 320, H = 240;
  static final double FAR = 50, NEAR = 44;

  static float[] texture(long seed) {
    Random r = new Random(seed);
    float[] t = new float[400 * 400];
    for (int i = 0; i < t.length; i++) t[i] = 40 + 180 * r.nextFloat();
    return t;
  }

  /** Bilinear texture over world XY with 4 cm cells. */
  static double sample(float[] t, double x, double y) {
    double u = x / .04 + 200, v = y / .04 + 200;
    int i = (int) Math.floor(u), j = (int) Math.floor(v);
    if (i < 0 || j < 0 || i + 1 >= 400 || j + 1 >= 400) return 128;
    double fx = u - i, fy = v - j;
    return (1 - fy) * ((1 - fx) * t[j * 400 + i] + fx * t[j * 400 + i + 1])
        + fy * ((1 - fx) * t[(j + 1) * 400 + i] + fx * t[(j + 1) * 400 + i + 1]);
  }

  static boolean inNear(double x, double y) {
    return Math.abs(x) < 4 && Math.abs(y) < 3;
  }

  static Camera camera(double cx, double cy, boolean rotated) {
    Camera c = new Camera(600, 600, W / 2.0, H / 2.0);
    c.rotation = rotated ? new double[] {-1, 0, 0, 0, -1, 0, 0, 0, 1} : Math3.identity();
    c.translation = Math3.times(Math3.mv(c.rotation, new double[] {cx, cy, 0}), -1);
    c.registered = true;
    return c;
  }

  /** Renders the scene; also returns the true depth when depth != null. */
  static byte[] render(Camera c, float[] far, float[] near, float[] depth) {
    byte[] g = new byte[W * H];
    double[] rt = Math3.transpose(c.rotation), centre = c.center();
    for (int v = 0; v < H; v++)
      for (int u = 0; u < W; u++) {
        double[] d = Math3.mv(rt, new double[] {(u - c.cx) / c.fx, (v - c.cy) / c.fy, 1});
        double s = (NEAR - centre[2]) / d[2], x = centre[0] + s * d[0], y = centre[1] + s * d[1];
        double value, z;
        if (inNear(x, y)) {
          value = sample(near, x, y);
          z = NEAR;
        } else {
          s = (FAR - centre[2]) / d[2];
          x = centre[0] + s * d[0];
          y = centre[1] + s * d[1];
          value = sample(far, x, y);
          z = FAR;
        }
        g[v * W + u] = (byte) Math.max(0, Math.min(255, (int) Math.round(value)));
        if (depth != null) depth[v * W + u] = (float) z;
      }
    return g;
  }

  static int run() {
    checks = 0;
    Options o = TestSupport.options();
    o.speckleSize = 20;
    float[] far = texture(1), near = texture(2);
    Camera ref = camera(0, 0, false);
    float[] truth = new float[W * H];
    byte[] refGray = render(ref, far, near, truth);
    Camera[] others = {camera(6, 0, false), camera(-6, 0, true), camera(0, 5, false), camera(0, -5, false)};
    byte[] ones = new byte[W * H];
    Arrays.fill(ones, (byte) 255);
    MultiViewSgm mv = new MultiViewSgm(o);
    double[][] shifts = new double[others.length][];
    for (int i = 0; i < others.length; i++) shifts[i] = MultiViewSgm.shift(ref, others[i]);
    List<double[]> samples = new ArrayList<>();
    for (int i = 0; i < 40; i++) samples.add(new double[] {1 / (i % 2 == 0 ? FAR : NEAR), 1});
    MultiViewSgm.Labels labels = mv.labels(samples, W, H, shifts);
    check(labels.q(0) <= 1 / FAR && labels.q(labels.count() - 1) >= 1 / NEAR, "label range covers the scene");
    MultiViewSgm.Reference reference = new MultiViewSgm.Reference(W, H, refGray, ones, mv.census);
    MultiViewSgm.Warped[] views = new MultiViewSgm.Warped[others.length];
    List<MultiViewSgm.Neighbor> neighbours = new ArrayList<>();
    for (int i = 0; i < others.length; i++) {
      byte[] g = render(others[i], far, near, null);
      views[i] = MultiViewSgm.warp(ref, W, H, labels, i + 1, others[i], g, ones, W, H, mv.census);
      neighbours.add(new MultiViewSgm.Neighbor(i + 1, 1, 100, 7));
    }
    MultiViewSgm.Plan plan = new MultiViewSgm.Plan(0, neighbours, labels, reference, views);

    // 1. CPU full-range solve: depth accuracy away from the depth edges.
    DisparityBands full = DisparityBands.full(W, H, labels.count(), reference.use);
    int[] cpu = mv.cpuSummary(plan, full);
    MultiViewSgm.DepthMap map = mv.select(plan, cpu);
    int good = 0, used = 0;
    for (int y = 10; y < H - 10; y++)
      for (int x = 10; x < W - 10; x++) {
        int p = y * W + x;
        if (!reference.use[p] || nearEdge(truth, x, y)) continue;
        used++;
        if (map.depth()[p] > 0 && Math.abs(map.depth()[p] - truth[p]) <= .005 * truth[p]) good++;
      }
    check(good >= .9 * used, "CPU mvsgm depth within 0.5% for " + good + " of " + used + " pixels");

    // 2. GPU = CPU (full and random narrow bands), when a GPU is usable.
    Options g = o.copy();
    g.compute = "auto";
    MultiViewSgm gm = new MultiViewSgm(g);
    if (GpuMultiViewSgm.use(g, gm, plan, full)) {
      try (var session = new GpuMultiViewSgm.Session(g, gm, plan, 0)) {
        check(Arrays.equals(session.summary(full), cpu), "GPU full-band summary equals the CPU reference");
        Random r = new Random(7);
        for (int trial = 0; trial < 3; trial++) {
          DisparityBands narrow = new DisparityBands(W, H, labels.count());
          for (int p = 0; p < W * H; p++) {
            if (!reference.use[p] || r.nextDouble() < .05) continue;
            int lo = r.nextInt(labels.count()), n = 1 + r.nextInt(Math.min(24, labels.count() - lo));
            narrow.low[p] = lo;
            narrow.count[p] = n;
          }
          narrow.changed();
          check(Arrays.equals(session.summary(narrow).clone(), mv.cpuSummary(plan, narrow)),
              "GPU narrow-band summary equals the CPU reference (trial " + trial + ")");
        }
      }
    } else System.out.println("MultiViewSgmRegression: GPU part skipped (no usable OpenCL device)");

    // 3. Coarse-to-fine banding agrees with the full-range solve.
    o.tsgmLevels = 2;
    MultiViewSgm.Solved banded = mv.solve(plan, new double[0]);
    check(banded.levels().size() == 3, "two coarse levels plus full resolution");
    check(banded.bands().total() < full.total() / 2, "bands are narrower than the full range");
    int both = 0, same = 0;
    for (int p = 0; p < W * H; p++) {
      int a = cpu[6 * p], b = banded.summary()[6 * p];
      if (a < 0 || b < 0 || nearEdge(truth, p % W, p / W)) continue;
      both++;
      if (Math.abs(a - b) <= 1) same++;
    }
    check(both > 0 && same >= .98 * both, "banded winners within one label of full-range: " + same + "/" + both);

    // 4. Level table: a level pixel maps to the downsampled canvas position of the full mapping.
    float[] t2 = MultiViewSgm.table(labels, views, 2, (labels.count() - 1) / 2 + 1);
    float[] t1 = MultiViewSgm.table(labels, views);
    int V = views.length;
    for (int j = 0; j < 5; j++)
      for (int v = 0; v < V; v++) {
        int l = 2 * j, at1 = 3 * (l * V + v), at2 = 3 * (j * V + v);
        double xl = 37.0, xf = (xl + .5) * 2 - .5;
        double expected = (xf * t1[at1] + t1[at1 + 1] + .5) / 2 - .5, actual = xl * t2[at2] + t2[at2 + 1];
        check(Math.abs(expected - actual) < 1e-3, "level table x mapping " + expected + " vs " + actual);
      }

    // 5. Options.
    java.nio.file.Path input;
    try {
      input = java.nio.file.Files.createTempDirectory("mvsgm-options");
    } catch (java.io.IOException e) {
      throw new java.io.UncheckedIOException(e);
    }
    String[] base = {"-i", input.toString(), "-o", input.resolve("out").toString(), "--dense-method", "mvsgm"};
    try {
      Options.parse(base);
      check(false, "mvsgm without --fusion-mode image must be rejected");
    } catch (IllegalArgumentException expected) {
      check(expected.getMessage().contains("--fusion-mode image"), "mvsgm requires image fusion");
    }
    List<String> args = new ArrayList<>(List.of(base));
    args.addAll(List.of("--fusion-mode", "image", "--two-view-confidence-threshold", "0.9", "--mvsgm-neighbors", "5"));
    Options parsed = Options.parse(args.toArray(new String[0]));
    check(Double.isNaN(parsed.twoViewConfidenceThreshold) && parsed.mvsgmNeighbors == 5 && parsed.copy().mvsgmNeighbors == 5,
        "mvsgm options parsed, copied; two-view gate off");
    try {
      args.addAll(List.of("--mvsgm-best-k", "7"));
      Options.parse(args.toArray(new String[0]));
      check(false, "best-k above the neighbour count must be rejected");
    } catch (IllegalArgumentException expected) {
      checks++;
    }
    try {
      java.nio.file.Files.deleteIfExists(input);
    } catch (java.io.IOException ignored) {
    }
    return checks;
  }

  /** Within 3 pixels of a depth discontinuity of the true map. */
  static boolean nearEdge(float[] truth, int x, int y) {
    float z = truth[y * W + x];
    for (int dy = -3; dy <= 3; dy++)
      for (int dx = -3; dx <= 3; dx++) {
        int xx = x + dx, yy = y + dy;
        if (xx < 0 || yy < 0 || xx >= W || yy >= H) continue;
        if (truth[yy * W + xx] != z) return true;
      }
    return false;
  }
}
