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

    // 6. --mvsgm-edge-trim: a 10x10 near box on a far background loses exactly an R-pixel ring.
    {
      int tw = 40, th = 30;
      float[] lab = new float[tw * th];
      for (int y = 10; y < 20; y++) for (int x = 10; x < 20; x++) lab[y * tw + x] = 10;
      long removed = mv.edgeTrim(lab, tw, th, 2, 2);
      boolean ring = true, inner = true, background = true;
      for (int y = 0; y < th; y++)
        for (int x = 0; x < tw; x++) {
          float v = lab[y * tw + x];
          boolean inBox = x >= 10 && x < 20 && y >= 10 && y < 20, core = x >= 12 && x < 18 && y >= 12 && y < 18;
          if (core) inner &= v == 10;
          else if (inBox) ring &= Float.isNaN(v);
          else background &= v == 0;
        }
      check(ring && inner && background && removed == 64,
          "edge trim removes the 2-pixel ring of a near box only (" + removed + " removed)");
      float[] small = new float[tw * th];
      for (int y = 10; y < 20; y++) for (int x = 10; x < 20; x++) small[y * tw + x] = 1.5f;
      check(mv.edgeTrim(small, tw, th, 2, 2) == 0, "edge trim keeps jumps of at most J labels");
      float[] spikes = new float[tw * th];
      Arrays.fill(spikes, 10);
      spikes[15 * tw + 15] = 0;
      float[] spikes2 = spikes.clone();
      long one = mv.edgeTrim(spikes, tw, th, 2, 2, 1), three = mv.edgeTrim(spikes2, tw, th, 2, 2, 3);
      check(one == 24 && three == 0, "a single far outlier trims its 24 neighbours with N=1 but none with N=3 (" + one + ", " + three + ")");
    }

    // 7. --mvsgm-census-bands color: packed per-band census; identical bands repeat the gray bits,
    // penalties scale with the bits, and a colour solve is as accurate as the grayscale one.
    {
      Options c3 = o.copy();
      c3.mvsgmCensus = 3;
      Options cc = c3.copy();
      cc.mvsgmCensusBands = "color";
      MultiViewSgm gray3 = new MultiViewSgm(c3), col = new MultiViewSgm(cc);
      check(col.bits() == 24 && col.penalty() == 3 * gray3.penalty() && col.p2() >= 3 * gray3.p2() - 3,
          "colour census uses 24 bits and scales penalties");
      long[] one = CensusSgm.census(refGray, W, H, 3)[0];
      long[] packed = MultiViewSgm.censusOf(refGray, new byte[][] {refGray, refGray, refGray}, W, H, 3);
      boolean repeated = true;
      for (int p = 0; p < one.length; p++) repeated &= packed[p] == (one[p] | one[p] << 8 | one[p] << 16);
      check(repeated, "identical colour bands pack the gray census three times");
      MultiViewSgm.Reference cref = new MultiViewSgm.Reference(W, H, refGray, ones, 3, false,
          new byte[][] {refGray, refGray, refGray});
      MultiViewSgm.Warped[] cviews = new MultiViewSgm.Warped[others.length];
      for (int i = 0; i < others.length; i++) {
        byte[] gi = render(others[i], far, near, null);
        cviews[i] = MultiViewSgm.warp(ref, W, H, labels, i + 1, others[i], gi, ones, W, H, 3, false, new byte[][] {gi, gi, gi});
      }
      MultiViewSgm.Plan cplan = new MultiViewSgm.Plan(0, neighbours, labels, cref, cviews);
      DisparityBands cfull = DisparityBands.full(W, H, labels.count(), cref.use);
      int[] csum = col.cpuSummary(cplan, cfull);
      MultiViewSgm.DepthMap cmap = col.select(cplan, csum);
      int cgood = 0, cused = 0;
      for (int y = 10; y < H - 10; y++)
        for (int x = 10; x < W - 10; x++) {
          int p = y * W + x;
          if (!cref.use[p] || nearEdge(truth, x, y)) continue;
          cused++;
          if (cmap.depth()[p] > 0 && Math.abs(cmap.depth()[p] - truth[p]) <= .005 * truth[p]) cgood++;
        }
      check(cused > 0 && cgood >= .9 * cused, "colour-census depth within 0.5% for " + cgood + " of " + cused + " pixels");
      Options gc = cc.copy();
      gc.compute = "auto";
      MultiViewSgm gcol = new MultiViewSgm(gc);
      check(!GpuMvsgmCensus.enabled(gc), "colour census is computed on the CPU");
      if (GpuMultiViewSgm.use(gc, gcol, cplan, cfull)) {
        try (var session = new GpuMultiViewSgm.Session(gc, gcol, cplan, 0)) {
          check(Arrays.equals(session.summary(cfull), csum), "GPU colour-census summary equals the CPU reference");
        }
      }
    }

    // 8. --mvsgm-half-pixel: half-pixel canvases reduce exactly to the whole-pixel ones, the solve
    // is at least as accurate, depths lock less to whole labels, and the GPU equals the CPU.
    {
      Options ho = o.copy();
      ho.mvsgmHalfPixel = true;
      MultiViewSgm half = new MultiViewSgm(ho);
      check(half.canvasScale == 2 && mv.canvasScale == 1, "half-pixel canvas scale");
      MultiViewSgm.Warped[] hviews = new MultiViewSgm.Warped[others.length];
      boolean reduces = true;
      for (int i = 0; i < others.length; i++) {
        byte[] gi = render(others[i], far, near, null);
        hviews[i] = MultiViewSgm.warp(ref, W, H, labels, i + 1, others[i], gi, ones, W, H, mv.census, false, null, 2);
        MultiViewSgm.Warped whole = hviews[i].wholePixel();
        reduces &= hviews[i].w == 2 * views[i].w - 1 && whole.w == views[i].w && whole.h == views[i].h
            && Arrays.equals(whole.gray, views[i].gray) && Arrays.equals(whole.cpuCensus(), views[i].cpuCensus());
      }
      check(reduces, "half-pixel canvases reduce to the whole-pixel canvases bit for bit");
      long[] halfCensus = hviews[0].cpuCensus(), wholeCensus = views[0].cpuCensus();
      boolean evenSame = true;
      for (int y = 4; y < views[0].h - 4; y++)
        for (int x = 4; x < views[0].w - 4; x++)
          evenSame &= halfCensus[2 * y * hviews[0].w + 2 * x] == wholeCensus[y * views[0].w + x];
      check(evenSame, "half-pixel census at whole-pixel samples equals the whole-pixel census");
      MultiViewSgm.Plan hplan = new MultiViewSgm.Plan(0, neighbours, labels, reference, hviews);
      int[] hsum = half.cpuSummary(hplan, full);
      MultiViewSgm.DepthMap hmap = half.select(hplan, hsum);
      int hgood = 0, hused = 0;
      double lockedWhole = 0, lockedHalf = 0;
      for (int y = 10; y < H - 10; y++)
        for (int x = 10; x < W - 10; x++) {
          int p = y * W + x;
          if (!reference.use[p] || nearEdge(truth, x, y)) continue;
          hused++;
          if (hmap.depth()[p] > 0 && Math.abs(hmap.depth()[p] - truth[p]) <= .005 * truth[p]) hgood++;
          // Distance of the fractional label from a whole label (0 = locked, 0.5 = halfway).
          if (hmap.depth()[p] > 0) {
            double l = (1 / hmap.depth()[p] - labels.low()) / labels.step();
            lockedHalf += Math.abs(l - Math.rint(l));
          }
          if (map.depth()[p] > 0) {
            double l = (1 / map.depth()[p] - labels.low()) / labels.step();
            lockedWhole += Math.abs(l - Math.rint(l));
          }
        }
      check(hused > 0 && hgood >= .9 * hused, "half-pixel depth within 0.5% for " + hgood + " of " + hused + " pixels");
      Options dfo = ho.copy();
      dfo.mvsgmSubpixel = "data";
      MultiViewSgm dfit = new MultiViewSgm(dfo);
      MultiViewSgm.DepthMap dmap = dfit.select(hplan, hsum);
      int dgood = 0;
      for (int y = 10; y < H - 10; y++)
        for (int x = 10; x < W - 10; x++) {
          int p = y * W + x;
          if (!reference.use[p] || nearEdge(truth, x, y)) continue;
          if (dmap.depth()[p] > 0 && Math.abs(dmap.depth()[p] - truth[p]) <= .005 * truth[p]) dgood++;
        }
      check(dgood >= .9 * hused && ((Number) dmap.report().get("data_subpixel")).longValue() > hused / 2,
          "data-cost subpixel depth within 0.5% for " + dgood + " of " + hused + " pixels");
      System.out.printf(Locale.ROOT, "MultiViewSgmRegression: mean distance from a whole label %.3f (whole-pixel) vs %.3f (half-pixel)%n",
          lockedWhole / used, lockedHalf / hused);
      Options hg = ho.copy();
      hg.compute = "auto";
      MultiViewSgm hgm = new MultiViewSgm(hg);
      check(GpuMvsgmCensus.enabled(hg) == GpuMvsgmCensus.enabled(g), "half-pixel census uses the GPU census when the grayscale one does");
      if (GpuMultiViewSgm.use(hg, hgm, hplan, full)) {
        try (var session = new GpuMultiViewSgm.Session(hg, hgm, hplan, 0)) {
          check(Arrays.equals(session.summary(full), hsum), "GPU half-pixel summary equals the CPU reference");
          int[] gsums = session.dataSums();
          MultiViewSgm dcpu = new MultiViewSgm(dfo);
          long[] rcs = reference.cpuCensus();
          int[] eachS = new int[hviews.length + 1];
          long diff = 0;
          for (int y = 0; y < H; y++)
            for (int x = 0; x < W; x++) {
              int q = y * W + x, best = hsum[6 * q];
              double c = dcpu.dataOffset(hplan, rcs, x, y, best, eachS), gv = dcpu.sumsOffset(gsums, q, best, hplan.count);
              if (Double.compare(c, gv) != 0) diff++;
            }
          check(diff == 0, "GPU data-fit sums give the CPU offsets for every pixel (" + diff + " differ)");
        }
      }
    }

    // 9. --mvsgm-planar-smooth: a noisy plane becomes the plane, a cone (a crown) is kept, and a
    // roof next to the ground is smoothed up to its edge without taking ground labels.
    {
      int tw = 60, th = 40;
      Random rnd = new Random(3);
      float[] plane = new float[tw * th], cone = new float[tw * th], step = new float[tw * th];
      for (int y = 0; y < th; y++)
        for (int x = 0; x < tw; x++) {
          int p = y * tw + x;
          plane[p] = (float) (10 + .05 * x - .03 * y + .08 * rnd.nextGaussian());
          cone[p] = (float) (20 - 1.5 * Math.hypot(x - 30, y - 20));
          step[p] = (float) ((x < 30 ? 5 : 15) + .08 * rnd.nextGaussian());
        }
      float[] coneIn = cone.clone();
      double before = 0, after = 0;
      float[] planeIn = plane.clone();
      mv.planarSmooth(plane, tw, th, 2, .3, 1.5);
      for (int y = 3; y < th - 3; y++)
        for (int x = 3; x < tw - 3; x++) {
          double ideal = 10 + .05 * x - .03 * y;
          before += Math.abs(planeIn[y * tw + x] - ideal);
          after += Math.abs(plane[y * tw + x] - ideal);
        }
      check(after < .5 * before, "planar smoothing reduces plane noise: " + before + " -> " + after);
      long coneChanged = mv.planarSmooth(cone, tw, th, 2, .3, 1.5);
      int coneTop = 20 * tw + 30;
      check(cone[coneTop] == coneIn[coneTop], "planar smoothing keeps a cone apex (" + coneChanged + " changed)");
      mv.planarSmooth(step, tw, th, 2, .3, 1.5);
      boolean sides = true;
      for (int y = 3; y < th - 3; y++) {
        sides &= Math.abs(step[y * tw + 29] - 5) < .3 && Math.abs(step[y * tw + 30] - 15) < .3;
      }
      check(sides, "planar smoothing keeps both sides of a roof edge apart");
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
    check(parsed.mvsgmCensusBands.equals("gray") && parsed.mvsgmCensus == 5 && parsed.mvsgmLevels == 2 && parsed.p2 == 8
        && parsed.fusionSigmaK == 1 && parsed.mvsgmHalfPixel && parsed.mvsgmSubpixel.equals("data")
        && parsed.mvsgmPlanarSmooth == 3 && parsed.mvsgmPlanarResidual == .5 && parsed.mvsgmEdgeTrim == 2
        && parsed.mvsgmEdgeTrimLabels == 2 && parsed.mvsgmEdgeTrimCount == 2 && parsed.mvsgmEdgeTrimMedian, "mvsgm defaults");
    List<String> own = new ArrayList<>(args);
    own.addAll(List.of("--p2", "20", "--mvsgm-levels", "0", "--mvsgm-half-pixel", "off", "--mvsgm-edge-trim", "off"));
    Options ownParsed = Options.parse(own.toArray(new String[0]));
    check(ownParsed.p2 == 20 && ownParsed.mvsgmLevels == 0 && !ownParsed.mvsgmHalfPixel && ownParsed.mvsgmEdgeTrim == 0
        && ownParsed.mvsgmCensus == 5, "explicit options override the mvsgm defaults");
    List<String> pair = new ArrayList<>(List.of("-i", input.toString(), "-o", input.resolve("out2").toString()));
    Options pairParsed = Options.parse(pair.toArray(new String[0]));
    check(pairParsed.p2 == 24 && pairParsed.fusionSigmaK == 3, "other dense methods keep the global defaults");
    List<String> trimArgs = new ArrayList<>(args);
    trimArgs.addAll(List.of("--mvsgm-edge-trim", "2,3", "--mvsgm-census-window", "3", "--mvsgm-census-bands", "color",
        "--mvsgm-half-pixel", "on", "--mvsgm-subpixel", "data", "--mvsgm-planar-smooth", "3,0.4,2", "--mvsgm-edge-trim", "2,3,4"));
    Options trimmed = Options.parse(trimArgs.toArray(new String[0]));
    check(trimmed.mvsgmEdgeTrim == 2 && trimmed.mvsgmEdgeTrimLabels == 3 && trimmed.copy().mvsgmEdgeTrim == 2
        && trimmed.copy().mvsgmCensusBands.equals("color") && trimmed.mvsgmHalfPixel && trimmed.copy().mvsgmHalfPixel && trimmed.copy().mvsgmSubpixel.equals("data") && trimmed.copy().mvsgmPlanarSmooth == 3 && trimmed.mvsgmPlanarResidual == .4 && trimmed.mvsgmPlanarBand == 2 && trimmed.copy().mvsgmEdgeTrimCount == 4, "edge trim and colour census parsed and copied");
    List<String> offArgs = new ArrayList<>(trimArgs);
    offArgs.addAll(List.of("--mvsgm-edge-trim", "off"));
    check(Options.parse(offArgs.toArray(new String[0])).mvsgmEdgeTrim == 0, "--mvsgm-edge-trim off");
    for (String[] bad : new String[][] {{"--mvsgm-edge-trim", "9"}, {"--mvsgm-edge-trim", "2,0"},
        {"--mvsgm-census-bands", "color", "--mvsgm-census-window", "5"}, {"--mvsgm-census-bands", "rgb"}, {"--mvsgm-half-pixel", "yes"}, {"--mvsgm-subpixel", "cost"}, {"--mvsgm-planar-smooth", "9"}, {"--mvsgm-planar-smooth", "2,0"}, {"--mvsgm-edge-trim", "1,2,9"}}) {
      List<String> b = new ArrayList<>(args);
      b.addAll(List.of(bad));
      try {
        Options.parse(b.toArray(new String[0]));
        check(false, "must be rejected: " + String.join(" ", bad));
      } catch (IllegalArgumentException expected) {
        checks++;
      }
    }
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
