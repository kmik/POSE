package sgm;

import java.util.*;

/**
 * Opt-in depth-edge options: --census-edge-mask, --census-shift and --disparity-edge-erosion.
 *
 * <ul>
 *   <li>Off: no OpenCL extension, cost() == value(), and a mask that keeps every comparison
 *       reproduces the ordinary disparities exactly.
 *   <li>Erosion: exact pixels removed on a hand-made step, both sides, holes untouched.
 *   <li>Synthetic step-edge scene (textured foreground block over a textured background, with true
 *       occlusions): wrong and fattened disparities next to the jump are measured for each option.
 *       Erosion can only remove pixels, so it must reduce wrong ones.
 *   <li>CPU and GPU (full and banded TSGM search) give identical disparities with the new costs,
 *       when an OpenCL device is present.
 * </ul>
 */
final class DepthEdgeRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) {
    nu.pattern.OpenCV.loadLocally();
    System.out.println("PASS depth-edge options: " + run());
  }

  static final int W = 320, H = 160, FG = 18, BG = 6;
  static final int X0 = 120, X1 = 220, Y0 = 40, Y1 = 120;

  static boolean foreground(int x, int y) {
    return x >= X0 && x < X1 && y >= Y0 && y < Y1;
  }

  /** Left/right images of the scene; textures are fixed in left-image coordinates. */
  static byte[][] scene() {
    Random random = new Random(7);
    int n = W * H;
    int[] fg = new int[n], bg = new int[n];
    for (int i = 0; i < n; i++) {
      bg[i] = 60 + random.nextInt(50); // darker background
      fg[i] = 150 + random.nextInt(50); // brighter roof
    }
    byte[] left = new byte[n], right = new byte[n];
    for (int y = 0; y < H; y++)
      for (int x = 0; x < W; x++) {
        int p = y * W + x;
        left[p] = (byte) (foreground(x, y) ? fg[p] : bg[p]);
        int xf = x + FG, xb = Math.min(W - 1, x + BG);
        right[p] =
            (byte)
                (xf < W && foreground(xf, y) ? fg[y * W + xf] : bg[y * W + xb]); // nearest surface
      }
    return new byte[][] {left, right};
  }

  /** True left disparity, or NaN where the left pixel is occluded in the right image. */
  static float truth(int x, int y) {
    if (foreground(x, y)) return FG;
    int xr = x - BG, xf = xr + FG;
    return xf < W && foreground(xf, y) ? Float.NaN : BG;
  }

  /** {valid, wrong (>1 px from truth or valid in occlusion), wrong within 4 px of the jump}. */
  static long[] score(float[] d) {
    long valid = 0, wrong = 0, edgeWrong = 0;
    for (int y = 8; y < H - 8; y++)
      for (int x = 40; x < W - 8; x++) {
        float v = d[y * W + x];
        if (!Float.isFinite(v)) continue;
        valid++;
        float t = truth(x, y);
        boolean bad = !Float.isFinite(t) || Math.abs(v - t) > 1;
        if (!bad) continue;
        wrong++;
        boolean nearEdge = false;
        for (int dy = -4; dy <= 4 && !nearEdge; dy++)
          for (int dx = -4; dx <= 4 && !nearEdge; dx++)
            nearEdge = foreground(x + dx, y + dy) != foreground(x, y);
        if (nearEdge) edgeWrong++;
      }
    return new long[] {valid, wrong, edgeWrong};
  }

  static Options base(String compute, String search) {
    Options o = TestSupport.options();
    o.compute = compute;
    o.pairwiseSearch = search;
    o.minDisparity = 0;
    o.disparities = 32;
    o.disparitiesExplicit = true;
    o.speckleSize = 0;
    return o;
  }

  static void windowed(byte[][] images, boolean device) {
    Options o = base(device ? "gpu" : "cpu", "full");
    o.censusEdgeMask = 24;
    o.censusShift = 2;
    o.disparityEdgeErosion = 2;
    o.speckleSize = 7;
    o.memoryMb = 256;
    o.gpuMemoryMb = 256;
    o.gpuStripRows = 13;
    o.gpuDevice = Integer.getInteger("sgm.test.gpu", 0);
    try {
      o.output = java.nio.file.Files.createTempDirectory(java.nio.file.Paths.get("target"), "depth-edge-");
      float[] reference = disparities(o, images);
      int n = W * H, differ = 0;
      try (var store = new WindowedRasters(o)) {
        var left = store.new Image(W, H);
        var right = store.new Image(W, H);
        for (int i = 0; i < n; i++) {
          left.gray.set(i, images[0][i] & 255);
          right.gray.set(i, images[1][i] & 255);
          left.mask.set(i, 255);
          right.mask.set(i, 255);
          left.color.set(i, (images[0][i] & 255) * 0x010101);
          right.color.set(i, (images[1][i] & 255) * 0x010101);
        }
        try (var matcher = new WindowedStereo(store, left, right, o)) {
          matcher.compute();
          for (int p = 0; p < n; p++)
            if (Float.floatToIntBits(reference[p])
                != Float.floatToIntBits(matcher.get(p, WindowedStereo.FINAL))) differ++;
          check(
              matcher.stats.edgeErosionRejected > 0,
              "Windowed erosion rejects pixels (" + (device ? "GPU" : "CPU") + ")");
        }
      }
      check(
          differ == 0,
          "Windowed equals in-memory matching with mask, shift 2 and erosion ("
              + (device ? "GPU" : "CPU")
              + "): "
              + differ
              + " pixels differ");
    } catch (java.io.IOException e) {
      throw new java.io.UncheckedIOException(e);
    }
  }

  static float[] disparities(Options o, byte[][] images) {
    return new CensusSgm(o).compute(images[0], images[1], W, H).disparity;
  }

  static int run() {
    checks = 0;
    Options off = base("cpu", "full");
    check(StereoCost.gpuSource(off).isEmpty(), "Default options add no OpenCL cost extension");

    // Erosion on a hand-made row: jump between x=3 (5) and x=4 (9); NaN hole stays untouched.
    float[] row = {5, 5, 5, 5, 9, 9, 9, 9, Float.NaN, 9};
    byte[] reasons = new byte[row.length];
    long removed = CensusSgm.erodeDisparityEdges(row, row.length, 1, 1, 2.0, reasons);
    check(
        removed == 2 && Float.isNaN(row[3]) && Float.isNaN(row[4]) && row[2] == 5 && row[5] == 9
            && row[9] == 9 && reasons[3] == 8 && reasons[4] == 8 && reasons[2] == 0,
        "Erosion radius 1 removes exactly one pixel on each side: " + Arrays.toString(row));
    float[] wide = {5, 5, 5, 5, 9, 9, 9, 9};
    check(
        CensusSgm.erodeDisparityEdges(wide, wide.length, 1, 2, 2.0, null) == 4
            && Float.isNaN(wide[2]) && Float.isNaN(wide[5]) && wide[1] == 5 && wide[6] == 9,
        "Erosion radius 2 removes two pixels on each side: " + Arrays.toString(wide));
    // Occlusion gap of 3 between disparities 18 and 6: one pixel at each end is removed; a gap wider
    // than difference + 2*radius (an untextured hole) is left alone.
    float n = Float.NaN;
    float[] gap = {18, 18, 18, n, n, n, 6, 6, 6};
    check(
        CensusSgm.erodeDisparityEdges(gap, gap.length, 1, 1, 2.0, null) == 2
            && Float.isNaN(gap[2]) && Float.isNaN(gap[6]) && gap[1] == 18 && gap[7] == 6,
        "Erosion removes the run ends across an occlusion gap: " + Arrays.toString(gap));
    float[] hole = new float[24];
    Arrays.fill(hole, n);
    hole[0] = 9;
    hole[23] = 6;
    check(
        CensusSgm.erodeDisparityEdges(hole, hole.length, 1, 1, 2.0, null) == 0,
        "Erosion ignores jumps across gaps wider than the disparity difference + 2*radius");
    float[] small = {5, 5, 6.5f, 6.5f};
    check(
        CensusSgm.erodeDisparityEdges(small, small.length, 1, 1, 2.0, null) == 0,
        "Steps up to the jump threshold are kept");

    byte[][] images = scene();
    float[] ordinary = disparities(off, images);

    // A mask keeping every comparison reproduces ordinary census exactly (CPU path).
    Options all = off.copy();
    all.censusEdgeMask = 255;
    check(
        Arrays.equals(ordinary, disparities(all, images)),
        "Census edge mask 255 keeps all comparisons and matches ordinary census");

    Map<String, Options> variants = new LinkedHashMap<>();
    variants.put("ordinary", off);
    Options mask = off.copy();
    mask.censusEdgeMask = 24;
    variants.put("edge mask 24", mask);
    Options shift = off.copy();
    shift.censusShift = 1;
    variants.put("shift 1", shift);
    Options both = mask.copy();
    both.censusShift = 1;
    variants.put("mask 24 + shift 1", both);
    Options erosion = off.copy();
    erosion.disparityEdgeErosion = 2;
    variants.put("erosion 2", erosion);
    Options combined = both.copy();
    combined.disparityEdgeErosion = 2;
    variants.put("mask + shift + erosion 2", combined);

    Map<String, long[]> scores = new LinkedHashMap<>();
    Map<String, float[]> cpu = new LinkedHashMap<>();
    for (var e : variants.entrySet()) {
      float[] d = e.getKey().equals("ordinary") ? ordinary : disparities(e.getValue(), images);
      cpu.put(e.getKey(), d);
      long[] s = score(d);
      scores.put(e.getKey(), s);
      System.out.printf(
          Locale.ROOT,
          "Depth-edge scene %-26s valid=%6d wrong=%5d wrong-near-jump=%5d%n",
          e.getKey(),
          s[0],
          s[1],
          s[2]);
    }
    long[] o = scores.get("ordinary"), er = scores.get("erosion 2");
    check(o[2] > 0, "The ordinary matcher shows wrong disparities at the jump: " + o[2]);
    check(er[2] < o[2] && er[1] <= o[1], "Erosion reduces wrong disparities next to the jump");
    check(
        scores.get("edge mask 24")[2] < o[2],
        "Edge-masked census reduces fattening next to the jump");
    check(
        scores.get("mask 24 + shift 1")[2] < o[2],
        "Mask plus shiftable windows reduce fattening next to the jump");

    // The disk-backed windowed matcher equals the in-memory one with all options (13-row windows:
    // shifted windows, masks and erosion cross window borders).
    boolean gpu = !GpuSgm.devices().isEmpty();
    for (boolean device : gpu ? new boolean[] {false, true} : new boolean[] {false})
      windowed(images, device);

    // CPU/GPU equality for the new costs (full search and banded TSGM search).
    if (!gpu) {
      System.out.println("DepthEdgeRegression: GPU comparison skipped (no OpenCL device)");
      return checks;
    }
    for (String search : List.of("full", "tsgm"))
      for (String name : List.of("edge mask 24", "shift 1", "mask 24 + shift 1")) {
        Options c = variants.get(name).copy(), g = c.copy();
        c.pairwiseSearch = search;
        g.pairwiseSearch = search;
        g.compute = "gpu";
        float[] reference = search.equals("full") ? cpu.get(name) : disparities(c, images);
        float[] device = disparities(g, images);
        int differ = 0;
        for (int i = 0; i < device.length; i++)
          if (Float.floatToIntBits(device[i]) != Float.floatToIntBits(reference[i])) differ++;
        check(differ == 0, "GPU equals CPU, " + search + " search, " + name + ": " + differ + " pixels differ");
      }
    return checks;
  }
}
