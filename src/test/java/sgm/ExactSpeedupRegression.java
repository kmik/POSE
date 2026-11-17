package sgm;

import java.nio.file.*;
import java.util.*;

/**
 * Exact replacements from the 2026-11-16 speed work: the strip-parallel speckle filter against
 * the sequential one, and memory-mapped depth-map region writes against channel writes.
 */
final class ExactSpeedupRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    Path root = Files.createTempDirectory(Path.of("target"), "exact speedup ");
    System.out.println("PASS exact speedups checks=" + run(root) + "; fixtures=" + root);
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Files.createDirectories(root);
    speckles();
    regions(root);
    return checks;
  }

  /** Piecewise-smooth random map with holes, steps and isolated specks. */
  private static float[] map(Random r, int w, int h, double holes) {
    float[] a = new float[w * h];
    float[] blocks = new float[64];
    for (int i = 0; i < blocks.length; i++) blocks[i] = r.nextInt(40);
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) {
        int b = (y * 8 / h) * 8 + x * 8 / w;
        float v = blocks[b] + (float) (0.002 * x + 0.003 * y) + (float) (r.nextGaussian() * 0.3);
        if (r.nextDouble() < holes) v = Float.NaN;
        if (r.nextDouble() < 0.01) v = Float.POSITIVE_INFINITY;
        if (r.nextDouble() < 0.02) v += 5 + r.nextInt(5); // specks
        a[y * w + x] = v;
      }
    return a;
  }

  private static void speckles() {
    Random r = new Random(17);
    int[][] sizes = {{1024, 1024}, {1500, 800}, {640, 1700}, {2176, 600}};
    for (int[] s : sizes)
      for (int connectivity : new int[] {4, 8})
        for (double holes : new double[] {0.02, 0.3})
          for (int minimum : new int[] {2, 25, 400}) {
            int w = s[0], h = s[1];
            float[] a = map(r, w, h, holes), b = a.clone();
            long sequential = CensusSgm.removeSpecklesSequential(a, w, h, minimum, 1.0, connectivity);
            long parallel = CensusSgm.removeSpecklesParallel(b, w, h, minimum, 1.0, connectivity);
            boolean same = sequential == parallel;
            for (int i = 0; same && i < a.length; i++)
              same = Float.floatToRawIntBits(a[i]) == Float.floatToRawIntBits(b[i]);
            check(same && sequential > 0,
                "parallel speckles " + w + "x" + h + " c" + connectivity + " holes " + holes + " min " + minimum
                    + ": " + sequential + " vs " + parallel);
          }
    // One long snake crossing every strip border stays one component.
    int w = 300, h = 1200;
    float[] snake = new float[w * h];
    Arrays.fill(snake, Float.NaN);
    for (int y = 0; y < h; y++) snake[y * w + (y % 2 == 0 ? 10 : 11)] = 1;
    for (int y = 0; y < h; y += 2) snake[y * w + 11] = 1;
    float[] copy = snake.clone();
    check(CensusSgm.removeSpecklesParallel(snake, w, h, 1000, 1.0, 4) == CensusSgm.removeSpecklesSequential(copy, w, h, 1000, 1.0, 4)
        && Arrays.equals(snake, copy), "component spanning all strips");
    check(CensusSgm.removeSpeckles(new float[16], 4, 4, 1, 1.0, 4) == 0, "minimum 1 removes nothing");
  }

  private static void regions(Path root) throws Exception {
    Random r = new Random(3);
    Camera camera = new Camera(1000, 1000, 500, 400);
    int W = 700, H = 500;
    Path mapped = root.resolve("mapped.depth"), channel = root.resolve("channel.depth");
    DepthMapFile.create(mapped, camera, W, H);
    DepthMapFile.create(channel, camera, W, H);
    int[][] regions = {{0, 0, 256, 256}, {256, 0, 256, 256}, {512, 0, 188, 256}, {0, 256, 700, 244}, {13, 17, 1, 1}, {699, 499, 1, 1}};
    for (int[] g : regions) {
      int sw = g[2] + 9, sx0 = 4, sy0 = 5;
      float[] src = new float[sw * (g[3] + 11)];
      for (int i = 0; i < src.length; i++) src[i] = i % 97 == 0 ? Float.NaN : (float) r.nextGaussian();
      DepthMapFile.writeRegion(mapped, W, g[0], g[1], g[2], g[3], src, sw, sx0, sy0);
      System.setProperty("sgm.depthMapFile.channelWrites", "true");
      try {
        DepthMapFile.writeRegion(channel, W, g[0], g[1], g[2], g[3], src, sw, sx0, sy0);
      } finally {
        System.clearProperty("sgm.depthMapFile.channelWrites");
      }
    }
    check(Arrays.equals(Files.readAllBytes(mapped), Files.readAllBytes(channel)), "mapped region writes give the same file");
    var back = DepthMapFile.read(mapped);
    check(back.width() == W && back.height() == H, "header kept");
    DepthMapFile.writeRegion(mapped, W, 0, 0, 0, 5, new float[1], 1, 0, 0);
    check(Arrays.equals(Files.readAllBytes(mapped), Files.readAllBytes(channel)), "empty region writes nothing");
    Files.delete(mapped); // unmapped: Windows can delete it
    check(!Files.exists(mapped), "file deletable after writes");
  }
}
