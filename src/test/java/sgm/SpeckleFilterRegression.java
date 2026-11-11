package sgm;

import java.lang.foreign.Arena;
import java.lang.foreign.MemorySegment;
import java.util.Arrays;
import java.util.Random;
import java.util.concurrent.ForkJoinPool;

/**
 * Image fusion's 2D speckle filter (SpeckleFilter; FUSION_MEMORY.md): the parallel strip filter
 * must give the map and the removed count of the two former implementations, kept here as
 * references: the untiled filter (one depth-first search over the whole image) and the sequential
 * strip filter (components per strip, union-find over strip borders, a second labelling sweep).
 *
 * <p>Maps: random depth patches with invalid pixels, spirals, long thin lines and combs that
 * cross every strip border, blobs of sizes around the speckle size on and off strip borders,
 * depth ramps near the connection limit, and NaN depths. Each is filtered with many strip
 * heights and thread counts, on a heap array and on a memory segment.
 */
final class SpeckleFilterRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) {
    System.out.println("PASS speckle filter: " + run());
  }

  static int run() {
    checks = 0;
    ForkJoinPool pool = new ForkJoinPool(8);
    try {
      Random random = new Random(20261111);
      int[] heights = {1, 2, 3, 5, 7, 16, 33, 64, 1000};
      int[] threadCounts = {1, 2, 3, 8};
      for (int scene = 0; scene < 40; scene++) {
        int w = scene < 4 ? 1 + scene : 20 + random.nextInt(140), h = scene % 7 == 6 ? 1 + random.nextInt(3) : 20 + random.nextInt(140);
        int minSize = new int[] {1, 2, 10, 50, 50, 50, 200, 3000}[scene % 8];
        double limit = scene % 3 == 0 ? .0025 : .01;
        float[] map = scene(scene, w, h, minSize, limit, random);
        float[] expected = map.clone();
        long expectedRemoved = untiledReference(expected, w, h, limit, minSize);
        // (The former strip filter did not join NaN depths across strip borders; NaN scenes skip it.)
        for (int rows : scene % 5 == 4 ? new int[0] : new int[] {1, 3, 8, 1000}) {
          float[] strip = map.clone();
          long removed = stripReference(strip, w, h, limit, minSize, rows);
          check(removed == expectedRemoved && same(strip, expected),
              "scene " + scene + ": former strip filter (rows " + rows + ") equals the former untiled filter");
        }
        for (int rows : heights)
          for (int threads : threadCounts) {
            float[] out = map.clone();
            long removed = SpeckleFilter.filter(new ImageFusion.Raster(out), w, h, limit, minSize, rows, threads, pool);
            check(removed == expectedRemoved, "scene " + scene + " rows " + rows + " threads " + threads
                + ": removed " + removed + ", expected " + expectedRemoved);
            check(same(out, expected), "scene " + scene + " rows " + rows + " threads " + threads + ": map differs at "
                + firstDifference(out, expected));
          }
        // The calling thread only (no pool), and a memory segment as on the strip path.
        float[] out = map.clone();
        check(SpeckleFilter.filter(new ImageFusion.Raster(out), w, h, limit, minSize, 4, 1, null) == expectedRemoved
            && same(out, expected), "scene " + scene + ": no pool");
        try (Arena arena = Arena.ofShared()) {
          MemorySegment segment = arena.allocate(4L * w * h, 4);
          MemorySegment.copy(map, 0, segment, MappedFiles.FLOAT, 0, w * h);
          long removed = SpeckleFilter.filter(new ImageFusion.Raster(segment), w, h, limit, minSize, 6, 4, pool);
          float[] back = new float[w * h];
          MemorySegment.copy(segment, MappedFiles.FLOAT, 0, back, 0, w * h);
          check(removed == expectedRemoved && same(back, expected), "scene " + scene + ": memory segment");
        }
        // The default strip height and a thread count limited by a budget.
        int threads = SpeckleFilter.threadsFor(w, h, 8, 20000);
        out = map.clone();
        check(SpeckleFilter.filter(new ImageFusion.Raster(out), w, h, limit, minSize, SpeckleFilter.rows(h, threads), threads, pool)
            == expectedRemoved && same(out, expected), "scene " + scene + ": planned rows and threads");
      }

      // A larger map (many strips really run at once): noise patches and thin structures.
      int w = 900, h = 700;
      float[] map = scene(1, w, h, 50, .01, random);
      spiral(map, w, h, 450, 350, 300, 12f);
      float[] expected = map.clone();
      long expectedRemoved = untiledReference(expected, w, h, .01, 50);
      check(expectedRemoved > 1000, "the large map has speckles to remove (" + expectedRemoved + ")");
      for (int rows : new int[] {1, 9, 16, 41, 128}) {
        float[] out = map.clone();
        check(SpeckleFilter.filter(new ImageFusion.Raster(out), w, h, .01, 50, rows, 8, pool) == expectedRemoved
            && same(out, expected), "large map, rows " + rows);
      }

      // Speckle sizes above 65,536: the former untiled filter removed only the first 65,536 members
      // of a too-small component; the strip filters remove the whole component (documented).
      w = 400;
      h = 300;
      map = new float[w * h];
      for (int y = 10; y < 290; y++) for (int x = 10; x < 290; x++) map[y * w + x] = 10; // 78,400 pixels
      for (int y = 10; y < 290; y++) for (int x = 300; x < 390; x++) map[y * w + x] = 20; // 25,200 pixels
      float[] all = map.clone(), untiled = map.clone();
      long allRemoved = stripReference(all, w, h, .01, 100_000, 1000);
      check(allRemoved == 78_400 + 25_200, "strip reference removes whole components below a size of 100,000");
      check(untiledReference(untiled, w, h, .01, 100_000) == 65_536 + 25_200,
          "former untiled filter stops at 65,536 members of one component");
      for (int rows : new int[] {7, 64, 1000}) {
        float[] out = map.clone();
        check(SpeckleFilter.filter(new ImageFusion.Raster(out), w, h, .01, 100_000, rows, 8, pool) == allRemoved
            && same(out, all), "speckle size above 65,536: whole components removed (rows " + rows + ")");
      }
      // Up to 65,536 nothing differs, also for a component larger than the member buffer was.
      float[] kept = map.clone();
      expected = map.clone();
      expectedRemoved = untiledReference(expected, w, h, .01, 65_536);
      check(expectedRemoved == 25_200, "speckle size 65,536 removes the 25,200-pixel component only");
      check(SpeckleFilter.filter(new ImageFusion.Raster(kept), w, h, .01, 65_536, 16, 8, pool) == expectedRemoved
          && same(kept, expected), "speckle size 65,536 equals the former untiled filter");

      // Memory plan helpers.
      check(SpeckleFilter.rows(13440, 12) >= SpeckleFilter.MIN_ROWS && SpeckleFilter.rows(13440, 12) <= 128,
          "strip height for a large frame");
      check(SpeckleFilter.rows(5, 12) == 5 && SpeckleFilter.rows(1, 1) == 1, "strip height never exceeds the image");
      long large = SpeckleFilter.workingBytes(31520, 13440, SpeckleFilter.rows(13440, 12), 12);
      check(large < 2L * 31520 * 13440, "large frame: working set below 2 bytes per pixel (" + large + ")");
      check(SpeckleFilter.threadsFor(31520, 13440, 12, Long.MAX_VALUE) == 12, "all threads when the budget allows");
      int few = SpeckleFilter.threadsFor(31520, 13440, 12, 150L << 20);
      check(few >= 1 && few < 12 && SpeckleFilter.workingBytes(31520, 13440, SpeckleFilter.rows(13440, few), few) <= 150L << 20,
          "fewer threads for a small budget (" + few + ")");
      check(SpeckleFilter.threadsFor(31520, 13440, 12, 1) == 1, "one thread at least");
    } finally {
      pool.shutdown();
    }
    return checks;
  }

  private static boolean same(float[] a, float[] b) {
    if (a.length != b.length) return false;
    for (int i = 0; i < a.length; i++) if (Float.floatToRawIntBits(a[i]) != Float.floatToRawIntBits(b[i])) return false;
    return true;
  }

  private static int firstDifference(float[] a, float[] b) {
    for (int i = 0; i < a.length; i++) if (Float.floatToRawIntBits(a[i]) != Float.floatToRawIntBits(b[i])) return i;
    return -1;
  }

  private static float[] scene(int scene, int w, int h, int minSize, double limit, Random random) {
    float[] z = new float[w * h];
    switch (scene % 5) {
      case 0 -> {
        // Uncorrelated noise: depths from a few levels, about a third invalid.
        for (int p = 0; p < z.length; p++) z[p] = random.nextInt(3) == 0 ? 0 : 10 + random.nextInt(3) * .5f;
      }
      case 1 -> {
        // Patches of constant depth with holes: components of all sizes.
        int cell = 2 + random.nextInt(9);
        float[] level = new float[(w / cell + 1) * (h / cell + 1)];
        for (int i = 0; i < level.length; i++) level[i] = random.nextInt(4) == 0 ? 0 : 5 + random.nextInt(6);
        for (int y = 0; y < h; y++)
          for (int x = 0; x < w; x++)
            z[y * w + x] = random.nextInt(12) == 0 ? 0 : level[(y / cell) * (w / cell + 1) + x / cell];
      }
      case 2 -> {
        // Thin structures crossing every strip border: vertical lines of random length, a comb
        // joined at the bottom, and spirals; each at its own depth.
        float depth = 10;
        for (int x = 1; x < w; x += 2 + random.nextInt(3)) {
          int y0 = random.nextInt(h), length = 1 + random.nextInt(Math.max(1, 2 * minSize));
          depth += 1;
          for (int y = y0; y < Math.min(h, y0 + length); y++) z[y * w + x] = depth;
        }
        if (w > 30 && h > 30) {
          spiral(z, w, h, w / 3, h / 3, Math.min(w, h) / 4, 200f);
          spiral(z, w, h, 2 * w / 3, 2 * h / 3, 3 + random.nextInt(6), 300f);
        }
      }
      case 3 -> {
        // Blobs with sizes around the speckle size, many of them across rows 8, 16, 33, 64.
        for (int blob = 0; blob < 60; blob++) {
          int size = Math.max(1, minSize - 3 + random.nextInt(7));
          int bw = 1 + random.nextInt(Math.max(1, Math.min(w, 12))), x0 = random.nextInt(Math.max(1, w - bw + 1));
          int y0 = blob % 2 == 0 ? random.nextInt(h) : Math.max(0, new int[] {8, 16, 33, 64}[blob % 4] - 1 - random.nextInt(3));
          float depth = 20 + 2 * blob;
          for (int i = 0, p = 0; i < size; p++) {
            int x = x0 + p % bw, y = y0 + p / bw;
            if (y >= h) break;
            if (x < w) {
              z[y * w + x] = depth;
              i++;
            }
          }
        }
      }
      default -> {
        // Depth ramps with steps around the connection limit, and NaN depths (they pass the
        // z <= 0 test and connect to everything, as in the former filters).
        for (int y = 0; y < h; y++) {
          float v = 10 + random.nextFloat();
          for (int x = 0; x < w; x++) {
            v *= (float) (1 + limit * (.2 + 1.6 * random.nextDouble()) * (random.nextBoolean() ? 1 : -1));
            z[y * w + x] = random.nextInt(9) == 0 ? 0 : random.nextInt(400) == 0 ? Float.NaN : Math.max(.5f, v);
          }
        }
      }
    }
    return z;
  }

  /** A one-pixel-wide rectangular spiral around (cx, cy), written over whatever is there. */
  private static void spiral(float[] z, int w, int h, int cx, int cy, int turns, float depth) {
    int x = cx, y = cy, dx = 1, dy = 0, length = 2;
    for (int leg = 0; leg < 2 * turns; leg++) {
      for (int i = 0; i < length; i++) {
        if (x >= 0 && y >= 0 && x < w && y < h) {
          z[y * w + x] = depth;
          // Keep the arm one pixel wide: clear the outer side.
          int ox = x - dy, oy = y + dx;
          if (i > 0 && ox >= 0 && oy >= 0 && ox < w && oy < h && z[oy * w + ox] != depth) z[oy * w + ox] = 0;
        }
        x += dx;
        y += dy;
      }
      int t = dx;
      dx = -dy;
      dy = t;
      if (leg % 2 == 1) length += 2;
    }
  }

  /** The former untiled filter (ImageFusion.speckles before the parallel filter), unchanged. */
  static long untiledReference(float[] z, int w, int h, double limit, int speckleSize) {
    if (speckleSize <= 0) return 0;
    int[] label = new int[w * h], stack = new int[w * h];
    long removed = 0;
    int next = 0;
    int[] members = new int[Math.min(speckleSize, 1 << 16)];
    for (int start = 0; start < w * h; start++) {
      if (z[start] <= 0 || label[start] != 0) continue;
      next++;
      int top = 0, size = 0, stored = 0;
      stack[top++] = start;
      label[start] = next;
      while (top > 0) {
        int p = stack[--top];
        if (stored < members.length) members[stored++] = p;
        size++;
        int x = p % w, y = p / w;
        for (int d = 0; d < 4; d++) {
          int xx = x + (d == 0 ? 1 : d == 1 ? -1 : 0), yy = y + (d == 2 ? 1 : d == 3 ? -1 : 0);
          if (xx < 0 || yy < 0 || xx >= w || yy >= h) continue;
          int q = yy * w + xx;
          if (label[q] != 0 || z[q] <= 0) continue;
          double near = Math.min(z[p], z[q]);
          if (Math.abs(z[q] - z[p]) > limit * near) continue;
          label[q] = next;
          stack[top++] = q;
        }
      }
      if (size < speckleSize)
        for (int i = 0; i < stored; i++) {
          z[members[i]] = 0;
          removed++;
        }
    }
    return removed;
  }

  /** The former sequential strip filter (ImageFusion.stripSpeckles), on a heap array. */
  static long stripReference(float[] z, int w, int h, double limit, int speckleSize, int stripRows) {
    if (speckleSize <= 0) return 0;
    int rows = Math.max(1, Math.min(h, stripRows)), strips = Math.ceilDiv(h, rows), cells = rows * w;
    float[] zs = new float[cells];
    int[] label = new int[cells], stack = new int[cells], size = new int[cells + 1], node = new int[cells + 1];
    boolean[] edge = new boolean[cells + 1];
    int[] parent = new int[1 << 12];
    long[] total = new long[1 << 12];
    int nodes = 0;
    int[] firstNode = new int[strips], prevNode = new int[w];
    float[] prevZ = new float[w];
    for (int s = 0; s < strips; s++) {
      int y0 = s * rows, nr = Math.min(rows, h - y0);
      System.arraycopy(z, y0 * w, zs, 0, nr * w);
      int k = components(zs, w, nr, limit, label, stack, size, edge);
      firstNode[s] = nodes;
      for (int i = 1; i <= k; i++) {
        if (!edge[i]) continue;
        if (nodes == parent.length) {
          parent = Arrays.copyOf(parent, 2 * nodes);
          total = Arrays.copyOf(total, 2 * nodes);
        }
        parent[nodes] = nodes;
        total[nodes] = size[i];
        node[i] = nodes++;
      }
      if (s > 0)
        for (int x = 0; x < w; x++) {
          float a = prevZ[x], b = zs[x];
          if (prevNode[x] < 0 || b <= 0) continue;
          double near = Math.min(a, b);
          if (Math.abs(b - a) > limit * near) continue;
          int ra = find(parent, prevNode[x]), rb = find(parent, node[label[x]]);
          if (ra == rb) continue;
          parent[rb] = ra;
          total[ra] += total[rb];
        }
      int last = (nr - 1) * w;
      for (int x = 0; x < w; x++) {
        prevZ[x] = zs[last + x];
        prevNode[x] = zs[last + x] > 0 ? node[label[last + x]] : -1;
      }
    }
    long removed = 0;
    for (int s = 0; s < strips; s++) {
      int y0 = s * rows, nr = Math.min(rows, h - y0);
      System.arraycopy(z, y0 * w, zs, 0, nr * w);
      int k = components(zs, w, nr, limit, label, stack, size, edge);
      int next = firstNode[s];
      for (int i = 1; i <= k; i++) node[i] = edge[i] ? next++ : -1;
      boolean changed = false;
      for (int p = 0; p < nr * w; p++) {
        int l = label[p];
        if (l == 0) continue;
        long components = node[l] < 0 ? size[l] : total[find(parent, node[l])];
        if (components < speckleSize) {
          zs[p] = 0;
          removed++;
          changed = true;
        }
      }
      if (changed) System.arraycopy(zs, 0, z, y0 * w, nr * w);
    }
    return removed;
  }

  private static int find(int[] parent, int i) {
    while (parent[i] != i) {
      parent[i] = parent[parent[i]];
      i = parent[i];
    }
    return i;
  }

  private static int components(float[] zs, int w, int rows, double limit, int[] label, int[] stack, int[] size, boolean[] edge) {
    Arrays.fill(label, 0, w * rows, 0);
    int next = 0;
    for (int start = 0; start < w * rows; start++) {
      if (zs[start] <= 0 || label[start] != 0) continue;
      next++;
      int top = 0, count = 0;
      boolean touches = false;
      stack[top++] = start;
      label[start] = next;
      while (top > 0) {
        int p = stack[--top];
        count++;
        int x = p % w, y = p / w;
        if (y == 0 || y == rows - 1) touches = true;
        for (int d = 0; d < 4; d++) {
          int xx = x + (d == 0 ? 1 : d == 1 ? -1 : 0), yy = y + (d == 2 ? 1 : d == 3 ? -1 : 0);
          if (xx < 0 || yy < 0 || xx >= w || yy >= rows) continue;
          int q = yy * w + xx;
          if (label[q] != 0 || zs[q] <= 0) continue;
          double near = Math.min(zs[p], zs[q]);
          if (Math.abs(zs[q] - zs[p]) > limit * near) continue;
          label[q] = next;
          stack[top++] = q;
        }
      }
      size[next] = count;
      edge[next] = touches;
    }
    return next;
  }
}
