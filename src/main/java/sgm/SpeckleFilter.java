package sgm;

import java.util.Arrays;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ForkJoinPool;
import java.util.concurrent.Semaphore;
import java.util.concurrent.atomic.AtomicLong;
import java.util.stream.IntStream;

/**
 * Image fusion's 2D speckle filter (FUSION_MEMORY.md): removes 4-connected components of a depth
 * map (0 = no depth) that have fewer than minSize pixels. Two neighbouring pixels p and q connect
 * when the float |z_q - z_p| is at most limit * min(z_p, z_q) in double; the rule is symmetric,
 * so components are the connected components of a graph and do not depend on the scan order.
 *
 * <p>The map is cut into row strips that are labelled in parallel, each in a per-thread buffer:
 *
 * <ol>
 *   <li>per strip (parallel): components by breadth-first search; a component that touches neither
 *       the strip's first nor last row is complete and is removed at once when too small; the
 *       others ("border components") get a node with their size, and the strip's first and last
 *       rows remember each pixel's node;
 *   <li>(sequential, two rows per border) border components are joined across each strip border
 *       with union-find where the two border pixels connect, sizes summed at the root;
 *   <li>per strip with a too-small joined component (parallel): its pixels are found by flooding
 *       from the strip's border pixels of that component, and removed. No strip is labelled twice.
 * </ol>
 *
 * Working memory: per thread 9 bytes per strip pixel (depths, queue, marks), plus per strip its
 * two border rows of node numbers and the union-find nodes; no per-image arrays. The result does
 * not depend on the strip height or the thread count.
 */
final class SpeckleFilter {
  private SpeckleFilter() {}

  static final int MIN_ROWS = 16, MAX_ROWS = 1024;

  /** Strip height that minimises workingBytes for a thread count. */
  static int rows(int h, int threads) {
    int r = (int) Math.round(Math.sqrt(40.0 * h / (9.0 * Math.max(1, threads))));
    return Math.max(1, Math.min(h, Math.max(MIN_ROWS, Math.min(MAX_ROWS, r))));
  }

  /**
   * Upper bound of the filter's heap use: per thread the padded strip buffers (float depths, int
   * queue, byte marks) and a size list; per strip two border rows (int) and at most 2 w border
   * components with their size, parent and total (worst case: every border pixel a component of
   * its own); two rows of depths for the joins.
   */
  static long workingBytes(int w, int h, int rows, int threads) {
    long strips = Math.ceilDiv(h, Math.max(1, rows));
    return Math.min(Math.max(1, threads), strips) * (9L * (rows + 2) * (w + 2) + 8L * w + 4096)
        + strips * (40L * w + 64) + 8L * w;
  }

  /** The largest thread count up to threads whose working set fits budget (at least 1). */
  static int threadsFor(int w, int h, int threads, long budget) {
    int t = Math.max(1, threads);
    while (t > 1 && workingBytes(w, h, rows(h, t), t) > budget) t--;
    return t;
  }

  private static final class Buffers {
    final float[] zs;
    final int[] queue;
    final boolean[] visited;
    int[] sizes = new int[256];

    Buffers(int cells) {
      zs = new float[cells];
      queue = new int[cells];
      visited = new boolean[cells];
    }
  }

  /**
   * Filters z (w x h) in place with strips of the given height on up to threads threads of pool
   * (null or one thread: on the calling thread). Returns the number of removed pixels.
   */
  static long filter(ImageFusion.Raster z, int w, int h, double limit, int minSize, int rows, int threads, ForkJoinPool pool) {
    if (minSize <= 0 || w <= 0 || h <= 0) return 0;
    int stripRows = Math.max(1, Math.min(h, rows)), strips = Math.ceilDiv(h, stripRows);
    int width = w + 2, cells = (stripRows + 2) * width;
    int workers = pool == null ? 1 : Math.max(1, Math.min(threads, strips));
    ConcurrentLinkedQueue<Buffers> free = new ConcurrentLinkedQueue<>();
    Semaphore permits = new Semaphore(workers);
    AtomicLong removed = new AtomicLong();
    int[][] top = new int[strips][], bottom = new int[strips][], sizes = new int[strips][];

    // 1. Label every strip; remove complete small components; keep border components as nodes.
    run(pool, workers, strips, permits, s -> {
      Buffers b = take(free, cells);
      try {
        int y0 = s * stripRows, nr = Math.min(stripRows, h - y0), n = (nr + 2) * width;
        float[] zs = b.zs;
        int[] queue = b.queue;
        boolean[] visited = b.visited;
        load(z, w, y0, nr, zs);
        Arrays.fill(visited, 0, n, false);
        int[] first = new int[w], last = nr > 1 ? new int[w] : first;
        Arrays.fill(first, -1);
        if (last != first) Arrays.fill(last, -1);
        int firstEnd = 2 * width, lastStart = nr * width, nodes = 0;
        long gone = 0;
        for (int y = 1; y <= nr; y++)
          for (int start = y * width + 1, end = start + w; start < end; start++) {
            if (zs[start] <= 0 || visited[start]) continue;
            int tail = flood(zs, visited, queue, 0, start, width, limit);
            boolean touches = false;
            for (int i = 0; i < tail; i++) {
              int p = queue[i];
              if (p < firstEnd) {
                first[p - width - 1] = nodes;
                touches = true;
              }
              if (p >= lastStart) {
                last[p - lastStart - 1] = nodes;
                touches = true;
              }
            }
            if (touches) {
              if (nodes == b.sizes.length) b.sizes = Arrays.copyOf(b.sizes, 2 * nodes);
              b.sizes[nodes++] = tail;
            } else if (tail < minSize) {
              for (int i = 0; i < tail; i++) zs[queue[i]] = 0;
              gone += tail;
            }
          }
        if (gone > 0) {
          store(z, w, y0, nr, zs);
          removed.addAndGet(gone);
        }
        top[s] = first;
        bottom[s] = last;
        sizes[s] = Arrays.copyOf(b.sizes, nodes);
      } finally {
        free.add(b);
      }
    });

    // 2. Join border components across strip borders; sizes are summed at the root (saturating).
    int[] base = new int[strips + 1];
    for (int s = 0; s < strips; s++) base[s + 1] = Math.addExact(base[s], sizes[s].length);
    int count = base[strips];
    if (count == 0) return removed.get();
    int[] parent = new int[count], total = new int[count];
    for (int s = 0; s < strips; s++) {
      System.arraycopy(sizes[s], 0, total, base[s], sizes[s].length);
      sizes[s] = null;
    }
    for (int i = 0; i < count; i++) parent[i] = i;
    float[] above = new float[w], below = new float[w];
    for (int s = 1; s < strips; s++) {
      long row = (long) s * stripRows * w;
      z.read(row - w, above, 0, w);
      z.read(row, below, 0, w);
      int[] na = bottom[s - 1], nb = top[s];
      for (int x = 0; x < w; x++) {
        if (na[x] < 0 || nb[x] < 0) continue;
        float a = above[x], c = below[x];
        double near = Math.min(a, c);
        if (Math.abs(c - a) > limit * near) continue;
        int ra = find(parent, base[s - 1] + na[x]), rb = find(parent, base[s] + nb[x]);
        if (ra == rb) continue;
        parent[rb] = ra;
        total[ra] = (int) Math.min(Integer.MAX_VALUE, (long) total[ra] + total[rb]);
      }
    }
    // Every node points at its root, so the parallel step below only reads.
    for (int i = 0; i < count; i++) parent[i] = find(parent, i);

    // 3. Remove the too-small joined components, flooding from their border pixels.
    run(pool, workers, strips, permits, s -> {
      boolean any = false;
      for (int i = base[s]; i < base[s + 1] && !any; i++) any = total[parent[i]] < minSize;
      if (!any) return;
      Buffers b = take(free, cells);
      try {
        int y0 = s * stripRows, nr = Math.min(stripRows, h - y0), n = (nr + 2) * width;
        float[] zs = b.zs;
        int[] queue = b.queue;
        boolean[] visited = b.visited;
        load(z, w, y0, nr, zs);
        Arrays.fill(visited, 0, n, false);
        int tail = 0;
        for (int edge = 0; edge < 2; edge++) {
          int[] nodes = edge == 0 ? top[s] : bottom[s];
          int at = (edge == 0 ? 1 : nr) * width + 1;
          for (int x = 0; x < w; x++) {
            if (nodes[x] < 0 || visited[at + x] || total[parent[base[s] + nodes[x]]] >= minSize) continue;
            tail = flood(zs, visited, queue, tail, at + x, width, limit);
          }
        }
        for (int i = 0; i < tail; i++) zs[queue[i]] = 0;
        store(z, w, y0, nr, zs);
        removed.addAndGet(tail);
      } finally {
        free.add(b);
      }
    });
    return removed.get();
  }

  /**
   * Breadth-first search from start over the padded strip (a border of zeros around it, so no
   * bounds are tested): appends the component's pixels to queue[from..) and marks them visited.
   * Returns the new end of the queue.
   */
  private static int flood(float[] zs, boolean[] visited, int[] queue, int from, int start, int width, double limit) {
    int head = from, tail = from;
    queue[tail++] = start;
    visited[start] = true;
    while (head < tail) {
      int p = queue[head++];
      float zp = zs[p];
      for (int d = 0; d < 4; d++) {
        int q = d == 0 ? p + 1 : d == 1 ? p - 1 : d == 2 ? p + width : p - width;
        if (visited[q]) continue;
        float zq = zs[q];
        if (zq <= 0) continue;
        double near = Math.min(zp, zq);
        if (Math.abs(zq - zp) > limit * near) continue;
        visited[q] = true;
        queue[tail++] = q;
      }
    }
    return tail;
  }

  private static Buffers take(ConcurrentLinkedQueue<Buffers> free, int cells) {
    Buffers b = free.poll();
    return b != null ? b : new Buffers(cells);
  }

  /** Rows [y0, y0 + nr) of z into the padded buffer (row r at (r + 1) * (w + 2) + 1; zeros around). */
  private static void load(ImageFusion.Raster z, int w, int y0, int nr, float[] zs) {
    int width = w + 2;
    Arrays.fill(zs, 0, width, 0);
    for (int r = 0; r < nr; r++) {
      int at = (r + 1) * width;
      zs[at] = 0;
      z.read((long) (y0 + r) * w, zs, at + 1, w);
      zs[at + w + 1] = 0;
    }
    Arrays.fill(zs, (nr + 1) * width, (nr + 2) * width, 0);
  }

  private static void store(ImageFusion.Raster z, int w, int y0, int nr, float[] zs) {
    for (int r = 0; r < nr; r++) z.write((long) (y0 + r) * w, zs, (r + 1) * (w + 2) + 1, w);
  }

  private static int find(int[] parent, int i) {
    while (parent[i] != i) {
      parent[i] = parent[parent[i]];
      i = parent[i];
    }
    return i;
  }

  /** action(0..n) on the calling thread, or on the pool with at most workers running at once. */
  private static void run(ForkJoinPool pool, int workers, int n, Semaphore permits, java.util.function.IntConsumer action) {
    if (pool == null || workers <= 1) {
      for (int i = 0; i < n; i++) action.accept(i);
      return;
    }
    try {
      pool.submit(() -> IntStream.range(0, n).parallel().forEach(i -> {
        permits.acquireUninterruptibly();
        try {
          action.accept(i);
        } finally {
          permits.release();
        }
      })).get();
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      throw new IllegalStateException("Image fusion interrupted", e);
    } catch (ExecutionException e) {
      if (e.getCause() instanceof RuntimeException r) throw r;
      if (e.getCause() instanceof Error error) throw error;
      throw new IllegalStateException(e.getCause());
    }
  }
}
