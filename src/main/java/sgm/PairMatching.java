package sgm;

import java.io.*;
import java.util.*;
import java.util.concurrent.*;
import java.util.function.Consumer;
import org.opencv.core.*;

/**
 * Concurrent pair matching with results consumed in the original pair order.
 *
 * <p>FLANN index construction and RANSAC draw from OpenCV's thread-local RNG. In parallel mode each
 * pair reseeds that RNG from its image IDs before the descriptor search and again before geometric
 * verification, so a pair's result depends only on its own inputs, not on thread count, scheduling
 * or earlier pairs. Because a search is then a pure function of its descriptors and seed, a search
 * already made by focal initialization on byte-identical descriptors is reused by stage 03 instead
 * of being repeated. Serial mode keeps the earlier single RNG stream across pairs.
 */
final class PairMatching {
  private static final Map<String, Reciprocal> memo = new HashMap<>();
  private static long memoBytes;

  private PairMatching() {}

  static boolean parallel(Options o) {
    return o.pairMatching.equals("parallel");
  }

  static int threads(Options o) {
    return parallel(o) ? Math.max(1, o.cores) : 1;
  }

  interface Task<T> {
    T run(int[] edge) throws IOException;
  }

  interface Sink<T> {
    void accept(int[] edge, T result) throws IOException;
  }

  /** Runs {@code task} for each edge on up to {@code threads} workers; {@code sink} sees edges in order. */
  static <T> void ordered(List<int[]> edges, int threads, Task<T> task, Sink<T> sink)
      throws IOException {
    if (threads <= 1 || edges.size() < 2) {
      for (int[] edge : edges) sink.accept(edge, task.run(edge));
      return;
    }
    ExecutorService pool =
        Executors.newFixedThreadPool(
            Math.min(threads, edges.size()),
            r -> {
              Thread t = new Thread(r, "pair-matching");
              t.setDaemon(true);
              return t;
            });
    ArrayDeque<Future<T>> queue = new ArrayDeque<>();
    try {
      int next = 0;
      for (int done = 0; done < edges.size(); done++) {
        // Bounded look-ahead keeps finished-but-unconsumed match lists small.
        while (next < edges.size() && queue.size() < 2 * threads) {
          int[] edge = edges.get(next++);
          queue.add(pool.submit(() -> task.run(edge)));
        }
        sink.accept(edges.get(done), join(queue.poll()));
      }
    } finally {
      pool.shutdownNow();
      try {
        // Workers may still hold frames; let them finish before callers release them.
        pool.awaitTermination(10, TimeUnit.MINUTES);
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
      }
    }
  }

  private static <T> T join(Future<T> task) throws IOException {
    try {
      return task.get();
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      throw new IOException("Pair matching interrupted", e);
    } catch (ExecutionException e) {
      Throwable cause = e.getCause();
      if (cause instanceof IOException io) throw io;
      if (cause instanceof RuntimeException r) throw r;
      if (cause instanceof Error error) throw error;
      throw new IOException(cause);
    }
  }

  /** Seeds this thread's OpenCV RNG from the ordered image pair and matching phase. */
  static void seed(ImageFrame a, ImageFrame b, int phase) {
    long h = 0x9E3779B97F4A7C15L * (a.id + 1L);
    h ^= 0xC2B2AE3D27D4EB4FL * (b.id + 1L) + phase;
    h ^= h >>> 31;
    h *= 0xBF58476D1CE4E5B9L;
    h ^= h >>> 29;
    Core.setRNGSeed((int) (h ^ (h >>> 32)));
  }

  /**
   * {@link FeatureAlgorithms#reciprocal} for one pair. In parallel mode the search is seeded, can
   * be served from an identical earlier search (reported in {@code reused}), and is stored for
   * reuse when {@code remember}.
   */
  static int reciprocal(
      Options o,
      ImageFrame a,
      ImageFrame b,
      List<MatOfDMatch> forward,
      Map<Integer, Integer> reverse,
      Consumer<String> progress,
      boolean remember,
      boolean[] reused) {
    if (!parallel(o))
      return FeatureAlgorithms.reciprocal(
          o, a.descriptors, b.descriptors, forward, reverse, progress);
    seed(a, b, 1);
    String key = key(o, a, b);
    Reciprocal hit;
    synchronized (PairMatching.class) {
      hit = key == null ? null : memo.get(key);
    }
    int searched;
    if (hit != null) {
      progress.accept("reusing identical earlier descriptor search");
      searched = hit.restore(forward, reverse);
      if (reused != null) reused[0] = true;
    } else {
      searched =
          FeatureAlgorithms.reciprocal(o, a.descriptors, b.descriptors, forward, reverse, progress);
      if (remember && key != null) {
        Reciprocal copy = new Reciprocal(forward, reverse, searched);
        long limit = (long) o.descriptorMemoryMb * 1048576;
        synchronized (PairMatching.class) {
          if (memoBytes + copy.bytes() <= limit && memo.putIfAbsent(key, copy) == null)
            memoBytes += copy.bytes();
        }
      }
    }
    // Geometric verification sees the same RNG state whether or not the search was reused.
    seed(a, b, 2);
    return searched;
  }

  static synchronized void clear() {
    memo.clear();
    memoBytes = 0;
  }

  static synchronized int remembered() {
    return memo.size();
  }

  private static String key(Options o, ImageFrame a, ImageFrame b) {
    if (a.descriptorDigest == null || b.descriptorDigest == null) return null;
    return a.id
        + ":"
        + b.id
        + ":"
        + a.descriptorDigest
        + ":"
        + b.descriptorDigest
        + ":"
        + o.descriptorMatcher
        + ":"
        + Double.doubleToLongBits(o.ratio);
  }

  /** Compact copy of one search; {@link #restore} rebuilds equal forward rows and decisions. */
  private static final class Reciprocal {
    final int[] counts, query, train, image, reverseKeys, reverseValues;
    final float[] distance;
    final int searched;

    Reciprocal(List<MatOfDMatch> forward, Map<Integer, Integer> reverse, int searched) {
      this.searched = searched;
      counts = new int[forward.size()];
      List<DMatch> all = new ArrayList<>();
      for (int i = 0; i < forward.size(); i++) {
        DMatch[] row = forward.get(i).toArray();
        counts[i] = row.length;
        all.addAll(Arrays.asList(row));
      }
      query = new int[all.size()];
      train = new int[all.size()];
      image = new int[all.size()];
      distance = new float[all.size()];
      for (int i = 0; i < all.size(); i++) {
        DMatch m = all.get(i);
        query[i] = m.queryIdx;
        train[i] = m.trainIdx;
        image[i] = m.imgIdx;
        distance[i] = m.distance;
      }
      reverseKeys = new int[reverse.size()];
      reverseValues = new int[reverse.size()];
      int i = 0;
      for (var e : reverse.entrySet()) {
        reverseKeys[i] = e.getKey();
        reverseValues[i++] = e.getValue();
      }
    }

    long bytes() {
      return 4L * (counts.length + 4L * query.length + 2L * reverseKeys.length) + 128;
    }

    int restore(List<MatOfDMatch> forward, Map<Integer, Integer> reverse) {
      int at = 0;
      for (int count : counts) {
        DMatch[] row = new DMatch[count];
        for (int k = 0; k < count; k++, at++)
          row[k] = new DMatch(query[at], train[at], image[at], distance[at]);
        forward.add(new MatOfDMatch(row));
      }
      for (int i = 0; i < reverseKeys.length; i++) reverse.put(reverseKeys[i], reverseValues[i]);
      return searched;
    }
  }
}
