package sgm;

import java.io.*;
import java.util.*;

/** Native processed rasters are loaded on demand; metadata and keypoints survive eviction. */
final class RasterCache {
  private static final LinkedHashMap<ImageFrame, Long> resident =
      new LinkedHashMap<>(16, .75f, true);
  private static long used, peak;
  // Frames being decoded outside the lock (prefetch, preload) and their bytes, not yet in `used`.
  // Nobody else touches such a frame's rasters until it leaves this set (pair() waits for it).
  private static final Set<ImageFrame> loading = new HashSet<>();
  private static long reserved;
  private static long prefetched;

  static long limit(Options o) {
    if (o.fusionHostMemoryMb > 0)
      return Math.min((o.imageMemoryMb > 0 ? o.imageMemoryMb : Math.max(1, Math.min(2048, o.memoryMb / 4))) * 1048576L,
          Math.max(1, o.fusionHostMemoryMb / 8) * 1048576L);
    return (long)
            (o.imageMemoryMb > 0 ? o.imageMemoryMb : Math.max(1, Math.min(2048, o.memoryMb / 4)))
        * 1048576;
  }

  static synchronized void pair(ImageFrame a, ImageFrame b) throws IOException {
    awaitLoading(a, b);
    long wanted = a.rasterBytes() + (a == b ? 0 : b.rasterBytes()),
        limit = Math.min(limit(a.options), limit(b.options));
    if (wanted > limit)
      throw new IllegalArgumentException(
          "Processed source image pair needs "
              + ((wanted + 1048575) / 1048576)
              + " MiB of rasters; raise --image-memory-mb or lower dense/alignment resolution");
    long extra =
        (a.gray.empty() ? a.rasterBytes() : 0)
            + (a == b ? 0 : b.gray.empty() ? b.rasterBytes() : 0);
    evict(extra, limit, a, b);
    load(a);
    if (a != b) load(b);
  }

  /** Releases least recently used frames (never a or b) until extra bytes fit the limit. Lock held. */
  private static void evict(long extra, long limit, ImageFrame a, ImageFrame b) {
    var it = resident.entrySet().iterator();
    while (used + reserved + extra > limit && it.hasNext()) {
      var e = it.next();
      if (e.getKey() == a || e.getKey() == b) continue;
      e.getKey().releaseRasters();
      used -= e.getValue();
      it.remove();
    }
  }

  /** Waits (releasing the lock) while another thread decodes a or b. Lock held. */
  private static void awaitLoading(ImageFrame a, ImageFrame b) throws IOException {
    while (loading.contains(a) || loading.contains(b)) {
      try {
        RasterCache.class.wait();
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
        throw new InterruptedIOException("interrupted while waiting for an image decode");
      }
    }
  }

  /**
   * Makes f's rasters resident, decoding them outside the cache lock, so other threads keep
   * reading other resident frames meanwhile (several mvsgm cameras in flight read source regions
   * through the lock). Decoding is deterministic, so the rasters equal those pair() loads; a frame
   * evicted again before the caller's pair() is simply reloaded there.
   */
  static void prefetch(ImageFrame f) throws IOException {
    if (!f.managedRasters) return;
    long bytes = f.rasterBytes();
    synchronized (RasterCache.class) {
      awaitLoading(f, f);
      if (!f.gray.empty() || bytes > limit(f.options)) return;
      long limit = limit(f.options);
      evict(bytes, limit, f, f);
      // Other decodes in progress fill the budget: the caller's pair() loads under the lock.
      if (used + reserved + bytes > limit) return;
      loading.add(f);
      reserved += bytes;
    }
    boolean ok = false;
    try {
      f.loadRasters();
      ok = true;
    } finally {
      synchronized (RasterCache.class) {
        loading.remove(f);
        reserved -= bytes;
        if (ok) {
          load(f);
          prefetched++;
        }
        RasterCache.class.notifyAll();
      }
    }
  }

  /**
   * Decodes frames in parallel, in the given order, while they fit the cache limit together.
   * Decoding is deterministic, so this only moves work earlier; later pair() calls find the rasters
   * resident. Returns the number of frames decoded here.
   */
  static int preload(List<ImageFrame> frames, int threads) throws IOException {
    List<ImageFrame> wanted = new ArrayList<>();
    long budget = 0;
    synchronized (RasterCache.class) {
      for (ImageFrame f : frames) {
        if (wanted.contains(f) || loading.contains(f) || !f.gray.empty()) continue;
        if (budget + used + reserved + f.rasterBytes() > limit(f.options)) break;
        budget += f.rasterBytes();
        wanted.add(f);
      }
      if (wanted.size() < 2 || threads < 2) return 0;
      loading.addAll(wanted);
      reserved += budget;
    }
    boolean ok = false;
    var pool = java.util.concurrent.Executors.newFixedThreadPool(Math.min(threads, wanted.size()));
    try {
      List<java.util.concurrent.Future<?>> tasks = new ArrayList<>();
      for (ImageFrame f : wanted) tasks.add(pool.submit(() -> { f.loadRasters(); return null; }));
      for (var task : tasks) {
        try {
          task.get();
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
          throw new IOException("Image preload interrupted", e);
        } catch (java.util.concurrent.ExecutionException e) {
          if (e.getCause() instanceof IOException io) throw io;
          if (e.getCause() instanceof RuntimeException r) throw r;
          if (e.getCause() instanceof Error error) throw error;
          throw new IOException(e.getCause());
        }
      }
      ok = true;
    } finally {
      pool.shutdown();
      if (!ok) {
        // Let running decodes end before their frames are released.
        try {
          pool.awaitTermination(1, java.util.concurrent.TimeUnit.HOURS);
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
        }
      }
      synchronized (RasterCache.class) {
        loading.removeAll(wanted);
        reserved -= budget;
        for (ImageFrame f : wanted) {
          if (ok) load(f);
          else f.releaseRasters();
        }
        RasterCache.class.notifyAll();
      }
    }
    return wanted.size();
  }

  private static void load(ImageFrame f) throws IOException {
    if (f.gray.empty()) f.loadRasters();
    if (f.managedRasters) {
      if (!resident.containsKey(f)) {
        resident.put(f, f.rasterBytes());
        used += f.rasterBytes();
        peak = Math.max(peak, used);
      } else resident.get(f);
    }
  }

  static synchronized void forget(ImageFrame f) {
    Long old = resident.remove(f);
    if (old != null) used -= old;
  }

  static synchronized long bytes() {
    return used;
  }

  /** Frames decoded by prefetch() (outside the lock) so far. */
  static synchronized long prefetched() {
    return prefetched;
  }

  static synchronized void clear() {
    for (ImageFrame frame : resident.keySet()) frame.releaseRasters();
    resident.clear();
    used = 0;
  }

  static synchronized long peak() {
    return peak;
  }
}
