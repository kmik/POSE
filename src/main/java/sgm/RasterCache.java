package sgm;

import java.io.*;
import java.util.*;

/** Native processed rasters are loaded on demand; metadata and keypoints survive eviction. */
final class RasterCache {
  private static final LinkedHashMap<ImageFrame, Long> resident =
      new LinkedHashMap<>(16, .75f, true);
  private static long used, peak;

  static long limit(Options o) {
    return (long)
            (o.imageMemoryMb > 0 ? o.imageMemoryMb : Math.max(1, Math.min(2048, o.memoryMb / 4)))
        * 1048576;
  }

  static synchronized void pair(ImageFrame a, ImageFrame b) throws IOException {
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
    var it = resident.entrySet().iterator();
    while (used + extra > limit && it.hasNext()) {
      var e = it.next();
      if (e.getKey() == a || e.getKey() == b) continue;
      e.getKey().releaseRasters();
      used -= e.getValue();
      it.remove();
    }
    load(a);
    if (a != b) load(b);
  }

  /**
   * Decodes frames in parallel, in the given order, while they fit the cache limit together.
   * Decoding is deterministic, so this only moves work earlier; later pair() calls find the rasters
   * resident. Returns the number of frames decoded here.
   */
  static int preload(List<ImageFrame> frames, int threads) throws IOException {
    List<ImageFrame> wanted = new ArrayList<>();
    synchronized (RasterCache.class) {
      long budget = 0;
      for (ImageFrame f : frames) {
        if (wanted.contains(f) || !f.gray.empty()) continue;
        if (budget + used + f.rasterBytes() > limit(f.options)) break;
        budget += f.rasterBytes();
        wanted.add(f);
      }
    }
    if (wanted.size() < 2 || threads < 2) return 0;
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
    } finally {
      pool.shutdown();
    }
    synchronized (RasterCache.class) {
      for (ImageFrame f : wanted) load(f);
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

  static synchronized void clear() {
    for (ImageFrame frame : resident.keySet()) frame.releaseRasters();
    resident.clear();
    used = 0;
  }

  static synchronized long peak() {
    return peak;
  }
}
