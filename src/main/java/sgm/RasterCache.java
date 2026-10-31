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

  static synchronized long peak() {
    return peak;
  }
}
