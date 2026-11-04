package sgm;

import java.util.*;

/** Lossless sparse per-camera measurement atlas. Every available pair layer retains its bits. */
final class CameraEvidence {
  static final class Image {
    int[] mask, starts;
    long[] values;

    Image(int[] mask, int[] starts, long[] values) {
      this.mask = mask;
      this.starts = starts;
      this.values = values;
    }

    int pixels() {
      return mask.length;
    }

    long cells() {
      return (long) mask.length + values.length;
    }

    long cell(long at) {
      return at < mask.length
          ? ((long) mask[(int) at] << 32) | (starts[(int) at] & 0xffffffffL)
          : values[Math.toIntExact(at - mask.length)];
    }

    long value(int pixel, int rank) {
      int bit = 1 << rank, m = mask[pixel];
      if ((m & bit) == 0) return 0;
      return values[starts[pixel] + Integer.bitCount(m & (bit - 1))];
    }
  }

  final Image[] images;
  final long bytes, measurements, allocationBytes;

  private CameraEvidence(Image[] images, long bytes, long measurements, long allocationBytes) {
    this.images = images;
    this.bytes = bytes;
    this.measurements = measurements;
    this.allocationBytes = allocationBytes;
  }

  long cells() {
    long count = 0;
    for (Image image : images) count += image.cells();
    return count;
  }

  void clear() {
    for (Image image : images) {
      image.mask = new int[0];
      image.starts = new int[0];
      image.values = new long[0];
    }
  }

  static CameraEvidence build(
      List<List<DepthFusion.Endpoint>> endpoints,
      int[] widths,
      int[] heights,
      long budget,
      long rawBytes,
      boolean automatic) {
    int count = endpoints.size();
    Image[] images = new Image[count];
    int[] sizes = new int[count];
    long[] counts = new long[count];
    long bytes = 0, entries = 0, allocation = 0;
    var admission = MemorySafety.snapshot();
    budget = admission.allowance(budget);
    for (var list : endpoints) if (list.size() > 32) return null;
    for (int image = 0; image < count; image++) {
      int n =
          endpoints.get(image).isEmpty() ? 0 : Math.multiplyExact(widths[image], heights[image]);
      sizes[image] = n;
      allocation += allocation(n, 0, admission.regionBytes());
    }
    if (allocation > budget) {
      System.out.println(
          "Camera evidence: directory peak exceeds safe heap/commit allowance; retaining paged"
              + " pairs");
      return null;
    }
    try (var work = WorkProgress.start("Count camera evidence measurements", count, "cameras")) {
      for (int image = 0; image < count; image++) {
        var maps = endpoints.get(image);
        int n = sizes[image];
        long measurements = 0;
        for (int rank = 0; rank < maps.size(); rank++) {
          DepthRaster map = maps.get(rank).map();
          for (int p = 0; p < n; p++)
            if (!(map.get(p) <= 0)) {
              measurements++;
            }
        }
        if (measurements > Integer.MAX_VALUE - 8) return null;
        bytes += 8L * (n + 1) + measurements * 8;
        counts[image] = measurements;
        allocation +=
            MemorySafety.arrayBytes(measurements, 8, admission.regionBytes())
                - MemorySafety.arrayBytes(0, 8, admission.regionBytes());
        if (allocation > budget) return null;
        entries += measurements;
        work.update(image + 1);
      }
      work.done();
    }
    if (automatic && bytes >= rawBytes * .8) return null;
    if (!MemorySafety.admits(allocation, budget)) return null;
    try (var work = WorkProgress.start("Construct camera evidence atlas", count, "cameras")) {
      for (int image = 0; image < count; image++) {
        long remaining = 0;
        for (int j = image; j < count; j++)
          remaining += allocation(sizes[j], counts[j], admission.regionBytes());
        if (!MemorySafety.admits(remaining, budget)) return null;
        int[] mask = new int[sizes[image]], starts = new int[mask.length + 1];
        var maps = endpoints.get(image);
        for (int rank = 0; rank < maps.size(); rank++)
          for (int p = 0; p < mask.length; p++)
            if (!(maps.get(rank).map().get(p) <= 0)) mask[p] |= 1 << rank;
        for (int p = 0; p < mask.length; p++)
          starts[p + 1] = Math.addExact(starts[p], Integer.bitCount(mask[p]));
        long[] values = new long[starts[mask.length]];
        for (int p = 0; p < mask.length; p++) {
          int bits = mask[p], at = starts[p];
          while (bits != 0) {
            int rank = Integer.numberOfTrailingZeros(bits);
            bits &= bits - 1;
            DepthRaster map = maps.get(rank).map();
            values[at++] =
                ((long) map.offset(p) << 32) | (Float.floatToRawIntBits(map.get(p)) & 0xffffffffL);
          }
        }
        images[image] = new Image(mask, starts, values);
        work.update(image + 1);
      }
      work.done();
      return new CameraEvidence(images, bytes, entries, allocation);
    }
  }

  static long allocation(int pixels, long entries, long region) {
    return MemorySafety.arrayBytes(pixels, 4, region)
        + MemorySafety.arrayBytes((long) pixels + 1, 4, region)
        + MemorySafety.arrayBytes(entries, 8, region);
  }
}
