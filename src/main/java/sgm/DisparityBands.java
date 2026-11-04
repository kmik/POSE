package sgm;

import java.util.Arrays;

/**
 * Per-pixel contiguous label bands within one pair's disparity range. Labels are indices relative
 * to the matching minimum disparity (0..D-1). Pixels without labels have count 0.
 */
final class DisparityBands {
  final int width, height, labels;
  final int[] low, count;
  private int[] offset;
  private long total = -1;

  DisparityBands(int width, int height, int labels) {
    this.width = width;
    this.height = height;
    this.labels = labels;
    low = new int[width * height];
    count = new int[width * height];
  }

  static DisparityBands full(int width, int height, int labels, boolean[] valid) {
    DisparityBands bands = new DisparityBands(width, height, labels);
    for (int p = 0; p < bands.count.length; p++) if (valid[p]) bands.count[p] = labels;
    return bands;
  }

  /** Packed start of each pixel's labels; throws if the packed volume exceeds int indexing. */
  int[] offsets() {
    if (offset == null) {
      offset = new int[count.length];
      long at = 0;
      for (int p = 0; p < count.length; p++) {
        offset[p] = (int) at;
        at += count[p];
        if (at > Integer.MAX_VALUE - 8)
          throw new IllegalArgumentException("Packed disparity bands exceed 2^31 cells");
      }
      total = at;
    }
    return offset;
  }

  long total() {
    offsets();
    return total;
  }

  /** Packed cell count without building int offsets (never throws for large volumes). */
  long cells() {
    long sum = 0;
    for (int c : count) sum += c;
    return sum;
  }

  /** Cells per row, for planning strips of oversized volumes. */
  long[] rowCells() {
    long[] rows = new long[height];
    for (int y = 0; y < height; y++)
      for (int x = 0, p = y * width; x < width; x++, p++) rows[y] += count[p];
    return rows;
  }

  /** Rows [y0, y1) as an independent band set. */
  DisparityBands rows(int y0, int y1) {
    DisparityBands out = new DisparityBands(width, y1 - y0, labels);
    System.arraycopy(low, y0 * width, out.low, 0, (y1 - y0) * width);
    System.arraycopy(count, y0 * width, out.count, 0, (y1 - y0) * width);
    return out;
  }

  int maxCount() {
    int max = 0;
    for (int c : count) max = Math.max(max, c);
    return max;
  }

  void changed() {
    offset = null;
    total = -1;
  }

  /** Includes label k for pixel p, extending the contiguous band. */
  void include(int p, int k) {
    k = Math.clamp(k, 0, labels - 1);
    if (count[p] == 0) {
      low[p] = k;
      count[p] = 1;
      return;
    }
    int lo = Math.min(low[p], k), hi = Math.max(low[p] + count[p] - 1, k);
    low[p] = lo;
    count[p] = hi - lo + 1;
  }

  /**
   * Right-image bands as the union of left disparities that land on each right pixel. A left winner
   * d at x is therefore always inside the band of right pixel x-d, so LR checks are not limited by
   * the right search. Labels are the same disparity indices; minimum disparity offsets cancel.
   */
  DisparityBands warpRight(int minDisparity, boolean[] validRight) {
    DisparityBands right = new DisparityBands(width, height, labels);
    int[] lo = new int[count.length], hi = new int[count.length];
    Arrays.fill(lo, Integer.MAX_VALUE);
    Arrays.fill(hi, Integer.MIN_VALUE);
    // Each left row only touches the same right row, so rows run independently.
    StereoRows.rows(0, height, y -> {
      for (int x = 0; x < width; x++) {
        int p = y * width + x, n = count[p];
        if (n == 0) continue;
        // Only the band's end labels can set a right pixel's min/max; interior labels fill gaps.
        for (int k = low[p]; k < low[p] + n; k++) {
          long xr = (long) x - minDisparity - k;
          if (xr < 0 || xr >= width) continue;
          int q = y * width + (int) xr;
          if (k < lo[q]) lo[q] = k;
          if (k > hi[q]) hi[q] = k;
        }
      }
    });
    for (int q = 0; q < count.length; q++)
      if (validRight[q] && hi[q] >= lo[q]) {
        right.low[q] = lo[q];
        right.count[q] = hi[q] - lo[q] + 1;
      }
    return right;
  }

  /**
   * Restricts bands to labels whose target column lies inside the other image's matchable interior
   * [r, width-r). Such labels always cost the invalid sentinel, so this only drops unmatchable
   * labels; pixels with no matchable label lose their band. Returns the number of emptied pixels.
   */
  long clampToImage(int minDisparity, boolean reverse, int r) {
    long emptied = 0;
    for (int y = 0; y < height; y++)
      for (int x = 0; x < width; x++) {
        int p = y * width + x;
        if (count[p] == 0) continue;
        // Left: target = x - md - k; right: target = x + md + k; keep r <= target < width - r.
        long lowest = reverse ? (long) r - x - minDisparity : (long) x - minDisparity - (width - r - 1);
        long highest = reverse ? (long) width - r - 1 - x - minDisparity : (long) x - minDisparity - r;
        long lo = Math.max(low[p], lowest), hi = Math.min(low[p] + count[p] - 1L, highest);
        if (hi < lo) {
          count[p] = 0;
          low[p] = 0;
          emptied++;
        } else {
          low[p] = (int) lo;
          count[p] = (int) (hi - lo + 1);
        }
      }
    changed();
    return emptied;
  }

  /** Label-count histogram (bins: 0, 1-2, 3-4, 5-8, ... powers of two, full). */
  Object histogram() {
    int bins = 1;
    while ((1 << (bins - 1)) < labels) bins++;
    long[] h = new long[bins + 1];
    long pixels = 0, cells = 0;
    for (int c : count) {
      if (c == 0) {
        h[0]++;
        continue;
      }
      pixels++;
      cells += c;
      if (c == labels) h[bins]++;
      else h[Math.min(bins - 1, 1 + (31 - Integer.numberOfLeadingZeros(Math.max(1, c - 1))))]++;
    }
    var out = new java.util.LinkedHashMap<String, Object>();
    out.put("labels_in_range", labels);
    out.put("pixels_with_labels", pixels);
    out.put("packed_cells", cells);
    out.put("mean_labels_per_pixel", pixels == 0 ? 0 : cells / (double) pixels);
    var bins2 = new java.util.LinkedHashMap<String, Long>();
    bins2.put("0", h[0]);
    for (int b = 1; b < bins; b++)
      bins2.put(b == 1 ? "1-2" : ((1 << (b - 1)) + 1) + "-" + (1 << b), h[b]);
    bins2.put("full_range", h[bins]);
    out.put("histogram", bins2);
    return out;
  }
}
