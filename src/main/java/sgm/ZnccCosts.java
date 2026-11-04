package sgm;

/** Exact integer-disparity box statistics with O(width*window) workspace. */
final class ZnccCosts {
  static long workspace(int width, int window) {
    return (long) width * 5 * (window + 1) * 8;
  }

  static void fill(
      PagedVolume costs,
      byte[] a,
      byte[] b,
      boolean[] validA,
      boolean[] validB,
      int w,
      int h,
      int minimum,
      int disparities,
      int window,
      boolean reverse) {
    int r = window / 2, n = window * window, bits = n - 1;
    for (int k = 0; k < disparities; k++) {
      int disparity = minimum + k;
      long shifted = reverse ? (long) disparity : -(long) disparity;
      long low = Math.max(r, r - shifted), high = Math.min(w - r, w - r - shifted);
      if (low >= high) continue;
      int lo = (int) low, hi = (int) high, shift = (int) shifted;
      long[][] ring = new long[window][w * 5];
      long[] vertical = new long[w * 5];
      for (int y = 0; y < h; y++) {
        long sa = 0, sb = 0, saa = 0, sbb = 0, sab = 0;
        for (int dx = -r; dx <= r; dx++) {
          int av = a[y * w + lo + dx] & 255, bv = b[y * w + lo + dx + shift] & 255;
          sa += av;
          sb += bv;
          saa += av * av;
          sbb += bv * bv;
          sab += av * bv;
        }
        long[] old = ring[y % window];
        for (int x = lo; x < hi; x++) {
          int at = x * 5;
          vertical[at] += sa - old[at];
          vertical[at + 1] += sb - old[at + 1];
          vertical[at + 2] += saa - old[at + 2];
          vertical[at + 3] += sbb - old[at + 3];
          vertical[at + 4] += sab - old[at + 4];
          old[at] = sa;
          old[at + 1] = sb;
          old[at + 2] = saa;
          old[at + 3] = sbb;
          old[at + 4] = sab;
          if (y >= window - 1) {
            int p = (y - r) * w + x, q = p + shift;
            if (validA[p] && validB[q]) {
              long aa = vertical[at],
                  bb = vertical[at + 1],
                  va = n * vertical[at + 2] - aa * aa,
                  vb = n * vertical[at + 3] - bb * bb;
              int cost = 512;
              if (va > 0 && vb > 0) {
                double correlation = (n * vertical[at + 4] - aa * bb) / Math.sqrt((double) va * vb);
                cost = (int) Math.round((1 - Math.max(-1, Math.min(1, correlation))) * .5 * bits);
              }
              costs.set((long) p * disparities + k, cost);
            }
          }
          if (x + 1 < hi) {
            int ax = a[y * w + x - r] & 255,
                bx = b[y * w + x - r + shift] & 255,
                ay = a[y * w + x + r + 1] & 255,
                by = b[y * w + x + r + 1 + shift] & 255;
            sa += ay - ax;
            sb += by - bx;
            saa += ay * ay - ax * ax;
            sbb += by * by - bx * bx;
            sab += ay * by - ax * bx;
          }
        }
      }
    }
  }
}
