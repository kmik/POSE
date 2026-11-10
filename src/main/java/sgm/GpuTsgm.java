package sgm;

import static java.lang.foreign.ValueLayout.*;

import java.lang.foreign.*;
import java.util.Arrays;

/**
 * TSGM band derivation ({@link TsgmSearch#derive}) on the GPU: windowed minimum/maximum of the
 * parent level's checked disparities for the four fallback radii (1, 4, 16, 64), then each pixel's
 * band. Minimum and maximum over a set of floats are exact whatever the order (signed zeros cannot
 * matter: bands use floor(2*min) and ceil(2*max)), and the band arithmetic is the same float/int
 * code as derive(), so the bands are identical.
 *
 * <p>Returns -1 whenever the GPU path does not apply (--compute cpu, small levels, pool memory not
 * available) or fails; the caller then runs the CPU code. The first use in a process is compared
 * with the CPU derive() on the whole level; a difference or failure disables the GPU path for the
 * process. Device memory is bounded by the level and its parent (about 44 bytes per parent pixel
 * plus 9 per pixel) and reserved from the pair-worker pool.
 */
final class GpuTsgm {
  private GpuTsgm() {}

  /** Smallest level (pixels) derived on the GPU. */
  static final long MIN_PIXELS = 100_000;

  private static final int[] RADII = {1, 4, 16, 64};
  private static volatile boolean disabled, verified;

  static boolean applies(Options o, int w, int h) {
    return !disabled
        && !o.compute.equals("cpu")
        && !Boolean.getBoolean("sgm.cpuTsgmPlan")
        && (long) w * h >= MIN_PIXELS;
  }

  private static void failed(String message) {
    if (!disabled) {
      disabled = true;
      System.out.println("WARNING: GPU TSGM band planning unavailable, using the CPU (same results): " + message);
    }
  }

  /** TsgmSearch.derive with radius 1 on the GPU; returns the full-range fallback count or -1. */
  static long derive(
      Options o, float[] parent, int pw, int ph, DisparityBands bands, boolean[] valid, int lmin,
      boolean reverse) {
    int w = bands.width, h = bands.height;
    if (!applies(o, w, h)) return -1;
    long bytes = 44L * pw * ph + 9L * w * h + 64;
    long full;
    int[] low, count;
    try (GpuMemoryPool.Reservation reservation = GpuMemoryPool.reserveAuxiliary(bytes)) {
      if (reservation == null) return -1;
      int[][] out = new int[2][];
      full = run(o, parent, pw, ph, w, h, bands.labels, valid, lmin, reverse, out);
      low = out[0];
      count = out[1];
    } catch (RuntimeException | Error e) {
      failed(e.toString());
      return -1;
    }
    if (!verified && !check(o, parent, pw, ph, bands, valid, lmin, reverse, low, count, full)) return -1;
    System.arraycopy(low, 0, bands.low, 0, low.length);
    System.arraycopy(count, 0, bands.count, 0, count.length);
    return full;
  }

  private static synchronized boolean check(
      Options o, float[] parent, int pw, int ph, DisparityBands bands, boolean[] valid, int lmin,
      boolean reverse, int[] low, int[] count, long full) {
    if (verified) return true;
    DisparityBands cpu = new DisparityBands(bands.width, bands.height, bands.labels);
    long cpuFull = TsgmSearch.deriveCpu(o, parent, pw, ph, cpu, valid, lmin, 1, reverse);
    if (cpuFull != full || !Arrays.equals(cpu.low, low) || !Arrays.equals(cpu.count, count)) {
      failed("bands differ from the CPU derive (full-range " + full + " vs " + cpuFull + ")");
      return false;
    }
    verified = true;
    System.out.printf(
        java.util.Locale.ROOT,
        "GPU TSGM band planning: %dx%d level identical to the CPU derive (%d full-range pixels)%n",
        bands.width, bands.height, full);
    return true;
  }

  private static long run(
      Options o, float[] parent, int pw, int ph, int w, int h, int labels, boolean[] valid, int lmin,
      boolean reverse, int[][] out) {
    long pn = (long) pw * ph, n = (long) w * h;
    try (GpuSgm.Compute cl = new GpuSgm.Compute(o, SOURCE);
        Arena arena = Arena.ofConfined()) {
      MemorySegment bParent = cl.buffer(4 * pn), rowMin = cl.buffer(4 * pn), rowMax = cl.buffer(4 * pn);
      MemorySegment[] ext = new MemorySegment[8];
      for (int i = 0; i < 8; i++) ext[i] = cl.buffer(4 * pn);
      MemorySegment bValid = cl.buffer(n), bLow = cl.buffer(4 * n), bCount = cl.buffer(4 * n), bFull = cl.buffer(8);
      cl.transfer(bParent, 0, arena.allocateFrom(JAVA_FLOAT, parent), false);
      MemorySegment v = arena.allocate(n);
      for (int p = 0; p < n; p++) if (valid[p]) v.set(JAVA_BYTE, p, (byte) 1);
      cl.transfer(bValid, 0, v, false);
      cl.transfer(bFull, 0, arena.allocate(8), false);
      MemorySegment rows = cl.kernel("rows"), columns = cl.kernel("columns"), derive = cl.kernel("derive");
      for (int k = 0; k < RADII.length; k++) {
        cl.run(rows, pn, bParent, rowMin, rowMax, pw, ph, RADII[k]);
        cl.run(columns, pn, rowMin, rowMax, ext[2 * k], ext[2 * k + 1], pw, ph, RADII[k]);
      }
      cl.run(
          derive, n, ext[0], ext[1], ext[2], ext[3], ext[4], ext[5], ext[6], ext[7], bValid, bLow, bCount,
          bFull, w, h, pw, ph, labels, o.tsgmMargin, lmin, o.census / 2, reverse ? 1 : 0);
      MemorySegment hostLow = arena.allocate(4 * n, 4), hostCount = arena.allocate(4 * n, 4),
          hostFull = arena.allocate(8, 4);
      cl.transfer(bLow, 0, hostLow, true);
      cl.transfer(bCount, 0, hostCount, true);
      cl.transfer(bFull, 0, hostFull, true);
      out[0] = hostLow.toArray(JAVA_INT);
      out[1] = hostCount.toArray(JAVA_INT);
      return Integer.toUnsignedLong(hostFull.get(JAVA_INT, 0));
    }
  }

  static final String SOURCE =
      """
// TsgmSearch.extremes: min/max of the finite values in each clipped (2r+1)^2 window; NaN where none.
__kernel void rows(__global const float* in, __global float* rowMin, __global float* rowMax, int pw, int ph, int r) {
  int p = get_global_id(0);
  if (p >= pw * ph) return;
  int x = p % pw, y = p / pw;
  float mn = INFINITY, mx = -INFINITY;
  for (int j = max(0, x - r); j <= min(pw - 1, x + r); j++) {
    float v = in[y * pw + j];
    if (isfinite(v)) { mn = fmin(mn, v); mx = fmax(mx, v); }
  }
  rowMin[p] = mn; rowMax[p] = mx;
}

__kernel void columns(__global const float* rowMin, __global const float* rowMax, __global float* outMin,
                      __global float* outMax, int pw, int ph, int r) {
  int p = get_global_id(0);
  if (p >= pw * ph) return;
  int x = p % pw, y = p / pw;
  float mn = INFINITY, mx = -INFINITY;
  for (int i = max(0, y - r); i <= min(ph - 1, y + r); i++) {
    float a = rowMin[i * pw + x], b = rowMax[i * pw + x];
    if (isfinite(a)) mn = fmin(mn, a);
    if (isfinite(b)) mx = fmax(mx, b);
  }
  if (!(mn <= mx)) { mn = NAN; mx = NAN; }
  outMin[p] = mn; outMax[p] = mx;
}

// TsgmSearch.derive (radius 1 with 4, 16 and 64 as fallbacks).
__kernel void derive(__global const float* nearMin, __global const float* nearMax,
                     __global const float* widerMin, __global const float* widerMax,
                     __global const float* wideMin, __global const float* wideMax,
                     __global const float* widestMin, __global const float* widestMax,
                     __global const uchar* valid, __global int* low, __global int* count, __global uint* full,
                     int w, int h, int pw, int ph, int D, int k, int lmin, int r, int reverse) {
  int p = get_global_id(0);
  if (p >= w * h) return;
  int x = p % w, y = p / w;
  low[p] = 0; count[p] = 0;
  if (!valid[p]) return;
  if (reverse ? (long)w - r - 1 - x - lmin < 0 || (long)r - x - lmin > D - 1
              : (long)x - lmin - r < 0 || (long)x - lmin - (w - r - 1) > D - 1) return;
  int c = min(ph - 1, y / 2) * pw + min(pw - 1, x / 2);
  float minimum, maximum;
  if (!isnan(nearMin[c])) { minimum = nearMin[c]; maximum = nearMax[c]; }
  else if (!isnan(widerMin[c])) { minimum = widerMin[c]; maximum = widerMax[c]; }
  else if (!isnan(wideMin[c])) { minimum = wideMin[c]; maximum = wideMax[c]; }
  else { minimum = widestMin[c]; maximum = widestMax[c]; }
  if (isnan(minimum)) { low[p] = 0; count[p] = D; atomic_inc(full); return; }
  int margin = (maximum - minimum > 2 ? 2 * k : k) + 1;
  int lo = convert_int_sat(floor(2 * minimum)) - margin - lmin,
      hi = convert_int_sat(ceil(2 * maximum)) + margin - lmin;
  lo = clamp(lo, 0, D - 1);
  hi = clamp(hi, lo, D - 1);
  low[p] = lo; count[p] = hi - lo + 1;
}
""";
}
