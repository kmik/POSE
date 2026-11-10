package sgm;

import static java.lang.foreign.ValueLayout.*;

import java.lang.foreign.*;
import java.util.stream.IntStream;

/**
 * Edge-aware depth outlier decisions ({@link DepthOutlierFilter}) on the GPU. The kernel repeats
 * DepthOutlierFilter.decision in FP64 with the same operation order and no FMA contraction
 * (division and sqrt are correctly rounded in OpenCL FP64, as in Java); the median of the residuals
 * is the same order statistic. Pixels that reach structure() (Math.hypot/atan2/sin/cos, which are
 * not portable bit for bit) come back with code {@link #CPU} and are decided by the Java code.
 *
 * <p>Returns null whenever the GPU path does not apply (--compute cpu, small rasters, radius above
 * {@link #MAX_RADIUS}, no pool memory without waiting while holding some) or fails; callers then
 * run the CPU code. The first use in a process is compared with the CPU on every eighth row; a
 * mismatch or GPU failure disables the GPU path for the rest of the process. Device memory is
 * bounded by the raster (about 10 bytes per pixel) and is reserved from the shared pair-worker pool.
 */
final class GpuDepthOutlier {
  private GpuDepthOutlier() {}

  static final int MAX_RADIUS = 5;
  /** Code of pixels returned to the CPU (structure check). */
  static final int CPU = 255;

  private static volatile boolean disabled, verified;

  static boolean applies(Options o, int w, int rows) {
    return !disabled
        && !o.compute.equals("cpu")
        && !Boolean.getBoolean("sgm.cpuDepthOutlier")
        && (long) w * rows >= GpuStereoFilters.MIN_PIXELS
        && o.depthOutlierRadius <= MAX_RADIUS;
  }

  private static void failed(String message) {
    if (!disabled) {
      disabled = true;
      System.out.println(
          "WARNING: GPU depth outlier filter unavailable, using the CPU (same results): " + message);
    }
  }

  /** Codes of rows [from, to) as DepthOutlierFilter.codes would give them, or null (use the CPU). */
  static byte[] codes(
      Options o, float[] depth, byte[] gray, int w, int h, int from, int to, float[] lrError) {
    if (!applies(o, w, to - from)) return null;
    byte[] codes;
    long bytes = 5L * w * h + (lrError == null ? 4 : 4L * w * h) + (long) (to - from) * w + 64;
    try (GpuMemoryPool.Reservation reservation = GpuMemoryPool.reserveAuxiliary(bytes)) {
      if (reservation == null) return null;
      codes = run(o, depth, gray, w, h, from, to, lrError);
    } catch (RuntimeException | Error e) {
      failed(e.toString());
      return null;
    }
    long returned = resolve(o, depth, gray, w, h, from, to, lrError, codes);
    if (!verified && !check(o, depth, gray, w, h, from, to, lrError, codes, returned)) return null;
    return codes;
  }

  /** Decides the pixels the kernel returned (code CPU) with the Java code; returns their number. */
  private static long resolve(
      Options o, float[] depth, byte[] gray, int w, int h, int from, int to, float[] lrError,
      byte[] codes) {
    return IntStream.range(from, to)
        .parallel()
        .mapToLong(
            y -> {
              long n = 0;
              for (int x = 0, at = (y - from) * w; x < w; x++, at++)
                if ((codes[at] & 255) == CPU) {
                  codes[at] = (byte) DepthOutlierFilter.code(o, depth, gray, w, h, x, y, lrError);
                  n++;
                }
              return n;
            })
        .sum();
  }

  private static synchronized boolean check(
      Options o, float[] depth, byte[] gray, int w, int h, int from, int to, float[] lrError,
      byte[] codes, long returned) {
    if (verified) return true;
    int step = 8;
    byte[] cpu = DepthOutlierFilter.codes(o, depth, gray, w, h, from, to, lrError, step);
    long rows = 0, mismatches = 0, decided = 0;
    for (int y = from; y < to; y += step, rows++)
      for (int x = 0, at = (y - from) * w; x < w; x++, at++) {
        if (cpu[at] != codes[at]) mismatches++;
        if (cpu[at] > 1) decided++;
      }
    if (mismatches > 0) {
      failed(mismatches + " of " + rows * w + " sampled decisions differ from the CPU");
      return false;
    }
    verified = true;
    System.out.printf(
        java.util.Locale.ROOT,
        "GPU depth outlier filter: %d sampled rows (%d decisions) identical to the CPU code;"
            + " %d of %d pixels returned to the CPU for the structure check%n",
        rows, decided, returned, (long) (to - from) * w);
    return true;
  }

  private static byte[] run(
      Options o, float[] depth, byte[] gray, int w, int h, int from, int to, float[] lrError) {
    int rows = to - from;
    long n = (long) w * h, count = (long) rows * w;
    int r = o.depthOutlierRadius;
    try (GpuSgm.Compute cl = new GpuSgm.Compute(o, "#define MAXN " + ((2 * r + 1) * (2 * r + 1) - 1) + "\n" + SOURCE);
        Arena arena = Arena.ofConfined()) {
      MemorySegment bD = cl.buffer(4 * n), bG = cl.buffer(n), bC = cl.buffer(count);
      MemorySegment bL = cl.buffer(lrError == null ? 4 : 4 * n), params = cl.buffer(16);
      cl.transfer(bD, 0, arena.allocateFrom(JAVA_FLOAT, depth), false);
      cl.transfer(bG, 0, arena.allocateFrom(JAVA_BYTE, gray), false);
      if (lrError != null) cl.transfer(bL, 0, arena.allocateFrom(JAVA_FLOAT, lrError), false);
      // Same values as DepthOutlierFilter.decision computes them.
      double tolerance = Math.max(.25, o.depthOutlierDeviation * .35);
      cl.transfer(params, 0, arena.allocateFrom(JAVA_DOUBLE, o.depthOutlierDeviation, tolerance), false);
      cl.run(
          cl.kernel("outlier"), count, bD, bG, bL, bC, params, w, h, from, rows,
          o.depthOutlierRadius, o.depthOutlierEdgeThreshold, lrError == null ? 0 : 1);
      MemorySegment host = arena.allocate(count);
      cl.transfer(bC, 0, host, true);
      return host.toArray(JAVA_BYTE);
    }
  }

  static final String SOURCE =
      """
#pragma OPENCL EXTENSION cl_khr_fp64 : enable
#pragma OPENCL FP_CONTRACT OFF

#define CPU_CODE 255

// fabs(z - a*x - b*y - c) <= tolerance exactly as Java decides it in double. The float estimate
// differs from the exact residual (and from Java's double evaluation) by far less than 32 float
// ulps of the magnitudes involved (a, b, c, tolerance rounded to float; four rounded operations);
// only tests within that margin of the tolerance evaluate the double expression. NaN/overflow fall
// through to the double expression as well.
inline int within(float z, float x, float y, float af, float bf, float cf, double a, double b, double c,
                  float tf, double tolerance) {
  float e = fabs(((z - af * x) - bf * y) - cf);
  float margin = (fabs(z) + fabs(af * x) + fabs(bf * y) + fabs(cf) + tf) * 0x1p-19f + 0x1p-100f;
  if (e < tf - margin) return 1;
  if (e > tf + margin) return 0;
  return fabs((double)z - a * (double)x - b * (double)y - c) <= tolerance;
}

int sector(int x, int y) {
  if (y < 0) return x < 0 ? 1 : x == 0 ? 2 : 4;
  if (y == 0) return x < 0 ? 8 : 16;
  return x < 0 ? 32 : x == 0 ? 64 : 128;
}

// Java Math.max(double, double) (NaN if either is NaN).
inline double jmax(double a, double b) {
  if (isnan(a) || isnan(b)) return a + b;
  if (a == b) return signbit(a) ? b : a;
  return a > b ? a : b;
}
inline double jmin(double a, double b) {
  if (isnan(a) || isnan(b)) return a + b;
  if (a == b) return signbit(a) ? a : b;
  return a < b ? a : b;
}

// DepthOutlierFilter.decision; returns CPU_CODE where the Java code calls structure().
int decision(__global const float* d, __global const uchar* gray, int w, int x, int y, int r,
             int edge, double deviation, double tolerance, double lr) {
  int p = y * w + x, g = gray[p], sectors = 0;
  double center = (double)d[p];
  float sz[MAXN], sx[MAXN], sy[MAXN];
  float tf = (float)tolerance;
  int n = 0;
  for (int dy = -r; dy <= r; dy++)
    for (int dx = -r; dx <= r; dx++) {
      if (dx == 0 && dy == 0) continue;
      int q = p + dy * w + dx;
      float v = d[q];
      if (!isfinite(v) || abs((int)gray[q] - g) > edge) continue;
      sx[n] = (float)dx; sy[n] = (float)dy; sz[n] = v; n++;
      sectors |= sector(dx, dy);
    }
  if (n < 12 || popcount(sectors) < 6) return 2;
  int required = (int)ceil((double)n * .65), best = 0;
  double bestA = 0, bestB = 0, bestC = 0;
  for (int trial = 0; trial < 48; trial++) {
    int i = (trial * 7) % n, j = (trial * 13 + n / 3) % n, k = (trial * 19 + 2 * n / 3) % n;
    double xi = (double)sx[i], yi = (double)sy[i], zi = (double)sz[i];
    double ux = (double)sx[j] - xi, uy = (double)sy[j] - yi, vx = (double)sx[k] - xi, vy = (double)sy[k] - yi;
    double determinant = ux * vy - uy * vx;
    if (fabs(determinant) < 2) continue;
    double uz = (double)sz[j] - zi, vz = (double)sz[k] - zi;
    double a = (uz * vy - uy * vz) / determinant, b = (ux * vz - uz * vx) / determinant;
    double c = zi - a * xi - b * yi;
    float af = (float)a, bf = (float)b, cf = (float)c;
    int support = 0;
    for (int t = 0; t < n;) {
      int end = min(n, t + 8);
      for (; t < end; t++)
        support += within(sz[t], sx[t], sy[t], af, bf, cf, a, b, c, tf, tolerance);
      if (support + (n - t) <= best) break;
    }
    if (support > best) { best = support; bestA = a; bestB = b; bestC = c; }
    if (best == n) break;
  }
  if (best < required) return 3;
  float bestAf = (float)bestA, bestBf = (float)bestB, bestCf = (float)bestC;
  double mx = 0, my = 0, mz = 0;
  for (int i = 0; i < n; i++)
    if (within(sz[i], sx[i], sy[i], bestAf, bestBf, bestCf, bestA, bestB, bestC, tf, tolerance)) {
      mx += (double)sx[i]; my += (double)sy[i]; mz += (double)sz[i];
    }
  mx /= best; my /= best; mz /= best;
  double xx = 0, xy = 0, yy = 0, xz = 0, yz = 0;
  for (int i = 0; i < n; i++)
    if (within(sz[i], sx[i], sy[i], bestAf, bestBf, bestCf, bestA, bestB, bestC, tf, tolerance)) {
      double dx = (double)sx[i] - mx, dy = (double)sy[i] - my, dz = (double)sz[i] - mz;
      xx += dx * dx; xy += dx * dy; yy += dy * dy; xz += dx * dz; yz += dy * dz;
    }
  double det = xx * yy - xy * xy;
  if (det < 1e-6) return 4;
  double a = (xz * yy - yz * xy) / det, b = (yz * xx - xz * xy) / det, c = mz - a * mx - b * my;
  float af = (float)a, bf = (float)b, cf = (float)c;
  int support = 0;
  sectors = 0;
  for (int i = 0; i < n; i++) {
    if (within(sz[i], sx[i], sy[i], af, bf, cf, a, b, c, tf, tolerance)) { support++; sectors |= sector((int)sx[i], (int)sy[i]); }
  }
  if (support < required || popcount(sectors) < 6) return 7;
  double departure = fabs(center - c);
  if (departure <= deviation) return 5;
  // Median of all residuals (Arrays.sort then element n/2): insertion sort, same order statistic.
  double residuals[MAXN];
  for (int i = 0; i < n; i++) {
    double v = fabs((double)sz[i] - a * (double)sx[i] - b * (double)sy[i] - c);
    int at = i;
    while (at > 0 && residuals[at - 1] > v) { residuals[at] = residuals[at - 1]; at--; }
    residuals[at] = v;
  }
  double noise = 1.4826 * residuals[n / 2];
  double leverage = 1.0 / (double)support + (yy * mx * mx - 2 * xy * mx * my + xx * my * my) / det;
  double measurement = isfinite(lr) ? jmax(.25, jmin(2, fabs(lr) * .5)) : .25;
  double band = jmax(deviation, 3 * sqrt(noise * noise * (1 + leverage) + measurement * measurement));
  if (departure <= band) return departure > deviation ? 10 : 5;
  return CPU_CODE;
}

__kernel void outlier(__global const float* d, __global const uchar* gray, __global const float* lrError,
                      __global uchar* codes, __global const double* params, int w, int h, int from, int rows,
                      int r, int edge, int hasLr) {
  int i = get_global_id(0);
  if (i >= w * rows) return;
  int y = from + i / w, x = i % w, p = y * w + x;
  if (!isfinite(d[p])) { codes[i] = 0; return; }
  if (x < r || x >= w - r || y < r || y >= h - r) { codes[i] = 1; return; }
  double lr = hasLr ? (double)lrError[p] : (double)NAN;
  codes[i] = (uchar)decision(d, gray, w, x, y, r, edge, params[0], params[1], lr);
}
""";
}
