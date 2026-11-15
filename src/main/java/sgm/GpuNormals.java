package sgm;

import static java.lang.foreign.ValueLayout.*;

import java.lang.foreign.*;
import java.util.List;
import java.util.function.IntFunction;

/**
 * Surface normals and neighbourhood attributes of image fusion ({@code ImageFusion.surfaceSample})
 * on the GPU, opt-in with --fusion-gpu-normals on. One work item per emitted point: up to three
 * 7 x 7 neighbourhoods of the merged depth map are back-projected, a plane is fitted by the same
 * 3 x 3 Jacobi eigen-decomposition, trimmed by the median residual and refitted, and the same
 * acceptance tests decide; the attributes follow SurfaceAttributes.of.
 *
 * <p>Not bit-identical to the CPU: the Jacobi rotation uses sqrt(1 + tau * tau) where the Java
 * code uses Math.hypot(1, tau), and the rare fallback fit uses the device's atan2/cos/sin. Normals
 * and attributes therefore differ in the last bits, and a point whose test value lies on a
 * threshold can, rarely, keep or lose its normal. Everything else (+ - * / sqrt, comparisons,
 * selection, no FMA contraction) follows the Java code operation by operation.
 *
 * <p>The first batch of the process is compared with the CPU code on every 16th point; more than
 * 0.1% different outcomes or a normal more than 1e-6 apart switches the GPU path off (the CPU code
 * then runs, as it does with --compute cpu, without an FP64 device, or when the band does not fit
 * the GPU budget).
 */
final class GpuNormals {
  private GpuNormals() {}

  /** Points per kernel launch (display drivers stop long kernels). */
  private static final int LAUNCH = 1 << 18;
  /** Rows above and below a band that the neighbourhoods reach (3 x stride 4). */
  static final int HALO = 12;
  private static final int OUT_D = 9, OUT_I = 4;

  private static volatile boolean disabled, verified;
  private static GpuSgm.Compute cl;
  private static MemorySegment kernel;
  static long points, batches, cpuBatches, differentOutcomes, checked;
  static double seconds, largestNormalDifference;

  /** Tests: treat the CPU self-check as done (their reference is not the CPU code). */
  static void skipSelfCheck() {
    verified = true;
  }

  static boolean requested(Options o) {
    return o.fusionGpuNormals && !o.compute.equals("cpu") && !disabled;
  }

  private static boolean noted;

  /** One line the first time a batch goes to the CPU for memory. */
  private static void note(String why) {
    if (noted) return;
    noted = true;
    System.out.println("GPU normals: " + why + "; this batch uses the CPU");
  }

  private static void failed(String message) {
    if (!disabled) {
      disabled = true;
      System.out.println("WARNING: GPU normals unavailable, using the CPU: " + message);
    }
    close();
  }

  /** Device bytes for rows uploaded rows of width w and count points. */
  static long bytes(long w, long rows, long count) {
    return 12 * w * rows + count * (8L + 24 + 8L * OUT_D + 4L * OUT_I) + 4096;
  }

  /**
   * Surface samples of the points (x[i], y[i] local rows of the merged map, p = 3 coordinates per
   * point), or null when the GPU path does not apply (use the CPU). The map rows [from, to) cover
   * every neighbourhood. cpu gives the CPU result of point i for the self-check.
   */
  static synchronized DepthFusion.SurfaceSample[] compute(
      Options o, Camera camera, float[] z, float[] du, float[] dv, int w, int h, int row0,
      int from, int to, int[] x, int[] y, double[] p, IntFunction<DepthFusion.SurfaceSample> cpu) {
    int n = x.length;
    if (n == 0 || !requested(o)) return null;
    long clock = System.nanoTime();
    GpuMemoryPool.Reservation reservation = null;
    try {
      List<GpuSgm.Device> devices = GpuSgm.devices();
      if (o.gpuDevice >= devices.size()) { note("no OpenCL device " + o.gpuDevice); return null; }
      GpuSgm.Device device = devices.get(o.gpuDevice);
      int rows = to - from;
      long need = bytes(w, rows, n);
      if (need > GpuSgm.budget(device, o) || 4L * w * rows > device.allocation()
          || (long) n * 8 * OUT_D > device.allocation()) {
        cpuBatches++;
        note(String.format(java.util.Locale.ROOT, "a band of %,d points needs %,d KiB of device memory, the budget is %,d KiB", n, need >> 10, GpuSgm.budget(device, o) >> 10));
        return null;
      }
      reservation = GpuMemoryPool.reserveAuxiliary(need);
      if (reservation == null) {
        note("the shared GPU memory pool has no room");
        cpuBatches++;
        return null;
      }
      if (cl == null) {
        cl = new GpuSgm.Compute(o, SOURCE, true);
        kernel = cl.kernel("normals");
      }
      double[] out = new double[n * OUT_D];
      int[] outI = new int[n * OUT_I];
      try (Arena arena = Arena.ofConfined()) {
        double[] rt = Math3.transpose(camera.rotation), centre = camera.center();
        double[] params = new double[22];
        params[0] = camera.fx; params[1] = camera.fy; params[2] = camera.cx; params[3] = camera.cy;
        System.arraycopy(rt, 0, params, 4, 9);
        System.arraycopy(camera.translation, 0, params, 13, 3);
        System.arraycopy(centre, 0, params, 16, 3);
        params[19] = o.fusionRelative;
        params[20] = Math.min(camera.fx, camera.fy);
        MemorySegment bZ = cl.buffer(4L * w * rows), bU = cl.buffer(4L * w * rows), bV = cl.buffer(4L * w * rows),
            bX = cl.buffer(4L * n), bY = cl.buffer(4L * n), bP = cl.buffer(24L * n), bParams = cl.buffer(8L * params.length),
            bOut = cl.buffer(8L * OUT_D * n), bOutI = cl.buffer(4L * OUT_I * n);
        long cells = (long) w * rows;
        cl.transfer(bZ, 0, host(arena, MemorySegment.ofArray(z).asSlice(4L * from * w, 4 * cells)), false);
        cl.transfer(bU, 0, host(arena, MemorySegment.ofArray(du).asSlice(4L * from * w, 4 * cells)), false);
        cl.transfer(bV, 0, host(arena, MemorySegment.ofArray(dv).asSlice(4L * from * w, 4 * cells)), false);
        cl.transfer(bX, 0, host(arena, MemorySegment.ofArray(x)), false);
        cl.transfer(bY, 0, host(arena, MemorySegment.ofArray(y)), false);
        cl.transfer(bP, 0, host(arena, MemorySegment.ofArray(p)), false);
        cl.transfer(bParams, 0, host(arena, MemorySegment.ofArray(params)), false);
        for (int first = 0; first < n; first += LAUNCH) {
          cl.run(kernel, Math.min(LAUNCH, n - first), bZ, bU, bV, w, h, row0, from, to, bX, bY, bP, bParams, bOut, bOutI, first, n);
          cl.finish();
        }
        MemorySegment hostOut = arena.allocate(8L * OUT_D * n, 8), hostOutI = arena.allocate(4L * OUT_I * n, 4);
        cl.transfer(bOut, 0, hostOut, true);
        MemorySegment.copy(hostOut, JAVA_DOUBLE, 0, out, 0, out.length);
        cl.transfer(bOutI, 0, hostOutI, true);
        MemorySegment.copy(hostOutI, JAVA_INT, 0, outI, 0, outI.length);
      } finally {
        cl.releaseBuffers();
      }
      DepthFusion.SurfaceSample[] result = new DepthFusion.SurfaceSample[n];
      for (int i = 0; i < n; i++) result[i] = sample(out, outI, i);
      if (!verified && !check(result, cpu)) return null;
      points += n;
      batches++;
      seconds += (System.nanoTime() - clock) / 1e9;
      return result;
    } catch (RuntimeException | Error e) {
      failed(e.toString());
      return null;
    } finally {
      if (reservation != null) reservation.close();
    }
  }

  /** A native copy of a heap segment (OpenCL transfers need native memory). */
  private static MemorySegment host(Arena arena, MemorySegment heap) {
    MemorySegment copy = arena.allocate(heap.byteSize(), 8);
    copy.copyFrom(heap);
    return copy;
  }

  private static DepthFusion.SurfaceSample sample(double[] d, int[] k, int i) {
    int e = i * OUT_D, f = i * OUT_I;
    double footprint = d[e + 3], sigma = d[e + 4];
    if (k[f] == 0) return new DepthFusion.SurfaceSample(null, footprint, sigma);
    double[] normal = {d[e], d[e + 1], d[e + 2]};
    SurfaceAttributes a =
        k[f + 2] == 2
            ? SurfaceAttributes.missing(true)
            : new SurfaceAttributes(d[e + 5], d[e + 6], d[e + 7], d[e + 8], k[f + 1], 0, k[f + 3] != 0);
    return new DepthFusion.SurfaceSample(normal, footprint, sigma, a);
  }

  /** First batch: every 16th point against the CPU code. */
  private static boolean check(DepthFusion.SurfaceSample[] gpu, IntFunction<DepthFusion.SurfaceSample> cpu) {
    long compared = 0, different = 0;
    double worst = 0;
    for (int i = 0; i < gpu.length; i += 16) {
      DepthFusion.SurfaceSample a = gpu[i], b = cpu.apply(i);
      compared++;
      boolean hasA = a.normal() != null, hasB = b.normal() != null;
      if (hasA != hasB || a.attributes().status() != b.attributes().status()) {
        different++;
        continue;
      }
      if (hasA)
        for (int k = 0; k < 3; k++) worst = Math.max(worst, Math.abs(a.normal()[k] - b.normal()[k]));
    }
    checked = compared;
    differentOutcomes = different;
    largestNormalDifference = worst;
    if (compared == 0) return true;
    if (different > Math.max(0, compared / 1000) || worst > 1e-6) {
      failed(different + " of " + compared + " sampled points decided differently, largest normal difference " + worst);
      return false;
    }
    verified = true;
    System.out.printf(
        java.util.Locale.ROOT,
        "GPU normals: %,d sampled points checked against the CPU code; %d decided differently, largest normal difference %.1e%n",
        compared, different, worst);
    return true;
  }

  static synchronized void close() {
    GpuSgm.Compute open = cl;
    cl = null;
    kernel = null;
    if (open != null)
      try {
        open.close();
      } catch (RuntimeException ignored) {
      }
  }

  static final String SOURCE =
      """
#pragma OPENCL EXTENSION cl_khr_fp64 : enable
#pragma OPENCL FP_CONTRACT OFF

#define MAXN 49

// Java Math.max / Math.min for values that are not NaN.
inline double jmax(double a, double b) { return a >= b ? a : b; }
inline double jmin(double a, double b) { return a <= b ? a : b; }

// SurfaceGeometry.plane / referencePlane on points[idx[0..n)]. Returns 0 for null.
// out: normal[3], centre[3], smallest, middle, largest.
int plane(double pts[MAXN][3], int idx[MAXN], int n, double* out) {
  if (n < 6) return 0;
  double origin[3], mean[3] = {0, 0, 0};
  for (int k = 0; k < 3; k++) origin[k] = pts[idx[0]][k];
  for (int i = 0; i < n; i++) for (int k = 0; k < 3; k++) mean[k] += pts[idx[i]][k] - origin[k];
  for (int k = 0; k < 3; k++) mean[k] /= n;
  for (int fallback = 0; fallback < 2; fallback++) {
    double a[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, v[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    if (fallback == 0) {
      for (int i = 0; i < n; i++) {
        double x = pts[idx[i]][0] - origin[0] - mean[0],
               y = pts[idx[i]][1] - origin[1] - mean[1],
               z = pts[idx[i]][2] - origin[2] - mean[2];
        a[0][0] += x * x / n; a[0][1] += x * y / n; a[0][2] += x * z / n;
        a[1][1] += y * y / n; a[1][2] += y * z / n; a[2][2] += z * z / n;
      }
      a[1][0] = a[0][1]; a[2][0] = a[0][2]; a[2][1] = a[1][2];
    } else {
      for (int i = 0; i < n; i++) {
        double d[3];
        for (int k = 0; k < 3; k++) d[k] = pts[idx[i]][k] - origin[k] - mean[k];
        for (int k = 0; k < 3; k++) for (int j = 0; j < 3; j++) a[k][j] += d[k] * d[j] / n;
      }
    }
    for (int iteration = 0; iteration < 24; iteration++) {
      int p = 0, q = 1;
      if (fabs(a[0][2]) > fabs(a[p][q])) { p = 0; q = 2; }
      if (fabs(a[1][2]) > fabs(a[p][q])) { p = 1; q = 2; }
      if (fabs(a[p][q]) < 1e-15 * jmax(1e-20, a[0][0] + a[1][1] + a[2][2])) break;
      double c, s;
      if (fallback == 0) {
        double tau = (a[q][q] - a[p][p]) / (2 * a[p][q]);
        // Java: Math.hypot(1, tau).
        double t = copysign(1.0, tau) / (fabs(tau) + sqrt(1 + tau * tau));
        c = 1 / sqrt(1 + t * t);
        s = t * c;
      } else {
        double angle = .5 * atan2(2 * a[p][q], a[q][q] - a[p][p]);
        c = cos(angle);
        s = sin(angle);
      }
      double ap = a[p][p], aq = a[q][q], cross = a[p][q];
      for (int k = 0; k < 3; k++)
        if (k != p && k != q) {
          double x = a[k][p], y = a[k][q];
          a[k][p] = a[p][k] = c * x - s * y;
          a[k][q] = a[q][k] = s * x + c * y;
        }
      a[p][p] = c * c * ap - 2 * s * c * cross + s * s * aq;
      a[q][q] = s * s * ap + 2 * s * c * cross + c * c * aq;
      a[p][q] = a[q][p] = 0;
      for (int k = 0; k < 3; k++) {
        double x = v[k][p], y = v[k][q];
        v[k][p] = c * x - s * y;
        v[k][q] = s * x + c * y;
      }
    }
    int low = 0, high = 0;
    for (int k = 1; k < 3; k++) {
      if (a[k][k] < a[low][low]) low = k;
      if (a[k][k] > a[high][high]) high = k;
    }
    if (low == high) return 0;
    int mid = 3 - low - high;
    if (fallback == 0) {
      double trace = jmax(1e-20, a[0][0] + a[1][1] + a[2][2]);
      if (!isfinite(trace)
          || jmax(fabs(a[0][1]), jmax(fabs(a[0][2]), fabs(a[1][2]))) > 1e-13 * trace
          || jmin(a[mid][mid] - a[low][low], a[high][high] - a[mid][mid]) < 1e-10 * trace)
        continue;
    }
    out[0] = v[0][low]; out[1] = v[1][low]; out[2] = v[2][low];
    for (int k = 0; k < 3; k++) out[3 + k] = origin[k] + mean[k];
    out[6] = jmax(0, a[low][low]); out[7] = jmax(0, a[mid][mid]); out[8] = jmax(0, a[high][high]);
    return 1;
  }
  return 0;
}

inline double distance(double* p, double* c, double* n) {
  return (p[0] - c[0]) * n[0] + (p[1] - c[1]) * n[1] + (p[2] - c[2]) * n[2];
}

// ImageFusion.select: the k-th smallest of a[0, n).
double kth(double* a, int n, int k) {
  int lo = 0, hi = n - 1;
  while (hi > lo) {
    double pivot = a[(lo + hi) >> 1];
    int i = lo, j = hi;
    while (i <= j) {
      while (a[i] < pivot) i++;
      while (a[j] > pivot) j--;
      if (i <= j) { double t = a[i]; a[i++] = a[j]; a[j--] = t; }
    }
    if (k <= j) hi = j; else if (k >= i) lo = i; else break;
  }
  return a[k];
}

// out per point: normal[3], footprint, sigma, roughness, variation, planarity, radius;
// outI: has normal, neighbours, attribute status (0 valid, 2 normal only), trimmed.
__kernel void normals(__global const float* zs, __global const float* dus, __global const float* dvs,
                      int w, int h, int row0, int from, int to,
                      __global const int* xs, __global const int* ys, __global const double* ps,
                      __global const double* params, __global double* out, __global int* outI,
                      int first, int count) {
  int i = first + (int)get_global_id(0);
  if (i >= count) return;
  double fx = params[0], fy = params[1], cx = params[2], cy = params[3];
  double rt[9], tr[3], centre[3];
  for (int k = 0; k < 9; k++) rt[k] = params[4 + k];
  for (int k = 0; k < 3; k++) { tr[k] = params[13 + k]; centre[k] = params[16 + k]; }
  double relative = params[19], focal = params[20];
  int x = xs[i], y = ys[i];
  double p[3] = {ps[3 * i], ps[3 * i + 1], ps[3 * i + 2]};
  double mz = zs[(y - from) * w + x];
  double footprint = mz / focal, sigma = footprint * .1;
  __global double* o = out + 9 * i;
  __global int* oi = outI + 4 * i;
  o[3] = footprint; o[4] = sigma; oi[0] = 0;
  double pts[MAXN][3], residuals[MAXN];
  int idx[MAXN];
  double fit[9];
  int strides[3] = {1, 2, 4};
  for (int si = 0; si < 3; si++) {
    int stride = strides[si], n = 0;
    for (int dy = -3; dy <= 3; dy++)
      for (int dx = -3; dx <= 3; dx++) {
        int xx = x + dx * stride, yy = y + dy * stride;
        if (xx < 0 || yy < 0 || xx >= w || yy >= h) continue;
        int at = (yy - from) * w + xx;
        double z = zs[at];
        if (z <= 0 || fabs(z - mz) > relative * jmin(z, mz)) continue;
        // ImageFusion.backProject with u = x + du, v = (y + row0) + dv in float, as in Java.
        float uf = (float)xx + dus[at], vf = (float)(yy + row0) + dvs[at];
        double u = uf, v = vf;
        double l0 = (u - cx) / fx * z - tr[0], l1 = (v - cy) / fy * z - tr[1], l2 = z - tr[2];
        pts[n][0] = rt[0] * l0 + rt[1] * l1 + rt[2] * l2;
        pts[n][1] = rt[3] * l0 + rt[4] * l1 + rt[5] * l2;
        pts[n][2] = rt[6] * l0 + rt[7] * l1 + rt[8] * l2;
        idx[n] = n;
        n++;
      }
    if (!plane(pts, idx, n, fit) || n < 9 || fit[7] < footprint * footprint * .2 || fit[6] > .12 * fit[7]) continue;
    double normal[3] = {fit[0], fit[1], fit[2]}, centre1[3] = {fit[3], fit[4], fit[5]};
    for (int k = 0; k < n; k++) residuals[k] = fabs(distance(pts[idx[k]], centre1, normal));
    double band = jmax(footprint * .5, 3 * 1.4826 * kth(residuals, n, n / 2));
    int kept = 0;
    for (int k = 0; k < n; k++)
      if (fabs(distance(pts[idx[k]], centre1, normal)) <= band) idx[kept++] = idx[k];
    if (kept < n * .8) return;
    if (!plane(pts, idx, kept, fit) || fit[6] > .12 * fit[7]) continue;
    normal[0] = fit[0]; normal[1] = fit[1]; normal[2] = fit[2];
    double centre2[3] = {fit[3], fit[4], fit[5]};
    double centerError = fabs(distance(p, centre2, normal));
    if (centerError > jmax(.75 * footprint, 2 * sqrt(fit[6]))) return;
    double ray[3] = {p[0] - centre[0], p[1] - centre[1], p[2] - centre[2]};
    double dotr = normal[0] * ray[0] + normal[1] * ray[1] + normal[2] * ray[2];
    double cosine = fabs(dotr) / jmax(1e-20, sqrt(ray[0] * ray[0] + ray[1] * ray[1] + ray[2] * ray[2]));
    if (cosine < .15) return;
    if (dotr > 0) { normal[0] = -normal[0]; normal[1] = -normal[1]; normal[2] = -normal[2]; }
    o[0] = normal[0]; o[1] = normal[1]; o[2] = normal[2];
    o[4] = jmax(.1 * footprint, sigma * cosine);
    oi[0] = 1;
    // SurfaceAttributes.of(plane, kept neighbours, p, kept < n).
    double sum = fit[6] + fit[7] + fit[8], radius = 0;
    for (int k = 0; k < kept; k++) {
      double ax = pts[idx[k]][0] - p[0], ay = pts[idx[k]][1] - p[1], az = pts[idx[k]][2] - p[2];
      radius = jmax(radius, ax * ax + ay * ay + az * az);
    }
    radius = sqrt(radius);
    if (!(sum > 0 && fit[8] > 0)) { oi[2] = 2; oi[1] = 0; oi[3] = 0; return; }
    o[5] = sqrt(fit[6]); o[6] = fit[6] / sum; o[7] = (fit[7] - fit[6]) / fit[8]; o[8] = radius;
    oi[1] = kept; oi[2] = 0; oi[3] = kept < n ? 1 : 0;
    return;
  }
}
""";
}
