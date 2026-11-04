package sgm;

import static java.lang.foreign.ValueLayout.*;

import java.lang.foreign.*;
import java.util.*;

/**
 * Per-pair stereo steps after SGM on the GPU (S2): left/right check, median (classic or
 * edge-preserving), reciprocal/ZNCC check, and the left and right point confidences. The kernels
 * reproduce the Java code bit for bit: FP64 for ZNCC and the scores, no FMA contraction, Java's
 * Math.round(float), Math.max/Math.clamp semantics for NaN and -0.0, and Arrays.sort order for the
 * median. Speckle removal (a sequential flood fill) stays on the CPU. Returns null whenever the GPU
 * path does not apply (--compute cpu, small images, no FP64 device, unsupported windows); callers
 * then run the CPU code. A GPU failure is reported once and the CPU code is used from then on.
 */
final class GpuStereoFilters {
  private GpuStereoFilters() {}

  /** Smallest image handled on the GPU; transfers dominate below it. */
  static final long MIN_PIXELS = 250_000;

  private static volatile boolean disabled;

  static boolean applies(Options o, int w, int h) {
    return !disabled
        && !o.compute.equals("cpu")
        && !Boolean.getBoolean("sgm.cpuStereoFilters")
        && (long) w * h >= MIN_PIXELS
        && o.median <= 7
        && o.correlationWindow / 2 <= 15;
  }

  private static void failed(RuntimeException e) {
    if (!disabled) {
      disabled = true;
      System.out.println(
          "WARNING: GPU stereo filters unavailable, using the CPU (same results): " + e.getMessage());
    }
  }

  static CensusSgm.Filtered filter(
      Options o, float[] dl, float[] dr, byte[] left, byte[] photoLeft, byte[] photoRight, int channels,
      int w, int h, int radius) {
    if (!applies(o, w, h)) return null;
    try {
      return run(o, dl, dr, left, photoLeft, photoRight, channels, w, h, radius);
    } catch (RuntimeException e) {
      failed(e);
      return null;
    }
  }

  private static CensusSgm.Filtered run(
      Options o, float[] dl, float[] dr, byte[] left, byte[] photoLeft, byte[] photoRight, int channels,
      int w, int h, int radius) {
    int n = w * h;
    try (GpuSgm.Compute cl = new GpuSgm.Compute(o, SOURCE);
        Arena arena = Arena.ofConfined()) {
      MemorySegment bL = cl.buffer(4L * n), bR = cl.buffer(4L * n), bC = cl.buffer(4L * n), bM = cl.buffer(4L * n);
      MemorySegment bCorr = cl.buffer(4L * n), bErr = cl.buffer(4L * n), bGray = cl.buffer(n);
      MemorySegment bPL = cl.buffer((long) n * channels), bPR = cl.buffer((long) n * channels);
      MemorySegment counts = cl.buffer(16), params = cl.buffer(32);
      cl.transfer(bL, 0, arena.allocateFrom(JAVA_FLOAT, dl), false);
      cl.transfer(bR, 0, arena.allocateFrom(JAVA_FLOAT, dr), false);
      cl.transfer(bGray, 0, arena.allocateFrom(JAVA_BYTE, left), false);
      cl.transfer(bPL, 0, arena.allocateFrom(JAVA_BYTE, photoLeft), false);
      cl.transfer(bPR, 0, arena.allocateFrom(JAVA_BYTE, photoRight), false);
      cl.transfer(counts, 0, arena.allocate(16), false);
      cl.transfer(
          params,
          0,
          arena.allocateFrom(JAVA_DOUBLE, o.lrThreshold, o.medianMaxChange, o.correlation, 0),
          false);
      cl.run(cl.kernel("lr_check"), n, bL, bR, bC, counts, params, w, h, radius);
      MemorySegment checked = bC;
      if (o.median > 1) {
        cl.run(
            cl.kernel("median"), n, bC, bGray, bM, params, w, h, o.median,
            o.medianMode.equals("classic") ? 0 : 1, o.medianEdgeThreshold);
        checked = bM;
      }
      cl.run(
          cl.kernel("post_check"), n, checked, bR, bPL, bPR, bCorr, bErr, counts, params, w, h, radius,
          channels, o.correlationWindow / 2);
      MemorySegment hostChecked = arena.allocate(4L * n, 4), hostCorr = arena.allocate(4L * n, 4),
          hostErr = arena.allocate(4L * n, 4), hostCounts = arena.allocate(16, 4);
      cl.transfer(checked, 0, hostChecked, true);
      cl.transfer(bCorr, 0, hostCorr, true);
      cl.transfer(bErr, 0, hostErr, true);
      cl.transfer(counts, 0, hostCounts, true);
      int[] c = hostCounts.toArray(JAVA_INT);
      return new CensusSgm.Filtered(
          hostChecked.toArray(JAVA_FLOAT), hostCorr.toArray(JAVA_FLOAT), hostErr.toArray(JAVA_FLOAT),
          Integer.toUnsignedLong(c[0]), Integer.toUnsignedLong(c[1]), Integer.toUnsignedLong(c[2]));
    }
  }

  /** PointConfidence.stereo (aggregate cost margins present) on the GPU; false if not applied. */
  static boolean stereoConfidence(CensusSgm.Result r, byte[] left, byte[] right, Options o) {
    if (!applies(o, r.width, r.height)
        || r.costMargin == null
        || r.costSharpness == null
        || r.correlation == null
        || r.leftRightError == null) return false;
    try {
      int w = r.width, h = r.height, n = w * h;
      try (GpuSgm.Compute cl = new GpuSgm.Compute(o, SOURCE);
          Arena arena = Arena.ofConfined()) {
        MemorySegment bD = upload(cl, arena, r.disparity), bCorr = upload(cl, arena, r.correlation),
            bErr = upload(cl, arena, r.leftRightError), bM = upload(cl, arena, r.costMargin),
            bS = upload(cl, arena, r.costSharpness), bOut = cl.buffer(4L * n);
        MemorySegment bA = cl.buffer(n), bB = cl.buffer(n);
        cl.transfer(bA, 0, arena.allocateFrom(JAVA_BYTE, left), false);
        cl.transfer(bB, 0, arena.allocateFrom(JAVA_BYTE, right), false);
        cl.run(cl.kernel("left_confidence"), n, bD, bCorr, bErr, bM, bS, bA, bB, bOut, w, h);
        MemorySegment out = arena.allocate(4L * n, 4);
        cl.transfer(bOut, 0, out, true);
        r.confidence = out.toArray(JAVA_FLOAT);
        return true;
      }
    } catch (RuntimeException e) {
      failed(e);
      return false;
    }
  }

  /** PointConfidence.reverse on the GPU; false if not applied. */
  static boolean reverseConfidence(
      CensusSgm.Result r, byte[] left, byte[] right, byte[] photoLeft, byte[] photoRight, int channels,
      float[] dr, PointConfidence.Selection selected, byte[] maskLeft, byte[] maskRight, Options o) {
    if (!applies(o, r.width, r.height)) return false;
    try {
      int w = r.width, h = r.height, n = w * h, radius = Math.max(3, o.correlationWindow / 2);
      try (GpuSgm.Compute cl = new GpuSgm.Compute(o, SOURCE);
          Arena arena = Arena.ofConfined()) {
        MemorySegment bD = upload(cl, arena, r.disparity), bR = upload(cl, arena, dr),
            bM = upload(cl, arena, selected.margin), bS = upload(cl, arena, selected.sharpness),
            bOut = cl.buffer(4L * n);
        MemorySegment bA = bytes(cl, arena, left), bB = bytes(cl, arena, right);
        MemorySegment bPL = bytes(cl, arena, photoLeft), bPR = bytes(cl, arena, photoRight);
        MemorySegment bML = bytes(cl, arena, maskLeft == null ? new byte[] {1} : maskLeft),
            bMR = bytes(cl, arena, maskRight == null ? new byte[] {1} : maskRight);
        MemorySegment params = cl.buffer(32);
        cl.transfer(params, 0, arena.allocateFrom(JAVA_DOUBLE, o.lrThreshold, o.correlation, 0, 0), false);
        cl.run(
            cl.kernel("right_confidence"), n, bD, bR, bM, bS, bA, bB, bPL, bPR, bML, bMR, bOut, params, w, h,
            radius, o.correlationWindow / 2, channels, maskLeft == null ? 0 : 1, maskRight == null ? 0 : 1);
        MemorySegment out = arena.allocate(4L * n, 4);
        cl.transfer(bOut, 0, out, true);
        r.rightConfidence = out.toArray(JAVA_FLOAT);
        return true;
      }
    } catch (RuntimeException e) {
      failed(e);
      return false;
    }
  }

  private static MemorySegment upload(GpuSgm.Compute cl, Arena arena, float[] values) {
    MemorySegment b = cl.buffer(4L * values.length);
    cl.transfer(b, 0, arena.allocateFrom(JAVA_FLOAT, values), false);
    return b;
  }

  private static MemorySegment bytes(GpuSgm.Compute cl, Arena arena, byte[] values) {
    MemorySegment b = cl.buffer(Math.max(1, values.length));
    cl.transfer(b, 0, arena.allocateFrom(JAVA_BYTE, values), false);
    return b;
  }

  static final String SOURCE =
      """
#pragma OPENCL EXTENSION cl_khr_fp64 : enable
#pragma OPENCL FP_CONTRACT OFF

// Java Float.NaN (0x7fc00000): stored NaNs get these bits, whatever the GPU arithmetic produced.
#define JNAN as_float(0x7fc00000u)
inline float jf(float v) { return isnan(v) ? JNAN : v; }

// Java Math.round(float) for the finite values used here: floor(v + 1/2) without the rounding of
// v + 0.5f (v - floor(v) is exact for |v| < 2^23).
inline int jround(float v) { float f = floor(v); return (int)((v - f) >= 0.5f ? f + 1.0f : f); }
// Java Math.max(double, double): NaN if either is NaN, +0.0 above -0.0.
inline double jmax(double a, double b) {
  if (isnan(a) || isnan(b)) return (double)JNAN;
  if (a == b) return signbit(a) ? b : a;
  return a > b ? a : b;
}
// Java Math.clamp(value, 0, 1): NaN stays NaN, -0.0 becomes +0.0.
inline double jclamp(double v) { if (isnan(v)) return v; if (v <= 0.0) return 0.0; if (v >= 1.0) return 1.0; return v; }
inline int jvalid(double s) { return isfinite(s) && s >= 0 && s <= 1; }
// Arrays.sort(float[]) order for finite values: -0.0 before +0.0.
inline int fless(float a, float b) { return a < b || (a == b && signbit(a) && !signbit(b)); }

// CensusSgm.zncc
double zncc(__global const uchar* a, __global const uchar* b, int w, int h, int x, int y,
            double disparity, int r, int channels) {
  double target = x - disparity;
  int bx = (int)floor(target);
  double fraction = target - bx;
  if (x - r < 0 || x + r >= w || y - r < 0 || y + r >= h || bx - r < 0 || bx + r + 1 >= w) return -1;
  double covariance = 0, varA = 0, varB = 0;
  int n = (2 * r + 1) * (2 * r + 1);
  for (int channel = 0; channel < channels; channel++) {
    double sa = 0, sb = 0, saa = 0, sbb = 0, sab = 0;
    for (int dy = -r; dy <= r; dy++)
      for (int dx = -r; dx <= r; dx++) {
        int ai = ((y + dy) * w + x + dx) * channels + channel, bi = ((y + dy) * w + bx + dx) * channels + channel;
        double av = (double)a[ai];
        double bv = (1 - fraction) * (double)b[bi] + fraction * (double)b[bi + channels];
        sa += av; sb += bv; saa += av * av; sbb += bv * bv; sab += av * bv;
      }
    covariance += sab - sa * sb / n;
    varA += saa - sa * sa / n;
    varB += sbb - sb * sb / n;
  }
  if (varA < channels * n * 4 || varB < channels * n * 4) return -1;
  return covariance / sqrt(jmax(1e-12, varA * varB));
}

// PointConfidence.score
double score(double photo, double margin, double sharpness, double lr) {
  if (!isfinite(photo) || !isfinite(margin) || !isfinite(sharpness) || !isfinite(lr) || lr < 0) return (double)JNAN;
  return sqrt(jclamp(photo)) * sqrt(jclamp(margin) * jclamp(sharpness)) / (1 + lr);
}

__kernel void lr_check(__global const float* dl, __global const float* dr, __global float* checked,
                       __global uint* counts, __global const double* params, int w, int h, int radius) {
  int p = get_global_id(0);
  if (p >= w * h) return;
  int x = p % w, y = p / w;
  checked[p] = JNAN;
  if (y < radius || y >= h - radius || x < radius || x >= w - radius) return;
  float d = dl[p];
  if (!isfinite(d)) return;
  int xr = jround((float)x - d);
  if (xr < radius || xr >= w - radius) return;
  float other = dr[y * w + xr];
  if (isfinite(other) && (double)fabs(d - other) <= params[0]) checked[p] = d;
  else atomic_inc(&counts[0]);
}

__kernel void median(__global const float* a, __global const uchar* gray, __global float* out,
                     __global const double* params, int w, int h, int window, int edge, int edgeThreshold) {
  int p = get_global_id(0);
  if (p >= w * h) return;
  int x = p % w, y = p / w, r = window / 2;
  float center = a[p];
  out[p] = center;
  if (!isfinite(center)) return;
  float values[49];
  int n = 0;
  for (int yy = max(0, y - r); yy <= min(h - 1, y + r); yy++)
    for (int xx = max(0, x - r); xx <= min(w - 1, x + r); xx++) {
      float v = a[yy * w + xx];
      if (isfinite(v)) {
        int i = n++;
        while (i > 0 && fless(v, values[i - 1])) { values[i] = values[i - 1]; i--; }
        values[i] = v;
      }
    }
  float m = values[n / 2];
  if (edge && !((double)fabs(m - center) < params[1])) {
    int k = 0;
    int g = gray[p];
    for (int yy = max(0, y - r); yy <= min(h - 1, y + r); yy++)
      for (int xx = max(0, x - r); xx <= min(w - 1, x + r); xx++) {
        float v = a[yy * w + xx];
        if (isfinite(v) && (double)fabs(v - center) <= params[1]
            && (edgeThreshold == 0 || abs((int)gray[yy * w + xx] - g) <= edgeThreshold)) {
          int i = k++;
          while (i > 0 && fless(v, values[i - 1])) { values[i] = values[i - 1]; i--; }
          values[i] = v;
        }
      }
    if (k > 0) m = values[k / 2];
  }
  out[p] = m;
}

__kernel void post_check(__global float* checked, __global const float* dr, __global const uchar* pl,
                         __global const uchar* pr, __global float* correlation, __global float* lrError,
                         __global uint* counts, __global const double* params, int w, int h, int radius,
                         int channels, int zr) {
  int p = get_global_id(0);
  if (p >= w * h) return;
  int x = p % w, y = p / w;
  correlation[p] = JNAN;
  lrError[p] = JNAN;
  float disparity = checked[p];
  if (!isfinite(disparity)) return;
  int xr = jround((float)x - disparity);
  if (xr >= 0 && xr < w && isfinite(dr[y * w + xr])) lrError[p] = fabs(disparity - dr[y * w + xr]);
  int reciprocal = y >= radius && y < h - radius && xr >= radius && xr < w - radius
      && isfinite(dr[y * w + xr]) && (double)fabs(disparity - dr[y * w + xr]) <= params[0];
  if (!reciprocal) {
    checked[p] = JNAN;
    atomic_inc(&counts[1]);
    return;
  }
  double s = zncc(pl, pr, w, h, x, y, (double)disparity, zr, channels);
  correlation[p] = jf((float)s);
  if (s < params[2]) {
    checked[p] = JNAN;
    atomic_inc(&counts[2]);
  }
}

__kernel void left_confidence(__global const float* disparity, __global const float* correlation,
                              __global const float* lrError, __global const float* costMargin,
                              __global const float* costSharpness, __global const uchar* left,
                              __global const uchar* right, __global float* out, int w, int h) {
  int p = get_global_id(0);
  if (p >= w * h) return;
  int x = p % w, y = p / w;
  out[p] = JNAN;
  if (y < 1 || y >= h - 1 || x < 1 || x >= w - 1) return;
  float d = disparity[p];
  if (!isfinite(d)) return;
  double center = zncc(left, right, w, h, x, y, (double)d, 1, 1);
  double minus = zncc(left, right, w, h, x, y, (double)(d - 1), 1, 1);
  double plus = zncc(left, right, w, h, x, y, (double)(d + 1), 1, 1);
  double competitor = jmax(zncc(left, right, w, h, x, y, (double)(d - 2), 1, 1),
                           zncc(left, right, w, h, x, y, (double)(d + 2), 1, 1));
  double margin = jclamp((center - competitor) / jmax(.1, 1 - competitor));
  double sharpness = jclamp((2 * center - minus - plus) / jmax(.1, 2 - minus - plus));
  double photo = (double)correlation[p], lr = (double)lrError[p];
  double strength = sqrt(jclamp(costMargin[p] / .1) * jclamp(costSharpness[p] / .1));
  out[p] = jf((float)(score(photo, margin, sharpness, lr) * strength));
}

int patch(__global const uchar* mask, int masked, int w, int h, int x, int y, int radius) {
  if (x - radius < 0 || x + radius >= w || y - radius < 0 || y + radius >= h) return 0;
  if (masked)
    for (int yy = y - radius; yy <= y + radius; yy++)
      for (int xx = x - radius; xx <= x + radius; xx++) if (mask[yy * w + xx] == 0) return 0;
  return 1;
}

__kernel void right_confidence(__global const float* disparity, __global const float* dr,
                               __global const float* selMargin, __global const float* selSharpness,
                               __global const uchar* left, __global const uchar* right,
                               __global const uchar* photoLeft, __global const uchar* photoRight,
                               __global const uchar* maskLeft, __global const uchar* maskRight,
                               __global float* out, __global const double* params, int w, int h,
                               int radius, int zr, int channels, int maskedLeft, int maskedRight) {
  int p = get_global_id(0);
  if (p >= w * h) return;
  int x = p % w, y = p / w;
  out[p] = JNAN;
  if (y < radius || y >= h - radius) return;
  float d = disparity[p];
  if (!isfinite(d)) return;
  int rx = jround((float)x - d);
  if (rx < radius || rx + radius >= w) return;
  int q = y * w + rx;
  float rd = dr[q];
  if (!isfinite(rd) || !jvalid((double)selMargin[q]) || !jvalid((double)selSharpness[q])) return;
  int back = jround((float)rx + rd);
  if (back < 0 || back >= w || !isfinite(disparity[y * w + back])) return;
  double lr = (double)fabs(rd - disparity[y * w + back]);
  int fl = (int)floor((float)rx + rd);
  if (lr > params[0] || !patch(maskRight, maskedRight, w, h, rx, y, radius)
      || !patch(maskLeft, maskedLeft, w, h, fl, y, radius)
      || !patch(maskLeft, maskedLeft, w, h, fl + 1, y, radius)) return;
  double photo = zncc(photoRight, photoLeft, w, h, rx, y, (double)(-rd), zr, channels);
  if (photo < params[1]) { out[p] = 0; return; }
  double center = zncc(right, left, w, h, rx, y, (double)(-rd), 1, 1);
  double minus = zncc(right, left, w, h, rx, y, (double)(-rd - 1), 1, 1);
  double plus = zncc(right, left, w, h, rx, y, (double)(-rd + 1), 1, 1);
  double competitor = jmax(zncc(right, left, w, h, rx, y, (double)(-rd - 2), 1, 1),
                           zncc(right, left, w, h, rx, y, (double)(-rd + 2), 1, 1));
  double margin = jclamp((center - competitor) / jmax(.1, 1 - competitor));
  double sharpness = jclamp((2 * center - minus - plus) / jmax(.1, 2 - minus - plus));
  double strength = sqrt(jclamp(selMargin[q] / .1) * jclamp(selSharpness[q] / .1));
  out[p] = jf((float)(score(photo, margin, sharpness, lr) * strength));
}
""";
}
