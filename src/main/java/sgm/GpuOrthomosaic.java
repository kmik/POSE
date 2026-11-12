package sgm;

import static java.lang.foreign.ValueLayout.*;

import java.lang.foreign.*;
import java.util.List;

/**
 * The orthomosaic's choice of a camera per pixel ({@link Orthomosaic#chooseRow}) on the GPU: the
 * surface height, the projection into every candidate camera and the line-of-sight walks over the
 * surface grid. The kernel repeats the Java code in FP64 with the same operation order and no FMA
 * contraction; it uses only operations that are correctly rounded on both sides (+ - * / sqrt,
 * floor, comparisons), so the result is the same bit for bit.
 *
 * <p>Device memory: the surface grid (5 bytes per cell, uploaded once) and the outputs of one
 * block (7 bytes per pixel). {@link #open} returns null, and the CPU code runs, when that does not
 * fit the GPU budget (--gpu-memory-mb, default 70% of the device) or a single allocation, with
 * --compute cpu, without an OpenCL FP64 device, or when the shared pool has no room. A block with
 * more than {@link #MAX_CAMERAS} candidate cameras is left to the CPU. The work is sent in chunks
 * of rows sized to about a quarter of a second each (display drivers stop longer kernels). The
 * first block is compared with the CPU code on every sixteenth row; a difference or a GPU failure
 * switches the GPU path off for the rest of the process.
 */
final class GpuOrthomosaic implements AutoCloseable {
  static final int MAX_CAMERAS = 64, VIEW = 27;
  private static volatile boolean disabled, verified;

  private final GpuSgm.Compute cl;
  private final GpuMemoryPool.Reservation reservation;
  private final Orthomosaic.Surface surface;
  private final MemorySegment kernel, bZ, bKind, bViews, bParams, bChosen, bHeights, bState;
  private final int capacity;
  private int chunk = 32;
  long blocks, cpuBlocks, deviceBytes;
  double seconds;

  private static void failed(String message) {
    if (!disabled) {
      disabled = true;
      System.out.println("WARNING: GPU orthomosaic unavailable, using the CPU (same results): " + message);
    }
  }

  /** Device bytes for a surface of cells cells and blocks of pixels pixels. */
  static long bytes(long cells, long pixels) {
    return 5 * cells + 7 * pixels + 8L * (MAX_CAMERAS * VIEW + 8) + 4096;
  }

  /** A session for this surface and blocks of up to pixels pixels, or null (use the CPU). */
  static GpuOrthomosaic open(Options o, Orthomosaic.Surface surface, int pixels) {
    if (disabled || o == null || o.compute.equals("cpu") || Boolean.getBoolean("sgm.cpuOrthomosaic") || surface.kind == null) return null;
    long cells = surface.z.length, bytes = bytes(cells, pixels);
    GpuMemoryPool.Reservation reservation = null;
    try {
      GpuSgm.Device device = GpuSgm.selectedDevice(o);
      long budget = GpuSgm.budget(device, o);
      if (bytes > budget || 4 * cells > device.allocation()) {
        System.out.printf(
            java.util.Locale.ROOT,
            "GPU orthomosaic: the surface needs %,d MiB of device memory, the budget is %,d MiB; using the CPU (same results)%n",
            bytes >> 20, budget >> 20);
        return null;
      }
      reservation = GpuMemoryPool.reserveAuxiliary(bytes);
      if (reservation == null) return null;
      return new GpuOrthomosaic(o, surface, pixels, reservation, bytes);
    } catch (RuntimeException | Error e) {
      if (reservation != null) reservation.close();
      failed(e.toString());
      return null;
    }
  }

  private GpuOrthomosaic(Options o, Orthomosaic.Surface surface, int pixels, GpuMemoryPool.Reservation reservation, long bytes) {
    this.surface = surface;
    this.reservation = reservation;
    capacity = pixels;
    deviceBytes = bytes;
    cl = new GpuSgm.Compute(o, SOURCE);
    try (Arena arena = Arena.ofConfined()) {
      long cells = surface.z.length;
      kernel = cl.kernel("choose");
      bZ = cl.buffer(4 * cells);
      bKind = cl.buffer(cells);
      bViews = cl.buffer(8L * MAX_CAMERAS * VIEW);
      bParams = cl.buffer(64);
      bChosen = cl.buffer(2L * pixels);
      bHeights = cl.buffer(4L * pixels);
      bState = cl.buffer(pixels);
      // The grid goes up in slices, so the host copy stays small.
      int slice = 1 << 22;
      for (long from = 0; from < cells; from += slice) {
        int n = (int) Math.min(slice, cells - from);
        try (Arena part = Arena.ofConfined()) {
          MemorySegment floats = part.allocate(4L * n, 4);
          MemorySegment.copy(surface.z, (int) from, floats, JAVA_FLOAT, 0, n);
          cl.transfer(bZ, 4 * from, floats, false);
          MemorySegment kinds = part.allocate(n);
          MemorySegment.copy(surface.kind, (int) from, kinds, JAVA_BYTE, 0, n);
          cl.transfer(bKind, from, kinds, false);
        }
      }
    } catch (RuntimeException | Error e) {
      cl.close();
      throw e;
    }
  }

  /**
   * Fills chosen (index into views or -1), heights and state for the block as chooseRow does for
   * each of its rows; false when the block is left to the CPU (nothing usable was written).
   */
  boolean choose(
      List<Orthomosaic.View> views, int[] candidates, double left, double upper, double cell, int bw, int bh, int workers,
      boolean occlusion, boolean relax, short[] chosen, float[] heights, byte[] state) {
    if (disabled || candidates.length == 0 || candidates.length > MAX_CAMERAS || (long) bw * bh > capacity) {
      cpuBlocks++;
      return false;
    }
    long clock = System.nanoTime();
    int n = bw * bh;
    try (Arena arena = Arena.ofConfined()) {
      double[] packed = new double[candidates.length * VIEW];
      for (int k = 0; k < candidates.length; k++) {
        Orthomosaic.View v = views.get(candidates[k]);
        int at = k * VIEW;
        System.arraycopy(v.rotation, 0, packed, at, 9);
        System.arraycopy(v.translation, 0, packed, at + 9, 3);
        System.arraycopy(v.distortion, 0, packed, at + 12, 5);
        packed[at + 17] = v.fx;
        packed[at + 18] = v.fy;
        packed[at + 19] = v.cx;
        packed[at + 20] = v.cy;
        packed[at + 21] = v.scale;
        System.arraycopy(v.centre, 0, packed, at + 22, 3);
        packed[at + 25] = v.width;
        packed[at + 26] = v.height;
      }
      cl.transfer(bViews, 0, arena.allocateFrom(JAVA_DOUBLE, packed), false);
      cl.transfer(
          bParams, 0,
          arena.allocateFrom(JAVA_DOUBLE, surface.x0, surface.yTop, surface.cell, left, upper, cell, (double) surface.top, 0), false);
      int flags = (occlusion ? 1 : 0) | (relax ? 2 : 0);
      for (int row = 0; row < bh; ) {
        int rows = Math.min(chunk, bh - row);
        long started = System.nanoTime();
        cl.run(kernel, (long) rows * bw, bZ, bKind, bViews, bParams, bChosen, bHeights, bState, surface.w, surface.h, bw, row, rows, candidates.length, flags);
        cl.finish();
        double took = (System.nanoTime() - started) / 1e9;
        row += rows;
        // Aim at a quarter of a second per chunk; never more than doubling at once.
        if (took < .12) chunk = Math.min(bh, chunk * 2);
        else if (took > .5) chunk = Math.max(1, chunk / 2);
      }
      MemorySegment c = arena.allocate(2L * n, 2), hgt = arena.allocate(4L * n, 4), s = arena.allocate(n);
      cl.transfer(bChosen, 0, c, true);
      cl.transfer(bHeights, 0, hgt, true);
      cl.transfer(bState, 0, s, true);
      MemorySegment.copy(c, JAVA_SHORT, 0, chosen, 0, n);
      MemorySegment.copy(hgt, JAVA_FLOAT, 0, heights, 0, n);
      MemorySegment.copy(s, JAVA_BYTE, 0, state, 0, n);
    } catch (RuntimeException | Error e) {
      failed(e.toString());
      cpuBlocks++;
      return false;
    }
    // The kernel answers with the position in the candidate list.
    for (int p = 0; p < n; p++) if (chosen[p] >= 0) chosen[p] = (short) candidates[chosen[p]];
    if (!verified && !check(views, candidates, left, upper, cell, bw, bh, occlusion, relax, chosen, heights, state)) {
      cpuBlocks++;
      return false;
    }
    blocks++;
    seconds += (System.nanoTime() - clock) / 1e9;
    return true;
  }

  private synchronized boolean check(
      List<Orthomosaic.View> views, int[] candidates, double left, double upper, double cell, int bw, int bh,
      boolean occlusion, boolean relax, short[] chosen, float[] heights, byte[] state) {
    if (verified) return true;
    int step = 16, rows = (bh + step - 1) / step;
    short[] c = new short[rows * bw];
    float[] h = new float[rows * bw];
    byte[] s = new byte[rows * bw];
    Parallel.forEach(rows, Runtime.getRuntime().availableProcessors(), i ->
        Orthomosaic.chooseRow(surface, views, candidates, left, upper, cell, bw, i * step, occlusion, relax, c, h, s, i * bw));
    long mismatches = 0, decided = 0;
    for (int i = 0; i < rows; i++)
      for (int x = 0; x < bw; x++) {
        int a = i * bw + x, b = i * step * bw + x;
        if (c[a] != chosen[b] || s[a] != state[b] || Float.floatToRawIntBits(h[a]) != Float.floatToRawIntBits(heights[b])) mismatches++;
        if (s[a] != Orthomosaic.UNKNOWN) decided++;
      }
    if (mismatches > 0) {
      failed(mismatches + " of " + (long) rows * bw + " sampled pixels differ from the CPU");
      return false;
    }
    // A block without surface proves nothing; the next one is checked instead.
    if (decided == 0) return true;
    verified = true;
    System.out.printf(
        java.util.Locale.ROOT, "GPU orthomosaic: %d sampled rows (%d pixels with surface) identical to the CPU code; %,d MiB of device memory%n",
        rows, decided, deviceBytes >> 20);
    return true;
  }

  @Override
  public void close() {
    try {
      cl.close();
    } finally {
      reservation.close();
    }
  }

  static final String SOURCE =
      """
#pragma OPENCL EXTENSION cl_khr_fp64 : enable
#pragma OPENCL FP_CONTRACT OFF

#define VIEW 27
#define MAXC 64
#define SOFT 2

// Java Math.min / Math.max for values that are never NaN here.
inline double jmin(double a, double b) { return a < b ? a : b; }
inline double jmax(double a, double b) { return a > b ? a : b; }

// Orthomosaic.View.project.
inline int project(__global const double* c, double x, double y, double z, double* u, double* v) {
  double scale = c[21];
  double px = x / scale, py = y / scale, pz = z / scale;
  double q2 = c[6] * px + c[7] * py + c[8] * pz + c[11];
  if (!(q2 > 1e-9)) return 0;
  double xn = (c[0] * px + c[1] * py + c[2] * pz + c[9]) / q2;
  double yn = (c[3] * px + c[4] * py + c[5] * pz + c[10]) / q2;
  double fx = c[17], fy = c[18], cx = c[19], cy = c[20], width = c[25], height = c[26];
  double pu = fx * xn + cx, pv = fy * yn + cy;
  if (pu < -.25 * width || pv < -.25 * height || pu > 1.25 * width || pv > 1.25 * height) return 0;
  double r = xn * xn + yn * yn;
  double radial = 1 + c[12] * r + c[13] * r * r + c[16] * r * r * r;
  *u = fx * (xn * radial + 2 * c[14] * xn * yn + c[15] * (r + 2 * xn * xn)) + cx;
  *v = fy * (yn * radial + c[14] * (r + 2 * yn * yn) + 2 * c[15] * xn * yn) + cy;
  return 1;
}

// Orthomosaic.Surface.height.
inline double height(__global const float* z, int w, int h, double x0, double yTop, double cell, double x, double y) {
  double gx = (x - x0) / cell - .5, gy = (yTop - y) / cell - .5;
  int ix = (int)floor(gx), iy = (int)floor(gy);
  double fx = gx - ix, fy = gy - iy;
  int nx = max(0, min(w - 1, fx < .5 ? ix : ix + 1)), ny = max(0, min(h - 1, fy < .5 ? iy : iy + 1));
  float nearest = z[ny * w + nx];
  if (isnan(nearest) || ix < 0 || iy < 0 || ix >= w - 1 || iy >= h - 1) return (double)nearest;
  float a = z[iy * w + ix], b = z[iy * w + ix + 1], c = z[(iy + 1) * w + ix], d = z[(iy + 1) * w + ix + 1];
  if (isnan(a) || isnan(b) || isnan(c) || isnan(d)) return (double)nearest;
  float low = a, high = a;
  if (b < low) low = b;
  if (c < low) low = c;
  if (d < low) low = d;
  if (b > high) high = b;
  if (c > high) high = c;
  if (d > high) high = d;
  float range = high - low;
  if (!((double)range <= 2 * cell)) return (double)nearest;
  return (1 - fy) * ((1 - fx) * (double)a + fx * (double)b) + fy * ((1 - fx) * (double)c + fx * (double)d);
}

// Orthomosaic.Surface.visible.
inline int visible(__global const float* z, int w, int h, double x0, double yTop, double cell, double top,
                   __global const double* camera, double x, double y, double v, int lenient) {
  double dx = camera[0] - x, dy = camera[1] - y, dz = camera[2] - v, length = sqrt(dx * dx + dy * dy);
  if (length < cell || !(dz > 0)) return dz > 0;
  double end = jmin(length, length * (top - v) / dz), step = .75 * cell;
  for (double s = 2 * step; s < end; s += step) {
    double t = s / length;
    int col = (int)floor((x + dx * t - x0) / cell), row = (int)floor((yTop - y - dy * t) / cell);
    if (col < 0 || row < 0 || col >= w || row >= h) continue;
    double surface = (double)z[row * w + col];
    double tolerance = lenient ? 1.5 * cell : jmax(0, 1.5 * cell - .5 * (s - 2 * step));
    if (surface > v + dz * t + tolerance) return 0;
  }
  return 1;
}

// Orthomosaic.Surface.obstruction.
inline double obstruction(__global const float* z, __global const uchar* kind, int w, int h, double x0, double yTop,
                          double cell, double top, __global const double* camera, double x, double y, double v,
                          double limit) {
  double dx = camera[0] - x, dy = camera[1] - y, dz = camera[2] - v, length = sqrt(dx * dx + dy * dy);
  if (!(dz > 0)) return INFINITY;
  if (length < cell) return 0;
  double end = jmin(length, length * (top - v) / dz), step = .75 * cell, worst = 0;
  for (double s = 2 * step; s < end; s += step) {
    double t = s / length;
    int col = (int)floor((x + dx * t - x0) / cell), row = (int)floor((yTop - y - dy * t) / cell);
    if (col < 0 || row < 0 || col >= w || row >= h) continue;
    double over = (double)z[row * w + col] - (v + dz * t + 1.5 * cell);
    if (!(over > 0)) continue;
    if ((kind[row * w + col] & SOFT) == 0) return INFINITY;
    if (over > worst) {
      worst = over;
      if (worst > limit) return INFINITY;
    }
  }
  return worst;
}

// Orthomosaic.chooseRow for one pixel. state: 0 painted, 1 no surface, 2 seen by no camera,
// 3 painted by the least obstructed camera.
__kernel void choose(__global const float* z, __global const uchar* kind, __global const double* views,
                     __global const double* params, __global short* chosen, __global float* heights,
                     __global uchar* state, int w, int h, int bw, int row0, int rows, int n, int flags) {
  int i = get_global_id(0);
  if (i >= bw * rows) return;
  int row = row0 + i / bw, col = i % bw, p = row * bw + col;
  double x0 = params[0], yTop = params[1], cell = params[2], left = params[3], upper = params[4], pixel = params[5];
  double top = params[6];
  int occlusion = flags & 1, relax = flags & 2;
  double y = upper - (row + .5) * pixel, x = left + (col + .5) * pixel;
  double v = height(z, w, h, x0, yTop, cell, x, y);
  heights[p] = (float)v;
  chosen[p] = -1;
  if (isnan(v)) { state[p] = 1; return; }
  double scores[MAXC], pending[MAXC], u0, u1;
  for (int k = 0; k < n; k++) {
    __global const double* c = views + k * VIEW;
    scores[k] = INFINITY;
    if (project(c, x, y, v, &u0, &u1) && u0 >= 0 && u1 >= 0 && u0 <= c[25] - 1 && u1 <= c[26] - 1) {
      double a = (u0 - c[19]) / c[17], b = (u1 - c[20]) / c[18];
      scores[k] = a * a + b * b;
    }
    pending[k] = scores[k];
  }
  int pick = -1, lenient = -1;
  while (1) {
    int k = -1;
    double best = INFINITY;
    for (int j = 0; j < n; j++)
      if (pending[j] < best) {
        best = pending[j];
        k = j;
      }
    if (k < 0) break;
    pending[k] = INFINITY;
    __global const double* c = views + k * VIEW;
    if (!occlusion || visible(z, w, h, x0, yTop, cell, top, c + 22, x, y, v, 0)) {
      pick = k;
      break;
    }
    if (lenient < 0 && visible(z, w, h, x0, yTop, cell, top, c + 22, x, y, v, 1)) lenient = k;
  }
  if (pick < 0) pick = lenient;
  int relaxed = 0;
  if (pick < 0 && relax) {
    double excess[MAXC];
    double least = INFINITY;
    for (int k = 0; k < n; k++) {
      __global const double* c = views + k * VIEW;
      excess[k] = INFINITY;
      if (scores[k] == INFINITY) continue;
      excess[k] = obstruction(z, kind, w, h, x0, yTop, cell, top, c + 22, x, y, v, 1.25 * least + 4 * cell);
      least = jmin(least, excess[k]);
    }
    if (least < INFINITY) {
      double limit = 1.25 * least + 4 * cell, best = INFINITY;
      for (int k = 0; k < n; k++)
        if (excess[k] <= limit && scores[k] < best) {
          best = scores[k];
          pick = k;
        }
      relaxed = 1;
    }
  }
  chosen[p] = (short)pick;
  state[p] = relaxed ? 3 : pick < 0 ? 2 : 0;
}
""";
}
