package sgm;

import static java.lang.foreign.ValueLayout.*;

import java.lang.foreign.*;
import java.util.List;

/**
 * The image fusion's spike filter ({@link ImageFusion#spikeRow}) on the GPU: the 5 x 5 median and
 * MAD decision per pixel of a merged depth map. The kernel repeats the Java code with the same
 * float and double operations in the same order and no FMA contraction; it uses only operations
 * that are correctly rounded on both sides (+ - * /, comparisons) and the same order statistics
 * (a sort instead of a selection), so the marks are the same bit for bit.
 *
 * <p>Device memory: 4 bytes per uploaded pixel (the rows of a band and two halo rows on each side)
 * and 1 byte per decided pixel. A map is sent in bands of rows that fit the GPU budget
 * (--gpu-memory-mb, default 70% of the device), a single device allocation and {@link #BAND_PIXELS};
 * {@link #marks} returns null, and the CPU code runs, when not even {@link #MIN_BAND_ROWS} rows
 * fit, with --compute cpu or -Dsgm.cpuSpikeFilter=true, for small maps, without an OpenCL FP64
 * device, or when the shared pool has no room. Cameras that reach the filter at the same time take
 * turns, so one band is on the device at a time. The first map is compared with the CPU code on
 * every eighth row; a difference or a GPU failure switches the GPU path off for the rest of the
 * process.
 */
final class GpuSpikeFilter {
  private GpuSpikeFilter() {}

  static final long MIN_PIXELS = 250_000, BAND_PIXELS = 16L << 20;
  static final int MIN_BAND_ROWS = 256, HALO = 2;
  /** Pixels per kernel launch (display drivers stop long kernels; this one takes milliseconds). */
  private static final int LAUNCH_PIXELS = 4 << 20;

  private static volatile boolean disabled, verified;
  private static boolean reported;
  private static GpuSgm.Compute cl;
  private static MemorySegment kernel;
  /** Maps and bands decided on the GPU, maps left to the CPU for memory, seconds on the GPU path. */
  static long maps, bands, cpuMaps;
  static double seconds;

  static boolean applies(Options o, int w, int rows) {
    return !disabled
        && o != null
        && !o.compute.equals("cpu")
        && !Boolean.getBoolean("sgm.cpuSpikeFilter")
        && (long) w * rows >= MIN_PIXELS;
  }

  private static void failed(String message) {
    if (!disabled) {
      disabled = true;
      System.out.println("WARNING: GPU spike filter unavailable, using the CPU (same results): " + message);
    }
    close();
  }

  /** Device bytes for a band of rows rows (plus halo) of width w. */
  static long bytes(long w, long rows) {
    return 4 * w * (rows + 2 * HALO) + w * rows + 4096;
  }

  /** Rows per band for a map of width w within budget and allocation; 0 when too few fit. */
  static int bandRows(long w, int rows, long budget, long allocation) {
    long fit = Math.min((budget - 4096 - 8 * HALO * w) / (5 * w),allocation / (4 * w) - 2 * HALO);
    fit = Math.min(fit, Math.max(MIN_BAND_ROWS, BAND_PIXELS / w));
    fit = Math.min(fit, rows);
    return fit < Math.min(MIN_BAND_ROWS, rows) ? 0 : (int) fit;
  }

  /**
   * Spike marks (1 = spike) of rows [from, to) of the w x h map z, index (y - from) * w + x, as
   * ImageFusion.spikeRow gives them; null when the GPU path does not apply (use the CPU).
   */
  static synchronized byte[] marks(Options o, float[] z, int w, int h, int from, int to, double focal) {
    int rows = to - from;
    if (rows <= 0 || !applies(o, w, rows)) return null;
    long clock = System.nanoTime();
    byte[] marks = new byte[rows * w];
    GpuMemoryPool.Reservation reservation = null;
    try {
      List<GpuSgm.Device> devices = GpuSgm.devices();
      if (o.gpuDevice >= devices.size()) return null;
      GpuSgm.Device device = devices.get(o.gpuDevice);
      long budget = GpuSgm.budget(device, o);
      int band = bandRows(w, rows, budget, device.allocation());
      if (band == 0) {
        cpuMaps++;
        if (!reported) {
          reported = true;
          System.out.printf(
              java.util.Locale.ROOT,
              "GPU spike filter: %d rows of a %d pixel wide map need %,d KiB of device memory, the budget is %,d KiB; using the CPU (same results)%n",
              Math.min(MIN_BAND_ROWS, rows), w, bytes(w, Math.min(MIN_BAND_ROWS, rows)) >> 10, budget >> 10);
        }
        return null;
      }
      long deviceBytes = bytes(w, band);
      reservation = GpuMemoryPool.reserveAuxiliary(deviceBytes);
      if (reservation == null) return null;
      if (cl == null) {
        cl = new GpuSgm.Compute(o, SOURCE, true);
        kernel = cl.kernel("spikes");
      }
      MemorySegment bZ = cl.buffer(4L * w * (band + 2 * HALO)), bMarks = cl.buffer((long) w * band), bParams = cl.buffer(8);
      try (Arena arena = Arena.ofConfined()) {
        cl.transfer(bParams, 0, arena.allocateFrom(JAVA_DOUBLE, focal), false);
        int slice = 1 << 22;
        MemorySegment floats = arena.allocate(4L * slice, 4), bytes = arena.allocate((long) w * band);
        for (int a = from; a < to; a += band) {
          int b = Math.min(to, a + band), u0 = Math.max(0, a - HALO), u1 = Math.min(h, b + HALO);
          long count = (long) (u1 - u0) * w;
          // The rows go up in slices, so the host copy stays small.
          for (long at = 0; at < count; at += slice) {
            int n = (int) Math.min(slice, count - at);
            MemorySegment.copy(z, (int) ((long) u0 * w + at), floats, JAVA_FLOAT, 0, n);
            cl.transfer(bZ, 4 * at, floats.asSlice(0, 4L * n), false);
          }
          int pixels = (b - a) * w;
          for (int first = 0; first < pixels; first += LAUNCH_PIXELS) {
            cl.run(kernel, Math.min(LAUNCH_PIXELS, pixels - first), bZ, bMarks, bParams, w, h, u0, a, first, pixels);
            cl.finish();
          }
          MemorySegment part = bytes.asSlice(0, pixels);
          cl.transfer(bMarks, 0, part, true);
          MemorySegment.copy(part, JAVA_BYTE, 0, marks, (a - from) * w, pixels);
          bands++;
        }
      } finally {
        cl.releaseBuffers();
      }
      if (!verified && !check(z, w, h, from, to, focal, marks, deviceBytes)) return null;
      maps++;
      seconds += (System.nanoTime() - clock) / 1e9;
      return marks;
    } catch (RuntimeException | Error e) {
      failed(e.toString());
      return null;
    } finally {
      if (reservation != null) reservation.close();
    }
  }

  private static boolean check(float[] z, int w, int h, int from, int to, double focal, byte[] marks, long deviceBytes) {
    int step = 8, rows = (to - from + step - 1) / step;
    long[] differ = new long[rows], valid = new long[rows], spikes = new long[rows];
    Parallel.forEach(rows, Runtime.getRuntime().availableProcessors(), i -> {
      int y = from + i * step, at = (y - from) * w;
      byte[] cpu = new byte[w];
      spikes[i] = ImageFusion.spikeRow(z, w, h, focal, y, new float[24], new float[24], cpu, 0);
      for (int x = 0; x < w; x++) {
        if (cpu[x] != marks[at + x]) differ[i]++;
        if (z[y * w + x] > 0) valid[i]++;
      }
    });
    long mismatches = 0, decided = 0, found = 0;
    for (int i = 0; i < rows; i++) {
      mismatches += differ[i];
      decided += valid[i];
      found += spikes[i];
    }
    if (mismatches > 0) {
      failed(mismatches + " of " + (long) rows * w + " sampled pixels differ from the CPU");
      return false;
    }
    // A map without depths proves nothing; the next one is checked instead.
    if (decided == 0) return true;
    verified = true;
    System.out.printf(
        java.util.Locale.ROOT,
        "GPU spike filter: %d sampled rows (%d depths, %d spikes) identical to the CPU code; %,d MiB of device memory per band%n",
        rows, decided, found, Math.max(1, deviceBytes >> 20));
    return true;
  }

  /** Releases the device context; the next use opens it again. */
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

// Java Math.max(double, double) for values that are not NaN (a NaN operand gives NaN, as in Java).
inline double jmax(double a, double b) {
  if (isnan(a) || isnan(b)) return a + b;
  return a > b ? a : b;
}

// ImageFusion.spikeRow for one pixel. z holds image rows from u0 on; pixel i of the launch is
// pixel first + i of the band that starts at image row a.
__kernel void spikes(__global const float* z, __global uchar* marks, __global const double* params,
                     int w, int h, int u0, int a, int first, int pixels) {
  int i = first + (int)get_global_id(0);
  if (i >= pixels) return;
  int y = a + i / w, x = i % w;
  marks[i] = 0;
  float centre = z[(y - u0) * w + x];
  if (centre <= 0) return;
  double focal = params[0];
  float v[24], d[24];
  int n = 0;
  for (int dy = -2; dy <= 2; dy++)
    for (int dx = -2; dx <= 2; dx++) {
      int xx = x + dx, yy = y + dy;
      if ((dx | dy) == 0 || xx < 0 || yy < 0 || xx >= w || yy >= h) continue;
      float value = z[(yy - u0) * w + xx];
      if (value > 0) {
        // Insertion sort: the same order statistics as the selection on the CPU.
        int at = n++;
        while (at > 0 && v[at - 1] > value) { v[at] = v[at - 1]; at--; }
        v[at] = value;
      }
    }
  if (n < 12) return;
  float upper = v[n / 2];
  float median = n % 2 == 1 ? upper : .5f * (v[n / 2 - 1] + upper);
  float off = fabs(centre - median);
  double footprint = (double)median / focal;
  if ((double)off <= 2 * footprint) return;
  for (int k = 0; k < n; k++) {
    float value = fabs(v[k] - median);
    int at = k;
    while (at > 0 && d[at - 1] > value) { d[at] = d[at - 1]; at--; }
    d[at] = value;
  }
  double middle = n % 2 == 1 ? (double)d[n / 2] : .5 * (double)(d[n / 2 - 1] + d[n / 2]);
  double spread = 1.4826 * middle;
  double limit = jmax(4 * spread, 2 * footprint);
  if ((double)off <= limit) return;
  int own = 0;
  double near = jmax(spread, footprint);
  for (int k = 0; k < n && own < 2; k++) if ((double)fabs(v[k] - centre) <= near) own++;
  if (own >= 2) return;
  marks[i] = 1;
}
""";
}
