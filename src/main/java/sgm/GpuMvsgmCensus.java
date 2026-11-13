package sgm;

import static java.lang.foreign.ValueLayout.*;

import java.lang.foreign.*;
import java.util.function.Supplier;

/**
 * Integer census of the CPU's warped/downsampled gray, written directly to an SGM session's
 * descriptor buffers. Warp, validity erosion, texture and downsample arithmetic stay on the CPU.
 * A reusable gray band includes the census halo; descriptors never need a normal-path readback.
 * Scratch is released before the packed cost volume is allocated, and the session reserves the
 * larger of the census and solve peaks. CPU descriptors are lazy, for checks and fallback only.
 */
final class GpuMvsgmCensus {
  private GpuMvsgmCensus() {}

  static final long MAX_SCRATCH = 4L << 20, MIN_BUDGET = 2L << 20;
  static final int MIN_ROWS = 64, LAUNCH_PIXELS = 4 << 20;
  private static volatile boolean disabled, verifiedReference, verifiedNeighbour;
  private static boolean reportedMemory;
  static long canvases, bands, grayBytes;

  static boolean enabled(Options o) {
    if (disabled || o.compute.equals("cpu") || Boolean.getBoolean("sgm.cpuMvsgm")
        || Boolean.getBoolean("sgm.cpuMvsgmCensus")) return false;
    if (o.gpuMemoryMb > 0 && o.gpuMemoryMb * 1048576L < MIN_BUDGET) {
      memoryFallback();
      return false;
    }
    return true;
  }

  static synchronized void memoryFallback() {
    if (!reportedMemory) {
      reportedMemory = true;
      System.out.println("mvsgm census: GPU memory or device solve unavailable; using the CPU (same results)");
    }
  }

  static synchronized void failed(Throwable error) {
    if (!disabled) {
      disabled = true;
      System.out.println("WARNING: mvsgm GPU census failed; using the CPU for the rest of this process (same results): " + error.getMessage());
    }
  }

  /** A census build failure must not prevent the existing CPU-descriptor GPU solve. */
  static GpuSgm.Compute compute(Options o, String stereoSource) {
    if (!enabled(o)) return new GpuSgm.Compute(o, stereoSource);
    try {
      return new GpuSgm.Compute(o, stereoSource + SOURCE);
    } catch (RuntimeException | Error error) {
      failed(error);
      return new GpuSgm.Compute(o, stereoSource);
    }
  }

  static int bandRows(int w, int h, int window, long bytes) {
    if (bytes >= (long) w * h) return h;
    long rows = Math.min(h, bytes / w - 2L * (window / 2));
    return rows < Math.min(MIN_ROWS, h) ? 0 : (int) rows;
  }

  static long scratchBytes(MultiViewSgm.Plan plan, long available, long allocation) {
    long largest = 0;
    for (var v : plan.views) largest = Math.max(largest, (long) v.w * v.h);
    long bytes = Math.min(Math.min(MAX_SCRATCH, largest), Math.min(available, allocation));
    for (var v : plan.views) if (bandRows(v.w, v.h, v.window, bytes) == 0) return 0;
    return Math.max(0, bytes);
  }

  /** The source buffer is the reference gray already uploaded, or a reusable neighbour band. */
  static void generate(GpuSgm.Compute cl, MemorySegment kernel, MemorySegment source,
      MemorySegment out, int offset, byte[] gray, int w, int h, int window,
      boolean reference, long scratchBytes, Supplier<long[]> cpu) {
    int rows = reference ? h : bandRows(w, h, window, scratchBytes);
    if (rows == 0) throw new IllegalArgumentException("No census gray band fits");
    try (Arena arena = Arena.ofConfined()) {
      MemorySegment host = reference ? null : arena.allocate(Math.min((long) gray.length, scratchBytes));
      for (int a = 0; a < h; a += rows) {
        int b = Math.min(h, a + rows), u0 = reference ? 0 : Math.max(0, a - window / 2);
        int u1 = reference ? h : Math.min(h, b + window / 2);
        if (!reference) {
          long count = (long) (u1 - u0) * w;
          MemorySegment part = host.asSlice(0, count);
          MemorySegment.copy(gray, u0 * w, part, JAVA_BYTE, 0, (int) count);
          cl.transfer(source, 0, part, false);
          grayBytes += count;
        }
        int pixels = (b - a) * w;
        for (int first = 0; first < pixels; first += LAUNCH_PIXELS)
          cl.run(kernel, Math.min(LAUNCH_PIXELS, pixels - first), source, out,
              offset, w, h, window / 2, u0, a, first, pixels);
        bands++;
      }
      // The host band and device scratch may be reused/freed only after their commands finish.
      cl.finish();
    }
    if (reference ? !verifiedReference : !verifiedNeighbour) check(cl, out, offset, w, h, cpu, reference);
    canvases++;
  }

  private static synchronized void check(GpuSgm.Compute cl, MemorySegment out, int offset,
      int w, int h, Supplier<long[]> cpu, boolean reference) {
    if (reference ? verifiedReference : verifiedNeighbour) return;
    long[] expected = cpu.get();
    long differ = 0;
    try (Arena arena = Arena.ofConfined()) {
      MemorySegment row = arena.allocate(8L * w, 8);
      for (int y = 0; y < h; y += 8) {
        cl.transfer(out, 8L * (offset + (long) y * w), row, true);
        for (int x = 0; x < w; x++) if (row.getAtIndex(JAVA_LONG, x) != expected[y * w + x]) differ++;
      }
    }
    if (differ != 0) throw new IllegalStateException(differ + " census descriptors differ from the CPU");
    if (reference) verifiedReference = true;
    else verifiedNeighbour = true;
    System.out.printf(java.util.Locale.ROOT,
        "mvsgm GPU census: %s %dx%d, every eighth row identical to the CPU%n",
        reference ? "reference" : "neighbour", w, h);
  }

  static final String SOURCE = """
#pragma OPENCL FP_CONTRACT OFF
__kernel void mvcensus(__global const uchar *gray, __global ulong *out,
    int offset, int w, int h, int r, int u0, int a, int first, int pixels) {
  int i = first + (int)get_global_id(0); if (i >= pixels) return;
  int x = i % w, y = a + i / w;
  ulong word = 0;
  if (x >= r && x < w-r && y >= r && y < h-r) {
    int p = (y-u0)*w+x, bit = 0;
    uchar center = gray[p];
    for (int yy=-r; yy<=r; yy++) for (int xx=-r; xx<=r; xx++) {
      if (xx==0 && yy==0) continue;
      word |= ((ulong)(gray[p+yy*w+xx] < center)) << bit;
      bit++;
    }
  }
  out[(size_t)offset+(size_t)y*w+x] = word;
}
""";
}
