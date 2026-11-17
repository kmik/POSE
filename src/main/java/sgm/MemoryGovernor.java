package sgm;

import java.util.Locale;

/**
 * Admission of parallel work by the process's real memory, heap and native together
 * (MEMORY_FIX_20261109.md, 16 November evening). The heap plan bounds Java objects only; OpenCV
 * images, SIFT pyramids and FLANN indexes of every worker come on top, so a run with many cores
 * can use far more RAM than --memory-mb or --ram-limit-mb suggest, and the kernel kills it when a
 * shared machine runs out.
 *
 * <p>Each unit of parallel work (one image's features, one image pair) declares an estimate of the
 * native memory it needs and is admitted while
 *
 * <ul>
 *   <li>committed process memory ({@link NativeMemory#committedBytes}) + the estimates of the units
 *       running now + this estimate stay within {@link #LIMIT_FRACTION} of the RAM limit
 *       (--ram-limit-mb, a cgroup/Slurm job limit or the host's RAM), and
 *   <li>the host keeps max(2 GiB, 5%) available after this estimate (other users' processes).
 * </ul>
 *
 * Running units are counted twice (measured and estimated), which errs on the safe side near the
 * limit and costs nothing far from it. When a unit must wait, freed native memory is handed back
 * first ({@link NativeMemory#trim}); one unit always runs, so a run never stalls. The order and the
 * content of results do not change: only when independent units start.
 */
final class MemoryGovernor {
  private MemoryGovernor() {}

  static final double LIMIT_FRACTION = 0.95;
  private static final long MEASURE_NANOS = 100_000_000L;

  private static int active;
  private static long reserved;
  private static long measuredAt;
  private static long committed = -1, limit = -1, hostAvailable = -1, hostTotal = -1;
  private static long lastTrim;
  static long waits, waitNanos, admitted;
  private static boolean reported, pretending;

  /** A running unit; close() releases its admission. */
  interface Admission extends AutoCloseable {
    @Override
    void close();
  }

  private static final Admission NONE = () -> {};

  /** Off with -Dsgm.memoryGovernor=false or an estimate of 0. */
  static boolean enabled() {
    return !"false".equals(System.getProperty("sgm.memoryGovernor"));
  }

  /** Waits until a unit needing about bytes of native memory fits, then admits it. */
  static Admission admit(String what, long bytes, Options o) {
    if (!enabled() || bytes <= 0) return NONE;
    long started = 0;
    boolean waited = false;
    synchronized (MemoryGovernor.class) {
      while (active > 0 && !fits(bytes, o)) {
        if (!waited) {
          waited = true;
          started = System.nanoTime();
          waits++;
          if (!reported) {
            reported = true;
            System.out.printf(
                Locale.ROOT,
                "Memory governor: %s waits for memory (process %,d MiB, running units %,d MiB,"
                    + " limit %,d MiB, host available %,d MiB); fewer units run at once%n",
                what,
                committed >> 20,
                reserved >> 20,
                limit >> 20,
                hostAvailable >> 20);
          }
        }
        // Hand freed native memory back at most once a second while waiting.
        if (System.nanoTime() - lastTrim > 1_000_000_000L) {
          lastTrim = System.nanoTime();
          NativeMemory.trim("while waiting for memory");
          measuredAt = 0;
          continue;
        }
        try {
          MemoryGovernor.class.wait(200);
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
          throw new IllegalStateException("Interrupted while waiting for memory", e);
        }
        measuredAt = 0;
      }
      active++;
      reserved += bytes;
      admitted++;
      if (waited) waitNanos += System.nanoTime() - started;
    }
    long held = bytes;
    return new Admission() {
      boolean closed;

      @Override
      public void close() {
        synchronized (MemoryGovernor.class) {
          if (closed) return;
          closed = true;
          active--;
          reserved -= held;
          measuredAt = 0;
          MemoryGovernor.class.notifyAll();
        }
      }
    };
  }

  /** Caller holds the lock. */
  private static boolean fits(long bytes, Options o) {
    measure(o);
    long room = Long.MAX_VALUE;
    if (limit > 0 && committed >= 0) room = (long) (limit * LIMIT_FRACTION) - committed;
    if (hostAvailable >= 0) {
      long reserve = Math.max(2L << 30, hostTotal > 0 ? hostTotal / 20 : 0);
      room = Math.min(room, hostAvailable - reserve);
    }
    return room == Long.MAX_VALUE || reserved + bytes <= room;
  }

  private static void measure(Options o) {
    long now = System.nanoTime();
    if (pretending || measuredAt != 0 && now - measuredAt < MEASURE_NANOS) return;
    measuredAt = now;
    committed = NativeMemory.committedBytes();
    MemorySafety.Limits host = MemorySafety.limits();
    hostTotal = host.total();
    hostAvailable = host.available();
    limit = o != null && o.ramLimitMb > 0 ? LinuxMemory.minimum(host.total(), o.ramLimitMb * MemorySafety.MIB) : host.total();
  }

  /** Native bytes of one image's feature detection (decoded raster + detector scale space). */
  static long featureBytes(ImageFrame f, Options o) {
    long pixels = (long) f.width * f.height;
    // OpenCV SIFT doubles the image and keeps float Gaussian and difference-of-Gaussian pyramids
    // (6 + 5 levels per octave, 4/3 for the octaves): about 4 x 4 x 11 x 4/3 = 235 bytes per pixel.
    long detector = o.matchingAlgorithm.equals("sift") ? 240 : 64;
    return pixels * (detector + 8);
  }

  /** Native bytes of one pair's matching: both descriptor sets, the matcher index and matches. */
  static long pairBytes(Options o) {
    int descriptorBytes = o.matchingAlgorithm.equals("sift") ? 512 : 64;
    return 4L * o.features * descriptorBytes + (16L << 20);
  }

  static synchronized java.util.Map<String, Object> summary() {
    return Json.object(
        "admitted_units", admitted,
        "waits", waits,
        "wait_seconds", waitNanos / 1e9,
        "malloc_trims", NativeMemory.trims,
        "malloc_trim_returned_mib", NativeMemory.trimmedBytes >> 20);
  }

  /** Test hook: forget measurements and counters. */
  static synchronized void reset() {
    pretending = false;
    active = 0;
    reserved = 0;
    measuredAt = 0;
    waits = 0;
    waitNanos = 0;
    admitted = 0;
    reported = false;
  }

  /** Test hook: fixed measurements instead of the operating system's. */
  static synchronized void pretend(long committedBytes, long limitBytes, long available, long total) {
    committed = committedBytes;
    limit = limitBytes;
    hostAvailable = available;
    hostTotal = total;
    pretending = true;
  }
}
