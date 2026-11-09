package sgm;

import static java.lang.foreign.ValueLayout.*;

import com.sun.management.HotSpotDiagnosticMXBean;
import java.lang.foreign.*;
import java.lang.invoke.MethodHandle;
import java.lang.management.ManagementFactory;
import java.nio.file.*;
import java.util.Locale;

/** Prevent native commit failure before allocating; catch(OOME) cannot recover from it. */
final class MemorySafety {
  static final long MIB = 1048576L;

  record Limits(long total, long available, String source) {}

  static Limits limits() {
    if (System.getProperty("os.name").toLowerCase(Locale.ROOT).contains("linux")) {
      var linux = LinuxMemory.read(Path.of("/proc"), System.getenv());
      return new Limits(linux.total(), linux.available(), linux.source());
    }
    long[] windows = Windows.read();
    long total = -1, free = windows[1];
    if (ManagementFactory.getOperatingSystemMXBean() instanceof com.sun.management.OperatingSystemMXBean mx) {
      total = mx.getTotalMemorySize();
      if (free < 0) free = mx.getFreeMemorySize();
    }
    return new Limits(total, LinuxMemory.minimum(windows[0], free), "Host physical/commit memory");
  }

  static Limits limits(long ramLimitMiB) {
    Limits host = limits();
    if (ramLimitMiB <= 0) return host;
    long limit = Math.multiplyExact(ramLimitMiB, MIB);
    long rss = RunTelemetry.residentBytes();
    return new Limits(LinuxMemory.minimum(host.total(), limit),
        rss > 0 ? LinuxMemory.minimum(host.available(), Math.max(0, limit - rss)) : host.available(),
        host.source() + " + --ram-limit-mb");
  }

  record Snapshot(
      long heapUsed,
      long heapCommitted,
      long heapMaximum,
      long availableCommit,
      long availablePhysical,
      long regionBytes) {
    long allowance(long requested) {
      long heap = Math.max(0, heapMaximum - heapUsed) * 2 / 3;
      long commit =
          availableCommit < 0
              ? Long.MAX_VALUE
              : Math.max(
                      0,
                      availableCommit
                          - Math.max(512 * MIB, availableCommit / 8)
                          + Math.max(0, heapCommitted - heapUsed))
                  * 2
                  / 3;
      return Math.min(requested, Math.min(heap, commit));
    }
  }

  static Snapshot snapshot() {
    Runtime r = Runtime.getRuntime();
    long[] os = Windows.read();
    if (System.getProperty("os.name").toLowerCase(Locale.ROOT).contains("linux")) {
      Limits linux = limits();
      os = new long[] {linux.available(), linux.available()};
    }
    long physical = os[1];
    if (physical <= 0
        && ManagementFactory.getOperatingSystemMXBean()
            instanceof com.sun.management.OperatingSystemMXBean mx)
      physical = mx.getFreeMemorySize();
    return new Snapshot(
        r.totalMemory() - r.freeMemory(),
        r.totalMemory(),
        r.maxMemory(),
        os[0],
        physical,
        regionBytes());
  }

  static long regionBytes() {
    try {
      long value =
          Long.parseLong(
              ManagementFactory.getPlatformMXBean(HotSpotDiagnosticMXBean.class)
                  .getVMOption("G1HeapRegionSize")
                  .getValue());
      if (value > 0) return value;
    } catch (RuntimeException ignored) {
    }
    return 32 * MIB;
  }

  static long arrayBytes(long count, int itemBytes, long region) {
    long bytes = Math.addExact(24, Math.multiplyExact(count, itemBytes));
    bytes = (bytes + 7) / 8 * 8;
    return bytes >= region / 2 ? Math.multiplyExact((bytes + region - 1) / region, region) : bytes;
  }

  static boolean admits(long bytes, long requested) {
    return bytes <= snapshot().allowance(requested);
  }

  /** Conservative admission, not a replacement for the scheduler's hard RSS limit. */
  static void requireHeap(long bytes, Options options, String operation) {
    if (bytes < 0) throw new IllegalArgumentException(operation + ": allocation estimate overflow");
    Snapshot heap = snapshot();
    Limits limits = limits(options.ramLimitMb);
    long allowance = heapAllowance(heap, limits, options);
    // At stage boundaries the previous graph may be unreachable but not collected.
    // Retry admission once after collection; do not reject a fitting graph just because
    // a large -Xmx delayed GC. This is not the native Mat lifetime fix.
    if (bytes > allowance && bytes <= options.memoryMb * MIB
        && bytes <= heap.heapMaximum() * 3 / 4 && heap.heapUsed() > heap.heapMaximum() / 3) {
      System.gc();
      heap = snapshot(); limits = limits(options.ramLimitMb);
      allowance = heapAllowance(heap, limits, options);
    }
    if (bytes > allowance)
      throw new IllegalArgumentException(String.format(Locale.ROOT,
          "%s needs about %,d MiB additional heap/workspace; safe available allowance is %,d MiB "
              + "(heap used %,d MiB, heap maximum %,d MiB; %s). Reduce features or process a smaller block, "
              + "or request a larger job allocation. --memory-mb is not a total RAM limit.",
          operation, bytes / MIB + (bytes % MIB == 0 ? 0 : 1), allowance / MIB,
          heap.heapUsed() / MIB, heap.heapMaximum() / MIB, limits.source()));
  }

  private static long heapAllowance(Snapshot heap, Limits limits, Options options) {
    long bytes = Math.min(options.memoryMb * MIB, Math.max(0, heap.heapMaximum() - heap.heapUsed()) * 3 / 4);
    return limits.available() < 0 ? bytes : Math.min(bytes, limits.available() * 3 / 4);
  }

  static void validateHostHeap(Options options) {
    var limit = limits(options.ramLimitMb);
    long heap = Runtime.getRuntime().maxMemory();
    if (limit.total() > 0 && heap > limit.total() / 100 * 65 + regionBytes())
      throw new IllegalArgumentException(String.format(Locale.ROOT,
          "JVM maximum heap %,d MiB leaves insufficient native headroom under %,d MiB RAM (%s). "
              + "Restart through the photogrammetry launcher or set -Xmx to at most %,d MiB; "
              + "an embedded Main.run call cannot resize its host JVM.",
          heap / MIB, limit.total() / MIB, limit.source(), limit.total() / 100 * 65 / MIB));
  }

  private static final class Windows {
    static MethodHandle status;
    static boolean unavailable =
        !System.getProperty("os.name").toLowerCase(Locale.ROOT).contains("win");

    static synchronized long[] read() {
      if (unavailable) return new long[] {-1, -1};
      try (Arena arena = Arena.ofConfined()) {
        if (status == null)
          status =
              Linker.nativeLinker()
                  .downcallHandle(
                      SymbolLookup.libraryLookup("kernel32.dll", Arena.global())
                          .find("GlobalMemoryStatusEx")
                          .orElseThrow(),
                      FunctionDescriptor.of(JAVA_INT, ADDRESS));
        MemorySegment data = arena.allocate(64, 8);
        data.set(JAVA_INT, 0, 64);
        int ok = (int) status.invokeExact(data);
        return ok == 0
            ? new long[] {-1, -1}
            : new long[] {data.get(JAVA_LONG, 32), data.get(JAVA_LONG, 16)};
      } catch (Throwable e) {
        if (e instanceof VirtualMachineError error) throw error;
        unavailable = true;
        return new long[] {-1, -1};
      }
    }
  }
}
