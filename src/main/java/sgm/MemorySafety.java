package sgm;

import static java.lang.foreign.ValueLayout.*;

import com.sun.management.HotSpotDiagnosticMXBean;
import java.lang.foreign.*;
import java.lang.invoke.MethodHandle;
import java.lang.management.ManagementFactory;
import java.util.Locale;

/** Prevent native commit failure before allocating; catch(OOME) cannot recover from it. */
final class MemorySafety {
  static final long MIB = 1048576L;

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
