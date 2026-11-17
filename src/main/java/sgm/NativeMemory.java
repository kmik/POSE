package sgm;

import static java.lang.foreign.ValueLayout.*;

import java.io.IOException;
import java.lang.foreign.*;
import java.lang.invoke.MethodHandle;
import java.nio.file.*;
import java.util.Locale;

/**
 * The process's own memory as the operating system counts it against RAM (MEMORY_GOVERNOR in
 * MEMORY_FIX_20261109.md), and handing freed native memory back to it.
 *
 * <p>OpenCV, SIFT pyramids, FLANN and the OpenCL driver allocate outside the Java heap through the
 * C library. glibc keeps freed blocks in its arenas for reuse, so after a stage with many worker
 * threads the process can hold tens of GB it no longer uses. {@link #trim} calls glibc's
 * {@code malloc_trim(0)}, which returns those free pages; it changes no result. Elsewhere (Windows,
 * macOS, musl) it does nothing.
 */
final class NativeMemory {
  private NativeMemory() {}

  private static final boolean LINUX =
      System.getProperty("os.name").toLowerCase(Locale.ROOT).contains("linux");
  private static MethodHandle mallocTrim;
  private static boolean trimUnavailable = !LINUX || Boolean.getBoolean("sgm.noMallocTrim");
  static long trims, trimmedBytes;

  /**
   * Bytes of this process that need RAM or swap: Linux RssAnon + RssShmem (not file pages the
   * kernel can drop), Windows private bytes; -1 when unknown.
   */
  static long committedBytes() {
    if (LINUX) {
      try {
        long anon = -1, shmem = 0;
        for (String line : Files.readAllLines(Path.of("/proc/self/status"))) {
          if (line.startsWith("RssAnon:")) anon = kib(line);
          else if (line.startsWith("RssShmem:")) shmem = kib(line);
        }
        return anon < 0 ? -1 : anon + shmem;
      } catch (IOException | RuntimeException e) {
        return -1;
      }
    }
    long[] windows = RunTelemetry.processMemoryBytes();
    return windows[1] > 0 ? windows[1] : -1;
  }

  private static long kib(String line) {
    return Long.parseLong(line.replaceAll("[^0-9]", "")) * 1024;
  }

  /**
   * glibc malloc_trim(0); returns the drop in committed bytes (may be 0), or -1 when not available.
   * Prints a line when at least 64 MiB came back.
   */
  static synchronized long trim(String when) {
    if (trimUnavailable) return -1;
    try {
      if (mallocTrim == null) {
        var symbol = Linker.nativeLinker().defaultLookup().find("malloc_trim");
        if (symbol.isEmpty()) {
          trimUnavailable = true;
          return -1;
        }
        mallocTrim =
            Linker.nativeLinker().downcallHandle(symbol.get(), FunctionDescriptor.of(JAVA_INT, JAVA_LONG));
      }
      long before = committedBytes();
      int released = (int) mallocTrim.invokeExact(0L);
      long after = committedBytes();
      trims++;
      long freed = before > 0 && after > 0 ? Math.max(0, before - after) : 0;
      trimmedBytes += freed;
      if (freed >= 64L << 20)
        System.out.printf(
            Locale.ROOT,
            "Native memory: %,d MiB returned to the system %s (process %,d -> %,d MiB)%n",
            freed >> 20,
            when,
            before >> 20,
            after >> 20);
      return released == 0 ? 0 : freed;
    } catch (Throwable e) {
      if (e instanceof VirtualMachineError error) throw error;
      trimUnavailable = true;
      return -1;
    }
  }
}
