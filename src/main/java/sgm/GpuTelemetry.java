package sgm;

import static java.lang.foreign.ValueLayout.*;

import java.lang.foreign.*;
import java.lang.invoke.MethodHandle;
import java.util.*;

/** Optional read-only NVML measurements. Values describe the selected device, not this process. */
final class GpuTelemetry {
  record Sample(double utilization, long usedBytes, long totalBytes) {}

  private static SymbolLookup library;
  private static boolean attempted;
  private static final Map<String, MethodHandle> functions = new HashMap<>();
  private final MemorySegment device;

  private GpuTelemetry(MemorySegment device) {
    this.device = device;
  }

  static synchronized GpuTelemetry open(String selectedName) {
    try {
      if (!attempted) {
        attempted = true;
        library =
            SymbolLookup.libraryLookup(
                System.getProperty("os.name").toLowerCase(Locale.ROOT).contains("win")
                    ? "nvml.dll"
                    : "libnvidia-ml.so.1",
                Arena.global());
        if (call("nvmlInit_v2", new MemoryLayout[0]) != 0) library = null;
      }
      if (library == null) return null;
      try (Arena arena = Arena.ofConfined()) {
        MemorySegment count = arena.allocate(JAVA_INT),
            handle = arena.allocate(ADDRESS),
            name = arena.allocate(256);
        if (call("nvmlDeviceGetCount_v2", new MemoryLayout[] {ADDRESS}, count) != 0) return null;
        for (int i = 0; i < count.get(JAVA_INT, 0); i++) {
          if (call(
                  "nvmlDeviceGetHandleByIndex_v2",
                  new MemoryLayout[] {JAVA_INT, ADDRESS},
                  i,
                  handle)
              != 0) continue;
          MemorySegment device = handle.get(ADDRESS, 0);
          if (call(
                      "nvmlDeviceGetName",
                      new MemoryLayout[] {ADDRESS, ADDRESS, JAVA_INT},
                      device,
                      name,
                      256)
                  == 0
              && name.getString(0).equalsIgnoreCase(selectedName)) return new GpuTelemetry(device);
        }
      }
    } catch (RuntimeException | UnsatisfiedLinkError missing) {
      library = null;
    }
    return null;
  }

  Sample sample() {
    try (Arena arena = Arena.ofConfined()) {
      MemorySegment utilization = arena.allocate(8), memory = arena.allocate(24);
      if (call(
                  "nvmlDeviceGetUtilizationRates",
                  new MemoryLayout[] {ADDRESS, ADDRESS},
                  device,
                  utilization)
              != 0
          || call("nvmlDeviceGetMemoryInfo", new MemoryLayout[] {ADDRESS, ADDRESS}, device, memory)
              != 0) return null;
      return new Sample(
          utilization.get(JAVA_INT, 0), memory.get(JAVA_LONG, 16), memory.get(JAVA_LONG, 0));
    } catch (RuntimeException unavailable) {
      return null;
    }
  }

  private static synchronized int call(String name, MemoryLayout[] signature, Object... args) {
    try {
      MethodHandle function = functions.get(name);
      if (function == null) {
        function =
            Linker.nativeLinker()
                .downcallHandle(
                    library.find(name).orElseThrow(), FunctionDescriptor.of(JAVA_INT, signature));
        functions.put(name, function);
      }
      return (int) function.invokeWithArguments(args);
    } catch (Throwable failure) {
      if (failure instanceof VirtualMachineError fatal) throw fatal;
      throw new IllegalStateException("NVML " + name + ": " + failure.getMessage(), failure);
    }
  }
}
