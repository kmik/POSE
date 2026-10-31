package sgm;

import static java.lang.foreign.ValueLayout.*;

import java.io.*;
import java.lang.foreign.*;
import java.lang.invoke.*;
import java.lang.management.*;
import java.nio.file.*;
import java.time.*;
import java.util.*;
import java.util.concurrent.*;

/** Sampled process resources and monotonic task timings; no environment-variable collection. */
final class RunTelemetry implements AutoCloseable {
  private static final InheritableThreadLocal<RunTelemetry> CURRENT =
      new InheritableThreadLocal<>();
  private final long began = System.nanoTime();
  private final Path run;
  private final Options options;
  private final Map<String, Object> initial;
  private final Object selectedDeviceMetadata;
  private final List<Object> tasks = new ArrayList<>();
  private final Map<WorkProgress, String> active = new IdentityHashMap<>();
  private final ScheduledExecutorService sampler;
  private long peakHeap, peakCommitted, peakRss, peakPrivate, peakDevice, samples;
  private static final java.util.concurrent.atomic.AtomicLong DEVICE =
      new java.util.concurrent.atomic.AtomicLong();
  private boolean closed;
  private long lastSample, largestSamplingGap;

  RunTelemetry(Path run, Options options) {
    this.run = run;
    this.options = options;
    initial = parameters(options);
    Object deviceInfo = null;
    if (!options.compute.equals("cpu")
        || options.fusionCompute.equals("gpu")
        || options.fusionCompute.equals("auto")) {
      try {
        var d = GpuSgm.selectedDevice(options);
        deviceInfo =
            Json.object(
                "name",
                d.name(),
                "driver",
                d.driver(),
                "global_memory_bytes",
                d.memory(),
                "maximum_allocation_bytes",
                d.allocation(),
                "local_memory_bytes",
                d.localMemory(),
                "effective_working_budget_bytes",
                GpuSgm.budget(d, options),
                "scope",
                "Selected eligible device; actual execution/fallback is recorded in stage"
                    + " profiles");
      } catch (RuntimeException unavailable) {
        deviceInfo = Json.object("unavailable", unavailable.getMessage());
      }
    }
    selectedDeviceMetadata = deviceInfo;
    CURRENT.set(this);
    sampler =
        Executors.newSingleThreadScheduledExecutor(
            r -> {
              Thread t = new Thread(r, "processing-resources");
              t.setDaemon(true);
              return t;
            });
    sample();
    sampler.scheduleWithFixedDelay(this::sample, 1, 1, TimeUnit.SECONDS);
  }

  static RunTelemetry current() {
    return CURRENT.get();
  }

  static void deviceAllocated(long bytes) {
    long now = DEVICE.addAndGet(bytes);
    RunTelemetry t = current();
    if (t != null)
      synchronized (t) {
        t.peakDevice = Math.max(t.peakDevice, now);
      }
  }

  static void deviceFreed(long bytes) {
    DEVICE.addAndGet(-bytes);
  }

  synchronized void started(WorkProgress task, String label, long clock) {
    active.put(task, label);
  }

  synchronized void finished(WorkProgress task, String label, long clock, boolean success) {
    active.remove(task);
    tasks.add(
        Json.object(
            "task",
            label,
            "started_offset_seconds",
            (clock - began) / 1e9,
            "wall_seconds",
            (System.nanoTime() - clock) / 1e9,
            "status",
            success ? "complete" : "interrupted_or_failed",
            "thread",
            Thread.currentThread().getName()));
  }

  private synchronized void sample() {
    try {
      long now = System.nanoTime();
      if (lastSample > 0) largestSamplingGap = Math.max(largestSamplingGap, now - lastSample);
      lastSample = now;
      var mem = ManagementFactory.getMemoryMXBean().getHeapMemoryUsage();
      peakHeap = Math.max(peakHeap, mem.getUsed());
      peakCommitted = Math.max(peakCommitted, mem.getCommitted());
      long[] process = processMemory();
      peakRss = Math.max(peakRss, process[0]);
      peakPrivate = Math.max(peakPrivate, process[1]);
      peakDevice = Math.max(peakDevice, DEVICE.get());
      samples++;
    } catch (RuntimeException ignored) {
    }
  }

  static void record(String label, long started) {
    RunTelemetry t = current();
    if (t != null)
      synchronized (t) {
        t.tasks.add(
            Json.object(
                "task",
                label,
                "started_offset_seconds",
                (started - t.began) / 1e9,
                "wall_seconds",
                (System.nanoTime() - started) / 1e9,
                "status",
                "complete",
                "thread",
                Thread.currentThread().getName()));
      }
  }

  static Map<String, Object> parameters(Options o) {
    Map<String, Object> values = new TreeMap<>();
    try {
      for (var f : Options.class.getFields())
        if (!java.lang.reflect.Modifier.isStatic(f.getModifiers())) {
          Object v = f.get(o);
          if (v instanceof Path) v = v.toString();
          if (v instanceof Double d && !Double.isFinite(d))
            v = d.isNaN() ? "automatic/unset" : d.toString();
          values.put(f.getName(), v);
        }
    } catch (IllegalAccessException e) {
      throw new IllegalStateException(e);
    }
    values.put("effectiveImageCacheBytes", RasterCache.limit(o));
    values.put(
        "effectiveDescriptorCacheBytes",
        Math.min(o.descriptorMemoryMb * 1048576L, Math.max(1048576L, o.memoryMb * 1048576L / 4)));
    values.put("effectiveP1", StereoCost.penalty(o.p1, o));
    values.put("effectiveP2", StereoCost.p2(o));
    return values;
  }

  synchronized Map<String, Object> snapshot() {
    sample();
    return Json.object(
        "schema",
        "collinea-processing-telemetry-1",
        "processing_engine_sha256",
        ReuseCache.engineIdentifier(),
        "selected_opencl_device",
        selectedDeviceMetadata,
        "hardware",
        Json.object(
            "java_version",
            System.getProperty("java.version"),
            "os",
            System.getProperty("os.name"),
            "architecture",
            System.getProperty("os.arch"),
            "available_processors",
            Runtime.getRuntime().availableProcessors()),
        "elapsed_wall_seconds",
        (System.nanoTime() - began) / 1e9,
        "process_cpu_seconds_since_launch",
        ProcessHandle.current().info().totalCpuDuration().map(d -> d.toNanos() / 1e9).orElse(null),
        "largest_sampling_gap_seconds",
        largestSamplingGap / 1e9,
        "wall_time_note",
        "Wall times include scheduling/suspension gaps; sampling is best effort",
        "sample_period_seconds",
        1,
        "samples",
        samples,
        "peak_sampled_heap_used_bytes",
        peakHeap,
        "peak_sampled_heap_committed_bytes",
        peakCommitted,
        "maximum_heap_bytes",
        Runtime.getRuntime().maxMemory(),
        "peak_sampled_process_resident_bytes",
        peakRss == 0 ? null : peakRss,
        "peak_sampled_process_private_bytes",
        peakPrivate == 0 ? null : peakPrivate,
        "peak_tracked_opencl_buffer_bytes",
        peakDevice,
        "resource_scope",
        "Process resident/private samples where available; OpenCL tracks this program's allocated"
            + " buffers, not driver/display VRAM. Heap is included in process RAM; do not add them."
            + " Overlapping/nested task times are not additive.",
        "initial_parameters",
        initial,
        "effective_parameters",
        parameters(options),
        "tasks",
        new ArrayList<>(tasks),
        "active_tasks",
        new ArrayList<>(active.values()));
  }

  void persist() throws IOException {
    Json.write(run.resolve("logs/processing.json"), snapshot());
  }

  public void close() throws IOException {
    if (closed) return;
    closed = true;
    sampler.shutdownNow();
    sample();
    persist();
    CURRENT.remove();
  }

  private static long[] processMemory() {
    if (System.getProperty("os.name").toLowerCase(Locale.ROOT).contains("win"))
      return WindowsMemory.read();
    Path status = Path.of("/proc/self/status");
    long rss = 0;
    try {
      for (String line : Files.readAllLines(status))
        if (line.startsWith("VmRSS:")) rss = Long.parseLong(line.replaceAll("[^0-9]", "")) * 1024;
    } catch (IOException | NumberFormatException ignored) {
    }
    return new long[] {rss, 0};
  }

  private static final class WindowsMemory {
    static MethodHandle current, info;
    static boolean unavailable;

    static synchronized long[] read() {
      if (unavailable) return new long[2];
      try (Arena arena = Arena.ofConfined()) {
        if (current == null) {
          var linker = Linker.nativeLinker();
          var kernel = SymbolLookup.libraryLookup("kernel32.dll", Arena.global());
          var psapi = SymbolLookup.libraryLookup("psapi.dll", Arena.global());
          current =
              linker.downcallHandle(
                  kernel.find("GetCurrentProcess").orElseThrow(), FunctionDescriptor.of(ADDRESS));
          info =
              linker.downcallHandle(
                  psapi.find("GetProcessMemoryInfo").orElseThrow(),
                  FunctionDescriptor.of(JAVA_INT, ADDRESS, ADDRESS, JAVA_INT));
        }
        MemorySegment counters = arena.allocate(80, 8);
        counters.set(JAVA_INT, 0, 80);
        MemorySegment process = (MemorySegment) current.invokeExact();
        int ok = (int) info.invokeExact(process, counters, 80);
        return ok == 0
            ? new long[2]
            : new long[] {counters.get(JAVA_LONG, 16), counters.get(JAVA_LONG, 72)};
      } catch (Throwable failure) {
        if (failure instanceof VirtualMachineError e) throw e;
        unavailable = true;
        return new long[2];
      }
    }
  }
}
