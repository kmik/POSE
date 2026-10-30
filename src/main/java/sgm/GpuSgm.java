package sgm;

import static java.lang.foreign.ValueLayout.*;

import java.lang.foreign.*;
import java.lang.invoke.MethodHandle;
import java.util.*;

/** OpenCL 1.2 via the JDK 25 FFM API. Only GPU devices are eligible. No bundled JNI/CUDA. */
final class GpuSgm implements AutoCloseable {
  record Device(
      MemorySegment id,
      String name,
      String driver,
      long memory,
      long allocation,
      long localMemory,
      long groupSize) {}

  record Local(long bytes) {}

  private static final ThreadLocal<GpuSgm> SESSION = new ThreadLocal<>();
  private static final ThreadLocal<Set<String>> WARNED = ThreadLocal.withInitial(HashSet::new);
  private static SymbolLookup library;
  private static final Map<String, MethodHandle> FUNCTIONS = new HashMap<>();
  final Device device;
  private MemorySegment context, queue;
  private final int group;

  private static synchronized Object invoke(
      String name, MemoryLayout result, MemoryLayout[] parameters, Object... arguments) {
    try {
      if (library == null) {
        String os = System.getProperty("os.name").toLowerCase(Locale.ROOT);
        if (ADDRESS.byteSize() != 8)
          throw new IllegalStateException("OpenCL backend requires a 64-bit JVM");
        library =
            SymbolLookup.libraryLookup(
                os.contains("win") ? "OpenCL.dll" : "libOpenCL.so.1", Arena.global());
      }
      MethodHandle f = FUNCTIONS.get(name);
      if (f == null) {
        f =
            Linker.nativeLinker()
                .downcallHandle(
                    library.find(name).orElseThrow(), FunctionDescriptor.of(result, parameters));
        FUNCTIONS.put(name, f);
      }
      return f.invokeWithArguments(arguments);
    } catch (Throwable e) {
      if (e instanceof VirtualMachineError error) throw error;
      throw new IllegalStateException("OpenCL " + name + ": " + e.getMessage(), e);
    }
  }

  private static void check(int result, String operation) {
    if (result != 0) throw new IllegalStateException(operation + " failed (OpenCL " + result + ")");
  }

  private static int info(MemorySegment device, int key, long size, MemorySegment out) {
    return (int)
        invoke(
            "clGetDeviceInfo",
            JAVA_INT,
            new MemoryLayout[] {ADDRESS, JAVA_INT, JAVA_LONG, ADDRESS, ADDRESS},
            device,
            key,
            size,
            out,
            MemorySegment.NULL);
  }

  private static String textInfo(MemorySegment device, int key, Arena arena) {
    MemorySegment out = arena.allocate(4096);
    check(info(device, key, out.byteSize(), out), "Device information");
    return out.getString(0);
  }

  static List<Device> devices() {
    try (Arena arena = Arena.ofConfined()) {
      MemorySegment platforms = arena.allocate(32 * ADDRESS.byteSize()),
          count = arena.allocate(JAVA_INT);
      int status =
          (int)
              invoke(
                  "clGetPlatformIDs",
                  JAVA_INT,
                  new MemoryLayout[] {JAVA_INT, ADDRESS, ADDRESS},
                  32,
                  platforms,
                  count);
      if (status == -1001) return List.of();
      check(status, "Platform enumeration");
      List<Device> result = new ArrayList<>();
      for (int i = 0; i < count.get(JAVA_INT, 0); i++) {
        MemorySegment ids = arena.allocate(32 * ADDRESS.byteSize()),
            number = arena.allocate(JAVA_INT);
        status =
            (int)
                invoke(
                    "clGetDeviceIDs",
                    JAVA_INT,
                    new MemoryLayout[] {ADDRESS, JAVA_LONG, JAVA_INT, ADDRESS, ADDRESS},
                    platforms.getAtIndex(ADDRESS, i),
                    4L,
                    32,
                    ids,
                    number);
        if (status == -1) continue;
        check(status, "GPU enumeration");
        for (int j = 0; j < number.get(JAVA_INT, 0); j++) {
          MemorySegment id = ids.getAtIndex(ADDRESS, j), value = arena.allocate(JAVA_LONG);
          long[] limits = new long[4];
          int[] keys = {0x101F, 0x1010, 0x1023, 0x1004};
          for (int k = 0; k < keys.length; k++) {
            check(info(id, keys[k], 8, value), "GPU limits");
            limits[k] = value.get(JAVA_LONG, 0);
          }
          result.add(
              new Device(
                  id,
                  textInfo(id, 0x102B, arena),
                  textInfo(id, 0x102D, arena),
                  limits[0],
                  limits[1],
                  limits[2],
                  limits[3]));
        }
      }
      return result;
    }
  }

  static void list() {
    try {
      List<Device> devices = devices();
      if (devices.isEmpty()) System.out.println("No OpenCL GPU devices found.");
      for (int i = 0; i < devices.size(); i++) {
        Device d = devices.get(i);
        System.out.printf(
            Locale.ROOT,
            "%d: %s; VRAM %.0f MiB; max allocation %.0f MiB; driver %s%n",
            i,
            d.name,
            d.memory / 1048576.,
            d.allocation / 1048576.,
            d.driver);
      }
    } catch (RuntimeException e) {
      System.out.println("OpenCL GPU unavailable: " + e.getMessage());
    }
  }

  private static GpuSgm session(Options o) {
    GpuSgm current = SESSION.get();
    List<Device> devices;
    if (current != null && current.index == o.gpuDevice) return current;
    if (current != null) closeSession();
    devices = devices();
    if (o.gpuDevice >= devices.size())
      throw new IllegalStateException(
          "No OpenCL GPU at index " + o.gpuDevice + "; use --list-gpus");
    current = new GpuSgm(devices.get(o.gpuDevice), o.gpuDevice);
    SESSION.set(current);
    System.out.println(
        "GPU SGM: "
            + current.device.name
            + "; OpenCL integer costs and global "
            + "4/8-path aggregation; CPU selection/filtering");
    return current;
  }

  final int index;

  private GpuSgm(Device device, int index) {
    this.device = device;
    this.index = index;
    int size = 1;
    while (size * 2 <= Math.min(128, device.groupSize)) size *= 2;
    group = size;
    try (Arena a = Arena.ofConfined()) {
      MemorySegment err = a.allocate(JAVA_INT), id = a.allocate(ADDRESS);
      id.set(ADDRESS, 0, device.id);
      context =
          (MemorySegment)
              invoke(
                  "clCreateContext",
                  ADDRESS,
                  new MemoryLayout[] {ADDRESS, JAVA_INT, ADDRESS, ADDRESS, ADDRESS, ADDRESS},
                  MemorySegment.NULL,
                  1,
                  id,
                  MemorySegment.NULL,
                  MemorySegment.NULL,
                  err);
      check(err.get(JAVA_INT, 0), "Context creation");
      queue =
          (MemorySegment)
              invoke(
                  "clCreateCommandQueue",
                  ADDRESS,
                  new MemoryLayout[] {ADDRESS, ADDRESS, JAVA_LONG, ADDRESS},
                  context,
                  device.id,
                  2L,
                  err);
      check(err.get(JAVA_INT, 0), "Queue creation");
    } catch (RuntimeException e) {
      close();
      throw e;
    }
  }

  static long memoryEstimate(int w, int h, int disparities, int words) {
    long n = (long) w * h;
    return 6 * n * disparities + n * (16L * words + 28);
  }

  static long memoryEstimate(int w, int h, int d, Options o) {
    return (long) (2 + StereoCost.aggregateBytes(o)) * w * h * d
        + (long) w * h * (16L * ((o.census * o.census - 1 + 63) / 64) + 28);
  }

  private int limit(int w, int h, Options o) {
    return GpuStereo.limit(device, group, w, h, o);
  }

  static int disparityLimit(int w, int h, Options o) {
    if (o.compute.equals("cpu")) return 0;
    try {
      return session(o).limit(w, h, o);
    } catch (RuntimeException e) {
      failure(o, e);
      return 0;
    }
  }

  static boolean use(int w, int h, Options o) {
    if (o.compute.equals("cpu")
        || o.compute.equals("auto") && (long) w * h * o.disparities < 1000000) return false;
    int limit = disparityLimit(w, h, o);
    if (o.disparities <= limit) return true;
    if (o.compute.equals("gpu"))
      throw new IllegalArgumentException(
          "GPU SGM search exceeds VRAM/allocation/local-memory budget; limit="
              + limit
              + " disparities. Use smaller dense tiles/resolution or --compute auto/cpu.");
    return false;
  }

  private static void failure(Options o, RuntimeException e) {
    if (o.compute.equals("gpu"))
      throw new IllegalArgumentException("GPU execution required: " + e.getMessage(), e);
    if (WARNED.get().add(e.getMessage()))
      System.out.println("GPU unavailable; using CPU: " + e.getMessage());
  }

  static float[] match(
      Options o,
      long[][] a,
      long[][] b,
      byte[] grayA,
      byte[] grayB,
      boolean[] ta,
      boolean[] tb,
      int w,
      int h,
      boolean reverse,
      CensusSgm.Stats stats,
      byte[] reasons,
      byte[] searchBoundary) {
    if (!use(w, h, o)) return null;
    try {
      CensusSgm.Stats completed = new CensusSgm.Stats();
      byte[] codes = reasons == null ? null : reasons.clone(),
          boundary = searchBoundary == null ? null : searchBoundary.clone();
      float[] result =
          GpuStereo.execute(
              o, a, b, grayA, grayB, ta, tb, w, h, reverse, completed, codes, boundary);
      stats.lowerBoundaryRejected += completed.lowerBoundaryRejected;
      stats.fullIntervalPixels += completed.fullIntervalPixels;
      stats.boundaryRejected += completed.boundaryRejected;
      stats.upperBoundaryRejected += completed.upperBoundaryRejected;
      stats.ambiguityRejected += completed.ambiguityRejected;
      stats.selected += completed.selected;
      if (codes != null) System.arraycopy(codes, 0, reasons, 0, codes.length);
      if (boundary != null) System.arraycopy(boundary, 0, searchBoundary, 0, boundary.length);
      return result;
    } catch (RuntimeException e) {
      closeSession();
      failure(o, e);
      return null;
    }
  }

  private MemorySegment buffer(
      long size, MemorySegment data, Arena a, List<MemorySegment> buffers) {
    MemorySegment err = a.allocate(JAVA_INT);
    MemorySegment result =
        (MemorySegment)
            invoke(
                "clCreateBuffer",
                ADDRESS,
                new MemoryLayout[] {ADDRESS, JAVA_LONG, JAVA_LONG, ADDRESS, ADDRESS},
                context,
                data.equals(MemorySegment.NULL) ? 1L : 36L,
                size,
                data,
                err);
    check(err.get(JAVA_INT, 0), "GPU buffer allocation (" + size / 1048576 + " MiB)");
    buffers.add(result);
    return result;
  }

  private void args(MemorySegment kernel, Arena a, Object... values) {
    for (int i = 0; i < values.length; i++) {
      long size;
      MemorySegment value;
      if (values[i] instanceof Local local) {
        size = local.bytes;
        value = MemorySegment.NULL;
      } else if (values[i] instanceof MemorySegment segment) {
        size = 8;
        value = a.allocate(ADDRESS);
        value.set(ADDRESS, 0, segment);
      } else {
        size = 4;
        value = a.allocate(JAVA_INT);
        value.set(JAVA_INT, 0, (Integer) values[i]);
      }
      check(
          (int)
              invoke(
                  "clSetKernelArg",
                  JAVA_INT,
                  new MemoryLayout[] {ADDRESS, JAVA_INT, JAVA_LONG, ADDRESS},
                  kernel,
                  i,
                  size,
                  value),
          "Kernel argument");
    }
  }

  private void launch(MemorySegment kernel, long count, Arena a) {
    launch(kernel, count, a, MemorySegment.NULL);
  }

  private void launch(MemorySegment kernel, long count, Arena a, MemorySegment event) {
    MemorySegment global = a.allocate(JAVA_LONG), local = a.allocate(JAVA_LONG);
    global.set(JAVA_LONG, 0, ((count + group - 1) / group) * group);
    local.set(JAVA_LONG, 0, (long) group);
    check(
        (int)
            invoke(
                "clEnqueueNDRangeKernel",
                JAVA_INT,
                new MemoryLayout[] {
                  ADDRESS, ADDRESS, JAVA_INT, ADDRESS, ADDRESS, ADDRESS, JAVA_INT, ADDRESS, ADDRESS
                },
                queue,
                kernel,
                1,
                MemorySegment.NULL,
                global,
                local,
                0,
                MemorySegment.NULL,
                event),
        "Kernel dispatch");
  }

  private static void release(String function, MemorySegment object) {
    if (object != null && object.address() != 0)
      invoke(function, JAVA_INT, new MemoryLayout[] {ADDRESS}, object);
  }

  public void close() {
    release("clReleaseCommandQueue", queue);
    release("clReleaseContext", context);
  }

  static void closeSession() {
    GpuSgm session = SESSION.get();
    SESSION.remove();
    if (session != null) session.close();
    WARNED.remove();
  }

  /** A bounded auxiliary program sharing the selected device/context and its in-order queue. */
  static final class Compute implements AutoCloseable {
    final GpuSgm owner;
    final Device device;

    int group() {
      return owner.group;
    }

    private MemorySegment program;
    private final List<MemorySegment> buffers = new ArrayList<>(), kernels = new ArrayList<>();
    private final Map<MemorySegment, String> kernelNames = new HashMap<>();

    private record Event(MemorySegment id, String kernel) {}

    private final List<Event> events = new ArrayList<>();
    private final Map<String, Long> deviceNanos = new LinkedHashMap<>();
    private long writeNanos, readNanos, writeBytes, readBytes;

    Compute(Options options, String source) {
      owner = session(options);
      device = owner.device;
      try (Arena arena = Arena.ofConfined()) {
        MemorySegment err = arena.allocate(JAVA_INT), sources = arena.allocate(ADDRESS);
        sources.set(ADDRESS, 0, arena.allocateFrom(source));
        program =
            (MemorySegment)
                invoke(
                    "clCreateProgramWithSource",
                    ADDRESS,
                    new MemoryLayout[] {ADDRESS, JAVA_INT, ADDRESS, ADDRESS, ADDRESS},
                    owner.context,
                    1,
                    sources,
                    MemorySegment.NULL,
                    err);
        check(err.get(JAVA_INT, 0), "OpenCL program creation");
        int status =
            (int)
                invoke(
                    "clBuildProgram",
                    JAVA_INT,
                    new MemoryLayout[] {ADDRESS, JAVA_INT, ADDRESS, ADDRESS, ADDRESS, ADDRESS},
                    program,
                    0,
                    MemorySegment.NULL,
                    arena.allocateFrom("-cl-std=CL1.2"),
                    MemorySegment.NULL,
                    MemorySegment.NULL);
        if (status != 0) {
          MemorySegment log = arena.allocate(65536);
          invoke(
              "clGetProgramBuildInfo",
              JAVA_INT,
              new MemoryLayout[] {ADDRESS, ADDRESS, JAVA_INT, JAVA_LONG, ADDRESS, ADDRESS},
              program,
              device.id,
              0x1183,
              log.byteSize(),
              log,
              MemorySegment.NULL);
          throw new IllegalStateException(
              "OpenCL kernel build failed (fusion/ZNCC require FP64): " + log.getString(0));
        }
      } catch (RuntimeException e) {
        close();
        throw e;
      }
    }

    MemorySegment kernel(String name) {
      try (Arena a = Arena.ofConfined()) {
        MemorySegment err = a.allocate(JAVA_INT);
        MemorySegment k =
            (MemorySegment)
                invoke(
                    "clCreateKernel",
                    ADDRESS,
                    new MemoryLayout[] {ADDRESS, ADDRESS, ADDRESS},
                    program,
                    a.allocateFrom(name),
                    err);
        check(err.get(JAVA_INT, 0), "OpenCL kernel creation");
        kernels.add(k);
        MemorySegment limit = a.allocate(JAVA_LONG);
        check(
            (int)
                invoke(
                    "clGetKernelWorkGroupInfo",
                    JAVA_INT,
                    new MemoryLayout[] {ADDRESS, ADDRESS, JAVA_INT, JAVA_LONG, ADDRESS, ADDRESS},
                    k,
                    device.id,
                    0x11B0,
                    8L,
                    limit,
                    MemorySegment.NULL),
            "OpenCL kernel group limit");
        if (owner.group > limit.get(JAVA_LONG, 0))
          throw new IllegalStateException("Selected OpenCL kernel work-group limit is too small");
        kernelNames.put(k, name);
        return k;
      }
    }

    MemorySegment buffer(long bytes) {
      if (bytes <= 0 || bytes > device.allocation)
        throw new IllegalArgumentException("OpenCL allocation exceeds device limit: " + bytes);
      try (Arena a = Arena.ofConfined()) {
        return owner.buffer(bytes, MemorySegment.NULL, a, buffers);
      }
    }

    synchronized void transfer(
        MemorySegment buffer, long offset, MemorySegment data, boolean read) {
      long started = System.nanoTime();
      check(
          (int)
              invoke(
                  read ? "clEnqueueReadBuffer" : "clEnqueueWriteBuffer",
                  JAVA_INT,
                  new MemoryLayout[] {
                    ADDRESS, ADDRESS, JAVA_INT, JAVA_LONG, JAVA_LONG, ADDRESS, JAVA_INT, ADDRESS,
                    ADDRESS
                  },
                  owner.queue,
                  buffer,
                  1,
                  offset,
                  data.byteSize(),
                  data,
                  0,
                  MemorySegment.NULL,
                  MemorySegment.NULL),
          "OpenCL buffer transfer");
      if (read) {
        readNanos += System.nanoTime() - started;
        readBytes += data.byteSize();
        collect();
      } else {
        writeNanos += System.nanoTime() - started;
        writeBytes += data.byteSize();
      }
    }

    synchronized void run(MemorySegment kernel, long count, Object... arguments) {
      try (Arena a = Arena.ofConfined()) {
        owner.args(kernel, a, arguments);
        MemorySegment event = a.allocate(ADDRESS);
        owner.launch(kernel, count, a, event);
        events.add(new Event(event.get(ADDRESS, 0), kernelNames.get(kernel)));
      }
    }

    private void collect() {
      try (Arena a = Arena.ofConfined()) {
        MemorySegment start = a.allocate(JAVA_LONG), end = a.allocate(JAVA_LONG);
        for (Event e : events) {
          int result =
              (int)
                  invoke(
                      "clGetEventProfilingInfo",
                      JAVA_INT,
                      new MemoryLayout[] {ADDRESS, JAVA_INT, JAVA_LONG, ADDRESS, ADDRESS},
                      e.id,
                      0x1282,
                      8L,
                      start,
                      MemorySegment.NULL);
          int result2 =
              (int)
                  invoke(
                      "clGetEventProfilingInfo",
                      JAVA_INT,
                      new MemoryLayout[] {ADDRESS, JAVA_INT, JAVA_LONG, ADDRESS, ADDRESS},
                      e.id,
                      0x1283,
                      8L,
                      end,
                      MemorySegment.NULL);
          if (result == 0 && result2 == 0)
            deviceNanos.merge(e.kernel, end.get(JAVA_LONG, 0) - start.get(JAVA_LONG, 0), Long::sum);
          release("clReleaseEvent", e.id);
        }
        events.clear();
      }
    }

    synchronized Object profile() {
      return Json.object(
          "device_kernel_nanoseconds",
          new LinkedHashMap<>(deviceNanos),
          "blocking_write_nanoseconds",
          writeNanos,
          "blocking_read_nanoseconds",
          readNanos,
          "uploaded_bytes",
          writeBytes,
          "downloaded_bytes",
          readBytes);
    }

    public synchronized void close() {
      if (owner != null && owner.queue != null)
        invoke("clFinish", JAVA_INT, new MemoryLayout[] {ADDRESS}, owner.queue);
      collect();
      for (MemorySegment k : kernels) release("clReleaseKernel", k);
      kernels.clear();
      for (MemorySegment b : buffers) release("clReleaseMemObject", b);
      buffers.clear();
      release("clReleaseProgram", program);
      program = null;
    }
  }

  static Device selectedDevice(Options o) {
    return session(o).device;
  }

  static int selectedGroup(Options o) {
    return session(o).group;
  }

  static long budget(Device d, Options o) {
    return o.gpuMemoryMb == 0
        ? d.memory * 7 / 10
        : Math.min(d.memory * 95 / 100, Math.multiplyExact(o.gpuMemoryMb, 1048576L));
  }
}
