package sgm;

import static java.lang.foreign.ValueLayout.*;

import java.lang.foreign.*;
import java.lang.invoke.*;
import java.lang.reflect.*;
import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicReference;

/** Exercise the real FFM calling convention with CPU upcalls; never opens an OpenCL device. */
public final class OpenClBindingRegression {
  private static int checks;
  private static Object[] expected;
  private static MemorySegment addressResult;
  private static final AtomicReference<Throwable> callbackFailure = new AtomicReference<>();
  private static CountDownLatch concurrent;

  private record Binding(String name, String signature) {}

  private static final Binding[] BINDINGS = {
    new Binding("clGetDeviceInfo", "IPILPP"),
    new Binding("clGetPlatformIDs", "IIPP"),
    new Binding("clGetDeviceIDs", "IPLIPP"),
    new Binding("clCreateContext", "PPIPPPP"),
    new Binding("clCreateCommandQueue", "PPPLP"),
    new Binding("clCreateBuffer", "PPLLPP"),
    new Binding("clSetKernelArg", "IPILP"),
    new Binding("clEnqueueNDRangeKernel", "IPPIPPPIPP"),
    new Binding("clCreateProgramWithSource", "PPIPPP"),
    new Binding("clBuildProgram", "IPIPPPP"),
    new Binding("clGetProgramBuildInfo", "IPPILPP"),
    new Binding("clCreateKernel", "PPPP"),
    new Binding("clGetKernelWorkGroupInfo", "IPPILPP"),
    new Binding("clGetEventProfilingInfo", "IPILPP"),
    new Binding("clFinish", "IP"),
    new Binding("clReleaseEvent", "IP"),
    new Binding("clReleaseKernel", "IP"),
    new Binding("clReleaseProgram", "IP"),
    new Binding("clReleaseCommandQueue", "IP"),
    new Binding("clReleaseContext", "IP"),
    new Binding("clReleaseMemObject", "IP"),
    new Binding("clRetainProgram", "IP"),
    new Binding("clEnqueueReadBuffer", "IPPILLPIPP"),
    new Binding("clEnqueueWriteBuffer", "IPPILLPIPP")
  };

  private static MemoryLayout layout(char code) {
    return switch (code) {
      case 'P' -> ADDRESS;
      case 'I' -> JAVA_INT;
      case 'L' -> JAVA_LONG;
      default -> throw new AssertionError(code);
    };
  }

  private static void check(boolean condition, String description) {
    checks++;
    if (!condition) throw new AssertionError(description);
  }

  private static void receive(Object[] values) {
    try {
      if (Thread.holdsLock(GpuSgm.class)) throw new AssertionError("Global lock held in native call");
      if (values.length != expected.length) throw new AssertionError("Argument count");
      for (int i = 0; i < values.length; i++) {
        boolean same = values[i] instanceof MemorySegment segment
            ? segment.address() == ((MemorySegment) expected[i]).address()
            : values[i].equals(expected[i]);
        if (!same) throw new AssertionError("Argument order/value at " + i);
      }
    } catch (Throwable failure) {
      callbackFailure.set(failure);
    }
  }

  public static int integerCallback(Object[] values) {
    receive(values);
    return 0x12345678;
  }

  public static MemorySegment addressCallback(Object[] values) {
    receive(values);
    return addressResult;
  }

  public static int overlappingCallback(MemorySegment ignored) {
    concurrent.countDown();
    try {
      return !Thread.holdsLock(GpuSgm.class) && concurrent.await(3, TimeUnit.SECONDS) ? 17 : -1;
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      return -2;
    }
  }

  @SuppressWarnings("unchecked")
  public static void main(String[] args) throws Throwable {
    Field field = GpuSgm.class.getDeclaredField("FUNCTIONS");
    field.setAccessible(true);
    Map<String, MethodHandle> functions = (Map<String, MethodHandle>) field.get(null);
    Map<String, MethodHandle> previous = new HashMap<>(functions);
    Method invoke = GpuSgm.class.getDeclaredMethod(
        "invoke", String.class, MemoryLayout.class, MemoryLayout[].class, Object[].class);
    invoke.setAccessible(true);
    MethodHandles.Lookup lookup = MethodHandles.lookup();
    Linker linker = Linker.nativeLinker();
    try (Arena arena = Arena.ofShared()) {
      addressResult = arena.allocate(16);
      for (Binding binding : BINDINGS) {
        MemoryLayout result = layout(binding.signature.charAt(0));
        MemoryLayout[] parameters = binding.signature.substring(1).chars()
            .mapToObj(c -> layout((char) c)).toArray(MemoryLayout[]::new);
        FunctionDescriptor descriptor = FunctionDescriptor.of(result, parameters);
        expected = new Object[parameters.length];
        for (int i = 0; i < parameters.length; i++) {
          if (parameters[i].equals(ADDRESS)) expected[i] = arena.allocate(8 + i);
          else if (parameters[i].equals(JAVA_LONG)) expected[i] = 0x1234567800000000L + i;
          else expected[i] = 0x13570000 + i;
        }
        boolean pointer = result.equals(ADDRESS);
        MethodHandle callback = lookup.findStatic(OpenClBindingRegression.class,
            pointer ? "addressCallback" : "integerCallback",
            MethodType.methodType(pointer ? MemorySegment.class : int.class, Object[].class));
        callback = callback.asCollector(Object[].class, parameters.length)
            .asType(descriptor.toMethodType());
        MemorySegment stub = linker.upcallStub(callback, descriptor, arena);
        functions.put(binding.name, linker.downcallHandle(stub, descriptor));
        Object actual = invoke.invoke(null, binding.name, result, parameters, expected);
        check(callbackFailure.get() == null, binding.name + ": " + callbackFailure.get());
        check(pointer ? ((MemorySegment) actual).address() == addressResult.address()
            : actual.equals(0x12345678), binding.name + " return value");
      }
      concurrent = new CountDownLatch(2);
      FunctionDescriptor descriptor = FunctionDescriptor.of(JAVA_INT, ADDRESS);
      MethodHandle callback = lookup.findStatic(OpenClBindingRegression.class, "overlappingCallback",
          MethodType.methodType(int.class, MemorySegment.class));
      functions.put("clFinish", linker.downcallHandle(linker.upcallStub(callback, descriptor, arena), descriptor));
      try (ExecutorService pool = Executors.newFixedThreadPool(2)) {
        Callable<Object> call = () -> invoke.invoke(null, "clFinish", JAVA_INT,
            new MemoryLayout[] {ADDRESS}, new Object[] {MemorySegment.NULL});
        Future<Object> a = pool.submit(call), b = pool.submit(call);
        check(a.get().equals(17) && b.get().equals(17), "Independent blocking native calls overlap");
      }
      try {
        invoke.invoke(null, "clFinish", JAVA_INT, new MemoryLayout[] {ADDRESS}, new Object[] {1});
        throw new AssertionError("Malformed binding should fail");
      } catch (InvocationTargetException e) {
        check(e.getCause() instanceof IllegalStateException
            && e.getCause().getMessage().startsWith("OpenCL clFinish:"), "Error wrapper retained");
      }
    } finally {
      functions.clear();
      functions.putAll(previous);
    }
    System.out.println("PASS CPU-only OpenCL FFM bindings: " + checks + " checks, 24 signatures");
  }
}
