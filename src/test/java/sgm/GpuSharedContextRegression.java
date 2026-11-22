package sgm;

import static java.lang.foreign.ValueLayout.JAVA_INT;

import java.lang.foreign.Arena;
import java.lang.foreign.MemorySegment;
import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.*;

/**
 * One OpenCL context per device shared by all threads, each with its own queue (GpuSgm.Shared):
 * eight threads run kernels at the same time on one context with correct results, a dedicated
 * compute joins the same context, and the context is released after the last session closes.
 * Skipped without an OpenCL GPU.
 */
final class GpuSharedContextRegression {
  private static int checks;

  private static void check(boolean value, String why) {
    checks++;
    if (!value) throw new AssertionError(why);
  }

  public static void main(String[] args) throws Exception {
    System.out.println("PASS GPU shared context: " + run());
  }

  static final String SOURCE =
      "__kernel void fill(__global int* out, int base, int n) {"
          + " int i = get_global_id(0); if (i < n) out[i] = base + i * 3; }";

  static int run() throws Exception {
    checks = 0;
    if (GpuSgm.devices().isEmpty()) {
      System.out.println("GPU shared context: no OpenCL GPU; skipped");
      return 0;
    }
    Options o = new Options();
    // Earlier work in this process (a SelfTest suite) may still hold sessions: measure from here.
    GpuSgm.closeSession();
    int before = GpuSgm.contexts();
    check(before <= 1, "at most one context per device before this test: " + before);
    int threads = 8, rounds = 12, n = 1 << 16;
    CountDownLatch open = new CountDownLatch(threads), observed = new CountDownLatch(1);
    AtomicInteger wrong = new AtomicInteger(), peak = new AtomicInteger();
    ExecutorService pool = Executors.newFixedThreadPool(threads);
    List<Future<?>> done = new ArrayList<>();
    for (int t = 0; t < threads; t++) {
      int thread = t;
      done.add(pool.submit(() -> {
        try {
          for (int r = 0; r < rounds; r++)
            try (GpuSgm.Compute cl = new GpuSgm.Compute(o, SOURCE); Arena arena = Arena.ofConfined()) {
              if (r == 0) {
                // Every thread holds a session before the context count is read.
                open.countDown();
                open.await();
                peak.accumulateAndGet(GpuSgm.contexts(), Math::max);
                if (thread == 0) observed.countDown();
              }
              MemorySegment buffer = cl.buffer(4L * n), kernel = cl.kernel("fill");
              int base = thread * 1_000_000 + r * 1000;
              cl.run(kernel, n, buffer, base, n);
              MemorySegment host = arena.allocate(4L * n, 4);
              cl.transfer(buffer, 0, host, true);
              for (int i = 0; i < n; i++) if (host.getAtIndex(JAVA_INT, i) != base + i * 3) { wrong.incrementAndGet(); break; }
            }
          return null;
        } finally {
          GpuSgm.closeSession();
        }
      }));
    }
    observed.await(2, TimeUnit.MINUTES);
    // A dedicated compute (own queue) joins the same context.
    try (GpuSgm.Compute dedicated = new GpuSgm.Compute(o, SOURCE, true)) {
      peak.accumulateAndGet(GpuSgm.contexts(), Math::max);
    }
    for (Future<?> f : done) f.get(5, TimeUnit.MINUTES);
    pool.shutdown();
    check(peak.get() == 1, "one OpenCL context for " + threads + " threads and a dedicated compute: " + peak.get());
    check(wrong.get() == 0, "kernels on the shared context computed correctly in every thread: " + wrong.get() + " wrong");
    check(GpuSgm.contexts() == before, "context released after the last session: " + GpuSgm.contexts() + " (before " + before + ")");
    // A new session after that creates a fresh context.
    try (GpuSgm.Compute again = new GpuSgm.Compute(o, SOURCE)) {
      check(GpuSgm.contexts() == 1, "a later session opens (or joins) the one context");
    } finally {
      GpuSgm.closeSession();
    }
    check(GpuSgm.contexts() == before, "and leaves the count as it was");
    return checks;
  }
}
