package sgm;

import java.util.function.IntConsumer;

/** Runs independent indices on a few platform threads in contiguous chunks (results by index). */
final class Parallel {
  private Parallel() {}

  static void forEach(int count, int workers, IntConsumer action) {
    int t = Math.max(1, Math.min(workers, count));
    if (t == 1) {
      for (int i = 0; i < count; i++) action.accept(i);
      return;
    }
    Thread[] pool = new Thread[t];
    Throwable[] failure = new Throwable[1];
    int chunk = (count + t - 1) / t;
    for (int k = 0; k < t; k++) {
      int from = k * chunk, to = Math.min(count, from + chunk);
      pool[k] =
          Thread.ofPlatform()
              .daemon()
              .name("parallel-" + k)
              .start(
                  () -> {
                    try {
                      for (int i = from; i < to; i++) action.accept(i);
                    } catch (Throwable e) {
                      synchronized (failure) {
                        if (failure[0] == null) failure[0] = e;
                      }
                    }
                  });
    }
    for (Thread thread : pool)
      try {
        thread.join();
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
        throw new IllegalStateException("Interrupted", e);
      }
    if (failure[0] instanceof RuntimeException r) throw r;
    if (failure[0] instanceof Error e) throw e;
    if (failure[0] != null) throw new IllegalStateException(failure[0]);
  }
}
