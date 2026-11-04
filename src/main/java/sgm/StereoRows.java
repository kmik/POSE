package sgm;

import java.util.ArrayList;
import java.util.concurrent.*;
import java.util.function.IntConsumer;

/** Bounded phase-owned row workers; nested calls stay serial to avoid oversubscription. */
final class StereoRows {
  private static final ThreadLocal<Scope> CURRENT = new ThreadLocal<>();
  static Scope open(int cores) { return new Scope(cores); }
  static final class Scope implements AutoCloseable {
    final Scope previous = CURRENT.get();
    final int cores;
    final ExecutorService workers;
    Scope(int requested) {
      cores = previous == null ? Math.max(1, requested) : previous.cores;
      workers = previous != null ? previous.workers : cores > 1 ? Executors.newFixedThreadPool(cores, r -> {
        Thread t = new Thread(r, "stereo-rows"); t.setDaemon(true); return t;
      }) : null;
      CURRENT.set(this);
    }
    public void close() {
      if (previous == null) CURRENT.remove(); else CURRENT.set(previous);
      if (previous == null && workers != null) workers.close();
    }
  }
  static void rows(int from, int to, IntConsumer action) {
    Scope scope = CURRENT.get();
    if (scope == null || scope.workers == null || to - from < 32) {
      for (int y = from; y < to; y++) action.accept(y);
      return;
    }
    var tasks = new ArrayList<Future<?>>();
    int chunk = Math.max(1, (to - from + scope.cores - 1) / scope.cores);
    for (int start = from; start < to; start += chunk) {
      int a = start, b = Math.min(to, start + chunk);
      tasks.add(scope.workers.submit(() -> { for (int y = a; y < b; y++) action.accept(y); }));
    }
    RuntimeException failure = null;
    for (var task : tasks) {
      try { task.get(); }
      catch (InterruptedException e) { Thread.currentThread().interrupt(); failure = new IllegalStateException("Stereo interrupted", e); }
      catch (ExecutionException e) {
        if (e.getCause() instanceof Error error) throw error;
        failure = e.getCause() instanceof RuntimeException r ? r : new IllegalStateException(e.getCause());
      }
    }
    if (failure != null) throw failure;
  }
}
