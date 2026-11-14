package sgm;

import java.io.IOException;
import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicLong;

/**
 * {@code --mvsgm-cameras-in-flight}: the raw depth maps of several cameras computed at once.
 *
 * <p>A camera's raw map depends only on its own plan (source images, sparse points, options), never
 * on another camera's result, so the order and the number of cameras in flight cannot change any
 * map. The consumer (the overlap window or the phase-1 loop) still takes the maps one by one with
 * {@link #await}, in its unchanged order; everything after a raw map (consistency, fusion) runs on
 * the consumer thread as before.
 *
 * <p>At most {@code inFlight} cameras are started and not yet taken (running, or finished and
 * waiting), so heap and temporary disk ahead of the consumer stay bounded. Cameras are started in
 * the predicted order of the consumer's requests; a request outside that order starts its camera
 * at once (one camera beyond the bound at most). GPU sessions of the workers share the installed {@link GpuMemoryPool} and wait there
 * for device memory. If a worker runs out of heap, the workers are drained and the remaining
 * cameras are computed one at a time on the consumer thread (same results).
 */
final class MvsgmCameraPool implements AutoCloseable {
  /** Prepares and solves one camera on the calling thread; returns its report entry. */
  interface Task {
    Object compute(int camera) throws IOException;
  }

  final int inFlight, rowCores;
  private final Task task;
  private final int[] order;
  private final ExecutorService workers;
  private final Map<Integer, Future<Object>> started = new HashMap<>();
  private final Set<Integer> taken = new HashSet<>();
  private final AtomicLong workerNanos = new AtomicLong();
  private int cursor, outstanding, peakOutstanding, outOfOrder, serialCameras;
  private long blockedNanos;
  private boolean serial, closed;

  MvsgmCameraPool(int inFlight, int cores, int[] order, Task task) {
    this.inFlight = Math.max(1, inFlight);
    this.rowCores = Math.max(1, cores / this.inFlight);
    this.order = order.clone();
    this.task = task;
    workers = Executors.newFixedThreadPool(this.inFlight, r -> {
      Thread t = new Thread(r, "mvsgm-camera");
      t.setDaemon(true);
      return t;
    });
  }

  private Object run(int camera) throws IOException {
    long clock = System.nanoTime();
    // Row loops (census, medians, band derivation) are row-parallel inside a scope only; rows are
    // independent, so the number of row workers never changes a result.
    try (var rows = StereoRows.open(rowCores)) {
      return task.compute(camera);
    } finally {
      workerNanos.addAndGet(System.nanoTime() - clock);
    }
  }

  private Future<Object> start(int camera) {
    Future<Object> future = workers.submit(() -> run(camera));
    started.put(camera, future);
    peakOutstanding = Math.max(peakOutstanding, ++outstanding);
    return future;
  }

  private void pump() {
    while (!serial && outstanding < inFlight && cursor < order.length) {
      int camera = order[cursor++];
      if (!started.containsKey(camera) && !taken.contains(camera)) start(camera);
    }
  }

  /** The camera's report once its raw maps are complete. Consumer thread only; once per camera. */
  Object await(int camera) throws IOException {
    if (closed) throw new IllegalStateException("mvsgm camera pool is closed");
    if (taken.contains(camera)) throw new IllegalStateException("mvsgm camera " + camera + " was taken twice");
    long clock = System.nanoTime();
    try {
      pump();
      Future<Object> future = started.get(camera);
      if (future == null && !serial) {
        // Not among the predicted next cameras: start it now, one beyond the bound at most.
        outOfOrder++;
        future = start(camera);
      }
      taken.add(camera);
      Object result;
      if (future == null) result = alone(camera);
      else {
        try {
          result = future.get();
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
          throw new IOException("mvsgm interrupted", e);
        } catch (ExecutionException e) {
          Throwable cause = e.getCause();
          if (!outOfMemory(cause)) {
            if (cause instanceof IOException io) throw io;
            if (cause instanceof RuntimeException r) throw r;
            if (cause instanceof Error error) throw error;
            throw new IOException(cause);
          }
          drain(camera);
          result = alone(camera);
        } finally {
          started.remove(camera);
          outstanding--;
        }
      }
      pump();
      return result;
    } finally {
      blockedNanos += System.nanoTime() - clock;
    }
  }

  private Object alone(int camera) throws IOException {
    serialCameras++;
    return run(camera);
  }

  /** After a worker ran out of heap: no new starts; cameras that failed the same way start over. */
  private void drain(int failed) throws IOException {
    serial = true;
    System.out.println("mvsgm: camera " + failed + " ran out of heap with " + inFlight
        + " cameras in flight; the remaining cameras are computed one at a time (same results)");
    for (var it = started.entrySet().iterator(); it.hasNext();) {
      var entry = it.next();
      if (entry.getKey() == failed) continue;
      try {
        entry.getValue().get();
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
        throw new IOException("mvsgm interrupted", e);
      } catch (ExecutionException e) {
        // Other failures stay with their camera and are reported when it is taken.
        if (outOfMemory(e.getCause())) {
          it.remove();
          outstanding--;
        }
      }
    }
    System.gc();
  }

  static boolean outOfMemory(Throwable failure) {
    for (Throwable t = failure; t != null; t = t.getCause()) if (t instanceof OutOfMemoryError) return true;
    return false;
  }

  Map<String, Object> report() {
    return Json.object("cameras_in_flight", inFlight, "row_workers_per_camera", rowCores,
        "peak_cameras_ahead", peakOutstanding, "requests_outside_predicted_order", outOfOrder,
        "cameras_computed_one_at_a_time", serialCameras, "worker_seconds", workerNanos.get() / 1e9,
        "consumer_wait_seconds", blockedNanos / 1e9);
  }

  @Override public void close() {
    if (closed) return;
    closed = true;
    workers.shutdownNow();
    boolean interrupted = false;
    for (;;) try {
      if (workers.awaitTermination(1, TimeUnit.SECONDS)) break;
    } catch (InterruptedException e) {
      interrupted = true;
    }
    if (interrupted) Thread.currentThread().interrupt();
  }
}
