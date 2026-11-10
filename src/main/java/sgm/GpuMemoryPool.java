package sgm;

import java.util.ArrayDeque;

/**
 * Shared GPU working memory for concurrent stereo pairs. Without it, every pair worker received a
 * fixed 1/N share of the GPU budget, and a pair whose packed volume did not fit that share fell
 * back to the (exact, but much slower) CPU banded solve. With the pool every worker may use the
 * whole budget; a banded session reserves its device bytes before allocating and waits (first
 * come, first served) until enough is free. A worker never waits while holding a reservation (a
 * session that must grow is closed first), so workers cannot deadlock. Buffer sizes do not change
 * any result.
 */
final class GpuMemoryPool {
  private static volatile GpuMemoryPool active;

  private final long capacity;
  private long free;
  private final ArrayDeque<Object> queue = new ArrayDeque<>();
  private long waits, waitNanos, peak;

  private GpuMemoryPool(long capacity) {
    this.capacity = capacity;
    this.free = capacity;
  }

  /** Installs a pool for the duration of a concurrent pair phase; close() removes it. */
  static Scope install(long capacity) {
    GpuMemoryPool pool = new GpuMemoryPool(capacity);
    active = pool;
    return new Scope(pool);
  }

  static boolean enabled() {
    return active != null;
  }

  /** A held reservation; a no-op when no pool is installed. */
  static Reservation reserve(long bytes) {
    GpuMemoryPool pool = active;
    if (pool == null || bytes <= 0) return new Reservation(null, 0);
    long wanted = Math.min(bytes, pool.capacity);
    long held = HELD.get().get();
    if (held > 0) {
      // Waiting while holding pool memory can deadlock (with other workers, or with itself).
      Reservation now = pool.tryAcquire(wanted);
      if (now != null) return now;
      throw new IllegalStateException(
          "GPU memory pool: this thread holds " + (held >> 20) + " MiB and would wait for "
              + (wanted >> 20) + " MiB more");
    }
    return pool.acquire(wanted);
  }

  /** Pool bytes held by each thread (a thread holding pool memory must never wait for more). */
  private static final ThreadLocal<java.util.concurrent.atomic.AtomicLong> HELD =
      ThreadLocal.withInitial(java.util.concurrent.atomic.AtomicLong::new);

  /**
   * Working memory of a short auxiliary GPU step (per-pair filters). Waits like {@link #reserve}
   * only when the calling thread holds no pool memory; otherwise it takes the bytes only if they
   * are free now. Returns null when they cannot be had without risking a deadlock, or exceed the
   * pool; the caller then runs its CPU path, which gives the same result.
   */
  static Reservation reserveAuxiliary(long bytes) {
    GpuMemoryPool pool = active;
    if (pool == null || bytes <= 0) return new Reservation(null, 0);
    if (bytes > pool.capacity) return null;
    if (HELD.get().get() > 0) return pool.tryAcquire(bytes);
    return pool.acquire(bytes);
  }

  private synchronized Reservation tryAcquire(long bytes) {
    if (!queue.isEmpty() || free < bytes) return null;
    free -= bytes;
    peak = Math.max(peak, capacity - free);
    return new Reservation(this, bytes);
  }

  private synchronized Reservation acquire(long bytes) {
    Object ticket = new Object();
    queue.add(ticket);
    long started = 0;
    try {
      while (queue.peek() != ticket || free < bytes) {
        if (started == 0) {
          started = System.nanoTime();
          waits++;
        }
        wait();
      }
    } catch (InterruptedException e) {
      queue.remove(ticket);
      notifyAll();
      Thread.currentThread().interrupt();
      throw new IllegalStateException("Interrupted while waiting for GPU memory", e);
    }
    queue.poll();
    free -= bytes;
    peak = Math.max(peak, capacity - free);
    if (started != 0) waitNanos += System.nanoTime() - started;
    notifyAll();
    return new Reservation(this, bytes);
  }

  private synchronized void release(long bytes) {
    free = Math.min(capacity, free + bytes);
    notifyAll();
  }

  /**
   * After all pair workers have stopped (and the caller holds no reservation), any bytes still
   * reserved were leaked by a failed worker; returns them so the caller cannot wait forever.
   * Returns the number of reclaimed bytes.
   */
  static long reclaimAfterWorkersStopped() {
    GpuMemoryPool pool = active;
    if (pool == null) return 0;
    synchronized (pool) {
      long leaked = pool.capacity - pool.free;
      pool.free = pool.capacity;
      pool.notifyAll();
      return leaked;
    }
  }

  static final class Reservation implements AutoCloseable {
    private GpuMemoryPool pool;
    final long bytes;

    private final java.util.concurrent.atomic.AtomicLong holder;

    private Reservation(GpuMemoryPool pool, long bytes) {
      this.pool = pool;
      this.bytes = bytes;
      holder = pool == null ? null : HELD.get();
      if (holder != null) holder.addAndGet(bytes);
    }

    @Override
    public void close() {
      if (pool != null) {
        pool.release(bytes);
        holder.addAndGet(-bytes);
      }
      pool = null;
    }
  }

  static final class Scope implements AutoCloseable {
    private final GpuMemoryPool pool;

    private Scope(GpuMemoryPool pool) {
      this.pool = pool;
    }

    String summary() {
      synchronized (pool) {
        return String.format(
            java.util.Locale.ROOT,
            "GPU memory pool: %d MiB shared by the pair workers; peak reserved %d MiB; %d waits"
                + " (%.1f s)",
            pool.capacity >> 20,
            pool.peak >> 20,
            pool.waits,
            pool.waitNanos / 1e9);
      }
    }

    @Override
    public void close() {
      if (active == pool) active = null;
    }
  }
}
