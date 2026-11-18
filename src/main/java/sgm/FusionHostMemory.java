package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import java.util.concurrent.*;

/** One opt-in allowance across stereo, fusion, retained state, raster and map caches, and output.
 * This is conservative application accounting, not a hard process RSS or system-cache limit.
 */
final class FusionHostMemory implements AutoCloseable {
  final Options options;
  final long allowance, retained, nativeReserve, cacheBytes, cacheMetadataBytes, outputReserve, workBytes;
  final HostMapCache cache;
  private final ScheduledExecutorService reporter;
  private volatile int activeCameras, retiredCameras;
  private volatile long upcoming, peakUpcoming;
  private volatile long outputRetained, peakOutputRetained, currentWorkspace;
  private volatile boolean pressure;
  private long pressureEvents;

  FusionHostMemory(Options o) {
    this(o, 0);
  }
  FusionHostMemory(Options o, long futureMetadata) {
    options = o;
    // Establish retained project state before reserving the next workspace. Full GC only at this
    // stage boundary (or on a failed admission), never per pixel/strip.
    RasterCache.clear();
    NativeMemory.trim("before bounded fusion planning");
    System.gc();
    var heap = MemorySafety.snapshot();
    retained = heap.heapUsed() + (32L << 20) + futureMetadata;
    nativeReserve = Math.max(64L << 20, Math.max(0, NativeMemory.committedBytes() - heap.heapCommitted()))
        + RasterCache.limit(o);
    allowance = Math.min(o.fusionHostMemoryMb * MemorySafety.MIB, heap.heapMaximum() * 85 / 100 + nativeReserve);
    cacheBytes = Math.min((long) o.fusionHostCacheMb << 20, allowance / 8);
    cacheMetadataBytes = 256L * Math.ceilDiv(cacheBytes, HostMapCache.PAGE);
    long outputBuffers = 8L << 20;
    if (o.tiledLasOutput > 0) outputBuffers += (long) CloudWriter.OPEN_TILES << 16;
    if (o.surfaceConsolidation.equals("normal")) {
      long surface = Math.min(o.surfaceMemoryMb, Math.max(8, o.memoryMb / 4)) * MemorySafety.MIB;
      long decoded = Math.max(surface / 8, Math.min(512L << 20, o.memoryMb * MemorySafety.MIB / 16));
      outputBuffers += 3 * surface + decoded + (16L << 20);
    }
    outputReserve = outputBuffers;
    workBytes = allowance - retained - nativeReserve - cacheBytes - cacheMetadataBytes - outputReserve;
    if (workBytes < 64L << 20)
      throw new IllegalArgumentException("--fusion-host-memory-mb leaves less than 64 MiB after retained project,"
          + " native/image, map-cache and output reservations; increase the allowance/job RAM or reduce features");
    cache = new HostMapCache(cacheBytes);
    System.out.printf(Locale.ROOT, "Bounded host fusion: allowance %,d MiB; retained project %,d MiB;"
        + " native/image reserve %,d MiB; output buffers %,d MiB; shared map cache %,d MiB; work/output %,d MiB; stereo concurrency 1%n",
        allowance >> 20, retained >> 20, nativeReserve >> 20, outputReserve >> 20, cacheBytes >> 20, workBytes >> 20);
    reporter = Executors.newSingleThreadScheduledExecutor(r -> {
      Thread t = new Thread(r, "fusion-host-memory"); t.setDaemon(true); return t;
    });
    reporter.scheduleAtFixedRate(this::diagnose, 0, 30, TimeUnit.SECONDS);
  }

  void cameras(int active, int retired) { activeCameras = active; retiredCameras = retired; }
  /** Reserve before allocating. Stereo and fusion run serially, but pending output is included. */
  void require(long bytes, String operation) throws IOException {
    if (bytes < 0) throw new IOException("Allocation estimate overflow before " + operation);
    if (Thread.currentThread().isInterrupted()) throw new InterruptedIOException("Cancelled before " + operation);
    currentWorkspace = bytes;
    try { bytes = Math.addExact(bytes, outputRetained); }
    catch (ArithmeticException e) { throw new IOException("Allocation estimate overflow before " + operation, e); }
    upcoming = bytes; peakUpcoming = Math.max(peakUpcoming, bytes);
    if (pressure) {
      cache.trim(); RasterCache.clear(); NativeMemory.trim("before " + operation);
    }
    var host = MemorySafety.limits(options.ramLimitMb);
    var heap = MemorySafety.snapshot();
    long heapRoom = Math.max(0, heap.heapMaximum() - heap.heapUsed());
    if (bytes > heapRoom) {
      System.gc(); heap = MemorySafety.snapshot();
      heapRoom = Math.max(0, heap.heapMaximum() - heap.heapUsed());
      if (bytes > heapRoom) {
        cache.trim(); System.gc(); heap = MemorySafety.snapshot();
        heapRoom = Math.max(0, heap.heapMaximum() - heap.heapUsed());
      }
    }
    long hostRoom = host.available() < 0 ? Long.MAX_VALUE : Math.max(0, host.available() - (64L << 20));
    // Unused committed heap is not necessarily resident: do not add it to host/job RSS room.
    if (bytes > workBytes || bytes > heapRoom || bytes > hostRoom)
      throw new IOException(String.format(Locale.ROOT,
          "Bounded host fusion refuses %s: estimated %,d MiB, work allowance %,d MiB, heap room %,d MiB,"
              + " host/job room %,d MiB. Increase --fusion-host-memory-mb with sufficient --memory-mb/job RAM,"
              + " reduce --fusion-host-cache-mb, or lower --quality. No dependencies were removed.",
          operation, bytes >> 20, workBytes >> 20, heapRoom >> 20, hostRoom >> 20));
  }
  boolean pressured() { return pressure; }
  void workspaceFinished() { currentWorkspace = 0; }

  /** Output hash sets/tile metadata may grow; admit each new entry before its allocation. */
  void retainOutput(long bytes) throws IOException {
    if (bytes < 0 || bytes > workBytes - currentWorkspace)
      throw new IOException("Bounded host fusion refuses growing output voxel/tile metadata: increase"
          + " --fusion-host-memory-mb with sufficient job RAM, use --voxel 0 or a larger --tiled-las-output;"
          + " reconstruction quality and dependencies were not changed");
    outputRetained = bytes;
    peakOutputRetained = Math.max(peakOutputRetained, bytes);
  }

  /** Join every active row even after failure/interruption, before a scoped reader may close. */
  static void parallel(ForkJoinPool pool, int count, java.util.function.IntConsumer action, boolean serial) {
    if (serial) {
      for (int i = 0; i < count; i++) {
        if (Thread.currentThread().isInterrupted()) throw new IllegalStateException("Bounded fusion row work interrupted");
        action.accept(i);
      }
      return;
    }
    int workers = Math.min(count, pool.getParallelism());
    List<Future<?>> tasks = new ArrayList<>();
    var stop = new java.util.concurrent.atomic.AtomicBoolean();
    Throwable failure = null;
    try {
    for (int t = 0; t < workers; t++) {
      int from = (int) ((long) count * t / workers), to = (int) ((long) count * (t + 1) / workers);
      tasks.add(pool.submit(() -> {
        try { for (int i = from; i < to && !stop.get(); i++) action.accept(i); }
        catch (RuntimeException | Error e) { stop.set(true); throw e; }
      }));
    }
    } catch (RuntimeException | Error e) { failure = e; stop.set(true); }
    boolean interrupted = false;
    for (Future<?> task : tasks) for (;;) try { task.get(); break; }
    catch (InterruptedException e) { interrupted = true; stop.set(true); }
    catch (ExecutionException e) { if (failure == null) failure = e.getCause(); break; }
    if (interrupted) Thread.currentThread().interrupt();
    if (failure instanceof RuntimeException e) throw e;
    if (failure instanceof Error e) throw e;
    if (failure != null || interrupted) throw new IllegalStateException("Bounded fusion row work interrupted/failed", failure);
  }
  private void diagnose() {
    try {
      var h = MemorySafety.snapshot(); var limits = MemorySafety.limits(options.ramLimitMb);
      long[] process = RunTelemetry.processMemoryBytes();
      pressure = limits.available() >= 0 && limits.available() < Math.max(128L << 20, allowance / 20)
          || h.heapUsed() > h.heapMaximum() * 9 / 10;
      if (pressure) { pressureEvents++; cache.trim(); }
      String rss = "rss=" + mib(process[0]) + " private=" + mib(process[1]);
      Path status = Path.of("/proc/self/status");
      if (Files.isRegularFile(status))
        for (String line : Files.readAllLines(status))
          if (line.startsWith("RssAnon:") || line.startsWith("RssFile:") || line.startsWith("RssShmem:")) rss += " " + line.strip();
      System.out.printf(Locale.ROOT, "Host fusion memory: heap=%s/%s MiB; available host/job=%s MiB (%s); %s;"
          + " mapped file extents=%s MiB (not resident RAM); cache=%s MiB; active/retired cameras=%d/%d; evictions=%s; pressure=%s%n",
          mib(h.heapUsed()), mib(h.heapMaximum()), mib(limits.available()), limits.source(), rss,
          mib(MappedFiles.liveBytes()), mib(cache.bytes()), activeCameras, retiredCameras, cache.report().get("evictions"), pressure);
    } catch (IOException | RuntimeException e) { System.err.println("Host fusion diagnostic: " + e.getMessage()); }
  }
  private static String mib(long n) { return n < 0 ? "unknown" : Long.toString(n >> 20); }
  Map<String, Object> report() {
    return Json.object("allowance_bytes", allowance, "retained_project_bytes", retained,
        "native_image_reserve_bytes", nativeReserve, "work_output_allowance_bytes", workBytes,
        "cache_metadata_reservation_bytes", cacheMetadataBytes,
        "output_buffer_reserve_bytes", outputReserve, "output_retained_bytes", outputRetained,
        "peak_output_retained_bytes", peakOutputRetained,
        "peak_workspace_estimate_bytes", peakUpcoming, "pressure_events", pressureEvents,
        "cache", cache.report(), "scope", "Application accounting; not a strict total RAM guarantee; OS file cache and driver allocations are external");
  }
  @Override public void close() throws IOException {
    reporter.shutdownNow();
    boolean interrupted = Thread.interrupted();
    for (;;) try { if (reporter.awaitTermination(1, TimeUnit.SECONDS)) break; }
    catch (InterruptedException e) { interrupted = true; }
    try { cache.close(); } finally { if (interrupted) Thread.currentThread().interrupt(); }
  }
}
