package sgm;

import java.io.*;
import java.util.concurrent.*;

/** One batch ahead: validation overlaps ordered CPU output without sharing mutable depth pages. */
final class FusionPipeline implements AutoCloseable {
  record Batch(
      double[][] points,
      int[] source,
      int[] pixel,
      int[] rgb,
      DepthFusion.Result[] results,
      DepthFusion.SurfaceSample[] surface) {}

  private final DataInputStream in;
  private final DepthFusion fusion;
  private final ExecutorService worker;
  private Future<Batch> next;
  private long remaining;
  private final boolean normals;
  private final int capacity;
  private long normalNanos;

  long normalNanos() {
    return normalNanos;
  }

  int capacity() {
    return capacity;
  }

  FusionPipeline(DataInputStream in, long count, DepthFusion fusion, boolean asynchronous) {
    this(in, count, fusion, asynchronous, false);
  }

  FusionPipeline(
      DataInputStream in, long count, DepthFusion fusion, boolean asynchronous, boolean normals) {
    this.in = in;
    this.fusion = fusion;
    this.normals = normals;
    capacity = fusion.pipelineCapacity();
    remaining = count;
    worker =
        asynchronous
            ? Executors.newSingleThreadExecutor(r -> new Thread(r, "fusion-validation"))
            : null;
    if (worker != null) next = worker.submit(this::read);
  }

  private Batch read() throws IOException {
    int n = (int) Math.min(capacity, remaining);
    if (n == 0) return null;
    remaining -= n;
    double[][] p = new double[n][];
    int[] source = new int[n], pixel = new int[n], rgb = new int[n];
    for (int i = 0; i < n; i++) {
      p[i] = new double[] {in.readDouble(), in.readDouble(), in.readDouble()};
      rgb[i] = in.readUnsignedByte() << 16 | in.readUnsignedByte() << 8 | in.readUnsignedByte();
      source[i] = in.readInt();
      pixel[i] = in.readInt();
    }
    DepthFusion.Result[] results = fusion.fuseBatch(source, p, n);
    DepthFusion.SurfaceSample[] geometry = new DepthFusion.SurfaceSample[n];
    // Depth rasters/page caches have one owner. The output thread must never query them
    // while validation is prefetched; normal extraction belongs to this same worker.
    if (normals) {
      long started = System.nanoTime();
      for (int i = 0; i < n; i++)
        if (results[i].accepted()) geometry[i] = fusion.surfaceSample(source[i], results[i]);
      normalNanos += System.nanoTime() - started;
    }
    return new Batch(p, source, pixel, rgb, results, geometry);
  }

  Batch next() throws IOException {
    if (worker == null) return read();
    try {
      Batch batch = next.get();
      next = remaining > 0 ? worker.submit(this::read) : null;
      return batch;
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      throw new IOException("Fusion interrupted", e);
    } catch (ExecutionException e) {
      Throwable cause = e.getCause();
      if (cause instanceof RuntimeException r) throw r;
      if (cause instanceof IOException io) throw io;
      if (cause instanceof Error error) throw error;
      throw new IOException(cause);
    }
  }

  public void close() {
    if (worker == null) return;
    worker.shutdown();
    boolean interrupted = false;
    for (; ; )
      try {
        if (worker.awaitTermination(5, TimeUnit.SECONDS)) break;
      } catch (InterruptedException e) {
        interrupted = true;
      }
    if (interrupted) Thread.currentThread().interrupt();
  }
}
