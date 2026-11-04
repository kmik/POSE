package sgm;

import java.io.*;
import java.util.concurrent.*;

/** One batch ahead: validation overlaps ordered CPU output without sharing mutable depth pages. */
final class FusionPipeline implements AutoCloseable {
  record Batch(
      double[][] points,
      long[] candidateIds,
      int[] source,
      int[] pixel,
      int[] rgb,
      DepthFusion.Result[] results,
      DepthFusion.SurfaceSample[] surface) {}

  private final DataInputStream in;
  private final DataInputStream prepared;
  private final DepthFusion fusion;
  private final ExecutorService worker;
  private Future<Batch> next;
  private long remaining;
  private final boolean normals, matchingEvidence;
  private final int capacity;
  private long normalNanos;
  private long readNanos, fuseNanos, waitNanos, batches;
  private long consumed;

  long normalNanos() {
    return normalNanos;
  }

  /** Records evidence reads, depth fusion and consumer waits accumulated over all batches. */
  void recordTimings() {
    RunTelemetry.recordTotal("Fusion batches: evidence read", readNanos, batches);
    RunTelemetry.recordTotal("Fusion batches: depth fusion", fuseNanos, batches);
    RunTelemetry.recordTotal("Fusion batches: surface normals", normalNanos, batches);
    RunTelemetry.recordTotal("Fusion batches: output wait for validation", waitNanos, batches);
  }

  int capacity() {
    return capacity;
  }

  FusionPipeline(DataInputStream in, long count, DepthFusion fusion, boolean asynchronous) {
    this(in, count, fusion, asynchronous, false);
  }

  FusionPipeline(
      DataInputStream in, long count, DepthFusion fusion, boolean asynchronous, boolean normals) {
    this(in, count, fusion, asynchronous, normals, false);
  }

  FusionPipeline(
      DataInputStream in,
      long count,
      DepthFusion fusion,
      boolean asynchronous,
      boolean normals,
      boolean matchingEvidence) {
    this(in, count, fusion, asynchronous, normals, matchingEvidence, null);
  }

  FusionPipeline(
      DataInputStream in,
      long count,
      DepthFusion fusion,
      boolean asynchronous,
      boolean normals,
      boolean matchingEvidence,
      DataInputStream prepared) {
    this.prepared = prepared;
    this.matchingEvidence = matchingEvidence;
    this.in = in;
    this.fusion = fusion;
    this.normals = normals;
    capacity = fusion.pipelineCapacity();
    remaining = count;
    worker =
        asynchronous
            ? Executors.newSingleThreadExecutor(
                r ->
                    new Thread(
                        () -> {
                          try {
                            r.run();
                          } finally {
                            fusion.releaseReader();
                          }
                        },
                        "fusion-validation"))
            : null;
    if (worker != null) next = worker.submit(this::read);
  }

  private Batch read() throws IOException {
    int n = (int) Math.min(capacity, remaining);
    if (n == 0) return null;
    long readStarted = System.nanoTime();
    batches++;
    remaining -= n;
    double[][] p = new double[n][];
    int[] source = new int[n], pixel = new int[n], rgb = new int[n];
    int evidenceFields = Double.isFinite(fusion.twoViewConfidenceThreshold()) ? 6 : 5;
    float[][] evidence = matchingEvidence ? new float[n][evidenceFields] : null;
    for (int i = 0; i < n; i++) {
      p[i] = new double[] {in.readDouble(), in.readDouble(), in.readDouble()};
      rgb[i] = in.readUnsignedByte() << 16 | in.readUnsignedByte() << 8 | in.readUnsignedByte();
      source[i] = in.readInt();
      pixel[i] = in.readInt();
      if (matchingEvidence)
        for (int k = 0; k < evidenceFields; k++) evidence[i][k] = in.readFloat();
    }
    DepthFusion.Result[] results;
    long[] candidateIds = new long[n];
    for (int i = 0; i < n; i++) candidateIds[i] = consumed + i;
    consumed += n;
    long fuseStarted = System.nanoTime();
    readNanos += fuseStarted - readStarted;
    if (prepared == null) results = fusion.fuseBatch(source, p, n);
    else {
      results = new DepthFusion.Result[n];
      for (int i = 0; i < n; i++) {
        candidateIds[i] = prepared.readLong();
        results[i] = IncrementalFusion.readResult(prepared);
      }
    }
    fuseNanos += System.nanoTime() - fuseStarted;
    if (matchingEvidence)
      for (int i = 0; i < n; i++) {
        var r = results[i];
        r =
            results[i] =
                TwoViewConfidence.apply(
                    r,
                    evidence[i][0],
                    evidenceFields == 6 ? evidence[i][5] : Double.NaN,
                    fusion.twoViewConfidenceThreshold());
        r.matchingConfidence = evidence[i][0];
        r.costMargin = evidence[i][1];
        r.costSharpness = evidence[i][2];
        r.photometric = evidence[i][3];
        r.lrError = evidence[i][4];
        r.confidence = PointConfidence.finalScore(r.matchingConfidence, r.relativeSigma);
      }
    DepthFusion.SurfaceSample[] geometry = new DepthFusion.SurfaceSample[n];
    // Depth rasters/page caches have one owner. The output thread must never query them
    // while validation is prefetched; normal extraction belongs to this same worker.
    if (normals) {
      long started = System.nanoTime();
      geometry = fusion.surfaceBatch(source, results, n);
      normalNanos += System.nanoTime() - started;
    }
    return new Batch(p, candidateIds, source, pixel, rgb, results, geometry);
  }

  Batch next() throws IOException {
    if (worker == null) return read();
    try {
      long waitStarted = System.nanoTime();
      Batch batch = next.get();
      waitNanos += System.nanoTime() - waitStarted;
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
