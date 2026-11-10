package sgm;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.*;

/**
 * Versioned lossless metadata in cloud order; CSV conversion is optional and never feeds fusion.
 */
final class PointRecords {
  static final long MAGIC = 0x53474d5054530002L;
  static final String HEADER =
      "point_id,candidate_id,fusion_pair_id,x,y,z,red,green,blue,supporting_views,independent_pairs,estimated_relative_stereo_sigma,refinement_shift,normal_x,normal_y,normal_z,surface_shift,merged_observations,roughness_rms,surface_variation,planarity,normal_radius,normal_neighbors,surface_status,normal_trimmed,confidence,matching_confidence,matching_cost_margin,matching_cost_sharpness,zncc,left_right_error,supporting_camera_ids\n";

  record Row(long id, SurfaceConsolidator.Sample sample, double score) {
    String csv() {
      var p = sample;
      StringJoiner ids = new StringJoiner(";");
      if (p.supportingCameras != null)
        for (int i = p.supportingCameras.nextSetBit(0);
            i >= 0;
            i = p.supportingCameras.nextSetBit(i + 1)) ids.add(Integer.toString(i));
      return id
          + ","
          + p.id
          + ","
          + p.source
          + ","
          + p.point[0]
          + ","
          + p.point[1]
          + ","
          + p.point[2]
          + ","
          + (p.rgb >>> 16)
          + ","
          + (p.rgb >>> 8 & 255)
          + ","
          + (p.rgb & 255)
          + ","
          + p.views
          + ","
          + p.pairs
          + ","
          + p.sigma
          + ","
          + p.shift
          + ","
          + (p.normal == null ? "NaN,NaN,NaN" : p.normal[0] + "," + p.normal[1] + "," + p.normal[2])
          + ","
          + p.surfaceShift
          + ","
          + p.merged
          + ","
          + p.attributes.csv(1)
          + ","
          + score
          + ","
          + p.matchingConfidence
          + ","
          + p.costMargin
          + ","
          + p.costSharpness
          + ","
          + p.photometric
          + ","
          + p.lrError
          + ","
          + (ids.length() == 0 ? "unavailable" : ids.toString())
          + "\n";
    }
  }

  static final class Writer implements AutoCloseable {
    private final DataOutputStream out;
    private boolean closed;
    long nanos, bytes;

    Writer(Path file) throws IOException {
      out = new DataOutputStream(new BufferedOutputStream(Files.newOutputStream(file), 1 << 20));
      out.writeLong(MAGIC);
    }

    // One buffer per record (the same big-endian bytes as DataOutputStream, without a stream call
    // per field).
    private java.nio.ByteBuffer record = java.nio.ByteBuffer.allocate(256);

    void point(long id, SurfaceConsolidator.Sample p, double scale, double score)
        throws IOException {
      long started = System.nanoTime();
      int idBytes = p.supportingCameras == null ? 0 : (p.supportingCameras.length() + 7) / 8;
      if (record.capacity() < 206 + idBytes) record = java.nio.ByteBuffer.allocate(206 + idBytes);
      java.nio.ByteBuffer b = record.clear();
      b.put((byte) 1);
      b.putLong(id);
      b.putLong(p.id);
      b.putInt(p.source);
      b.putInt(p.pixel);
      b.putInt(p.rgb);
      b.putInt(p.views);
      b.putInt(p.pairs);
      b.putInt(p.merged);
      for (double v : p.point) b.putDouble(v * scale);
      b.putDouble(p.sigma);
      b.putDouble(p.shift * scale);
      for (int k = 0; k < 3; k++) b.putDouble(p.normal == null ? Double.NaN : p.normal[k]);
      b.putDouble(p.surfaceShift * scale);
      var a = p.attributes;
      b.putDouble(a.roughness() * scale);
      b.putDouble(a.variation());
      b.putDouble(a.planarity());
      b.putDouble(a.radius() * scale);
      b.putInt(a.neighbors());
      b.putInt(a.status());
      b.put((byte) (a.trimmed() ? 1 : 0));
      b.putDouble(score);
      b.putDouble(p.matchingConfidence);
      b.putDouble(p.costMargin);
      b.putDouble(p.costSharpness);
      b.putDouble(p.photometric);
      b.putDouble(p.lrError);
      b.putInt(idBytes);
      // Reuse record storage for the little-endian BitSet bytes; no per-point bitmap array.
      int start = b.position();
      for (int i = 0; i < idBytes; i++) b.put((byte) 0);
      if (p.supportingCameras != null)
        for (int bit = p.supportingCameras.nextSetBit(0); bit >= 0;
            bit = p.supportingCameras.nextSetBit(bit + 1)) {
          int at = start + (bit >>> 3);
          b.put(at, (byte) (b.get(at) | (1 << (bit & 7))));
        }
      out.write(b.array(), 0, b.position());
      bytes += 206L + idBytes;
      nanos += System.nanoTime() - started;
    }

    void flush() throws IOException {
      out.flush();
    }

    public void close() throws IOException {
      if (!closed) {
        closed = true;
        try {
          out.writeByte(0);
        } finally {
          out.close();
        }
      }
    }
  }

  static final class Reader implements AutoCloseable {
    private final DataInputStream in;
    private boolean ended;
    private final boolean version2;

    Reader(Path file) throws IOException {
      in = new DataInputStream(new BufferedInputStream(Files.newInputStream(file), 1 << 16));
      long magic;
      try {
        magic = in.readLong();
      } catch (IOException e) {
        in.close();
        throw e;
      }
      version2 = magic == MAGIC;
      if (!version2 && magic != 0x53474d5054530001L) {
        in.close();
        throw new IOException("Unknown point metadata format: " + file);
      }
    }

    private final byte[] fixed = new byte[205];

    Row next() throws IOException {
      if (ended) return null;
      int marker = in.read();
      if (marker < 0) throw new java.io.EOFException();
      if (marker == 0) {
        ended = true;
        if (in.read() != -1) throw new IOException("Trailing point metadata");
        return null;
      }
      if (marker != 1) throw new IOException("Corrupt point metadata record");
      // The fixed part in one read, decoded from a buffer (same big-endian layout).
      int size = version2 ? 205 : 201;
      in.readFully(fixed, 0, size);
      java.nio.ByteBuffer b = java.nio.ByteBuffer.wrap(fixed, 0, size);
      long id = b.getLong();
      var p = new SurfaceConsolidator.Sample();
      p.id = b.getLong();
      p.source = b.getInt();
      p.pixel = version2 ? b.getInt() : 0;
      p.rgb = b.getInt();
      p.views = b.getInt();
      p.pairs = b.getInt();
      p.merged = b.getInt();
      p.point = new double[] {b.getDouble(), b.getDouble(), b.getDouble()};
      p.sigma = b.getDouble();
      p.shift = b.getDouble();
      p.normal = new double[] {b.getDouble(), b.getDouble(), b.getDouble()};
      p.surfaceShift = b.getDouble();
      double roughness = b.getDouble(), variation = b.getDouble(), planarity = b.getDouble(), radius = b.getDouble();
      int neighbors = b.getInt(), status = b.getInt();
      boolean trimmed = b.get() != 0;
      p.attributes = new SurfaceAttributes(roughness, variation, planarity, radius, neighbors, status, trimmed);
      double score = b.getDouble();
      p.matchingConfidence = b.getDouble();
      p.costMargin = b.getDouble();
      p.costSharpness = b.getDouble();
      p.photometric = b.getDouble();
      p.lrError = b.getDouble();
      int length = b.getInt();
      if (length < 0 || length > 1 << 20) throw new IOException("Invalid camera bitmap size");
      byte[] ids = new byte[length];
      in.readFully(ids);
      p.supportingCameras = BitSet.valueOf(ids);
      return new Row(id, p, score);
    }

    public void close() throws IOException {
      in.close();
    }
  }

  /** Legacy CSV and binary records share the attribution reader without forcing CSV export. */
  static final class Rows implements AutoCloseable {
    private final Reader binary;
    private final BufferedReader csv;

    Rows(Path run) throws IOException {
      Path dir = run.resolve("07_results");
      if (Files.exists(dir.resolve("point-confidence.bin"))) {
        binary = new Reader(dir.resolve("point-confidence.bin"));
        csv = null;
      } else {
        binary = null;
        csv =
            new BufferedReader(
                new InputStreamReader(
                    new java.util.zip.GZIPInputStream(
                        Files.newInputStream(dir.resolve("point-confidence.csv.gz")), 65536),
                    StandardCharsets.UTF_8));
        csv.readLine();
      }
    }

    String next(boolean wanted) throws IOException {
      if (csv != null) return csv.readLine();
      Row row = binary.next();
      return row == null ? null : wanted ? row.csv().stripTrailing() : "";
    }

    public void close() throws IOException {
      if (binary != null) binary.close();
      else csv.close();
    }
  }

  static void export(Path run, int compression) throws IOException {
    export(run, compression, 1);
  }

  static void export(Path run, int compression, int cores) throws IOException {
    if (compression < 1 || compression > 9 || cores < 1)
      throw new IllegalArgumentException(
          "Confidence export requires compression 1..9 and positive cores");
    Path dir = Files.isDirectory(run.resolve("07_results")) ? run.resolve("07_results") : run;
    Path partial = dir.resolve("point-confidence.csv.gz.part"),
        target = dir.resolve("point-confidence.csv.gz");
    int workers = Math.max(1, Math.min(16, cores));
    long started = System.nanoTime(), rows = 0;
    var os = java.lang.management.ManagementFactory.getOperatingSystemMXBean();
    long cpuStart =
        os instanceof com.sun.management.OperatingSystemMXBean measured
            ? measured.getProcessCpuTime()
            : -1;
    var formatting = new java.util.concurrent.atomic.LongAdder();
    var compressionTime = new java.util.concurrent.atomic.LongAdder();
    var executor =
        java.util.concurrent.Executors.newFixedThreadPool(
            workers, r -> new Thread(r, "confidence-export"));
    ArrayDeque<java.util.concurrent.Future<byte[]>> pending = new ArrayDeque<>();
    try {
      try (Reader in = new Reader(dir.resolve("point-confidence.bin"));
          var out = new BufferedOutputStream(Files.newOutputStream(partial), 1 << 20)) {
        boolean first = true, ended = false;
        while (!ended) {
          List<Row> chunk = new ArrayList<>(512);
          Row row;
          while (chunk.size() < 512) {
            row = in.next();
            if (row == null) {
              ended = true;
              break;
            }
            chunk.add(row);
            rows++;
          }
          if (chunk.isEmpty() && !first) break;
          boolean header = first;
          first = false;
          pending.add(
              executor.submit(
                  () -> {
                    long clock = System.nanoTime();
                    StringBuilder csv = new StringBuilder(512 * 512);
                    if (header) csv.append(HEADER);
                    for (var item : chunk) csv.append(item.csv());
                    byte[] encoded = csv.toString().getBytes(StandardCharsets.UTF_8);
                    formatting.add(System.nanoTime() - clock);
                    clock = System.nanoTime();
                    ByteArrayOutputStream bytes = new ByteArrayOutputStream(65536);
                    try (var compressed = Compression.gzip(bytes, compression)) {
                      compressed.write(encoded);
                    }
                    compressionTime.add(System.nanoTime() - clock);
                    return bytes.toByteArray();
                  }));
          if (pending.size() >= workers * 2) out.write(await(pending.remove()));
        }
        while (!pending.isEmpty()) out.write(await(pending.remove()));
      }
      CloudWriter.publish(partial, target);
      Json.write(
          dir.resolve("confidence-export.json"),
          Json.object(
              "rows",
              rows,
              "workers",
              workers,
              "compression_level",
              compression,
              "wall_nanoseconds",
              System.nanoTime() - started,
              "process_cpu_nanoseconds",
              cpuStart < 0
                  ? null
                  : ((com.sun.management.OperatingSystemMXBean) os).getProcessCpuTime() - cpuStart,
              "summed_worker_formatting_nanoseconds",
              formatting.sum(),
              "summed_worker_compression_nanoseconds",
              compressionTime.sum(),
              "compressed_bytes",
              Files.size(target),
              "format",
              "Ordered concatenated gzip members; unchanged CSV schema and numeric values"));
    } finally {
      executor.shutdownNow();
      Files.deleteIfExists(partial);
    }
  }

  private static byte[] await(java.util.concurrent.Future<byte[]> future) throws IOException {
    try {
      return future.get();
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      throw new IOException("Confidence export interrupted", e);
    } catch (java.util.concurrent.ExecutionException e) {
      throw new IOException("Confidence export failed", e.getCause());
    }
  }
}
