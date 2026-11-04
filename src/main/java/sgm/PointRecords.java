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

    void point(long id, SurfaceConsolidator.Sample p, double scale, double score)
        throws IOException {
      long started = System.nanoTime();
      out.writeByte(1);
      out.writeLong(id);
      out.writeLong(p.id);
      out.writeInt(p.source);
      out.writeInt(p.pixel);
      out.writeInt(p.rgb);
      out.writeInt(p.views);
      out.writeInt(p.pairs);
      out.writeInt(p.merged);
      for (double v : p.point) out.writeDouble(v * scale);
      out.writeDouble(p.sigma);
      out.writeDouble(p.shift * scale);
      for (int k = 0; k < 3; k++) out.writeDouble(p.normal == null ? Double.NaN : p.normal[k]);
      out.writeDouble(p.surfaceShift * scale);
      var a = p.attributes;
      out.writeDouble(a.roughness() * scale);
      out.writeDouble(a.variation());
      out.writeDouble(a.planarity());
      out.writeDouble(a.radius() * scale);
      out.writeInt(a.neighbors());
      out.writeInt(a.status());
      out.writeBoolean(a.trimmed());
      out.writeDouble(score);
      out.writeDouble(p.matchingConfidence);
      out.writeDouble(p.costMargin);
      out.writeDouble(p.costSharpness);
      out.writeDouble(p.photometric);
      out.writeDouble(p.lrError);
      byte[] ids = p.supportingCameras == null ? new byte[0] : p.supportingCameras.toByteArray();
      out.writeInt(ids.length);
      out.write(ids);
      bytes += 206L + ids.length;
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

    Row next() throws IOException {
      if (ended) return null;
      int marker = in.readUnsignedByte();
      if (marker == 0) {
        ended = true;
        if (in.read() != -1) throw new IOException("Trailing point metadata");
        return null;
      }
      if (marker != 1) throw new IOException("Corrupt point metadata record");
      long id = in.readLong();
      var p = new SurfaceConsolidator.Sample();
      p.id = in.readLong();
      p.source = in.readInt();
      p.pixel = version2 ? in.readInt() : 0;
      p.rgb = in.readInt();
      p.views = in.readInt();
      p.pairs = in.readInt();
      p.merged = in.readInt();
      p.point = new double[] {in.readDouble(), in.readDouble(), in.readDouble()};
      p.sigma = in.readDouble();
      p.shift = in.readDouble();
      p.normal = new double[] {in.readDouble(), in.readDouble(), in.readDouble()};
      p.surfaceShift = in.readDouble();
      p.attributes =
          new SurfaceAttributes(
              in.readDouble(),
              in.readDouble(),
              in.readDouble(),
              in.readDouble(),
              in.readInt(),
              in.readInt(),
              in.readBoolean());
      double score = in.readDouble();
      p.matchingConfidence = in.readDouble();
      p.costMargin = in.readDouble();
      p.costSharpness = in.readDouble();
      p.photometric = in.readDouble();
      p.lrError = in.readDouble();
      int length = in.readInt();
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
