package sgm;

import java.io.*;
import java.nio.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.time.LocalDate;
import java.util.*;

/**
 * Streams RGB LAS 1.2 format 2 and binary little-endian PLY without holding the cloud in memory.
 * LAS output is directly consumable by LAS.utils, with no dependency on its old OpenCV classpath.
 */
public final class CloudWriter implements AutoCloseable {
  private final Path directory, spool;
  private final DataOutputStream data;
  private final double scale;
  private final Crs crs;
  private final double[] min = {
    Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY
  };
  private final double[] max = {
    Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY
  };
  private final boolean attributes;
  private final ByteBuffer extraRecord =
      ByteBuffer.allocate(SurfaceAttributes.BYTES).order(ByteOrder.LITTLE_ENDIAN);
  private final ByteBuffer spoolRecord;
  private long count;
  private boolean closed;
  private boolean ply = true;

  /** false: write only dense.las (--ply off). The LAS bytes do not depend on it. */
  void writePly(boolean write) {
    ply = write;
  }

  public CloudWriter(Path directory, double scale) throws IOException {
    this(directory, scale, null);
  }

  CloudWriter(Path directory, double scale, Crs crs) throws IOException {
    this(directory, scale, crs, false);
  }

  CloudWriter(Path directory, double scale, Crs crs, boolean attributes) throws IOException {
    this.attributes = attributes;
    spoolRecord = ByteBuffer.allocate(27 + (attributes ? SurfaceAttributes.BYTES : 0));
    if (crs != null && scale != 1)
      throw new IllegalArgumentException("A CRS-tagged cloud requires scale 1");
    this.directory = directory;
    this.scale = scale;
    this.crs = crs;
    Files.createDirectories(directory);
    spool = Files.createTempFile(directory, "cloud-", ".bin");
    data = new DataOutputStream(new BufferedOutputStream(Files.newOutputStream(spool), 1 << 20));
  }

  public void point(double x, double y, double z, int red, int green, int blue) throws IOException {
    point(x, y, z, red, green, blue, null, SurfaceAttributes.missing(false));
  }

  void point(
      double x,
      double y,
      double z,
      int red,
      int green,
      int blue,
      double[] normal,
      SurfaceAttributes surface)
      throws IOException {
    if (closed) throw new IOException("Cloud writer is closed");
    x *= scale; y *= scale; z *= scale;
    if (!Double.isFinite(x + y + z)) throw new IOException("Non-finite cloud point");
    if (count == 0xffffffffL) throw new IOException("LAS 1.2 point-count limit exceeded");
    spoolRecord.clear();
    spoolRecord.putDouble(x).putDouble(y).putDouble(z).put((byte) red).put((byte) green).put((byte) blue);
    min[0] = Math.min(min[0], x); max[0] = Math.max(max[0], x);
    min[1] = Math.min(min[1], y); max[1] = Math.max(max[1], y);
    min[2] = Math.min(min[2], z); max[2] = Math.max(max[2], z);
    if (attributes) {
      extraRecord.clear();
      surface.binary(extraRecord, normal, scale);
      spoolRecord.put(extraRecord.array());
    }
    data.write(spoolRecord.array());
    count++;
  }

  public long count() {
    return count;
  }

  public void abort() throws IOException {
    if (closed) return;
    closed = true;
    try {
      data.close();
    } finally {
      Files.deleteIfExists(spool);
    }
  }

  @Override
  public void close() throws IOException {
    if (closed) return;
    closed = true;
    try {
      data.close();
      if (count == 0)
        throw new IOException("No valid dense points were reconstructed; no LAS/PLY was published");
      double quantization = .0001;
      for (int i = 0; i < 3; i++)
        quantization = Math.max(quantization, (max[i] - min[i]) / (2.0 * Integer.MAX_VALUE - 2));
      double[] offsets = new double[3];
      for (int i = 0; i < 3; i++) offsets[i] = min[i] + (max[i] - min[i]) / 2;
      Path las = directory.resolve("dense.las.part"), ply = directory.resolve("dense.ply.part");
      try (WorkProgress work = WorkProgress.start("Final LAS/PLY export", count, "points");
          OutputStream lasOut = new BufferedOutputStream(Files.newOutputStream(las), 1 << 20);
          OutputStream plyOut =
              this.ply
                  ? new BufferedOutputStream(Files.newOutputStream(ply), 1 << 20)
                  : OutputStream.nullOutputStream();
          DataInputStream in =
              new DataInputStream(new BufferedInputStream(Files.newInputStream(spool)))) {
        byte[] vlrs = crs == null ? new byte[0] : projectionVlr(crs.epsg);
        byte[] extras = attributes ? extraVlr() : new byte[0];
        lasOut.write(
            lasHeader(
                count,
                quantization,
                offsets,
                min,
                max,
                vlrs.length + extras.length,
                (crs == null ? 0 : 1) + (attributes ? 1 : 0),
                attributes ? SurfaceAttributes.BYTES : 0));
        lasOut.write(vlrs);
        lasOut.write(extras);
        plyOut.write(
            ("ply\n"
                    + "format binary_little_endian 1.0\n"
                    + "comment POSE " + PdfReport.VERSION + "; coordinates follow report.txt\n"
                    + (crs == null
                        ? ""
                        : "comment horizontal_crs "
                            + crs.name
                            + "; x=easting y=northing; Z=EXIF altitude, unspecified vertical datum,"
                            + " metres\n")
                    + "element vertex "
                    + count
                    + "\n"
                    + "property double x\n"
                    + "property double y\n"
                    + "property double z\n"
                    + "property uchar red\n"
                    + "property uchar green\n"
                    + "property uchar blue\n"
                    + (attributes ? plyAttributes() : "")
                    + "end_header\n")
                .getBytes(StandardCharsets.US_ASCII));
        ByteBuffer
            record =
                ByteBuffer.allocate(26 + (attributes ? SurfaceAttributes.BYTES : 0))
                    .order(ByteOrder.LITTLE_ENDIAN),
            vertex =
                ByteBuffer.allocate(27 + (attributes ? SurfaceAttributes.BYTES : 0))
                    .order(ByteOrder.LITTLE_ENDIAN);
        byte[] extra = attributes ? new byte[SurfaceAttributes.BYTES] : new byte[0];
        for (long p = 0; p < count; p++) {
          if ((p & 8191) == 0) work.update(p);
          double x = in.readDouble(), y = in.readDouble(), z = in.readDouble();
          int r = in.readUnsignedByte(), g = in.readUnsignedByte(), b = in.readUnsignedByte();
          record.clear();
          record
              .putInt(coordinate(x, offsets[0], quantization))
              .putInt(coordinate(y, offsets[1], quantization))
              .putInt(coordinate(z, offsets[2], quantization));
          record
              .putShort((short) 0)
              .put((byte) 9)
              .put((byte) 1)
              .put((byte) 0)
              .put((byte) 0)
              .putShort((short) 0);
          record
              .putShort((short) (r * 257))
              .putShort((short) (g * 257))
              .putShort((short) (b * 257));
          if (attributes) in.readFully(extra);
          record.put(extra);
          lasOut.write(record.array());
          if (this.ply) {
            vertex.clear();
            vertex.putDouble(x).putDouble(y).putDouble(z).put((byte) r).put((byte) g).put((byte) b);
            vertex.put(extra);
            plyOut.write(vertex.array());
          }
        }
        work.done();
      }
      publish(las, directory.resolve("dense.las"));
      if (this.ply) publish(ply, directory.resolve("dense.ply"));
      else Files.deleteIfExists(directory.resolve("dense.ply"));
    } finally {
      Files.deleteIfExists(spool);
    }
  }

  private static int coordinate(double value, double offset, double scale) throws IOException {
    long q = Math.round((value - offset) / scale);
    if (q < Integer.MIN_VALUE || q > Integer.MAX_VALUE)
      throw new IOException("LAS quantization overflow");
    return (int) q;
  }

  private static byte[] lasHeader(
      long count,
      double scale,
      double[] offsets,
      double[] min,
      double[] max,
      int vlrBytes,
      int vlrCount,
      int extraBytes) {
    ByteBuffer h = ByteBuffer.allocate(227).order(ByteOrder.LITTLE_ENDIAN);
    h.put("LASF".getBytes(StandardCharsets.US_ASCII));
    h.position(24);
    h.put((byte) 1).put((byte) 2);
    ascii(h, "LAS.utils photogrammetry", 32);
    ascii(h, "POSE " + PdfReport.VERSION, 32);
    LocalDate now = LocalDate.now();
    h.putShort((short) now.getDayOfYear()).putShort((short) now.getYear());
    h.putShort((short) 227)
        .putInt(227 + vlrBytes)
        .putInt(vlrCount)
        .put((byte) 2)
        .putShort((short) (26 + extraBytes))
        .putInt((int) count);
    h.putInt((int) count);
    for (int i = 0; i < 4; i++) h.putInt(0);
    for (int i = 0; i < 3; i++) h.putDouble(scale);
    for (double offset : offsets) h.putDouble(offset);
    for (int i = 0; i < 3; i++) {
      h.putDouble(max[i]);
      h.putDouble(min[i]);
    }
    return h.array();
  }

  private static String plyAttributes() {
    StringBuilder s =
        new StringBuilder(
            "comment surface_status: 0=valid,1=unreliable,2=normal_only\n"
                + "comment normals: source-depth fit, oriented toward source camera, not"
                + " final-cloud refit\n");
    for (int i = 0; i < SurfaceAttributes.NAMES.length; i++)
      s.append("property ")
          .append(i < 7 ? "float" : i == 7 ? "ushort" : "uchar")
          .append(' ')
          .append(i < 3 ? new String[] {"nx", "ny", "nz"}[i] : SurfaceAttributes.NAMES[i])
          .append('\n');
    return s.toString();
  }

  private static byte[] extraVlr() {
    // ASPRS LAS Extra Bytes record 4: 192-byte descriptors, float no-data upcast to double.
    ByteBuffer b =
        ByteBuffer.allocate(54 + 192 * SurfaceAttributes.NAMES.length)
            .order(ByteOrder.LITTLE_ENDIAN);
    b.putShort((short) 0);
    ascii(b, "LASF_Spec", 16);
    b.putShort((short) 4).putShort((short) (192 * SurfaceAttributes.NAMES.length));
    ascii(b, "Surface neighborhood attributes", 32);
    for (int i = 0; i < SurfaceAttributes.NAMES.length; i++) {
      int start = b.position();
      b.putShort((short) 0).put((byte) (i < 7 ? 9 : i == 7 ? 3 : 1)).put((byte) (i < 7 ? 1 : 0));
      ascii(b, SurfaceAttributes.NAMES[i], 32);
      b.putInt(0);
      if (i < 7) b.putDouble(Double.NaN);
      b.position(start + 160);
      ascii(b, SurfaceAttributes.DESCRIPTIONS[i], 32);
    }
    return b.array();
  }

  private static byte[] projectionVlr(int epsg) {
    // LAS 1.2 uses the GeoTIFF GeoKeyDirectoryTag (ASPRS specification, record 34735).
    ByteBuffer b = ByteBuffer.allocate(54 + 40).order(ByteOrder.LITTLE_ENDIAN);
    b.putShort((short) 0);
    ascii(b, "LASF_Projection", 16);
    b.putShort((short) 34735).putShort((short) 40);
    ascii(b, "Horizontal projected CRS", 32);
    b.putShort((short) 1).putShort((short) 1).putShort((short) 0).putShort((short) 4);
    for (int[] entry : new int[][] {{1024, 1}, {3072, epsg}, {3076, 9001}, {4099, 9001}})
      b.putShort((short) entry[0])
          .putShort((short) 0)
          .putShort((short) 1)
          .putShort((short) entry[1]);
    return b.array();
  }

  private static void ascii(ByteBuffer h, String text, int length) {
    byte[] b = text.getBytes(StandardCharsets.US_ASCII);
    h.put(b, 0, Math.min(b.length, length));
    for (int i = b.length; i < length; i++) h.put((byte) 0);
  }

  static void publish(Path source, Path target) throws IOException {
    try {
      Files.move(
          source, target, StandardCopyOption.ATOMIC_MOVE, StandardCopyOption.REPLACE_EXISTING);
    } catch (AtomicMoveNotSupportedException e) {
      Files.move(source, target, StandardCopyOption.REPLACE_EXISTING);
    }
  }
}
