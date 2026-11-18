package sgm;

import java.io.*;
import java.nio.*;
import java.nio.channels.FileChannel;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.time.LocalDate;
import java.util.*;

/**
 * Streams RGB LAS point format 2 and binary little-endian PLY without holding the cloud in memory.
 * LAS output is directly consumable by LAS.utils, with no dependency on its old OpenCV classpath.
 *
 * <p>One pass (since 14 November 2026): LAS records and PLY vertices go straight to their final
 * files and only the headers (count, bounds) are patched at the end. The LAS offsets are therefore
 * chosen from the first point (whole units; the lower-left corner of a tile) instead of the centre
 * of the bounds, with the fixed 0.1 mm quantization. Two rare cases fall back to a second pass:
 * a point farther than about 214 km (2^31 quantization steps) from the offset (the file is then
 * rewritten with centre offsets and an adaptive quantization, as before), and more than
 * 4,294,967,295 points, where the file is converted in place to LAS 1.4 (64-bit counts; point
 * format 2 is a legacy format that 1.4 still allows).
 *
 * <p>{@link #tiles(double, Path)} splits the LAS output into square tiles of a fixed size in
 * output units (--tiled-las-output); dense.las is then not written. The PLY stays one file.
 */
public final class CloudWriter implements AutoCloseable {
  static final long LAS12_POINT_LIMIT = 0xffffffffL;

  /** Point count above which LAS 1.4 is written; -Dsgm.cloud.las14Above=N lowers it for tests. */
  static long las14Above() {
    return Math.min(LAS12_POINT_LIMIT, Long.getLong("sgm.cloud.las14Above", LAS12_POINT_LIMIT));
  }
  static final double QUANTIZATION = .0001;
  static final int HEADER_12 = 227, HEADER_14 = 375;
  /** Open tile streams at most; further tiles are closed and reopened for appending. */
  static final int OPEN_TILES = 96;
  private static final int PLY_COUNT_DIGITS = 20;

  private final Path directory;
  private final double scale;
  private final Crs crs;
  private final double[] min = {
    Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY
  };
  private final double[] max = {
    Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY
  };
  private final boolean attributes;
  private final int lasRecordBytes, plyRecordBytes;
  private final ByteBuffer extraRecord =
      ByteBuffer.allocate(SurfaceAttributes.BYTES).order(ByteOrder.LITTLE_ENDIAN);
  private final ByteBuffer lasRecord, plyRecord;
  private final byte[] noExtra = new byte[0];
  private long count;
  private long spoolBufferBytes;
  private boolean closed, opened;
  private boolean ply = true;
  private OutputDiskSpace diskSpace;
  private double tileSize;
  private Path tileDirectory;
  private Las single;
  private final Map<Long, Las> tiles = new HashMap<>();
  private final LinkedHashMap<Las, Boolean> openTiles = new LinkedHashMap<>(16, .75f, true);
  private Path plyPart;
  private OutputStream plyOut;
  private byte[] vlrs;
  private int vlrCount;

  void diskSpace(OutputDiskSpace value) { diskSpace = value; }

  /** false: write only the LAS output (--ply off). The LAS bytes do not depend on it. */
  void writePly(boolean write) {
    if (opened) throw new IllegalStateException("PLY choice after the first point");
    ply = write;
  }

  /**
   * Square LAS tiles of {@code size} output units (after the unit scale), written to {@code
   * folder} as tile_X_Y.las (X, Y = lower-left corner) plus tiles.csv, instead of dense.las.
   */
  void tiles(double size, Path folder) {
    if (opened) throw new IllegalStateException("Tiling after the first point");
    if (!(size > 0) || !Double.isFinite(size))
      throw new IllegalArgumentException("Tile size must be positive");
    tileSize = size;
    tileDirectory = folder;
  }

  public CloudWriter(Path directory, double scale) throws IOException {
    this(directory, scale, null);
  }

  CloudWriter(Path directory, double scale, Crs crs) throws IOException {
    this(directory, scale, crs, false);
  }

  CloudWriter(Path directory, double scale, Crs crs, boolean attributes) throws IOException {
    this.attributes = attributes;
    if (crs != null && scale != 1)
      throw new IllegalArgumentException("A CRS-tagged cloud requires scale 1");
    this.directory = directory;
    this.scale = scale;
    this.crs = crs;
    int extra = attributes ? SurfaceAttributes.BYTES : 0;
    lasRecordBytes = 26 + extra;
    plyRecordBytes = 27 + extra;
    lasRecord = ByteBuffer.allocate(lasRecordBytes).order(ByteOrder.LITTLE_ENDIAN);
    plyRecord = ByteBuffer.allocate(plyRecordBytes).order(ByteOrder.LITTLE_ENDIAN);
    Files.createDirectories(directory);
  }

  private long pointBytes() {
    return lasRecordBytes + (ply ? plyRecordBytes : 0);
  }

  private void open() throws IOException {
    opened = true;
    byte[] projection = crs == null ? new byte[0] : projectionVlr(crs.epsg);
    byte[] extras = attributes ? extraVlr() : new byte[0];
    vlrs = new byte[projection.length + extras.length];
    System.arraycopy(projection, 0, vlrs, 0, projection.length);
    System.arraycopy(extras, 0, vlrs, projection.length, extras.length);
    vlrCount = (crs == null ? 0 : 1) + (attributes ? 1 : 0);
    if (tileDirectory != null) {
      Files.createDirectories(tileDirectory);
      try (var files = Files.list(tileDirectory)) {
        for (Path p : (Iterable<Path>) files::iterator) {
          String name = p.getFileName().toString();
          if (name.equals("tiles.csv") || name.startsWith("tile_") && (name.endsWith(".las") || name.endsWith(".las.part")))
            Files.deleteIfExists(p);
        }
      }
    } else single = new Las(directory.resolve("dense.las.part"), directory.resolve("dense.las"), false, 0, 0);
    if (ply) {
      plyPart = directory.resolve("dense.ply.part");
      plyOut = new BufferedOutputStream(Files.newOutputStream(plyPart), 1 << 20);
      plyOut.write(plyHeader(0));
    }
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
    if (diskSpace != null && count % OutputDiskSpace.BATCH == 0) {
      // Records go straight to their final files: nothing is left to export at the end.
      diskSpace.exports(0, count);
      diskSpace.require(OutputDiskSpace.BATCH * pointBytes(), "dense cloud output");
    }
    if (!opened) open();
    byte[] extra = noExtra;
    if (attributes) {
      extraRecord.clear();
      surface.binary(extraRecord, normal, scale);
      extra = extraRecord.array();
    }
    min[0] = Math.min(min[0], x); max[0] = Math.max(max[0], x);
    min[1] = Math.min(min[1], y); max[1] = Math.max(max[1], y);
    min[2] = Math.min(min[2], z); max[2] = Math.max(max[2], z);
    Las target = single != null ? single : tile(x, y);
    target.add(x, y, z, red, green, blue, extra);
    if (ply) {
      plyRecord.clear();
      plyRecord.putDouble(x).putDouble(y).putDouble(z).put((byte) red).put((byte) green).put((byte) blue);
      plyRecord.put(extra);
      plyOut.write(plyRecord.array());
    }
    count++;
  }

  private Las tile(double x, double y) throws IOException {
    double fx = Math.floor(x / tileSize), fy = Math.floor(y / tileSize);
    if (!(Math.abs(fx) < Integer.MAX_VALUE && Math.abs(fy) < Integer.MAX_VALUE))
      throw new IOException("Tile index out of range; use a larger --tiled-las-output");
    int col = (int) fx, row = (int) fy;
    long key = ((long) col << 32) | (row & 0xffffffffL);
    Las las = tiles.get(key);
    if (las == null) {
      String name = "tile_" + number(col * tileSize) + "_" + number(row * tileSize) + ".las";
      las = new Las(tileDirectory.resolve(name + ".part"), tileDirectory.resolve(name), true, col, row);
      tiles.put(key, las);
    }
    return las;
  }

  private static String number(double v) {
    if (v == Math.rint(v) && Math.abs(v) < 1e15) return Long.toString((long) v);
    return java.math.BigDecimal.valueOf(v).stripTrailingZeros().toPlainString();
  }

  public long count() {
    return count;
  }

  /** Conservative retained tile objects plus any overflow-spool buffers. */
  long retainedMetadataBytes() { return 4096L * tiles.size() + spoolBufferBytes; }

  public void abort() throws IOException {
    if (closed) return;
    closed = true;
    IOException failure = null;
    List<Las> all = new ArrayList<>(tiles.values());
    if (single != null) all.add(single);
    for (Las las : all)
      try {
        las.discard();
      } catch (IOException e) {
        if (failure == null) failure = e; else failure.addSuppressed(e);
      }
    try {
      if (plyOut != null) plyOut.close();
    } catch (IOException e) {
      if (failure == null) failure = e; else failure.addSuppressed(e);
    } finally {
      if (plyPart != null) Files.deleteIfExists(plyPart);
      if (diskSpace != null) diskSpace.exports(0, 0);
    }
    if (failure != null) throw failure;
  }

  @Override
  public void close() throws IOException {
    if (closed) return;
    boolean done = false;
    try {
      if (count == 0)
        throw new IOException("No valid dense points were reconstructed; no LAS/PLY was published");
      if (diskSpace != null) {
        diskSpace.exports(0, count);
        diskSpace.require(0, "final LAS/PLY export");
      }
      try (WorkProgress work = WorkProgress.start("Final LAS/PLY headers", count, "points")) {
        if (ply) {
          plyOut.close();
          plyOut = null;
          try (FileChannel ch = FileChannel.open(plyPart, StandardOpenOption.WRITE)) {
            write(ch, ByteBuffer.wrap(plyHeader(count)), 0);
          }
        }
        List<Las> all = new ArrayList<>(tiles.values());
        all.sort(Comparator.comparingInt((Las t) -> t.col).thenComparingInt(t -> t.row));
        if (single != null) all.add(single);
        for (Las las : all) las.finish();
        work.done();
      }
      closed = true;
      for (Las las : tiles.values()) publish(las.part, las.target);
      if (tileDirectory != null) {
        writeTileIndex();
        Files.deleteIfExists(directory.resolve("dense.las"));
      }
      if (single != null) publish(single.part, single.target);
      if (ply) publish(plyPart, directory.resolve("dense.ply"));
      else Files.deleteIfExists(directory.resolve("dense.ply"));
      done = true;
      // The run catalogue needs a SHA-256 of each cloud file; compute it while later work runs.
      for (Las las : tiles.values()) ArtifactDigests.start(las.target);
      if (single != null) ArtifactDigests.start(single.target);
      if (ply) ArtifactDigests.start(directory.resolve("dense.ply"));
    } finally {
      if (!done) {
        closed = false;
        abort();
      }
      if (diskSpace != null) diskSpace.exports(0, 0);
    }
  }

  /** Tiles in the order of their names' coordinates, with their bounds and LAS version. */
  private void writeTileIndex() throws IOException {
    List<Las> all = new ArrayList<>(tiles.values());
    all.sort(Comparator.comparingInt((Las t) -> t.row).thenComparingInt(t -> t.col));
    StringBuilder s =
        new StringBuilder(
            "file,tile_min_x,tile_min_y,tile_size,points,las_version,min_x,min_y,min_z,max_x,max_y,max_z\n");
    for (Las t : all)
      s.append(t.target.getFileName())
          .append(',').append(number(t.col * tileSize))
          .append(',').append(number(t.row * tileSize))
          .append(',').append(number(tileSize))
          .append(',').append(t.count)
          .append(',').append(t.count > las14Above() ? "1.4" : "1.2")
          .append(String.format(Locale.ROOT, ",%.4f,%.4f,%.4f,%.4f,%.4f,%.4f%n",
              t.min[0], t.min[1], t.min[2], t.max[0], t.max[1], t.max[2]));
    Path part = tileDirectory.resolve("tiles.csv.part");
    Files.writeString(part, s, StandardCharsets.US_ASCII);
    publish(part, tileDirectory.resolve("tiles.csv"));
  }

  /** One LAS output file: the whole cloud or one tile. */
  private final class Las {
    final Path part, target;
    final boolean tile;
    final int col, row;
    final double[] offset = new double[3];
    final double[] min = {
      Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY
    };
    final double[] max = {
      Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY
    };
    long count, direct;
    OutputStream out;
    // Fallback after a coordinate outside the fixed offsets' range: every later point is kept
    // here (doubles, as the old spool) and the file is rewritten once at the end.
    Path spool;
    DataOutputStream spoolOut;

    Las(Path part, Path target, boolean tile, int col, int row) {
      this.part = part;
      this.target = target;
      this.tile = tile;
      this.col = col;
      this.row = row;
    }

    private OutputStream stream() throws IOException {
      if (out != null) {
        if (tile) openTiles.get(this);
        return out;
      }
      if (tile) {
        while (openTiles.size() >= OPEN_TILES) {
          var eldest = openTiles.keySet().iterator().next();
          openTiles.remove(eldest);
          eldest.out.close();
          eldest.out = null;
        }
        openTiles.put(this, Boolean.TRUE);
      }
      boolean fresh = count == 0;
      out =
          new BufferedOutputStream(
              fresh
                  ? Files.newOutputStream(part)
                  : Files.newOutputStream(part, StandardOpenOption.APPEND),
              tile ? 1 << 16 : 1 << 20);
      return out;
    }

    void add(double x, double y, double z, int r, int g, int b, byte[] extra) throws IOException {
      if (count == 0) {
        if (tile) {
          offset[0] = col * tileSize;
          offset[1] = row * tileSize;
        } else {
          offset[0] = Math.round(x);
          offset[1] = Math.round(y);
        }
        offset[2] = Math.round(z);
        OutputStream o = stream();
        o.write(new byte[HEADER_12]);
        o.write(vlrs);
      }
      min[0] = Math.min(min[0], x); max[0] = Math.max(max[0], x);
      min[1] = Math.min(min[1], y); max[1] = Math.max(max[1], y);
      min[2] = Math.min(min[2], z); max[2] = Math.max(max[2], z);
      if (spoolOut == null) {
        long qx = Math.round((x - offset[0]) / QUANTIZATION),
            qy = Math.round((y - offset[1]) / QUANTIZATION),
            qz = Math.round((z - offset[2]) / QUANTIZATION);
        if (fits(qx) && fits(qy) && fits(qz)) {
          record(lasRecord, (int) qx, (int) qy, (int) qz, r, g, b, extra);
          stream().write(lasRecord.array());
          direct++;
          count++;
          return;
        }
        spool = Files.createTempFile(part.getParent(), "cloud-", ".bin");
        spoolOut = new DataOutputStream(new BufferedOutputStream(Files.newOutputStream(spool), 1 << 20));
        spoolBufferBytes += 1L << 20;
      }
      spoolOut.writeDouble(x);
      spoolOut.writeDouble(y);
      spoolOut.writeDouble(z);
      spoolOut.write(r);
      spoolOut.write(g);
      spoolOut.write(b);
      spoolOut.write(extra);
      count++;
    }

    void finish() throws IOException {
      if (out != null) {
        out.close();
        out = null;
        openTiles.remove(this);
      }
      if (spoolOut != null) {
        spoolOut.close();
        rewrite();
        return;
      }
      long data = HEADER_12 + (long) vlrs.length;
      try (FileChannel ch = FileChannel.open(part, StandardOpenOption.READ, StandardOpenOption.WRITE)) {
        if (count > las14Above()) {
          if (diskSpace != null) diskSpace.require(HEADER_14 - HEADER_12, "LAS 1.4 header");
          shift(ch, HEADER_12, ch.size() - HEADER_12, HEADER_14 - HEADER_12);
        }
        write(ch, ByteBuffer.wrap(lasHeader(count, QUANTIZATION, offset, min, max, vlrs.length,
            vlrCount, lasRecordBytes - 26, "POSE " + PdfReport.VERSION)), 0);
        if (count <= las14Above() && ch.size() != data + count * lasRecordBytes)
          throw new IOException("LAS size mismatch: " + part);
      }
    }

    /** Second pass, only after a coordinate overflow: centre offsets, adaptive quantization. */
    private void rewrite() throws IOException {
      double quantization = QUANTIZATION;
      for (int i = 0; i < 3; i++)
        quantization = Math.max(quantization, (max[i] - min[i]) / (2.0 * Integer.MAX_VALUE - 2));
      double[] centre = new double[3];
      for (int i = 0; i < 3; i++) centre[i] = min[i] + (max[i] - min[i]) / 2;
      Path again = part.resolveSibling(part.getFileName() + ".rewrite");
      if (diskSpace != null)
        diskSpace.require(Files.size(part) + Files.size(spool), "LAS rewrite after a far point");
      System.out.println(
          "Cloud: a point lies more than 2^31 quantization steps from the LAS offset of "
              + target.getFileName() + "; rewriting it once with centred offsets");
      try (OutputStream o = new BufferedOutputStream(Files.newOutputStream(again), 1 << 20);
          DataInputStream records =
              new DataInputStream(new BufferedInputStream(Files.newInputStream(part), 1 << 20));
          DataInputStream rest =
              new DataInputStream(new BufferedInputStream(Files.newInputStream(spool), 1 << 20))) {
        o.write(lasHeader(count, quantization, centre, min, max, vlrs.length, vlrCount,
            lasRecordBytes - 26, "POSE " + PdfReport.VERSION));
        o.write(vlrs);
        records.skipNBytes(HEADER_12 + (long) vlrs.length);
        ByteBuffer in = ByteBuffer.allocate(lasRecordBytes).order(ByteOrder.LITTLE_ENDIAN);
        for (long p = 0; p < direct; p++) {
          records.readFully(in.array());
          for (int k = 0; k < 3; k++) {
            double v = in.getInt(4 * k) * QUANTIZATION + offset[k];
            in.putInt(4 * k, coordinate(v, centre[k], quantization));
          }
          o.write(in.array());
        }
        byte[] extra = new byte[lasRecordBytes - 26];
        for (long p = direct; p < count; p++) {
          double x = rest.readDouble(), y = rest.readDouble(), z = rest.readDouble();
          int r = rest.readUnsignedByte(), g = rest.readUnsignedByte(), b = rest.readUnsignedByte();
          rest.readFully(extra);
          record(lasRecord, coordinate(x, centre[0], quantization), coordinate(y, centre[1], quantization),
              coordinate(z, centre[2], quantization), r, g, b, extra);
          o.write(lasRecord.array());
        }
      }
      Files.move(again, part, StandardCopyOption.REPLACE_EXISTING);
      Files.deleteIfExists(spool);
      spool = null;
      spoolOut = null;
    }

    void discard() throws IOException {
      try {
        if (out != null) out.close();
        if (spoolOut != null) spoolOut.close();
      } finally {
        out = null;
        spoolOut = null;
        openTiles.remove(this);
        Files.deleteIfExists(part);
        Files.deleteIfExists(part.resolveSibling(part.getFileName() + ".rewrite"));
        if (spool != null) Files.deleteIfExists(spool);
      }
    }
  }

  private static boolean fits(long q) {
    return q >= Integer.MIN_VALUE && q <= Integer.MAX_VALUE;
  }

  private static void record(
      ByteBuffer record, int x, int y, int z, int r, int g, int b, byte[] extra) {
    record.clear();
    record.putInt(x).putInt(y).putInt(z);
    record
        .putShort((short) 0)
        .put((byte) 9)
        .put((byte) 1)
        .put((byte) 0)
        .put((byte) 0)
        .putShort((short) 0);
    record.putShort((short) (r * 257)).putShort((short) (g * 257)).putShort((short) (b * 257));
    record.put(extra);
  }

  private static void write(FileChannel ch, ByteBuffer b, long at) throws IOException {
    while (b.hasRemaining()) at += ch.write(b, at);
  }

  /** Moves bytes [from, from + length) forward by {@code by}, last block first, in place. */
  static void shift(FileChannel ch, long from, long length, int by) throws IOException {
    ByteBuffer b = ByteBuffer.allocateDirect(8 << 20);
    long end = from + length;
    while (end > from) {
      int n = (int) Math.min(b.capacity(), end - from);
      long start = end - n;
      b.clear().limit(n);
      while (b.hasRemaining()) {
        int got = ch.read(b, start + b.position());
        if (got < 0) throw new EOFException("LAS shift");
      }
      b.flip();
      write(ch, b, start + by);
      end = start;
    }
  }

  private byte[] plyHeader(long points) {
    String number = Long.toString(points);
    String header =
        "ply\n"
            + "format binary_little_endian 1.0\n"
            + "comment POSE " + PdfReport.VERSION + "; coordinates follow report.txt\n"
            + (crs == null
                ? ""
                : "comment horizontal_crs "
                    + crs.name
                    + "; x=easting y=northing; Z=EXIF altitude, unspecified vertical datum,"
                    + " metres\n")
            // Written before the count is known and patched in place: this line keeps the
            // header length the same for any count.
            + "comment reserved" + " ".repeat(PLY_COUNT_DIGITS - number.length()) + "\n"
            + "element vertex "
            + number
            + "\n"
            + "property double x\n"
            + "property double y\n"
            + "property double z\n"
            + "property uchar red\n"
            + "property uchar green\n"
            + "property uchar blue\n"
            + (attributes ? plyAttributes() : "")
            + "end_header\n";
    return header.getBytes(StandardCharsets.US_ASCII);
  }

  private static int coordinate(double value, double offset, double scale) throws IOException {
    long q = Math.round((value - offset) / scale);
    if (q < Integer.MIN_VALUE || q > Integer.MAX_VALUE)
      throw new IOException("LAS quantization overflow");
    return (int) q;
  }

  /**
   * LAS 1.2 header (227 bytes) up to 4,294,967,295 points, else LAS 1.4 (375 bytes, legacy counts
   * zero, 64-bit counts). Point format 2 and the same VLRs in both.
   */
  static byte[] lasHeader(
      long count,
      double scale,
      double[] offsets,
      double[] min,
      double[] max,
      int vlrBytes,
      int vlrCount,
      int extraBytes,
      String software) {
    boolean v14 = count > las14Above();
    int size = v14 ? HEADER_14 : HEADER_12;
    ByteBuffer h = ByteBuffer.allocate(size).order(ByteOrder.LITTLE_ENDIAN);
    h.put("LASF".getBytes(StandardCharsets.US_ASCII));
    h.position(24);
    h.put((byte) 1).put((byte) (v14 ? 4 : 2));
    ascii(h, "LAS.utils photogrammetry", 32);
    ascii(h, software, 32);
    LocalDate now = LocalDate.now();
    h.putShort((short) now.getDayOfYear()).putShort((short) now.getYear());
    int legacy = v14 ? 0 : (int) count;
    h.putShort((short) size)
        .putInt(size + vlrBytes)
        .putInt(vlrCount)
        .put((byte) 2)
        .putShort((short) (26 + extraBytes))
        .putInt(legacy);
    h.putInt(legacy);
    for (int i = 0; i < 4; i++) h.putInt(0);
    for (int i = 0; i < 3; i++) h.putDouble(scale);
    for (double offset : offsets) h.putDouble(offset);
    for (int i = 0; i < 3; i++) {
      h.putDouble(max[i]);
      h.putDouble(min[i]);
    }
    if (v14) {
      h.putLong(0); // start of waveform data packet record
      h.putLong(0); // start of first extended VLR
      h.putInt(0); // number of extended VLRs
      h.putLong(count);
      h.putLong(count); // all points are single returns
      for (int i = 1; i < 15; i++) h.putLong(0);
    }
    return h.array();
  }

  /**
   * Point count of a LAS header read from byte 0 (at least 255 bytes for LAS 1.4 files): the
   * 64-bit count of LAS 1.4 when present, else the legacy 32-bit one.
   */
  static long lasPointCount(ByteBuffer h) {
    long legacy = h.getInt(107) & 0xffffffffL;
    if (h.get(25) >= 4 && (h.getShort(94) & 0xffff) >= HEADER_14 && h.limit() >= 255) {
      long extended = h.getLong(247);
      if (extended > 0) return extended;
    }
    return legacy;
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
    // GeoTIFF GeoKeyDirectoryTag (ASPRS specification, record 34735); LAS 1.4 allows it for the
    // legacy point formats 0-5.
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
