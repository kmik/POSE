package sgm;

import java.io.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.FileChannel;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.*;

/**
 * Reading LAS (1.0-1.4, point formats 0-10) and LAZ (LASzip, see {@link LazDecoder}) files through
 * one interface: {@link #open} gives a {@link Reader} that streams raw LAS point records in blocks,
 * never the whole file, whichever the encoding. The record accessors ({@link #x}, {@link
 * #classification} ...) read those raw records, so LAS and LAZ readers return identical bytes for the
 * same points. Also the coordinate reference system from the GeoTIFF keys or the WKT (E)VLR.
 */
final class LasFile {
  private LasFile() {}

  /** Base record sizes of point formats 0-10. */
  static final int[] BASE = {20, 28, 26, 34, 57, 63, 30, 36, 38, 59, 67};

  /** CRS declared in the file: EPSG codes (0 = not declared), the WKT text if any, and a source note. */
  record CrsInfo(int horizontal, int vertical, String wkt, String source) {
    boolean declared() {
      return horizontal > 0;
    }
  }

  record Header(
      Path path,
      int versionMajor,
      int versionMinor,
      int format,
      int recordLength,
      long count,
      long dataOffset,
      double[] scale,
      double[] offset,
      double[] min,
      double[] max,
      boolean compressed,
      LazDecoder.Vlr laz,
      CrsInfo crs,
      String software,
      int globalEncoding) {
    String version() {
      return versionMajor + "." + versionMinor;
    }

    double x(int raw) {
      return raw * scale[0] + offset[0];
    }

    double y(int raw) {
      return raw * scale[1] + offset[1];
    }

    double z(int raw) {
      return raw * scale[2] + offset[2];
    }
  }

  /** Streams raw point records. */
  interface Reader extends Closeable {
    Header header();

    /**
     * Reads up to {@code max} records into {@code buffer} (record i at {@code i *
     * header().recordLength()}); returns how many, 0 at the end.
     */
    int read(byte[] buffer, int max) throws IOException;

    @Override
    void close() throws IOException;
  }

  static boolean isPointFile(Path p) {
    String n = p.getFileName().toString().toLowerCase(Locale.ROOT);
    return Files.isRegularFile(p) && (n.endsWith(".las") || n.endsWith(".laz"));
  }

  /** A file, or every .las/.laz in a folder (not recursive), sorted by name. */
  static List<Path> files(Path fileOrFolder) throws IOException {
    if (Files.isDirectory(fileOrFolder)) {
      List<Path> out = new ArrayList<>();
      try (var s = Files.list(fileOrFolder)) {
        s.filter(LasFile::isPointFile).sorted().forEach(out::add);
      }
      if (out.isEmpty()) throw new IOException("No .las or .laz files in " + fileOrFolder);
      return out;
    }
    if (!Files.isRegularFile(fileOrFolder)) throw new IOException("Lidar reference not found: " + fileOrFolder);
    if (!isPointFile(fileOrFolder)) throw new IOException("Not a .las or .laz file: " + fileOrFolder);
    return List.of(fileOrFolder);
  }

  static Header header(Path path) throws IOException {
    try (FileChannel ch = FileChannel.open(path, StandardOpenOption.READ)) {
      return header(path, ch);
    }
  }

  private static byte[] bytes(FileChannel ch, long at, int length) throws IOException {
    ByteBuffer b = ByteBuffer.allocate(length);
    while (b.hasRemaining()) {
      int n = ch.read(b, at + b.position());
      if (n < 0) throw new EOFException("Unexpected end of file at byte " + (at + b.position()));
    }
    return b.array();
  }

  static Header header(Path path, FileChannel ch) throws IOException {
    if (ch.size() < 227) throw new IOException("Not a LAS/LAZ file (too short): " + path);
    byte[] h0 = bytes(ch, 0, (int) Math.min(375, ch.size()));
    ByteBuffer h = ByteBuffer.wrap(h0).order(ByteOrder.LITTLE_ENDIAN);
    if (h0[0] != 'L' || h0[1] != 'A' || h0[2] != 'S' || h0[3] != 'F') throw new IOException("Not a LAS/LAZ file: " + path);
    int major = h0[24] & 255, minor = h0[25] & 255;
    if (major != 1 || minor > 4) throw new IOException("Unsupported LAS version " + major + "." + minor + ": " + path);
    int headerSize = h.getShort(94) & 0xffff;
    long dataOffset = h.getInt(96) & 0xffffffffL;
    long vlrCount = h.getInt(100) & 0xffffffffL;
    int formatByte = h0[104] & 255;
    boolean compressed = (formatByte & 0x80) != 0;
    int format = formatByte & 0x3f;
    int recordLength = h.getShort(105) & 0xffff;
    long count = h.getInt(107) & 0xffffffffL;
    if (minor >= 4 && headerSize >= 375 && h0.length >= 255) {
      long count14 = h.getLong(247);
      if (count14 > 0) count = count14;
    }
    if (format > 10) throw new IOException("Unsupported LAS point format " + format + ": " + path);
    if (recordLength < BASE[format])
      throw new IOException("Record length " + recordLength + " is shorter than point format " + format + " needs: " + path);
    double[] scale = {h.getDouble(131), h.getDouble(139), h.getDouble(147)};
    double[] offset = {h.getDouble(155), h.getDouble(163), h.getDouble(171)};
    double[] max = {h.getDouble(179), h.getDouble(195), h.getDouble(211)};
    double[] min = {h.getDouble(187), h.getDouble(203), h.getDouble(219)};
    String software = new String(h0, 58, 32, StandardCharsets.US_ASCII).replace("\0", "").trim();
    boolean wktFlag = (h.getShort(6) & 16) != 0;
    // VLRs (and EVLRs of LAS 1.4).
    LazDecoder.Vlr laz = null;
    int[] keys = null;
    double[] doubleParams = null;
    String wkt = null, wktSource = null;
    long at = headerSize;
    for (long v = 0; v < vlrCount && at + 54 <= dataOffset; v++) {
      byte[] vh = bytes(ch, at, 54);
      String user = new String(vh, 2, 16, StandardCharsets.US_ASCII).replace("\0", "").trim();
      int record = (vh[18] & 255) | (vh[19] & 255) << 8, length = (vh[20] & 255) | (vh[21] & 255) << 8;
      byte[] body = length == 0 ? new byte[0] : bytes(ch, at + 54, length);
      if (user.equals("laszip encoded") && record == 22204) laz = LazDecoder.Vlr.parse(body);
      else if (user.equals("LASF_Projection") && record == 34735) keys = shorts(body);
      else if (user.equals("LASF_Projection") && record == 34736) doubleParams = doubles(body);
      else if (user.equals("LASF_Projection") && record == 2112) {
        wkt = cString(body);
        wktSource = "WKT VLR";
      }
      at += 54 + length;
    }
    if (minor >= 4 && h0.length >= 247) {
      long evlrStart = h.getLong(235), evlrs = h.getInt(243) & 0xffffffffL;
      long e = evlrStart;
      for (long v = 0; v < evlrs && e > 0 && e + 60 <= ch.size(); v++) {
        byte[] eh = bytes(ch, e, 60);
        ByteBuffer ebb = ByteBuffer.wrap(eh).order(ByteOrder.LITTLE_ENDIAN);
        String user = new String(eh, 2, 16, StandardCharsets.US_ASCII).replace("\0", "").trim();
        int record = ebb.getShort(18) & 0xffff;
        long length = ebb.getLong(20);
        if (user.equals("LASF_Projection") && record == 2112 && length < 1 << 20) {
          wkt = cString(bytes(ch, e + 60, (int) length));
          wktSource = "WKT EVLR";
        }
        e += 60 + length;
      }
    }
    if (compressed && laz == null) throw new IOException("Compressed LAZ without its laszip VLR: " + path);
    if (compressed) {
      int size = laz.check();
      if (size != recordLength)
        throw new IOException("LAZ items (" + size + " bytes) do not match the record length " + recordLength + ": " + path);
    } else if (laz != null && !compressed) laz = null;
    CrsInfo crs = crs(keys, wkt, wktSource, wktFlag);
    return new Header(path, major, minor, format, recordLength, count, dataOffset, scale, offset, min, max, compressed, laz, crs, software, h.getShort(6) & 0xffff);
  }

  private static int[] shorts(byte[] b) {
    int[] out = new int[b.length / 2];
    for (int i = 0; i < out.length; i++) out[i] = (b[2 * i] & 255) | (b[2 * i + 1] & 255) << 8;
    return out;
  }

  private static double[] doubles(byte[] b) {
    ByteBuffer bb = ByteBuffer.wrap(b).order(ByteOrder.LITTLE_ENDIAN);
    double[] out = new double[b.length / 8];
    for (int i = 0; i < out.length; i++) out[i] = bb.getDouble(8 * i);
    return out;
  }

  private static String cString(byte[] b) {
    int n = 0;
    while (n < b.length && b[n] != 0) n++;
    return new String(b, 0, n, StandardCharsets.UTF_8).trim();
  }

  /** EPSG codes from GeoTIFF keys (3072 projected, 4096 vertical) or the WKT's top-level authority codes. */
  static CrsInfo crs(int[] keys, String wkt, String wktSource, boolean wktFlag) {
    int horizontal = 0, vertical = 0;
    String source = null;
    if (keys != null && keys.length >= 4) {
      int n = keys[3];
      for (int i = 0; i < n && 4 + 4 * i + 3 < keys.length; i++) {
        int id = keys[4 + 4 * i], location = keys[4 + 4 * i + 1], value = keys[4 + 4 * i + 3];
        if (location != 0) continue; // only directly stored SHORT values
        if (id == 3072 && value > 0 && value < 32767) horizontal = value;
        if (id == 4096 && value > 0 && value < 32767) vertical = value;
      }
      if (horizontal > 0) source = "GeoTIFF keys";
    }
    if (wkt != null && !wkt.isEmpty()) {
      int[] codes = wktCodes(wkt);
      if (horizontal == 0 && codes[0] > 0) {
        horizontal = codes[0];
        source = wktSource;
      }
      if (vertical == 0 && codes[1] > 0) vertical = codes[1];
      if (source == null) source = wktSource;
    }
    return new CrsInfo(horizontal, vertical, wkt, source == null ? (wktFlag ? "WKT flag set, no CRS found" : "none") : source);
  }

  /**
   * {projected EPSG, vertical EPSG} from WKT1 (PROJCS, VERT_CS, COMPD_CS with AUTHORITY) or WKT2
   * (PROJCRS, VERTCRS, COMPOUNDCRS with ID): the authority that is a direct child of the node.
   */
  static int[] wktCodes(String wkt) {
    int[] out = new int[2];
    scan(wkt, 0, out, null);
    return out;
  }

  /** Parses node at {@code at} (NAME[ ... ]); returns the index after it. */
  private static int scan(String s, int at, int[] out, String parent) {
    int i = at;
    while (i < s.length() && Character.isWhitespace(s.charAt(i))) i++;
    int nameStart = i;
    while (i < s.length() && (Character.isLetterOrDigit(s.charAt(i)) || s.charAt(i) == '_')) i++;
    String name = s.substring(nameStart, i).toUpperCase(Locale.ROOT);
    while (i < s.length() && Character.isWhitespace(s.charAt(i))) i++;
    if (i >= s.length() || s.charAt(i) != '[' && s.charAt(i) != '(') return i;
    char close = s.charAt(i) == '[' ? ']' : ')';
    i++;
    List<String> values = new ArrayList<>();
    while (i < s.length()) {
      while (i < s.length() && (Character.isWhitespace(s.charAt(i)) || s.charAt(i) == ',')) i++;
      if (i >= s.length()) break;
      char c = s.charAt(i);
      if (c == close) {
        i++;
        break;
      }
      if (c == '"') {
        int end = s.indexOf('"', i + 1);
        while (end >= 0 && end + 1 < s.length() && s.charAt(end + 1) == '"') end = s.indexOf('"', end + 2);
        if (end < 0) return s.length();
        values.add(s.substring(i + 1, end));
        i = end + 1;
      } else if (Character.isLetter(c)) {
        i = scan(s, i, out, name);
      } else {
        int start = i;
        while (i < s.length() && s.charAt(i) != ',' && s.charAt(i) != close) i++;
        values.add(s.substring(start, i).trim());
      }
    }
    if ((name.equals("AUTHORITY") || name.equals("ID")) && values.size() >= 2 && values.get(0).equalsIgnoreCase("EPSG") && parent != null) {
      int code;
      try {
        code = Integer.parseInt(values.get(1).trim());
      } catch (NumberFormatException e) {
        return i;
      }
      if (Set.of("PROJCS", "PROJCRS", "PROJECTEDCRS").contains(parent) && out[0] == 0) out[0] = code;
      if (Set.of("VERT_CS", "VERTCS", "VERTCRS", "VERTICALCRS").contains(parent) && out[1] == 0) out[1] = code;
    }
    return i;
  }

  /** Names for common vertical EPSG codes (for messages and reports). */
  static String verticalName(int epsg) {
    return switch (epsg) {
      case 3900 -> "EPSG:3900 (N2000 height)";
      case 5717 -> "EPSG:5717 (N60 height)";
      case 5703 -> "EPSG:5703 (NAVD88 height)";
      case 5941 -> "EPSG:5941 (NN2000 height)";
      case 5613 -> "EPSG:5613 (RH2000 height)";
      case 5799 -> "EPSG:5799 (DVR90 height)";
      case 7837 -> "EPSG:7837 (DHHN2016 height)";
      case 5709 -> "EPSG:5709 (NAP height)";
      case 3855 -> "EPSG:3855 (EGM2008 height)";
      case 5773 -> "EPSG:5773 (EGM96 height)";
      default -> epsg > 0 ? "EPSG:" + epsg : null;
    };
  }

  // ---- Readers.

  static Reader open(Path path) throws IOException {
    FileChannel ch = FileChannel.open(path, StandardOpenOption.READ);
    try {
      Header h = header(path, ch);
      return h.compressed ? new Laz(h, ch) : new Plain(h, ch);
    } catch (IOException | RuntimeException e) {
      ch.close();
      throw e;
    }
  }

  private static final class Plain implements Reader {
    final Header h;
    final FileChannel ch;
    long next;

    Plain(Header h, FileChannel ch) throws IOException {
      this.h = h;
      this.ch = ch;
      long needed = h.dataOffset + h.count * h.recordLength;
      if (ch.size() < needed)
        throw new IOException("LAS file is truncated: " + h.count + " points need " + needed + " bytes, file has " + ch.size() + ": " + h.path);
    }

    public Header header() {
      return h;
    }

    public int read(byte[] buffer, int max) throws IOException {
      int n = (int) Math.min(max, h.count - next);
      if (n <= 0) return 0;
      ByteBuffer b = ByteBuffer.wrap(buffer, 0, n * h.recordLength);
      long at = h.dataOffset + next * h.recordLength;
      while (b.hasRemaining()) if (ch.read(b, at + b.position()) < 0) throw new EOFException(h.path.toString());
      next += n;
      return n;
    }

    public void close() throws IOException {
      ch.close();
    }
  }

  private static final class Laz implements Reader {
    final Header h;
    final FileChannel ch;
    final long[][] chunks;
    final long[] chunkStart;
    int chunk;
    long pointsDone;
    byte[] decoded = new byte[0];
    int decodedCount, decodedNext;

    Laz(Header h, FileChannel ch) throws IOException {
      this.h = h;
      this.ch = ch;
      LazDecoder.Vlr vlr = h.laz;
      if (vlr.compressor() == 1) {
        // Point-wise without chunks: one stream to the end of the file.
        chunks = new long[][] {{h.count, ch.size() - h.dataOffset}};
        chunkStart = new long[] {h.dataOffset};
        return;
      }
      long tableAt = ByteBuffer.wrap(bytes(ch, h.dataOffset, 8)).order(ByteOrder.LITTLE_ENDIAN).getLong();
      if (tableAt <= h.dataOffset) {
        tableAt = ByteBuffer.wrap(bytes(ch, ch.size() - 8, 8)).order(ByteOrder.LITTLE_ENDIAN).getLong();
        if (tableAt <= h.dataOffset)
          throw new IOException("LAZ file has no chunk table (written by a streaming writer?); rewrite it with laszip: " + h.path);
      }
      if (tableAt + 8 > ch.size()) throw new IOException("LAZ chunk table offset is outside the file: " + h.path);
      long tableLength = ch.size() - tableAt;
      byte[] table = bytes(ch, tableAt, (int) Math.min(tableLength, 64L << 20));
      chunks = LazDecoder.chunkTable(table, vlr.variableChunks());
      chunkStart = new long[chunks.length];
      long at = h.dataOffset + 8, remaining = h.count;
      for (int i = 0; i < chunks.length; i++) {
        chunkStart[i] = at;
        at += chunks[i][1];
        if (!vlr.variableChunks()) chunks[i][0] = Math.min(vlr.chunkSize() & 0xffffffffL, remaining);
        remaining -= chunks[i][0];
      }
      if (at > tableAt) throw new IOException("LAZ chunk table does not fit the file: " + h.path);
    }

    public Header header() {
      return h;
    }

    public int read(byte[] buffer, int max) throws IOException {
      int done = 0;
      int rl = h.recordLength;
      while (done < max && pointsDone < h.count) {
        if (decodedNext >= decodedCount) {
          if (chunk >= chunks.length) throw new IOException("LAZ: fewer points in the chunks than the header says: " + h.path);
          long points = chunks[chunk][0], bytes = chunks[chunk][1];
          if (points > Integer.MAX_VALUE / Math.max(1, rl) || bytes > Integer.MAX_VALUE)
            throw new IOException("LAZ chunk too large: " + h.path);
          byte[] src = bytes(ch, chunkStart[chunk], (int) bytes);
          int np = (int) Math.min(points, h.count - pointsDone);
          if (decoded.length < np * rl) decoded = new byte[np * rl];
          try {
            LazDecoder.chunk(h.laz, src, 0, src.length, np, decoded, rl);
          } catch (RuntimeException e) {
            throw new IOException("LAZ: corrupt chunk " + chunk + " in " + h.path + ": " + e, e);
          }
          decodedCount = np;
          decodedNext = 0;
          chunk++;
        }
        int n = Math.min(max - done, decodedCount - decodedNext);
        System.arraycopy(decoded, decodedNext * rl, buffer, done * rl, n * rl);
        decodedNext += n;
        done += n;
        pointsDone += n;
      }
      return done;
    }

    public void close() throws IOException {
      ch.close();
    }
  }

  // ---- Record accessors (raw LAS records as read).

  static int rawX(byte[] b, int at) {
    return LazDecoder.i32(b, at);
  }

  static int rawY(byte[] b, int at) {
    return LazDecoder.i32(b, at + 4);
  }

  static int rawZ(byte[] b, int at) {
    return LazDecoder.i32(b, at + 8);
  }

  static int classification(int format, byte[] b, int at) {
    return format >= 6 ? b[at + 16] & 255 : b[at + 15] & 31;
  }

  /** Withheld flag (excluded from use by the data provider). */
  static boolean withheld(int format, byte[] b, int at) {
    return format >= 6 ? (b[at + 15] & 4) != 0 : (b[at + 15] & 128) != 0;
  }

  static int returnNumber(int format, byte[] b, int at) {
    return format >= 6 ? b[at + 14] & 15 : b[at + 14] & 7;
  }

  static int returnCount(int format, byte[] b, int at) {
    return format >= 6 ? (b[at + 14] >> 4) & 15 : (b[at + 14] >> 3) & 7;
  }

  /** Byte offset of the GPS time within a record, or -1 (formats 0 and 2 have none). */
  static int gpsTimeOffset(int format) {
    return format >= 6 ? 22 : format == 1 || format >= 3 ? 20 : -1;
  }

  /**
   * Calendar date of an adjusted standard GPS time (global encoding bit 0: GPS seconds minus 1e9),
   * or null when the file stores GPS week seconds (no date).
   */
  static java.time.LocalDate gpsDate(double time, int globalEncoding) {
    if ((globalEncoding & 1) == 0 || !Double.isFinite(time)) return null;
    return java.time.LocalDateTime.of(1980, 1, 6, 0, 0).plusSeconds((long) (time + 1e9)).toLocalDate();
  }

  /** Byte offset of R,G,B within a record, or -1. */
  static int colourOffset(int format) {
    return switch (format) {
      case 2 -> 20;
      case 3, 5 -> 28;
      case 7, 8, 10 -> 30;
      default -> -1;
    };
  }
}
