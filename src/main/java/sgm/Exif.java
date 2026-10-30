package sgm;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.*;

/** Bounded JPEG EXIF and classic TIFF metadata reader. Raster orientation is not changed. */
final class Exif {
  static final class Gps {
    final double latitude, longitude, altitude;
    final String datum;

    Gps(double latitude, double longitude, double altitude) {
      this(latitude, longitude, altitude, "WGS84 (assumed)");
    }

    Gps(double latitude, double longitude, double altitude, String datum) {
      this.latitude = latitude;
      this.longitude = longitude;
      this.altitude = altitude;
      this.datum = datum;
    }

    boolean usable() {
      String d = datum.toUpperCase(Locale.ROOT).replaceAll("[^A-Z0-9]", "");
      return Double.isFinite(latitude + longitude + altitude)
          && Math.abs(latitude) <= 90
          && Math.abs(longitude) <= 180
          && (d.equals("WGS84") || d.equals("WGS1984") || d.equals("WGS84ASSUMED"));
    }
  }

  static final class Metadata {
    Gps gps;
    double focalMm = Double.NaN, focal35Mm = Double.NaN;
    String make = "", model = "";
  }

  static Gps gps(Path path) throws IOException {
    return read(path).gps;
  }

  static Metadata read(Path path) throws IOException {
    try (RandomAccessFile file = new RandomAccessFile(path.toFile(), "r")) {
      int signature = file.readUnsignedShort();
      if (signature == 0x4949 || signature == 0x4d4d) return tiff(new Tiff(file));
      if (signature != 0xffd8) return new Metadata();
      while (true) {
        int b;
        do {
          b = file.readUnsignedByte();
        } while (b != 255);
        int marker;
        do {
          marker = file.readUnsignedByte();
        } while (marker == 255);
        if (marker == 0xda || marker == 0xd9) return new Metadata();
        if (marker == 0x01 || marker >= 0xd0 && marker <= 0xd7) continue;
        int length = file.readUnsignedShort() - 2;
        if (length < 0) return new Metadata();
        byte[] segment = new byte[length];
        file.readFully(segment);
        if (marker == 0xe1
            && length > 14
            && segment[0] == 'E'
            && segment[1] == 'x'
            && segment[2] == 'i'
            && segment[3] == 'f'
            && segment[4] == 0
            && segment[5] == 0) return tiff(new Tiff(Arrays.copyOfRange(segment, 6, length)));
      }
    } catch (EOFException | IllegalArgumentException | IndexOutOfBoundsException e) {
      return new Metadata();
    }
  }

  private static Metadata tiff(Tiff t) throws IOException {
    Metadata m = new Metadata();
    int magic = t.u16(0);
    if (magic != 0x4949 && magic != 0x4d4d) return m;
    t.little = magic == 0x4949;
    if (t.u16(2) != 42) return m;
    Map<Integer, Entry> root = t.ifd(t.u32(4));
    m.make = t.text(root.get(0x010f));
    m.model = t.text(root.get(0x0110));
    Entry exif = root.get(0x8769), gps = root.get(0x8825);
    if (exif != null) {
      Map<Integer, Entry> e = t.ifd(t.integer(exif));
      Entry focal = e.get(0x920a), equiv = e.get(0xa405);
      if (focal != null && focal.type == 5) m.focalMm = t.rational(focal.offset);
      if (equiv != null) {
        double f = t.integer(equiv);
        if (f > 0) m.focal35Mm = f;
      }
    }
    if (gps != null) {
      Map<Integer, Entry> g = t.ifd(t.integer(gps));
      Entry lat = g.get(2), lon = g.get(4), alt = g.get(6), ref = g.get(5);
      String latRef = t.text(g.get(1)), lonRef = t.text(g.get(3));
      if (lat != null
          && lon != null
          && (latRef.equals("N") || latRef.equals("S"))
          && (lonRef.equals("E") || lonRef.equals("W"))) {
        double a = t.dms(lat), b = t.dms(lon), z = Double.NaN;
        if (alt != null && alt.type == 5 && ref != null) {
          long altitudeRef = t.integer(ref);
          if (altitudeRef == 0 || altitudeRef == 1)
            z = t.rational(alt.offset) * (altitudeRef == 1 ? -1 : 1);
        }
        String datum = t.text(g.get(18));
        if (Double.isFinite(a + b) && a <= 90 && b <= 180)
          m.gps =
              new Gps(
                  latRef.equals("S") ? -a : a,
                  lonRef.equals("W") ? -b : b,
                  z,
                  datum.isBlank() ? "WGS84 (assumed)" : datum);
      }
    }
    return m;
  }

  private static final class Entry {
    int type;
    long count, offset;

    Entry(int type, long count, long offset) {
      this.type = type;
      this.count = count;
      this.offset = offset;
    }
  }

  private static final class Tiff {
    final RandomAccessFile file;
    final byte[] bytes;
    final long length;
    boolean little;

    Tiff(RandomAccessFile file) throws IOException {
      this.file = file;
      bytes = null;
      length = file.length();
    }

    Tiff(byte[] bytes) {
      this.bytes = bytes;
      file = null;
      length = bytes.length;
    }

    int u8(long offset) throws IOException {
      if (offset < 0 || offset >= length) throw new EOFException("EXIF offset outside metadata");
      if (bytes != null) return bytes[(int) offset] & 255;
      file.seek(offset);
      return file.readUnsignedByte();
    }

    int u16(long p) throws IOException {
      int a = u8(p), b = u8(p + 1);
      return little ? a | b << 8 : a << 8 | b;
    }

    long u32(long p) throws IOException {
      long a = u16(p), b = u16(p + 2);
      return little ? a | b << 16 : a << 16 | b;
    }

    Map<Integer, Entry> ifd(long offset) throws IOException {
      Map<Integer, Entry> out = new HashMap<>();
      int n = u16(offset);
      if (n > 4096) throw new IllegalArgumentException("EXIF IFD too large");
      for (int i = 0; i < n; i++) {
        long p = offset + 2 + i * 12L, count = u32(p + 4);
        int type = u16(p + 2),
            size =
                type == 1 || type == 2 || type == 7
                    ? 1
                    : type == 3 ? 2 : type == 4 ? 4 : type == 5 ? 8 : 0;
        if (size == 0 || count < 1 || count > 1048576) continue;
        long data = count * size <= 4 ? p + 8 : u32(p + 8);
        if (data < 0 || data + count * size > length)
          throw new EOFException("EXIF value outside metadata");
        out.put(u16(p), new Entry(type, count, data));
      }
      return out;
    }

    long integer(Entry e) throws IOException {
      if (e.type == 1) return u8(e.offset);
      if (e.type == 3) return u16(e.offset);
      if (e.type == 4) return u32(e.offset);
      throw new IllegalArgumentException("Invalid EXIF integer type");
    }

    String text(Entry e) throws IOException {
      if (e == null || e.type != 2) return "";
      int n = (int) Math.min(1024, e.count);
      byte[] text = new byte[n];
      for (int i = 0; i < n; i++) {
        text[i] = (byte) u8(e.offset + i);
        if (text[i] == 0) {
          n = i;
          break;
        }
      }
      return new String(text, 0, n, StandardCharsets.US_ASCII).trim();
    }

    double rational(long p) throws IOException {
      long denominator = u32(p + 4);
      return denominator == 0 ? Double.NaN : u32(p) / (double) denominator;
    }

    double dms(Entry e) throws IOException {
      if (e.type != 5 || e.count < 3) return Double.NaN;
      double degrees = rational(e.offset),
          minutes = rational(e.offset + 8),
          seconds = rational(e.offset + 16);
      return minutes < 0 || minutes >= 60 || seconds < 0 || seconds >= 60
          ? Double.NaN
          : degrees + minutes / 60 + seconds / 3600;
    }
  }
}
