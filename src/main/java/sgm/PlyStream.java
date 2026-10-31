package sgm;

import java.io.*;
import java.nio.*;
import java.nio.channels.*;
import java.nio.file.*;
import java.util.*;

/** Bounded sequential reader for scalar binary little-endian PLY vertices. */
final class PlyStream {
  interface Consumer {
    void point(double x, double y, double z) throws IOException;
  }

  final Path file;
  final long count, header;
  final int bytes, x, y, z;
  final boolean xd, yd, zd;

  PlyStream(Path file) throws IOException {
    this.file = file;
    long h = 0, n = -1;
    int size = 0, ox = -1, oy = -1, oz = -1;
    boolean dx = false, dy = false, dz = false, vertex = false, format = false;
    try (InputStream in = new BufferedInputStream(Files.newInputStream(file))) {
      while (true) {
        ByteArrayOutputStream b = new ByteArrayOutputStream();
        int c;
        while ((c = in.read()) >= 0) {
          h++;
          if (c == 10) break;
          b.write(c);
          if (h > 65536) throw new IOException("PLY header exceeds limit");
        }
        if (c < 0) throw new EOFException("Truncated PLY header");
        String line = b.toString(java.nio.charset.StandardCharsets.US_ASCII).trim();
        if (line.startsWith("format ")) format = line.equals("format binary_little_endian 1.0");
        if (line.startsWith("element ")) {
          vertex = line.startsWith("element vertex ");
          if (vertex) n = Long.parseLong(line.substring(15));
        }
        if (vertex && line.startsWith("property ")) {
          String[] p = line.split("\\s+");
          if (p.length != 3)
            throw new IOException("List properties are unsupported in vertex records");
          int width =
              switch (p[1]) {
                case "double", "float64" -> 8;
                case "float", "float32", "int", "uint", "int32", "uint32" -> 4;
                case "short", "ushort", "int16", "uint16" -> 2;
                case "char", "uchar", "int8", "uint8" -> 1;
                default -> throw new IOException("Unsupported PLY property type");
              };
          if ((p[2].equals("x") || p[2].equals("y") || p[2].equals("z"))
              && !Set.of("float", "float32", "double", "float64").contains(p[1]))
            throw new IOException("PLY coordinates must be float/double");
          if (p[2].equals("x")) {
            ox = size;
            dx = width == 8;
          }
          if (p[2].equals("y")) {
            oy = size;
            dy = width == 8;
          }
          if (p[2].equals("z")) {
            oz = size;
            dz = width == 8;
          }
          size += width;
        }
        if (line.equals("end_header")) break;
      }
    }
    if (!format || n < 0 || size < 12 || ox < 0 || oy < 0 || oz < 0 || size > 4096)
      throw new IOException("Unsupported PLY XYZ layout");
    count = n;
    header = h;
    bytes = size;
    x = ox;
    y = oy;
    z = oz;
    xd = dx;
    yd = dy;
    zd = dz;
    if (Files.size(file) < header + Math.multiplyExact(count, bytes))
      throw new EOFException("Truncated PLY vertices");
  }

  void scan(String label, Consumer consumer) throws IOException {
    try (FileChannel in = FileChannel.open(file);
        WorkProgress work = WorkProgress.start(label, count, "points")) {
      in.position(header);
      ByteBuffer b = ByteBuffer.allocateDirect(bytes * 8192).order(ByteOrder.LITTLE_ENDIAN);
      for (long at = 0; at < count; at += 8192) {
        int n = (int) Math.min(8192, count - at);
        b.clear();
        b.limit(n * bytes);
        while (b.hasRemaining()) {
          int got = in.read(b);
          if (got < 0) throw new EOFException();
          if (got == 0) throw new IOException("PLY read made no progress");
        }
        for (int i = 0; i < n; i++) {
          int p = i * bytes;
          consumer.point(
              xd ? b.getDouble(p + x) : b.getFloat(p + x),
              yd ? b.getDouble(p + y) : b.getFloat(p + y),
              zd ? b.getDouble(p + z) : b.getFloat(p + z));
        }
        work.update(at + n);
      }
      work.done();
    }
  }
}
