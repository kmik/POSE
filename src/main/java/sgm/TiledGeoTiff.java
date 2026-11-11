package sgm;

import java.io.*;
import java.nio.*;
import java.nio.channels.FileChannel;
import java.nio.file.*;
import java.util.*;
import java.util.zip.Deflater;

/**
 * Writer for a north-up, pixel-is-area GeoTIFF of 8-bit RGBA tiles (unassociated alpha: 0 = no
 * data), Deflate-compressed. Tiles may arrive in any order and are appended as they come; the
 * directory is written on close. With {@code big} the file is a BigTIFF (64-bit offsets), needed
 * above 4 GiB. Every tile must be written before closing.
 */
final class TiledGeoTiff implements Closeable {
  private record Tag(int id, int type, long count, byte[] value) {}

  final int width, height, tile, across, down;
  private final boolean big;
  private final double x, y, cell;
  private final int epsg;
  private final FileChannel out;
  private final long[] offsets, counts;
  private long position;

  /** x, y: world coordinates of the outer corner of the top-left pixel; cell: pixel size. */
  TiledGeoTiff(Path path, int width, int height, int tile, double x, double y, double cell, int epsg, boolean big)
      throws IOException {
    if (width <= 0 || height <= 0 || tile < 16 || tile % 16 != 0)
      throw new IllegalArgumentException("Invalid GeoTIFF size or tile (a multiple of 16)");
    this.width = width;
    this.height = height;
    this.tile = tile;
    this.x = x;
    this.y = y;
    this.cell = cell;
    this.epsg = epsg;
    this.big = big;
    across = Math.ceilDiv(width, tile);
    down = Math.ceilDiv(height, tile);
    offsets = new long[Math.multiplyExact(across, down)];
    counts = new long[offsets.length];
    out =
        FileChannel.open(
            path, StandardOpenOption.CREATE, StandardOpenOption.TRUNCATE_EXISTING, StandardOpenOption.WRITE);
    // Header; the directory offset is patched on close.
    position = big ? 16 : 8;
    put(ByteBuffer.allocate((int) position), 0);
  }

  /** Raw size above which a classic TIFF may not hold the compressed tiles. */
  static boolean needsBig(long width, long height) {
    return width * height * 4 > 3_500_000_000L;
  }

  /** Deflate of one tile's samples (tile * tile * 4 bytes, rows top to bottom, R G B A). */
  static byte[] compress(byte[] samples) {
    Deflater deflater = new Deflater(6);
    try {
      deflater.setInput(samples);
      deflater.finish();
      ByteArrayOutputStream bytes = new ByteArrayOutputStream(Math.max(64, samples.length / 4));
      byte[] buffer = new byte[1 << 16];
      while (!deflater.finished()) bytes.write(buffer, 0, deflater.deflate(buffer));
      return bytes.toByteArray();
    } finally {
      deflater.end();
    }
  }

  /** Appends the compressed tile at column tx, row ty. */
  synchronized void write(int tx, int ty, byte[] compressed) throws IOException {
    int index = ty * across + tx;
    if (tx < 0 || ty < 0 || tx >= across || ty >= down || counts[index] != 0)
      throw new IllegalArgumentException("Tile outside the image or written twice: " + tx + "," + ty);
    if (!big && position + compressed.length > 0xfff00000L)
      throw new IOException("GeoTIFF exceeds 4 GiB; it must be written as BigTIFF");
    offsets[index] = position;
    counts[index] = compressed.length;
    put(ByteBuffer.wrap(compressed), position);
    position += compressed.length;
  }

  private void put(ByteBuffer b, long at) throws IOException {
    while (b.hasRemaining()) at += out.write(b, at);
  }

  private static byte[] shorts(int... values) {
    ByteBuffer b = ByteBuffer.allocate(2 * values.length).order(ByteOrder.LITTLE_ENDIAN);
    for (int v : values) b.putShort((short) v);
    return b.array();
  }

  private static byte[] doubles(double... values) {
    ByteBuffer b = ByteBuffer.allocate(8 * values.length).order(ByteOrder.LITTLE_ENDIAN);
    for (double v : values) b.putDouble(v);
    return b.array();
  }

  private byte[] longs(long[] values) {
    ByteBuffer b = ByteBuffer.allocate((big ? 8 : 4) * values.length).order(ByteOrder.LITTLE_ENDIAN);
    for (long v : values)
      if (big) b.putLong(v);
      else b.putInt((int) v);
    return b.array();
  }

  @Override
  public synchronized void close() throws IOException {
    try (out) {
      for (int i = 0; i < counts.length; i++)
        if (counts[i] == 0) throw new IOException("GeoTIFF tile " + i + " was never written");
      int wide = big ? 16 : 4;
      List<Tag> tags = new ArrayList<>();
      tags.add(new Tag(256, 4, 1, longs4(width)));
      tags.add(new Tag(257, 4, 1, longs4(height)));
      tags.add(new Tag(258, 3, 4, shorts(8, 8, 8, 8)));
      tags.add(new Tag(259, 3, 1, shorts(8)));
      tags.add(new Tag(262, 3, 1, shorts(2)));
      tags.add(new Tag(277, 3, 1, shorts(4)));
      tags.add(new Tag(284, 3, 1, shorts(1)));
      tags.add(new Tag(322, 4, 1, longs4(tile)));
      tags.add(new Tag(323, 4, 1, longs4(tile)));
      tags.add(new Tag(324, wide, offsets.length, longs(offsets)));
      tags.add(new Tag(325, wide, counts.length, longs(counts)));
      tags.add(new Tag(338, 3, 1, shorts(2)));
      tags.add(new Tag(339, 3, 4, shorts(1, 1, 1, 1)));
      tags.add(new Tag(33550, 12, 3, doubles(cell, cell, 0)));
      tags.add(new Tag(33922, 12, 6, doubles(0, 0, 0, x, y, 0)));
      if (epsg > 0)
        tags.add(new Tag(34735, 3, 20, shorts(1, 1, 0, 4, 1024, 0, 1, 1, 1025, 0, 1, 1, 3072, 0, 1, epsg, 3076, 0, 1, 9001)));
      tags.sort(Comparator.comparingInt(Tag::id));
      long directory = position + (position & 1);
      int inline = big ? 8 : 4, entry = big ? 20 : 12;
      int head = (big ? 8 : 2) + tags.size() * entry + (big ? 8 : 4);
      long extra = 0;
      for (Tag t : tags) if (t.value.length > inline) extra += t.value.length + (t.value.length & 1);
      if (!big && directory + head + extra > 0xffffffffL)
        throw new IOException("GeoTIFF exceeds 4 GiB; it must be written as BigTIFF");
      ByteBuffer b = ByteBuffer.allocate(Math.toIntExact(head + extra)).order(ByteOrder.LITTLE_ENDIAN);
      if (big) b.putLong(tags.size());
      else b.putShort((short) tags.size());
      int cursor = head;
      for (Tag t : tags) {
        b.putShort((short) t.id).putShort((short) t.type);
        if (big) b.putLong(t.count);
        else b.putInt((int) t.count);
        if (t.value.length <= inline) {
          b.put(t.value);
          for (int i = t.value.length; i < inline; i++) b.put((byte) 0);
        } else {
          if (big) b.putLong(directory + cursor);
          else b.putInt((int) (directory + cursor));
          int save = b.position();
          b.position(cursor);
          b.put(t.value);
          cursor += t.value.length + (t.value.length & 1);
          b.position(save);
        }
      }
      if (big) b.putLong(0);
      else b.putInt(0);
      b.position(0);
      put(b, directory);
      ByteBuffer header = ByteBuffer.allocate(big ? 16 : 8).order(ByteOrder.LITTLE_ENDIAN);
      header.put((byte) 'I').put((byte) 'I');
      if (big) header.putShort((short) 43).putShort((short) 8).putShort((short) 0).putLong(directory);
      else header.putShort((short) 42).putInt((int) directory);
      header.flip();
      put(header, 0);
    }
  }

  private static byte[] longs4(int value) {
    return ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN).putInt(value).array();
  }
}
