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

  /**
   * Reader for the files this class writes (classic TIFF or BigTIFF, RGBA Deflate tiles): pixel
   * values 0xAARRGGBB, decoded tiles kept in a small cache. For report previews.
   */
  static final class Reader implements Closeable {
    final int width, height, tile, across;
    private final FileChannel in;
    private final long[] offsets, counts;
    private final LinkedHashMap<Integer, byte[]> cache = new LinkedHashMap<>(16, .75f, true);

    Reader(Path path) throws IOException {
      in = FileChannel.open(path, StandardOpenOption.READ);
      ByteBuffer head = ByteBuffer.allocate(16);
      in.read(head, 0);
      ByteOrder order = head.get(0) == 'I' ? ByteOrder.LITTLE_ENDIAN : ByteOrder.BIG_ENDIAN;
      head.order(order);
      boolean bigTiff = head.getShort(2) == 43;
      long ifd = bigTiff ? head.getLong(8) : Integer.toUnsignedLong(head.getInt(4));
      ByteBuffer count = read(ifd, bigTiff ? 8 : 2, order);
      long entries = bigTiff ? count.getLong(0) : count.getShort(0) & 0xffff;
      int entrySize = bigTiff ? 20 : 12;
      ByteBuffer dir = read(ifd + (bigTiff ? 8 : 2), (int) (entries * entrySize), order);
      int w = 0, h = 0, t = 0;
      long[] off = null, cnt = null;
      for (int e = 0; e < entries; e++) {
        int at = e * entrySize, id = dir.getShort(at) & 0xffff, type = dir.getShort(at + 2) & 0xffff;
        long n = bigTiff ? dir.getLong(at + 4) : Integer.toUnsignedLong(dir.getInt(at + 4));
        int valueAt = at + (bigTiff ? 12 : 8);
        long scalar = type == 3 ? dir.getShort(valueAt) & 0xffff : type == 16 ? dir.getLong(valueAt) : Integer.toUnsignedLong(dir.getInt(valueAt));
        if (id == 256) w = (int) scalar;
        else if (id == 257) h = (int) scalar;
        else if (id == 322) t = (int) scalar;
        else if (id == 324 || id == 325) {
          int size = type == 16 ? 8 : 4;
          long[] values = new long[(int) n];
          ByteBuffer data = n * size <= (bigTiff ? 8 : 4) ? dir.slice(valueAt, (int) (n * size)).order(order)
              : read(bigTiff ? dir.getLong(valueAt) : Integer.toUnsignedLong(dir.getInt(valueAt)), (int) (n * size), order);
          for (int k = 0; k < n; k++) values[k] = size == 8 ? data.getLong(8 * k) : Integer.toUnsignedLong(data.getInt(4 * k));
          if (id == 324) off = values;
          else cnt = values;
        }
      }
      if (w <= 0 || h <= 0 || t <= 0 || off == null || cnt == null) throw new IOException("not a tiled orthomosaic: " + path);
      width = w;
      height = h;
      tile = t;
      across = Math.ceilDiv(w, t);
      offsets = off;
      counts = cnt;
    }

    private ByteBuffer read(long at, int length, ByteOrder order) throws IOException {
      ByteBuffer buffer = ByteBuffer.allocate(length);
      while (buffer.hasRemaining() && in.read(buffer, at + buffer.position()) >= 0) {}
      return buffer.flip().order(order);
    }

    /** Pixel (x, y) as 0xAARRGGBB (0 outside the image). */
    int pixel(int x, int y) throws IOException {
      if (x < 0 || y < 0 || x >= width || y >= height) return 0;
      int index = (y / tile) * across + x / tile;
      byte[] samples = cache.get(index);
      if (samples == null) {
        byte[] packed = read(offsets[index], (int) counts[index], ByteOrder.BIG_ENDIAN).array();
        java.util.zip.Inflater inflater = new java.util.zip.Inflater();
        samples = new byte[tile * tile * 4];
        try {
          inflater.setInput(packed);
          inflater.inflate(samples);
        } catch (java.util.zip.DataFormatException e) {
          throw new IOException("corrupt orthomosaic tile " + index, e);
        } finally {
          inflater.end();
        }
        cache.put(index, samples);
        if (cache.size() > 2 * across + 8) cache.remove(cache.keySet().iterator().next());
      }
      int at = 4 * ((y % tile) * tile + x % tile);
      return (samples[at + 3] & 255) << 24 | (samples[at] & 255) << 16 | (samples[at + 1] & 255) << 8 | (samples[at + 2] & 255);
    }

    @Override
    public void close() throws IOException {
      in.close();
    }
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
