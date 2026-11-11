package sgm;

import com.twelvemonkeys.imageio.metadata.*;
import com.twelvemonkeys.imageio.metadata.tiff.*;
import java.awt.Rectangle;
import java.awt.image.BufferedImage;
import java.io.*;
import java.nio.ByteOrder;
import java.util.*;
import javax.imageio.*;
import javax.imageio.stream.*;

/**
 * Virtual classic-TIFF view of requested BigTIFF blocks. Data stays at its original long file
 * offsets; only a small IFD and mappings are materialized. The JDK decoder handles compression,
 * color and region clipping. Unrequested tile offsets are never accessed by that decoder.
 */
final class BigTiffWindows {
  private record Block(long virtual, long physical, long length) {}

  private final ImageInputStream source;
  private final List<Entry> fields = new ArrayList<>();
  private final ByteOrder order;
  private final long[] offsets, counts;
  private final int width, height, tileWidth, tileHeight, planes, offsetTag, countTag;

  int width() {
    return width;
  }

  int height() {
    return height;
  }
  private static final Set<Integer> PIXEL_TAGS =
      Set.of(
          256, 257, 258, 259, 262, 266, 273, 277, 278, 279, 284, 292, 293, 301, 317, 320, 322, 323,
          324, 325, 338, 339, 347, 512, 513, 514, 515, 519, 520, 521, 529, 530, 531, 532, 34675);

  static boolean isBig(java.nio.file.Path path) throws IOException {
    try (var file = new RandomAccessFile(path.toFile(), "r")) {
      if (file.length() < 4) return false;
      int a = file.readUnsignedByte(),
          b = file.readUnsignedByte(),
          c = file.readUnsignedByte(),
          d = file.readUnsignedByte();
      return a == 'I' && b == 'I' && c == 43 && d == 0 || a == 'M' && b == 'M' && c == 0 && d == 43;
    }
  }

  BigTiffWindows(ImageInputStream input) throws IOException {
    source = input;
    input.seek(0);
    int a = input.readUnsignedByte();
    order = a == 'I' ? ByteOrder.LITTLE_ENDIAN : ByteOrder.BIG_ENDIAN;
    input.seek(0);
    Directory directory = new TIFFReader().read(input);
    width = integer(directory, 256, 0);
    height = integer(directory, 257, 0);
    if (width <= 0 || height <= 0) throw new IOException("Invalid BigTIFF image size");
    boolean tiled = directory.getEntryById(324) != null;
    offsetTag = tiled ? 324 : 273;
    countTag = tiled ? 325 : 279;
    offsets = vector(directory.getEntryById(offsetTag));
    counts = vector(directory.getEntryById(countTag));
    tileWidth = tiled ? integer(directory, 322, 0) : width;
    tileHeight = tiled ? integer(directory, 323, 0) : integer(directory, 278, height);
    planes = integer(directory, 284, 1) == 2 ? integer(directory, 277, 1) : 1;
    if (integer(directory, 259, 1) == 6)
      throw new IOException(
          "Old-style JPEG BigTIFF is unsupported; export TIFF with modern JPEG/LZW/Deflate"
              + " compression");
    if (tileWidth <= 0 || tileHeight <= 0 || planes < 1 || planes > 16)
      throw new IOException("Invalid BigTIFF tile layout");
    long blocks =
        ((width + (long) tileWidth - 1) / tileWidth)
            * ((height + (long) tileHeight - 1) / tileHeight)
            * planes;
    if (blocks != offsets.length || counts.length != offsets.length)
      throw new IOException("BigTIFF block offset/count arrays disagree");
    for (Entry entry : directory) {
      int tag = ((Number) entry.getIdentifier()).intValue();
      if (!PIXEL_TAGS.contains(tag) || tag == offsetTag || tag == countTag) continue;
      if (entry instanceof TIFFEntry t && t.getType() >= 16) {
        fields.add(new TIFFEntry(tag, (short) 4, vector(entry)));
      } else fields.add(entry);
    }
  }

  private static int integer(Directory d, int tag, int fallback) throws IOException {
    Entry e = d.getEntryById(tag);
    if (e == null) return fallback;
    long v = vector(e)[0];
    if (v < 1 || v > Integer.MAX_VALUE)
      throw new IOException("Unsupported BigTIFF dimension/tag " + tag);
    return (int) v;
  }

  private static long[] vector(Entry e) throws IOException {
    if (e == null) throw new IOException("Missing BigTIFF pixel tag");
    Object value = e.getValue();
    if (value instanceof Number n) {
      long v = n.longValue();
      if (e instanceof TIFFEntry t) {
        if (t.getType() == 3) v &= 65535;
        else if (t.getType() == 4 || t.getType() == 13) v &= 0xffffffffL;
      }
      return new long[] {v};
    }
    if (value instanceof long[] a) return a;
    if (value instanceof int[] a) {
      long[] b = new long[a.length];
      for (int i = 0; i < a.length; i++) b[i] = Integer.toUnsignedLong(a[i]);
      return b;
    }
    if (value instanceof short[] a) {
      long[] b = new long[a.length];
      for (int i = 0; i < a.length; i++) b[i] = a[i] & 65535;
      return b;
    }
    throw new IOException("Unsupported BigTIFF integer field " + e.getIdentifier());
  }

  private byte[] header(long[] virtualOffsets, long[] byteCounts) throws IOException {
    List<Entry> entries = new ArrayList<>(fields);
    entries.add(new TIFFEntry(offsetTag, (short) 4, virtualOffsets));
    entries.add(new TIFFEntry(countTag, (short) 4, byteCounts));
    var bytes = new ByteArrayOutputStream();
    try (var output = new MemoryCacheImageOutputStream(bytes)) {
      output.setByteOrder(order);
      new TIFFWriter(4).write(entries, output);
      output.flush();
    }
    return bytes.toByteArray();
  }

  BufferedImage read(Rectangle region) throws IOException {
    long[] virtualOffsets = new long[offsets.length], byteCounts = new long[counts.length];
    byte[] header = header(virtualOffsets, byteCounts);
    long position = (header.length + 7L) / 8 * 8;
    List<Block> blocks = new ArrayList<>();
    int across = (int) ((width + (long) tileWidth - 1) / tileWidth),
        down = (int) ((height + (long) tileHeight - 1) / tileHeight);
    int x0 = region.x / tileWidth,
        x1 = (region.x + region.width - 1) / tileWidth,
        y0 = region.y / tileHeight,
        y1 = (region.y + region.height - 1) / tileHeight;
    long fileLength = source.length();
    for (int band = 0; band < planes; band++)
      for (int y = y0; y <= y1; y++)
        for (int x = x0; x <= x1; x++) {
          int index = (band * down + y) * across + x;
          long length = counts[index], offset = offsets[index];
          if (offset <= 0
              || length <= 0
              || length > 0xffffffffL
              || offset > Long.MAX_VALUE - length
              || fileLength >= 0 && offset + length > fileLength)
            throw new IOException("Invalid/incomplete/sparse BigTIFF block " + index);
          if (position + length > 0xffffffffL)
            throw new IOException(
                "Requested BigTIFF window spans more than 4 GiB of compressed blocks; use a smaller"
                    + " image window or retile the source");
          virtualOffsets[index] = position;
          byteCounts[index] = length;
          blocks.add(new Block(position, offset, length));
          position += length;
        }
    header = header(virtualOffsets, byteCounts);
    try (var view = new View(source, header, blocks, position)) {
      ImageReader reader = jdkTiff();
      try {
        reader.setInput(view, false, true);
        ImageReadParam p = reader.getDefaultReadParam();
        p.setSourceRegion(region);
        return reader.read(0, p);
      } finally {
        reader.dispose();
      }
    }
  }

  static ImageReader jdkTiff() throws IOException {
    var readers = ImageIO.getImageReadersByFormatName("TIFF");
    while (readers.hasNext()) {
      ImageReader reader = readers.next();
      if (reader.getClass().getName().startsWith("com.sun.imageio.plugins.tiff.")) return reader;
      reader.dispose();
    }
    throw new IOException("JDK TIFF region decoder is unavailable");
  }

  private static final class View extends ImageInputStreamImpl {
    private final ImageInputStream source;
    private final byte[] header;
    private final List<Block> blocks;
    private final long length;

    View(ImageInputStream source, byte[] header, List<Block> blocks, long length) {
      this.source = source;
      this.header = header;
      this.blocks = blocks;
      this.length = length;
    }

    public long length() {
      return length;
    }

    public int read() throws IOException {
      byte[] one = new byte[1];
      return read(one, 0, 1) < 0 ? -1 : one[0] & 255;
    }

    public int read(byte[] b, int offset, int count) throws IOException {
      checkClosed();
      Objects.checkFromIndexSize(offset, count, b.length);
      if (count == 0) return 0;
      if (streamPos >= length) return -1;
      int done = 0;
      bitOffset = 0;
      while (done < count && streamPos < length) {
        if (streamPos < header.length) {
          int n = (int) Math.min(count - done, header.length - streamPos);
          System.arraycopy(header, (int) streamPos, b, offset + done, n);
          done += n;
          streamPos += n;
          continue;
        }
        Block found = null;
        long next = length;
        for (Block block : blocks) {
          if (streamPos >= block.virtual && streamPos < block.virtual + block.length) {
            found = block;
            break;
          }
          if (block.virtual > streamPos) {
            next = block.virtual;
            break;
          }
        }
        int n;
        if (found == null) {
          n = (int) Math.min(count - done, next - streamPos);
          Arrays.fill(b, offset + done, offset + done + n, (byte) 0);
        } else {
          n = (int) Math.min(count - done, found.virtual + found.length - streamPos);
          source.seek(found.physical + streamPos - found.virtual);
          source.readFully(b, offset + done, n);
        }
        if (n <= 0) throw new IOException("Invalid BigTIFF window mapping");
        done += n;
        streamPos += n;
      }
      return done;
    }
  }
}
