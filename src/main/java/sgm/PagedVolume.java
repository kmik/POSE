package sgm;

import java.io.*;
import java.nio.*;
import java.nio.channels.*;
import java.nio.file.*;
import java.util.*;

/** Bounded resident tiles with explicit I/O, rather than unbounded memory mapping. */
final class PagedVolume implements AutoCloseable {
  private static final class Page {
    final ByteBuffer bytes;
    boolean resident = true;
    boolean dirty;

    Page(ByteBuffer bytes) {
      this.bytes = bytes;
    }
  }

  static final class Pool {
    private record Owner(PagedVolume volume, long block) {}

    final long limit;
    long used;
    final LinkedHashMap<Page, Owner> entries = new LinkedHashMap<>(16, .75f, true);
    private final LinkedHashMap<PagedVolume, FileChannel> channels =
        new LinkedHashMap<>(16, .75f, true);
    final int channelLimit;
    long evictions, fileOpens, reads, writes;
    int peakChannels;

    Pool(long limit) {
      this(limit, 32);
    }

    Pool(long limit, int channelLimit) {
      this.limit = Math.max(65536, limit);
      this.channelLimit = Math.max(1, channelLimit);
    }

    void touch(Page page) {
      entries.get(page);
    }

    void add(Page page, PagedVolume owner, long block) throws IOException {
      entries.put(page, new Owner(owner, block));
      used += page.bytes.capacity();
      while (used > limit && entries.size() > 1) {
        var it = entries.entrySet().iterator();
        var entry = it.next();
        Page old = entry.getKey();
        Owner source = entry.getValue();
        it.remove();
        used -= old.bytes.capacity();
        evictions++;
        source.volume.evict(source.block, old);
      }
    }

    void drop(Page page) {
      if (entries.remove(page) != null) used -= page.bytes.capacity();
      page.resident = false;
    }

    private FileChannel channel(PagedVolume owner) throws IOException {
      FileChannel channel = channels.get(owner);
      if (channel != null) return channel;
      if (channels.size() >= channelLimit) {
        var it = channels.entrySet().iterator();
        var old = it.next();
        old.getValue().close();
        it.remove();
      }
      channel = FileChannel.open(owner.path, StandardOpenOption.READ, StandardOpenOption.WRITE);
      channels.put(owner, channel);
      fileOpens++;
      peakChannels = Math.max(peakChannels, channels.size());
      return channel;
    }

    private void releaseFile(PagedVolume owner) throws IOException {
      FileChannel channel = channels.remove(owner);
      if (channel != null) channel.close();
    }

    int openFiles() {
      return channels.size();
    }
  }

  private final long count;
  private final int bytes, initial, cells, capacity;
  private final int[] ints;
  private final short[] shorts;
  private final FileChannel file;
  private final Path path;
  private final boolean temporary;
  private final Pool pool;
  private final BitSet initialized = new BitSet();
  private final LinkedHashMap<Long, Page> pages = new LinkedHashMap<>(16, .75f, true);
  private long lastIndex = -1;
  private Page last;
  private boolean closed;

  PagedVolume(long count, int bytes, int initial, boolean disk, Options o, String name)
      throws IOException {
    this(count, bytes, initial, disk, o, name, true);
  }

  PagedVolume(
      long count, int bytes, int initial, boolean disk, Options o, String name, boolean temporary)
      throws IOException {
    this(count, bytes, initial, disk, o, name, temporary, null);
  }

  PagedVolume(
      long count,
      int bytes,
      int initial,
      boolean disk,
      Options o,
      String name,
      boolean temporary,
      Pool pool)
      throws IOException {
    this.count = count;
    this.bytes = bytes;
    this.initial = initial;
    this.temporary = temporary;
    this.pool = pool;
    cells = Math.max(1024, Math.min(262144, o.tileSize * Math.max(1, o.disparities)));
    capacity =
        Math.max(
            2,
            (int) Math.min(32 * 1048576L, Math.max(1048576L, o.memoryMb * 1048576L / 16))
                / (cells * bytes));
    if (disk) {
      Files.createDirectories(o.output);
      path = Files.createTempFile(o.output, name, ".tiles");
      file =
          temporary
              ? FileChannel.open(path, StandardOpenOption.READ, StandardOpenOption.WRITE)
              : null;
      ints = null;
      shorts = null;
    } else {
      path = null;
      file = null;
      int n = Math.toIntExact(count);
      ints = bytes == 4 ? new int[n] : null;
      shorts = bytes == 2 ? new short[n] : null;
      if (initial != 0) {
        if (ints != null) Arrays.fill(ints, initial);
        else Arrays.fill(shorts, (short) initial);
      }
    }
  }

  int get(long index) {
    if (ints != null) return ints[(int) index];
    if (shorts != null) return shorts[(int) index] & 65535;
    Page p = page(index);
    int at = (int) (index % cells) * bytes;
    return bytes == 2 ? p.bytes.getShort(at) & 65535 : p.bytes.getInt(at);
  }

  /** Contiguous exact transfers for streamed GPU aggregation; no per-cell paging or boxing. */
  void transfer(long index, java.lang.foreign.MemorySegment data, boolean write) {
    if (data.byteSize() % bytes != 0 || index < 0 || index + data.byteSize() / bytes > count)
      throw new IllegalArgumentException("Invalid bulk volume transfer");
    long remaining = data.byteSize() / bytes, offset = 0;
    if (ints != null || shorts != null) {
      var array =
          ints != null
              ? java.lang.foreign.MemorySegment.ofArray(ints)
              : java.lang.foreign.MemorySegment.ofArray(shorts);
      if (write)
        java.lang.foreign.MemorySegment.copy(data, 0, array, index * bytes, data.byteSize());
      else java.lang.foreign.MemorySegment.copy(array, index * bytes, data, 0, data.byteSize());
      return;
    }
    while (remaining > 0) {
      Page p = page(index);
      int at = (int) (index % cells), n = (int) Math.min(remaining, cells - at);
      var segment = java.lang.foreign.MemorySegment.ofBuffer(p.bytes);
      if (write) {
        java.lang.foreign.MemorySegment.copy(
            data, offset, segment, (long) at * bytes, (long) n * bytes);
        p.dirty = true;
      } else
        java.lang.foreign.MemorySegment.copy(
            segment, (long) at * bytes, data, offset, (long) n * bytes);
      index += n;
      remaining -= n;
      offset += (long) n * bytes;
    }
  }

  void set(long index, int value) {
    if (ints != null) {
      ints[(int) index] = value;
      return;
    }
    if (shorts != null) {
      shorts[(int) index] = (short) value;
      return;
    }
    Page p = page(index);
    p.dirty = true;
    int at = (int) (index % cells) * bytes;
    if (bytes == 2) p.bytes.putShort(at, (short) value);
    else p.bytes.putInt(at, value);
  }

  private Page page(long index) {
    if (index < 0 || index >= count) throw new IndexOutOfBoundsException();
    long block = index / cells;
    if (block == lastIndex && last != null && last.resident) {
      if (pool != null) pool.touch(last);
      return last;
    }
    try {
      Page p = pages.get(block);
      if (p == null) {
        // Shared depth maps compete only for the global budget. An additional
        // per-map cap caused thrashing even with most of that budget unused.
        if (pool == null && pages.size() >= capacity) {
          var it = pages.entrySet().iterator();
          var oldest = it.next();
          flush(oldest.getKey(), oldest.getValue());
          if (pool != null) pool.drop(oldest.getValue());
          else oldest.getValue().resident = false;
          it.remove();
        }
        ByteBuffer b = ByteBuffer.allocate(cells * bytes).order(ByteOrder.LITTLE_ENDIAN);
        p = new Page(b);
        if (initialized.get(Math.toIntExact(block))) {
          if (file != null) read(file, b, block * cells * bytes);
          else if (pool != null) {
            read(pool.channel(this), b, block * cells * bytes);
            pool.reads++;
          } else
            try (FileChannel in = FileChannel.open(path, StandardOpenOption.READ)) {
              read(in, b, block * cells * bytes);
            }
        } else {
          if (initial != 0)
            for (int i = 0; i < cells; i++) {
              if (bytes == 2) b.putShort(i * bytes, (short) initial);
              else b.putInt(i * bytes, initial);
            }
          initialized.set(Math.toIntExact(block));
          p.dirty = true;
        }
        b.clear();
        pages.put(block, p);
        if (pool != null) pool.add(p, this, block);
      }
      last = p;
      lastIndex = block;
      if (pool != null) pool.touch(p);
      return p;
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  private void flush(long block, Page page) throws IOException {
    if (!page.dirty) return;
    if (file != null) flush(file, block, page);
    else if (pool != null) flush(pool.channel(this), block, page);
    else
      try (FileChannel out = FileChannel.open(path, StandardOpenOption.WRITE)) {
        flush(out, block, page);
      }
  }

  private void evict(long block, Page page) throws IOException {
    flush(block, page);
    pages.remove(block);
    page.resident = false;
  }

  private static void read(FileChannel file, ByteBuffer b, long at) throws IOException {
    while (b.hasRemaining()) {
      int n = file.read(b, at + b.position());
      if (n < 0) break;
    }
  }

  private void flush(FileChannel file, long block, Page page) throws IOException {
    if (!page.dirty) return;
    ByteBuffer b = page.bytes.duplicate();
    b.clear();
    long at = block * cells * bytes;
    while (b.hasRemaining()) file.write(b, at + b.position());
    page.dirty = false;
    if (pool != null) pool.writes++;
  }

  public void close() throws IOException {
    if (closed) return;
    closed = true;
    try {
      if (path != null && !temporary && !pages.isEmpty()) {
        if (pool != null) {
          FileChannel out = pool.channel(this);
          for (var e : pages.entrySet()) flush(out, e.getKey(), e.getValue());
        } else
          try (FileChannel out = FileChannel.open(path, StandardOpenOption.WRITE)) {
            for (var e : pages.entrySet()) flush(out, e.getKey(), e.getValue());
          }
      }
    } finally {
      try {
        if (pool != null) pool.releaseFile(this);
        if (file != null) file.close();
        if (path != null && temporary) Files.deleteIfExists(path);
      } finally {
        if (pool != null) for (Page p : pages.values()) pool.drop(p);
        pages.clear();
        last = null;
      }
    }
  }

  void deleteBackingFile() throws IOException {
    if (path != null) Files.deleteIfExists(path);
  }
}
