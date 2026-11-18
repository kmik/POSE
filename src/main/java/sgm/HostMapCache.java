package sgm;

import java.io.*;
import java.nio.*;
import java.nio.channels.FileChannel;
import java.nio.file.*;
import java.util.*;
import java.util.concurrent.atomic.AtomicReferenceArray;

/** Bounded, shared heap pages for source and fusion files. No mapped-file residency is assumed.
 * Mutable pages and bulk copies are locked; immutable scalar readers validate page lifetime.
 * File lifetime is independent of page lifetime. A separate bounded LRU reuses file channels.
 */
final class HostMapCache implements AutoCloseable {
  static final int PAGE = 64 * 1024;
  private final LinkedHashSet<Page> clock = new LinkedHashSet<>();
  private final LinkedHashMap<File, FileChannel> channels = new LinkedHashMap<>(16, .75f, true);
  private static final int MAX_CHANNELS = 64;
  private long limit, bytes, peak, loads, evictions, readBytes, writtenBytes;
  private final long pageLimit;
  private int peakPages;
  private boolean closed;

  HostMapCache(long limit) {
    if (limit < PAGE) throw new IllegalArgumentException("Host map cache needs at least 64 KiB");
    this.limit = limit;
    pageLimit = Math.ceilDiv(limit, PAGE);
  }

  final class File {
    final Path path;
    final long offset, length;
    final boolean writable;
    final AtomicReferenceArray<Page> pages;
    private boolean retired;
    private volatile boolean frozen;
    void freeze() { frozen = true; }
    File(Path path, long offset, long length, boolean writable) {
      this.path = path; this.offset = offset; this.length = length; this.writable = writable;
      pages = new AtomicReferenceArray<>(Math.toIntExact(Math.ceilDiv(length, PAGE)));
    }
    private void bounds(long at, long count) {
      if (at < 0 || count < 0 || at > length - count) throw new IndexOutOfBoundsException(path + ": " + at);
    }
    int getInt(long at) {
      bounds(at, 4);
      if ((at & 3) != 0) throw new IllegalArgumentException("Unaligned map value");
      int index = (int) (at / PAGE), within = (int) (at % PAGE);
      for (;;) {
        Page p = page(this, index);
        if (writable && !frozen) synchronized (p) {
          if (!p.valid) continue;
          if (!p.recent) p.recent = true;
          return p.buffer.getInt(within);
        }
        if (!p.valid) continue;
        // Immutable page read; recheck invalidation so eviction cannot return a stale page.
        // The local heap page remains alive for this read even after cache removal. There is at
        // most one such transient page per active scalar reader, reserved by the workspace.
        int value = p.buffer.getInt(within);
        if (!p.recent) p.recent = true;
        if (p.valid) return value;
      }
    }
    float get(long index) { return Float.intBitsToFloat(getInt(4 * index)); }
    void setInt(long at, int value) {
      bounds(at, 4);
      if (!writable || frozen) throw new IllegalStateException("Read-only map: " + path);
      for (;;) {
        Page p = page(this, (int) (at / PAGE));
        synchronized (p) {
          if (!p.valid) continue;
          p.buffer.putInt((int) (at % PAGE), value);
          p.dirty = true; p.recent = true;
          return;
        }
      }
    }
    void set(long index, float value) { setInt(4 * index, Float.floatToRawIntBits(value)); }
    long getLong(long index) {
      long at = 8 * index;
      return Integer.toUnsignedLong(getInt(at)) | (long) getInt(at + 4) << 32;
    }
    void setLong(long index, long value) {
      setInt(8 * index, (int) value); setInt(8 * index + 4, (int) (value >>> 32));
    }
    void read(long index, float[] dst, int at, int count) {
      transfer(index, dst, null, at, count, false);
    }
    void readInts(long index, int[] dst, int at, int count) {
      transfer(index, null, dst, at, count, false);
    }
    void write(long index, float[] src, int at, int count) {
      transfer(index, src, null, at, count, true);
    }
    private void transfer(long index, float[] floats, int[] ints, int at, int count, boolean write) {
      bounds(4 * index, 4L * count);
      if (write && (!writable || frozen)) throw new IllegalStateException("Read-only map: " + path);
      while (count > 0) {
        long pos = 4 * index;
        Page p = page(this, (int) (pos / PAGE));
        synchronized (p) {
          if (!p.valid) continue;
          int off = (int) (pos % PAGE), n = Math.min(count, (p.data.length - off) / 4);
          ByteBuffer b = p.buffer.duplicate().order(ByteOrder.LITTLE_ENDIAN);
          b.position(off);
          if (floats != null) {
            if (write) b.asFloatBuffer().put(floats, at, n);
            else b.asFloatBuffer().get(floats, at, n);
          } else b.asIntBuffer().get(ints, at, n);
          p.dirty |= write; p.recent = true;
          index += n; at += n; count -= n;
        }
      }
    }
  }

  private static final class Page {
    final File file;
    final int index;
    final byte[] data;
    final ByteBuffer buffer;
    volatile boolean valid = true;
    boolean dirty;
    volatile boolean recent = true;
    Page(File file, int index, int size) {
      this.file = file; this.index = index; data = new byte[size];
      buffer = ByteBuffer.wrap(data).order(ByteOrder.LITTLE_ENDIAN);
    }
  }

  synchronized File open(Path path, long offset, long length, boolean writable) throws IOException {
    if (closed) throw new IllegalStateException("Host cache closed");
    if (Files.size(path) < offset + length) throw new EOFException("Truncated map: " + path);
    return new File(path, offset, length, writable);
  }
  File create(Path path, long length) throws IOException {
    Files.createDirectories(path.toAbsolutePath().getParent());
    try (RandomAccessFile f = new RandomAccessFile(path.toFile(), "rw")) {
      f.setLength(0); f.setLength(length);
    }
    return open(path, 0, length, true);
  }
  private Page page(File file, int index) {
    Page p = file.pages.get(index);
    if (p != null) return p;
    synchronized (this) {
      if (closed || file.retired) throw new IllegalStateException("Closed host map: " + file.path);
      p = file.pages.get(index);
      if (p != null) return p;
      int size = (int) Math.min(PAGE, file.length - (long) index * PAGE);
      try {
        // Evict first; the replacement array is allocated only after space is available.
        // Count partial pages too: many tiny files must not grow unaccounted page metadata.
        while (bytes + size > limit || clock.size() >= pageLimit) evictOne();
        p = new Page(file, index, size);
        {
          FileChannel ch = channel(file);
          ByteBuffer b = ByteBuffer.wrap(p.data);
          long pos = file.offset + (long) index * PAGE;
          while (b.hasRemaining()) {
            int n = ch.read(b, pos);
            if (n < 0) throw new EOFException("Truncated map: " + file.path);
            pos += n;
          }
        }
        loads++; readBytes += size; bytes += size; peak = Math.max(peak, bytes);
        clock.add(p); file.pages.set(index, p);
        peakPages = Math.max(peakPages, clock.size());
        return p;
      } catch (IOException e) { throw new UncheckedIOException(e); }
    }
  }
  private void flush(Page p) throws IOException {
    if (!p.dirty) return;
    {
      FileChannel ch = channel(p.file);
      ByteBuffer b = ByteBuffer.wrap(p.data);
      long pos = p.file.offset + (long) p.index * PAGE;
      while (b.hasRemaining()) pos += ch.write(b, pos);
    }
    writtenBytes += p.data.length; p.dirty = false;
  }
  /** Caller holds the cache lock. A bounded handle LRU avoids reopening for every page miss. */
  private FileChannel channel(File file) throws IOException {
    FileChannel ch = channels.get(file);
    if (ch != null && ch.isOpen()) return ch;
    if (ch != null) channels.remove(file); // NIO interruption can close a cached channel.
    if (channels.size() >= MAX_CHANNELS) {
      var it = channels.entrySet().iterator();
      var oldest = it.next(); oldest.getValue().close(); it.remove();
    }
    ch = file.writable ? FileChannel.open(file.path, StandardOpenOption.READ, StandardOpenOption.WRITE)
        : FileChannel.open(file.path, StandardOpenOption.READ);
    channels.put(file, ch);
    return ch;
  }
  private void evictOne() throws IOException {
    int secondChances = clock.size();
    for (;;) {
      Iterator<Page> it = clock.iterator();
      if (!it.hasNext()) throw new IllegalStateException("Host cache accounting is empty");
      Page p = it.next(); it.remove();
      if (p.recent && secondChances-- > 0) { p.recent = false; clock.add(p); continue; }
      synchronized (p) {
        try { flush(p); }
        catch (IOException e) { clock.add(p); throw e; }
        p.valid = false; p.file.pages.set(p.index, null);
        bytes -= p.data.length; evictions++;
      }
      return;
    }
  }
  /** Flush and forget pages, keeping the file available for a later reader. */
  synchronized void trim() throws IOException { while (!clock.isEmpty()) evictOne(); }
  synchronized void retire(File file) throws IOException {
    if (file == null) return;
    boolean interrupted = Thread.interrupted();
    try {
    for (Iterator<Page> it = clock.iterator(); it.hasNext();) {
      Page p = it.next();
      if (p.file != file) continue;
      synchronized (p) {
        flush(p); p.valid = false; file.pages.set(p.index, null);
        bytes -= p.data.length; it.remove();
      }
    }
    file.retired = true;
    FileChannel ch = channels.remove(file);
    if (ch != null) ch.close();
    } finally { if (interrupted) Thread.currentThread().interrupt(); }
  }
  synchronized Map<String, Object> report() {
    return Json.object("limit_bytes", limit, "cached_bytes", bytes, "peak_cached_bytes", peak,
        "page_loads", loads, "evictions", evictions, "bytes_read", readBytes, "bytes_written", writtenBytes,
        "mapped_bytes", 0, "page_bytes", PAGE, "open_channels", channels.size(), "channel_limit", MAX_CHANNELS,
        "cached_pages", clock.size(), "peak_pages", peakPages, "page_limit", pageLimit);
  }
  synchronized long bytes() { return bytes; }
  @Override public synchronized void close() throws IOException {
    boolean interrupted = Thread.interrupted();
    try {
    IOException failure = null;
    try { trim(); } catch (IOException e) { failure = e; }
    for (FileChannel ch : channels.values()) try { ch.close(); }
    catch (IOException e) { if (failure == null) failure = e; else failure.addSuppressed(e); }
    channels.clear(); closed = true;
    if (failure != null) throw failure;
    } finally { if (interrupted) Thread.currentThread().interrupt(); }
  }
}
