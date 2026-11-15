package sgm;

import java.io.IOException;
import java.io.RandomAccessFile;
import java.lang.foreign.Arena;
import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.nio.ByteOrder;
import java.nio.channels.FileChannel;
import java.nio.file.*;

/**
 * Memory-mapped raster files (STREAMED_PROCESSING.md): values are read and written in place
 * through the operating system's file cache and never copied whole into the heap, so free RAM
 * speeds them up and a small heap still works. Little-endian, as DepthMapFile. One shared arena
 * per owner: closing it unmaps every file at once (Windows cannot delete a mapped file).
 */
final class MappedFiles implements AutoCloseable {
  static final ValueLayout.OfFloat FLOAT = ValueLayout.JAVA_FLOAT_UNALIGNED.withOrder(ByteOrder.LITTLE_ENDIAN);
  static final ValueLayout.OfInt INT = ValueLayout.JAVA_INT_UNALIGNED.withOrder(ByteOrder.LITTLE_ENDIAN);
  static final ValueLayout.OfLong LONG = ValueLayout.JAVA_LONG_UNALIGNED.withOrder(ByteOrder.LITTLE_ENDIAN);

  // The files are little-endian; on a little-endian machine their bytes are the array's bytes, so
  // bulk copies go byte for byte (identical values) and skip the per-call element-alignment check
  // of the layout copies (2.6% of fusion samples on large frames). Other machines keep the layout.
  private static final boolean NATIVE = ByteOrder.nativeOrder() == ByteOrder.LITTLE_ENDIAN;

  /** n floats from byte offset of src into dst[at..]. */
  static void copy(MemorySegment src, long offset, float[] dst, int at, int n) {
    if (NATIVE) MemorySegment.copy(src, offset, MemorySegment.ofArray(dst), 4L * at, 4L * n);
    else MemorySegment.copy(src, FLOAT, offset, dst, at, n);
  }

  /** n ints from byte offset of src into dst[at..]. */
  static void copy(MemorySegment src, long offset, int[] dst, int at, int n) {
    if (NATIVE) MemorySegment.copy(src, offset, MemorySegment.ofArray(dst), 4L * at, 4L * n);
    else MemorySegment.copy(src, INT, offset, dst, at, n);
  }

  /** n floats of src[at..] to byte offset of dst. */
  static void copy(float[] src, int at, MemorySegment dst, long offset, int n) {
    if (NATIVE) MemorySegment.copy(MemorySegment.ofArray(src), 4L * at, dst, offset, 4L * n);
    else MemorySegment.copy(src, at, dst, FLOAT, offset, n);
  }

  private final Arena arena = Arena.ofShared();
  private long mappedBytes;

  /** bytes of an existing file from offset, read-only or read-write. */
  MemorySegment map(Path path, long offset, long bytes, boolean write) throws IOException {
    try (FileChannel ch = write
        ? FileChannel.open(path, StandardOpenOption.READ, StandardOpenOption.WRITE)
        : FileChannel.open(path, StandardOpenOption.READ)) {
      if (ch.size() < offset + bytes)
        throw new IOException("Map file " + path + " holds " + ch.size() + " bytes, expected " + (offset + bytes));
      mappedBytes += bytes;
      return ch.map(write ? FileChannel.MapMode.READ_WRITE : FileChannel.MapMode.READ_ONLY, offset, bytes, arena);
    }
  }

  /** A new file of bytes zeros (sparse where the file system allows), mapped read-write. */
  MemorySegment create(Path path, long bytes) throws IOException {
    Files.createDirectories(path.toAbsolutePath().getParent());
    try (RandomAccessFile f = new RandomAccessFile(path.toFile(), "rw")) {
      f.setLength(0);
      f.setLength(bytes);
    }
    return map(path, 0, bytes, true);
  }

  long mappedBytes() {
    return mappedBytes;
  }

  @Override
  public void close() {
    arena.close();
  }
}
