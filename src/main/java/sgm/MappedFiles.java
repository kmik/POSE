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
