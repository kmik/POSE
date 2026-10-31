package sgm;

import java.io.*;
import java.lang.foreign.*;
import java.nio.*;
import java.nio.channels.*;
import java.nio.file.*;

/** Long-indexed exact aggregate storage. Native RAM is explicitly budgeted and freed on close. */
final class GpuAggregate implements AutoCloseable {
  static final class AllocationFailure extends IllegalArgumentException {
    AllocationFailure(OutOfMemoryError cause) {
      super(
          "Cannot allocate planned native SGM aggregate; lower --memory-mb or use --gpu-aggregate"
              + " recompute",
          cause);
    }
  }

  private final Arena arena;
  private final MemorySegment ram;
  private final FileChannel file;
  private final Path path;
  private final long bytes;

  GpuAggregate(long bytes, boolean disk, Path directory) throws IOException {
    if (bytes < 1) throw new IllegalArgumentException("Empty GPU aggregate");
    this.bytes = bytes;
    if (disk) {
      Files.createDirectories(directory);
      path = Files.createTempFile(directory, "gpu-sgm-aggregate-", ".tiles");
      arena = null;
      ram = null;
      try {
        file = FileChannel.open(path, StandardOpenOption.READ, StandardOpenOption.WRITE);
      } catch (IOException e) {
        Files.deleteIfExists(path);
        throw e;
      }
    } else {
      path = null;
      file = null;
      arena = Arena.ofConfined();
      try {
        ram = arena.allocate(bytes, 8);
      } catch (OutOfMemoryError unavailable) {
        arena.close();
        throw new AllocationFailure(unavailable);
      } catch (Throwable e) {
        arena.close();
        throw e;
      }
    }
  }

  void transfer(long offset, MemorySegment data, boolean write) throws IOException {
    if (offset < 0 || offset > bytes || data.byteSize() > bytes - offset)
      throw new IllegalArgumentException("Aggregate transfer outside budgeted storage");
    if (ram != null) {
      if (write) MemorySegment.copy(data, 0, ram, offset, data.byteSize());
      else MemorySegment.copy(ram, offset, data, 0, data.byteSize());
      return;
    }
    // Direct contiguous chunks: never load an old page before a full overwrite.
    for (long at = 0; at < data.byteSize(); ) {
      int length = (int) Math.min(64L * 1048576, data.byteSize() - at);
      ByteBuffer buffer = data.asSlice(at, length).asByteBuffer();
      while (buffer.hasRemaining()) {
        int n =
            write
                ? file.write(buffer, offset + at + buffer.position())
                : file.read(buffer, offset + at + buffer.position());
        if (n < 0) throw new EOFException("Incomplete streamed SGM aggregate");
        if (n == 0) throw new IOException("Streamed aggregate I/O made no progress");
      }
      at += length;
    }
  }

  public void close() throws IOException {
    if (arena != null) arena.close();
    try {
      if (file != null) file.close();
    } finally {
      if (path != null) Files.deleteIfExists(path);
    }
  }
}
