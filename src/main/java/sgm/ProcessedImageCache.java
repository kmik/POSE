package sgm;

import java.io.IOException;
import java.lang.foreign.Arena;
import java.lang.foreign.MemorySegment;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.FileChannel;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.nio.file.attribute.BasicFileAttributes;
import java.security.MessageDigest;
import java.util.HexFormat;
import java.util.concurrent.atomic.AtomicLong;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

/**
 * Decode once (--processed-image-cache, 14 November 2026): the decoded and resized source image
 * (8-bit BGR at the processed size, before undistortion) is kept as a raw file in the run's
 * cache/processed-images folder. Later loads of the same image at the same size and with the same
 * reader read it back instead of decoding the photograph again: the dense step after a feature
 * step at the same resolution, and frames evicted from the raster cache (large frames, of which
 * only a few fit). The bytes are the decoder's own output, so everything downstream is unchanged.
 * Only images at the dense processing size are written (an alignment-only size is not reused).
 * The folder is deleted when the dense step ends and again at finish.
 */
final class ProcessedImageCache {
  private static final long MAGIC = 0x504f5345_50494331L; // "POSEPIC1"
  private static final int HEADER = 64;
  static final AtomicLong hits = new AtomicLong(), writes = new AtomicLong(), written = new AtomicLong();

  private ProcessedImageCache() {}

  /**
   * The folder, set by the pipeline only while the steps up to dense run (so nothing is written
   * by code outside a run, nor by the report after the dense step); null = no cache. An
   * alignment copy of the options uses its dense options' folder.
   */
  static Path folder(Options o) {
    Options owner = o.denseOptions != null ? o.denseOptions : o;
    return owner.processedImageCacheDir;
  }

  static boolean enabled(Options o) {
    return folder(o) != null && !o.processedImageCache.equals("off");
  }

  /** Pipeline: cache folder of the run while steps up to dense run. */
  static void open(Options o) {
    o.processedImageCacheDir = o.output.resolve("cache/processed-images");
  }

  private static String key(ImageFrame f, String reader) throws IOException {
    BasicFileAttributes a = Files.readAttributes(f.path, BasicFileAttributes.class);
    String text =
        f.path.toAbsolutePath().normalize()
            + "|" + a.size() + "|" + a.lastModifiedTime().toMillis()
            + "|" + f.sourceWidth + "x" + f.sourceHeight
            + "|" + f.width + "x" + f.height + "|" + reader + "|" + f.options.imageWindow
            + "|" + WindowedImage.wholeDecodeBytes(f.options);
    try {
      byte[] d = MessageDigest.getInstance("SHA-256").digest(text.getBytes(StandardCharsets.UTF_8));
      return HexFormat.of().formatHex(d, 0, 16);
    } catch (java.security.NoSuchAlgorithmException e) {
      throw new IllegalStateException(e);
    }
  }

  private static Path file(ImageFrame f, String reader) throws IOException {
    return folder(f.options).resolve(String.format("image_%04d_%s.bgr", f.id, key(f, reader)));
  }

  /** The cached processed image, or null (not cached, disabled or unreadable). */
  static Mat read(ImageFrame f, String reader) {
    if (!enabled(f.options)) return null;
    try {
      Path file = file(f, reader);
      if (!Files.isRegularFile(file)) return null;
      long bytes = 3L * f.width * f.height;
      long started = System.nanoTime();
      try (FileChannel ch = FileChannel.open(file, StandardOpenOption.READ);
          Arena arena = Arena.ofConfined()) {
        if (ch.size() != HEADER + bytes) return null;
        ByteBuffer h = ByteBuffer.allocate(HEADER).order(ByteOrder.LITTLE_ENDIAN);
        while (h.hasRemaining()) if (ch.read(h, h.position()) < 0) return null;
        if (h.getLong(0) != MAGIC || h.getInt(8) != f.width || h.getInt(12) != f.height) return null;
        Mat out = new Mat(f.height, f.width, CvType.CV_8UC3);
        try {
          MemorySegment target = MemorySegment.ofAddress(out.dataAddr()).reinterpret(bytes);
          MemorySegment source = ch.map(FileChannel.MapMode.READ_ONLY, HEADER, bytes, arena);
          MemorySegment.copy(source, 0, target, 0, bytes);
        } catch (RuntimeException | Error e) {
          out.release();
          throw e;
        }
        hits.incrementAndGet();
        RunTelemetry.record("Processed image cache read " + f.path.getFileName(), started);
        return out;
      }
    } catch (IOException | RuntimeException e) {
      return null;
    }
  }

  /**
   * Keeps a decoded image when it is at the dense processing size and the disk has room: auto
   * leaves at least 16 GiB and a tenth of the volume free, on at least 1 GiB.
   */
  static void write(ImageFrame f, String reader, Mat resized) {
    if (!enabled(f.options) || !resized.isContinuous() || resized.type() != CvType.CV_8UC3
        || resized.cols() != f.width || resized.rows() != f.height) return;
    Options dense = f.options.denseOptions != null ? f.options.denseOptions : f.options;
    int[] size = dense.processedSize(f.sourceWidth, f.sourceHeight);
    if (size[0] != f.width || size[1] != f.height) return;
    long bytes = 3L * f.width * f.height;
    Path part = null;
    try {
      Path folder = Files.createDirectories(folder(f.options));
      Path file = file(f, reader);
      if (Files.isRegularFile(file)) return;
      FileStore store = Files.getFileStore(folder);
      long free = store.getUsableSpace() - bytes - HEADER;
      boolean room =
          f.options.processedImageCache.equals("on")
              ? free >= 1L << 30
              : free >= Math.max(16L << 30, store.getTotalSpace() / 10);
      if (!room) return;
      long started = System.nanoTime();
      part = Files.createTempFile(folder, "image-", ".part");
      try (FileChannel ch = FileChannel.open(part, StandardOpenOption.WRITE)) {
        ByteBuffer h = ByteBuffer.allocate(HEADER).order(ByteOrder.LITTLE_ENDIAN);
        h.putLong(0, MAGIC).putInt(8, f.width).putInt(12, f.height);
        long at = 0;
        while (h.hasRemaining()) at += ch.write(h, at);
        MemorySegment source = MemorySegment.ofAddress(resized.dataAddr()).reinterpret(bytes);
        for (long off = 0; off < bytes; ) {
          long n = Math.min(1L << 28, bytes - off);
          ByteBuffer b = source.asSlice(off, n).asByteBuffer();
          while (b.hasRemaining()) at += ch.write(b, at);
          off += n;
        }
      }
      try {
        Files.move(part, file, StandardCopyOption.ATOMIC_MOVE);
      } catch (FileAlreadyExistsException e) {
        Files.deleteIfExists(part);
      }
      part = null;
      writes.incrementAndGet();
      written.addAndGet(bytes);
      RunTelemetry.record("Processed image cache write " + f.path.getFileName(), started);
    } catch (IOException | RuntimeException e) {
      // Only an optimisation: the image is decoded again when needed.
    } finally {
      if (part != null)
        try {
          Files.deleteIfExists(part);
        } catch (IOException ignored) {
        }
    }
  }

  /** Deletes the run's cache folder (end of the dense step, finish). */
  static void delete(Options o) {
    Path folder = folder(o);
    o.processedImageCacheDir = null;
    if (folder == null || !Files.isDirectory(folder)) return;
    try (var files = Files.list(folder)) {
      for (Path p : (Iterable<Path>) files::iterator) Files.deleteIfExists(p);
      Files.deleteIfExists(folder);
      try {
        Files.deleteIfExists(folder.getParent());
      } catch (DirectoryNotEmptyException otherCaches) {
      }
    } catch (IOException | RuntimeException e) {
      System.out.println("Processed image cache not fully deleted: " + e.getMessage());
    }
  }
}
