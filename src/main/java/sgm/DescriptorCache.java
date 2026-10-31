package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;

/**
 * Bounded native descriptor residency; immutable run-local spill files are deleted on frame close.
 */
final class DescriptorCache {
  private static final LinkedHashMap<ImageFrame, Long> resident =
      new LinkedHashMap<>(16, .75f, true);
  private static long used;

  private static long limit(ImageFrame f) {
    return Math.min(
        (long) f.options.descriptorMemoryMb * 1048576,
        Math.max(1048576L, f.options.memoryMb * 1048576L / 4));
  }

  static synchronized void prepare(ImageFrame f) {
    evictFor(f, null, Math.min(limit(f), 512L * f.options.features));
  }

  static synchronized void retain(ImageFrame f) throws IOException {
    {
      Long previous = resident.remove(f);
      if (previous != null) used -= previous;
      if (f.descriptorFile != null) {
        Files.deleteIfExists(f.descriptorFile);
        f.descriptorFile = null;
      }
      f.descriptorBytes = bytes(f);
      if (f.descriptorBytes > limit(f))
        throw new IllegalArgumentException(
            "Descriptor set exceeds --descriptor-memory-mb/processing budget; lower --features or"
                + " raise memory");
      Files.createDirectories(f.options.output);
      Path file = Files.createTempFile(f.options.output, "feature-descriptors-", ".bin");
      boolean complete = false;
      try (var out = new DataOutputStream(new BufferedOutputStream(Files.newOutputStream(file)))) {
        out.writeInt(f.descriptors.rows());
        out.writeInt(f.descriptors.cols());
        out.writeInt(f.descriptors.type());
        int bytes = Math.toIntExact(f.descriptors.cols() * f.descriptors.elemSize());
        // Preserve native descriptor bytes for both binary and floating descriptors.
        Mat raw =
            f.descriptors.empty() ? new Mat() : f.descriptors.reshape(1, f.descriptors.rows());
        try {
          if (raw.depth() == CvType.CV_8U) {
            byte[] row = new byte[bytes];
            for (int i = 0; i < raw.rows(); i++) {
              raw.get(i, 0, row);
              out.write(row);
            }
          } else {
            float[] row = new float[raw.cols()];
            for (int i = 0; i < raw.rows(); i++) {
              raw.get(i, 0, row);
              for (float v : row) out.writeInt(Float.floatToRawIntBits(v));
            }
          }
        } finally {
          raw.release();
        }
        complete = true;
      } finally {
        if (!complete) Files.deleteIfExists(file);
      }
      f.descriptorFile = file;
    }
    evictFor(f, null, f.descriptorBytes);
    load(f);
  }

  static synchronized void pair(ImageFrame a, ImageFrame b) throws IOException {
    long required =
        (a.descriptorFile == null ? bytes(a) : a.descriptorBytes)
            + (a == b ? 0 : b.descriptorFile == null ? bytes(b) : b.descriptorBytes);
    if (required > Math.min(limit(a), limit(b)))
      throw new IllegalArgumentException(
          "The two descriptor sets exceed --descriptor-memory-mb/processing budget; raise memory or"
              + " lower --features");
    long missing =
        (resident.containsKey(a) ? 0 : a.descriptorFile == null ? bytes(a) : a.descriptorBytes)
            + (a == b || resident.containsKey(b)
                ? 0
                : b.descriptorFile == null ? bytes(b) : b.descriptorBytes);
    evictFor(a, b, missing);
    load(a);
    if (a != b) load(b);
  }

  private static long bytes(ImageFrame f) {
    return f.descriptors.total() * f.descriptors.elemSize();
  }

  private static void load(ImageFrame f) throws IOException {
    if (resident.containsKey(f)) {
      resident.get(f);
      return;
    }
    if (f.descriptors.empty() && f.descriptorFile != null) {
      try (var in =
          new DataInputStream(new BufferedInputStream(Files.newInputStream(f.descriptorFile)))) {
        int rows = in.readInt(), cols = in.readInt(), type = in.readInt();
        f.descriptors.create(rows, cols, type);
        if (f.descriptors.depth() == CvType.CV_8U) {
          byte[] row = new byte[cols];
          for (int i = 0; i < rows; i++) {
            in.readFully(row);
            f.descriptors.put(i, 0, row);
          }
        } else {
          float[] row = new float[cols];
          for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) row[j] = in.readFloat();
            f.descriptors.put(i, 0, row);
          }
        }
      }
    }
    long size = bytes(f);
    resident.put(f, size);
    used += size;
  }

  private static void evictFor(ImageFrame a, ImageFrame b, long extra) {
    long limit = b == null ? limit(a) : Math.min(limit(a), limit(b));
    var iterator = resident.entrySet().iterator();
    while (used + extra > limit && iterator.hasNext()) {
      var entry = iterator.next();
      ImageFrame f = entry.getKey();
      if (f == a || f == b) continue;
      if (f.descriptorFile != null) {
        used -= entry.getValue();
        f.descriptors.release();
        iterator.remove();
      }
    }
  }

  static synchronized void forget(ImageFrame f) {
    Long old = resident.remove(f);
    if (old != null) used -= old;
    if (f.descriptorFile != null)
      try {
        Files.deleteIfExists(f.descriptorFile);
      } catch (IOException e) {
        throw new UncheckedIOException(e);
      } finally {
        f.descriptorFile = null;
      }
  }

  static synchronized long residentBytes() {
    return used;
  }
}
