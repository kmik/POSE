package sgm;

import java.io.*;

/** Two exact 32-bit channels. Project maps share a bounded global page/file-handle pool. */
final class DepthRaster implements AutoCloseable {
  final int length;
  private final PagedVolume values;
  private final CameraEvidence.Image image;
  private final int rank;

  DepthRaster(int n, Options o) throws IOException {
    this(n, o, null);
  }

  DepthRaster(int n, Options o, PagedVolume.Pool pool) throws IOException {
    length = n;
    image = null;
    rank = 0;
    Options pages = o.copy();
    pages.output = o.output.resolve("06_fusion/storage");
    // A page holds side*side interleaved 32-bit cells, not side cells.
    pages.tileSize = o.fusionPageSide * o.fusionPageSide;
    pages.disparities = 1;
    pages.memoryMb = 16;
    boolean disk = o.diskDepth && (o.projectDirectory != null || o.cacheDirectory != null);
    values = new PagedVolume(2L * n, 4, 0, disk, pages, "depth-offset-", false, pool);
  }

  DepthRaster(CameraEvidence.Image image, int rank) {
    this.image = image;
    this.rank = rank;
    length = image.pixels();
    values = null;
  }

  float get(int p) {
    return Float.intBitsToFloat(
        image == null ? values.existing(2L * p) : (int) image.value(p, rank));
  }

  void freeze(boolean resident) throws IOException {
    if (values != null) values.freeze(resident);
  }

  boolean resident() {
    return image != null || values.resident();
  }

  int offset(int p) {
    return image == null ? values.existing(2L * p + 1) : (int) (image.value(p, rank) >>> 32);
  }

  void transfer(int start, int pixels, java.lang.foreign.MemorySegment destination) {
    if (start < 0
        || pixels < 0
        || start + (long) pixels > length
        || destination.byteSize() != pixels * 8L)
      throw new IllegalArgumentException("Invalid evidence window transfer");
    if (values != null) {
      values.transfer(2L * start, destination, false);
      return;
    }
    for (int i = 0; i < pixels; i++)
      destination.set(
          java.lang.foreign.ValueLayout.JAVA_LONG, i * 8L, image.value(start + i, rank));
  }

  void put(int p, float z, int packed) {
    if (values == null) throw new IllegalStateException("Camera evidence is immutable");
    values.set(2L * p, Float.floatToRawIntBits(z));
    values.set(2L * p + 1, packed);
  }

  public void close() throws IOException {
    if (values != null) values.close();
  }

  void discard() throws IOException {
    if (values != null) values.deleteBackingFile();
  }
}
