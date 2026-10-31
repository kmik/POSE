package sgm;

import java.io.*;

/** Two exact 32-bit channels. Project maps share a bounded global page/file-handle pool. */
final class DepthRaster implements AutoCloseable {
  final int length;
  private final PagedVolume values;

  DepthRaster(int n, Options o) throws IOException {
    this(n, o, null);
  }

  DepthRaster(int n, Options o, PagedVolume.Pool pool) throws IOException {
    length = n;
    Options pages = o.copy();
    pages.output = o.output.resolve("06_fusion/storage");
    pages.tileSize = o.fusionPageSide;
    pages.disparities = 1;
    pages.memoryMb = 16;
    boolean disk = o.diskDepth && (o.projectDirectory != null || o.cacheDirectory != null);
    values = new PagedVolume(2L * n, 4, 0, disk, pages, "depth-offset-", false, pool);
  }

  float get(int p) {
    return Float.intBitsToFloat(values.get(2L * p));
  }

  int offset(int p) {
    return values.get(2L * p + 1);
  }

  void put(int p, float z, int packed) {
    values.set(2L * p, Float.floatToRawIntBits(z));
    values.set(2L * p + 1, packed);
  }

  public void close() throws IOException {
    values.close();
  }

  void discard() throws IOException {
    values.deleteBackingFile();
  }
}
