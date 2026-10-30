package sgm;

import java.io.*;

/** Two exact 32-bit channels. Project maps share a bounded global page/file-handle pool. */
final class DepthRaster implements AutoCloseable {
  final int length;
  private final PagedVolume depth, offset;

  DepthRaster(int n, Options o) throws IOException {
    this(n, o, null);
  }

  DepthRaster(int n, Options o, PagedVolume.Pool pool) throws IOException {
    length = n;
    Options pages = o.copy();
    pages.output = o.output.resolve("06_fusion/storage");
    pages.tileSize = 512;
    pages.disparities = 1;
    pages.memoryMb = 16;
    boolean disk = o.diskDepth && (o.projectDirectory != null || o.cacheDirectory != null);
    depth = new PagedVolume(n, 4, 0, disk, pages, "depth-", false, pool);
    offset = new PagedVolume(n, 4, 0, disk, pages, "offset-", false, pool);
  }

  float get(int p) {
    return Float.intBitsToFloat(depth.get(p));
  }

  int offset(int p) {
    return offset.get(p);
  }

  void put(int p, float z, int packed) {
    depth.set(p, Float.floatToRawIntBits(z));
    offset.set(p, packed);
  }

  public void close() throws IOException {
    try {
      depth.close();
    } finally {
      offset.close();
    }
  }

  void discard() throws IOException {
    depth.deleteBackingFile();
    offset.deleteBackingFile();
  }
}
