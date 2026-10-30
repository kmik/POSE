package sgm;

import java.nio.file.*;
import java.util.*;

/** Same sheared raster workload can run with the before/fixed PagedVolume classpaths. */
final class DepthIoProbe {
  public static void main(String[] args) throws Exception {
    Path root = Files.createTempDirectory(Path.of("target"), "depth probe ");
    Options o = new Options();
    o.output = root;
    o.projectDirectory = root;
    o.cacheDirectory = root;
    PagedVolume.Pool pool = new PagedVolume.Pool(16 * 1048576L);
    List<DepthRaster> maps = new ArrayList<>();
    int rows = 16;
    for (int i = 0; i < 2; i++) maps.add(new DepthRaster(750000, o, pool));
    long start = System.nanoTime();
    long checksum = 0;
    try {
      for (int y = 0; y < rows; y++)
        for (int x = 0; x < 1000; x++) {
          int p = ((y + x / 2) % 750) * 1000 + x;
          for (int i = 0; i < 2; i++)
            maps.get(i).put(p, p + 1.0f + i * 1000000, DepthFusion.pack(.13, -.22));
        }
      for (int y = rows - 1; y >= 0; y--)
        for (int x = 999; x >= 0; x--) {
          int p = ((y + x / 2) % 750) * 1000 + x;
          for (int i = 0; i < 2; i++) {
            if (maps.get(i).get(p) != p + 1.0f + i * 1000000)
              throw new AssertionError("Depth changed");
            checksum += Float.floatToIntBits(maps.get(i).get(p));
            checksum += maps.get(i).offset(p);
          }
        }
      System.out.printf(
          Locale.ROOT,
          "Depth I/O workload: %.3f seconds; checksum=%d; resident=%.3f MiB%n",
          (System.nanoTime() - start) / 1e9,
          checksum,
          pool.used / 1048576.);
    } finally {
      for (DepthRaster map : maps) {
        map.close();
        map.discard();
      }
    }
    System.out.printf(
        Locale.ROOT,
        "Total including final flush/cleanup: %.3f seconds%n",
        (System.nanoTime() - start) / 1e9);
  }
}
