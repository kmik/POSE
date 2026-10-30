package sgm;

import java.nio.file.*;
import java.util.*;

/** Shared depth-cache capacity, forced eviction, bounded handles and cleanup. */
final class DepthIoRegression {
  private static int checks;

  private static void check(boolean ok, String reason) {
    checks++;
    if (!ok) throw new AssertionError(reason);
  }

  public static void main(String[] args) throws Exception {
    Path root = Files.createTempDirectory(Path.of("target"), "depth io ");
    System.out.println("PASS depth I/O checks=" + run(root) + "; fixtures=" + root);
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Path dir = Files.createDirectories(root.resolve("depth io regression"));
    Options options = new Options();
    options.output = dir;
    options.projectDirectory = dir;
    PagedVolume.Pool pool = new PagedVolume.Pool(16 * 1048576L);
    List<DepthRaster> maps = new ArrayList<>();
    int packed = DepthFusion.pack(.13, -.22);
    try {
      for (int i = 0; i < 2; i++) maps.add(new DepthRaster(750000, options, pool));
      for (int y = 0; y < 32; y++)
        for (int x = 0; x < 1000; x++) {
          int p = ((y + x / 2) % 750) * 1000 + x;
          for (int i = 0; i < 2; i++) maps.get(i).put(p, p + 1.0f + i * 1000000, packed);
        }
      boolean exact = true;
      for (int y = 31; y >= 0; y--)
        for (int x = 999; x >= 0; x--) {
          int p = ((y + x / 2) % 750) * 1000 + x;
          for (int i = 0; i < 2; i++)
            if (maps.get(i).get(p) != p + 1.0f + i * 1000000 || maps.get(i).offset(p) != packed)
              exact = false;
        }
      check(exact, "Sheared camera-raster writes/readback preserve depth and subpixel offsets");
      check(
          pool.used > 4 * 1048576L && pool.used <= pool.limit,
          "Maps use the shared budget beyond the old one-MiB-per-channel cap");
      check(
          pool.evictions == 0,
          "No premature eviction while all active depth pages fit the shared pool");
      check(
          pool.fileOpens == 0 && pool.reads == 0 && pool.writes == 0,
          "Fitting depth-cache working set performs no paging I/O before close");
      check(maps.get(0).get(749999) == 0, "Unwritten camera depth stays empty");
    } finally {
      for (DepthRaster map : maps) map.close();
    }
    check(
        pool.used == 0 && pool.openFiles() == 0,
        "Closing fitting maps releases pages and file handles");
    for (DepthRaster map : maps) map.discard();

    maps.clear();
    pool = new PagedVolume.Pool(65536, 2);
    Path sentinel = dir.resolve("06_fusion/storage/unrelated.txt");
    Files.writeString(sentinel, "keep");
    try {
      for (int i = 0; i < 8; i++) {
        DepthRaster map = new DepthRaster(10000, options, pool);
        maps.add(map);
        for (int p = 0; p < 10000; p += 17) map.put(p, 1 + i * 100000 + p, packed);
      }
      boolean exact = true;
      for (int i = 7; i >= 0; i--)
        for (int p = 0; p < 10000; p += 17)
          if (maps.get(i).get(p) != 1 + i * 100000 + p || maps.get(i).offset(p) != packed)
            exact = false;
      check(exact, "Depth and offsets survive shared-cache and file-handle evictions");
      check(pool.used <= pool.limit, "Forced eviction respects global RAM budget");
      check(
          pool.evictions > 0 && pool.reads > 0 && pool.writes > 0,
          "Small-pool fixture exercises disk writeback and readback");
      check(
          pool.peakChannels <= 2 && pool.openFiles() <= 2,
          "File handle cache remains bounded across sixteen volume files");
    } finally {
      for (DepthRaster map : maps) map.close();
    }
    check(
        pool.used == 0 && pool.openFiles() == 0,
        "Closing evicted maps releases every cached handle and page");
    for (DepthRaster map : maps) map.discard();
    try (var files = Files.walk(dir)) {
      check(
          files.noneMatch(p -> p.toString().endsWith(".tiles")),
          "Closed depth files can be removed on Windows");
    }
    check(Files.readString(sentinel).equals("keep"), "Depth cleanup preserves unrelated files");

    Options pages = new Options();
    pages.output = dir;
    pages.tileSize = 32;
    pages.disparities = 1;
    pool = new PagedVolume.Pool(65536, 2);
    try (PagedVolume volume =
        new PagedVolume(100000, 4, 17, true, pages, "read-only-", false, pool)) {
      for (int p = 0; p < 100000; p += 1024) volume.set(p, p ^ 9182);
      for (int p = 0; p < 100000; p += 1024) checkValue(volume, p);
      long writes = pool.writes;
      for (int p = 0; p < 100000; p += 1024) checkValue(volume, p);
      check(pool.writes == writes, "Clean pages read from disk are not written back again");
      check(volume.get(91) == 17, "Nonzero default values survive paged reads");
      check(pool.fileOpens == 1, "Repeated readback reuses a single cached file handle");
    }
    check(
        pool.used == 0 && pool.openFiles() == 0,
        "Direct paged-volume close releases its shared resources");
    return checks;
  }

  private static void checkValue(PagedVolume volume, int p) {
    if (volume.get(p) != (p ^ 9182)) throw new AssertionError("Paged integer changed");
  }
}
