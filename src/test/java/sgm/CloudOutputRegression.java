package sgm;

import java.io.*;
import java.nio.*;
import java.nio.channels.FileChannel;
import java.nio.file.*;
import java.util.*;

/**
 * One-pass cloud output (14 November 2026): LAS/PLY written directly with patched headers, the
 * far-point rewrite, the automatic LAS 1.4 switch (threshold lowered with -Dsgm.cloud.las14Above)
 * and --tiled-las-output tiles, including more tiles than open file handles.
 */
final class CloudOutputRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    Path root = Files.createTempDirectory(Path.of("target"), "cloud-output-");
    System.out.println("PASS cloud output=" + run(root));
  }

  record Las(int minor, int headerSize, long dataOffset, int recordLength, long count,
      double[] scale, double[] offset, double[] min, double[] max, List<double[]> points,
      List<byte[]> tails) {}

  static Las read(Path file) throws IOException {
    try (FileChannel ch = FileChannel.open(file)) {
      ByteBuffer h = ByteBuffer.allocate(CloudWriter.HEADER_14).order(ByteOrder.LITTLE_ENDIAN);
      ch.read(h, 0);
      h.flip();
      int minor = h.get(25), size = h.getShort(94) & 0xffff, length = h.getShort(105) & 0xffff;
      long offset = h.getInt(96) & 0xffffffffL, count = CloudWriter.lasPointCount(h);
      double[] scale = new double[3], origin = new double[3], min = new double[3], max = new double[3];
      for (int k = 0; k < 3; k++) {
        scale[k] = h.getDouble(131 + 8 * k);
        origin[k] = h.getDouble(155 + 8 * k);
        max[k] = h.getDouble(179 + 16 * k);
        min[k] = h.getDouble(187 + 16 * k);
      }
      check(ch.size() == offset + count * length, "LAS size matches its header: " + file);
      ByteBuffer all = ByteBuffer.allocate((int) (count * length)).order(ByteOrder.LITTLE_ENDIAN);
      while (all.hasRemaining()) if (ch.read(all, offset + all.position()) < 0) throw new EOFException();
      List<double[]> points = new ArrayList<>();
      List<byte[]> tails = new ArrayList<>();
      for (long i = 0; i < count; i++) {
        int p = (int) (i * length);
        points.add(new double[] {
          all.getInt(p) * scale[0] + origin[0],
          all.getInt(p + 4) * scale[1] + origin[1],
          all.getInt(p + 8) * scale[2] + origin[2]});
        tails.add(Arrays.copyOfRange(all.array(), p + 12, p + length));
      }
      return new Las(minor, size, offset, length, count, scale, origin, min, max, points, tails);
    }
  }

  private static double[][] points(int n, long seed, double x0, double y0) {
    Random r = new Random(seed);
    double[][] p = new double[n][];
    for (int i = 0; i < n; i++)
      p[i] = new double[] {x0 + r.nextDouble() * 900, y0 + r.nextDouble() * 700, 80 + r.nextDouble() * 40};
    return p;
  }

  private static void write(CloudWriter w, double[][] p) throws IOException {
    for (int i = 0; i < p.length; i++) w.point(p[i][0], p[i][1], p[i][2], i & 255, (i * 7) & 255, (i * 13) & 255);
  }

  private static void same(Las las, double[][] p, double tolerance, String what) {
    check(las.count == p.length, what + ": count " + las.count + " vs " + p.length);
    double worst = 0;
    for (int i = 0; i < p.length; i++)
      for (int k = 0; k < 3; k++) worst = Math.max(worst, Math.abs(las.points.get(i)[k] - p[i][k]));
    check(worst <= tolerance, what + ": coordinates within " + tolerance + " (worst " + worst + ")");
    for (int i = 0; i < p.length; i++) {
      byte[] t = las.tails.get(i);
      ByteBuffer b = ByteBuffer.wrap(t).order(ByteOrder.LITTLE_ENDIAN);
      if (b.getShort(8) != (short) ((i & 255) * 257) || b.getShort(12) != (short) (((i * 13) & 255) * 257)) {
        check(false, what + ": colour of point " + i);
      }
    }
    check(true, what + ": colours in cloud order");
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Files.createDirectories(root);
    double half = CloudWriter.QUANTIZATION / 2 + 1e-9;

    // 1. Default output: LAS 1.2 + PLY in one pass, offsets from the first point.
    Path basic = Files.createDirectories(root.resolve("basic"));
    double[][] p = points(5000, 1, 401234.5, 7012345.25);
    try (CloudWriter w = new CloudWriter(basic, 1, new Crs("3067"), true)) {
      write(w, p);
    }
    Las las = read(basic.resolve("dense.las"));
    check(las.minor == 2 && las.headerSize == 227, "Default output is LAS 1.2");
    check(las.offset[0] == Math.round(p[0][0]) && las.offset[1] == Math.round(p[0][1])
        && las.offset[2] == Math.round(p[0][2]), "Offsets are the first point in whole units");
    check(las.scale[0] == CloudWriter.QUANTIZATION && las.recordLength == 26 + SurfaceAttributes.BYTES,
        "0.1 mm quantization and surface attributes");
    same(las, p, half, "LAS 1.2");
    double[] min = {Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE}, max = {-Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE};
    for (double[] q : p) for (int k = 0; k < 3; k++) { min[k] = Math.min(min[k], q[k]); max[k] = Math.max(max[k], q[k]); }
    check(Arrays.equals(las.min, min) && Arrays.equals(las.max, max), "Header bounds are the exact input bounds");
    PlyStream ply = new PlyStream(basic.resolve("dense.ply"));
    check(ply.count == p.length, "PLY count patched in place");
    List<double[]> back = new ArrayList<>();
    ply.scan("test", (x, y, z) -> back.add(new double[] {x, y, z}));
    boolean exact = back.size() == p.length;
    for (int i = 0; exact && i < p.length; i++) exact = Arrays.equals(back.get(i), p[i]);
    check(exact, "PLY keeps exact double coordinates in cloud order");
    try (var files = Files.list(basic)) {
      check(files.map(f -> f.getFileName().toString()).sorted().toList().equals(List.of("dense.las", "dense.ply")),
          "No spool or part files remain");
    }
    check(PlyStream.las(basic.resolve("dense.las")).count == p.length, "PlyStream reads the LAS");

    // 2. A point beyond the fixed offsets' range: rewritten once with centred offsets, order kept.
    Path far = Files.createDirectories(root.resolve("far"));
    double[][] f = points(3000, 2, 1000, 2000);
    f[1500] = new double[] {300000.25, 2100, 90};
    try (CloudWriter w = new CloudWriter(far, 1, null, false)) {
      w.writePly(false);
      write(w, f);
    }
    las = read(far.resolve("dense.las"));
    check(las.offset[0] == min(f, 0) + (max(f, 0) - min(f, 0)) / 2, "Far point: centred offsets");
    same(las, f, 2 * half, "Far-point rewrite");
    check(!Files.exists(far.resolve("dense.ply")), "PLY off stays off");
    try (var files = Files.list(far)) { check(files.count() == 1, "Far point: spool deleted"); }

    // 3. LAS 1.4 above the (lowered) point-count limit, read back by every LAS reader.
    String previous = System.getProperty("sgm.cloud.las14Above");
    try {
      System.setProperty("sgm.cloud.las14Above", "1000");
      Path v14 = Files.createDirectories(root.resolve("las14"));
      double[][] big = points(2500, 3, 380000, 6900000);
      try (CloudWriter w = new CloudWriter(v14, 1, new Crs("3067"), true)) {
        write(w, big);
      }
      las = read(v14.resolve("dense.las"));
      check(las.minor == 4 && las.headerSize == 375, "LAS 1.4 above the limit");
      try (FileChannel ch = FileChannel.open(v14.resolve("dense.las"))) {
        ByteBuffer h = ByteBuffer.allocate(375).order(ByteOrder.LITTLE_ENDIAN);
        ch.read(h, 0);
        check(h.getInt(107) == 0 && h.getInt(111) == 0 && h.getLong(247) == 2500 && h.getLong(255) == 2500,
            "LAS 1.4: legacy counts zero, 64-bit counts set");
      }
      same(las, big, half, "LAS 1.4 in-place conversion");
      check(PlyStream.las(v14.resolve("dense.las")).count == 2500, "PlyStream reads LAS 1.4 counts");
      var cloud = GroundClassification.read(v14.resolve("dense.las"), v14.resolve("none.bin"));
      check(cloud.n == 2500, "Ground classification reads LAS 1.4 counts");
      // LAS 1.4 written by the far-point rewrite.
      Path both = Files.createDirectories(root.resolve("las14-far"));
      double[][] g = points(1200, 4, 0, 0);
      g[10] = new double[] {250000, 1, 1};
      try (CloudWriter w = new CloudWriter(both, 1, null, false)) { w.writePly(false); write(w, g); }
      las = read(both.resolve("dense.las"));
      check(las.minor == 4, "Far-point rewrite also writes LAS 1.4");
      same(las, g, 2 * half, "LAS 1.4 rewrite");
    } finally {
      if (previous == null) System.clearProperty("sgm.cloud.las14Above");
      else System.setProperty("sgm.cloud.las14Above", previous);
    }
    byte[] h12 = CloudWriter.lasHeader(CloudWriter.LAS12_POINT_LIMIT, 1e-4, new double[3], new double[3], new double[3], 0, 0, 0, "x");
    byte[] h14 = CloudWriter.lasHeader(CloudWriter.LAS12_POINT_LIMIT + 1, 1e-4, new double[3], new double[3], new double[3], 0, 0, 0, "x");
    check(h12.length == 227 && h12[25] == 2 && ByteBuffer.wrap(h12).order(ByteOrder.LITTLE_ENDIAN).getInt(107) == -1,
        "4,294,967,295 points still fit LAS 1.2");
    check(h14.length == 375 && h14[25] == 4
        && CloudWriter.lasPointCount(ByteBuffer.wrap(h14).order(ByteOrder.LITTLE_ENDIAN)) == CloudWriter.LAS12_POINT_LIMIT + 1,
        "4,294,967,296 points switch to LAS 1.4");

    // 4. Tiles: 200 tiles of 10 units, points interleaved so tiles are closed and reopened.
    Path run = Files.createDirectories(root.resolve("tiled"));
    Path folder = run.resolve("07_results/dense_tiles");
    Files.createDirectories(folder);
    Files.writeString(folder.resolve("tile_stale.las"), "old");
    int cols = 20, rows = 10, n = 20000;
    double[][] t = new double[n][];
    Random r = new Random(5);
    for (int i = 0; i < n; i++) {
      int tile = i % (cols * rows);
      t[i] = new double[] {-50 + (tile % cols) * 10 + r.nextDouble() * 10, 500 + (tile / cols) * 10 + r.nextDouble() * 10, r.nextDouble()};
    }
    t[7] = new double[] {-40, 510, 0}; // exactly on a corner: belongs to the tile starting there
    try (CloudWriter w = new CloudWriter(run, 1, null, false)) {
      w.writePly(false);
      w.tiles(10, folder);
      write(w, t);
    }
    check(!Files.exists(run.resolve("dense.las")) && !Files.exists(folder.resolve("tile_stale.las")),
        "Tiles replace dense.las; stale tiles removed");
    List<String> index = Files.readAllLines(folder.resolve("tiles.csv"));
    check(index.size() == 1 + cols * rows, "tiles.csv lists every tile");
    long total = 0;
    boolean inside = true, order = true, corner = false;
    for (String line : index.subList(1, index.size())) {
      String[] c = line.split(",");
      double x0 = Double.parseDouble(c[1]), y0 = Double.parseDouble(c[2]);
      check(c[0].equals("tile_" + c[1] + "_" + c[2] + ".las"), "Tile name is its lower-left corner");
      Las tile = read(folder.resolve(c[0]));
      check(tile.count == Long.parseLong(c[4]) && tile.offset[0] == x0 && tile.offset[1] == y0,
          "Tile header: count and corner offsets");
      total += tile.count;
      int k = 0;
      for (int i = 0; i < n; i++) {
        if (!(Math.floor(t[i][0] / 10) * 10 == x0 && Math.floor(t[i][1] / 10) * 10 == y0)) continue;
        double[] q = tile.points.get(k++);
        for (int d = 0; d < 3; d++) order &= Math.abs(q[d] - t[i][d]) <= half;
        if (i == 7) corner = x0 == -40 && y0 == 510;
      }
      inside &= k == tile.count;
      for (double[] q : tile.points) inside &= q[0] >= x0 && q[0] < x0 + 10 && q[1] >= y0 && q[1] < y0 + 10;
    }
    check(total == n && inside, "Every point is in exactly the tile that contains it");
    check(order, "Tiles keep the cloud order of their points");
    check(corner, "A point on a tile corner goes to the tile starting there");
    PlyStream tiles = PlyStream.finalCloud(run);
    check(tiles != null && tiles.count == n, "Report readers find the tile set with --ply off");

    // 5. Abort and empty clouds publish nothing.
    Path aborted = Files.createDirectories(root.resolve("aborted"));
    CloudWriter w = new CloudWriter(aborted, 1);
    w.tiles(100, aborted.resolve("tiles"));
    write(w, points(500, 6, 0, 0));
    w.abort();
    try (var files = Files.walk(aborted)) {
      check(files.filter(Files::isRegularFile).findAny().isEmpty(), "Abort leaves no files");
    }
    Path empty = Files.createDirectories(root.resolve("empty"));
    boolean refused = false;
    try (CloudWriter e = new CloudWriter(empty, 1)) {
    } catch (IOException expected) {
      refused = expected.getMessage().contains("No valid dense points");
    }
    check(refused && !Files.exists(empty.resolve("dense.las")), "An empty cloud is refused");
    return checks;
  }

  private static double min(double[][] p, int k) {
    double m = Double.MAX_VALUE;
    for (double[] q : p) m = Math.min(m, q[k]);
    return m;
  }

  private static double max(double[][] p, int k) {
    double m = -Double.MAX_VALUE;
    for (double[] q : p) m = Math.max(m, q[k]);
    return m;
  }
}
