package sgm;

import java.io.*;
import java.nio.*;
import java.nio.channels.FileChannel;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.*;

/**
 * Block processing pieces that do not need a full run: default off, argument handling, k-d
 * partition with overlap, the block agreement solve (recovers known offsets/rotations/scales from
 * shared cameras), nearest-camera ownership and the LAS merge (transformed coordinates, ownership,
 * header and the other record bytes preserved).
 */
final class BlockProcessingRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    System.out.println("PASS block processing: " + run(Files.createTempDirectory("blocks")));
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Files.createDirectories(root);
    // Default off; block options are package-private (not part of run settings or reports).
    Options o = TestSupport.options();
    check(o.blockProcessing.equals("off") && o.blockImages == 0 && o.blockOverlap == 1, "defaults");
    for (var f : Options.class.getFields())
      check(!f.getName().startsWith("block"), "block options must not be public fields: " + f.getName());
    check(BlockProcessing.mode(new String[] {"-i", "x"}).equals("off"), "absent = off");
    check(BlockProcessing.mode(new String[] {"--block-processing", "ON"}).equals("on"), "mode value");
    check(BlockProcessing.mode(new String[] {"--block-processing=auto"}).equals("auto"), "mode inline");
    List<String> base = BlockProcessing.baseArguments(new String[] {
        "-i", "imgs", "-o", "proj", "--quality", "high", "--block-processing", "on", "--block-images=30",
        "--block-overlap", "2", "--output=x", "--features", "20000"});
    check(base.equals(List.of("--quality", "high", "--features", "20000")), "base arguments: " + base);

    // Partition: 10 x 7 grid of positions, 20 m apart; footprint 60 m.
    List<BlockProcessing.Image> images = new ArrayList<>();
    for (int r = 0; r < 7; r++)
      for (int c = 0; c < 10; c++)
        images.add(new BlockProcessing.Image(Path.of("i" + r + "_" + c + ".jpg"), "i" + r + "_" + c + ".jpg", 20 * c, 20 * r, 60));
    var blocks = BlockProcessing.partition(images, 12, 0.5);
    int[] home = new int[images.size()];
    for (var b : blocks) {
      check(b.home().size() <= 12 && b.home().size() >= 6, "core size " + b.home().size());
      for (int i : b.home()) {
        home[i]++;
        check(b.images().contains(i), "core images belong to their block");
      }
      check(b.images().size() > b.home().size(), "overlap adds images");
      for (int i : b.images()) {
        var im = images.get(i);
        double[] c = b.core();
        check(im.x() >= c[0] - 30 && im.x() <= c[2] + 30 && im.y() >= c[1] - 30 && im.y() <= c[3] + 30, "overlap bound");
      }
    }
    for (int h : home) check(h == 1, "every image has exactly one home block");
    // A huge overlap is shrunk per block until the block fits the memory limit (15 images).
    for (var b : BlockProcessing.partition(images, 12, 10, 15)) {
      check(b.images().size() <= 15, "overlap shrunk to the memory limit: " + b.images().size());
      check(b.images().containsAll(b.home()), "core images stay in their block");
    }

    // Agreement: three blocks see the same 30 cameras; blocks 1 and 2 are moved by known small
    // similarities. The solve must bring the shared cameras back together.
    Random random = new Random(3);
    List<double[]> truth = new ArrayList<>();
    for (int i = 0; i < 30; i++) truth.add(new double[] {random.nextDouble() * 300, random.nextDouble() * 200, 100 + random.nextGaussian()});
    BlockProcessing.Similarity[] moves = {
      new BlockProcessing.Similarity(new double[] {150, 100, 100}, new double[] {0, 0, 0}, new double[] {0, 0, 0}, 0),
      new BlockProcessing.Similarity(new double[] {150, 100, 100}, new double[] {.6, -.4, .3}, new double[] {.002, -.001, .003}, .002),
      new BlockProcessing.Similarity(new double[] {150, 100, 100}, new double[] {-.3, .5, -.2}, new double[] {-.001, .002, -.002}, -.001)};
    List<Map<String, BlockProcessing.Pose>> poses = new ArrayList<>();
    for (var move : moves) {
      Map<String, BlockProcessing.Pose> m = new LinkedHashMap<>();
      for (int i = 0; i < truth.size(); i++)
        m.put("c" + i, new BlockProcessing.Pose(move.apply(truth.get(i)), transpose(BlockProcessing.Similarity.rodrigues(move.w()))));
      poses.add(m);
    }
    var solved = BlockProcessing.agree(poses);
    for (int i = 0; i < 3; i++)
      for (int j = i + 1; j < 3; j++) {
        double before = BlockProcessing.seam(poses.get(i), poses.get(j), null, null)[1];
        double after = BlockProcessing.seam(poses.get(i), poses.get(j), solved[i], solved[j])[1];
        check(before > .2, "test blocks disagree before: " + before);
        check(after < .01, "blocks agree after the solve: " + after + " m (before " + before + ")");
        double turned = BlockProcessing.seam(poses.get(i), poses.get(j), null, null)[2];
        double aligned = BlockProcessing.seam(poses.get(i), poses.get(j), solved[i], solved[j])[2];
        check(turned > .05 && aligned < .005, "orientations agree after the solve: " + aligned + " deg (before " + turned + ")");
      }

    // Ownership: nearest camera in plan view.
    var owners = new BlockProcessing.Owners(
        List.of(new double[] {0, 0, 0}, new double[] {100, 0, 0}, new double[] {0, 100, 0}), List.of(0, 1, 2));
    check(owners.owner(10, 5) == 0 && owners.owner(90, -20) == 1 && owners.owner(-30, 80) == 2, "nearest camera");
    check(owners.owner(5000, 0) == 1, "far points still get the nearest camera");

    // LAS merge: two blocks, point format 2 with 4 extra bytes; block 1 shifted by +1 m in x.
    Path runA = root.resolve("a"), runB = root.resolve("b");
    writeLas(runA, new double[][] {{0, 0, 10}, {40, 0, 11}, {60, 0, 12}}, 30);
    writeLas(runB, new double[][] {{40, 0, 20}, {59, 0, 21}, {99, 0, 22}}, 30);
    BlockProcessing.Similarity identity = new BlockProcessing.Similarity(new double[3], new double[3], new double[3], 0);
    BlockProcessing.Similarity shift = new BlockProcessing.Similarity(new double[3], new double[] {1, 0, 0}, new double[3], 0);
    var split = new BlockProcessing.Owners(List.of(new double[] {0, 0, 0}, new double[] {100, 0, 0}), List.of(0, 1));
    Path merged = root.resolve("merged.las");
    long[] kept = BlockProcessing.mergeLas(List.of(runA, runB), new BlockProcessing.Similarity[] {identity, shift}, split, merged);
    check(kept[0] == 2 && kept[1] == 2, "ownership keeps 2 + 2 points: " + Arrays.toString(kept));
    try (FileChannel ch = FileChannel.open(merged)) {
      ByteBuffer h = ByteBuffer.allocate((int) ch.size()).order(ByteOrder.LITTLE_ENDIAN);
      ch.read(h, 0);
      int offset = h.getInt(96), length = h.getShort(105) & 0xffff;
      long count = h.getInt(107) & 0xffffffffL;
      check(length == 30 && count == 4 && h.getInt(100) == 0, "header: record length, count, VLRs");
      double scale = h.getDouble(131), ox = h.getDouble(155);
      double[] xs = new double[4];
      for (int i = 0; i < 4; i++) {
        xs[i] = h.getInt(offset + i * length) * scale + ox;
        check(h.get(offset + i * length + 26) == (byte) (i < 2 ? 11 : 22), "extra bytes preserved");
      }
      check(Math.abs(xs[0] - 0) < 1e-3 && Math.abs(xs[1] - 40) < 1e-3, "block 0 points unchanged: " + Arrays.toString(xs));
      check(Math.abs(xs[2] - 60) < 1e-3 && Math.abs(xs[3] - 100) < 1e-3, "block 1 points shifted: " + Arrays.toString(xs));
      check(Math.abs(h.getDouble(179) - 100) < 1e-6 && Math.abs(h.getDouble(187) - 0) < 1e-6, "x bounds in header");
    }
    return checks;
  }

  private static double[] transpose(double[] m) {
    return new double[] {m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]};
  }

  /** Minimal LAS 1.2 point format 2 file (record length 26 + extra) under run/07_results. */
  private static void writeLas(Path run, double[][] points, int length) throws IOException {
    Path dir = Files.createDirectories(run.resolve("07_results"));
    ByteBuffer b = ByteBuffer.allocate(227 + points.length * length).order(ByteOrder.LITTLE_ENDIAN);
    b.put("LASF".getBytes(StandardCharsets.US_ASCII));
    b.position(24);
    b.put((byte) 1).put((byte) 2);
    b.position(94);
    b.putShort((short) 227).putInt(227).putInt(0).put((byte) 2).putShort((short) length).putInt(points.length).putInt(points.length);
    b.position(131);
    for (int i = 0; i < 3; i++) b.putDouble(.001);
    for (int i = 0; i < 3; i++) b.putDouble(0);
    b.position(227);
    for (double[] p : points) {
      int at = b.position();
      for (int k = 0; k < 3; k++) b.putInt((int) Math.round(p[k] / .001));
      b.position(at + 26);
      for (int k = 26; k < length; k++) b.put((byte) (p[2] >= 20 ? 22 : 11));
    }
    Files.write(dir.resolve("dense.las"), b.array());
  }
}
