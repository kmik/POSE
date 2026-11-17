package sgm;

import java.nio.file.*;
import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.*;

/** MemoryGovernor admission, NativeMemory probes and the pair matching checkpoint file. */
final class MemoryGovernorRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Path root = Files.createTempDirectory(Path.of("target"), "memory governor ");
    System.out.println("PASS memory governor checks=" + run(root) + "; fixtures=" + root);
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Files.createDirectories(root);
    governor();
    probes();
    checkpoint(root);
    return checks;
  }

  private static final long GIB = 1L << 30;

  private static void governor() throws Exception {
    Options o = TestSupport.options();
    try {
      // Plenty of room: units run together without waiting.
      MemoryGovernor.reset();
      MemoryGovernor.pretend(2 * GIB, 100 * GIB, 90 * GIB, 100 * GIB);
      try (var a = MemoryGovernor.admit("test", 5 * GIB, o); var b = MemoryGovernor.admit("test", 5 * GIB, o)) {
        check(MemoryGovernor.waits == 0 && MemoryGovernor.admitted == 2, "room: no wait");
      }
      // Limit 10 GiB, process 2 GiB: one 5 GiB unit fits (2 + 5 <= 9.5), a second waits.
      MemoryGovernor.reset();
      MemoryGovernor.pretend(2 * GIB, 10 * GIB, 90 * GIB, 100 * GIB);
      var first = MemoryGovernor.admit("test", 5 * GIB, o);
      AtomicBoolean secondRan = new AtomicBoolean();
      CountDownLatch done = new CountDownLatch(1);
      Thread t = new Thread(() -> {
        try (var second = MemoryGovernor.admit("test", 5 * GIB, o)) {
          secondRan.set(true);
        }
        done.countDown();
      });
      t.start();
      Thread.sleep(400);
      check(!secondRan.get(), "second unit waits near the limit");
      first.close();
      check(done.await(10, TimeUnit.SECONDS) && secondRan.get(), "second unit runs after the first ends");
      check(MemoryGovernor.waits == 1, "wait counted: " + MemoryGovernor.waits);
      first.close(); // closing twice is harmless
      // One unit always runs, however large.
      MemoryGovernor.reset();
      MemoryGovernor.pretend(9 * GIB, 10 * GIB, 1 * GIB, 100 * GIB);
      try (var big = MemoryGovernor.admit("test", 50 * GIB, o)) {
        check(MemoryGovernor.admitted == 1, "a lone unit is admitted");
      }
      // Host reserve: limit far away, but the machine has 6 GiB available and keeps 5 GiB (5% of 100).
      MemoryGovernor.reset();
      MemoryGovernor.pretend(2 * GIB, 200 * GIB, 6 * GIB, 100 * GIB);
      var one = MemoryGovernor.admit("test", 512L << 20, o);
      AtomicBoolean other = new AtomicBoolean();
      Thread u = new Thread(() -> {
        try (var two = MemoryGovernor.admit("test", 1 * GIB, o)) {
          other.set(true);
        }
      });
      u.start();
      Thread.sleep(400);
      check(!other.get(), "host reserve holds the second unit back");
      one.close();
      u.join(10000);
      check(other.get(), "and lets it run later");
      // Zero estimate or disabled: no admission bookkeeping.
      MemoryGovernor.reset();
      try (var none = MemoryGovernor.admit("test", 0, o)) {
        check(MemoryGovernor.admitted == 0, "zero estimate bypasses");
      }
      System.setProperty("sgm.memoryGovernor", "false");
      try (var none = MemoryGovernor.admit("test", 5 * GIB, o)) {
        check(MemoryGovernor.admitted == 0, "disabled bypasses");
      } finally {
        System.clearProperty("sgm.memoryGovernor");
      }
    } finally {
      MemoryGovernor.reset();
    }
    // Estimates: 240 + 8 bytes per pixel for SIFT (measured 239 on a 20.9 Mpx photograph).
    Options s = TestSupport.options();
    s.matchingAlgorithm = "sift";
    s.features = 40000;
    check(MemoryGovernor.pairBytes(s) == 4L * 40000 * 512 + (16L << 20), "pair estimate");
  }

  private static void probes() {
    long committed = NativeMemory.committedBytes();
    boolean linux = System.getProperty("os.name").toLowerCase(Locale.ROOT).contains("linux");
    boolean windows = System.getProperty("os.name").toLowerCase(Locale.ROOT).contains("win");
    if (linux || windows) check(committed > 0, "committed bytes measured: " + committed);
    long trimmed = NativeMemory.trim("in the test");
    check(linux ? trimmed >= -1 : trimmed == -1, "malloc_trim only on Linux: " + trimmed);
    check(MemoryGovernor.summary().containsKey("malloc_trims"), "summary");
  }

  private static void checkpoint(Path root) throws Exception {
    Options o = TestSupport.options();
    o.output = root.resolve("run");
    o.pairMatching = "parallel";
    o.storage = "compact";
    o.diagnostics = false;
    Path file = o.output.resolve("03_matches/pair_checkpoint.bin");
    ImageFrame[] f = frames(4);
    FeatureTracks.Pair p = new FeatureTracks.Pair(0, 1);
    for (int i = 0; i < 50; i++) p.matches.add(new FeatureTracks.Match(i, 2 * i + 1));
    p.rotation = new double[] {1, 0, 0, 0, 1, 0, 0, 0, 1};
    p.translation = new double[] {0.5, -0.25, 1};
    p.essential = null;
    p.reason = "accepted";
    FeatureTracks.Pair q = new FeatureTracks.Pair(1, 2);
    q.reason = "insufficient_geometric_inliers";
    try (MatchCheckpoint c = openFor(o)) {
      check(c.get(f[0], f[1]) == null, "empty checkpoint");
      c.put(f[0], f[1], p, 4000, 80, 50);
      c.put(f[1], f[2], q, 4000, 12, 5);
      c.put(f[0], f[1], p, 1, 1, 1); // duplicate key ignored
      check(c.written == 2, "two records");
    }
    try (MatchCheckpoint c = openFor(o)) {
      MatchCheckpoint.Record r = c.get(f[0], f[1]);
      check(r != null && r.queries() == 4000 && r.ransac() == 80 && r.cheirality() == 50, "counts restored");
      check(r.pair().matches.size() == 50 && r.pair().matches.get(49).a == 49 && r.pair().matches.get(49).b == 99,
          "matches restored");
      check(Arrays.equals(r.pair().rotation, p.rotation) && Arrays.equals(r.pair().translation, p.translation)
          && r.pair().essential == null && r.pair().reason.equals("accepted"), "pose restored");
      MatchCheckpoint.Record s = c.get(f[1], f[2]);
      check(s != null && s.pair().matches.isEmpty() && s.pair().rotation == null
          && s.pair().reason.equals("insufficient_geometric_inliers"), "rejected pair restored");
      check(c.get(f[2], f[3]) == null, "unknown pair");
    }
    // A record cut by a kill: dropped, the earlier ones kept, and the next append overwrites it.
    long size = Files.size(file);
    try (var ch = java.nio.channels.FileChannel.open(file, StandardOpenOption.WRITE)) {
      ch.truncate(size - 7);
    }
    try (MatchCheckpoint c = openFor(o)) {
      check(c.get(f[0], f[1]) != null && c.get(f[1], f[2]) == null, "cut record dropped");
      c.put(f[2], f[3], q, 10, 0, 0);
    }
    try (MatchCheckpoint c = openFor(o)) {
      check(c.get(f[0], f[1]) != null && c.get(f[2], f[3]) != null, "append after a cut record");
    }
    // Other settings or other descriptors: nothing restored.
    o.ratio = 0.8;
    try (MatchCheckpoint c = openFor(o)) {
      check(c.get(f[0], f[1]) == null, "settings changed: checkpoint reset");
    }
    f[1].descriptorDigest = "changed";
    try (MatchCheckpoint c = openFor(o)) {
      c.put(f[0], f[2], p, 1, 1, 1);
    }
    try (MatchCheckpoint c = openFor(o)) {
      check(c.get(f[0], f[1]) == null && c.get(f[0], f[2]) != null, "descriptor digest is part of the key");
      c.close(true);
    }
    check(!Files.exists(file), "deleted when the step finishes");
    for (ImageFrame x : f) x.close();
  }

  private static MatchCheckpoint openFor(Options o) throws Exception {
    check(!MatchCheckpoint.enabled(o, null), "no checkpoint without a project");
    return MatchCheckpoint.openUnchecked(o);
  }

  private static ImageFrame[] frames(int n) throws Exception {
    Path dir = Files.createTempDirectory(Path.of("target"), "checkpoint frames ");
    Path image = dir.resolve("0.png");
    org.opencv.imgcodecs.Imgcodecs.imwrite(image.toString(),
        new org.opencv.core.Mat(20, 20, org.opencv.core.CvType.CV_8UC1, new org.opencv.core.Scalar(9)));
    Options o = TestSupport.options();
    o.output = dir.resolve("out");
    Calibration calibration = new Calibration(20, 20, 9.5, 9.5, new double[5], 20, 20);
    ImageFrame[] f = new ImageFrame[n];
    for (int i = 0; i < n; i++) {
      f[i] = new ImageFrame(i, image, calibration, o);
      f[i].descriptorDigest = "d" + i;
    }
    return f;
  }
}
