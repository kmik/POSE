package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.*;

/** Bounds independent of dependency/camera count, exact page values and reader failure barriers. */
final class FusionHostMemoryRegression {
  private static int checks;
  private static void check(boolean value, String why) {
    checks++; if (!value) throw new AssertionError(why);
  }
  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Path root = Files.createTempDirectory(Path.of("target"), "host-memory-");
    try { System.out.println("PASS host fusion memory: " + run(root)); }
    finally { LargeFormatRegression.delete(root); }
  }
  static int run(Path root) throws Exception {
    checks = 0; Files.createDirectories(root);
    Options defaults = new Options();
    check(defaults.fusionHostMemoryMb == 0, "Legacy remains default");
    Options parsed = Options.parse(new String[] {"-i", ".", "--dense-method", "mvsgm", "--fusion-mode", "image",
        "--disparity-refinement", "off", "--fusion-host-memory-mb", "512", "--fusion-host-cache-mb", "2"});
    check(parsed.copy().fusionHostMemoryMb == 512 && parsed.copy().fusionHostCacheMb == 2, "Opt-in settings copy");
    check(GuiOptions.all().stream().anyMatch(f -> f.flag().equals("--fusion-host-memory-mb")), "GUI discovers allowance");
    check(GuiOptions.all().stream().anyMatch(f -> f.flag().equals("--fusion-host-cache-mb")), "GUI discovers cache");
    boolean invalid = false;
    try { Options.parse(new String[] {"-i", ".", "--fusion-host-memory-mb", "512"}); }
    catch (IllegalArgumentException e) { invalid = true; }
    check(invalid, "Inapplicable opt-in fails clearly");
    for (int count : new int[] {32, 256, 3013}) scaling(root.resolve("scale-" + count), count);
    try (HostMapCache tiny = new HostMapCache(HostMapCache.PAGE)) {
      List<HostMapCache.File> files = new ArrayList<>();
      for (int c = 0; c < 512; c++) {
        var f = tiny.create(root.resolve("tiny-" + c), 16); f.setInt(0, c + 1); files.add(f);
        check(((Number) tiny.report().get("cached_pages")).intValue() <= 1, "Partial-page metadata stays bounded");
      }
      for (int c = 0; c < files.size(); c++) check(files.get(c).getInt(0) == c + 1, "Tiny file reopens exactly");
      check(((Number) tiny.report().get("peak_pages")).intValue() == 1, "Tiny maps cannot bypass the page-count cap");
      for (var f : files) { tiny.retire(f); Files.delete(f.path); }
    }
    parallel(root);
    HostMapCache cleanup = new HostMapCache(HostMapCache.PAGE);
    var dirty = cleanup.create(root.resolve("interrupted-cleanup"), 16);
    dirty.setInt(0, 12345);
    Thread.currentThread().interrupt();
    try { cleanup.close(); check(Thread.currentThread().isInterrupted(), "Cleanup preserves cancellation status"); }
    finally { Thread.interrupted(); }
    check(Files.readAllBytes(dirty.path)[0] == (byte) 12345, "Interrupted cleanup flushes dirty data");
    check(((Number) cleanup.report().get("open_channels")).intValue() == 0, "Cleanup closes every channel");
    Files.delete(dirty.path);
    try (HostMapCache interruptedIo = new HostMapCache(HostMapCache.PAGE)) {
      var f = interruptedIo.create(root.resolve("interrupted-io"), 2L * HostMapCache.PAGE);
      f.setInt(0, 0x12345678);
      boolean cancelledIo = false;
      Thread.currentThread().interrupt();
      try { f.getInt(HostMapCache.PAGE); }
      catch (UncheckedIOException expected) { cancelledIo = true; }
      try {
        interruptedIo.retire(f);
        check(cancelledIo && Thread.currentThread().isInterrupted(), "Interrupted I/O retirement preserves cancellation");
      } finally { Thread.interrupted(); }
      check(java.nio.ByteBuffer.wrap(Files.readAllBytes(f.path)).order(java.nio.ByteOrder.LITTLE_ENDIAN).getInt() == 0x12345678,
          "Cleanup reopens an interrupt-closed channel and flushes dirty data");
      check(((Number) interruptedIo.report().get("open_channels")).intValue() == 0, "Interrupted I/O closes all handles");
      Files.delete(f.path);
    }
    source(root.resolve("source"));
    oversizedFilter(root.resolve("oversized-filter"));
    exactFusion(root.resolve("fusion"));
    return checks;
  }
  private static void scaling(Path root, int count) throws Exception {
    Files.createDirectories(root);
    long began = System.nanoTime();
    try (HostMapCache cache = new HostMapCache(4L * HostMapCache.PAGE)) {
      List<HostMapCache.File> files = new ArrayList<>();
      for (int c = 0; c < count; c++) {
        var file = cache.create(root.resolve("camera_" + c), HostMapCache.PAGE + 16);
        files.add(file);
        file.setInt(0, c); file.setInt(HostMapCache.PAGE + 12, ~c);
        check(files.get(0).getInt(0) == 0, "First camera remains a late dependency");
        check(cache.bytes() <= 4L * HostMapCache.PAGE, "Cache bounded before replacement");
      }
      cache.trim();
      for (int c = files.size() - 1; c >= 0; c--) {
        var file = files.get(c);
        check(file.getInt(0) == c && file.getInt(HostMapCache.PAGE + 12) == ~c, "Dirty eviction/reopen exact");
      }
      var report = cache.report();
      check(((Number) report.get("peak_cached_bytes")).longValue() <= 4L * HostMapCache.PAGE, "Peak does not grow with camera count");
      check(((Number) report.get("peak_pages")).longValue() <= 4, "Page metadata bound does not grow with camera count");
      check(((Number) report.get("evictions")).longValue() > count, "Repeated evictions exercised");
      System.out.println("Host cache scaling " + count + " cameras, " + (System.nanoTime() - began) / 1e9 + " s: " + report);
      for (var f : files) { cache.retire(f); Files.delete(f.path); }
      check(cache.bytes() == 0, "Retirement releases all pages");
    }
    // Identical camera frusta force every fusion dependency; geometry is never truncated.
    int n = count;
    Camera[] cameras = new Camera[n]; int[] widths = new int[n], heights = new int[n];
    MultiViewSgm.Plan[] plans = new MultiViewSgm.Plan[n];
    for (int c = 0; c < n; c++) {
      cameras[c] = new Camera(100, 100, 16, 12); widths[c] = 32; heights[c] = 24;
      plans[c] = MultiViewSgm.Plan.header(c, List.of(new MultiViewSgm.Neighbor((c + 1) % n, 1, 100, 4)),
          new MultiViewSgm.Labels(.015, .0001, 101, .015, .025, 100));
    }
    CameraWindow graph = new CameraWindow(cameras, widths, heights, plans, false);
    check(graph.peakCameras == n && graph.possible[0].length == n - 1, "Oversized dependency window preserved");
  }
  private static void parallel(Path root) throws Exception {
    try (HostMapCache cache = new HostMapCache(HostMapCache.PAGE)) {
      var file = cache.create(root.resolve("parallel-pages"), 16L * HostMapCache.PAGE);
      for (int i = 0; i < 16; i++) file.setInt((long) i * HostMapCache.PAGE, i + 99);
      file.freeze();
      ForkJoinPool pool = new ForkJoinPool(4);
      AtomicInteger done = new AtomicInteger();
      try {
        FusionHostMemory.parallel(pool, 2000, i -> {
          int page = i % 16;
          if (file.getInt((long) page * HostMapCache.PAGE) != page + 99) throw new AssertionError("Parallel reader value");
          done.incrementAndGet();
        }, false);
        check(done.get() == 2000, "Parallel readers survive concurrent eviction");
        AtomicInteger active = new AtomicInteger();
        CountDownLatch started = new CountDownLatch(2);
        boolean failed = false;
        try { FusionHostMemory.parallel(pool, 4, i -> {
          active.incrementAndGet(); started.countDown();
          try {
            started.await();
            if (i == 0) throw new IllegalArgumentException("injected");
            Thread.sleep(30);
          } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
          finally { active.decrementAndGet(); }
        }, false); } catch (IllegalArgumentException e) { failed = true; }
        check(failed && active.get() == 0, "Failure joins active readers before closing scope");
        AtomicBoolean interrupted = new AtomicBoolean();
        Thread caller = new Thread(() -> {
          try { FusionHostMemory.parallel(pool, 100, i -> {
            try { Thread.sleep(5); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
          }, false); } catch (IllegalStateException expected) { interrupted.set(Thread.currentThread().isInterrupted()); }
        });
        caller.start(); Thread.sleep(15); caller.interrupt(); caller.join(5000);
        check(!caller.isAlive() && interrupted.get(), "Cancellation joins tasks and preserves interrupt");
      } finally { pool.shutdownNow(); pool.awaitTermination(5, TimeUnit.SECONDS); }
      cache.retire(file); Files.delete(file.path);
      check(cache.bytes() == 0, "Parallel cleanup releases pages");
    }
  }
  private static void source(Path root) throws Exception {
    Files.createDirectories(root);
    Options o = new Options(); o.fusionHostMemoryMb = 4096; o.fusionHostCacheMb = 1;
    try (MvsgmMaps maps = new MvsgmMaps(root, true)) {
      maps.hostMemory = new FusionHostMemory(o);
      Camera c = new Camera(100, 100, 8, 8);
      maps.add(new MvsgmMaps.Camera(0, 0, 16, 16, new long[17], false));
      for (int kind = 0; kind < MvsgmMaps.KINDS.length; kind++)
        DepthMapFile.write(maps.file(0, kind), c, 16, 16, new float[256]);
      try (var reader = maps.reader(0)) {
        float[] row = new float[16]; reader.read(MvsgmMaps.DEPTH, 0, row);
        boolean protectedReader = false;
        try { maps.releaseCamera(0); } catch (IOException expected) { protectedReader = true; }
        check(protectedReader, "Camera cannot retire an active scoped source reader");
        maps.hostMemory.cache.trim(); reader.read(MvsgmMaps.DEPTH, 0, row);
        // Simulate pressure without exhausting the host or relying on a sampling race.
        var reporterField = FusionHostMemory.class.getDeclaredField("reporter"); reporterField.setAccessible(true);
        var reporter = (ScheduledExecutorService) reporterField.get(maps.hostMemory);
        reporter.shutdownNow(); reporter.awaitTermination(5, TimeUnit.SECONDS);
        var pressureField = FusionHostMemory.class.getDeclaredField("pressure"); pressureField.setAccessible(true);
        pressureField.setBoolean(maps.hostMemory, true);
        maps.hostMemory.require(0, "simulated pressure");
        check(maps.hostMemory.cache.bytes() == 0, "Pressure trims inactive pages while the source lease protects its file");
        reader.read(MvsgmMaps.DEPTH, 0, row);
        check(row[0] == 0 && maps.hostMemory.cache.bytes() > 0, "Active source reader reopens after pressure trimming");
        ForkJoinPool pressuredRows = new ForkJoinPool(4);
        long caller = Thread.currentThread().threadId(); AtomicInteger serialRows = new AtomicInteger();
        try { FusionHostMemory.parallel(pressuredRows, 32, i -> {
          if (Thread.currentThread().threadId() != caller) throw new AssertionError("Pressure worker escaped serial mode");
          serialRows.incrementAndGet();
        }, maps.hostMemory.pressured()); }
        finally { pressuredRows.shutdown(); }
        check(serialRows.get() == 32, "Pressure reduces row concurrency without dropping work");
        pressureField.setBoolean(maps.hostMemory, false);
      }
      maps.releaseCamera(0);
      check(Files.exists(maps.file(0, 0)), "Unmapping/retirement respects retained file lifetime");
      boolean outputRefused = false;
      try { maps.hostMemory.retainOutput(maps.hostMemory.workBytes + 1); }
      catch (IOException expected) { outputRefused = expected.getMessage().contains("output voxel/tile metadata"); }
      check(outputRefused, "Growing output metadata cannot bypass the shared allowance");
      maps.hostMemory.retainOutput(maps.hostMemory.workBytes / 2);
      boolean futureRefused = false;
      try { maps.hostMemory.require(maps.hostMemory.workBytes / 2 + 1, "future stereo with retained output"); }
      catch (IOException expected) { futureRefused = expected.getMessage().contains("future stereo with retained output"); }
      check(futureRefused, "Stereo admission includes output data retained by earlier cameras");
      maps.hostMemory.retainOutput(0);
      boolean refused = false;
      try { maps.hostMemory.require(Long.MAX_VALUE, "injected unsafe allocation"); }
      catch (IOException expected) { refused = expected.getMessage().contains("No dependencies were removed"); }
      check(refused, "Unsafe upcoming allocation is refused with advice");
    }
  }
  private static void exactFusion(Path root) throws Exception {
    exactFusion(root, false);
  }
  /** Reuse the exact ownership/attribute fixtures to isolate source lifetime alone. */
  static int exactScopedFusion(Path root) throws Exception {
    int before = checks;
    exactFusion(root, true);
    return checks - before;
  }
  private static void exactFusion(Path root, boolean scopedOnly) throws Exception {
    String oldRows = System.getProperty("sgm.fusion.tileRows");
    try {
      // Non-dividing strips exercise halos, marks, surface context and final short strips.
      System.setProperty("sgm.fusion.tileRows", "32");
      for (String strategy : List.of("owner", "owner-consume", "graph-average", "graph-average-precision",
          "graph-average-voxel", "graph-average-tiles", "graph-average-surface")) {
        Path[] out = new Path[2];
        long[] counts = new long[2];
        String[] plans = new String[2];
        for (int mode = 0; mode < 2; mode++) {
          Options o = new Options();
          if (mode == 0 && !scopedOnly) System.clearProperty("sgm.fusion.tileRows");
          else System.setProperty("sgm.fusion.tileRows", "32");
          o.output = Files.createDirectories(root.resolve(strategy + "-" + mode));
          out[mode] = o.output; o.projectDirectory = o.output;
          o.denseMethod = "mvsgm"; o.fusionMode = "image";
          o.fusionStrategy = strategy.startsWith("graph-average") ? "graph-average" : strategy;
          if (strategy.equals("graph-average-precision")) o.fusionSeedOrder = "precision";
          o.pointAttributes = "surface"; o.writePly = true; o.writePointConfidence = true;
          o.cores = 4; o.fusionDepthOutliers = true; o.depthOutlierFilter = "edge-aware";
          o.surfaceConsolidation = "none"; o.speckleSize = 4;
          if (strategy.equals("graph-average-voxel")) o.voxel = .15;
          if (strategy.equals("graph-average-tiles")) o.tiledLasOutput = .5;
          if (strategy.equals("graph-average-surface")) o.surfaceConsolidation = "normal";
          o.minViews = 2; o.compute = "cpu"; o.fusionLocalSigma = 2;
          // 0.1 m baselines give relative sigma ~0.5: without this only unconfirmed edge rows
          // (credited to the mvsgm partner before 2026-11-21) survived the owner strategies.
          o.maxDepthUncertainty = 1;
          int n = 8, w = 96, h = 80;
          Camera[] cameras = new Camera[n]; int[] widths = new int[n], heights = new int[n];
          for (int c = 0; c < n; c++) {
            widths[c] = c % 2 == 0 ? w : w - 16;
            heights[c] = c % 3 == 0 ? h - 8 : h;
            cameras[c] = new Camera(100, 100, (widths[c] - 1) / 2., (heights[c] - 1) / 2.);
            cameras[c].translation[0] = -.1 * c; cameras[c].registered = true;
          }
          if (scopedOnly && mode == 1) o.fusionSourceMaps = "scoped";
          try (var maps = new MvsgmMaps(Files.createDirectories(o.output.resolve("source")), false,
                  o.fusionSourceMaps.equals("scoped"));
              var fusion = new DepthFusion(cameras, widths, heights, o)) {
            if (mode == 1 && !scopedOnly) { o.fusionHostMemoryMb = 4096; o.fusionHostCacheMb = 1; maps.hostMemory = new FusionHostMemory(o); }
            long[] starts = new long[n + 1];
            for (int c = 0; c < n; c++) {
              int cw = widths[c], ch = heights[c];
              int pair = fusion.addPair(c, (c + 1) % n); fusion.markSingleView(pair);
              long[] rows = new long[ch + 1]; for (int y = 0; y <= ch; y++) rows[y] = (long) y * cw;
              maps.add(new MvsgmMaps.Camera(c, pair, cw, ch, rows, false)); starts[c + 1] = starts[c] + (long) cw * ch;
              for (int k = 0; k < MvsgmMaps.KINDS.length; k++) {
                float[] values = new float[cw * ch];
                for (int p = 0; p < values.length; p++) values[p] = k == MvsgmMaps.DEPTH ? 10 + (p / cw % 7) * .0001f
                    : k == MvsgmMaps.RGB ? Float.intBitsToFloat(0x2277AA) : k == MvsgmMaps.LR ? 0 : .9f;
                DepthMapFile.write(maps.file(c, k), cameras[c], cw, ch, values);
              }
            }
            try (FusionOutput output = new FusionOutput(o, fusion, null, null)) {
              Path candidates = Files.createFile(o.output.resolve("empty-candidates"));
              var counters = ImageFusion.run(o, fusion, candidates, starts, output, new TwoViewConfidence.Diagnostics(o), maps);
              plans[mode] = (String) ((Map<?, ?>) counters.memory).get("plan");
              counts[mode] = counters.emitted; output.finish();
            }
            if (mode == 1 && scopedOnly) {
              check(maps.hostMemory == null, "Source-only opt-in creates no memory controller");
              check(((Number) maps.sourceReport().get("mapped_source_bytes")).longValue() == 0,
                  "Scoped fusion closes all source mappings");
            }
            if (mode == 1 && !scopedOnly) check(maps.hostMemory.cache.bytes() <= 1L << 20, "Fusion cache cap enforced");
            if (mode == 1 && !scopedOnly && (o.voxel > 0 || o.tiledLasOutput > 0))
              check(((Number) maps.hostMemory.report().get("peak_output_retained_bytes")).longValue() > 0,
                  "Voxel/tile retention is charged to the shared allowance");
          }
        }
        check(counts[0] > 0 && counts[0] == counts[1], strategy + " counts match");
        if (scopedOnly) check(plans[0].equals(plans[1]), "Source lifetime leaves the fusion plan unchanged");
        for (String file : List.of("dense.ply", "07_results/point-confidence.bin"))
          check(Files.mismatch(out[0].resolve(file), out[1].resolve(file)) == -1,
              strategy + " ordered cloud/attributes/candidate IDs byte equal: " + file);
        if (strategy.equals("graph-average-tiles")) {
          Path a = out[0].resolve("07_results/dense_tiles"), b = out[1].resolve("07_results/dense_tiles");
          try (var files = Files.list(a)) {
            List<Path> tiles = files.filter(p -> p.toString().endsWith(".las")).toList();
            check(!tiles.isEmpty(), "Tiled LAS fixture emits tiles");
            for (Path tile : tiles) check(Files.mismatch(tile, b.resolve(tile.getFileName())) == -1,
                "Tiled LAS bytes preserved: " + tile.getFileName());
          }
        }
      }
    } finally {
      if (oldRows == null) System.clearProperty("sgm.fusion.tileRows"); else System.setProperty("sgm.fusion.tileRows", oldRows);
    }
  }

  /** Planning-only huge raster: refuse a filter that exceeds even the one-worker allowance. */
  private static void oversizedFilter(Path root) throws Exception {
    Options o = new Options(); o.output = Files.createDirectories(root);
    o.memoryMb = 16; o.fusionHostMemoryMb = 4096; o.fusionHostCacheMb = 1;
    o.cores = 4; o.pointAttributes = "none";
    Camera[] cameras = {new Camera(100, 100, 0, 0), new Camera(100, 100, 0, 0)};
    try (MvsgmMaps maps = new MvsgmMaps(root, false);
        DepthFusion fusion = new DepthFusion(cameras, new int[] {10000, 10000},
            new int[] {8000000, 8000000}, o)) {
      maps.hostMemory = new FusionHostMemory(o);
      var constructor = ImageFusion.class.getDeclaredConstructor(Options.class, DepthFusion.class,
          Path.class, long[].class, MvsgmMaps.class);
      constructor.setAccessible(true);
      boolean refused = false;
      try { constructor.newInstance(o, fusion, root.resolve("not-opened"), new long[] {0}, maps); }
      catch (java.lang.reflect.InvocationTargetException e) {
        refused = e.getCause() instanceof IllegalArgumentException
            && e.getCause().getMessage().contains("speckle worker");
      }
      check(refused, "Oversized strip filter is refused before allocating image-sized arrays");
    }
  }
}
