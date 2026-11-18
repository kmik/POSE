package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.*;

/** Source mappings may close after copying; dependency files and all other planning stay intact. */
final class ScopedSourceMapsRegression {
  private static int checks;
  private static void check(boolean value, String why) {
    checks++; if (!value) throw new AssertionError(why);
  }
  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Path root = Files.createTempDirectory(Path.of("target"), "source-scopes-");
    try { System.out.println("PASS scoped source maps: " + run(root)); }
    finally { LargeFormatRegression.delete(root); }
  }
  static int run(Path root) throws Exception {
    checks = 0; Files.createDirectories(root);
    check(new Options().fusionSourceMaps.equals("retained"), "Default stays retained");
    Options o = Options.parse(new String[] {"-i", ".", "--dense-method", "mvsgm", "--fusion-mode", "image",
        "--fusion-source-maps", "scoped", "--fusion-host-memory-mb", "0"});
    check(o.fusionHostMemoryMb == 0 && o.copy().fusionSourceMaps.equals("scoped"), "Independent setting copies");
    check(GuiOptions.all().stream().anyMatch(f -> f.flag().equals("--fusion-source-maps")
        && f.choices().containsAll(List.of("retained", "scoped"))), "GUI discovers the lifetime choices");
    boolean rejected = false;
    try { Options.parse(new String[] {"-i", ".", "--fusion-source-maps", "scoped"}); }
    catch (IllegalArgumentException expected) { rejected = true; }
    check(rejected, "Inapplicable option is refused");
    for (int count : new int[] {32, 256, 3013}) scaling(root.resolve("scale-" + count), count);
    leases(root.resolve("leases"));
    barriers(root.resolve("barriers"));
    checks += FusionHostMemoryRegression.exactScopedFusion(root.resolve("fusion"));
    return checks;
  }
  private static void add(MvsgmMaps maps, int c, int w, int h) throws IOException {
    long[] rows = new long[h + 1]; for (int y = 0; y <= h; y++) rows[y] = (long) y * w;
    maps.add(new MvsgmMaps.Camera(c, c, w, h, rows, false));
    float[] z = new float[w * h];
    for (int p = 0; p < z.length; p++) z[p] = c + p / 1024f;
    DepthMapFile.write(maps.file(c, 0), new Camera(100, 100, 0, 0), w, h, z);
  }
  private static long number(MvsgmMaps maps, String key) {
    return ((Number) maps.sourceReport().get(key)).longValue();
  }
  private static void scaling(Path root, int count) throws Exception {
    Files.createDirectories(root);
    long originalMapped = MappedFiles.liveBytes();
    try (MvsgmMaps maps = new MvsgmMaps(root, false, true)) {
      float[] row = new float[17];
      for (int c = 0; c < count; c++) {
        add(maps, c, 17, 9);
        try (var source = maps.reader(c)) {
          source.read(0, 8 * 17, row);
          check(row[16] == c + 152 / 1024f, "Exact source values");
        }
        check(number(maps, "mapped_source_bytes") == 0, "Each source scope unmaps immediately");
        check(Files.isRegularFile(maps.file(0, 0)), "Late dependency file stays available");
      }
      // Every camera remains a disk dependency; reopen first and last after all later cameras.
      for (int c : new int[] {0, count - 1, 0}) try (var source = maps.reader(c)) {
        source.read(0, 0, row);
        check(row[0] == c && row[16] == c + 16 / 1024f, "Late reopening is exact");
      }
      check(maps.cameras().size() == count, "No dependency camera was retired");
      check(number(maps, "peak_mapped_source_bytes") == 17L * 9 * 4, "Extents do not grow with camera count");
      check(number(maps, "file_mapping_opens") == count + 3L, "Reopening exercised");
      check(number(maps, "file_mapping_opens") == number(maps, "file_mapping_closes"), "All opens have closed");
      check(MappedFiles.liveBytes() == originalMapped, "Source scopes release mapped extents");
      System.out.println("Scoped source scaling " + count + " cameras: " + maps.sourceReport());
    }
    try (var files = Files.list(root)) { check(files.findAny().isEmpty(), "Final retirement removes disposable files"); }
  }
  private static void leases(Path root) throws Exception {
    Files.createDirectories(root);
    try (MvsgmMaps maps = new MvsgmMaps(root, true, true)) {
      add(maps, 0, 8, 8);
      var a = maps.reader(0); var b = maps.reader(0);
      float[] row = new float[8]; a.read(0, 0, row);
      var borrowed = a.segment(0); a.close();
      check(number(maps, "mapped_source_bytes") == 256, "Another active reader protects mappings");
      check(borrowed.get(MappedFiles.FLOAT, 0) == 0, "Arena remains readable while another lease is active");
      b.read(0, 8, row); check(row[0] == 8 / 1024f, "Remaining reader works");
      boolean refused = false;
      try { maps.releaseCamera(0); } catch (IOException expected) { refused = true; }
      check(refused, "Retirement cannot invalidate an active reader");
      b.close(); b.close();
      boolean unmapped = false;
      try { borrowed.get(MappedFiles.FLOAT, 0); } catch (IllegalStateException expected) { unmapped = true; }
      check(unmapped, "Last lease actually closes the arena, not only its accounting");
      check(number(maps, "mapped_source_bytes") == 0, "Last reader unmaps, close is idempotent");
      check(Files.exists(maps.file(0, 0)) && maps.camera(0) != null, "Unmapping does not retire files/camera");
      maps.releaseCamera(0);
      check(Files.exists(maps.file(0, 0)), "Retained-file setting respected");
    }
    try (MvsgmMaps legacy = new MvsgmMaps(root, true)) {
      add(legacy, 1, 8, 8);
      try (var source = legacy.reader(1)) { source.read(0, 0, new float[8]); }
      check(number(legacy, "mapped_source_bytes") == 256, "Default keeps the legacy mapping lifetime");
    }
  }
  private static void barriers(Path root) throws Exception {
    Files.createDirectories(root);
    ForkJoinPool pool = new ForkJoinPool(4);
    try (MvsgmMaps maps = new MvsgmMaps(root, true, true)) {
      add(maps, 0, 8, 8);
      AtomicInteger active = new AtomicInteger();
      CountDownLatch started = new CountDownLatch(4);
      boolean failed = false;
      try (var source = maps.reader(0)) {
        FusionHostMemory.parallel(pool, 64, i -> {
          active.incrementAndGet();
          try {
            started.countDown();
            if (!started.await(5, TimeUnit.SECONDS)) throw new AssertionError("Workers did not start");
            source.read(0, 0, new float[8]);
            if (i == 0) throw new IllegalArgumentException("Injected source-copy failure");
            Thread.sleep(20);
          } catch (InterruptedException e) { throw new IllegalStateException(e); }
          finally { active.decrementAndGet(); }
        }, false);
      } catch (IllegalArgumentException expected) { failed = true; }
      check(failed && active.get() == 0, "Copy failure joins all siblings before source close");
      check(number(maps, "mapped_source_bytes") == 0, "Failure releases mappings");
      CountDownLatch entered = new CountDownLatch(4);
      AtomicBoolean interrupted = new AtomicBoolean();
      AtomicReference<Throwable> unexpected = new AtomicReference<>();
      Thread caller = new Thread(() -> {
        try (var source = maps.reader(0)) {
          FusionHostMemory.parallel(pool, 4096, i -> {
            active.incrementAndGet();
            try { source.read(0, 0, new float[8]); entered.countDown(); Thread.sleep(20); }
            catch (InterruptedException e) { throw new IllegalStateException(e); }
            finally { active.decrementAndGet(); }
          }, false);
        } catch (IllegalStateException expected) { interrupted.set(Thread.currentThread().isInterrupted()); }
        catch (Throwable e) { unexpected.set(e); }
      }, "scoped-source-cancellation-test");
      caller.start();
      check(entered.await(5, TimeUnit.SECONDS), "Cancellation workers started");
      caller.interrupt(); caller.join(10000);
      check(!caller.isAlive() && unexpected.get() == null && interrupted.get() && active.get() == 0,
          "Cancellation joins active reads and preserves interrupt status");
      check(number(maps, "mapped_source_bytes") == 0, "Cancellation releases mappings");
    } finally { pool.shutdown(); check(pool.awaitTermination(10, TimeUnit.SECONDS), "Copy workers terminate"); }
  }
}
