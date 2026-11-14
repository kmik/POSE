package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import java.util.concurrent.atomic.*;

/** The 89-photo refusal, live export reservations, disk pressure and exact cloud bytes. */
final class DiskAdmissionRegression {
  private static int checks;
  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }
  public static void main(String[] args) throws Exception {
    Path root = Files.createTempDirectory(Path.of("target"), "disk-admission-");
    System.out.println("PASS disk admission=" + run(root));
  }
  static int run(Path root) throws Exception {
    checks = 0; Files.createDirectories(root);
    long pixels = 89L * 2640 * 1978;
    Options o = new Options(); o.writePly = false; o.writePointConfidence = false;
    o.surfaceConsolidation = "none";
    long maps = pixels * 37 + 89L * 6 * DepthMapFile.HEADER;
    MvsgmDiskPlan plan = MvsgmDiskPlan.of(pixels, 89, maps, 0, o);
    // The 54449 MiB bound of the failed run included a 27 B/point cloud spool, removed on
    // 14 November 2026 (one-pass cloud output): the bound is now the exports plus 5%.
    check(plan.worstCase() == pixels * 58 + Math.max(256L << 20, pixels * 58 / 20),
        "All-pixels bound without the cloud spool");
    check((plan.maps() >> 20) == 16399, "Actual case map bound is 16399 MiB");
    check(plan.spoolUpper() == 0 && plan.exportsUpper() == pixels * 58,
        "LAS-only has no cloud staging; surface attribute bytes counted");
    check(plan.startup() < (41672L << 20), "Actual case is admitted with reported free disk");
    check(plan.startup() > plan.maps() && plan.startup() < (18000L << 20),
        "Startup retains map allowance, batch staging and safety headroom");
    String previous = System.getProperty("sgm.mvsgm.diskAvailableMb");
    Path shortage = Files.createDirectories(root.resolve("shortage"));
    boolean startupRefused = false;
    try {
      System.setProperty("sgm.mvsgm.diskAvailableMb", "1");
      try { MvsgmWindow.checkDisk(shortage, plan, 89, 89); }
      catch (IOException expected) {
        startupRefused = expected.getMessage().contains("no temporary maps")
            && expected.getMessage().contains("available 1 MiB");
      }
    } finally {
      if (previous == null) System.clearProperty("sgm.mvsgm.diskAvailableMb");
      else System.setProperty("sgm.mvsgm.diskAvailableMb", previous);
    }
    check(startupRefused && Files.exists(shortage.resolve("05_stereo/mvsgm/disk_plan.json")),
        "Real admission path refuses a map shortage and records its breakdown");
    Json.write(root.resolve("failed_run_reproduction.json"), Json.object("pixels", pixels,
        "map_bytes", plan.maps(), "old_all_pixels_upper_bytes", plan.worstCase(),
        "new_startup_bytes", plan.startup(), "reported_available_bytes", 41672L << 20));
    o.writePointConfidence = true;
    check(MvsgmDiskPlan.of(pixels, 89, maps, 0, o).startup() < (41672L << 20),
        "Streaming confidence records do not require the all-pixels bound");
    o.surfaceConsolidation = "normal";
    var deferred = MvsgmDiskPlan.of(pixels, 89, maps, 0, o);
    check(deferred.startup() >= deferred.worstCase(), "Deferred surface work keeps conservative admission");
    o.surfaceConsolidation = "none"; o.diagnostics = true;
    var diagnostic = MvsgmDiskPlan.of(pixels, 89, maps, maps, o);
    check(diagnostic.startup() >= diagnostic.worstCase(), "Detailed diagnostics keep conservative admission");
    boolean overflow = false;
    try { MvsgmDiskPlan.of(Long.MAX_VALUE, 89, maps, 0, o); }
    catch (ArithmeticException expected) { overflow = true; }
    check(overflow, "Overflow cannot turn an excessive budget into admission");
    AtomicLong available = new AtomicLong(1000);
    OutputDiskSpace space = new OutputDiskSpace(available::get, 100);
    space.exports(500, 50); space.require(400, "map");
    available.set(999);
    boolean refused = false;
    try { space.require(400, "map"); }
    catch (IOException expected) { refused = expected.getMessage().contains("pending final cloud exports"); }
    check(refused, "Map writes preserve outstanding cloud export space");
    available.set(4000); space.exports(0, 0); space.require(3900, "after export");
    parity(root, false, false); parity(root, true, true);
    pressure(root);
    return checks;
  }
  private static void parity(Path root, boolean attributes, boolean ply) throws Exception {
    Path a = Files.createDirectories(root.resolve("baseline-" + attributes));
    Path b = Files.createDirectories(root.resolve("guarded-" + attributes));
    AtomicInteger queries = new AtomicInteger();
    for (int trial = 0; trial < 2; trial++)
      try (CloudWriter writer = new CloudWriter(trial == 0 ? a : b, 1, null, attributes)) {
        writer.writePly(ply);
        if (trial == 1) writer.diskSpace(new OutputDiskSpace(() -> {
          queries.incrementAndGet(); return Long.MAX_VALUE;
        }, 1L << 20));
        for (int i = 0; i < 1000; i++) writer.point(i * .01, i * .03, i * .07, 3, 4, 5);
      }
    check(Files.mismatch(a.resolve("dense.las"), b.resolve("dense.las")) == -1,
        "Live guards preserve exact LAS bytes: attributes=" + attributes);
    if (ply) check(Files.mismatch(a.resolve("dense.ply"), b.resolve("dense.ply")) == -1,
        "Live guards preserve exact PLY bytes");
    else check(!Files.exists(b.resolve("dense.ply")), "PLY off remains off");
    check(queries.get() == 2, "Disk queried per batch/final export, not per point");
  }
  private static void pressure(Path root) throws Exception {
    AtomicLong available = new AtomicLong(Long.MAX_VALUE);
    OutputDiskSpace space = new OutputDiskSpace(available::get, 100);
    Path a = Files.createDirectories(root.resolve("growing"));
    CloudWriter writer = new CloudWriter(a, 1); writer.diskSpace(space);
    for (int i = 0; i < OutputDiskSpace.BATCH; i++) writer.point(i, 0, 1, 0, 0, 0);
    available.set(0); boolean refused = false;
    try { writer.point(1, 1, 1, 0, 0, 0); }
    catch (IOException expected) { refused = true; }
    check(refused && writer.count() == OutputDiskSpace.BATCH, "Growth is refused before the next batch is written");
    writer.abort();
    check(!Files.exists(a.resolve("dense.las")), "Aborted staging publishes no partial cloud");
    try (var files = Files.list(a)) { check(files.findAny().isEmpty(), "Aborted spool is deleted"); }
    available.set(Long.MAX_VALUE);
    Path b = Files.createDirectories(root.resolve("export"));
    writer = new CloudWriter(b, 1); writer.diskSpace(space);
    writer.point(1, 2, 3, 0, 0, 0); available.set(0); refused = false;
    try { writer.close(); } catch (IOException expected) { refused = true; }
    check(refused && !Files.exists(b.resolve("dense.las")), "Final export rechecks actual available space");
    available.set(100); space.require(0, "after failure");
    check(true, "Failure releases pending export reservation");
  }
}
