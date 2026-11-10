package sgm;

import static java.lang.foreign.ValueLayout.*;

import java.io.*;
import java.lang.foreign.*;
import java.nio.file.*;
import java.util.*;

/** Banked resident or exact strip-streamed SGM. Path state crosses every strip boundary. */
final class GpuStereo {
  record Plan(
      boolean streamed,
      int rows,
      long bankBytes,
      long deviceBytes,
      long hostBytes,
      String aggregateStorage,
      int blockRows,
      long aggregateBytes) {}

  private static long availableRam() {
    var bean = java.lang.management.ManagementFactory.getOperatingSystemMXBean();
    return bean instanceof com.sun.management.OperatingSystemMXBean os
        ? os.getFreeMemorySize()
        : Long.MAX_VALUE;
  }

  private static long bankBytes(GpuSgm.Device d, Options o) {
    return Math.min(d.allocation(), o.gpuBankMb * 1048576L) / 8 * 8;
  }

  static Plan plan(GpuSgm.Device device, int group, int w, int h, int D, Options o) {
    int sumBytes = StereoCost.aggregateBytes(o);
    String aggregation = o.gpuSgm.equals("esgm") ? "recompute" : o.gpuAggregate;
    long n = (long) w * h, bank = bankBytes(device, o), budget = GpuSgm.budget(device, o);
    int words = (o.census * o.census - 1 + 63) / 64;
    long input = n * (16L * words + 4),
        host = n * (116L + 32 * words) + PointConfidence.extraHostBytes(o, n);
    long hostBudget =
        Math.min(o.memoryMb * 1048576L, (long) (Runtime.getRuntime().maxMemory() * .65))
            - o.gpuHostReserve;
    if (host > hostBudget) return null;
    if (D < 3
        || D > 4096
        || (2L * D + group + 2) * 4 > device.localMemory()
        || bank < 8
        || n * words * 8 > device.allocation()) return null;
    long fullCells = n * D, resident = input + fullCells * (2 + sumBytes) + n * 24 + 64;
    if (!o.gpuSgm.equals("streamed")
        && !o.gpuSgm.equals("hybrid")
        && !o.gpuSgm.equals("esgm")
        && resident <= budget
        && fullCells * sumBytes <= bank * 8
        && n * 24 <= device.allocation())
      return new Plan(false, h, bank, resident, host, "resident", h, 0);
    if (o.gpuSgm.equals("resident") || (long) w * D * 4 > device.allocation()) return null;
    if ((o.gpuSgm.equals("auto") && o.gpuAggregate.equals("auto") || o.gpuSgm.equals("hybrid"))
        && fullCells * sumBytes <= bank * 8) {
      for (int rows = Math.min(h, o.gpuStripRows); rows >= 1; rows /= 2) {
        long cells = (long) w * rows * D;
        long bytes =
            input + fullCells * sumBytes + cells * 2 + (long) w * rows * 24 + 8L * w * D + 256;
        long staging = host + (long) w * rows * 24;
        if (bytes <= budget
            && cells * 2 <= bank * 8
            && (long) w * rows * 24 <= device.allocation()
            && staging <= hostBudget)
          return new Plan(true, rows, bank, bytes, staging, "gpu", h, fullCells * sumBytes);
      }
    }
    if (o.gpuSgm.equals("hybrid")) return null;
    for (int rows = Math.min(h, o.gpuStripRows); rows >= 1; rows /= 2) {
      long cells = (long) w * rows * D,
          bytes = input + cells * (2 + sumBytes) + (long) w * rows * 24 + 8L * w * D + 64;
      long staging = host + (long) w * rows * ((long) sumBytes * D + 24);
      if (bytes <= budget
          && cells * sumBytes <= bank * 8
          && (long) w * rows * 24 <= device.allocation()
          && staging <= hostBudget) {
        long fullAggregate = fullCells * sumBytes;
        boolean fitsRam =
            fullAggregate <= hostBudget - staging && fullAggregate <= availableRam() / 4 * 3;
        if (aggregation.equals("ram") && !fitsRam) continue;
        if (aggregation.equals("disk"))
          return new Plan(true, rows, bank, bytes, staging, "disk", h, fullAggregate);
        if (!aggregation.equals("recompute") && fitsRam)
          return new Plan(
              true, rows, bank, bytes, staging + fullAggregate, "ram", h, fullAggregate);
        long stripeBytes = (long) w * rows * D * sumBytes;
        long stripes = (hostBudget - staging) / stripeBytes;
        if (stripes < 1) continue;
        int blockRows =
            (int) Math.min(h, Math.min(Integer.MAX_VALUE / (long) rows, stripes) * rows);
        long aggregate = (long) w * blockRows * D * sumBytes;
        return new Plan(
            true, rows, bank, bytes, staging + aggregate, "recompute", blockRows, aggregate);
      }
    }
    return null;
  }

  static int limit(GpuSgm.Device d, int group, int w, int h, Options o) {
    int low = 2, high = 4096;
    while (low < high) {
      int mid = (low + high + 1) / 2;
      if (plan(d, group, w, h, mid, o) != null) low = mid;
      else high = mid - 1;
    }
    return low == 2 ? 0 : low;
  }

  static Object report(int w, int h, Options o) {
    if (o.compute.equals("cpu")) return null;
    try {
      var device = GpuSgm.selectedDevice(o);
      int group = GpuSgm.selectedGroup(o);
      Plan p = plan(device, group, w, h, o.disparities, o);
      if (p == null) return null;
      boolean wanted =
          o.gpuSgm.equals("esgm")
              || (o.gpuSgm.equals("auto")
                  && o.gpuAggregate.equals("auto")
                  && p.aggregateStorage.equals("recompute"));
      GpuCandidateStereo.CandidatePlan compact = wanted ? GpuCandidateStereo.plan(o, w, h) : null;
      Plan verification = p;
      long reserve = o.gpuHostReserve;
      if (compact != null) {
        Options exact = o.copy();
        exact.gpuSgm = "streamed";
        exact.gpuAggregate = "recompute";
        exact.gpuHostReserve = 40L * w * h;
        verification = plan(device, group, w, h, o.disparities, exact);
        reserve = exact.gpuHostReserve;
      }
      int blocks = (h + verification.blockRows - 1) / verification.blockRows;
      return Json.object(
          "storage",
          compact != null
              ? "compact_with_sparse_exact_verification"
              : p.aggregateStorage.equals("gpu")
                  ? "hybrid_gpu_aggregate"
                  : p.streamed ? "streamed_" + p.aggregateStorage : "resident_banked",
          "rows",
          compact != null ? compact.rows() : p.rows,
          "bank_bytes",
          p.bankBytes,
          "estimated_device_bytes",
          compact != null
              ? Math.max(compact.deviceBytes(), verification.deviceBytes)
              : p.deviceBytes,
          "estimated_host_bytes",
          compact != null
              ? Math.max(compact.hostBytes(), verification.hostBytes + reserve)
              : p.hostBytes + reserve,
          "host_reserve_bytes",
          reserve,
          "aggregate_bits",
          8 * StereoCost.aggregateBytes(o),
          "aggregate_proved_bound",
          StereoCost.aggregateBound(o),
          "cost_generation_passes",
          compact != null ? o.directions * (2 + blocks) : p.streamed ? o.directions * blocks : 1,
          "cost_generation_passes_are_upper_bound",
          compact != null || p.aggregateStorage.equals("recompute"),
          "aggregate_volume_reads",
          p.streamed && !p.aggregateStorage.equals("gpu") ? o.directions - 1 : 0,
          "aggregate_volume_writes",
          p.streamed && !p.aggregateStorage.equals("gpu") ? o.directions - 1 : 0,
          "aggregate_disk_bytes",
          compact == null && p.aggregateStorage.equals("disk") ? p.aggregateBytes : 0,
          "aggregate_ram_bytes",
          compact != null
              ? verification.aggregateBytes
              : p.aggregateStorage.equals("ram") || p.aggregateStorage.equals("recompute")
                  ? p.aggregateBytes
                  : 0,
          "aggregate_device_bytes",
          p.aggregateStorage.equals("gpu") ? p.aggregateBytes : 0,
          "verification_block_rows",
          verification.blockRows,
          "recomputation_blocks",
          blocks,
          "candidates_per_path",
          compact == null ? null : compact.pool(),
          "candidate_device_bytes",
          compact == null ? 0 : compact.deviceBytes());
    } catch (RuntimeException e) {
      return Json.object("unavailable", e.getMessage());
    }
  }

  static float[] execute(
      Options o,
      long[][] a,
      long[][] b,
      byte[] grayA,
      byte[] grayB,
      boolean[] ta,
      boolean[] tb,
      int w,
      int h,
      boolean reverse,
      CensusSgm.Stats stats,
      byte[] reasons,
      byte[] boundary) {
    Plan selected = plan(GpuSgm.selectedDevice(o), GpuSgm.selectedGroup(o), w, h, o.disparities, o);
    boolean compact =
        o.gpuSgm.equals("esgm")
            || (o.gpuSgm.equals("auto")
                && o.gpuAggregate.equals("auto")
                && selected != null
                && selected.aggregateStorage.equals("recompute"));
    if (compact && GpuCandidateStereo.supported(o, w, h))
      return GpuCandidateStereo.execute(
          o, a, b, grayA, grayB, ta, tb, w, h, reverse, stats, reasons, boundary);
    if (o.gpuSgm.equals("esgm"))
      throw new IllegalArgumentException(
          "No compact Census/16-bit GPU plan fits; use --gpu-sgm streamed for exact recomputation");
    try {
      return executeTargets(
          o, a, b, grayA, grayB, ta, tb, w, h, reverse, stats, reasons, boundary, null, null, null);
    } catch (GpuAggregate.AllocationFailure unavailable) {
      if (!o.gpuAggregate.equals("auto")) throw unavailable;
      Options retry = o.copy();
      retry.gpuSgm = "streamed";
      retry.gpuAggregate = "recompute";
      System.out.println("Planned aggregate RAM unavailable; using exact bounded recomputation.");
      return executeTargets(
          retry, a, b, grayA, grayB, ta, tb, w, h, reverse, stats, reasons, boundary, null, null,
          null);
    }
  }

  static float[] executeTargets(
      Options o,
      long[][] a,
      long[][] b,
      byte[] grayA,
      byte[] grayB,
      boolean[] ta,
      boolean[] tb,
      int w,
      int h,
      boolean reverse,
      CensusSgm.Stats stats,
      byte[] reasons,
      byte[] boundary,
      int[] slots,
      int[] targetPixels,
      int[] candidateSummary) {
    GpuSgm.Device device = GpuSgm.selectedDevice(o);
    Plan plan = plan(device, GpuSgm.selectedGroup(o), w, h, o.disparities, o);
    if (plan == null)
      throw new IllegalArgumentException("No GPU SGM memory plan fits the requested range");
    try (GpuSgm.Compute cl =
            new GpuSgm.Compute(
                o, kernels(o.matchingCost.equals("zncc"), StereoCost.aggregateBytes(o), StereoCost.gpuSource(o)));
        Arena arena = Arena.ofConfined()) {
      int n = w * h,
          D = o.disparities,
          rows = plan.rows,
          words = a.length,
          sumBytes = StereoCost.aggregateBytes(o);
      long stripCells = (long) w * rows * D;
      boolean hybrid = plan.aggregateStorage.equals("gpu");
      MemorySegment first = arena.allocate((long) n * words * 8),
          second = arena.allocate(first.byteSize());
      for (int word = 0; word < words; word++) {
        MemorySegment.copy(
            MemorySegment.ofArray(a[word]), 0, first, (long) word * n * 8, (long) n * 8);
        MemorySegment.copy(
            MemorySegment.ofArray(b[word]), 0, second, (long) word * n * 8, (long) n * 8);
      }
      MemorySegment ga = upload(cl, first), gb = upload(cl, second);
      byte[] ma = new byte[n], mb = new byte[n];
      for (int i = 0; i < n; i++) {
        ma[i] = (byte) (ta[i] ? 1 : 0);
        mb[i] = (byte) (tb[i] ? 1 : 0);
      }
      MemorySegment gma = upload(cl, arena.allocateFrom(JAVA_BYTE, ma)),
          gmb = upload(cl, arena.allocateFrom(JAVA_BYTE, mb)),
          gra = upload(cl, arena.allocateFrom(JAVA_BYTE, grayA)),
          grb = upload(cl, arena.allocateFrom(JAVA_BYTE, grayB));
      MemorySegment[] cost = banks(cl, stripCells * 2, plan.bankBytes),
          sum = banks(cl, (hybrid ? (long) n * D : stripCells) * sumBytes, plan.bankBytes);
      MemorySegment out = cl.buffer((long) w * rows * 24),
          carry0 = cl.buffer(plan.streamed ? 4L * w * D : 8),
          carry1 = cl.buffer(plan.streamed ? 4L * w * D : 8);
      MemorySegment costs = cl.kernel("costs"),
          paths = cl.kernel("paths"),
          winners = cl.kernel("winners");
      // Small range-guard solves (one per pair) are not reported individually.
      if (!o.gpuSmallSolves) System.out.printf(
          Locale.ROOT,
          "GPU SGM storage: %s; %d-row working set, %.1f MiB device; cost=%s; P1/P2=%d/%d%n",
          plan.streamed
              ? "streamed, continuous paths, " + plan.aggregateStorage + " aggregate"
              : "resident allocation banks",
          rows,
          plan.deviceBytes / 1048576.,
          o.matchingCost,
          StereoCost.penalty(o.p1, o),
          StereoCost.p2(o));
      float[] result = new float[n];
      Arrays.fill(result, Float.NaN);
      if (slots != null)
        select(
            MemorySegment.ofArray(candidateSummary),
            0,
            n,
            D,
            o,
            result,
            stats,
            reasons,
            boundary,
            slots,
            -1,
            0);
      Options storage = o.copy();
      storage.tileSize = Math.max(1024, Math.min(262144, w));
      storage.disparities = 1;
      if (plan.aggregateStorage.equals("disk")) {
        Files.createDirectories(o.output);
        long disk = (long) sumBytes * n * D,
            available = Files.getFileStore(o.output).getUsableSpace();
        if (available > 0 && disk + 16777216L > available)
          throw new IllegalArgumentException(
              "Streamed GPU SGM needs "
                  + disk / 1048576
                  + " MiB temporary aggregate storage; insufficient disk space");
      }
      MemorySegment host = arena.allocate(plan.streamed && !hybrid ? stripCells * sumBytes : 8),
          summary = arena.allocate((long) w * rows * 24);
      try (GpuAggregate aggregate =
          plan.streamed && !hybrid
              ? new GpuAggregate(
                  slots == null
                      ? plan.aggregateBytes
                      : Math.min(plan.aggregateBytes, (long) targetPixels.length * D * sumBytes),
                  plan.aggregateStorage.equals("disk"),
                  o.output)
              : null) {
        int blockRows = plan.blockRows;
        MemorySegment checkpoint = arena.allocate(8);
        long started = System.nanoTime();
        int pixelsPerGroup =
            slots == null
                ? blockRows * w
                : (int) Math.min(n, plan.aggregateBytes / ((long) D * sumBytes));
        int targetCount = slots == null ? n : targetPixels.length;
        int groups = (int) (((long) targetCount + pixelsPerGroup - 1) / pixelsPerGroup);
        for (int group = 0; group < groups; group++) {
          int low = group * pixelsPerGroup, high = Math.min(targetCount, low + pixelsPerGroup);
          int blockY = slots == null ? group * blockRows : (targetPixels[low] / w / rows) * rows;
          int blockEnd =
              slots == null
                  ? Math.min(h, blockY + blockRows)
                  : Math.min(h, (targetPixels[high - 1] / w / rows + 1) * rows);
          int[][] directions = {
            {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, 1}, {1, -1}, {-1, -1}
          };
          for (int direction = 0; direction < o.directions; direction++) {
            int dx = directions[direction][0], dy = directions[direction][1];
            MemorySegment input = carry0, output = carry1;
            int stripes = (h + rows - 1) / rows;
            int start = dy == 0 ? blockY / rows : 0;
            int stop =
                plan.aggregateStorage.equals("recompute")
                    ? (dy < 0 ? stripes - blockY / rows : (blockEnd + rows - 1) / rows)
                    : stripes;
            if (dy == 0) stop = (blockEnd + rows - 1) / rows;
            for (int ordinal = start; ordinal < stop; ordinal++) {
              if (Thread.currentThread().isInterrupted())
                throw new IllegalStateException("GPU SGM interrupted");
              int stripe = dy < 0 ? stripes - 1 - ordinal : ordinal,
                  y0 = stripe * rows,
                  actual = Math.min(rows, h - y0);
              if (plan.streamed || direction == 0)
                cost(
                    cl,
                    costs,
                    ga,
                    gb,
                    gma,
                    gmb,
                    gra,
                    grb,
                    cost,
                    sum,
                    plan.bankBytes,
                    w,
                    h,
                    D,
                    o,
                    reverse,
                    words,
                    y0,
                    actual,
                    hybrid ? y0 : 0,
                    !hybrid || direction == 0);
              boolean target = y0 >= blockY && y0 < blockEnd;
              if (plan.streamed && !hybrid && target && direction > 0) {
                MemorySegment data = host.asSlice(0, (long) sumBytes * w * actual * D);
                aggregateTransfer(
                    aggregate, data, y0, w, actual, D, sumBytes, blockY, slots, low, high, false);
                transferBanks(cl, sum, plan.bankBytes, data, false);
              }
              List<Object> arguments = new ArrayList<>();
              bankArguments(arguments, cost, sum, plan.bankBytes, sumBytes);
              Collections.addAll(
                  arguments,
                  gma,
                  gra,
                  w,
                  h,
                  D,
                  dx,
                  dy,
                  StereoCost.penalty(o.p1, o),
                  StereoCost.p2(o),
                  o.sgmEdgeScale,
                  y0,
                  actual,
                  plan.streamed ? 1 : 0,
                  ordinal > 0 ? 1 : 0,
                  hybrid ? y0 : 0,
                  input,
                  output,
                  new GpuSgm.Local(4L * (2 * D + cl.group() + 2)));
              long lines = dy == 0 ? actual : dx == 0 ? w : w + actual - 1;
              cl.run(paths, lines * cl.group(), arguments.toArray());
              if (plan.streamed && target && direction == o.directions - 1) {
                winners(
                    cl,
                    winners,
                    cost,
                    sum,
                    plan.bankBytes,
                    out,
                    summary,
                    w,
                    h,
                    D,
                    o,
                    y0,
                    actual,
                    hybrid ? y0 : 0,
                    result,
                    stats,
                    reasons,
                    boundary,
                    slots,
                    low,
                    high);
              } else if (plan.streamed && !hybrid && target) {
                MemorySegment data = host.asSlice(0, (long) sumBytes * w * actual * D);
                transferBanks(cl, sum, plan.bankBytes, data, true);
                aggregateTransfer(
                    aggregate, data, y0, w, actual, D, sumBytes, blockY, slots, low, high, true);
              }
              MemorySegment swap = input;
              input = output;
              output = swap;
            }
            if (plan.streamed && (long) n * D >= 1000000) {
              cl.transfer(carry0, 0, checkpoint, true);
              System.out.printf(
                  Locale.ROOT,
                  "GPU SGM %s: block %d/%d, path %d/%d complete; %.1f s%n",
                  reverse ? "right" : "left",
                  group + 1,
                  groups,
                  direction + 1,
                  o.directions,
                  (System.nanoTime() - started) / 1e9);
            }
          }
        }
        if (!plan.streamed)
          winners(
              cl,
              winners,
              cost,
              sum,
              plan.bankBytes,
              out,
              summary,
              w,
              h,
              D,
              o,
              0,
              h,
              0,
              result,
              stats,
              reasons,
              boundary,
              null,
              0,
              Integer.MAX_VALUE);
      }
      RunTelemetry.gpuProfile(
          "SGM "
              + (reverse ? "right" : "left")
              + " "
              + w
              + "x"
              + h
              + " D="
              + D
              + " "
              + plan.aggregateStorage,
          cl.profile());
      return result;
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  private static void aggregateTransfer(
      GpuAggregate aggregate,
      MemorySegment data,
      int y0,
      int w,
      int rows,
      int D,
      int bytes,
      int blockY,
      int[] slots,
      int low,
      int high,
      boolean write)
      throws IOException {
    if (slots == null) {
      aggregate.transfer((long) (y0 - blockY) * w * D * bytes, data, write);
      return;
    }
    if (!write) data.fill((byte) 0);
    int first = y0 * w, end = first + w * rows;
    for (int p = first; p < end; ) {
      if (slots[p] < low || slots[p] >= high) {
        p++;
        continue;
      }
      int start = p, slot = slots[p++];
      while (p < end && slots[p] == slot + (p - start) && slots[p] < high) p++;
      aggregate.transfer(
          (long) (slot - low) * D * bytes,
          data.asSlice((long) (start - first) * D * bytes, (long) (p - start) * D * bytes),
          write);
    }
  }

  private static void winners(
      GpuSgm.Compute cl,
      MemorySegment kernel,
      MemorySegment[] cost,
      MemorySegment[] sum,
      long bank,
      MemorySegment out,
      MemorySegment summary,
      int w,
      int h,
      int D,
      Options o,
      int y0,
      int actual,
      int sumY,
      float[] result,
      CensusSgm.Stats stats,
      byte[] reasons,
      byte[] boundary,
      int[] slots,
      int low,
      int high) {
    List<Object> args = new ArrayList<>();
    bankArguments(args, cost, sum, bank, StereoCost.aggregateBytes(o));
    Collections.addAll(args, out, w, h, D, o.census / 2, y0, actual, sumY);
    cl.run(kernel, (long) w * actual, args.toArray());
    MemorySegment data = summary.asSlice(0, (long) w * actual * 24);
    cl.transfer(out, 0, data, true);
    select(data, y0 * w, w * actual, D, o, result, stats, reasons, boundary, slots, low, high);
  }

  private static MemorySegment upload(GpuSgm.Compute cl, MemorySegment data) {
    MemorySegment b = cl.buffer(data.byteSize());
    cl.transfer(b, 0, data, false);
    return b;
  }

  private static MemorySegment[] banks(GpuSgm.Compute cl, long total, long bank) {
    if (total > bank * 8)
      throw new IllegalArgumentException("SGM volume exceeds allocation-bank capacity");
    MemorySegment[] b = new MemorySegment[8];
    for (int i = 0; i < 8; i++) b[i] = cl.buffer(Math.max(8, Math.min(bank, total - i * bank)));
    return b;
  }

  private static void bankArguments(
      List<Object> arguments, MemorySegment[] cost, MemorySegment[] sum, long bytes, int sumBytes) {
    Collections.addAll(arguments, cost);
    Collections.addAll(arguments, sum);
    arguments.add((int) (bytes / 2));
    arguments.add((int) (bytes / sumBytes));
  }

  private static void transferBanks(
      GpuSgm.Compute cl, MemorySegment[] bank, long bytes, MemorySegment data, boolean read) {
    for (long offset = 0; offset < data.byteSize(); offset += bytes)
      cl.transfer(
          bank[(int) (offset / bytes)],
          0,
          data.asSlice(offset, Math.min(bytes, data.byteSize() - offset)),
          read);
  }

  private static void cost(
      GpuSgm.Compute cl,
      MemorySegment kernel,
      MemorySegment a,
      MemorySegment b,
      MemorySegment ma,
      MemorySegment mb,
      MemorySegment grayA,
      MemorySegment grayB,
      MemorySegment[] cost,
      MemorySegment[] sum,
      long bank,
      int w,
      int h,
      int D,
      Options o,
      boolean reverse,
      int words,
      int y0,
      int rows,
      int sumY,
      boolean clearSum) {
    List<Object> args = new ArrayList<>();
    Collections.addAll(args, a, b, ma, mb, grayA, grayB);
    bankArguments(args, cost, sum, bank, StereoCost.aggregateBytes(o));
    Collections.addAll(
        args,
        w,
        h,
        D,
        o.minDisparity,
        reverse ? 1 : 0,
        o.census / 2,
        words,
        y0,
        rows,
        StereoCost.code(o),
        (int) Math.round(o.censusWeight * 1000),
        o.adTruncation,
        o.gradientTruncation,
        sumY,
        clearSum ? 1 : 0);
    cl.run(kernel, (long) w * rows * D, args.toArray());
  }

  static void select(
      MemorySegment host,
      int first,
      int count,
      int D,
      Options o,
      float[] result,
      CensusSgm.Stats stats,
      byte[] reasons,
      byte[] boundary) {
    select(host, first, count, D, o, result, stats, reasons, boundary, null, 0, Integer.MAX_VALUE);
  }

  static void select(
      MemorySegment host,
      int first,
      int count,
      int D,
      Options o,
      float[] result,
      CensusSgm.Stats stats,
      byte[] reasons,
      byte[] boundary,
      int[] slots,
      int low,
      int high) {
    // Element-wise VarHandle reads of a segment cost more than the selection itself; copy the
    // summary into a Java array in bounded chunks and select from that (same values, same order).
    int chunk = Math.max(1, Math.min(count, 1 << 16));
    int[] words = new int[6 * chunk];
    for (int done = 0; done < count; done += chunk) {
      int part = Math.min(chunk, count - done);
      MemorySegment.copy(host, JAVA_INT, done * 24L, words, 0, 6 * part);
      select(words, 0, first + done, part, D, o, result, stats, reasons, boundary, slots, low, high);
    }
  }

  /** Summary words in a Java array: 6 ints per pixel, pixel j of the batch at {@code base0 + 6j}. */
  static void select(
      int[] host,
      int base0,
      int first,
      int count,
      int D,
      Options o,
      float[] result,
      CensusSgm.Stats stats,
      byte[] reasons,
      byte[] boundary,
      int[] slots,
      int low,
      int high) {
    for (int j = 0; j < count; j++) {
      if (slots != null && (slots[first + j] < low || slots[first + j] >= high)) continue;
      int base = base0 + 6 * j;
      int p = first + j,
          best = host[base],
          value = host[base + 1],
          second = host[base + 2],
          before = host[base + 3],
          after = host[base + 4];
      if (best < 0) continue;
      boolean full = host[base + 5] == 3;
      if (full) {
        stats.fullIntervalPixels++;
        if (boundary != null) boundary[p] = 4;
      }
      if (best == 0 || best == D - 1) {
        stats.boundaryRejected++;
        if (reasons != null) reasons[p] = 2;
        if (full && best == 0) {
          stats.lowerBoundaryRejected++;
          if (boundary != null) boundary[p] |= 1;
        }
        if (full && best == D - 1) {
          stats.upperBoundaryRejected++;
          if (boundary != null) boundary[p] |= 2;
        }
        continue;
      }
      if (second != Integer.MAX_VALUE && second - value <= Math.max(1, value * o.uniqueness)) {
        stats.ambiguityRejected++;
        if (reasons != null) reasons[p] = 3;
        continue;
      }
      double offset = before >= 0 && after >= 0 ? StereoCost.subpixel(o, before, value, after) : 0;
      result[p] = (float) (o.minDisparity + best + offset);
      if (o.stereoSelection != null) o.stereoSelection.put(p, value, second, before, after);
      stats.selected++;
      if (reasons != null) reasons[p] = 0;
    }
  }

  static String kernels(boolean zncc, int sumBytes) {
    return kernels(zncc, sumBytes, "");
  }

  /** costExtension: StereoCost.gpuSource(o), empty unless the opt-in census modes are set. */
  static String kernels(boolean zncc, int sumBytes, String costExtension) {
    return (zncc ? "#pragma OPENCL EXTENSION cl_khr_fp64 : enable\n#define ZNCC 1\n" : "")
        + "#pragma OPENCL FP_CONTRACT OFF\n"
        + (sumBytes == 2 ? "typedef ushort SUM;\n" : "typedef int SUM;\n")
        + costExtension
        + KERNELS;
  }

  private static final String KERNELS =
      """
#define CARGS __global ushort *c0,__global ushort *c1,__global ushort *c2,__global ushort *c3,__global ushort *c4,__global ushort *c5,__global ushort *c6,__global ushort *c7
#define SARGS __global SUM *s0,__global SUM *s1,__global SUM *s2,__global SUM *s3,__global SUM *s4,__global SUM *s5,__global SUM *s6,__global SUM *s7
#define CV c0,c1,c2,c3,c4,c5,c6,c7
#define SV s0,s1,s2,s3,s4,s5,s6,s7
int cr(ulong at,int B,CARGS){int bank=(int)(at/(ulong)B),i=(int)(at%(ulong)B);switch(bank){case 0:return c0[i];case 1:return c1[i];case 2:return c2[i];case 3:return c3[i];case 4:return c4[i];case 5:return c5[i];case 6:return c6[i];default:return c7[i];}}
void cw(ulong at,int B,int v,CARGS){int bank=(int)(at/(ulong)B),i=(int)(at%(ulong)B);switch(bank){case 0:c0[i]=v;break;case 1:c1[i]=v;break;case 2:c2[i]=v;break;case 3:c3[i]=v;break;case 4:c4[i]=v;break;case 5:c5[i]=v;break;case 6:c6[i]=v;break;default:c7[i]=v;}}
int sr(ulong at,int B,SARGS){int bank=(int)(at/(ulong)B),i=(int)(at%(ulong)B);switch(bank){case 0:return s0[i];case 1:return s1[i];case 2:return s2[i];case 3:return s3[i];case 4:return s4[i];case 5:return s5[i];case 6:return s6[i];default:return s7[i];}}
void sw(ulong at,int B,int v,SARGS){int bank=(int)(at/(ulong)B),i=(int)(at%(ulong)B);switch(bank){case 0:s0[i]=v;break;case 1:s1[i]=v;break;case 2:s2[i]=v;break;case 3:s3[i]=v;break;case 4:s4[i]=v;break;case 5:s5[i]=v;break;case 6:s6[i]=v;break;default:s7[i]=v;}}
// One 64-bit division per pixel: (bank, offset) of a pixel's first label; label k carries over banks.
typedef struct{int bank;int off;} BK;
BK bk(ulong at,int B){BK r;r.bank=(int)(at/(ulong)B);r.off=(int)(at%(ulong)B);return r;}
int crk(BK b,int k,int B,CARGS){int i=b.off+k,bank=b.bank;while(i>=B){i-=B;bank++;}switch(bank){case 0:return c0[i];case 1:return c1[i];case 2:return c2[i];case 3:return c3[i];case 4:return c4[i];case 5:return c5[i];case 6:return c6[i];default:return c7[i];}}
int srk(BK b,int k,int B,SARGS){int i=b.off+k,bank=b.bank;while(i>=B){i-=B;bank++;}switch(bank){case 0:return s0[i];case 1:return s1[i];case 2:return s2[i];case 3:return s3[i];case 4:return s4[i];case 5:return s5[i];case 6:return s6[i];default:return s7[i];}}
void swk(BK b,int k,int B,int v,SARGS){int i=b.off+k,bank=b.bank;while(i>=B){i-=B;bank++;}switch(bank){case 0:s0[i]=v;break;case 1:s1[i]=v;break;case 2:s2[i]=v;break;case 3:s3[i]=v;break;case 4:s4[i]=v;break;case 5:s5[i]=v;break;case 6:s6[i]=v;break;default:s7[i]=v;}}
__kernel void costs(__global const ulong *a,__global const ulong *b,__global const uchar *ta,__global const uchar *tb,
 __global const uchar *ga,__global const uchar *gb,CARGS,SARGS,int CB,int SB,int w,int h,int D,int md,int reverse,int r,int words,int y0,int rows,int mode,int weight,int adcap,int gradcap,int sumY,int clearSum) {
  size_t i=get_global_id(0),n=(size_t)w*h;if(i>=(size_t)w*rows*D)return;
  int localp=i/D,p=localp+y0*w,k=i%D,x=p%w,y=p/w,disp=md+k;
  long target=(long)x+(reverse?(long)disp:-(long)disp);int value=512;
  if(x>=r&&x<w-r&&y>=r&&y<h-r&&ta[p]&&target>=r&&target<w-r) {
    int q=y*w+(int)target;
    if(tb[q]) {
#ifdef CENSUS_EXT
      value=ccost(a,b,ta,tb,ga,gb,n,p,q,w,h,r,words,mode,weight,adcap,gradcap);
#else
      int ham=0;for(int j=0;j<words;j++)ham+=popcount(a[(size_t)j*n+p]^b[(size_t)j*n+q]);
      value=ham;int bits=(2*r+1)*(2*r+1)-1;
      if(mode==1||mode==2){int difference,cap;
        if(mode==1){difference=abs((int)ga[p]-(int)gb[q]);cap=adcap;}
        else{int ax=(int)ga[p+1]-ga[p-1],ay=(int)ga[p+w]-ga[p-w],bx=(int)gb[q+1]-gb[q-1],by=(int)gb[q+w]-gb[q-w];difference=abs(ax-bx)+abs(ay-by);cap=gradcap;}
        int den=1000*cap;value=(weight*ham*cap+(1000-weight)*min(difference,cap)*bits+den/2)/den;
      }
#endif
#ifdef ZNCC
      if(mode==3){long sa=0,sb=0,saa=0,sbb=0,sab=0;int count=(2*r+1)*(2*r+1);
        for(int dy=-r;dy<=r;dy++)for(int dx=-r;dx<=r;dx++){int av=ga[p+dy*w+dx],bv=gb[q+dy*w+dx];sa+=av;sb+=bv;saa+=av*av;sbb+=bv*bv;sab+=av*bv;}
        long va=count*saa-sa*sa,vb=count*sbb-sb*sb;
        if(va<=0||vb<=0)value=512;else{double correlation=(double)(count*sab-sa*sb)/sqrt((double)va*(double)vb);value=(int)floor((1-fmax(-1.0,fmin(1.0,correlation)))*.5*bits+.5);}
      }
#endif
    }
  }
  cw(i,CB,value,CV);if(clearSum)sw(i+(ulong)sumY*w*D,SB,0,SV);
}
__kernel void paths(CARGS,SARGS,int CB,int SB,__global const uchar *valid,__global const uchar *gray,int w,int h,int D,int dx,int dy,int p1,int p2,int edgeScale,
 int y0,int rows,int streamed,int carry,int sumY,__global const int *incoming,__global int *outgoing,__local int *state) {
  int line=get_group_id(0),tid=get_local_id(0),L=get_local_size(0),x,y;
  if(dy==0){x=dx>0?0:w-1;y=line;}
  else if(dx==0){x=line;y=dy>0?0:rows-1;}
  else if(line<w){x=line;y=dy>0?0:rows-1;}
  else{x=dx>0?0:w-1;y=dy>0?line-w+1:rows-2-(line-w);}
  __local int *prev=state,*next=state+D,*mins=state+2*D,*slot=state+2*D+L;int step=0,minimum=0;
  // Double-buffered atomic minimum: two barriers per step instead of a log2(L) tree.
  if(tid==0){slot[0]=2147483647;slot[1]=2147483647;}barrier(CLK_LOCAL_MEM_FENCE);
  while(x>=0&&x<w&&y>=0&&y<rows) {
    int globalp=(y+y0)*w+x,px=x-dx,py=y+y0-dy;
    int has=px>=0&&px<w&&py>=0&&py<h&&(step>0||(streamed&&carry&&dy!=0))&&valid[py*w+px];
    // x, y and step are uniform per work-group, so these barriers are reached by every item.
    if(step==0&&has){
      for(int k=tid;k<D;k+=L)prev[k]=incoming[(size_t)px*D+k];
      barrier(CLK_LOCAL_MEM_FENCE);
      int carried=2147483647;for(int k=tid;k<D;k+=L)carried=min(carried,prev[k]);
      mins[tid]=carried;barrier(CLK_LOCAL_MEM_FENCE);
      for(int stride=L/2;stride>0;stride/=2){if(tid<stride)mins[tid]=min(mins[tid],mins[tid+stride]);barrier(CLK_LOCAL_MEM_FENCE);}
      minimum=mins[0];barrier(CLK_LOCAL_MEM_FENCE);
    }
    int localP2=p2;if(has&&edgeScale>0){int difference=abs((int)gray[globalp]-(int)gray[py*w+px]);localP2=max(p1+1,(int)((long)p2*edgeScale/(edgeScale+difference)));}
    BK cbase=bk(((ulong)y*w+x)*D,CB),sbase=bk(((ulong)y*w+x)*D+(ulong)sumY*w*D,SB);int mine=2147483647;
    for(int k=tid;k<D;k+=L){int v=crk(cbase,k,CB,CV);
      if(has){int t=min(prev[k],minimum+localP2);if(k>0)t=min(t,prev[k-1]+p1);if(k+1<D)t=min(t,prev[k+1]+p1);v+=t-minimum;}
      next[k]=v;mine=min(mine,v);swk(sbase,k,SB,srk(sbase,k,SB,SV)+v,SV);
      if(streamed&&dy!=0&&y==(dy>0?rows-1:0))outgoing[(size_t)x*D+k]=v;
    }
    // Minimum of this pixel's path costs: the next step's predecessor minimum.
    atomic_min(&slot[step&1],mine);barrier(CLK_LOCAL_MEM_FENCE);
    minimum=slot[step&1];if(tid==0)slot[(step+1)&1]=2147483647;
    barrier(CLK_LOCAL_MEM_FENCE);__local int *tmp=prev;prev=next;next=tmp;x+=dx;y+=dy;step++;
  }
}
__kernel void winners(CARGS,SARGS,int CB,int SB,__global int *out,int w,int h,int D,int r,int y0,int rows,int sumY) {
  int p=get_global_id(0);if((size_t)p>=(size_t)w*rows)return;
  int best=-1,value=2147483647,second=2147483647,before=-1,after=-1,x=p%w,y=p/w+y0;ulong base=(ulong)p*D,agg=base+(ulong)sumY*w*D;
  BK cb=bk(base,CB),sb=bk(agg,SB);
  if(x>=r&&x<w-r&&y>=r&&y<h-r){
    for(int k=0;k<D;k++)if(crk(cb,k,CB,CV)!=512){int s=srk(sb,k,SB,SV);if(s<value){best=k;value=s;}}
    if(best>=0){for(int k=0;k<D;k++)if(abs(k-best)>1&&crk(cb,k,CB,CV)!=512)second=min(second,srk(sb,k,SB,SV));
      if(best>0&&crk(cb,best-1,CB,CV)!=512)before=srk(sb,best-1,SB,SV);
      if(best+1<D&&crk(cb,best+1,CB,CV)!=512)after=srk(sb,best+1,SB,SV);}
  }
  size_t at=(size_t)p*6;out[at]=best;out[at+1]=value;out[at+2]=second;out[at+3]=before;out[at+4]=after;
  out[at+5]=(crk(cb,0,CB,CV)!=512?1:0)|(crk(cb,D-1,CB,CV)!=512?2:0);
}
""";
}
