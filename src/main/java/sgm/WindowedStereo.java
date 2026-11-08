package sgm;

import static java.lang.foreign.ValueLayout.*;

import java.io.*;
import java.lang.foreign.*;
import java.nio.file.*;
import java.util.*;

/** Exact continuous-path SGM with bounded input strips and disk-backed selection/filter rasters. */
final class WindowedStereo implements AutoCloseable {
  static boolean shouldUse(int width, int height, Options o) {
    if (o.windowedMode.equals("off")) return false;
    if (o.windowedMode.equals("on")) return true;
    long pixels = (long) width * height, words = (o.census * o.census - 1 + 63) / 64;
    long estimate = pixels * (148 + 32 * words) + PointConfidence.extraHostBytes(o, pixels);
    if (o.compute.equals("cpu") && !o.tiled)
      estimate += pixels * o.disparities * (2 + StereoCost.aggregateBytes(o));
    if (estimate > MemorySafety.snapshot().allowance(o.memoryMb * 1048576L)
        || 10 * pixels > RasterCache.limit(o)) return true;
    return !o.compute.equals("cpu")
        && o.disparities
            > GpuSgm.disparityLimit(Math.max(width, height), Math.min(width, height), o);
  }

  static long hostEstimate(int width, int disparities, int rows, Options o) {
    long fixed = 8L * width * disparities + Math.max(4, Math.min(64, o.memoryMb / 8)) * 1048576L;
    long perRow =
        (long) width
            * (disparities * (2 + 2 * StereoCost.aggregateBytes(o))
                + 256
                + 64 * ((o.census * o.census - 1 + 63) / 64));
    return fixed + perRow * (rows + 2L * o.census) + 64 * 1048576L;
  }

  static int limit(int width, int height, Options o) {
    long allowance = MemorySafety.snapshot().allowance(o.memoryMb * 1048576L);
    GpuSgm.Device device = null;
    if (!o.compute.equals("cpu"))
      try {
        device = GpuSgm.selectedDevice(o);
      } catch (RuntimeException e) {
        if (o.compute.equals("gpu")) throw e;
      }
    for (int d = Math.min(4096, width - o.minDisparity); d >= 3; d--) {
      if (hostEstimate(width, d, 1, o) > allowance) continue;
      if (device != null
          && (4L * width * d > device.allocation()
              || 4L * (2 * d + GpuSgm.selectedGroup(o) + 2) > device.localMemory()
              || hostEstimate(width, d, 1, o) > GpuSgm.budget(device, o))) continue;
      return d;
    }
    return 0;
  }

  static final int LEFT = 0,
      RIGHT = 1,
      LM = 2,
      LS = 3,
      RM = 4,
      RS = 5,
      CHECK = 6,
      FINAL = 7,
      PHOTO = 8,
      LR = 9,
      CONFIDENCE = 10,
      REVERSE = 11,
      FIELDS = 12;
  final WindowedRasters store;
  final WindowedRasters.Image left, right;
  final Options o;
  final int w, h;
  final PagedVolume result;
  private PagedVolume descriptorA, descriptorB;
  final CensusSgm.Stats stats = new CensusSgm.Stats();
  GpuSgm.Compute cl;
  int rows;
  long ioBytes, matchNanos, filterNanos;

  WindowedStereo(
      WindowedRasters store,
      WindowedRasters.Image left,
      WindowedRasters.Image right,
      Options options)
      throws IOException {
    this.store = store;
    this.left = left;
    this.right = right;
    o = options;
    w = left.width;
    h = left.height;
    if (o.pyramidLevels != 1)
      throw new IllegalArgumentException(
          "Windowed stereo currently requires --pyramid-levels 1; no pyramid is silently"
              + " substituted");
    rows = Math.max(1, Math.min(h, o.gpuStripRows));
    long fixed = 8L * w * o.disparities + store.pool.limit + 64 * 1048576L;
    long perRow =
        (long) w
            * (o.disparities * (2 + 2 * StereoCost.aggregateBytes(o))
                + 256
                + 64 * ((o.census * o.census - 1 + 63) / 64));
    long allowance = MemorySafety.snapshot().allowance(o.memoryMb * 1048576L);
    while (rows > 1 && fixed + perRow * (rows + 2 * o.census) > allowance)
      rows = Math.max(1, rows / 2);
    if (fixed + perRow * (rows + 2 * o.census) > allowance)
      throw new IllegalArgumentException(
          "Windowed SGM path carry and minimum strip exceed safe host budget");
    long disk = (long) w * h * (192L + StereoCost.aggregateBytes(o) * o.disparities);
    Files.createDirectories(o.output);
    long free = Files.getFileStore(o.output).getUsableSpace();
    if (disk + 64 * 1048576L > free)
      throw new IOException(
          "Windowed SGM needs approximately "
              + disk / 1048576
              + " MiB temporary disk, available "
              + free / 1048576
              + " MiB");
    if (!o.compute.equals("cpu"))
      try {
        cl = new GpuSgm.Compute(o, kernels(o));
        long gpuBudget = GpuSgm.budget(cl.device, o);
        while (rows > 1
            && (fixed + perRow * (rows + 2 * o.census) > gpuBudget
                || (long) rows * w * o.disparities * Math.max(2, StereoCost.aggregateBytes(o))
                    > cl.device.allocation())) rows = Math.max(1, rows / 2);
        if (8L * w * o.disparities + perRow * (rows + 2 * o.census) > gpuBudget
            || 4L * w * o.disparities > cl.device.allocation()
            || 4L * (2 * o.disparities + cl.group() + 2) > cl.device.localMemory())
          throw new IllegalArgumentException(
              "Windowed GPU carry/strip/local state exceeds device budget");
      } catch (RuntimeException failure) {
        if (cl != null) cl.close();
        cl = null;
        if (o.compute.equals("gpu")) throw failure;
        System.out.println(
            "Windowed SGM GPU unavailable; bounded CPU path: " + failure.getMessage());
      }
    try {
      result = store.volume((long) w * h * FIELDS, Float.floatToRawIntBits(Float.NaN));
    } catch (IOException | RuntimeException failure) {
      if (cl != null) cl.close();
      throw failure;
    }
    System.out.printf(
        Locale.ROOT,
        "Windowed SGM: %s, %dx%d, %d disparities, %d-row input strips; %.1f MiB disk plan;"
            + " continuous %d paths%n",
        cl == null ? "CPU" : "OpenCL GPU",
        w,
        h,
        o.disparities,
        rows,
        disk / 1048576.,
        o.directions);
  }

  float get(long p, int field) {
    return Float.intBitsToFloat(result.get(p * FIELDS + field));
  }

  void put(long p, int field, float value) {
    result.set(p * FIELDS + field, Float.floatToRawIntBits(value));
  }

  float[] floats(int field, int y, int length) {
    float[] out = new float[w * length];
    for (int i = 0; i < out.length; i++) out[i] = get((long) y * w + i, field);
    return out;
  }

  void write(int field, int y, int length, float[] values, int offset) {
    for (int i = 0; i < w * length; i++) put((long) y * w + i, field, values[offset + i]);
  }

  void compute() throws IOException {
    long began = System.nanoTime();
    prepareDescriptors();
    match(false, stats);
    match(true, new CensusSgm.Stats());
    matchNanos = System.nanoTime() - began;
    began = System.nanoTime();
    filter();
    filterNanos = System.nanoTime() - began;
    Json.write(
        (o.stereoReportDirectory == null ? o.output : o.stereoReportDirectory)
            .resolve("windowed_stereo.json"),
        Json.object(
            "backend",
            cl == null ? "cpu" : "gpu",
            "width",
            w,
            "height",
            h,
            "disparities",
            o.disparities,
            "strip_rows",
            rows,
            "aggregate_io_bytes",
            ioBytes,
            "matching_nanoseconds",
            matchNanos,
            "filter_nanoseconds",
            filterNanos,
            "continuous_paths",
            true,
            "host_estimated_bytes",
            hostEstimate(w, o.disparities, rows, o),
            "raster_page_reads",
            store.pool.reads,
            "raster_page_writes",
            store.pool.writes,
            "raster_page_evictions",
            store.pool.evictions));
  }

  private record Input(
      byte[] grayA,
      byte[] grayB,
      byte[] maskA,
      byte[] maskB,
      long[][] a,
      long[][] b,
      boolean[] ta,
      boolean[] tb,
      int y,
      int height) {}

  private void prepareDescriptors() throws IOException {
    int words = (o.census * o.census - 1 + 63) / 64, fields = 2 * words + 1;
    descriptorA = store.volume((long) w * h * fields, 0);
    descriptorB = store.volume((long) w * h * fields, 0);
    for (boolean reverse : new boolean[] {false, true}) {
      var image = reverse ? right : left;
      var descriptor = reverse ? descriptorB : descriptorA;
      try (var work =
          WorkProgress.start("Windowed Census " + (reverse ? "right" : "left"), h, "rows")) {
        for (int y = 0; y < h; y += rows) {
          int top = Math.max(0, y - o.census / 2),
              bottom = Math.min(h, y + rows + o.census / 2),
              height = bottom - top,
              actual = Math.min(rows, h - y);
          byte[] gray = image.bytes(image.gray, top, height, 1),
              mask = image.bytes(image.mask, top, height, 1);
          long[][] bits;
          boolean[] textured;
          if (o.censusMode.equals("color-distance")) {
            var value =
                CensusSgm.colorCensus(
                    image.bytes(image.color, top, height, 3), w, height, o.census);
            bits = value.bits;
            textured = value.textured;
          } else {
            bits = CensusSgm.census(gray, w, height, o.census);
            textured = CensusSgm.textureMask(gray, w, height, o.census);
          }
          for (int p = 0; p < actual * w; p++) {
            int local = (y - top) * w + p;
            long base = ((long) y * w + p) * fields;
            for (int j = 0; j < words; j++) {
              long value = bits[j][local];
              descriptor.set(base + 2 * j, (int) value);
              descriptor.set(base + 2 * j + 1, (int) (value >>> 32));
            }
            descriptor.set(base + 2 * words, textured[local] && mask[local] != 0 ? 1 : 0);
          }
          work.update(y + actual);
        }
        work.done();
      }
    }
  }

  private Input input(boolean reverse, int y, int actual) {
    var a = reverse ? right : left;
    var b = reverse ? left : right;
    // --census-shift reads shifted windows up to S rows away (0 when off: halo unchanged).
    int halo = o.census / 2 + 1 + o.censusShift,
        top = Math.max(0, y - halo),
        bottom = Math.min(h, y + actual + halo),
        height = bottom - top;
    byte[] ga = a.bytes(a.gray, top, height, 1),
        gb = b.bytes(b.gray, top, height, 1),
        ma = a.bytes(a.mask, top, height, 1),
        mb = b.bytes(b.mask, top, height, 1);
    int words = (o.census * o.census - 1 + 63) / 64, fields = words * 2 + 1, np = w * height;
    long[][] da = new long[words][np], db = new long[words][np];
    boolean[] ta = new boolean[np], tb = new boolean[np];
    var cachedA = reverse ? descriptorB : descriptorA;
    var cachedB = reverse ? descriptorA : descriptorB;
    for (int p = 0; p < np; p++) {
      long at = ((long) top * w + p) * fields;
      for (int j = 0; j < words; j++) {
        da[j][p] =
            (cachedA.get(at + 2 * j) & 0xffffffffL) | ((long) cachedA.get(at + 2 * j + 1) << 32);
        db[j][p] =
            (cachedB.get(at + 2 * j) & 0xffffffffL) | ((long) cachedB.get(at + 2 * j + 1) << 32);
      }
      ta[p] = cachedA.get(at + 2 * words) != 0;
      tb[p] = cachedB.get(at + 2 * words) != 0;
    }
    return new Input(ga, gb, ma, mb, da, db, ta, tb, top, height);
  }

  private void match(boolean reverse, CensusSgm.Stats statistics) throws IOException {
    int d = o.disparities, sb = StereoCost.aggregateBytes(o), r = o.census / 2;
    long count = (long) w * h * d;
    try (var aggregate = new GpuAggregate(count * sb, true, o.output);
        Arena arena = Arena.ofConfined()) {
      long cells = (long) w * rows * d;
      var host = arena.allocate(cells * sb, 8);
      var costsHost = cl == null ? arena.allocate(cells * 2, 8) : MemorySegment.NULL;
      MemorySegment[] cb = new MemorySegment[8], ab = new MemorySegment[8];
      MemorySegment ka = null,
          kp = null,
          kw = null,
          ga = null,
          gb = null,
          ta = null,
          tb = null,
          da = null,
          db = null,
          c0 = null,
          c1 = null,
          out = null;
      int halo = r + 1 + o.censusShift,
          maxPixels = w * Math.min(h, rows + 2 * halo),
          words = (o.census * o.census - 1 + 63) / 64;
      if (cl != null) {
        for (int i = 0; i < 8; i++) {
          cb[i] = cl.buffer(i == 0 ? cells * 2 : 8);
          ab[i] = cl.buffer(i == 0 ? cells * sb : 8);
        }
        ga = cl.buffer(maxPixels);
        gb = cl.buffer(maxPixels);
        ta = cl.buffer(maxPixels);
        tb = cl.buffer(maxPixels);
        da = cl.buffer(8L * maxPixels * words);
        db = cl.buffer(8L * maxPixels * words);
        c0 = cl.buffer(4L * w * d);
        c1 = cl.buffer(4L * w * d);
        out = cl.buffer((long) w * rows * 24);
        ka = cl.kernel("costs");
        kp = cl.kernel("paths");
        kw = cl.kernel("winners");
      }
      int[] previous = cl == null ? new int[w * d] : null, row = cl == null ? new int[w * d] : null;
      int[] previousMin=cl==null?new int[w]:null,rowMin=cl==null?new int[w]:null;
      int[][] paths = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, 1}, {1, -1}, {-1, -1}};
      int stripes = (h + rows - 1) / rows;
      for (int path = 0; path < o.directions; path++) {
        int dx = paths[path][0], dy = paths[path][1];
        MemorySegment incoming = c0, outgoing = c1;
        try (var work =
            WorkProgress.start(
                "Windowed SGM "
                    + (reverse ? "right" : "left")
                    + " path "
                    + (path + 1)
                    + "/"
                    + o.directions,
                stripes,
                "strips")) {
          for (int ordinal = 0; ordinal < stripes; ordinal++) {
            if (Thread.currentThread().isInterrupted())
              throw new IOException("Windowed SGM interrupted");
            int stripe = dy < 0 ? stripes - 1 - ordinal : ordinal,
                y0 = stripe * rows,
                actual = Math.min(rows, h - y0);
            Input in = input(reverse, y0, actual);
            var data = host.asSlice(0, (long) w * actual * d * sb);
            if (path == 0) data.fill((byte) 0);
            else {
              aggregate.transfer((long) y0 * w * d * sb, data, false);
              ioBytes += data.byteSize();
            }
            if (cl != null) {
              try (Arena staging = Arena.ofConfined()) {
                int np = in.grayA.length;
                var bitsA = staging.allocate(8L * np * words, 8);
                var bitsB = staging.allocate(8L * np * words, 8);
                for (int j = 0; j < words; j++) {
                  MemorySegment.copy(
                      MemorySegment.ofArray(in.a[j]), 0, bitsA, 8L * np * j, 8L * np);
                  MemorySegment.copy(
                      MemorySegment.ofArray(in.b[j]), 0, bitsB, 8L * np * j, 8L * np);
                }
                byte[] va = new byte[np], vb = new byte[np];
                for (int i = 0; i < np; i++) {
                  va[i] = (byte) (in.ta[i] ? 1 : 0);
                  vb[i] = (byte) (in.tb[i] ? 1 : 0);
                }
                cl.transfer(da, 0, bitsA, false);
                cl.transfer(db, 0, bitsB, false);
                cl.transfer(ga, 0, staging.allocateFrom(JAVA_BYTE, in.grayA), false);
                cl.transfer(gb, 0, staging.allocateFrom(JAVA_BYTE, in.grayB), false);
                cl.transfer(ta, 0, staging.allocateFrom(JAVA_BYTE, va), false);
                cl.transfer(tb, 0, staging.allocateFrom(JAVA_BYTE, vb), false);
                cl.transfer(ab[0], 0, data, false);
                List<Object> args = new ArrayList<>();
                Collections.addAll(args, da, db, ta, tb, ga, gb);
                banks(args, cb, ab, cells, sb);
                Collections.addAll(
                    args,
                    w,
                    h,
                    d,
                    o.minDisparity,
                    reverse ? 1 : 0,
                    r,
                    words,
                    y0,
                    actual,
                    StereoCost.code(o),
                    (int) Math.round(o.censusWeight * 1000),
                    o.adTruncation,
                    o.gradientTruncation,
                    0,
                    0,
                    in.y,
                    in.height);
                cl.run(ka, (long) w * actual * d, args.toArray());
                args.clear();
                banks(args, cb, ab, cells, sb);
                Collections.addAll(
                    args,
                    ta,
                    ga,
                    w,
                    h,
                    d,
                    dx,
                    dy,
                    StereoCost.penalty(o.p1, o),
                    StereoCost.p2(o),
                    o.sgmEdgeScale,
                    y0,
                    actual,
                    1,
                    ordinal > 0 ? 1 : 0,
                    0,
                    incoming,
                    outgoing,
                    in.y,
                    new GpuSgm.Local(4L * (2 * d + cl.group() + 2)));
                long lines = dy == 0 ? actual : dx == 0 ? w : w + actual - 1;
                cl.run(kp, lines * cl.group(), args.toArray());
                if (path == o.directions - 1) {
                  args.clear();
                  banks(args, cb, ab, cells, sb);
                  Collections.addAll(args, out, w, h, d, r, y0, actual, 0);
                  cl.run(kw, (long) w * actual, args.toArray());
                  var selected = staging.allocate((long) w * actual * 24, 8);
                  cl.transfer(out, 0, selected, true);
                  select(selected, reverse, y0, actual, statistics);
                } else {
                  cl.transfer(ab[0], 0, data, true);
                  aggregate.transfer((long) y0 * w * d * sb, data, true);
                  ioBytes += data.byteSize();
                }
              }
            } else {
              var costs = costsHost.asSlice(0, (long) w * actual * d * 2);
              costs.fill((byte) 0);
              for (int yy = 0; yy < actual; yy++)
                for (int x = 0; x < w; x++)
                  for (int k = 0; k < d; k++) {
                    int p = (y0 + yy - in.y) * w + x,
                        target = x + (reverse ? o.minDisparity + k : -(o.minDisparity + k)),
                        value = 512;
                    if (in.ta[p] && x >= r && x < w - r && target >= r && target < w - r) {
                      int q = (y0 + yy - in.y) * w + target;
                      if (in.tb[q])
                        value = StereoCost.cost(o, in.a, in.b, in.grayA, in.grayB, in.ta, in.tb, p, q, w, in.grayA.length / w);
                    }
                    costs.set(JAVA_SHORT, (((long) yy * w + x) * d + k) * 2, (short) value);
                  }
              int firstY = dy < 0 ? actual - 1 : 0, stepY = dy < 0 ? -1 : 1;
              for (int yi = 0, yy = firstY; yi < actual; yi++, yy += stepY) {
                int gy = y0 + yy;
                for (int xi = 0, x = dx < 0 ? w - 1 : 0; xi < w; xi++, x += dx < 0 ? -1 : 1) {
                  int px = x - dx, py = gy - dy;
                  boolean has =
                      px >= 0 && px < w && py >= 0 && py < h && in.ta[(py - in.y) * w + px];
                  int[] prev = dy == 0 ? row : previous;
                  int pb = has ? px * d : 0;
                  int min=has?(dy==0?rowMin:previousMin)[px]:Integer.MAX_VALUE;
                  int currentMin=Integer.MAX_VALUE;
                  int lp2 =
                      has
                          ? StereoCost.pathP2(
                              StereoCost.penalty(o.p1, o),
                              StereoCost.p2(o),
                              o.sgmEdgeScale,
                              in.grayA,
                              (gy - in.y) * w + x,
                              (py - in.y) * w + px)
                          : StereoCost.p2(o);
                  for (int k = 0; k < d; k++) {
                    long at = ((long) yy * w + x) * d + k;
                    int v = costs.get(JAVA_SHORT, at * 2) & 65535;
                    if (has) {
                      int transition = Math.min(prev[pb + k], min + lp2);
                      if (k > 0)
                        transition =
                            Math.min(transition, prev[pb + k - 1] + StereoCost.penalty(o.p1, o));
                      if (k + 1 < d)
                        transition =
                            Math.min(transition, prev[pb + k + 1] + StereoCost.penalty(o.p1, o));
                      v += transition - min;
                    }
                    row[x * d + k] = v;
                    currentMin=Math.min(currentMin,v);
                    int sum = read(data, at, sb) + v;
                    write(data, at, sb, sum);
                  }
                  rowMin[x]=currentMin;
                }
                int[] swap = previous;
                previous = row;
                row = swap;
                swap=previousMin;previousMin=rowMin;rowMin=swap;
              }
              if (path == o.directions - 1) {
                try (Arena staging = Arena.ofConfined()) {
                  var selected = staging.allocate((long) w * actual * 24, 8);
                  for (int p = 0; p < w * actual; p++) {
                    long base = (long) p * d;
                    int best = -1,
                        value = Integer.MAX_VALUE,
                        second = Integer.MAX_VALUE,
                        before = -1,
                        after = -1;
                    int x = p % w, y = y0 + p / w;
                    if (x >= r && x < w - r && y >= r && y < h - r) {
                      for (int k = 0; k < d; k++)
                        if ((costs.get(JAVA_SHORT, (base + k) * 2) & 65535) != 512
                            && read(data, base + k, sb) < value) {
                          best = k;
                          value = read(data, base + k, sb);
                        }
                      if (best >= 0) {
                        for (int k = 0; k < d; k++)
                          if (Math.abs(k - best) > 1
                              && (costs.get(JAVA_SHORT, (base + k) * 2) & 65535) != 512)
                            second = Math.min(second, read(data, base + k, sb));
                        if (best > 0
                            && (costs.get(JAVA_SHORT, (base + best - 1) * 2) & 65535) != 512)
                          before = read(data, base + best - 1, sb);
                        if (best + 1 < d
                            && (costs.get(JAVA_SHORT, (base + best + 1) * 2) & 65535) != 512)
                          after = read(data, base + best + 1, sb);
                      }
                    }
                    int full =
                        ((costs.get(JAVA_SHORT, base * 2) & 65535) != 512 ? 1 : 0)
                            | ((costs.get(JAVA_SHORT, (base + d - 1) * 2) & 65535) != 512 ? 2 : 0);
                    int[] record = {best, value, second, before, after, full};
                    for (int j = 0; j < 6; j++) selected.set(JAVA_INT, p * 24L + j * 4L, record[j]);
                  }
                  select(selected, reverse, y0, actual, statistics);
                }
              } else {
                aggregate.transfer((long) y0 * w * d * sb, data, true);
                ioBytes += data.byteSize();
              }
            }
            MemorySegment swap = incoming;
            incoming = outgoing;
            outgoing = swap;
            work.update(ordinal + 1);
          }
          work.done();
        }
      }
    }
    if (cl != null) cl.releaseBuffers();
  }

  private static int read(MemorySegment data, long p, int bytes) {
    return bytes == 2 ? data.get(JAVA_SHORT, p * 2) & 65535 : data.get(JAVA_INT, p * 4);
  }

  private static void write(MemorySegment data, long p, int bytes, int value) {
    if (bytes == 2) data.set(JAVA_SHORT, p * 2, (short) value);
    else data.set(JAVA_INT, p * 4, value);
  }

  private static void banks(
      List<Object> args, MemorySegment[] cost, MemorySegment[] sum, long cells, int sb) {
    Collections.addAll(args, cost);
    Collections.addAll(args, sum);
    args.add(Math.toIntExact(cells));
    args.add(Math.toIntExact(cells));
  }

  private void select(
      MemorySegment selected, boolean reverse, int y, int actual, CensusSgm.Stats statistics) {
    float[] disparity = new float[w * actual];
    Arrays.fill(disparity, Float.NaN);
    Options selection = o.copy();
    selection.stereoSelection = new PointConfidence.Selection(disparity.length);
    GpuStereo.select(
        selected, 0, disparity.length, o.disparities, selection, disparity, statistics, null, null);
    write(reverse ? RIGHT : LEFT, y, actual, disparity, 0);
    write(reverse ? RM : LM, y, actual, selection.stereoSelection.margin, 0);
    write(reverse ? RS : LS, y, actual, selection.stereoSelection.sharpness, 0);
  }

  private static String kernels(Options o) {
    return GpuStereo.kernels(
            o.matchingCost.equals("zncc"), StereoCost.aggregateBytes(o), StereoCost.gpuSource(o))
        .replace("int sumY,int clearSum) {", "int sumY,int clearSum,int inputY,int inputRows) {")
        // Opt-in census modes: pass window-local indices, word stride and row count.
        .replace(
            "value=ccost(a,b,ta,tb,ga,gb,n,p,q,w,h,",
            "value=ccost(a,b,ta,tb,ga,gb,(size_t)w*inputRows,p-inputY*w,q-inputY*w,w,inputRows,")
        .replace("a[(size_t)j*n+p]", "a[(size_t)j*w*inputRows+p-inputY*w]")
        .replace("b[(size_t)j*n+q]", "b[(size_t)j*w*inputRows+q-inputY*w]")
        .replace("ta[p]", "ta[p-inputY*w]")
        .replace("tb[q]", "tb[q-inputY*w]")
        .replace("ga[p", "ga[p-inputY*w")
        .replace("gb[q", "gb[q-inputY*w")
        .replace(
            "__global int *outgoing,__local int *state)",
            "__global int *outgoing,int inputY,__local int *state)")
        .replace("valid[py*w+px]", "valid[(py-inputY)*w+px]")
        .replace("gray[globalp]", "gray[globalp-inputY*w]")
        .replace("gray[py*w+px]", "gray[(py-inputY)*w+px]");
  }

  private void filter() throws IOException {
    int r = o.census / 2;
    if (o.subpixelRefinement.equals("zncc"))
      for (boolean reverse : new boolean[] {false, true}) {
        int field = reverse ? RIGHT : LEFT;
        var original = store.volume((long) w * h, Float.floatToRawIntBits(Float.NaN));
        for (long p = 0; p < (long) w * h; p++)
          original.set(p, Float.floatToRawIntBits(get(p, field)));
        int halo = Math.max(1, o.correlationWindow / 2);
        for (int y = 0; y < h; y += rows) {
          int top = Math.max(0, y - halo),
              bottom = Math.min(h, y + rows + halo),
              actual = Math.min(rows, h - y),
              hh = bottom - top;
          float[] selected = new float[w * hh];
          for (int p = 0; p < selected.length; p++)
            selected[p] = Float.intBitsToFloat(original.get((long) top * w + p));
          var a = reverse ? right : left;
          var b = reverse ? left : right;
          PhotometricRefinement.refine(
              selected,
              a.bytes(a.gray, top, hh, 1),
              b.bytes(b.gray, top, hh, 1),
              w,
              hh,
              reverse,
              o);
          write(field, y, actual, selected, (y - top) * w);
        }
        original.close();
        store.files.remove(original);
      }
    for (int y = r; y < h - r; y++)
      for (int x = r; x < w - r; x++) {
        long p = (long) y * w + x;
        float d = get(p, LEFT);
        int xr = Math.round(x - d);
        if (!Float.isFinite(d) || xr < r || xr >= w - r) continue;
        float other = get((long) y * w + xr, RIGHT);
        if (Float.isFinite(other) && Math.abs(d - other) <= o.lrThreshold) put(p, CHECK, d);
        else stats.lrRejected++;
      }
    int halo =
        Math.max(o.census / 2, Math.max(Math.max(3, o.correlationWindow / 2), o.median / 2)) + 1;
    for (int y = 0; y < h; y += rows) {
      int top = Math.max(0, y - halo),
          bottom = Math.min(h, y + rows + halo),
          hh = bottom - top,
          actual = Math.min(rows, h - y);
      byte[] ga = left.bytes(left.gray, top, hh, 1), gb = right.bytes(right.gray, top, hh, 1);
      float[] checked = floats(CHECK, top, hh), dr = floats(RIGHT, top, hh);
      if (o.median > 1)
        checked =
            o.medianMode.equals("classic")
                ? CensusSgm.medianValid(checked, w, hh, o.median)
                : CensusSgm.medianEdgePreserving(
                    checked, w, hh, o.median, o.medianMaxChange, ga, o.medianEdgeThreshold);
      boolean color = o.censusMode.equals("color-distance");
      byte[] pa = color ? left.bytes(left.color, top, hh, 3) : ga,
          pb = color ? right.bytes(right.color, top, hh, 3) : gb;
      for (int yy = y - top; yy < y - top + actual; yy++)
        for (int x = 0; x < w; x++) {
          int p = yy * w + x;
          long gp = (long) (yy + top) * w + x;
          float d = checked[p];
          if (!Float.isFinite(d)) continue;
          int xr = Math.round(x - d);
          if (xr >= 0 && xr < w && Float.isFinite(dr[yy * w + xr]))
            put(gp, LR, Math.abs(d - dr[yy * w + xr]));
          if (!CensusSgm.reciprocal(dr, w, hh, x, yy, d, o.lrThreshold, r)) {
            stats.postMedianLrRejected++;
            continue;
          }
          double score =
              CensusSgm.zncc(pa, pb, w, hh, x, yy, d, o.correlationWindow / 2, color ? 3 : 1);
          put(gp, PHOTO, (float) score);
          if (score < o.correlation) {
            stats.photometricRejected++;
            continue;
          }
          put(gp, FINAL, d);
        }
    }
    if (o.disparityEdgeErosion > 0) erodeEdges();
    speckles();
    if(DepthOutlierFilter.enabled(o))depthOutliers();
    for (int y = 0; y < h; y += rows) {
      int top = Math.max(0, y - halo),
          bottom = Math.min(h, y + rows + halo),
          hh = bottom - top,
          actual = Math.min(rows, h - y);
      byte[] ga = left.bytes(left.gray, top, hh, 1), gb = right.bytes(right.gray, top, hh, 1);
      boolean color = o.censusMode.equals("color-distance");
      var local =
          new CensusSgm.Result(
              w,
              hh,
              floats(FINAL, top, hh),
              new CensusSgm.Stats(),
              floats(PHOTO, top, hh),
              floats(LR, top, hh));
      local.costMargin = floats(LM, top, hh);
      local.costSharpness = floats(LS, top, hh);
      PointConfidence.stereo(local, ga, gb);
      write(CONFIDENCE, y, actual, local.confidence, (y - top) * w);
      if (PointConfidence.bidirectional(o)) {
        var selection = new PointConfidence.Selection(w * hh);
        System.arraycopy(floats(RM, top, hh), 0, selection.margin, 0, w * hh);
        System.arraycopy(floats(RS, top, hh), 0, selection.sharpness, 0, w * hh);
        PointConfidence.reverse(
            local,
            ga,
            gb,
            color ? left.bytes(left.color, top, hh, 3) : ga,
            color ? right.bytes(right.color, top, hh, 3) : gb,
            color ? 3 : 1,
            floats(RIGHT, top, hh),
            selection,
            left.bytes(left.mask, top, hh, 1),
            right.bytes(right.mask, top, hh, 1),
            o);
        write(REVERSE, y, actual, local.rightConfidence, (y - top) * w);
      }
    }
    for (long p = 0; p < (long) w * h; p++) if (Float.isFinite(get(p, FINAL))) stats.valid++;
  }

  private void depthOutliers() throws IOException {
    // One bit per pixel on bounded paged storage; do not mutate FINAL until all
    // strips have made their decisions from the original, unfiltered raster.
    var rejected=store.volume(Math.ceilDiv((long)w*h,32),0);
    try {
      var diagnostics=new DepthOutlierFilter.Diagnostics();
      int radius=o.depthOutlierRadius;
      int strip=Math.max(1,Math.min(rows,(int)Math.max(1,o.memoryMb*1048576L/16/(5L*w)-2L*radius)));
      for(int y=0;y<h;y+=strip) {
        int top=Math.max(0,y-radius),bottom=Math.min(h,y+strip+radius),actual=Math.min(strip,h-y);
        var mask=DepthOutlierFilter.mask(o,floats(FINAL,top,bottom-top),
            left.bytes(left.gray,top,bottom-top,1),w,bottom-top,y-top,y-top+actual,
            floats(LR,top,bottom-top),diagnostics,null);
        for(int p=mask.nextSetBit(0);p>=0;p=mask.nextSetBit(p+1)) {
          long global=(long)top*w+p,word=global/32;
          rejected.set(word,rejected.get(word)|(1<<(int)(global%32)));
        }
      }
      for(long word=0;word<Math.ceilDiv((long)w*h,32);word++) {
        int bits=rejected.get(word);
        while(bits!=0) {
          int bit=Integer.numberOfTrailingZeros(bits);
          put(word*32+bit,FINAL,Float.NaN);stats.depthOutlierRejected++;bits&=bits-1;
        }
      }
      stats.depthOutlierReasons=diagnostics.counts;
      DepthOutlierFilter.report(o,stats.depthOutlierRejected,diagnostics);
    } finally {rejected.close();store.files.remove(rejected);}
  }

  /** Opt-in --disparity-edge-erosion on the disk-backed FINAL raster; same rule as CensusSgm. */
  private void erodeEdges() throws IOException {
    int radius = o.disparityEdgeErosion;
    BitSet reject = new BitSet();
    // Row blocks with a radius-row halo; decisions read only the unmodified FINAL raster.
    for (int y = 0; y < h; y += rows) {
      int top = Math.max(0, y - radius),
          bottom = Math.min(h, y + rows + radius),
          actual = Math.min(rows, h - y);
      boolean[] flags =
          CensusSgm.edgeFlags(
              floats(FINAL, top, bottom - top),
              w,
              bottom - top,
              y - top,
              y - top + actual,
              radius,
              o.disparityEdgeJump);
      for (int i = (y - top) * w; i < (y - top + actual) * w; i++)
        if (flags[i]) reject.set(top * w + i);
    }
    for (int i = reject.nextSetBit(0); i >= 0; i = reject.nextSetBit(i + 1)) {
      put(i, FINAL, Float.NaN);
      stats.edgeErosionRejected++;
    }
  }

  private void speckles() throws IOException {
    if (o.speckleSize <= 1) return;
    var seen = store.volume((long) w * h, 0);
    var queue = store.volume((long) w * h, 0);
    for (int start = 0; start < (long) w * h; start++) {
      if (seen.get(start) != 0 || !Float.isFinite(get(start, FINAL))) continue;
      long head = 0, tail = 0;
      queue.set(tail++, start);
      seen.set(start, 1);
      while (head < tail) {
        int p = queue.get(head++), x = p % w, y = p / w;
        float d = get(p, FINAL);
        for (int dy = -1; dy <= 1; dy++)
          for (int dx = -1; dx <= 1; dx++) {
            if (dx == 0 && dy == 0 || o.speckleConnectivity == 4 && dx != 0 && dy != 0) continue;
            int xx = x + dx, yy = y + dy;
            if (xx < 0 || xx >= w || yy < 0 || yy >= h) continue;
            int q = yy * w + xx;
            float v = get(q, FINAL);
            if (seen.get(q) != 0 || !Float.isFinite(v) || Math.abs(v - d) > o.speckleRange)
              continue;
            seen.set(q, 1);
            queue.set(tail++, q);
          }
      }
      if (tail < o.speckleSize)
        for (long i = 0; i < tail; i++) {
          put(queue.get(i), FINAL, Float.NaN);
          stats.speckleRejected++;
        }
    }
    seen.close();
    queue.close();
    store.files.remove(seen);
    store.files.remove(queue);
  }

  public void close() {
    if (cl != null) {
      RunTelemetry.gpuProfile("Windowed stereo", cl.profile());
      cl.close();
      cl = null;
    }
  }
}
