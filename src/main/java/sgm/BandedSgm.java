package sgm;

import java.lang.foreign.MemorySegment;
import java.util.*;

/**
 * SGM over per-pixel disparity bands with continuous whole-image paths. The CPU code is the integer
 * reference for {@link GpuBandedStereo}; with full bands both reproduce ordinary SGM exactly.
 *
 * <p>Winner summaries use the GpuStereo layout (best label, value, second, before, after, flags);
 * flag bit 4 marks a winner on a band edge that is not a full-range edge.
 */
final class BandedSgm {
  static final int INVALID_COST = 512, BAND_EDGE = 4;
  // Rows solved above and below each strip's core rows, so its paths start before the core.
  static final int STRIP_OVERLAP = 64;

  /**
   * Largest packed volume solved in one piece: the int32 index limit, the CPU reference budget and,
   * when the GPU may be used, its working budget. {@code -Dsgm.test.stripCells} lowers it (tests).
   */
  static final long INDEX_LIMIT = Integer.MAX_VALUE - 8;

  static long cpuBudget(Options o) {
    return Math.min(o.memoryMb * 1048576L, (long) (Runtime.getRuntime().maxMemory() * .65));
  }

  /** Whether the GPU can solve these bands whole (never throws; ineligible means false). */
  static boolean gpuFits(Options o, DisparityBands bands) {
    try {
      return GpuBandedStereo.use(o, bands.width, bands.height, bands);
    } catch (RuntimeException e) {
      return false;
    }
  }

  /**
   * Strip size for a volume that cannot be solved whole: the larger of the CPU reference budget and
   * (when a GPU is usable) its working budget, never above the int32 index limit.
   * {@code -Dsgm.test.stripCells} sets it (tests).
   */
  static long stripLimit(Options o, int w, int h) {
    Long forced = Long.getLong("sgm.test.stripCells");
    if (forced != null) return forced;
    long limit = Math.max(1, (cpuBudget(o) - 24L * w * h) / 6);
    if (!o.compute.equals("cpu"))
      try {
        GpuSgm.Device device = GpuSgm.selectedDevice(o);
        long perCell = 2L + StereoCost.aggregateBytes(o);
        long gpu = (GpuSgm.budget(device, o) - GpuBandedStereo.deviceBytes(o, w, h, 0)) / perCell;
        limit = Math.max(limit, Math.min(gpu, 8 * GpuBandedStereo.bankCells(o, device)));
      } catch (RuntimeException ignored) {
        // No usable GPU: the CPU budget applies.
      }
    return Math.min(INDEX_LIMIT - (1L << 20), limit);
  }

  /** Outcome of one banded solve: the selected disparities and the bands actually searched. */
  record Outcome(float[] disparity, DisparityBands bands, long retried, long edgeRejected, String backend) {}

  private BandedSgm() {}

  /**
   * One rectified pair's matching inputs in forward (left, right) order. A GPU session, if used, is
   * created on the first eligible solve and keeps inputs resident for both directions and retries.
   */
  static final class Pair implements AutoCloseable {
    final long[][] a, b;
    final byte[] grayA, grayB;
    final boolean[] ta, tb;
    final int w, h;
    private GpuBandedStereo.Session gpu;
    private boolean gpuFailed;

    Pair(long[][] a, long[][] b, byte[] grayA, byte[] grayB, boolean[] ta, boolean[] tb, int w, int h) {
      this.a = a;
      this.b = b;
      this.grayA = grayA;
      this.grayB = grayB;
      this.ta = ta;
      this.tb = tb;
      this.w = w;
      this.h = h;
    }

    int[] summary(Options o, boolean reverse, DisparityBands bands, String[] backend) {
      long cells = bands.cells();
      // Strips only where a whole solve cannot run at all, so every pair that worked before is
      // solved exactly as before.
      Long forced = Long.getLong("sgm.test.stripCells");
      boolean whole = forced == null ? cells <= INDEX_LIMIT && (cpuBytes(bands, o) <= cpuBudget(o) || gpuFits(o, bands)) : cells <= forced;
      if (whole) return whole(o, reverse, bands, backend);
      return stripped(o, reverse, bands, backend, cells, stripLimit(o, w, h));
    }

    /**
     * Packed volumes are int32-indexed (DisparityBands.offsets, GPU banks) and must fit the CPU or
     * GPU working budget. Wide bands at depth edges (cities, wide baselines, full resolution) can
     * exceed that; such pairs are solved in overlapping horizontal strips instead of failing.
     */
    private int[] stripped(
        Options o, boolean reverse, DisparityBands bands, String[] backend, long cells, long limit) {
      long[] rows = bands.rowCells();
      int[] out = new int[w * h * 6];
      List<int[]> strips = new ArrayList<>();
      int overlap = STRIP_OVERLAP;
      for (int core = 0; core < h; ) {
        int end = core + 1;
        while (end < h && span(rows, core, end + 1, overlap, h) <= limit) end++;
        if (span(rows, core, end, overlap, h) > limit) {
          // Even one core row with its overlap is too large: shrink the overlap, then give up.
          while (overlap > 0 && span(rows, core, end, overlap, h) > limit) overlap /= 2;
          if (span(rows, core, end, overlap, h) > limit)
            throw new IllegalArgumentException(
                "One image row of disparity bands exceeds the banded SGM budget; lower dense"
                    + " resolution or --disparities");
        }
        strips.add(new int[] {core, end, overlap});
        core = end;
        overlap = STRIP_OVERLAP;
      }
      System.out.printf(
          Locale.ROOT,
          "Banded SGM: %,d packed cells exceed the %,d-cell limit; solving %d overlapping strips"
              + " (%d-row overlap)%n",
          cells,
          limit,
          strips.size(),
          STRIP_OVERLAP);
      Set<String> used = new TreeSet<>();
      for (int[] s : strips) {
        int y0 = Math.max(0, s[0] - s[2]), y1 = Math.min(h, s[1] + s[2]);
        try (Pair part = rows(y0, y1)) {
          String[] b = new String[1];
          int[] summary = part.whole(o, reverse, bands.rows(y0, y1), b);
          used.add(b[0]);
          System.arraycopy(summary, (s[0] - y0) * w * 6, out, s[0] * w * 6, (s[1] - s[0]) * w * 6);
        }
      }
      backend[0] = String.join("+", used) + " strips=" + strips.size();
      return out;
    }

    private static long span(long[] rows, int core, int end, int overlap, int h) {
      long sum = 0;
      for (int y = Math.max(0, core - overlap); y < Math.min(h, end + overlap); y++) sum += rows[y];
      return sum;
    }

    /** Rows [y0, y1) of this pair's inputs as an independent pair. */
    private Pair rows(int y0, int y1) {
      int from = y0 * w, length = (y1 - y0) * w;
      return new Pair(
          slice(a, from, length),
          slice(b, from, length),
          Arrays.copyOfRange(grayA, from, from + length),
          Arrays.copyOfRange(grayB, from, from + length),
          Arrays.copyOfRange(ta, from, from + length),
          Arrays.copyOfRange(tb, from, from + length),
          w,
          y1 - y0);
    }

    private static long[][] slice(long[][] words, int from, int length) {
      long[][] out = new long[words.length][];
      for (int k = 0; k < words.length; k++) out[k] = Arrays.copyOfRange(words[k], from, from + length);
      return out;
    }

    private int[] whole(Options o, boolean reverse, DisparityBands bands, String[] backend) {
      if (!gpuFailed && GpuBandedStereo.use(o, w, h, bands)) {
        try {
          long total = Math.max(1, bands.total());
          // With the shared pair-worker pool, a session holds a fixed reservation: one that would
          // have to grow is closed first and re-created with a new reservation (never waiting
          // while holding memory). Volume capacity does not affect results.
          if (gpu != null && GpuMemoryPool.enabled() && total > gpu.capacity()) {
            gpu.close();
            gpu = null;
          }
          if (gpu == null)
            gpu =
                new GpuBandedStereo.Session(
                    o, a, b, grayA, grayB, ta, tb, w, h,
                    GpuMemoryPool.enabled() ? GpuBandedStereo.plannedBytes(o, w, h, total) : 0);
          int[] result = gpu.summary(o, reverse, bands);
          backend[0] = "gpu";
          if (o.gpuSmallSolves && total < 200_000 && !GpuBandedStereo.smallVerified)
            return verifySmall(o, reverse, bands, result, backend);
          return result;
        } catch (RuntimeException e) {
          gpuFailed = true;
          if (gpu != null) {
            try {
              gpu.close();
            } catch (RuntimeException ignored) {
            }
            gpu = null;
          }
          GpuSgm.closeSession();
          if (o.compute.equals("gpu"))
            throw new IllegalArgumentException("GPU execution required: " + e.getMessage(), e);
          System.out.println("GPU banded SGM failed (" + e.getMessage() + "); using the CPU reference for this pair");
        }
      }
      backend[0] = "cpu";
      return reverse
          ? cpuSummary(o, b, a, grayB, grayA, tb, ta, w, h, true, bands)
          : cpuSummary(o, a, b, grayA, grayB, ta, tb, w, h, false, bands);
    }

    /**
     * First small (coarse TSGM) GPU solve in a process: compared with the CPU reference; a
     * difference disables small GPU solves and the CPU summary is used.
     */
    private int[] verifySmall(
        Options o, boolean reverse, DisparityBands bands, int[] result, String[] backend) {
      int[] cpu =
          reverse
              ? cpuSummary(o, b, a, grayB, grayA, tb, ta, w, h, true, bands)
              : cpuSummary(o, a, b, grayA, grayB, ta, tb, w, h, false, bands);
      synchronized (GpuBandedStereo.class) {
        if (!Arrays.equals(cpu, result)) {
          if (!GpuBandedStereo.smallDisabled)
            System.out.println(
                "WARNING: small GPU banded solve differs from the CPU reference; coarse TSGM levels"
                    + " below 200,000 cells use the CPU (same results as before)");
          GpuBandedStereo.smallDisabled = true;
          backend[0] = "cpu";
          return cpu;
        }
        if (!GpuBandedStereo.smallVerified)
          System.out.println(
              "GPU banded SGM: small coarse-level solve (" + w + "x" + h + ", " + bands.total()
                  + " cells) identical to the CPU reference");
        GpuBandedStereo.smallVerified = true;
      }
      return result;
    }

    @Override
    public void close() {
      if (gpu != null) gpu.close();
      gpu = null;
    }
  }

  /** Ordinary selection of a banded solve, with one band-edge retry when {@code retry} is set. */
  static Outcome match(
      Options o,
      Pair pair,
      boolean reverse,
      DisparityBands bands,
      boolean retry,
      CensusSgm.Stats stats,
      byte[] reasons,
      byte[] boundary) {
    String[] backend = new String[1];
    int w = pair.w, h = pair.h;
    int[] summary = pair.summary(o, reverse, bands, backend);
    long retried = 0;
    if (retry) {
      long[] touched = new long[1];
      DisparityBands widened = widen(o, bands, summary, w, h, touched);
      if (widened != null) {
        retried = touched[0];
        bands = widened;
        summary = pair.summary(o, reverse, bands, backend);
      }
    }
    long edgeRejected = 0;
    int n = w * h;
    for (int p = 0; p < n; p++) {
      int at = p * 6;
      if ((summary[at + 5] & BAND_EDGE) == 0) continue;
      summary[at + 5] &= ~BAND_EDGE;
      summary[at] = -1;
      edgeRejected++;
      stats.boundaryRejected++;
      if (reasons != null) reasons[p] = 2;
    }
    float[] result = new float[n];
    Arrays.fill(result, Float.NaN);
    GpuStereo.select(
        summary, 0, 0, n, o.disparities, o, result, stats, reasons, boundary, null, 0, Integer.MAX_VALUE);
    return new Outcome(result, bands, retried, edgeRejected, backend[0]);
  }

  /**
   * Widens band-edge winners and a one-pixel halo on the offending side by max(2*margin, count)
   * labels. Returns null when no pixel needs it; {@code touched[0]} receives the widened pixels.
   */
  static DisparityBands widen(
      Options o, DisparityBands bands, int[] summary, int w, int h, long[] touched) {
    int n = w * h, D = bands.labels, grow = Math.max(2, 2 * o.tsgmMargin);
    DisparityBands out = null;
    long widened = 0;
    for (int p = 0; p < n; p++) {
      int at = p * 6;
      if ((summary[at + 5] & BAND_EDGE) == 0) continue;
      if (out == null) {
        out = new DisparityBands(w, h, D);
        System.arraycopy(bands.low, 0, out.low, 0, n);
        System.arraycopy(bands.count, 0, out.count, 0, n);
      }
      boolean lower = summary[at] == bands.low[p];
      int x = p % w, y = p / w;
      for (int yy = Math.max(0, y - 1); yy <= Math.min(h - 1, y + 1); yy++)
        for (int xx = Math.max(0, x - 1); xx <= Math.min(w - 1, x + 1); xx++) {
          int q = yy * w + xx;
          if (out.count[q] == 0) continue;
          int amount = Math.max(grow, bands.count[q]);
          int before = out.count[q];
          if (lower) out.include(q, out.low[q] - amount);
          else out.include(q, out.low[q] + out.count[q] - 1 + amount);
          if (out.count[q] != before) widened++;
        }
    }
    touched[0] = widened;
    if (out != null) out.changed();
    return out;
  }

  /** Whether label k at interior pixel (x, y) has an in-image, textured target. */
  static boolean matchable(
      Options o, boolean[] tb, int w, int h, int x, int y, int k, boolean reverse) {
    int r = o.census / 2;
    long disparity = (long) o.minDisparity + k, target = reverse ? x + disparity : x - disparity;
    return target >= r && target < w - r && tb[y * w + (int) target];
  }

  static long cpuBytes(DisparityBands bands, Options o) {
    return bands.total() * (2L + 4L) + 24L * bands.count.length;
  }

  /** Integer reference: packed costs, 4/8 continuous paths, winners. */
  static int[] cpuSummary(
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
      DisparityBands bands) {
    int n = w * h, r = o.census / 2, md = o.minDisparity;
    int[] offset = bands.offsets(), low = bands.low, count = bands.count;
    long total = bands.total();
    long budget = Math.min(o.memoryMb * 1048576L, (long) (Runtime.getRuntime().maxMemory() * .65));
    if (cpuBytes(bands, o) > budget)
      throw new IllegalArgumentException(
          "Banded CPU SGM needs " + cpuBytes(bands, o) / 1048576 + " MiB; raise --memory-mb/heap");
    short[] cost = new short[(int) total];
    int[] sum = new int[(int) total];
    StereoRows.rows(0, h, y -> {
      for (int x = 0; x < w; x++) {
        int p = y * w + x, at = offset[p];
        for (int j = 0; j < count[p]; j++) {
          int value = INVALID_COST;
          long target = (long) x + (reverse ? (long) md + low[p] + j : -((long) md + low[p] + j));
          if (x >= r && x < w - r && y >= r && y < h - r && ta[p] && target >= r && target < w - r) {
            int q = y * w + (int) target;
            if (tb[q]) value = StereoCost.cost(o, a, b, grayA, grayB, ta, tb, p, q, w, h);
          }
          cost[at + j] = (short) value;
        }
      }
    });
    int p1 = StereoCost.penalty(o.p1, o), p2 = StereoCost.p2(o);
    int[][] dirs = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, 1}, {1, -1}, {-1, -1}};
    int maxRow = 0;
    for (int y = 0; y < h; y++) {
      long cells = (y + 1 < h ? offset[(y + 1) * w] : total) - offset[y * w];
      maxRow = (int) Math.max(maxRow, cells);
    }
    for (int path = 0; path < o.directions; path++) {
      int dx = dirs[path][0], dy = dirs[path][1];
      int[] previous = new int[maxRow], row = new int[maxRow];
      int[] previousMin = new int[w], rowMin = new int[w];
      int yStart = dy < 0 ? h - 1 : 0, yStep = dy < 0 ? -1 : 1;
      int xStart = dx < 0 ? w - 1 : 0, xStep = dx < 0 ? -1 : 1;
      for (int yi = 0, y = yStart; yi < h; yi++, y += yStep) {
        int rowBase = offset[y * w], prevRowBase = dy == 0 ? rowBase : (y - dy >= 0 && y - dy < h ? offset[(y - dy) * w] : 0);
        for (int xi = 0, x = xStart; xi < w; xi++, x += xStep) {
          int p = y * w + x, px = x - dx, py = y - dy, lo = low[p], cnt = count[p], at = offset[p];
          int q = py * w + px;
          boolean has = px >= 0 && px < w && py >= 0 && py < h && ta[q] && count[q] > 0;
          int[] prev = dy == 0 ? row : previous;
          int plo = has ? low[q] : 0, pn = has ? count[q] : 0;
          int prevBase = has ? offset[q] - prevRowBase : 0;
          int minPrev = has ? (dy == 0 ? rowMin : previousMin)[px] : 0;
          int localP2 = has ? StereoCost.pathP2(p1, p2, o.sgmEdgeScale, grayA, p, q) : p2;
          int currentMin = Integer.MAX_VALUE, dest = at - rowBase;
          for (int j = 0; j < cnt; j++) {
            int value = cost[at + j];
            if (has) {
              int k = lo + j - plo, t = minPrev + localP2;
              if (k >= 0 && k < pn) t = Math.min(t, prev[prevBase + k]);
              if (k - 1 >= 0 && k - 1 < pn) t = Math.min(t, prev[prevBase + k - 1] + p1);
              if (k + 1 >= 0 && k + 1 < pn) t = Math.min(t, prev[prevBase + k + 1] + p1);
              value += t - minPrev;
            }
            row[dest + j] = value;
            currentMin = Math.min(currentMin, value);
            sum[at + j] += value;
          }
          rowMin[x] = currentMin;
        }
        int[] swap = previous;
        previous = row;
        row = swap;
        swap = previousMin;
        previousMin = rowMin;
        rowMin = swap;
      }
    }
    int[] summary = new int[n * 6];
    int D = o.disparities;
    for (int p = 0; p < n; p++) {
      int x = p % w, y = p / w, lo = low[p], cnt = count[p], at = offset[p];
      int best = -1, value = Integer.MAX_VALUE, second = Integer.MAX_VALUE, before = -1, after = -1;
      if (x >= r && x < w - r && y >= r && y < h - r && cnt > 0) {
        for (int j = 0; j < cnt; j++)
          if (cost[at + j] != INVALID_COST && sum[at + j] < value) {
            best = j;
            value = sum[at + j];
          }
        if (best >= 0) {
          for (int j = 0; j < cnt; j++)
            if (Math.abs(j - best) > 1 && cost[at + j] != INVALID_COST)
              second = Math.min(second, sum[at + j]);
          if (best > 0 && cost[at + best - 1] != INVALID_COST) before = sum[at + best - 1];
          if (best + 1 < cnt && cost[at + best + 1] != INVALID_COST) after = sum[at + best + 1];
        }
      }
      // A band-edge winner needs a retry only if the label just outside the band could match.
      boolean edge =
          best >= 0
              && ((best == 0 && lo > 0 && matchable(o, tb, w, h, x, y, lo - 1, reverse))
                  || (best == cnt - 1
                      && lo + cnt < D
                      && matchable(o, tb, w, h, x, y, lo + cnt, reverse)));
      int flags =
          (cnt > 0 && lo == 0 && cost[at] != INVALID_COST ? 1 : 0)
              | (cnt > 0 && lo + cnt == D && cost[at + cnt - 1] != INVALID_COST ? 2 : 0)
              | (edge ? BAND_EDGE : 0);
      int s = p * 6;
      summary[s] = best < 0 ? -1 : lo + best;
      summary[s + 1] = value;
      summary[s + 2] = second;
      summary[s + 3] = before;
      summary[s + 4] = after;
      summary[s + 5] = flags;
    }
    return summary;
  }
}
