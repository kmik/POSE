package sgm;

import java.io.*;
import java.util.*;

/**
 * Selectable costs, edge-aware Hirschmuller recurrence, 4/8 paths and LR validation. CPU primitive
 * arrays/paging or optional OpenCL kernels; no OpenCV stereo matcher or C++ process.
 */
public final class CensusSgm {
  private static final int INVALID_COST = 512;
  private final Options options;
  private static final ThreadLocal<Prepared> PREPARED = new ThreadLocal<>();
  static final class Prepared {
    final byte[] left,right,ca,cb,ma,mb;
    final int w,h,window;
    final String mode;
    final long[][] a,b;
    final boolean[] ta,tb;
    Prepared(Options o,byte[] l,byte[] r,byte[] ca,byte[] cb,int w,int h,byte[] ma,byte[] mb,
        long[][] a,long[][] b,boolean[] ta,boolean[] tb) {
      left=l;right=r;this.ca=ca;this.cb=cb;this.w=w;this.h=h;this.ma=ma;this.mb=mb;
      window=o.census;mode=o.censusMode;this.a=a;this.b=b;this.ta=ta;this.tb=tb;
    }
    boolean matches(Options o,byte[] l,byte[] r,byte[] ca,byte[] cb,int w,int h,byte[] ma,byte[] mb) {
      return left==l&&right==r&&this.ca==ca&&this.cb==cb&&this.w==w&&this.h==h
          &&this.ma==ma&&this.mb==mb&&window==o.census&&mode.equals(o.censusMode);
    }
  }
  static Prepared prepare(Options o,byte[] left,byte[] right,byte[] ca,byte[] cb,int w,int h,byte[] ma,byte[] mb) {
    int n=Math.multiplyExact(w,h);
    try(var rows=StereoRows.open(o.cores)) {
    long[][] a, b;
    boolean[] texturedLeft, texturedRight;
    if (o.censusMode.equals("color-distance")) {
      if (ca == null
          || cb == null
          || ca.length != (long) n * 3
          || cb.length != ca.length)
        throw new IllegalArgumentException("Color-distance census requires interleaved BGR inputs");
      DescriptorSet first = colorCensus(ca, w, h, o.census),
          second = colorCensus(cb, w, h, o.census);
      a = first.bits;
      b = second.bits;
      texturedLeft = first.textured;
      texturedRight = second.textured;
    } else {
      a = census(left, w, h, o.census);
      b = census(right, w, h, o.census);
      texturedLeft = textureMask(left, w, h, o.census);
      texturedRight = textureMask(right, w, h, o.census);
    }
    if (ma != null) {
      if (mb == null || ma.length != n || mb.length != n)
        throw new IllegalArgumentException("Invalid stereo mask dimensions");
      for (int i = 0; i < n; i++) {
        texturedLeft[i] &= ma[i] != 0;
        texturedRight[i] &= mb[i] != 0;
      }
    }

      return new Prepared(o,left,right,ca,cb,w,h,ma,mb,a,b,texturedLeft,texturedRight);
    }
  }
  static AutoCloseablePrepared use(Prepared prepared) { return new AutoCloseablePrepared(prepared); }
  static final class AutoCloseablePrepared implements AutoCloseable {
    final Prepared previous=PREPARED.get();
    AutoCloseablePrepared(Prepared p) { PREPARED.set(p); }
    public void close() { if(previous==null)PREPARED.remove();else PREPARED.set(previous); }
  }
  private PointConfidence.Selection selection;
  private PointConfidence.Selection reverseSelection;
  // Lightweight boundary capture for tile retries, without enabling diagnostic observers
  // (which intentionally select the CPU reference post-filters).
  boolean captureSearchBoundary;
  byte[] lastSearchBoundary;

  public CensusSgm(Options options) {
    this.options = options;
  }

  public Result compute(byte[] left, byte[] right, int width, int height) {
    return compute(left, right, null, null, width, height);
  }

  /** Optional interleaved BGR inputs enable the RGB-distance descriptor used by Image.h. */
  public Result compute(
      byte[] left, byte[] right, byte[] leftBgr, byte[] rightBgr, int width, int height) {
    return compute(left, right, leftBgr, rightBgr, width, height, null, null);
  }

  public Result compute(
      byte[] left,
      byte[] right,
      byte[] leftBgr,
      byte[] rightBgr,
      int width,
      int height,
      byte[] validLeft,
      byte[] validRight) {
    try {
      return computeRecorded(
          left, right, leftBgr, rightBgr, width, height, validLeft, validRight, null);
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  Result computeRecorded(
      byte[] left,
      byte[] right,
      byte[] leftBgr,
      byte[] rightBgr,
      int width,
      int height,
      byte[] validLeft,
      byte[] validRight,
      Observer observer)
      throws IOException {
    try (var rows = StereoRows.open(options.stereoPipeline ? Math.max(1,options.cores/2) : options.cores)) {
      return computeInternal(left, right, leftBgr, rightBgr, width, height, validLeft, validRight, observer);
    }
  }

  Result computeInternal(
      byte[] left,
      byte[] right,
      byte[] leftBgr,
      byte[] rightBgr,
      int width,
      int height,
      byte[] validLeft,
      byte[] validRight,
      Observer observer)
      throws IOException {
    if (width < options.census + 2
        || height < options.census + 2
        || left.length != (long) width * height
        || right.length != left.length)
      throw new IllegalArgumentException("Invalid stereo image dimensions");
    int n = left.length, d = options.disparities;
    long cells = (long) n * d;
    long estimate = memoryEstimate(width, height, d, options);
    long budget =
        Math.min(options.memoryMb * 1024 * 1024, (long) (Runtime.getRuntime().maxMemory() * .65));
    boolean tsgm = options.pairwiseSearch.equals("tsgm");
    boolean gpu = !tsgm && GpuSgm.use(width, height, options);
    if (cells >= 1_000_000)
      System.out.printf(
          Locale.ROOT,
          "Stereo backend: %s; %dx%d, %d disparities%n",
          tsgm
              ? "tsgm banded"
              : gpu ? "OpenCL GPU" : options.tiled ? "CPU (disk-paged SGM)" : "CPU (RAM SGM)",
          width,
          height,
          d);
    if (tsgm) {
      // Banded volumes are planned and budgeted by BandedSgm/GpuBandedStereo.
    } else if (gpu && (long) n * (116 + 32 * words(options.census)) > budget)
      throw new IllegalArgumentException(
          "GPU host descriptors/output buffers exceed --memory-mb/heap budget");
    if (!tsgm && !gpu && options.tiled && tiledMemoryEstimate(width, height, d, options) > budget)
      throw new IllegalArgumentException(
          "Tiled SGM rows/descriptors/output rasters exceed the memory budget; lower dense"
              + " resolution or raise --memory-mb/heap");
    if (!tsgm && !gpu && !options.tiled && (cells > Integer.MAX_VALUE - 8 || estimate > budget))
      throw new IllegalArgumentException(
          String.format(
              Locale.ROOT,
              "SGM needs approximately %.0f MiB (budget %.0f MiB). Lower"
                  + " --max-dimension/--disparities or raise --memory-mb and heap.",
              estimate / 1048576.0,
              budget / 1048576.0));
    if (!tsgm && !gpu && options.tiled) {
      java.nio.file.Files.createDirectories(options.output);
      long free = java.nio.file.Files.getFileStore(options.output).getUsableSpace();
      if (free > 0 && cells * (2 + StereoCost.aggregateBytes(options)) + 16777216L > free)
        throw new IllegalArgumentException(
            "Tiled SGM needs approximately "
                + (cells * (2 + StereoCost.aggregateBytes(options)) / 1048576)
                + " MiB temporary disk space; only "
                + (free / 1048576)
                + " MiB available");
    }
    Stats stats = new Stats();
    byte[] reasons = observer == null && !captureSearchBoundary ? null : new byte[n];
    byte[] searchBoundary = observer == null && !captureSearchBoundary ? null : new byte[n];
    if (reasons != null) Arrays.fill(reasons, (byte) 1);
    String size = " " + width + "x" + height + " D=" + d;
    long clock = System.nanoTime();
    Prepared prepared = PREPARED.get();
    if (prepared == null || !prepared.matches(options,left,right,leftBgr,rightBgr,width,height,validLeft,validRight)) {
      prepared = prepare(options,left,right,leftBgr,rightBgr,width,height,validLeft,validRight);
      RunTelemetry.record("Stereo census/texture (not reused)" + size, clock);
    }
    clock = System.nanoTime();
    long[][] a=prepared.a,b=prepared.b;
    boolean[] texturedLeft=prepared.ta,texturedRight=prepared.tb;
    selection = new PointConfidence.Selection(n);
    reverseSelection =
        PointConfidence.bidirectional(options) ? new PointConfidence.Selection(n) : null;
    TsgmSearch.Plan plan = null;
    if (options.pairwiseSearch.equals("tsgm")) {
      plan =
          TsgmSearch.plan(
              options, left, right, validLeft, validRight, texturedLeft, texturedRight, width,
              height);
      RunTelemetry.record("Stereo tsgm pyramid/bands" + size, clock);
      clock = System.nanoTime();
    }
    float[] dl;
    BandedSgm.Pair pair =
        plan == null
            ? null
            : new BandedSgm.Pair(a, b, left, right, texturedLeft, texturedRight, width, height);
    float[] dr;
    try {
    if (plan != null) {
      BandedSgm.Outcome outcome = banded(pair, false, plan.left(), stats, reasons, searchBoundary);
      TsgmSearch.write(options, "left", plan, outcome, (System.nanoTime() - clock) / 1e9);
      dl = outcome.disparity();
    } else
      dl =
        match(
            a,
            b,
            left,
            right,
            texturedLeft,
            texturedRight,
            width,
            height,
            false,
            stats,
            reasons,
            searchBoundary);
    RunTelemetry.record(
        "Stereo SGM left" + (plan != null ? " (banded)" : gpu ? " (GPU)" : " (CPU)") + size, clock);
    clock = System.nanoTime();
    if (plan != null) {
      DisparityBands rightBands = plan.right();
      if (options.tsgmLevels > 0)
        TsgmSearch.includeWinners(rightBands, dl, options.minDisparity, texturedRight);
      BandedSgm.Outcome outcome = banded(pair, true, rightBands, new Stats(), null, null);
      TsgmSearch.write(options, "right", null, outcome, (System.nanoTime() - clock) / 1e9);
      dr = outcome.disparity();
    } else
      dr =
        match(
            b,
            a,
            right,
            left,
            texturedRight,
            texturedLeft,
            width,
            height,
            true,
            new Stats(),
            null,
            null);
    } finally {
      if (pair != null) pair.close();
    }
    RunTelemetry.record(
        "Stereo SGM right" + (plan != null ? " (banded)" : gpu ? " (GPU)" : " (CPU)") + size, clock);
    clock = System.nanoTime();
    if (captureSearchBoundary)
      for (int i = 0; i < n; i++)
        if (reasons[i] == 2) searchBoundary[i] |= 8; // unresolved packed-band edge
    if (observer != null) observer.searchBoundary(searchBoundary, width, height);
    if (observer != null) {
      observer.raster("00_left_selected", dl, width, height);
      observer.raster("01_right_selected", dr, width, height);
    }
    if (options.subpixelRefinement.equals("zncc")) {
      long refined = PhotometricRefinement.refine(dl, left, right, width, height, false, options);
      PhotometricRefinement.refine(dr, right, left, width, height, true, options);
      System.out.printf(
          Locale.ROOT,
          "Photometric subpixel refinement: %,d left disparities improved; original masks, LR and"
              + " depth-edge checks retained%n",
          refined);
      if (observer != null) {
        observer.raster("00b_left_photometric_subpixel", dl, width, height);
        observer.raster("01b_right_photometric_subpixel", dr, width, height);
      }
      RunTelemetry.record("Stereo ZNCC subpixel refinement" + size, clock);
    }
    clock = System.nanoTime();
    byte[] photoLeft = options.censusMode.equals("color-distance") ? leftBgr : left;
    byte[] photoRight = options.censusMode.equals("color-distance") ? rightBgr : right;
    int channels = options.censusMode.equals("color-distance") ? 3 : 1;
    int radius = options.census / 2;
    // Left/right check, median and reciprocal/ZNCC check: on the GPU when available (bit-identical
    // to the CPU code below), otherwise on the CPU.
    Filtered filtered =
        observer == null ? GpuStereoFilters.filter(options, dl, dr, left, photoLeft, photoRight, channels, width, height, radius) : null;
    if (filtered == null)
      filtered = filter(options, dl, dr, left, photoLeft, photoRight, channels, width, height, radius, reasons, observer, size);
    else RunTelemetry.record("Stereo left/right, median, ZNCC check (GPU)" + size, clock);
    float[] checked = filtered.checked(), correlation = filtered.correlation(), lrError = filtered.lrError();
    stats.lrRejected += filtered.lrRejected();
    stats.postMedianLrRejected += filtered.postRejected();
    stats.photometricRejected += filtered.photoRejected();

    if (observer != null) {
      observer.raster("04_photometric", checked, width, height);
      observer.raster("zncc", correlation, width, height);
      observer.raster("left_right_error", lrError, width, height);
    }
    if (options.disparityEdgeErosion > 0) {
      clock = System.nanoTime();
      stats.edgeErosionRejected =
          erodeDisparityEdges(
              checked,
              width,
              height,
              options.disparityEdgeErosion,
              options.disparityEdgeJump,
              reasons);
      RunTelemetry.record("Stereo disparity edge erosion" + size, clock);
      System.out.printf(
          Locale.ROOT,
          "Disparity edge erosion: %,d pixels within %d px of jumps > %.2f px rejected%n",
          stats.edgeErosionRejected,
          options.disparityEdgeErosion,
          options.disparityEdgeJump);
      if (observer != null) observer.raster("04b_edge_eroded", checked, width, height);
    }
    float[] beforeSpeckle = observer == null ? null : checked.clone();
    clock = System.nanoTime();
    stats.speckleRejected =
        removeSpeckles(
            checked,
            width,
            height,
            options.speckleSize,
            options.speckleRange,
            options.speckleConnectivity);
    RunTelemetry.record("Stereo speckle removal" + size, clock);
    for (float value : checked) if (Float.isFinite(value)) stats.valid++;
    if (observer != null) {
      for (int i = 0; i < n; i++)
        if (Float.isFinite(beforeSpeckle[i]) && !Float.isFinite(checked[i])) reasons[i] = 7;
      observer.raster("05_final_stereo", checked, width, height);
      observer.reasons(reasons, width, height);
    }
    Result result = new Result(width, height, checked, stats, correlation, lrError);
    result.costMargin = selection.margin;
    result.costSharpness = selection.sharpness;
    clock = System.nanoTime();
    if (reverseSelection != null)
      PointConfidence.reverse(
          result,
          left,
          right,
          photoLeft,
          photoRight,
          channels,
          dr,
          reverseSelection,
          validLeft,
          validRight,
          options);
    if (reverseSelection != null) RunTelemetry.record("Stereo right-image confidence" + size, clock);
    selection = null;
    reverseSelection = null;
    lastSearchBoundary = captureSearchBoundary ? searchBoundary : null;
    return result;
  }

  /** Disparities after the left/right check, median and reciprocal/ZNCC check, with counts. */
  record Filtered(
      float[] checked, float[] correlation, float[] lrError, long lrRejected, long postRejected, long photoRejected) {}

  /**
   * CPU filter chain after SGM: left/right check, median (classic or edge-preserving), then the
   * reciprocal check and ZNCC at the median-filtered disparity. GpuStereoFilters reproduces it
   * bit for bit.
   */
  static Filtered filter(
      Options options, float[] dl, float[] dr, byte[] left, byte[] photoLeft, byte[] photoRight, int channels,
      int width, int height, int radius, byte[] reasons, Observer observer, String size)
      throws IOException {
    int n = width * height;
    long clock = System.nanoTime();
    float[] checked = new float[n];
    Arrays.fill(checked, Float.NaN);
    final float[] lrChecked=checked;
    long[] lrRejected=new long[height];
    StereoRows.rows(radius,height-radius,y -> {
      for (int x = radius; x < width - radius; x++) {
        int p = y * width + x;
        if (!Float.isFinite(dl[p])) continue;
        int xr = Math.round(x - dl[p]);
        if (xr < radius || xr >= width - radius) {
          if (reasons != null) reasons[p] = 4;
          continue;
        }
        float other = dr[y * width + xr];
        if (Float.isFinite(other) && Math.abs(dl[p] - other) <= options.lrThreshold)
          lrChecked[p] = dl[p];
        else {
          lrRejected[y]++;
          if (reasons != null) reasons[p] = 4;
        }
      }
    });
    long lrTotal = 0;
    for(long rejected:lrRejected)lrTotal+=rejected;
    RunTelemetry.record("Stereo left/right check" + size, clock);
    clock = System.nanoTime();

    if (observer != null) observer.raster("02_left_right_validated", checked, width, height);
    if (options.median > 1)
      checked =
          options.medianMode.equals("classic")
              ? medianValid(checked, width, height, options.median)
              : medianEdgePreserving(
                  checked,
                  width,
                  height,
                  options.median,
                  options.medianMaxChange,
                  left,
                  options.medianEdgeThreshold);
    if (observer != null) observer.raster("03_median", checked, width, height);
    RunTelemetry.record("Stereo median" + size, clock);
    clock = System.nanoTime();
    float[] correlation = new float[n], lrError = new float[n];
    Arrays.fill(correlation, Float.NaN);
    Arrays.fill(lrError, Float.NaN);
    // Median changes the matched coordinate, so validation must be repeated afterwards.
    final float[] photoChecked=checked;
    long[] postRejected=new long[height],photoRejected=new long[height];
    StereoRows.rows(0,height,y -> {
      for (int x = 0; x < width; x++) {
        int p = y * width + x;
        float disparity = photoChecked[p];
        if (!Float.isFinite(disparity)) continue;
        int xr = Math.round(x - disparity);
        if (xr >= 0 && xr < width && Float.isFinite(dr[y * width + xr]))
          lrError[p] = Math.abs(disparity - dr[y * width + xr]);
        if (!reciprocal(dr, width, height, x, y, disparity, options.lrThreshold, radius)) {
          photoChecked[p] = Float.NaN;
          postRejected[y]++;
          if (reasons != null) reasons[p] = 5;
          continue;
        }
        double score =
            zncc(photoLeft, photoRight, width, height, x, y, disparity, options.correlationWindow / 2, channels);
        correlation[p] = (float) score;
        if (score < options.correlation) {
          photoChecked[p] = Float.NaN;
          photoRejected[y]++;
          if (reasons != null) reasons[p] = 6;
        }
      }
    });
    long post = 0, photo = 0;
    for(long rejected:postRejected)post+=rejected;
    for(long rejected:photoRejected)photo+=rejected;
    RunTelemetry.record("Stereo post-median reciprocal/ZNCC check" + size, clock);
    return new Filtered(checked, correlation, lrError, lrTotal, post, photo);
  }

  static long memoryEstimate(int width, int height, int disparities, int census) {
    return (long) width * height * disparities * 6
        + (long) width * disparities * 8
        + (long) width * height * (68 + 16 * words(census));
  }

  static long memoryEstimate(int w, int h, int d, Options o) {
    return memoryEstimate(w, h, d, o.census)
        - (long) w * h * d * (4 - StereoCost.aggregateBytes(o))
        + PointConfidence.extraHostBytes(o, (long) w * h)
        + (o.matchingCost.equals("zncc") && !o.tiled ? ZnccCosts.workspace(w, o.census) : 0);
  }

  static long tiledMemoryEstimate(
      int width, int height, int disparities, int census, long memoryMb) {
    long cache = Math.min(32 * 1048576L, Math.max(1048576L, memoryMb * 1048576L / 16));
    return (long) width * height * (68 + 16 * words(census))
        + (long) width * disparities * 8
        + Math.max(1048576L, cache)
        + Math.max(2097152L, cache);
  }

  static long tiledMemoryEstimate(int width, int height, int disparities, Options o) {
    return tiledMemoryEstimate(width, height, disparities, o.census, o.memoryMb)
        + PointConfidence.extraHostBytes(o, (long) width * height);
  }

  static DescriptorSet colorCensus(byte[] bgr, int w, int h, int window) {
    int radius = window / 2;
    long[][] bits = new long[words(window)][w * h];
    boolean[] textured = new boolean[w * h];
    ThreadLocal<double[]> scratch = ThreadLocal.withInitial(() -> new double[window * window - 1]);
    StereoRows.rows(radius, h - radius, y -> {
      double[] distances = scratch.get();

      for (int x = radius; x < w - radius; x++) {
        int p = y * w + x, center = 3 * p, bit = 0;
        double total = 0, maximum = 0;
        for (int dy = -radius; dy <= radius; dy++)
          for (int dx = -radius; dx <= radius; dx++) {
            if (dx == 0 && dy == 0) continue;
            int q = 3 * ((y + dy) * w + x + dx);
            double squared = 0;
            for (int channel = 0; channel < 3; channel++) {
              int difference = (bgr[q + channel] & 255) - (bgr[center + channel] & 255);
              squared += difference * difference;
            }
            double distance = Math.sqrt(squared);
            distances[bit++] = distance;
            total += distance;
            maximum = Math.max(maximum, distance);
          }
        // Legacy mean includes the zero-distance center pixel; its fixed zero bit is omitted.
        double mean = total / (window * window);
        textured[p] = maximum >= 4;
        for (int k = 0; k < distances.length; k++)
          if (distances[k] >= mean) bits[k / 64][p] |= 1L << (k % 64);
      }
    });
    return new DescriptorSet(bits, textured);
  }

  static final class DescriptorSet {
    final long[][] bits;
    final boolean[] textured;

    DescriptorSet(long[][] bits, boolean[] textured) {
      this.bits = bits;
      this.textured = textured;
    }
  }

  private BandedSgm.Outcome banded(
      BandedSgm.Pair pair,
      boolean reverse,
      DisparityBands bands,
      Stats stats,
      byte[] reasons,
      byte[] searchBoundary) {
    Options matching = options.copy();
    matching.stereoSelection = reverse ? reverseSelection : selection;
    return BandedSgm.match(
        matching, pair, reverse, bands, options.tsgmLevels > 0, stats, reasons, searchBoundary);
  }

  private float[] match(
      long[][] a,
      long[][] b,
      byte[] grayA,
      byte[] grayB,
      boolean[] texturedA,
      boolean[] texturedB,
      int width,
      int height,
      boolean reverse,
      Stats stats,
      byte[] reasons,
      byte[] searchBoundary)
      throws IOException {
    int count = width * height, nd = options.disparities, radius = options.census / 2;
    Options matching = options.copy();
    matching.stereoSelection = reverse ? reverseSelection : selection;
    float[] gpu =
        GpuSgm.match(
            matching,
            a,
            b,
            grayA,
            grayB,
            texturedA,
            texturedB,
            width,
            height,
            reverse,
            stats,
            reasons,
            searchBoundary);
    if (gpu != null) return gpu;
    long budget =
        Math.min(options.memoryMb * 1048576L, (long) (Runtime.getRuntime().maxMemory() * .65));
    if (!options.tiled
        && ((long) count * nd > Integer.MAX_VALUE - 8
            || memoryEstimate(width, height, nd, options) > budget))
      throw new IllegalArgumentException(
          "CPU fallback SGM volume exceeds memory budget; enable --tiled or reduce dense"
              + " resolution");
    if (options.tiled) {
      if (tiledMemoryEstimate(width, height, nd, options) > budget)
        throw new IllegalArgumentException(
            "CPU fallback SGM rows/descriptors exceed the memory budget");
      java.nio.file.Files.createDirectories(options.output);
      long free = java.nio.file.Files.getFileStore(options.output).getUsableSpace();
      if (free > 0
          && (long) count * nd * (2 + StereoCost.aggregateBytes(options)) + 16777216L > free)
        throw new IllegalArgumentException(
            "CPU fallback needs more temporary disk space for its tiled volume");
    }
    try (PagedVolume costs =
            new PagedVolume(
                (long) count * nd, 2, INVALID_COST, options.tiled, options, "sgm-cost-");
        PagedVolume aggregate =
            new PagedVolume(
                (long) count * nd,
                StereoCost.aggregateBytes(options),
                0,
                options.tiled,
                options,
                "sgm-aggregate-")) {
      if (options.matchingCost.equals("zncc") && !options.tiled)
        ZnccCosts.fill(
            costs,
            grayA,
            grayB,
            texturedA,
            texturedB,
            width,
            height,
            options.minDisparity,
            nd,
            options.census,
            reverse);
      else
        for (int y = radius; y < height - radius; y++) {
          for (int x = radius; x < width - radius; x++) {
            int p = y * width + x;
            long base = (long) p * nd;
            if (!texturedA[p]) continue;
            for (int k = 0; k < nd; k++) {
              int disparity = options.minDisparity + k;
              long target = (long) x + (reverse ? disparity : -(long) disparity);
              if (target < radius || target >= width - radius) continue;
              int q = y * width + (int) target, hamming = 0;
              if (!texturedB[q]) continue;
              hamming = StereoCost.cost(options, a, b, grayA, grayB, texturedA, texturedB, p, q, width, height);
              costs.set(base + k, hamming);
            }
          }
        }
      int[][] paths = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, 1}, {1, -1}, {-1, -1}};
      int p1 = StereoCost.penalty(options.p1, options), p2 = StereoCost.p2(options);
      for (int path = 0; path < options.directions; path++) {
        int dx = paths[path][0], dy = paths[path][1];
        int[] previous = new int[width * nd], row = new int[width * nd];
        int[] previousMin = new int[width], rowMin = new int[width];
        int yStart = dy < 0 ? height - 1 : 0, yStep = dy < 0 ? -1 : 1;
        int xStart = dx < 0 ? width - 1 : 0, xStep = dx < 0 ? -1 : 1;
        for (int yi = 0, y = yStart; yi < height; yi++, y += yStep) {
          for (int xi = 0, x = xStart; xi < width; xi++, x += xStep) {
            int px = x - dx, py = y - dy, dest = x * nd;
            long base = ((long) y * width + x) * nd;
            boolean hasPrevious =
                px >= 0 && px < width && py >= 0 && py < height && texturedA[py * width + px];
            int[] prev = dy == 0 ? row : previous;
            int prevBase = hasPrevious ? px * nd : 0;
            int minPrev = hasPrevious ? (dy == 0 ? rowMin : previousMin)[px] : Integer.MAX_VALUE;
            int currentMin = Integer.MAX_VALUE;
            int localP2 =
                hasPrevious
                    ? StereoCost.pathP2(
                        p1, p2, options.sgmEdgeScale, grayA, y * width + x, py * width + px)
                    : p2;
            for (int k = 0; k < nd; k++) {
              int value = costs.get(base + k);
              if (hasPrevious) {
                int transition = Math.min(prev[prevBase + k], minPrev + localP2);
                if (k > 0) transition = Math.min(transition, prev[prevBase + k - 1] + p1);
                if (k + 1 < nd) transition = Math.min(transition, prev[prevBase + k + 1] + p1);
                value += transition - minPrev;
              }
              row[dest + k] = value;
              currentMin = Math.min(currentMin, value);
              aggregate.set(base + k, aggregate.get(base + k) + value);
            }
            rowMin[x] = currentMin;
          }
          int[] swap = previous;
          previous = row;
          row = swap;
          swap = previousMin; previousMin = rowMin; rowMin = swap;
        }
      }
      float[] result = new float[count];
      Arrays.fill(result, Float.NaN);
      for (int y = radius; y < height - radius; y++)
        for (int x = radius; x < width - radius; x++) {
          int pixel = y * width + x, best = -1, bestCost = Integer.MAX_VALUE;
          long base = (long) pixel * nd;
          for (int k = 0; k < nd; k++)
            if (costs.get(base + k) != INVALID_COST && aggregate.get(base + k) < bestCost) {
              best = k;
              bestCost = aggregate.get(base + k);
            }
          if (best < 0) continue;
          boolean full =
              costs.get(base) != INVALID_COST && costs.get(base + nd - 1) != INVALID_COST;
          if (full) {
            stats.fullIntervalPixels++;
            if (searchBoundary != null) searchBoundary[pixel] = 4;
          }
          if (best == 0 || best == nd - 1) {
            stats.boundaryRejected++;
            if (reasons != null) reasons[pixel] = 2;
            if (full && best == 0) {
              stats.lowerBoundaryRejected++;
              if (searchBoundary != null) searchBoundary[pixel] |= 1;
            }
            if (full && best == nd - 1) {
              stats.upperBoundaryRejected++;
              if (searchBoundary != null) searchBoundary[pixel] |= 2;
            }
            continue;
          }
          int second = Integer.MAX_VALUE;
          for (int k = 0; k < nd; k++)
            if (Math.abs(k - best) > 1 && costs.get(base + k) != INVALID_COST)
              second = Math.min(second, aggregate.get(base + k));
          if (second != Integer.MAX_VALUE
              && second - bestCost <= Math.max(1, bestCost * options.uniqueness)) {
            stats.ambiguityRejected++;
            if (reasons != null) reasons[pixel] = 3;
            continue;
          }
          double offset = 0;
          if (best > 0
              && best + 1 < nd
              && costs.get(base + best - 1) != INVALID_COST
              && costs.get(base + best + 1) != INVALID_COST) {
            double before = aggregate.get(base + best - 1), after = aggregate.get(base + best + 1);
            offset = StereoCost.subpixel(options, before, bestCost, after);
          }
          result[pixel] = (float) (options.minDisparity + best + offset);
          if (matching.stereoSelection != null) {
            double before =
                costs.get(base + best - 1) != INVALID_COST ? aggregate.get(base + best - 1) : -1;
            double after =
                costs.get(base + best + 1) != INVALID_COST ? aggregate.get(base + best + 1) : -1;
            matching.stereoSelection.put(pixel, bestCost, second, before, after);
          }
          stats.selected++;
          if (reasons != null) reasons[pixel] = 0;
        }
      return result;
    }
  }

  private static int words(int window) {
    return (window * window - 1 + 63) / 64;
  }

  static boolean[] textureMask(byte[] input, int w, int h, int window) {
    boolean[] mask = new boolean[w * h];
    int r = window / 2;
    StereoRows.rows(r, h - r, y -> {
      // Separable window extrema (column, then row): the same minimum and maximum.
      int[] columnLow = new int[w], columnHigh = new int[w];
      for (int x = 0; x < w; x++) {
        int low = 255, high = 0;
        for (int yy = -r; yy <= r; yy++) {
          int v = input[(y + yy) * w + x] & 255;
          low = Math.min(low, v);
          high = Math.max(high, v);
        }
        columnLow[x] = low;
        columnHigh[x] = high;
      }
      for (int x = r; x < w - r; x++) {
        int low = 255, high = 0;
        for (int xx = -r; xx <= r; xx++) {
          low = Math.min(low, columnLow[x + xx]);
          high = Math.max(high, columnHigh[x + xx]);
        }
        mask[y * w + x] = high - low >= 4;
      }
    });
    return mask;
  }

  static long[][] census(byte[] input, int w, int h, int window) {
    int r = window / 2;
    long[][] descriptors = new long[words(window)][w * h];
    StereoRows.rows(r, h - r, y -> {
      // Same bits as setting descriptors[bit / 64][p] per neighbour, accumulated per word.
      for (int x = r; x < w - r; x++) {
        int p = y * w + x, bit = 0, center = input[p] & 255;
        long word = 0;
        for (int yy = -r; yy <= r; yy++) {
          int row = (y + yy) * w + x;
          for (int xx = -r; xx <= r; xx++) {
            if (xx == 0 && yy == 0) continue;
            // Branch-free "neighbour < center": the sign bit of the difference.
            word |= (long) (((input[row + xx] & 255) - center) >>> 31) << (bit & 63);
            bit++;
            if ((bit & 63) == 0) {
              descriptors[(bit >>> 6) - 1][p] = word;
              word = 0;
            }
          }
        }
        if ((bit & 63) != 0) descriptors[bit >>> 6][p] = word;
      }
    });
    return descriptors;
  }

  /**
   * The element Arrays.sort(a, 0, n) would leave at a[n / 2], for finite values (n >= 1); a is
   * reordered. Bit for bit the same value, including the sign of a zero: Arrays.sort orders -0.0
   * before 0.0, as Math.min and Math.max do.
   */
  static float middle(float[] a, int n) {
    if (n == 9) {
      // Median of nine by a 19-exchange network (each exchange leaves min, max).
      float p0 = a[0], p1 = a[1], p2 = a[2], p3 = a[3], p4 = a[4], p5 = a[5], p6 = a[6], p7 = a[7], p8 = a[8], t;
      t = Math.min(p1, p2); p2 = Math.max(p1, p2); p1 = t;
      t = Math.min(p4, p5); p5 = Math.max(p4, p5); p4 = t;
      t = Math.min(p7, p8); p8 = Math.max(p7, p8); p7 = t;
      t = Math.min(p0, p1); p1 = Math.max(p0, p1); p0 = t;
      t = Math.min(p3, p4); p4 = Math.max(p3, p4); p3 = t;
      t = Math.min(p6, p7); p7 = Math.max(p6, p7); p6 = t;
      t = Math.min(p1, p2); p2 = Math.max(p1, p2); p1 = t;
      t = Math.min(p4, p5); p5 = Math.max(p4, p5); p4 = t;
      t = Math.min(p7, p8); p8 = Math.max(p7, p8); p7 = t;
      p3 = Math.max(p0, p3);
      p5 = Math.min(p5, p8);
      t = Math.min(p4, p7); p7 = Math.max(p4, p7); p4 = t;
      p6 = Math.max(p3, p6);
      p4 = Math.max(p1, p4);
      p2 = Math.min(p2, p5);
      p4 = Math.min(p4, p7);
      t = Math.min(p4, p2); p2 = Math.max(p4, p2); p4 = t;
      p4 = Math.max(p6, p4);
      return Math.min(p4, p2);
    }
    if (n < 16) {
      Arrays.sort(a, 0, n);
      return a[n / 2];
    }
    int k = n / 2;
    float v = ImageFusion.select(a, n, k);
    if (v != 0) return v;
    // A zero: the selection does not order -0.0 before 0.0; count what the sort puts before 0.0.
    int before = 0;
    for (int i = 0; i < n; i++) if (a[i] < 0 || Float.floatToRawIntBits(a[i]) == 0x80000000) before++;
    return k < before ? -0f : 0f;
  }

  static float[] medianValid(float[] a, int w, int h, int window) {
    float[] out = a.clone();
    ThreadLocal<float[]> scratch = ThreadLocal.withInitial(() -> new float[window * window]);
    int r = window / 2;
    StereoRows.rows(0, h, y -> {
      float[] values = scratch.get();

      for (int x = 0; x < w; x++) {
        if (!Float.isFinite(a[y * w + x])) continue; // do not fill occlusions
        int n = 0;
        for (int yy = Math.max(0, y - r); yy <= Math.min(h - 1, y + r); yy++)
          for (int xx = Math.max(0, x - r); xx <= Math.min(w - 1, x + r); xx++) {
            float v = a[yy * w + xx];
            if (Float.isFinite(v)) values[n++] = v;
          }
        out[y * w + x] = middle(values, n);
      }
    });
    return out;
  }

  /** Ordinary median on smooth support; restrict to the center's layer at a depth discontinuity. */
  static float[] medianEdgePreserving(
      float[] a, int w, int h, int window, double maximumChange, byte[] gray, int edgeThreshold) {
    float[] out = medianValid(a, w, h, window);
    ThreadLocal<float[]> scratch = ThreadLocal.withInitial(() -> new float[window * window]);
    int r = window / 2;
    StereoRows.rows(0, h, y -> {
      float[] values = scratch.get();

      for (int x = 0; x < w; x++) {
        float center = a[y * w + x];
        if (!Float.isFinite(center)) continue;
        if (Math.abs(out[y * w + x] - center) < maximumChange) continue;
        int n = 0;
        for (int yy = Math.max(0, y - r); yy <= Math.min(h - 1, y + r); yy++)
          for (int xx = Math.max(0, x - r); xx <= Math.min(w - 1, x + r); xx++) {
            float value = a[yy * w + xx];
            if (Float.isFinite(value)
                && Math.abs(value - center) <= maximumChange
                && (edgeThreshold == 0
                    || Math.abs((gray[yy * w + xx] & 255) - (gray[y * w + x] & 255))
                        <= edgeThreshold)) values[n++] = value;
          }
        if (n > 0) {
          out[y * w + x] = middle(values, n);
        }
      }
    });
    return out;
  }

  /** Zero-mean normalized cross correlation at the subpixel correspondence. */
  static boolean reciprocal(
      float[] right, int w, int h, int x, int y, float disparity, double tolerance, int border) {
    if (!Float.isFinite(disparity) || y < border || y >= h - border) return false;
    int xr = Math.round(x - disparity);
    return xr >= border
        && xr < w - border
        && Float.isFinite(right[y * w + xr])
        && Math.abs(disparity - right[y * w + xr]) <= tolerance;
  }

  /** Zero-mean normalized cross correlation at the subpixel correspondence. */
  static double zncc(
      byte[] a, byte[] b, int w, int h, int x, int y, double disparity, int r, int channels) {
    double target = x - disparity;
    int bx = (int) Math.floor(target);
    double fraction = target - bx;
    if (x - r < 0 || x + r >= w || y - r < 0 || y + r >= h || bx - r < 0 || bx + r + 1 >= w)
      return -1;
    double covariance = 0, varA = 0, varB = 0;
    int n = (2 * r + 1) * (2 * r + 1);
    for (int channel = 0; channel < channels; channel++) {
      double sa = 0, sb = 0, saa = 0, sbb = 0, sab = 0;
      for (int dy = -r; dy <= r; dy++)
        for (int dx = -r; dx <= r; dx++) {
          int ai = ((y + dy) * w + x + dx) * channels + channel,
              bi = ((y + dy) * w + bx + dx) * channels + channel;
          double av = a[ai] & 255,
              bv = (1 - fraction) * (b[bi] & 255) + fraction * (b[bi + channels] & 255);
          sa += av;
          sb += bv;
          saa += av * av;
          sbb += bv * bv;
          sab += av * bv;
        }
      covariance += sab - sa * sb / n;
      varA += saa - sa * sa / n;
      varB += sbb - sb * sb / n;
    }
    if (varA < channels * n * 4 || varB < channels * n * 4) return -1;
    return covariance / Math.sqrt(Math.max(1e-12, varA * varB));
  }

  /**
   * Opt-in --disparity-edge-erosion: rejects valid disparities next to depth jumps, about radius
   * pixels deep on both sides. A pixel is rejected when
   *
   * <ol>
   *   <li>another valid disparity within its (2*radius+1)^2 square differs by more than jump, or
   *   <li>it is within radius pixels of the end of its valid run along the row, and the valid pixel
   *       across the following invalid gap differs by more than jump, the gap being no wider than
   *       ceil(difference) + 2*radius. Rectified occlusions are horizontal gaps about one disparity
   *       difference wide; fattened disparities border them rather than the other surface.
   * </ol>
   *
   * Holes are never filled and values never moved. Decisions use the unmodified input only, so the
   * result is independent of scan order and of row blocking (rule 2 reads one row, rule 1 radius
   * rows above and below).
   */
  static long erodeDisparityEdges(
      float[] d, int w, int h, int radius, double jump, byte[] reasons) {
    if (radius <= 0) return 0;
    boolean[] reject = edgeFlags(d, w, h, 0, h, radius, jump);
    long rejected = 0;
    for (int i = 0; i < d.length; i++)
      if (reject[i]) {
        d[i] = Float.NaN;
        if (reasons != null) reasons[i] = 8;
        rejected++;
      }
    return rejected;
  }

  /** Edge-erosion decisions for rows [from, to) of d (w x h); see erodeDisparityEdges. */
  static boolean[] edgeFlags(
      float[] d, int w, int h, int from, int to, int radius, double jump) {
    boolean[] reject = new boolean[d.length];
    StereoRows.rows(from, to, y -> {
      int row = y * w;
      for (int x = 0; x < w; x++) {
        float v = d[row + x];
        if (!Float.isFinite(v)) continue;
        search:
        for (int yy = Math.max(0, y - radius); yy <= Math.min(h - 1, y + radius); yy++)
          for (int xx = Math.max(0, x - radius); xx <= Math.min(w - 1, x + radius); xx++) {
            float u = d[yy * w + xx];
            if (Float.isFinite(u) && Math.abs(u - v) > jump) {
              reject[row + x] = true;
              break search;
            }
          }
      }
      // Jumps across horizontal gaps (occlusions): flag radius pixels at both run ends.
      int x = 0;
      while (x < w && !Float.isFinite(d[row + x])) x++;
      while (x < w) {
        int end = x;
        while (end + 1 < w && Float.isFinite(d[row + end + 1])) end++;
        int next = end + 1;
        while (next < w && !Float.isFinite(d[row + next])) next++;
        if (next >= w) break;
        double difference = Math.abs(d[row + next] - d[row + end]);
        int gap = next - end - 1;
        if (difference > jump && gap <= Math.ceil(difference) + 2 * radius) {
          for (int k = Math.max(x, end - radius + 1); k <= end; k++) reject[row + k] = true;
          int nextEnd = next;
          while (nextEnd + 1 < w && Float.isFinite(d[row + nextEnd + 1])) nextEnd++;
          for (int k = next; k <= Math.min(nextEnd, next + radius - 1); k++) reject[row + k] = true;
        }
        x = next;
      }
    });
    return reject;
  }

  /** Compatibility entry point for four-connected disparity components. */
  static long removeSpeckles(float[] a, int w, int h, int minimum, double range) {
    return removeSpeckles(a, w, h, minimum, range, 4);
  }

  static long removeSpeckles(float[] a, int w, int h, int minimum, double range, int connectivity) {
    if (minimum <= 1) return 0;
    boolean[] seen = new boolean[a.length];
    int[] queue = new int[a.length];
    long rejected = 0;
    for (int start = 0; start < a.length; start++) {
      if (seen[start] || !Float.isFinite(a[start])) continue;
      int head = 0, tail = 0;
      queue[tail++] = start;
      seen[start] = true;
      while (head < tail) {
        int p = queue[head++], x = p % w, y = p / w;
        for (int dy = -1; dy <= 1; dy++)
          for (int dx = -1; dx <= 1; dx++) {
            if (dx == 0 && dy == 0 || connectivity == 4 && dx != 0 && dy != 0) continue;
            int xx = x + dx, yy = y + dy;
            if (xx < 0 || xx >= w || yy < 0 || yy >= h) continue;
            int q = yy * w + xx;
            if (seen[q] || !Float.isFinite(a[q]) || Math.abs(a[q] - a[p]) > range) continue;
            seen[q] = true;
            queue[tail++] = q;
          }
      }
      if (tail < minimum)
        for (int i = 0; i < tail; i++) {
          a[queue[i]] = Float.NaN;
          rejected++;
        }
    }
    return rejected;
  }

  public static final class Stats {
    public long depthOutlierRejected;
    /** --disparity-edge-erosion rejections (not part of the stereo-quality CSV). */
    public long edgeErosionRejected;
    public long[] depthOutlierReasons;
    public long lowerBoundaryRejected, fullIntervalPixels;
    public long coarseLowerBoundaryRejected, coarseUpperBoundaryRejected, coarseFullIntervalPixels;
    public long boundaryRejected,
        upperBoundaryRejected,
        ambiguityRejected,
        selected,
        lrRejected,
        postMedianLrRejected,
        photometricRejected,
        speckleRejected,
        valid;

    double lowerPressure() {
      return Math.max(
          lowerBoundaryRejected / (double) Math.max(1, fullIntervalPixels),
          coarseLowerBoundaryRejected / (double) Math.max(1, coarseFullIntervalPixels));
    }

    double upperPressure() {
      return Math.max(
          upperBoundaryRejected / (double) Math.max(1, fullIntervalPixels),
          coarseUpperBoundaryRejected / (double) Math.max(1, coarseFullIntervalPixels));
    }

    boolean lowerClipped() {
      return (lowerBoundaryRejected >= 24 || coarseLowerBoundaryRejected >= 12)
          && lowerPressure() > .02;
    }

    boolean upperClipped() {
      return (upperBoundaryRejected >= 24 || coarseUpperBoundaryRejected >= 12)
          && upperPressure() > .02;
    }

    public String csv() {
      return boundaryRejected
          + ","
          + upperBoundaryRejected
          + ","
          + ambiguityRejected
          + ","
          + selected
          + ","
          + lrRejected
          + ","
          + postMedianLrRejected
          + ","
          + photometricRejected
          + ","
          + speckleRejected
          + ","
          + valid;
    }
  }

  public static final class Result {
    public final int width, height;
    public final float[] disparity;
    public final Stats stats;
    public final float[] correlation, leftRightError;
    public float[] confidence, costMargin, costSharpness;
    public float[] rightConfidence;

    public Result(int width, int height, float[] disparity) {
      this(width, height, disparity, new Stats());
    }

    public Result(int width, int height, float[] disparity, Stats stats) {
      this(width, height, disparity, stats, null, null);
    }

    Result(
        int width,
        int height,
        float[] disparity,
        Stats stats,
        float[] correlation,
        float[] leftRightError) {
      this.width = width;
      this.height = height;
      this.disparity = disparity;
      this.stats = stats;
      this.correlation = correlation;
      this.leftRightError = leftRightError;
    }
  }

  interface Observer {
    void raster(String name, float[] values, int width, int height) throws IOException;

    default void reasons(byte[] codes, int width, int height) throws IOException {}

    default void searchBoundary(byte[] flags, int width, int height) throws IOException {}
  }
}
