package sgm;

import java.io.*;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import java.nio.file.*;
import java.util.*;
import java.util.concurrent.*;
import java.util.stream.IntStream;

/**
 * {@code --fusion-mode image}: per-camera fusion in image space (P2_IMAGE_FUSION.md).
 *
 * <ol>
 *   <li>Every stereo candidate is splatted into both cameras of its pair (nearest depth wins per
 *       pixel, as in DepthFusion.put).
 *   <li>Per camera pixel, the camera's pair samples are merged by the reference depth-mode rule
 *       (relative tolerance, majority fraction, triangulation-angle weights).
 *   <li>Merged maps are speckle filtered in 2D.
 *   <li>Each merged pixel is back-projected in FP64 and checked against overlapping cameras' merged
 *       maps: agreement (any merged depth within {@code --fusion-agreement-radius} pixels of the
 *       projection) adds a supporting camera, reliable free space at the centre pixel contradicts,
 *       missing depth and nearer occluders are neutral.
 *   <li>{@code --fusion-strategy}: {@code owner} (default) emits each surface once, by the
 *       confirming camera with the finest footprint, at that camera's depth. {@code owner-consume}
 *       adds, with cameras processed in order: a pixel is also a copy when an earlier agreeing camera
 *       already emitted a point there or an emitted point marked it as represented. {@code
 *       graph-average}: every unused pixel seeds one point at the inverse-variance-weighted mean of
 *       its own and all agreeing cameras' depths along its ray; the agreeing member pixels are then
 *       used. Copies are combined instead of chosen.
 * </ol>
 *
 * <p>Bounded memory: candidates are read pair by pair from the candidate stream. Pass 1 keeps only
 * each camera's filtered merged depth (4 bytes per pixel); pass 2 re-merges one camera at a time
 * (deterministically identical) for cross-checks and emission. Camera sets are multi-word masks.
 */
final class ImageFusion {
  /** Rejection and emission counters for the fusion summary. */
  static final class Counters {
    long pixels, ambiguous, spikes, freeSpace, speckle, unsupported, visibility, uncertainty, twoView, notOwner, consumed, emitted, refined;
    long members, depthOutliers;
    long ambiguousKept, pairReads, bytesRead;
    long cacheHits, cachePeakBytes, cacheLimitBytes;
    long edgeSupported,edgeFreeSpace,edgeUnknown;
    long patchSuspects,patchUnsupported,patchFreeSpace,uncertainAverageSkipped,averageReverted;
    long precisionRejected, narrowRejected, localSigmaRejected;
    boolean edgeEnabled, thinEnabled;
    // Memory plan and depth-map spill counters (FUSION_MEMORY.md).
    Object memory;

    long rejected() {
      return ambiguous + spikes + freeSpace + speckle + unsupported + visibility + uncertainty + twoView
          + precisionRejected + narrowRejected + localSigmaRejected;
    }

    Object json() {
      Map<String, Object> out = Json.object(
          "merged_valid_pixels", pixels,
          "ambiguous_layer_rejected", ambiguous,
          "ambiguous_nearer_layer_kept", ambiguousKept,
          "spike_rejected", spikes,
          "free_space_rejected", freeSpace,
          "speckle_rejected", speckle,
          "depth_outlier_rejected", depthOutliers,
          "support_rejected", unsupported,
          "visibility_contradiction_rejected", visibility,
          "depth_uncertainty_rejected", uncertainty,
          "two_view_rejected", twoView,
          "owned_by_other_camera", notOwner,
          "consumed_by_earlier_point", consumed,
          "graph_members_averaged", members,
          "emitted", emitted,
          "pair_reads", pairReads,
          "candidate_bytes_read", bytesRead,
          "candidate_cache_hits", cacheHits,
          "candidate_cache_peak_bytes", cachePeakBytes,
          "candidate_cache_limit_bytes", cacheLimitBytes,
          "edge_aware_camera_checks",edgeEnabled?Json.object(
              "version",3,"positive_support","Precise center measurement; uncertainty compatibility is neutral",
              "supported_checks",edgeSupported,"free_space_rejected_checks",edgeFreeSpace,
              "unknown_or_occluded_checks",edgeUnknown,"surface_patch_suspects",patchSuspects,
              "surface_patch_without_precise_support_rejected",patchUnsupported,
              "surface_patch_free_space_votes",patchFreeSpace,
              "uncertain_averaging_members_skipped",uncertainAverageSkipped,
              "surface_departing_averages_reverted",averageReverted):null);
      if (memory != null) out.put("memory", memory);
      if (thinEnabled)
        out.put(
            "thin_surface_checks",
            Json.object(
                "precision_rejected", precisionRejected,
                "narrow_unconfirmed_rejected", narrowRejected,
                "local_sigma_rejected", localSigmaRejected));
      return out;
    }
  }

  private final Options o;
  private final DepthFusion fusion;
  private final int cameras, words, fields, record;
  private final Path candidates;
  private final long[] start;
  private final LinkedHashMap<Integer, PairData> pairCache = new LinkedHashMap<>(16, .75f, true);
  private final long cacheLimit;
  private final int passOneCameras;
  private long cachedBytes;
  private ByteBuffer readBuffer;

  /**
   * A per-camera float map indexed by pixel: a heap array, or (fusion in strips) a memory-mapped
   * file read and written in place.
   */
  static final class Raster {
    final float[] a;
    final java.lang.foreign.MemorySegment s;

    Raster(float[] a) {
      this.a = a;
      s = null;
    }

    Raster(java.lang.foreign.MemorySegment s) {
      a = null;
      this.s = s;
    }

    float get(long i) {
      return a != null ? a[(int) i] : s.getAtIndex(MappedFiles.FLOAT, i);
    }

    void set(long i, float v) {
      if (a != null) a[(int) i] = v;
      else s.setAtIndex(MappedFiles.FLOAT, i, v);
    }

    /** Copies count values from offset into dst[at..]. */
    void read(long offset, float[] dst, int at, int count) {
      if (a != null) System.arraycopy(a, (int) offset, dst, at, count);
      else java.lang.foreign.MemorySegment.copy(s, MappedFiles.FLOAT, 4 * offset, dst, at, count);
    }

    /** Copies src[at..at + count) to offset. */
    void write(long offset, float[] src, int at, int count) {
      if (a != null) System.arraycopy(src, at, a, (int) offset, count);
      else java.lang.foreign.MemorySegment.copy(src, at, s, MappedFiles.FLOAT, 4 * offset, count);
    }
  }

  /** Per-pixel marks: a heap bit set, or (fusion in strips) a memory-mapped one. */
  static final class Bits {
    final long[] a;
    final java.lang.foreign.MemorySegment s;

    Bits(long[] a) {
      this.a = a;
      s = null;
    }

    Bits(java.lang.foreign.MemorySegment s) {
      a = null;
      this.s = s;
    }

    boolean get(int q) {
      long word = a != null ? a[q >>> 6] : s.getAtIndex(MappedFiles.LONG, q >>> 6);
      return (word & 1L << (q & 63)) != 0;
    }

    void set(int q) {
      if (a != null) a[q >>> 6] |= 1L << (q & 63);
      else s.setAtIndex(MappedFiles.LONG, q >>> 6, s.getAtIndex(MappedFiles.LONG, q >>> 6) | 1L << (q & 63));
    }
  }

  private final ForkJoinPool pool;
  private final Raster[] compact;
  private SurfacePatchGrid[] surfaceContext;
  private final int[][] neighbours;
  // owner-consume / graph-average: per camera, merged pixels already represented by an emitted point.
  private final Bits[] used;
  // owner-consume: per camera, pixels whose own point was emitted.
  private final Bits[] emitted;
  // graph-average: per camera, the merged pixels' inverse-variance weight sums.
  private final Raster[] compactWeight;
  private final List<List<Integer>> pairsOf = new ArrayList<>();
  // --thin-surfaces parts (all off: identical to the previous output).
  private final boolean sigmaTolerance, precisionCheck, storeWeights;
  private final int tierCount;
  private final double sigmaK, seedMinSine, localSigma;
  // --fusion-local-sigma: per camera, the smallest merged sigma (depth units) within 4 pixels.
  private final Raster[] localBest;

  // Fusion in row strips (STREAMED_PROCESSING.md): when one camera's work does not fit the heap
  // (or --streamed-processing on, or -Dsgm.fusion.tileRows=N), each camera is processed in strips
  // of stripRows rows plus a halo covering every neighbourhood operation, and every camera's
  // maps (depth, weights, local sigma, marks) live in memory-mapped files under 06_fusion/map-spill.
  // The result is identical to the untiled path.
  private final boolean tiled;
  private final int stripRows;
  private MappedFiles tileFiles;
  // Halo rows: pass 1 spike filter (5x5 window: 2), pass 2 normals (7x7 at stride 4: 12); +1 for
  // the merge's 3x3 search, which makes a region's outermost row inexact.
  static final int PASS_ONE_HALO = 3, PASS_TWO_HALO = 13, SIGMA_HALO = 4, MIN_STRIP_ROWS = 32;

  // Memory plan and resident set (FUSION_MEMORY.md). A camera's maps (compact, compactWeight,
  // localBest, used, emitted, surfaceContext) are either resident in those arrays or spilled to
  // spillDirectory; hasMap says whether the camera has a map at all.
  static final int BAND_PIXELS = 1 << 18, MIN_BAND_PIXELS = 1 << 14;
  private final int bandPixels;
  // Heap the speckle filter may use on the strip path (SpeckleFilter.threadsFor).
  private final long speckleBudget;
  // Surface-patch cache cells per camera (SurfacePatchGrid.cacheCells, or the whole grid).
  private int gridCells;
  private final long workingBytes;
  static final long EMISSION_BYTES = 640;
  private final boolean[] hasMap, spilledMaps;
  private final long mapBudget;
  private final boolean normalCache;
  private final String plan;
  private Path spillDirectory;
  private long allMapBytes, worstMap, residentBytes;
  private final LinkedHashMap<Integer, Long> resident = new LinkedHashMap<>(16, .75f, true);
  private long mapLoads, mapEvictions, spillWritten, spillRead;

  /** One pair's candidates, read from the stream; sinef = f * sin(triangulation angle). */
  private static final class PairData {
    final int pair, count;
    final double[] world;
    final float[] sinef, evidence;
    final int[] rgb, pixel;
    // --fusion-seed-min-angle: sinef below this is a narrow-pair candidate.
    double narrowSinef;
    // Pair-local index of element 0 (a row range of an mvsgm camera's candidates; 0 otherwise).
    long first;

    PairData(int pair, int count, int fields, boolean full) {
      this.pair = pair;
      this.count = count;
      world = new double[3 * count];
      sinef = new float[count];
      rgb = full ? new int[count] : null;
      pixel = full ? new int[count] : null;
      evidence = full ? new float[count * fields] : null;
    }

    double[] point(int i) {
      return new double[] {world[3 * i], world[3 * i + 1], world[3 * i + 2]};
    }

    long bytes() {
      // Include array/object headers and the LRU entry conservatively.
      return 256L + 8L * world.length + 4L * sinef.length
          + (rgb == null ? 0 : 4L * rgb.length + 4L * pixel.length + 4L * evidence.length);
    }
  }

  /**
   * Merged per-camera depth: 0 = invalid. Offsets are the representative's subpixel position.
   * Covers image rows [row0, row0 + h) (the whole image unless fusing in strips); arrays are
   * indexed by local pixel (y - row0) * w + x.
   */
  private static final class Merged {
    final int w, h, words;
    int row0;
    final float[] z, wsum, du, dv;
    final int[] repPair, repIndex;
    final byte[] support;
    final long[] partners;
    final boolean[] ambiguous;
    // --fusion-seed-min-angle: every merged member came from a narrow-angle candidate.
    final boolean[] narrow;

    Merged(int w, int h, int words) {
      this.w = w;
      this.h = h;
      this.words = words;
      int n = w * h;
      z = new float[n];
      wsum = new float[n];
      du = new float[n];
      dv = new float[n];
      repPair = new int[n];
      repIndex = new int[n];
      support = new byte[n];
      partners = new long[(long) n * words > Integer.MAX_VALUE - 8 ? 0 : n * words];
      ambiguous = new boolean[n];
      narrow = new boolean[n];
      Arrays.fill(repIndex, -1);
    }
  }

  /** One camera's splat of one pair: the winning candidate (pair-local index) and depth per pixel. */
  private record Splat(PairData data, int other, int[] sample, float[] z, float[] du, float[] dv) {}

  // --dense-method mvsgm: candidates come from the cameras' maps instead of the candidate file.
  private final MvsgmMaps maps;

  private ImageFusion(Options o, DepthFusion fusion, Path candidates, long[] start, MvsgmMaps maps) {
    this.o = o;
    this.fusion = fusion;
    this.candidates = candidates;
    this.start = start;
    this.maps = maps;
    sigmaTolerance = o.fusionTolerance.equals("sigma");
    precisionCheck = o.fusionPrecisionCheck;
    sigmaK = o.fusionSigmaK;
    seedMinSine = o.fusionSeedMinAngle > 0 ? Math.sin(Math.toRadians(o.fusionSeedMinAngle)) : 0;
    localSigma = o.fusionLocalSigma;
    tierCount = o.fusionSeedOrder.equals("precision") ? (seedMinSine > 0 ? 3 : 2) : 1;
    storeWeights =
        o.fusionStrategy.equals("graph-average") || DepthOutlierFilter.enabled(o) || o.thinSurfaceFusion();
    cameras = fusion.cameraCount();
    words = (cameras + 63) / 64;
    fields = Double.isFinite(fusion.twoViewConfidenceThreshold()) ? 6 : 5;
    record = 24 + 3 + 8 + 4 * fields;
    for (int c = 0; c < cameras; c++) pairsOf.add(new ArrayList<>());
    for (int k = 0; k < fusion.pairCount(); k++) {
      int a = fusion.pairCamera(k, false), b = fusion.pairCamera(k, true);
      pairsOf.get(a).add(k);
      if (b != a && !fusion.singleView(k)) pairsOf.get(b).add(k);
    }
    // Memory plan (FUSION_MEMORY.md). Per camera: its filtered depth map and companions (weights,
    // local sigma, mark bits, surface-patch cache); one camera at a time also needs its pairs'
    // candidates, splats and merge (worstCamera), filter temporaries, the normal cache and the
    // emissions of up to three row bands. If every camera's maps fit next to that, all stay
    // resident (no temporary files; the fastest path). Otherwise maps are spilled to files under
    // 06_fusion and only the camera being fused and its overlapping cameras are resident; the
    // operating system's file cache then uses whatever RAM is free. Neither changes any result.
    gridCells = SurfacePatchGrid.cacheCells(o, cameras);
    long worstCamera = 0, worstPixels = 0, speckleWork = 0, speckleLimit = Long.MAX_VALUE;
    int threads = Math.max(1, o.cores), speckleThreads = threads;
    for (int c = 0; c < cameras; c++) {
      long pixels = (long) fusion.width(c) * fusion.height(c), pairBytes = 0;
      allMapBytes += mapBytes(c);
      worstMap = Math.max(worstMap, mapBytes(c));
      worstPixels = Math.max(worstPixels, pixels);
      for (int k : pairsOf.get(c)) pairBytes += (start[k + 1] - start[k]) * (24L + 12 + 4L * fields) + 20 * pixels;
      // + spike marks (1 byte per pixel) and the speckle filter's strip buffers and border nodes.
      long speckle =
          SpeckleFilter.workingBytes(
              fusion.width(c), fusion.height(c), SpeckleFilter.rows(fusion.height(c), threads), threads);
      speckleWork = Math.max(speckleWork, speckle);
      worstCamera = Math.max(worstCamera, pairBytes + pixels * (34L + 8L * words + 1) + speckle);
    }
    boolean surfaceAttributes = o.pointAttributes.equals("surface") || o.surfaceConsolidation.equals("normal");
    long normalBytes = surfaceAttributes && 24L * worstPixels <= 512L << 20 ? 24L * worstPixels : 0;
    long budget = (long) (Runtime.getRuntime().maxMemory() * .75);
    long forced = Long.getLong("sgm.fusion.mapBudgetMb", -1L);
    long requestedCache = o.memoryMb * 1048576 / 4;
    if (o.imageFusionCacheMb >= 0)
      requestedCache = Math.min(requestedCache, (long) o.imageFusionCacheMb * 1048576);
    long avail = budget - budget / 8, base = worstCamera + normalBytes;
    long minEmit = 3L * MIN_BAND_PIXELS * EMISSION_BYTES;
    long needed = allMapBytes + base + minEmit;
    // Strips: forced by a test property or --streamed-processing on; with auto only when even the
    // spilled plan cannot hold one camera's work.
    int forcedRows = Integer.getInteger("sgm.fusion.tileRows", 0);
    boolean wantTiles = forcedRows > 0 || o.streamedProcessing.equals("on");
    boolean spillFits = avail - worstCamera - minEmit >= 2 * worstMap;
    tiled = wantTiles || !(forced < 0 && needed <= avail) && !spillFits && !o.streamedProcessing.equals("off");
    if (tiled) {
      // One camera at a time, strip by strip; maps are memory-mapped files, so the heap holds
      // one strip's work (merge, splats, candidates, normal cache), emission bands and the
      // surface-patch caches. Pairs of the candidate file are read whole for every strip.
      passOneCameras = 1;
      cacheLimit = 0;
      mapBudget = Long.MAX_VALUE;
      normalCache = surfaceAttributes;
      int widest = 1, tallest = 1, pairsMax = 1;
      long pairData = 0;
      for (int c = 0; c < cameras; c++) {
        widest = Math.max(widest, fusion.width(c));
        tallest = Math.max(tallest, fusion.height(c));
        pairsMax = Math.max(pairsMax, pairsOf.get(c).size());
        long bytes = 0;
        for (int k : pairsOf.get(c)) bytes += (start[k + 1] - start[k]) * (36L + 4L * fields);
        pairData = Math.max(pairData, bytes);
      }
      long perPixel = 27 + 8L * words + 16L * pairsMax + 1 + (maps != null ? 36 + 4L * fields : 0) + (normalCache ? 24 : 0);
      long fixed = (DepthOutlierFilter.enabled(o) ? 4096L * gridCells * cameras : 0) + (maps == null ? pairData : 0);
      long left = avail - fixed;
      bandPixels = bandFor(left / 4);
      left -= 3L * bandPixels * EMISSION_BYTES;
      long rows = left / 2 / (perPixel * widest) - 2L * PASS_TWO_HALO;
      if (forcedRows <= 0 && rows < MIN_STRIP_ROWS)
        throw new IllegalArgumentException(
            "Image-space fusion needs at least about "
                + (fixed + minEmit + 2 * perPixel * widest * (MIN_STRIP_ROWS + 2L * PASS_TWO_HALO)) * 8 / 7 * 4 / 3 / 1048576
                + " MiB heap even in strips (" + perPixel + " bytes per pixel of a strip, " + fixed / 1048576
                + " MiB fixed); raise PHOTOGRAMMETRY_HEAP/-Xmx or lower --quality");
      stripRows = (int) Math.max(1, Math.min(tallest, forcedRows > 0 ? forcedRows : rows));
      // The speckle filter runs over a camera's mapped map when no strip is in work: it gets the
      // strips' share of the heap, on as many threads as fit (at least one).
      speckleLimit = Math.max(1, left / 2);
      speckleWork = 0;
      for (int c = 0; c < cameras; c++) {
        int sw = fusion.width(c), sh = fusion.height(c);
        int t = SpeckleFilter.threadsFor(sw, sh, threads, speckleLimit);
        speckleThreads = Math.min(speckleThreads, t);
        speckleWork = Math.max(speckleWork, SpeckleFilter.workingBytes(sw, sh, SpeckleFilter.rows(sh, t), t));
      }
      workingBytes = perPixel * widest * (stripRows + 2L * PASS_TWO_HALO) + 3L * bandPixels * EMISSION_BYTES + fixed;
      spillDirectory = o.output.resolve("06_fusion/map-spill");
    } else if (forced < 0 && needed <= avail) {
      stripRows = 0;
      mapBudget = Long.MAX_VALUE;
      normalCache = true;
      // Emission bands get up to half of what is left (full bands when there is room).
      // Spare heap first lets every surface-patch grid cache its whole image (no model is built
      // twice; models are deterministic), keeping one more camera's work in reserve.
      if (DepthOutlierFilter.enabled(o)) {
        int full = 16;
        for (int c = 0; c < cameras; c++)
          full = Math.max(full, Math.ceilDiv(fusion.width(c), 16) * Math.ceilDiv(fusion.height(c), 16));
        long extra = 4096L * Math.max(0, full - gridCells) * cameras;
        if (extra > 0 && allMapBytes + extra + base + minEmit + worstCamera <= avail) {
          gridCells = full;
          allMapBytes += extra;
        }
      }
      bandPixels = bandFor((avail - allMapBytes - base) / 2);
      long working = base + 3L * bandPixels * EMISSION_BYTES;
      // Pass 1 may work on several cameras at once: each extra camera needs another worst-case
      // camera workspace. At most half of the remaining headroom is used for that; the pair cache
      // gets what is left. Neither changes any result.
      long headroom = Math.max(0, avail - allMapBytes - working);
      passOneCameras =
          (int) Math.max(1, Math.min(Math.max(1, o.cores / 2), 1 + headroom / 2 / Math.max(1, worstCamera)));
      cacheLimit = Math.max(0, Math.min(requestedCache, headroom - (passOneCameras - 1) * worstCamera));
      workingBytes = working;
    } else {
      // Spilled maps: one camera at a time; maps get what one camera's work leaves (at least two
      // cameras' worth, checked again once the overlap is known), the pair cache the rest.
      stripRows = 0;
      passOneCameras = 1;
      long free = avail - worstCamera - minEmit;
      if (free < 2 * worstMap)
        throw new IllegalArgumentException(
            "Image-space fusion needs at least about "
                + (budget / 8 + worstCamera + minEmit + 2 * worstMap) * 4 / 3 / 1048576
                + " MiB heap even with depth maps on disk (largest camera's work "
                + worstCamera / 1048576 + " MiB, maps " + worstMap / 1048576
                + " MiB per camera); raise PHOTOGRAMMETRY_HEAP/-Xmx, lower --quality or use"
                + " --streamed-processing auto (fusion in strips)");
      normalCache = normalBytes > 0 && free - normalBytes >= 4 * worstMap;
      if (normalCache) free -= normalBytes;
      bandPixels = bandFor(free / 4 + minEmit);
      free -= 3L * bandPixels * EMISSION_BYTES - minEmit;
      mapBudget = forced >= 0 ? forced << 20 : Math.max(2 * worstMap, free * 3 / 4);
      cacheLimit = Math.max(0, Math.min(requestedCache, free - mapBudget));
      workingBytes = worstCamera + (normalCache ? normalBytes : 0) + 3L * bandPixels * EMISSION_BYTES;
      spillDirectory = o.output.resolve("06_fusion/map-spill");
    }
    plan = String.format(
        Locale.ROOT,
        "Image fusion memory plan: heap budget %,d MiB; depth maps %,d MiB for %d cameras (%s);"
            + " largest camera's work %,d MiB; %d camera(s) at once in pass 1; candidate cache %,d MiB;"
            + " normal cache %s; row bands of %,d pixels; speckle filter %,d MiB on up to %d thread(s)",
        budget >> 20, allMapBytes >> 20, cameras,
        tiled ? "memory-mapped files; fusion in strips of " + stripRows + " rows"
            : mapBudget == Long.MAX_VALUE ? "all resident" : "spilled to disk, " + (mapBudget >> 20) + " MiB resident",
        workingBytes >> 20, passOneCameras, cacheLimit >> 20, normalCache ? "on" : "off", bandPixels,
        speckleWork + 1048575 >> 20, speckleThreads);
    speckleBudget = speckleLimit;
    System.out.println(plan);
    compact = new Raster[cameras];
    neighbours = new int[cameras][];
    used = new Bits[cameras];
    emitted = new Bits[cameras];
    compactWeight = new Raster[cameras];
    localBest = new Raster[cameras];
    hasMap = new boolean[cameras];
    spilledMaps = new boolean[cameras];
    pool = new ForkJoinPool(Math.max(1, o.cores));
  }

  static Counters run(
      Options o,
      DepthFusion fusion,
      Path candidates,
      long[] pairStarts,
      FusionOutput output,
      TwoViewConfidence.Diagnostics diagnostics)
      throws IOException {
    return run(o, fusion, candidates, pairStarts, output, diagnostics, null);
  }

  /** maps: the mvsgm hand-off (each camera's maps as its single-view pair), or null. */
  static Counters run(
      Options o,
      DepthFusion fusion,
      Path candidates,
      long[] pairStarts,
      FusionOutput output,
      TwoViewConfidence.Diagnostics diagnostics,
      MvsgmMaps maps)
      throws IOException {
    ImageFusion f = new ImageFusion(o, fusion, candidates, pairStarts, maps);
    try {
      Counters c = f.fuse(output, diagnostics);
      c.memory = Json.object(
          "plan", f.plan, "map_loads", f.mapLoads, "map_evictions", f.mapEvictions,
          "spill_bytes_written", f.spillWritten, "spill_bytes_read", f.spillRead,
          "strip_rows", f.stripRows, "mapped_bytes", f.tileFiles == null ? 0 : f.tileFiles.mappedBytes());
      if (f.tiled)
        System.out.printf(Locale.ROOT, "Image fusion in strips of %,d rows: %,d MiB of memory-mapped maps%n",
            f.stripRows, f.tileFiles == null ? 0 : f.tileFiles.mappedBytes() >> 20);
      else if (f.spillDirectory != null)
        System.out.printf(Locale.ROOT, "Image fusion depth maps on disk: %,d loads, %,d evictions, %,d MiB written, %,d MiB read%n",
            f.mapLoads, f.mapEvictions, f.spillWritten >> 20, f.spillRead >> 20);
      return c;
    } finally {
      f.pairCache.clear();
      f.readBuffer = null;
      f.pool.shutdown();
      f.deleteSpill();
    }
  }

  // ---------------------------------------------------------------------------------------------
  // Resident set of per-camera maps

  /** Row-band size whose emissions (three bands alive: one checked, two queued) fit the bytes. */
  static int bandFor(long bytes) {
    long pixels = bytes / (3 * EMISSION_BYTES);
    return (int) Math.max(MIN_BAND_PIXELS, Math.min(BAND_PIXELS, pixels));
  }

  /** Heap bytes of camera c's resident maps (depth, weights, local sigma, marks, patch cache). */
  private long mapBytes(int c) {
    long pixels = (long) fusion.width(c) * fusion.height(c);
    long bytes = 4 * pixels;
    if (storeWeights) bytes += 4 * pixels;
    if (localSigma > 0) bytes += 4 * pixels;
    if (o.fusionStrategy.equals("owner-consume") || o.fusionStrategy.equals("graph-average")) bytes += pixels / 8 + 64;
    if (o.fusionStrategy.equals("owner-consume")) bytes += pixels / 8 + 64;
    if (DepthOutlierFilter.enabled(o)) bytes += 4096L * gridCells;
    return bytes;
  }

  /** Pass 1: camera c's maps are complete; record them, spilling others if over the plan. */
  private synchronized void admit(int c, Raster z, Raster weight, Raster best) throws IOException {
    compact[c] = z;
    compactWeight[c] = weight;
    localBest[c] = best;
    hasMap[c] = true;
    // Strips: every map is a memory-mapped file; nothing is resident on the heap.
    if (tiled) return;
    resident.put(c, mapBytes(c));
    residentBytes += mapBytes(c);
    evictBeyond(mapBudget, Set.of(c));
  }

  /** The camera's surface-patch grid over its resident depth map (models are deterministic). */
  private void grid(int c) {
    if (surfaceContext == null || compact[c] == null) return;
    final float[] array = compact[c].a;
    final java.lang.foreign.MemorySegment mapped = compact[c].s;
    surfaceContext[c] = new SurfacePatchGrid(fusion.width(c), fusion.height(c),
        array != null ? p -> array[p] : p -> mapped.getAtIndex(MappedFiles.FLOAT, p), o.fusionRelative, gridCells);
  }

  /** Pass 2: camera c and the cameras it cross-checks become resident; others may be spilled. */
  private synchronized void residentFor(int c) throws IOException {
    if (tiled) return;
    Set<Integer> needed = new HashSet<>();
    needed.add(c);
    for (int j : neighbours[c]) needed.add(j);
    for (int j : needed) {
      if (!hasMap[j]) continue;
      if (resident.containsKey(j)) resident.get(j);
      else load(j);
    }
    evictBeyond(mapBudget, needed);
  }

  /** Spilled mode needs one camera plus all of its overlapping cameras resident at once. */
  private void checkWorkingSet() {
    if (spillDirectory == null || tiled) return;
    long most = 0;
    int widest = 0;
    for (int c = 0; c < cameras; c++) {
      if (!hasMap[c]) continue;
      long bytes = mapBytes(c);
      for (int j : neighbours[c]) bytes += mapBytes(j);
      most = Math.max(most, bytes);
      widest = Math.max(widest, neighbours[c].length);
    }
    if (most > mapBudget)
      System.out.printf(Locale.ROOT,
          "Image fusion: up to %d overlapping cameras need %,d MiB of resident maps (plan %,d MiB);"
              + " the resident set grows to that while a camera is fused%n",
          widest, most >> 20, mapBudget >> 20);
  }

  private void evictBeyond(long limit, Set<Integer> keep) throws IOException {
    if (residentBytes <= limit) return;
    var it = resident.entrySet().iterator();
    while (residentBytes > limit && it.hasNext()) {
      var e = it.next();
      int j = e.getKey();
      if (keep.contains(j)) continue;
      spill(j);
      residentBytes -= e.getValue();
      it.remove();
      mapEvictions++;
    }
  }

  private Path mapFile(int c) {
    return spillDirectory.resolve("camera_" + c + ".map");
  }

  private Path bitsFile(int c) {
    return spillDirectory.resolve("camera_" + c + ".bits");
  }

  /** Writes camera j's maps (depth etc. once; marks every time) and drops them from the heap. */
  private void spill(int j) throws IOException {
    if (spillDirectory == null) spillDirectory = o.output.resolve("06_fusion/map-spill");
    Files.createDirectories(spillDirectory);
    if (!spilledMaps[j]) {
      try (FileChannel ch = FileChannel.open(mapFile(j), StandardOpenOption.CREATE, StandardOpenOption.TRUNCATE_EXISTING, StandardOpenOption.WRITE)) {
        writeFloats(ch, compact[j].a);
        if (compactWeight[j] != null) writeFloats(ch, compactWeight[j].a);
        if (localBest[j] != null) writeFloats(ch, localBest[j].a);
      }
      spilledMaps[j] = true;
    }
    if (used[j] != null || emitted[j] != null)
      try (FileChannel ch = FileChannel.open(bitsFile(j), StandardOpenOption.CREATE, StandardOpenOption.TRUNCATE_EXISTING, StandardOpenOption.WRITE)) {
        ByteBuffer head = ByteBuffer.allocate(2).put((byte) (used[j] != null ? 1 : 0)).put((byte) (emitted[j] != null ? 1 : 0)).flip();
        while (head.hasRemaining()) ch.write(head);
        spillWritten += 2;
        if (used[j] != null) writeLongs(ch, used[j].a);
        if (emitted[j] != null) writeLongs(ch, emitted[j].a);
      }
    compact[j] = null;
    compactWeight[j] = null;
    localBest[j] = null;
    used[j] = null;
    emitted[j] = null;
    if (surfaceContext != null) surfaceContext[j] = null;
  }

  private void load(int j) throws IOException {
    int n = fusion.width(j) * fusion.height(j);
    try (FileChannel ch = FileChannel.open(mapFile(j), StandardOpenOption.READ)) {
      compact[j] = new Raster(readFloats(ch, n));
      if (storeWeights) compactWeight[j] = new Raster(readFloats(ch, n));
      if (localSigma > 0) localBest[j] = new Raster(readFloats(ch, n));
    }
    if (Files.exists(bitsFile(j)))
      try (FileChannel ch = FileChannel.open(bitsFile(j), StandardOpenOption.READ)) {
        ByteBuffer head = ByteBuffer.allocate(2);
        while (head.hasRemaining()) if (ch.read(head) < 0) throw new EOFException("Fusion spill truncated");
        head.flip();
        boolean u = head.get() != 0, e = head.get() != 0;
        int words = (n + 63) / 64;
        if (u) used[j] = new Bits(readLongs(ch, words));
        if (e) emitted[j] = new Bits(readLongs(ch, words));
      }
    grid(j);
    resident.put(j, mapBytes(j));
    residentBytes += mapBytes(j);
    mapLoads++;
  }

  private void writeFloats(FileChannel ch, float[] a) throws IOException {
    ByteBuffer b = ByteBuffer.allocate(1 << 20).order(java.nio.ByteOrder.LITTLE_ENDIAN);
    for (int i = 0; i < a.length; ) {
      b.clear();
      int k = Math.min(a.length - i, b.capacity() / 4);
      b.asFloatBuffer().put(a, i, k);
      b.limit(4 * k);
      while (b.hasRemaining()) ch.write(b);
      i += k;
    }
    spillWritten += 4L * a.length;
  }

  private void writeLongs(FileChannel ch, long[] a) throws IOException {
    ByteBuffer b = ByteBuffer.allocate(1 << 20).order(java.nio.ByteOrder.LITTLE_ENDIAN);
    for (int i = 0; i < a.length; ) {
      b.clear();
      int k = Math.min(a.length - i, b.capacity() / 8);
      b.asLongBuffer().put(a, i, k);
      b.limit(8 * k);
      while (b.hasRemaining()) ch.write(b);
      i += k;
    }
    spillWritten += 8L * a.length;
  }

  private float[] readFloats(FileChannel ch, int n) throws IOException {
    float[] a = new float[n];
    ByteBuffer b = ByteBuffer.allocate(1 << 20).order(java.nio.ByteOrder.LITTLE_ENDIAN);
    for (int i = 0; i < n; ) {
      b.clear();
      int k = Math.min(n - i, b.capacity() / 4);
      b.limit(4 * k);
      while (b.hasRemaining()) if (ch.read(b) < 0) throw new EOFException("Fusion spill truncated");
      b.flip();
      b.asFloatBuffer().get(a, i, k);
      i += k;
    }
    spillRead += 4L * n;
    return a;
  }

  private long[] readLongs(FileChannel ch, int n) throws IOException {
    long[] a = new long[n];
    ByteBuffer b = ByteBuffer.allocate(1 << 20).order(java.nio.ByteOrder.LITTLE_ENDIAN);
    for (int i = 0; i < n; ) {
      b.clear();
      int k = Math.min(n - i, b.capacity() / 8);
      b.limit(8 * k);
      while (b.hasRemaining()) if (ch.read(b) < 0) throw new EOFException("Fusion spill truncated");
      b.flip();
      b.asLongBuffer().get(a, i, k);
      i += k;
    }
    spillRead += 8L * n;
    return a;
  }

  static final String[] TILE_KINDS = {"z", "w", "best", "used", "emitted"};

  /** Strips: camera c's memory-mapped map file of the given kind (TILE_KINDS). */
  private Path tileFile(int c, String kind) {
    return spillDirectory.resolve("camera_" + c + "." + kind);
  }

  /** A new zero map file of camera c, memory-mapped read-write. */
  private java.lang.foreign.MemorySegment tileMap(int c, String kind, long bytes) throws IOException {
    if (tileFiles == null) tileFiles = new MappedFiles();
    return tileFiles.create(tileFile(c, kind), bytes);
  }

  /** used/emitted marks of camera j, created on first use (memory-mapped in strips). */
  private Bits newBits(int j, String kind) throws IOException {
    long words = ((long) fusion.width(j) * fusion.height(j) + 63) / 64;
    return tiled ? new Bits(tileMap(j, kind, 8 * words)) : new Bits(new long[(int) words]);
  }

  private void deleteSpill() {
    if (tileFiles != null) tileFiles.close();
    tileFiles = null;
    if (spillDirectory == null || !Files.isDirectory(spillDirectory)) return;
    for (int c = 0; c < cameras; c++)
      try {
        Files.deleteIfExists(mapFile(c));
        Files.deleteIfExists(bitsFile(c));
        for (String kind : TILE_KINDS) Files.deleteIfExists(tileFile(c, kind));
      } catch (IOException ignored) {
        // Temporary files; a later run overwrites them.
      }
    try {
      Files.deleteIfExists(spillDirectory);
    } catch (IOException ignored) {
    }
  }

  private void parallel(int n, java.util.function.IntConsumer action) {
    try {
      pool.submit(() -> IntStream.range(0, n).parallel().forEach(action)).get();
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      throw new IllegalStateException("Image fusion interrupted", e);
    } catch (ExecutionException e) {
      if (e.getCause() instanceof RuntimeException r) throw r;
      if (e.getCause() instanceof Error error) throw error;
      throw new IllegalStateException(e.getCause());
    }
  }

  /** Reads pair k's candidate records (big-endian, as DenseReconstruction writes them). */
  private PairData load(FileChannel channel, int k, boolean full, Counters counters) throws IOException {
    // Pass 1 runs several cameras at once: the shared channel position, read buffer and cache are
    // used under one lock. Cached data is immutable once published.
    synchronized (pairCache) {
      return loadLocked(channel, k, full, counters);
    }
  }

  private PairData loadLocked(FileChannel channel, int k, boolean full, Counters counters) throws IOException {
    PairData cached = pairCache.get(k);
    if (cached != null) {
      counters.cacheHits++;
      return cached;
    }
    long first = start[k];
    int n = Math.toIntExact(start[k + 1] - first);
    long fullBytes = 256L + n * (36L + 4L * fields);
    boolean retain = cacheLimit > 0 && fullBytes <= cacheLimit;
    if (retain) {
      // Evict before allocation: the memory limit covers the new entry too.
      var entries = pairCache.entrySet().iterator();
      while (cachedBytes + fullBytes > cacheLimit && entries.hasNext()) {
        cachedBytes -= entries.next().getValue().bytes();
        entries.remove();
      }
      full = true;
    }
    PairData data = new PairData(k, n, fields, full);
    if (readBuffer == null) readBuffer = ByteBuffer.allocateDirect(record * 65536);
    ByteBuffer buffer = readBuffer;
    long position = first * record;
    int i = 0;
    while (i < n) {
      buffer.clear();
      buffer.limit((int) Math.min(buffer.capacity(), (long) (n - i) * record));
      while (buffer.hasRemaining()) {
        int read = channel.read(buffer, position);
        if (read < 0) throw new EOFException("Candidate stream truncated");
        position += read;
      }
      buffer.flip();
      for (; buffer.remaining() >= record; i++) {
        data.world[3 * i] = buffer.getDouble();
        data.world[3 * i + 1] = buffer.getDouble();
        data.world[3 * i + 2] = buffer.getDouble();
        int r = buffer.get() & 255, g = buffer.get() & 255, b = buffer.get() & 255;
        buffer.getInt(); // pair id: equal to k by construction
        int pixel = buffer.getInt();
        if (full) {
          data.rgb[i] = r << 16 | g << 8 | b;
          data.pixel[i] = pixel;
          for (int f = 0; f < fields; f++) data.evidence[i * fields + f] = buffer.getFloat();
        } else buffer.position(buffer.position() + 4 * fields);
      }
    }
    counters.pairReads++;
    counters.bytesRead += (long) n * record;
    sines(data);
    if (retain) {
      pairCache.put(k, data);
      cachedBytes += data.bytes();
      counters.cachePeakBytes = Math.max(counters.cachePeakBytes, cachedBytes);
    }
    return data;
  }

  /** f * sin(triangulation angle) per candidate: DepthFusion.weight without the depth term. */
  private void sines(PairData data) {
    int a = fusion.pairCamera(data.pair, false), b = fusion.pairCamera(data.pair, true);
    Camera ca = fusion.camera(a), cb = fusion.camera(b);
    double[] centerA = ca.center(), centerB = cb.center();
    double focal = Math.min(Math.min(ca.fx, ca.fy), Math.min(cb.fx, cb.fy));
    data.narrowSinef = focal * seedMinSine;
    parallel(data.count, j -> {
      double[] p = data.point(j), u = Math3.sub(p, centerA), v = Math3.sub(p, centerB);
      double denominator = Math3.norm(u) * Math3.norm(v);
      double x = u[1] * v[2] - u[2] * v[1], y = u[2] * v[0] - u[0] * v[2], z = u[0] * v[1] - u[1] * v[0];
      double sine = denominator == 0 ? 0 : Math.min(1, Math.sqrt(x * x + y * y + z * z) / denominator);
      data.sinef[j] = (float) Math.max(1e-9, focal * sine);
    });
  }

  /**
   * --dense-method mvsgm: the candidates of rows [r0, r1) of pair k's camera, read from its maps
   * (MvsgmMaps) with the arithmetic the former candidate file used (world point from the map
   * depth in FP64; evidence as floats; colour packed), in raster order. Element 0 is candidate
   * {@code first} of the pair, so candidate ids are unchanged.
   */
  private PairData mapPair(int k, int r0, int r1, boolean full, Counters counters) throws IOException {
    int c = fusion.pairCamera(k, false);
    MvsgmMaps.Camera mc = maps.camera(c);
    if (mc == null || mc.pair() != k) throw new IllegalStateException("No mvsgm maps for fusion pair " + k);
    int w = mc.width();
    long[] rowStart = mc.rowStart();
    long first = rowStart[r0];
    PairData data = new PairData(k, Math.toIntExact(rowStart[r1] - first), fields, full);
    data.first = first;
    var z = maps.segment(c, MvsgmMaps.DEPTH);
    var lr = maps.segment(c, MvsgmMaps.LR);
    var margin = full ? maps.segment(c, MvsgmMaps.MARGIN) : null;
    var sharp = full ? maps.segment(c, MvsgmMaps.SHARP) : null;
    var photo = full ? maps.segment(c, MvsgmMaps.PHOTO) : null;
    var rgb = full ? maps.segment(c, MvsgmMaps.RGB) : null;
    Camera cam = fusion.camera(c);
    double[] rt = Math3.transpose(cam.rotation);
    // Rows are copied out of the mapped files in bulk (one copy per map and row).
    ThreadLocal<float[][]> rows = ThreadLocal.withInitial(() -> new float[5][w]);
    ThreadLocal<int[]> colours = ThreadLocal.withInitial(() -> new int[w]);
    parallel(r1 - r0, row -> {
      int y = r0 + row, i = (int) (rowStart[y] - first);
      if (rowStart[y + 1] == rowStart[y]) return;
      long at = (long) y * w;
      float[][] v = rows.get();
      float[] zr = v[0], lrr = v[1], mr = v[2], sr = v[3], pr = v[4];
      int[] cr = colours.get();
      java.lang.foreign.MemorySegment.copy(z, MappedFiles.FLOAT, 4 * at, zr, 0, w);
      java.lang.foreign.MemorySegment.copy(lr, MappedFiles.FLOAT, 4 * at, lrr, 0, w);
      if (full) {
        java.lang.foreign.MemorySegment.copy(margin, MappedFiles.FLOAT, 4 * at, mr, 0, w);
        java.lang.foreign.MemorySegment.copy(sharp, MappedFiles.FLOAT, 4 * at, sr, 0, w);
        java.lang.foreign.MemorySegment.copy(photo, MappedFiles.FLOAT, 4 * at, pr, 0, w);
        java.lang.foreign.MemorySegment.copy(rgb, MappedFiles.INT, 4 * at, cr, 0, w);
      }
      for (int x = 0; x < w; x++) {
        float zp = zr[x];
        if (zp <= 0) continue;
        float l = lrr[x];
        if (Float.isNaN(l)) continue;
        double[] X = MultiViewSgm.world(cam, rt, x, y, zp);
        data.world[3 * i] = X[0];
        data.world[3 * i + 1] = X[1];
        data.world[3 * i + 2] = X[2];
        if (full) {
          float mg = mr[x], sh = sr[x], ph = pr[x];
          data.rgb[i] = cr[x];
          data.pixel[i] = (int) (at + x);
          int e = i * fields;
          data.evidence[e] = (float) PointConfidence.score(ph, mg, sh, l);
          data.evidence[e + 1] = mg;
          data.evidence[e + 2] = sh;
          data.evidence[e + 3] = ph;
          data.evidence[e + 4] = l;
          if (fields == 6) data.evidence[e + 5] = Float.NaN;
        }
        i++;
      }
    });
    counters.pairReads++;
    counters.bytesRead += (long) (r1 - r0) * w * (full ? 24 : 8);
    sines(data);
    return data;
  }

  /** DepthFusion.weight: inverse variance of a depth z from the candidate's triangulation angle. */
  private static double weight(PairData data, int i, double z) {
    double sigma = z * .5 / data.sinef[i];
    return 1 / Math.max(1e-20, sigma * sigma);
  }

  private Counters fuse(FusionOutput output, TwoViewConfidence.Diagnostics diagnostics)
      throws IOException {
    Counters counters = new Counters();
    counters.cacheLimitBytes = cacheLimit;
    counters.edgeEnabled=DepthOutlierFilter.enabled(o);
    counters.thinEnabled = o.thinSurfaceFusion();
    double[][] samples = new double[cameras][];
    long splatNanos = 0, mergeNanos = 0, speckleNanos = 0, componentNanos = 0;
    try (FileChannel channel = FileChannel.open(candidates, StandardOpenOption.READ)) {
      WorkProgress progress = WorkProgress.start("Image fusion cameras", 2L * cameras, "camera passes");
      // Pass 1: filtered merged depth per camera, plus sample points for overlap detection. A
      // camera's pass 1 reads only its own pairs and writes only its own maps, so several cameras
      // run at once (bounded by heap; see passOneCameras). Counts are added in camera order.
      long[][] times = new long[cameras][4];
      Counters[] perCamera = new Counters[cameras];
      int concurrent = Math.max(1, Math.min(passOneCameras, cameras));
      ExecutorService cameraWorkers =
          concurrent > 1
              ? Executors.newFixedThreadPool(
                  concurrent,
                  r -> {
                    Thread t = new Thread(r, "fusion-camera");
                    t.setDaemon(true);
                    return t;
                  })
              : null;
      try {
        List<Future<?>> tasks = new ArrayList<>();
        for (int c = 0; c < cameras; c++) {
          if (pairsOf.get(c).isEmpty()) continue;
          final int camera = c;
          Callable<Void> task =
              () -> {
                Counters own = new Counters();
                if (tiled) {
                  passOneTiled(channel, camera, own, times[camera], samples, counters);
                  perCamera[camera] = own;
                  return null;
                }
                int h = fusion.height(camera);
                long clock = System.nanoTime();
                Splat[] splats = splat(channel, camera, false, counters, 0, h);
                times[camera][0] = System.nanoTime() - clock;
                clock = System.nanoTime();
                Merged m = merge(camera, splats, own, 0, h, 0, h);
                times[camera][1] = System.nanoTime() - clock;
                clock = System.nanoTime();
                own.spikes += spikes(m, fusion.camera(camera), 0, h);
                long componentClock = System.nanoTime();
                own.speckle += speckles(m, fusion.camera(camera));
                times[camera][3] = System.nanoTime() - componentClock;
                if (o.fusionDepthOutliers) own.depthOutliers += depthOutliers(m.z);
                times[camera][2] = System.nanoTime() - clock;
                for (float z : m.z) if (z > 0) own.pixels++;
                samples[camera] = sample(camera, m, splats);
                // Diagnostic (-Dsgm.dumpMergedMaps=true): the filtered pass-1 merged depth map, for
                // comparisons with other depth-map methods. Writes files only; results unchanged.
                if (Boolean.getBoolean("sgm.dumpMergedMaps"))
                  DepthMapFile.write(
                      o.output.resolve("06_fusion/merged_maps/camera_" + camera + ".depth"),
                      fusion.camera(camera), fusion.width(camera), fusion.height(camera), m.z);
                admit(camera, new Raster(m.z), storeWeights ? new Raster(m.wsum) : null,
                    localSigma > 0 ? new Raster(localMinimumSigma(m.z, m.wsum, m.w, m.h, 4)) : null);
                perCamera[camera] = own;
                return null;
              };
          if (cameraWorkers == null) {
            progress.update(c);
            try {
              task.call();
            } catch (IOException | RuntimeException | Error e) {
              throw e;
            } catch (Exception e) {
              throw new IOException(e);
            }
          } else tasks.add(cameraWorkers.submit(task));
        }
        int finished = 0;
        for (Future<?> task : tasks) {
          try {
            task.get();
          } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new IOException("Image fusion interrupted", e);
          } catch (ExecutionException e) {
            if (e.getCause() instanceof IOException io) throw io;
            if (e.getCause() instanceof RuntimeException r) throw r;
            if (e.getCause() instanceof Error error) throw error;
            throw new IOException(e.getCause());
          }
          progress.update(++finished);
        }
      } finally {
        if (cameraWorkers != null) cameraWorkers.shutdownNow();
      }
      for (int c = 0; c < cameras; c++) {
        if (perCamera[c] == null) continue;
        counters.ambiguousKept += perCamera[c].ambiguousKept;
        counters.spikes += perCamera[c].spikes;
        counters.speckle += perCamera[c].speckle;
        counters.depthOutliers += perCamera[c].depthOutliers;
        counters.pixels += perCamera[c].pixels;
        splatNanos += times[c][0];
        mergeNanos += times[c][1];
        speckleNanos += times[c][2];
        componentNanos += times[c][3];
      }
      RunTelemetry.recordTotal("Image fusion: pass 1 splats (pair reads)", splatNanos, cameras);
      RunTelemetry.recordTotal("Image fusion: pass 1 per-camera merge", mergeNanos, cameras);
      // Spike filter, speckle filter and depth sanity together; then the speckle filter alone.
      RunTelemetry.recordTotal("Image fusion: pass 1 2D speckle filter", speckleNanos, cameras);
      RunTelemetry.recordTotal("Image fusion: pass 1 speckle components (within the filter time)", componentNanos, cameras);
      long clock = System.nanoTime();
      if (DepthOutlierFilter.enabled(o)) surfaceContext = new SurfacePatchGrid[cameras];
      for (int c = 0; c < cameras; c++) if (compact[c] != null) grid(c);
      for (int c = 0; c < cameras; c++) neighbours[c] = overlapping(c, samples);
      checkWorkingSet();
      RunTelemetry.record("Image fusion: camera overlap", clock);
      long remergeNanos = 0, checkNanos = 0, outputNanos = 0;
      // Pass 2: re-merge one camera at a time (identical), cross-check, own and emit, in row bands.
      // --fusion-seed-order precision repeats it per precision tier (precise seeds first).
      // Ownership marks (used/emitted) are applied after each band, before the next band or camera
      // is checked: within a camera the cross-check reads no mark it writes (it reads its own
      // camera's used marks and earlier cameras' emitted marks; it writes other cameras' used
      // marks and its own emitted marks), so bands give the same result as the whole camera.
      // Appending the emitted points to the output only has to keep their order, so one writer
      // thread does it in camera and row order, overlapping the next band's cross-check; at most
      // two bands' emissions wait for it.
      ExecutorService writer =
          Executors.newSingleThreadExecutor(
              r -> {
                Thread t = new Thread(r, "fusion-output");
                t.setDaemon(true);
                return t;
              });
      ArrayDeque<Future<Long>> pending = new ArrayDeque<>();
      try {
      for (int tier = 0; tier < tierCount; tier++)
      for (int c = 0; c < cameras; c++) {
        progress.update(cameras + c);
        if (!hasMap[c]) continue;
        residentFor(c);
        // The whole image at once, or (fusion in strips) strip by strip with a halo covering the
        // normal fits (+-12 rows) and the merge's 3x3 search. Bands and strips never change results.
        int h = fusion.height(c), strip = tiled ? stripRows : h, halo = tiled ? PASS_TWO_HALO : 0;
        for (int s0 = 0; s0 < h; s0 += strip) {
        int s1 = Math.min(h, s0 + strip), r0 = Math.max(0, s0 - halo), r1 = Math.min(h, s1 + halo);
        clock = System.nanoTime();
        Splat[] splats = splat(channel, c, true, counters, r0, r1 - r0);
        Merged m = merge(c, splats, null, r0, r1 - r0, 0, 0);
        // Pass 1's speckle decisions, without recounting.
        Raster kept = compact[c];
        long base = (long) r0 * m.w;
        for (int p = 0; p < m.z.length; p++) if (kept.get(base + p) <= 0) m.z[p] = 0;
        remergeNanos += System.nanoTime() - clock;
        double[] points = pointCache(c, m);
        int w = m.w, bandRows = Math.max(1, bandPixels / Math.max(1, w));
        for (int y0 = s0; y0 < s1; y0 += bandRows) {
          clock = System.nanoTime();
          Emission[][] rows = crossCheck(c, m, splats, counters, tier, points, y0, Math.min(s1, y0 + bandRows));
          checkNanos += System.nanoTime() - clock;
          for (Emission[] row : rows)
            if (row != null)
              for (Emission e : row) {
                if (e == null || !e.result.accepted()) continue;
                if (e.result.shift > 1e-8) counters.refined++;
                counters.emitted++;
                if (e.agreements() != null && o.fusionStrategy.equals("graph-average")) {
                  // Members were averaged into this point: exactly those pixels are used.
                  markUsed(e, 0);
                  counters.members += e.agreements().length;
                } else if (e.agreements() != null) {
                  markUsed(e, agreementRadius(o));
                  if (emitted[c] == null) emitted[c] = newBits(c, "emitted");
                  emitted[c].set(e.cameraPixel());
                }
              }
          while (pending.size() >= 2) outputNanos += awaitOutput(pending.poll());
          pending.add(
              writer.submit(
                  () -> {
                    long started = System.nanoTime();
                    for (Emission[] row : rows)
                      if (row != null)
                        for (Emission e : row) {
                          if (e == null) continue;
                          diagnostics.record(e.result);
                          if (!e.result.accepted()) continue;
                          output.add(e.candidate, e.pair, e.pixel, e.rgb, e.result, e.geometry);
                        }
                    return System.nanoTime() - started;
                  }));
        }
        }
      }
      while (!pending.isEmpty()) outputNanos += awaitOutput(pending.poll());
      } finally {
        writer.shutdownNow();
      }
      RunTelemetry.recordTotal("Image fusion: pass 2 pair reads and re-merge", remergeNanos, cameras);
      RunTelemetry.recordTotal("Image fusion: cross-check, ownership, normals", checkNanos, cameras);
      RunTelemetry.recordTotal("Image fusion: candidate output staging", outputNanos, cameras);
      progress.done();
      progress.close();
    }
    return counters;
  }

  private static long awaitOutput(Future<Long> task) throws IOException {
    try {
      return task.get();
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      throw new IOException("Image fusion output interrupted", e);
    } catch (ExecutionException e) {
      if (e.getCause() instanceof IOException io) throw io;
      if (e.getCause() instanceof RuntimeException r) throw r;
      if (e.getCause() instanceof Error error) throw error;
      throw new IOException(e.getCause());
    }
  }

  /**
   * An emitted point represents the surface in every camera that agreed with it. Those cameras'
   * merged pixels around the agreeing position (within radius, and only where their depth agrees
   * too) can no longer emit a copy: owner-consume uses the agreement radius, graph-average only the
   * averaged member pixel. Marks only affect other cameras and are unions, so results do not depend
   * on row scheduling.
   */
  private void markUsed(Emission e, int radius) throws IOException {
    double tol = o.fusionRelative;
    for (int k = 0; k < e.agreements().length; k++) {
      long a = e.agreements()[k];
      int j = (int) (a >>> 42), y = (int) (a >>> 21 & 0x1FFFFF), x = (int) (a & 0x1FFFFF);
      double d = e.agreementDepths()[k];
      Raster z = compact[j];
      int w = fusion.width(j), h = fusion.height(j);
      if (used[j] == null) used[j] = newBits(j, "used");
      Bits marks = used[j];
      for (int dy = -radius; dy <= radius; dy++)
        for (int dx = -radius; dx <= radius; dx++) {
          int xx = x + dx, yy = y + dy;
          if (xx < 0 || yy < 0 || xx >= w || yy >= h) continue;
          int q = yy * w + xx;
          float zq = z.get(q);
          if (zq > 0 && Math.abs(zq - d) <= tol * Math.min(zq, d)) marks.set(q);
        }
    }
  }

  /**
   * Splats each of camera c's pairs into c's rows [r0, r0 + rows) (the whole image unless fusing
   * in strips; splat arrays are indexed by local pixel). mvsgm pairs read only the rows that can
   * land there (one more on each side).
   */
  private Splat[] splat(FileChannel channel, int c, boolean full, Counters counters, int r0, int rows) throws IOException {
    List<Integer> pairs = pairsOf.get(c);
    Splat[] out = new Splat[pairs.size()];
    Camera cam = fusion.camera(c);
    int w = fusion.width(c), h = rows, H = fusion.height(c);
    PairData[] loaded = new PairData[out.length];
    for (int s = 0; s < out.length; s++)
      loaded[s] = maps != null
          ? mapPair(pairs.get(s), Math.max(0, r0 - 1), Math.min(H, r0 + rows + 1), full, counters)
          : load(channel, pairs.get(s), full, counters);
    // Each pair's splat is independent (its own arrays, filled serially in candidate order), so
    // the pairs are splatted concurrently with identical results.
    parallel(out.length, s -> {
      int k = pairs.get(s);
      PairData data = loaded[s];
      int other = fusion.pairCamera(k, false) == c ? fusion.pairCamera(k, true) : fusion.pairCamera(k, false);
      int[] sample = new int[w * h];
      float[] z = new float[w * h], du = new float[w * h], dv = new float[w * h];
      Arrays.fill(sample, -1);
      double[] v = new double[3];
      // Serial, in candidate order: the nearest depth wins and equal depths keep the first.
      for (int i = 0; i < data.count; i++) {
        cam.viewInto(data.world[3 * i], data.world[3 * i + 1], data.world[3 * i + 2], v);
        if (v[2] <= 0) continue;
        double u = cam.fx * v[0] / v[2] + cam.cx, t = cam.fy * v[1] / v[2] + cam.cy;
        int x = (int) Math.round(u), y = (int) Math.round(t);
        if (x < 0 || x >= w || y < r0 || y >= r0 + h) continue;
        int at = (y - r0) * w + x;
        if (sample[at] < 0 || v[2] < z[at]) {
          sample[at] = i;
          z[at] = (float) v[2];
          du[at] = (float) (u - x);
          dv[at] = (float) (t - y);
        }
      }
      out[s] = new Splat(data, other, sample, z, du, dv);
    });
    return out;
  }

  /**
   * Reference depth-mode merge per pixel; ambiguous pixels keep the nearer cluster, flagged. The
   * splats cover rows [r0, r0 + rows); ambiguous pixels are counted in local rows [countFrom, countTo).
   */
  private Merged merge(int c, Splat[] splats, Counters counters, int r0, int rows, int countFrom, int countTo) {
    int w = fusion.width(c), h = rows;
    Merged m = new Merged(w, h, words);
    m.row0 = r0;
    double limit = o.fusionPixels * o.fusionPixels;
    double focal = Math.min(fusion.camera(c).fx, fusion.camera(c).fy);
    long[] ambiguous = new long[h];
    parallel(h, y -> {
      int n = splats.length;
      int[] samples = new int[n], from = new int[n];
      double[] depth = new double[n], weight = new double[n], offsetX = new double[n], offsetY = new double[n];
      for (int x = 0; x < w; x++) {
        int p = y * w + x, found = 0;
        for (int s = 0; s < n; s++) {
          Splat splat = splats[s];
          int i = splat.sample()[p], at = p;
          if (i < 0) {
            // Sparse candidates (step > 1): nearest 3x3 sample within the reprojection tolerance.
            double best = limit;
            for (int dy = -1; dy <= 1; dy++)
              for (int dx = -1; dx <= 1; dx++) {
                int xx = x + dx, yy = y + dy;
                if ((dx | dy) == 0 || xx < 0 || yy < 0 || xx >= w || yy >= h) continue;
                int q = yy * w + xx, j = splat.sample()[q];
                if (j < 0) continue;
                double u = dx + splat.du()[q], t = dy + splat.dv()[q];
                if (u * u + t * t <= best) {
                  best = u * u + t * t;
                  i = j;
                  at = q;
                }
              }
          }
          if (i < 0) continue;
          samples[found] = i;
          from[found] = s;
          depth[found] = splat.z()[at];
          weight[found] = weight(splat.data(), i, depth[found]);
          offsetX[found] = at % w - x + splat.du()[at];
          offsetY[found] = at / w - y + splat.dv()[at];
          found++;
        }
        if (found == 0) continue;
        // Largest tolerance cluster (ties: larger weight), as DepthFusion.mode.
        int bestCenter = -1, bestCount = 0;
        double bestWeight = 0;
        for (int a = 0; a < found; a++) {
          int members = 0;
          double weights = 0;
          for (int b = 0; b < found; b++)
            if (agree(depth[b], depth[a], weight[b], weight[a], focal)) {
              members++;
              weights += weight[b];
            }
          if (members > bestCount || members == bestCount && weights > bestWeight) {
            bestCenter = a;
            bestCount = members;
            bestWeight = weights;
          }
        }
        boolean majority = bestCount >= Math.ceil(found * o.fusionConsensus);
        if (!majority) {
          // Two credible layers: keep the nearer cluster provisionally; cross-checks decide.
          int nearest = 0;
          for (int a = 1; a < found; a++) if (depth[a] < depth[nearest]) nearest = a;
          bestCenter = nearest;
          ambiguous[y]++;
        }
        double center = depth[bestCenter], sum = 0, wsum = 0, top = -1;
        int chosen = -1, members = 0;
        boolean allNarrow = seedMinSine > 0;
        for (int b = 0; b < found; b++) {
          if (!agree(depth[b], center, weight[b], weight[bestCenter], focal)) continue;
          if (allNarrow) {
            PairData member = splats[from[b]].data();
            allNarrow = member.sinef[samples[b]] < member.narrowSinef;
          }
          sum += weight[b] * depth[b];
          wsum += weight[b];
          members++;
          int other = splats[from[b]].other();
          if (m.partners.length > 0) m.partners[p * words + (other >>> 6)] |= 1L << (other & 63);
          // Heaviest member represents the pixel; ties: lower pair id, then lower candidate index.
          int pair = splats[from[b]].data().pair;
          if (weight[b] > top
              || weight[b] == top && (pair < m.repPair[p] || pair == m.repPair[p] && samples[b] < m.repIndex[p])) {
            top = weight[b];
            chosen = b;
            m.repPair[p] = pair;
            m.repIndex[p] = samples[b];
          }
        }
        m.z[p] = (float) (sum / wsum);
        m.wsum[p] = (float) wsum;
        m.support[p] = (byte) Math.min(127, members);
        m.ambiguous[p] = !majority;
        m.narrow[p] = allNarrow;
        m.du[p] = (float) offsetX[chosen];
        m.dv[p] = (float) offsetY[chosen];
      }
    });
    if (counters != null) for (int y = countFrom; y < countTo; y++) counters.ambiguousKept += ambiguous[y];
    return m;
  }

  /** Up to 4096 back-projected merged points, evenly spread, for camera overlap detection. */
  private double[] sample(int c, Merged m, Splat[] splats) {
    int valid = 0;
    for (float z : m.z) if (z > 0) valid++;
    if (valid == 0) return new double[0];
    int stride = Math.max(1, valid / 4096), seen = 0, n = 0;
    double[] out = new double[3 * Math.min(valid, 4096 + 1)];
    Camera cam = fusion.camera(c);
    double[] rotationT = Math3.transpose(cam.rotation);
    for (int p = 0; p < m.z.length && n < out.length / 3; p++) {
      if (m.z[p] <= 0 || seen++ % stride != 0) continue;
      double[] X = backProject(cam, rotationT, p % m.w + m.du[p], p / m.w + m.dv[p], m.z[p]);
      out[3 * n] = X[0];
      out[3 * n + 1] = X[1];
      out[3 * n + 2] = X[2];
      n++;
    }
    return Arrays.copyOf(out, 3 * n);
  }

  /** Cameras that see at least 1% of c's sampled surface (in front, inside the image). */
  private int[] overlapping(int c, double[][] samples) {
    if (samples[c] == null || samples[c].length == 0) return new int[0];
    int n = samples[c].length / 3;
    List<Integer> out = new ArrayList<>();
    for (int j = 0; j < cameras; j++) {
      if (j == c || !hasMap[j]) continue;
      Camera other = fusion.camera(j);
      int inside = 0;
      for (int i = 0; i < n; i++) {
        double[] v = other.view(new double[] {samples[c][3 * i], samples[c][3 * i + 1], samples[c][3 * i + 2]});
        if (v[2] <= 0) continue;
        double u = other.fx * v[0] / v[2] + other.cx, t = other.fy * v[1] / v[2] + other.cy;
        if (u >= 0 && t >= 0 && u < fusion.width(j) && t < fusion.height(j)) inside++;
      }
      if (inside * 100L >= n) out.add(j);
    }
    return out.stream().mapToInt(Integer::intValue).toArray();
  }

  private static double[] backProject(Camera cam, double[] rotationT, double u, double v, double z) {
    double[] local = {(u - cam.cx) / cam.fx * z, (v - cam.cy) / cam.fy * z, z};
    return Math3.mv(rotationT, Math3.sub(local, cam.translation));
  }

  /**
   * Per-camera depth sanity (T2): removes merged depths beyond 1.5x the camera's 99.5th percentile
   * or below 0.5x its 0.5th percentile. Such points lie far below the ground or far above the
   * scene (forest_10: about 290 m against 90–120 m), are few, and would otherwise also confirm
   * nothing but each other. Percentiles come from up to 1M evenly sampled valid depths.
   */
  private static long depthOutliers(float[] z) {
    int valid = 0;
    for (float v : z) if (v > 0) valid++;
    if (valid < 100) return 0;
    int stride = Math.max(1, valid / 1_000_000), n = 0, seen = 0;
    float[] sample = new float[valid / stride + 1];
    for (float v : z) if (v > 0 && seen++ % stride == 0) sample[n++] = v;
    Arrays.sort(sample, 0, n);
    double low = .5 * sample[(int) (.005 * (n - 1))], high = 1.5 * sample[(int) Math.ceil(.995 * (n - 1))];
    long removed = 0;
    for (int p = 0; p < z.length; p++)
      if (z[p] > 0 && (z[p] > high || z[p] < low)) {
        z[p] = 0;
        removed++;
      }
    return removed;
  }

  // ---------------------------------------------------------------------------------------------
  // Fusion in strips (STREAMED_PROCESSING.md). Every step below gives the untiled result: the
  // halos cover each neighbourhood operation, the speckle filter's components are joined across
  // strip borders, and the global steps (depth percentiles, overlap samples) read the whole
  // memory-mapped map in the same order as the untiled code.

  /**
   * Pass 1 of camera c in strips: merge and spike filter per strip (halo PASS_ONE_HALO), then the
   * speckle filter, depth outliers, overlap samples and local sigma over the camera's
   * memory-mapped map files.
   */
  private void passOneTiled(FileChannel channel, int c, Counters own, long[] times, double[][] samples, Counters counters)
      throws IOException {
    int w = fusion.width(c), h = fusion.height(c);
    long n = (long) w * h;
    Camera cam = fusion.camera(c);
    Raster z = new Raster(tileMap(c, "z", 4 * n));
    Raster weight = storeWeights || localSigma > 0 ? new Raster(tileMap(c, "w", 4 * n)) : null;
    for (int y0 = 0; y0 < h; y0 += stripRows) {
      int y1 = Math.min(h, y0 + stripRows), r0 = Math.max(0, y0 - PASS_ONE_HALO), r1 = Math.min(h, y1 + PASS_ONE_HALO);
      long clock = System.nanoTime();
      Splat[] splats = splat(channel, c, false, counters, r0, r1 - r0);
      times[0] += System.nanoTime() - clock;
      clock = System.nanoTime();
      Merged m = merge(c, splats, own, r0, r1 - r0, y0 - r0, y1 - r0);
      times[1] += System.nanoTime() - clock;
      clock = System.nanoTime();
      own.spikes += spikes(m, cam, y0 - r0, y1 - r0);
      int at = (y0 - r0) * w, count = (y1 - y0) * w;
      z.write((long) y0 * w, m.z, at, count);
      if (weight != null) weight.write((long) y0 * w, m.wsum, at, count);
      times[2] += System.nanoTime() - clock;
    }
    long clock = System.nanoTime();
    own.speckle += stripSpeckles(z, w, h, cam);
    times[3] += System.nanoTime() - clock;
    if (o.fusionDepthOutliers) own.depthOutliers += depthOutliers(z, n);
    times[2] += System.nanoTime() - clock;
    long valid = 0;
    for (long p = 0; p < n; p++) if (z.get(p) > 0) valid++;
    own.pixels += valid;
    clock = System.nanoTime();
    samples[c] = sampleTiled(channel, c, z, valid, counters);
    times[0] += System.nanoTime() - clock;
    if (Boolean.getBoolean("sgm.dumpMergedMaps")) {
      Path file = o.output.resolve("06_fusion/merged_maps/camera_" + c + ".depth");
      DepthMapFile.create(file, cam, w, h);
      float[] rows = new float[stripRows * w];
      for (int y0 = 0; y0 < h; y0 += stripRows) {
        int nr = Math.min(stripRows, h - y0);
        z.read((long) y0 * w, rows, 0, nr * w);
        DepthMapFile.writeRegion(file, w, 0, y0, w, nr, rows, w, 0, 0);
      }
    }
    admit(c, z, storeWeights ? weight : null, localSigma > 0 ? localBestTiled(c, z, weight) : null);
  }

  /** depthOutliers over a memory-mapped map of length pixels (same samples and decisions). */
  private static long depthOutliers(Raster z, long length) {
    long count = 0;
    for (long p = 0; p < length; p++) if (z.get(p) > 0) count++;
    if (count < 100) return 0;
    int valid = Math.toIntExact(count);
    int stride = Math.max(1, valid / 1_000_000), n = 0, seen = 0;
    float[] sample = new float[valid / stride + 1];
    for (long p = 0; p < length; p++) {
      float v = z.get(p);
      if (v > 0 && seen++ % stride == 0) sample[n++] = v;
    }
    Arrays.sort(sample, 0, n);
    double low = .5 * sample[(int) (.005 * (n - 1))], high = 1.5 * sample[(int) Math.ceil(.995 * (n - 1))];
    long removed = 0;
    for (long p = 0; p < length; p++) {
      float v = z.get(p);
      if (v > 0 && (v > high || v < low)) {
        z.set(p, 0);
        removed++;
      }
    }
    return removed;
  }

  /**
   * sample() in strips: the same evenly spread valid pixels, back-projected with the subpixel
   * offsets of a re-merge of each strip (halo 1 for the merge's 3x3 search).
   */
  private double[] sampleTiled(FileChannel channel, int c, Raster z, long count, Counters counters) throws IOException {
    if (count == 0) return new double[0];
    int w = fusion.width(c), h = fusion.height(c), valid = Math.toIntExact(count);
    int stride = Math.max(1, valid / 4096), seen = 0, n = 0;
    double[] out = new double[3 * Math.min(valid, 4096 + 1)];
    Camera cam = fusion.camera(c);
    double[] rotationT = Math3.transpose(cam.rotation);
    for (int y0 = 0; y0 < h && n < out.length / 3; y0 += stripRows) {
      int y1 = Math.min(h, y0 + stripRows), r0 = Math.max(0, y0 - 1), r1 = Math.min(h, y1 + 1);
      Merged m = merge(c, splat(channel, c, false, counters, r0, r1 - r0), null, r0, r1 - r0, 0, 0);
      for (int y = y0; y < y1 && n < out.length / 3; y++)
        for (int x = 0; x < w && n < out.length / 3; x++) {
          float zp = z.get((long) y * w + x);
          if (zp <= 0 || seen++ % stride != 0) continue;
          int q = (y - r0) * w + x;
          double[] X = backProject(cam, rotationT, x + m.du[q], y + m.dv[q], zp);
          out[3 * n] = X[0];
          out[3 * n + 1] = X[1];
          out[3 * n + 2] = X[2];
          n++;
        }
    }
    return Arrays.copyOf(out, 3 * n);
  }

  /** localMinimumSigma in strips (halo SIGMA_HALO rows), written to camera c's "best" map file. */
  private Raster localBestTiled(int c, Raster z, Raster weight) throws IOException {
    int w = fusion.width(c), h = fusion.height(c);
    Raster best = new Raster(tileMap(c, "best", 4L * w * h));
    for (int y0 = 0; y0 < h; y0 += stripRows) {
      int y1 = Math.min(h, y0 + stripRows), r0 = Math.max(0, y0 - SIGMA_HALO), r1 = Math.min(h, y1 + SIGMA_HALO);
      int cells = (r1 - r0) * w;
      float[] zs = new float[cells], ws = new float[cells];
      z.read((long) r0 * w, zs, 0, cells);
      weight.read((long) r0 * w, ws, 0, cells);
      best.write((long) y0 * w, localMinimumSigma(zs, ws, w, r1 - r0, 4), (y0 - r0) * w, (y1 - y0) * w);
    }
    return best;
  }

  /**
   * The 2D speckle filter over a memory-mapped map (fusion in strips): the same SpeckleFilter as
   * speckles(), on as many threads as the plan's speckle budget allows. The result does not
   * depend on the strips or threads, so it equals the untiled filter.
   */
  private long stripSpeckles(Raster z, int w, int h, Camera camera) {
    if (o.speckleSize <= 0) return 0;
    double limit = Math.max(o.fusionRelative, 2 / Math.min(camera.fx, camera.fy));
    int threads = SpeckleFilter.threadsFor(w, h, Math.max(1, o.cores), speckleBudget);
    return SpeckleFilter.filter(z, w, h, limit, o.speckleSize, SpeckleFilter.rows(h, threads), threads, pool);
  }

  /**
   * Impulse ("salt and pepper") filter on a merged depth map: a pixel whose depth stands out from
   * a coherent 5 x 5 neighbourhood is a spike, not a surface. Coherent: at least 12 of the 24
   * neighbours valid and their robust spread (1.4826 MAD) at most one footprint (depth / focal,
   * the pixel's size on the surface). Spike: further from the neighbours' median than max(4
   * spreads, 2 footprints). On smooth surfaces this removes isolated depths a few decimetres off
   * (a single nearer depth winning an ambiguous pixel, then "confirmed" by the surface itself
   * within the relative fusion tolerance); in vegetation and at depth edges the neighbourhood is
   * not coherent and nothing is removed. Decided on the unfiltered map, so the result does not
   * depend on order or threads.
   */

  /**
   * The k-th smallest of a[0..n) (0-based), as Arrays.sort(a, 0, n) would leave at a[k];
   * reorders a[0..n) so that a[0..k) <= a[k] <= a[k+1..n). For the finite, non-negative values
   * used here the numeric order equals Arrays.sort's order.
   */
  static float select(float[] a, int n, int k) {
    int lo = 0, hi = n - 1;
    while (hi > lo) {
      float pivot = a[(lo + hi) >>> 1];
      int i = lo, j = hi;
      while (i <= j) {
        while (a[i] < pivot) i++;
        while (a[j] > pivot) j--;
        if (i <= j) {
          float t = a[i];
          a[i++] = a[j];
          a[j--] = t;
        }
      }
      if (k <= j) hi = j;
      else if (k >= i) lo = i;
      else break;
    }
    return a[k];
  }

  /** The largest of a[0..k) (after select(a, n, k): the (k-1)-th smallest). */
  static float maxBelow(float[] a, int k) {
    float m = a[0];
    for (int i = 1; i < k; i++) m = Math.max(m, a[i]);
    return m;
  }

  /** Double version of {@link #select(float[], int, int)}. */
  static double select(double[] a, int n, int k) {
    int lo = 0, hi = n - 1;
    while (hi > lo) {
      double pivot = a[(lo + hi) >>> 1];
      int i = lo, j = hi;
      while (i <= j) {
        while (a[i] < pivot) i++;
        while (a[j] > pivot) j--;
        if (i <= j) {
          double t = a[i];
          a[i++] = a[j];
          a[j--] = t;
        }
      }
      if (k <= j) hi = j;
      else if (k >= i) lo = i;
      else break;
    }
    return a[k];
  }

  /** Spike filter decisions for local rows [from, to) of m (all rows of m are read). */
  private long spikes(Merged m, Camera camera, int from, int to) {
    if (!o.fusionSpikeFilter) return 0;
    int w = m.w, h = m.h, r = 2;
    double focal = Math.min(camera.fx, camera.fy);
    float[] z = m.z;
    boolean[] spike = new boolean[w * h];
    long[] counts = new long[h];
    parallel(to - from, row -> {
      int y = from + row;
      float[] values = new float[(2 * r + 1) * (2 * r + 1)], deviations = new float[values.length];
      for (int x = 0; x < w; x++) {
        int p = y * w + x;
        float centre = z[p];
        if (centre <= 0) continue;
        int n = 0;
        for (int dy = -r; dy <= r; dy++)
          for (int dx = -r; dx <= r; dx++) {
            int xx = x + dx, yy = y + dy;
            if ((dx | dy) == 0 || xx < 0 || yy < 0 || xx >= w || yy >= h) continue;
            float v = z[yy * w + xx];
            if (v > 0) values[n++] = v;
          }
        if (n < 12) continue;
        // The median by selection: the same order statistic(s) as after Arrays.sort; the later
        // loops over values do not depend on their order.
        float upper = select(values, n, n / 2);
        float median = n % 2 == 1 ? upper : .5f * (maxBelow(values, n / 2) + upper);
        // limit >= 2 footprints: within that the pixel is kept whatever the spread is (identical
        // decision without the second sort).
        if (Math.abs(centre - median) <= 2 * (median / focal)) continue;
        for (int i = 0; i < n; i++) deviations[i] = Math.abs(values[i] - median);
        Arrays.sort(deviations, 0, n);
        double spread = 1.4826 * (n % 2 == 1 ? deviations[n / 2] : .5 * (deviations[n / 2 - 1] + deviations[n / 2]));
        double footprint = median / focal, limit = Math.max(4 * spread, 2 * footprint);
        if (Math.abs(centre - median) <= limit) continue;
        // A thin structure (pole, wire, branch) is a line of pixels: two neighbours at its depth
        // keep it. A spike stands alone.
        int own = 0;
        double near = Math.max(spread, footprint);
        for (int i = 0; i < n && own < 2; i++) if (Math.abs(values[i] - centre) <= near) own++;
        if (own >= 2) continue;
        spike[p] = true;
        counts[y]++;
      }
    });
    long removed = 0;
    for (int p = 0; p < z.length; p++) if (spike[p]) z[p] = 0;
    for (long c : counts) removed += c;
    return removed;
  }

  /**
   * Removes 4-connected components smaller than --speckle-size from the merged depth map; returns
   * removed pixels. Neighbours connect within the relative fusion tolerance or two pixel
   * footprints (z / f), so slanted surfaces at coarse resolution are not fragmented.
   * SpeckleFilter: row strips labelled in parallel and joined at the strip borders, with
   * per-thread strip buffers only.
   */
  private long speckles(Merged m, Camera camera) {
    if (o.speckleSize <= 0) return 0;
    double limit = Math.max(o.fusionRelative, 2 / Math.min(camera.fx, camera.fy));
    // Cameras running pass 1 at once share the pool's threads.
    int threads = Math.max(1, Math.max(1, o.cores) / Math.max(1, Math.min(passOneCameras, cameras)));
    return SpeckleFilter.filter(
        new Raster(m.z), m.w, m.h, limit, o.speckleSize, SpeckleFilter.rows(m.h, threads), threads, pool);
  }

  private record Emission(
      long candidate,
      int pair,
      int pixel,
      int rgb,
      DepthFusion.Result result,
      DepthFusion.SurfaceSample geometry,
      int cameraPixel,
      long[] agreements,
      float[] agreementDepths) {}

  /**
   * Normal estimation back-projects up to 147 neighbours per emitted pixel; neighbouring pixels
   * share them, so each merged pixel is back-projected once per camera (same values; m is
   * read-only during the cross-check). Null (direct computation) above 512 MB, when the memory
   * plan has no room for it, or with -Dsgm.noNormalCache=true.
   */
  private double[] pointCache(int c, Merged m) {
    int w = m.w, h = m.h;
    boolean surface = o.pointAttributes.equals("surface") || o.surfaceConsolidation.equals("normal");
    if (!surface || !normalCache || 24L * w * h > 512L << 20 || Boolean.getBoolean("sgm.noNormalCache")) return null;
    Camera cam = fusion.camera(c);
    double[] rotationT = Math3.transpose(cam.rotation);
    double[] points = new double[3 * w * h];
    parallel(h, y -> {
      for (int x = 0; x < w; x++) {
        int at = y * w + x;
        if (m.z[at] <= 0) continue;
        double[] q = backProject(cam, rotationT, x + m.du[at], (y + m.row0) + m.dv[at], m.z[at]);
        System.arraycopy(q, 0, points, 3 * at, 3);
      }
    });
    return points;
  }

  /** Cross-check, ownership and emission for rows [y0, y1) of camera c, row-parallel and row-ordered. */
  private Emission[][] crossCheck(int c, Merged m, Splat[] splats, Counters counters, int tier, double[] points, int y0, int y1) {
    int w = m.w, h = m.h, required = fusion.requiredViews();
    // p: image pixel (marks, local sigma, emission); q: pixel in m (rows from m.row0).
    int base = m.row0 * w;
    Camera cam = fusion.camera(c);
    double[] rotationT = Math3.transpose(cam.rotation);
    double tol = o.fusionRelative;
    boolean robust = o.fusionOutliers.equals("robust");
    boolean surface = o.pointAttributes.equals("surface") || o.surfaceConsolidation.equals("normal");
    double twoView = fusion.twoViewConfidenceThreshold();
    int[] near = neighbours[c];
    int agreementRadius = agreementRadius(o);
    boolean consume = o.fusionStrategy.equals("owner-consume"), freeSpace = !o.fusionFreeSpace.equals("off"), strictFreeSpace = o.fusionFreeSpace.equals("strict");
    boolean graph = o.fusionStrategy.equals("graph-average");
    // Inverse-variance averaging along the ray: graph-average, or --fusion-precision-check (snap a
    // confirmed point onto the precise depths that confirmed it).
    boolean average = graph || precisionCheck;
    double focal = Math.min(cam.fx, cam.fy);
    Raster best = localBest[c];
    Bits taken = consume || graph ? used[c] : null;
    Map<Integer, PairData> byPair = new HashMap<>();
    for (Splat s : splats) byPair.put(s.data().pair, s.data());
    Emission[][] rows = new Emission[y1 - y0][];
    long[][] tallies = new long[y1 - y0][20];
    parallel(y1 - y0, rowInBand -> {
      int y = y0 + rowInBand;
      List<Emission> list = null;
      long[] t = tallies[rowInBand];
      long[] views = new long[words], confirmed = new long[words];
      long[] agreeAt = new long[near.length];
      float[] agreeDepth = new float[near.length];
      double[] vj = new double[3];
      for (int x = 0; x < w; x++) {
        int p = y * w + x, q = p - base;
        float mz = m.z[q];
        if (mz <= 0) continue;
        if (tierCount > 1 && tier(m, q, focal) != tier) continue;
        if (taken != null && taken.get(p)) {
          t[6]++;
          continue;
        }
        int agreed = 0;
        boolean duplicate = false;
        PairData data = byPair.get(m.repPair[q]);
        int rep = m.repIndex[q];
        // FP64 back-projection along the representative's ray at the merged depth.
        double u = x + m.du[q], v = y + m.dv[q];
        double[] X = backProject(cam, rotationT, u, v, mz);
        // graph-average: inverse-variance sums of depths along this pixel's ray. dX/dz is the ray
        // direction per unit of this camera's depth; another camera's depth is linear along it.
        double sumW = average ? m.wsum[q] : 0, sumWZ = average ? m.wsum[q] * (double) mz : 0;
        double[] ray = average ? Math3.mv(rotationT, new double[] {(u - cam.cx) / cam.fx, (v - cam.cy) / cam.fy, 1}) : null;
        Arrays.fill(confirmed, 0);
        for (int k = 0; k < words; k++) views[k] = m.partners.length > 0 ? m.partners[q * words + k] : 0;
        views[c >>> 6] |= 1L << (c & 63);
        long independent = m.support[q];
        int contradictions = 0, closeViews = 0, seeThrough = 0;
        int edgeSupport=0,edgeThrough=0;
        var ownPatch=surfaceContext==null?SurfacePatchGrid.Prediction.UNKNOWN:surfaceContext[c].predict(u,v,mz);
        boolean patchSuspect=ownPatch.kind()==2||ownPatch.kind()==3;
        boolean outsideSupport = false;
        double seedWeight = m.wsum[q];
        int preciseConfirm = 0, preciseDisagree = 0, outsideConfirm = 0;
        boolean narrowSeed = seedMinSine > 0 && m.narrow[q];
        for (int j : near) {
          // Neighbour maps in locals: an array (resident) or a mapped file (strips); see at().
          float[] oa = compact[j].a, wa = compactWeight[j] == null ? null : compactWeight[j].a;
          java.lang.foreign.MemorySegment os = compact[j].s, ws = compactWeight[j] == null ? null : compactWeight[j].s;
          boolean hasWeights = compactWeight[j] != null;
          Camera oc = fusion.camera(j);
          int ow = fusion.width(j), oh = fusion.height(j);
          oc.viewInto(X[0], X[1], X[2], vj);
          if (vj[2] <= 0) continue;
          double uj = oc.fx * vj[0] / vj[2] + oc.cx, wj = oc.fy * vj[1] / vj[2] + oc.cy;
          int xj = (int) Math.round(uj), yj = (int) Math.round(wj);
          if (xj < 1 || yj < 1 || xj >= ow - 1 || yj >= oh - 1) continue;
          double zj = at(oa, os, yj * ow + xj);
          boolean preciseAgreement=false;
          if(DepthOutlierFilter.enabled(o)) {
            // The merged map is one vote per camera. Source-pair cameras cannot independently
            // confirm their own mismatch. Uncertainty comes from the generating pair and the
            // other camera's merged inverse-variance weight, where available.
            int pair=data.pair;
            {
              double low=Double.POSITIVE_INFINITY,high=0;int count=0;
              for(int dy=-1;dy<=1;dy++)for(int dx=-1;dx<=1;dx++) {
                double z=at(oa, os, (yj+dy)*ow+xj+dx);
                if(z>0){low=Math.min(low,z);high=Math.max(high,z);count++;}
              }
              double footprint=vj[2]/Math.min(oc.fx,oc.fy);
              double sourceSigma=vj[2]/Math.sqrt(Math.max(1e-30,m.wsum[q]))/mz;
              double weight=hasWeights?at(wa,ws,yj*ow+xj):0;
              double otherSigma=weight>0?1/Math.sqrt(weight):footprint;
              int vote=DepthOutlierFilter.cameraVote(vj[2],zj,low,high,count,
                  sourceSigma,otherSigma,footprint,o.fusionRelative);
              preciseAgreement=vote==1;
              if(j!=fusion.pairCamera(pair,false) && j!=fusion.pairCamera(pair,true)) {
                if(vote==0) {
                  var patch=surfaceContext[j].predict(uj,wj,vj[2]);
                  if(patch.kind()==2) {
                    patchSuspect=true;
                    double band=3*Math.hypot(sourceSigma,Math.hypot(patch.sigma(),footprint));
                    if(vj[2]<patch.depth()-band){vote=2;t[13]++;}
                  }
                }
                if(vote==1)edgeSupport++;else if(vote==2)edgeThrough++;
              }
            }
          }
          // Merged maps are sparse at step > 1 and broken at canopy gaps, so the exact pixel may
          // be empty or an occluder while the same surface lies one sample away. Agreement is
          // therefore accepted within a small neighbourhood (centre first); contradiction still
          // uses the centre pixel only. Without this, two copies of one surface that miss each
          // other's samples are both "unconfirmed", both own themselves and both emit.
          // --fusion-tolerance sigma: the seed's sigma moved to j's depth scale.

          double focalJ = Math.min(oc.fx, oc.fy);
          double seedJ = seedWeight * ((double) mz / vj[2]) * ((double) mz / vj[2]);
          boolean agrees = zj > 0 && agree(zj, vj[2], hasWeights ? at(wa, ws, yj * ow + xj) : 0, seedJ, focalJ);
          int ax = xj, ay = yj;
          for (int dy = -agreementRadius; !agrees && dy <= agreementRadius; dy++)
            for (int dx = -agreementRadius; !agrees && dx <= agreementRadius; dx++) {
              int xx = xj + dx, yy = yj + dy;
              if ((dx | dy) == 0 || xx < 0 || yy < 0 || xx >= ow || yy >= oh) continue;
              double zn = at(oa, os, yy * ow + xx);
              agrees = zn > 0 && agree(zn, vj[2], hasWeights ? at(wa, ws, yy * ow + xx) : 0, seedJ, focalJ);
              if (agrees) {
                ax = xx;
                ay = yy;
              }
            }
          if (precisionCheck && zj > 0 && hasWeights && at(wa, ws, yj * ow + xj) > 4 * seedJ) {
            // Camera j is at least 2x more precise here. Confirmation at its precision, or a near
            // miss: the same surface within the relative band but outside K sigma (occluders and
            // other layers lie beyond the band and stay neutral).
            boolean tight = false;
            for (int dy = -agreementRadius; !tight && dy <= agreementRadius; dy++)
              for (int dx = -agreementRadius; !tight && dx <= agreementRadius; dx++) {
                int xx = xj + dx, yy = yj + dy;
                if (xx < 0 || yy < 0 || xx >= ow || yy >= oh) continue;
                double zn = at(oa, os, yy * ow + xx), wn = at(wa, ws, yy * ow + xx);
                tight = zn > 0 && wn > 0 && agreeSigma(zn, vj[2], wn, seedJ, focalJ);
              }
            if (tight) preciseConfirm++;
            else if (Math.abs(zj - vj[2]) <= tol * Math.min(zj, vj[2]) && reliable(oa, os, ow, xj, yj)) preciseDisagree++;
          }
          if (agrees && (m.partners.length == 0 || (m.partners[q * words + (j >>> 6)] & 1L << (j & 63)) == 0))
            outsideConfirm++;
          if(agrees && DepthOutlierFilter.enabled(o) && !preciseAgreement)t[14]++;
          // Sigma agreement is already precise; the coherent-patch vote is only needed for the
          // relative band.
          boolean averageAgreement=agrees&&(!DepthOutlierFilter.enabled(o)||preciseAgreement||sigmaTolerance);
          if(averageAgreement&&DepthOutlierFilter.enabled(o)&&preciseAgreement){ax=xj;ay=yj;}
          if (averageAgreement && consume) {
            agreeAt[agreed] = (long) j << 42 | (long) ay << 21 | ax;
            agreeDepth[agreed++] = (float) vj[2];
            // An earlier camera that agrees already emitted this surface here: this is a copy.
            if (j < c && !duplicate && emitted[j] != null)
              duplicate = emittedNear(j, xj, yj, vj[2], Math.max(agreementRadius, o.step));
          }
          if (averageAgreement && average) {
            // The member's depth along j's ray at (ax, ay), moved to this pixel's ray: depth in j
            // changes by slope per unit of this camera's depth. Grazing members are not averaged.
            double slope = oc.rotation[6] * ray[0] + oc.rotation[7] * ray[1] + oc.rotation[8] * ray[2];
            double zm = at(oa, os, ay * ow + ax), wm = at(wa, ws, ay * ow + ax);
            if (slope > .2 && wm > 0) {
              sumW += wm;
              sumWZ += wm * (mz + (zm - vj[2]) / slope);
              if (graph) {
                agreeAt[agreed] = (long) j << 42 | (long) ay << 21 | ax;
                agreeDepth[agreed++] = (float) vj[2];
              }
            }
          }
          if (freeSpace && (zj > 0 || agrees)) {
            // Free-space vote. Margin: three footprints (beyond depth noise, far below the 3 x
            // relative tolerance of the contradiction test). "on": cameras confirming under the
            // relative agreement above count for the point. "strict": confirmation at the same
            // image-scale margin too (removes more floaters, also some roof and crown edges).
            double margin = 3 * vj[2] / Math.min(oc.fx, oc.fy);
            boolean close = agrees && !strictFreeSpace;
            for (int dy = -agreementRadius; strictFreeSpace && !close && dy <= agreementRadius; dy++)
              for (int dx = -agreementRadius; !close && dx <= agreementRadius; dx++) {
                int xx = xj + dx, yy = yj + dy;
                if (xx < 0 || yy < 0 || xx >= ow || yy >= oh) continue;
                double zn = at(oa, os, yy * ow + xx);
                close = zn > 0 && Math.abs(zn - vj[2]) <= margin;
              }
            if (close) closeViews++;
            else if (zj > 0 && vj[2] < zj - margin && !(vj[2] < zj * (1 - 3 * tol)) && reliable(oa, os, ow, xj, yj)) seeThrough++;
          }
          if (!agrees && zj <= 0) continue;
          if (agrees) {
            if ((views[j >>> 6] & 1L << (j & 63)) == 0) outsideSupport = true;
            views[j >>> 6] |= 1L << (j & 63);
            confirmed[j >>> 6] |= 1L << (j & 63);
            independent++;
          } else if (vj[2] < zj * (1 - 3 * tol) && reliable(oa, os, ow, xj, yj)) contradictions++;
        }
        int viewCount = 0;
        for (long word : views) viewCount += Long.bitCount(word);
        int needed = robust ? Math.max(1, (viewCount + 1) / 2) : 1;
        String status = "accepted";
        if(DepthOutlierFilter.enabled(o)) {
          if(patchSuspect)t[11]++;
          if(DepthOutlierFilter.cameraReject(edgeSupport,edgeThrough))t[9]++;
          else if(edgeSupport>0)t[8]++;else t[10]++;
        }
        double sigma = 1 / Math.sqrt(Math.max(1e-30, m.wsum[q])) / mz;
        if (DepthOutlierFilter.enabled(o) && DepthOutlierFilter.cameraReject(edgeSupport,edgeThrough)) status="free_space_rejected";
        else if (patchSuspect && edgeSupport==0) {status="independent_support_rejected";t[12]++;}
        else if (contradictions >= needed) status = "visibility_contradiction_rejected";
        // Free space: cameras whose surface lies clearly behind this point (more than three
        // footprints, beyond depth noise) see through it; when more of them do than confirm it, it
        // floats in front of the surface (a mismatch the relative tolerance let through).
        else if (seeThrough >= 2 && seeThrough > closeViews) status = "free_space_rejected";
        else if (m.ambiguous[q] && !outsideSupport) status = "depth_consensus_rejected";
        else if (viewCount < required) status = "independent_support_rejected";
        else if (sigma > o.maxDepthUncertainty) status = "depth_uncertainty_rejected";
        // --fusion-precision-check: more precise cameras see this surface elsewhere than they confirm.
        else if (precisionCheck && preciseDisagree > preciseConfirm) status = "precision_rejected";
        // --fusion-seed-min-angle: a narrow-pair-only depth needs two confirmations outside its pairs.
        else if (narrowSeed && outsideConfirm < 2) status = "narrow_unconfirmed_rejected";
        if (status.equals("accepted") && duplicate) {
          t[6]++;
          continue;
        }
        if (!status.equals("accepted")) {
          switch (status) {
            case "visibility_contradiction_rejected" -> t[0]++;
            case "free_space_rejected" -> t[7]++;
            case "depth_consensus_rejected" -> t[1]++;
            case "independent_support_rejected" -> t[2]++;
            case "precision_rejected" -> t[16]++;
            case "narrow_unconfirmed_rejected" -> t[17]++;
            default -> t[3]++;
          }
          continue;
        }
        // Ownership: finest footprint (depth / focal), then lowest id, among cameras whose merged
        // map holds this surface (pair partners alone do not qualify: their map may not). With
        // owner-consume the earlier-emission checks above apply in addition.
        // graph-average does not choose an owner: this unused pixel seeds one averaged point and its
        // members are marked used, so they seed nothing later.
        int owner = c;
        double finest = mz / Math.min(cam.fx, cam.fy);
        for (int j : graph ? new int[0] : near) {
          if ((confirmed[j >>> 6] & 1L << (j & 63)) == 0) continue;
          Camera oc = fusion.camera(j);
          double footprint = oc.view(X)[2] / Math.min(oc.fx, oc.fy);
          if (footprint < finest || footprint == finest && j < owner) {
            finest = footprint;
            owner = j;
          }
        }
        if (owner != c) {
          t[4]++;
          continue;
        }
        // Only the representative's own cell emits it (sparse maps reuse neighbouring samples).
        if (Math.round(u) != x || Math.round(v) != y) continue;
        BitSet supporting = BitSet.valueOf(views.clone());
        if (average && sumW > 0) {
          double zf = sumWZ / sumW;
          if(surfaceContext!=null && ownPatch.kind()==1) {
            int after=surfaceContext[c].predict(u,v,zf).kind();
            if(after==2||after==3){zf=mz;sumW=m.wsum[q];t[15]++;}
          }
          X = backProject(cam, rotationT, u, v, zf);
          sigma = 1 / Math.sqrt(sumW) / zf;
        }
        // --fusion-local-sigma: precise evidence nearby that this point did not join.
        if (best != null) {
          double own = 1 / Math.sqrt(Math.max(1e-30, average && sumW > 0 ? sumW : m.wsum[q]));
          if (best.get(p) > 0 && own > localSigma * best.get(p)) {
            t[18]++;
            continue;
          }
        }
        DepthFusion.Result r =
            new DepthFusion.Result(X, viewCount, (int) independent, sigma, Math3.norm(Math3.sub(X, data.point(rep))), "accepted");
        r.supportingCameras = supporting;
        int e = rep * fields;
        r = TwoViewConfidence.apply(r, data.evidence[e], fields == 6 ? data.evidence[e + 5] : Double.NaN, twoView);
        r.matchingConfidence = data.evidence[e];
        r.costMargin = data.evidence[e + 1];
        r.costSharpness = data.evidence[e + 2];
        r.photometric = data.evidence[e + 3];
        r.lrError = data.evidence[e + 4];
        r.confidence = PointConfidence.finalScore(r.matchingConfidence, r.relativeSigma);
        if (!r.accepted()) t[5]++;
        DepthFusion.SurfaceSample geometry =
            surface && r.accepted() ? surfaceSample(c, m, X, mz, x, y - m.row0, points) : null;
        if (list == null) list = new ArrayList<>();
        list.add(
            new Emission(
                start[data.pair] + data.first + rep,
                data.pair,
                data.pixel[rep],
                data.rgb[rep],
                r,
                geometry,
                p,
                consume || graph ? Arrays.copyOf(agreeAt, agreed) : null,
                consume || graph ? Arrays.copyOf(agreeDepth, agreed) : null));
      }
      if (list != null) rows[rowInBand] = list.toArray(Emission[]::new);
    });
    for (long[] t : tallies) {
      counters.visibility += t[0];
      counters.ambiguous += t[1];
      counters.unsupported += t[2];
      counters.uncertainty += t[3];
      counters.notOwner += t[4];
      counters.twoView += t[5];
      counters.consumed += t[6];
      counters.freeSpace += t[7];
      counters.edgeSupported+=t[8];counters.edgeFreeSpace+=t[9];counters.edgeUnknown+=t[10];
      counters.patchSuspects+=t[11];counters.patchUnsupported+=t[12];counters.patchFreeSpace+=t[13];
      counters.uncertainAverageSkipped+=t[14];counters.averageReverted+=t[15];
      counters.precisionRejected += t[16];
      counters.narrowRejected += t[17];
      counters.localSigmaRejected += t[18];
    }
    return rows;
  }

  /** Whether camera j emitted a point within radius of (x, y) at a depth agreeing with d. */
  private boolean emittedNear(int j, int x, int y, double d, int radius) {
    Bits bits = emitted[j];
    Raster z = compact[j];
    int w = fusion.width(j), h = fusion.height(j);
    double tol = o.fusionRelative;
    for (int dy = -radius; dy <= radius; dy++)
      for (int dx = -radius; dx <= radius; dx++) {
        int xx = x + dx, yy = y + dy;
        if (xx < 0 || yy < 0 || xx >= w || yy >= h) continue;
        int q = yy * w + xx;
        if (bits.get(q) && z.get(q) > 0 && Math.abs(z.get(q) - d) <= tol * Math.min(z.get(q), d))
          return true;
      }
    return false;
  }

  /**
   * --fusion-agreement-radius. auto: 0 for owner (the exact previous output), otherwise the
   * candidate step (sparser maps need a wider look).
   */
  static int agreementRadius(Options o) {
    if (o.fusionAgreementRadius >= 0) return o.fusionAgreementRadius;
    // mvsgm hands fusion every pixel of its maps (MultiViewSgm.run), as --step 1 does.
    return o.fusionStrategy.equals("owner") ? 0 : Math.max(1, o.denseMethod.equals("mvsgm") ? 1 : o.step);
  }

  /**
   * Depth agreement. --fusion-tolerance relative: within --fusion-relative x depth (the previous
   * rule, bit for bit). sigma: {@link #agreeSigma} where both inverse-variance weights are known.
   */
  private boolean agree(double a, double b, double wa, double wb, double focal) {
    if (sigmaTolerance && wa > 0 && wb > 0) return agreeSigma(a, b, wa, wb, focal);
    return Math.abs(a - b) <= o.fusionRelative * Math.min(a, b);
  }

  /**
   * |a - b| within K combined stereo sigmas (weights are inverse variances), at least one pixel
   * footprint (depth / focal) and at most the relative band.
   */
  private boolean agreeSigma(double a, double b, double wa, double wb, double focal) {
    double near = Math.min(a, b);
    double band = Math.max(sigmaK * Math.sqrt(1 / wa + 1 / wb), near / focal);
    return Math.abs(a - b) <= Math.min(band, o.fusionRelative * near);
  }

  /**
   * --fusion-seed-order precision tier of a merged pixel: 0 = merged sigma within one footprint, 1
   * = the rest, 2 = narrow-pair-only depth (--fusion-seed-min-angle).
   */
  private int tier(Merged m, int p, double focal) {
    if (seedMinSine > 0 && m.narrow[p]) return 2;
    double sigma = 1 / Math.sqrt(Math.max(1e-30, m.wsum[p]));
    return sigma <= m.z[p] / focal ? 0 : 1;
  }

  /** Separable (2r+1)^2 minimum of merged sigma = 1/sqrt(wsum) over valid pixels; 0 = none. */
  static float[] localMinimumSigma(float[] z, float[] wsum, int w, int h, int r) {
    float[] sigma = new float[w * h], rows = new float[w * h], out = new float[w * h];
    for (int p = 0; p < sigma.length; p++)
      sigma[p] = z[p] > 0 && wsum[p] > 0 ? (float) (1 / Math.sqrt(wsum[p])) : Float.POSITIVE_INFINITY;
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) {
        float best = Float.POSITIVE_INFINITY;
        for (int xx = Math.max(0, x - r); xx <= Math.min(w - 1, x + r); xx++) best = Math.min(best, sigma[y * w + xx]);
        rows[y * w + x] = best;
      }
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) {
        float best = Float.POSITIVE_INFINITY;
        for (int yy = Math.max(0, y - r); yy <= Math.min(h - 1, y + r); yy++) best = Math.min(best, rows[yy * w + x]);
        out[y * w + x] = Float.isInfinite(best) ? 0 : best;
      }
    return out;
  }

  /** A coherent, densely observed 3x3 patch (as the reference visibility test requires). */
  /**
   * A map value in the cross-check's hot loop: the array (resident maps) or the mapped file
   * (strips), with both held in locals by the caller so the resident path reads the array directly.
   */
  private static float at(float[] a, java.lang.foreign.MemorySegment s, int i) {
    return a != null ? a[i] : s.getAtIndex(MappedFiles.FLOAT, i);
  }

  private boolean reliable(float[] a, java.lang.foreign.MemorySegment s, int w, int x, int y) {
    int valid = 0;
    double lo = Double.POSITIVE_INFINITY, hi = 0;
    for (int dy = -1; dy <= 1; dy++)
      for (int dx = -1; dx <= 1; dx++) {
        double value = at(a, s, (y + dy) * w + x + dx);
        if (value <= 0) continue;
        valid++;
        lo = Math.min(lo, value);
        hi = Math.max(hi, value);
      }
    return valid >= 7 && hi - lo <= o.fusionRelative * lo;
  }

  /** Plane fit on the owner's merged map; mirrors DepthFusion.surfaceSample. */
  private static final class NormalScratch {
    final double[][] pool = new double[49][3], neighbors = new double[49][];
    final double[] residuals = new double[49];
  }

  private static final ThreadLocal<NormalScratch> NORMAL_SCRATCH =
      ThreadLocal.withInitial(NormalScratch::new);

  private DepthFusion.SurfaceSample surfaceSample(int c, Merged m, double[] p, double mz, int x, int y, double[] points) {
    Camera camera = fusion.camera(c);
    double[] rotationT = Math3.transpose(camera.rotation), center = camera.center();
    double footprint = mz / Math.min(camera.fx, camera.fy);
    double sigma = footprint * .1;
    // Per-thread scratch for the neighbour coordinates (copied from the back-projection cache;
    // nothing below retains them). Same values as freshly allocated arrays.
    NormalScratch scratch = NORMAL_SCRATCH.get();
    for (int stride : new int[] {1, 2, 4}) {
      double[][] neighbors = scratch.neighbors;
      int n = 0;
      for (int dy = -3; dy <= 3; dy++)
        for (int dx = -3; dx <= 3; dx++) {
          int xx = x + dx * stride, yy = y + dy * stride;
          if (xx < 0 || yy < 0 || xx >= m.w || yy >= m.h) continue;
          int at = yy * m.w + xx;
          double z = m.z[at];
          if (z <= 0 || Math.abs(z - mz) > o.fusionRelative * Math.min(z, mz)) continue;
          if (points != null) {
            double[] q = scratch.pool[n];
            q[0] = points[3 * at];
            q[1] = points[3 * at + 1];
            q[2] = points[3 * at + 2];
            neighbors[n++] = q;
          } else neighbors[n++] = backProject(camera, rotationT, xx + m.du[at], (yy + m.row0) + m.dv[at], z);
        }
      SurfaceGeometry.Plane plane = SurfaceGeometry.plane(neighbors, n);
      if (plane == null
          || n < 9
          || plane.middle() < footprint * footprint * .2
          || plane.smallest() > .12 * plane.middle()) continue;
      double[] normal = plane.normal();
      double[] residuals = scratch.residuals;
      for (int i = 0; i < n; i++)
        residuals[i] = Math.abs(SurfaceGeometry.distance(neighbors[i], plane.center(), normal));
      // The median residual by selection: the same order statistic as after Arrays.sort.
      double band = Math.max(footprint * .5, 3 * 1.4826 * select(residuals, n, n / 2));
      int kept = 0;
      for (int i = 0; i < n; i++)
        if (Math.abs(SurfaceGeometry.distance(neighbors[i], plane.center(), normal)) <= band)
          neighbors[kept++] = neighbors[i];
      if (kept < n * .8) return new DepthFusion.SurfaceSample(null, footprint, sigma);
      plane = SurfaceGeometry.plane(neighbors, kept);
      if (plane == null || plane.smallest() > .12 * plane.middle()) continue;
      normal = plane.normal();
      double centerError = Math.abs(SurfaceGeometry.distance(p, plane.center(), normal));
      if (centerError > Math.max(.75 * footprint, 2 * Math.sqrt(plane.smallest())))
        return new DepthFusion.SurfaceSample(null, footprint, sigma);
      double[] ray = Math3.sub(p, center);
      double cosine = Math.abs(SurfaceGeometry.dot(normal, ray)) / Math.max(1e-20, Math3.norm(ray));
      if (cosine < .15) return new DepthFusion.SurfaceSample(null, footprint, sigma);
      if (SurfaceGeometry.dot(normal, ray) > 0) normal = Math3.times(normal, -1);
      return new DepthFusion.SurfaceSample(
          normal,
          footprint,
          Math.max(.1 * footprint, sigma * cosine),
          SurfaceAttributes.of(plane, neighbors, kept, p, kept < n));
    }
    return new DepthFusion.SurfaceSample(null, footprint, sigma);
  }
}
