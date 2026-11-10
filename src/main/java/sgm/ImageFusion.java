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

  private final ForkJoinPool pool;
  private final float[][] compact;
  private SurfacePatchGrid[] surfaceContext;
  private final int[][] neighbours;
  // owner-consume / graph-average: per camera, merged pixels already represented by an emitted point.
  private final long[][] used;
  // owner-consume: per camera, pixels whose own point was emitted.
  private final long[][] emitted;
  // graph-average: per camera, the merged pixels' inverse-variance weight sums.
  private final float[][] compactWeight;
  private final List<List<Integer>> pairsOf = new ArrayList<>();
  // --thin-surfaces parts (all off: identical to the previous output).
  private final boolean sigmaTolerance, precisionCheck, storeWeights;
  private final int tierCount;
  private final double sigmaK, seedMinSine, localSigma;
  // --fusion-local-sigma: per camera, the smallest merged sigma (depth units) within 4 pixels.
  private final float[][] localBest;

  /** One pair's candidates, read from the stream; sinef = f * sin(triangulation angle). */
  private static final class PairData {
    final int pair, count;
    final double[] world;
    final float[] sinef, evidence;
    final int[] rgb, pixel;
    // --fusion-seed-min-angle: sinef below this is a narrow-pair candidate.
    double narrowSinef;

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

  /** Merged per-camera depth: 0 = invalid. Offsets are the representative's subpixel position. */
  private static final class Merged {
    final int w, h, words;
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

  private ImageFusion(Options o, DepthFusion fusion, Path candidates, long[] start) {
    this.o = o;
    this.fusion = fusion;
    this.candidates = candidates;
    this.start = start;
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
      if (b != a) pairsOf.get(b).add(k);
    }
    // Resident: compact depth for every camera, plus one camera's pairs and merge at a time.
    long compactBytes = 0, worstCamera = 0;
    for (int c = 0; c < cameras; c++) {
      long pixels = (long) fusion.width(c) * fusion.height(c), pairBytes = 0;
      compactBytes += (DepthOutlierFilter.enabled(o)?8:4) * pixels;
      if (!DepthOutlierFilter.enabled(o) && o.thinSurfaceFusion()) compactBytes += 4 * pixels;
      if (o.fusionLocalSigma > 0) compactBytes += 4 * pixels;
      for (int k : pairsOf.get(c)) pairBytes += (start[k + 1] - start[k]) * (24L + 12 + 4L * fields) + 20 * pixels;
      worstCamera = Math.max(worstCamera, pairBytes + pixels * (34L + 8L * words));
    }
    long needed = compactBytes + worstCamera, budget = (long) (Runtime.getRuntime().maxMemory() * .75);
    if(DepthOutlierFilter.enabled(o))needed+=4096L*cameras*SurfacePatchGrid.cacheCells(o,cameras);
    if (needed > budget)
      throw new IllegalArgumentException(
          "Image-space fusion needs about " + needed / 1048576 + " MiB heap (compact maps "
              + compactBytes / 1048576 + " MiB + largest camera " + worstCamera / 1048576
              + " MiB); raise PHOTOGRAMMETRY_HEAP/-Xmx or use --fusion-mode reference");
    long requestedCache = o.memoryMb * 1048576 / 4;
    if (o.imageFusionCacheMb >= 0)
      requestedCache = Math.min(requestedCache, (long) o.imageFusionCacheMb * 1048576);
    // Pass 1 may work on several cameras at once: each extra camera needs another worst-case
    // camera workspace. At most half of the remaining headroom is used for that; the pair cache
    // gets what is left. Neither changes any result.
    long headroom = Math.max(0, budget - needed - budget / 8);
    passOneCameras =
        (int) Math.max(1, Math.min(Math.max(1, o.cores / 2), 1 + headroom / 2 / Math.max(1, worstCamera)));
    // Leave a further margin for output, normals and other live pipeline state.
    cacheLimit = Math.max(0, Math.min(requestedCache, headroom - (passOneCameras - 1) * worstCamera));
    compact = new float[cameras][];
    neighbours = new int[cameras][];
    used = new long[cameras][];
    emitted = new long[cameras][];
    compactWeight = new float[cameras][];
    localBest = new float[cameras][];
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
    ImageFusion f = new ImageFusion(o, fusion, candidates, pairStarts);
    try {
      return f.fuse(output, diagnostics);
    } finally {
      f.pairCache.clear();
      f.readBuffer = null;
      f.pool.shutdown();
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
    // f * sin(triangulation angle): DepthFusion.weight without the depth term.
    int a = fusion.pairCamera(k, false), b = fusion.pairCamera(k, true);
    Camera ca = fusion.camera(a), cb = fusion.camera(b);
    double[] centerA = ca.center(), centerB = cb.center();
    double focal = Math.min(Math.min(ca.fx, ca.fy), Math.min(cb.fx, cb.fy));
    data.narrowSinef = focal * seedMinSine;
    parallel(n, j -> {
      double[] p = data.point(j), u = Math3.sub(p, centerA), v = Math3.sub(p, centerB);
      double denominator = Math3.norm(u) * Math3.norm(v);
      double x = u[1] * v[2] - u[2] * v[1], y = u[2] * v[0] - u[0] * v[2], z = u[0] * v[1] - u[1] * v[0];
      double sine = denominator == 0 ? 0 : Math.min(1, Math.sqrt(x * x + y * y + z * z) / denominator);
      data.sinef[j] = (float) Math.max(1e-9, focal * sine);
    });
    if (retain) {
      pairCache.put(k, data);
      cachedBytes += data.bytes();
      counters.cachePeakBytes = Math.max(counters.cachePeakBytes, cachedBytes);
    }
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
    long splatNanos = 0, mergeNanos = 0, speckleNanos = 0;
    try (FileChannel channel = FileChannel.open(candidates, StandardOpenOption.READ)) {
      WorkProgress progress = WorkProgress.start("Image fusion cameras", 2L * cameras, "camera passes");
      // Pass 1: filtered merged depth per camera, plus sample points for overlap detection. A
      // camera's pass 1 reads only its own pairs and writes only its own maps, so several cameras
      // run at once (bounded by heap; see passOneCameras). Counts are added in camera order.
      long[][] times = new long[cameras][3];
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
                long clock = System.nanoTime();
                Splat[] splats = splat(channel, camera, false, counters);
                times[camera][0] = System.nanoTime() - clock;
                clock = System.nanoTime();
                Merged m = merge(camera, splats, own);
                times[camera][1] = System.nanoTime() - clock;
                clock = System.nanoTime();
                own.spikes += spikes(m, fusion.camera(camera));
                own.speckle += speckles(m, fusion.camera(camera));
                if (o.fusionDepthOutliers) own.depthOutliers += depthOutliers(m.z);
                times[camera][2] = System.nanoTime() - clock;
                for (float z : m.z) if (z > 0) own.pixels++;
                compact[camera] = m.z;
                if (storeWeights) compactWeight[camera] = m.wsum;
                if (localSigma > 0) localBest[camera] = localMinimumSigma(m.z, m.wsum, m.w, m.h, 4);
                samples[camera] = sample(camera, m, splats);
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
      }
      RunTelemetry.recordTotal("Image fusion: pass 1 splats (pair reads)", splatNanos, cameras);
      RunTelemetry.recordTotal("Image fusion: pass 1 per-camera merge", mergeNanos, cameras);
      RunTelemetry.recordTotal("Image fusion: pass 1 2D speckle filter", speckleNanos, cameras);
      long clock = System.nanoTime();
      if(DepthOutlierFilter.enabled(o)) {
        surfaceContext=new SurfacePatchGrid[cameras];
        for(int c=0;c<cameras;c++)if(compact[c]!=null) {
          final float[] raster=compact[c];
          surfaceContext[c]=new SurfacePatchGrid(fusion.width(c),fusion.height(c),p->raster[p],o.fusionRelative,SurfacePatchGrid.cacheCells(o,cameras));
        }
      }
      for (int c = 0; c < cameras; c++) neighbours[c] = overlapping(c, samples);
      RunTelemetry.record("Image fusion: camera overlap", clock);
      long remergeNanos = 0, checkNanos = 0, outputNanos = 0;
      // Pass 2: re-merge one camera at a time (identical), cross-check, own and emit.
      // --fusion-seed-order precision repeats it per precision tier (precise seeds first).
      // Ownership marks (used/emitted) are applied here before the next camera is checked, since
      // later cameras read them. Appending the emitted points to the output only has to keep
      // their order, so one writer thread does it in camera order, overlapping the next camera's
      // cross-check; at most two cameras' emissions wait for it.
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
        if (compact[c] == null) continue;
        clock = System.nanoTime();
        Splat[] splats = splat(channel, c, true, counters);
        Merged m = merge(c, splats, null);
        // Pass 1's speckle decisions, without recounting.
        float[] kept = compact[c];
        for (int p = 0; p < kept.length; p++) if (kept[p] <= 0) m.z[p] = 0;
        remergeNanos += System.nanoTime() - clock;
        clock = System.nanoTime();
        Emission[][] rows = crossCheck(c, m, splats, counters, tier);
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
                if (emitted[c] == null) emitted[c] = new long[(compact[c].length + 63) / 64];
                emitted[c][e.cameraPixel() >>> 6] |= 1L << (e.cameraPixel() & 63);
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
  private void markUsed(Emission e, int radius) {
    double tol = o.fusionRelative;
    for (int k = 0; k < e.agreements().length; k++) {
      long a = e.agreements()[k];
      int j = (int) (a >>> 42), y = (int) (a >>> 21 & 0x1FFFFF), x = (int) (a & 0x1FFFFF);
      double d = e.agreementDepths()[k];
      float[] z = compact[j];
      int w = fusion.width(j), h = fusion.height(j);
      if (used[j] == null) used[j] = new long[(w * h + 63) / 64];
      for (int dy = -radius; dy <= radius; dy++)
        for (int dx = -radius; dx <= radius; dx++) {
          int xx = x + dx, yy = y + dy;
          if (xx < 0 || yy < 0 || xx >= w || yy >= h) continue;
          int q = yy * w + xx;
          if (z[q] > 0 && Math.abs(z[q] - d) <= tol * Math.min(z[q], d))
            used[j][q >>> 6] |= 1L << (q & 63);
        }
    }
  }

  /** Splats each of camera c's pairs into c. */
  private Splat[] splat(FileChannel channel, int c, boolean full, Counters counters) throws IOException {
    List<Integer> pairs = pairsOf.get(c);
    Splat[] out = new Splat[pairs.size()];
    Camera cam = fusion.camera(c);
    int w = fusion.width(c), h = fusion.height(c);
    PairData[] loaded = new PairData[out.length];
    for (int s = 0; s < out.length; s++) loaded[s] = load(channel, pairs.get(s), full, counters);
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
        if (x < 0 || x >= w || y < 0 || y >= h) continue;
        int at = y * w + x;
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

  /** Reference depth-mode merge per pixel; ambiguous pixels keep the nearer cluster, flagged. */
  private Merged merge(int c, Splat[] splats, Counters counters) {
    int w = fusion.width(c), h = fusion.height(c);
    Merged m = new Merged(w, h, words);
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
    if (counters != null) for (long a : ambiguous) counters.ambiguousKept += a;
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
      if (j == c || compact[j] == null) continue;
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

  /** Removes 4-connected components smaller than the speckle size; returns removed pixels.
   * Neighbours connect within the relative fusion tolerance or two pixel footprints (z / f), so
   * slanted surfaces at coarse resolution are not fragmented. */
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

  private long spikes(Merged m, Camera camera) {
    if (!o.fusionSpikeFilter) return 0;
    int w = m.w, h = m.h, r = 2;
    double focal = Math.min(camera.fx, camera.fy);
    float[] z = m.z;
    boolean[] spike = new boolean[w * h];
    long[] counts = new long[h];
    parallel(h, y -> {
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
        Arrays.sort(values, 0, n);
        float median = n % 2 == 1 ? values[n / 2] : .5f * (values[n / 2 - 1] + values[n / 2]);
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

  private long speckles(Merged m, Camera camera) {
    if (o.speckleSize <= 0) return 0;
    double slope = 2 / Math.min(camera.fx, camera.fy);
    int w = m.w, h = m.h;
    int[] label = new int[w * h], stack = new int[w * h];
    long removed = 0;
    int next = 0;
    double tol = o.fusionRelative;
    int[] members = new int[Math.min(o.speckleSize, 1 << 16)];
    for (int start = 0; start < w * h; start++) {
      if (m.z[start] <= 0 || label[start] != 0) continue;
      next++;
      int top = 0, size = 0, stored = 0;
      stack[top++] = start;
      label[start] = next;
      while (top > 0) {
        int p = stack[--top];
        if (stored < members.length) members[stored++] = p;
        size++;
        int x = p % w, y = p / w;
        for (int d = 0; d < 4; d++) {
          int xx = x + (d == 0 ? 1 : d == 1 ? -1 : 0), yy = y + (d == 2 ? 1 : d == 3 ? -1 : 0);
          if (xx < 0 || yy < 0 || xx >= w || yy >= h) continue;
          int q = yy * w + xx;
          if (label[q] != 0 || m.z[q] <= 0) continue;
          double near = Math.min(m.z[p], m.z[q]);
          if (Math.abs(m.z[q] - m.z[p]) > Math.max(tol, slope) * near) continue;
          label[q] = next;
          stack[top++] = q;
        }
      }
      if (size < o.speckleSize)
        for (int i = 0; i < stored; i++) {
          m.z[members[i]] = 0;
          removed++;
        }
    }
    return removed;
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

  /** Cross-check, ownership and emission for camera c, row-parallel and row-ordered. */
  private Emission[][] crossCheck(int c, Merged m, Splat[] splats, Counters counters, int tier) {
    int w = m.w, h = m.h, required = fusion.requiredViews();
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
    float[] best = localBest[c];
    long[] taken = consume || graph ? used[c] : null;
    Map<Integer, PairData> byPair = new HashMap<>();
    for (Splat s : splats) byPair.put(s.data().pair, s.data());
    // Normal estimation back-projects up to 147 neighbours per emitted pixel; neighbouring pixels
    // share them, so each merged pixel is back-projected once here (same values; m is read-only
    // during the cross-check). Skipped above 512 MB, where the direct computation is used.
    double[] points =
        surface && 24L * w * h <= 512L << 20 && !Boolean.getBoolean("sgm.noNormalCache") ? new double[3 * w * h] : null;
    if (points != null)
      parallel(h, y -> {
        for (int x = 0; x < w; x++) {
          int at = y * w + x;
          if (m.z[at] <= 0) continue;
          double[] q = backProject(cam, rotationT, x + m.du[at], y + m.dv[at], m.z[at]);
          System.arraycopy(q, 0, points, 3 * at, 3);
        }
      });
    Emission[][] rows = new Emission[h][];
    long[][] tallies = new long[h][20];
    parallel(h, y -> {
      List<Emission> list = null;
      long[] t = tallies[y];
      long[] views = new long[words], confirmed = new long[words];
      long[] agreeAt = new long[near.length];
      float[] agreeDepth = new float[near.length];
      double[] vj = new double[3];
      for (int x = 0; x < w; x++) {
        int p = y * w + x;
        float mz = m.z[p];
        if (mz <= 0) continue;
        if (tierCount > 1 && tier(m, p, focal) != tier) continue;
        if (taken != null && (taken[p >>> 6] & 1L << (p & 63)) != 0) {
          t[6]++;
          continue;
        }
        int agreed = 0;
        boolean duplicate = false;
        PairData data = byPair.get(m.repPair[p]);
        int rep = m.repIndex[p];
        // FP64 back-projection along the representative's ray at the merged depth.
        double u = x + m.du[p], v = y + m.dv[p];
        double[] X = backProject(cam, rotationT, u, v, mz);
        // graph-average: inverse-variance sums of depths along this pixel's ray. dX/dz is the ray
        // direction per unit of this camera's depth; another camera's depth is linear along it.
        double sumW = average ? m.wsum[p] : 0, sumWZ = average ? m.wsum[p] * (double) mz : 0;
        double[] ray = average ? Math3.mv(rotationT, new double[] {(u - cam.cx) / cam.fx, (v - cam.cy) / cam.fy, 1}) : null;
        Arrays.fill(confirmed, 0);
        for (int k = 0; k < words; k++) views[k] = m.partners.length > 0 ? m.partners[p * words + k] : 0;
        views[c >>> 6] |= 1L << (c & 63);
        long independent = m.support[p];
        int contradictions = 0, closeViews = 0, seeThrough = 0;
        int edgeSupport=0,edgeThrough=0;
        var ownPatch=surfaceContext==null?SurfacePatchGrid.Prediction.UNKNOWN:surfaceContext[c].predict(u,v,mz);
        boolean patchSuspect=ownPatch.kind()==2||ownPatch.kind()==3;
        boolean outsideSupport = false;
        double seedWeight = m.wsum[p];
        int preciseConfirm = 0, preciseDisagree = 0, outsideConfirm = 0;
        boolean narrowSeed = seedMinSine > 0 && m.narrow[p];
        for (int j : near) {
          float[] other = compact[j];
          Camera oc = fusion.camera(j);
          int ow = fusion.width(j), oh = fusion.height(j);
          oc.viewInto(X[0], X[1], X[2], vj);
          if (vj[2] <= 0) continue;
          double uj = oc.fx * vj[0] / vj[2] + oc.cx, wj = oc.fy * vj[1] / vj[2] + oc.cy;
          int xj = (int) Math.round(uj), yj = (int) Math.round(wj);
          if (xj < 1 || yj < 1 || xj >= ow - 1 || yj >= oh - 1) continue;
          double zj = other[yj * ow + xj];
          boolean preciseAgreement=false;
          if(DepthOutlierFilter.enabled(o)) {
            // The merged map is one vote per camera. Source-pair cameras cannot independently
            // confirm their own mismatch. Uncertainty comes from the generating pair and the
            // other camera's merged inverse-variance weight, where available.
            int pair=data.pair;
            {
              double low=Double.POSITIVE_INFINITY,high=0;int count=0;
              for(int dy=-1;dy<=1;dy++)for(int dx=-1;dx<=1;dx++) {
                double z=other[(yj+dy)*ow+xj+dx];
                if(z>0){low=Math.min(low,z);high=Math.max(high,z);count++;}
              }
              double footprint=vj[2]/Math.min(oc.fx,oc.fy);
              double sourceSigma=vj[2]/Math.sqrt(Math.max(1e-30,m.wsum[p]))/mz;
              double weight=compactWeight[j]==null?0:compactWeight[j][yj*ow+xj];
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
          float[] weightsJ = compactWeight[j];
          double focalJ = Math.min(oc.fx, oc.fy);
          double seedJ = seedWeight * ((double) mz / vj[2]) * ((double) mz / vj[2]);
          boolean agrees = zj > 0 && agree(zj, vj[2], weightsJ == null ? 0 : weightsJ[yj * ow + xj], seedJ, focalJ);
          int ax = xj, ay = yj;
          for (int dy = -agreementRadius; !agrees && dy <= agreementRadius; dy++)
            for (int dx = -agreementRadius; !agrees && dx <= agreementRadius; dx++) {
              int xx = xj + dx, yy = yj + dy;
              if ((dx | dy) == 0 || xx < 0 || yy < 0 || xx >= ow || yy >= oh) continue;
              double zn = other[yy * ow + xx];
              agrees = zn > 0 && agree(zn, vj[2], weightsJ == null ? 0 : weightsJ[yy * ow + xx], seedJ, focalJ);
              if (agrees) {
                ax = xx;
                ay = yy;
              }
            }
          if (precisionCheck && zj > 0 && weightsJ != null && weightsJ[yj * ow + xj] > 4 * seedJ) {
            // Camera j is at least 2x more precise here. Confirmation at its precision, or a near
            // miss: the same surface within the relative band but outside K sigma (occluders and
            // other layers lie beyond the band and stay neutral).
            boolean tight = false;
            for (int dy = -agreementRadius; !tight && dy <= agreementRadius; dy++)
              for (int dx = -agreementRadius; !tight && dx <= agreementRadius; dx++) {
                int xx = xj + dx, yy = yj + dy;
                if (xx < 0 || yy < 0 || xx >= ow || yy >= oh) continue;
                double zn = other[yy * ow + xx], wn = weightsJ[yy * ow + xx];
                tight = zn > 0 && wn > 0 && agreeSigma(zn, vj[2], wn, seedJ, focalJ);
              }
            if (tight) preciseConfirm++;
            else if (Math.abs(zj - vj[2]) <= tol * Math.min(zj, vj[2]) && reliable(other, ow, xj, yj)) preciseDisagree++;
          }
          if (agrees && (m.partners.length == 0 || (m.partners[p * words + (j >>> 6)] & 1L << (j & 63)) == 0))
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
            double zm = other[ay * ow + ax], wm = compactWeight[j][ay * ow + ax];
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
                double zn = other[yy * ow + xx];
                close = zn > 0 && Math.abs(zn - vj[2]) <= margin;
              }
            if (close) closeViews++;
            else if (zj > 0 && vj[2] < zj - margin && !(vj[2] < zj * (1 - 3 * tol)) && reliable(other, ow, xj, yj)) seeThrough++;
          }
          if (!agrees && zj <= 0) continue;
          if (agrees) {
            if ((views[j >>> 6] & 1L << (j & 63)) == 0) outsideSupport = true;
            views[j >>> 6] |= 1L << (j & 63);
            confirmed[j >>> 6] |= 1L << (j & 63);
            independent++;
          } else if (vj[2] < zj * (1 - 3 * tol) && reliable(other, ow, xj, yj)) contradictions++;
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
        double sigma = 1 / Math.sqrt(Math.max(1e-30, m.wsum[p])) / mz;
        if (DepthOutlierFilter.enabled(o) && DepthOutlierFilter.cameraReject(edgeSupport,edgeThrough)) status="free_space_rejected";
        else if (patchSuspect && edgeSupport==0) {status="independent_support_rejected";t[12]++;}
        else if (contradictions >= needed) status = "visibility_contradiction_rejected";
        // Free space: cameras whose surface lies clearly behind this point (more than three
        // footprints, beyond depth noise) see through it; when more of them do than confirm it, it
        // floats in front of the surface (a mismatch the relative tolerance let through).
        else if (seeThrough >= 2 && seeThrough > closeViews) status = "free_space_rejected";
        else if (m.ambiguous[p] && !outsideSupport) status = "depth_consensus_rejected";
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
            if(after==2||after==3){zf=mz;sumW=m.wsum[p];t[15]++;}
          }
          X = backProject(cam, rotationT, u, v, zf);
          sigma = 1 / Math.sqrt(sumW) / zf;
        }
        // --fusion-local-sigma: precise evidence nearby that this point did not join.
        if (best != null) {
          double own = 1 / Math.sqrt(Math.max(1e-30, average && sumW > 0 ? sumW : m.wsum[p]));
          if (best[p] > 0 && own > localSigma * best[p]) {
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
            surface && r.accepted() ? surfaceSample(c, m, X, mz, x, y, points) : null;
        if (list == null) list = new ArrayList<>();
        list.add(
            new Emission(
                start[data.pair] + rep,
                data.pair,
                data.pixel[rep],
                data.rgb[rep],
                r,
                geometry,
                p,
                consume || graph ? Arrays.copyOf(agreeAt, agreed) : null,
                consume || graph ? Arrays.copyOf(agreeDepth, agreed) : null));
      }
      if (list != null) rows[y] = list.toArray(Emission[]::new);
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
    long[] bits = emitted[j];
    float[] z = compact[j];
    int w = fusion.width(j), h = fusion.height(j);
    double tol = o.fusionRelative;
    for (int dy = -radius; dy <= radius; dy++)
      for (int dx = -radius; dx <= radius; dx++) {
        int xx = x + dx, yy = y + dy;
        if (xx < 0 || yy < 0 || xx >= w || yy >= h) continue;
        int q = yy * w + xx;
        if ((bits[q >>> 6] & 1L << (q & 63)) != 0 && z[q] > 0 && Math.abs(z[q] - d) <= tol * Math.min(z[q], d))
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
    return o.fusionStrategy.equals("owner") ? 0 : Math.max(1, o.step);
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
  private boolean reliable(float[] z, int w, int x, int y) {
    int valid = 0;
    double lo = Double.POSITIVE_INFINITY, hi = 0;
    for (int dy = -1; dy <= 1; dy++)
      for (int dx = -1; dx <= 1; dx++) {
        double value = z[(y + dy) * w + x + dx];
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
          } else neighbors[n++] = backProject(camera, rotationT, xx + m.du[at], yy + m.dv[at], z);
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
      Arrays.sort(residuals, 0, n);
      double band = Math.max(footprint * .5, 3 * 1.4826 * residuals[n / 2]);
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
