package sgm;

import static java.lang.foreign.ValueLayout.*;

import java.lang.foreign.*;
import java.util.*;

/** Batched FP64 fusion. Exact float/offset evidence stays resident; no lossy depth conversion. */
final class GpuFusion implements AutoCloseable {
  static final int MAX_BATCH = 4096;
  static final String[] STATUS = {
    "accepted",
    "depth_consensus_rejected",
    "independent_support_rejected",
    "refinement_reprojection_rejected",
    "refinement_support_rejected",
    "depth_uncertainty_rejected",
    "visibility_contradiction_rejected"
  };
  private final GpuSgm.Compute cl;
  private final int images, pairs, batch, bankCells, required, refine, scratchEndpoints;
  private int endpoints;
  private final FusionScope scope;
  private final List<DepthFusion.Endpoint> allEvidence;
  private long[] allBases;
  private int activeSource = -1;
  private long scopeReplays;
  private final MemorySegment[] banks = new MemorySegment[8];
  private MemorySegment camera, dimensions, mapInfo, starts, adjacency, pairInfo, config;
  private MemorySegment points,
      supporting,
      sources,
      measurements,
      members,
      modes,
      indices,
      ordering,
      depths,
      meta,
      values;
  private MemorySegment lookup, finish, visibility;
  private final long[] guarded = new long[7];
  private final long started = System.nanoTime();
  private double preparationSeconds;
  private long residentBytes;
  private long surfaceBudget;
  private final boolean packed;
  private final boolean windowed;
  private List<DepthFusion.Endpoint> evidence;
  private final Camera[] cameras;
  private final int[] widths, heights;
  private final long evidenceCapacity;
  private final int windowHalo;
  private long windowUploads, windowBytes, windowNanos, windowFallbacks;
  private int[][] loadedBox;
  private long windowCacheHits;

  long surfaceBudget() {
    return surfaceBudget;
  }

  Object guards() {
    return Json.object(
        "threshold",
        guarded[0],
        "reprojection",
        guarded[1],
        "mode_tie",
        guarded[2],
        "independent_tie",
        guarded[3],
        "weighted_median",
        guarded[4],
        "lookup",
        guarded[5],
        "uncertainty",
        guarded[6]);
  }

  Object resources() {
    return Json.object(
        "device",
        cl.device.name(),
        "resident_evidence_bytes",
        residentBytes,
        "evidence_layout",
        windowed ? "regional_pair_windows" : packed ? "camera_sparse_layers" : "pair_maps",
        "window_uploads",
        windowUploads,
        "window_uploaded_bytes",
        windowBytes,
        "window_upload_nanoseconds",
        windowNanos,
        "window_cpu_guards",
        windowFallbacks,
        "window_cache_hits",
        windowCacheHits,
        "camera_culling", scope != null,
        "full_endpoint_count", allEvidence.size(),
        "scratch_endpoint_capacity", scratchEndpoints,
        "scope_cpu_replays", scopeReplays,
        "batch_capacity",
        batch,
        "preparation_seconds",
        preparationSeconds,
        "guard_reasons",
        guards(),
        "opencl_profile",
        cl.profile());
  }

  GpuFusion(
      Camera[] cameras,
      int[] widths,
      int[] heights,
      List<DepthFusion.Endpoint> evidence,
      int[][] pairImages,
      Options options,
      int requiredViews) {
    this(cameras, widths, heights, evidence, pairImages, options, requiredViews, 1L << 30, null);
  }

  GpuFusion(
      Camera[] cameras,
      int[] widths,
      int[] heights,
      List<DepthFusion.Endpoint> evidence,
      int[][] pairImages,
      Options options,
      int requiredViews,
      long bankCap) {
    this(cameras, widths, heights, evidence, pairImages, options, requiredViews, bankCap, null);
  }

  GpuFusion(
      Camera[] cameras,
      int[] widths,
      int[] heights,
      List<DepthFusion.Endpoint> evidence,
      int[][] pairImages,
      Options options,
      int requiredViews,
      CameraEvidence atlas) {
    this(cameras, widths, heights, evidence, pairImages, options, requiredViews, 1L << 30, atlas);
  }

  GpuFusion(
      Camera[] cameras,
      int[] widths,
      int[] heights,
      List<DepthFusion.Endpoint> evidence,
      int[][] pairImages,
      Options options,
      int requiredViews,
      long bankCap,
      CameraEvidence atlas) {
    this(cameras, widths, heights, evidence, pairImages, options, requiredViews, bankCap, atlas, null);
  }

  GpuFusion(
      Camera[] cameras,
      int[] widths,
      int[] heights,
      List<DepthFusion.Endpoint> evidence,
      int[][] pairImages,
      Options options,
      int requiredViews,
      long bankCap,
      CameraEvidence atlas, FusionScope scope) {
    this.evidence = evidence;
    this.allEvidence = List.copyOf(evidence);
    this.scope = scope;
    this.cameras = cameras;
    this.widths = widths;
    this.heights = heights;
    windowHalo = Math.max(16, (int) Math.ceil(options.fusionPixels) + 3);
    cl = new GpuSgm.Compute(options, kernelSource(options));
    images = cameras.length;
    pairs = pairImages.length;
    endpoints = evidence.size();
    scratchEndpoints = scope == null ? endpoints : Math.max(1, scope.maxEndpoints(evidence));
    required = requiredViews;
    refine =
        options.depthRefinement ? options.depthRefinementMethod.equals("reprojection") ? 2 : 1 : 0;
    try {
      if (images == 0 || pairs == 0 || endpoints == 0)
        throw new IllegalArgumentException("No fusion evidence");
      long totalCells = 0;
      for (DepthFusion.Endpoint e : evidence) totalCells += e.map().length;
      if (atlas != null) totalCells = atlas.cells();
      long bankBytes = Math.min(bankCap, cl.device.allocation()) / 8 * 8;
      bankCells = Math.toIntExact(bankBytes / 8);
      long evidenceBytes = Math.multiplyExact(totalCells, 8L);
      long budget = GpuSgm.budget(cl.device, options);
      long relativeBytes = images <= 256 ? 24L * images * images : 0;
      long perCandidate =
          24
              + 4
              + scratchEndpoints * 52L
              + images * 12L
              + pairs * 8L
              + (pairs + 1L) * 24
              + 56
              + ((images + 31) / 32) * 4L;
      long overhead = 1048576L + images * 256L + endpoints * 64L + relativeBytes;
      windowed =
          options.fusionGpuEvidence.equals("windowed")
              || options.fusionGpuEvidence.equals("auto")
                  && (evidenceBytes > bankBytes * 8
                      || evidenceBytes + overhead + 32 * perCandidate > budget);
      packed = atlas != null && !windowed;
      if (windowed)
        evidenceBytes =
            Math.max(
                0,
                Math.min(
                    bankBytes * 8, Math.min(budget / 2, budget - overhead - 32 * perCandidate)));
      if (windowed && options.fusionWindowMb > 0)
        evidenceBytes = Math.min(evidenceBytes, options.fusionWindowMb * 1048576L);
      if (bankBytes < 8 || evidenceBytes < 8 || evidenceBytes > bankBytes * 8)
        throw new IllegalArgumentException("Fusion evidence exceeds eight device allocation banks");
      evidenceCapacity = evidenceBytes;
      residentBytes = evidenceBytes;
      long fixed = evidenceBytes + overhead;
      int capacity = options.fusionBatchSize;
      while (capacity >= 32
          && (fixed + capacity * perCandidate > budget
              || capacity * (long) scratchEndpoints * 48 > cl.device.allocation())) capacity /= 2;
      if (capacity < 32)
        throw new IllegalArgumentException(
            "Fusion depth evidence/scratch exceeds GPU memory budget (evidence="
                + Math.round(evidenceBytes / 1048576.)
                + " MiB, budget="
                + budget / 1048576
                + " MiB)");
      batch = capacity;
      surfaceBudget = Math.max(0, budget - fixed - capacity * perCandidate);
      System.out.printf(
          Locale.ROOT,
          "GPU fusion: %s; %d cameras, %d pairs; %.1f MiB evidence, batch %d%n",
          cl.device.name(),
          images,
          pairs,
          evidenceBytes / 1048576.,
          batch);
      for (int i = 0; i < 8; i++)
        banks[i] = cl.buffer(Math.max(8, Math.min(bankBytes, evidenceBytes - i * bankBytes)));
      double[] cdata = new double[Math.toIntExact(images * 28L + relativeBytes / 8)];
      double[][] centers = new double[images][];
      for (int i = 0; i < images; i++) centers[i] = cameras[i].center();
      int[] size = new int[images * 2],
          info = new int[endpoints * 2],
          offsets = new int[images + 1],
          pi = new int[pairs * 3];
      long[] base =
          new long[windowed ? endpoints * 6 : endpoints + (atlas == null ? 0 : images * 2)];
      for (int i = 0; i < images; i++) {
        Camera c = cameras[i];
        int at = i * 28;
        System.arraycopy(c.rotation, 0, cdata, at, 9);
        System.arraycopy(c.translation, 0, cdata, at + 9, 3);
        cdata[at + 12] = c.fx;
        cdata[at + 13] = c.fy;
        cdata[at + 14] = c.cx;
        cdata[at + 15] = c.cy;
        System.arraycopy(c.center(), 0, cdata, at + 16, 3);
        System.arraycopy(Math3.transpose(c.rotation), 0, cdata, at + 19, 9);
        size[i * 2] = widths[i];
        size[i * 2 + 1] = heights[i];
      }
      if (images <= 256)
        for (int image = 0; image < images; image++)
          for (int anchor = 0; anchor < images; anchor++)
            System.arraycopy(
                cameras[image].view(centers[anchor]),
                0,
                cdata,
                images * 28 + (image * images + anchor) * 3,
                3);
      long cursor = 0;
      long progress = System.nanoTime();
      try (Arena arena = Arena.ofConfined()) {
        MemorySegment staging = arena.allocate(16 * 1048576L, 8);
        for (int e = 0; e < endpoints; e++) {
          DepthFusion.Endpoint item = evidence.get(e);
          info[e * 2] = item.image();
          info[e * 2 + 1] = item.pair();
          base[e] = !packed ? cursor : offsets[item.image() + 1];
          offsets[item.image() + 1]++;
          if (packed || windowed) continue;
          for (int at = 0; at < item.map().length; ) {
            int bank = (int) (cursor / bankCells), position = (int) (cursor % bankCells);
            int n =
                (int)
                    Math.min(
                        Math.min(staging.byteSize() / 8, item.map().length - at),
                        bankCells - position);
            for (int j = 0; j < n; j++) {
              staging.set(JAVA_INT, j * 8L, Float.floatToRawIntBits(item.map().get(at + j)));
              staging.set(JAVA_INT, j * 8L + 4, item.map().offset(at + j));
            }
            cl.transfer(banks[bank], position * 8L, staging.asSlice(0, n * 8L), false);
            at += n;
            cursor += n;
            if (System.nanoTime() - progress >= 5_000_000_000L) {
              System.out.printf(
                  Locale.ROOT,
                  "GPU fusion: uploaded %.1f / %.1f MiB depth evidence%n",
                  cursor * 8 / 1048576.,
                  evidenceBytes / 1048576.);
              progress = System.nanoTime();
            }
          }
        }
        if (packed)
          for (int image = 0; image < images; image++) {
            CameraEvidence.Image data = atlas.images[image];
            base[endpoints + image * 2] = cursor;
            base[endpoints + image * 2 + 1] = cursor + data.pixels();
            for (long at = 0; at < data.cells(); ) {
              int bank = (int) (cursor / bankCells), position = (int) (cursor % bankCells);
              int n =
                  (int)
                      Math.min(
                          Math.min(staging.byteSize() / 8, data.cells() - at),
                          bankCells - position);
              for (int j = 0; j < n; j++) staging.set(JAVA_LONG, j * 8L, data.cell(at + j));
              cl.transfer(banks[bank], position * 8L, staging.asSlice(0, n * 8L), false);
              at += n;
              cursor += n;
            }
          }
        for (int i = 1; i <= images; i++) offsets[i] += offsets[i - 1];
        Map<Long, Integer> identities = new HashMap<>();
        for (int i = 0; i < pairs; i++) {
          pi[i * 2] = pairImages[i][0];
          pi[i * 2 + 1] = pairImages[i][1];
          int a = pairImages[i][0], b = pairImages[i][1];
          long identity = ((long) Math.min(a, b) << 32) | (Math.max(a, b) & 0xffffffffL);
          Integer canonical = identities.putIfAbsent(identity, i);
          pi[2 * pairs + i] = canonical == null ? i : canonical;
        }
        allBases = base.clone();
        camera = upload(arena.allocateFrom(JAVA_DOUBLE, cdata));
        dimensions = upload(arena.allocateFrom(JAVA_INT, size));
        mapInfo = upload(arena.allocateFrom(JAVA_INT, info));
        starts = upload(arena.allocateFrom(JAVA_LONG, base));
        adjacency = upload(arena.allocateFrom(JAVA_INT, offsets));
        pairInfo = upload(arena.allocateFrom(JAVA_INT, pi));
        config =
            upload(
                arena.allocateFrom(
                    JAVA_DOUBLE,
                    options.fusionRelative,
                    options.fusionPixels,
                    options.fusionConsensus,
                    options.maxDepthUncertainty,
                    windowed ? 2 : packed ? 1 : 0,
                    options.fusionOutliers.equals("robust") ? 1 : 0,
                    identities.size() < pairs ? 1 : 0));
      }
      points = cl.buffer(batch * 24L);
      sources = cl.buffer(batch * 4L);
      measurements = cl.buffer(batch * (long) scratchEndpoints * 48);
      members = cl.buffer(batch * (long) scratchEndpoints * 4);
      modes = cl.buffer(batch * (long) images * 8);
      indices = cl.buffer(batch * (long) (pairs + images) * 4);
      ordering = cl.buffer(batch * (long) pairs * 4);
      depths = cl.buffer(batch * (pairs + 1L) * 24);
      meta = cl.buffer(batch * 16L);
      values = cl.buffer(batch * 40L);
      supporting = cl.buffer(batch * ((images + 31) / 32) * 4L);
      lookup = cl.kernel("lookup");
      finish = cl.kernel("finish_fusion");
      visibility = cl.kernel("check_visibility");
      preparationSeconds = (System.nanoTime() - started) / 1e9;
      System.out.printf(
          Locale.ROOT,
          "GPU fusion: %s evidence ready in %.2f s%n",
          windowed ? "bounded regional" : "resident",
          preparationSeconds);
    } catch (RuntimeException e) {
      cl.close();
      throw e;
    }
  }

  private MemorySegment upload(MemorySegment data) {
    MemorySegment b = cl.buffer(data.byteSize());
    cl.transfer(b, 0, data, false);
    return b;
  }

  private boolean windowFits;

  /** Conservative projected boxes. Refined queries outside these boxes are CPU guarded below. */
  private boolean loadWindow(double[][] world, int start, int n, Arena arena) {
    long began = System.nanoTime();
    int[][] box = new int[images][4];
    for (int image = 0; image < images; image++) {
      int x0 = widths[image], y0 = heights[image], x1 = -1, y1 = -1;
      Camera c = cameras[image];
      for (int i = 0; i < n; i++) {
        double[] v = c.view(world[start + i]);
        if (!(v[2] > 0)) continue;
        double x = c.fx * v[0] / v[2] + c.cx, y = c.fy * v[1] / v[2] + c.cy;
        if (!Double.isFinite(x + y)
            || x < -windowHalo
            || y < -windowHalo
            || x >= widths[image] + windowHalo
            || y >= heights[image] + windowHalo) continue;
        x0 = Math.min(x0, Math.max(0, (int) Math.floor(x) - windowHalo));
        y0 = Math.min(y0, Math.max(0, (int) Math.floor(y) - windowHalo));
        x1 = Math.max(x1, Math.min(widths[image] - 1, (int) Math.ceil(x) + windowHalo));
        y1 = Math.max(y1, Math.min(heights[image] - 1, (int) Math.ceil(y) + windowHalo));
      }
      box[image] = new int[] {x0, y0, Math.max(0, x1 - x0 + 1), Math.max(0, y1 - y0 + 1)};
    }
    long cells = 0;
    if (loadedBox != null) {
      boolean contained = true;
      for (int image = 0; image < images; image++) {
        var b = box[image];
        var old = loadedBox[image];
        if (b[2] > 0
            && b[3] > 0
            && (b[0] < old[0]
                || b[1] < old[1]
                || b[0] + b[2] > old[0] + old[2]
                || b[1] + b[3] > old[1] + old[3])) {
          contained = false;
          break;
        }
      }
      if (contained) {
        windowFits = true;
        windowCacheHits++;
        return true;
      }
    }
    for (var endpoint : evidence) {
      int[] b = box[endpoint.image()];
      cells += (long) b[2] * b[3];
    }
    windowFits = cells * 8 <= evidenceCapacity;
    if (!windowFits) return false;
    // Prefetch reusable margins when capacity permits. The exact query boxes remain the
    // admission floor; a tight budget never turns required evidence into missing pixels.
    for (int padding : new int[] {64, 32, 16, 8}) {
      int[][] padded = new int[images][4];
      long paddedCells = 0;
      for (int image = 0; image < images; image++) {
        int[] b = box[image];
        if (b[2] == 0 || b[3] == 0) {
          padded[image] = b;
          continue;
        }
        int x0 = Math.max(0, b[0] - padding), y0 = Math.max(0, b[1] - padding);
        int x1 = Math.min(widths[image], b[0] + b[2] + padding),
            y1 = Math.min(heights[image], b[1] + b[3] + padding);
        padded[image] = new int[] {x0, y0, x1 - x0, y1 - y0};
      }
      for (var endpoint : evidence) {
        int[] b = padded[endpoint.image()];
        paddedCells += (long) b[2] * b[3];
      }
      if (paddedCells * 8 <= evidenceCapacity) {
        box = padded;
        cells = paddedCells;
        break;
      }
    }
    long[] base = new long[endpoints * 6];
    // Bounded staging is shared across all maps. No array is proportional to the full image.
    try (Arena stagingArena = Arena.ofConfined()) {
      MemorySegment staging = stagingArena.allocate(Math.min(1048576L, Math.max(8, cells * 8)), 8);
      long cursor = 0;
      for (int e = 0; e < endpoints; e++) {
        var endpoint = evidence.get(e);
        int image = endpoint.image();
        int[] b = box[image];
        int at = e * 6;
        base[at] = cursor;
        base[at + 1] = b[0];
        base[at + 2] = b[1];
        base[at + 3] = b[2];
        base[at + 4] = b[3];
        base[at + 5] = widths[image];
        long length = (long) b[2] * b[3];
        for (long p = 0; p < length; ) {
          int bank = (int) (cursor / bankCells), pos = (int) (cursor % bankCells);
          int take = (int) Math.min(Math.min(length - p, staging.byteSize() / 8), bankCells - pos);
          for (int j = 0; j < take; ) {
            long local = p + j;
            int pixel =
                Math.toIntExact((b[1] + local / b[2]) * widths[image] + b[0] + local % b[2]);
            int rowCount = Math.min(take - j, b[2] - (int) (local % b[2]));
            endpoint.map().transfer(pixel, rowCount, staging.asSlice(j * 8L, rowCount * 8L));
            j += rowCount;
          }
          cl.transfer(banks[bank], pos * 8L, staging.asSlice(0, take * 8L), false);
          p += take;
          cursor += take;
        }
      }
      cl.transfer(starts, 0, arena.allocateFrom(JAVA_LONG, base), false);
      windowBytes += cursor * 8;
    }
    windowUploads++;
    loadedBox = box;
    windowNanos += System.nanoTime() - began;
    return true;
  }

  private void selectSource(int source) {
    if (scope == null || source == activeSource) return;
    var selected = new ArrayList<DepthFusion.Endpoint>();
    var original = new ArrayList<Integer>();
    for (int e=0;e<allEvidence.size();e++) {
      var endpoint=allEvidence.get(e);
      if(scope.keeps(source,endpoint.image())) { selected.add(endpoint); original.add(e); }
    }
    if(selected.size()>scratchEndpoints)throw new IllegalStateException("Fusion shortlist exceeds planned scratch");
    evidence=selected;endpoints=selected.size();
    int[] info=new int[endpoints*2],offsets=new int[images+1];
    long[] base=new long[windowed?endpoints*6:endpoints+(packed?images*2:0)];
    for(int e=0;e<endpoints;e++) {
      var p=selected.get(e);info[e*2]=p.image();info[e*2+1]=p.pair();offsets[p.image()+1]++;
      if(!windowed)base[e]=allBases[original.get(e)];
    }
    if(packed)System.arraycopy(allBases,allEvidence.size(),base,endpoints,images*2);
    for(int i=1;i<=images;i++)offsets[i]+=offsets[i-1];
    try(Arena arena=Arena.ofConfined()) {
      cl.transfer(mapInfo,0,arena.allocateFrom(JAVA_INT,info),false);
      cl.transfer(adjacency,0,arena.allocateFrom(JAVA_INT,offsets),false);
      if(!windowed)cl.transfer(starts,0,arena.allocateFrom(JAVA_LONG,base),false);
    }
    loadedBox=null; // Region descriptors use the newly compacted endpoint indices.
    activeSource=source;
  }

  DepthFusion.Result[] fuse(int[] source, double[][] world, int count) {
    return fuse(source, world, count, true);
  }

  DepthFusion.Result[] fuse(int[] source, double[][] world, int count, boolean refinement) {
    DepthFusion.Result[] result = new DepthFusion.Result[count];
    for (int start = 0; start < count; ) {
      if (scope != null && !scope.contains(source[start], world[start])) {
        scopeReplays++; start++; continue; // Surface proposals outside the recorded domain are unknown.
      }
      selectSource(source[start]);
      int n = Math.min(batch, count - start);
      if (scope != null) {
        int end=start+1;
        while(end<start+n && source[end]==source[start] && scope.contains(source[end],world[end]))end++;
        n=end-start;
      }
      try (Arena arena = Arena.ofConfined()) {
        if (windowed) {
          while (!loadWindow(world, start, n, arena) && n > 1) n = Math.max(1, n / 2);
          if (!windowFits) {
            windowFallbacks += n;
            start += n;
            continue;
          }
        }
        MemorySegment hostPoints = arena.allocate(n * 24L),
            hostSources = arena.allocate(n * 4L),
            hostMeta = arena.allocate(n * 16L),
            hostValues = arena.allocate(n * 40L);
        for (int i = 0; i < n; i++) {
          for (int j = 0; j < 3; j++)
            hostPoints.set(JAVA_DOUBLE, i * 24L + j * 8L, world[start + i][j]);
          hostSources.set(JAVA_INT, i * 4L, source[start + i]);
        }
        cl.transfer(points, 0, hostPoints, false);
        cl.transfer(sources, 0, hostSources, false);
        // In-order queue: every work item owns its measurement; later kernels consume completed
        // arrays.
        cl.run(
            lookup,
            (long) n * endpoints,
            points,
            camera,
            dimensions,
            mapInfo,
            starts,
            config,
            banks[0],
            banks[1],
            banks[2],
            banks[3],
            banks[4],
            banks[5],
            banks[6],
            banks[7],
            measurements,
            members,
            n,
            endpoints,
            bankCells);
        cl.run(
            finish,
            n,
            points,
            sources,
            camera,
            mapInfo,
            adjacency,
            pairInfo,
            config,
            measurements,
            members,
            modes,
            indices,
            ordering,
            depths,
            meta,
            values,
            supporting,
            n,
            endpoints,
            images,
            pairs,
            required,
            refinement ? refine : 0);
        cl.run(
            visibility,
            n,
            sources,
            camera,
            dimensions,
            mapInfo,
            starts,
            adjacency,
            pairInfo,
            config,
            banks[0],
            banks[1],
            banks[2],
            banks[3],
            banks[4],
            banks[5],
            banks[6],
            banks[7],
            meta,
            values,
            supporting,
            n,
            images,
            bankCells);
        cl.transfer(meta, 0, hostMeta, true);
        cl.transfer(values, 0, hostValues, true);
        MemorySegment hostSupport = arena.allocate(n * ((images + 31) / 32) * 4L);
        cl.transfer(supporting, 0, hostSupport, true);
        for (int i = 0; i < n; i++) {
          long m = i * 16L, v = i * 40L;
          int status = hostMeta.get(JAVA_INT, m), guard = hostMeta.get(JAVA_INT, m + 12);
          for (int bit = 0; bit < guarded.length; bit++)
            if ((guard & (1 << bit)) != 0) guarded[bit]++;
          if (guard != 0) {
            if ((guard & 128) != 0) windowFallbacks++;
            continue; // Includes an unloaded regional query: exact CPU replay, never missing depth.
          }
          if (status < 0 || status >= STATUS.length)
            throw new IllegalStateException("Invalid GPU fusion status " + status);
          result[start + i] =
              new DepthFusion.Result(
                  new double[] {
                    hostValues.get(JAVA_DOUBLE, v),
                    hostValues.get(JAVA_DOUBLE, v + 8),
                    hostValues.get(JAVA_DOUBLE, v + 16)
                  },
                  hostMeta.get(JAVA_INT, m + 4),
                  hostMeta.get(JAVA_INT, m + 8),
                  hostValues.get(JAVA_DOUBLE, v + 24),
                  hostValues.get(JAVA_DOUBLE, v + 32),
                  STATUS[status]);
          if (status == 0) {
            BitSet ids = new BitSet(images);
            for (int image = 0; image < images; image++)
              if ((hostSupport.get(JAVA_INT, ((long) i * ((images + 31) / 32) + image / 32) * 4)
                      & (1 << (image % 32)))
                  != 0) ids.set(image);
            result[start + i].supportingCameras = ids;
          }
        }
      }
      start += n;
    }
    return result;
  }

  public void close() {
    if (!telemetryClosed) {
      RunTelemetry.gpuProfile("Depth fusion", cl.profile());
      telemetryClosed = true;
    }
    cl.close();
  }

  private boolean telemetryClosed;

  /**
   * Generate equivalent SoA accesses so adjacent GPU lanes read adjacent candidates for one map.
   */
  private static String kernelSource(Options o) {
    String source = KERNELS;
    if (o.fusionLayout.equals("coalesced")) {
      source =
          source
              .replace("size_t id=(size_t)candidate*E+e;", "int id=candidate;")
              .replace(
                  "__global double *m=out+id*6;",
                  "__global double *measure=out; __global double *m=out;");
      source = array(source, "member", index -> "MB(e)");
      source =
          array(
              source,
              "m",
              index -> {
                int at = index.lastIndexOf("*6");
                return at < 0
                    ? "MM(e," + index + ")"
                    : "MM("
                        + index.substring(0, at)
                        + ","
                        + (index.length() == at + 2 ? "0" : index.substring(at + 3))
                        + ")";
              });
      // Rewrite innermost index families before the arrays that reference them.
      for (String[] pair :
          new String[][] {
            {"order", "ORD"},
            {"ind", "IND"},
            {"by", "BY"},
            {"mb", "MB"},
            {"md", "MD"},
            {"dz", "DZ"},
            {"dw", "DW"},
            {"sorted", "SORT"}
          }) source = array(source, pair[0], index -> pair[1] + "(" + index + ")");
      source =
          """
#define MM(e,k) measure[(((size_t)(k)*E)+(e))*n+id]
#define MB(e) member[(size_t)(e)*n+id]
#define MD(c) modes[(size_t)(c)*n+id]
#define IND(p) indices[(size_t)(p)*n+id]
#define BY(c) indices[(size_t)(P+(c))*n+id]
#define ORD(i) ordering[(size_t)(i)*n+id]
#define DZ(i) scratch[(size_t)(i)*n+id]
#define DW(i) scratch[(size_t)(P+1+(i))*n+id]
#define SORT(i) scratch[(size_t)(2*(P+1)+(i))*n+id]
"""
              + source;
    }
    return (o.fusionWeightMath.equals("direct") ? "#define DIRECT_WEIGHT 1\n" : "")
        + (o.fusionMeasuredReprojection ? "#define MEASURED_REPROJECTION 1\n" : "")
        + source;
  }

  private static String array(
      String source, String name, java.util.function.Function<String, String> convert) {
    StringBuilder out = new StringBuilder();
    int cursor = 0;
    while (cursor < source.length()) {
      int start = source.indexOf(name + "[", cursor);
      if (start < 0) {
        out.append(source, cursor, source.length());
        break;
      }
      if (start > 0 && Character.isJavaIdentifierPart(source.charAt(start - 1))) {
        out.append(source, cursor, start + name.length());
        cursor = start + name.length();
        continue;
      }
      int end = start + name.length() + 1, depth = 1;
      for (; end < source.length() && depth > 0; end++) {
        char c = source.charAt(end);
        if (c == '[') depth++;
        else if (c == ']') depth--;
      }
      if (depth != 0) throw new IllegalStateException("Malformed internal kernel access");
      out.append(source, cursor, start)
          .append(convert.apply(source.substring(start + name.length() + 1, end - 1)));
      cursor = end;
    }
    return out.toString();
  }

  private static final String KERNELS =
      """
#pragma OPENCL EXTENSION cl_khr_fp64 : enable
#pragma OPENCL FP_CONTRACT OFF
// Scalar expressions intentionally preserve the Java operation order. No fast-math build flags.
double3 view(__global const double *c, int image, double3 p) {
  c += image*28;
  return (double3)((c[0]*p.x+c[1]*p.y+c[2]*p.z)+c[9],
    (c[3]*p.x+c[4]*p.y+c[5]*p.z)+c[10],(c[6]*p.x+c[7]*p.y+c[8]*p.z)+c[11]);
}
double3 unview(__global const double *c,int image,double3 p) {
  c += image*28; p -= (double3)(c[9],c[10],c[11]);
  return (double3)(c[19]*p.x+c[20]*p.y+c[21]*p.z,c[22]*p.x+c[23]*p.y+c[24]*p.z,c[25]*p.x+c[26]*p.y+c[27]*p.z);
}
double length3(double3 v) { return sqrt(v.x*v.x+v.y*v.y+v.z*v.z); }
int near(double a,double b) { return isfinite(a)&&isfinite(b)&&fabs(a-b)<=1e-8*fmax(1.0,fmax(fabs(a),fabs(b))); }
int compatible(double m,double z,double rel,int *guard) {
  if(m<=0 || z<=0) return 0;
  double a=fabs(m-z),b=rel*fmin(m,z); if(near(a,b))*guard|=1; return a<=b;
}
int reprojection(__global const double *c,int image,double3 a,double3 b,double tolerance,int *guard) {
  double3 x=view(c,image,a),y=view(c,image,b); c+=image*28;
  if(near(x.z,1e-8)||near(y.z,1e-8))*guard|=2;
  if(x.z<=1e-8||y.z<=1e-8)return 0;
  double dx=(c[12]*x.x/x.z+c[14])-(c[12]*y.x/y.z+c[14]);
  double dy=(c[13]*x.y/x.z+c[15])-(c[13]*y.y/y.z+c[15]);
  double d=hypot(dx,dy); if(near(d,tolerance))*guard|=2; return d<=tolerance;
}
int back(__global const double *c,int a,int b,double3 p,double3 q,double tolerance,int *guard) {
  return reprojection(c,a,p,q,tolerance,guard)&&reprojection(c,b,p,q,tolerance,guard);
}
double weight(__global const double *c,int a,int b,double3 p,double z) {
  __global const double *ca=c+a*28,*cb=c+b*28;
  double3 va=p-(double3)(ca[16],ca[17],ca[18]),vb=p-(double3)(cb[16],cb[17],cb[18]);
  double denominator=length3(va)*length3(vb);
  double3 cross=(double3)(va.y*vb.z-va.z*vb.y,va.z*vb.x-va.x*vb.z,va.x*vb.y-va.y*vb.x);
#ifdef DIRECT_WEIGHT
  double sine=denominator==0?0:fmin(1.0,length3(cross)/denominator);
#else
  double sine=denominator==0?0:sin((asin(fmin(1.0,length3(cross)/denominator))*57.29577951308232)*0.017453292519943295);
#endif
  double f=fmin(fmin(ca[12],ca[13]),fmin(cb[12],cb[13]));
  double sigma=z*.5/fmax(1e-9,f*sine);return 1/fmax(1e-20,sigma*sigma);
}
// Huber objective, first derivative, Gauss-Newton curvature and weighted squared residual.
double4 reproj_terms(__global const double *c,int image,int anchor,double3 center,double3 ray,
    double3 measured,double rho,double huber,int C,int derivatives) {
  __global const double *k=c+image*28;
  double3 t;
  if(C<=256){__global const double *cached=c+28*C+3*(image*C+anchor);t=(double3)(cached[0],cached[1],cached[2]);}
  else t=view(c,image,center);
  double3 d=(double3)((k[0]*ray.x+k[1]*ray.y)+k[2]*ray.z,
      (k[3]*ray.x+k[4]*ray.y)+k[5]*ray.z,(k[6]*ray.x+k[7]*ray.y)+k[8]*ray.z);
  double linkedZ=view(c,anchor,measured).z;
  double3 v=d+t*rho,q=d+t*(1/linkedZ);
  if(v.z<=1e-12||q.z<=1e-8)return (double4)(INFINITY,0,0,0);
  double du=(k[12]*v.x/v.z+k[14])-(k[12]*q.x/q.z+k[14]);
  double dv=(k[13]*v.y/v.z+k[15])-(k[13]*q.y/q.z+k[15]);
  double r=hypot(du,dv),objective=r<=huber?.5*r*r:huber*(r-.5*huber);
  if(!derivatives)return (double4)(objective,0,0,0);
  double ju=k[12]*(t.x*v.z-v.x*t.z)/(v.z*v.z),jv=k[13]*(t.y*v.z-v.y*t.z)/(v.z*v.z),w=r<=huber?1:huber/r;
  return (double4)(objective,w*(ju*du+jv*dv),
      w*(ju*ju+jv*jv),w*r*r);
}
// Each map pixel stores the original float32 Z bits and signed packed offsets, never half precision.
uint2 pixel(ulong at,int bankCells,__global const uint2 *b0,__global const uint2 *b1,
 __global const uint2 *b2,__global const uint2 *b3,__global const uint2 *b4,__global const uint2 *b5,
 __global const uint2 *b6,__global const uint2 *b7) {
  int bank=(int)(at/(ulong)bankCells),i=(int)(at%(ulong)bankCells);
  switch(bank){case 0:return b0[i];case 1:return b1[i];case 2:return b2[i];case 3:return b3[i];
    case 4:return b4[i];case 5:return b5[i];case 6:return b6[i];default:return b7[i];}
}
uint2 evidence_pixel(ulong at,int e,int image,int E,int bankCells,__global const ulong *base,__global const double *cfg,
 __global const uint2 *b0,__global const uint2 *b1,__global const uint2 *b2,__global const uint2 *b3,
 __global const uint2 *b4,__global const uint2 *b5,__global const uint2 *b6,__global const uint2 *b7) {
 if(cfg[4]==0)return pixel(base[e]+at,bankCells,b0,b1,b2,b3,b4,b5,b6,b7);
 if(cfg[4]==2){
  __global const ulong *r=base+6*e;long x=(long)(at%r[5])-(long)r[1],y=(long)(at/r[5])-(long)r[2];
  if(x<0||y<0||x>=(long)r[3]||y>=(long)r[4])return (uint2)(0x7fc00001u,0xabcdef01u);
  return pixel(r[0]+(ulong)y*r[3]+(ulong)x,bankCells,b0,b1,b2,b3,b4,b5,b6,b7);
 }
 uint2 header=pixel(base[E+image*2]+at,bankCells,b0,b1,b2,b3,b4,b5,b6,b7);
 uint bit=1u<<(uint)base[e];if(!(header.y&bit))return (uint2)(0,0);
 uint rank=popcount(header.y&(bit-1u));
 return pixel(base[E+image*2+1]+header.x+rank,bankCells,b0,b1,b2,b3,b4,b5,b6,b7);
}
__kernel void lookup(__global const double *points,__global const double *c,__global const int *sizes,
 __global const int *info,__global const ulong *base,__global const double *config,
 __global const uint2 *b0,__global const uint2 *b1,__global const uint2 *b2,__global const uint2 *b3,
 __global const uint2 *b4,__global const uint2 *b5,__global const uint2 *b6,__global const uint2 *b7,
 __global double *out,__global int *member,int n,int E,int bankCells) {
  size_t work=get_global_id(0);if(work>=(size_t)n*E)return;
  int e=work/n,candidate=work%n,image=info[e*2];size_t id=(size_t)candidate*E+e;
  __global double *m=out+id*6; m[0]=0;member[id]=0;
  double3 p=vload3(candidate,points),v=view(c,image,p);int guard=0;
  if(!isfinite(v.x)||!isfinite(v.y)||!isfinite(v.z)){member[id]=2;return;}
  if(near(v.z,0.0))guard=1;
  if(v.z<=0){member[id]=guard*2;return;}
  __global const double *camera=c+image*28;
  double px=camera[12]*v.x/v.z+camera[14],py=camera[13]*v.y/v.z+camera[15];
  int w=sizes[image*2],h=sizes[image*2+1];
  if(near(px,0)||near(px,w-1)||near(py,0)||near(py,h-1))guard=1;
  if(px<0||px>w-1||py<0||py>h-1){member[id]=guard*2;return;}
  int x=(int)floor(px+.5),y=(int)floor(py+.5);
  if(near(px+.5,floor(px+.5))||near(py+.5,floor(py+.5)))guard=1;
  double distance=INFINITY,second=INFINITY,u=0,t=0,z=0;
  for(int dy=-1;dy<=1;dy++)for(int dx=-1;dx<=1;dx++) {
    int xx=x+dx,yy=y+dy;if(xx<0||xx>=w||yy<0||yy>=h)continue;
    uint2 value=evidence_pixel((ulong)yy*w+xx,e,image,E,bankCells,base,config,b0,b1,b2,b3,b4,b5,b6,b7);
    if(value.x==0x7fc00001u&&value.y==0xabcdef01u){member[id]=4;return;}
    double depth=(double)as_float(value.x);if(depth<=0)continue;
    double sx=xx+(double)(short)(value.y&65535)/65534.0,sy=yy+(double)(short)(value.y>>16)/65534.0;
    double d=(sx-px)*(sx-px)+(sy-py)*(sy-py);
    if(d<distance){second=distance;distance=d;u=sx;t=sy;z=depth;}
    else if(d<second)second=d;
  }
  if(near(distance,second))guard=1;
  if(near(distance,config[1]*config[1]))guard=1;
  member[id]=guard*2;
  if(z==0||distance>config[1]*config[1])return;
  double3 world=unview(c,image,(double3)((u-camera[14])/camera[12]*z,(t-camera[15])/camera[13]*z,z));
  m[0]=z;m[1]=distance;m[2]=world.x;m[3]=world.y;m[4]=world.z;
  // Pair endpoint cameras are recovered by a separate tiny table in the following kernel.
  m[5]=0;
}
// A work item owns a complete candidate. Scratch arrays are bounded by batch size and pair count.
__kernel void finish_fusion(__global const double *points,__global const int *sources,
 __global const double *c,__global const int *info,__global const int *adj,__global const int *pi,
 __global const double *cfg,__global double *measure,__global int *member,__global double *modes,
 __global int *indices,__global int *ordering,__global double *scratch,__global int *meta,
 __global double *values,__global uint *support,int n,int E,int C,int P,int required,int refine) {
  int id=get_global_id(0);if(id>=n)return;
  int supportWords=(C+31)/32;for(int word=0;word<supportWords;word++)support[id*supportWords+word]=0;
  double3 p=vload3(id,points); int source=sources[id],a=pi[source*2],b=pi[source*2+1],guard=0;
  __global double *m=measure+(size_t)id*E*6,*md=modes+(size_t)id*C;
  __global int *mb=member+(size_t)id*E,*ind=indices+(size_t)id*(P+C),*by=ind+P,*order=ordering+(size_t)id*P;
  __global double *dz=scratch+(size_t)id*(P+1)*3,*dw=dz+P+1,*sorted=dw+P+1;
  __global int *out=meta+id*4;__global double *val=values+id*5;
  out[0]=1;out[1]=0;out[2]=0;out[3]=0;val[0]=p.x;val[1]=p.y;val[2]=p.z;val[3]=NAN;val[4]=0;
  for(int e=0;e<E;e++) {
    if(mb[e]&2)guard|=32;
    if(mb[e]&4)guard|=128;
    if(m[e*6]>0){int pair=info[e*2+1];m[e*6+5]=weight(c,pi[pair*2],pi[pair*2+1],(double3)(m[e*6+2],m[e*6+3],m[e*6+4]),m[e*6]);}
  }
  for(int image=0;image<C;image++) {
    int begin=adj[image],end=adj[image+1],best=-1,bestCount=0,total=0;double bestWeight=0;
    if(cfg[5]>0&&cfg[6]>0)for(int e=begin;e<end;e++)if(m[e*6]>0){
      int identity=pi[2*P+info[e*2+1]];
      for(int j=begin;j<end;j++)if(j!=e&&m[j*6]>0&&pi[2*P+info[j*2+1]]==identity
          &&(m[j*6+1]<m[e*6+1]||(m[j*6+1]==m[e*6+1]&&j<e))){m[e*6]=0;break;}
    }
    for(int e=begin;e<end;e++)if(m[e*6]>0)total++;
    for(int center=begin;center<end;center++)if(m[center*6]>0) {
      int count=0;double weights=0;
      for(int e=begin;e<end;e++)if(m[e*6]>0&&compatible(m[center*6],m[e*6],cfg[0],&guard)){count++;weights+=m[e*6+5];}
      if(best>=0&&count==bestCount&&near(weights,bestWeight)) {
        for(int e=begin;e<end;e++)if(m[e*6]>0&&
          compatible(m[center*6],m[e*6],cfg[0],&guard)!=compatible(m[best*6],m[e*6],cfg[0],&guard))guard|=4;
      }
      if(count>bestCount||(count==bestCount&&weights>bestWeight)){best=center;bestCount=count;bestWeight=weights;}
    }
    md[image]=0;
    if(best>=0&&bestCount>=(int)ceil(total*cfg[2])) {
      // Median is selected without changing the original membership iteration order.
      int rank=bestCount/2;
      for(int e=begin;e<end;e++)if(m[e*6]>0&&compatible(m[best*6],m[e*6],cfg[0],&guard)) {
        mb[e]|=1;int less=0,equalBefore=0;
        for(int j=begin;j<end;j++)if(m[j*6]>0&&compatible(m[best*6],m[j*6],cfg[0],&guard)) {
          if(m[j*6]<m[e*6])less++;else if(m[j*6]==m[e*6]&&j<e)equalBefore++;
        }
        if(less+equalBefore==rank)md[image]=m[e*6];
      }
    }
  }
  if(!compatible(md[a],view(c,a,p).z,cfg[0],&guard)||!compatible(md[b],view(c,b,p).z,cfg[0],&guard)){out[3]=guard;return;}
  for(int pair=0;pair<P;pair++)ind[pair]=-1;
  for(int image=0;image<C;image++)by[image]=-1;
  int views= a==b?1:2,independent=0;
  for(int image=0;image<C;image++) {
    double z=view(c,image,p).z;if(!compatible(md[image],z,cfg[0],&guard))continue;
    int supported=(image==a||image==b);
    for(int e=adj[image];e<adj[image+1];e++)if(mb[e]&1) {
      int pair=info[e*2+1];double3 q=(double3)(m[e*6+2],m[e*6+3],m[e*6+4]);
      if((pi[pair*2]==a&&pi[pair*2+1]==b)||(pi[pair*2]==b&&pi[pair*2+1]==a))continue;
      if(pair==source||!compatible(m[e*6],z,cfg[0],&guard)||!back(c,a,b,p,q,cfg[1],&guard))continue;
      int vote=cfg[5]>0?pi[2*P+pair]:pair;
      supported=1;int previous=ind[vote];
      if(previous<0){order[independent++]=vote;ind[vote]=e;}
      else {
        if(fabs(m[e*6+1]-m[previous*6+1])<=1e-12*fmax(1e-6,fmax(m[e*6+1],m[previous*6+1]))&&m[e*6+1]!=m[previous*6+1]&&
          length3((double3)(m[e*6+2]-m[previous*6+2],m[e*6+3]-m[previous*6+3],m[e*6+4]-m[previous*6+4]))>1e-9)guard|=8;
        if(m[e*6+1]<m[previous*6+1])ind[vote]=e;
      }
      if(by[image]<0||m[e*6+5]>m[by[image]*6+5])by[image]=e;
    }
    if(supported&&image!=a&&image!=b)views++;
  }
  out[1]=views;out[2]=independent;out[3]=guard;
  if(views<required){out[0]=2;return;}
  double anchorZ=view(c,a,p).z;int D=1;
  dz[0]=anchorZ;dw[0]=weight(c,a,b,p,anchorZ);
  for(int i=0;i<independent;i++) {
    int e=ind[order[i]];double3 q=(double3)(m[e*6+2],m[e*6+3],m[e*6+4]);double z=view(c,a,q).z;
    if(z>0){int pair=order[i];dz[D]=z;dw[D]=weight(c,pi[pair*2],pi[pair*2+1],q,z);D++;}
  }
  for(int i=0;i<D;i++)sorted[i]=dw[i];
  for(int i=1;i<D;i++){double v=sorted[i];int j=i;while(j>0&&sorted[j-1]>v){sorted[j]=sorted[j-1];j--;}sorted[j]=v;}
  double medianWeight=sorted[D/2];
  for(int i=0;i<D;i++)dw[i]=fmin(dw[i],4*medianWeight);
  for(int i=1;i<D;i++){double z=dz[i],w=dw[i];int j=i;while(j>0&&dz[j-1]>z){dz[j]=dz[j-1];dw[j]=dw[j-1];j--;}dz[j]=z;dw[j]=w;}
  double total=0;for(int i=0;i<D;i++)total+=dw[i];
  double acc=0,medianDepth=anchorZ;for(int i=0;i<D;i++){acc+=dw[i];if(near(acc,total*.5)&&i+1<D&&fabs(dz[i+1]-dz[i])>1e-10)guard|=16;if(acc>=total*.5){medianDepth=dz[i];break;}}
  double sum=0,ws=0;for(int i=0;i<D;i++)if(compatible(dz[i],medianDepth,cfg[0],&guard)){sum+=dw[i]*dz[i];ws+=dw[i];}
  double refinedDepth=refine&&independent>0&&ws>0?sum/ws:anchorZ;
  double reprojSigma=0;
  if(refine==2&&independent>0) {
    __global const double *ca=c+a*28;
    double3 center=(double3)(ca[16],ca[17],ca[18]),ray=(p-center)*(1/anchorZ);
    double rho=1/anchorZ,trial=1/refinedDepth,huber=fmax(.25,cfg[1]);int observations=0;
    double4 first=(double4)(0),second=(double4)(0);
    for(int image=0;image<C;image++)if(image!=a&&(by[image]>=0||image==b)) {
      int e=by[image];double3 q=image==b?p:(double3)(m[e*6+2],m[e*6+3],m[e*6+4]);
      first+=reproj_terms(c,image,a,center,ray,q,rho,huber,C,0);
      second+=reproj_terms(c,image,a,center,ray,q,trial,huber,C,0);observations++;
    }
    if(trial>0&&second.x<first.x)rho=trial;
    for(int iteration=0;iteration<6;iteration++) {
      double4 f=(double4)(0);
      for(int image=0;image<C;image++)if(image!=a&&(by[image]>=0||image==b)) {
        int e=by[image];double3 q=image==b?p:(double3)(m[e*6+2],m[e*6+3],m[e*6+4]);
        f+=reproj_terms(c,image,a,center,ray,q,rho,huber,C,1);
      }
      if(!(f.z>1e-20)||!isfinite(f.y))break;
      double step=fmax(-rho*.25,fmin(rho*.25,-f.y/f.z));int improved=0;
      for(int line=0;line<8;line++) {
        trial=rho+step;double objective=0;
        for(int image=0;image<C;image++)if(image!=a&&(by[image]>=0||image==b)) {
          int e=by[image];double3 q=image==b?p:(double3)(m[e*6+2],m[e*6+3],m[e*6+4]);
          objective+=reproj_terms(c,image,a,center,ray,q,trial,huber,C,0).x;
        }
        if(trial>0&&objective<f.x){rho=trial;improved=1;break;}step*=.5;
      }
      if(!improved||fabs(step)<rho*1e-10)break;
    }
    double4 end=(double4)(0);
    for(int image=0;image<C;image++)if(image!=a&&(by[image]>=0||image==b)) {
      int e=by[image];double3 q=image==b?p:(double3)(m[e*6+2],m[e*6+3],m[e*6+4]);
      end+=reproj_terms(c,image,a,center,ray,q,rho,huber,C,1);
    }
    refinedDepth=1/rho;
    reprojSigma=fmax(.5,sqrt(end.w/max(1,2*observations-1)))/sqrt(fmax(1e-30,end.z))/rho;
  }
  double3 refined=unview(c,a,view(c,a,p)*(refinedDepth/anchorZ));
  double sigma=1/sqrt(fmax(1e-30,ws))/fmax(1e-12,refinedDepth)*sqrt(fmax(1.0,(double)D/max(1,views-1)));
  sigma=fmax(sigma,reprojSigma);
  val[3]=sigma;
  if(!back(c,a,b,p,refined,cfg[1],&guard)||!compatible(md[a],view(c,a,refined).z,cfg[0],&guard)||!compatible(md[b],view(c,b,refined).z,cfg[0],&guard)){out[0]=3;out[3]=guard;return;}
  int verified=a==b?1:2;
  support[id*supportWords+a/32]|=1u<<(a%32);support[id*supportWords+b/32]|=1u<<(b%32);
  for(int image=0;image<C;image++)if(by[image]>=0) {
    int e=by[image];double z=view(c,image,refined).z;double3 q=(double3)(m[e*6+2],m[e*6+3],m[e*6+4]);
    if(compatible(md[image],z,cfg[0],&guard)&&compatible(m[e*6],z,cfg[0],&guard)&&back(c,a,b,refined,q,cfg[1],&guard)
#ifdef MEASURED_REPROJECTION
      &&reprojection(c,image,refined,q,cfg[1],&guard)
#endif
      &&image!=a&&image!=b){verified++;support[id*supportWords+image/32]|=1u<<(image%32);}
  }
  if(verified<required){out[0]=4;out[1]=verified;out[3]=guard;return;}
  sigma*=sqrt(fmax(1.0,(views-1.0)/max(1,verified-1)));val[3]=sigma;out[1]=verified;
  if(near(sigma,cfg[3]))guard|=64;
  if(sigma>cfg[3]){out[0]=5;out[3]=guard;return;}
  out[0]=0;val[0]=refined.x;val[1]=refined.y;val[2]=refined.z;val[4]=length3(refined-p);
  if(!isfinite(val[0])||!isfinite(val[1])||!isfinite(val[2])||!isfinite(sigma))guard|=1;
  out[3]=guard;
}
// Bounded, visibility-aware free-space contradictions; holes and nearer occluders are neutral.
__kernel void check_visibility(__global const int*sources,__global const double*c,__global const int*sizes,
 __global const int*info,__global const ulong*base,__global const int*adj,__global const int*pi,__global const double*cfg,
 __global const uint2*b0,__global const uint2*b1,__global const uint2*b2,__global const uint2*b3,
 __global const uint2*b4,__global const uint2*b5,__global const uint2*b6,__global const uint2*b7,
 __global int*meta,__global const double*values,__global const uint*support,int N,int C,int bankCells){
 int id=get_global_id(0);if(id>=N)return;__global int*out=meta+id*4;if(out[0]!=0)return;
 double3 p=vload3(0,values+id*5);int source=sources[id],a=pi[source*2],b=pi[source*2+1],inspected=0,guard=out[3],contradictions=0;
 int robust=cfg[5]>0,needed=robust?max(1,(out[1]+1)/2):1;
 for(int image=0;image<C&&inspected<8;image++){
  if(image==a||image==b||adj[image+1]-adj[image]<2)continue;
  if(robust&&(support[(size_t)id*((C+31)/32)+image/32]&(1u<<(image%32))))continue;
  double3 v=view(c,image,p);if(v.z<=0)continue;__global const double*q=c+image*28;
  double px=q[12]*v.x/v.z+q[14],py=q[13]*v.y/v.z+q[15];int w=sizes[image*2],h=sizes[image*2+1];
  if(near(px,2)||near(py,2)||near(px,w-2)||near(py,h-2))guard|=1;
  if(px<2||py<2||px>=w-2||py>=h-2)continue;inspected++;
  if(near(px+.5,floor(px+.5))||near(py+.5,floor(py+.5)))guard|=1;
  int x=(int)floor(px+.5),y=(int)floor(py+.5),reliable=0,examined=0,partners[4];double lo=INFINITY,hi=0;
  for(int e=adj[image];e<adj[image+1];e++){
   if(info[e*2+1]==source)continue;if(++examined>4)break;int count=0;double mn=INFINITY,mx=0;
   int pair=info[e*2+1],partner=pi[pair*2]==image?pi[pair*2+1]:pi[pair*2],duplicate=0;
   if(robust){for(int j=0;j<reliable;j++)if(partners[j]==partner)duplicate=1;if(duplicate)continue;}
   for(int dy=-1;dy<=1;dy++)for(int dx=-1;dx<=1;dx++){
    uint2 value=evidence_pixel((ulong)(y+dy)*w+x+dx,e,image,adj[C],bankCells,base,cfg,b0,b1,b2,b3,b4,b5,b6,b7);
    if(value.x==0x7fc00001u&&value.y==0xabcdef01u){guard|=128;continue;}
    double z=(double)as_float(value.x);
    if(z>0){mn=fmin(mn,z);mx=fmax(mx,z);count++;}
   }
   if(near(mx-mn,cfg[0]*mn))guard|=1;
   if(count<7||mx-mn>cfg[0]*mn)continue;
   lo=fmin(lo,mn);hi=fmax(hi,mx);partners[reliable]=partner;reliable++;
  }
  if(near(hi-lo,cfg[0]*lo)||near(v.z,lo*(1-3*cfg[0])))guard|=1;
  if(reliable>=2&&hi-lo<=cfg[0]*lo&&v.z<lo*(1-3*cfg[0])&&++contradictions>=needed){out[0]=6;out[3]=guard;return;}
 }
 out[3]=guard;
}

""";
}
