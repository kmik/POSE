package sgm;

import java.io.*;
import java.nio.ByteBuffer;
import java.nio.file.*;
import java.util.*;
import java.util.concurrent.*;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

/** Adapts per-reference depth/normal maps to the common pair-provenance fusion stream. */
final class PatchMatchReconstruction {
  private final Options options;
  private final List<ImageFrame> frames;
  private final List<FeatureTracks.Track> tracks;
  private final DepthFusion fusion;
  private final ProjectArtifacts artifacts;
  private final List<int[]> sources = new ArrayList<>();
  private final Map<Long, Integer> pairIds = new LinkedHashMap<>();
  private final Map<Integer, Stored> maps = new HashMap<>();
  private long reciprocalRejected, matchingRejected, angleRejected, emitted;
  private long unavailableReference, geometricRejected, depthLimitRejected;
  private long ambiguityRejected, consistencyRejected;
  // Debug aid for equivalence checks: keep every per-level map (no effect on results).
  private static final boolean KEEP_LEVELS = Boolean.getBoolean("sgm.patchmatch.keepLevels");
  // Test aid: force the on-disk generation path even when the maps would fit in memory.
  private static final boolean FORCE_DISK = Boolean.getBoolean("sgm.patchmatch.disk");
  /** Generations held in memory (else on disk, as before); decided per run in estimateMaps. */
  private boolean inMemory;
  private int referencesInFlight = 1;
  private long hostNanos, mapIoNanos;
  // Grey and mask rasters per (frame, level): built once per level instead of once per scene.
  private final LinkedHashMap<Long, byte[][]> pyramid = new LinkedHashMap<>(16, .75f, true);
  private long pyramidBytes, pyramidBuilt, pyramidHits;

  /**
   * One reference's map of a generation. In memory (map and trusted sidecar, NaNs canonical as a
   * file round trip would leave them) or on disk; both give the same values to every reader.
   */
  private static final class Stored {
    final Path file;
    final PatchMatch.Map map;
    final float[] trusted;

    Stored(Path file, PatchMatch.Map map, float[] trusted) {
      this.file = file;
      this.map = map;
      this.trusted = trusted;
    }

    PatchMatch.Map map() throws IOException {
      return map != null ? map : read(file);
    }
  }

  PatchMatchReconstruction(
      Options o,
      List<ImageFrame> frames,
      List<FeatureTracks.Track> tracks,
      DepthFusion fusion,
      ProjectArtifacts artifacts) {
    options = o;
    this.frames = frames;
    this.tracks = tracks;
    this.fusion = fusion;
    this.artifacts = artifacts;
  }

  long run(List<FeatureTracks.Pair> pairs, DataOutputStream out) throws IOException {
    return run(pairs, out, null);
  }

  /**
   * pairStarts != null with --fusion-mode image: each camera becomes one single-view fusion pair
   * whose maps (mvsgm hand-off format) hold exactly the pixels the pair stream would have
   * accepted; {@link #handoff} then feeds ImageFusion and nothing goes to the candidate stream.
   */
  long run(List<FeatureTracks.Pair> pairs, DataOutputStream out, List<Long> pairStarts)
      throws IOException {
    boolean mapHandoff = pairStarts != null && options.fusionMode.equals("image");
    for (var f : frames) {
      List<Integer> choices = new ArrayList<>();
      if (!f.camera.registered) {
        sources.add(new int[0]);
        continue;
      }
      for (var p : pairs) {
        int other = p.a == f.id ? p.b : p.b == f.id ? p.a : -1;
        if (other >= 0
            && other != f.id
            && frames.get(other).camera.registered
            && !choices.contains(other)
            && Math3.norm(Math3.sub(f.camera.center(), frames.get(other).camera.center())) > 1e-8)
          choices.add(other);
      }
      choices.sort(
          Comparator.comparingDouble(
              i -> Math3.norm(Math3.sub(f.camera.center(), frames.get(i).camera.center()))));
      if (options.patchmatchNeighbors.equals("score")) {
        // mvsgm's view selection: shared tie points weighted by triangulation angle (peak 10
        // degrees), best first; not limited to the matched pair graph.
        choices.clear();
        for (var nb : MultiViewSgm.neighbours(frames, tracks, f.id, options.patchmatchViews))
          if (Math3.norm(Math3.sub(f.camera.center(), frames.get(nb.id()).camera.center())) > 1e-8)
            choices.add(nb.id());
      }
      sources.add(
          choices.stream().limit(options.patchmatchViews).mapToInt(Integer::intValue).toArray());
      for (int other : mapHandoff ? new int[0] : sources.get(f.id)) {
        int source = fusion.addPair(f.id, other);
        pairIds.put(key(f.id, other), source);
        if (artifacts != null) artifacts.patchmatchPair(source, f, frames.get(other));
      }
    }
    long solveStarted = System.nanoTime();
    estimateMaps();
    if (!filteredDone) {
      if (options.patchmatchSmooth > 0) smoothMaps();
      if (options.patchmatchSpeckle > 0) speckleMaps();
      if (checkpoint != null
          && checkpoint.committed()
          && (options.patchmatchSmooth > 0 || options.patchmatchSpeckle > 0))
        checkpoint.commit(checkpointState("filtered", 0, 2, List.of()));
    }
    if (Boolean.getBoolean("sgm.patchmatch.failBeforeEvidence"))
      throw new IOException("Simulated interruption before the evidence stage (-Dsgm.patchmatch.failBeforeEvidence)");
    long solveNanos = System.nanoTime() - solveStarted, evidenceStarted = System.nanoTime();
    if (mapHandoff) handoff(pairStarts);
    else
    // One owner and one source map resident at a time; no all-camera depth-map heap.
    try (var rows = StereoRows.open(options.cores)) {
      for (var f : frames) {
        if (!maps.containsKey(f.id)) continue;
        // Maps on disk are read through memory-mapped views (streamed by the OS), maps in memory
        // directly; the values are the same, so is every result.
        List<MapReader> opened = new ArrayList<>();
        try {
        MapReader map = open(maps.get(f.id), opened);
        for (int i = 0; i < map.width() * map.height(); i++) {
          if (map.view(i) == -2) geometricRejected++;
          else if (Float.isFinite(map.photo(i)) && map.photo(i) < options.correlation)
            matchingRejected++;
          else if (map.view(i) < 1 || !Float.isFinite(map.depth(i))) unavailableReference++;
          else if (map.depth(i) > options.maxDepth) depthLimitRejected++;
        }
        f.ensureRasters();
        byte[] colors = new byte[f.width * f.height * 3];
        f.color.get(0, 0, colors);
        int[] selected = sources.get(f.id);
        // All source maps at once only when other views must confirm (--patchmatch-consistent-views).
        MapReader[] all = new MapReader[selected.length];
        if (options.patchmatchConsistentViews > 1)
          for (int s = 0; s < selected.length; s++)
            if (maps.containsKey(selected[s])) all[s] = open(maps.get(selected[s]), opened);
        for (int slot = 0; slot < selected.length; slot++) {
          int image = selected[slot];
          if (!maps.containsKey(image)) continue;
          var other = frames.get(image);
          MapReader check =
              all[slot] != null ? all[slot] : open(maps.get(image), opened);
          int pair = pairIds.get(key(f.id, image));
          try (WorkProgress work =
              WorkProgress.start(
                  "PatchMatch reciprocal evidence " + f.id + "-" + image, f.height, "rows")) {
            // Rows are evaluated in parallel blocks; fusion and the candidate stream then take
            // the accepted pixels in the original row-major order (identical stream).
            int block = 64;
            for (int y0 = 2; y0 < f.height - 2; y0 += block) {
              int y1 = Math.min(f.height - 2, y0 + block), from = y0;
              Evidence[] found = new Evidence[y1 - y0];
              int s = slot;
              StereoRows.rows(
                  y0, y1, y -> found[y - from] = evidence(f, other, map, check, s, y, selected, all));
              for (Evidence row : found) {
                angleRejected += row.angle;
                reciprocalRejected += row.reciprocal;
                ambiguityRejected += row.ambiguous;
                consistencyRejected += row.inconsistent;
                double[] v = row.values;
                for (int k = 0; k < row.count; k++) {
                  int at = k * Evidence.STRIDE, i = (int) v[at + 3];
                  int x = i % f.width, y = i / f.width;
                  double[] world = {v[at], v[at + 1], v[at + 2]};
                  fusion.add(pair, world);
                  if ((x - 2) % options.step != 0 || (y - 2) % options.step != 0) continue;
                  out.writeDouble(world[0]);
                  out.writeDouble(world[1]);
                  out.writeDouble(world[2]);
                  out.writeByte(colors[i * 3 + 2] & 255);
                  out.writeByte(colors[i * 3 + 1] & 255);
                  out.writeByte(colors[i * 3] & 255);
                  out.writeInt(pair);
                  out.writeInt(i);
                  for (int q = 4; q < Evidence.STRIDE; q++) out.writeFloat((float) v[at + q]);
                  emitted++;
                }
              }
              work.update(y1);
            }
            work.done();
          }
        }
        } finally {
          for (MapReader r : opened) r.close();
        }
      }
    }
    long evidenceNanos = System.nanoTime() - evidenceStarted;
    // With a checkpoint the final maps stay until fusion completes (complete()).
    if ((!options.diagnostics || options.storage.equals("minimal")) && !KEEP_LEVELS && checkpoint == null)
      for (Stored map : maps.values()) Files.deleteIfExists(map.file);
    Json.write(
        options.output.resolve("05_stereo/patchmatch.json"),
        Json.object(
            "method",
            "patchmatch",
            "experimental",
            true,
            "confidence_model",
            PointConfidence.MODEL,
            "sampled_candidates",
            emitted,
            "photometric_rejected",
            matchingRejected,
            "ambiguous_producing_pair_pixels",
            ambiguityRejected,
            "pair_confidence_rule",
            "weaker photometry, margin and curvature from both producing maps; unavailable or"
                + " nonpositive uniqueness cannot corroborate another point",
            "unavailable_reference_pixels",
            unavailableReference,
            "geometric_contradiction_reference_pixels",
            geometricRejected,
            "depth_limit_reference_pixels",
            depthLimitRejected,
            "reciprocal_rejected",
            reciprocalRejected,
            "angle_rejected",
            angleRejected,
            "cache",
            "per-scale depth/plane hypotheses; complete image bytes, masks, relative camera blocks,"
                + " seed map, method and compute settings in key",
            "timing_seconds",
            Json.object(
                "solve_seconds",
                solveNanos / 1e9,
                "solve_host_thread_seconds",
                hostNanos / 1e9,
                "map_io_seconds",
                mapIoNanos / 1e9,
                "reciprocal_evidence_seconds",
                evidenceNanos / 1e9),
            "execution",
            Json.object(
                "fusion_handoff",
                mapHandoff ? "camera maps (05_stereo/patchmatch_maps)" : "pair candidate stream",
                "references_in_flight",
                referencesInFlight,
                "smoothed_depths",
                smoothed,
                "speckle_removed_depths",
                speckled,
                "checkpoint",
                checkpoint == null ? "off" : checkpoint.report(resumedFrom),
                "consistent_views_required",
                options.patchmatchConsistentViews,
                "consistency_rejected",
                consistencyRejected,
                "generations",
                inMemory ? "memory" : "disk",
                "level_rasters_built",
                pyramidBuilt,
                "level_raster_hits",
                pyramidHits)));
    return emitted;
  }

  /**
   * --patchmatch-smooth R: replaces each valid depth of the final maps by the photometric-score
   * weighted mean of the depths that the planes of valid pixels within R pixels (itself included)
   * predict at that pixel, over predictions within the relative tolerance of its own depth (at
   * least 3, else unchanged). Planes are p.w / (n . ray); normals, scores and validity are kept.
   * All maps are smoothed from the unsmoothed values before any is read again.
   */
  private void smoothMaps() throws IOException {
    long changed = 0;
    try (var rows = StereoRows.open(options.cores)) {
      for (ImageFrame f : frames) {
        Stored stored = maps.get(f.id);
        if (stored == null) continue;
        PatchMatch.Map map = stored.map();
        Camera c = f.camera;
        changed +=
            smoothDepths(
                map,
                c.fx,
                c.fy,
                c.cx,
                c.cy,
                options.patchmatchSmooth,
                options.patchmatchSmoothTolerance,
                i -> valid(map, i));
        storeFiltered(f, stored, map);
      }
    }
    smoothed = changed;
  }

  /**
   * Plane-consistent averaging of a map's depths in place (rows in parallel within the caller's
   * StereoRows scope; results do not depend on the row split). Returns the number of changed depths.
   */
  static long smoothDepths(
      PatchMatch.Map map,
      double fx,
      double fy,
      double cx,
      double cy,
      int r,
      double tolerance,
      java.util.function.IntPredicate valid) {
    int w = map.width(), h = map.height();
    float[] d = map.data(), out = new float[w * h];
    long[] count = new long[h];
    StereoRows.rows(
        0,
        h,
        y -> {
          for (int x = 0; x < w; x++) {
            int i = y * w + x;
            out[i] = d[i * 8];
            if (!valid.test(i)) continue;
            double z = d[i * 8], rx = (x - cx) / fx, ry = (y - cy) / fy;
            double sum = 0, weights = 0;
            int support = 0;
            for (int yy = Math.max(0, y - r); yy <= Math.min(h - 1, y + r); yy++)
              for (int xx = Math.max(0, x - r); xx <= Math.min(w - 1, x + r); xx++) {
                int j = yy * w + xx;
                if (!valid.test(j)) continue;
                int at = j * 8;
                double nx = d[at + 1], ny = d[at + 2], nz = d[at + 3];
                double plane = d[at] * (nx * (xx - cx) / fx + ny * (yy - cy) / fy + nz);
                double denominator = nx * rx + ny * ry + nz;
                if (!(denominator > .05)) continue;
                double predicted = plane / denominator;
                if (!(Math.abs(predicted - z) <= tolerance * z)) continue;
                double weight = Math.max(1e-3, map.photo(j));
                sum += weight * predicted;
                weights += weight;
                support++;
              }
            if (support >= 3) {
              out[i] = (float) (sum / weights);
              count[y]++;
            }
          }
        });
    for (int i = 0; i < w * h; i++) d[i * 8] = out[i];
    long changed = 0;
    for (long n : count) changed += n;
    return changed;
  }

  private long smoothed = -1;
  private long speckled = -1;

  /**
   * --patchmatch-speckle N: in every final map, 4-connected regions of valid pixels whose
   * neighbouring depths agree within --patchmatch-speckle-tolerance (relative) are found; regions
   * of fewer than N pixels become invalid (small islands of a wrong plane that propagation grew).
   * All maps are filtered before any is read for evidence.
   */
  private void speckleMaps() throws IOException {
    long removed = 0;
    for (ImageFrame f : frames) {
      Stored stored = maps.get(f.id);
      if (stored == null) continue;
      PatchMatch.Map map = stored.map();
      removed += speckle(map, options.patchmatchSpeckle, options.patchmatchSpeckleTolerance, i -> valid(map, i));
      storeFiltered(f, stored, map);
    }
    speckled = removed;
  }

  /** Invalidates (source slot -1) depth-continuous regions smaller than minimum; returns pixels removed. */
  static long speckle(
      PatchMatch.Map map, int minimum, double tolerance, java.util.function.IntPredicate valid) {
    int w = map.width(), h = map.height();
    float[] d = map.data();
    int[] label = new int[w * h], queue = new int[w * h];
    long removed = 0;
    int next = 0;
    for (int start = 0; start < w * h; start++) {
      if (label[start] != 0 || !valid.test(start)) continue;
      next++;
      int head = 0, tail = 0;
      queue[tail++] = start;
      label[start] = next;
      while (head < tail) {
        int i = queue[head++], x = i % w, y = i / w;
        float z = d[i * 8];
        for (int k = 0; k < 4; k++) {
          int xx = x + (k == 0 ? -1 : k == 1 ? 1 : 0), yy = y + (k == 2 ? -1 : k == 3 ? 1 : 0);
          if (xx < 0 || yy < 0 || xx >= w || yy >= h) continue;
          int j = yy * w + xx;
          if (label[j] != 0 || !valid.test(j)) continue;
          float o = d[j * 8];
          if (!(Math.abs(o - z) <= tolerance * Math.min(o, z))) continue;
          label[j] = next;
          queue[tail++] = j;
        }
      }
      if (tail < minimum) {
        for (int q = 0; q < tail; q++) d[queue[q] * 8 + 5] = -1;
        removed += tail;
      }
    }
    return removed;
  }

  /** --fusion-mode image hand-off (05_stereo/patchmatch_maps), or null. */
  MvsgmMaps handoff;

  /**
   * Writes each reference's accepted pixels as one single-view fusion pair in the mvsgm hand-off
   * format: depth (camera Z), margin, sharpness and photometric score (the weaker of the producing
   * maps), LR = reciprocal reprojection error (NaN = not a candidate) and packed colour. A pixel is
   * a candidate under exactly the pair stream's rules (valid, best source's angle, reciprocal
   * check, positive margin and sharpness); its pair is the reference and its first source.
   */
  private void handoff(List<Long> pairStarts) throws IOException {
    Path dir = options.output.resolve("05_stereo/patchmatch_maps");
    Files.createDirectories(dir);
    handoff = new MvsgmMaps(dir, options.diagnostics, false);
    long count = 0;
    try (var rowWorkers = StereoRows.open(options.cores);
        WorkProgress work =
            WorkProgress.start("PatchMatch map hand-off", maps.size(), "references")) {
      int done = 0;
      for (var f : frames) {
        if (!maps.containsKey(f.id)) continue;
        work.update(done++);
        List<MapReader> opened = new ArrayList<>();
        long[] rowStart;
        int w = f.width, h = f.height;
        int[] selected = sources.get(f.id);
        int pair = fusion.addPair(f.id, selected[0]);
        try {
        MapReader map = open(maps.get(f.id), opened);
        for (int i = 0; i < map.width() * map.height(); i++) {
          if (map.view(i) == -2) geometricRejected++;
          else if (Float.isFinite(map.photo(i)) && map.photo(i) < options.correlation)
            matchingRejected++;
          else if (map.view(i) < 1 || !Float.isFinite(map.depth(i))) unavailableReference++;
          else if (map.depth(i) > options.maxDepth) depthLimitRejected++;
        }
        fusion.markSingleView(pair);
        if (artifacts != null) artifacts.patchmatchPair(pair, f, frames.get(selected[0]));
        while (pairStarts.size() <= pair) pairStarts.add(count);
        MapReader[] checks = new MapReader[selected.length];
        for (int slot = 0; slot < selected.length; slot++)
          if (maps.containsKey(selected[slot])) checks[slot] = open(maps.get(selected[slot]), opened);
        // Streamed in row bands: the six map files are written region by region, so the heap
        // holds one band (about 4 Mpx) instead of six full-frame maps.
        for (int kind = 0; kind < MvsgmMaps.KINDS.length; kind++)
          DepthMapFile.create(handoff.file(f.id, kind), f.camera, w, h);
        rowStart = new long[h + 1];
        int band = Math.max(1, (1 << 22) / Math.max(1, w));
        for (int y0 = 0; y0 < h; y0 += band) {
          int y1 = Math.min(h, y0 + band), rows = y1 - y0, base = y0;
          float[] depth = new float[rows * w], margin = new float[rows * w], sharp = new float[rows * w];
          float[] photo = new float[rows * w], lr = new float[rows * w];
          Arrays.fill(lr, Float.NaN);
          long[] kept = new long[rows], angle = new long[rows], reciprocal = new long[rows];
          long[] ambiguous = new long[rows], inconsistent = new long[rows];
          StereoRows.rows(
              Math.max(2, y0),
              Math.max(Math.max(2, y0), Math.min(y1, h - 2)),
              y -> {
                int r = y - base;
                for (int slot = 0; slot < selected.length; slot++) {
                  if (checks[slot] == null) continue;
                  Evidence row = evidence(f, frames.get(selected[slot]), map, checks[slot], slot, y, selected, checks);
                  angle[r] += row.angle;
                  reciprocal[r] += row.reciprocal;
                  ambiguous[r] += row.ambiguous;
                  inconsistent[r] += row.inconsistent;
                  for (int k = 0; k < row.count; k++) {
                    int at = k * Evidence.STRIDE, i = (int) row.values[at + 3], local = i - base * w;
                    depth[local] = map.depth(i);
                    margin[local] = (float) row.values[at + 5];
                    sharp[local] = (float) row.values[at + 6];
                    photo[local] = (float) row.values[at + 7];
                    lr[local] = (float) row.values[at + 8];
                  }
                  kept[r] += row.count;
                }
              });
          for (int r = 0; r < rows; r++) {
            rowStart[y0 + r + 1] = rowStart[y0 + r] + kept[r];
            angleRejected += angle[r];
            reciprocalRejected += reciprocal[r];
            ambiguityRejected += ambiguous[r];
            consistencyRejected += inconsistent[r];
          }
          byte[] colors = new byte[rows * w * 3];
          synchronized (RasterCache.class) {
            f.ensureRasters();
            Mat part = f.color.submat(y0, y1, 0, w).clone();
            try {
              part.get(0, 0, colors);
            } finally {
              part.release();
            }
          }
          float[] rgb = new float[rows * w];
          for (int i = 0; i < rows * w; i++)
            rgb[i] =
                Float.intBitsToFloat(
                    (colors[i * 3 + 2] & 255) << 16
                        | (colors[i * 3 + 1] & 255) << 8
                        | colors[i * 3] & 255);
          float[][] kinds = {depth, margin, sharp, photo, lr, rgb};
          for (int kind = 0; kind < kinds.length; kind++)
            DepthMapFile.writeRegion(handoff.file(f.id, kind), w, 0, y0, w, rows, kinds[kind], w, 0, 0);
        }
        } finally {
          for (MapReader r : opened) r.close();
        }
        handoff.add(new MvsgmMaps.Camera(f.id, pair, w, h, rowStart, false));
        count += rowStart[h];
      }
      work.done();
    }
    emitted = count;
  }

  /** Accepted pixels of one row: x,y,z world, pixel index, matching, margin, sharpness, photo, lr. */
  private static final class Evidence {
    static final int STRIDE = 9;
    double[] values = new double[0];
    int count, angle, reciprocal, ambiguous, inconsistent;

    void add(double[] world, int i, double... confidence) {
      if ((count + 1) * STRIDE > values.length)
        values = Arrays.copyOf(values, Math.max(STRIDE * 16, values.length * 2));
      int at = count++ * STRIDE;
      values[at] = world[0];
      values[at + 1] = world[1];
      values[at + 2] = world[2];
      values[at + 3] = i;
      System.arraycopy(confidence, 0, values, at + 4, confidence.length);
    }
  }

  private Evidence evidence(
      ImageFrame f,
      ImageFrame other,
      MapReader map,
      MapReader check,
      int slot,
      int y,
      int[] selected,
      MapReader[] all) {
    Evidence row = new Evidence();
    for (int x = 2; x < f.width - 2; x++) {
      int i = y * f.width + x;
      if (map.view(i) != slot + 1) continue;
      if (!valid(map, i)) {
        continue;
      }
      double[] world = world(f.camera, x, y, map.depth(i));
      double angle =
          DenseReconstruction.triangulationAngle(world, f.camera.center(), other.camera.center());
      if (angle < options.minAngle) {
        row.angle++;
        continue;
      }
      double lr = reciprocal(f.camera, other.camera, world, check, options);
      if (!Double.isFinite(lr)) {
        row.reciprocal++;
        continue;
      }
      if (options.patchmatchConsistentViews > 1) {
        // --patchmatch-consistent-views: other source maps must confirm the depth as well (the
        // same reciprocal check), not only the best view.
        int agree = 1;
        for (int s = 0; s < selected.length && agree < options.patchmatchConsistentViews; s++)
          if (s != slot
              && all[s] != null
              && Double.isFinite(
                  reciprocal(f.camera, frames.get(selected[s]).camera, world, all[s], options)))
            agree++;
        if (agree < options.patchmatchConsistentViews) {
          row.inconsistent++;
          continue;
        }
      }
      double[] uv = other.camera.project(world);
      int sourcePixel = (int) Math.round(uv[1]) * check.width() + (int) Math.round(uv[0]);
      double margin = Math.min(map.margin(i), check.margin(sourcePixel));
      double sharpness = Math.min(map.sharpness(i), check.sharpness(sourcePixel));
      double photo = Math.min(map.photo(i), check.photo(sourcePixel));
      if (!(margin > 0 && sharpness > 0)) {
        row.ambiguous++;
        continue;
      }
      double matching = PointConfidence.score(photo, margin, sharpness, lr);
      row.add(world, i, matching, margin, sharpness, photo, lr);
    }
    return row;
  }

  private boolean valid(PatchMatch.Map map, int i) {
    return Float.isFinite(map.depth(i))
        && map.depth(i) > 0
        && map.depth(i) <= options.maxDepth
        && map.view(i) > 0
        && map.photo(i) >= options.correlation;
  }

  /** valid(PatchMatch.Map, i) for a map reader (same tests, same values). */
  private boolean valid(MapReader map, int i) {
    return Float.isFinite(map.depth(i))
        && map.depth(i) > 0
        && map.depth(i) <= options.maxDepth
        && map.view(i) > 0
        && map.photo(i) >= options.correlation;
  }

  /**
   * Read access to a final map: the in-memory map, or a memory-mapped view of its file (big-endian
   * floats after the 8-byte header, the values read() returns). Flat index = pixel * 8 + field.
   */
  interface MapReader extends AutoCloseable {
    int width();

    int height();

    float get(long index);

    default float depth(int i) {
      return get(i * 8L);
    }

    default int view(int i) {
      return (int) get(i * 8L + 5);
    }

    default float photo(int i) {
      return 1 - get(i * 8L + 4);
    }

    default float margin(int i) {
      return get(i * 8L + 6);
    }

    default float sharpness(int i) {
      return get(i * 8L + 7);
    }

    @Override
    default void close() {}
  }

  record HeapMap(PatchMatch.Map map) implements MapReader {
    public int width() {
      return map.width();
    }

    public int height() {
      return map.height();
    }

    public float get(long index) {
      return map.data()[(int) index];
    }
  }

  static final class MappedMap implements MapReader {
    private static final java.lang.foreign.ValueLayout.OfFloat FLOAT =
        java.lang.foreign.ValueLayout.JAVA_FLOAT_UNALIGNED.withOrder(java.nio.ByteOrder.BIG_ENDIAN);
    private final java.lang.foreign.Arena arena = java.lang.foreign.Arena.ofShared();
    private final java.lang.foreign.MemorySegment segment;
    private final int width, height;

    MappedMap(Path file) throws IOException {
      try (var channel = java.nio.channels.FileChannel.open(file, StandardOpenOption.READ)) {
        segment = channel.map(java.nio.channels.FileChannel.MapMode.READ_ONLY, 0, channel.size(), arena);
      } catch (IOException | RuntimeException e) {
        arena.close();
        throw e;
      }
      width = segment.get(java.lang.foreign.ValueLayout.JAVA_INT_UNALIGNED.withOrder(java.nio.ByteOrder.BIG_ENDIAN), 0);
      height = segment.get(java.lang.foreign.ValueLayout.JAVA_INT_UNALIGNED.withOrder(java.nio.ByteOrder.BIG_ENDIAN), 4);
      if (segment.byteSize() != 8 + 32L * width * height) {
        arena.close();
        throw new IOException("PatchMatch map " + file + " has an unexpected size");
      }
    }

    public int width() {
      return width;
    }

    public int height() {
      return height;
    }

    public float get(long index) {
      return segment.get(FLOAT, 8 + 4 * index);
    }

    @Override
    public void close() {
      arena.close();
    }
  }

  /** A reader of a stored final map, added to {@code opened} (closed by the caller). */
  private static MapReader open(Stored stored, List<MapReader> opened) throws IOException {
    MapReader r = stored.map != null ? new HeapMap(stored.map) : new MappedMap(stored.file);
    opened.add(r);
    return r;
  }

  /**
   * --patchmatch-references-in-flight resolved: auto is one reference per 3 cores (at most 4) on
   * the GPU and one per core on the CPU; any value is lowered until the per-reference heap estimate
   * fits 35% of the heap. References of one pass read only the previous generation, so the count
   * never changes a map.
   */
  private int referencesInFlight(Map<Integer, double[]> ranges) {
    int references = ranges.size(), requested = options.patchmatchReferencesInFlight;
    if (references < 2 || requested == 1) return 1;
    int wanted =
        requested > 0
            ? requested
            : options.compute.equals("cpu")
                ? options.cores
                : Math.max(1, Math.min(4, options.cores / 3));
    long perReference = 1;
    for (ImageFrame f : frames)
      if (ranges.containsKey(f.id)) {
        long n = (long) f.width * f.height, atlas = n;
        for (int s : sources.get(f.id)) atlas += (long) frames.get(s).width * frames.get(s).height;
        // Scene atlas, masks and geometry, seed and plane generations, result and previous map.
        perReference = Math.max(perReference, 10 * atlas + 360 * n);
      }
    long fit = Math.max(1, (long) (Runtime.getRuntime().maxMemory() * .35) / perReference);
    if (wanted > fit && requested > 0)
      System.out.printf(
          Locale.ROOT,
          "PatchMatch: --patchmatch-references-in-flight %d lowered to %d: about %,d MiB of heap per"
              + " reference in flight%n",
          requested,
          fit,
          perReference >> 20);
    return (int) Math.max(1, Math.min(Math.min(wanted, fit), references));
  }

  /** Synchronous rounds: every geometric fit reads the same frozen source-map generation. */
  // --dense-checkpoint (PatchMatchCheckpoint); null when not used.
  private PatchMatchCheckpoint checkpoint;
  private Map<Integer, List<Object>> levelReports = new HashMap<>();
  private Map<String, Object> resumedFrom;
  private boolean filteredDone;
  private int solvedReferences;
  // Test aid: fail after this many solved references (simulated interruption).
  private static final int FAIL_AFTER = Integer.getInteger("sgm.patchmatch.failAfter", -1);

  /** A reference's map file of (level, pass) as solve() names it. */
  private Path mapFile(int id, int level, int pass) {
    return options
        .output
        .resolve(String.format(Locale.ROOT, "05_stereo/reference_%04d", id))
        .resolve(level == 0 && pass == 2 ? "depth-normal-evidence.bin" : "level-" + level + "-pass-" + pass + ".bin");
  }

  /** The final map after --patchmatch-smooth/--patchmatch-speckle when checkpointing. */
  private Path filteredFile(int id) {
    return mapFile(id, 0, 2).resolveSibling("depth-normal-evidence.filtered.bin");
  }

  private int[] previousPass(int level, int pass) {
    if (pass > 0) return new int[] {level, pass - 1};
    return level < options.patchmatchLevels - 1 ? new int[] {level + 1, 2} : null;
  }

  private int[] nextPass(int level, int pass) {
    if (pass < 2) return new int[] {level, pass + 1};
    return level > 0 ? new int[] {level - 1, 0} : null;
  }

  /** Checkpoint state: stage, position, finished references of the pass, all report entries. */
  private Map<String, Object> checkpointState(String stage, int level, int pass, Collection<Integer> done) {
    // One entry per pass a reference has completed as far as the checkpoint is concerned: passes
    // before (level, pass), plus this pass for the references in done. A reference that a worker
    // finished beyond done is solved again on resume, so its entry for this pass is left out.
    int before = (options.patchmatchLevels - 1 - level) * 3 + pass;
    Set<Integer> finished = new HashSet<>(done);
    Map<String, Object> reports = new LinkedHashMap<>();
    for (var e : levelReports.entrySet())
      synchronized (e.getValue()) {
        int keep = stage.equals("solve") ? before + (finished.contains(e.getKey()) ? 1 : 0) : e.getValue().size();
        reports.put(String.valueOf(e.getKey()), new ArrayList<>(e.getValue().subList(0, Math.min(keep, e.getValue().size()))));
      }
    Map<String, Object> state = new LinkedHashMap<>();
    state.put("stage", stage);
    state.put("level", level);
    state.put("pass", pass);
    state.put("done", new ArrayList<>(new TreeSet<>(done)));
    state.put("reports", reports);
    return state;
  }

  /** After each solved reference (main thread, frame order): commit when due. */
  private void afterReference(int level, int pass, Set<Integer> done) throws IOException {
    solvedReferences++;
    if (checkpoint != null && checkpoint.due()) checkpoint.commit(checkpointState("solve", level, pass, done));
    if (FAIL_AFTER >= 0 && solvedReferences >= FAIL_AFTER)
      throw new IOException(
          "Simulated interruption after " + solvedReferences + " references (-Dsgm.patchmatch.failAfter)");
  }

  /** A filtered final map: in place in memory; on disk to its file, or with a checkpoint to a
   * separate file (an interrupted filter step is then redone from the unfiltered maps). */
  private void storeFiltered(ImageFrame f, Stored stored, PatchMatch.Map map) throws IOException {
    if (stored.map != null) return;
    Path target = checkpoint != null ? filteredFile(f.id) : stored.file;
    write(target, map.width(), map.height(), map.data());
    if (!target.equals(stored.file)) maps.put(f.id, new Stored(target, null, null));
  }

  /** The dense step finished (fusion included): checkpoint and kept map files go. */
  void complete() throws IOException {
    if (checkpoint == null) return;
    checkpoint.complete();
    if ((!options.diagnostics || options.storage.equals("minimal")) && !KEEP_LEVELS)
      for (int id : maps.keySet()) {
        Files.deleteIfExists(mapFile(id, 0, 2));
        Files.deleteIfExists(filteredFile(id));
      }
  }

  private void estimateMaps() throws IOException {
    long pixels = 0;
    for (ImageFrame f : frames)
      if (f.camera.registered && sources.get(f.id).length > 0)
        pixels = Math.addExact(pixels, (long) f.width * f.height);
    // Two frozen generations of float[8] maps and float[1] trusted-depth sidecars.
    // Final output/depth-evidence storage is additional and is budgeted by the normal pipeline.
    Files.createDirectories(options.output.resolve("05_stereo"));
    long scratch = Math.addExact(Math.multiplyExact(pixels, 72L), 16L << 20);
    long allowance =
        Math.min(options.memoryMb * 1048576L, (long) (Runtime.getRuntime().maxMemory() * .65));
    // Generations stay in memory when both fit in 40% of the processing allowance; otherwise
    // they go through scratch files as before. The values every reader sees are the same.
    inMemory = !FORCE_DISK && scratch <= allowance * .4;
    checkpoint = PatchMatchCheckpoint.open(options, inMemory);
    // A checkpoint refers to map files: generations go to disk (auto uses one only when they do).
    if (checkpoint != null && checkpoint.forcesDisk()) inMemory = false;
    long available = Files.getFileStore(options.output).getUsableSpace();
    if (!inMemory && available > 0 && scratch > available)
      throw new IOException(
          "PatchMatch synchronous map generations need up to "
              + scratch
              + " bytes of scratch storage; available "
              + available);
    Map<Integer, double[]> ranges = new HashMap<>();
    Map<Integer, List<Object>> reports = levelReports;
    for (ImageFrame f : frames)
      if (f.camera.registered && sources.get(f.id).length > 0) {
        ranges.put(f.id, bounds(f));
        reports.put(f.id, new ArrayList<>());
      }
    Map<Integer, Stored> generation = new HashMap<>(), resumed = new HashMap<>();
    int startLevel = options.patchmatchLevels - 1, startPass = 0;
    boolean solved = false;
    Map<String, Object> resume = checkpoint == null ? null : checkpoint.resume();
    if (resume != null) {
      String stage = (String) resume.get("stage");
      int level = ((Number) resume.get("level")).intValue(), pass = ((Number) resume.get("pass")).intValue();
      Map<Integer, Stored> input = new HashMap<>(), finished = new HashMap<>();
      List<Path> needed = new ArrayList<>();
      if (stage.equals("solve")) {
        int[] previous = previousPass(level, pass);
        if (previous != null)
          for (int id : ranges.keySet()) {
            Path file = mapFile(id, previous[0], previous[1]);
            input.put(id, new Stored(file, null, null));
            needed.add(file);
            needed.add(trusted(file));
          }
        for (Object d : (List<?>) resume.get("done")) {
          int id = ((Number) d).intValue();
          Path file = mapFile(id, level, pass);
          finished.put(id, new Stored(file, null, null));
          needed.add(file);
          if (nextPass(level, pass) != null) needed.add(trusted(file));
        }
      } else
        for (int id : ranges.keySet()) {
          Path file = stage.equals("filtered") ? filteredFile(id) : mapFile(id, 0, 2);
          input.put(id, new Stored(file, null, null));
          needed.add(file);
        }
      Map<String, Object> saved = DenseCheckpoint.map(resume, "reports");
      String missing = null;
      for (Path p : needed) if (!Files.isRegularFile(p)) missing = p.toString();
      if (!saved.keySet().equals(new TreeSet<>(ranges.keySet()).stream().map(String::valueOf).collect(java.util.stream.Collectors.toSet())))
        missing = "the references changed";
      if (missing != null) checkpoint.discard("missing " + missing);
      else {
        for (int id : ranges.keySet()) {
          reports.get(id).clear();
          for (Object entry : (List<?>) saved.get(String.valueOf(id))) reports.get(id).add(entry);
        }
        generation = input;
        resumed = finished;
        startLevel = level;
        startPass = pass;
        solved = !stage.equals("solve");
        filteredDone = stage.equals("filtered");
        resumedFrom = Json.object("stage", stage, "level", level, "pass", pass, "references_done", finished.size());
        System.out.printf(Locale.ROOT, "PatchMatch checkpoint found: resuming at %s level %d pass %d (%d references of the pass done)%n",
            stage, level, pass, finished.size());
      }
    }
    referencesInFlight = referencesInFlight(ranges);
    int rowCores = Math.max(1, options.cores / referencesInFlight);
    ExecutorService workers =
        referencesInFlight < 2
            ? null
            : Executors.newFixedThreadPool(
                referencesInFlight,
                r -> {
                  Thread t =
                      new Thread(
                          () -> {
                            try {
                              r.run();
                            } finally {
                              GpuSgm.closeSession();
                            }
                          },
                          "patchmatch-reference");
                  t.setDaemon(true);
                  return t;
                });
    GpuMemoryPool.Scope pool =
        workers == null || options.compute.equals("cpu") || GpuMemoryPool.enabled()
            ? null
            : GpuMemoryPool.install(GpuSgm.budget(GpuSgm.selectedDevice(options), options));
    if (referencesInFlight > 1)
      System.out.printf(
          Locale.ROOT,
          "PatchMatch: %d references in flight, %d row workers each; generations in %s%n",
          referencesInFlight,
          rowCores,
          inMemory ? "memory" : "scratch files");
    try (var rows = workers == null ? StereoRows.open(options.cores) : null) {
      for (int level = startLevel; level >= 0 && !solved; level--) {
        int factor = 1 << level;
        synchronized (pyramid) {
          // Rasters of the coarser level are no longer needed.
          var stale = pyramid.entrySet().iterator();
          while (stale.hasNext()) {
            var entry = stale.next();
            if ((entry.getKey() & 255) != factor) {
              pyramidBytes -= entry.getValue()[0].length * 2L;
              stale.remove();
            }
          }
        }
        for (int pass = level == startLevel ? startPass : 0; pass < 3; pass++) {
          String phase = pass == 0 ? "photometric" : "geometric-" + pass;
          Map<Integer, Stored> next = new HashMap<>(resumed), input = generation;
          resumed = new HashMap<>();
          List<ImageFrame> references = new ArrayList<>();
          for (ImageFrame f : frames) if (ranges.containsKey(f.id) && !next.containsKey(f.id)) references.add(f);
          try (WorkProgress work =
              WorkProgress.start(
                  "PatchMatch level " + level + " " + phase, ranges.size(), "references")) {
            int completed = 0, l = level, p = pass;
            if (workers == null)
              for (ImageFrame f : references) {
                work.detail("reference " + f.id);
                next.put(f.id, solve(f, l, p, phase, input, ranges, reports));
                afterReference(l, p, next.keySet());
                work.update(++completed);
              }
            else {
              List<Future<Stored>> started = new ArrayList<>();
              for (ImageFrame f : references)
                started.add(
                    workers.submit(
                        () -> {
                          try (var own =
                              StereoRows.open(options.compute.equals("cpu") ? rowCores : 1)) {
                            return solve(f, l, p, phase, input, ranges, reports);
                          }
                        }));
              for (int k = 0; k < references.size(); k++) {
                work.detail("reference " + references.get(k).id);
                next.put(references.get(k).id, await(started.get(k)));
                afterReference(l, p, next.keySet());
                work.update(++completed);
              }
            }
            work.done();
          }
          // A checkpoint moves past this pass before the files it referred to are deleted.
          int[] following = nextPass(level, pass);
          if (checkpoint != null && checkpoint.committed())
            checkpoint.commit(
                following == null
                    ? checkpointState("solved", 0, 2, List.of())
                    : checkpointState("solve", following[0], following[1], List.of()));
          // These paths were created above inside this run; no recursive or computed-tree deletion.
          if (!KEEP_LEVELS)
            for (Stored old : generation.values()) {
              Files.deleteIfExists(old.file);
              Files.deleteIfExists(trusted(old.file));
            }
          generation = next;
        }
      }
    } finally {
      if (workers != null) {
        workers.shutdownNow();
        try {
          workers.awaitTermination(1, TimeUnit.MINUTES);
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
        }
      }
      if (pool != null) pool.close();
      synchronized (pyramid) {
        pyramid.clear();
        pyramidBytes = 0;
      }
    }
    maps.putAll(generation);
    for (ImageFrame f : frames)
      if (ranges.containsKey(f.id)) {
        double[] range = ranges.get(f.id);
        Json.write(
            maps.get(f.id).file.resolveSibling("patchmatch.json"),
            Json.object(
                "method",
                "patchmatch",
                "experimental",
                true,
                "estimator_model",
                PatchMatch.MODEL,
                "reference_image",
                f.id,
                "source_images",
                sources.get(f.id),
                "camera_z_minimum",
                range[0],
                "camera_z_maximum",
                range[1],
                "depth_bounds_source",
                options.patchmatchMinDepth > 0
                    ? "explicit"
                    : "aligned sparse points; padded 1st/99th percentiles",
                "requested_minimum_sweeps",
                options.patchmatchIterations,
                "geometric_rounds_per_level",
                2,
                "levels",
                reports.get(f.id),
                "invalid_match",
                "NaN depth and source_slot<1 (-2 denotes geometric contradiction); infinite cost"
                    + " distinguishes missing from poor photometry",
                "binary_format",
                "Big endian int32 width,height then pixel-major float32[8]:"
                    + " camera-Z,nx,ny,nz,1-ZNCC,best_source_slot,margin,sharpness",
                "confidence_competitors",
                "valid alternatives at 2/4/8/16-pixel depth offsets and broad inverse-depth probes;"
                    + " invalid competitors never improve confidence",
                "geometric_evidence",
                "coherent valid source depths; two free-space contradictions invalidate a"
                    + " hypothesis; foreground occlusion and unavailable depth are neutral; source"
                    + " maps are frozen per round",
                "filtering",
                "correlation, LR, minimum angle, depth uncertainty, fusion and confidence controls"
                    + " apply; SGM speckle/median controls do not",
                "view_selection",
                "nearest noncoincident cameras on selected pair graph, bounded source count",
                "fallback",
                "explicit CPU reference only with --compute cpu; GPU failures abort; never"
                    + " substitutes SGM"));
      }
  }

  private static <T> T await(Future<T> future) throws IOException {
    try {
      return future.get();
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      throw new IOException("PatchMatch interrupted", e);
    } catch (ExecutionException e) {
      Throwable cause = e.getCause();
      if (cause instanceof IOException io) throw io;
      if (cause instanceof RuntimeException r) throw r;
      if (cause instanceof Error error) throw error;
      throw new IOException(cause);
    }
  }

  /** One reference of one pass; reads only the frozen previous generation. */
  private Stored solve(
      ImageFrame f,
      int level,
      int pass,
      String phase,
      Map<Integer, Stored> input,
      Map<Integer, double[]> ranges,
      Map<Integer, List<Object>> reports)
      throws IOException {
    long started = System.nanoTime();
    int factor = 1 << level;
    int width = Math.max(1, f.width / factor), height = Math.max(1, f.height / factor);
    PatchMatch.Map previous = input.containsKey(f.id) ? input.get(f.id).map() : null;
    int core = tileCore(f, factor, pass > 0, previous);
    PatchMatch.Map result;
    if (core == 0) {
      PatchMatch.Scene scene = scene(f, sources.get(f.id), factor, ranges.get(f.id));
      if (pass > 0) scene = geometry(scene, sources.get(f.id), input);
      if (factor == 1 && pass == 0 && Integer.getInteger("sgm.patchmatch.dumpScene", -1) == f.id)
        dumpScene(scene, options.output.resolve("05_stereo/scene-" + f.id + ".bin"));
      result = ReuseCache.patchmatch(options, scene, previous);
    } else result = tiled(f, factor, pass > 0, input, ranges.get(f.id), previous, core);
    Path directory =
        options.output.resolve(String.format(Locale.ROOT, "05_stereo/reference_%04d", f.id));
    Files.createDirectories(directory);
    boolean last = level == 0 && pass == 2;
    Path file =
        directory.resolve(
            last ? "depth-normal-evidence.bin" : "level-" + level + "-pass-" + pass + ".bin");
    float[] trusted = null;
    if (!last) {
      trusted = new float[result.width() * result.height()];
      for (int i = 0; i < trusted.length; i++)
        trusted[i] =
            valid(result, i) && result.margin(i) >= .05 && result.sharpness(i) >= .05
                ? result.depth(i)
                : 0;
    }
    long io = System.nanoTime();
    Stored stored;
    boolean keepFinal = options.diagnostics && !options.storage.equals("minimal");
    if (!inMemory || KEEP_LEVELS || last && keepFinal) {
      write(file, result.width(), result.height(), result.data());
      if (trusted != null && (!inMemory || KEEP_LEVELS))
        write(trusted(file), result.width(), result.height(), trusted);
    }
    if (inMemory) {
      float[] data = result.data().clone();
      for (int i = 0; i < data.length; i++) if (Float.isNaN(data[i])) data[i] = Float.NaN;
      stored = new Stored(file, new PatchMatch.Map(result.width(), result.height(), data, null), trusted);
    } else stored = new Stored(file, null, null);
    long finished = System.nanoTime();
    synchronized (this) {
      mapIoNanos += finished - io;
      hostNanos += finished - started;
    }
    List<Object> report = reports.get(f.id);
    synchronized (report) {
      report.add(
          Json.object(
              "level",
              level,
              "phase",
              phase,
              "width",
              width,
              "height",
              height,
              "effective_sweeps",
              PatchMatch.sweeps(width, height, options),
              "seconds",
              (finished - started) / 1e9,
              "backend",
              result.backend()));
    }
    System.out.printf(
        Locale.ROOT,
        "PatchMatch image %d level %d %s: %dx%d, %.2f s%n",
        f.id,
        level,
        phase,
        width,
        height,
        (finished - started) / 1e9);
    return stored;
  }

  private PatchMatch.Scene geometry(
      PatchMatch.Scene scene, int[] selected, Map<Integer, Stored> input) throws IOException {
    float[] depths = new float[scene.pixels().length];
    float[] cameras = scene.cameras();
    for (int slot = 0; slot < selected.length; slot++) {
      Stored stored = input.get(selected[slot]);
      if (stored == null) continue;
      int c = (slot + 1) * 20, width = (int) cameras[c + 16], height = (int) cameras[c + 17];
      int offset = (int) cameras[c + 18] * 65536 + (int) cameras[c + 19];
      if (stored.trusted != null) {
        if (stored.map.width() != width || stored.map.height() != height)
          throw new IOException("Geometric source-map resolution mismatch");
        System.arraycopy(stored.trusted, 0, depths, offset, width * height);
        continue;
      }
      long io = System.nanoTime();
      try (var in = new DataInputStream(Files.newInputStream(trusted(stored.file)))) {
        if (in.readInt() != width || in.readInt() != height)
          throw new IOException("Geometric source-map resolution mismatch");
        readFloats(in, depths, offset, width * height);
      }
      synchronized (this) {
        mapIoNanos += System.nanoTime() - io;
      }
    }
    return new PatchMatch.Scene(
        scene.width(),
        scene.height(),
        scene.pixels(),
        scene.masks(),
        cameras,
        scene.views(),
        scene.minimum(),
        scene.maximum(),
        depths,
        (float) options.fusionRelative,
        (float) options.patchmatchEdgeSigma);
  }

  private static long key(int a, int b) {
    return ((long) a << 32) | (b & 0xffffffffL);
  }

  private static Path trusted(Path file) {
    return file.resolveSibling(file.getFileName() + ".trusted");
  }

  static double[] world(Camera c, double x, double y, double z) {
    return Math3.mv(
        Math3.transpose(c.rotation),
        Math3.sub(new double[] {(x - c.cx) / c.fx * z, (y - c.cy) / c.fy * z, z}, c.translation));
  }

  static double reciprocal(
      Camera reference, Camera source, double[] world, MapReader map, Options o) {
    double[] uv = source.project(world), v = source.view(world);
    if (uv == null
        || uv[0] < 2
        || uv[1] < 2
        || uv[0] >= map.width() - 2
        || uv[1] >= map.height() - 2) return Double.NaN;
    int x = (int) Math.round(uv[0]), y = (int) Math.round(uv[1]), i = y * map.width() + x;
    if (map.view(i) < 1 || map.photo(i) < o.correlation || !(map.depth(i) > 0)) return Double.NaN;
    long at = i * 8L;
    float n1 = map.get(at + 1), n2 = map.get(at + 2), n3 = map.get(at + 3);
    double plane =
        map.depth(i)
            * (n1 * (x - source.cx) / source.fx
                + n2 * (y - source.cy) / source.fy
                + n3);
    double denominator =
        n1 * (uv[0] - source.cx) / source.fx
            + n2 * (uv[1] - source.cy) / source.fy
            + n3;
    double z = plane / denominator;
    if (!(denominator > .05)
        || !Double.isFinite(z)
        || z <= 0
        || Math.abs(z - v[2]) > Math.max(.01, 3 * o.fusionRelative) * Math.min(z, v[2]))
      return Double.NaN;
    double[] back = reference.project(world(source, uv[0], uv[1], z)),
        original = reference.project(world);
    if (back == null || original == null) return Double.NaN;
    double error = Math.hypot(back[0] - original[0], back[1] - original[1]);
    return error <= o.lrThreshold ? error : Double.NaN;
  }

  private double[] bounds(ImageFrame f) {
    if (options.patchmatchMinDepth > 0)
      return new double[] {options.patchmatchMinDepth, options.patchmatchMaxDepth};
    List<Double> z = new ArrayList<>();
    for (var t : tracks)
      if (t.point != null) {
        double[] p = f.camera.project(t.point);
        double d = f.camera.view(t.point)[2];
        if (p != null && d > 0 && p[0] >= 0 && p[1] >= 0 && p[0] < f.width && p[1] < f.height)
          z.add(d);
      }
    if (z.size() < 12)
      throw new IllegalArgumentException(
          "PatchMatch needs at least 12 visible sparse points for image "
              + f.id
              + " or explicit --patchmatch-min-depth and --patchmatch-max-depth (camera-Z,"
              + " reconstruction units)");
    Collections.sort(z);
    return new double[] {
      Math.max(1e-6, z.get((z.size() - 1) / 100) * .75), z.get((z.size() - 1) * 99 / 100) * 1.25
    };
  }

  /**
   * Grey and mask bytes of a frame resized to one level (the same OpenCV calls as before), cached
   * per level within the image-memory budget; the rasters are decoded under the RasterCache lock so
   * a parallel reference cannot evict them mid-resize.
   */
  private byte[][] levelRasters(ImageFrame f, int factor) throws IOException {
    long key = ((long) f.id << 8) | factor;
    synchronized (pyramid) {
      byte[][] hit = pyramid.get(key);
      if (hit != null) {
        pyramidHits++;
        return hit;
      }
    }
    int w = Math.max(1, f.width / factor), h = Math.max(1, f.height / factor);
    byte[] a = new byte[w * h], b = new byte[w * h];
    synchronized (RasterCache.class) {
      f.ensureRasters();
      Mat gray = new Mat(), mask = new Mat();
      try {
        Imgproc.resize(f.gray, gray, new Size(w, h), 0, 0, Imgproc.INTER_AREA);
        if (f.fullMask()) mask.create(h, w, CvType.CV_8U);
        if (f.fullMask()) mask.setTo(new Scalar(255));
        else Imgproc.resize(f.validMask, mask, new Size(w, h), 0, 0, Imgproc.INTER_NEAREST);
        gray.get(0, 0, a);
        mask.get(0, 0, b);
      } finally {
        gray.release();
        mask.release();
      }
    }
    byte[][] made = {a, b};
    synchronized (pyramid) {
      pyramidBuilt++;
      if (pyramid.putIfAbsent(key, made) == null) {
        pyramidBytes += 2L * a.length;
        long limit = RasterCache.limit(options);
        var oldest = pyramid.entrySet().iterator();
        while (pyramidBytes > limit && pyramid.size() > 1 && oldest.hasNext()) {
          var entry = oldest.next();
          if (entry.getKey() == key) continue;
          pyramidBytes -= entry.getValue()[0].length * 2L;
          oldest.remove();
        }
      }
    }
    return made;
  }

  /** Relative pose, level intrinsics, full level size and atlas offset of view f (20 floats). */
  private static void cameraBlock(
      ImageFrame reference, ImageFrame f, int w, int h, float[] cameras, int at, int offset) {
    double[] r = Math3.mul(f.camera.rotation, Math3.transpose(reference.camera.rotation));
    double[] t = Math3.sub(f.camera.translation, Math3.mv(r, reference.camera.translation));
    for (int k = 0; k < 9; k++) cameras[at + k] = (float) r[k];
    for (int k = 0; k < 3; k++) cameras[at + 9 + k] = (float) t[k];
    cameras[at + 12] = (float) (f.camera.fx * w / f.width);
    cameras[at + 13] = (float) (f.camera.fy * h / f.height);
    cameras[at + 14] = (float) ((f.camera.cx + .5) * w / f.width - .5);
    cameras[at + 15] = (float) ((f.camera.cy + .5) * h / f.height - .5);
    cameras[at + 16] = w;
    cameras[at + 17] = h;
    cameras[at + 18] = offset / 65536;
    cameras[at + 19] = offset % 65536;
  }

  static final int TILE_CORE = 2048, MAX_AUTO_HALO = 256, CROP_PAD = 16;

  /**
   * Sum of the sweep strides: a pixel's final plane depends only on pixels within this distance
   * (Jacobi sweeps, per-pixel seeds and finish), so a halo this wide makes a tile's core identical
   * to the untiled solve (when no sample falls outside its source crop).
   */
  static int dependencyRadius(int width, int height, Options o) {
    int radius = 0;
    for (int i = 0; i < PatchMatch.sweeps(width, height, o); i++)
      radius += PatchMatch.stride(width, height, i);
    return radius;
  }

  /** --patchmatch-tile-halo: auto = the dependency radius, at most MAX_AUTO_HALO. */
  int halo(int width, int height) {
    int radius = dependencyRadius(width, height, options);
    return options.patchmatchTileHalo >= 0
        ? options.patchmatchTileHalo
        : Math.min(radius, MAX_AUTO_HALO);
  }

  /**
   * Tile core size for one solve, 0 = untiled (today's path). --patchmatch-tile off never tiles,
   * N tiles every level raster wider or taller than N, auto tiles only when the untiled working set
   * exceeds the memory or GPU limits (where the untiled solve fails). Tiles are GPU-only.
   */
  private int tileCore(ImageFrame f, int factor, boolean geometric, PatchMatch.Map previous) {
    int setting = options.patchmatchTile;
    if (setting == 0) return 0;
    int width = Math.max(1, f.width / factor), height = Math.max(1, f.height / factor);
    if (setting > 0) return Math.max(width, height) > setting ? setting : 0;
    if (options.compute.equals("cpu") || untiledFits(f, factor, geometric, previous)) return 0;
    long budget = GpuSgm.budget(GpuSgm.selectedDevice(options), options);
    int views = 1 + sources.get(f.id).length;
    for (int core = TILE_CORE; core > 256; core /= 2) {
      long side = core + 2L * halo(f.width / factor, f.height / factor), n = side * side;
      // Source crops of a similar footprint, doubled for perspective and depth range.
      if (PatchMatch.tileDeviceBytes(n + 2 * n * (views - 1), n, geometric, views) <= budget / 2)
        return core;
    }
    return 256;
  }

  /** The untiled solve's own limits (scene(), PatchMatch.compute and the device allocation). */
  private boolean untiledFits(
      ImageFrame f, int factor, boolean geometric, PatchMatch.Map previous) {
    long total = Math.max(1, f.width / factor) * (long) Math.max(1, f.height / factor);
    long n = total;
    int views = 1;
    for (int id : sources.get(f.id)) {
      ImageFrame s = frames.get(id);
      total += Math.max(1, s.width / factor) * (long) Math.max(1, s.height / factor);
      views++;
    }
    long allowance =
        Math.min(options.memoryMb * 1048576L, (long) (Runtime.getRuntime().maxMemory() * .65));
    if (total > Integer.MAX_VALUE || 4 * total + 160 * n > allowance) return false;
    long device = (geometric ? 6L : 2L) * total + 64L * n + views * 80L + 8;
    long host = device + 64L * n + (previous == null ? 0 : previous.data().length * 4L);
    if (host > allowance) return false;
    if (options.compute.equals("cpu")) return true;
    var gpu = GpuSgm.selectedDevice(options);
    return device <= GpuSgm.budget(gpu, options)
        && (geometric ? 4 * total : total) <= gpu.allocation()
        && 32 * n <= gpu.allocation();
  }

  /**
   * Bounding box of the window's viewing frustum (depth bounds widened by 10 %) in source view v,
   * padded and clipped to the source raster: x0, y0, width, height. The whole raster when a corner
   * is behind the source camera; a 2 x 2 crop (nothing usable) when the box misses the raster.
   */
  private static int[] frustumCrop(
      float[] cameras, int v, int x0, int y0, int x1, int y1, double zmin, double zmax) {
    int c = v * 20, w = (int) cameras[c + 16], h = (int) cameras[c + 17];
    double minU = Double.POSITIVE_INFINITY, minV = minU, maxU = Double.NEGATIVE_INFINITY;
    double maxV = maxU;
    for (int corner = 0; corner < 8; corner++) {
      double x = (corner & 1) == 0 ? x0 - 3 : x1 + 2, y = (corner & 2) == 0 ? y0 - 3 : y1 + 2;
      double z = (corner & 4) == 0 ? zmin * .9 : zmax * 1.1;
      double rx = (x - cameras[14]) / cameras[12] * z, ry = (y - cameras[15]) / cameras[13] * z;
      double qx = cameras[c] * rx + cameras[c + 1] * ry + cameras[c + 2] * z + cameras[c + 9];
      double qy = cameras[c + 3] * rx + cameras[c + 4] * ry + cameras[c + 5] * z + cameras[c + 10];
      double qz = cameras[c + 6] * rx + cameras[c + 7] * ry + cameras[c + 8] * z + cameras[c + 11];
      if (!(qz > 1e-9)) return new int[] {0, 0, w, h};
      double u = cameras[c + 12] * qx / qz + cameras[c + 14];
      double vv = cameras[c + 13] * qy / qz + cameras[c + 15];
      minU = Math.min(minU, u);
      maxU = Math.max(maxU, u);
      minV = Math.min(minV, vv);
      maxV = Math.max(maxV, vv);
    }
    int ax = (int) Math.max(0, Math.floor(minU) - CROP_PAD);
    int ay = (int) Math.max(0, Math.floor(minV) - CROP_PAD);
    int bx = (int) Math.min(w, Math.ceil(maxU) + CROP_PAD + 1);
    int by = (int) Math.min(h, Math.ceil(maxV) + CROP_PAD + 1);
    if (bx - ax < 2 || by - ay < 2) return new int[] {0, 0, Math.min(2, w), Math.min(2, h)};
    return new int[] {ax, ay, bx - ax, by - ay};
  }

  /**
   * A reference solved in tiles of core x core pixels with a halo (halo()) on every side; each
   * tile keeps only its core. Seeds come from the previous (full) map; geometric passes read crops
   * of the sources' trusted maps. Not cached (ReuseCache keys describe whole scenes).
   */
  private PatchMatch.Map tiled(
      ImageFrame f,
      int factor,
      boolean geometric,
      Map<Integer, Stored> input,
      double[] bounds,
      PatchMatch.Map previous,
      int core)
      throws IOException {
    int width = Math.max(1, f.width / factor), height = Math.max(1, f.height / factor);
    int[] selected = sources.get(f.id);
    List<ImageFrame> views = new ArrayList<>();
    views.add(f);
    for (int id : selected) views.add(frames.get(id));
    float[] cameras = new float[views.size() * 20];
    byte[][][] rasters = new byte[views.size()][][];
    float[][] trusted = new float[views.size()][];
    for (int i = 0; i < views.size(); i++) {
      ImageFrame v = views.get(i);
      int w = Math.max(1, v.width / factor), h = Math.max(1, v.height / factor);
      cameraBlock(f, v, w, h, cameras, i * 20, 0);
      rasters[i] = levelRasters(v, factor);
      if (geometric && i > 0 && input.get(selected[i - 1]) != null)
        trusted[i] = trustedDepths(input.get(selected[i - 1]), w, h);
    }
    float[] data = new float[Math.multiplyExact(Math.multiplyExact(width, height), 8)];
    int tiles = 0, halo = halo(width, height);
    for (int ty = 0; ty < height; ty += core)
      for (int tx = 0; tx < width; tx += core) {
        int x0 = Math.max(0, tx - halo), y0 = Math.max(0, ty - halo);
        int x1 = Math.min(width, tx + core + halo), y1 = Math.min(height, ty + core + halo);
        int[] crops = new int[views.size() * 4];
        int rx = Math.max(0, x0 - 2), ry = Math.max(0, y0 - 2);
        crops[0] = rx;
        crops[1] = ry;
        crops[2] = Math.min(width, x1 + 2) - rx;
        crops[3] = Math.min(height, y1 + 2) - ry;
        for (int v = 1; v < views.size(); v++)
          System.arraycopy(
              frustumCrop(cameras, v, x0, y0, x1, y1, bounds[0], bounds[1]), 0, crops, v * 4, 4);
        long total = 0;
        for (int v = 0; v < views.size(); v++) total += (long) crops[v * 4 + 2] * crops[v * 4 + 3];
        if (total > Integer.MAX_VALUE)
          throw new IllegalArgumentException("PatchMatch tile atlas too large; lower --patchmatch-tile");
        byte[] pixels = new byte[(int) total], masks = new byte[(int) total];
        float[] depths = geometric ? new float[(int) total] : null;
        float[] tileCameras = cameras.clone();
        int offset = 0;
        for (int v = 0; v < views.size(); v++) {
          int cx = crops[v * 4], cy = crops[v * 4 + 1], cw = crops[v * 4 + 2], ch = crops[v * 4 + 3];
          int fullWidth = (int) cameras[v * 20 + 16];
          for (int row = 0; row < ch; row++) {
            int from = (cy + row) * fullWidth + cx;
            System.arraycopy(rasters[v][0], from, pixels, offset + row * cw, cw);
            System.arraycopy(rasters[v][1], from, masks, offset + row * cw, cw);
            if (depths != null && trusted[v] != null)
              System.arraycopy(trusted[v], from, depths, offset + row * cw, cw);
          }
          tileCameras[v * 20 + 18] = offset / 65536;
          tileCameras[v * 20 + 19] = offset % 65536;
          offset += cw * ch;
        }
        PatchMatch.Scene scene =
            new PatchMatch.Scene(
                x1 - x0,
                y1 - y0,
                pixels,
                masks,
                tileCameras,
                views.size(),
                (float) bounds[0],
                (float) bounds[1],
                depths,
                (float) options.fusionRelative,
                (float) options.patchmatchEdgeSigma);
        PatchMatch.Map part =
            PatchMatch.computeTile(
                scene,
                new PatchMatch.Tile(x0, y0, x1 - x0, y1 - y0, width, height, crops),
                options,
                previous);
        int coreWidth = Math.min(width, tx + core) - tx;
        for (int y = ty; y < Math.min(height, ty + core); y++)
          System.arraycopy(
              part.data(),
              ((y - y0) * (x1 - x0) + tx - x0) * 8,
              data,
              (y * width + tx) * 8,
              coreWidth * 8);
        tiles++;
      }
    return new PatchMatch.Map(
        width,
        height,
        data,
        Json.object(
            "backend",
            "opencl_patchmatch_tiled",
            "estimator_model",
            PatchMatch.MODEL,
            "tiles",
            tiles,
            "tile_core",
            core,
            "tile_halo",
            halo,
            "dependency_radius",
            dependencyRadius(width, height, options),
            "effective_sweeps",
            PatchMatch.sweeps(width, height, options),
            "geometric_pass",
            geometric));
  }

  /** A source's trusted depths (memory or its sidecar file), checked against the level size. */
  private static float[] trustedDepths(Stored stored, int width, int height) throws IOException {
    if (stored.trusted != null) {
      if (stored.map.width() != width || stored.map.height() != height)
        throw new IOException("Geometric source-map resolution mismatch");
      return stored.trusted;
    }
    try (var in = new DataInputStream(Files.newInputStream(trusted(stored.file)))) {
      if (in.readInt() != width || in.readInt() != height)
        throw new IOException("Geometric source-map resolution mismatch");
      float[] depths = new float[width * height];
      readFloats(in, depths, 0, depths.length);
      return depths;
    }
  }

  PatchMatch.Scene scene(ImageFrame reference, int[] selected, int factor, double[] bounds)
      throws IOException {
    List<ImageFrame> views = new ArrayList<>();
    views.add(reference);
    for (int id : selected) views.add(frames.get(id));
    long total = 0;
    for (var f : views)
      total += Math.max(1, f.width / factor) * (long) Math.max(1, f.height / factor);
    long n = (long) Math.max(1, reference.width / factor) * Math.max(1, reference.height / factor);
    long host = 4 * total + 160 * n;
    if (total > Integer.MAX_VALUE
        || host
            > Math.min(
                options.memoryMb * 1048576L, (long) (Runtime.getRuntime().maxMemory() * .65)))
      throw new IllegalArgumentException(
          "PatchMatch image/plane working set exceeds --memory-mb/heap; reduce views/resolution");
    byte[] pixels = new byte[(int) total], masks = new byte[(int) total];
    float[] cameras = new float[views.size() * 20];
    int offset = 0;
    for (int i = 0; i < views.size(); i++) {
      var f = views.get(i);
      int w = Math.max(1, f.width / factor), h = Math.max(1, f.height / factor);
      byte[][] level = levelRasters(f, factor);
      System.arraycopy(level[0], 0, pixels, offset, w * h);
      System.arraycopy(level[1], 0, masks, offset, w * h);
      cameraBlock(reference, f, w, h, cameras, i * 20, offset);
      offset += w * h;
    }
    return new PatchMatch.Scene(
        Math.max(1, reference.width / factor),
        Math.max(1, reference.height / factor),
        pixels,
        masks,
        cameras,
        views.size(),
        (float) bounds[0],
        (float) bounds[1],
        null,
        (float) options.fusionRelative,
        (float) options.patchmatchEdgeSigma);
  }

  /** Debug aid (-Dsgm.patchmatch.dumpScene=ID): the reference's level-0 photometric scene. */
  private static void dumpScene(PatchMatch.Scene s, Path file) throws IOException {
    try (var out = new DataOutputStream(new BufferedOutputStream(Files.newOutputStream(file), 1 << 20))) {
      out.writeInt(s.width());
      out.writeInt(s.height());
      out.writeInt(s.views());
      out.writeFloat(s.minimum());
      out.writeFloat(s.maximum());
      out.writeInt(s.pixels().length);
      out.write(s.pixels());
      out.write(s.masks());
      for (float c : s.cameras()) out.writeFloat(c);
    }
  }

  /** Big-endian int32 width, height, then float32 values (Float.floatToIntBits, as before). */
  private static void write(Path file, int width, int height, float[] values) throws IOException {
    try (OutputStream out = Files.newOutputStream(file)) {
      byte[] buffer = new byte[1 << 20];
      ByteBuffer header = ByteBuffer.allocate(8).putInt(width).putInt(height);
      out.write(header.array());
      int used = 0;
      for (float value : values) {
        int bits = Float.floatToIntBits(value);
        buffer[used] = (byte) (bits >>> 24);
        buffer[used + 1] = (byte) (bits >>> 16);
        buffer[used + 2] = (byte) (bits >>> 8);
        buffer[used + 3] = (byte) bits;
        used += 4;
        if (used == buffer.length) {
          out.write(buffer, 0, used);
          used = 0;
        }
      }
      out.write(buffer, 0, used);
    }
  }

  private static void readFloats(DataInputStream in, float[] target, int offset, int count)
      throws IOException {
    byte[] buffer = new byte[1 << 20];
    for (int done = 0; done < count; ) {
      int chunk = Math.min(count - done, buffer.length / 4);
      in.readFully(buffer, 0, chunk * 4);
      ByteBuffer.wrap(buffer, 0, chunk * 4).asFloatBuffer().get(target, offset + done, chunk);
      done += chunk;
    }
  }

  private static PatchMatch.Map read(Path file) throws IOException {
    try (var in = new DataInputStream(Files.newInputStream(file))) {
      int w = in.readInt(), h = in.readInt();
      float[] data = new float[Math.multiplyExact(Math.multiplyExact(w, h), 8)];
      readFloats(in, data, 0, data.length);
      return new PatchMatch.Map(w, h, data, null);
    }
  }
}
