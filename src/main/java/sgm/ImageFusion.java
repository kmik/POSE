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
    long pixels, ambiguous, speckle, unsupported, visibility, uncertainty, twoView, notOwner, consumed, emitted, refined;
    long members, depthOutliers;
    long ambiguousKept, pairReads, bytesRead;

    long rejected() {
      return ambiguous + speckle + unsupported + visibility + uncertainty + twoView;
    }

    Object json() {
      return Json.object(
          "merged_valid_pixels", pixels,
          "ambiguous_layer_rejected", ambiguous,
          "ambiguous_nearer_layer_kept", ambiguousKept,
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
          "candidate_bytes_read", bytesRead);
    }
  }

  private final Options o;
  private final DepthFusion fusion;
  private final int cameras, words, fields, record;
  private final Path candidates;
  private final long[] start;
  private final ForkJoinPool pool;
  private final float[][] compact;
  private final int[][] neighbours;
  // owner-consume / graph-average: per camera, merged pixels already represented by an emitted point.
  private final long[][] used;
  // owner-consume: per camera, pixels whose own point was emitted.
  private final long[][] emitted;
  // graph-average: per camera, the merged pixels' inverse-variance weight sums.
  private final float[][] compactWeight;
  private final List<List<Integer>> pairsOf = new ArrayList<>();

  /** One pair's candidates, read from the stream; sinef = f * sin(triangulation angle). */
  private static final class PairData {
    final int pair, count;
    final double[] world;
    final float[] sinef, evidence;
    final int[] rgb, pixel;

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
  }

  /** Merged per-camera depth: 0 = invalid. Offsets are the representative's subpixel position. */
  private static final class Merged {
    final int w, h, words;
    final float[] z, wsum, du, dv;
    final int[] repPair, repIndex;
    final byte[] support;
    final long[] partners;
    final boolean[] ambiguous;

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
      compactBytes += 4 * pixels;
      for (int k : pairsOf.get(c)) pairBytes += (start[k + 1] - start[k]) * (24L + 12 + 4L * fields) + 20 * pixels;
      worstCamera = Math.max(worstCamera, pairBytes + pixels * (34L + 8L * words));
    }
    long needed = compactBytes + worstCamera, budget = (long) (Runtime.getRuntime().maxMemory() * .75);
    if (needed > budget)
      throw new IllegalArgumentException(
          "Image-space fusion needs about " + needed / 1048576 + " MiB heap (compact maps "
              + compactBytes / 1048576 + " MiB + largest camera " + worstCamera / 1048576
              + " MiB); raise PHOTOGRAMMETRY_HEAP/-Xmx or use --fusion-mode reference");
    compact = new float[cameras][];
    neighbours = new int[cameras][];
    used = new long[cameras][];
    emitted = new long[cameras][];
    compactWeight = new float[cameras][];
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
    long first = start[k];
    int n = Math.toIntExact(start[k + 1] - first);
    PairData data = new PairData(k, n, fields, full);
    ByteBuffer buffer = ByteBuffer.allocateDirect(record * 65536);
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
    parallel(n, j -> {
      double[] p = data.point(j), u = Math3.sub(p, centerA), v = Math3.sub(p, centerB);
      double denominator = Math3.norm(u) * Math3.norm(v);
      double x = u[1] * v[2] - u[2] * v[1], y = u[2] * v[0] - u[0] * v[2], z = u[0] * v[1] - u[1] * v[0];
      double sine = denominator == 0 ? 0 : Math.min(1, Math.sqrt(x * x + y * y + z * z) / denominator);
      data.sinef[j] = (float) Math.max(1e-9, focal * sine);
    });
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
    double[][] samples = new double[cameras][];
    long splatNanos = 0, mergeNanos = 0, speckleNanos = 0;
    try (FileChannel channel = FileChannel.open(candidates, StandardOpenOption.READ)) {
      WorkProgress progress = WorkProgress.start("Image fusion cameras", 2L * cameras, "camera passes");
      // Pass 1: filtered merged depth per camera, plus sample points for overlap detection.
      for (int c = 0; c < cameras; c++) {
        progress.update(c);
        if (pairsOf.get(c).isEmpty()) continue;
        long clock = System.nanoTime();
        Splat[] splats = splat(channel, c, false, counters);
        splatNanos += System.nanoTime() - clock;
        clock = System.nanoTime();
        Merged m = merge(c, splats, counters);
        mergeNanos += System.nanoTime() - clock;
        clock = System.nanoTime();
        counters.speckle += speckles(m, fusion.camera(c));
        if (o.fusionDepthOutliers) counters.depthOutliers += depthOutliers(m.z);
        speckleNanos += System.nanoTime() - clock;
        for (float z : m.z) if (z > 0) counters.pixels++;
        compact[c] = m.z;
        if (o.fusionStrategy.equals("graph-average")) compactWeight[c] = m.wsum;
        samples[c] = sample(c, m, splats);
      }
      RunTelemetry.recordTotal("Image fusion: pass 1 splats (pair reads)", splatNanos, cameras);
      RunTelemetry.recordTotal("Image fusion: pass 1 per-camera merge", mergeNanos, cameras);
      RunTelemetry.recordTotal("Image fusion: pass 1 2D speckle filter", speckleNanos, cameras);
      long clock = System.nanoTime();
      for (int c = 0; c < cameras; c++) neighbours[c] = overlapping(c, samples);
      RunTelemetry.record("Image fusion: camera overlap", clock);
      long remergeNanos = 0, checkNanos = 0, outputNanos = 0;
      // Pass 2: re-merge one camera at a time (identical), cross-check, own and emit.
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
        Emission[][] rows = crossCheck(c, m, splats, counters);
        checkNanos += System.nanoTime() - clock;
        clock = System.nanoTime();
        for (Emission[] row : rows)
          if (row != null)
            for (Emission e : row) {
              if (e == null) continue;
              diagnostics.record(e.result);
              if (!e.result.accepted()) continue;
              if (e.result.shift > 1e-8) counters.refined++;
              output.add(e.candidate, e.pair, e.pixel, e.rgb, e.result, e.geometry);
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
        outputNanos += System.nanoTime() - clock;
      }
      RunTelemetry.recordTotal("Image fusion: pass 2 pair reads and re-merge", remergeNanos, cameras);
      RunTelemetry.recordTotal("Image fusion: cross-check, ownership, normals", checkNanos, cameras);
      RunTelemetry.recordTotal("Image fusion: candidate output staging", outputNanos, cameras);
      progress.done();
      progress.close();
    }
    return counters;
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
    for (int s = 0; s < out.length; s++) {
      int k = pairs.get(s);
      PairData data = load(channel, k, full, counters);
      int other = fusion.pairCamera(k, false) == c ? fusion.pairCamera(k, true) : fusion.pairCamera(k, false);
      int[] sample = new int[w * h];
      float[] z = new float[w * h], du = new float[w * h], dv = new float[w * h];
      Arrays.fill(sample, -1);
      // Serial, in candidate order: the nearest depth wins and equal depths keep the first.
      for (int i = 0; i < data.count; i++) {
        double[] v = cam.view(data.point(i));
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
    }
    return out;
  }

  /** Reference depth-mode merge per pixel; ambiguous pixels keep the nearer cluster, flagged. */
  private Merged merge(int c, Splat[] splats, Counters counters) {
    int w = fusion.width(c), h = fusion.height(c);
    Merged m = new Merged(w, h, words);
    double tol = o.fusionRelative, limit = o.fusionPixels * o.fusionPixels;
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
            if (Math.abs(depth[b] - depth[a]) <= tol * Math.min(depth[a], depth[b])) {
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
        for (int b = 0; b < found; b++) {
          if (Math.abs(depth[b] - center) > tol * Math.min(center, depth[b])) continue;
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
  private Emission[][] crossCheck(int c, Merged m, Splat[] splats, Counters counters) {
    int w = m.w, h = m.h, required = fusion.requiredViews();
    Camera cam = fusion.camera(c);
    double[] rotationT = Math3.transpose(cam.rotation);
    double tol = o.fusionRelative;
    boolean robust = o.fusionOutliers.equals("robust");
    boolean surface = o.pointAttributes.equals("surface") || o.surfaceConsolidation.equals("normal");
    double twoView = fusion.twoViewConfidenceThreshold();
    int[] near = neighbours[c];
    int agreementRadius = agreementRadius(o);
    boolean consume = o.fusionStrategy.equals("owner-consume");
    boolean graph = o.fusionStrategy.equals("graph-average");
    long[] taken = consume || graph ? used[c] : null;
    Map<Integer, PairData> byPair = new HashMap<>();
    for (Splat s : splats) byPair.put(s.data().pair, s.data());
    Emission[][] rows = new Emission[h][];
    long[][] tallies = new long[h][7];
    parallel(h, y -> {
      List<Emission> list = null;
      long[] t = tallies[y];
      long[] views = new long[words], confirmed = new long[words];
      long[] agreeAt = new long[near.length];
      float[] agreeDepth = new float[near.length];
      for (int x = 0; x < w; x++) {
        int p = y * w + x;
        float mz = m.z[p];
        if (mz <= 0) continue;
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
        double sumW = graph ? m.wsum[p] : 0, sumWZ = graph ? m.wsum[p] * (double) mz : 0;
        double[] ray = graph ? Math3.mv(rotationT, new double[] {(u - cam.cx) / cam.fx, (v - cam.cy) / cam.fy, 1}) : null;
        Arrays.fill(confirmed, 0);
        for (int k = 0; k < words; k++) views[k] = m.partners.length > 0 ? m.partners[p * words + k] : 0;
        views[c >>> 6] |= 1L << (c & 63);
        long independent = m.support[p];
        int contradictions = 0;
        boolean outsideSupport = false;
        for (int j : near) {
          float[] other = compact[j];
          Camera oc = fusion.camera(j);
          int ow = fusion.width(j), oh = fusion.height(j);
          double[] vj = oc.view(X);
          if (vj[2] <= 0) continue;
          double uj = oc.fx * vj[0] / vj[2] + oc.cx, wj = oc.fy * vj[1] / vj[2] + oc.cy;
          int xj = (int) Math.round(uj), yj = (int) Math.round(wj);
          if (xj < 1 || yj < 1 || xj >= ow - 1 || yj >= oh - 1) continue;
          double zj = other[yj * ow + xj];
          // Merged maps are sparse at step > 1 and broken at canopy gaps, so the exact pixel may
          // be empty or an occluder while the same surface lies one sample away. Agreement is
          // therefore accepted within a small neighbourhood (centre first); contradiction still
          // uses the centre pixel only. Without this, two copies of one surface that miss each
          // other's samples are both "unconfirmed", both own themselves and both emit.
          boolean agrees = zj > 0 && Math.abs(zj - vj[2]) <= tol * Math.min(zj, vj[2]);
          int ax = xj, ay = yj;
          for (int dy = -agreementRadius; !agrees && dy <= agreementRadius; dy++)
            for (int dx = -agreementRadius; !agrees && dx <= agreementRadius; dx++) {
              int xx = xj + dx, yy = yj + dy;
              if ((dx | dy) == 0 || xx < 0 || yy < 0 || xx >= ow || yy >= oh) continue;
              double zn = other[yy * ow + xx];
              agrees = zn > 0 && Math.abs(zn - vj[2]) <= tol * Math.min(zn, vj[2]);
              if (agrees) {
                ax = xx;
                ay = yy;
              }
            }
          if (agrees && consume) {
            agreeAt[agreed] = (long) j << 42 | (long) ay << 21 | ax;
            agreeDepth[agreed++] = (float) vj[2];
            // An earlier camera that agrees already emitted this surface here: this is a copy.
            if (j < c && !duplicate && emitted[j] != null)
              duplicate = emittedNear(j, xj, yj, vj[2], Math.max(agreementRadius, o.step));
          }
          if (agrees && graph) {
            // The member's depth along j's ray at (ax, ay), moved to this pixel's ray: depth in j
            // changes by slope per unit of this camera's depth. Grazing members are not averaged.
            double slope = oc.rotation[6] * ray[0] + oc.rotation[7] * ray[1] + oc.rotation[8] * ray[2];
            double zm = other[ay * ow + ax], wm = compactWeight[j][ay * ow + ax];
            if (slope > .2 && wm > 0) {
              sumW += wm;
              sumWZ += wm * (mz + (zm - vj[2]) / slope);
              agreeAt[agreed] = (long) j << 42 | (long) ay << 21 | ax;
              agreeDepth[agreed++] = (float) vj[2];
            }
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
        double sigma = 1 / Math.sqrt(Math.max(1e-30, m.wsum[p])) / mz;
        if (contradictions >= needed) status = "visibility_contradiction_rejected";
        else if (m.ambiguous[p] && !outsideSupport) status = "depth_consensus_rejected";
        else if (viewCount < required) status = "independent_support_rejected";
        else if (sigma > o.maxDepthUncertainty) status = "depth_uncertainty_rejected";
        if (status.equals("accepted") && duplicate) {
          t[6]++;
          continue;
        }
        if (!status.equals("accepted")) {
          switch (status) {
            case "visibility_contradiction_rejected" -> t[0]++;
            case "depth_consensus_rejected" -> t[1]++;
            case "independent_support_rejected" -> t[2]++;
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
        if (graph && sumW > 0) {
          double zf = sumWZ / sumW;
          X = backProject(cam, rotationT, u, v, zf);
          sigma = 1 / Math.sqrt(sumW) / zf;
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
            surface && r.accepted() ? surfaceSample(c, m, X, mz, x, y) : null;
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
  private DepthFusion.SurfaceSample surfaceSample(int c, Merged m, double[] p, double mz, int x, int y) {
    Camera camera = fusion.camera(c);
    double[] rotationT = Math3.transpose(camera.rotation), center = camera.center();
    double footprint = mz / Math.min(camera.fx, camera.fy);
    double sigma = footprint * .1;
    for (int stride : new int[] {1, 2, 4}) {
      double[][] neighbors = new double[49][];
      int n = 0;
      for (int dy = -3; dy <= 3; dy++)
        for (int dx = -3; dx <= 3; dx++) {
          int xx = x + dx * stride, yy = y + dy * stride;
          if (xx < 0 || yy < 0 || xx >= m.w || yy >= m.h) continue;
          int at = yy * m.w + xx;
          double z = m.z[at];
          if (z <= 0 || Math.abs(z - mz) > o.fusionRelative * Math.min(z, mz)) continue;
          neighbors[n++] = backProject(camera, rotationT, xx + m.du[at], yy + m.dv[at], z);
        }
      SurfaceGeometry.Plane plane = SurfaceGeometry.plane(neighbors, n);
      if (plane == null
          || n < 9
          || plane.middle() < footprint * footprint * .2
          || plane.smallest() > .12 * plane.middle()) continue;
      double[] normal = plane.normal();
      double[] residuals = new double[n];
      for (int i = 0; i < n; i++)
        residuals[i] = Math.abs(SurfaceGeometry.distance(neighbors[i], plane.center(), normal));
      Arrays.sort(residuals);
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
