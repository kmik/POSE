package sgm;

import java.io.*;
import java.nio.*;
import java.nio.file.*;
import java.util.*;

/** Disk-partitioned, normal-directed robust fusion. It makes no terrain or axis assumption. */
final class SurfaceConsolidator implements AutoCloseable {
  static final int RECORD_BYTES = 148;

  static final class Sample {
    long id;
    int source, pixel, rgb, views, pairs, merged = 1;
    double[] point, normal;
    SurfaceAttributes attributes = SurfaceAttributes.missing(false);
    double sigma, shift, footprint, normalSigma, surfaceShift;

    Sample(
        long id,
        int source,
        int pixel,
        int rgb,
        DepthFusion.Result r,
        DepthFusion.SurfaceSample s) {
      this.id = id;
      this.source = source;
      this.pixel = pixel;
      this.rgb = rgb;
      point = r.point;
      views = r.views;
      pairs = r.independentPairs;
      sigma = r.relativeSigma;
      shift = r.shift;
      normal = s.normal();
      footprint = s.footprint();
      normalSigma = s.sigma();
      attributes = s.attributes();
    }

    Sample() {}
  }

  interface Sink {
    void point(Sample p) throws IOException;

    default void merged(Sample p, long representative) throws IOException {}
  }

  private static final class Tile {
    final Path path;
    long count;
    long uncompressedBytes;
    double queryFootprint;
    final double[] min = {
      Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY
    };
    final double[] max = {
      Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY
    };

    Tile(Path p) {
      path = p;
    }

    void include(Sample p) {
      count++;
      uncompressedBytes += 108 + p.attributes.storageBytes();
      if (p.normal != null && Double.isFinite(p.footprint))
        queryFootprint = Math.max(queryFootprint, p.footprint);
      for (int k = 0; k < 3; k++) {
        min[k] = Math.min(min[k], p.point[k]);
        max[k] = Math.max(max[k], p.point[k]);
      }
    }

    void includeRaw(ByteBuffer record, int bytes) {
      count++;
      uncompressedBytes += bytes;
      double footprint = record.getDouble(68);
      if (Double.isFinite(record.getDouble(84)) && Double.isFinite(footprint))
        queryFootprint = Math.max(queryFootprint, footprint);
      for (int k = 0; k < 3; k++) {
        double value = record.getDouble(28 + k * 8);
        min[k] = Math.min(min[k], value);
        max[k] = Math.max(max[k], value);
      }
    }
  }

  private record Cell(long x, long y, long z) {}

  private static final class Indices {
    int[] values = new int[16];
    int size;

    void add(int value) {
      if (size == values.length) values = Arrays.copyOf(values, size * 2);
      values[size++] = value;
    }
  }

  private record Neighbor(int index, double offset, double tangent) {}

  private final Options options;
  private final DepthFusion fusion;
  private final Path directory;
  private final Tile root;
  private DataOutputStream spool;
  private final List<Tile> leaves = new ArrayList<>();
  private final int capacity;
  private double largestFootprint;
  private final double[] footprints = new double[8192];
  private int footprintCount;
  private long emitted, merged, moved, unreliable, unchangedBudget, recheckRejected, ambiguous;
  private double maxShift;
  private long temporaryBytes, peakTemporaryBytes, compressedInputBytes;
  private final long started = System.nanoTime();
  private boolean finished;
  private GpuSurface device;
  private Object deviceProfile;
  private long neighborhoodTrials, trialCpuEstimate, trialGpuNanos;
  private long medianNanos;
  private double[] pairWeights;
  private int[] pairSeen;
  private int weightStamp;
  private long splitNanos, loadNanos, gridNanos, neighborhoodNanos, validationNanos, outputNanos;
  private long loadedRecords, haloRecords, decompressedBytes, cacheHits, validationPoints;
  private double maximumHalo;
  private final LinkedHashMap<Path, byte[]> decoded = new LinkedHashMap<>(16, .75f, true);
  private long decodedBytes;
  private final long decodedLimit;

  private record Proposal(int index, List<Neighbor> neighbors, double[] point, double delta) {}

  private record MergeCheck(int anchor, int neighbor) {}

  long mergedCount() {
    return merged;
  }

  long movedCount() {
    return moved;
  }

  SurfaceConsolidator(Options options, DepthFusion fusion) throws IOException {
    this.options = options;
    this.fusion = fusion;
    pairWeights = new double[Math.max(1, fusion.pairCount())];
    pairSeen = new int[pairWeights.length];
    directory = Files.createTempDirectory(options.output, "surface-work-");
    root = new Tile(directory.resolve("accepted.bin.gz"));
    spool = output(root.path);
    // Includes Java object/hash-grid overhead, halo records and sort/query scratch.
    long budget = Math.min(options.surfaceMemoryMb, Math.max(8, options.memoryMb / 4)) * 1048576L;
    decodedLimit = budget / 8;
    capacity = (int) Math.min(Integer.MAX_VALUE / 4, Math.max(256, budget / 768));
  }

  void add(Sample p) throws IOException {
    if (finished) throw new IOException("Surface input is closed");
    write(spool, p);
    root.include(p);
    if (p.footprint > 0 && Double.isFinite(p.footprint)) {
      largestFootprint = Math.max(largestFootprint, p.footprint);
      // Deterministic reservoir, so later views also influence the typical sampling scale.
      long hash = (p.id * 0x9e3779b97f4a7c15L) >>> 1;
      if (footprintCount < footprints.length) footprints[footprintCount++] = p.footprint;
      else {
        long slot = hash % root.count;
        if (slot < footprints.length) footprints[(int) slot] = p.footprint;
      }
    }
  }

  void finish(Sink sink) throws IOException {
    spool.close();
    spool = null;
    finished = true;
    compressedInputBytes = Files.size(root.path);
    temporaryBytes = peakTemporaryBytes = compressedInputBytes;
    if (root.count == 0) {
      report();
      return;
    }
    long clock = System.nanoTime();
    split(root, 0);
    splitNanos += System.nanoTime() - clock;
    Arrays.sort(footprints, 0, footprintCount);
    double typical = footprintCount == 0 ? 1 : footprints[footprintCount / 2];
    double cellSize =
        Math.max(1e-12, typical * (options.surfaceRadius * options.step + options.surfaceMaxShift));

    System.out.printf(
        Locale.ROOT,
        "Surface consolidation: %,d accepted samples; %d spatial partitions; normal-aware median"
            + " and representative sampling%n",
        root.count,
        leaves.size());
    try {
      device = fusion.surfaceDevice(capacity);
    } catch (RuntimeException e) {
      if (options.surfaceValidation.equals("gpu")) throw e;
      System.out.println("Surface GPU neighborhoods unavailable; using CPU: " + e.getMessage());
    }
    long progress = System.nanoTime();
    for (int index = 0; index < leaves.size(); index++) {
      Tile owner = leaves.get(index);
      double halo =
          owner.queryFootprint
              * Math.hypot(options.surfaceRadius * options.step, options.surfaceMaxShift);
      // A normal-directed cylinder lies inside this sphere; round outwards for world-coordinate
      // error.
      halo =
          Math.nextUp(
              halo
                  + 1e-10
                  + 8
                      * Math.ulp(
                          Math.max(
                              Math.abs(owner.min[0]),
                              Math.max(Math.abs(owner.min[1]), Math.abs(owner.min[2])))));
      maximumHalo = Math.max(maximumHalo, halo);
      process(owner, halo, cellSize, sink);
      if (System.nanoTime() - progress > 5_000_000_000L || index + 1 == leaves.size()) {
        System.out.printf(
            Locale.ROOT,
            "Surface consolidation: %d/%d partitions; %,d output, %,d redundant samples combined,"
                + " %,d positions refined%n",
            index + 1,
            leaves.size(),
            emitted,
            merged,
            moved);
        progress = System.nanoTime();
      }
    }
    report();
  }

  private void split(Tile t, int depth) throws IOException {
    if (t.count <= capacity / 4 || depth >= 28) {
      leaves.add(t);
      return;
    }
    int axis = 0;
    for (int k = 1; k < 3; k++) if (t.max[k] - t.min[k] > t.max[axis] - t.min[axis]) axis = k;
    if (t.max[axis] - t.min[axis] < 1e-10) {
      leaves.add(t);
      return;
    }
    double mid = t.min[axis] + (t.max[axis] - t.min[axis]) * .5;
    Tile a = new Tile(Files.createTempFile(directory, "tile-", ".bin.gz")),
        b = new Tile(Files.createTempFile(directory, "tile-", ".bin.gz"));
    try (DataInputStream in = input(t.path);
        DataOutputStream left = output(a.path);
        DataOutputStream right = output(b.path)) {
      byte[] raw = new byte[RECORD_BYTES];
      ByteBuffer record = ByteBuffer.wrap(raw).order(ByteOrder.BIG_ENDIAN);
      for (long i = 0; i < t.count; i++) {
        int bytes = readRecord(in, raw);
        Tile owner = record.getDouble(28 + axis * 8) < mid ? a : b;
        (owner == a ? left : right).write(raw, 0, bytes);
        owner.includeRaw(record, bytes);
      }
    }
    temporaryBytes += Files.size(a.path) + Files.size(b.path);
    peakTemporaryBytes = Math.max(peakTemporaryBytes, temporaryBytes);
    temporaryBytes -= Files.size(t.path);
    Files.delete(t.path);
    if (a.count > 0) split(a, depth + 1);
    else {
      temporaryBytes -= Files.size(a.path);
      Files.delete(a.path);
    }
    if (b.count > 0) split(b, depth + 1);
    else {
      temporaryBytes -= Files.size(b.path);
      Files.delete(b.path);
    }
  }

  private static boolean intersects(Tile a, Tile b, double halo) {
    for (int k = 0; k < 3; k++)
      if (a.max[k] + halo < b.min[k] || a.min[k] - halo > b.max[k]) return false;
    return true;
  }

  private void process(Tile owner, double halo, double cellSize, Sink sink) throws IOException {
    if (owner.count > capacity) {
      passthrough(owner, sink);
      return;
    }
    long clock = System.nanoTime();
    List<Sample> points = new ArrayList<>();
    load(owner, owner, 0, points);
    int owned = points.size();
    boolean overflow = false;
    for (Tile t : leaves)
      if (t != owner && intersects(owner, t, halo)) {
        if (!load(t, owner, halo, points)) {
          overflow = true;
          break;
        }
      }
    loadNanos += System.nanoTime() - clock;
    // Extremely crowded/scale-degenerate neighborhoods remain unchanged rather than being
    // incompletely filtered. This is bounded-memory and explicitly reported, never silent loss.
    if (overflow) {
      passthrough(owner, sink);
      return;
    }
    clock = System.nanoTime();
    if (device != null) {
      try {
        device.partition(points);
      } catch (RuntimeException e) {
        deviceProfile = device.report();
        device.close();
        device = null;
        if (options.surfaceValidation.equals("gpu")) throw e;
        System.out.println("Surface GPU partition unavailable; using CPU: " + e.getMessage());
      }
    }
    Map<Cell, Indices> grid = new HashMap<>();
    for (int i = 0; i < points.size(); i++)
      grid.computeIfAbsent(cell(points.get(i).point, cellSize), k -> new Indices()).add(i);
    Integer[] order = new Integer[owned];
    for (int i = 0; i < owned; i++) order[i] = i;
    Arrays.sort(
        order,
        Comparator.<Integer>comparingInt(i -> -points.get(i).views)
            .thenComparingDouble(i -> points.get(i).footprint)
            .thenComparingLong(i -> points.get(i).id));
    boolean[] covered = new boolean[owned], processed = new boolean[owned];
    gridNanos += System.nanoTime() - clock;
    double cosine = Math.cos(Math.toRadians(options.surfaceNormalAngle));
    // Proposals depend on immutable positions/normals, not greedy covered/processed decisions.
    // Bounded look-ahead batches GPU work; emission/representative selection stays in original
    // order.
    for (int base = 0; base < owned; ) {
      int block = fusion.surfaceBatching() ? Math.max(1, Math.min(64, capacity / 64)) : 1;
      int length = Math.min(block, owned - base);
      Proposal[] proposals = new Proposal[length];
      List<Integer> sources = new ArrayList<>();
      List<double[]> positions = new ArrayList<>();
      int[] checks = new int[length];
      Arrays.fill(checks, -1);
      clock = System.nanoTime();
      List<List<Neighbor>> accelerated =
          device == null
              ? null
              : deviceNeighbors(order, base, length, covered, points, grid, cellSize, cosine);
      for (int k = 0; k < length; k++) {
        int i = order[base + k];
        Sample p = points.get(i);
        if (covered[i] || p.normal == null) continue;
        proposals[k] =
            propose(
                i,
                p,
                points,
                accelerated == null
                    ? neighbors(p, points, grid, cellSize, cosine)
                    : accelerated.get(k));
        Proposal q = proposals[k];
        if (q != null && Math.abs(q.delta) > 1e-12) {
          checks[k] = positions.size();
          sources.add(p.source);
          positions.add(q.point);
        }
      }
      neighborhoodNanos += System.nanoTime() - clock;
      DepthFusion.Result[] results = validate(sources, positions);
      List<MergeCheck> merges = new ArrayList<>();
      sources.clear();
      positions.clear();
      for (int k = 0; k < length; k++) {
        Proposal q = proposals[k];
        if (q == null) continue;
        Sample p = points.get(q.index);
        if (checks[k] >= 0
            && (!results[checks[k]].accepted()
                || !fusion.surfaceDisplacement(p.source, p.point, q.point))) continue;
        for (Neighbor n : q.neighbors) {
          int j = n.index;
          if (j >= owned || j == q.index || covered[j] || processed[j]) continue;
          Sample neighbor = points.get(j);
          if (neighbor.source == p.source) continue;
          double spacing =
              .5
                  * options.surfaceSpacing
                  * options.step
                  * Math.min(p.footprint, neighbor.footprint);
          if (n.tangent > spacing * spacing) continue;
          double offset = SurfaceGeometry.dot(Math3.sub(q.point, neighbor.point), p.normal);
          double[] projected = Math3.add(neighbor.point, Math3.times(p.normal, offset));
          if (!fusion.surfaceDisplacement(neighbor.source, neighbor.point, projected)) continue;
          merges.add(new MergeCheck(k, j));
          sources.add(neighbor.source);
          positions.add(projected);
        }
      }
      DepthFusion.Result[] mergeResults = validate(sources, positions);
      // Preserve original neighbor order within each anchor; candidates processed meanwhile are
      // skipped.
      int mergeAt = 0;
      for (int k = 0; k < length; k++) {
        int i = order[base + k];
        int first = mergeAt;
        while (mergeAt < merges.size() && merges.get(mergeAt).anchor == k) mergeAt++;
        if (covered[i]) continue;
        processed[i] = true;
        Sample p = points.get(i);
        if (p.normal == null) {
          unreliable++;
          emit(p, sink);
          continue;
        }
        Proposal q = proposals[k];
        if (q == null) {
          emit(p, sink);
          continue;
        }
        DepthFusion.Result check = checks[k] < 0 ? null : results[checks[k]];
        if (check != null
            && (!check.accepted() || !fusion.surfaceDisplacement(p.source, p.point, q.point))) {
          recheckRejected++;
          emit(p, sink);
          continue;
        }
        double[] before = p.point;
        if (check != null) {
          p.point = q.point;
          p.surfaceShift = Math.abs(q.delta);
          p.views = check.views;
          p.pairs = check.independentPairs;
          p.sigma = Math.max(p.sigma, check.relativeSigma);
          moved++;
          maxShift = Math.max(maxShift, p.surfaceShift);
        }
        for (int m = first; m < mergeAt; m++) {
          int j = merges.get(m).neighbor;
          if (covered[j] || processed[j] || !mergeResults[m].accepted()) continue;
          covered[j] = true;
          p.merged++;
          merged++;
          sink.merged(points.get(j), p.id);
        }
        emit(p, sink);
        p.point = before;
      }
      base += length;
    }
  }

  private DepthFusion.Result[] validate(List<Integer> sources, List<double[]> positions) {
    if (positions.isEmpty()) return new DepthFusion.Result[0];
    int[] ids = new int[sources.size()];
    for (int i = 0; i < ids.length; i++) ids[i] = sources.get(i);
    long clock = System.nanoTime();
    DepthFusion.Result[] result =
        fusion.checkSurfaceBatch(ids, positions.toArray(double[][]::new), ids.length);
    validationNanos += System.nanoTime() - clock;
    validationPoints += ids.length;
    return result;
  }

  private Proposal propose(int i, Sample p, List<Sample> points, List<Neighbor> nearby) {
    long clock = System.nanoTime();
    try {
      if (nearby.size() < 3) {
        return null;
      }
      nearby.sort(
          Comparator.comparingDouble(Neighbor::offset)
              .thenComparingLong(n -> points.get(n.index).id));
      // Split distinct parallel layers at a resolved gap; never median across both layers.
      double gap = Math.max(.5 * p.footprint, Math.min(p.footprint, p.normalSigma) * 1.5);
      int at = 0;
      for (int j = 0; j < nearby.size(); j++)
        if (nearby.get(j).index == i) {
          at = j;
          break;
        }
      int low = at, high = at;
      while (low > 0 && nearby.get(low).offset - nearby.get(low - 1).offset <= gap) low--;
      while (high + 1 < nearby.size()
          && nearby.get(high + 1).offset - nearby.get(high).offset <= gap) high++;
      if (low > 0 || high + 1 < nearby.size()) ambiguous++;
      nearby = new ArrayList<>(nearby.subList(low, high + 1));
      int stamp = ++weightStamp, pairCount = 0;
      if (stamp == 0) {
        Arrays.fill(pairSeen, 0);
        stamp = ++weightStamp;
      }
      for (Neighbor n : nearby) {
        Sample q = points.get(n.index);
        if (q.source >= pairWeights.length) {
          pairWeights = Arrays.copyOf(pairWeights, Math.max(q.source + 1, pairWeights.length * 2));
          pairSeen = Arrays.copyOf(pairSeen, pairWeights.length);
        }
        if (pairSeen[q.source] != stamp) {
          pairSeen[q.source] = stamp;
          pairWeights[q.source] = 0;
          pairCount++;
        }
        pairWeights[q.source] += weight(q);
      }
      if (nearby.size() < 3 || pairCount < 2) {
        return null;
      }
      double acc = 0, delta = 0;
      for (int k = 0; k < nearby.size(); k++) {
        Neighbor n = nearby.get(k);
        Sample q = points.get(n.index);
        acc += weight(q) / pairWeights[q.source];
        if (acc >= pairCount * .5 - 1e-12) {
          delta = n.offset;
          // Equal-weight even samples have an interval of L1 minimizers. Its midpoint
          // avoids systematically choosing the front or back of a noisy surface.
          if (Math.abs(acc - pairCount * .5) <= 1e-12 && k + 1 < nearby.size())
            delta = (delta + nearby.get(k + 1).offset) * .5;
          break;
        }
      }
      delta =
          Math.max(
              -options.surfaceMaxShift * p.footprint,
              Math.min(options.surfaceMaxShift * p.footprint, delta));

      return new Proposal(i, nearby, Math3.add(p.point, Math3.times(p.normal, delta)), delta);
    } finally {
      medianNanos += System.nanoTime() - clock;
    }
  }

  private List<List<Neighbor>> deviceNeighbors(
      Integer[] order,
      int base,
      int length,
      boolean[] covered,
      List<Sample> points,
      Map<Cell, Indices> grid,
      double cellSize,
      double cosine) {
    long started = System.nanoTime(), cpuEstimate = 0;
    if (options.surfaceValidation.equals("auto") && neighborhoodTrials < 8) {
      int active = 0, sampled = 0;
      long cpu = 0;
      for (int k = 0; k < length; k++) {
        int i = order[base + k];
        if (covered[i] || points.get(i).normal == null) continue;
        active++;
        if (sampled < 2) {
          long clock = System.nanoTime(), budgetBefore = unchangedBudget;
          neighbors(points.get(i), points, grid, cellSize, cosine);
          unchangedBudget = budgetBefore;
          cpu += System.nanoTime() - clock;
          sampled++;
        }
      }
      if (sampled > 0) cpuEstimate = cpu * active / sampled;
      started = System.nanoTime();
    }
    List<List<Neighbor>> out = new ArrayList<>();
    for (int k = 0; k < length; k++) out.add(new ArrayList<>());
    int[] anchors = new int[GpuSurface.BATCH],
        candidates = new int[GpuSurface.BATCH],
        slots = new int[GpuSurface.BATCH];
    int n = 0;
    try {
      for (int k = 0; k < length; k++) {
        int anchor = order[base + k];
        Sample p = points.get(anchor);
        if (covered[anchor] || p.normal == null) continue;
        int reach =
            (int)
                Math.ceil(
                    (options.surfaceRadius * options.step + options.surfaceMaxShift)
                        * p.footprint
                        / cellSize);
        if (reach > 8) {
          unchangedBudget++;
          continue;
        }
        Cell c = cell(p.point, cellSize);
        for (long x = c.x - reach; x <= c.x + reach; x++)
          for (long y = c.y - reach; y <= c.y + reach; y++)
            for (long z = c.z - reach; z <= c.z + reach; z++) {
              Indices members = grid.get(new Cell(x, y, z));
              if (members == null) continue;
              for (int member = 0; member < members.size; member++) {
                int index = members.values[member];
                // Cheap exact CPU broad-phase avoids transferring the much larger cubic-grid
                // candidate set. GPU handles the four camera projections per remaining neighbor.
                Sample q = points.get(index);
                if (q.normal == null || SurfaceGeometry.dot(p.normal, q.normal) < cosine) continue;
                double dx = q.point[0] - p.point[0],
                    dy = q.point[1] - p.point[1],
                    dz = q.point[2] - p.point[2];
                double offset = dx * p.normal[0] + dy * p.normal[1] + dz * p.normal[2];
                double tangent = Math.max(0, dx * dx + dy * dy + dz * dz - offset * offset);
                double radius = options.surfaceRadius * options.step * p.footprint;
                if (Math.abs(offset) > options.surfaceMaxShift * p.footprint
                    || tangent > radius * radius) continue;
                anchors[n] = anchor;
                candidates[n] = index;
                slots[n++] = k;
                if (n == anchors.length) {
                  filterNeighbors(anchors, candidates, slots, n, points, out, cosine);
                  n = 0;
                }
              }
            }
      }
      if (n > 0) filterNeighbors(anchors, candidates, slots, n, points, out, cosine);
      for (List<Neighbor> list : out) trimNeighbors(list, points);
      if (options.surfaceValidation.equals("auto") && neighborhoodTrials < 8) {
        // Warm-up/CPU broad phase/transfer overhead can outweigh four GPU camera projections.
        // Calibration affects backend choice only; predicates and CPU guards remain identical.
        neighborhoodTrials++;
        if (neighborhoodTrials > 2) {
          trialGpuNanos += System.nanoTime() - started;
          trialCpuEstimate += cpuEstimate;
        }
        if (neighborhoodTrials == 8 && trialGpuNanos > trialCpuEstimate) {
          deviceProfile = device.report();
          device.close();
          device = null;
          System.out.println(
              "Surface neighborhoods: CPU selected after timed trials; GPU depth revalidation"
                  + " remains available");
        }
      }
      return out;
    } catch (RuntimeException e) {
      deviceProfile = device.report();
      device.close();
      device = null;
      if (options.surfaceValidation.equals("gpu")) throw e;
      System.out.println(
          "Surface GPU neighborhoods failed; replaying unpublished queries on CPU: "
              + e.getMessage());
      return null;
    }
  }

  private void filterNeighbors(
      int[] anchors,
      int[] candidates,
      int[] slots,
      int n,
      List<Sample> points,
      List<List<Neighbor>> out,
      double cosine) {
    int[] flags =
        device.filter(
            anchors,
            candidates,
            n,
            options.surfaceRadius * options.step,
            options.surfaceMaxShift,
            cosine);
    for (int i = 0; i < Math.min(8, n); i++) {
      int at = i * (n - 1) / Math.max(1, Math.min(8, n) - 1);
      if (flags[at] != 2
          && (cpuNeighbor(
                      points.get(anchors[at]), points.get(candidates[at]), candidates[at], cosine)
                  != null)
              != (flags[at] == 1))
        throw new IllegalStateException("Surface CPU/GPU neighborhood audit disagreed");
      device.audited++;
    }
    for (int i = 0; i < n; i++) {
      if (flags[i] == 0) continue;
      Sample p = points.get(anchors[i]), q = points.get(candidates[i]);
      Neighbor neighbor;
      if (flags[i] == 2) neighbor = cpuNeighbor(p, q, candidates[i], cosine);
      else {
        double dx = q.point[0] - p.point[0],
            dy = q.point[1] - p.point[1],
            dz = q.point[2] - p.point[2];
        double offset = dx * p.normal[0] + dy * p.normal[1] + dz * p.normal[2];
        neighbor =
            new Neighbor(
                candidates[i], offset, Math.max(0, dx * dx + dy * dy + dz * dz - offset * offset));
      }
      if (neighbor != null) out.get(slots[i]).add(neighbor);
    }
    for (List<Neighbor> list : out) if (list.size() > 512) trimNeighbors(list, points);
  }

  private static void trimNeighbors(List<Neighbor> out, List<Sample> points) {
    if (out.size() <= 256) return;
    out.sort(
        Comparator.comparingDouble((Neighbor n) -> n.tangent + n.offset * n.offset)
            .thenComparingLong(n -> points.get(n.index).id));
    out.subList(256, out.size()).clear();
  }

  private Neighbor cpuNeighbor(Sample p, Sample q, int index, double cosine) {
    if (q.normal == null || SurfaceGeometry.dot(p.normal, q.normal) < cosine) return null;
    double dx = q.point[0] - p.point[0], dy = q.point[1] - p.point[1], dz = q.point[2] - p.point[2];
    double offset = dx * p.normal[0] + dy * p.normal[1] + dz * p.normal[2];
    double tangent = Math.max(0, dx * dx + dy * dy + dz * dz - offset * offset);
    double radius = options.surfaceRadius * options.step * p.footprint;
    if (Math.abs(offset) > options.surfaceMaxShift * p.footprint || tangent > radius * radius)
      return null;
    if (!fusion.surfaceDisplacement(
            p.source, p.point, Math3.add(p.point, Math3.times(p.normal, offset)))
        || !fusion.surfaceDisplacement(
            q.source, q.point, Math3.add(q.point, Math3.times(p.normal, -offset)))) return null;
    return new Neighbor(index, offset, tangent);
  }

  private List<Neighbor> neighbors(
      Sample p, List<Sample> points, Map<Cell, Indices> grid, double cellSize, double cosine) {
    double radius = options.surfaceRadius * options.step * p.footprint,
        height = options.surfaceMaxShift * p.footprint;
    int reach = (int) Math.ceil((radius + height) / cellSize);
    Cell c = cell(p.point, cellSize);
    if (reach > 8) {
      unchangedBudget++;
      return List.of();
    }
    List<Neighbor> out = new ArrayList<>();
    for (long x = c.x - reach; x <= c.x + reach; x++)
      for (long y = c.y - reach; y <= c.y + reach; y++)
        for (long z = c.z - reach; z <= c.z + reach; z++) {
          Indices members = grid.get(new Cell(x, y, z));
          if (members == null) continue;
          for (int member = 0; member < members.size; member++) {
            int index = members.values[member];
            Sample q = points.get(index);
            if (q.normal == null || SurfaceGeometry.dot(p.normal, q.normal) < cosine) continue;
            double dx = q.point[0] - p.point[0],
                dy = q.point[1] - p.point[1],
                dz = q.point[2] - p.point[2];
            double offset = dx * p.normal[0] + dy * p.normal[1] + dz * p.normal[2];
            double tangent = Math.max(0, dx * dx + dy * dy + dz * dz - offset * offset);
            if (Math.abs(offset) > height || tangent > radius * radius) continue;
            if (!fusion.surfaceDisplacement(
                    p.source, p.point, Math3.add(p.point, Math3.times(p.normal, offset)))
                || !fusion.surfaceDisplacement(
                    q.source, q.point, Math3.add(q.point, Math3.times(p.normal, -offset))))
              continue;
            out.add(new Neighbor(index, offset, tangent));
          }
        }
    // Bound query scratch even with extreme oversampling; retain the anchor and nearest evidence.
    if (out.size() > 256) {
      out.sort(
          Comparator.comparingDouble((Neighbor n) -> n.tangent + n.offset * n.offset)
              .thenComparingLong(n -> points.get(n.index).id));
      return new ArrayList<>(out.subList(0, 256));
    }
    return out;
  }

  private static double weight(Sample p) {
    return 1 / Math.max(1e-20, p.normalSigma * p.normalSigma);
  }

  private Cell cell(double[] p, double size) {
    return new Cell(
        (long) Math.floor((p[0] - root.min[0]) / size),
        (long) Math.floor((p[1] - root.min[1]) / size),
        (long) Math.floor((p[2] - root.min[2]) / size));
  }

  private boolean load(Tile t, Tile owner, double halo, List<Sample> points) throws IOException {
    try (DataInputStream in = cachedInput(t)) {
      byte[] raw = new byte[RECORD_BYTES];
      ByteBuffer record = ByteBuffer.wrap(raw).order(ByteOrder.BIG_ENDIAN);
      for (long i = 0; i < t.count; i++) {
        int bytes = readRecord(in, raw);
        loadedRecords++;
        boolean keep = t == owner;
        if (!keep) {
          keep = true;
          for (int k = 0; k < 3; k++)
            if (record.getDouble(28 + k * 8) < owner.min[k] - halo
                || record.getDouble(28 + k * 8) > owner.max[k] + halo) {
              keep = false;
              break;
            }
        }
        if (keep) {
          if (points.size() >= capacity) return false;
          points.add(read(new DataInputStream(new ByteArrayInputStream(raw, 0, bytes))));
          if (t != owner) haloRecords++;
        }
      }
    }
    return true;
  }

  /** Preserve full floating-point records while routing; deserialize only retained query data. */
  private static int readRecord(DataInputStream in, byte[] raw) throws IOException {
    in.readFully(raw, 0, 109);
    int status = raw[108] & 255;
    if (status == 0) {
      in.readFully(raw, 109, 37);
      return 146;
    }
    if (status == 1 || status == 2) return 109;
    throw new IOException("Invalid surface record status");
  }

  private DataInputStream cachedInput(Tile t) throws IOException {
    byte[] data = decoded.get(t.path);
    if (data != null) {
      cacheHits++;
      return new DataInputStream(new ByteArrayInputStream(data));
    }
    long bytes = t.uncompressedBytes;
    decompressedBytes += bytes;
    if (bytes > decodedLimit) return input(t.path);
    while (decodedBytes + bytes > decodedLimit && !decoded.isEmpty()) {
      var it = decoded.entrySet().iterator();
      decodedBytes -= it.next().getValue().length;
      it.remove();
    }
    try (DataInputStream in = input(t.path)) {
      data = in.readNBytes(Math.toIntExact(bytes));
      if (data.length != bytes) throw new EOFException("Truncated surface partition");
    }
    decoded.put(t.path, data);
    decodedBytes += data.length;
    return new DataInputStream(new ByteArrayInputStream(data));
  }

  private void passthrough(Tile t, Sink sink) throws IOException {
    try (DataInputStream in = input(t.path)) {
      for (long i = 0; i < t.count; i++) {
        emit(read(in), sink);
        unchangedBudget++;
      }
    }
  }

  private void emit(Sample p, Sink sink) throws IOException {
    long clock = System.nanoTime();
    sink.point(p);
    outputNanos += System.nanoTime() - clock;
    emitted++;
  }

  private void report() throws IOException {
    Json.write(
        options.output.resolve("06_fusion/surface_consolidation.json"),
        Json.object(
            "algorithm",
            "normal-directed median, per-pair balanced weights, bounded spatial partitions",
            "input_samples",
            root.count,
            "output_samples",
            emitted,
            "merged_samples",
            merged,
            "moved_samples",
            moved,
            "unreliable_normals_preserved",
            unreliable,
            "movement_revalidation_rejected",
            recheckRejected,
            "resolved_layer_gaps",
            ambiguous,
            "budget_passthrough_samples",
            unchangedBudget,
            "partitions",
            leaves.size(),
            "working_set_record_capacity",
            capacity,
            "maximum_shift_reconstruction_units",
            maxShift,
            "uncompressed_input_bytes",
            root.uncompressedBytes,
            "compressed_input_bytes",
            compressedInputBytes,
            "peak_temporary_partition_bytes",
            peakTemporaryBytes,
            "temporary_compression_level",
            1,
            "seconds",
            (System.nanoTime() - started) / 1e9,
            "footprint_units",
            "processed image pixels projected at camera depth",
            "normal_radius_pixels",
            new int[] {3, 6, 12},
            "surface_radius_candidate_spacings",
            options.surfaceRadius,
            "representative_spacing_candidate_spacings",
            options.surfaceSpacing,
            "candidate_pixel_step",
            options.step,
            "max_shift_processed_pixels",
            options.surfaceMaxShift,
            "normal_angle_degrees",
            options.surfaceNormalAngle,
            "profile",
            Json.object(
                "partition_seconds",
                splitNanos / 1e9,
                "load_seconds",
                loadNanos / 1e9,
                "grid_seconds",
                gridNanos / 1e9,
                "neighborhood_seconds",
                neighborhoodNanos / 1e9,
                "median_and_layer_seconds",
                medianNanos / 1e9,
                "validation_seconds",
                validationNanos / 1e9,
                "output_seconds",
                outputNanos / 1e9,
                "loaded_records",
                loadedRecords,
                "retained_halo_records",
                haloRecords,
                "decoded_partition_bytes",
                decompressedBytes,
                "decoded_cache_hits",
                cacheHits,
                "decoded_cache_limit_bytes",
                decodedLimit,
                "validation_points",
                validationPoints,
                "largest_global_footprint",
                largestFootprint,
                "maximum_local_halo",
                maximumHalo),
            "surface_validation",
            options.surfaceValidation,
            "neighborhood_device",
            device != null ? device.report() : deviceProfile,
            "neighborhood_backend_trials",
            Json.object(
                "batches",
                neighborhoodTrials,
                "gpu_query_nanoseconds",
                trialGpuNanos,
                "estimated_cpu_query_nanoseconds",
                trialCpuEstimate),
            "scope",
            "Ordered CPU representatives with batched CPU/GPU no-refinement checks; no ground"
                + " flattening"));
  }

  static void write(DataOutputStream out, Sample p) throws IOException {
    out.writeLong(p.id);
    out.writeInt(p.source);
    out.writeInt(p.pixel);
    out.writeInt(p.rgb);
    out.writeInt(p.views);
    out.writeInt(p.pairs);
    for (double v : p.point) out.writeDouble(v);
    out.writeDouble(p.sigma);
    out.writeDouble(p.shift);
    out.writeDouble(p.footprint);
    out.writeDouble(p.normalSigma);
    for (int k = 0; k < 3; k++) out.writeDouble(p.normal == null ? Double.NaN : p.normal[k]);
    p.attributes.write(out);
  }

  static Sample read(DataInputStream in) throws IOException {
    Sample p = new Sample();
    p.id = in.readLong();
    p.source = in.readInt();
    p.pixel = in.readInt();
    p.rgb = in.readInt();
    p.views = in.readInt();
    p.pairs = in.readInt();
    p.point = new double[] {in.readDouble(), in.readDouble(), in.readDouble()};
    p.sigma = in.readDouble();
    p.shift = in.readDouble();
    p.footprint = in.readDouble();
    p.normalSigma = in.readDouble();
    p.normal = new double[] {in.readDouble(), in.readDouble(), in.readDouble()};
    if (!Double.isFinite(p.normal[0])) p.normal = null;
    p.attributes = SurfaceAttributes.read(in);
    return p;
  }

  private static DataInputStream input(Path p) throws IOException {
    return new DataInputStream(
        new BufferedInputStream(
            new java.util.zip.GZIPInputStream(Files.newInputStream(p), 65536), 65536));
  }

  private static DataOutputStream output(Path p) throws IOException {
    return new DataOutputStream(
        new BufferedOutputStream(Compression.gzip(Files.newOutputStream(p), 1), 65536));
  }

  public void close() throws IOException {
    if (device != null) {
      device.close();
      device = null;
    }
    if (spool != null) {
      spool.close();
      spool = null;
    }
    decoded.clear();
    decodedBytes = 0;
    // Only files created in this private temporary directory are removed; no recursive traversal.
    try (var files = Files.list(directory)) {
      for (Path p : files.toList()) Files.deleteIfExists(p);
    }
    Files.deleteIfExists(directory);
  }
}
