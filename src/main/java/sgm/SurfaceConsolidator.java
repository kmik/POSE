package sgm;

import java.io.*;
import java.nio.*;
import java.nio.channels.*;
import java.nio.file.*;
import java.util.*;

/** Disk-partitioned, normal-directed robust fusion. It makes no terrain or axis assumption. */
final class SurfaceConsolidator implements AutoCloseable {
  static final int RECORD_BYTES = 172;

  static final class Sample {
    long id;
    int source, pixel, rgb, views, pairs, merged = 1;
    double[] point, normal;
    /** --surface-flatten: plane-fit normal for samples without one; consolidation-only. */
    double[] fitNormal;
    SurfaceAttributes attributes = SurfaceAttributes.missing(false);
    BitSet supportingCameras = new BitSet();
    double sigma, shift, footprint, normalSigma, surfaceShift;
    double matchingConfidence = Double.NaN,
        costMargin = Double.NaN,
        costSharpness = Double.NaN,
        photometric = Double.NaN,
        lrError = Double.NaN;

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
      if (r.supportingCameras != null) supportingCameras = (BitSet) r.supportingCameras.clone();
      sigma = r.relativeSigma;
      matchingConfidence = r.matchingConfidence;
      costMargin = r.costMargin;
      costSharpness = r.costSharpness;
      photometric = r.photometric;
      lrError = r.lrError;
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

    /** --surface-flatten removal: the sample is not published. */
    default void removed(Sample p) throws IOException {}
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
      uncompressedBytes +=
          132 + ((p.supportingCameras.length() + 7) / 8) + p.attributes.storageBytes();
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
  private DataOutputStream spool, coordinates;
  private Path coordinateFile;
  private long coordinateBytes, coordinatePasses, routedRecords;
  private String partitionStrategy;
  private WorkProgress partitionProgress, surfaceProgress;
  private final List<Tile> leaves = new ArrayList<>();
  private final int capacity;
  private final int maxRecordBytes;
  private double largestFootprint;
  private final double[] footprints = new double[8192];
  private int footprintCount;
  private long emitted, merged, moved, unreliable, recheckRejected, isolatedRemoved, fittedNormals;
  // Updated from parallel proposal workers.
  private final java.util.concurrent.atomic.AtomicLong unchangedBudget =
      new java.util.concurrent.atomic.AtomicLong(),
      ambiguous = new java.util.concurrent.atomic.AtomicLong(),
      medianNanos = new java.util.concurrent.atomic.AtomicLong();
  /** CPU look-ahead block when fusion has parallel workers (same output as one at a time). */
  static final int CPU_BLOCK = 512;
  private double maxShift;
  private long temporaryBytes, peakTemporaryBytes, compressedInputBytes;
  private final long started = System.nanoTime();
  private boolean finished;
  private GpuSurface device;
  private Object deviceProfile;
  private long neighborhoodTrials, trialCpuEstimate, trialGpuNanos;
  /** Per-thread pair-weight scratch for propose(). */
  private static final class WeightScratch {
    double[] weights;
    int[] seen;
    int stamp;

    WeightScratch(int pairs) {
      weights = new double[Math.max(1, pairs)];
      seen = new int[weights.length];
    }
  }

  private final ThreadLocal<WeightScratch> weightScratch;
  private long splitNanos, loadNanos, gridNanos, neighborhoodNanos, validationNanos, outputNanos,
      readerNanos;
  private long loadedRecords, haloRecords, decompressedBytes, cacheHits, validationPoints;
  private double maximumHalo;
  private final LinkedHashMap<Path, byte[]> decoded = new LinkedHashMap<>(16, .75f, true);
  private long decodedBytes;
  private final long decodedLimit;

  private record Proposal(
      int index, List<Neighbor> neighbors, double[] point, double delta, boolean remove) {
    Proposal(int index, List<Neighbor> neighbors, double[] point, double delta) {
      this(index, neighbors, point, delta, false);
    }
  }

  /** Source-depth normal, else the --surface-flatten fit (always null when that is off). */
  private static double[] nrm(Sample p) {
    return p.normal != null ? p.normal : p.fitNormal;
  }

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
    maxRecordBytes = RECORD_BYTES + (fusion.cameraCount() + 7) / 8;
    int pairCount = fusion.pairCount();
    weightScratch = ThreadLocal.withInitial(() -> new WeightScratch(pairCount));
    directory = Files.createTempDirectory(options.output, "surface-work-");
    root = new Tile(directory.resolve("accepted.bin.gz"));
    spool = output(root.path);
    partitionStrategy = options.surfacePartition;
    if (partitionStrategy.equals("routed")) {
      coordinateFile = directory.resolve("coordinates.bin");
      coordinates =
          new DataOutputStream(
              new BufferedOutputStream(Files.newOutputStream(coordinateFile), 1048576));
    }
    // Includes Java object/hash-grid overhead, halo records and sort/query scratch.
    long budget = Math.min(options.surfaceMemoryMb, Math.max(8, options.memoryMb / 4)) * 1048576L;
    // Decoded-partition cache only (no effect on partitioning or results): neighbouring
    // partitions reread the same halo files, so allow up to 512 MiB within --memory-mb/16.
    decodedLimit = Math.max(budget / 8, Math.min(512L << 20, options.memoryMb * 1048576L / 16));
    capacity =
        (int)
            Math.min(
                Integer.MAX_VALUE / 4,
                Math.max(256, budget / (768 + (fusion.cameraCount() + 7L) / 8)));
  }

  void add(Sample p) throws IOException {
    if (finished) throw new IOException("Surface input is closed");
    write(spool, p);
    root.include(p);
    if (coordinates != null) {
      coordinates.writeInt(0);
      for (double value : p.point) coordinates.writeDouble(value);
    }
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
    System.out.println("Surface preparation: finalizing accepted-sample spool");
    spool.close();
    spool = null;
    if (coordinates != null) {
      coordinates.close();
      coordinates = null;
      coordinateBytes = Files.size(coordinateFile);
    }
    finished = true;
    compressedInputBytes = Files.size(root.path);
    temporaryBytes = peakTemporaryBytes = compressedInputBytes + coordinateBytes;
    if (root.count == 0) {
      report();
      return;
    }
    long clock = System.nanoTime();
    try (WorkProgress work =
        WorkProgress.start("Surface spatial partition preparation", 0, "record visits")) {
      partitionProgress = work;
      if (partitionStrategy.equals("routed") && root.count > capacity / 4) splitRouted();
      else split(root, 0);
      work.done();
    } finally {
      partitionProgress = null;
    }
    if (coordinateFile != null) Files.deleteIfExists(coordinateFile);
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
    // Image fusion leaves the pair evidence unprepared; freeze it for parallel CPU re-validation.
    long readersStarted = System.nanoTime();
    if (!Boolean.getBoolean("sgm.surfaceNoFreeze")) fusion.prepareSurfaceReaders(root.count);
    readerNanos = System.nanoTime() - readersStarted;
    try {
      // --surface-flatten uses consolidation-only normals that the GPU neighbourhoods lack.
      if (options.surfaceFlatten == 0) device = fusion.surfaceDevice(capacity);
    } catch (RuntimeException e) {
      if (options.surfaceValidation.equals("gpu")) throw e;
      System.out.println("Surface GPU neighborhoods unavailable; using CPU: " + e.getMessage());
    }
    long progress = System.nanoTime();
    try (WorkProgress work =
        WorkProgress.start("Surface consolidation", leaves.size(), "partitions")) {
      surfaceProgress = work;
      for (int index = 0; index < leaves.size(); index++) {
        work.detail("partition " + (index + 1) + "/" + leaves.size() + "; loading halo evidence");
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
        work.update(index + 1);
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
      work.done();
    } finally {
      surfaceProgress = null;
    }
    report();
  }

  private void split(Tile t, int depth) throws IOException {
    if (partitionProgress != null)
      partitionProgress.detail(
          "recursive gzip split depth " + depth + "; completed partitions " + leaves.size());
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
      byte[] raw = new byte[maxRecordBytes];
      ByteBuffer record = ByteBuffer.wrap(raw).order(ByteOrder.BIG_ENDIAN);
      for (long i = 0; i < t.count; i++) {
        int bytes = readRecord(in, raw);
        Tile owner = record.getDouble(28 + axis * 8) < mid ? a : b;
        (owner == a ? left : right).write(raw, 0, bytes);
        owner.includeRaw(record, bytes);
        if (partitionProgress != null && (i & 16383) == 0)
          partitionProgress.update(routedRecords + i);
      }
    }
    routedRecords += t.count;
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

  private static final class SplitNode {
    final double[]
        min =
            new double[] {
              Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY
            },
        max =
            new double[] {
              Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY
            };
    final int depth;
    long count;
    int axis, left = -1, right = -1;
    double mid;
    Tile tile;

    SplitNode(int depth) {
      this.depth = depth;
    }

    void include(double x, double y, double z) {
      count++;
      min[0] = Math.min(min[0], x);
      min[1] = Math.min(min[1], y);
      min[2] = Math.min(min[2], z);
      max[0] = Math.max(max[0], x);
      max[1] = Math.max(max[1], y);
      max[2] = Math.max(max[2], z);
    }
  }

  /** Plan the same recursive midpoints using 28-byte coordinates, then route full records once. */
  private void splitRouted() throws IOException {
    List<SplitNode> nodes = new ArrayList<>();
    SplitNode first = new SplitNode(0);
    first.count = root.count;
    System.arraycopy(root.min, 0, first.min, 0, 3);
    System.arraycopy(root.max, 0, first.max, 0, 3);
    nodes.add(first);
    List<Integer> frontier = new ArrayList<>(List.of(0));
    try (FileChannel file =
        FileChannel.open(coordinateFile, StandardOpenOption.READ, StandardOpenOption.WRITE)) {
      ByteBuffer buffer = ByteBuffer.allocate(28 * 16384).order(ByteOrder.BIG_ENDIAN);
      while (!frontier.isEmpty()) {
        List<Integer> next = new ArrayList<>();
        for (int id : frontier) {
          SplitNode n = nodes.get(id);
          int axis = 0;
          for (int k = 1; k < 3; k++) if (n.max[k] - n.min[k] > n.max[axis] - n.min[axis]) axis = k;
          if (n.count <= capacity / 4 || n.depth >= 28 || n.max[axis] - n.min[axis] < 1e-10)
            continue;
          n.axis = axis;
          n.mid = n.min[axis] + (n.max[axis] - n.min[axis]) * .5;
          n.left = nodes.size();
          nodes.add(new SplitNode(n.depth + 1));
          n.right = nodes.size();
          nodes.add(new SplitNode(n.depth + 1));
          next.add(n.left);
          next.add(n.right);
        }
        if (next.isEmpty()) break;
        long planningLimit =
            Math.max(
                65536L,
                Math.min(options.surfaceMemoryMb, Math.max(8, options.memoryMb / 4))
                    * 1048576L
                    / 8);
        if (nodes.size() * 256L > planningLimit) {
          partitionStrategy = "recursive_planning_budget_fallback";
          System.out.println(
              "Surface partition plan exceeds bounded metadata allowance; using recursive"
                  + " reference");
          split(root, 0);
          return;
        }
        coordinatePasses++;
        partitionProgress.detail(
            "coordinate planning pass " + coordinatePasses + "; nodes " + nodes.size());
        long at = 0, visits = 0, coordinateLength = file.size();
        while (at < coordinateLength) {
          buffer.clear();
          buffer.limit((int) Math.min(buffer.capacity(), coordinateLength - at));
          int wanted = buffer.limit();
          readFully(file, buffer, at);
          buffer.flip();
          for (int p = 0; p < wanted; p += 28) {
            SplitNode n = nodes.get(buffer.getInt(p));
            if (n.left < 0) continue;
            double x = buffer.getDouble(p + 4),
                y = buffer.getDouble(p + 12),
                z = buffer.getDouble(p + 20);
            double value = n.axis == 0 ? x : n.axis == 1 ? y : z;
            int child = value < n.mid ? n.left : n.right;
            buffer.putInt(p, child);
            nodes.get(child).include(x, y, z);
          }
          buffer.position(0);
          writeFully(file, buffer, at);
          at += wanted;
          visits += wanted / 28;
          partitionProgress.update(routedRecords + visits);
        }
        routedRecords += root.count;
        frontier = next;
      }
    }
    if (nodes.size() == 1) {
      leaves.add(root);
      return;
    }
    addLeaves(nodes, 0);
    partitionProgress.detail("routing full samples once; final partitions " + leaves.size());
    // Records arrive in fusion order, scattered over all partitions. Buffer them per partition
    // and append large gzip members (same records, same order per partition) instead of
    // reopening a stream for nearly every record.
    Map<Tile, ByteArrayOutputStream> pending = new LinkedHashMap<>();
    long pendingBytes = 0;
    final int tileFlush = 1 << 20;
    final long totalFlush = 64L << 20;
    Compression.GzipPool gzipPool = new Compression.GzipPool();
    try (DataInputStream in = input(root.path);
        FileChannel ids = FileChannel.open(coordinateFile)) {
      if (ids.size() != Math.multiplyExact(root.count, 28L))
        throw new IOException("Surface coordinate count disagrees");
      ByteBuffer routing = ByteBuffer.allocate(28 * 16384).order(ByteOrder.BIG_ENDIAN);
      routing.limit(0);
      byte[] raw = new byte[maxRecordBytes];
      ByteBuffer record = ByteBuffer.wrap(raw).order(ByteOrder.BIG_ENDIAN);
      for (long i = 0; i < root.count; i++) {
        if (!routing.hasRemaining()) {
          routing.clear();
          routing.limit((int) Math.min(routing.capacity(), (root.count - i) * 28L));
          readFully(ids, routing, i * 28L);
          routing.flip();
        }
        int id = routing.getInt();
        routing.position(routing.position() + 24);
        Tile tile = nodes.get(id).tile;
        if (tile == null) throw new IOException("Invalid final spatial partition id");
        ByteArrayOutputStream buffer =
            pending.computeIfAbsent(tile, t -> new ByteArrayOutputStream(65536));
        int bytes = readRecord(in, raw);
        buffer.write(raw, 0, bytes);
        pendingBytes += bytes;
        tile.includeRaw(record, bytes);
        if (buffer.size() >= tileFlush) {
          pendingBytes -= buffer.size();
          appendMember(gzipPool, tile, buffer);
        }
        if (pendingBytes >= totalFlush) {
          for (var e : pending.entrySet()) appendMember(gzipPool, e.getKey(), e.getValue());
          pendingBytes = 0;
        }
        if ((i & 16383) == 0) {
          partitionProgress.update(routedRecords + i);
          if ((i & 1048575) == 0) updateTemporaryPeak();
        }
      }
      if (in.read() != -1 || ids.size() != root.count * 28L)
        throw new IOException("Surface coordinate/record counts disagree");
    } finally {
      IOException failure = null;
      for (var e : pending.entrySet())
        try {
          appendMember(gzipPool, e.getKey(), e.getValue());
        } catch (IOException x) {
          failure = x;
        }
      gzipPool.close();
      if (failure != null) throw failure;
    }
    routedRecords += root.count;
    updateTemporaryPeak();
    Files.delete(root.path);
    temporaryBytes = 0;
    for (Tile tile : leaves) temporaryBytes += Files.size(tile.path);
  }

  /** Appends buffered records to a partition file as one gzip member and empties the buffer. */
  private static void appendMember(
      Compression.GzipPool pool, Tile tile, ByteArrayOutputStream buffer) throws IOException {
    if (buffer.size() == 0) return;
    try (OutputStream out =
        pool.open(Files.newOutputStream(tile.path, StandardOpenOption.APPEND))) {
      buffer.writeTo(out);
    }
    buffer.reset();
  }

  private void addLeaves(List<SplitNode> nodes, int id) throws IOException {
    SplitNode n = nodes.get(id);
    if (n.count == 0) return;
    if (n.left >= 0) {
      addLeaves(nodes, n.left);
      addLeaves(nodes, n.right);
    } else {
      n.tile = new Tile(Files.createTempFile(directory, "tile-", ".bin.gz"));
      leaves.add(n.tile);
    }
  }

  private void updateTemporaryPeak() throws IOException {
    long bytes = 0;
    try (var files = Files.list(directory)) {
      for (Path p : files.toList()) bytes += Files.size(p);
    }
    peakTemporaryBytes = Math.max(peakTemporaryBytes, bytes);
  }

  private static void readFully(FileChannel file, ByteBuffer b, long offset) throws IOException {
    while (b.hasRemaining()) {
      int n = file.read(b, offset + b.position());
      if (n < 0) throw new EOFException("Truncated partition coordinates");
      if (n == 0) throw new IOException("Coordinate read made no progress");
    }
  }

  private static void writeFully(FileChannel file, ByteBuffer b, long offset) throws IOException {
    while (b.hasRemaining())
      if (file.write(b, offset + b.position()) == 0)
        throw new IOException("Coordinate write made no progress");
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
    prefetch(owner, halo);
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
    if (surfaceProgress != null)
      surfaceProgress.detail("building grid and ordered representatives");
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
    if (options.surfaceFlatten > 0) {
      // Fits read only positions and source-depth normals, so they are order independent.
      double[][] fitted = new double[points.size()][];
      final Map<Cell, Indices> fitGrid = grid;
      fusion.parallel(
          points.size(),
          i -> {
            Sample p = points.get(i);
            if (p.normal == null) fitted[i] = fitNormal(p, points, fitGrid, cellSize);
          });
      for (int i = 0; i < fitted.length; i++) {
        points.get(i).fitNormal = fitted[i];
        if (fitted[i] != null && i < owned) fittedNormals++;
      }
    }
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
      if (surfaceProgress != null && (base & 1023) == 0)
        surfaceProgress.detail(
            "neighborhood/refinement checks; "
                + base
                + "/"
                + owned
                + " owner samples; output "
                + emitted);
      // Look-ahead blocks give the same output as one sample at a time (proposals read only
      // immutable positions/normals/normal sigmas; emission re-checks covered/processed in
      // order). On the CPU path they let validation and proposals use all fusion workers.
      int block =
          fusion.surfaceBatching()
              ? Math.max(1, Math.min(options.surfaceBatchSize, capacity / 256))
              : fusion.cpuParallel() && !Boolean.getBoolean("sgm.surfaceSerial")
                  ? Math.max(1, Math.min(CPU_BLOCK, capacity / 256))
                  : 1;
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
      final int blockStart = base;
      java.util.function.IntConsumer proposeOne =
          k -> {
            int i = order[blockStart + k];
            Sample p = points.get(i);
            if (covered[i] || nrm(p) == null) return;
            if (options.surfaceFlatten > 0 && isolated(i, p, points, grid, cellSize, cosine)) {
              proposals[k] = new Proposal(i, List.of(), p.point, 0, true);
              return;
            }
            proposals[k] =
                propose(
                    i,
                    p,
                    points,
                    accelerated == null
                        ? neighbors(p, points, grid, cellSize, cosine)
                        : accelerated.get(k));
          };
      if (accelerated == null) fusion.parallel(length, proposeOne);
      else for (int k = 0; k < length; k++) proposeOne.accept(k);
      for (int k = 0; k < length; k++) {
        Proposal q = proposals[k];
        if (q != null && Math.abs(q.delta) > 1e-12) {
          checks[k] = positions.size();
          sources.add(points.get(q.index).source);
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
          double[] anchorNormal = nrm(p);
          double offset = SurfaceGeometry.dot(Math3.sub(q.point, neighbor.point), anchorNormal);
          double[] projected = Math3.add(neighbor.point, Math3.times(anchorNormal, offset));
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
        if (nrm(p) == null) {
          unreliable++;
          emit(p, sink);
          continue;
        }
        Proposal q = proposals[k];
        if (q == null) {
          emit(p, sink);
          continue;
        }
        if (q.remove) {
          // --surface-flatten: isolated off a dominant surface; neither cloud nor record row.
          isolatedRemoved++;
          sink.removed(p);
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
          if (check.supportingCameras != null)
            p.supportingCameras = (BitSet) check.supportingCameras.clone();
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
      if (low > 0 || high + 1 < nearby.size()) ambiguous.incrementAndGet();
      nearby = new ArrayList<>(nearby.subList(low, high + 1));
      WeightScratch s = weightScratch.get();
      int stamp = ++s.stamp, pairCount = 0;
      if (stamp == 0) {
        Arrays.fill(s.seen, 0);
        stamp = ++s.stamp;
      }
      for (Neighbor n : nearby) {
        Sample q = points.get(n.index);
        if (q.source >= s.weights.length) {
          s.weights = Arrays.copyOf(s.weights, Math.max(q.source + 1, s.weights.length * 2));
          s.seen = Arrays.copyOf(s.seen, s.weights.length);
        }
        if (s.seen[q.source] != stamp) {
          s.seen[q.source] = stamp;
          s.weights[q.source] = 0;
          pairCount++;
        }
        s.weights[q.source] += weight(q);
      }
      double[] pairWeights = s.weights;
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

      return new Proposal(i, nearby, Math3.add(p.point, Math3.times(nrm(p), delta)), delta);
    } finally {
      medianNanos.addAndGet(System.nanoTime() - clock);
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
          long clock = System.nanoTime(), budgetBefore = unchangedBudget.get();
          neighbors(points.get(i), points, grid, cellSize, cosine);
          unchangedBudget.set(budgetBefore);
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
          unchangedBudget.incrementAndGet();
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

  /**
   * --surface-flatten: true when p is isolated off a dominant surface. Neighbours within twice
   * the tangential radius and K footprints along p's normal (compatible normals, no displacement
   * guard) are split into layers at the usual gap. p's own layer is weak evidence (at most 2
   * samples, or all from a single stereo pair) while another layer has >= 6 samples from >= 2
   * pairs, at least 3x p's layer and more than half of all of them.
   */
  private boolean isolated(
      int i, Sample p, List<Sample> points, Map<Cell, Indices> grid, double cellSize, double cosine) {
    double[] pn = nrm(p);
    double radius = 2 * options.surfaceRadius * options.step * p.footprint,
        height = options.surfaceFlatten * p.footprint;
    int reach = (int) Math.ceil((radius + height) / cellSize);
    if (pn == null || reach > 8) return false;
    Cell c = cell(p.point, cellSize);
    List<Neighbor> all = new ArrayList<>();
    for (long x = c.x - reach; x <= c.x + reach; x++)
      for (long y = c.y - reach; y <= c.y + reach; y++)
        for (long z = c.z - reach; z <= c.z + reach; z++) {
          Indices members = grid.get(new Cell(x, y, z));
          if (members == null) continue;
          for (int member = 0; member < members.size; member++) {
            int index = members.values[member];
            Sample q = points.get(index);
            double[] qn = nrm(q);
            if (qn == null || SurfaceGeometry.dot(pn, qn) < cosine) continue;
            double dx = q.point[0] - p.point[0],
                dy = q.point[1] - p.point[1],
                dz = q.point[2] - p.point[2];
            double offset = dx * pn[0] + dy * pn[1] + dz * pn[2];
            double tangent = Math.max(0, dx * dx + dy * dy + dz * dz - offset * offset);
            if (Math.abs(offset) > height || tangent > radius * radius) continue;
            all.add(new Neighbor(index, offset, tangent));
          }
        }
    if (all.size() < 7) return false;
    all.sort(
        Comparator.comparingDouble(Neighbor::offset)
            .thenComparingLong(n -> points.get(n.index).id));
    double gap = Math.max(.5 * p.footprint, Math.min(p.footprint, p.normalSigma) * 1.5);
    int own = 0, ownPairs = 0, dominant = 0, dominantPairs = 0;
    for (int s = 0; s < all.size(); ) {
      int e = s;
      while (e + 1 < all.size() && all.get(e + 1).offset - all.get(e).offset <= gap) e++;
      boolean containsAnchor = false;
      BitSet sources = new BitSet();
      for (int k = s; k <= e; k++) {
        if (all.get(k).index == i) containsAnchor = true;
        sources.set(points.get(all.get(k).index).source);
      }
      int size = e - s + 1;
      if (containsAnchor) {
        own = size;
        ownPairs = sources.cardinality();
      } else if (size > dominant) {
        dominant = size;
        dominantPairs = sources.cardinality();
      }
      s = e + 1;
    }
    return own > 0
        && (own <= 2 || ownPairs <= 1)
        && dominant >= 6
        && dominantPairs >= 2
        && dominant >= 3 * own
        && 2 * dominant > all.size();
  }

  /**
   * --surface-flatten: robust plane-fit normal from the 3D neighbourhood of a sample without a
   * source-depth normal (sphere of max(radius, 3) footprints; trimmed PCA; oriented like the
   * neighbours' normals). Null when the neighbourhood is not clearly planar.
   */
  private double[] fitNormal(Sample p, List<Sample> points, Map<Cell, Indices> grid, double cellSize) {
    double rho = Math.max(options.surfaceRadius * options.step, 3) * p.footprint;
    int reach = (int) Math.ceil(rho / cellSize);
    if (!(rho > 0) || reach > 8) return null;
    Cell c = cell(p.point, cellSize);
    List<double[]> near = new ArrayList<>();
    double[] reference = new double[3];
    int valid = 0;
    for (long x = c.x - reach; x <= c.x + reach; x++)
      for (long y = c.y - reach; y <= c.y + reach; y++)
        for (long z = c.z - reach; z <= c.z + reach; z++) {
          Indices members = grid.get(new Cell(x, y, z));
          if (members == null) continue;
          for (int member = 0; member < members.size; member++) {
            Sample q = points.get(members.values[member]);
            double[] d = Math3.sub(q.point, p.point);
            if (d[0] * d[0] + d[1] * d[1] + d[2] * d[2] > rho * rho) continue;
            near.add(d);
            if (q.normal != null) {
              reference = Math3.add(reference, q.normal);
              valid++;
            }
          }
        }
    int n = near.size();
    double referenceLength = Math3.norm(reference);
    if (!(referenceLength > 0)) return null;
    // Fallback: the neighbours' own normals agree closely (mean resultant >= 0.9).
    double[] fallback =
        valid >= 6 && referenceLength >= .9 * valid ? Math3.times(reference, 1 / referenceLength) : null;
    if (n < 9) return fallback;
    double[][] pts = near.toArray(double[][]::new);
    SurfaceGeometry.Plane plane = SurfaceGeometry.plane(pts, n);
    if (plane == null) return fallback;
    double[] residuals = new double[n];
    for (int k = 0; k < n; k++)
      residuals[k] = Math.abs(SurfaceGeometry.distance(pts[k], plane.center(), plane.normal()));
    double[] sorted = residuals.clone();
    Arrays.sort(sorted);
    double band = Math.max(.5 * p.footprint, 3 * 1.4826 * sorted[n / 2]);
    int kept = 0;
    for (int k = 0; k < n; k++) if (residuals[k] <= band) pts[kept++] = pts[k];
    if (kept < .6 * n) return fallback;
    plane = SurfaceGeometry.plane(pts, kept);
    if (plane == null || plane.smallest() > .12 * plane.middle()) return fallback;
    double[] normal = plane.normal();
    double agreement = SurfaceGeometry.dot(normal, reference) / referenceLength;
    if (agreement < 0) {
      normal = Math3.times(normal, -1);
      agreement = -agreement;
    }
    return agreement >= .5 ? normal : fallback;
  }

  private List<Neighbor> neighbors(
      Sample p, List<Sample> points, Map<Cell, Indices> grid, double cellSize, double cosine) {
    double radius = options.surfaceRadius * options.step * p.footprint,
        height = options.surfaceMaxShift * p.footprint;
    int reach = (int) Math.ceil((radius + height) / cellSize);
    Cell c = cell(p.point, cellSize);
    if (reach > 8) {
      unchangedBudget.incrementAndGet();
      return List.of();
    }
    List<Neighbor> out = new ArrayList<>();
    double[] pn = nrm(p);
    for (long x = c.x - reach; x <= c.x + reach; x++)
      for (long y = c.y - reach; y <= c.y + reach; y++)
        for (long z = c.z - reach; z <= c.z + reach; z++) {
          Indices members = grid.get(new Cell(x, y, z));
          if (members == null) continue;
          for (int member = 0; member < members.size; member++) {
            int index = members.values[member];
            Sample q = points.get(index);
            double[] qn = nrm(q);
            if (qn == null || SurfaceGeometry.dot(pn, qn) < cosine) continue;
            double dx = q.point[0] - p.point[0],
                dy = q.point[1] - p.point[1],
                dz = q.point[2] - p.point[2];
            double offset = dx * pn[0] + dy * pn[1] + dz * pn[2];
            double tangent = Math.max(0, dx * dx + dy * dy + dz * dz - offset * offset);
            if (Math.abs(offset) > height || tangent > radius * radius) continue;
            if (!fusion.surfaceDisplacement(
                    p.source, p.point, Math3.add(p.point, Math3.times(pn, offset)))
                || !fusion.surfaceDisplacement(
                    q.source, q.point, Math3.add(q.point, Math3.times(pn, -offset))))
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
      byte[] raw = new byte[maxRecordBytes];
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
    in.readFully(raw, 0, 132);
    int supportBytes = ByteBuffer.wrap(raw).getInt(128);
    if (supportBytes < 0 || supportBytes > raw.length - 170)
      throw new IOException("Invalid camera support length");
    in.readFully(raw, 132, supportBytes + 1);
    int status = raw[132 + supportBytes] & 255;
    if (status == 0) {
      in.readFully(raw, 133 + supportBytes, 37);
      return 170 + supportBytes;
    }
    if (status == 1 || status == 2) return 133 + supportBytes;
    throw new IOException("Invalid surface record status");
  }

  /**
   * Decompresses the owner and halo partitions that are not cached, in parallel, into the decode
   * cache. load() then reads them in the usual order, so the loaded samples are unchanged.
   */
  private void prefetch(Tile owner, double halo) throws IOException {
    List<Tile> wanted = new ArrayList<>();
    long total = 0;
    for (Tile t : leaves)
      if (t == owner || intersects(owner, t, halo)) {
        // get() refreshes LRU order so cached halo files survive the insertions below.
        if (decoded.get(t.path) != null || t.uncompressedBytes > decodedLimit) continue;
        wanted.add(t);
        total += t.uncompressedBytes;
      }
    if (wanted.size() < 2 || total > decodedLimit) return;
    byte[][] data = new byte[wanted.size()][];
    try {
      java.util.stream.IntStream.range(0, wanted.size())
          .parallel()
          .forEach(
              k -> {
                Tile t = wanted.get(k);
                try (DataInputStream in = input(t.path)) {
                  byte[] bytes = in.readNBytes(Math.toIntExact(t.uncompressedBytes));
                  if (bytes.length != t.uncompressedBytes)
                    throw new EOFException("Truncated surface partition");
                  data[k] = bytes;
                } catch (IOException e) {
                  throw new UncheckedIOException(e);
                }
              });
    } catch (UncheckedIOException e) {
      throw e.getCause();
    }
    for (int k = 0; k < wanted.size(); k++) {
      while (decodedBytes + data[k].length > decodedLimit && !decoded.isEmpty()) {
        var it = decoded.entrySet().iterator();
        decodedBytes -= it.next().getValue().length;
        it.remove();
      }
      decompressedBytes += data[k].length;
      decoded.put(wanted.get(k).path, data[k]);
      decodedBytes += data[k].length;
    }
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
        unchangedBudget.incrementAndGet();
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
            "surface_flatten_footprints",
            options.surfaceFlatten,
            "fitted_normals",
            fittedNormals,
            "isolated_off_surface_removed",
            isolatedRemoved,
            "movement_revalidation_rejected",
            recheckRejected,
            "resolved_layer_gaps",
            ambiguous.get(),
            "budget_passthrough_samples",
            unchangedBudget.get(),
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
            "partition_strategy",
            partitionStrategy,
            "coordinate_scratch_bytes",
            coordinateBytes,
            "coordinate_passes",
            coordinatePasses,
            "full_record_routing_passes",
            partitionStrategy.equals("routed") ? (coordinatePasses > 0 ? 1 : 0) : null,
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
                "evidence_freeze_seconds",
                readerNanos / 1e9,
                "partition_seconds",
                splitNanos / 1e9,
                "load_seconds",
                loadNanos / 1e9,
                "grid_seconds",
                gridNanos / 1e9,
                "neighborhood_seconds",
                neighborhoodNanos / 1e9,
                "median_and_layer_seconds",
                medianNanos.get() / 1e9,
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
    out.writeFloat((float) p.matchingConfidence);
    out.writeFloat((float) p.costMargin);
    out.writeFloat((float) p.costSharpness);
    out.writeFloat((float) p.photometric);
    out.writeFloat((float) p.lrError);
    byte[] support = p.supportingCameras.toByteArray();
    out.writeInt(support.length);
    out.write(support);
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
    p.matchingConfidence = in.readFloat();
    p.costMargin = in.readFloat();
    p.costSharpness = in.readFloat();
    p.photometric = in.readFloat();
    p.lrError = in.readFloat();
    int supportBytes = in.readInt();
    if (supportBytes < 0 || supportBytes > 1048576)
      throw new IOException("Invalid camera support length");
    byte[] support = new byte[supportBytes];
    in.readFully(support);
    p.supportingCameras = BitSet.valueOf(support);
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
    if (coordinates != null) {
      coordinates.close();
      coordinates = null;
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
