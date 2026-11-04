package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;

/**
 * Provisional observation index built while stereo runs. External sorting retains alternatives and
 * separate layers; no confidence rejection or publication occurs before all evidence is complete.
 */
final class IncrementalFusion implements AutoCloseable {
  private record Observation(
      CameraEmission.Key key,
      long pairKey,
      long candidateId,
      byte[] bytes,
      int source,
      int pixel,
      double[] point,
      float confidence,
      float rightConfidence) {}

  private record Cursor(Observation observation, int stream) {}

  private static final Comparator<Observation> ORDER =
      Comparator.comparingInt((Observation p) -> p.key.image())
          .thenComparingLong(p -> p.key.pixel())
          .thenComparingDouble(p -> p.key.depth())
          .thenComparingLong(p -> p.pairKey)
          .thenComparingInt(p -> p.pixel)
          .thenComparingDouble(p -> p.point[0])
          .thenComparingDouble(p -> p.point[1])
          .thenComparingDouble(p -> p.point[2])
          .thenComparingDouble(p -> p.confidence)
          .thenComparingDouble(p -> p.rightConfidence);
  final Options options;
  final DepthFusion fusion;
  final Path scratch;
  final int recordBytes, capacity;
  final List<Path> files = new ArrayList<>(), runs = new ArrayList<>();
  final List<Observation> buffer = new ArrayList<>();
  private long input, hypotheses, validated, accepted, rejected, alternatives, splitGroups;
  private int passes;
  private final Map<String, Long> decisions = new TreeMap<>();

  IncrementalFusion(Options options, DepthFusion fusion) throws IOException {
    this.options = options;
    this.fusion = fusion;
    recordBytes = PointConfidence.bidirectional(options) ? 59 : 55;
    Files.createDirectories(options.output.resolve("06_fusion"));
    scratch = Files.createTempDirectory(options.output.resolve("06_fusion"), "incremental-");
    capacity = (int) Math.max(128, Math.min(16384, options.memoryMb * 1048576L / 64 / 512));
  }

  OutputStream stream() {
    return new ObservationStream();
  }

  private final class ObservationStream extends OutputStream {
    final byte[] record = new byte[recordBytes];
    int used;

    public void write(int value) throws IOException {
      record[used++] = (byte) value;
      if (used == recordBytes) {
        add(record.clone());
        used = 0;
      }
    }

    public void write(byte[] bytes, int offset, int length) throws IOException {
      while (length > 0) {
        int n = Math.min(length, recordBytes - used);
        System.arraycopy(bytes, offset, record, used, n);
        used += n;
        offset += n;
        length -= n;
        if (used == recordBytes) {
          add(record.clone());
          used = 0;
        }
      }
    }

    public void close() throws IOException {
      if (used != 0) throw new EOFException("Partial incremental stereo observation");
      flushRun();
    }
  }

  private Observation parse(byte[] bytes, long candidateId) throws IOException {
    return parse(bytes, candidateId, null, 0);
  }

  private Observation parse(byte[] bytes, long candidateId, CameraEmission.Key savedKey, long savedPair) throws IOException {
    var in = new DataInputStream(new ByteArrayInputStream(bytes));
    double[] point = {in.readDouble(), in.readDouble(), in.readDouble()};
    in.skipNBytes(3);
    int source = in.readInt(), pixel = in.readInt();
    float confidence = in.readFloat();
    in.skipNBytes(16);
    float right = recordBytes == 59 ? in.readFloat() : Float.NaN;
    var key = savedKey == null ? fusion.hypothesisKey(source, point) : savedKey;
    // Invalid projection cannot share a hypothesis with another observation.
    if (key.image() < 0)
      key = new CameraEmission.Key(-1, ((long) source << 32) | (pixel & 0xffffffffL), 0);
    return new Observation(
        key,
        savedKey == null ? fusion.producingPairKey(source) : savedPair,
        candidateId,
        bytes,
        source,
        pixel,
        point,
        confidence,
        right);
  }

  private void add(byte[] bytes) throws IOException {
    buffer.add(parse(bytes, input));
    input++;
    if (buffer.size() >= capacity) flushRun();
  }

  private Path file() throws IOException {
    Path path = Files.createTempFile(scratch, "observations-", ".bin");
    files.add(path);
    return path;
  }

  private void flushRun() throws IOException {
    if (buffer.isEmpty()) return;
    buffer.sort(ORDER);
    Path path = file();
    try (var out = new DataOutputStream(new BufferedOutputStream(Files.newOutputStream(path)))) {
      for (var p : buffer) {
        writeObservation(out, p);
      }
    }
    runs.add(path);
    buffer.clear();
  }

  private void writeObservation(DataOutputStream out, Observation p) throws IOException {
    out.writeInt(p.key.image()); out.writeLong(p.key.pixel()); out.writeDouble(p.key.depth());
    out.writeLong(p.pairKey); out.writeLong(p.candidateId); out.write(p.bytes);
  }

  private Observation read(InputStream in) throws IOException {
    // Persist association keys once; merge passes never reproject every observation.
    byte[] bytes = in.readNBytes(recordBytes + 36);
    if (bytes.length == 0) return null;
    if (bytes.length != recordBytes + 36)
      throw new EOFException("Truncated provisional fusion observation");
    var data = new DataInputStream(new ByteArrayInputStream(bytes));
    var key = new CameraEmission.Key(data.readInt(), data.readLong(), data.readDouble());
    long pair = data.readLong(), candidate = data.readLong();
    return parse(Arrays.copyOfRange(bytes, 36, bytes.length), candidate, key, pair);
  }

  interface Sink {
    void accept(Observation p) throws IOException;
  }

  private void merge(List<Path> paths, Sink sink) throws IOException {
    List<InputStream> streams = new ArrayList<>();
    var queue = new PriorityQueue<Cursor>((a, b) -> ORDER.compare(a.observation, b.observation));
    try {
      for (Path path : paths) {
        var in = new BufferedInputStream(Files.newInputStream(path));
        streams.add(in);
        var p = read(in);
        if (p != null) queue.add(new Cursor(p, streams.size() - 1));
      }
      while (!queue.isEmpty()) {
        var p = queue.remove();
        sink.accept(p.observation);
        var next = read(streams.get(p.stream));
        if (next != null) queue.add(new Cursor(next, p.stream));
      }
    } finally {
      IOException error = null;
      for (var in : streams)
        try {
          in.close();
        } catch (IOException e) {
          error = e;
        }
      if (error != null) throw error;
    }
  }

  private boolean same(Observation a, Observation b) {
    return a.key.image() >= 0
        && a.key.image() == b.key.image()
        && a.key.pixel() == b.key.pixel()
        && fusion.sameHypothesis(a.source, a.point, b.source, b.point);
  }

  private double strength(Observation p) {
    return PointConfidence.valid(p.confidence) ? p.confidence : -1;
  }

  long finish(Path candidates, Path results) throws IOException {
    flushRun();
    while (runs.size() > 16) {
      List<Path> next = new ArrayList<>();
      for (int i = 0; i < runs.size(); i += 16) {
        var group = new ArrayList<>(runs.subList(i, Math.min(runs.size(), i + 16)));
        Path path = file();
        try (var out =
            new DataOutputStream(new BufferedOutputStream(Files.newOutputStream(path)))) {
          merge(
              group,
              p -> {
                writeObservation(out, p);
              });
        }
        next.add(path);
        for (Path old : group) Files.deleteIfExists(old);
      }
      runs.clear();
      runs.addAll(next);
      passes++;
    }
    List<List<Observation>> batch = new ArrayList<>();
    int[] pending = {0};
    List<Observation> group = new ArrayList<>();
    try (var out =
            new DataOutputStream(new BufferedOutputStream(Files.newOutputStream(candidates)));
        var prepared =
            new DataOutputStream(new BufferedOutputStream(Files.newOutputStream(results)));
        var work =
            WorkProgress.start("Finalize provisional fusion hypotheses", input, "observations")) {
      long[] visited = {0};
      merge(
          runs,
          p -> {
            if (!group.isEmpty() && (!same(group.getFirst(), p) || group.size() >= 512)) {
              if (group.size() >= 512) splitGroups++;
              batch.add(new ArrayList<>(group));
              hypotheses++;
              pending[0] += group.size();
              group.clear();
              if (batch.size() >= Math.min(4096, options.fusionBatchSize) || pending[0] >= capacity) {
                validate(batch, out, prepared);
                batch.clear();
                pending[0] = 0;
              }
            }
            group.add(p);
            work.update(++visited[0]);
          });
      if (!group.isEmpty()) {
        batch.add(group);
        hypotheses++;
      }
      if (!batch.isEmpty()) validate(batch, out, prepared);
      work.done();
    }
    Json.write(
        options.output.resolve("06_fusion/incremental.json"),
        Json.object(
            "experimental",
            true,
            "model",
            options.fusionAssociation.equals("camera") ? "provisional-camera-layers-v2" : "provisional-owner-pixel-layers-v1",
            "association_owner", options.fusionAssociation,
            "association_pixel_tolerance", options.fusionAssociation.equals("camera") ? .75 : .25,
            "persistent_sort_keys", true,
            "input_observations",
            input,
            "hypotheses",
            hypotheses,
            "validated_observations",
            validated,
            "retained_representatives",
            accepted,
            "rejected_hypotheses",
            rejected,
            "alternative_replays",
            alternatives,
            "bounded_group_splits",
            splitGroups,
            "validation_attempt_decisions",
            decisions,
            "sort_passes",
            passes,
            "chunk_capacity",
            capacity,
            "publication",
            "after all pair evidence is complete",
            "association",
            "Same owner pixel, configured anchored pixel reprojection in all producing cameras, <=min(0.25%,"
                + " relative tolerance/4) depth discrepancy; anchored sorted groups prevent"
                + " chaining",
            "provenance",
            "Original pair/subpixel/confidence observations retained through finalization; support"
                + " counted only by geometric fusion, never by group size",
            "sampling_change",
            "One accepted original representative per compatible layer; reference mode retains all"
                + " pair candidates. Completeness quality gate remains open."));
    return accepted;
  }

  private void validate(
      List<List<Observation>> groups, DataOutputStream out, DataOutputStream prepared)
      throws IOException {
    Comparator<Observation> strongest =
        Comparator.comparingDouble(this::strength).reversed().thenComparing(ORDER);
    for (var group : groups) group.sort(strongest);
    int[] cursor = new int[groups.size()];
    boolean[] done = new boolean[groups.size()];
    while (true) {
      List<Integer> active = new ArrayList<>();
      for (int i = 0; i < groups.size(); i++)
        if (!done[i] && cursor[i] < groups.get(i).size()) active.add(i);
      if (active.isEmpty()) break;
      int[] sources = new int[active.size()];
      double[][] points = new double[active.size()][];
      for (int j = 0; j < active.size(); j++) {
        int g = active.get(j);
        var p = groups.get(g).get(cursor[g]);
        sources[j] = p.source;
        points[j] = p.point;
      }
      var fused = fusion.fuseBatch(sources, points, active.size());
      validated += active.size();
      for (int j = 0; j < active.size(); j++) {
        int g = active.get(j);
        var p = groups.get(g).get(cursor[g]++);
        var r =
            TwoViewConfidence.apply(
                fused[j], p.confidence, p.rightConfidence, fusion.twoViewConfidenceThreshold());
        decisions.merge(r.status, 1L, Long::sum);
        if (!r.accepted()
            || !PointConfidence.passes(
                PointConfidence.finalScore(p.confidence, r.relativeSigma),
                options.confidenceThreshold)) {
          if (cursor[g] < groups.get(g).size()) alternatives++;
          continue;
        }
        out.write(p.bytes);
        prepared.writeLong(p.candidateId);
        writeResult(prepared, r);
        accepted++;
        done[g] = true;
      }
    }
    for (boolean success : done) if (!success) rejected++;
  }

  static void writeResult(DataOutputStream out, DepthFusion.Result r) throws IOException {
    for (double x : r.point) out.writeDouble(x);
    out.writeInt(r.views);
    out.writeInt(r.independentPairs);
    out.writeDouble(r.relativeSigma);
    out.writeDouble(r.shift);
    out.writeUTF(r.status);
    long[] support = r.supportingCameras == null ? new long[0] : r.supportingCameras.toLongArray();
    out.writeInt(support.length);
    for (long word : support) out.writeLong(word);
  }

  long rejectedHypotheses() {
    return rejected;
  }

  static DepthFusion.Result readResult(DataInputStream in) throws IOException {
    double[] point = {in.readDouble(), in.readDouble(), in.readDouble()};
    int views = in.readInt(), pairs = in.readInt();
    var r =
        new DepthFusion.Result(point, views, pairs, in.readDouble(), in.readDouble(), in.readUTF());
    int words = in.readInt();
    if (words < 0 || words > 65536) throw new IOException("Invalid prepared fusion support size");
    long[] support = new long[words];
    for (int i = 0; i < words; i++) support[i] = in.readLong();
    r.supportingCameras = BitSet.valueOf(support);
    return r;
  }

  public void close() throws IOException {
    IOException error = null;
    for (Path path : files)
      try {
        Files.deleteIfExists(path);
      } catch (IOException e) {
        error = e;
      }
    try {
      Files.deleteIfExists(scratch);
    } catch (IOException e) {
      error = e;
    }
    if (error != null) throw error;
  }
}
