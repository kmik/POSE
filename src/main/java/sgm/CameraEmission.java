package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;

/** Experimental bounded external sort of accepted samples into layered owner-camera pixels. */
final class CameraEmission implements AutoCloseable {
  interface Sink {
    void points(List<SurfaceConsolidator.Sample> points) throws IOException;
  }

  record Key(int image, long pixel, double depth) {}

  private record Entry(Key key, PointRecords.Row row) {}

  private record Cursor(Entry entry, int reader) {}

  private final DepthFusion fusion;
  private final Options options;
  private final Path scratch;
  private final List<Entry> buffer = new ArrayList<>();
  private final List<Path> runs = new ArrayList<>(), files = new ArrayList<>();
  private final int capacity, fanIn;
  private long input, kept, merged;
  private int passes;
  private static final Comparator<Entry> ORDER =
      Comparator.comparingInt((Entry e) -> e.key.image)
          .thenComparingLong(e -> e.key.pixel)
          .thenComparingDouble(e -> e.key.depth)
          .thenComparingLong(e -> e.row.sample().id);

  CameraEmission(Options options, DepthFusion fusion) throws IOException {
    this.options = options;
    this.fusion = fusion;
    scratch = Files.createTempDirectory(options.output.resolve("06_fusion"), "camera-emission-");
    capacity =
        (int) Math.max(128, Math.min(65536, (options.surfaceMemoryMb - 4L) * 1048576 / 2048));
    fanIn = Math.max(2, Math.min(16, options.surfaceMemoryMb / 2));
  }

  void add(SurfaceConsolidator.Sample p) throws IOException {
    buffer.add(
        new Entry(
            fusion.ownerKey(p),
            new PointRecords.Row(
                p.id, p, PointConfidence.finalScore(p.matchingConfidence, p.sigma))));
    input++;
    if (buffer.size() >= capacity) flush();
  }

  private Path file() throws IOException {
    Path p = Files.createTempFile(scratch, "sorted-", ".bin");
    files.add(p);
    return p;
  }

  private void flush() throws IOException {
    if (buffer.isEmpty()) return;
    buffer.sort(ORDER);
    Path path = file();
    try (var out = new PointRecords.Writer(path)) {
      for (var entry : buffer) out.point(entry.row.id(), entry.row.sample(), 1, entry.row.score());
    }
    runs.add(path);
    buffer.clear();
  }

  private Entry next(PointRecords.Reader reader) throws IOException {
    var row = reader.next();
    return row == null ? null : new Entry(fusion.ownerKey(row.sample()), row);
  }

  private void sorted(List<Path> paths, java.util.function.Consumer<Entry> sink)
      throws IOException {
    List<PointRecords.Reader> readers = new ArrayList<>();
    var queue = new PriorityQueue<Cursor>((a, b) -> ORDER.compare(a.entry, b.entry));
    try {
      for (Path p : paths) {
        var reader = new PointRecords.Reader(p);
        readers.add(reader);
        var entry = next(reader);
        if (entry != null) queue.add(new Cursor(entry, readers.size() - 1));
      }
      while (!queue.isEmpty()) {
        var cursor = queue.remove();
        sink.accept(cursor.entry);
        var entry = next(readers.get(cursor.reader));
        if (entry != null) queue.add(new Cursor(entry, cursor.reader));
      }
    } catch (UncheckedIOException e) {
      throw e.getCause();
    } finally {
      IOException failure = null;
      for (var r : readers)
        try {
          r.close();
        } catch (IOException e) {
          failure = e;
        }
      if (failure != null) throw failure;
    }
  }

  void finish(Sink sink) throws IOException {
    flush();
    while (runs.size() > fanIn) {
      List<Path> next = new ArrayList<>();
      for (int i = 0; i < runs.size(); i += fanIn) {
        List<Path> group = new ArrayList<>(runs.subList(i, Math.min(runs.size(), i + fanIn)));
        Path path = file();
        try (var out = new PointRecords.Writer(path)) {
          sorted(
              group,
              e -> {
                try {
                  out.point(e.row.id(), e.row.sample(), 1, e.row.score());
                } catch (IOException x) {
                  throw new UncheckedIOException(x);
                }
              });
        }
        next.add(path);
        for (Path p : group) Files.deleteIfExists(p);
      }
      runs.clear();
      runs.addAll(next);
      passes++;
    }
    final Entry[] first = {null}, best = {null};
    final int[] observations = {0};
    List<SurfaceConsolidator.Sample> batch = new ArrayList<>();
    sorted(
        runs,
        e -> {
          try {
            if (first[0] != null && !same(first[0], e)) {
              emit(best[0], observations[0], batch, sink);
              first[0] = null;
            }
            if (first[0] == null) {
              first[0] = e;
              best[0] = e;
              observations[0] = 0;
            }
            observations[0] = Math.addExact(observations[0], e.row.sample().merged);
            if (better(e, best[0])) best[0] = e;
          } catch (IOException x) {
            throw new UncheckedIOException(x);
          }
        });
    if (best[0] != null) emit(best[0], observations[0], batch, sink);
    if (!batch.isEmpty()) sink.points(batch);
    Json.write(
        options.output.resolve("06_fusion/camera_emission.json"),
        Json.object(
            "experimental",
            true,
            "input_accepted",
            input,
            "representatives",
            kept,
            "redundant_observations",
            merged,
            "sort_passes",
            passes,
            "chunk_capacity",
            capacity,
            "merge_fan_in",
            fanIn,
            "scope",
            "Strongest accepted point per owner-camera pixel/depth layer; XYZ, confidence and"
                + " supporting-camera provenance are not averaged or boosted. All pair depth"
                + " evidence and geometric checks remain in use. Dense completeness quality gate"
                + " remains open."));
  }

  private boolean same(Entry a, Entry b) {
    return a.key.image == b.key.image
        && a.key.pixel == b.key.pixel
        && a.key.depth > 0
        && Math.abs(a.key.depth - b.key.depth)
            <= options.fusionRelative * Math.min(a.key.depth, b.key.depth);
  }

  private boolean better(Entry a, Entry b) {
    double x = PointConfidence.valid(a.row.score()) ? a.row.score() : -1,
        y = PointConfidence.valid(b.row.score()) ? b.row.score() : -1;
    return x > y
        || x == y
            && (a.row.sample().sigma < b.row.sample().sigma
                || a.row.sample().sigma == b.row.sample().sigma
                    && a.row.sample().id < b.row.sample().id);
  }

  private void emit(
      Entry entry, int observations, List<SurfaceConsolidator.Sample> batch, Sink sink)
      throws IOException {
    var p = entry.row.sample();
    p.merged = observations;
    batch.add(p);
    kept++;
    merged += observations - 1;
    if (batch.size() >= 256) {
      sink.points(batch);
      batch.clear();
    }
  }

  long merged() {
    return merged;
  }

  public void close() throws IOException {
    IOException failure = null;
    for (Path p : files)
      try {
        Files.deleteIfExists(p);
      } catch (IOException e) {
        failure = e;
      }
    try {
      Files.deleteIfExists(scratch);
    } catch (IOException e) {
      failure = e;
    }
    if (failure != null) throw failure;
  }
}
