package sgm;

import java.io.IOException;
import java.lang.foreign.MemorySegment;
import java.nio.file.*;
import java.util.*;

/**
 * The hand-off from {@code --dense-method mvsgm} to image fusion (STREAMED_PROCESSING.md): each
 * camera's consistent depth and its evidence maps in 05_stereo/mvsgm, used directly as the
 * camera's single-view fusion pair instead of a candidate file. Per camera: depth, margin,
 * sharpness, photometric (phase 1), LR agreement in labels (NaN = rejected by the consistency
 * check) and packed colour (phase 2); all DepthMapFile layout, read memory-mapped.
 *
 * <p>A pixel is a candidate when its depth is positive and its agreement is not NaN. Candidates
 * of a camera are numbered in raster order, as the candidate file numbered them, so candidate ids
 * (pair start + rank) are unchanged; {@link #before} gives the rank of a row's first pixel.
 */
final class MvsgmMaps implements AutoCloseable {
  static final String[] KINDS = {".depth", ".margin", ".sharp", ".photo", ".lr", ".rgb"};
  static final int DEPTH = 0, MARGIN = 1, SHARP = 2, PHOTO = 3, LR = 4, RGB = 5;

  /** One camera's maps: fusion pair, size and the consistent-pixel count before each row. */
  record Camera(int camera, int pair, int width, int height, long[] rowStart, boolean tiled) {}

  final Path dir;
  private final Map<Integer, Camera> cameras = new TreeMap<>();
  private final Map<Integer, MemorySegment[]> segments = new HashMap<>();
  private final boolean keep;
  final boolean scoped;
  private final Map<Integer, MappedFiles> mapped = new HashMap<>();
  private long sourceMappedBytes, peakSourceMappedBytes, mappingOpens, mappingCloses;
  private int mappedFiles, peakMappedFiles;
  MvsgmWindow window;
  /** --dense-checkpoint: retired map files wait for the next checkpoint before deletion. */
  DenseCheckpoint checkpoint;
  FusionHostMemory hostMemory;
  private final Map<Integer, HostMapCache.File[]> cachedFiles = new HashMap<>();
  private final Map<Integer, Integer> readers = new HashMap<>();
  private boolean closing;

  /** Scoped source reader. mapPair joins all row tasks before closing; its result owns arrays. */
  final class Reader implements AutoCloseable {
    final int camera;
    private boolean closed;
    Reader(int camera) { this.camera = camera; }
    /** Resolve on the copying caller before parallel work; retain the lease through its join. */
    MemorySegment segment(int kind) throws IOException {
      if (closed) throw new IllegalStateException("Closed source reader");
      if (hostMemory != null) throw new IllegalStateException("Bounded source maps use page reads");
      return mappedSegment(camera, kind);
    }
    void read(int kind, long at, float[] dst) {
      if (closed) throw new IllegalStateException("Closed source reader");
      try {
      if (hostMemory == null) MappedFiles.copy(mappedSegment(camera, kind), at * 4, dst, 0, dst.length);
      else cached(camera, kind).read(at, dst, 0, dst.length);
      } catch (IOException e) { throw new java.io.UncheckedIOException(e); }
    }
    void readInts(int kind, long at, int[] dst) {
      if (closed) throw new IllegalStateException("Closed source reader");
      try {
      if (hostMemory == null) MappedFiles.copy(mappedSegment(camera, kind), at * 4, dst, 0, dst.length);
      else cached(camera, kind).readInts(at, dst, 0, dst.length);
      } catch (IOException e) { throw new java.io.UncheckedIOException(e); }
    }
    @Override public void close() {
      synchronized (MvsgmMaps.this) {
        if (closed) return;
        closed = true;
        readers.compute(camera, (k, n) -> n == 1 ? null : n - 1);
        try {
          if (scoped && hostMemory == null && !readers.containsKey(camera)) closeMappings(camera);
        } finally { MvsgmMaps.this.notifyAll(); }
      }
    }
  }
  synchronized Reader reader(int camera) {
    if (closing) throw new IllegalStateException("Source maps are closing");
    if (!cameras.containsKey(camera)) throw new IllegalStateException("Retired source camera " + camera);
    readers.merge(camera, 1, Integer::sum);
    return new Reader(camera);
  }
  private synchronized HostMapCache.File cached(int c, int kind) throws IOException {
    HostMapCache.File[] files = cachedFiles.computeIfAbsent(c, k -> new HostMapCache.File[KINDS.length]);
    if (files[kind] == null) {
      Camera cam = cameras.get(c);
      files[kind] = hostMemory.cache.open(file(c, kind), DepthMapFile.HEADER, 4L * cam.width() * cam.height(), false);
    }
    return files[kind];
  }

  MvsgmMaps(Path dir, boolean keep) {
    this(dir, keep, false);
  }

  MvsgmMaps(Path dir, boolean keep, boolean scoped) {
    this.dir = dir;
    this.keep = keep;
    this.scoped = scoped;
  }

  boolean scopedReaders() { return scoped || hostMemory != null; }

  synchronized Map<String, Object> sourceReport() {
    return Json.object("storage", hostMemory != null ? "bounded-pages" : scoped ? "scoped-mappings" : "retained-mappings",
        "mapped_source_bytes", sourceMappedBytes, "peak_mapped_source_bytes", peakSourceMappedBytes,
        "mapped_source_files", mappedFiles, "peak_mapped_source_files", peakMappedFiles,
        "file_mapping_opens", mappingOpens, "file_mapping_closes", mappingCloses,
        "active_reader_scopes", readers.values().stream().mapToInt(Integer::intValue).sum(),
        "scope", "Source-map extents, not resident RAM; file lifetime follows the existing dependency graph");
  }

  void add(Camera camera) {
    cameras.put(camera.camera(), camera);
  }

  /** A camera retired before a resumed checkpoint: no maps, no files to delete. */
  synchronized void forget(int c) {
    cameras.remove(c);
  }

  boolean keepsFiles() {
    return keep;
  }

  Camera camera(int c) {
    return cameras.get(c);
  }

  Collection<Camera> cameras() {
    return cameras.values();
  }

  Path file(int c, int kind) {
    return dir.resolve("camera_" + c + KINDS[kind]);
  }

  /** Consistent pixels (candidates) of camera c before row y. */
  long before(int c, int y) {
    return cameras.get(c).rowStart()[y];
  }

  /** Map kind of camera c (memory-mapped on first use, read-only). */
  synchronized MemorySegment segment(int c, int kind) throws IOException {
    if (scopedReaders()) throw new IllegalStateException("Source maps require a scoped reader");
    return mappedSegment(c, kind);
  }

  private synchronized MemorySegment mappedSegment(int c, int kind) throws IOException {
    MemorySegment[] s = segments.computeIfAbsent(c, k -> new MemorySegment[KINDS.length]);
    if (s[kind] == null) {
      Camera cam = cameras.get(c);
      s[kind] = mapped.computeIfAbsent(c, k -> new MappedFiles())
          .map(file(c, kind), DepthMapFile.HEADER, 4L * cam.width() * cam.height(), false);
      sourceMappedBytes += s[kind].byteSize();
      peakSourceMappedBytes = Math.max(peakSourceMappedBytes, sourceMappedBytes);
      mappingOpens++; mappedFiles++;
      peakMappedFiles = Math.max(peakMappedFiles, mappedFiles);
    }
    return s[kind];
  }

  /** Residency ends independently of disk dependency lifetime. Caller holds this monitor. */
  private void closeMappings(int c) {
    MappedFiles owner = mapped.get(c);
    if (owner == null) return;
    owner.close();
    sourceMappedBytes -= owner.mappedBytes();
    for (MemorySegment segment : segments.remove(c)) if (segment != null) {
      mappingCloses++; mappedFiles--;
    }
    mapped.remove(c);
  }

  /** Releases one finished camera, unmapping before deletion (required on Windows). */
  synchronized void releaseCamera(int c) throws IOException {
    if (readers.containsKey(c)) throw new IOException("Cannot retire source camera " + c + " while a reader is active");
    HostMapCache.File[] files = cachedFiles.remove(c);
    if (files != null) for (var file : files) hostMemory.cache.retire(file);
    closeMappings(c);
    if (!keep) {
      IOException failure = null;
      for (int k = 0; k < KINDS.length; k++)
        try {
          if (checkpoint != null && checkpoint.keepsWork()) checkpoint.defer(file(c, k));
          else Files.deleteIfExists(file(c, k));
        } catch (IOException e) {
          if (failure == null) failure = e;
          else failure.addSuppressed(e);
        }
      // Keep the camera registered if deletion failed so close() can retry every file.
      if (failure != null) throw failure;
    }
    cameras.remove(c);
  }

  /** Fusion may finish before a later consistency pass has read this camera's raw depth. */
  void finishedCamera(int c) throws IOException {
    if (window == null) releaseCamera(c);
    else window.fusionFinished(c);
  }

  /** Unmaps; deletes the maps unless --diagnostics or -Dsgm.mvsgm.keepMaps=true asked to keep them. */
  @Override
  public synchronized void close() throws IOException {
    closing = true;
    boolean interrupted = hostMemory != null && Thread.interrupted();
    if (window != null) window.close();
    if (hostMemory != null) interrupted |= Thread.interrupted();
    while (!readers.isEmpty()) try { wait(); }
    catch (InterruptedException e) { interrupted = true; }
    IOException failure = null;
    for (int c : new ArrayList<>(cameras.keySet()))
      try {
        releaseCamera(c);
      } catch (IOException e) {
        if (failure == null) failure = e;
        else failure.addSuppressed(e);
      }
    try { if (hostMemory != null) hostMemory.close(); }
    catch (IOException e) { if (failure == null) failure = e; else failure.addSuppressed(e); }
    if (interrupted) Thread.currentThread().interrupt();
    if (failure != null) throw failure;
  }
}
