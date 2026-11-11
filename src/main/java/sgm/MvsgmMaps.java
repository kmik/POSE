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
  private MappedFiles mapped;

  MvsgmMaps(Path dir, boolean keep) {
    this.dir = dir;
    this.keep = keep;
  }

  void add(Camera camera) {
    cameras.put(camera.camera(), camera);
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
    MemorySegment[] s = segments.computeIfAbsent(c, k -> new MemorySegment[KINDS.length]);
    if (s[kind] == null) {
      if (mapped == null) mapped = new MappedFiles();
      Camera cam = cameras.get(c);
      s[kind] = mapped.map(file(c, kind), DepthMapFile.HEADER, 4L * cam.width() * cam.height(), false);
    }
    return s[kind];
  }

  /** Unmaps; deletes the maps unless --diagnostics or -Dsgm.mvsgm.keepMaps=true asked to keep them. */
  @Override
  public synchronized void close() throws IOException {
    if (mapped != null) mapped.close();
    mapped = null;
    segments.clear();
    if (keep) return;
    for (int c : cameras.keySet())
      for (int k = 0; k < KINDS.length; k++) Files.deleteIfExists(file(c, k));
  }
}
