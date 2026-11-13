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
  private final Map<Integer, MappedFiles> mapped = new HashMap<>();
  MvsgmWindow window;

  MvsgmMaps(Path dir, boolean keep) {
    this.dir = dir;
    this.keep = keep;
  }

  void add(Camera camera) {
    cameras.put(camera.camera(), camera);
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
    MemorySegment[] s = segments.computeIfAbsent(c, k -> new MemorySegment[KINDS.length]);
    if (s[kind] == null) {
      Camera cam = cameras.get(c);
      s[kind] = mapped.computeIfAbsent(c, k -> new MappedFiles())
          .map(file(c, kind), DepthMapFile.HEADER, 4L * cam.width() * cam.height(), false);
    }
    return s[kind];
  }

  /** Releases one finished camera, unmapping before deletion (required on Windows). */
  synchronized void releaseCamera(int c) throws IOException {
    MappedFiles owner = mapped.remove(c);
    if (owner != null) owner.close();
    segments.remove(c);
    if (!keep) {
      IOException failure = null;
      for (int k = 0; k < KINDS.length; k++)
        try {
          Files.deleteIfExists(file(c, k));
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
    if (window != null) window.close();
    IOException failure = null;
    for (int c : new ArrayList<>(cameras.keySet()))
      try {
        releaseCamera(c);
      } catch (IOException e) {
        if (failure == null) failure = e;
        else failure.addSuppressed(e);
      }
    if (failure != null) throw failure;
  }
}
