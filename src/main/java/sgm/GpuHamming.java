package sgm;

import static java.lang.foreign.ValueLayout.*;

import java.lang.foreign.*;
import java.util.*;
import org.opencv.core.*;
import org.opencv.features2d.*;

/**
 * Exact brute-force k=2 Hamming nearest neighbours on the GPU for binary descriptors (ORB, BRISK,
 * AKAZE), replacing OpenCV's CPU {@code BFMatcher(NORM_HAMMING).knnMatch(..., 2)}.
 *
 * <p>Same result as the CPU matcher: every train descriptor is compared in index order and a
 * candidate replaces a neighbour only when strictly closer, so ties keep the lower train index for
 * the best neighbour and the next equal one becomes the second (OpenCV's insertion order). The first
 * search of a run is additionally checked against the CPU matcher on a query sample; any mismatch
 * or GPU failure switches the run to the CPU matcher. {@code --gpu-feature-matching off} or {@code
 * --compute cpu} disables it.
 *
 * <p>One dedicated OpenCL session serves all matching threads; searches are serialized on it.
 */
final class GpuHamming {
  private static final String SOURCE =
      """
      // One work-item per query; train descriptors stream through local memory in index order.
      __kernel void knn2(__global const uint* query, int nq, __global const uint* train, int nt,
          __global int* out, __local uint* tile) {
        int gid = get_global_id(0), lid = get_local_id(0), ls = get_local_size(0);
        uint q[WORDS];
        for (int k = 0; k < WORDS; k++) q[k] = gid < nq ? query[(long) gid * WORDS + k] : 0u;
        int b1 = 0x7fffffff, i1 = -1, b2 = 0x7fffffff, i2 = -1;
        for (int base = 0; base < nt; base += ls) {
          int idx = base + lid;
          for (int k = 0; k < WORDS; k++) tile[lid * WORDS + k] = idx < nt ? train[(long) idx * WORDS + k] : 0u;
          barrier(CLK_LOCAL_MEM_FENCE);
          int limit = min(ls, nt - base);
          for (int j = 0; j < limit; j++) {
            int d = 0;
            for (int k = 0; k < WORDS; k++) d += popcount(q[k] ^ tile[j * WORDS + k]);
            if (d < b1) { b2 = b1; i2 = i1; b1 = d; i1 = base + j; }
            else if (d < b2) { b2 = d; i2 = base + j; }
          }
          barrier(CLK_LOCAL_MEM_FENCE);
        }
        if (gid < nq) {
          out[4 * gid] = i1; out[4 * gid + 1] = b1; out[4 * gid + 2] = i2; out[4 * gid + 3] = b2;
        }
      }
      """;

  private static GpuHamming instance;
  private static boolean disabled;
  private static boolean verified;

  private final GpuSgm.Compute cl;
  private final MemorySegment kernel;
  private final int words;
  long searches, queries, nanos;

  private GpuHamming(Options o, int words) {
    this.words = words;
    cl = new GpuSgm.Compute(o, "#define WORDS " + words + "\n" + SOURCE, true);
    try {
      kernel = cl.kernel("knn2");
    } catch (RuntimeException e) {
      cl.close();
      throw e;
    }
  }

  /** Whether the GPU path may be tried for these descriptors. */
  static boolean eligible(Options o, Mat a, Mat b) {
    return !disabled
        && !o.compute.equals("cpu")
        && o.gpuFeatureMatching
        && a.depth() == CvType.CV_8U
        && b.depth() == CvType.CV_8U
        && a.cols() == b.cols()
        && a.cols() % 4 == 0
        && a.cols() > 0
        && a.rows() > 0
        && b.rows() >= 2;
  }

  /**
   * knnMatch(query, train, out, 2) on the GPU. Returns false (out untouched) when the GPU path is
   * unavailable; the caller then uses the CPU matcher.
   */
  static boolean knn2(Options o, Mat query, Mat train, List<MatOfDMatch> out) {
    if (!eligible(o, query, train)) return false;
    GpuHamming g;
    synchronized (GpuHamming.class) {
      if (disabled) return false;
      int words = query.cols() / 4;
      if (instance != null && instance.words != words) close();
      if (instance == null) {
        try {
          instance = new GpuHamming(o, words);
        } catch (RuntimeException | Error e) {
          disable("GPU feature matching unavailable; using CPU matcher: " + e.getMessage());
          return false;
        }
      }
      g = instance;
    }
    int[] result;
    try {
      result = g.search(query, train);
    } catch (RuntimeException | Error e) {
      disable("GPU feature matching failed; using CPU matcher: " + e.getMessage());
      return false;
    }
    synchronized (GpuHamming.class) {
      if (!verified) {
        String problem = verify(query, train, result);
        if (problem != null) {
          disable("GPU feature matching disagreed with the CPU matcher (" + problem + "); using CPU");
          return false;
        }
        verified = true;
        System.out.println("GPU feature matching: exact Hamming k=2 search verified against CPU matcher");
      }
    }
    for (int i = 0; i < query.rows(); i++) {
      int i1 = result[4 * i], i2 = result[4 * i + 2];
      DMatch[] row =
          i2 >= 0
              ? new DMatch[] {new DMatch(i, i1, 0, result[4 * i + 1]), new DMatch(i, i2, 0, result[4 * i + 3])}
              : new DMatch[] {new DMatch(i, i1, 0, result[4 * i + 1])};
      out.add(new MatOfDMatch(row));
    }
    return true;
  }

  private synchronized int[] search(Mat query, Mat train) {
    long started = System.nanoTime();
    int nq = query.rows(), nt = train.rows();
    GpuSgm.Compute program = cl;
    try (Arena arena = Arena.ofConfined()) {
      MemorySegment q = program.buffer(4L * words * nq), t = program.buffer(4L * words * nt), r = program.buffer(16L * nq);
      try {
        program.transfer(q, 0, bytes(arena, query), false);
        program.transfer(t, 0, bytes(arena, train), false);
        int group = program.group();
        program.run(kernel, nq, q, nq, t, nt, r, new GpuSgm.Local(4L * words * group));
        MemorySegment host = arena.allocate(16L * nq, 4);
        program.transfer(r, 0, host, true);
        int[] result = host.toArray(JAVA_INT);
        searches++;
        queries += nq;
        nanos += System.nanoTime() - started;
        return result;
      } finally {
        program.releaseBuffers();
      }
    }
  }

  private static MemorySegment bytes(Arena arena, Mat m) {
    Mat c = m.isContinuous() ? m : m.clone();
    try {
      byte[] data = new byte[(int) (c.total() * c.elemSize())];
      c.get(0, 0, data);
      return arena.allocateFrom(JAVA_BYTE, data);
    } finally {
      if (c != m) c.release();
    }
  }

  /** Compares up to 512 evenly spread queries with the CPU matcher; null when identical. */
  private static String verify(Mat query, Mat train, int[] result) {
    int n = query.rows(), step = Math.max(1, n / 512);
    BFMatcher cpu = BFMatcher.create(Core.NORM_HAMMING, false);
    List<MatOfDMatch> rows = new ArrayList<>();
    Mat sample = new Mat();
    try {
      List<Integer> ids = new ArrayList<>();
      for (int i = 0; i < n; i += step) ids.add(i);
      sample.create(ids.size(), query.cols(), query.type());
      for (int s = 0; s < ids.size(); s++) {
        Mat from = query.row(ids.get(s)), to = sample.row(s);
        from.copyTo(to);
        from.release();
        to.release();
      }
      cpu.knnMatch(sample, train, rows, 2);
      for (int s = 0; s < ids.size(); s++) {
        DMatch[] m = rows.get(s).toArray();
        int i = ids.get(s);
        if (m.length < 1 || m[0].trainIdx != result[4 * i] || m[0].distance != result[4 * i + 1])
          return "query " + i + " best neighbour";
        if (m.length >= 2 && (m[1].trainIdx != result[4 * i + 2] || m[1].distance != result[4 * i + 3]))
          return "query " + i + " second neighbour";
      }
      return null;
    } finally {
      for (Mat m : rows) m.release();
      sample.release();
      cpu.clear();
    }
  }

  private static void disable(String message) {
    disabled = true;
    System.out.println(message);
  }

  /** Summary for logs; null when unused. */
  static synchronized String summary() {
    if (instance == null) return null;
    return String.format(
        Locale.ROOT,
        "GPU feature matching: %,d searches, %,d queries, %.1f s",
        instance.searches,
        instance.queries,
        instance.nanos / 1e9);
  }

  /** Releases the GPU session (end of a matching phase); a later search opens a new one. */
  static synchronized void close() {
    if (instance == null) return;
    String s = summary();
    if (s != null && instance.searches > 0) System.out.println(s);
    instance.cl.close();
    instance = null;
  }
}
