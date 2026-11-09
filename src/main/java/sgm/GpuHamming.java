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
          __global int* out, __global const int* queryIds, int indexed, __local uint* tile) {
        int gid = get_global_id(0), lid = get_local_id(0), ls = get_local_size(0);
        int queryIndex = gid < nq ? (indexed ? queryIds[gid] : gid) : 0;
        uint q[WORDS];
        for (int k = 0; k < WORDS; k++) q[k] = gid < nq ? query[(long) queryIndex * WORDS + k] : 0u;
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
  private final int deviceIndex;
  long searches, queries, nanos;

  private GpuHamming(Options o, int words) {
    this.words = words;
    deviceIndex = o.gpuDevice;
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
        && a.channels() == 1 && b.channels() == 1
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
      if (instance != null && (instance.words != words || instance.deviceIndex != o.gpuDevice)) close();
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
        program.run(kernel, nq, q, nq, t, nt, r, r, 0, new GpuSgm.Local(4L * words * group));
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

  /** Both reciprocal directions reuse the same uploaded descriptors and result allocation. */
  static Integer reciprocal(Options o, Mat a, Mat b, List<MatOfDMatch> forward,
      Map<Integer, Integer> reverse, java.util.function.Consumer<String> progress) {
    if (!eligible(o, a, b) || a.rows() < 2) return null;
    // Keep initialization, first-use audit and teardown ordered with the legacy entry point.
    synchronized (GpuHamming.class) {
      if (disabled) return null;
      try {
        int words = a.cols() / 4;
        if (instance != null && (instance.words != words || instance.deviceIndex != o.gpuDevice)) close();
        if (instance == null) instance = new GpuHamming(o, words);
        return instance.reciprocalImpl(o, a, b, forward, reverse, progress);
      } catch (RuntimeException | Error failure) {
        disable("GPU reciprocal feature matching unavailable; using CPU: " + failure.getMessage());
        return null;
      }
    }
  }

  private synchronized int reciprocalImpl(Options o, Mat a, Mat b, List<MatOfDMatch> forward,
      Map<Integer, Integer> reverse, java.util.function.Consumer<String> progress) {
    long started = System.nanoTime();
    try (Arena arena = Arena.ofConfined()) {
      try {
        int na = a.rows(), nb = b.rows(), group = cl.group();
        MemorySegment da = cl.buffer(4L * words * na), db = cl.buffer(4L * words * nb);
        MemorySegment out = cl.buffer(16L * Math.max(na, nb));
        MemorySegment host = arena.allocate(16L * Math.max(na, nb), 4);
        MemorySegment idsDevice = cl.buffer(4L * nb);
        // Readback completes these uploads before arena memory can be released.
        if (o.gpuQueuedUploads) {
          cl.writeQueued(da, 0, bytes(arena, a));
          cl.writeQueued(db, 0, bytes(arena, b));
        } else {
          cl.transfer(da, 0, bytes(arena, a), false);
          cl.transfer(db, 0, bytes(arena, b), false);
        }
        progress.accept("GPU forward nearest neighbors; " + na + " queries");
        cl.run(kernel, na, da, na, db, nb, out, idsDevice, 0, new GpuSgm.Local(4L * words * group));
        cl.transfer(out, 0, host.asSlice(0, 16L * na), true);
        int[] first = host.asSlice(0, 16L * na).toArray(JAVA_INT);
        if (!verified) {
          String problem = verify(a, b, first);
          if (problem != null) throw new IllegalStateException("CPU forward audit: " + problem);
        }
        BitSet needed = new BitSet(nb);
        for (int i = 0; i < na; i++)
          if (first[4 * i + 1] < o.ratio * first[4 * i + 3]) needed.set(first[4 * i]);
        int count = needed.cardinality();
        boolean full = count * 5L >= nb * 4L;
        int[] ids = full ? null : needed.stream().toArray();
        int searched = full ? nb : count;
        int[] second = new int[0];
        if (searched > 0) {
          if (!full) cl.transfer(idsDevice, 0, arena.allocateFrom(JAVA_INT, ids), false);
          progress.accept("GPU reverse nearest neighbors; " + searched + "/" + nb + " queries (resident descriptors)");
          cl.run(kernel, searched, db, searched, da, na, out, idsDevice, full ? 0 : 1,
              new GpuSgm.Local(4L * words * group));
          cl.transfer(out, 0, host.asSlice(0, 16L * searched), true);
          second = host.asSlice(0, 16L * searched).toArray(JAVA_INT);
          if (!verified) {
            Mat sample = new Mat(searched, b.cols(), b.type());
            try {
              for (int i = 0; i < searched; i++) {
                Mat src = b.row(full ? i : ids[i]), dst = sample.row(i);
                try { src.copyTo(dst); } finally { src.release(); dst.release(); }
              }
              String problem = verify(sample, a, second);
              if (problem != null) throw new IllegalStateException("CPU reverse audit: " + problem);
            } finally { sample.release(); }
          }
        }
        // Publish only after both searches and audits succeed; fallback sees untouched outputs.
        List<MatOfDMatch> rows = new ArrayList<>();
        for (int i = 0; i < na; i++)
          rows.add(new MatOfDMatch(new DMatch(i, first[4 * i], 0, first[4 * i + 1]),
              new DMatch(i, first[4 * i + 2], 0, first[4 * i + 3])));
        for (int i = 0; i < searched; i++)
          if (second[4 * i + 1] < o.ratio * second[4 * i + 3])
            reverse.put(full ? i : ids[i], second[4 * i]);
        forward.addAll(rows);
        verified = true;
        searches += searched > 0 ? 2 : 1;
        queries += na + searched;
        nanos += System.nanoTime() - started;
        return searched;
      } finally {
        // Also finish on a kernel/driver error before freeing asynchronous source memory.
        try { cl.finish(); } finally { cl.releaseBuffers(); }
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
    try {
      instance.cl.finish();
      RunTelemetry.gpuProfile("GPU binary feature matching", instance.cl.profile());
    } finally {
      try { instance.cl.close(); }
      finally {
        instance = null;
        verified = false;
      }
    }
  }
}
