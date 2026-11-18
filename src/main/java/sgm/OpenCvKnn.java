package sgm;

import java.lang.invoke.*;
import java.lang.ref.Reference;
import java.util.Arrays;
import org.opencv.core.*;
import org.opencv.features2d.DescriptorMatcher;

/**
 * Bounded adapter for the pinned OpenCV 4.9 JNI binding. The ordinary knnMatch wrapper converts
 * each native result row into TWO finalizable Java Mat headers. Here we call that same JNI
 * search, copy its CV_32FC4 rows, and delete the raw headers before publishing any Java wrapper.
 *
 * Ownership is critical: only addresses created by this search's vector_Mat_to_Mat converter
 * are deleted. They are NEVER wrapped in a Java Mat, so no finalizer can delete them again.
 * Java-owned query/train/matcher/result headers are never manually deleted. No C++ structure
 * layouts or external compiler/library are needed; method handles are checked at initialization.
 */
final class OpenCvKnn implements AutoCloseable {
  private static final MethodHandle SEARCH, TOTAL, GET, DELETE;
  static {
    try {
      var lookup = MethodHandles.lookup();
      var matcher = MethodHandles.privateLookupIn(DescriptorMatcher.class, lookup);
      var mat = MethodHandles.privateLookupIn(Mat.class, lookup);
      SEARCH = matcher.findStatic(DescriptorMatcher.class, "knnMatch_2",
          MethodType.methodType(void.class, long.class, long.class, long.class, long.class, int.class));
      TOTAL = mat.findStatic(Mat.class, "n_total", MethodType.methodType(long.class, long.class));
      GET = mat.findStatic(Mat.class, "nGetF",
          MethodType.methodType(int.class, long.class, int.class, int.class, int.class, float[].class));
      DELETE = mat.findStatic(Mat.class, "n_delete", MethodType.methodType(void.class, long.class));
    } catch (ReflectiveOperationException e) {
      throw new ExceptionInInitializerError(new IllegalStateException(
          "Bounded matching requires the bundled OpenCV 4.9 JNI binding", e));
    }
  }

  private final Mat result = new Mat();
  private int[] addresses = new int[0];
  private final float[] row = new float[8];
  long copiedRows, deletedRows;

  void search(DescriptorMatcher matcher, Mat query, Mat train, KnnMatches out) {
    // Allocate before native ownership transfers, so Java allocation failure cannot strand rows.
    out.reset(query.rows());
    int size = Math.multiplyExact(query.rows(), 2);
    if (addresses.length < size) addresses = new int[size];
    Arrays.fill(addresses, 0);
    // The JNI converter reuses this exact table shape. If native allocation fails halfway
    // through filling it, untouched entries must be zero rather than uninitialized pointers.
    result.create(query.rows(), 1, CvType.CV_32SC2);
    if (size > 0) result.put(0, 0, addresses);
    int owned = 0;
    boolean read = false;
    try {
      SEARCH.invokeExact(matcher.getNativeObjAddr(), query.nativeObj, train.nativeObj, result.nativeObj, 2);
      if (result.type() != CvType.CV_32SC2 || result.cols() != 1 || result.rows() != out.size())
        throw new IllegalStateException("Unexpected OpenCV knnMatch result layout");
      owned = result.rows();
      result.get(0, 0, addresses);
      read = true;
      for (int i = 0; i < owned; i++) {
        long address = address(i);
        if (address == 0) throw new IllegalStateException("Null native match row");
        int count = Math.toIntExact((long) TOTAL.invokeExact(address));
        if (count < 0 || count > 2) throw new IllegalStateException("Unexpected nearest-neighbour count");
        if (count > 0) {
          int bytes = (int) GET.invokeExact(address, 0, 0, 4 * count, row);
          if (bytes != 16 * count) throw new IllegalStateException("Incomplete native match row");
        }
        out.set(i, count, row);
        copiedRows++;
        delete(i);
      }
    } catch (Throwable e) {
      throw failure(e);
    } finally {
      try {
        // A native exception can leave a completed converter table: clean it too, once only.
        if (!read && !result.empty() && result.type() == CvType.CV_32SC2 && result.cols() == 1) {
          owned = result.rows();
          if (addresses.length < 2 * owned) addresses = new int[Math.multiplyExact(owned, 2)];
          result.get(0, 0, addresses);
        }
        for (int i = 0; i < owned; i++) if (address(i) != 0) delete(i);
      } finally {
        result.release();
        Reference.reachabilityFence(matcher);
        Reference.reachabilityFence(query);
        Reference.reachabilityFence(train);
        Reference.reachabilityFence(this);
      }
    }
  }

  private long address(int i) {
    return ((long) addresses[2 * i] << 32) | (addresses[2 * i + 1] & 0xffffffffL);
  }

  private void delete(int i) {
    long address = address(i);
    addresses[2 * i] = addresses[2 * i + 1] = 0;
    try {
      DELETE.invokeExact(address);
      deletedRows++;
    } catch (Throwable e) { throw failure(e); }
  }

  private static RuntimeException failure(Throwable e) {
    if (e instanceof Error error) throw error;
    if (e instanceof RuntimeException exception) return exception;
    return new IllegalStateException("OpenCV bounded matching failed", e);
  }

  @Override public void close() { result.release(); addresses = new int[0]; }
}
