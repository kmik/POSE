package sgm;

import java.lang.reflect.*;
import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;

/**
 * Reusable bundle workspaces: a reused undamped system, damped trial system and scratch must give
 * bit for bit the results of a fresh one, across changing problem sizes, damping values, storage
 * page sizes, repeated solves and a failed refill; solutions never alias scratch; camera-order
 * traversal is point/edge order; measured calibration points keep their exact values.
 */
final class BundleWorkspaceRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Core.setNumThreads(1);
    Path root = Files.createTempDirectory(Path.of("target"), "bundle workspace ");
    System.out.println("PASS bundle workspace: " + run(root) + "; " + root);
  }

  static int run(Path root) throws Exception {
    checks = 0;
    try (var small = new AlignmentEquivalenceDigest.Grid(root.resolve("small"), 9, 300);
        var large = new AlignmentEquivalenceDigest.Grid(root.resolve("large"), 14, 700)) {
      for (boolean shared : new boolean[] {false, true}) {
        SharedCalibration smallLens = shared ? lens(small.frames) : null;
        SharedCalibration largeLens = shared ? lens(large.frames) : null;
        Object workspace = newWorkspace();
        // Same workspace across changing sizes (small, large, small): equal to fresh systems.
        for (int round = 0; round < 3; round++) {
          var f = round == 1 ? large : small;
          SharedCalibration model = round == 1 ? largeLens : smallLens;
          for (double lambda : new double[] {1e-3, 1e-1, 1e-3}) {
            Object reused = refill(damped(workspace, linearize(f, model, workspace)), lambda);
            Object fresh = fresh(f, model, lambda);
            same(reused, fresh, f, model, "reused workspace round " + round + " lambda " + lambda + " shared " + shared);
          }
        }
        // Refilled in place for the same shape (the next iteration of a fit), after the points move.
        Object first = linearize(small, smallLens, workspace);
        for (var t : small.tracks) t.point = Math3.add(t.point, new double[] {1e-4, -2e-4, 3e-4});
        Object second = linearize(small, smallLens, workspace);
        check(first == second, "Unchanged shape refills the undamped system in place");
        same(refill(damped(workspace, second), 1e-2), fresh(small, smallLens, 1e-2), small, smallLens,
            "In-place refill after moved points equals a fresh linearization");
        order(second);
        aliasing(refill(damped(workspace, second), 1e-2), small, smallLens);
        moves(refill(damped(workspace, second), 1e-2), small);
      }
      pages(small);
      failedRefill(root.resolve("singular"));
    }
    measured(root.resolve("measured"));
    estimates();
    progress();
    return checks;
  }

  /** Iteration estimate of a converging bundle (progress display) and the completed count. */
  private static void progress() {
    // Gains halving from 1.0 with threshold 1e-3: below it after 10 halvings (2^-10 < 1e-3).
    double[] gains = new double[4];
    int known = 0;
    for (int done = 1; done <= 3; done++) {
      System.arraycopy(gains, 0, gains, 1, 3);
      gains[0] = Math.pow(.5, done - 1);
      known++;
    }
    // After 3 iterations the newest gain is 0.25: 8 more halvings reach 2^-10, then one more.
    long estimate = BundleAdjuster.estimatedIterations(3, gains, known, 0, 1e-3, 100);
    check(estimate == 3 + 8 + 1, "Geometric gains give the iterations to the threshold plus one: " + estimate);
    check(BundleAdjuster.estimatedIterations(1, gains, 1, 0, 1e-3, 100) == 0, "One gain: no estimate");
    check(BundleAdjuster.estimatedIterations(3, new double[] {1, .5, 2, 0}, 3, 0, 1e-3, 100) > 0
        && BundleAdjuster.estimatedIterations(3, new double[] {2, 1, .5, 0}, 3, 0, 1e-3, 100) == 0,
        "Shrinking on average gives an estimate; growing gains give none");
    check(BundleAdjuster.estimatedIterations(5, gains, known, 1, 1e-3, 100) == 6, "One small gain: one more iteration");
    check(BundleAdjuster.estimatedIterations(3, new double[] {.9, .95, 1, 0}, 3, 0, 1e-6, 100) == 100,
        "Slowly shrinking gains are capped at the iteration limit");
    try (WorkProgress work = WorkProgress.start("progress check", 0, "iterations")) {
      work.total(40, true);
      work.update(7);
      check(work.total() == 40 && work.estimated(), "Revised estimated total is shown");
      work.done();
      check(work.total() == 7 && !work.estimated(), "On completion the total is the iterations actually run");
    }
  }

  private static SharedCalibration lens(List<ImageFrame> frames) {
    SharedCalibration lens = new SharedCalibration(frames);
    for (int k : new int[] {0, 1, 2, 3, 4}) lens.active[k] = true;
    return lens;
  }

  // ---- Reflection on the private solver classes

  private static Class<?> nested(String name) throws Exception {
    return Class.forName("sgm.BundleAdjuster$" + name);
  }

  private static Object newWorkspace() throws Exception {
    Constructor<?> c = nested("Workspace").getDeclaredConstructor();
    c.setAccessible(true);
    return c.newInstance();
  }

  private static int[] index(List<ImageFrame> frames) {
    int[] index = new int[frames.size()];
    index[0] = -1;
    for (int i = 1; i < index.length; i++) index[i] = i - 1;
    return index;
  }

  private static Object linearize(AlignmentEquivalenceDigest.Grid f, SharedCalibration lens, Object workspace) throws Exception {
    Method m = BundleAdjuster.class.getDeclaredMethod("linearize", List.class, List.class, int[].class, int.class,
        double.class, SharedCalibration.class, nested("Workspace"));
    m.setAccessible(true);
    return m.invoke(null, f.frames, f.tracks, index(f.frames), f.frames.size() - 1, 2., lens, workspace);
  }

  private static Object fresh(AlignmentEquivalenceDigest.Grid f, SharedCalibration lens, double lambda) throws Exception {
    Method m = BundleAdjuster.class.getDeclaredMethod("linearize", List.class, List.class, int[].class, int.class,
        double.class, double.class, SharedCalibration.class);
    m.setAccessible(true);
    return m.invoke(null, f.frames, f.tracks, index(f.frames), f.frames.size() - 1, 2., lambda, lens);
  }

  private static Object damped(Object workspace, Object linear) throws Exception {
    return call(workspace, "damped", new Class<?>[] {nested("Linear")}, linear);
  }

  private static Object refill(Object damped, double lambda) throws Exception {
    call(damped, "refill", new Class<?>[] {double.class}, lambda);
    return damped;
  }

  private static Object call(Object target, String name, Class<?>[] types, Object... args) throws Exception {
    Method m = target.getClass().getDeclaredMethod(name, types);
    m.setAccessible(true);
    try {
      return m.invoke(target, args);
    } catch (InvocationTargetException e) {
      if (e.getCause() instanceof Exception cause) throw cause;
      throw e;
    }
  }

  private static Object field(Object target, String name) throws Exception {
    Field f = target.getClass().getDeclaredField(name);
    f.setAccessible(true);
    return f.get(target);
  }

  private static double[] x(Object result) throws Exception {
    return (double[]) call(result, "x", new Class<?>[0]);
  }

  private static int n(Object damped) throws Exception {
    return (Integer) field(damped, "n");
  }

  /** Reduced matrix, products, solves (own and supplied right-hand sides) are bit-identical. */
  private static void same(Object a, Object b, AlignmentEquivalenceDigest.Grid f, SharedCalibration lens,
      String what) throws Exception {
    int n = n(a);
    check(n == n(b), what + ": same unknown count");
    double[] ha = ((double[]) call(a, "reducedMatrix", new Class<?>[] {int.class}, n)).clone();
    double[] hb = (double[]) call(b, "reducedMatrix", new Class<?>[] {int.class}, n);
    check(Arrays.equals(ha, hb), what + ": reduced matrix");
    Random random = new Random(31);
    for (int trial = 0; trial < 3; trial++) {
      double[] v = new double[n];
      for (int i = 0; i < n; i++) v[i] = random.nextGaussian();
      double[] pa = (double[]) call(a, "product", new Class<?>[] {double[].class}, (Object) v);
      double[] pb = (double[]) call(b, "product", new Class<?>[] {double[].class}, (Object) v);
      check(Arrays.equals(pa, pb), what + ": product");
      Object ra = call(a, "solve", new Class<?>[] {double[].class}, (Object) pa);
      Object rb = call(b, "solve", new Class<?>[] {double[].class}, (Object) pb);
      check(Arrays.equals(x(ra), x(rb)), what + ": supplied right-hand side solve");
    }
    Object sa = call(a, "solve", new Class<?>[0]), sb = call(b, "solve", new Class<?>[0]);
    check(Arrays.equals(x(sa), x(sb)), what + ": own right-hand side solve");
    // The matrix-free path (no direct workspace) on the same systems.
    String previous = System.getProperty("sgm.test.bundleDirectWorkspaceBytes");
    try {
      Field budget = a.getClass().getDeclaredField("workspaceBudget");
      budget.setAccessible(true);
      // A fresh damped system reads the property; compare two fresh PCG solves with reuse.
      System.setProperty("sgm.test.bundleDirectWorkspaceBytes", "0");
      Object pcgFresh = fresh(f, lens, 1e-2);
      Object pcgReused = refill(damped(newWorkspace(), linearize(f, lens, newWorkspace())), 1e-2);
      Object first = call(pcgReused, "solve", new Class<?>[0]);
      Object again = call(pcgReused, "solve", new Class<?>[0]);
      Object reference = call(pcgFresh, "solve", new Class<?>[0]);
      check("matrix_free_pcg".equals(call(reference, "method", new Class<?>[0])), what + ": PCG path exercised");
      check(Arrays.equals(x(first), x(reference)) && Arrays.equals(x(again), x(reference)),
          what + ": repeated PCG solves with reused scratch");
    } finally {
      if (previous == null) System.clearProperty("sgm.test.bundleDirectWorkspaceBytes");
      else System.setProperty("sgm.test.bundleDirectWorkspaceBytes", previous);
    }
  }

  /** Solutions are new arrays; a supplied right-hand side is only read. */
  private static void aliasing(Object damped, AlignmentEquivalenceDigest.Grid f, SharedCalibration lens) throws Exception {
    Object a = call(damped, "solve", new Class<?>[0]);
    double[] xa = x(a), copy = xa.clone();
    Object b = call(damped, "solve", new Class<?>[0]);
    check(x(b) != xa && Arrays.equals(x(b), copy), "Each solve returns its own solution array");
    Arrays.fill(xa, 7);
    Object c = call(damped, "solve", new Class<?>[0]);
    check(Arrays.equals(x(c), copy), "Changing a returned solution does not change later solves");
    double[] rhs = new double[n(damped)];
    Arrays.fill(rhs, .25);
    double[] kept = rhs.clone();
    call(damped, "solve", new Class<?>[] {double[].class}, (Object) rhs);
    check(Arrays.equals(rhs, kept), "A supplied right-hand side is not modified");
    double[] v = rhs.clone();
    double[] p1 = (double[]) call(damped, "product", new Class<?>[] {double[].class}, (Object) v);
    double[] p2 = (double[]) call(damped, "product", new Class<?>[] {double[].class}, (Object) v);
    check(p1 != p2 && Arrays.equals(p1, p2) && Arrays.equals(v, kept), "Products are new arrays; the input is only read");
  }

  /** Per-camera (point, edge) lists are exactly the point/edge order of the edges of that camera. */
  private static void order(Object linear) throws Exception {
    int count = (Integer) field(linear, "count"), points = (Integer) field(linear, "points"), edges = (Integer) field(linear, "edges");
    int[] edgeStart = (int[]) field(linear, "edgeStart"), edgeCamera = (int[]) field(linear, "edgeCamera");
    int[] cameraStart = (int[]) field(linear, "cameraStart"), cameraPoint = (int[]) field(linear, "cameraPoint"),
        cameraEdge = (int[]) field(linear, "cameraEdge");
    check(edgeStart[0] == 0 && edgeStart[points] == edges && cameraStart[count] == edges, "Edge ranges cover every edge once");
    for (int c = 0; c < count; c++) {
      List<int[]> expected = new ArrayList<>();
      for (int p = 0; p < points; p++)
        for (int e = edgeStart[p]; e < edgeStart[p + 1]; e++) if (edgeCamera[e] == c) expected.add(new int[] {p, e});
      check(cameraStart[c + 1] - cameraStart[c] == expected.size(), "Camera " + c + " edge count");
      for (int k = 0; k < expected.size(); k++)
        check(cameraPoint[cameraStart[c] + k] == expected.get(k)[0] && cameraEdge[cameraStart[c] + k] == expected.get(k)[1],
            "Camera " + c + " visits its edges in point/edge order");
    }
  }

  /** movePoints installs new arrays and leaves the previous ones untouched (exact restore). */
  private static void moves(Object damped, AlignmentEquivalenceDigest.Grid f) throws Exception {
    double[][] before = new double[f.tracks.size()][], values = new double[f.tracks.size()][];
    for (int i = 0; i < before.length; i++) {
      before[i] = f.tracks.get(i).point;
      values[i] = before[i].clone();
    }
    double[] step = new double[n(damped)];
    Arrays.fill(step, 1e-3);
    call(damped, "movePoints", new Class<?>[] {double[].class, double[].class, double.class}, step, new double[] {.1, .2, .3}, 1.01);
    boolean fresh = true, untouched = true;
    for (int i = 0; i < before.length; i++) {
      fresh &= f.tracks.get(i).point != before[i] && !Arrays.equals(f.tracks.get(i).point, before[i]);
      untouched &= Arrays.equals(before[i], values[i]);
    }
    check(fresh && untouched, "Moved points are new arrays; saved arrays keep their values");
    for (int i = 0; i < before.length; i++) f.tracks.get(i).point = before[i];
  }

  /** Storage pages of 4 points/edges give the same results as the default 8192. */
  private static void pages(AlignmentEquivalenceDigest.Grid f) throws Exception {
    String previous = System.getProperty("sgm.test.bundlePageShift");
    try {
      for (boolean shared : new boolean[] {false, true}) {
        SharedCalibration lens = shared ? lens(f.frames) : null;
        Object reference = fresh(f, lens, 1e-3);
        System.setProperty("sgm.test.bundlePageShift", "2");
        Object workspace = newWorkspace();
        Object paged = refill(damped(workspace, linearize(f, lens, workspace)), 1e-3);
        check(((double[][]) field(field(paged, "u"), "cross")).length > 100, "Small pages exercise many page boundaries");
        System.clearProperty("sgm.test.bundlePageShift");
        double[] a = x(call(paged, "solve", new Class<?>[0])), b = x(call(reference, "solve", new Class<?>[0]));
        check(Arrays.equals(a, b), "Page size does not change the solution (shared " + shared + ")");
        int n = n(paged);
        check(Arrays.equals((double[]) call(paged, "reducedMatrix", new Class<?>[] {int.class}, n),
            (double[]) call(reference, "reducedMatrix", new Class<?>[] {int.class}, n)), "Page size does not change the reduced matrix");
      }
    } finally {
      if (previous == null) System.clearProperty("sgm.test.bundlePageShift");
      else System.setProperty("sgm.test.bundlePageShift", previous);
    }
  }

  /** A singular point block fails the refill; the workspace is unusable until refilled, then exact. */
  private static void failedRefill(Path dir) throws Exception {
    try (var f = new AlignmentEquivalenceDigest.Grid(dir, 9, 300)) {
      // A triangulated track seen only by an unregistered image has a zero point block.
      FeatureTracks.Track lonely = new FeatureTracks.Track();
      lonely.point = new double[] {0, 0, 7};
      ImageFrame hidden = f.frames.get(8);
      hidden.camera.registered = false;
      lonely.observations.add(new FeatureTracks.Observation(8, 0));
      f.tracks.add(lonely);
      Object workspace = newWorkspace();
      Object damped = damped(workspace, linearize(f, null, workspace));
      boolean failed = false;
      try {
        refill(damped, 0);
      } catch (IllegalStateException expected) {
        failed = expected.getMessage().contains("Singular");
      }
      check(failed, "Undamped singular point block fails the refill");
      boolean refused = false;
      try {
        call(damped, "solve", new Class<?>[0]);
      } catch (IllegalStateException expected) {
        refused = true;
      }
      check(refused, "A failed refill leaves no usable stale system");
      refill(damped, 1e-3);
      check(Arrays.equals(x(call(damped, "solve", new Class<?>[0])), x(call(fresh(f, null, 1e-3), "solve", new Class<?>[0]))),
          "The next refill after a failure equals a fresh system");
      linearize(f, null, workspace);
      refused = false;
      try {
        call(damped, "solve", new Class<?>[0]);
      } catch (IllegalStateException expected) {
        refused = true;
      }
      check(refused, "A relinearized workspace refuses its damped system until refilled");
    }
  }

  /** Measured calibration points: the exact doubles of distorted(), two per keypoint; released on request. */
  private static void measured(Path dir) throws Exception {
    Files.createDirectories(dir);
    Options o = TestSupport.options();
    o.output = dir;
    Calibration c = new Calibration(650, 652, 318.25, 241.5, new double[] {-.12, .03, .001, -.002, .004}, 640, 480);
    List<ImageFrame> frames = SharedCalibrationRegression.frames(dir, c, o);
    try {
      SharedCalibrationRegression.scene(frames, c);
      SharedCalibration lens = new SharedCalibration(frames);
      boolean exact = true;
      long count = 0;
      for (ImageFrame f : frames) {
        exact &= lens.observed[f.id].length == 2 * f.keypoints.length;
        for (int i = 0; i < f.keypoints.length; i++, count++) {
          Point p = SharedCalibration.distorted(f.camera, f.calibration.distortion, f.keypoints[i].pt);
          exact &= Double.doubleToRawLongBits(p.x) == Double.doubleToRawLongBits(lens.observed[f.id][2 * i])
              && Double.doubleToRawLongBits(p.y) == Double.doubleToRawLongBits(lens.observed[f.id][2 * i + 1]);
        }
      }
      check(exact && count > 1000, "Measured points equal distorted() bit for bit (" + count + " points)");
      lens.releaseObservations();
      FeatureTracks.Track probe = new FeatureTracks.Track();
      probe.point = new double[] {0, 0, 5};
      probe.observations.add(new FeatureTracks.Observation(0, 0));
      boolean refused = false;
      try {
        lens.validationRms(frames, List.of(probe));
      } catch (NullPointerException expected) {
        refused = true;
      }
      check(refused, "Released measured points are not read silently");
    } finally {
      for (ImageFrame f : frames) f.close();
    }
  }

  private static void estimates() {
    check(AlignmentMemory.bundlePointBytes(26) > AlignmentMemory.bundlePointBytes(13)
        && AlignmentMemory.bundlePointBytes(13) > AlignmentMemory.bundlePointBytes(0),
        "Point estimate grows with the lens width (camera groups)");
    // 9 + 3 + 3 L undamped doubles, 9 inverse and 3 scratch doubles, int offset, two references, new point array.
    check(AlignmentMemory.bundlePointBytes(13) == (12 + 39 + 9 + 3) * 8 + 4 + 8 + 8 + 40, "Point estimate matches the representation");
    check(AlignmentMemory.BUNDLE_EDGE_BYTES == 156, "Edge estimate: 18 doubles and three int indices");
    Options o = TestSupport.options();
    o.memoryMb = 4096;
    check(AlignmentMemory.bundleWorkspaceBytes(o, 26, 40, 100000) > AlignmentMemory.bundleWorkspaceBytes(o, 0, 40, 100000)
        && AlignmentMemory.bundleWorkspaceBytes(o, 0, 200, 100000) < AlignmentMemory.bundleWorkspaceBytes(o, 0, 40, 100000),
        "Bounded workspace estimate includes the lens width and the direct solve only when eligible");
  }
}
