package sgm;

import java.lang.reflect.*;
import java.nio.file.*;
import java.security.MessageDigest;
import java.util.*;
import java.util.concurrent.*;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;

/**
 * --bundle-solver and its switches: the parallel dense Cholesky (factor accuracy, determinism
 * across thread counts), direct-versus-iterative selection at the limit, cluster-Jacobi PCG,
 * fast and legacy fits reaching the same optimum, local bundles holding the fixed cameras, the
 * option resolution, and a frozen digest of legacy fits (byte-identical to the previous solver).
 */
final class BundleSolverRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Core.setNumThreads(1);
    Path root = Files.createTempDirectory(Path.of(System.getProperty("sgm.test.root", "target")), "bundle solver ");
    if (args.length > 0 && args[0].equals("digest")) {
      // Prints the legacy digest with whatever build is on the classpath (used to freeze it).
      System.out.println("LEGACY_DIGEST " + legacyDigest(root));
      return;
    }
    System.out.println("PASS bundle solver: " + run(root));
  }

  /**
   * SHA-256 of every camera and point double (raw bits) and objective after legacy fits: a
   * 9-camera block (direct solve) and a 132-camera block (PCG), each unshared and with a shared
   * lens. Frozen from the solver before --bundle-solver existed.
   */
  static final String LEGACY_DIGEST = "2c667064517a798208d6dab58bf3a2e2f9c8dc605559fc5bcd6ee361b7cc6284";

  static int run(Path root) throws Exception {
    checks = 0;
    options();
    cholesky();
    selection(root.resolve("selection"));
    sameOptimum(root.resolve("optimum"));
    determinism(root.resolve("determinism"));
    localBundle(root.resolve("local"));
    memory();
    String digest = legacyDigest(root.resolve("legacy"));
    check(digest.equals(LEGACY_DIGEST), "legacy fits byte-identical to the previous solver: " + digest);
    return checks;
  }

  private static void options() {
    Options defaults = new Options();
    check(defaults.bundleSolverLegacy(), "default options are the legacy solver");
    check(defaults.bundleSolverKey().isEmpty(), "legacy solver adds nothing to cache keys");
    check(BundleAdjuster.SolverConfig.of(defaults) == BundleAdjuster.SolverConfig.LEGACY,
        "default options select the legacy linear solver");
    Options fast = parse("--bundle-solver", "fast");
    check(fast.bundleDirectLimit == Options.FAST_BUNDLE_DIRECT_LIMIT
        && fast.bundlePreconditioner.equals("cluster-jacobi") && fast.bundleInexactSteps
        && !fast.registrationLocalBundles && fast.lensFitStop.equals("objective"),
        "fast sets the solver switches, not local bundles or the lens-fit stop");
    Options all = parse("--bundle-solver", "fast", "--registration-local-bundles", "on", "--lens-fit-stop", "parameters");
    check(all.registrationLocalBundles && all.lensFitStop.equals("parameters"), "opt-in switches combine with fast");
    check(!fast.bundleSolverKey().isEmpty(), "fast enters the cache keys");
    Options partial = parse("--bundle-inexact-steps", "off", "--bundle-solver", "fast", "--bundle-direct-limit", "100");
    check(partial.bundleDirectLimit == 100 && !partial.bundleInexactSteps
        && partial.bundlePreconditioner.equals("cluster-jacobi"),
        "explicit switches override fast in any argument order");
    Options one = parse("--bundle-direct-limit", "2000");
    check(one.bundleDirectLimit == 2000 && one.bundlePreconditioner.equals("block-jacobi")
        && !one.bundleSolverLegacy(), "a single switch alone");
    check(parse("--bundle-direct-limit", "legacy").bundleSolverLegacy(), "direct limit legacy keyword");
    boolean rejected = false;
    try {
      parse("--bundle-preconditioner", "ilu");
    } catch (IllegalArgumentException e) {
      rejected = true;
    }
    check(rejected, "unknown preconditioner rejected");
    String help = Options.help();
    for (String flag : List.of("--bundle-solver", "--bundle-direct-limit", "--bundle-preconditioner",
        "--bundle-inexact-steps", "--registration-local-bundles", "--lens-fit-stop"))
      check(help.contains("  " + flag + " "), "help documents " + flag);
  }

  private static Options parse(String... extra) {
    List<String> all = new ArrayList<>(List.of("-i", ".", "-o", "bundle-solver-check"));
    all.addAll(List.of(extra));
    return Options.parse(all.toArray(String[]::new));
  }

  /** A random sparse-coupled SPD matrix: factor accuracy, zero-tile skipping, thread count. */
  private static void cholesky() throws Exception {
    int n = 333;
    Random random = new Random(4401);
    double[] g = new double[n * n];
    // Banded blocks plus a few far couplings: some tiles stay zero, others fill in.
    for (int i = 0; i < n; i++)
      for (int j = Math.max(0, i - 40); j <= i; j++) g[i * n + j] = random.nextGaussian();
    for (int k = 0; k < 20; k++) g[(n - 1 - k) * n + k] = random.nextGaussian();
    double[] a = new double[n * n];
    for (int i = 0; i < n; i++)
      for (int j = 0; j <= i; j++) {
        double s = 0;
        for (int k = 0; k < n; k++) s += g[i * n + k] * g[j * n + k];
        a[i * n + j] = a[j * n + i] = s + (i == j ? n : 0);
      }
    double[] one = factorIn(1, a, n), many = factorIn(6, a, n);
    check(one != null && Arrays.equals(one, many), "dense Cholesky identical with 1 and 6 threads");
    double[] b = new double[n], x = new double[n];
    for (int i = 0; i < n; i++) b[i] = random.nextGaussian();
    DenseCholesky.solve(one, n, b, x);
    double err = 0, norm = 0;
    for (int i = 0; i < n; i++) {
      double s = 0;
      for (int j = 0; j < n; j++) s += a[i * n + j] * x[j];
      err += (s - b[i]) * (s - b[i]);
      norm += b[i] * b[i];
    }
    check(Math.sqrt(err / norm) < 1e-12, "dense Cholesky solves the system: " + Math.sqrt(err / norm));
    double[] indefinite = a.clone();
    indefinite[(n / 2) * n + n / 2] = -1e6;
    check(!DenseCholesky.factor(indefinite, n, true), "indefinite matrix rejected");
    // Trailing marginal block equals the inverse's trailing block.
    double[] inverseTail = DenseCholesky.trailingInverse(one, n, 3);
    double[] e = new double[n];
    for (int k = 0; k < 3; k++) {
      Arrays.fill(e, 0);
      e[n - 3 + k] = 1;
      DenseCholesky.solve(one, n, e, e);
      for (int i = 0; i < 3; i++)
        check(Math.abs(e[n - 3 + i] - inverseTail[i * 3 + k]) < 1e-12 * Math.abs(inverseTail[k * 3 + k]),
            "trailing marginal covariance block");
    }
  }

  private static double[] factorIn(int threads, double[] a, int n) throws Exception {
    ForkJoinPool pool = new ForkJoinPool(threads);
    try {
      return pool.submit(() -> {
        double[] copy = a.clone();
        return DenseCholesky.factor(copy, n, true) ? copy : null;
      }).get();
    } finally {
      pool.shutdown();
    }
  }

  /** Dense at and below the limit, PCG above it, both preconditioners; the solutions agree. */
  private static void selection(Path dir) throws Exception {
    try (Fixture f = new Fixture(dir, 9, 300)) {
      for (boolean shared : new boolean[] {false, true}) {
        SharedCalibration lens = shared ? new SharedCalibration(f.frames) : null;
        if (lens != null) Arrays.fill(lens.active, true);
        int n = 8 * 6 + (shared ? SharedCalibration.SIZE : 0);
        f.options.bundleDirectLimit = -1;
        f.options.bundlePreconditioner = "block-jacobi";
        Object legacy = solve(f, lens, null);
        check(method(legacy).equals("direct_schur_cholesky"), "legacy direct below 768");
        f.options.bundleDirectLimit = n;
        Object dense = solve(f, lens, null);
        check(method(dense).equals("dense_cholesky"), "dense Cholesky at the limit");
        f.options.bundleDirectLimit = n - 1;
        Object block = solve(f, lens, null);
        check(method(block).equals("matrix_free_pcg"), "block-Jacobi PCG above the limit");
        f.options.bundlePreconditioner = "cluster-jacobi";
        Object cluster = solve(f, lens, null);
        check(method(cluster).equals("cluster_jacobi_pcg"), "cluster-Jacobi PCG above the limit");
        for (Object other : List.of(dense, block, cluster))
          check(relativeDifference(x(legacy), x(other)) < (other == dense ? 1e-8 : 1e-3), "solvers agree on the step: " + method(other) + " " + relativeDifference(x(legacy), x(other)) + " residual " + relative(other) + "/" + relative(legacy) + " iterations " + iterations(other));
        check(relative(dense) < 1e-6 && relative(cluster) < 1e-6, "solves meet the tolerance");
        f.options.bundleDirectLimit = n;
        String previous = System.getProperty("sgm.test.bundleDenseBytes");
        System.setProperty("sgm.test.bundleDenseBytes", "0");
        try {
          check(!method(solve(f, lens, null)).equals("dense_cholesky"), "dense matrix bounded by memory");
        } finally {
          if (previous == null) System.clearProperty("sgm.test.bundleDenseBytes");
          else System.setProperty("sgm.test.bundleDenseBytes", previous);
        }
        f.options.bundleDirectLimit = -1;
        f.options.bundlePreconditioner = "block-jacobi";
      }
    }
    // Large block (above the legacy direct limit): cluster-Jacobi needs fewer PCG iterations.
    try (Fixture f = new Fixture(dir.resolve("large"), 132, 660)) {
      f.options.bundleDirectLimit = 0;
      Object block = solve(f, null, null);
      f.options.bundlePreconditioner = "cluster-jacobi";
      Object cluster = solve(f, null, null);
      check(method(cluster).equals("cluster_jacobi_pcg"), "large block uses cluster-Jacobi PCG");
      check(iterations(cluster) <= iterations(block),
          "cluster-Jacobi needs no more iterations: " + iterations(cluster) + " vs " + iterations(block));
      f.options.bundleDirectLimit = 6000;
      Object dense = solve(f, null, null);
      check(method(dense).equals("dense_cholesky"), "large block below the raised limit is dense");
      check(relativeDifference(x(dense), x(cluster)) < 1e-3, "dense and PCG agree on the large block: " + relativeDifference(x(dense), x(cluster)));
    }
  }

  /** Legacy and fast fits (every switch) converge to the same cameras and objective. */
  private static void sameOptimum(Path dir) throws Exception {
    for (boolean shared : new boolean[] {false, true}) {
      double[][] legacy, fast;
      double legacyObjective, fastObjective;
      double[] legacyLens = null, fastLens = null;
      try (Fixture f = new Fixture(dir.resolve("legacy-" + shared), 30, 900)) {
        f.options.bundleIterationsExplicit = false;
        SharedCalibration lens = lens(f, shared);
        BundleAdjuster.optimizeImpl(f.frames, f.tracks, 8, 2, null, 0, 1, lens);
        legacy = centers(f);
        legacyObjective = BundleAdjuster.objective(f.frames, f.tracks, 2, lens);
        if (lens != null) legacyLens = lens.values.clone();
      }
      try (Fixture f = new Fixture(dir.resolve("fast-" + shared), 30, 900)) {
        f.options.bundleIterationsExplicit = false;
        f.options.applyFastBundleSolver(Set.of());
        SharedCalibration lens = lens(f, shared);
        BundleAdjuster.optimizeImpl(f.frames, f.tracks, 8, 2, null, 0, 1, lens);
        fast = centers(f);
        fastObjective = BundleAdjuster.objective(f.frames, f.tracks, 2, lens);
        if (lens != null) fastLens = lens.values.clone();
      }
      double worst = 0;
      for (int i = 0; i < legacy.length; i++)
        worst = Math.max(worst, Math.sqrt(Math.pow(legacy[i][0] - fast[i][0], 2)
            + Math.pow(legacy[i][1] - fast[i][1], 2) + Math.pow(legacy[i][2] - fast[i][2], 2)));
      // The block spans about 2.5 units; 1e-4 of it is far below the pixel noise.
      check(worst < 2.5e-4, "fast and legacy camera centres agree (shared=" + shared + "): " + worst);
      check(Math.abs(fastObjective - legacyObjective) <= 1e-3 * legacyObjective,
          "fast and legacy objectives agree (shared=" + shared + "): " + fastObjective + " vs " + legacyObjective);
      if (shared)
        for (int k = 0; k < 3; k++)
          check(Math.abs(fastLens[k] - legacyLens[k]) <= 1e-3 * Math.max(1, Math.abs(legacyLens[k])),
              "fast and legacy lens agree, parameter " + k + ": " + fastLens[k] + " vs " + legacyLens[k]);
    }
  }

  private static SharedCalibration lens(Fixture f, boolean shared) {
    if (!shared) return null;
    SharedCalibration lens = new SharedCalibration(f.frames);
    lens.active[0] = lens.active[1] = lens.active[2] = true;
    return lens;
  }

  /** The fast path gives bit-identical fits with 1 and 5 worker threads (dense and PCG). */
  private static void determinism(Path dir) throws Exception {
    for (int limit : new int[] {6000, 0}) {
      String one = fastFitIn(1, dir.resolve("one-" + limit), limit), five = fastFitIn(5, dir.resolve("five-" + limit), limit);
      check(one.equals(five), "fast fit identical with 1 and 5 threads (direct limit " + limit + ")");
    }
  }

  private static String fastFitIn(int threads, Path dir, int limit) throws Exception {
    ForkJoinPool pool = new ForkJoinPool(threads);
    try {
      return pool.submit(() -> {
        try (Fixture f = new Fixture(dir, 140, 1400)) {
          f.options.applyFastBundleSolver(Set.of());
          f.options.bundleDirectLimit = limit;
          SharedCalibration lens = new SharedCalibration(f.frames);
          lens.active[0] = lens.active[1] = true;
          BundleAdjuster.optimizeImpl(f.frames, f.tracks, 4, 2, null, 0, 1, lens);
          return digest(f, lens);
        }
      }).get();
    } finally {
      pool.shutdown();
    }
  }

  /** A local bundle moves only its variable cameras, improves the objective, restores flags. */
  private static void localBundle(Path dir) throws Exception {
    try (Fixture f = new Fixture(dir, 30, 900)) {
      f.options.applyFastBundleSolver(Set.of());
      boolean[] variable = new boolean[f.frames.size()];
      for (int i = 10; i < 16; i++) variable[i] = true;
      List<FeatureTracks.Track> local = new ArrayList<>();
      for (FeatureTracks.Track t : f.tracks)
        for (FeatureTracks.Observation ob : t.observations)
          if (variable[ob.image]) {
            local.add(t);
            break;
          }
      List<Camera> before = new ArrayList<>();
      for (ImageFrame frame : f.frames) before.add(frame.camera.copy());
      for (ImageFrame frame : f.frames) frame.camera.fixed = frame.id == 0;
      double objective = BundleAdjuster.error(f.frames, f.tracks, 2);
      BundleAdjuster.optimizeLocal(f.frames, local, 10, 2, 0, 1, variable);
      boolean fixedKept = true, moved = false;
      for (ImageFrame frame : f.frames) {
        boolean same = Arrays.equals(frame.camera.rotation, before.get(frame.id).rotation)
            && Arrays.equals(frame.camera.translation, before.get(frame.id).translation);
        if (!variable[frame.id] && !same) fixedKept = false;
        if (variable[frame.id] && !same) moved = true;
        check(frame.camera.fixed == (frame.id == 0), "fixed flags restored after a local bundle");
      }
      check(fixedKept, "local bundle leaves the other cameras bit-identical");
      check(moved, "local bundle moves its variable cameras");
      check(BundleAdjuster.error(f.frames, f.tracks, 2) < objective, "local bundle improves the objective");
    }
  }

  /** Workspace planning: legacy unchanged, the dense matrix counted when the fast path uses it. */
  private static void memory() {
    Options legacy = new Options();
    legacy.memoryMb = 16000;
    Options fast = new Options();
    fast.memoryMb = 16000;
    fast.applyFastBundleSolver(Set.of());
    long base = AlignmentMemory.bundleWorkspaceBytes(legacy, 0, 335, 1_000_000);
    long dense = AlignmentMemory.bundleWorkspaceBytes(fast, 0, 335, 1_000_000);
    long n = 6L * 335;
    check(dense - base >= 8 * n * n, "dense workspace counts the n^2 matrix");
    long above = AlignmentMemory.bundleWorkspaceBytes(fast, 0, 1500, 1_000_000);
    check(above < 8L * 9000 * 9000, "above the limit no dense matrix is planned");
    check(above > AlignmentMemory.bundleWorkspaceBytes(legacy, 0, 1500, 1_000_000),
        "cluster-Jacobi blocks are planned");
  }

  /** Legacy fits on four blocks, every double as raw bits (see {@link #LEGACY_DIGEST}). */
  static String legacyDigest(Path dir) throws Exception {
    MessageDigest sha = MessageDigest.getInstance("SHA-256");
    int scenario = 0;
    for (int[] size : new int[][] {{9, 300}, {132, 660}})
      for (boolean shared : new boolean[] {false, true}) {
        try (Fixture f = new Fixture(dir.resolve("digest-" + scenario++), size[0], size[1])) {
          SharedCalibration lens = null;
          if (shared) {
            lens = new SharedCalibration(f.frames);
            lens.active[0] = lens.active[1] = lens.active[2] = true;
          }
          BundleAdjuster.optimizeImpl(f.frames, f.tracks, 4, 2, null, 0, 1, lens);
          sha.update(digest(f, lens).getBytes(java.nio.charset.StandardCharsets.UTF_8));
          sha.update(Long.toHexString(Double.doubleToRawLongBits(
              BundleAdjuster.objective(f.frames, f.tracks, 2, lens))).getBytes());
        }
      }
    return HexFormat.of().formatHex(sha.digest());
  }

  private static String digest(Fixture f, SharedCalibration lens) {
    StringBuilder b = new StringBuilder();
    for (ImageFrame frame : f.frames) {
      for (double v : frame.camera.rotation) b.append(Long.toHexString(Double.doubleToRawLongBits(v))).append(',');
      for (double v : frame.camera.translation) b.append(Long.toHexString(Double.doubleToRawLongBits(v))).append(',');
    }
    for (FeatureTracks.Track t : f.tracks)
      if (t.point != null) for (double v : t.point) b.append(Long.toHexString(Double.doubleToRawLongBits(v))).append(',');
    if (lens != null) for (double v : lens.values) b.append(Long.toHexString(Double.doubleToRawLongBits(v))).append(',');
    return b.toString();
  }

  private static double[][] centers(Fixture f) {
    double[][] c = new double[f.frames.size()][];
    for (ImageFrame frame : f.frames) c[frame.id] = frame.camera.center();
    return c;
  }

  private static Object solve(Fixture f, SharedCalibration lens, double[] rhs) throws Exception {
    Method m = BundleAdjuster.class.getDeclaredMethod("linearize", List.class, List.class, int[].class,
        int.class, double.class, double.class, SharedCalibration.class);
    m.setAccessible(true);
    int[] index = new int[f.frames.size()];
    index[0] = -1;
    for (int i = 1; i < index.length; i++) index[i] = i - 1;
    Object system = m.invoke(null, f.frames, f.tracks, index, index.length - 1, 2., 1e-3, lens);
    Method solve = system.getClass().getDeclaredMethod("solve", double[].class);
    solve.setAccessible(true);
    Object result = solve.invoke(system, (Object) rhs);
    check(result != null, "linear solve succeeds");
    return result;
  }

  private static Object field(Object record, String name) throws Exception {
    Method m = record.getClass().getDeclaredMethod(name);
    m.setAccessible(true);
    return m.invoke(record);
  }

  private static String method(Object result) throws Exception {
    return (String) field(result, "method");
  }

  private static double[] x(Object result) throws Exception {
    return (double[]) field(result, "x");
  }

  private static int iterations(Object result) throws Exception {
    return (Integer) field(result, "iterations");
  }

  private static double relative(Object result) throws Exception {
    return (Double) field(result, "relative");
  }

  private static double relativeDifference(double[] a, double[] b) {
    double d = 0, n = 0;
    for (int i = 0; i < a.length; i++) {
      d += (a[i] - b[i]) * (a[i] - b[i]);
      n += a[i] * a[i];
    }
    return Math.sqrt(d / Math.max(1e-300, n));
  }

  /** Grid block as in BundleSchurRegression: perturbed cameras, four views per point. */
  static final class Fixture implements AutoCloseable {
    final Options options = TestSupport.options();
    final List<ImageFrame> frames = new ArrayList<>();
    final List<FeatureTracks.Track> tracks = new ArrayList<>();

    Fixture(Path dir, int cameras, int points) throws Exception {
      Files.createDirectories(dir);
      options.output = dir;
      options.maxDimension = 640;
      Path image = dir.resolve("blank.png");
      Mat blank = new Mat(480, 640, CvType.CV_8U, new Scalar(0));
      try {
        Imgcodecs.imwrite(image.toString(), blank);
      } finally {
        blank.release();
      }
      Calibration calibration = new Calibration(650, 650, 319.5, 239.5, new double[5], 640, 480);
      List<Camera> truth = new ArrayList<>();
      List<List<KeyPoint>> keys = new ArrayList<>();
      Random random = new Random(809123);
      for (int i = 0; i < cameras; i++) {
        ImageFrame frame = new ImageFrame(i, image, calibration, options, true);
        frames.add(frame);
        keys.add(new ArrayList<>());
        frame.camera.registered = true;
        frame.camera.rotation =
            Math3.rotation(new double[] {.01 * Math.sin(i), .005 * Math.cos(i), .002 * i / cameras});
        double[] center = {.25 * (i % 10), .25 * (i / 10), .03 * Math.sin(i * .7)};
        frame.camera.translation = Math3.times(Math3.mv(frame.camera.rotation, center), -1);
        truth.add(frame.camera.copy());
      }
      for (int i = 0; i < points; i++) {
        int owner = i % cameras;
        double[] p = {
          .25 * (owner % 10) + random.nextDouble() * 2 - 1,
          .25 * (owner / 10) + random.nextDouble() * 2 - 1,
          5 + random.nextDouble() * 5
        };
        FeatureTracks.Track track = new FeatureTracks.Track();
        track.id = i;
        track.point = new double[] {
          p[0] + random.nextGaussian() * .02, p[1] + random.nextGaussian() * .02, p[2] + random.nextGaussian() * .05
        };
        tracks.add(track);
        for (int j = 0; j < 4; j++) {
          // Neighbours along the row and in the next row, so cameras couple in two directions.
          int view = j < 2 ? (owner + j) % cameras : (owner + 10 + j - 2) % cameras;
          double[] uv = truth.get(view).project(p);
          track.observations.add(new FeatureTracks.Observation(view, keys.get(view).size()));
          // Half-pixel noise, deterministic.
          keys.get(view).add(new KeyPoint((float) (uv[0] + random.nextGaussian() * .5),
              (float) (uv[1] + random.nextGaussian() * .5), 1));
        }
      }
      for (int i = 0; i < cameras; i++) {
        frames.get(i).keypoints = keys.get(i).toArray(KeyPoint[]::new);
        if (i > 1) {
          Camera c = frames.get(i).camera;
          c.rotation = Math3.mul(Math3.rotation(new double[] {.003, -.004, .002}), c.rotation);
          c.translation = Math3.add(c.translation, new double[] {.005 * Math.sin(i), .005 * Math.cos(i), .005});
        }
      }
    }

    public void close() {
      for (ImageFrame frame : frames) frame.close();
    }
  }
}
