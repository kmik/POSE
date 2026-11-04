package sgm;

import java.lang.reflect.*;
import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;

/** Independent Schur-operator, residual, memory-fallback and large-block recovery controls. */
final class BundleSchurRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Core.setNumThreads(1);
    if (args.length > 0) benchmark(Path.of(args[0]));
    else {
      Path root = Files.createTempDirectory(Path.of("target"), "bundle schur ");
      System.out.println("PASS bundle Schur: " + run(root));
    }
  }

  static int run(Path root) throws Exception {
    checks = 0;
    try (Fixture f = new Fixture(root.resolve("schur"), 9, 300)) {
      for (boolean shared : new boolean[] {false, true}) {
        SharedCalibration lens = shared ? new SharedCalibration(f.frames) : null;
        if (lens != null) Arrays.fill(lens.active, true);
        Object system = linearize(f, lens);
        int n = 8 * 6 + (shared ? SharedCalibration.SIZE : 0);
        Method product = method(system, "product", double[].class);
        double[] h = (double[]) method(system, "reducedMatrix", int.class).invoke(system, n);
        Random random = new Random(908);
        for (int trial = 0; trial < 8; trial++) {
          double[] x = new double[n];
          for (int i = 0; i < n; i++) x[i] = random.nextGaussian();
          double[] implicit = (double[]) product.invoke(system, (Object) x);
          for (int i = 0; i < n; i++) {
            double sum = 0, magnitude = 0;
            for (int j = 0; j < n; j++) {
              sum += h[i * n + j] * x[j];
              magnitude += Math.abs(h[i * n + j] * x[j]);
            }
            check(
                Math.abs(sum - implicit[i]) < 1e-10 * Math.max(1, magnitude),
                "assembled operator agrees with point elimination");
          }
          // Use a realizable right-hand side and independently check the original operator.
          Object result = method(system, "solve", double[].class).invoke(system, (Object) implicit);
          check(result != null, "direct solve succeeds");
          check(
              method(result, "method").invoke(result).equals("direct_schur_cholesky"),
              "small block uses direct solve");
          double[] fitted = (double[]) method(result, "x").invoke(result);
          double[] actual = (double[]) product.invoke(system, (Object) fitted);
          double err = 0, norm = 0;
          for (int i = 0; i < n; i++) {
            err += Math.pow(actual[i] - implicit[i], 2);
            norm += implicit[i] * implicit[i];
          }
          check(
              Math.sqrt(err / norm) < 1e-6, "independent linear residual meets original tolerance");
        }
        f.options.memoryMb = 0; // Internal stress injection; public CLI rejects this budget.
        Object bounded = linearize(f, lens);
        Object fallback = method(bounded, "solve").invoke(bounded);
        check(
            fallback != null
                && method(fallback, "method").invoke(fallback).equals("matrix_free_pcg"),
            "insufficient workspace retains matrix-free solve");
        f.options.memoryMb = 4096;
      }
    }
    // Default (no --bundle-iterations): iterate until converged, not for a fixed budget.
    double fixed;
    try (Fixture f = new Fixture(root.resolve("fixed-budget"), 9, 300)) {
      BundleAdjuster.optimizeImpl(f.frames, f.tracks, 2, 2, null, 0, 1, null);
      fixed = BundleAdjuster.error(f.frames, f.tracks, 2);
    }
    try (Fixture f = new Fixture(root.resolve("converge"), 9, 300)) {
      f.options.bundleIterationsExplicit = false;
      BundleAdjuster.optimizeImpl(f.frames, f.tracks, 2, 2, null, 0, 1, null);
      double converged = BundleAdjuster.error(f.frames, f.tracks, 2);
      check(converged < fixed, "convergence mode runs past the nominal budget");
      f.options.bundleIterationsExplicit = true;
      BundleAdjuster.optimizeImpl(f.frames, f.tracks, 5, 2, null, 0, 1, null);
      check(
          converged - BundleAdjuster.error(f.frames, f.tracks, 2) < 1e-4 * Math.max(1, converged),
          "convergence mode stops only when further iterations gain little");
    }
    try (Fixture f = new Fixture(root.resolve("large-schur"), 132, 660)) {
      Object system = linearize(f, null);
      Object result = method(system, "solve").invoke(system);
      check(
          result != null && method(result, "method").invoke(result).equals("matrix_free_pcg"),
          "large camera block retains bounded matrix-free solver");
    }
    return checks;
  }

  private static Method method(Object target, String name, Class<?>... args) throws Exception {
    Method m = target.getClass().getDeclaredMethod(name, args);
    m.setAccessible(true);
    return m;
  }

  private static Object linearize(Fixture f, SharedCalibration lens) throws Exception {
    Method m =
        BundleAdjuster.class.getDeclaredMethod(
            "linearize",
            List.class,
            List.class,
            int[].class,
            int.class,
            double.class,
            double.class,
            SharedCalibration.class);
    m.setAccessible(true);
    int[] index = new int[f.frames.size()];
    index[0] = -1;
    for (int i = 1; i < index.length; i++) index[i] = i - 1;
    return m.invoke(null, f.frames, f.tracks, index, index.length - 1, 2., 1e-3, lens);
  }

  private static void benchmark(Path output) throws Exception {
    try (Fixture f = new Fixture(output, 89, 33000)) {
      SharedCalibration lens = new SharedCalibration(f.frames);
      lens.active[0] = true;
      lens.active[1] = true;
      lens.active[2] = true;
      double before = BundleAdjuster.error(f.frames, f.tracks, 2);
      long started = System.nanoTime();
      BundleAdjuster.optimizeImpl(f.frames, f.tracks, 8, 2, null, 0, 1, lens);
      double seconds = (System.nanoTime() - started) / 1e9;
      double after = 0;
      for (var t : f.tracks)
        for (var ob : t.observations) {
          double[] uv = lens.project(f.frames.get(ob.image), t.point);
          Point p = lens.observed[ob.image][ob.feature];
          after += Math.pow(uv[0] - p.x, 2) + Math.pow(uv[1] - p.y, 2);
        }
      Json.write(
          output.resolve("benchmark.json"),
          Json.object(
              "cameras",
              89,
              "points",
              33000,
              "iterations",
              8,
              "seconds",
              seconds,
              "initial_robust_objective",
              before,
              "final_rms_pixels",
              Math.sqrt(after / (33000 * 4)),
              "calibration",
              lens.values));
      System.out.println("BENCHMARK " + Files.readString(output.resolve("benchmark.json")));
    }
  }

  private static final class Fixture implements AutoCloseable {
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
            Math3.rotation(
                new double[] {.01 * Math.sin(i), .005 * Math.cos(i), .002 * i / cameras});
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
        track.point =
            new double[] {
              p[0] + random.nextGaussian() * .02,
              p[1] + random.nextGaussian() * .02,
              p[2] + random.nextGaussian() * .05
            };
        tracks.add(track);
        for (int j = 0; j < 4; j++) {
          int view = (owner + j) % cameras;
          double[] uv = truth.get(view).project(p);
          track.observations.add(new FeatureTracks.Observation(view, keys.get(view).size()));
          keys.get(view).add(new KeyPoint((float) uv[0], (float) uv[1], 1));
        }
      }
      for (int i = 0; i < cameras; i++) {
        frames.get(i).keypoints = keys.get(i).toArray(KeyPoint[]::new);
        if (i > 1) {
          Camera c = frames.get(i).camera;
          c.rotation = Math3.mul(Math3.rotation(new double[] {.003, -.004, .002}), c.rotation);
          c.translation =
              Math3.add(c.translation, new double[] {.005 * Math.sin(i), .005 * Math.cos(i), .005});
        }
      }
    }

    public void close() {
      for (ImageFrame frame : frames) frame.close();
    }
  }
}
