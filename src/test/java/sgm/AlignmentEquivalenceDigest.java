package sgm;

import java.io.*;
import java.lang.management.ManagementFactory;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.*;
import java.util.stream.*;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;

/**
 * Bit-level digest of alignment results for comparing two builds (e.g. before and after a memory
 * change). Uses only APIs that both builds share. Each scenario writes every camera, point and
 * lens double as raw bits, the per-trial solver log (lambda, PCG iterations, residual, accepted),
 * calibration reports without elapsed times, and the bundle console lines; then its total
 * allocated bytes and wall time (reported separately, never part of the digest).
 *
 * <p>Usage: AlignmentEquivalenceDigest OUTPUT_DIR [bench N]. Compare OUTPUT_DIR/digest-*.txt of
 * the two builds byte for byte.
 */
final class AlignmentEquivalenceDigest {
  private static final com.sun.management.ThreadMXBean THREADS =
      (com.sun.management.ThreadMXBean) ManagementFactory.getThreadMXBean();

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Core.setNumThreads(1);
    Path out = Path.of(args[0]).toAbsolutePath();
    Files.createDirectories(out);
    if (args.length > 1 && args[1].equals("bench")) {
      bench(out, Integer.parseInt(args[2]), args.length > 3 ? args[3] : "all");
      return;
    }
    if (args.length > 1 && args[1].equals("minheap")) {
      // One bundle on a larger grid; run under decreasing -Xmx to find the smallest heap that
      // completes (exit 0) for a build. Prints the final objective bits for a sanity check.
      int cameras = Integer.parseInt(args[2]), points = Integer.parseInt(args[3]);
      boolean shared = args.length > 4 && args[4].equals("shared");
      try (Grid f = new Grid(out.resolve("minheap-" + ProcessHandle.current().pid()), cameras, points)) {
        f.options.memoryMb = 100000; // only the heap limits this run, not admission budgets
        SharedCalibration lens = null;
        if (shared) {
          lens = new SharedCalibration(f.frames);
          for (int k : new int[] {0, 1, 2, 3, 4}) lens.active[k] = true;
        }
        SharedCalibration model = lens;
        capture(() -> {
          BundleAdjuster.optimizeImpl(f.frames, f.tracks, 3, 2, null, 0, 1, model);
          return "";
        });
        System.err.println("MINHEAP_OK " + hex(BundleAdjuster.objective(f.frames, f.tracks, 2, lens)));
      }
      return;
    }
    List<String> timing = new ArrayList<>();
    for (String name : SCENARIOS) {
      Path dir = out.resolve("work-" + name);
      Files.createDirectories(dir);
      long allocated = THREADS.getTotalThreadAllocatedBytes(), started = System.nanoTime();
      String digest = capture(() -> scenario(name, dir));
      double seconds = (System.nanoTime() - started) / 1e9;
      allocated = THREADS.getTotalThreadAllocatedBytes() - allocated;
      // Not results: the output location, progress rates and Properties timestamps.
      digest = digest.replace(out.toString(), "<out>").lines()
          .filter(l -> !l.endsWith("/s") && !l.contains("complete;")
              && !l.matches("#\\w{3} \\w{3} \\d+ .*"))
          .collect(Collectors.joining("\n", "", "\n"));
      Files.writeString(out.resolve("digest-" + name + ".txt"), digest);
      timing.add(String.format(Locale.ROOT, "%s,%.3f,%d", name, seconds, allocated));
      System.err.println("scenario " + name + " done");
    }
    Files.writeString(out.resolve("timing.csv"), "scenario,seconds,allocated_bytes\n" + String.join("\n", timing) + "\n");
  }

  static final List<String> SCENARIOS = List.of(
      "unshared-fixed", "unshared-converge", "unshared-rejections", "unshared-pcg", "unshared-large-pcg",
      "unshared-batches", "priors", "shared-direct", "shared-pcg", "shared-batches", "shared-groups",
      "antenna", "covariance", "refine-full", "refine-joint", "refine-subset");

  interface Body {
    String run() throws Exception;
  }

  /** Runs body with System.out captured; returns its digest plus the captured bundle lines. */
  private static String capture(Body body) throws Exception {
    PrintStream original = System.out;
    ByteArrayOutputStream bytes = new ByteArrayOutputStream();
    String digest;
    try (PrintStream stream = new PrintStream(bytes, true, StandardCharsets.UTF_8)) {
      System.setOut(stream);
      digest = body.run();
    } finally {
      System.setOut(original);
    }
    String console = bytes.toString(StandardCharsets.UTF_8).lines()
        .filter(l -> l.startsWith("Bundle") || l.startsWith("Shared alignment"))
        .map(l -> l.replaceAll("[0-9.]+ s\\b", "<t>"))
        .collect(Collectors.joining("\n"));
    return digest + "console\n" + console + "\n";
  }

  private static String scenario(String name, Path dir) throws Exception {
    String previousBatch = System.getProperty("sgm.test.bundleBatchObservations");
    String previousWorkspace = System.getProperty("sgm.test.bundleDirectWorkspaceBytes");
    String previousPage = System.getProperty("sgm.test.bundlePageShift");
    try {
      if (name.endsWith("-pcg")) System.setProperty("sgm.test.bundleDirectWorkspaceBytes", "0");
      if (name.endsWith("-batches")) {
        System.setProperty("sgm.test.bundleBatchObservations", "17");
        System.setProperty("sgm.test.bundlePageShift", "3");
      }
      return switch (name) {
        case "unshared-fixed" -> unshared(dir, 9, 300, 6, true, false, false);
        case "unshared-converge" -> unshared(dir, 9, 300, 2, false, false, false);
        case "unshared-rejections" -> unshared(dir, 9, 300, 40, true, false, true);
        case "unshared-pcg", "unshared-batches" -> unshared(dir, 9, 300, 5, true, false, true);
        case "unshared-large-pcg" -> unshared(dir, 132, 660, 3, true, false, false);
        case "priors" -> unshared(dir, 12, 400, 6, true, true, false);
        case "shared-direct", "shared-pcg", "shared-batches" -> shared(dir, false, 6);
        case "shared-groups" -> shared(dir, true, 6);
        case "antenna" -> antenna(dir);
        case "covariance" -> covariance(dir);
        case "refine-full", "refine-joint", "refine-subset" -> refine(dir, name.substring("refine-".length()));
        default -> throw new IllegalArgumentException(name);
      };
    } finally {
      restore("sgm.test.bundleBatchObservations", previousBatch);
      restore("sgm.test.bundleDirectWorkspaceBytes", previousWorkspace);
      restore("sgm.test.bundlePageShift", previousPage);
    }
  }

  private static void restore(String key, String value) {
    if (value == null) System.clearProperty(key);
    else System.setProperty(key, value);
  }

  // ---- Scenarios

  private static String unshared(Path dir, int cameras, int points, int iterations, boolean explicit,
      boolean priors, boolean outliers) throws Exception {
    try (Grid f = new Grid(dir, cameras, points)) {
      f.options.bundleIterationsExplicit = explicit;
      if (outliers) {
        // Gross outliers on every 6th observation and larger pose errors: robust steps overshoot,
        // so damping trials are rejected and restored.
        Random random = new Random(77);
        int n = 0;
        for (FeatureTracks.Track t : f.tracks)
          for (FeatureTracks.Observation ob : t.observations)
            if (n++ % 6 == 0) {
              KeyPoint k = f.frames.get(ob.image).keypoints[ob.feature];
              k.pt.x += 30 + 60 * random.nextDouble();
              k.pt.y -= 30 + 60 * random.nextDouble();
            }
        for (int i = 2; i < f.frames.size(); i++) {
          Camera c = f.frames.get(i).camera;
          c.rotation = Math3.mul(Math3.rotation(new double[] {.03 * Math.cos(i), -.02, .025}), c.rotation);
          c.translation = Math3.add(c.translation, new double[] {.08, -.05 * Math.sin(i), .06});
        }
      }
      if (priors) {
        // Camera centre/rotation priors on every camera, GCP priors on every 25th track.
        for (ImageFrame frame : f.frames) {
          Camera c = frame.camera;
          c.reference = f.truth.get(frame.id).copy();
          c.positionSigma = .05;
          c.heightSigma = .08;
          c.rotationSigma = .01;
        }
        for (int i = 0; i < f.tracks.size(); i += 25) {
          FeatureTracks.Track t = f.tracks.get(i);
          t.prior = f.truePoints.get(i).clone();
          t.priorSigma = new double[] {.02, .02, .03};
        }
      }
      StringBuilder out = new StringBuilder();
      out.append("before ").append(hex(BundleAdjuster.error(f.frames, f.tracks, 2))).append('\n');
      Project project = Project.open(f.options, new String[0]);
      try {
        BundleAdjuster.optimizeImpl(f.frames, f.tracks, iterations, 2, project, 0, 1, null);
        project.finish(true, null);
      } catch (Exception e) {
        project.finish(false, e);
        throw e;
      }
      out.append("after ").append(hex(BundleAdjuster.error(f.frames, f.tracks, 2))).append('\n');
      state(out, f.frames, f.tracks, null);
      Path solver = Files.walk(dir).filter(p -> p.getFileName().toString().equals("linear_solver.csv"))
          .findFirst().orElseThrow();
      out.append("linear_solver\n").append(Files.readString(solver));
      return out.toString();
    }
  }

  private static String shared(Path dir, boolean groups, int iterations) throws Exception {
    Calibration truth = new Calibration(650, 650, 319.5, 239.5, new double[] {-.1, .02, 0, 0, 0}, 640, 480);
    Calibration start = new Calibration(660, 660, 322, 237, new double[5], 640, 480);
    Options o = TestSupport.options();
    o.output = dir;
    o.cores = 4;
    if (groups) o.cameraGroups = twoGroups(start);
    List<ImageFrame> frames = SharedCalibrationRegression.frames(dir, start, o);
    try {
      List<FeatureTracks.Track> tracks = SharedCalibrationRegression.scene(frames, truth);
      perturb(tracks);
      frames.get(0).camera.fixed = true;
      SharedCalibration lens = new SharedCalibration(frames);
      for (int g = 0; g < lens.groups; g++)
        for (int k = 0; k < SharedCalibration.LENS; k++) lens.active[g * SharedCalibration.SIZE + k] = true;
      StringBuilder out = new StringBuilder();
      out.append("before ").append(hex(BundleAdjuster.objective(frames, tracks, 2, lens))).append('\n');
      BundleAdjuster.optimizeImpl(frames, tracks, iterations, 2, null, 0, 1, lens);
      out.append("after ").append(hex(BundleAdjuster.objective(frames, tracks, 2, lens))).append('\n');
      out.append("validation ").append(hex(lens.validationRms(frames, tracks))).append('\n');
      state(out, frames, tracks, lens);
      return out.toString();
    } finally {
      for (ImageFrame f : frames) f.close();
      Geometry.releaseWorkspaces();
    }
  }

  private static String antenna(Path dir) throws Exception {
    Calibration truth = new Calibration(650, 650, 319.5, 239.5, new double[5], 640, 480);
    Options o = TestSupport.options();
    o.output = dir;
    List<ImageFrame> frames = SharedCalibrationRegression.frames(dir, truth, o);
    try {
      List<FeatureTracks.Track> tracks = SharedCalibrationRegression.scene(frames, truth);
      perturb(tracks);
      for (ImageFrame f : frames) {
        Camera c = f.camera;
        c.reference = c.copy();
        c.reference.translation = Math3.add(c.translation, new double[] {.01, -.02, .015});
        c.positionSigma = .05;
        c.heightSigma = .1;
        c.rotationSigma = .02;
        c.velocity = new double[] {3 + .1 * f.id, -1, .05};
      }
      SharedCalibration lens = new SharedCalibration(frames);
      int[] gps = {SharedCalibration.READOUT, SharedCalibration.TIME, SharedCalibration.LEVER,
          SharedCalibration.LEVER + 1, SharedCalibration.LEVER + 2, 0};
      for (int k : gps) lens.active[k] = true;
      StringBuilder out = new StringBuilder();
      BundleAdjuster.optimizeImpl(frames, tracks, 6, 2, null, 0, 1, lens);
      out.append("after ").append(hex(BundleAdjuster.objective(frames, tracks, 2, lens))).append('\n');
      state(out, frames, tracks, lens);
      return out.toString();
    } finally {
      for (ImageFrame f : frames) f.close();
      Geometry.releaseWorkspaces();
    }
  }

  private static String covariance(Path dir) throws Exception {
    Calibration truth = new Calibration(650, 650, 319.5, 239.5, new double[] {-.1, .02, 0, 0, 0}, 640, 480);
    Options o = TestSupport.options();
    o.output = dir;
    List<ImageFrame> frames = SharedCalibrationRegression.frames(dir, truth, o);
    try {
      List<FeatureTracks.Track> tracks = SharedCalibrationRegression.scene(frames, truth);
      frames.get(0).camera.fixed = true;
      SharedCalibration lens = new SharedCalibration(frames);
      for (int k : new int[] {0, 1, 2, 3, 4}) lens.active[k] = true;
      StringBuilder out = new StringBuilder();
      for (double noise : new double[] {.35, 1}) {
        double[][] covariance = BundleAdjuster.calibrationCovariance(frames, tracks, 2, lens, noise);
        out.append("covariance ").append(noise).append('\n');
        if (covariance == null) out.append("null\n");
        else for (double[] row : covariance) out.append(hex(row)).append('\n');
      }
      var info = BundleAdjuster.calibrationDataInformation(frames, tracks, 2, lens);
      out.append("information\n");
      if (info == null) out.append("null\n");
      else out.append(hex(info.matrix())).append('\n').append(hex(info.observationDiagonal())).append('\n');
      var assessment = CalibrationObservability.assess(frames, tracks, 2, lens, .35);
      out.append("assessment ").append(assessment.reason()).append(' ')
          .append(hex(assessment.minimumRelativeInformation())).append('\n');
      if (assessment.sigma() != null) out.append(hex(assessment.sigma())).append('\n');
      return out.toString();
    } finally {
      for (ImageFrame f : frames) f.close();
      Geometry.releaseWorkspaces();
    }
  }

  private static String refine(Path dir, String method) throws Exception {
    Calibration truth = new Calibration(650, 650, 319.5, 239.5, new double[] {-.1, .02, 0, 0, 0}, 640, 480);
    Calibration start = new Calibration(663, 663, 321, 238, new double[5], 640, 480);
    Options o = TestSupport.options();
    o.output = dir;
    o.cores = 4;
    o.calibrationTrialMethod = method;
    o.alignmentCalibration = true;
    if (method.equals("subset")) o.calibrationTrialCameras = 5;
    List<ImageFrame> frames = SharedCalibrationRegression.frames(dir, start, o);
    try {
      List<FeatureTracks.Track> tracks = SharedCalibrationRegression.scene(frames, truth);
      perturb(tracks);
      StringBuilder out = new StringBuilder();
      SharedCalibration.refine(frames, tracks, 12, 2, null, 0, 1);
      state(out, frames, tracks, null);
      try (Stream<Path> files = Files.walk(dir)) {
        for (Path p : files.filter(Files::isRegularFile)
            .filter(p -> p.toString().endsWith(".json") || p.toString().endsWith(".csv")
                || p.getFileName().toString().equals("camera.txt"))
            .sorted().toList())
          out.append("file ").append(dir.relativize(p).toString().replace('\\', '/')).append('\n')
              .append(Files.readString(p).lines().filter(l -> !l.contains("elapsed"))
                  .collect(Collectors.joining("\n"))).append('\n');
      }
      return out.toString();
    } finally {
      for (ImageFrame f : frames) f.close();
      Geometry.releaseWorkspaces();
    }
  }

  // ---- Benchmark: repeated solves on a moderate synthetic block.

  private static void bench(Path out, int repeats, String which) throws Exception {
    List<String> rows = new ArrayList<>();
    for (String kind : List.of("unshared", "shared", "unshared-pcg")) {
      if (!which.equals("all") && !which.equals(kind)) continue;
      for (int r = -1; r < repeats; r++) { // r = -1: warm-up, not recorded
        Path dir = Files.createTempDirectory(out, "bench");
        String previous = System.getProperty("sgm.test.bundleDirectWorkspaceBytes");
        if (kind.endsWith("-pcg")) System.setProperty("sgm.test.bundleDirectWorkspaceBytes", "0");
        try (Grid f = new Grid(dir, 40, 12000)) {
          SharedCalibration lens = null;
          if (kind.equals("shared")) {
            lens = new SharedCalibration(f.frames);
            for (int k : new int[] {0, 1, 2, 3, 4}) lens.active[k] = true;
          }
          long allocated = THREADS.getTotalThreadAllocatedBytes(), started = System.nanoTime();
          SharedCalibration model = lens;
          capture(() -> {
            BundleAdjuster.optimizeImpl(f.frames, f.tracks, 6, 2, null, 0, 1, model);
            return "";
          });
          double seconds = (System.nanoTime() - started) / 1e9;
          allocated = THREADS.getTotalThreadAllocatedBytes() - allocated;
          if (r >= 0) rows.add(String.format(Locale.ROOT, "%s,%d,%.3f,%d", kind, r, seconds, allocated));
        } finally {
          restore("sgm.test.bundleDirectWorkspaceBytes", previous);
        }
      }
    }
    Path csv = out.resolve("bench.csv");
    Files.writeString(csv, "kind,repeat,seconds,allocated_bytes\n" + String.join("\n", rows) + "\n");
    System.err.println(Files.readString(csv));
  }

  // ---- Fixtures and serialization

  private static CameraGroups twoGroups(Calibration c) {
    CameraGroups.Group a = new CameraGroups.Group("camera_a", 640, 480), b = new CameraGroups.Group("camera_b", 640, 480);
    for (int i = 0; i < 8; i++) (i < 4 ? a : b).images.add("view" + i + ".png");
    a.calibration = c;
    b.calibration = c;
    return new CameraGroups(List.of(a, b));
  }

  private static void perturb(List<FeatureTracks.Track> tracks) {
    Random random = new Random(4411);
    for (FeatureTracks.Track t : tracks)
      t.point = Math3.add(t.point, new double[] {random.nextGaussian() * .01, random.nextGaussian() * .01, random.nextGaussian() * .02});
  }

  private static void state(StringBuilder out, List<ImageFrame> frames, List<FeatureTracks.Track> tracks,
      SharedCalibration lens) {
    for (ImageFrame f : frames) {
      Calibration c = f.calibration;
      out.append("camera ").append(f.id).append(' ').append(f.camera.registered).append(' ').append(f.camera.fixed)
          .append(' ').append(hex(f.camera.rotation)).append(' ').append(hex(f.camera.translation))
          .append(' ').append(hex(new double[] {c.fx, c.fy, c.cx, c.cy})).append(' ').append(hex(c.distortion))
          .append(' ').append(hex(new double[] {f.camera.fx, f.camera.fy, f.camera.cx, f.camera.cy})).append('\n');
    }
    for (FeatureTracks.Track t : tracks) out.append(t.point == null ? "null" : hex(t.point)).append('\n');
    if (lens != null) out.append("lens ").append(hex(lens.values)).append('\n');
  }

  static String hex(double v) {
    return Long.toHexString(Double.doubleToRawLongBits(v));
  }

  static String hex(double[] values) {
    StringBuilder b = new StringBuilder();
    for (double v : values) b.append(hex(v)).append(',');
    return b.toString();
  }

  /** Synthetic grid block (as BundleSchurRegression): 4 views per point, two perturbed seeds kept. */
  static final class Grid implements AutoCloseable {
    final Options options = TestSupport.options();
    final List<ImageFrame> frames = new ArrayList<>();
    final List<FeatureTracks.Track> tracks = new ArrayList<>();
    final List<Camera> truth = new ArrayList<>();
    final List<double[]> truePoints = new ArrayList<>();

    Grid(Path dir, int cameras, int points) throws Exception {
      Files.createDirectories(dir.resolve("in"));
      options.output = dir.resolve("out");
      options.input = dir.resolve("in");
      options.maxDimension = 640;
      Path image = dir.resolve("in").resolve("blank.png");
      Mat blank = new Mat(480, 640, CvType.CV_8U, new Scalar(0));
      try {
        Imgcodecs.imwrite(image.toString(), blank);
      } finally {
        blank.release();
      }
      Calibration calibration = new Calibration(650, 650, 319.5, 239.5, new double[5], 640, 480);
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
        truePoints.add(p);
        FeatureTracks.Track track = new FeatureTracks.Track();
        track.id = i;
        track.point = new double[] {
          p[0] + random.nextGaussian() * .02, p[1] + random.nextGaussian() * .02, p[2] + random.nextGaussian() * .05
        };
        tracks.add(track);
        for (int j = 0; j < 4; j++) {
          int view = (owner + j) % cameras;
          double[] uv = truth.get(view).project(p);
          track.observations.add(new FeatureTracks.Observation(view, keys.get(view).size()));
          keys.get(view).add(new KeyPoint((float) (uv[0] + random.nextGaussian() * .3), (float) (uv[1] + random.nextGaussian() * .3), 1));
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
