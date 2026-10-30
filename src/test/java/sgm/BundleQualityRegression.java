package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;

/** Independent camera/structure recovery, minimal gauge and real resolution/planning workflows. */
final class BundleQualityRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Core.setNumThreads(1);
    Path root = Files.createTempDirectory(Path.of("target"), "bundle quality ");
    System.out.println("PASS bundle/quality checks=" + run(root) + "; fixtures=" + root);
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Path dir = Files.createDirectory(root.resolve("bundle quality regression")),
        images = Files.createDirectory(dir.resolve("images"));
    int w = 512, h = 384;
    byte[] left = new byte[w * h], right = new byte[w * h];
    new Random(900).nextBytes(left);
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) right[y * w + x] = left[y * w + Math.min(w - 1, x + 20)];
    for (int i = 0; i < 2; i++) {
      Mat m = new Mat(h, w, CvType.CV_8U);
      try {
        m.put(0, 0, i == 0 ? left : right);
        Imgcodecs.imwrite(images.resolve(i + ".png").toString(), m);
      } finally {
        m.release();
      }
    }
    Path calibration = dir.resolve("camera.txt");
    Files.writeString(calibration, "width=512\nheight=384\nfx=360\nfy=360\ncx=255.5\ncy=191.5\n");
    quality(dir, images, calibration);
    bundle(dir, images);
    return checks;
  }

  private static Options parse(Path images, String... extra) {
    List<String> args = new ArrayList<>(List.of("-i", images.toString(), "--no-gps"));
    args.addAll(Arrays.asList(extra));
    return Options.parse(args.toArray(new String[0]));
  }

  private static void quality(Path dir, Path images, Path calibration) throws Exception {
    Options legacy = parse(images);
    check(
        Arrays.equals(legacy.processedSize(5280, 3956), new int[] {1600, 1199}),
        "Default preserves legacy 1600 cap");
    String[] presets = {"ultra", "high", "medium", "low"};
    for (int i = 0; i < presets.length; i++) {
      Options o = parse(images, "--quality", presets[i]);
      int factor = 1 << i;
      check(
          Arrays.equals(
              o.processedSize(5280, 3956),
              new int[] {5280 / factor, (int) Math.round(3956. / factor)}),
          "Preset uses original raster dimensions: " + presets[i]);
      check(
          Arrays.equals(
              o.processedSize(100, 80),
              new int[] {(int) Math.round(100. / factor), (int) Math.round(80. / factor)}),
          "Preset never upsamples small source: " + presets[i]);
      check(
          o.filteringSummary().equals(legacy.filteringSummary()),
          "Quality leaves filtering intensity unchanged");
      Options copy = o.copy();
      check(
          copy.quality.equals(o.quality)
              && Arrays.equals(copy.processedSize(5280, 3956), o.processedSize(5280, 3956)),
          "Copy preserves resolution plan");
    }
    Options a = parse(images, "--quality", "high", "--max-dimension", "1200"),
        b = parse(images, "--max-dimension", "1200", "--quality", "HIGH");
    check(
        Arrays.equals(a.processedSize(5280, 3956), b.processedSize(5280, 3956))
            && a.processedSize(5280, 3956)[0] == 1200,
        "Explicit resolution cap composes with quality independent of argument order");
    check(
        parse(images, "--quality", "low", "--max-dimension", "2000").processedSize(5280, 3956)[0]
            == 660,
        "Explicit cap never raises preset resolution");
    check(
        Arrays.equals(
            parse(images, "--max-dimension", "2000").processedSize(100, 80), new int[] {100, 80}),
        "Custom resolution never upsamples");
    boolean invalid = false;
    try {
      parse(images, "--quality", "extreme");
    } catch (IllegalArgumentException e) {
      invalid = true;
    }
    check(invalid, "Invalid preset rejected clearly");
    Calibration c = new Calibration(360, 360, 255.5, 191.5, new double[5], 512, 384);
    Options high = parse(images, "--quality", "high");
    try (ImageFrame f = new ImageFrame(0, images.resolve("0.png"), c, high)) {
      check(
          f.gray.cols() == 256 && f.gray.rows() == 192,
          "Image decode applies planned half resolution");
      check(
          f.camera.fx == 180 && f.camera.cx == 127.5 && f.camera.cy == 95.5,
          "Intrinsics scale consistently with decoded pixels");
    }
    Options plan =
        parse(
            images,
            "--quality",
            "ultra",
            "--estimate-memory",
            "--disparities",
            "4096",
            "--memory-mb",
            "16");
    plan.output = Files.createDirectory(dir.resolve("standalone plan"));
    ResolutionPlan.write(Main.images(plan), plan, null);
    String json = Files.readString(plan.output.resolve("00_inputs/resolution_plan.json"));
    check(
        json.contains("\"sgm_budget_bytes\":16777216")
            && json.contains("\"fits_sgm_budget\":false"),
        "Header-only plan reports oversized disparity scenarios without attempting allocation");
    plan.estimateMemory = false;
    boolean rejected = false;
    try {
      ResolutionPlan.write(Main.images(plan), plan, null);
    } catch (IllegalArgumentException e) {
      rejected = e.getMessage().contains("cannot fit");
    }
    check(rejected, "Actual reconstruction rejects an impossible explicit search before decoding");
    Path only = dir.resolve("planning project");
    check(
        TestSupport.run(
                new String[] {
                  "-i",
                  images.toString(),
                  "-o",
                  only.toString(),
                  "--quality",
                  "medium",
                  "--estimate-memory",
                  "--no-gps"
                })
            == 0,
        "Public planning-only invocation succeeds without calibration");
    Path onlyRun = latest(only);
    String run = Files.readString(onlyRun.resolve("run.json"));
    check(
        Files.exists(onlyRun.resolve("00_inputs/resolution_plan.json"))
            && !Files.exists(only.resolve("dense.las"))
            && run.contains("planning only"),
        "Planning-only project records skipped stages and emits no cloud");
    Path output = dir.resolve("half resolution project");
    check(
        TestSupport.run(
                new String[] {
                  "-i",
                  images.toString(),
                  "-o",
                  output.toString(),
                  "--rectified",
                  "--calibration",
                  calibration.toString(),
                  "--baseline",
                  ".25",
                  "--quality",
                  "high",
                  "--disparities",
                  "32",
                  "--step",
                  "1",
                  "--speckle-size",
                  "0",
                  "--no-gps"
                })
            == 0,
        "Half-resolution stereo completes through public CLI");
    Path actualRun = latest(output);
    check(
        Files.size(output.resolve("dense.las")) > 227 + 1000 * 26,
        "Half-resolution reconstruction retains dense geometry");
    String actual =
        Files.readString(actualRun.resolve("05_stereo/pair_0000_0001/disparity_range.json"));
    check(
        actual.contains("\"processed_matching_width\":256")
            && actual.contains(
                "\"sgm_estimated_bytes\":"
                    + CensusSgm.memoryEstimate(256, 192, 32, TestSupport.options())),
        "Per-pair memory record uses actual resolution and disparity count");
    check(
        Files.readString(actualRun.resolve("logs/run-settings.properties"))
            .contains("quality=high"),
        "Quality preset persists in project provenance");
    check(
        CensusSgm.memoryEstimate(256, 192, 32, TestSupport.options())
            < CensusSgm.memoryEstimate(512, 384, 32, TestSupport.options()) / 3,
        "Half-sized dimensions reduce SGM memory approximately fourfold at fixed disparity count");
  }

  private static Path latest(Path root) throws IOException {
    Properties p = new Properties();
    try (Reader r = Files.newBufferedReader(root.resolve("project.properties"))) {
      p.load(r);
    }
    return root.resolve("runs").resolve(p.getProperty("latest_run"));
  }

  private static void bundle(Path dir, Path images) throws Exception {
    Options o = TestSupport.options();
    o.input = images;
    o.output = dir.resolve("joint bundle project");
    o.maxDimension = 512;
    Calibration calibration = new Calibration(360, 360, 255.5, 191.5, new double[5], 512, 384);
    List<ImageFrame> frames = new ArrayList<>();
    List<Camera> truth = new ArrayList<>();
    List<FeatureTracks.Track> tracks = new ArrayList<>();
    List<double[]> points = new ArrayList<>();
    int n = 16, anchor = 3, scale = 9;
    Random random = new Random(51);
    try {
      for (int i = 0; i < n; i++) {
        ImageFrame f = new ImageFrame(i, images.resolve("0.png"), calibration, o);
        frames.add(f);
        f.camera.registered = true;
        f.camera.fixed = i == anchor || i == scale;
        f.camera.rotation =
            Math3.rotation(new double[] {.015 * Math.sin(i), .012 * (i - 3), -.007 * Math.cos(i)});
        double[] center = {3 + .12 * i, -2 + .10 * Math.sin(i * .6), .07 * Math.cos(i * .4)};
        f.camera.translation = Math3.times(Math3.mv(f.camera.rotation, center), -1);
        truth.add(f.camera.copy());
      }
      for (int k = 0; k < 300; k++) {
        double[] p = {
          3 + random.nextDouble() * 3, -2 + random.nextDouble() * 2 - 1, 5 + random.nextDouble() * 6
        };
        points.add(p);
        FeatureTracks.Track t = new FeatureTracks.Track();
        t.id = k;
        t.point =
            new double[] {
              p[0] + random.nextGaussian() * .03,
              p[1] + random.nextGaussian() * .03,
              p[2] + random.nextGaussian() * .10
            };
        for (int i = 0; i < n; i++) t.observations.add(new FeatureTracks.Observation(i, k));
        tracks.add(t);
      }
      for (int i = 0; i < n; i++) {
        ImageFrame f = frames.get(i);
        f.keypoints = new KeyPoint[points.size()];
        for (int k = 0; k < points.size(); k++) {
          double[] uv = truth.get(i).project(points.get(k));
          f.keypoints[k] = new KeyPoint((float) uv[0], (float) uv[1], 1);
        }
      }
      double[] origin = truth.get(anchor).center();
      double baseline = Math3.norm(Math3.sub(truth.get(scale).center(), origin));
      for (int i = 0; i < n; i++)
        if (i != anchor) {
          Camera c = frames.get(i).camera;
          c.rotation = Math3.mul(Math3.rotation(new double[] {.006, -.009, .004}), c.rotation);
          double[] center =
              Math3.add(c.center(), new double[] {.015 * Math.sin(i + 1), .02 * Math.cos(i), .012});
          if (i == scale)
            center =
                Math3.add(
                    origin,
                    Math3.times(
                        Math3.sub(center, origin),
                        baseline / Math3.norm(Math3.sub(center, origin))));
          c.translation = Math3.times(Math3.mv(c.rotation, center), -1);
        }
      Camera saved = frames.get(anchor).camera.copy();
      double[] seedRotation = frames.get(scale).camera.rotation.clone();
      double before = BundleAdjuster.error(frames, tracks, 2);
      Project project = Project.open(o, new String[0]);
      try {
        BundleAdjuster.optimize(frames, tracks, 25, 2, project, anchor, scale);
        project.finish(true, null);
      } catch (Exception e) {
        project.finish(false, e);
        throw e;
      }
      double after = BundleAdjuster.error(frames, tracks, 2);
      check(
          after < before * 1e-6,
          "Joint solve recovers independent multi-camera projections including perturbed second"
              + " seed");
      check(
          Arrays.equals(saved.rotation, frames.get(anchor).camera.rotation)
              && Arrays.equals(saved.translation, frames.get(anchor).camera.translation),
          "Nonidentity first seed pose remains bitwise fixed");
      check(
          Math.abs(Math3.norm(Math3.sub(frames.get(scale).camera.center(), origin)) - baseline)
              < 1e-10,
          "Measured baseline norm remains fixed");
      check(
          !Arrays.equals(seedRotation, frames.get(scale).camera.rotation)
              && !frames.get(scale).camera.fixed,
          "Second seed pose is refined instead of frozen");
      double maximum = 0;
      for (int i = 0; i < n; i++) {
        maximum =
            Math.max(
                maximum,
                Math3.norm(Math3.sub(frames.get(i).camera.center(), truth.get(i).center())));
        check(
            frames.get(i).camera.fx == 360
                && frames.get(i).camera.fy == 360
                && frames.get(i).camera.cx == 255.5,
            "Accepted intrinsics remain fixed");
      }
      check(maximum < .001, "All 16 camera centers recover within 0.001 scene units");
      double pointRms = 0;
      for (int k = 0; k < points.size(); k++) {
        double d = Math3.norm(Math3.sub(tracks.get(k).point, points.get(k)));
        pointRms += d * d;
      }
      check(
          Math.sqrt(pointRms / points.size()) < .001,
          "Structure recovers independently known scene coordinates");
      List<String> history =
          Files.readAllLines(o.output.resolve("04_geometry/bundle/iterations.csv"));
      for (int i = 1; i < history.size(); i++) {
        String[] row = history.get(i).split(",");
        check(
            Double.parseDouble(row[2]) <= Double.parseDouble(row[1]),
            "Every committed bundle iteration lowers the robust objective");
      }
      String solver = Files.readString(o.output.resolve("04_geometry/bundle/solver.json"));
      check(
          solver.contains("\"variable_cameras\":15")
              && Files.exists(o.output.resolve("04_geometry/bundle/linear_solver.csv")),
          "No 12-camera bootstrap cap applies to final solve; linear diagnostics exported");
      // Corrupt a minority of observations, leaving all cameras connected to abundant clean tracks.
      for (int i = 0; i < n; i++)
        for (int k = 0; k < points.size(); k++)
          if ((k * 7 + i * 11) % 37 == 0) {
            frames.get(i).keypoints[k].pt.x += 25;
            frames.get(i).keypoints[k].pt.y -= 17;
          }
      double robustBefore = BundleAdjuster.error(frames, tracks, 2);
      BundleAdjuster.optimize(frames, tracks, 8, 2, null, anchor, scale);
      check(
          BundleAdjuster.error(frames, tracks, 2) < robustBefore,
          "Huber reweighting improves objective with gross observation outliers");
      double cleanError = 0;
      int cleanCount = 0;
      for (int i = 0; i < n; i++)
        for (int k = 0; k < points.size(); k++)
          if ((k * 7 + i * 11) % 37 != 0) {
            double[] uv = frames.get(i).camera.project(tracks.get(k).point);
            Point p = frames.get(i).keypoints[k].pt;
            cleanError += Math.hypot(uv[0] - p.x, uv[1] - p.y);
            cleanCount++;
          }
      check(
          cleanError / cleanCount < .2,
          "Gross outliers do not pull the clean observations away from their measured projections");
    } finally {
      for (ImageFrame f : frames) f.close();
    }
  }
}
