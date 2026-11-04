package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;

/**
 * Replays the GPS-weighted adjustment on a finished run's exported sparse geometry
 * (04_geometry/final: interior, exterior, tracks, observations), without re-running SfM.
 *
 * <p>Usage: GpsAdjustmentReplay RUN_DIR IMAGE_DIR OUT_DIR [variant ...]; variants: g1 (poses
 * only), lens, time, rs, full (defaults). Prints GPS/reprojection RMS, the fitted parameters and
 * the median sparse height (the ground/canopy height moves with the focal length).
 */
final class GpsAdjustmentReplay {
  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Path run = Path.of(args[0]), images = Path.of(args[1]), out = Path.of(args[2]);
    List<String> variants =
        args.length > 3 ? List.of(Arrays.copyOfRange(args, 3, args.length)) : List.of("g1", "full");
    for (String variant : variants) {
      Path dir = out.resolve(variant);
      Files.createDirectories(dir);
      Options o = TestSupport.options();
      o.output = dir;
      o.iterations = Integer.getInteger("sgm.iterations", 10);
      // Without -Dsgm.iterations the production default applies: iterate to convergence.
      o.bundleIterationsExplicit = System.getProperty("sgm.iterations") != null;
      switch (variant) {
        case "g1" -> {
          o.gpsIntrinsics = "none";
          o.gpsTimeOffset = "off";
          o.gpsLeverArm = "off";
          o.rollingShutter = "off";
        }
        case "lens" -> {
          o.gpsTimeOffset = "off";
          o.gpsLeverArm = "off";
          o.rollingShutter = "off";
        }
        case "time" -> {
          o.gpsIntrinsics = "none";
          o.rollingShutter = "off";
        }
        case "rs" -> {
          o.gpsIntrinsics = "none";
          o.gpsTimeOffset = "off";
          o.gpsLeverArm = "off";
        }
        case "full", "converge" -> {}
        default -> throw new IllegalArgumentException("Unknown variant " + variant);
      }
      o.gpsIntrinsics = System.getProperty("sgm.intrinsics", o.gpsIntrinsics);
      o.gpsFocalPrior = System.getProperty("sgm.focalPrior", o.gpsFocalPrior);
      o.gpsFocalSigma = Double.parseDouble(System.getProperty("sgm.focalSigma", "" + o.gpsFocalSigma));
      o.gpsAdjustmentRounds = Integer.getInteger("sgm.rounds", o.gpsAdjustmentRounds);
      o.gpsHeightSigma = System.getProperty("sgm.heightSigma", o.gpsHeightSigma);
      Fixture f = load(run.resolve("04_geometry/final"), images, o);
      if (variant.startsWith("converge")) {
        // Convergence profile of the plain image-only bundle (as after SfM).
        int anchor = 0, second = 1;
        long started = System.nanoTime();
        BundleAdjuster.optimizeImpl(
            f.frames, f.tracks, o.iterations, o.reprojection, null, anchor, second, null);
        System.out.printf(
            Locale.ROOT, "REPLAY %s: %.1f s%n", variant, (System.nanoTime() - started) / 1e9);
        for (ImageFrame frame : f.frames) frame.close();
        continue;
      }
      Crs crs = new Crs(o.crs);
      Map<Integer, double[]> controls = new LinkedHashMap<>();
      double square = 0;
      for (ImageFrame frame : f.frames)
        if (frame.camera.registered && frame.gps != null && frame.gps.usable()) {
          double[] target = crs.project(frame.gps);
          double e = Math3.norm(Math3.sub(frame.camera.center(), target));
          if (e <= o.gpsThreshold) {
            controls.put(frame.id, target);
            square += e * e;
          }
        }
      double rms = Math.sqrt(square / controls.size());
      double heightBefore = medianHeight(f.tracks);
      long started = System.nanoTime();
      GpsAdjustment.Outcome result =
          GpsAdjustment.run(f.frames, f.tracks, o, crs, controls, dir, rms);
      System.out.printf(
          Locale.ROOT,
          "REPLAY %s: %.1f s; median sparse height %.2f -> %.2f m; %s%n",
          variant,
          (System.nanoTime() - started) / 1e9,
          heightBefore,
          medianHeight(f.tracks),
          result.summary());
      System.out.println("REPLAY " + variant + " block vs GPS " + affine(f.frames, controls));
      for (ImageFrame frame : f.frames) frame.close();
    }
  }

  /** Horizontal affine fit camera centre -> GPS: north and east scale, shear (block stretch). */
  static String affine(List<ImageFrame> frames, Map<Integer, double[]> controls) {
    double[][] a = new double[3][3];
    double[] bx = new double[3], by = new double[3];
    double[] mean = new double[2];
    for (var e : controls.entrySet()) {
      double[] c = frames.get(e.getKey()).camera.center();
      mean[0] += c[0] / controls.size();
      mean[1] += c[1] / controls.size();
    }
    for (var e : controls.entrySet()) {
      double[] c = frames.get(e.getKey()).camera.center(), g = e.getValue();
      double[] row = {c[0] - mean[0], c[1] - mean[1], 1};
      for (int i = 0; i < 3; i++) {
        bx[i] += row[i] * (g[0] - mean[0]);
        by[i] += row[i] * (g[1] - mean[1]);
        for (int j = 0; j < 3; j++) a[i][j] += row[i] * row[j];
      }
    }
    double[] x = Math3.solve(a, bx), y = Math3.solve(a, by);
    List<double[]> source = new ArrayList<>(), target = new ArrayList<>();
    for (var e : controls.entrySet()) {
      source.add(frames.get(e.getKey()).camera.center());
      target.add(e.getValue());
    }
    Georeference.Similarity s = Georeference.fit(source, target);
    double square = 0;
    for (int i = 0; i < source.size(); i++)
      square += Math.pow(Math3.norm(Math3.sub(s.apply(source.get(i)), target.get(i))), 2);
    return String.format(
        Locale.ROOT,
        "affine E<-[%.5f %.5f] N<-[%.5f %.5f] (1 = no stretch); similarity scale %.5f, RMS %.3f m",
        x[0],
        x[1],
        y[0],
        y[1],
        s.scale,
        Math.sqrt(square / source.size()));
  }

  static double medianHeight(List<FeatureTracks.Track> tracks) {
    List<Double> z = new ArrayList<>();
    for (var t : tracks) if (t.point != null) z.add(t.point[2]);
    Collections.sort(z);
    return z.isEmpty() ? Double.NaN : z.get(z.size() / 2);
  }

  record Fixture(List<ImageFrame> frames, List<FeatureTracks.Track> tracks) {}

  static Fixture load(Path dir, Path images, Options o) throws IOException {
    List<String[]> interior = rows(dir.resolve("interior.csv"));
    List<String[]> exterior = rows(dir.resolve("exterior.csv"));
    List<ImageFrame> frames = new ArrayList<>();
    for (int i = 0; i < exterior.size(); i++) {
      String[] in = interior.get(i), ex = exterior.get(i);
      Path path = images.resolve(ex[1].replace("\"", ""));
      int w = Integer.parseInt(in[2]), h = Integer.parseInt(in[3]);
      int[] source = ResolutionPlan.originalSize(path);
      double sx = (double) w / source[0], sy = (double) h / source[1];
      double[] d = {
        Double.parseDouble(in[9]),
        Double.parseDouble(in[10]),
        Double.parseDouble(in[11]),
        Double.parseDouble(in[12]),
        Double.parseDouble(in[13])
      };
      Calibration c =
          new Calibration(
              Double.parseDouble(in[4]) / sx,
              Double.parseDouble(in[5]) / sy,
              (Double.parseDouble(in[6]) + .5) / sx - .5,
              (Double.parseDouble(in[7]) + .5) / sy - .5,
              d,
              source[0],
              source[1]);
      if (!frames.isEmpty() && SharedCalibration.same(frames.get(0).calibration, c) == false) {
        Calibration first = frames.get(0).calibration;
        if (Math.abs(first.fx - c.fx) < 1e-6 * first.fx) c = first;
      }
      o.maxDimension = Math.max(w, h);
      o.maxDimensionExplicit = true;
      o.quality = "custom";
      ImageFrame frame = new ImageFrame(i, path, c, o, true);
      frame.camera.registered = Boolean.parseBoolean(ex[2]);
      double[] r = new double[9], t = new double[3];
      for (int k = 0; k < 9; k++) r[k] = Double.parseDouble(ex[4 + k]);
      for (int k = 0; k < 3; k++) t[k] = Double.parseDouble(ex[13 + k]);
      frame.camera.rotation = r;
      frame.camera.translation = t;
      frames.add(frame);
    }
    Map<Integer, FeatureTracks.Track> byId = new LinkedHashMap<>();
    for (String[] row : rows(dir.resolve("tracks.csv"))) {
      FeatureTracks.Track track = new FeatureTracks.Track();
      track.id = Integer.parseInt(row[0]);
      if (Boolean.parseBoolean(row[1]))
        track.point =
            new double[] {
              Double.parseDouble(row[2]), Double.parseDouble(row[3]), Double.parseDouble(row[4])
            };
      byId.put(track.id, track);
    }
    Map<Integer, Map<Integer, Point>> keys = new HashMap<>();
    for (String[] row : rows(dir.resolve("observations.csv"))) {
      int track = Integer.parseInt(row[0]), image = Integer.parseInt(row[1]);
      int feature = Integer.parseInt(row[2]);
      byId.get(track).observations.add(new FeatureTracks.Observation(image, feature));
      keys.computeIfAbsent(image, k -> new HashMap<>())
          .put(feature, new Point(Double.parseDouble(row[3]), Double.parseDouble(row[4])));
    }
    for (ImageFrame frame : frames) {
      Map<Integer, Point> k = keys.getOrDefault(frame.id, Map.of());
      int n = k.keySet().stream().mapToInt(Integer::intValue).max().orElse(-1) + 1;
      frame.keypoints = new KeyPoint[n];
      for (int i = 0; i < n; i++) {
        Point p = k.getOrDefault(i, new Point(0, 0));
        frame.keypoints[i] = new KeyPoint((float) p.x, (float) p.y, 1);
      }
    }
    return new Fixture(frames, new ArrayList<>(byId.values()));
  }

  private static List<String[]> rows(Path file) throws IOException {
    List<String[]> out = new ArrayList<>();
    try (BufferedReader r = Files.newBufferedReader(file)) {
      r.readLine();
      for (String line; (line = r.readLine()) != null; ) out.add(line.split(",", -1));
    }
    return out;
  }
}
