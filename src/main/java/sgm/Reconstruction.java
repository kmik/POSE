package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;

final class Reconstruction implements AutoCloseable {
  private final Options options;
  private final List<ImageFrame> frames = new ArrayList<>();
  private final Project project;

  Reconstruction(Options options) {
    this(options, null);
  }

  Reconstruction(Options options, Project project) {
    this.options = options;
    this.project = project;
  }

  void run() throws IOException {
    long started = System.nanoTime();
    if (project != null)
      project.stage("00_inputs", "running", "Read photographs and preserve input provenance");
    List<Path> paths = Main.images(options);
    OrientationIO.prepare(options, paths);
    if (paths.size() < 2) throw new IllegalArgumentException("At least 2 input images required");
    if (options.rectified && paths.size() != 2)
      throw new IllegalArgumentException(
          "--rectified requires exactly 2 images, sorted left then right");
    ResolutionPlan.write(paths, options, project);
    if (options.estimateMemory) {
      if (project != null) {
        project.stage("00_inputs", "complete", "Header-only resolution and memory estimate");
        for (String stage :
            new String[] {
              "01_calibration",
              "02_features",
              "03_matches",
              "04_geometry",
              "05_stereo",
              "06_fusion",
              "07_results"
            }) project.stage(stage, "skipped", "--estimate-memory: planning only");
      }
      return;
    }
    Mat first =
        Imgcodecs.imread(
            paths.get(0).toString(),
            Imgcodecs.IMREAD_GRAYSCALE | Imgcodecs.IMREAD_IGNORE_ORIENTATION);
    Calibration calibration;
    try {
      if (first.empty()) throw new IOException("Cannot decode " + paths.get(0));
      calibration = ReuseCache.calibration(paths, options, null);
      if (calibration == null) {
        calibration =
            options.calibration == null && Double.isNaN(options.focal)
                ? AutomaticCalibration.estimate(options, paths, first.cols(), first.rows(), project)
                : Calibration.load(options, first.cols(), first.rows());
        if (options.calibration == null && !Double.isNaN(options.focal)) {
          calibration = SelfCalibration.refine(calibration, paths, options, project);
          calibration.write(
              options.output.resolve("camera.txt"),
              "Focal/sensor initialization followed by guarded radial refinement");
        }
        if (options.calibration != null && !options.intrinsicsMode.equals("fixed"))
          calibration = SelfCalibration.refine(calibration, paths, options, project);
        ReuseCache.calibration(paths, options, calibration);
      }
      calibration.write(
          options.output.resolve("camera.txt"),
          "Applied calibration; mode=" + options.intrinsicsMode + "; source resolution values");
      Path constraints = options.output.resolve("01_calibration/constraints.json");
      Files.createDirectories(constraints.getParent());
      Json.write(
          constraints,
          Json.object(
              "mode",
              options.calibration == null ? "automatic_refine" : options.intrinsicsMode,
              "refined_parameters",
              options.refineIntrinsics,
              "focal_relative_sigma",
              options.focalSigma,
              "radial_coefficient_sigma",
              options.radialSigma,
              "supplied_file",
              options.calibration == null ? null : options.calibration.toString(),
              "applied_focal",
              calibration.fx,
              "applied_distortion",
              calibration.distortion,
              "note",
              "Fixed parameters remain exact; prior uncertainties are assumptions, not measured"
                  + " accuracy. Rejected guarded fits retain supplied calibration."));
    } finally {
      first.release();
    }
    Options alignment = options.alignmentOptions();
    for (Path path : paths) frames.add(new ImageFrame(frames.size(), path, calibration, alignment));
    writeSettings(calibration);
    if (project != null) {
      project.data.inputs(frames, calibration);
      project.stage(
          "00_inputs", "complete", "Image inventory, processed rasters and maps exported");
      project.stage(
          "01_calibration", "complete", "Supplied or derived interior orientation exported");
    }
    FeatureTracks features = new FeatureTracks();
    Core.setRNGSeed(1);
    String coordinates;
    if (options.rectified) {
      skipFeatures("Already-rectified stereo bypasses SIFT and SfM");
      if (project != null)
        project.stage("04_geometry", "running", "Known rectified stereo exterior orientation");
      Camera a = frames.get(0).camera, b = frames.get(1).camera;
      if (Math.abs(a.fx - b.fx)
              + Math.abs(a.fy - b.fy)
              + Math.abs(a.cx - b.cx)
              + Math.abs(a.cy - b.cy)
          > 1e-6) throw new IllegalArgumentException("Rectified cameras must share intrinsics");
      a.registered = b.registered = true;
      a.fixed = b.fixed = true;
      b.translation = new double[] {-options.baseline, 0, 0};
      features.pairs.add(new FeatureTracks.Pair(0, 1));
      coordinates =
          "Rectified left-camera frame; baseline=" + options.baseline + " in supplied output units";
    } else if (options.poses != null) {
      if (options.extrinsicsMode.equals("fixed")) skipFeatures("Supplied camera poses fixed");
      if (project != null)
        project.stage("04_geometry", "running", "Import supplied exterior orientations");
      PoseIO.read(options.poses, frames);
      List<Camera> suppliedCameras = frames.stream().map(f -> f.camera.copy()).toList();
      if (!options.extrinsicsMode.equals("fixed")) {
        for (ImageFrame f : frames) {
          if (options.extrinsicsMode.equals("prior")) {
            f.camera.reference = f.camera.copy();
            f.camera.positionSigma = options.positionSigma;
            f.camera.rotationSigma = Math.toRadians(options.rotationSigma);
          }
          f.camera.fixed = false;
        }
        features.build(frames, options, project);
        String suppliedGeometryKey = ReuseCache.geometryKey(frames, features, options);
        if (!ReuseCache.geometry(frames, features, options, suppliedGeometryKey, false)) {
          Geometry.triangulateTracks(frames, features.tracks, options.reprojection * 2);
          if (features.tracks.stream().filter(t -> t.point != null).count() < 8)
            throw new IllegalArgumentException(
                "Supplied pose refinement needs at least eight triangulated matching tracks; check"
                    + " calibration, poses and overlap");
          BundleAdjuster.optimize(
              frames, features.tracks, options.iterations, options.reprojection, project);
          int scaleCamera = 1;
          while (scaleCamera < frames.size() - 1
              && Math3.norm(
                      Math3.sub(
                          frames.get(scaleCamera).camera.center(), frames.get(0).camera.center()))
                  < 1e-9) scaleCamera++;
          SparseCleanup.run(frames, features.tracks, options, project, 0, scaleCamera);
          ReuseCache.geometry(frames, features, options, suppliedGeometryKey, true);
        }
      }
      if (features.pairs.isEmpty())
        for (int[] edge : PairSelection.select(frames, options, project))
          features.pairs.add(new FeatureTracks.Pair(edge[0], edge[1]));
      coordinates = "World coordinate system and units supplied in --poses";
      List<Object> changes = new ArrayList<>();
      for (ImageFrame f : frames)
        changes.add(
            Json.object(
                "image",
                f.path.getFileName().toString(),
                "mode",
                options.extrinsicsMode,
                "position_sigma_pose_units",
                options.positionSigma,
                "rotation_sigma_degrees",
                options.rotationSigma,
                "adjusted_center",
                f.camera.center(),
                "adjusted_R",
                f.camera.rotation,
                "initial_center",
                suppliedCameras.get(f.id).center(),
                "initial_R",
                suppliedCameras.get(f.id).rotation,
                "reference_center",
                f.camera.reference == null ? null : f.camera.reference.center(),
                "prior_cost",
                BundleAdjuster.priorCost(f.camera)));
      Json.write(options.output.resolve("04_geometry/pose_constraints.json"), changes);
    } else {
      features.build(frames, options, project);
      if (project != null)
        project.stage("04_geometry", "running", "Camera registration and sparse reconstruction");
      String geometryKey = ReuseCache.geometryKey(frames, features, options);
      if (!ReuseCache.geometry(frames, features, options, geometryKey, false)) {
        Geometry.reconstruct(frames, features, options, project);
        ReuseCache.geometry(frames, features, options, geometryKey, true);
      }
      coordinates =
          "Seed-camera frame; seed="
              + frames.get(features.seed.a).path.getFileName()
              + " / "
              + frames.get(features.seed.b).path.getFileName()
              + "; seed baseline="
              + options.baseline
              + " (arbitrary units unless this is a measured baseline)";
    }
    Georeference.Result georeference = Georeference.align(frames, features.tracks, options);
    coordinates += "; " + georeference.description;
    if (georeference.crs != null) coordinates = georeference.description;
    if (project != null) project.data.snapshot("final", frames, features.tracks, coordinates);
    PoseIO.write(options.output.resolve("cameras.csv"), frames);
    writeSparse(features.tracks);
    String geometryQuality = GeometryQuality.write(options.output, frames, features.tracks);
    if (project != null) {
      project.stage("04_geometry", "complete", coordinates);
      project.stage("05_stereo", "running", "Rectification and Java SGM");
    }
    boolean reload = false;
    for (ImageFrame frame : frames) {
      int[] size = options.processedSize(frame.sourceWidth, frame.sourceHeight);
      if (size[0] != frame.gray.cols() || size[1] != frame.gray.rows()) reload = true;
    }
    if (reload) {
      List<ImageFrame> denseFrames = new ArrayList<>();
      try {
        for (ImageFrame frame : frames) {
          ImageFrame denseFrame = new ImageFrame(frame.id, frame.path, calibration, options);
          denseFrames.add(denseFrame);
          denseFrame.camera.rotation = frame.camera.rotation.clone();
          denseFrame.camera.translation = frame.camera.translation.clone();
          denseFrame.camera.registered = frame.camera.registered;
          denseFrame.camera.fixed = frame.camera.fixed;
          denseFrame.inheritKeypoints(frame);
          frame.close();
        }
      } catch (IOException | RuntimeException e) {
        for (ImageFrame f : denseFrames) f.close();
        throw e;
      }
      frames.clear();
      frames.addAll(denseFrames);
      if (project != null) project.data.denseInputs(frames);
    }
    OrientationIO.archive(frames, options, georeference.crs == null ? null : georeference.crs.name);
    if (OverlapSelection.enabled(options)) {
      List<int[]> edges = new ArrayList<>();
      for (var p : features.pairs) edges.add(new int[] {p.a, p.b});
      List<int[]> accepted =
          OverlapSelection.filter(frames, edges, features.tracks, options, "dense");
      Set<Long> keys = new HashSet<>();
      for (int[] e : accepted) keys.add(((long) e[0] << 32) | (e[1] & 0xffffffffL));
      features.pairs.removeIf(p -> !keys.contains(((long) p.a << 32) | (p.b & 0xffffffffL)));
      if (features.pairs.isEmpty())
        throw new IllegalArgumentException(
            "No stereo pairs satisfy the enabled overlap thresholds; inspect overlap_dense.json");
    }
    DenseReconstruction dense =
        new DenseReconstruction(options, frames, project, georeference.crs, features.tracks);
    long points = dense.run(features.pairs);
    double seconds = (System.nanoTime() - started) / 1e9;
    long sparse = features.tracks.stream().filter(t -> t.point != null).count();
    String report =
        "mSGM Java 0.1\nImages="
            + frames.size()
            + "\n"
            + options.filteringSummary()
            + "\nQuality="
            + options.quality
            + "; downsample factor="
            + options.qualityDownsample()
            + "; resolution and memory estimates: 00_inputs/resolution_plan.json"
            + "\nPairs="
            + features.pairs.size()
            + "\nSparse tracks="
            + sparse
            + "\nDense points="
            + points
            + "\nCoordinates="
            + coordinates
            + "\nOutput coordinate multiplier="
            + options.scale
            + "\n"
            + "Camera CSV and sparse.csv use pre-multiplier coordinates.\n"
            + (georeference.crs == null
                ? "No EPSG CRS is assigned to LAS.\n"
                : "LAS GeoTIFF horizontal CRS="
                    + georeference.crs.name
                    + "; see coordinates.json.\n")
            + (options.calibration == null
                ? "Automatic calibration: see 01_calibration/refinement/result.json and"
                    + " reference_camera.properties. "
                    + AutomaticCalibration.ASSUMPTIONS
                    + "\n"
                : "")
            + "OpenCV="
            + Core.VERSION
            + "\nSGM=Java "
            + options.censusMode
            + " census/Hamming, "
            + options.directions
            + " paths, search-boundary rejection, post-median LR, ZNCC, speckle filtering\n"
            + "Minimum triangulation angle degrees="
            + options.minAngle
            + "\n"
            + geometryQuality
            + "\n"
            + dense.summary()
            + (options.rectified || options.poses != null && options.extrinsicsMode.equals("fixed")
                ? "Final bundle adjustment=skipped; supplied exterior orientations preserved\n"
                : "Final undistorted-image bundle adjustment=joint robust Schur"
                    + " Levenberg-Marquardt, fixed accepted intrinsics, first seed pose and seed"
                    + " baseline length\n")
            + "Legacy C++ numerical parity has not been established.\n"
            + "Elapsed seconds="
            + seconds
            + "\n";
    Files.writeString(options.output.resolve("report.txt"), report);
    System.out.printf(
        Locale.ROOT,
        "Wrote %,d RGB points to %s in %.2f s%n",
        points,
        options.output.toAbsolutePath(),
        seconds);
    List<String> missing = new ArrayList<>();
    for (ImageFrame f : frames)
      if (!f.camera.registered) missing.add(f.path.getFileName().toString());
    String registration =
        "Registered cameras="
            + (frames.size() - missing.size())
            + "/"
            + frames.size()
            + "; failed images="
            + missing;
    System.out.println(registration);
    Files.writeString(
        options.output.resolve("report.txt"), registration + "\n", StandardOpenOption.APPEND);
  }

  private void skipFeatures(String reason) throws IOException {
    if (project != null) {
      project.stage("02_features", "skipped", reason);
      project.stage("03_matches", "skipped", reason);
    }
  }

  private void writeSparse(List<FeatureTracks.Track> tracks) throws IOException {
    try (BufferedWriter w = Files.newBufferedWriter(options.output.resolve("sparse.csv"))) {
      w.write("x,y,z,observations\n");
      for (FeatureTracks.Track track : tracks)
        if (track.point != null)
          w.write(
              track.point[0]
                  + ","
                  + track.point[1]
                  + ","
                  + track.point[2]
                  + ","
                  + track.observations.size()
                  + "\n");
    }
  }

  private void writeSettings(Calibration calibration) throws IOException {
    Properties settings = new Properties();
    try {
      for (java.lang.reflect.Field field : Options.class.getFields()) {
        Object value = field.get(options);
        if (value != null)
          settings.setProperty(
              field.getName(),
              value instanceof Path
                  ? ((Path) value).toAbsolutePath().normalize().toString()
                  : String.valueOf(value));
      }
    } catch (IllegalAccessException e) {
      throw new IllegalStateException(e);
    }
    settings.setProperty("java.version", System.getProperty("java.version"));
    settings.setProperty("os.name", System.getProperty("os.name"));
    settings.setProperty("calibration.reference.width", String.valueOf(calibration.width));
    settings.setProperty("calibration.reference.height", String.valueOf(calibration.height));
    for (ImageFrame frame : frames) {
      String key = "image." + frame.id + ".";
      settings.setProperty(key + "name", frame.path.getFileName().toString());
      settings.setProperty(key + "width", String.valueOf(frame.gray.cols()));
      settings.setProperty(key + "height", String.valueOf(frame.gray.rows()));
      settings.setProperty(key + "fx", String.valueOf(frame.camera.fx));
      settings.setProperty(key + "fy", String.valueOf(frame.camera.fy));
    }
    try (Writer writer =
        Files.newBufferedWriter(options.output.resolve("run-settings.properties"))) {
      settings.store(writer, "Effective reconstruction settings; not a camera calibration file");
    }
  }

  public void close() {
    for (ImageFrame frame : frames) frame.close();
  }
}
