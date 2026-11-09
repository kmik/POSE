package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;

/**
 * The reconstruction as separate steps. Every step loads the saved project state of its run
 * (RUN/state, see {@link PipelineState}), does its part and saves the state again, so the steps
 * can run one by one in separate processes or chained in one process with the same result.
 *
 * <pre>
 * inputs       00  image inventory, orientation imports, resolution plan
 * calibrate    01  interior orientation (supplied, EXIF or automatic), alignment frames
 * match        02-03 features, pair matching, tracks (supplied poses are read here)
 * align        04  relative orientation: SfM, bundle adjustment, sparse cleanup, lens refinement
 * georeference 04  GPS similarity, GPS-weighted adjustment, camera screening, final exports
 * gcp          04  ground control points: predicted markers; after pinning, the GCP adjustment
 * dense        05-06 dense matching and fusion, dense.las / dense.ply, report.txt
 * ground       07  optional ground classification, pair correction, ground-aware fusion
 * finish       07  file organisation, processing report, artifact catalogue
 * </pre>
 *
 * The gcp step ends "waiting" until markers are pinned (photogrammetry gcp-edit); run it again.
 */
final class Pipeline {
  enum Step {
    INPUTS("inputs", "Read photographs, orientation imports and resolution plan"),
    CALIBRATE("calibrate", "Interior orientation and alignment images"),
    MATCH("match", "Features, pair matching and tracks"),
    ALIGN("align", "Camera alignment and sparse reconstruction"),
    GEOREFERENCE("georeference", "GPS georeferencing, GPS-weighted adjustment and camera screening"),
    GCP("gcp", "Ground control points: markers, adjustment and residuals (optional)"),
    DENSE("dense", "Dense matching and fusion"),
    GROUND("ground", "Ground classification (optional)"),
    FINISH("finish", "Processing report and final project files");

    final String name, description;

    Step(String name, String description) {
      this.name = name;
      this.description = description;
    }

    static Step of(String name) {
      for (Step s : values()) if (s.name.equalsIgnoreCase(name)) return s;
      return null;
    }
  }

  final Options options;
  final Project project;
  final PipelineState state;
  private List<Path> paths;
  private Calibration calibration;
  private final List<ImageFrame> frames = new ArrayList<>();
  private FeatureTracks features;

  Pipeline(Options options, Project project, PipelineState state) {
    this.options = options;
    this.project = project;
    this.state = state;
  }

  /** Runs one step: checks earlier steps, loads state, works, saves. */
  void run(Step step) throws IOException {
    String current = state.status(step);
    if (current.equals("complete") || current.equals("skipped"))
      throw new IllegalArgumentException(
          "Step '" + step.name + "' is already " + current + " in this run; use --branch to redo it in a new run");
    for (Step prior : Step.values()) {
      if (prior == step) break;
      String status = state.status(prior);
      if (status.equals("pending") && notRequested(prior, options)) {
        // An optional step that was not asked for (no --gcp, no ground option) is simply skipped.
        state.mark(prior, "skipped", prior.description);
        state.snapshot(prior);
        continue;
      }
      if (!status.equals("complete") && !status.equals("skipped"))
        throw new IllegalArgumentException(
            "Step '" + step.name + "' needs '" + prior.name + "' first (status: " + status + ")");
    }
    state.mark(step, "running", step.description);
    System.out.println("Pipeline step: " + step.name + " - " + step.description);
    // Every step starts from the same random state, whether chained or run separately.
    Core.setRNGSeed(1);
    String outcome;
    try {
      outcome =
          switch (step) {
            case INPUTS -> inputs();
            case CALIBRATE -> calibrate();
            case MATCH -> match();
            case ALIGN -> align();
            case GEOREFERENCE -> georeference();
            case GCP -> gcp();
            case DENSE -> dense();
            case GROUND -> ground();
            case FINISH -> "complete";
          };
    } catch (IOException | RuntimeException | Error e) {
      state.mark(step, "failed", e.toString());
      throw e;
    } finally {
      try { for (ImageFrame f : frames) f.close(); }
      finally {
        frames.clear();
        features = null; // do not retain the previous graph while the next step restores it
        Geometry.releaseWorkspaces();
      }
    }
    state.mark(step, outcome, outcome.equals("waiting") ? state.string("gcp_waiting") : step.description);
    if (!outcome.equals("waiting")) state.snapshot(step);
  }

  /**
   * Checks a step list against a run's state before anything is opened, so a refused command
   * leaves the run untouched: steps in pipeline order, none already done, earlier ones done.
   */
  static void check(PipelineState state, List<Step> steps, Options options) {
    for (int i = 1; i < steps.size(); i++)
      if (steps.get(i).ordinal() <= steps.get(i - 1).ordinal())
        throw new IllegalArgumentException(
            "Give the steps in pipeline order: inputs calibrate match align georeference gcp dense ground finish");
    Set<Step> planned = EnumSet.copyOf(steps);
    for (Step step : steps) {
      String current = state.status(step);
      if (current.equals("complete") || current.equals("skipped"))
        throw new IllegalArgumentException(
            "Step '" + step.name + "' is already " + current + " in this run; use --branch to redo it in a new run");
      for (Step prior : Step.values()) {
        if (prior == step) break;
        String status = state.status(prior);
        if (!planned.contains(prior)
            && !status.equals("complete")
            && !status.equals("skipped")
            && !(status.equals("pending") && notRequested(prior, options)))
          throw new IllegalArgumentException(
              "Step '" + step.name + "' needs '" + prior.name + "' first (status: " + status + ")");
      }
    }
  }

  /**
   * Optional steps run only when asked for: gcp with --gcp, ground with a ground option. Pending
   * and not asked for, they are skipped when a later step runs.
   */
  static boolean notRequested(Step step, Options o) {
    if (o == null) return false;
    return switch (step) {
      case GCP -> o.gcp == null;
      case GROUND -> !(o.classifyGround || o.groundPairCorrection || o.groundAwareFusion);
      default -> false;
    };
  }

  /** First step that is neither complete nor skipped (nor an optional step not asked for), or null. */
  static String next(PipelineState state, Options options) {
    for (Step s : Step.values()) {
      String status = state.status(s);
      if (status.equals("pending") && notRequested(s, options)) continue;
      if (!status.equals("complete") && !status.equals("skipped")) return s.name;
    }
    return null;
  }

  /** Run folders written by each step (copied when a later step branches). */
  private static List<String> folders(Step step) {
    return switch (step) {
      case INPUTS -> List.of("00_inputs");
      case CALIBRATE -> List.of("00_inputs", "01_calibration");
      case MATCH -> List.of("02_features", "03_matches");
      case ALIGN, GEOREFERENCE, GCP -> List.of("04_geometry");
      case DENSE -> List.of("05_stereo", "06_fusion", "07_results");
      case GROUND -> List.of("07_results");
      case FINISH -> List.of();
    };
  }

  /**
   * --branch: a new run continuing from an existing run's state before {@code from}. The earlier
   * steps' folders and run-level files are carried over (hard links where possible; dense.las and
   * dense.ply are copied because the ground step edits them in place), their stage history is
   * imported, and {@code from} and later steps start pending. The source run is not changed.
   */
  static PipelineState branch(Project project, Path source, Step from, String[] arguments) throws IOException {
    if (from == Step.INPUTS)
      throw new IllegalArgumentException("Branching at 'inputs' is a new run: run the inputs step without --branch");
    PipelineState original = PipelineState.load(source);
    for (Step prior : Step.values()) {
      if (prior == from) break;
      String status = original.status(prior);
      if (!status.equals("complete") && !status.equals("skipped"))
        throw new IllegalArgumentException(
            "Cannot branch at '" + from.name + "': step '" + prior.name + "' of the source run is " + status);
    }
    Step before = Step.values()[from.ordinal() - 1];
    Path snapshot = source.resolve("state/after-" + before.name);
    if (!Files.isDirectory(snapshot))
      throw new IllegalArgumentException(
          "The source run has no saved state after '" + before.name + "'; branch from a newer run");
    Set<String> carried = new LinkedHashSet<>();
    for (Step prior : Step.values()) {
      if (prior == from) break;
      carried.addAll(folders(prior));
    }
    // Later steps rewrite files in 00-04 and 07 in place, so those are copied; stereo and fusion
    // outputs are never written again and may share storage with the source run.
    for (String folder : carried)
      transfer(source.resolve(folder), project.run.resolve(folder), folder.equals("05_stereo") || folder.equals("06_fusion"));
    boolean denseDone = from.ordinal() > Step.DENSE.ordinal();
    Set<String> denseFiles = Set.of("dense.las", "dense.ply", "report.txt", "stereo-quality.csv", "fusion-quality.txt");
    Set<String> runFiles = Set.of("README.md", "run.json", "artifacts.csv", "stages.csv");
    try (var files = Files.list(source)) {
      for (Path file : (Iterable<Path>) files.filter(Files::isRegularFile)::iterator) {
        String name = file.getFileName().toString();
        if (runFiles.contains(name) || !denseDone && denseFiles.contains(name)) continue;
        Files.copy(file, project.run.resolve(name), StandardCopyOption.REPLACE_EXISTING);
      }
    }
    // State, camera.txt and constraints.json exactly as they were after the step before 'from',
    // plus the earlier snapshots so the new run can be branched again.
    Path stateDir = project.run.resolve("state");
    Files.createDirectories(stateDir);
    for (String name :
        new String[] {"state.json", "frames.bin", "tracks.bin", "calibration.bin", "calibration_poses.bin"}) {
      Path file = snapshot.resolve(name);
      if (Files.isRegularFile(file)) Files.copy(file, stateDir.resolve(name), StandardCopyOption.REPLACE_EXISTING);
    }
    if (Files.isRegularFile(snapshot.resolve("camera.txt")))
      Files.copy(snapshot.resolve("camera.txt"), project.run.resolve("camera.txt"), StandardCopyOption.REPLACE_EXISTING);
    if (Files.isRegularFile(snapshot.resolve("constraints.json"))) {
      Files.createDirectories(project.run.resolve("01_calibration"));
      Files.copy(
          snapshot.resolve("constraints.json"),
          project.run.resolve("01_calibration/constraints.json"),
          StandardCopyOption.REPLACE_EXISTING);
    }
    for (Step prior : Step.values()) {
      if (prior == from) break;
      transfer(source.resolve("state/after-" + prior.name), stateDir.resolve("after-" + prior.name), false);
    }
    PipelineState state = PipelineState.load(project.run);
    state.put("arguments", Arrays.asList(arguments));
    state.put("branched_from", source.getFileName().toString());
    state.put("started_epoch_millis", System.currentTimeMillis());
    for (Step s : Step.values()) if (s.ordinal() >= from.ordinal()) state.mark(s, "pending", "Branched; not run yet");
    state.write();
    Map<String, Object> history = JsonRead.object(JsonRead.object(JsonRead.read(source.resolve("run.json"))).get("stages"));
    project.importStages(history, carried);
    project.event("branched", Json.object("source_run", source.getFileName().toString(), "from_step", from.name));
    System.out.println("Branched from run " + source.getFileName() + " at step " + from.name);
    return state;
  }

  /** Copies a folder tree; with {@code links}, hard-links files where the volume allows. */
  private static void transfer(Path from, Path to, boolean links) throws IOException {
    if (!Files.isDirectory(from)) return;
    try (var tree = Files.walk(from)) {
      for (Path p : (Iterable<Path>) tree::iterator) {
        Path target = to.resolve(from.relativize(p).toString());
        if (Files.isDirectory(p)) {
          Files.createDirectories(target);
          continue;
        }
        Files.deleteIfExists(target);
        if (links)
          try {
            Files.createLink(target, p);
            continue;
          } catch (IOException | UnsupportedOperationException e) {
            // Different volume or no hard links: copy instead.
          }
        Files.copy(p, target);
      }
    }
  }

  private boolean planningOnly() {
    return Boolean.TRUE.equals(state.get("estimate_memory_only"));
  }

  // ---- Shared loading.

  private void images() throws IOException {
    paths = Main.images(options);
    OrientationIO.prepare(options, paths);
    Object[] saved = state.readCalibrations("calibration.bin");
    calibration = (Calibration) saved[0];
    @SuppressWarnings("unchecked")
    Map<String, Calibration> perImage = (Map<String, Calibration>) saved[1];
    if (!perImage.isEmpty()) options.orientationCalibrations = perImage;
    Map<String, Camera> poses = state.readPoses("calibration_poses.bin");
    if (!poses.isEmpty()) options.calibrationPoses = poses;
  }

  private Options alignment() {
    Options alignment = options.alignmentOptions();
    alignment.alignmentCalibration = true;
    return alignment;
  }

  private void loadFrames() throws IOException {
    frames.addAll(state.readFrames(alignment()));
    if (project != null) project.data.restore(frames, calibration);
  }

  private void saveFrames() throws IOException {
    state.writeFrames(frames);
    state.writeCalibrations("calibration.bin", calibration, options.orientationCalibrations);
    state.writePoses("calibration_poses.bin", options.calibrationPoses);
  }

  // ---- Steps.

  private String inputs() throws IOException {
    if (project != null)
      project.stage("00_inputs", "running", "Read photographs and preserve input provenance");
    paths = Main.images(options);
    OrientationIO.prepare(options, paths);
    if (paths.size() < 2) throw new IllegalArgumentException("At least 2 input images required");
    if (options.rectified && paths.size() != 2)
      throw new IllegalArgumentException(
          "--rectified requires exactly 2 images, sorted left then right");
    ResolutionPlan.write(paths, options, project);
    List<String> names = new ArrayList<>();
    for (Path p : paths) names.add(p.getFileName().toString());
    state.put("images", names);
    if (options.estimateMemory) {
      if (project != null) {
        project.stage("00_inputs", "complete", "Header-only resolution and memory estimate");
        for (String stage :
            new String[] {
              "01_calibration", "02_features", "03_matches", "04_geometry", "05_stereo", "06_fusion", "07_results"
            }) project.stage(stage, "skipped", "--estimate-memory: planning only");
      }
      state.put("estimate_memory_only", true);
    }
    state.write();
    return "complete";
  }

  private String calibrate() throws IOException {
    if (planningOnly()) return "skipped";
    paths = Main.images(options);
    OrientationIO.prepare(options, paths);
    int[] first = ResolutionPlan.originalSize(paths.get(0));
    calibration = ReuseCache.calibration(paths, options, null);
    if (calibration == null) {
      calibration =
          options.calibration == null && Double.isNaN(options.focal)
              ? AutomaticCalibration.estimate(options, paths, first[0], first[1], project)
              : Calibration.load(options, first[0], first[1]);
      if (options.calibration == null
          && !Double.isNaN(options.focal)
          && !options.calibrationPipeline.equals("joint")) {
        calibration = SelfCalibration.refine(calibration, paths, options, project);
        calibration.write(
            options.output.resolve("camera.txt"),
            "Focal/sensor initialization followed by guarded radial refinement");
      }
      if (options.calibration != null
          && !options.intrinsicsMode.equals("fixed")
          && !options.calibrationPipeline.equals("joint"))
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
    Options alignment = alignment();
    for (Path path : paths) frames.add(new ImageFrame(frames.size(), path, calibration, alignment, true));
    writeSettings();
    if (project != null) {
      project.data.inputs(frames, calibration);
      project.stage("00_inputs", "complete", "Image inventory, processed rasters and maps exported");
      project.stage("01_calibration", "complete", "Supplied or derived interior orientation exported");
    }
    saveFrames();
    return "complete";
  }

  private String match() throws IOException {
    if (planningOnly()) return "skipped";
    images();
    loadFrames();
    features = new FeatureTracks();
    if (options.rectified) skipFeatures("Already-rectified stereo bypasses SIFT and SfM");
    else if (options.poses != null) {
      boolean refineFixedIntrinsics = SharedCalibration.enabled(alignment());
      if (options.extrinsicsMode.equals("fixed") && !refineFixedIntrinsics)
        skipFeatures("Supplied camera poses fixed");
      if (project != null) project.stage("04_geometry", "running", "Import supplied exterior orientations");
      PoseIO.read(options.poses, frames);
      if (!options.extrinsicsMode.equals("fixed") || refineFixedIntrinsics) {
        for (ImageFrame f : frames) f.camera.fixed = options.extrinsicsMode.equals("fixed");
        features.build(frames, options, project);
      }
    } else features.build(frames, options, project);
    saveFrames();
    state.writeTracks(features);
    return options.rectified ? "skipped" : "complete";
  }

  private String align() throws IOException {
    if (planningOnly()) return "skipped";
    images();
    loadFrames();
    features = state.readTracks(options);
    String coordinates;
    if (options.rectified) {
      if (project != null) project.stage("04_geometry", "running", "Known rectified stereo exterior orientation");
      Camera a = frames.get(0).camera, b = frames.get(1).camera;
      if (Math.abs(a.fx - b.fx) + Math.abs(a.fy - b.fy) + Math.abs(a.cx - b.cx) + Math.abs(a.cy - b.cy) > 1e-6)
        throw new IllegalArgumentException("Rectified cameras must share intrinsics");
      a.registered = b.registered = true;
      a.fixed = b.fixed = true;
      b.translation = new double[] {-options.baseline, 0, 0};
      features.pairs.add(new FeatureTracks.Pair(0, 1));
      coordinates = "Rectified left-camera frame; baseline=" + options.baseline + " in supplied output units";
    } else if (options.poses != null) {
      boolean refineFixedIntrinsics = SharedCalibration.enabled(alignment());
      // The match step read the supplied poses; they are the cameras as loaded.
      List<Camera> suppliedCameras = frames.stream().map(f -> f.camera.copy()).toList();
      if (!options.extrinsicsMode.equals("fixed") || refineFixedIntrinsics) {
        for (ImageFrame f : frames) {
          if (options.extrinsicsMode.equals("prior")) {
            f.camera.reference = f.camera.copy();
            f.camera.positionSigma = options.positionSigma;
            f.camera.rotationSigma = Math.toRadians(options.rotationSigma);
          }
          f.camera.fixed = options.extrinsicsMode.equals("fixed");
        }
        String key = options.cacheDirectory == null ? null : ReuseCache.geometryKey(frames, features, options);
        if (!ReuseCache.geometry(frames, features, options, key, false)) {
          Geometry.triangulateTracks(frames, features.tracks, options.reprojection * 2);
          if (features.tracks.stream().filter(t -> t.point != null).count() < 8)
            throw new IllegalArgumentException(
                "Supplied pose refinement needs at least eight triangulated matching tracks; check"
                    + " calibration, poses and overlap");
          BundleAdjuster.optimize(frames, features.tracks, options.iterations, options.reprojection, project);
          int scaleCamera = 1;
          while (scaleCamera < frames.size() - 1
              && Math3.norm(Math3.sub(frames.get(scaleCamera).camera.center(), frames.get(0).camera.center()))
                  < 1e-9) scaleCamera++;
          SparseCleanup.run(frames, features.tracks, options, project, 0, scaleCamera);
          ReuseCache.geometry(frames, features, options, key, true);
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
                "image", f.path.getFileName().toString(),
                "mode", options.extrinsicsMode,
                "position_sigma_pose_units", options.positionSigma,
                "rotation_sigma_degrees", options.rotationSigma,
                "adjusted_center", f.camera.center(),
                "adjusted_R", f.camera.rotation,
                "initial_center", suppliedCameras.get(f.id).center(),
                "initial_R", suppliedCameras.get(f.id).rotation,
                "reference_center", f.camera.reference == null ? null : f.camera.reference.center(),
                "prior_cost", BundleAdjuster.priorCost(f.camera)));
      Json.write(options.output.resolve("04_geometry/pose_constraints.json"), changes);
    } else {
      if (project != null) project.stage("04_geometry", "running", "Camera registration and sparse reconstruction");
      String key = options.cacheDirectory == null ? null : ReuseCache.geometryKey(frames, features, options);
      if (!ReuseCache.geometry(frames, features, options, key, false)) {
        Geometry.reconstruct(frames, features, options, project);
        ReuseCache.geometry(frames, features, options, key, true);
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
    calibration = frames.get(0).calibration;
    Path finalConstraints = options.output.resolve("01_calibration/constraints.json");
    Map<String, Object> finalCalibration = JsonRead.object(JsonRead.read(finalConstraints));
    finalCalibration.put("bootstrap_applied_focal", finalCalibration.get("applied_focal"));
    finalCalibration.put("bootstrap_applied_distortion", finalCalibration.get("applied_distortion"));
    finalCalibration.put("applied_focal", calibration.fx);
    finalCalibration.put("applied_distortion", calibration.distortion);
    finalCalibration.put("applied_principal_point", new double[] {calibration.cx, calibration.cy});
    finalCalibration.put("alignment_refinement_report", "04_geometry/bundle/shared_calibration.json");
    Json.write(finalConstraints, finalCalibration);
    if (project != null) project.data.referenceCalibration(calibration);
    calibration.write(options.output.resolve("camera.txt"), "Final applied alignment calibration");
    if (!options.orientationCalibrations.isEmpty()) {
      Map<String, Calibration> updated = new LinkedHashMap<>();
      for (ImageFrame f : frames) updated.put(f.path.getFileName().toString(), f.calibration);
      options.orientationCalibrations = updated;
    }
    state.put("coordinates", coordinates);
    saveFrames();
    state.writeTracks(features);
    return "complete";
  }

  private String georeference() throws IOException {
    if (planningOnly()) return "skipped";
    images();
    loadFrames();
    features = state.readTracks(options);
    String coordinates = state.string("coordinates");
    Path finalConstraints = options.output.resolve("01_calibration/constraints.json");
    Georeference.Result georeference = Georeference.align(frames, features.tracks, options);
    if (!SharedCalibration.same(calibration, frames.get(0).calibration)) {
      // The GPS-weighted adjustment refitted the shared lens; publish it like the bundle result.
      calibration = frames.get(0).calibration;
      Map<String, Object> adjusted = JsonRead.object(JsonRead.read(finalConstraints));
      adjusted.put("pre_gps_adjustment_focal", adjusted.get("applied_focal"));
      adjusted.put("pre_gps_adjustment_distortion", adjusted.get("applied_distortion"));
      adjusted.put("applied_focal", calibration.fx);
      adjusted.put("applied_distortion", calibration.distortion);
      adjusted.put("applied_principal_point", new double[] {calibration.cx, calibration.cy});
      adjusted.put("gps_adjustment_report", "04_geometry/georeference/gps_bundle.json");
      Json.write(finalConstraints, adjusted);
      if (project != null) project.data.referenceCalibration(calibration);
      calibration.write(
          options.output.resolve("camera.txt"),
          "Final applied calibration; shared lens refitted in the GPS-weighted adjustment");
      if (!options.orientationCalibrations.isEmpty()) {
        Map<String, Calibration> updated = new LinkedHashMap<>();
        for (ImageFrame f : frames) updated.put(f.path.getFileName().toString(), f.calibration);
        options.orientationCalibrations = updated;
      }
    }
    CameraScreening.run(frames, features.tracks, options, georeference.crs);
    coordinates += "; " + georeference.description;
    if (georeference.crs != null) coordinates = georeference.description;
    if (project != null) project.data.snapshot("final", frames, features.tracks, coordinates);
    PoseIO.write(options.output.resolve("cameras.csv"), frames);
    writeSparse(features.tracks);
    String geometryQuality = GeometryQuality.write(options.output, frames, features.tracks);
    if (project != null) project.stage("04_geometry", "complete", coordinates);
    state.put("coordinates", coordinates);
    state.put("crs", georeference.crs == null ? null : georeference.crs.name);
    state.put("georeference_description", georeference.description);
    state.put("geometry_quality", geometryQuality);
    saveFrames();
    state.writeTracks(features);
    return "complete";
  }

  /**
   * Ground control points. First run: the GCPs are projected into the images (predicted markers,
   * 04_geometry/gcp/markers.json) and the step waits for pins (photogrammetry gcp-edit). Next run
   * with enough pins: GCP adjustment, residuals and refreshed exports.
   */
  private String gcp() throws IOException {
    if (planningOnly() || options.gcp == null) return "skipped";
    images();
    loadFrames();
    features = state.readTracks(options);
    Crs crs = crs();
    Path file = GroundControl.markersFile(options.output);
    GroundControl.Markers markers =
        Files.isRegularFile(file)
            ? GroundControl.load(file)
            : options.gcpMarkers != null ? GroundControl.load(options.gcpMarkers) : new GroundControl.Markers();
    // The GCP file is read every time, so corrected coordinates or sigmas take effect.
    List<GroundControl.Point3> points = GroundControl.read(options.gcp, options.gcpSigma, options.gcpHeightSigma);
    markers.gcps.clear();
    markers.gcps.addAll(points);
    markers.markers.removeIf(k -> markers.gcp(k.gcp) == null || k.image >= frames.size());
    markers.crs = crs == null ? new Crs(options.crs).name : crs.name;
    GroundControl.Occlusion occlusion = crs != null ? GroundControl.Occlusion.of(features.tracks) : null;
    GroundControl.predict(markers, frames, crs != null, occlusion);
    GroundControl.save(markers, file, frames);
    String waiting = GroundControl.readiness(markers, crs != null);
    if (waiting != null) {
      String root = project == null ? "PROJECT" : project.root.toString();
      String message =
          "Place the GCP markers: "
              + waiting
              + ". Open the editor with 'photogrammetry gcp-edit -o "
              + root
              + "', then run 'photogrammetry gcp -o "
              + root
              + "' again.";
      state.put("gcp_waiting", message);
      System.out.println(message);
      return "waiting";
    }
    Calibration before = calibration;
    GcpAdjustment.Result result =
        GcpAdjustment.run(frames, features.tracks, options, crs, markers, options.output.resolve("04_geometry/gcp"));
    publishCalibration(before, "gcp_adjustment_report", "04_geometry/gcp/gcp_report.json", "shared lens refitted in the GCP adjustment");
    GroundControl.predict(markers, frames, true, GroundControl.Occlusion.of(features.tracks));
    GroundControl.save(markers, file, frames);
    String coordinates =
        result.crs().name + " easting/northing, metres; datum from ground control points. " + result.summary();
    Json.write(
        options.output.resolve("coordinates.json"),
        Json.object(
            "georeferenced", true,
            "horizontal_crs", result.crs().name,
            "epsg", result.crs().epsg,
            "xy_order", "easting,northing",
            "xyz_units", "metres",
            "vertical_crs", null,
            "height_note", "Heights follow the ground control points' vertical reference.",
            "datum", "ground control points",
            "gcp_report", "04_geometry/gcp/gcp_report.json",
            "coordinate_multiplier", 1));
    if (project != null) project.data.snapshot("final", frames, features.tracks, coordinates);
    PoseIO.write(options.output.resolve("cameras.csv"), frames);
    writeSparse(features.tracks);
    state.put("geometry_quality", GeometryQuality.write(options.output, frames, features.tracks));
    state.put("coordinates", coordinates);
    state.put("crs", result.crs().name);
    state.put("gcp_summary", result.summary());
    state.json.remove("gcp_waiting");
    saveFrames();
    state.writeTracks(features);
    return "complete";
  }

  /** After an adjustment that may have refitted the lens: publish it like the bundle result. */
  private void publishCalibration(Calibration before, String reportKey, String reportPath, String note) throws IOException {
    if (SharedCalibration.same(before, frames.get(0).calibration)) return;
    calibration = frames.get(0).calibration;
    Path constraints = options.output.resolve("01_calibration/constraints.json");
    Map<String, Object> adjusted = JsonRead.object(JsonRead.read(constraints));
    adjusted.put("applied_focal", calibration.fx);
    adjusted.put("applied_distortion", calibration.distortion);
    adjusted.put("applied_principal_point", new double[] {calibration.cx, calibration.cy});
    adjusted.put(reportKey, reportPath);
    Json.write(constraints, adjusted);
    if (project != null) project.data.referenceCalibration(calibration);
    calibration.write(options.output.resolve("camera.txt"), "Final applied calibration; " + note);
    if (!options.orientationCalibrations.isEmpty()) {
      Map<String, Calibration> updated = new LinkedHashMap<>();
      for (ImageFrame f : frames) updated.put(f.path.getFileName().toString(), f.calibration);
      options.orientationCalibrations = updated;
    }
  }

  private Crs crs() {
    String name = state.string("crs");
    return name == null ? null : new Crs(name);
  }

  private String dense() throws IOException {
    if (planningOnly()) return "skipped";
    images();
    loadFrames();
    features = state.readTracks(options);
    Crs crs = crs();
    if (project != null)
      project.stage(
          "05_stereo",
          "running",
          options.densePipeline.equals("aerial-tiled")
              ? "Source-region rectification and GPU aerial tiles"
              : options.denseMethod.equals("patchmatch")
              ? "PatchMatch multi-view depth and plane estimation"
              : "Rectification and Java SGM");
    boolean reload = false;
    for (ImageFrame frame : frames) {
      int[] size = options.processedSize(frame.sourceWidth, frame.sourceHeight);
      if (size[0] != frame.width || size[1] != frame.height) reload = true;
    }
    if (reload) {
      List<ImageFrame> denseFrames = new ArrayList<>();
      try {
        for (ImageFrame frame : frames) {
          ImageFrame denseFrame = new ImageFrame(frame.id, frame.path, frame.calibration, options, true);
          denseFrames.add(denseFrame);
          denseFrame.camera.rotation = frame.camera.rotation.clone();
          denseFrame.camera.translation = frame.camera.translation.clone();
          denseFrame.camera.registered = frame.camera.registered;
          denseFrame.camera.fixed = frame.camera.fixed;
          denseFrame.camera.denseExcluded = frame.camera.denseExcluded;
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
    OrientationIO.archive(frames, options, crs == null ? null : crs.name);
    if (OverlapSelection.enabled(options)) {
      List<int[]> edges = new ArrayList<>();
      for (var p : features.pairs) edges.add(new int[] {p.a, p.b});
      List<int[]> accepted = OverlapSelection.filter(frames, edges, features.tracks, options, "dense");
      Set<Long> keys = new HashSet<>();
      for (int[] e : accepted) keys.add(((long) e[0] << 32) | (e[1] & 0xffffffffL));
      features.pairs.removeIf(p -> !keys.contains(((long) p.a << 32) | (p.b & 0xffffffffL)));
      if (features.pairs.isEmpty())
        throw new IllegalArgumentException(
            "No stereo pairs satisfy the enabled overlap thresholds; inspect overlap_dense.json");
    }
    DenseReconstruction dense = new DenseReconstruction(options, frames, project, crs, features.tracks);
    long points = dense.run(features.pairs);
    double seconds = (System.currentTimeMillis() - ((Number) state.get("started_epoch_millis")).longValue()) / 1e3;
    long sparse = features.tracks.stream().filter(t -> t.point != null).count();
    String report =
        "POSE " + PdfReport.VERSION + "\nImages="
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
            + state.string("coordinates")
            + "\nOutput coordinate multiplier="
            + options.scale
            + "\n"
            + "Camera CSV and sparse.csv use pre-multiplier coordinates.\n"
            + (crs == null ? "No EPSG CRS is assigned to LAS.\n" : "LAS GeoTIFF horizontal CRS=" + crs.name + "; see coordinates.json.\n")
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
            + state.string("geometry_quality")
            + "\n"
            + dense.summary()
            + (options.rectified || options.poses != null && options.extrinsicsMode.equals("fixed")
                ? "Final bundle adjustment=skipped; supplied exterior orientations preserved\n"
                : "Final undistorted-image bundle adjustment=joint robust Schur"
                    + " Levenberg-Marquardt, fixed accepted intrinsics, first seed pose and seed"
                    + " baseline length\n")
            + "Legacy C++ numerical parity has not been established.\n"
            + "Elapsed seconds since the run started="
            + seconds
            + "\n";
    Files.writeString(options.output.resolve("report.txt"), report);
    System.out.printf(Locale.ROOT, "Wrote %,d RGB points to %s%n", points, options.output.toAbsolutePath());
    List<String> missing = new ArrayList<>();
    for (ImageFrame f : frames) if (!f.camera.registered) missing.add(f.path.getFileName().toString());
    String registration =
        "Registered cameras=" + (frames.size() - missing.size()) + "/" + frames.size() + "; failed images=" + missing;
    System.out.println(registration);
    Files.writeString(options.output.resolve("report.txt"), registration + "\n", StandardOpenOption.APPEND);
    state.put("dense_points", points);
    state.put("dense_pairs", features.pairs.size());
    return "complete";
  }

  private String ground() throws IOException {
    if (planningOnly()) return "skipped";
    if (!(options.classifyGround || options.groundPairCorrection || options.groundAwareFusion)) return "skipped";
    images();
    loadFrames();
    GroundClassification.run(options, frames, crs());
    return "complete";
  }

  private void skipFeatures(String reason) throws IOException {
    PairMatching.clear();
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
          w.write(track.point[0] + "," + track.point[1] + "," + track.point[2] + "," + track.observations.size() + "\n");
    }
  }

  private void writeSettings() throws IOException {
    Properties settings = new Properties();
    try {
      for (java.lang.reflect.Field field : Options.class.getFields()) {
        Object value = field.get(options);
        if (value != null)
          settings.setProperty(
              field.getName(),
              value instanceof Path ? ((Path) value).toAbsolutePath().normalize().toString() : String.valueOf(value));
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
      settings.setProperty(key + "width", String.valueOf(frame.width));
      settings.setProperty(key + "height", String.valueOf(frame.height));
      settings.setProperty(key + "fx", String.valueOf(frame.camera.fx));
      settings.setProperty(key + "fy", String.valueOf(frame.camera.fy));
    }
    try (Writer writer = Files.newBufferedWriter(options.output.resolve("run-settings.properties"))) {
      settings.store(writer, "Effective reconstruction settings; not a camera calibration file");
    }
  }
}
