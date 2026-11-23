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
 * cleanup      04  optional tie-point cleanup by quality (--tie-point-cleanup) and adjustment
 * georeference 04  GPS similarity, GPS-weighted adjustment, camera screening, final exports
 * lidar        04  optional: block moved onto an ALS reference surface (--lidar-reference)
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
    CLEANUP("cleanup", "Tie-point cleanup by quality and adjustment (optional)"),
    GEOREFERENCE("georeference", "GPS georeferencing, GPS-weighted adjustment and camera screening"),
    LIDAR("lidar", "Lidar-referenced georeferencing (optional)"),
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
    // Decoded images may be kept on disk while the steps up to dense run (ProcessedImageCache).
    if (step.ordinal() <= Step.DENSE.ordinal()) ProcessedImageCache.open(options);
    String outcome;
    try {
      outcome =
          switch (step) {
            case INPUTS -> inputs();
            case CALIBRATE -> calibrate();
            case MATCH -> match();
            case ALIGN -> align();
            case CLEANUP -> cleanup();
            case GEOREFERENCE -> georeference();
            case LIDAR -> lidar();
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
        // Decoded images are reused up to and including the dense step (ProcessedImageCache).
        if (step == Step.DENSE || step == Step.FINISH) {
          ProcessedImageCache.open(options);
          ProcessedImageCache.delete(options);
        }
      }
    }
    state.mark(step, outcome, outcome.equals("waiting") ? state.string(step.name + "_waiting") : step.description);
    if (!outcome.equals("waiting")) state.snapshot(step);
    // The step's workers freed their native buffers; give glibc's cached pages back (Linux).
    NativeMemory.trim("after the " + step.name + " step");
  }

  /**
   * Checks a step list against a run's state before anything is opened, so a refused command
   * leaves the run untouched: steps in pipeline order, none already done, earlier ones done.
   */
  static void check(PipelineState state, List<Step> steps, Options options) {
    for (int i = 1; i < steps.size(); i++)
      if (steps.get(i).ordinal() <= steps.get(i - 1).ordinal())
        throw new IllegalArgumentException(
            "Give the steps in pipeline order: inputs calibrate match align cleanup georeference lidar gcp dense ground finish");
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
      case CLEANUP -> o.tiePointCleanup.equals("off");
      case LIDAR -> o.lidarReference == null;
      case GCP -> o.gcp == null;
      case GROUND -> !(o.classifyGround || o.groundPairCorrection || o.groundAwareFusion);
      default -> false;
    };
  }

  /** Steps that run only when asked for (notRequested can be true). */
  static boolean optional(Step step) {
    return step == Step.CLEANUP || step == Step.LIDAR || step == Step.GCP || step == Step.GROUND;
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
      case ALIGN, CLEANUP, GEOREFERENCE, LIDAR, GCP -> List.of("04_geometry");
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
      if (!status.equals("complete") && !status.equals("skipped") && !(status.equals("pending") && optional(prior)))
        throw new IllegalArgumentException(
            "Cannot branch at '" + from.name + "': step '" + prior.name + "' of the source run is " + status);
    }
    // The state after the step before 'from'; an optional step without a saved state (skipped in a
    // run made before the step existed, or pending and not asked for) changed nothing, so the state
    // after the step before it is the same.
    Step before = Step.values()[from.ordinal() - 1];
    while (before.ordinal() > 0 && optional(before) && !Files.isDirectory(source.resolve("state/after-" + before.name))
        && !original.status(before).equals("complete"))
      before = Step.values()[before.ordinal() - 1];
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
    // coordinates.json comes from the georeference step but a finished run keeps it in 07_results,
    // which belongs to the dense step and is not carried when branching at dense or earlier.
    // Without it the branch's report took the cloud for a local frame (no DSM, no orthomosaic).
    if (from.ordinal() > Step.GEOREFERENCE.ordinal()
        && !Files.isRegularFile(project.run.resolve("coordinates.json"))
        && !Files.isRegularFile(project.run.resolve("07_results/coordinates.json")))
      for (Path file :
          new Path[] {source.resolve("coordinates.json"), source.resolve("07_results/coordinates.json")})
        if (Files.isRegularFile(file)) {
          Files.copy(file, project.run.resolve("coordinates.json"), StandardCopyOption.REPLACE_EXISTING);
          break;
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
    // A run made before camera groups existed holds one camera.
    CameraGroups groups = (CameraGroups) saved[2];
    if (groups == null)
      groups =
          CameraGroups.legacy(paths.stream().map(p -> p.getFileName().toString()).toList(), calibration);
    options.cameraGroups = groups;
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
    CameraGroups groups = options.cameraGroups;
    if (groups != null && groups.multiple()) {
      // Every image keeps its own camera's calibration, also after an adjustment refitted lenses,
      // so frames rebuilt later (dense step) never see a stale or another camera's calibration.
      Map<String, Calibration> updated = new LinkedHashMap<>();
      for (ImageFrame f : frames) updated.put(f.path.getFileName().toString(), f.calibration);
      options.orientationCalibrations = updated;
    }
    if (groups != null) {
      groups.refresh(frames);
      groups.export(options, options.output);
    }
    state.writeFrames(frames);
    state.writeCalibrations("calibration.bin", calibration, options.orientationCalibrations, groups);
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
    // Input provenance hashes for images.csv, in the background while calibration decodes.
    if (project != null) project.data.hashSources(paths);
    CameraGroups groups =
        CameraGroups.resolve(options, paths, options.importedCameraIds, options.importedPixelSizes);
    options.cameraGroups = groups;
    groups.checkImported(options);
    if (groups.multiple()) return calibrateCameras(groups);
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
    CameraGroups.Group camera = groups.get(0);
    camera.supplied = options.calibration != null;
    camera.initialization =
        options.calibration != null
            ? "supplied"
            : !Double.isNaN(options.focal) ? "focal_and_sensor_width" : "automatic";
    return framesAndSettings();
  }

  /**
   * Interior orientation of every camera group (two or more cameras), each in its own source
   * pixels. Per camera, the first source that applies: imported orientation, --calibration
   * CAMERA=FILE, --focal/--s_w (CAMERA= or plain), then automatic: the focal bootstrap from image
   * pairs, or for a camera with fewer than {@link #BOOTSTRAP_MIN_IMAGES} images its EXIF/XMP
   * focal. The guarded bootstrap refinement follows as for one camera. Working files of each
   * camera: 01_calibration/cameras/ID/.
   */
  /**
   * The calibration of one camera from --calibration CAMERA=FILE, in any supported format
   * (--calibration-format, default auto): POSE camera.txt or the 14-number calib.result, or a
   * foreign file (OpenCV, Metashape, MicMac, COLMAP, INPHO, ...). The file must describe one
   * camera: its image-less (shared) calibration, else the calibration of this camera's images in
   * it, else its only calibration. A file with several different ones is an error.
   */
  static Calibration cameraFile(Path file, CameraGroups.Group g, List<Path> members, Options go)
      throws IOException {
    Map<String, int[]> sizes = new LinkedHashMap<>();
    for (Path p : members) sizes.put(p.getFileName().toString(), new int[] {g.width, g.height});
    // A one-camera INPHO file: --orientation-pixel-size CAMERA=MM of this camera applies.
    Double pixel = go.orientationPixelSizes.get(g.id);
    if (pixel != null) go.orientationPixelSize = pixel;
    OrientationIO.Model m = OrientationIO.read(file, go.calibrationFormat, sizes, go);
    List<Calibration> found = new ArrayList<>();
    OrientationIO.View shared = m.views.get("*");
    if (shared != null && shared.calibration() != null) found.add(shared.calibration());
    else {
      for (var e : m.views.entrySet())
        if (sizes.containsKey(e.getKey()) && e.getValue().calibration() != null) found.add(e.getValue().calibration());
      if (found.isEmpty())
        for (OrientationIO.View v : m.views.values()) if (v.calibration() != null) found.add(v.calibration());
    }
    List<Calibration> distinct = new ArrayList<>();
    for (Calibration c : found) if (distinct.stream().noneMatch(d -> sameCalibration(c, d))) distinct.add(c);
    if (distinct.isEmpty())
      throw new IllegalArgumentException("--calibration " + g.id + "=" + file + ": no interior orientation in the file");
    if (distinct.size() > 1)
      throw new IllegalArgumentException(
          "--calibration " + g.id + "=" + file + " holds " + distinct.size() + " different calibrations;"
              + " give a file of this camera only, or the whole file with --orientation-input"
              + " (its camera IDs then name the cameras)");
    return distinct.getFirst();
  }

  private static boolean sameCalibration(Calibration a, Calibration b) {
    return a.fx == b.fx && a.fy == b.fy && a.cx == b.cx && a.cy == b.cy && a.width == b.width
        && a.height == b.height && Arrays.equals(a.distortion, b.distortion);
  }

  private String calibrateCameras(CameraGroups groups) throws IOException {
    if (options.calibration != null && options.importedCalibrations.isEmpty())
      throw new IllegalArgumentException(
          "--calibration FILE describes one camera, but the images come from "
              + groups.size()
              + " cameras "
              + groups.groups.stream().map(g -> g.id + " (" + g.label + ")").toList()
              + ": give each its own file with --calibration CAMERA=FILE");
    Map<String, Calibration> perImage = new LinkedHashMap<>();
    Map<String, Camera> seedPoses = Map.of();
    int seedPoseCount = 0;
    boolean joint = options.calibrationPipeline.equals("joint");
    List<Object> summary = new ArrayList<>();
    for (CameraGroups.Group g : groups.groups) {
      List<Path> members = new ArrayList<>();
      for (Path p : paths) if (groups.index(p) == groups.groups.indexOf(g)) members.add(p);
      String policy = CameraGroups.policy(g, options);
      Path dir = options.output.resolve("01_calibration/cameras/" + CameraGroups.safe(g.id));
      Files.createDirectories(dir);
      Options go = options.copy();
      go.output = dir;
      go.calibration = null;
      go.focal = Double.NaN;
      go.sensorWidth = Double.NaN;
      go.orientationCalibrations = Map.of();
      go.calibrationPoses = Map.of();
      // The camera's own images are calibrated as a one-camera block.
      go.cameraGroups = new CameraGroups(List.of(g));
      go.intrinsicsMode = policy.equals("free") ? options.intrinsicsMode : policy;
      boolean imported =
          !policy.equals("free")
              && members.stream()
                  .allMatch(p -> options.importedCalibrations.containsKey(p.getFileName().toString()));
      Path file = options.calibrationFiles.get(g.id);
      double focal = options.focalByCamera.getOrDefault(g.id, options.focal),
          sensor = options.sensorWidthByCamera.getOrDefault(g.id, options.sensorWidth);
      Calibration c;
      if (imported) {
        for (Path p : members) {
          String name = p.getFileName().toString();
          perImage.put(name, options.importedCalibrations.get(name).atSize(g.width, g.height));
        }
        c = perImage.get(members.getFirst().getFileName().toString());
        // Marks the camera as supplied for the guarded refinement and the bundle priors.
        go.calibration = options.calibration;
        g.supplied = true;
        g.initialization = "imported";
      } else if (file != null) {
        go.calibration = file;
        c = cameraFile(file, g, members, go).atSize(g.width, g.height);
        g.supplied = true;
        g.initialization = "supplied_file";
      } else if (!Double.isNaN(focal)) {
        go.focal = focal;
        go.sensorWidth = sensor;
        c = Calibration.load(go, g.width, g.height);
        g.initialization = "focal_and_sensor_width";
      } else if (members.size() >= BOOTSTRAP_MIN_IMAGES) {
        c = AutomaticCalibration.estimate(go, members, g.width, g.height, null);
        g.initialization = "automatic";
      } else {
        c = exifCalibration(g, members);
        g.initialization = "exif_prior_small_camera";
      }
      if (!imported && !g.supplied && !Double.isNaN(go.focal) && !joint)
        c = SelfCalibration.refine(c, members, go, null);
      if (g.supplied && !policy.equals("fixed") && !joint)
        c = SelfCalibration.refine(c, members, go, null);
      g.calibration = c;
      if (!imported) for (Path p : members) perImage.put(p.getFileName().toString(), c);
      // Bootstrap poses seed alignment; poses of different cameras are in different frames, so
      // only one camera's (the one with most of them) are kept.
      if (go.calibrationPoses.size() > seedPoseCount) {
        seedPoses = go.calibrationPoses;
        seedPoseCount = seedPoses.size();
      }
      c.write(dir.resolve("camera.txt"), "Camera " + g.id + " initial calibration (" + g.initialization + ")");
      summary.add(
          Json.object(
              "camera", g.id, "label", g.label, "images", members.size(), "policy", policy,
              "initialization", g.initialization, "focal_pixels", c.fx, "distortion", c.distortion,
              "width", c.width, "height", c.height));
      System.out.printf(
          Locale.ROOT,
          "Camera %s (%s): %d images, %s, f=%.3f px%n",
          g.id,
          g.label,
          members.size(),
          g.initialization,
          c.fx);
    }
    options.orientationCalibrations = perImage;
    options.calibrationPoses = seedPoses;
    calibration = groups.get(0).calibration;
    calibration.write(
        options.output.resolve("camera.txt"),
        "Applied calibration of camera " + groups.get(0).id + " (first image); "
            + groups.size() + " cameras: see 01_calibration/cameras.json");
    Path constraints = options.output.resolve("01_calibration/constraints.json");
    Json.write(
        constraints,
        Json.object(
            "mode", "camera_groups",
            "refined_parameters", options.refineIntrinsics,
            "focal_relative_sigma", options.focalSigma,
            "radial_coefficient_sigma", options.radialSigma,
            "supplied_file", options.calibration == null ? null : options.calibration.toString(),
            "applied_focal", calibration.fx,
            "applied_distortion", calibration.distortion,
            "camera_groups", "01_calibration/cameras.json",
            "cameras", summary,
            "note",
            "Several cameras: applied_* describe the first image's camera. Fixed parameters remain"
                + " exact; prior uncertainties are assumptions, not measured accuracy."));
    return framesAndSettings();
  }

  static final int BOOTSTRAP_MIN_IMAGES = 3;

  /** A small camera's start: EXIF 35 mm equivalent, else the DJI XMP calibrated focal. */
  private static Calibration exifCalibration(CameraGroups.Group g, List<Path> members) throws IOException {
    List<Double> equivalent = new ArrayList<>(), xmp = new ArrayList<>();
    for (Path p : members) {
      Exif.Metadata m = Exif.read(p);
      if (m.focal35Mm > 0) equivalent.add(m.focal35Mm / Math.hypot(36, 24) * Math.hypot(g.width, g.height));
      if (m.calibratedFocal > 0) xmp.add(m.calibratedFocal);
    }
    double f = !equivalent.isEmpty() ? median(equivalent) : !xmp.isEmpty() ? median(xmp) : Double.NaN;
    if (!(f > 0))
      throw new IllegalArgumentException(
          "Camera "
              + g.id
              + " ("
              + g.label
              + ") has "
              + members.size()
              + " image(s): too few for automatic calibration, and no EXIF 35 mm focal length. Supply"
              + " --calibration "
              + g.id
              + "=FILE, or --focal "
              + g.id
              + "=MM --s_w "
              + g.id
              + "=MM");
    return new Calibration(f, f, (g.width - 1) / 2., (g.height - 1) / 2., new double[5], g.width, g.height);
  }

  private static double median(List<Double> values) {
    List<Double> v = new ArrayList<>(values);
    Collections.sort(v);
    return v.get(v.size() / 2);
  }

  /** Alignment frames, run settings and inventory once the calibrations are known. */
  private String framesAndSettings() throws IOException {
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
      boolean tiePoints = fixedPoseTiePoints(refineFixedIntrinsics);
      if (options.extrinsicsMode.equals("fixed") && !refineFixedIntrinsics && !tiePoints)
        skipFeatures("Supplied camera poses fixed");
      if (project != null) project.stage("04_geometry", "running", "Import supplied exterior orientations");
      PoseIO.read(options.poses, frames);
      if (!options.extrinsicsMode.equals("fixed") || refineFixedIntrinsics || tiePoints) {
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
      if (fixedPoseTiePoints(refineFixedIntrinsics)) {
        // --dense-method mvsgm with fixed supplied poses: tie points are triangulated with the
        // poses and calibration exactly as supplied (no adjustment); mvsgm takes its neighbours
        // and depth ranges from them.
        for (ImageFrame f : frames) f.camera.fixed = true;
        Geometry.triangulateTracks(frames, features.tracks, options.reprojection * 2);
        long triangulated = features.tracks.stream().filter(t -> t.point != null).count();
        System.out.printf(
            Locale.ROOT,
            "Tie points with the supplied poses (fixed, not adjusted): %,d of %,d tracks triangulated"
                + " for --dense-method mvsgm%n",
            triangulated,
            features.tracks.size());
        if (triangulated < 12)
          throw new IllegalArgumentException(
              "--dense-method mvsgm needs sparse tie points, but only " + triangulated
                  + " tracks triangulate with the supplied poses (reprojection limit "
                  + options.reprojection * 2 + " px); check the poses, calibration and image overlap,"
                  + " or use --dense-method sgm");
      } else if (!options.extrinsicsMode.equals("fixed") || refineFixedIntrinsics) {
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

  /**
   * Tie-point cleanup by quality (--tie-point-cleanup, TiePointCleanup). auto: the --tie-point-*
   * limits are applied and the block adjusted again. pause: the first run writes the per-point
   * measures and waits; the POSE window writes its choice (04_geometry/tie_points/selection.json)
   * and runs the step again, which applies or skips it. A choice left in a run copied by --branch
   * is ignored: only a choice made while this run waits counts.
   */
  private String cleanup() throws IOException {
    if (planningOnly() || options.tiePointCleanup.equals("off")) return "skipped";
    images();
    loadFrames();
    features = state.readTracks(options);
    Path dir = options.output.resolve(TiePointCleanup.FOLDER);
    Files.createDirectories(dir);
    if (project != null) project.stage("04_geometry", "running", "Tie-point cleanup by quality");
    // The per-point table is written first and read back: the window, the automatic limits and
    // the selection all use the same (rounded) values.
    TiePointCleanup.write(dir.resolve("metrics.csv"), TiePointCleanup.measure(frames, features.tracks));
    TiePointCleanup.writeCameras(dir.resolve("cameras.csv"), frames);
    List<TiePointCleanup.Point> points = TiePointCleanup.read(dir.resolve("metrics.csv"));
    TiePointCleanup.Thresholds limits;
    if (options.tiePointCleanup.equals("pause")) {
      boolean waited = state.string("cleanup_waiting") != null;
      TiePointCleanup.Choice choice = waited ? TiePointCleanup.readChoice(options.output) : null;
      if (choice == null) {
        Files.deleteIfExists(TiePointCleanup.choiceFile(options.output));
        String root = project == null ? "PROJECT" : project.root.toString();
        String message =
            "Choose the tie-point cleanup limits: in the POSE window (Project tab, Clean tie points), or run"
                + " 'photogrammetry cleanup -o " + root + " --tie-point-cleanup auto' (automatic limits) or '--tie-point-cleanup custom [--tie-point-max-error PX"
                + " --tie-point-min-views N --tie-point-min-angle DEG]' (--tie-point-cleanup off skips it). "
                + String.format(Locale.ROOT, "%,d tie points measured.", points.size());
        state.put("cleanup_waiting", message);
        System.out.println(message);
        return "waiting";
      }
      state.json.remove("cleanup_waiting");
      if (!choice.apply()) {
        Json.write(dir.resolve("cleanup.json"), Json.object("mode", "pause", "action", "skipped by the user",
            "points", points.size()));
        System.out.println("Tie-point cleanup skipped (chosen in the POSE window).");
        return "skipped";
      }
      limits = choice.thresholds();
    } else {
      state.json.remove("cleanup_waiting");
      // auto: the automatic limits; custom: the --tie-point-* limits (unset ones at their defaults).
      limits = options.tiePointCleanup.equals("custom") ? TiePointCleanup.thresholds(options, points) : TiePointCleanup.automatic(points);
    }
    TiePointCleanup.Selection s = TiePointCleanup.select(points, frames.size(), limits);
    double before = GpsAdjustment.reprojectionRms(frames, features.tracks, null);
    int registeredBefore = (int) frames.stream().filter(f -> f.camera.registered).count();
    for (int i = 0; i < points.size(); i++)
      if (s.remove()[i]) features.tracks.get(points.get(i).track()).point = null;
    if (s.removed() > 0) {
      BundleAdjuster.optimize(frames, features.tracks, options.iterations, options.reprojection, project);
      // As at the end of align: points the adjustment could not fit are dropped.
      for (FeatureTracks.Track t : features.tracks)
        if (t.point != null && t.prior == null && !Geometry.valid(frames, t, t.point, options.reprojection * 2)) t.point = null;
    }
    double after = GpsAdjustment.reprojectionRms(frames, features.tracks, null);
    int registeredAfter = (int) frames.stream().filter(f -> f.camera.registered).count();
    long remaining = features.tracks.stream().filter(t -> t.point != null && t.prior == null).count();
    String summary = String.format(Locale.ROOT,
        "Tie-point cleanup (%s): removed %,d of %,d tie points (error > %.2f px: %,d; views < %d: %,d; angle < %.1f deg: %,d;"
            + " kept for camera support: %,d; over the %.0f%% limit: %,d); reprojection RMS %.3f -> %.3f px",
        options.tiePointCleanup, s.removed(), points.size(), limits.maxError(), s.byError(), limits.minViews(), s.byViews(),
        limits.minAngle(), s.byAngle(), s.protectedBySupport(), limits.maxRemovedPercent(), s.capped(), before, after);
    System.out.println(summary);
    Map<String, Object> report = new LinkedHashMap<>();
    report.put("mode", options.tiePointCleanup);
    report.put("limits", limits.json());
    report.put("points_before", points.size());
    report.put("removed", s.removed());
    report.put("failing_max_error", s.byError());
    report.put("failing_min_views", s.byViews());
    report.put("failing_min_angle", s.byAngle());
    report.put("kept_for_camera_support", s.protectedBySupport());
    report.put("kept_over_removal_limit", s.capped());
    report.put("points_after_adjustment", remaining);
    report.put("reprojection_rms_before_pixels", before);
    report.put("reprojection_rms_after_pixels", after);
    report.put("registered_cameras_before", registeredBefore);
    report.put("registered_cameras_after", registeredAfter);
    report.put("summary", summary);
    Json.write(dir.resolve("cleanup.json"), report);
    state.put("tie_point_cleanup", summary);
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
    state.put("crs", georeference.crs == null ? null : georeference.crs.spec());
    state.put("georeference_description", georeference.description);
    state.put("geometry_quality", geometryQuality);
    saveFrames();
    state.writeTracks(features);
    return "complete";
  }

  /**
   * Lidar-referenced georeferencing (--lidar-reference, LidarReference): checks that the lidar
   * covers the block, estimates the rigid transformation onto the ALS surface and moves cameras and
   * tie points with it, so dense matching and all products are in the lidar's frame and height
   * datum. A separate step (not part of georeference) so it can be redone with other settings by
   * --branch without repeating the GPS adjustment, and so a missing tile stops the run before the
   * dense step with a clear message.
   */
  private String lidar() throws IOException {
    if (planningOnly() || options.lidarReference == null) return "skipped";
    images();
    loadFrames();
    features = state.readTracks(options);
    Crs crs = crs();
    if (crs == null)
      throw new IllegalArgumentException(
          "--lidar-reference needs a block georeferenced in a projected CRS first (GPS positions in the images, or supplied"
              + " orientations with --orientation-crs); the georeference step left it in a local frame: "
              + state.string("georeference_description"));
    if (project != null) project.stage("04_geometry", "running", "Lidar-referenced georeferencing");
    LidarReference.Result result =
        LidarReference.run(frames, features.tracks, options, crs, options.output.resolve(LidarReference.FOLDER), System.out::println);
    String vertical = result.source().vertical();
    String coordinates = crs.name + " easting/northing, metres; heights: " + vertical + "; datum from the lidar reference. " + result.summary();
    int verticalEpsg = 0;
    for (LasFile.Header h : result.source().files()) verticalEpsg = Math.max(verticalEpsg, h.crs().vertical());
    // The lidar's declared height system labels the outputs; else the vertical part of --crs.
    int labelled = verticalEpsg > 0 && options.lidarVerticalDatum == null ? verticalEpsg : crs.vertical;
    if (crs.vertical > 0 && labelled != crs.vertical)
      System.out.println("Warning: --crs names heights " + Crs.verticalName(crs.vertical) + " but the lidar reference declares "
          + Crs.verticalName(labelled) + "; outputs follow the lidar and are labelled " + Crs.verticalName(labelled) + ".");
    if (labelled != crs.vertical) state.put("crs", crs.withVertical(labelled).spec());
    Json.write(
        options.output.resolve("coordinates.json"),
        Json.object(
            "georeferenced", true,
            "horizontal_crs", crs.name,
            "epsg", crs.epsg,
            "xy_order", "easting,northing",
            "xyz_units", "metres",
            "vertical_crs", labelled > 0 ? "EPSG:" + labelled : null,
            "vertical_datum", vertical,
            "height_note", "Heights follow the lidar reference: " + vertical + ".",
            "datum", "lidar reference",
            "lidar_report", LidarReference.FOLDER + "/lidar_reference.json",
            "coordinate_multiplier", 1));
    if (project != null) project.data.snapshot("final", frames, features.tracks, coordinates);
    PoseIO.write(options.output.resolve("cameras.csv"), frames);
    writeSparse(features.tracks);
    state.put("geometry_quality", GeometryQuality.write(options.output, frames, features.tracks));
    if (project != null) project.stage("04_geometry", "complete", coordinates);
    state.put("coordinates", coordinates);
    state.put("lidar_summary", result.summary());
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
    GroundControl.Selection selection =
        GroundControl.holdOut(
            GroundControl.read(options.gcp, options.gcpSigma, options.gcpHeightSigma), options.gcpCheck, GroundControl.ids(options.gcpCheckIds));
    List<GroundControl.Point3> points = selection.points();
    markers.checkNote = selection.note();
    if (selection.note() != null) System.out.println(selection.note());
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
            "vertical_crs", result.crs().vertical > 0 ? "EPSG:" + result.crs().vertical : null,
            "height_note", "Heights follow the ground control points' vertical reference.",
            "datum", "ground control points",
            "gcp_report", "04_geometry/gcp/gcp_report.json",
            "coordinate_multiplier", 1));
    if (project != null) project.data.snapshot("final", frames, features.tracks, coordinates);
    PoseIO.write(options.output.resolve("cameras.csv"), frames);
    writeSparse(features.tracks);
    state.put("geometry_quality", GeometryQuality.write(options.output, frames, features.tracks));
    state.put("coordinates", coordinates);
    state.put("crs", result.crs().spec());
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
              : options.denseMethod.equals("mvsgm")
              ? "Multi-view plane-sweep SGM depth maps per camera"
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
    if (!options.denseMethod.equals("mvsgm")) {
      // Two-view rectification works on one image size; pairs of two cameras whose processed sizes
      // differ are left out (each camera still pairs with itself; mvsgm uses every camera).
      int before = features.pairs.size();
      features.pairs.removeIf(
          p -> frames.get(p.a).width != frames.get(p.b).width || frames.get(p.a).height != frames.get(p.b).height);
      if (features.pairs.size() < before)
        System.out.println(
            "Dense pairs: "
                + (before - features.pairs.size())
                + " of "
                + before
                + " pairs join cameras of different image sizes and are left out of two-view stereo"
                + " (--dense-method mvsgm uses them)");
      if (features.pairs.isEmpty())
        throw new IllegalArgumentException(
            "No dense pair joins two images of the same size; with cameras of different image sizes use"
                + " --dense-method mvsgm");
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
    if (!Files.isRegularFile(options.output.resolve("07_results/point-confidence.bin")))
      System.err.println(
          "Warning: 07_results/point-confidence.bin is missing (dense step ran with"
              + " --point-confidence off): ground classification runs without camera-support"
              + " free-space checks and ground sheets are skipped. Re-run dense with it on for"
              + " full ground processing.");
    GroundClassification.run(options, frames, crs());
    return "complete";
  }

  /**
   * Fixed supplied poses normally skip features and matching. --dense-method mvsgm needs sparse
   * tie points (neighbour choice and per-camera depth ranges), so they are still matched and later
   * triangulated with the poses held fixed.
   */
  private boolean fixedPoseTiePoints(boolean refineFixedIntrinsics) {
    return options.poses != null
        && options.extrinsicsMode.equals("fixed")
        && !refineFixedIntrinsics
        && options.denseMethod.equals("mvsgm");
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
