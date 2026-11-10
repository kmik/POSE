package sgm;

import java.io.*;
import java.nio.channels.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.security.*;
import java.time.*;
import java.time.format.DateTimeFormatter;
import java.util.*;

/** Append-only project history. Existing run directories are never overwritten or deleted. */
final class Project {
  final Path root, run;
  final String id;
  final ProjectArtifacts data;
  private final Properties marker = new Properties();
  private final Map<String, Map<String, Object>> stages = new LinkedHashMap<>();
  private Instant started = Instant.now();
  private String[] arguments;
  // True when this process continues a run started by an earlier pipeline step.
  private final boolean attached;
  private final Options options;

  private record VerifiedHash(
      long bytes, java.nio.file.attribute.FileTime modified, Object fileKey, String hash) {}

  private final Map<Path, VerifiedHash> artifactHashes = new HashMap<>();
  private FileChannel lockChannel;
  private RunTelemetry telemetry;
  private final Map<String, WorkProgress> stageWork = new LinkedHashMap<>();
  private final Map<String, Long> stageStarted = new LinkedHashMap<>();
  private FileLock lock;
  private static final String FORMAT = "msgm-java-project-v1";
  private static final String[] STAGES = {
    "00_inputs",
    "01_calibration",
    "02_features",
    "03_matches",
    "04_geometry",
    "05_stereo",
    "06_fusion",
    "07_results"
  };

  static Project open(Options options, String[] args) throws IOException {
    return new Project(options, args);
  }

  /**
   * Continues an existing run (a later pipeline step in a new process): same run directory and
   * id, stage history and start time from run.json, logs appended, telemetry merged.
   */
  static Project attach(Options options, String[] args, String runId) throws IOException {
    return new Project(options, args, runId);
  }

  private Project(Options options, String[] args, String runId) throws IOException {
    attached = true;
    this.options = options;
    root = options.output.toAbsolutePath().normalize();
    Path markerPath = root.resolve("project.properties");
    if (!Files.exists(markerPath))
      throw new IllegalArgumentException("Not a POSE project: " + root);
    try (Reader reader = Files.newBufferedReader(markerPath)) {
      marker.load(reader);
    }
    if (!FORMAT.equals(marker.getProperty("format")))
      throw new IOException("Unrecognized project.properties in " + root);
    id = runId;
    run = root.resolve("runs").resolve(id);
    if (!Files.isRegularFile(run.resolve("run.json")))
      throw new IllegalArgumentException("No run " + id + " in " + root);
    lockChannel =
        FileChannel.open(
            root.resolve(".project.lock"), StandardOpenOption.CREATE, StandardOpenOption.WRITE);
    try {
      lock = lockChannel.tryLock();
      if (lock == null)
        throw new IOException("Another photogrammetry process is using this project");
    } catch (IOException | OverlappingFileLockException e) {
      lockChannel.close();
      throw new IOException("Cannot lock project: another process may be using it", e);
    }
    try {
      Map<String, Object> previous = JsonRead.object(JsonRead.read(run.resolve("run.json")));
      started = Instant.parse(String.valueOf(previous.get("started_utc")));
      List<String> original = new ArrayList<>();
      if (previous.get("arguments") instanceof List<?> list)
        for (Object a : list) original.add(String.valueOf(a));
      arguments = original.toArray(String[]::new);
      Map<String, Object> saved = JsonRead.object(previous.get("stages"));
      for (String stage : STAGES) {
        Files.createDirectories(run.resolve(stage));
        Object entry = saved.get(stage);
        stages.put(
            stage,
            entry == null
                ? Json.object("status", "pending")
                : new LinkedHashMap<>(JsonRead.object(entry)));
      }
      data = new ProjectArtifacts(this, options);
      options.output = run;
      options.projectDirectory = root;
      options.cacheDirectory =
          options.cache
              ? root.resolve("cache/v3")
                  .resolve(options.storage + (options.diagnostics ? "-diagnostics" : "-compact"))
              : null;
      if (options.cacheDirectory == null)
        options.runFeatureDirectory = Files.createTempDirectory(run, ".feature-reuse-");
      marker.setProperty("latest_run", id);
      persistProject("running");
      Json.write(run.resolve("run.json"), runDescription("running", null));
      event("step_started", Json.object("arguments", args, "project", root.toString()));
      telemetry = new RunTelemetry(run, options);
      telemetry.continueFrom(
          run.resolve("logs/processing.json"), Duration.between(started, Instant.now()).toMillis() / 1e3);
    } catch (IOException | RuntimeException e) {
      if (telemetry != null) telemetry.close();
      releaseLock();
      throw e;
    }
  }

  /**
   * Ends a pipeline step without finalizing the run: closes artifact writers, records status,
   * merges telemetry and releases the project lock. The last step calls {@link #finish} instead.
   */
  void suspend(boolean success, Throwable error, String nextStep) throws IOException {
    try {
      try {
        data.close();
      } finally {
        cleanupRunFeatures();
      }
      String status = success ? "incomplete" : "failed";
      for (String name : STAGES)
        if (!success && "running".equals(stages.get(name).get("status")))
          stageFinal(name, "failed", error == null ? "Step failed" : error.toString());
      if (error != null) {
        StringWriter stack = new StringWriter();
        error.printStackTrace(new PrintWriter(stack));
        Files.writeString(run.resolve("logs/error.txt"), stack.toString());
      }
      event(
          success ? "step_complete" : "step_failed",
          Json.object("error", error == null ? null : error.toString(), "next_step", nextStep));
      Map<String, Object> description = JsonRead.object(runDescription(status, error));
      description.put("next_step", nextStep);
      Json.write(run.resolve("run.json"), description);
      if (telemetry != null) telemetry.persist();
      persistProject(status);
    } finally {
      try {
        if (telemetry != null) telemetry.close();
      } finally {
        for (WorkProgress work : stageWork.values()) work.close();
        stageWork.clear();
        releaseLock();
      }
    }
  }

  /** Copies stage history (status, detail, timings) from a run this run branched from. */
  void importStages(Map<String, Object> source, Collection<String> names) throws IOException {
    for (String name : names) {
      Object entry = source.get(name);
      if (entry != null && stages.containsKey(name)) stages.put(name, new LinkedHashMap<>(JsonRead.object(entry)));
    }
    Json.write(run.resolve("run.json"), runDescription("running", null));
  }

  private Project(Options options, String[] args) throws IOException {
    attached = false;
    this.options = options;
    this.arguments = args.clone();
    root = options.output.toAbsolutePath().normalize();
    Path input = options.input.toAbsolutePath().normalize();
    Files.createDirectories(root);
    if (Files.isSameFile(root, input))
      throw new IllegalArgumentException("Project directory must differ from input directory");
    Path markerPath = root.resolve("project.properties");
    boolean newProject = !Files.exists(markerPath);
    if (Files.exists(markerPath)) {
      try (Reader reader = Files.newBufferedReader(markerPath)) {
        marker.load(reader);
      }
      if (!FORMAT.equals(marker.getProperty("format")))
        throw new IOException("Unrecognized project.properties in " + root);
    } else {
      try (java.util.stream.Stream<Path> children = Files.list(root)) {
        if (!options.overwrite
            && children.anyMatch(p -> !p.getFileName().toString().equals(".project.lock")))
          throw new IllegalArgumentException(
              "Directory is not a POSE project and is not empty; use a new directory or"
                  + " --overwrite to adopt it");
      }
      marker.setProperty("format", FORMAT);
      marker.setProperty("created_utc", started.toString());
    }
    lockChannel =
        FileChannel.open(
            root.resolve(".project.lock"), StandardOpenOption.CREATE, StandardOpenOption.WRITE);
    try {
      lock = lockChannel.tryLock();
      if (lock == null)
        throw new IOException("Another photogrammetry process is using this project");
    } catch (IOException | OverlappingFileLockException e) {
      lockChannel.close();
      throw new IOException("Cannot lock project: another process may be using it", e);
    }
    id =
        DateTimeFormatter.ofPattern("uuuuMMdd'T'HHmmssSSS'Z'")
                .withZone(ZoneOffset.UTC)
                .format(started)
            + "_"
            + UUID.randomUUID().toString().substring(0, 8);
    try {
      run = root.resolve("runs").resolve(id);
      Files.createDirectories(run.resolve("logs"));
      for (String stage : STAGES) {
        Files.createDirectories(run.resolve(stage));
        stages.put(stage, Json.object("status", "pending"));
      }
      data = new ProjectArtifacts(this, options);
      options.output = run;
      options.projectDirectory = root;
      options.cacheDirectory =
          options.cache
              ? root.resolve("cache/v3")
                  .resolve(options.storage + (options.diagnostics ? "-diagnostics" : "-compact"))
              : null;
      if (options.cacheDirectory == null)
        options.runFeatureDirectory = Files.createTempDirectory(run, ".feature-reuse-");
      if (newProject) archiveReservedRootFiles();
      if (newProject || !Files.exists(root.resolve("latest-files.csv")))
        Files.writeString(
            root.resolve("latest-files.csv"), "filename,run_id,canonical_relative_path,sha256\n");
      marker.setProperty("latest_run", id);
      persistProject("running");
      Json.write(run.resolve("run.json"), runDescription("running", null));
      event("run_started", Json.object("arguments", arguments, "project", root.toString()));
      Json.write(
          run.resolve("logs/request.json"),
          Json.object(
              "arguments",
              arguments,
              "requested_project_root",
              root.toString(),
              "java_version",
              System.getProperty("java.version"),
              "os",
              System.getProperty("os.name"),
              "native_opencv_version",
              org.opencv.core.Core.VERSION));
      telemetry = new RunTelemetry(run, options);
      Files.writeString(run.resolve("logs/command-help.txt"), Options.help());
      Properties effective = new Properties();
      try {
        for (java.lang.reflect.Field field : Options.class.getFields()) {
          Object value = field.get(options);
          if (value != null) effective.setProperty(field.getName(), String.valueOf(value));
        }
      } catch (IllegalAccessException e) {
        throw new IllegalStateException(e);
      }
      effective.setProperty("project_root", root.toString());
      effective.setProperty("java.version", System.getProperty("java.version"));
      try (Writer writer = Files.newBufferedWriter(run.resolve("logs/run-settings.properties"))) {
        effective.store(
            writer, "Effective run options; reconstruction adds processed camera information");
      }
    } catch (IOException | RuntimeException e) {
      if (telemetry != null) telemetry.close();
      releaseLock();
      throw e;
    }
  }

  void stage(String name, String status, String detail) throws IOException {
    if (!stages.containsKey(name))
      throw new IllegalArgumentException("Unknown project stage " + name);
    transition(name, status, detail);
    Terminal.stage(name, status, detail);
    event("stage", Json.object("stage", name, "status", status, "detail", detail));
    Json.write(run.resolve("run.json"), runDescription("running", null));
  }

  private void archiveReservedRootFiles() throws IOException {
    List<Object> archived = new ArrayList<>();
    try (java.util.stream.Stream<Path> files = Files.list(root)) {
      for (Path source : (Iterable<Path>) files.filter(Files::isRegularFile)::iterator) {
        String name = source.getFileName().toString();
        String lower = name.toLowerCase(Locale.ROOT);
        if (!(lower.equals("readme.md")
            || lower.equals("project.json")
            || lower.equals("latest-files.csv")
            || isAlias(lower))) continue;
        Path directory = run.resolve("00_inputs/adopted_root_files");
        Files.createDirectories(directory);
        Files.copy(source, directory.resolve(name));
        archived.add(
            Json.object(
                "original_root_filename",
                name,
                "archived_path",
                "adopted_root_files/" + name,
                "sha256",
                sha256(source)));
      }
    }
    if (!archived.isEmpty())
      Json.write(
          run.resolve("00_inputs/adopted_root_files.json"),
          Json.object(
              "reason",
              "Preserve reserved filenames before adopting a preexisting flat output directory",
              "files",
              archived));
  }

  private static boolean isAlias(String name) {
    return name.equals("camera.txt")
        || name.equals("dense.las")
        || name.equals("dense.ply")
        || name.equals("report.txt")
        || name.equals("coordinates.json")
        || name.equals("cameras.csv")
        || name.equals("sparse.csv")
        || name.equals("geometry-quality.csv")
        || name.equals("stereo-quality.csv")
        || name.equals("fusion-quality.txt")
        || name.equals("run-settings.properties")
        || name.startsWith("pair_") && (name.endsWith(".pfm") || name.endsWith(".png"));
  }

  void event(String type, Object detail) throws IOException {
    Files.writeString(
        run.resolve("logs/events.jsonl"),
        Json.string(Json.object("utc", Instant.now().toString(), "type", type, "detail", detail))
            + "\n",
        StandardOpenOption.CREATE,
        StandardOpenOption.APPEND);
  }

  LogCapture capture() throws IOException {
    return new LogCapture(run.resolve("logs/console.log"), attached);
  }

  void finish(boolean success, Throwable error) throws IOException {
    try {
      finishLocked(success, error);
    } finally {
      try {
        if (telemetry != null) telemetry.close();
      } finally {
        for (WorkProgress work : stageWork.values()) work.close();
        stageWork.clear();
        releaseLock();
      }
    }
  }

  private void finishLocked(boolean success, Throwable error) throws IOException {
    IOException ioFailure = null;
    try {
      try {
        data.close();
      } finally {
        cleanupRunFeatures();
      }
      try (WorkProgress work =
          WorkProgress.start("Finalize project exports and compatibility copies", 0, "files")) {
        organize();
        if (success) publishAliases();
        work.done();
      }
    } catch (IOException e) {
      ioFailure = e;
      success = false;
      if (error == null) error = e;
      else error.addSuppressed(e);
    }
    String status = success ? "complete" : "failed";
    if (success) stageFinal("07_results", "complete", "Final exports and report");
    for (String name : STAGES) {
      String state = (String) stages.get(name).get("status");
      if (state.equals("pending"))
        stageFinal(
            name, "skipped", success ? "Not applicable in this mode" : "Not reached after failure");
      else if (!success && state.equals("running"))
        stageFinal(name, "failed", error == null ? "Run failed" : error.toString());
    }
    if (error != null) {
      StringWriter stack = new StringWriter();
      error.printStackTrace(new PrintWriter(stack));
      Files.writeString(run.resolve("logs/error.txt"), stack.toString());
    }
    event(
        success ? "run_complete" : "run_failed",
        Json.object("error", error == null ? null : error.toString()));
    Json.write(run.resolve("run.json"), runDescription(status, error));
    if (telemetry != null) telemetry.persist();
    if (!options.processingReport.equals("none"))
      try (WorkProgress work = WorkProgress.start("Generate final processing report", 0, "files")) {
        ProcessingReport.generate(options, run, run.resolve("07_results"));
        work.done();
        Json.write(
            run.resolve("07_results/report-status.json"),
            Json.object("status", "complete", "format", options.processingReport));
      } catch (IOException | RuntimeException | LinkageError reportFailure) {
        System.err.println(
            "Processing report failed; reconstruction status retained: "
                + reportFailure.getMessage());
        Json.write(
            run.resolve("07_results/report-status.json"),
            Json.object(
                "status",
                "failed",
                "reason",
                reportFailure.toString(),
                "reconstruction_status",
                status));
      }
    if (telemetry != null) telemetry.close();
    writeIndex();
    try (WorkProgress work = WorkProgress.start("Catalog project artifacts", 0, "files")) {
      catalog();
      work.done();
    }
    Json.write(run.resolve("run.json"), runDescription(status, error));
    if (success) marker.setProperty("latest_successful_run", id);
    persistProject(status);
    if (ioFailure != null) throw ioFailure;
  }

  private void releaseLock() throws IOException {
    try {
      if (lock != null && lock.isValid()) lock.release();
    } finally {
      if (lockChannel != null) lockChannel.close();
    }
  }

  private void stageFinal(String name, String state, String detail) {
    transition(name, state, detail);
    Terminal.stage(name, state, detail);
  }

  private void transition(String name, String state, String detail) {
    Map<String, Object> previous = stages.get(name);
    Map<String, Object> entry = new LinkedHashMap<>(previous);
    entry.put("status", state);
    entry.put("updated_utc", Instant.now().toString());
    entry.put("detail", detail);
    if (state.equals("running")) {
      if (!stageWork.containsKey(name)) {
        stageStarted.put(name, System.nanoTime());
        stageWork.put(name, WorkProgress.start("Stage " + name + " — " + detail, 0, "units"));
        entry.put("started_utc", Instant.now().toString());
      } else stageWork.get(name).detail(detail);
    } else {
      WorkProgress work = stageWork.remove(name);
      if (work != null) {
        if (state.equals("complete")) work.done();
        work.close();
        entry.put("elapsed_seconds", (System.nanoTime() - stageStarted.get(name)) / 1e9);
      } else if (entry.get("started_utc") != null && !"skipped".equals(state)) {
        // Stage started by an earlier pipeline step (another process).
        entry.put(
            "elapsed_seconds",
            Duration.between(Instant.parse(String.valueOf(entry.get("started_utc"))), Instant.now())
                    .toMillis()
                / 1e3);
      }
    }
    stages.put(name, entry);
  }

  private Object runDescription(String status, Throwable error) {
    return Json.object(
        "schema",
        FORMAT,
        "run_id",
        id,
        "status",
        status,
        "started_utc",
        started.toString(),
        "updated_utc",
        Instant.now().toString(),
        "mode",
        options.calibrate
            ? "camera_calibration"
            : options.rectified
                ? "rectified_stereo"
                : options.poses != null ? "supplied_poses" : "incremental_sfm",
        "project_root",
        root.toString(),
        "arguments",
        arguments,
        "error",
        error == null ? null : error.toString(),
        "stages",
        stages,
        "artifact_index",
        "artifacts.csv",
        "coordinate_conventions",
        "README.md");
  }

  private void persistProject(String status) throws IOException {
    try (Writer writer = Files.newBufferedWriter(root.resolve("project.properties"))) {
      marker.store(writer, "POSE project history; canonical artifacts are under runs/");
    }
    List<String> runs = new ArrayList<>();
    Path directory = root.resolve("runs");
    if (Files.isDirectory(directory))
      try (java.util.stream.Stream<Path> children = Files.list(directory)) {
        children
            .filter(Files::isDirectory)
            .map(p -> p.getFileName().toString())
            .sorted()
            .forEach(runs::add);
      }
    Json.write(
        root.resolve("project.json"),
        Json.object(
            "schema",
            FORMAT,
            "latest_run",
            id,
            "latest_run_status",
            status,
            "latest_successful_run",
            marker.getProperty("latest_successful_run"),
            "runs",
            runs));
    String latest = "runs/" + id + "/";
    Files.writeString(
        root.resolve("README.md"),
        "# POSE project\n\nLatest run: ["
            + id
            + "]("
            + latest
            + "README.md), status **"
            + status
            + "**.\n\n"
            + "Each invocation creates a new immutable run directory. Earlier runs and their"
            + " artifacts are retained.\n\n"
            + "- [Latest stage status]("
            + latest
            + "run.json)\n- [Latest artifact index]("
            + latest
            + "artifacts.csv)\n- [Console log]("
            + latest
            + "logs/console.log)\n\n"
            + "Root-level files are compatibility copies published when that file is produced."
            + " [latest-files.csv](latest-files.csv) records each managed copy's source run."
            + " Different modes produce different file sets; per-run artifacts are"
            + " authoritative.\n\n"
            + "Latest successful run: "
            + (marker.containsKey("latest_successful_run")
                ? "["
                    + marker.getProperty("latest_successful_run")
                    + "](runs/"
                    + marker.getProperty("latest_successful_run")
                    + "/README.md)"
                : "none")
            + ". See project.json for run history.\n");
  }

  private void organize() throws IOException {
    try (java.util.stream.Stream<Path> files = Files.list(run)) {
      for (Path file : (Iterable<Path>) files.filter(Files::isRegularFile)::iterator) {
        String name = file.getFileName().toString();
        String stage = null;
        if (name.equals("dense.las")
            || name.equals("dense.ply")
            || name.equals("report.txt")
            || name.equals("coordinates.json")) stage = "07_results";
        else if (name.equals("cameras.csv")
            || name.equals("sparse.csv")
            || name.equals("geometry-quality.csv")) stage = "04_geometry";
        else if (name.equals("stereo-quality.csv")) stage = "05_stereo";
        else if (name.equals("fusion-quality.txt")) stage = "06_fusion";
        else if (name.equals("run-settings.properties")) stage = "logs";
        else if (name.equals("camera.txt")) stage = "01_calibration";
        else if (name.startsWith("pair_") && (name.endsWith(".pfm") || name.endsWith(".png")))
          stage = "05_stereo";
        if (stage != null)
          Files.move(file, run.resolve(stage).resolve(name), StandardCopyOption.REPLACE_EXISTING);
      }
    }
  }

  private void publishAliases() throws IOException {
    Map<String, List<String>> aliases = new TreeMap<>();
    Path map = root.resolve("latest-files.csv");
    if (Files.exists(map))
      try (BufferedReader reader = Files.newBufferedReader(map)) {
        reader.readLine();
        String line;
        while ((line = reader.readLine()) != null) {
          List<String> row = PoseIO.csv(line);
          if (row.size() == 4) aliases.put(row.get(0), row);
        }
      }
    for (String stage :
        new String[] {
          "01_calibration", "04_geometry", "05_stereo", "06_fusion", "07_results", "logs"
        })
      try (java.util.stream.Stream<Path> files = Files.list(run.resolve(stage))) {
        for (Path source : (Iterable<Path>) files.filter(Files::isRegularFile)::iterator) {
          String name = source.getFileName().toString();
          if (!(name.equals("camera.txt")
              || name.equals("dense.las")
              || name.equals("dense.ply")
              || name.equals("report.txt")
              || name.equals("coordinates.json")
              || name.equals("cameras.csv")
              || name.equals("sparse.csv")
              || name.equals("geometry-quality.csv")
              || name.equals("stereo-quality.csv")
              || name.equals("fusion-quality.txt")
              || name.equals("run-settings.properties")
              || name.startsWith("pair_") && (name.endsWith(".pfm") || name.endsWith(".png"))))
            continue;
          if (!options.compatibilityCopies
              && (name.equals("dense.las") || name.equals("dense.ply"))) {
            Path oldCopy = root.resolve(name);
            if (Files.isRegularFile(oldCopy)) {
              Path retired = run.resolve("00_inputs/retired_compatibility_copies").normalize();
              if (!retired.toAbsolutePath().startsWith(run.toAbsolutePath().normalize()))
                throw new IOException("Compatibility archive escapes run");
              Files.createDirectories(retired);
              try {
                Files.move(oldCopy, retired.resolve(name));
              } catch (IOException e) {
                System.err.println(
                    "Previous root "
                        + name
                        + " could not be archived; use latest-files.csv for the new cloud: "
                        + e.getMessage());
              }
            }
            aliases.put(
                name,
                Arrays.asList(
                    name,
                    id,
                    "runs/" + id + "/" + run.relativize(source).toString().replace('\\', '/'),
                    artifactHash(source)));
            continue;
          }
          Path temporary = Files.createTempFile(root, ".publish-", ".part");
          try {
            Files.copy(source, temporary, StandardCopyOption.REPLACE_EXISTING);
            CloudWriter.publish(temporary, root.resolve(name));
            aliases.put(
                name,
                Arrays.asList(
                    name,
                    id,
                    "runs/" + id + "/" + run.relativize(source).toString().replace('\\', '/'),
                    artifactHash(source)));
          } finally {
            Files.deleteIfExists(temporary);
          }
        }
      }
    // A run written with --ply off has no PLY: do not keep pointing at an earlier run's.
    if (!Files.isRegularFile(run.resolve("07_results/dense.ply"))) aliases.remove("dense.ply");
    try (BufferedWriter writer = Files.newBufferedWriter(map)) {
      writer.write("filename,run_id,canonical_relative_path,sha256\n");
      for (List<String> row : aliases.values()) {
        for (int i = 0; i < row.size(); i++) {
          if (i > 0) writer.write(',');
          writer.write(ProjectArtifacts.csv(row.get(i)));
        }
        writer.newLine();
      }
    }
  }

  private void catalog() throws IOException {
    List<Path> files = new ArrayList<>();
    try (java.util.stream.Stream<Path> paths = Files.walk(run)) {
      paths
          .filter(Files::isRegularFile)
          .filter(
              p ->
                  !p.getFileName().toString().equals("run.json")
                      && !p.getFileName().toString().equals("artifacts.csv"))
          .sorted()
          .forEach(files::add);
    }
    try (BufferedWriter writer = Files.newBufferedWriter(run.resolve("artifacts.csv"))) {
      writer.write("relative_path,bytes,sha256\n");
      for (Path file : files) {
        String relative = run.relativize(file).toString().replace('\\', '/');
        writer.write(
            ProjectArtifacts.csv(relative)
                + ","
                + Files.size(file)
                + ","
                + artifactHash(file)
                + "\n");
      }
    }
  }

  static String sha256(Path file) throws IOException {
    try {
      MessageDigest digest = MessageDigest.getInstance("SHA-256");
      try (InputStream in = Files.newInputStream(file)) {
        byte[] buffer = new byte[65536];
        int n;
        while ((n = in.read(buffer)) != -1) digest.update(buffer, 0, n);
      }
      StringBuilder text = new StringBuilder();
      for (byte b : digest.digest()) text.append(String.format(Locale.ROOT, "%02x", b & 255));
      return text.toString();
    } catch (NoSuchAlgorithmException e) {
      throw new IllegalStateException(e);
    }
  }

  private String artifactHash(Path file) throws IOException {
    String name = file.getFileName().toString();
    // Only closed cloud outputs are reused. Telemetry/log files can still change at this point.
    if (!name.equals("dense.las") && !name.equals("dense.ply")) return sha256(file);
    Path key = file.toAbsolutePath().normalize();
    var a = Files.readAttributes(key, java.nio.file.attribute.BasicFileAttributes.class);
    var old = artifactHashes.get(key);
    if (old != null
        && old.bytes == a.size()
        && old.modified.equals(a.lastModifiedTime())
        && Objects.equals(old.fileKey, a.fileKey())) return old.hash;
    String hash = sha256(key);
    var after = Files.readAttributes(key, java.nio.file.attribute.BasicFileAttributes.class);
    if (a.size() != after.size()
        || !a.lastModifiedTime().equals(after.lastModifiedTime())
        || !Objects.equals(a.fileKey(), after.fileKey()))
      throw new IOException("Artifact changed during hashing: " + key);
    artifactHashes.put(key, new VerifiedHash(a.size(), a.lastModifiedTime(), a.fileKey(), hash));
    return hash;
  }

  private void cleanupRunFeatures() throws IOException {
    Path folder = options.runFeatureDirectory;
    if (folder == null || !Files.exists(folder)) return;
    Path actual = folder.toRealPath(), allowed = run.toRealPath();
    if (actual.equals(allowed) || !actual.startsWith(allowed))
      throw new IOException("Feature scratch escapes run directory");
    Json.write(
        run.resolve("02_features/reuse.json"),
        Json.object(
            "in_run_feature_hits",
            options.runFeatureHits.get(),
            "scope",
            "Only identical processed pixels/masks, dimensions and detector settings; no persistent"
                + " cache enabled. Changed distortion/resolution invalidates reuse."));
    try (var files = Files.walk(actual)) {
      for (Path p : files.sorted(Comparator.reverseOrder()).toList()) Files.delete(p);
    }
    options.runFeatureDirectory = null;
  }

  private void writeIndex() throws IOException {
    StringBuilder links = new StringBuilder("\n## Quick links\n\n");
    for (String relative :
        new String[] {
          "07_results/dense.las",
          "07_results/dense.ply",
          "07_results/report.txt",
          "07_results/processing-report.html",
          "07_results/processing-report.pdf",
          "07_results/processing-report.json",
          "07_results/dsm.tif",
          "logs/processing.json",
          "07_results/coordinates.json",
          "00_inputs/images.csv",
          "01_calibration/reference_interior.json",
          "01_calibration/processed_interior.csv",
          "01_calibration/calibration.json",
          "01_calibration/self_calibration.json",
          "01_calibration/refinement/result.json",
          "04_geometry/georeference/transform.json",
          "04_geometry/final/exterior.csv",
          "04_geometry/final/orientations.json",
          "04_geometry/final/sparse.ply",
          "04_geometry/final/camera_centers.ply",
          "04_geometry/final/camera_frustums.ply",
          "04_geometry/bundle/iterations.csv",
          "03_matches/pairs.csv",
          "05_stereo/stereo-quality.csv",
          "06_fusion/fusion-quality.txt",
          "logs/run-settings.properties",
          "logs/error.txt"
        })
      if (Files.isRegularFile(run.resolve(relative)))
        links.append("- [").append(relative).append("](").append(relative).append(")\n");
    Files.writeString(run.resolve("README.md"), ProjectArtifacts.guide() + links);
    List<String> rows = new ArrayList<>();
    rows.add("stage,status,detail");
    for (Map.Entry<String, Map<String, Object>> stage : stages.entrySet())
      rows.add(
          stage.getKey()
              + ","
              + stage.getValue().get("status")
              + ","
              + ProjectArtifacts.csv(String.valueOf(stage.getValue().get("detail"))));
    Files.write(run.resolve("stages.csv"), rows);
  }

  static final class LogCapture implements AutoCloseable {
    private final PrintStream originalOut = System.out, originalErr = System.err;
    private final OutputStream file;
    private final PrintStream out, err;

    LogCapture(Path path) throws IOException {
      this(path, false);
    }

    LogCapture(Path path, boolean append) throws IOException {
      file =
          new BufferedOutputStream(
              append
                  ? Files.newOutputStream(path, StandardOpenOption.CREATE, StandardOpenOption.APPEND)
                  : Files.newOutputStream(path));
      out = new PrintStream(new Tee(Terminal.sink(originalOut, false), file), true, StandardCharsets.UTF_8);
      err = new PrintStream(new Tee(Terminal.sink(originalErr, true), file), true, StandardCharsets.UTF_8);
      System.setOut(out);
      System.setErr(err);
    }

    public void close() throws IOException {
      out.flush();
      err.flush();
      System.setOut(originalOut);
      System.setErr(originalErr);
      file.close();
    }
  }

  private static final class Tee extends OutputStream {
    final OutputStream terminal, file;

    Tee(OutputStream terminal, OutputStream file) {
      this.terminal = terminal;
      this.file = file;
    }

    public void write(int value) throws IOException {
      terminal.write(value);
      synchronized (file) {
        file.write(value);
      }
    }

    public void write(byte[] bytes, int offset, int length) throws IOException {
      terminal.write(bytes, offset, length);
      synchronized (file) {
        file.write(bytes, offset, length);
      }
    }

    public void flush() throws IOException {
      terminal.flush();
      synchronized (file) {
        file.flush();
      }
    }
  }
}
