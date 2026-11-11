package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;

/**
 * Command line. Without step names the whole pipeline runs in one process, as before, saving the
 * project state after every step. With step names only those steps run:
 *
 * <pre>
 * photogrammetry inputs -i IMAGES -o PROJECT [options]   start a new run (first step)
 * photogrammetry calibrate match align -o PROJECT        continue the latest run
 * photogrammetry dense -o PROJECT --run ID [options]     continue a given run
 * photogrammetry dense -o PROJECT --branch [options]     copy the run, redo from this step
 * photogrammetry status -o PROJECT [--run ID]            show the steps of a run
 * </pre>
 *
 * Later steps reuse the arguments saved by the first step; options given to a later step are
 * appended and override them.
 */
public final class Main {
  private Main() {}

  public static void main(String[] args) {
    MemoryLauncher.main(args);
  }

  /** Leading step names, the rest of the arguments, and the pipeline-only flags. */
  record Command(
      List<Pipeline.Step> steps, boolean status, String[] rest, String runId, boolean branch, boolean editor) {}

  static Command command(String[] args) {
    List<Pipeline.Step> steps = new ArrayList<>();
    int first = 0;
    boolean status = false, editor = false;
    if (args.length > 0 && args[0].equals("status")) {
      status = true;
      first = 1;
    }
    if (args.length > 0 && args[0].equals("gcp-edit")) {
      editor = true;
      status = true;
      first = 1;
    }
    while (!status && first < args.length && Pipeline.Step.of(args[first]) != null)
      steps.add(Pipeline.Step.of(args[first++]));
    List<String> rest = new ArrayList<>();
    String runId = null;
    boolean branch = false;
    for (int i = first; i < args.length; i++) {
      if (args[i].equals("--run") && i + 1 < args.length) runId = args[++i];
      else if (args[i].equals("--branch")) branch = true;
      else rest.add(args[i]);
    }
    return new Command(steps, status, rest.toArray(String[]::new), runId, branch, editor);
  }

  /**
   * Project root from -o/--output, without full option parsing. The last one wins, as in Options:
   * a continued step's saved arguments come first, so a copied or moved project resolves to the
   * -o of this command, not the one saved when the run started.
   */
  static Path projectRoot(String[] args) {
    Path root = null;
    for (int i = 0; i + 1 < args.length; i++)
      if (args[i].equals("-o") || args[i].equals("--output")) root = Path.of(args[i + 1]).toAbsolutePath().normalize();
    if (root == null) throw new IllegalArgumentException("Give the project directory with -o PROJECT");
    return root;
  }

  static String latestRun(Path root) throws IOException {
    Properties marker = new Properties();
    Path file = root.resolve("project.properties");
    if (!Files.isRegularFile(file)) throw new IllegalArgumentException("Not a POSE project: " + root);
    try (Reader r = Files.newBufferedReader(file)) {
      marker.load(r);
    }
    String id = marker.getProperty("latest_run");
    if (id == null) throw new IllegalArgumentException("Project has no runs yet: " + root);
    return id;
  }

  /** Saved first-step arguments followed by this command's, for a step that continues a run. */
  static String[] continued(Command c) throws IOException {
    Path root = projectRoot(c.rest());
    String id = c.runId() == null ? latestRun(root) : c.runId();
    Path run = root.resolve("runs").resolve(id);
    if (!PipelineState.exists(run))
      throw new IllegalArgumentException("Run " + id + " has no saved pipeline state; start with the inputs step");
    List<String> all = new ArrayList<>(PipelineState.load(run).arguments());
    all.addAll(Arrays.asList(c.rest()));
    return all.toArray(String[]::new);
  }

  /** Arguments the heap launcher should see (saved ones for a continued step). */
  static String[] launchArguments(String[] args) {
    try {
      Command c = command(args);
      if (c.status() || c.steps().isEmpty() || c.steps().get(0) == Pipeline.Step.INPUTS) return c.rest();
      return continued(c);
    } catch (IOException | RuntimeException e) {
      return args;
    }
  }

  public static int run(String[] args) {
    Command c;
    try {
      c = command(args);
      if (c.editor()) return GcpEditor.serve(c);
      if (c.status()) return status(c);
    } catch (IllegalArgumentException | IOException e) {
      System.err.println("photogrammetry: " + e.getMessage());
      return 2;
    }
    // Opt-in block processing (default off: this whole branch is skipped).
    if (!BlockProcessing.mode(c.rest()).equals("off")) {
      if (!c.steps().isEmpty()) {
        System.err.println("photogrammetry: --block-processing runs the whole pipeline; give it on the"
            + " whole-run command (-i IMAGES -o PROJECT) without step names");
        return 2;
      }
      try {
        Integer code = BlockProcessing.run(c.rest());
        if (code != null) return code;
      } catch (IllegalArgumentException | IOException e) {
        System.err.println("photogrammetry: block processing: " + e.getMessage());
        return 1;
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
        return 1;
      }
    }
    boolean chain = c.steps().isEmpty();
    boolean fresh = chain || c.steps().get(0) == Pipeline.Step.INPUTS || c.branch();
    Project project = null;
    Project.LogCapture capture = null;
    Throwable failure = null;
    int status = 1;
    boolean finishing = chain || c.steps().contains(Pipeline.Step.FINISH);
    String next = null;
    try {
      String[] effective = fresh && !c.branch() ? c.rest() : continued(c);
      Options options = Options.parse(effective);
      if (options.help) {
        System.out.print(Options.help(options.helpTopic));
        return 0;
      }
      if (options.listGpus) {
        GpuSgm.list();
        return 0;
      }
      if (options.exportConfidence != null) {
        PointRecords.export(options.exportConfidence, options.outputCompression, options.cores);
        return 0;
      }
      MemorySafety.validateHostHeap(options);
      nu.pattern.OpenCV.loadLocally();
      if (options.exportReport != null) {
        ProcessingReport.export(options);
        return 0;
      }
      if (options.analyzeRun != null) {
        SurfaceDiagnostics.analyze(options);
        return 0;
      }
      if (options.exportOrientations != null) {
        OrientationIO.exportRun(options);
        return 0;
      }
      Core.setNumThreads(options.cores);
      Core.setRNGSeed(1);
      Path root = projectRoot(effective);
      String source = fresh ? null : c.runId() == null ? latestRun(root) : c.runId();
      if (c.branch()) source = c.runId() == null ? latestRun(root) : c.runId();
      // Refuse an impossible step list before the run is opened (it must stay as it is).
      if (!chain && !c.branch() && source != null)
        Pipeline.check(PipelineState.load(root.resolve("runs").resolve(source)), c.steps(), options);
      project = fresh ? Project.open(options, effective) : Project.attach(options, effective, source);
      Terminal.configure(options.verbose, System.out);
      capture = project.capture();
      System.out.println("Project: " + project.root + "\nRun: " + project.id);
      MemoryLauncher.report(options);
      if (!options.calibrate) System.out.println(options.filteringSummary());
      if (options.calibrate) {
        Calibration.calibrate(options, project);
        finishing = true;
      } else {
        PipelineState state;
        if (c.branch()) state = Pipeline.branch(project, root.resolve("runs").resolve(source), c.steps().get(0), effective);
        else if (fresh) state = PipelineState.create(project.run, effective);
        else {
          state = PipelineState.load(project.run);
          // Options given to this step also hold for the steps after it.
          if (c.rest().length > 2) {
            state.put("arguments", Arrays.asList(effective));
            state.write();
          }
        }
        Pipeline pipeline = new Pipeline(options, project, state);
        List<Pipeline.Step> steps = chain ? Arrays.asList(Pipeline.Step.values()) : c.steps();
        for (Pipeline.Step step : steps) {
          pipeline.run(step);
          if (state.status(step).equals("waiting")) {
            // The gcp step waits for markers: keep the run open instead of finishing it.
            finishing = false;
            break;
          }
        }
        next = Pipeline.next(state, options);
        if (!finishing)
          System.out.println(
              next == null
                  ? "All steps done; run the finish step to write the report."
                  : "Next step: photogrammetry " + next + " -o " + root);
      }
      System.out.println(
          options.estimateMemory
              ? "Memory planning completed; no reconstruction performed."
              : finishing ? "Reconstruction stages completed." : "Pipeline steps completed.");
      status = 0;
    } catch (IllegalArgumentException | IOException e) {
      System.err.println("photogrammetry: " + e.getMessage());
      failure = e;
      status = 2;
    } catch (Exception | LinkageError e) {
      System.err.println("photogrammetry failed: " + e.getMessage());
      e.printStackTrace(System.err);
      failure = e;
      status = 1;
    } finally {
      GpuSgm.closeSession();
      try {
        if (capture != null) capture.close();
      } catch (IOException e) {
        System.err.println("Cannot close project log: " + e);
        if (failure == null) failure = e;
        status = 2;
      }
      // Report and catalog work after the log closes: keep the terminal filtered until the end.
      PrintStream terminal = System.out;
      if (!Terminal.verbose())
        System.setOut(new PrintStream(Terminal.sink(terminal, false), true, java.nio.charset.StandardCharsets.UTF_8));
      if (project != null)
        try {
          if (finishing || status != 0 && chain) project.finish(status == 0, failure);
          else project.suspend(status == 0, failure, next);
        } catch (IOException e) {
          System.err.println("Cannot finalize project artifacts: " + e);
          status = 2;
        }
      System.setOut(terminal);
      Terminal.stop();
      if (project != null)
        System.out.println(
            "Project run "
                + (status != 0 ? "failed" : finishing ? "complete" : "saved after step")
                + ": "
                + project.run);
    }
    return status;
  }

  private static int status(Command c) throws IOException {
    Path root = projectRoot(c.rest());
    if (BlockProcessing.isBlockProject(root) && !Files.isRegularFile(root.resolve("project.properties")))
      return BlockProcessing.status(root);
    String id = c.runId() == null ? latestRun(root) : c.runId();
    Path run = root.resolve("runs").resolve(id);
    if (!PipelineState.exists(run)) {
      System.out.println("Run " + id + ": no pipeline state (finished before steps existed, or not started).");
      return 0;
    }
    PipelineState state = PipelineState.load(run);
    System.out.println("Run " + id + " (" + run + ")");
    Options options = null;
    try {
      options = Options.parse(continued(c));
    } catch (RuntimeException unreadable) {
      // Without the saved options optional steps are listed as they are.
    }
    for (Pipeline.Step s : Pipeline.Step.values()) {
      var entry = JsonRead.object(state.steps().get(s.name));
      boolean optional = "pending".equals(state.status(s)) && Pipeline.notRequested(s, options);
      System.out.printf(
          Locale.ROOT,
          "  %-13s %-9s %s%n",
          s.name,
          entry.get("status"),
          optional ? "(not requested; skipped when a later step runs)" : entry.get("updated_utc") == null ? "" : entry.get("updated_utc"));
    }
    String next = Pipeline.next(state, options);
    System.out.println(next == null ? "All steps done." : "Next step: " + next);
    return 0;
  }

  static List<Path> images(Options o) throws IOException {
    Set<String> extensions =
        new HashSet<>(Arrays.asList(o.format.toLowerCase(Locale.ROOT).split(",")));
    List<Path> result = new ArrayList<>();
    try (java.util.stream.Stream<Path> files = Files.list(o.input)) {
      files
          .filter(Files::isRegularFile)
          .forEach(
              p -> {
                String n = p.getFileName().toString().toLowerCase(Locale.ROOT);
                int dot = n.lastIndexOf('.');
                if (dot >= 0 && extensions.contains(n.substring(dot + 1))) result.add(p);
              });
    }
    result.sort(Comparator.comparing(p -> p.getFileName().toString()));
    return result;
  }
}
