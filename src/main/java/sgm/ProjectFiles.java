package sgm;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.time.*;
import java.util.*;
import java.util.regex.*;

/**
 * Reads a POSE project for the windows: the runs (project.json), each run's steps with status and
 * times (state/state.json, logs/events.jsonl), the report summary and quality flags
 * (07_results/processing-report.json), and builds the CLI arguments of the actions on a run.
 * Read only; no window here.
 */
final class ProjectFiles {
  private ProjectFiles() {}

  static final List<String> STEPS = Arrays.stream(Pipeline.Step.values()).map(s -> s.name).toList();

  record Step(String name, String status, String detail, Instant started, Instant finished) {
    Duration duration() {
      return started == null || finished == null || finished.isBefore(started) ? null : Duration.between(started, finished);
    }

    boolean done() {
      return status.equals("complete") || status.equals("skipped");
    }
  }

  record Run(
      String id, Path dir, Instant started, String status, List<Step> steps, Long points, List<String> flags, List<String[]> summary, List<String> arguments) {
    Step step(String name) {
      for (Step s : steps) if (s.name.equals(name)) return s;
      return null;
    }

    /** Steps that are not complete or skipped, in pipeline order. */
    List<String> remaining() {
      List<String> out = new ArrayList<>();
      for (Step s : steps) if (!s.done()) out.add(s.name);
      return out;
    }

    /** The cleanup step waits for the tie-point limits (--tie-point-cleanup pause). */
    boolean waitingForCleanup() {
      Step c = step("cleanup");
      return c != null && c.status.equals("waiting");
    }

    boolean waitingForGcp() {
      Step g = step("gcp");
      return g != null && g.status.equals("waiting");
    }

    Path results() {
      return dir.resolve("07_results");
    }

    /** The newest processing report of this kind (pdf or html): a re-exported one or the run's. */
    Path report(String extension) {
      Path again = results().resolve("report").resolve("processing-report." + extension), first = results().resolve("processing-report." + extension);
      try {
        if (Files.isRegularFile(again) && (!Files.isRegularFile(first) || Files.getLastModifiedTime(again).compareTo(Files.getLastModifiedTime(first)) > 0)) return again;
      } catch (IOException e) {
        // fall through
      }
      return first;
    }
  }

  static boolean isProject(Path root) {
    return root != null && (Files.isRegularFile(root.resolve("project.json")) || Files.isRegularFile(root.resolve("project.properties")));
  }

  /** Run ids of a project, oldest first (project.json; else the runs folder). */
  static List<String> runIds(Path root) {
    List<String> ids = new ArrayList<>();
    try {
      if (Files.isRegularFile(root.resolve("project.json")))
        for (Object o : list(JsonRead.object(JsonRead.read(root.resolve("project.json"))).get("runs"))) ids.add(String.valueOf(o));
    } catch (IOException | RuntimeException e) {
      ids.clear();
    }
    Path runs = root.resolve("runs");
    if (Files.isDirectory(runs))
      try (var s = Files.list(runs)) {
        for (Path p : s.filter(Files::isDirectory).sorted().toList()) if (!ids.contains(p.getFileName().toString())) ids.add(p.getFileName().toString());
      } catch (IOException e) {
        // keep what project.json had
      }
    ids.removeIf(id -> !Files.isDirectory(runs.resolve(id)));
    return ids;
  }

  /** The newest run id that was not in {@code before} (the run a new command created), or null. */
  static String newRun(List<String> before, Path root) {
    if (root == null || !isProject(root)) return null;
    String found = null;
    for (String id : runIds(root)) if (!before.contains(id) && (found == null || id.compareTo(found) > 0)) found = id;
    return found;
  }

  /** All runs, newest first. */
  static List<Run> runs(Path root) {
    List<Run> out = new ArrayList<>();
    for (String id : runIds(root)) out.add(run(root, id));
    out.sort(Comparator.comparing((Run r) -> r.started == null ? Instant.EPOCH : r.started).reversed());
    return out;
  }

  static Run run(Path root, String id) {
    Path dir = root.resolve("runs").resolve(id);
    Map<String, Object> state = map(dir.resolve("state/state.json")), runJson = map(dir.resolve("run.json"));
    Map<String, Object> report = map(dir.resolve("07_results/processing-report.json"));
    Instant started = instant(runJson.get("started_utc"));
    if (started == null && state.get("started_epoch_millis") instanceof Number n) started = Instant.ofEpochMilli(n.longValue());
    List<Step> steps = steps(state, invocations(dir.resolve("logs/events.jsonl"), started));
    String status = status(steps, String.valueOf(runJson.getOrDefault("status", "")), !state.isEmpty());
    List<String> arguments = new ArrayList<>();
    for (Object o : list(state.isEmpty() ? runJson.get("arguments") : state.get("arguments"))) arguments.add(String.valueOf(o));
    List<String> flags = new ArrayList<>();
    for (Object o : list(report.get("quality_flags"))) flags.add(String.valueOf(o));
    return new Run(id, dir, started, status, steps, points(report), flags, summary(report), arguments);
  }

  /** Overall status: failed, waiting (gcp), running, complete, or "incomplete". */
  static String status(List<Step> steps, String runStatus, boolean pipeline) {
    if (!pipeline) return runStatus.isBlank() ? "unknown" : runStatus;
    for (Step s : steps) if (s.status.equals("failed")) return "failed";
    for (Step s : steps) if (s.status.equals("waiting")) return "waiting";
    for (Step s : steps) if (s.status.equals("running")) return "running";
    if (steps.stream().allMatch(Step::done)) return "complete";
    if (runStatus.equals("failed")) return "failed";
    return "incomplete";
  }

  /** Starts of the commands that worked on the run (run_started, step_started events). */
  static List<Instant> invocations(Path events, Instant runStarted) {
    List<Instant> out = new ArrayList<>();
    if (runStarted != null) out.add(runStarted);
    if (Files.isRegularFile(events))
      try (BufferedReader r = Files.newBufferedReader(events, StandardCharsets.UTF_8)) {
        String line;
        while ((line = r.readLine()) != null) {
          if (!line.contains("\"type\":\"run_started\"") && !line.contains("\"type\":\"step_started\"")) continue;
          Matcher m = UTC.matcher(line);
          if (m.find()) {
            Instant t = instant(m.group(1));
            if (t != null) out.add(t);
          }
        }
      } catch (IOException | UncheckedIOException e) {
        // times stay partial
      }
    Collections.sort(out);
    return out;
  }

  static final Pattern UTC = Pattern.compile("\"utc\":\"([^\"]+)\"");

  /**
   * Steps in pipeline order. A finished step's time is when state.json last marked it; it started
   * at the later of the previous step's end and the start of the command that ran it. A running
   * step's mark is its start.
   */
  static List<Step> steps(Map<String, Object> state, List<Instant> invocations) {
    Map<String, Object> marks = map(state.get("steps"));
    List<Step> out = new ArrayList<>();
    Instant previous = null;
    for (String name : STEPS) {
      Map<String, Object> m = map(marks.get(name));
      // As PipelineState.status: a step added after the run was created (cleanup in older runs)
      // did not exist for it: skipped.
      String status = m.isEmpty() ? (marks.isEmpty() || marks.containsKey(name) ? "pending" : "skipped") : String.valueOf(m.getOrDefault("status", "pending"));
      Instant updated = instant(m.get("updated_utc"));
      String detail = m.get("detail") == null ? "" : String.valueOf(m.get("detail"));
      Instant started = null, finished = null;
      if (status.equals("running")) started = updated;
      else if (updated != null && !status.equals("pending")) {
        finished = updated;
        Instant command = null;
        for (Instant t : invocations) if (!t.isAfter(updated)) command = t;
        started = previous != null && !previous.isAfter(updated) && (command == null || previous.isAfter(command)) ? previous : command;
        previous = updated;
      }
      out.add(new Step(name, status, detail, started, finished));
    }
    return out;
  }

  static Long points(Map<String, Object> report) {
    Map<String, Object> dense = map(report.get("dense"));
    if (map(dense.get("views_per_point")).get("points") instanceof Number n) return n.longValue();
    if (map(dense.get("summary")).get("Exported points") instanceof Number n) return n.longValue();
    return null;
  }

  /** Label / value rows of the report's main numbers. */
  static List<String[]> summary(Map<String, Object> report) {
    List<String[]> out = new ArrayList<>();
    if (report.isEmpty()) return out;
    Map<String, Object> survey = map(report.get("survey")), coordinates = map(report.get("coordinates"));
    add(out, "Images aligned", survey.get("aligned") instanceof Number a && survey.get("images") instanceof Number n ? a.intValue() + " of " + n.intValue() : null);
    add(out, "Tie points", count(survey.get("tie_points")));
    add(out, "Reprojection RMS", number(survey.get("reprojection_rms_pixels"), "%.2f px"));
    add(out, "Ground resolution", number(survey.get("ground_resolution"), "%.3f m/px"));
    add(out, "Flying altitude", number(survey.get("flying_altitude"), "%.1f m"));
    add(out, "Coverage area", survey.get("coverage_area") instanceof Number a ? String.format(Locale.ROOT, "%.2f ha", a.doubleValue() / 1e4) : null);
    add(out, "Point density", number(survey.get("point_density"), "%.0f points/m²"));
    add(out, "Dense points", count(points(report)));
    add(out, "Coordinate system", Boolean.TRUE.equals(coordinates.get("georeferenced")) ? String.valueOf(coordinates.get("horizontal_crs")) : "local (not georeferenced)");
    add(out, "GPS fit RMS", number(coordinates.get("inlier_rms_metres"), "%.2f m"));
    Map<String, Object> lidar = map(map(report.get("lidar")).get("reference"));
    if (!lidar.isEmpty()) {
      add(out, "Lidar reference", PdfReport.lidarLine(lidar));
      add(out, "Heights", coordinates.get("vertical_datum") instanceof String v ? v : null);
    }
    return out;
  }

  private static void add(List<String[]> out, String label, String value) {
    if (value != null) out.add(new String[] {label, value});
  }

  private static String number(Object o, String format) {
    return o instanceof Number n && Double.isFinite(n.doubleValue()) ? String.format(Locale.ROOT, format, n.doubleValue()) : null;
  }

  private static String count(Object o) {
    return o instanceof Number n ? String.format(Locale.ROOT, "%,d", n.longValue()) : null;
  }

  static String duration(Duration d) {
    if (d == null) return "";
    long s = d.toSeconds();
    return s >= 3600 ? String.format(Locale.ROOT, "%d:%02d:%02d", s / 3600, s / 60 % 60, s % 60) : String.format(Locale.ROOT, "%d:%02d", s / 60, s % 60);
  }

  /** Local date and time of an instant for tables. */
  static String local(Instant t) {
    if (t == null) return "";
    LocalDateTime l = LocalDateTime.ofInstant(t, ZoneId.systemDefault());
    return String.format(Locale.ROOT, "%04d-%02d-%02d %02d:%02d:%02d", l.getYear(), l.getMonthValue(), l.getDayOfMonth(), l.getHour(), l.getMinute(), l.getSecond());
  }

  // --- Error details (copied from the window for debugging) ---

  /** The run's recorded error (run.json "error"), or null. */
  static String runError(Path runDir) {
    Object e = map(runDir.resolve("run.json")).get("error");
    return e == null || String.valueOf(e).isBlank() || String.valueOf(e).equals("null") ? null : String.valueOf(e);
  }

  /** The last {@code n} lines of the run's logs/console.log (empty when there is none). */
  static List<String> consoleTail(Path runDir, int n) {
    Path log = runDir.resolve("logs/console.log");
    if (!Files.isRegularFile(log)) return List.of();
    ArrayDeque<String> tail = new ArrayDeque<>();
    try (BufferedReader r = Files.newBufferedReader(log, StandardCharsets.UTF_8)) {
      String line;
      while ((line = r.readLine()) != null) {
        tail.add(line);
        if (tail.size() > n) tail.removeFirst();
      }
    } catch (IOException | UncheckedIOException e) {
      // Not UTF-8 or being written: whatever was read so far.
    }
    return new ArrayList<>(tail);
  }

  /** Lines of the log that report errors or exceptions (the first {@code n}). */
  static List<String> errorLines(Path runDir, int n) {
    List<String> out = new ArrayList<>();
    for (String line : consoleTail(runDir, 5000)) {
      String l = line.toLowerCase(Locale.ROOT);
      if ((l.startsWith("photogrammetry") && l.contains(":")) || l.contains("exception") || l.contains("error:") || l.startsWith("\tat ") || l.startsWith("caused by"))
        if (out.size() < n) out.add(line);
    }
    return out;
  }

  /** Plain text about one step (status, times, the full message; for a failure the log's context). */
  static String stepDetails(Run r, Step s) {
    StringBuilder t = new StringBuilder();
    t.append("Run: ").append(r.id()).append(" (").append(r.status()).append(")\n");
    t.append("Step: ").append(s.name()).append(" - ").append(s.status()).append('\n');
    if (s.started() != null) t.append("Started: ").append(local(s.started())).append('\n');
    if (s.finished() != null) t.append("Finished: ").append(local(s.finished())).append(s.duration() == null ? "" : " (" + duration(s.duration()) + ")").append('\n');
    t.append("\nMessage:\n").append(s.detail().isBlank() ? "-" : s.detail()).append('\n');
    if (s.status().equals("failed") || s.status().equals("running")) appendFailure(t, r);
    return t.toString();
  }

  /** Plain text about a run for a bug report: status, failed step, error, arguments, log tail. */
  static String runDetails(Run r) {
    StringBuilder t = new StringBuilder();
    t.append("Run: ").append(r.id()).append('\n').append("Folder: ").append(r.dir()).append('\n').append("Status: ").append(r.status()).append('\n');
    if (r.started() != null) t.append("Started: ").append(local(r.started())).append('\n');
    t.append("\nSteps:\n");
    for (Step s : r.steps())
      t.append(String.format(Locale.ROOT, "  %-13s %-9s %8s  %s%n", s.name(), s.status(), duration(s.duration()), s.status().equals("failed") || s.status().equals("waiting") ? s.detail() : ""));
    if (!r.flags().isEmpty()) {
      t.append("\nQuality flags:\n");
      for (String f : r.flags()) t.append("  ").append(f).append('\n');
    }
    t.append("\nArguments: ").append(String.join(" ", r.arguments())).append('\n');
    if (!r.status().equals("complete")) appendFailure(t, r);
    return t.toString();
  }

  private static void appendFailure(StringBuilder t, Run r) {
    String error = runError(r.dir());
    if (error != null) t.append("\nRun error:\n").append(error).append('\n');
    List<String> errors = errorLines(r.dir(), 40);
    if (!errors.isEmpty()) t.append("\nErrors in logs/console.log:\n").append(String.join("\n", errors)).append('\n');
    List<String> tail = consoleTail(r.dir(), 40);
    if (!tail.isEmpty()) t.append("\nEnd of logs/console.log:\n").append(String.join("\n", tail)).append('\n');
  }

  // --- Actions on a run (CLI arguments; the window starts them with PoseProcess) ---

  /** Continue a run with these steps (pipeline order is checked by the CLI). */
  static List<String> continueSteps(Path root, String id, List<String> steps, List<String> extra) {
    List<String> out = new ArrayList<>(steps);
    out.addAll(List.of("-o", root.toString(), "--run", id));
    out.addAll(extra);
    return out;
  }

  /**
   * Steps a new run can redo from this run: every step after inputs whose earlier steps are all
   * complete or skipped here (the branch copies the state saved after the step before).
   */
  static List<String> branchable(Run r) {
    List<String> out = new ArrayList<>();
    // As Pipeline.branch: an optional step without a saved state (skipped in a run made before
    // the step existed, or pending and not asked for) passes the state before it on.
    boolean base = false;
    for (int i = 1; i < r.steps().size(); i++) {
      Step prior = r.steps().get(i - 1);
      Pipeline.Step kind = Pipeline.Step.of(prior.name());
      boolean optional = kind != null && Pipeline.optional(kind);
      if (!prior.done() && !(optional && prior.status().equals("pending"))) break;
      if (Files.isDirectory(r.dir().resolve("state/after-" + prior.name()))) base = true;
      else if (!optional || prior.status().equals("complete")) base = false;
      if (base) out.add(STEPS.get(i));
    }
    return out;
  }

  /** A new run copied from this one just before {@code from}, redoing from there to finish. */
  static List<String> branch(Path root, String id, String from, List<String> extra) {
    return branch(root, id, from, "finish", extra);
  }

  /**
   * A new run copied from this one just before {@code from} (nothing earlier is run again),
   * redoing the steps {@code from} .. {@code until}; the steps after stay pending in the new run.
   */
  static List<String> branch(Path root, String id, String from, String until, List<String> extra) {
    int a = STEPS.indexOf(from), b = STEPS.indexOf(until);
    if (a < 1 || b < a) throw new IllegalArgumentException("Redo from a step after inputs, up to the same or a later step");
    List<String> steps = STEPS.subList(a, b + 1);
    List<String> out = new ArrayList<>(steps);
    out.addAll(List.of("-o", root.toString(), "--run", id, "--branch"));
    out.addAll(extra);
    return out;
  }

  /** The GCP marker editor on this run; the window continues the run itself afterwards. */
  static List<String> editGcps(Path root, String id) {
    return List.of("gcp-edit", "-o", root.toString(), "--run", id, "--no-continue");
  }

  /** Regenerate the report of a run (07_results/report). */
  static List<String> exportReport(Path runDir) {
    return List.of("--export-report", runDir.toString());
  }

  /** Make the orthomosaic of a run without reprocessing (07_results). */
  static List<String> exportOrthomosaic(Path runDir) {
    return List.of("--export-orthomosaic", runDir.toString());
  }

  // --- Console output of a running command ---

  static final Pattern RUN_LINE = Pattern.compile("^Run: (\\S+)\\s*$"), PROJECT_LINE = Pattern.compile("^Project: (.+?)\\s*$");
  static final Pattern STAGE = Pattern.compile("^\\[(\\d+)/(\\d+)\\]\\s+(.+?)(?:\\s{2,}|:|$)"), PERCENT = Pattern.compile("\\s(\\d{1,3})%\\s");

  /** The run id announced by a command ("Run: ID"), or null. */
  static String runId(String line) {
    Matcher m = RUN_LINE.matcher(line);
    return m.find() ? m.group(1) : null;
  }

  static String projectRoot(String line) {
    Matcher m = PROJECT_LINE.matcher(line);
    return m.find() ? m.group(1) : null;
  }

  /** Stage progress in a terminal line: stage index, stage count, name and percent (-1 if none). */
  record Progress(int stage, int stages, String name, int percent) {}

  static Progress progress(String line) {
    Matcher m = STAGE.matcher(line);
    if (!m.find()) return null;
    Matcher p = PERCENT.matcher(line + " ");
    int percent = -1;
    if (p.find()) percent = Math.min(100, Integer.parseInt(p.group(1)));
    if (line.substring(m.end()).trim().startsWith("done") || line.contains(" done ")) percent = 100;
    return new Progress(Integer.parseInt(m.group(1)), Integer.parseInt(m.group(2)), m.group(3).trim(), percent);
  }

  // --- JSON helpers ---

  static Map<String, Object> map(Path file) {
    try {
      return Files.isRegularFile(file) ? JsonRead.object(JsonRead.read(file)) : Map.of();
    } catch (IOException | RuntimeException e) {
      return Map.of(); // being rewritten or partial: read again next time
    }
  }

  @SuppressWarnings("unchecked")
  static Map<String, Object> map(Object o) {
    return o instanceof Map<?, ?> m ? (Map<String, Object>) m : Map.of();
  }

  static List<Object> list(Object o) {
    return o instanceof List<?> l ? new ArrayList<>(l) : List.of();
  }

  static Instant instant(Object o) {
    if (o == null) return null;
    try {
      return Instant.parse(String.valueOf(o));
    } catch (RuntimeException e) {
      return null;
    }
  }
}
