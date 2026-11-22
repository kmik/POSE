package sgm;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.*;
import java.util.concurrent.TimeUnit;
import java.util.regex.*;

/**
 * Parameter sweeps for quality/speed studies. Kept apart from the pipeline.
 *
 * <p>{@code plan} writes a trial table from {@code sweep/options-inventory.tsv}: two baseline runs
 * (the second checks that results repeat) and one trial per option level, each changing one option
 * of the baseline (options that only act together with another get that one too).
 *
 * <p>{@code run} works through a trial table. Per dataset it aligns once (the steps before
 * {@code dense}) and keeps that run as the base; every trial is {@code dense --run BASE --branch}
 * with the trial's options, so all trials share one orientation. The cloud is scored with {@link
 * CloudScore} in the dataset's frame (fitted on the first trial scored, normally the baseline, and
 * then frozen), one CSV row is appended, and the trial run is deleted. Finished trials are skipped,
 * so an interrupted sweep continues where it stopped.
 */
final class ParamSweep {
  private ParamSweep() {}

  static final String HELP =
      """
      sgm.ParamSweep plan --inventory sweep/options-inventory.tsv --datasets a,b --out plan.tsv
                          [--classes dense,fusion] [--method mvsgm] [--only FLAG,FLAG]
      sgm.ParamSweep run --config sweep.properties --plan plan.tsv --results results.csv
                          [--keep] [--min-free-gb 20] [--timeout-factor 5] [--trials PREFIX,...]

      Config (properties; relative paths are relative to the config file, which is also the
      working folder of the photogrammetry runs):
        work = output/sweep                 projects go to work/DATASET
        heap = 12g                          Java heap of the photogrammetry runs
        datasets = city,forest9
        city.images = ../example_data/forest/city
        city.options = --cores 12 --memory-mb 16000 --quality medium ...   baseline options
        city.reference = ../example_data/forest/city/metashape.las         optional
        city.stable = planar                planar | ground | all (CloudScore --stable)
        city.region = x0,y0,x1,y1           optional scoring rectangle
        city.align-steps = inputs calibrate match align georeference       (default)
      """;

  // ---------------------------------------------------------------------------------------------
  // Plan

  record Trial(String id, String dataset, List<String> options) {
    String line() {
      return id + "\t" + dataset + "\t" + String.join(" ", options);
    }

    static Trial parse(String line) {
      String[] f = line.split("\t", -1);
      List<String> o = f.length > 2 && !f[2].isBlank() ? List.of(f[2].strip().split("\\s+")) : List.of();
      return new Trial(f[0], f[1], o);
    }
  }

  /** Inventory row (sweep/options-inventory.tsv). */
  record Option(String flag, String cls, String step, Set<String> methods, String kind, String baseline, List<String> levels, String requires) {}

  static List<Option> inventory(Path p) throws IOException {
    List<Option> out = new ArrayList<>();
    List<String> lines = Files.readAllLines(p, StandardCharsets.UTF_8);
    String[] head = lines.get(0).split("\t", -1);
    Map<String, Integer> col = new HashMap<>();
    for (int i = 0; i < head.length; i++) col.put(head[i], i);
    for (String line : lines.subList(1, lines.size())) {
      if (line.isBlank()) continue;
      String[] f = line.split("\t", -1);
      String levels = f[col.get("levels")];
      out.add(new Option(f[col.get("flag")], f[col.get("class")], f[col.get("step")], Set.of(f[col.get("methods")].split(",")),
          f[col.get("kind")], f[col.get("baseline")], levels.equals("-") || levels.isBlank() ? List.of() : List.of(levels.split(",")),
          f[col.get("requires")]));
    }
    return out;
  }

  /** Baselines + one trial per option level, for each dataset. */
  static List<Trial> plan(List<Option> inventory, List<String> datasets, Set<String> classes, String method, Set<String> only) {
    List<Trial> out = new ArrayList<>();
    for (String d : datasets) {
      out.add(new Trial("baseline", d, List.of()));
      out.add(new Trial("baseline-repeat", d, List.of()));
    }
    for (Option o : inventory) {
      if (!classes.contains(o.cls()) || o.levels().isEmpty()) continue;
      if (!o.methods().contains("all") && !o.methods().contains(method)) continue;
      if (!only.isEmpty() && !only.contains(o.flag())) continue;
      for (String level : o.levels()) {
        List<String> opts = new ArrayList<>();
        if (!o.requires().isBlank()) opts.addAll(List.of(o.requires().split("\\s+")));
        if (o.kind().equals("FLAG")) {
          if (level.equals("on")) opts.add(o.flag());
          else if (level.equals("off") && o.flag().equals("--quick-calibration")) opts.add("--no-quick-calibration");
          else continue;
        } else {
          opts.add(o.flag());
          opts.add(level);
        }
        String id = o.flag().replaceFirst("^-+", "") + "=" + level;
        for (String d : datasets) out.add(new Trial(id, d, List.copyOf(opts)));
      }
    }
    return out;
  }

  // ---------------------------------------------------------------------------------------------
  // Run

  /** Launches photogrammetry; replaceable in tests. */
  interface Runner {
    /** Runs photogrammetry with {@code args} in {@code dir}; output to {@code log}. Exit code, or -1 on timeout. */
    int run(List<String> args, Path dir, Path log, long timeoutSeconds) throws IOException, InterruptedException;
  }

  /** A separate JVM per call (same Java and class path as this one). */
  static Runner processRunner(String heap) {
    return (args, dir, log, timeout) -> {
      List<String> cmd = new ArrayList<>();
      cmd.add(Path.of(System.getProperty("java.home"), "bin", "java").toString());
      cmd.add("--enable-native-access=ALL-UNNAMED");
      cmd.add("-Xmx" + heap);
      cmd.add("-cp");
      cmd.add(absoluteClassPath());
      cmd.add("photogrammetry");
      cmd.addAll(args);
      Process p = new ProcessBuilder(cmd).directory(dir.toFile()).redirectErrorStream(true).redirectOutput(log.toFile()).start();
      if (p.waitFor(timeout, TimeUnit.SECONDS)) return p.exitValue();
      p.descendants().forEach(ProcessHandle::destroyForcibly);
      p.destroyForcibly();
      p.waitFor(30, TimeUnit.SECONDS);
      return -1;
    };
  }

  /** This JVM's class path with relative entries made absolute (the runs start in another folder). */
  static String absoluteClassPath() {
    List<String> out = new ArrayList<>();
    for (String e : System.getProperty("java.class.path").split(File.pathSeparator)) {
      if (e.isEmpty()) continue;
      boolean wild = e.endsWith("*");
      String base = wild ? e.substring(0, e.length() - 1) : e;
      String abs = Path.of(base.isEmpty() ? "." : base).toAbsolutePath().normalize().toString();
      out.add(wild ? abs + File.separator + "*" : abs);
    }
    return String.join(File.pathSeparator, out);
  }

  static final class Config {
    Path dir, work;
    String heap = "12g";
    List<String> datasets = new ArrayList<>();
    Map<String, Properties> ds = new LinkedHashMap<>();
    Properties raw = new Properties();

    static Config load(Path file) throws IOException {
      Config c = new Config();
      try (Reader r = Files.newBufferedReader(file, StandardCharsets.UTF_8)) {
        c.raw.load(r);
      }
      c.dir = file.toAbsolutePath().getParent();
      c.work = c.dir.resolve(c.raw.getProperty("work", "output/sweep")).normalize();
      c.heap = c.raw.getProperty("heap", "12g");
      for (String d : c.raw.getProperty("datasets", "").split(",")) if (!d.isBlank()) c.datasets.add(d.strip());
      if (c.datasets.isEmpty()) throw new IllegalArgumentException("Config names no datasets");
      for (String d : c.datasets) {
        if (c.raw.getProperty(d + ".images") == null) throw new IllegalArgumentException("Config misses " + d + ".images");
        Properties p = new Properties();
        for (String k : c.raw.stringPropertyNames()) if (k.startsWith(d + ".")) p.setProperty(k.substring(d.length() + 1), c.raw.getProperty(k));
        c.ds.put(d, p);
      }
      return c;
    }

    List<String> options(String d) {
      String o = ds.get(d).getProperty("options", "").strip();
      return o.isEmpty() ? List.of() : List.of(o.split("\\s+"));
    }

    /** The driver's own files (base marker, frame, logs): beside the project, which must stay a POSE project. */
    Path meta(String d) {
      return work.resolve(d + "-sweep");
    }

    Path project(String d) {
      return work.resolve(d);
    }
  }

  /** Fixed result columns (CloudScore keys flattened with '.'). */
  static final List<String> METRICS =
      List.of(
          "points", "artefacts.area_m2", "artefacts.double_sheet_cells_fraction", "artefacts.far_below_fraction",
          "artefacts.far_above_fraction", "shape.outside_reference_fraction", "shape.all.median_m", "shape.all.p90_m",
          "shape.all.p95_m", "shape.all.far_over_1m", "shape.stable.median_m", "shape.stable.p90_m", "shape.stable.far_over_1m",
          "shape.stable.bias_m", "deformation.warp_std_m", "deformation.tilt_mm_per_100m", "deformation.tile_offset_std_m",
          "flat_noise.median_rms_m", "flat_noise.p90_rms_m", "completeness.covered_fraction",
          "completeness.points_outside_area_fraction", "position.fit_residual_rms_m");

  static final List<String> HEAD =
      List.of("dataset", "trial", "status", "options", "wall_s", "stereo_s", "fusion_s", "results_s", "peak_rss_gb", "peak_vram_gb");

  static void flatten(CloudScore.Result r, String prefix, Map<String, Double> out) {
    for (var e : r.entrySet()) {
      String k = prefix.isEmpty() ? e.getKey() : prefix + "." + e.getKey();
      if (e.getValue() instanceof CloudScore.Result sub) flatten(sub, k, out);
      else if (e.getValue() instanceof Double d) out.put(k, d);
    }
  }

  static final Pattern STAGE = Pattern.compile("\"task\":\"Stage (0\\d_[a-z]+)[^\"]*\",\"started_offset_seconds\":[-0-9.eE]+,\"wall_seconds\":([0-9.eE+-]+)");
  static final Pattern RSS = Pattern.compile("\"peak_sampled_process_resident_bytes\":([0-9.eE+]+)");
  static final Pattern VRAM = Pattern.compile("\"peak_device_vram_used_bytes\":([0-9.eE+]+)");

  /** Stage wall times (seconds) and peaks (bytes) from a run's logs/processing.json. */
  static Map<String, Double> telemetry(Path processingJson) throws IOException {
    Map<String, Double> out = new HashMap<>();
    if (!Files.isRegularFile(processingJson)) return out;
    String s = Files.readString(processingJson, StandardCharsets.UTF_8);
    Matcher m = STAGE.matcher(s);
    while (m.find()) out.merge(m.group(1), Double.parseDouble(m.group(2)), Double::sum);
    for (Pattern p : List.of(RSS, VRAM)) {
      Matcher q = p.matcher(s);
      double max = Double.NaN;
      while (q.find()) max = Double.isNaN(max) ? Double.parseDouble(q.group(1)) : Math.max(max, Double.parseDouble(q.group(1)));
      out.put(p == RSS ? "rss" : "vram", max);
    }
    return out;
  }

  static String csv(String v) {
    return v.contains(",") || v.contains("\"") ? "\"" + v.replace("\"", "\"\"") + "\"" : v;
  }

  static String num(double d) {
    return Double.isNaN(d) ? "" : String.format(Locale.ROOT, "%.6g", d);
  }

  /** Trials already finished (status ok) in the results file: "dataset\ttrial". */
  static Set<String> finished(Path results) throws IOException {
    Set<String> done = new HashSet<>();
    if (!Files.isRegularFile(results)) return done;
    for (String line : Files.readAllLines(results, StandardCharsets.UTF_8).stream().skip(1).toList()) {
      String[] f = line.split(",", 4);
      if (f.length >= 3 && f[2].equals("ok")) done.add(f[0] + "\t" + f[1]);
    }
    return done;
  }

  static List<Path> runs(Path project) throws IOException {
    Path runs = project.resolve("runs");
    if (!Files.isDirectory(runs)) return List.of();
    try (var s = Files.list(runs)) {
      return s.filter(Files::isDirectory).sorted().toList();
    }
  }

  static Path newRun(List<Path> before, List<Path> after) {
    for (Path p : after) if (!before.contains(p)) return p;
    return null;
  }

  static void delete(Path p) throws IOException {
    if (!Files.exists(p)) return;
    try (var s = Files.walk(p)) {
      for (Path q : s.sorted(Comparator.reverseOrder()).toList()) Files.deleteIfExists(q);
    }
  }

  static Path find(Path root, String name) throws IOException {
    try (var s = Files.walk(root)) {
      return s.filter(q -> q.getFileName().toString().equals(name)).findFirst().orElse(null);
    }
  }

  /** Aligns a dataset once (steps before dense) and returns the base run id (kept in work/DATASET-sweep/base.txt). */
  static String base(Config c, String d, Runner runner, PrintStream log) throws IOException, InterruptedException {
    Path project = c.project(d), meta = c.meta(d), marker = meta.resolve("base.txt");
    String options = String.join(" ", c.options(d));
    if (Files.isRegularFile(marker)) {
      List<String> m = Files.readAllLines(marker, StandardCharsets.UTF_8);
      if (m.size() >= 2 && m.get(1).equals(options) && Files.isDirectory(project.resolve("runs").resolve(m.get(0)))) return m.get(0);
      throw new IllegalStateException(d + ": the base run was made with other options or is gone; use a new work folder (" + marker + ")");
    }
    Files.createDirectories(meta);
    List<String> args = new ArrayList<>(List.of(c.ds.get(d).getProperty("align-steps", "inputs calibrate match align georeference").split("\\s+")));
    args.addAll(List.of("-i", c.dir.resolve(c.ds.get(d).getProperty("images")).toString(), "-o", project.toString()));
    args.addAll(c.options(d));
    List<Path> before = runs(project);
    log.println(d + ": aligning (" + String.join(" ", args) + ")");
    int code = runner.run(args, c.dir, meta.resolve("base.log"), 24 * 3600);
    Path run = newRun(before, runs(project));
    if (code != 0 || run == null) throw new IllegalStateException(d + ": alignment failed (exit " + code + "), see " + meta.resolve("base.log"));
    String id = run.getFileName().toString();
    Files.writeString(marker, id + "\n" + options + "\n", StandardCharsets.UTF_8);
    return id;
  }

  static final class RunSettings {
    Path config, plan, results;
    boolean keep;
    double minFreeGb = 20, timeoutFactor = 5;
    List<String> trials = List.of();
  }

  /**
   * Trials whose options photogrammetry would refuse (baseline + trial options through {@link
   * Options#parse}): "dataset trial: reason". A sweep with such trials does not start.
   */
  static List<String> invalid(Config c, List<Trial> plan) {
    List<String> out = new ArrayList<>();
    for (Trial t : plan) {
      if (!c.ds.containsKey(t.dataset())) {
        out.add(t.dataset() + " " + t.id() + ": dataset not in the config");
        continue;
      }
      List<String> args = new ArrayList<>(List.of("-i", ".", "-o", "sweep-check"));
      args.addAll(c.options(t.dataset()));
      args.addAll(t.options());
      try {
        Options.parse(args.toArray(String[]::new));
      } catch (RuntimeException e) {
        out.add(t.dataset() + " " + t.id() + ": " + String.valueOf(e.getMessage()).split("\n")[0]);
      }
    }
    return out;
  }

  static void run(RunSettings s, Runner runner, PrintStream log) throws IOException, InterruptedException {
    Config c = Config.load(s.config);
    List<Trial> plan = new ArrayList<>();
    for (String line : Files.readAllLines(s.plan, StandardCharsets.UTF_8))
      if (!line.isBlank() && !line.startsWith("#") && !line.startsWith("trial\t")) plan.add(Trial.parse(line));
    List<String> bad = invalid(c, plan);
    if (!bad.isEmpty()) throw new IllegalArgumentException("The plan has trials photogrammetry would refuse:\n  " + String.join("\n  ", bad));
    Set<String> done = finished(s.results);
    if (!Files.isRegularFile(s.results)) {
      List<String> head = new ArrayList<>(HEAD);
      head.addAll(METRICS);
      Files.createDirectories(s.results.toAbsolutePath().getParent());
      Files.writeString(s.results, String.join(",", head) + "\n", StandardCharsets.UTF_8);
    }
    Map<String, String> bases = new HashMap<>();
    Map<String, Double> baselineWall = new HashMap<>();
    int ran = 0, skipped = 0, failed = 0;
    for (Trial t : plan) {
      if (!c.ds.containsKey(t.dataset())) throw new IllegalArgumentException("Plan names dataset " + t.dataset() + ", not in the config");
      if (!s.trials.isEmpty() && s.trials.stream().noneMatch(p -> t.id().startsWith(p))) continue;
      if (done.contains(t.dataset() + "\t" + t.id())) {
        skipped++;
        continue;
      }
      long free = Files.getFileStore(Files.exists(c.work) ? c.work : c.dir).getUsableSpace();
      if (free < s.minFreeGb * (1L << 30)) {
        log.printf(Locale.ROOT, "Stopping: %.1f GB free, below --min-free-gb %.0f%n", free / (double) (1L << 30), s.minFreeGb);
        break;
      }
      String base = bases.get(t.dataset());
      if (base == null) bases.put(t.dataset(), base = base(c, t.dataset(), runner, log));
      Path project = c.project(t.dataset());
      List<String> args = new ArrayList<>(List.of("dense", "-o", project.toString(), "--run", base, "--branch"));
      args.addAll(t.options());
      Path trialLog = c.meta(t.dataset()).resolve("trial.log");
      List<Path> before = runs(project);
      long timeout = (long) Math.max(1800, s.timeoutFactor * baselineWall.getOrDefault(t.dataset(), 3600.0));
      long t0 = System.nanoTime();
      int code = runner.run(args, c.dir, trialLog, timeout);
      double wall = (System.nanoTime() - t0) / 1e9;
      Path run = newRun(before, runs(project));
      String status = code == 0 ? "ok" : code == -1 ? "timeout" : "failed";
      Map<String, Double> tel = run == null ? Map.of() : telemetry(run.resolve("logs").resolve("processing.json"));
      Map<String, Double> metrics = new HashMap<>();
      if (status.equals("ok")) {
        Path las = run == null ? null : find(run, "dense.las");
        if (las == null) status = "no_cloud";
        else {
          try {
            List<String> a = new ArrayList<>(List.of("--cloud", las.toString()));
            String ref = c.ds.get(t.dataset()).getProperty("reference");
            if (ref != null) {
              Path frame = c.meta(t.dataset()).resolve("frame.properties");
              a.addAll(List.of("--reference", c.dir.resolve(ref).toString(), "--stable", c.ds.get(t.dataset()).getProperty("stable", "planar")));
              a.addAll(Files.isRegularFile(frame) ? List.of("--frame", frame.toString()) : List.of("--write-frame", frame.toString()));
            }
            String region = c.ds.get(t.dataset()).getProperty("region");
            if (region != null) a.addAll(List.of("--region", region));
            flatten(CloudScore.score(CloudScore.Settings.parse(a.toArray(String[]::new))), "", metrics);
          } catch (Exception e) {
            status = "score_failed";
            log.println(t.dataset() + " " + t.id() + ": scoring failed: " + e);
          }
        }
      }
      if (status.equals("ok") && t.id().equals("baseline")) baselineWall.put(t.dataset(), wall);
      List<String> row = new ArrayList<>(List.of(t.dataset(), t.id(), status, csv(String.join(" ", t.options())), num(wall),
          num(tel.getOrDefault("05_stereo", Double.NaN)), num(tel.getOrDefault("06_fusion", Double.NaN)),
          num(tel.getOrDefault("07_results", Double.NaN)), num(tel.getOrDefault("rss", Double.NaN) / (1L << 30)),
          num(tel.getOrDefault("vram", Double.NaN) / (1L << 30))));
      for (String k : METRICS) row.add(num(metrics.getOrDefault(k, Double.NaN)));
      Files.writeString(s.results, String.join(",", row) + "\n", StandardCharsets.UTF_8, StandardOpenOption.APPEND);
      if (!status.equals("ok")) {
        failed++;
        Path keep = c.meta(t.dataset()).resolve("failed");
        Files.createDirectories(keep);
        if (Files.isRegularFile(trialLog)) Files.copy(trialLog, keep.resolve(t.id().replaceAll("[^A-Za-z0-9=._-]", "_") + ".log"), StandardCopyOption.REPLACE_EXISTING);
      } else ran++;
      log.printf(Locale.ROOT, "%s %s: %s, %.0f s, %s points%n", t.dataset(), t.id(), status, wall, num(metrics.getOrDefault("points", Double.NaN)));
      if (run != null && !s.keep) delete(run);
      Files.deleteIfExists(trialLog);
    }
    log.printf("Sweep: %d run, %d already done, %d not ok%n", ran, skipped, failed);
  }

  // ---------------------------------------------------------------------------------------------

  public static void main(String[] a) throws Exception {
    if (a.length == 0 || a[0].equals("--help") || a[0].equals("-h")) {
      System.out.print(HELP);
      return;
    }
    Map<String, String> o = new LinkedHashMap<>();
    Set<String> flags = new HashSet<>();
    for (int i = 1; i < a.length; i++) {
      if (a[i].equals("--keep")) flags.add(a[i]);
      else if (i + 1 < a.length) o.put(a[i], a[++i]);
      else throw new IllegalArgumentException("Missing value for " + a[i] + "\n" + HELP);
    }
    switch (a[0]) {
      case "plan" -> {
        List<Trial> plan = plan(inventory(Path.of(o.getOrDefault("--inventory", "sweep/options-inventory.tsv"))),
            List.of(req(o, "--datasets").split(",")), Set.of(o.getOrDefault("--classes", "dense,fusion").split(",")),
            o.getOrDefault("--method", "mvsgm"), o.containsKey("--only") ? Set.of(o.get("--only").split(",")) : Set.of());
        StringBuilder b = new StringBuilder("trial\tdataset\toptions\n");
        for (Trial t : plan) b.append(t.line()).append('\n');
        Files.writeString(Path.of(req(o, "--out")), b, StandardCharsets.UTF_8);
        System.out.println(plan.size() + " trials -> " + o.get("--out"));
      }
      case "run" -> {
        RunSettings s = new RunSettings();
        s.config = Path.of(req(o, "--config"));
        s.plan = Path.of(req(o, "--plan"));
        s.results = Path.of(req(o, "--results"));
        s.keep = flags.contains("--keep");
        if (o.containsKey("--min-free-gb")) s.minFreeGb = Double.parseDouble(o.get("--min-free-gb"));
        if (o.containsKey("--timeout-factor")) s.timeoutFactor = Double.parseDouble(o.get("--timeout-factor"));
        if (o.containsKey("--trials")) s.trials = List.of(o.get("--trials").split(","));
        run(s, processRunner(Config.load(s.config).heap), System.out);
      }
      default -> throw new IllegalArgumentException("Unknown command " + a[0] + "\n" + HELP);
    }
  }

  static String req(Map<String, String> o, String k) {
    String v = o.get(k);
    if (v == null) throw new IllegalArgumentException(k + " is required\n" + HELP);
    return v;
  }
}
