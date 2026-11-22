package sgm;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.*;

/**
 * ParamSweep without real processing: plan generation from an inventory (method filter, flags,
 * required companion options), and the run loop with a fake photogrammetry runner that writes a
 * synthetic cloud and telemetry: one alignment per dataset, branch per trial, scoring in one frame,
 * telemetry columns, failure logs, deletion of trial runs and resuming.
 */
final class ParamSweepRegression {
  private static int checks;

  private static void check(boolean value, String why) {
    checks++;
    if (!value) throw new AssertionError(why);
  }

  public static void main(String[] args) throws Exception {
    Path root = Files.createTempDirectory(Path.of("target"), "param-sweep-");
    try {
      System.out.println("PASS param sweep: " + run(root));
    } finally {
      LargeFormatRegression.delete(root);
    }
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Files.createDirectories(root);
    Path inv = root.resolve("inventory.tsv");
    Files.writeString(inv, String.join("\n",
        "flag\tclass\tstep\tmethods\tkind\tbaseline\tlevels\trequires\tnote\thelp_section\thelp_default",
        "--min-views\tfusion\tdense\tall\tTEXT\t2\t3,4\t\t\tx\t3",
        "--mvsgm-best-k\tdense\tdense\tmvsgm\tTEXT\t3\t2\t\t\tx\t3",
        "--p1\tdense\tdense\tsgm\tTEXT\t4\t2\t\t\tx\t4",
        "--thin-surfaces\tfusion\tdense\tall\tFLAG\toff\ton\t\t\tx\t",
        "--fusion-sigma-k\tfusion\tdense\tall\tTEXT\t3\t2\t--fusion-tolerance sigma\twith --fusion-tolerance sigma\tx\t3",
        "--features\talign\tcalibrate\tall\tTEXT\t20000\t10000\t\t\tx\t8000",
        "--cores\tspeed\t-\tall\tTEXT\t-\t-\t\t\tx\t") + "\n");
    List<ParamSweep.Trial> plan = ParamSweep.plan(ParamSweep.inventory(inv), List.of("a"), Set.of("dense", "fusion"), "mvsgm", Set.of());
    List<String> ids = plan.stream().map(ParamSweep.Trial::id).toList();
    check(ids.equals(List.of("baseline", "baseline-repeat", "min-views=3", "min-views=4", "mvsgm-best-k=2", "thin-surfaces=on", "fusion-sigma-k=2")),
        "Plan: baselines, one trial per level, method and class filters: " + ids);
    check(plan.get(5).options().equals(List.of("--thin-surfaces")), "Flag level 'on' adds the flag");
    check(plan.get(6).options().equals(List.of("--fusion-tolerance", "sigma", "--fusion-sigma-k", "2")), "Companion option is added");
    check(ParamSweep.Trial.parse(plan.get(6).line()).equals(plan.get(6)), "Trial line round trip");

    // Config + fake runner.
    Path ref = root.resolve("reference.las");
    CloudScoreRegression.writeLas(ref, CloudScoreRegression.scene(1, .005), 2);
    Files.createDirectories(root.resolve("images"));
    Files.writeString(root.resolve("sweep.properties"), String.join("\n",
        "work = work", "datasets = a", "a.images = images", "a.options = --quality medium --dense-method mvsgm --fusion-mode image", "a.reference = reference.las", "a.stable = planar") + "\n");
    Path planFile = root.resolve("plan.tsv");
    List<ParamSweep.Trial> trials = new ArrayList<>(plan);
    trials.add(new ParamSweep.Trial("broken", "a", List.of("--min-angle", "7")));
    StringBuilder b = new StringBuilder("trial\tdataset\toptions\n");
    for (ParamSweep.Trial t : trials) b.append(t.line()).append('\n');
    Files.writeString(planFile, b);

    int[] counter = {0};
    List<List<String>> calls = new ArrayList<>();
    List<double[]> truth = CloudScoreRegression.moved(CloudScoreRegression.scene(2, .01), new double[] {.4, -.3, .5}, Math.toRadians(.05));
    ParamSweep.Runner fake = (args, dir, log, timeout) -> {
      calls.add(List.copyOf(args));
      Path project = Path.of(args.get(args.indexOf("-o") + 1));
      // Like photogrammetry: a new project needs an empty (or missing) folder.
      if (args.get(0).equals("inputs") && Files.isDirectory(project))
        try (var l = Files.list(project)) {
          if (l.findAny().isPresent()) return 2;
        }
      Path run = project.resolve("runs").resolve(String.format("R%04d", ++counter[0]));
      Files.createDirectories(run.resolve("logs"));
      Files.writeString(log, "fake run " + args + "\n");
      if (args.contains("--min-angle") && args.contains("7")) return 3; // the "broken" trial
      if (args.get(0).equals("dense")) {
        Files.createDirectories(run.resolve("07_results"));
        List<double[]> pts = truth;
        if (args.contains("--min-views") && args.get(args.indexOf("--min-views") + 1).equals("3")) pts = truth.stream().filter(p -> p[0] < 40).toList();
        CloudScoreRegression.writeLas(run.resolve("07_results").resolve("dense.las"), pts, 0);
        Files.writeString(run.resolve("logs").resolve("processing.json"),
            "{\"tasks\":[{\"task\":\"Stage 05_stereo — fake\",\"started_offset_seconds\":1.0,\"wall_seconds\":12.5,\"status\":\"complete\"},"
                + "{\"task\":\"Stage 06_fusion — fake\",\"started_offset_seconds\":14.0,\"wall_seconds\":3.25,\"status\":\"complete\"}],"
                + "\"peak_sampled_process_resident_bytes\":2147483648,\"peak_device_vram_used_bytes\":1073741824}");
      }
      return 0;
    };
    ParamSweep.RunSettings s = new ParamSweep.RunSettings();
    s.config = root.resolve("sweep.properties");
    s.plan = planFile;
    s.results = root.resolve("results.csv");
    s.minFreeGb = 0;
    ByteArrayOutputStream out = new ByteArrayOutputStream();
    ParamSweep.run(s, fake, new PrintStream(out, true, StandardCharsets.UTF_8));
    List<String> rows = Files.readAllLines(s.results);
    check(rows.size() == 1 + trials.size(), "One CSV row per trial: " + rows.size());
    check(calls.get(0).get(0).equals("inputs") && calls.get(0).contains("--quality"), "Dataset aligned once with its baseline options");
    check(calls.stream().filter(c -> c.get(0).equals("inputs")).count() == 1, "Alignment runs once per dataset");
    check(calls.get(1).subList(0, 6).equals(List.of("dense", "-o", root.resolve("work").resolve("a").toAbsolutePath().normalize().toString(), "--run", "R0001", "--branch")),
        "Trials branch from the base run: " + calls.get(1));
    Map<String, String[]> byTrial = new HashMap<>();
    List<String> head = List.of(rows.get(0).split(","));
    for (String r : rows.subList(1, rows.size())) {
      String[] f = r.split(",", -1);
      byTrial.put(f[1], f);
    }
    String[] baseRow = byTrial.get("baseline"), rep = byTrial.get("baseline-repeat"), mv3 = byTrial.get("min-views=3");
    int cov = head.indexOf("completeness.covered_fraction"), med = head.indexOf("shape.stable.median_m");
    check(baseRow[2].equals("ok") && Double.parseDouble(baseRow[med]) < .02, "Baseline scored in its fitted frame");
    int fitCol = head.indexOf("position.fit_residual_rms_m");
    check(Arrays.equals(baseRow, 10, fitCol, rep, 10, fitCol), "Same cloud in the saved frame gives the same metrics");
    check(!baseRow[fitCol].isEmpty() && rep[fitCol].isEmpty(), "Only the first trial fits; later ones use the frame");
    check(Double.parseDouble(mv3[cov]) < Double.parseDouble(baseRow[cov]) - .2, "Trial differences show in the metrics (completeness): " + mv3[cov] + " vs " + baseRow[cov]);
    check(baseRow[head.indexOf("stereo_s")].equals("12.5000") && baseRow[head.indexOf("peak_rss_gb")].equals("2.00000"), "Telemetry columns");
    check(byTrial.get("broken")[2].equals("failed") && Files.isRegularFile(root.resolve("work/a-sweep/failed/broken.log")), "Failed trial recorded with its log");
    check(Files.isRegularFile(root.resolve("work/a-sweep/frame.properties")), "Frame saved for the dataset");
    check(ParamSweep.runs(root.resolve("work/a")).size() == 1, "Trial runs deleted, base run kept");

    // Resume: finished trials are skipped, the failed one is tried again.
    int before = calls.size();
    ParamSweep.run(s, fake, new PrintStream(new ByteArrayOutputStream(), true, StandardCharsets.UTF_8));
    check(calls.size() == before + 1 && calls.get(calls.size() - 1).contains("7"), "Resume runs only unfinished trials");
    check(Files.readAllLines(s.results).size() == rows.size() + 1, "Resume appends");

    // A plan with an option value photogrammetry refuses does not start.
    Path badPlan = root.resolve("bad-plan.tsv");
    Files.writeString(badPlan, "trial\tdataset\toptions\nmedian=0\ta\t--ws_mFilter 0\n");
    ParamSweep.RunSettings sb = new ParamSweep.RunSettings();
    sb.config = s.config;
    sb.plan = badPlan;
    sb.results = root.resolve("bad-results.csv");
    int callsBefore = calls.size();
    boolean stopped = false;
    try {
      ParamSweep.run(sb, fake, new PrintStream(new ByteArrayOutputStream(), true, StandardCharsets.UTF_8));
    } catch (IllegalArgumentException e) {
      stopped = e.getMessage().contains("median=0");
    }
    check(stopped && calls.size() == callsBefore, "Invalid trial options stop the sweep before any run");

    // Changed baseline options with the same work folder are refused.
    Files.writeString(root.resolve("sweep.properties"), String.join("\n",
        "work = work", "datasets = a", "a.images = images", "a.options = --quality high --dense-method mvsgm --fusion-mode image") + "\n");
    Files.writeString(s.results, rows.get(0) + "\n");
    boolean refused = false;
    try {
      ParamSweep.run(s, fake, new PrintStream(new ByteArrayOutputStream(), true, StandardCharsets.UTF_8));
    } catch (IllegalStateException e) {
      refused = e.getMessage().contains("other options");
    }
    check(refused, "A base run made with other options is not reused");
    return checks;
  }
}
