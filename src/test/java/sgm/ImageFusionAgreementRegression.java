package sgm;

import java.nio.file.*;
import java.util.*;

/**
 * Image-fusion strategies (--fusion-strategy owner|owner-consume|graph-average): option parsing,
 * the strategy-dependent auto agreement radius, and (given forest_tiny) end-to-end checks: the
 * default reproduces the previous owner output, owner-consume is a subset of owner, and
 * graph-average is deterministic. Usage: ImageFusionAgreementRegression [forest_tiny dir].
 */
final class ImageFusionAgreementRegression {
  private static int checks;

  private static void check(boolean ok, String why) {
    checks++;
    if (!ok) throw new AssertionError(why);
  }

  static int run(Path images) throws Exception {
    checks = 0;
    Options d = new Options();
    check(d.fusionAgreementRadius == -1, "Default agreement radius is auto");
    check(d.fusionStrategy.equals("owner"), "Default strategy is the previous owner rule");
    for (int step = 1; step <= 4; step++) {
      d.step = step;
      d.fusionStrategy = "owner";
      check(ImageFusion.agreementRadius(d) == 0, "Auto radius keeps owner exact: " + step);
      for (String s : new String[] {"owner-consume", "graph-average"}) {
        d.fusionStrategy = s;
        check(ImageFusion.agreementRadius(d) == Math.max(1, step), "Auto radius follows the step for " + s);
      }
    }
    check(Options.parse(new String[] {"-i", ".", "--fusion-dedupe", "consume"}).fusionStrategy.equals("owner-consume"), "--fusion-dedupe consume alias");
    check(Options.parse(new String[] {"-i", ".", "--fusion-strategy", "graph-average"}).fusionStrategy.equals("graph-average"), "graph-average parses");
    boolean unknown = false;
    try {
      Options.parse(new String[] {"-i", ".", "--fusion-strategy", "average"});
    } catch (IllegalArgumentException e) {
      unknown = true;
    }
    check(unknown, "Unknown strategy is rejected");
    check(Options.parse(new String[] {"-i", ".", "--fusion-agreement-radius", "0"}).fusionAgreementRadius == 0, "Radius 0 parses");
    check(Options.parse(new String[] {"-i", ".", "--fusion-agreement-radius", "auto"}).fusionAgreementRadius == -1, "auto parses");
    boolean rejected = false;
    try {
      Options.parse(new String[] {"-i", ".", "--fusion-agreement-radius", "9"});
    } catch (IllegalArgumentException e) {
      rejected = true;
    }
    check(rejected, "Radius above 8 is rejected");
    if (images == null || !Files.isDirectory(images)) return checks;
    Path project = Files.createTempDirectory(Path.of("target"), "image fusion agreement ");
    List<String> common =
        List.of(
            "-i", images.toString(), "-o", project.toString(), "--quality", "custom",
            "--max-dimension", "480", "--alignment-max-dimension", "1600", "--features", "8000",
            "--descriptor-matcher", "flann", "--neighbors", "3", "--memory-mb", "4000",
            "--dense-method", "sgm", "--pairwise-search", "tsgm", "--fusion-mode", "image",
            "--surface-consolidation", "none", "--filtering", "mild", "--min-views", "2",
            "--report", "none", "--compute", "cpu", "--cores", "4");
    Map<String, Path> runs = new LinkedHashMap<>();
    String[][] cases = {
      {"0", "owner"}, {"auto", "owner"}, {"auto", "owner-consume"}, {"auto", "graph-average"}, {"auto", "graph-average"}
    };
    for (String[] c : cases) {
      String radius = c[0];
      List<String> args = new ArrayList<>(common);
      args.addAll(List.of("--fusion-agreement-radius", radius, "--fusion-strategy", c[1]));
      args.add(runs.isEmpty() ? "--cache" : "--resume");
      if (!runs.isEmpty()) args.addAll(List.of("--rerun-from", "fusion"));
      check(Main.run(args.toArray(String[]::new)) == 0, "Pipeline run with radius " + radius);
      Path latest;
      try (var dirs = Files.list(project.resolve("runs"))) {
        latest = dirs.sorted(Comparator.reverseOrder()).findFirst().orElseThrow();
      }
      runs.put(radius + "/" + c[1] + "#" + runs.size(), latest);
    }
    List<Path> r = new ArrayList<>(runs.values());
    // Auto radius under owner is 0: identical to the explicit previous behaviour.
    check(
        Arrays.equals(
            Files.readAllBytes(r.get(0).resolve("07_results/dense.ply")),
            Files.readAllBytes(r.get(1).resolve("07_results/dense.ply"))),
        "Default owner output equals radius 0");
    long owner = points(r.get(0)), consumed = points(r.get(2)), graph = points(r.get(3));
    // owner-consume keeps owner's rule and only adds suppression, so its output is a subset.
    check(consumed <= owner, "owner-consume emits no more than owner: " + owner + " -> " + consumed);
    check(graph > 0, "graph-average emits points");
    check(
        Arrays.equals(
            Files.readAllBytes(r.get(3).resolve("07_results/dense.ply")),
            Files.readAllBytes(r.get(4).resolve("07_results/dense.ply"))),
        "Repeated graph-average fusion is byte-identical");
    return checks;
  }

  private static long points(Path run) throws Exception {
    String stages = Files.readString(run.resolve("stages.csv"));
    var m = java.util.regex.Pattern.compile("06_fusion,complete,\"(\\d+) points").matcher(stages);
    check(m.find(), "Fusion summary present in " + run);
    return Long.parseLong(m.group(1));
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    System.out.println(
        "PASS image fusion agreement checks=" + run(args.length > 0 ? Path.of(args[0]) : null));
  }
}
