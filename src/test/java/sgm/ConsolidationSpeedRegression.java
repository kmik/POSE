package sgm;

import java.nio.file.*;
import java.util.*;

/**
 * Surface consolidation speed work must not change results. With image-space fusion, the previous
 * path re-validated one sample at a time against unfrozen pair evidence; the current path freezes
 * the evidence and re-validates look-ahead blocks on all fusion workers, and routes/decodes
 * partitions in bulk. Both must publish byte-identical clouds and confidence records. The opt-in
 * --surface-flatten mode is checked for its accounting: it only removes samples (never adds) and
 * reports what it did.
 */
final class ConsolidationSpeedRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Path root = Files.createTempDirectory(Paths.get("target"), "consolidation-speed-");
    System.out.println("PASS consolidation speed/flatten: " + run(root));
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Path dir = Files.createDirectories(root.resolve("consolidation speed"));
    Path images = SelfTest.scene(dir, 3, false),
        calibration = dir.resolve("camera.properties"),
        poses = dir.resolve("poses.csv");
    Files.writeString(calibration, "width=400\nheight=300\nfx=240\nfy=240\ncx=199.5\ncy=149.5\n");
    Files.writeString(
        poses,
        PoseIO.HEADER
            + "\n"
            + "00.png,1,0,0,0,1,0,0,0,1,0,0,0\n"
            + "01.png,1,0,0,0,1,0,0,0,1,-0.25,0,0\n"
            + "02.png,1,0,0,0,1,0,0,0,1,-0.5,0,0\n");
    String[] variants = {"previous", "current", "flatten"};
    Path[] runs = new Path[variants.length];
    for (int v = 0; v < variants.length; v++) {
      List<String> args =
          new ArrayList<>(
              List.of(
                  "-i", images.toString(),
                  "-o", dir.resolve(variants[v]).toString(),
                  "--calibration", calibration.toString(),
                  "--poses", poses.toString(),
                  "--disparities", "40",
                  "--step", "1",
                  "--compute", "cpu",
                  "--cores", "4",
                  "--fusion-mode", "image",
                  "--min-views", "2",
                  "--depth-outlier-filter", "edge-aware",
                  "--surface-consolidation", "normal",
                  "--surface-memory-mb", "8"));
      if (variants[v].equals("flatten")) Collections.addAll(args, "--surface-flatten", "8");
      boolean previous = variants[v].equals("previous");
      if (previous) {
        System.setProperty("sgm.surfaceSerial", "true");
        System.setProperty("sgm.surfaceNoFreeze", "true");
      }
      try {
        check(Main.run(args.toArray(String[]::new)) == 0, variants[v] + " consolidation run completes");
      } finally {
        System.clearProperty("sgm.surfaceSerial");
        System.clearProperty("sgm.surfaceNoFreeze");
      }
      runs[v] = ProjectRegression.latest(dir.resolve(variants[v]), "latest_successful_run");
    }
    Map<String, Object> previous = report(runs[0]), current = report(runs[1]), flat = report(runs[2]);
    check(
        JsonRead.number(previous.get("input_samples")) > 2000
            && JsonRead.number(previous.get("partitions")) > 1,
        "Fixture exercises several partitions: " + previous.get("input_samples"));
    for (String file : List.of("07_results/dense.las", "07_results/point-confidence.bin"))
      check(
          Arrays.equals(
              Files.readAllBytes(runs[0].resolve(file)), Files.readAllBytes(runs[1].resolve(file))),
          "Parallel consolidation publishes the same bytes as the serial path: " + file);
    for (String key : List.of("output_samples", "merged_samples", "moved_samples"))
      check(
          JsonRead.number(previous.get(key)) == JsonRead.number(current.get(key)),
          "Same consolidation counts: " + key);
    double removed = JsonRead.number(flat.get("isolated_off_surface_removed"));
    check(
        JsonRead.number(flat.get("output_samples")) + JsonRead.number(flat.get("merged_samples"))
                + removed
            == JsonRead.number(flat.get("input_samples")),
        "Flatten accounts for every sample (output + merged + removed = input)");
    check(
        JsonRead.number(flat.get("fitted_normals")) > 0
            && JsonRead.number(current.get("fitted_normals")) == 0
            && JsonRead.number(current.get("isolated_off_surface_removed")) == 0,
        "Flatten fits missing normals; the default does not");
    return checks;
  }

  private static Map<String, Object> report(Path run) throws Exception {
    return JsonRead.object(JsonRead.read(run.resolve("06_fusion/surface_consolidation.json")));
  }
}
