package sgm;

import java.nio.file.*;
import java.util.*;
import java.util.regex.*;

final class LauncherRegression {
  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Path root = Files.createTempDirectory(Path.of("target"), "launcher test ");
    System.out.println("PASS launcher/help=" + run(root));
  }

  static int run(Path root) throws Exception {
    Path images = Files.createDirectories(root.resolve("launcher input with spaces"));
    org.opencv.core.Mat m =
        new org.opencv.core.Mat(
            64, 96, org.opencv.core.CvType.CV_8UC3, new org.opencv.core.Scalar(80));
    try {
      org.opencv.imgcodecs.Imgcodecs.imwrite(images.resolve("0.png").toString(), m);
      org.opencv.imgcodecs.Imgcodecs.imwrite(images.resolve("1.png").toString(), m);
    } finally {
      m.release();
    }
    String javaBin =
        Path.of(
                System.getProperty("java.home"),
                "bin",
                System.getProperty("os.name").toLowerCase(Locale.ROOT).contains("win")
                    ? "java.exe"
                    : "java")
            .toString();
    Path log = root.resolve("launcher-memory.log");
    Path project = root.resolve("launcher project with spaces");
    Process p =
        new ProcessBuilder(
                javaBin,
                "--enable-native-access=ALL-UNNAMED",
                "-Xmx64m",
                "-cp",
                System.getProperty("java.class.path"),
                "photogrammetry",
                "-i",
                images.toString(),
                "-o",
                project.toString(),
                "--memory-mb=128",
                "--compute",
                "cpu",
                "--estimate-memory",
                "--max-dimension",
                "64")
            .redirectErrorStream(true)
            .redirectOutput(log.toFile())
            .start();
    if (!p.waitFor(60, java.util.concurrent.TimeUnit.SECONDS)) {
      p.destroyForcibly();
      throw new AssertionError("Heap relaunch timed out");
    }
    String text = Files.readString(log);
    if (p.exitValue() != 0 || !text.contains("starting JVM with 213 MiB heap"))
      throw new AssertionError("CLI heap relaunch/budget/paths failed: " + text);
    // Non-verbose planning no longer prints the detailed memory summary. Check the
    // child's actual budget and heap in its structured plan, not terminal formatting.
    Path run = ProjectRegression.latest(project, "latest_successful_run");
    Map<String, Object> plan = JsonRead.object(JsonRead.read(run.resolve("00_inputs/resolution_plan.json")));
    double budget = JsonRead.number(plan.get("sgm_budget_bytes"));
    double heap = JsonRead.number(plan.get("jvm_max_heap_bytes"));
    if (budget != 128L * 1048576 || heap < 213L * 1048576 || heap >= 256L * 1048576)
      throw new AssertionError("Relaunched child did not retain its 128 MiB budget/213 MiB heap: " + plan);
    int checks = 2;
    Matcher keys =
        Pattern.compile("case \"(-{1,2}[A-Za-z0-9_-]+)\"")
            .matcher(Files.readString(Path.of("src/main/java/sgm/Options.java")));
    while (keys.find()) {
      checks++;
      if (!Options.help().contains(keys.group(1)))
        throw new AssertionError("Help omits modifiable argument " + keys.group(1));
    }
    return checks;
  }
}
