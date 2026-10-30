package sgm;

import java.io.*;
import java.nio.*;
import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

/** Preset precedence, preserved matching settings, real fusion behavior and project provenance. */
final class FilteringPresetRegression {
  private static int checks;

  private static void check(boolean value, String message) {
    checks++;
    if (!value) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Core.setNumThreads(1);
    Path root = Files.createTempDirectory(Path.of("target"), "filtering presets ");
    System.out.println("PASS filtering-preset checks=" + run(root) + "; fixtures=" + root);
  }

  private static Options parse(Path images, String... extra) {
    List<String> args = new ArrayList<>(Arrays.asList("-i", images.toString(), "--no-gps"));
    args.addAll(Arrays.asList(extra));
    return Options.parse(args.toArray(new String[0]));
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Path dir = Files.createDirectory(root.resolve("filtering preset regression")),
        images = Files.createDirectory(dir.resolve("images"));
    String[] names = {"mild", "moderate", "aggressive"};
    double[][] expected = {
      {.01, 1.25, .55, .45, 20}, {.005, .75, .6, .55, 50}, {.0025, .5, .75, .65, 100}
    };
    Options defaults = parse(images);
    for (int i = 0; i < names.length; i++) {
      Options o = parse(images, "--filtering", names[i]);
      double[] actual = {
        o.fusionRelative, o.fusionPixels, o.fusionConsensus, o.correlation, o.speckleSize
      };
      check(
          Arrays.equals(actual, expected[i]),
          "All five thresholds match the " + names[i] + " preset");
      check(
          o.maxDimension == defaults.maxDimension
              && o.features == defaults.features
              && o.disparities == defaults.disparities
              && o.disparitiesExplicit == defaults.disparitiesExplicit
              && o.census == defaults.census
              && o.directions == defaults.directions
              && o.p1 == defaults.p1
              && o.p2 == defaults.p2
              && o.uniqueness == defaults.uniqueness
              && o.lrThreshold == defaults.lrThreshold
              && o.minViews == defaults.minViews
              && o.minAngle == defaults.minAngle
              && o.maxDepthUncertainty == defaults.maxDepthUncertainty,
          "Preset leaves resolution, matching, required views and basic geometry settings"
              + " unchanged");
      Options copy = o.copy();
      check(
          copy.filtering.equals(o.filtering)
              && copy.fusionRelative == o.fusionRelative
              && copy.correlation == o.correlation,
          "Per-pair options preserve the preset and effective thresholds");
    }
    check(
        defaults.filtering.equals("moderate")
            && defaults.fusionRelative == .005
            && defaults.correlation == .55,
        "Omitted preset preserves current moderate defaults");
    String[] overrides = {
      "--fusion-relative",
      "0.007",
      "--fusion-pixels",
      "1.1",
      "--fusion-consensus",
      "0.7",
      "--min-correlation",
      "0.58",
      "--speckle-size",
      "37"
    };
    List<String> before = new ArrayList<>(Arrays.asList(overrides));
    before.addAll(Arrays.asList("--filtering", "mild"));
    List<String> after = new ArrayList<>(Arrays.asList("--filtering", "mild"));
    after.addAll(Arrays.asList(overrides));
    Options a = parse(images, before.toArray(new String[0])),
        b = parse(images, after.toArray(new String[0]));
    check(
        a.fusionRelative == .007
            && a.fusionPixels == 1.1
            && a.fusionConsensus == .7
            && a.correlation == .58
            && a.speckleSize == 37,
        "Explicit thresholds override all five preset values");
    check(
        a.filteringSummary().equals(b.filteringSummary()),
        "Explicit override precedence is independent of argument order");
    check(
        a.filteringOverrides.equals(
            "--fusion-consensus,--fusion-pixels,--fusion-relative,--min-correlation,--speckle-size"),
        "Override provenance is complete and deterministically ordered");
    Options single = parse(images, "--speckle-size", "77", "--filtering=aggressive");
    check(
        single.speckleSize == 77 && single.fusionRelative == .0025 && single.correlation == .65,
        "A partial override preserves remaining preset values");
    Options last =
        parse(
            images, "--filtering", "mild", "--min-correlation", ".5", "--filtering", "AGGRESSIVE");
    check(
        last.filtering.equals("aggressive") && last.correlation == .5 && last.speckleSize == 100,
        "Last preset wins while explicitly selected thresholds remain authoritative");
    for (String[] invalid :
        new String[][] {
          {"--filtering", "none"},
          {"--filtering"},
          {"--fusion-consensus", ".4", "--filtering", "mild"}
        }) {
      boolean rejected = false;
      try {
        parse(images, invalid);
      } catch (IllegalArgumentException e) {
        rejected = true;
      }
      check(rejected, "Invalid preset, missing value or invalid explicit threshold fails clearly");
    }
    for (String name : names) {
      Options o = parse(images, "--filtering", name);
      check(fuse(o, 20.14).accepted(), "All intensities retain a consistently measured surface");
      check(
          fuse(o, 20.00).accepted() == name.equals("mild"),
          "Only mild accepts the controlled 0.7-percent depth disagreement");
      check(
          fuse(o, 20.08).accepted() != name.equals("aggressive"),
          "Moderate accepts a 0.3-percent disagreement rejected by aggressive");
    }
    render(images);
    Path calibration = dir.resolve("camera.txt");
    new Calibration(120, 120, 63.5, 39.5, new double[5], 128, 80)
        .write(calibration, "Known rectified camera");
    for (String name : names) {
      Path project = dir.resolve(name);
      runProject(images, project, calibration, name);
      Properties settings = new Properties();
      try (Reader reader = Files.newBufferedReader(project.resolve("run-settings.properties"))) {
        settings.load(reader);
      }
      check(
          settings.getProperty("filtering").equals(name)
              && settings.getProperty("filteringOverrides").isEmpty(),
          "Project settings preserve selected preset and override provenance");
      Properties marker = new Properties();
      try (Reader reader = Files.newBufferedReader(project.resolve("project.properties"))) {
        marker.load(reader);
      }
      Path run = project.resolve("runs").resolve(marker.getProperty("latest_successful_run"));
      String fusion = Files.readString(run.resolve("06_fusion/fusion.json"));
      check(
          fusion.contains("\"filtering_preset\":\"" + name + "\"")
              && fusion.contains("minimum_correlation")
              && fusion.contains("minimum_speckle_component_pixels"),
          "Fusion audit records preset and effective stereo thresholds");
      check(
          Files.readString(project.resolve("report.txt")).contains("Point filtering=" + name),
          "Human-readable report identifies preset and effective settings");
      ByteBuffer las =
          ByteBuffer.wrap(Files.readAllBytes(project.resolve("dense.las")))
              .order(ByteOrder.LITTLE_ENDIAN);
      check(las.getInt(107) > 100, "Each preset completes a valid dense stereo reconstruction");
    }
    runProject(images, dir.resolve("default"), calibration, null);
    check(
        Project.sha256(dir.resolve("moderate/dense.las"))
            .equals(Project.sha256(dir.resolve("default/dense.las"))),
        "Explicit moderate and omitted preset produce byte-identical LAS output");
    Path custom = dir.resolve("custom");
    runProject(images, custom, calibration, "mild", "--speckle-size", "37");
    Properties customSettings = new Properties();
    try (Reader reader = Files.newBufferedReader(custom.resolve("run-settings.properties"))) {
      customSettings.load(reader);
    }
    check(
        customSettings.getProperty("filtering").equals("mild")
            && customSettings.getProperty("filteringOverrides").equals("--speckle-size")
            && customSettings.getProperty("speckleSize").equals("37")
            && customSettings.getProperty("fusionRelative").equals("0.01"),
        "Run settings distinguish a named preset from overridden effective values");
    Properties customMarker = new Properties();
    try (Reader reader = Files.newBufferedReader(custom.resolve("project.properties"))) {
      customMarker.load(reader);
    }
    String customFusion =
        Files.readString(
            custom
                .resolve("runs")
                .resolve(customMarker.getProperty("latest_successful_run"))
                .resolve("06_fusion/fusion.json"));
    check(
        customFusion.contains("\"filtering_explicit_overrides\":\"--speckle-size\"")
            && customFusion.contains("\"minimum_speckle_component_pixels\":37"),
        "Fusion audit stores actual overridden threshold rather than nominal preset value");
    return checks;
  }

  private static DepthFusion.Result fuse(Options o, double sourceDepth) {
    Camera[] cameras = {
      new Camera(400, 400, 200, 80), new Camera(400, 400, 200, 80), new Camera(400, 400, 200, 80)
    };
    cameras[0].translation[0] = 1;
    cameras[2].translation[0] = -1;
    DepthFusion fusion =
        new DepthFusion(cameras, new int[] {400, 400, 400}, new int[] {160, 160, 160}, o);
    int source = fusion.addPair(0, 1), one = fusion.addPair(0, 2), two = fusion.addPair(1, 2);
    fusion.add(source, new double[] {0, 0, sourceDepth});
    fusion.add(one, new double[] {0, 0, 20.14});
    fusion.add(two, new double[] {0, 0, 20.14});
    return fusion.fuse(source, new double[] {0, 0, sourceDepth});
  }

  private static void render(Path images) throws IOException {
    Mat texture = new Mat(80, 140, CvType.CV_8UC3);
    byte[] values = new byte[80 * 140 * 3];
    new Random(524).nextBytes(values);
    texture.put(0, 0, values);
    Imgproc.GaussianBlur(texture, texture, new Size(3, 3), .6);
    Mat left = texture.submat(0, 80, 0, 128), right = texture.submat(0, 80, 12, 140);
    try {
      if (!Imgcodecs.imwrite(images.resolve("0.png").toString(), left)
          || !Imgcodecs.imwrite(images.resolve("1.png").toString(), right))
        throw new IOException("Cannot write synthetic stereo");
    } finally {
      left.release();
      right.release();
      texture.release();
    }
  }

  private static void runProject(
      Path images, Path project, Path calibration, String preset, String... overrides) {
    List<String> args =
        new ArrayList<>(
            Arrays.asList(
                "-i",
                images.toString(),
                "-o",
                project.toString(),
                "--calibration",
                calibration.toString(),
                "--rectified",
                "--baseline",
                ".25",
                "--disparities",
                "24",
                "--cores",
                "1",
                "--no-gps"));
    args.addAll(Arrays.asList(overrides));
    if (preset != null) args.addAll(Arrays.asList("--filtering", preset));
    check(
        TestSupport.run(args.toArray(new String[0])) == 0,
        "CLI reconstructs with filtering=" + (preset == null ? "default" : preset));
  }
}
