package sgm;

import java.io.*;
import java.nio.*;
import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

/**
 * Camera groups (several cameras in one block): grouping rules, the guard against one camera's
 * calibration being rescaled onto another, multi-camera orientation import/export round trips,
 * runs saved before camera groups, and a synthetic two-camera block with known lenses and poses.
 */
final class MultiCameraRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  private static void near(double actual, double expected, double tolerance, String message) {
    check(
        Math.abs(actual - expected) <= tolerance,
        message + ": actual=" + actual + ", expected=" + expected);
  }

  interface Action {
    void run() throws Exception;
  }

  private static String rejects(Action a, String message) throws Exception {
    try {
      a.run();
    } catch (IllegalArgumentException e) {
      checks++;
      return e.getMessage();
    }
    throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Core.setNumThreads(1);
    Core.setRNGSeed(1);
    Path root = Files.createTempDirectory(Paths.get(System.getProperty("sgm.test.root", "target")), "multi camera ");
    boolean full = !Arrays.asList(args).contains("--quick");
    System.out.println("PASS multi-camera: " + run(root, full) + " checks; " + root);
  }

  static int run(Path root) throws Exception {
    return run(root, true);
  }

  static int run(Path root, boolean pipeline) throws Exception {
    checks = 0;
    Files.createDirectories(root);
    grouping(root.resolve("grouping"));
    silentRescale(root.resolve("rescale"));
    inpho(root.resolve("inpho"));
    roundTrips(root.resolve("round trips"));
    legacyState(root.resolve("legacy"));
    quickDefault(root.resolve("quick"));
    cameraFiles(root.resolve("camera files"));
    if (pipeline) block(root.resolve("block"));
    return checks;
  }

  /** Quick calibration is the command-line default; off forms; old runs; the GUI box. */
  private static void quickDefault(Path dir) throws Exception {
    Path images = dir.resolve("images");
    photo(images, "a1.jpg", 64, 48, "Acme", "Model-One", null, 8.8);
    String in = images.toString(), out = dir.resolve("p").toString();
    Options d = Options.parse(new String[] {"-i", in, "-o", out});
    check(d.quickCalibration && d.calibrationPipeline.equals("joint"), "Quick calibration is the command-line default");
    check(!new Options().quickCalibration && new Options().calibrationPipeline.equals("bootstrap"),
        "Programmatic options keep the bootstrap path");
    for (String[] off : new String[][] {
        {"--quick-calibration", "off"}, {"--quick-calibration=off"}, {"--no-quick-calibration"}, {"--calibration-pipeline", "bootstrap"}}) {
      List<String> a = new ArrayList<>(List.of("-i", in, "-o", out));
      a.addAll(List.of(off));
      Options p = Options.parse(a.toArray(String[]::new));
      check(!p.quickCalibration && p.calibrationPipeline.equals("bootstrap"), String.join(" ", off) + " restores the bootstrap");
    }
    Options bare = Options.parse(new String[] {"-i", in, "--quick-calibration", "-o", out});
    check(bare.quickCalibration && bare.output.toString().equals(out), "Bare --quick-calibration takes no value from the next option");
    Options later = Options.parse(new String[] {"-i", in, "-o", out, "--no-quick-calibration", "--quick-calibration", "on"});
    check(later.quickCalibration, "The last quick-calibration choice wins");
    rejects(() -> Options.parse(new String[] {"-i", in, "-o", out, "--quick-calibration", "on", "--calibration-pipeline", "bootstrap"}),
        "--quick-calibration on with --calibration-pipeline bootstrap rejected");
    check(Options.legacyQuickCalibration(List.of("-i", in)).equals(List.of("-i", in, "--no-quick-calibration")),
        "Saved arguments without a choice (run before the change) continue with quick calibration off");
    check(Options.legacyQuickCalibration(List.of("-i", in, "--quick-calibration")).equals(List.of("-i", in, "--quick-calibration"))
            && Options.legacyQuickCalibration(List.of("--no-quick-calibration")).size() == 1,
        "Saved arguments with a choice stay as they are");
    List<String> cleared = GuiOptions.arguments(Map.of("-i", in, "-o", out), "");
    int at = cleared.indexOf("--quick-calibration");
    check(at >= 0 && cleared.get(at + 1).equals("off") && GuiOptions.validate(cleared) == null, "GUI: a cleared box passes --quick-calibration off: " + cleared);
    List<String> set = GuiOptions.arguments(Map.of("-i", in, "-o", out, "--quick-calibration", "true"), "");
    check(set.contains("--quick-calibration") && !set.contains("off"), "GUI: a set box passes the bare flag");
    // --calibration-trial-method / --calibration-trial-cameras.
    check(d.calibrationTrialMethod.equals("full") && d.calibrationTrialCameras == 0, "Trial method defaults: full, auto subset size");
    Options joint = Options.parse(new String[] {"-i", in, "-o", out, "--calibration-trial-method", "JOINT", "--calibration-trial-cameras", "60"});
    check(joint.calibrationTrialMethod.equals("joint") && joint.calibrationTrialCameras == 60, "Trial method and subset size parsed");
    check(Options.parse(new String[] {"-i", in, "-o", out, "--calibration-trial-cameras", "auto"}).calibrationTrialCameras == 0, "Subset size auto");
    rejects(() -> Options.parse(new String[] {"-i", in, "-o", out, "--calibration-trial-method", "fast"}), "Unknown trial method rejected");
    rejects(() -> Options.parse(new String[] {"-i", in, "-o", out, "--calibration-trial-cameras", "2"}), "Too small subset rejected");
    check(CalibrationSubset.limit(1) == 125 && CalibrationSubset.size(0, 1) == 100 && CalibrationSubset.size(500, 1) == 125
            && CalibrationSubset.size(500, 3) == (768 - 39) / 6,
        "Subset size: default 100, bounded by the observability limit per camera count");
    List<String> gui = GuiOptions.arguments(Map.of("-i", in, "-o", out, "--quick-calibration", "true", "--calibration-trial-method", "subset"), "");
    check(gui.contains("--calibration-trial-method") && gui.get(gui.indexOf("--calibration-trial-method") + 1).equals("subset")
        && GuiOptions.validate(gui) == null, "GUI: trial method drop-down passes its choice");
  }

  /** --calibration CAMERA=FILE in POSE and foreign formats; a file of several cameras is refused. */
  private static void cameraFiles(Path dir) throws Exception {
    Files.createDirectories(dir);
    CameraGroups.Group g = new CameraGroups.Group("camA", 640, 480);
    List<Path> members = List.of(dir.resolve("a1.jpg"), dir.resolve("a2.jpg"));
    Options o = TestSupport.options();
    Path yaml = dir.resolve("camA.yml");
    Files.writeString(yaml, """
        %YAML:1.0
        image_width: 640
        image_height: 480
        camera_matrix: !!opencv-matrix
           rows: 3
           cols: 3
           dt: d
           data: [ 800, 0, 319.5, 0, 810, 239.5, 0, 0, 1 ]
        distortion_coefficients: !!opencv-matrix
           rows: 1
           cols: 5
           dt: d
           data: [ -0.1, 0.01, 0.003, -0.004, 0 ]
        """);
    Calibration cv = Pipeline.cameraFile(yaml, g, members, o);
    check(cv.fx == 800 && cv.fy == 810 && cv.distortion[0] == -.1 && cv.width == 640, "CAMERA=FILE reads OpenCV YAML");
    Path txt = dir.resolve("camA.txt");
    Files.writeString(txt, "fx=700\nfy=700\ncx=319.5\ncy=239.5\nk1=-0.05\n");
    Calibration own = Pipeline.cameraFile(txt, g, members, o);
    Options legacy = o.copy();
    legacy.calibration = txt;
    Calibration direct = Calibration.load(legacy, 640, 480);
    check(own.fx == direct.fx && own.cx == direct.cx && Arrays.equals(own.distortion, direct.distortion) && own.width == 640,
        "CAMERA=FILE reads POSE camera.txt as before");
    Path colmap = dir.resolve("colmap two");
    Files.createDirectories(colmap);
    Files.writeString(colmap.resolve("cameras.txt"), "1 PINHOLE 640 480 800 800 319.5 239.5\n2 PINHOLE 640 480 600 600 319.5 239.5\n");
    Files.writeString(colmap.resolve("images.txt"),
        "1 1 0 0 0 0 0 0 1 x1.jpg\n\n2 1 0 0 0 1 0 0 2 x2.jpg\n\n");
    Files.writeString(colmap.resolve("points3D.txt"), "");
    String message = rejects(() -> Pipeline.cameraFile(colmap, g, members, o), "A file of two different cameras is refused");
    check(message.contains("2 different calibrations"), "The refusal says why: " + message);
    Files.writeString(colmap.resolve("images.txt"),
        "1 1 0 0 0 0 0 0 1 a1.jpg\n\n2 1 0 0 0 1 0 0 2 x2.jpg\n\n");
    check(Pipeline.cameraFile(colmap, g, members, o).fx == 800, "Several cameras: the one of this camera's images is used");
  }

  // ---- Fixtures.

  /** A JPEG of the given size with EXIF make, model, optional body serial and focal length. */
  static Path photo(Path dir, String name, int w, int h, String make, String model, String serial, double focalMm)
      throws IOException {
    Files.createDirectories(dir);
    Path file = dir.resolve(name);
    Mat m = new Mat(h, w, CvType.CV_8UC3, new Scalar(90, 120, 150));
    try {
      Imgcodecs.imwrite(file.toString(), m);
    } finally {
      m.release();
    }
    if (make != null) GeoCalibrationRegression.inject(file, exif(make, model, serial, focalMm));
    return file;
  }

  /** Little-endian TIFF/EXIF block: IFD0 Make, Model, ExifIFD; ExifIFD FocalLength, BodySerialNumber. */
  static byte[] exif(String make, String model, String serial, double focalMm) {
    ByteBuffer b = ByteBuffer.allocate(512).order(ByteOrder.LITTLE_ENDIAN);
    b.putShort((short) 0x4949).putShort((short) 42).putInt(8);
    b.position(8);
    b.putShort((short) 3);
    ascii(b, 0x010f, make, 200);
    ascii(b, 0x0110, model, 260);
    b.putShort((short) 0x8769).putShort((short) 4).putInt(1).putInt(60);
    b.putInt(0);
    b.position(60);
    b.putShort((short) (serial == null ? 1 : 2));
    b.putShort((short) 0x920a).putShort((short) 5).putInt(1).putInt(320);
    if (serial != null) ascii(b, 0xa431, serial, 340);
    b.putInt(0);
    b.position(320);
    b.putInt((int) Math.round(focalMm * 1000)).putInt(1000);
    return Arrays.copyOf(b.array(), 512);
  }

  private static void ascii(ByteBuffer b, int tag, String text, int offset) {
    byte[] bytes = (text + "\0").getBytes(java.nio.charset.StandardCharsets.US_ASCII);
    b.putShort((short) tag).putShort((short) 2).putInt(bytes.length).putInt(offset);
    int at = b.position();
    b.position(offset);
    b.put(bytes);
    b.position(at);
  }

  private static Options options(Path input, Path output) throws IOException {
    Options o = TestSupport.options();
    o.input = input;
    o.output = output;
    Files.createDirectories(output);
    return o;
  }

  // ---- Grouping rules.

  private static void grouping(Path dir) throws Exception {
    Path images = dir.resolve("images");
    photo(images, "a1.jpg", 64, 48, "Acme", "Model-One", null, 8.8);
    photo(images, "a2.jpg", 64, 48, "Acme", "Model-One", null, 8.8);
    photo(images, "b1.jpg", 64, 48, "Acme", "Model-Two", null, 8.8);
    photo(images, "c1.jpg", 48, 36, "Acme", "Model-One", null, 8.8);
    photo(images, "d1.jpg", 64, 48, "Acme", "Model-One", "SN-0002", 8.8);
    photo(images, "e1.jpg", 64, 48, "Acme", "Model-One", null, 12.04);
    Options o = options(images, dir.resolve("out"));
    List<Path> paths = Main.images(o);
    check(Exif.read(images.resolve("d1.jpg")).serial.equals("SN-0002"), "EXIF body serial read");
    CameraGroups g = CameraGroups.resolve(o, paths, Map.of(), Map.of());
    check(g.size() == 5, "EXIF key: model, serial, focal and size split the cameras: " + g.size());
    check(g.get(0).images.equals(List.of("a1.jpg", "a2.jpg")), "Same camera shares a group; group 0 holds the first image");
    check(g.of("c1.jpg") != g.of("a1.jpg"), "Same EXIF camera, other image size: separate group");
    check(g.of("d1.jpg") != g.of("a1.jpg"), "Body serial separates two bodies of one model");
    check(g.of("e1.jpg") != g.of("a1.jpg"), "Focal length (0.1 mm) separates zoom settings");
    check(g.get(g.index("c1.jpg")).width == 48 && g.get(g.index("c1.jpg")).height == 36, "Group keeps its image size");
    check(g.get(0).id.equals("camera_1") && g.get(0).label.contains("Acme Model-One"), "EXIF group ID and label");

    o.cameraGrouping = "single";
    CameraGroups single = CameraGroups.resolve(o, paths, Map.of(), Map.of());
    check(single.size() == 2, "single: one camera per image size, never one calibration for two sizes");
    o.cameraGrouping = "per-image";
    CameraGroups each = CameraGroups.resolve(o, paths, Map.of(), Map.of());
    check(each.size() == paths.size() && each.of("b1.jpg").id.equals("b1"), "per-image: every image its own camera");

    o.cameraGrouping = "auto";
    Path csv = dir.resolve("groups.csv");
    Files.writeString(csv, "image,camera\na1.jpg,left\na2.jpg,right\nb1.jpg,left\nc1.jpg,left\nd1.jpg,right\ne1.jpg,right\n");
    o.cameraGroupsFile = csv;
    CameraGroups file = CameraGroups.resolve(o, paths, Map.of(), Map.of());
    check(file.size() == 3 && file.of("a1.jpg").id.equals("left") && file.of("a2.jpg").id.equals("right"),
        "--camera-groups names the cameras");
    check(file.of("c1.jpg").id.equals("left_48x36"), "A named camera with two image sizes splits by size");
    Files.writeString(csv, "a1.jpg,left\n");
    rejects(() -> CameraGroups.resolve(o, paths, Map.of(), Map.of()), "--camera-groups must list every image");
    o.cameraGroupsFile = null;

    Map<String, String> imported = new LinkedHashMap<>();
    for (Path p : paths) imported.put(p.getFileName().toString(), p.getFileName().toString().startsWith("a") ? "inA" : "inB");
    CameraGroups fromImport = CameraGroups.resolve(o, paths, imported, Map.of("inA", .004));
    check(fromImport.of("a1.jpg").id.equals("inA") && fromImport.of("b1.jpg").id.equals("inB"),
        "Imported camera IDs group the images first");
    near(fromImport.of("a1.jpg").pixelSizeMm, .004, 0, "Imported pixel size kept per camera");
    Map<String, String> perPhoto = new LinkedHashMap<>();
    for (Path p : paths) perPhoto.put(p.getFileName().toString(), "cam_" + p.getFileName());
    check(CameraGroups.resolve(o, paths, perPhoto, Map.of()).size() == 5,
        "One camera ID per photo carries no grouping: EXIF decides");
    Map<String, String> partial = new LinkedHashMap<>(imported);
    partial.remove("b1.jpg");
    rejects(() -> CameraGroups.resolve(o, paths, partial, Map.of()), "Photo without an imported camera is an error");

    o.intrinsicsModes = Map.of("nosuch", "fixed");
    String message = rejects(() -> CameraGroups.resolve(o, paths, Map.of(), Map.of()), "Unknown camera in CAMERA=");
    check(message.contains("nosuch"), "Unknown camera named in the error");
    o.intrinsicsModes = Map.of();

    String[] pair = Options.cameraValue("camA=fixed");
    check(pair != null && pair[0].equals("camA") && pair[1].equals("fixed"), "CAMERA=VALUE parsed");
    check(Options.cameraValue("C:\\cal=x.txt") == null && Options.cameraValue("0.0045") == null,
        "Plain values and drive paths are not camera forms");
    Options parsed =
        Options.parse(
            new String[] {
              "-i", images.toString(), "-o", dir.resolve("p").toString(), "--intrinsics-mode", "camA=prior",
              "--intrinsics-mode", "camB=free", "--orientation-pixel-size", "camA=0.004", "--orientation-pixel-size", "0.005",
              "--camera-grouping", "per-image"
            });
    check(parsed.intrinsicsModes.equals(Map.of("camA", "prior", "camB", "free")) && parsed.intrinsicsMode.equals("fixed"),
        "Per-camera intrinsics modes parsed beside the global one");
    check(parsed.orientationPixelSizes.get("camA") == .004 && parsed.orientationPixelSize == .005, "Per-camera pixel size");
    rejects(
        () -> Options.parse(new String[] {"-i", images.toString(), "-o", dir.resolve("q").toString(), "--intrinsics-mode", "camA=wild"}),
        "Unknown per-camera mode rejected");

    // State trailer round trip.
    CameraGroups.Group a = g.get(0);
    a.calibration = new Calibration(50, 50, 31.5, 23.5, new double[] {-.1, 0, 0, 0, 0}, 64, 48);
    a.supplied = true;
    a.initialization = "supplied_file";
    ByteArrayOutputStream bytes = new ByteArrayOutputStream();
    try (DataOutputStream out = new DataOutputStream(bytes)) {
      g.write(out);
    }
    CameraGroups back = CameraGroups.read(new DataInputStream(new ByteArrayInputStream(bytes.toByteArray())));
    check(back.size() == g.size() && back.get(0).supplied && back.get(0).calibration.fx == 50
            && back.of("e1.jpg").id.equals(g.of("e1.jpg").id),
        "Camera groups survive the state file");
  }

  // ---- The silent rescale: same aspect ratio, other size and focal.

  private static void silentRescale(Path dir) throws Exception {
    Path images = dir.resolve("images");
    photo(images, "big.jpg", 640, 480, "Acme", "Wide-Cam", null, 8.8);
    photo(images, "small.jpg", 480, 360, "Acme", "Wide-Cam", null, 8.8);
    Options o = options(images, dir.resolve("out"));
    List<Path> paths = Main.images(o);
    CameraGroups groups = CameraGroups.resolve(o, paths, Map.of(), Map.of());
    check(groups.size() == 2, "Same camera model, same aspect ratio, other size: two groups");
    o.cameraGroups = groups;
    Calibration big = new Calibration(420, 420, 319.5, 239.5, new double[5], 640, 480);
    String message =
        rejects(
            () -> new ImageFrame(1, images.resolve("small.jpg"), big, o, true).close(),
            "A calibration for 640x480 must not be rescaled onto a 480x360 camera");
    check(message.contains("small.jpg") && message.contains("640x480"), "The error names the photo and sizes: " + message);
    Calibration small = new Calibration(300, 300, 239.5, 179.5, new double[5], 480, 360);
    try (ImageFrame f = new ImageFrame(1, images.resolve("small.jpg"), small, o, true)) {
      check(f.group == 1, "Frame knows its camera group");
      near(f.camera.fx, 300 * f.width / 480., 1e-9, "Each camera keeps its own focal");
    }
    // Through the calibrate step: --calibration CAMERA=FILE per camera, no rescaling between them.
    Path a = dir.resolve("camera-big.txt"), b = dir.resolve("camera-small.txt");
    big.write(a, "big");
    small.write(b, "small");
    Path project = dir.resolve("project");
    int result =
        steps(
            "inputs", "calibrate", "-i", images.toString(), "-o", project.toString(), "--max-dimension", "320",
            "--calibration", "camera_1=" + a, "--calibration", "camera_2=" + b, "--cores", "1");
    check(result == 0, "calibrate step with two cameras");
    Path run = latestRun(project);
    Map<String, Object> cameras = JsonRead.object(JsonRead.read(run.resolve("01_calibration/cameras.json")));
    List<Object> list = JsonRead.array(cameras.get("cameras"));
    check(list.size() == 2, "cameras.json lists both cameras");
    near(JsonRead.number(JsonRead.object(JsonRead.object(list.get(1)).get("calibration")).get("fx")), 300, 0,
        "Second camera keeps its own focal (not 420 x 0.75)");
    check(Files.isRegularFile(run.resolve("camera_camera_1.txt")) && Files.isRegularFile(run.resolve("camera_camera_2.txt")),
        "camera_ID.txt per camera");
    String one =
        rejects(
            () -> {
              int r =
                  steps(
                      "inputs", "calibrate", "-i", images.toString(), "-o", dir.resolve("p2").toString(),
                      "--calibration", a.toString());
              if (r != 0) throw new IllegalArgumentException("exit " + r);
            },
            "One --calibration FILE cannot describe two cameras");
    check(one != null, "Single calibration with two cameras fails clearly");
  }

  /** Pipeline step names first, then the frozen regression flags of TestSupport, then the rest. */
  static int steps(String... args) {
    List<String> names = new ArrayList<>(), rest = new ArrayList<>();
    for (String a : args) (Pipeline.Step.of(a) != null && rest.isEmpty() ? names : rest).add(a);
    List<String> all = new ArrayList<>(names);
    all.addAll(List.of("--pair-selection", "sequential", "--compute", "cpu", "--storage", "debug"));
    all.addAll(rest);
    return Main.run(all.toArray(String[]::new));
  }

  static Path latestRun(Path project) throws IOException {
    try (var runs = Files.list(project.resolve("runs"))) {
      return runs.filter(Files::isDirectory).sorted().reduce((x, y) -> y).orElseThrow();
    }
  }

  // ---- INPHO: two cameras, two calibration sets, different pixel sizes.

  static final String PRJ =
      """
      $PROJECT 5.0.0
        $LINEAR_UNITS_OF_OBJECT : m
        $LINEAR_UNITS_OF_IMAGE : mm
        $ANGULAR_UNITS : deg
      $END
      $CAMERA_DEFINITION
        $ID : CamA
        $KIND : CCDFrame
        $CCD_COLUMNS : 640
        $CCD_ROWS : 480
        $PIXEL_REFERENCE : CenterTopLeft
        $ACTIVE_CALIBRATION : 2
        $CALIBRATION_SET :
          $ID : 1
          $FOCAL_LENGTH : 1.5
          $PRINCIPAL_POINT_PPA : 0.4 0.4
          $CCD_INTERIOR_ORIENTATION :
            250 0 319.5
            0 -250 239.5
        $END
        $CALIBRATION_SET :
          $ID : 2
          $FOCAL_LENGTH : 1.68
          $PRINCIPAL_POINT_PPA : 0.012 -0.008
          $CCD_INTERIOR_ORIENTATION :
            250 0 319.5
            0 -250 239.5
        $END
      $END
      $CAMERA_DEFINITION
        $ID : CamB
        $KIND : CCDFrame
        $CCD_COLUMNS : 480
        $CCD_ROWS : 360
        $PIXEL_REFERENCE : CenterTopLeft
        $ACTIVE_CALIBRATION : 1
        $CALIBRATION_SET :
          $ID : 1
          $FOCAL_LENGTH : 1.8
          $PRINCIPAL_POINT_PPA : 0 0
          $CCD_INTERIOR_ORIENTATION :
            166.66666666666666 0 239.5
            0 -166.66666666666666 179.5
        $END
      $END
      """;

  static String photoBlock(int n, String file, String camera, double f, double[] c) {
    return "$PHOTO\n  $PHOTO_NUM : " + n + "\n  $PHOTO_FILE : " + file + "\n"
        + (camera == null ? "" : "  $CAMERA_ID : " + camera + "\n")
        + "  $EXT_ORI : 00:00:00 01/01/2000\n    " + f + " " + c[0] + " " + c[1] + " " + c[2]
        + "\n    1 0 0\n    0 1 0\n    0 0 1\n$END\n";
  }

  private static void inpho(Path dir) throws Exception {
    Files.createDirectories(dir);
    StringBuilder prj = new StringBuilder(PRJ);
    for (int i = 0; i < 3; i++) prj.append(photoBlock(i + 1, "a" + i + ".tif", "CamA", 1.68, new double[] {i, 0, 100}));
    for (int i = 0; i < 3; i++) prj.append(photoBlock(i + 4, "b" + i + ".tif", "CamB", 1.8, new double[] {i, 5, 100}));
    Path file = dir.resolve("two-cameras.prj");
    Files.writeString(file, prj);
    Options o = TestSupport.options();
    o.output = dir;
    OrientationIO.Model m = OrientationIO.read(file, "inpho", Map.of(), o);
    check(m.views.size() == 6, "Six photos imported");
    OrientationIO.View a = m.views.get("a0.tif"), b = m.views.get("b2.tif");
    check("CamA".equals(a.camera()) && "CamB".equals(b.camera()), "Camera IDs kept from $CAMERA_ID");
    near(a.calibration().fx, 1.68 * 250, 1e-9, "Camera A: focal / its pixel size (0.004 mm)");
    near(b.calibration().fx, 1.8 / .006, 1e-6, "Camera B: focal / its own pixel size (0.006 mm)");
    near(a.calibration().cx, 319.5 + .012 * 250, 1e-9, "Active calibration set 2 of camera A (PPA x)");
    near(a.calibration().cy, 239.5 + .008 * 250, 1e-9, "Active calibration set 2 of camera A (PPA y)");
    check(a.calibration().width == 640 && b.calibration().width == 480, "Each camera's CCD size");
    near(m.pixelSizes.get("CamA"), .004, 1e-12, "Pixel size per camera");
    near(m.pixelSizes.get("CamB"), .006, 1e-12, "Pixel size per camera B");
    check(m.grouped(), "Two cameras: grouped model");

    // Export -> re-import keeps the cameras, names, pixel sizes and calibrations.
    Options e = TestSupport.options();
    e.output = dir;
    e.orientationOutput = dir.resolve("export");
    e.orientationOutputFormat = "inpho";
    e.orientationUnits = "m";
    OrientationIO.export(m, e);
    Path out = dir.resolve("export/inpho/orientations.prj");
    String text = Files.readString(out);
    check(text.split("\\$CAMERA_DEFINITION", -1).length - 1 == 2, "One $CAMERA_DEFINITION per camera");
    check(text.contains("$ID : CamA") && text.contains("$ID : CamB"), "Imported camera names kept");
    OrientationIO.Model again = OrientationIO.read(out, "inpho", Map.of(), o);
    for (String image : m.views.keySet()) {
      OrientationIO.View x = m.views.get(image), y = again.views.get(image);
      check(Objects.equals(x.camera(), y.camera()), "Round trip keeps the camera of " + image);
      near(y.calibration().fx, x.calibration().fx, 1e-6, "Round trip focal " + image);
      near(y.calibration().cx, x.calibration().cx, 1e-6, "Round trip cx " + image);
      near(y.calibration().cy, x.calibration().cy, 1e-6, "Round trip cy " + image);
      near(Math3.norm(Math3.sub(y.pose().center(), x.pose().center())), 0, 1e-9, "Round trip centre " + image);
    }
    near(again.pixelSizes.get("CamB"), .006, 1e-12, "Round trip keeps camera B's pixel size");

    // A photo without a camera reference, or an unknown one, is an error.
    Path missing = dir.resolve("missing.prj");
    Files.writeString(missing, PRJ + photoBlock(1, "a0.tif", "CamA", 1.68, new double[] {0, 0, 100})
        + photoBlock(2, "x.tif", null, 1.68, new double[] {1, 0, 100}));
    String message = rejects(() -> OrientationIO.read(missing, "inpho", Map.of(), o), "Photo without $CAMERA_ID");
    check(message.contains("x.tif") && message.contains("$CAMERA_ID"), "Error names the photo: " + message);
    Path unknown = dir.resolve("unknown.prj");
    Files.writeString(unknown, PRJ + photoBlock(1, "a0.tif", "CamZ", 1.68, new double[] {0, 0, 100}));
    message = rejects(() -> OrientationIO.read(unknown, "inpho", Map.of(), o), "Unknown camera");
    check(message.contains("CamZ"), "Error names the unknown camera");

    // --orientation-pixel-size CAMERA=MM applies to that camera only (when the file states none).
    Path bare = dir.resolve("bare.prj");
    Files.writeString(
        bare,
        "$PROJECT 5.0.0\n$END\n$CAMERA_DEFINITION\n  $ID : CamC\n  $CCD_COLUMNS : 100\n  $CCD_ROWS : 80\n$END\n"
            + photoBlock(1, "c.tif", "CamC", 2.0, new double[] {0, 0, 10}));
    Options pixel = TestSupport.options();
    pixel.orientationPixelSizes = Map.of("CamC", .005);
    near(OrientationIO.read(bare, "inpho", Map.of(), pixel).views.get("c.tif").calibration().fx, 400, 1e-9,
        "Per-camera pixel size used on import");
  }

  // ---- Round trips through the other multi-camera formats.

  static OrientationIO.Model twoCameraModel() {
    OrientationIO.Model m = new OrientationIO.Model();
    m.units = "m";
    Calibration ca = new Calibration(420, 420, 322.25, 238.75, new double[] {-.08, .01, 0, 0, 0}, 640, 480);
    Calibration cb = new Calibration(300, 300, 239.5, 179.5, new double[] {.05, 0, 0, 0, 0}, 480, 360);
    for (int i = 0; i < 4; i++) {
      Camera p = OrientationIO.centered(Math3.rotation(new double[] {.01 * i, -.02, .03}), new double[] {i, .5 * i, 30});
      m.add(new OrientationIO.View("img" + i + ".jpg", i % 2 == 0 ? ca : cb, p, i % 2 == 0 ? "camA" : "camB"));
    }
    return m;
  }

  private static void roundTrips(Path dir) throws Exception {
    OrientationIO.Model m = twoCameraModel();
    Options e = TestSupport.options();
    e.output = dir;
    e.orientationOutput = dir.resolve("export");
    e.orientationOutputFormat = "metashape,colmap,openmvg,opensfm,csv,native,bundler";
    e.orientationAllowLossy = true;
    OrientationIO.export(m, e);
    Map<String, int[]> sizes = new HashMap<>();
    for (var v : m.views.values()) sizes.put(v.image(), new int[] {v.calibration().width, v.calibration().height});
    check(Files.readString(dir.resolve("export/metashape/cameras.xml")).split("<sensor ", -1).length - 1 == 2,
        "Metashape: one sensor per camera");
    check(Files.readAllLines(dir.resolve("export/colmap/cameras.txt")).stream().filter(l -> !l.startsWith("#")).count() == 2,
        "COLMAP: one camera per group");
    for (String format : List.of("metashape", "colmap", "openmvg", "opensfm", "csv", "native", "bundler")) {
      Options o = TestSupport.options();
      o.orientationAllowLossy = true;
      OrientationIO.Model back = OrientationIO.read(dir.resolve("export/" + format), format, sizes, o);
      Map<String, String> pairing = new HashMap<>();
      for (var v : m.views.values()) {
        OrientationIO.View w = back.views.get(v.image());
        check(w != null && w.camera() != null, format + ": camera ID read back for " + v.image());
        String old = pairing.putIfAbsent(v.camera(), w.camera());
        check(old == null || old.equals(w.camera()), format + ": cameras keep their grouping");
        if (!format.equals("bundler")) {
          near(w.calibration().fx, v.calibration().fx, 1e-6, format + ": focal of " + v.image());
          near(w.calibration().cx, v.calibration().cx, 1e-6, format + ": cx of " + v.image());
          near(w.calibration().distortion[0], v.calibration().distortion[0], 1e-12, format + ": k1 of " + v.image());
        }
        near(Math3.norm(Math3.sub(w.pose().center(), v.pose().center())), 0, 1e-6, format + ": centre of " + v.image());
      }
      check(new HashSet<>(pairing.values()).size() == 2, format + ": two distinct cameras after the round trip");
      check(pairing.get("camA").equals("camA") && pairing.get("camB").equals("camB"),
          format + ": camera names kept " + pairing);
    }
    check(Files.isRegularFile(dir.resolve("export/colmap/camera_groups.csv")), "Grouped exports carry camera_groups.csv");
    // One camera: the per-photo layout of earlier versions, no camera names written.
    OrientationIO.Model single = new OrientationIO.Model();
    single.units = "m";
    for (var v : m.views.values())
      if (v.camera().equals("camA")) single.add(new OrientationIO.View(v.image(), v.calibration(), v.pose()));
    Options s = TestSupport.options();
    s.output = dir;
    s.orientationOutput = dir.resolve("single");
    s.orientationOutputFormat = "colmap,metashape";
    OrientationIO.export(single, s);
    check(Files.readAllLines(dir.resolve("single/colmap/cameras.txt")).size() == 2
            && !Files.exists(dir.resolve("single/colmap/camera_groups.csv"))
            && !Files.readString(dir.resolve("single/metashape/cameras.xml")).contains("label=\"camA\""),
        "Single-camera exports unchanged: one entry per photo, no camera names");
  }

  // ---- A run saved before camera groups: calibration.bin without the trailer.

  private static void legacyState(Path dir) throws Exception {
    Files.createDirectories(dir.resolve("state"));
    Path file = dir.resolve("state/calibration.bin");
    try (DataOutputStream out = new DataOutputStream(Files.newOutputStream(file))) {
      out.writeBoolean(true);
      PipelineState.writeCalibration(out, new Calibration(500, 500, 319.5, 239.5, new double[5], 640, 480));
      out.writeInt(0);
    }
    try (DataInputStream in = new DataInputStream(new BufferedInputStream(Files.newInputStream(file)))) {
      in.readBoolean();
      PipelineState.readCalibration(in);
      in.readInt();
      check(CameraGroups.read(in) == null, "Old calibration.bin: no camera groups (one camera)");
    }
    CameraGroups legacy = CameraGroups.legacy(List.of("x.jpg", "y.jpg"), new Calibration(500, 500, 319.5, 239.5, new double[5], 640, 480));
    check(!legacy.multiple() && legacy.index("y.jpg") == 0 && legacy.get(0).calibration.fx == 500,
        "Old run loads as one camera");
  }

  // ---- Synthetic two-camera block with known lenses and poses.

  record Lens(int width, int height, double f, double cx, double cy, double k1) {}

  static final Lens A = new Lens(640, 480, 430, 325.5, 236.5, -.07), B = new Lens(512, 384, 300, 255.5, 191.5, .05);

  static Camera truth(int i) {
    int column = i % 5, row = i / 5;
    double[] centre = {-1.0 + .5 * column + (row == 1 ? .12 : 0), -.35 + .7 * row, .08 * Math.sin(i * 1.3)};
    Camera c = new Camera(1, 1, 0, 0);
    c.rotation = Math3.rotation(new double[] {.05 * Math.sin(i), .06 * Math.cos(1.7 * i), .04 * Math.sin(2.3 * i)});
    c.translation = Math3.times(Math3.mv(c.rotation, centre), -1);
    return c;
  }

  static boolean cameraA(int i) {
    return (i % 5 + i / 5) % 2 == 0;
  }

  /** Renders the stepped three-plane scene of GeoCalibrationRegression through a distorted lens. */
  static void render(Path file, Camera pose, Lens lens, byte[][] textures, int tw, int th) throws IOException {
    int w = lens.width, h = lens.height;
    double[] rt = Math3.transpose(pose.rotation), center = pose.center();
    byte[] color = new byte[w * h * 3];
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) {
        // Undistort the pixel's normalized coordinates: x_d = x (1 + k1 r^2).
        double xd = (x - lens.cx) / lens.f, yd = (y - lens.cy) / lens.f, xu = xd, yu = yd;
        for (int k = 0; k < 20; k++) {
          double radial = 1 + lens.k1 * (xu * xu + yu * yu);
          xu = xd / radial;
          yu = yd / radial;
        }
        double[] ray = Math3.mv(rt, new double[] {xu, yu, 1});
        for (int k = 0; k < 3; k++) {
          double depth = (new double[] {4, 7, 11}[k] - center[2]) / ray[2],
              wx = center[0] + depth * ray[0],
              wy = center[1] + depth * ray[1];
          if (depth <= 0
              || (k == 0 && wx >= -.8)
              || (k == 1 && (wx < -.8 || wx > .8))
              || (k == 2 && wx <= .8)) continue;
          double scale = new double[] {90, 50, 32}[k];
          double tx = wx * scale + tw / 2., ty = wy * scale + th / 2.;
          int ix = (int) Math.floor(tx), iy = (int) Math.floor(ty);
          if (ix < 0 || iy < 0 || ix >= tw - 1 || iy >= th - 1) break;
          double ax = tx - ix, ay = ty - iy;
          byte[] texture = textures[k];
          for (int c = 0; c < 3; c++) {
            int p = (iy * tw + ix) * 3 + c;
            double v =
                (texture[p] & 255) * (1 - ax) * (1 - ay)
                    + (texture[p + 3] & 255) * ax * (1 - ay)
                    + (texture[p + tw * 3] & 255) * (1 - ax) * ay
                    + (texture[p + tw * 3 + 3] & 255) * ax * ay;
            color[(y * w + x) * 3 + c] = (byte) Math.round(v);
          }
          break;
        }
      }
    Mat image = new Mat(h, w, CvType.CV_8UC3);
    image.put(0, 0, color);
    try {
      if (!Imgcodecs.imwrite(file.toString(), image)) throw new IOException("Cannot save fixture");
    } finally {
      image.release();
    }
  }

  static byte[][] textures(int tw, int th) {
    byte[][] textures = new byte[3][];
    for (int k = 0; k < 3; k++) {
      Mat texture = new Mat(th, tw, CvType.CV_8UC3);
      byte[] v = new byte[tw * th * 3];
      new Random(731 + k).nextBytes(v);
      texture.put(0, 0, v);
      Imgproc.GaussianBlur(texture, texture, new Size(3, 3), .6);
      texture.get(0, 0, v);
      textures[k] = v;
      texture.release();
    }
    return textures;
  }

  /** Renders the block; camera A images only when {@code onlyA}. Returns the image folder. */
  static Path renderBlock(Path dir, boolean onlyA) throws IOException {
    Path images = dir.resolve(onlyA ? "camera A" : "two cameras");
    Files.createDirectories(images);
    int tw = 1400, th = 1000;
    byte[][] textures = textures(tw, th);
    for (int i = 0; i < 10; i++) {
      if (onlyA && !cameraA(i)) continue;
      render(images.resolve(String.format(Locale.ROOT, "%02d.png", i)), truth(i), cameraA(i) ? A : B, textures, tw, th);
    }
    return images;
  }

  static String[] blockArguments(Path images, Path project, String... more) {
    List<String> a =
        new ArrayList<>(
            List.of(
                "-i", images.toString(), "-o", project.toString(), "--max-dimension", "640", "--disparities", "96",
                "--step", "6", "--cores", "2", "--features", "4000", "--neighbors", "4", "--no-gps"));
    a.addAll(Arrays.asList(more));
    return a.toArray(String[]::new);
  }

  private static void block(Path dir) throws Exception {
    Files.createDirectories(dir);
    Path images = renderBlock(dir, false);
    Path csv = dir.resolve("cameras.csv");
    StringBuilder rows = new StringBuilder("image,camera\n");
    for (int i = 0; i < 10; i++) rows.append(String.format(Locale.ROOT, "%02d.png,%s\n", i, cameraA(i) ? "camA" : "camB"));
    Files.writeString(csv, rows);
    Path project = dir.resolve("project");
    int result = TestSupport.run(blockArguments(images, project, "--camera-groups", csv.toString()));
    check(result == 0, "Two-camera block processed end to end");
    Path run = latestRun(project);
    Map<String, Object> json = JsonRead.object(JsonRead.read(run.resolve("01_calibration/cameras.json")));
    Map<String, Map<String, Object>> byId = new HashMap<>();
    for (Object c : JsonRead.array(json.get("cameras"))) {
      Map<String, Object> m = JsonRead.object(c);
      byId.put(m.get("id").toString(), JsonRead.object(m.get("calibration")));
    }
    check(byId.keySet().equals(Set.of("camA", "camB")), "Both cameras in cameras.json: " + byId.keySet());
    double fa = JsonRead.number(byId.get("camA").get("fx")), fb = JsonRead.number(byId.get("camB").get("fx"));
    double ka = JsonRead.number(JsonRead.array(byId.get("camA").get("distortion")).get(0)),
        kb = JsonRead.number(JsonRead.array(byId.get("camB").get("distortion")).get(0));
    System.out.printf(Locale.ROOT, "Two-camera block: camA f=%.2f (true %.1f) k1=%.4f (true %.3f); camB f=%.2f (true %.1f) k1=%.4f (true %.3f)%n",
        fa, A.f, ka, A.k1, fb, B.f, kb, B.k1);
    near(fa / A.f, 1, .03, "Camera A focal recovered");
    near(fb / B.f, 1, .03, "Camera B focal recovered (not A's rescaled 344 px)");
    near(ka, A.k1, .03, "Camera A radial distortion recovered");
    near(kb, B.k1, .03, "Camera B radial distortion recovered");
    // Poses: camera centres up to a similarity.
    List<double[]> estimated = new ArrayList<>(), known = new ArrayList<>();
    List<String> lines = Files.readAllLines(project.resolve("cameras.csv"));
    for (int i = 1; i < lines.size(); i++) {
      List<String> v = PoseIO.csv(lines.get(i));
      int id = Integer.parseInt(v.get(0).substring(0, 2));
      double[] r = new double[9], t = new double[3];
      for (int k = 0; k < 9; k++) r[k] = Double.parseDouble(v.get(k + 1));
      for (int k = 0; k < 3; k++) t[k] = Double.parseDouble(v.get(k + 10));
      estimated.add(Math3.times(Math3.mv(Math3.transpose(r), t), -1));
      known.add(truth(id).center());
    }
    check(estimated.size() == 10, "All ten cameras registered: " + estimated.size());
    Georeference.Similarity s = Georeference.fit(estimated, known);
    double square = 0;
    for (int i = 0; i < estimated.size(); i++)
      square += Math.pow(Math3.norm(Math3.sub(s.apply(estimated.get(i)), known.get(i))), 2);
    double rms = Math.sqrt(square / estimated.size());
    System.out.printf(Locale.ROOT, "Two-camera block: camera centre RMS after similarity %.4f (block 2.1 units wide)%n", rms);
    near(rms, 0, .02, "Camera centres recovered (RMS after similarity, block 2.1 units)");
    String shared = Files.readString(run.resolve("04_geometry/bundle/shared_calibration.json"));
    check(shared.contains("\"cameras\"") && shared.contains("camA") && shared.contains("camB"),
        "shared_calibration.json has one section per camera");
    check(Files.isRegularFile(run.resolve("camera_camA.txt")) && Files.isRegularFile(run.resolve("camera_camB.txt")),
        "camera_ID.txt per camera");
    check(Files.size(run.resolve("07_results/dense.las")) > 1000 || Files.size(project.resolve("dense.las")) > 1000,
        "Dense cloud from both cameras");
    // Exports keep the two cameras.
    String model = Files.readString(run.resolve("04_geometry/orientation_model.json"));
    check(model.contains("\"camera\":\"camA\"") && model.contains("\"camera\":\"camB\""), "Orientation archive keeps camera IDs");

    // One camera: auto grouping equals --camera-grouping single, file for file.
    Path onlyA = renderBlock(dir, true);
    Path auto = dir.resolve("single auto"), forced = dir.resolve("single forced");
    check(TestSupport.run(blockArguments(onlyA, auto)) == 0, "Single-camera block (auto)");
    check(TestSupport.run(blockArguments(onlyA, forced, "--camera-grouping", "single")) == 0, "Single-camera block (single)");
    Path ra = latestRun(auto), rf = latestRun(forced);
    for (String f : List.of("07_results/dense.ply", "04_geometry/final/orientations.json", "04_geometry/final/interior.csv"))
      if (Files.isRegularFile(ra.resolve(f)))
        check(Files.mismatch(ra.resolve(f), rf.resolve(f)) == -1, "One camera: identical " + f);
    check(!Files.exists(ra.resolve("camera_camera_1.txt")), "One camera writes no camera_ID.txt");
    trialMethods(dir, onlyA);
  }

  /** --calibration-trial-method subset (4 of the 5 cameras) and joint recover camera A's lens. */
  private static void trialMethods(Path dir, Path images) throws Exception {
    for (String[] variant : new String[][] {
        {"subset", "--calibration-trial-method", "subset", "--calibration-trial-cameras", "4"},
        {"joint", "--calibration-trial-method", "joint"}}) {
      Path project = dir.resolve("trials " + variant[0]);
      check(TestSupport.run(blockArguments(images, project, Arrays.copyOfRange(variant, 1, variant.length))) == 0,
          "Single-camera block with --calibration-trial-method " + variant[0]);
      Path run = latestRun(project);
      Map<String, Object> report = JsonRead.object(JsonRead.read(run.resolve("04_geometry/bundle/shared_calibration.json")));
      check(variant[0].equals(report.get("trial_method")), variant[0] + ": method recorded");
      if (variant[0].equals("subset")) {
        Map<String, Object> subset = JsonRead.object(report.get("subset"));
        check(JsonRead.number(subset.get("cameras")) == 4, "subset: trials on 4 cameras: " + subset);
        check(!Boolean.TRUE.equals(report.get("accepted")) || report.get("whole_block_check") != null,
            "subset: an accepted model passed the whole-block check");
      } else
        check(JsonRead.array(report.get("stages")).stream().allMatch(s -> "joint".equals(JsonRead.object(s).get("parameter"))),
            "joint: one joint fit, no parameter-wise trials");
      Options load = TestSupport.options();
      load.calibration = project.resolve("camera.txt");
      Calibration c = Calibration.load(load, A.width(), A.height());
      System.out.printf(Locale.ROOT, "Trial method %s: f=%.2f (true %.1f) k1=%.4f (true %.3f); accepted %s%n",
          variant[0], c.fx, A.f(), c.distortion[0], A.k1(), report.get("accepted_parameters"));
      near(c.fx / A.f(), 1, .03, variant[0] + ": focal recovered");
      near(c.distortion[0], A.k1(), .03, variant[0] + ": radial distortion recovered");
    }
  }
}
