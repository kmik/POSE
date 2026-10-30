package sgm;

import static sgm.OrientationIO.*;

import java.io.*;
import java.nio.*;
import java.nio.file.*;
import java.util.*;

/**
 * Round trips plus independently specified conventions, strict losses and completed-run recovery.
 */
final class OrientationRegression {
  static int checks;

  static void check(boolean yes, String text) {
    checks++;
    if (!yes) throw new AssertionError(text);
  }

  static void near(double a, double b, String text) {
    check(Math.abs(a - b) < 1e-8, text + ": " + a + " != " + b);
  }

  interface Action {
    void run() throws Exception;
  }

  static void rejects(Action a, String text) throws Exception {
    boolean rejected = false;
    try {
      a.run();
    } catch (IllegalArgumentException | IOException | NullPointerException e) {
      rejected = true;
    }
    check(rejected, text);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    if (args.length > 0) {
      Model read = read(Path.of(args[0]), args[1], Map.of(), new Options());
      Model expected = model(args[1].equals("micmac"), false);
      expected.views.keySet().retainAll(read.views.keySet());
      compare(expected, read, true);
      System.out.println("PASS relative orientation checks=" + checks);
      return;
    }
    Path root = Files.createTempDirectory(Path.of("target"), "orientation test ");
    System.out.println("PASS orientations=" + run(root) + "; " + root);
  }

  static Model model(boolean radial, boolean centred) {
    Model m = new Model();
    m.crs = "EPSG:3067";
    m.units = "metres";
    for (int i = 0; i < 2; i++)
      m.add(
          new View(
              "image" + i + ".jpg",
              calibration(
                  800,
                  radial ? 800 : 812,
                  centred ? 320 : 317.25,
                  centred ? 240 : 238.75,
                  radial
                      ? new double[] {-.08, .02, 0, 0, .001}
                      : new double[] {-.08, .02, .0003, -.0004, .001},
                  640,
                  480),
              centered(
                  Math3.rotation(new double[] {.1 + i * .03, -.2, .07}),
                  new double[] {400000 + i, 7200000 - i, 120 + i})));
    return m;
  }

  static void compare(Model a, Model b, boolean distortion) {
    check(a.views.keySet().equals(b.views.keySet()), "Image associations survive conversion");
    for (String key : a.views.keySet()) {
      View x = a.views.get(key), y = b.views.get(key);
      Calibration c = x.calibration(), d = y.calibration();
      check(c.width == d.width && c.height == d.height, "Dimensions");
      near(c.fx, d.fx, "fx");
      near(c.fy, d.fy, "fy");
      near(c.cx, d.cx, "cx");
      near(c.cy, d.cy, "cy");
      if (distortion)
        for (int i = 0; i < 5; i++) near(c.distortion[i], d.distortion[i], "distortion " + i);
      for (int i = 0; i < 9; i++) near(x.pose().rotation[i], y.pose().rotation[i], "R " + i);
      double[] cx = x.pose().center(), cy = y.pose().center();
      for (int i = 0; i < 3; i++) near(cx[i], cy[i], "centre " + i);
    }
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Path base = Files.createDirectories(root.resolve("orientations"));
    Options o = TestSupport.options();
    o.output = base;
    for (String format :
        List.of(
            "native",
            "csv",
            "csv-opk",
            "colmap",
            "opensfm",
            "metashape",
            "opencv",
            "openmvg",
            "micmac",
            "bundler",
            "inpho")) {
      Model m =
          model(
              format.equals("openmvg")
                  || format.equals("micmac")
                  || format.equals("bundler")
                  || format.equals("inpho"),
              format.equals("bundler"));
      if (format.equals("bundler"))
        for (View v : m.views.values()) v.calibration().distortion[4] = 0;
      if (format.equals("inpho"))
        for (View v : m.views.values()) Arrays.fill(v.calibration().distortion, 0);
      Options export = o.copy();
      export.orientationOutput = base.resolve(format);
      export.orientationOutputFormat = format;
      export.orientationPixelSize = .004;
      OrientationIO.export(m, export);
      Path folder = export.orientationOutput.resolve(format);
      Model imported = read(folder, format.equals("csv-opk") ? "csv-opk" : "auto", Map.of(), o);
      compare(m, imported, true);
      check(
          Files.isRegularFile(folder.resolve("conventions.json")),
          "Convention sidecar for " + format);
    }
    Path colmap = Files.createDirectories(base.resolve("independent-colmap"));
    Files.writeString(
        colmap.resolve("cameras.txt"), "7 OPENCV 640 480 800 810 320 240 -0.1 0.01 0.003 -0.004\n");
    Files.writeString(
        colmap.resolve("images.txt"),
        "2 0.7071067811865476 0 0 0.7071067811865476 2 3 4 7 external.jpg\n\n");
    View external = read(colmap, "auto", Map.of(), o).views.get("external.jpg");
    near(external.calibration().cx, 319.5, "COLMAP half-pixel conversion");
    near(external.pose().rotation[1], -1, "COLMAP quaternion is world-to-camera");
    near(external.pose().center()[0], -3, "COLMAP centre from translation");
    Path binary = Files.createDirectories(base.resolve("binary-colmap"));
    ByteBuffer cameras =
        ByteBuffer.allocate(8 + 4 + 4 + 8 + 8 + 8 * 3).order(ByteOrder.LITTLE_ENDIAN);
    cameras
        .putLong(1)
        .putInt(7)
        .putInt(0)
        .putLong(640)
        .putLong(480)
        .putDouble(800)
        .putDouble(320)
        .putDouble(240);
    Files.write(binary.resolve("cameras.bin"), cameras.array());
    ByteBuffer image =
        ByteBuffer.allocate(8 + 4 + 7 * 8 + 4 + 13 + 8).order(ByteOrder.LITTLE_ENDIAN);
    image
        .putLong(1)
        .putInt(2)
        .putDouble(1)
        .putDouble(0)
        .putDouble(0)
        .putDouble(0)
        .putDouble(2)
        .putDouble(3)
        .putDouble(4)
        .putInt(7)
        .put("external.jpg".getBytes(java.nio.charset.StandardCharsets.UTF_8))
        .put((byte) 0)
        .putLong(0);
    Files.write(binary.resolve("images.bin"), image.array());
    View bin = read(binary, "auto", Map.of(), o).views.get("external.jpg");
    near(bin.calibration().fx, 800, "Binary COLMAP intrinsics");
    near(bin.pose().center()[0], -2, "Binary COLMAP translation");
    Path meta = base.resolve("independent-metashape.xml");
    Files.writeString(
        meta,
        """
<document><chunk><sensors><sensor id="0"><resolution width="640" height="480"/><calibration type="frame" class="adjusted"><f>800</f><cx>7.25</cx><cy>-3.5</cy><p1>0.003</p1><p2>-0.004</p2></calibration></sensor></sensors>
<cameras><camera label="external.jpg" sensor_id="0"><transform>1 0 0 1 0 1 0 2 0 0 1 3 0 0 0 1</transform></camera></cameras>
<transform><rotation>0 -1 0 1 0 0 0 0 1</rotation><translation>10 20 30</translation><scale>2</scale></transform></chunk></document>
""");
    View met = read(meta, "auto", Map.of(), o).views.get("external.jpg");
    near(met.calibration().cx, 326.75, "Metashape principal point offset and pixel origin");
    near(met.calibration().distortion[2], -.004, "Metashape tangential order");
    near(met.pose().rotation[1], 1, "Metashape world-to-camera conversion");
    near(met.pose().center()[0], 6, "Metashape chunk scale and translation");
    near(met.pose().center()[2], 36, "Metashape chunk Z");
    Path csv = base.resolve("zero-opk.csv");
    Files.writeString(csv, "image,x,y,z,omega,phi,kappa\nexternal.jpg,1,2,3,0,0,0\n");
    Camera opk = read(csv, "csv-opk", Map.of(), o).views.get("external.jpg").pose();
    near(opk.rotation[4], -1, "OPK photogrammetric Y");
    near(opk.rotation[8], -1, "OPK looks down negative world Z at zero angles");
    near(opk.center()[2], 3, "OPK centre");
    Path cv = base.resolve("independent-opencv.yml");
    Files.writeString(
        cv,
        """
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
    near(
        read(cv, "auto", Map.of(), o).views.get("*").calibration().distortion[2],
        .003,
        "Independent OpenCV YAML Brown coefficients");
    Path mic = base.resolve("independent-micmac.xml");
    Files.writeString(
        mic,
        """
<OrientationConique><Interne><KnownConv>eConvApero_DistM2C</KnownConv><PP>319.5 239.5</PP><F>800</F><SzIm>640 480</SzIm><CalibDistortion><ModRad><CDist>319.5 239.5</CDist><CoeffDist>-0.0000001</CoeffDist></ModRad></CalibDistortion></Interne>
<Externe><Centre>1 2 3</Centre><ParamRotation><CodageMatr><L1>0 -1 0</L1><L2>1 0 0</L2><L3>0 0 1</L3></CodageMatr></ParamRotation></Externe><NameIm>external.jpg</NameIm></OrientationConique>
""");
    View mm = read(mic, "micmac", Map.of(), o).views.get("external.jpg");
    near(mm.pose().rotation[1], 1, "MicMac matrix camera-to-world");
    near(
        mm.calibration().distortion[0],
        -.064,
        "MicMac pixel radial coefficients converted to normalized Brown");
    Model radial = model(true, false);
    Options strict = o.copy();
    strict.orientationOutput = base.resolve("loss-test");
    strict.orientationOutputFormat = "bundler";
    rejects(() -> OrientationIO.export(radial, strict), "Lossy Bundler export is opt-in");
    Path bad = base.resolve("bad.xml");
    Files.writeString(
        bad,
        "<!DOCTYPE a [<!ENTITY x SYSTEM"
            + " 'file:///no-such-secret'>]><opencv_storage>&x;</opencv_storage>");
    rejects(() -> read(bad, "opencv", Map.of(), o), "XML external entities disabled");
    rejects(
        () -> pose(new double[] {1, 0, 0, 0, 1, 0, 0, 0, -1}, new double[3]),
        "Reflected rotations rejected");
    rejects(() -> JsonRead.parse("{\"x\":1,\"x\":2}"), "Duplicate JSON keys rejected");
    Path mvg = base.resolve("independent-sfm_data.json");
    Files.writeString(
        mvg,
        """
{"sfm_data_version":"0.3","intrinsics":[{"key":3,"value":{"polymorphic_name":"pinhole_brown_t2","ptr_wrapper":{"data":{"width":640,"height":480,"focal_length":800,"principal_point":[319.5,239.5],"disto_t2":[-0.1,0.01,0.001,0.003,-0.004]}}}}],"views":[{"key":7,"value":{"ptr_wrapper":{"data":{"filename":"external.jpg","id_intrinsic":3,"id_pose":4}}}}],"extrinsics":[{"key":4,"value":{"rotation":[[0,-1,0],[1,0,0],[0,0,1]],"center":[1,2,3]}}]}
""");
    View mv = read(mvg, "auto", Map.of(), o).views.get("external.jpg");
    near(
        mv.calibration().distortion[2],
        .003,
        "OpenMVG Brown serializes all five terms in disto_t2");
    near(mv.calibration().distortion[4], .001, "OpenMVG Brown k3 order");
    near(mv.pose().center()[1], 2, "OpenMVG centre is not translation");
    Path inpho = base.resolve("inpho/inpho/orientations.prj");
    String prj =
        Files.readString(inpho)
            .replace(
                "$ACTIVE_CALIBRATION : 1",
                "$ACTIVE_CALIBRATION : 1\n"
                    + "  $CALIBRATION_SET :\n"
                    + "    $ID : 0\n"
                    + "    $RADIAL_COEFFS : 123 456\n"
                    + "  $END");
    Path active = base.resolve("active.prj");
    Files.writeString(active, prj);
    compare(read(inpho, "inpho", Map.of(), o), read(active, "inpho", Map.of(), o), true);
    Path run = Files.createDirectories(base.resolve("legacy-run"));
    Files.createDirectories(run.resolve("01_calibration"));
    Files.createDirectories(run.resolve("00_inputs"));
    Files.createDirectories(run.resolve("04_geometry"));
    Files.writeString(
        run.resolve("01_calibration/reference_camera.properties"),
        "width=640\nheight=480\nfx=800\nfy=800\ncx=319.5\ncy=239.5\nk1=-0.1\n");
    Files.writeString(
        run.resolve("00_inputs/images.csv"),
        "image_id,filename,original_path,bytes,sha256,original_width,original_height\n"
            + "0,external.jpg,no-original-files-needed,0,unused,640,480\n");
    Files.writeString(
        run.resolve("04_geometry/cameras.csv"),
        PoseIO.HEADER + "\nexternal.jpg,1,0,0,0,1,0,0,0,1,2,3,4\n");
    Files.writeString(
        run.resolve("01_calibration/processed_interior.csv"),
        "image_id,filename,width,height,fx,fy,cx,cy\n"
            + "0,external.jpg,320,240,400,400,159.75,119.75\n");
    Json.write(run.resolve("coordinates.json"), Json.object("horizontal_crs", "EPSG:3067"));
    Options post = o.copy();
    post.exportOrientations = run;
    post.orientationOutput = base.resolve("posthoc");
    post.orientationOutputFormat = "csv,colmap";
    exportRun(post);
    near(
        readRun(run, post).views.get("external.jpg").calibration().distortion[0],
        -.1,
        "Legacy source distortion recovered without photos");
    post.orientationPixels = "processed";
    near(
        readRun(run, post).views.get("external.jpg").calibration().fx,
        400,
        "Legacy processed K recovered");
    check(
        Files.isRegularFile(post.orientationOutput.resolve("colmap/images.txt")),
        "Completed-run exterior export");
    check(readRun(run, post).units.equals("metres"), "Legacy CRS export retains metre units");
    Path imageDir = Files.createDirectories(base.resolve("import-images"));
    List<Path> images = new ArrayList<>();
    org.opencv.core.Mat raster =
        new org.opencv.core.Mat(
            480, 640, org.opencv.core.CvType.CV_8UC3, new org.opencv.core.Scalar(80));
    try {
      for (int i = 0; i < 2; i++) {
        Path file = imageDir.resolve("image" + i + ".jpg");
        org.opencv.imgcodecs.Imgcodecs.imwrite(file.toString(), raster);
        images.add(file);
      }
    } finally {
      raster.release();
    }
    Model heterogeneous = model(false, false);
    View old = heterogeneous.views.get("image1.jpg");
    Calibration cc = old.calibration();
    heterogeneous.views.put(
        "image1.jpg",
        new View(
            old.image(), calibration(840, 850, cc.cx, cc.cy, cc.distortion, 640, 480), old.pose()));
    heterogeneous.notes.add("Import convention note preserved");
    Path source = base.resolve("heterogeneous.json");
    Json.write(source, json(heterogeneous));
    Options imported = o.copy();
    imported.output = base.resolve("import-project");
    imported.orientationInput = source;
    prepare(imported, images);
    String normalizedHash = ReuseCache.hash(imported.calibration);
    prepare(imported, images);
    check(
        normalizedHash.equals(ReuseCache.hash(imported.calibration))
            && Files.readAllLines(imported.calibration).stream()
                .noneMatch(line -> line.startsWith("#")),
        "Imported calibration excludes timestamps from reusable cache identity");
    Calibration reference = Calibration.load(imported, 640, 480);
    List<ImageFrame> frames = new ArrayList<>();
    try {
      for (int i = 0; i < 2; i++) frames.add(new ImageFrame(i, images.get(i), reference, imported));
      PoseIO.read(imported.poses, frames);
      near(frames.get(1).camera.fx, 840, "Distinct imported calibration reaches image processing");
      archive(frames, imported, "EPSG:3067");
      Model saved = readRun(imported.output, imported);
      near(
          saved.views.get("image1.jpg").calibration().fy,
          850,
          "Distinct camera survives stored archive");
      check(
          saved.notes.contains("Import convention note preserved"),
          "Imported convention/loss notes survive completed-run export");
    } finally {
      for (ImageFrame frame : frames) frame.close();
    }
    Options constrained = imported.copy();
    constrained.intrinsicsMode = "prior";
    rejects(
        () -> prepare(constrained, images),
        "Shared intrinsics refinement cannot silently collapse distinct calibrations");
    Path bom = base.resolve("bom-poses.csv");
    Files.writeString(bom, "\ufeff" + PoseIO.HEADER + "\nexternal.jpg,1,0,0,0,1,0,0,0,1,2,3,4\n");
    near(
        read(bom, "native", Map.of(), o).views.get("external.jpg").pose().center()[0],
        -2,
        "Windows UTF-8 BOM pose import");
    return checks;
  }
}
