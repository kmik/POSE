package sgm;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.*;
import java.util.zip.Deflater;

/**
 * PdfText on a synthetic Word-like PDF, the UltraCam report parser, and ultracam-inpho output read
 * back through POSE's own INPHO reader: ground points must land on the expected image side for
 * every pixel layout and for both flight directions.
 */
final class UltraCamInphoRegression {
  static int checks;

  static void check(boolean yes, String text) {
    checks++;
    if (!yes) throw new AssertionError(text);
  }

  public static void main(String[] args) throws Exception {
    Path root = Files.createTempDirectory("ultracam-inpho");
    System.out.println("UltraCamInphoRegression: " + run(root) + " checks passed");
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Files.createDirectories(root);
    Path pdf = root.resolve("report.pdf");
    Files.write(pdf, reportPdf());

    // PDF text: WinAnsi literal strings, kerned TJ, an Identity-H hex string with ToUnicode, object stream.
    List<String> pages = PdfText.pages(pdf);
    check(pages.size() == 2, "two pages: " + pages.size());
    check(pages.get(0).contains("Camera: UltraCam Eagle Mark 1 f100"), "page 1 camera line: " + pages.get(0));
    check(pages.get(0).contains("Date of Report: Jun-21-2019"), "page 1 date line");
    check(pages.get(1).contains("Pixel Size 5.200 µm*5.200 µm"), "Identity-H micro sign: " + pages.get(1));
    check(pages.get(1).contains("Focal Length ck 100.500 mm ± 0.002 mm"), "plus-minus and kerning: " + pages.get(1));
    check(pages.get(1).indexOf("long track") < pages.get(1).indexOf("cross track"), "lines top to bottom");

    UltraCamInpho.Report report = UltraCamInpho.readReport(pdf);
    check(report.camera().equals("UltraCam Eagle Mark 1 f100") && report.serial().equals("UC-E-1-60715585-f100"), "camera and serial");
    check(report.revision().equals("Rev09.00") && report.reportDate().equals("Jun-21-2019"), "revision and date");
    UltraCamInpho.Sensor pan = report.pan(), ms = report.ms();
    check(pan.focalMm() == 100.5 && pan.pixelMm() == 0.0052 && pan.alongPx() == 13080 && pan.crossPx() == 20010, "pan table: " + pan);
    check(ms.pixelMm() == 0.0156 && ms.alongPx() == 4360 && ms.crossPx() == 6670, "ms table: " + ms);
    check(pan.ppaXmm() == 0 && pan.ppaYmm() == 0.01, "principal point: " + pan);
    check(UltraCamInpho.reportDate("Jun-21-2019").equals("21/06/2019"), "INPHO date");
    check(UltraCamInpho.defaultCameraId(report, pan).equals("UC-E-1-60715585-f100_Rev09.00"), "camera id");
    rejects(() -> UltraCamInpho.parseReport("Camera: X\nnothing else"), "a PDF without camera tables");
    rejects(() -> UltraCamInpho.parseReport("Large Format Panchromatic\nPixel Size 5.2 µm\n"), "a table without focal length");

    // Exterior file: header line, commas, extra column; one eastward and one westward photo.
    Path eo = root.resolve("eo.txt");
    Files.writeString(eo, "name X Y Z omega phi kappa sigma\n"
        + "  east_0001   359581.3654   6769824.8068  7882.0084  0.0  0.0  0.0  1.42\n"
        + "west_0002,348058.0144,6809039.2914,7879.8824,0.0,0.0,200.0,1.49\n");
    List<String> warnings = new ArrayList<>();
    check(UltraCamInpho.readExteriors(eo, warnings).size() == 2 && warnings.size() == 1, "exteriors with a header");
    Path dup = root.resolve("dup.txt");
    Files.writeString(dup, "a 1 2 3 0 0 0\na 1 2 3 0 0 0\n");
    rejects(() -> UltraCamInpho.readExteriors(dup, new ArrayList<>()), "duplicate image names");
    Path bad = root.resolve("bad.txt");
    Files.writeString(bad, "a 1 2 3 0 0 0\nb 1 2 x 0 0 0\n");
    rejects(() -> UltraCamInpho.readExteriors(bad, new ArrayList<>()), "a bad line after data");

    // No images: the report's own frame.
    expect(root, pdf, eo, null, "auto", "along-track", 13080, 20010, new int[] {1, 0}, new int[] {0, -1});
    // Images 13080 wide: along-track; 20010 wide: up (flight towards the top).
    Path portrait = images(root.resolve("portrait"), 13080, 20010);
    expect(root, pdf, eo, portrait, "auto", "along-track", 13080, 20010, new int[] {1, 0}, new int[] {0, -1});
    Path landscape = images(root.resolve("landscape"), 20010, 13080);
    expect(root, pdf, eo, landscape, "auto", "up", 20010, 13080, new int[] {0, -1}, new int[] {-1, 0});
    expect(root, pdf, eo, landscape, "down", "down", 20010, 13080, new int[] {0, 1}, new int[] {1, 0});
    expect(root, pdf, eo, portrait, "along-track-reversed", "along-track-reversed", 13080, 20010, new int[] {-1, 0}, new int[] {0, 1});
    Path wrong = images(root.resolve("wrong"), 1000, 800);
    rejects(() -> UltraCamInpho.convert(settings(eo, pdf, root.resolve("w.prj"), wrong, "auto", "pan")), "images of another size");
    Path msImages = images(root.resolve("ms"), 6670, 4360);
    UltraCamInpho.Result r = UltraCamInpho.convert(settings(eo, pdf, root.resolve("ms.prj"), msImages, "auto", "ms"));
    check(r.layout().equals("up") && r.columns() == 6670 && Files.readString(r.output()).contains("$ID : UC-E-1-60715585-f100_Rev09.00_ms"), "multispectral format");
    rejects(() -> UltraCamInpho.convert(settings(eo, pdf, root.resolve("m.prj"), root.resolve("missing"), "auto", "pan")), "a missing image folder");

    multiCamera(root, pdf, eo, portrait);

    // Command line.
    check(UltraCamInpho.main(new String[] {"--exteriors", eo.toString()}) == 2, "missing options exit 2");
    check(UltraCamInpho.main(new String[] {"--exteriors", eo.toString(), "--calibration-report", pdf.toString(), "--output", root.resolve("cli.prj").toString(), "--layout", "up"}) == 0
        && Files.readString(root.resolve("cli.prj")).contains("$CCD_COLUMNS : 20010"), "command line run");
    check(UltraCamInpho.main(new String[] {"--bogus", "1"}) == 2, "unknown option exit 2");
    check(PoseUltraCamDialog.defaultOutput(Path.of("a", "exterior_orientation.txt")).getFileName().toString().equals("exterior_orientation_inpho.prj"), "GUI default output name");
    return checks;
  }

  interface Action {
    void run() throws Exception;
  }

  static void rejects(Action a, String text) throws Exception {
    boolean rejected = false;
    try {
      a.run();
    } catch (IllegalArgumentException | IOException e) {
      rejected = true;
    }
    check(rejected, "rejects " + text);
  }

  static UltraCamInpho.Settings settings(Path eo, Path pdf, Path out, Path images, String layout, String sensor) {
    return new UltraCamInpho.Settings(eo, pdf, out, images, ".tif", layout, sensor, "gon", 100.0, null);
  }

  /**
   * Converts, reads back with OrientationIO, and checks where ground 500 m east of each camera lands:
   * eastSign is the (column, row) direction for the eastward photo; the westward photo must be the opposite.
   */
  static void expect(Path root, Path pdf, Path eo, Path images, String layout, String expected, int w, int h, int[] east, int[] north) throws Exception {
    Path out = root.resolve(expected + "-" + (images == null ? "none" : images.getFileName()) + ".prj");
    UltraCamInpho.Result r = UltraCamInpho.convert(settings(eo, pdf, out, images, layout, "pan"));
    check(r.layout().equals(expected), "layout " + r.layout() + " != " + expected);
    check(r.columns() == w && r.rows() == h, "size " + r.columns() + "x" + r.rows());
    String text = Files.readString(out);
    check(text.contains("$TERRAIN_HEIGHT : 100.000000") && text.contains("$CCD_COLUMNS : " + w), "camera block and terrain height");
    if (images != null) check(text.contains("$PHOTO_FILE : " + images.resolve("east_0001.tif").toAbsolutePath()), "photo file from the image folder");

    Path dir = images != null ? images : Files.createDirectories(root.resolve("empty"));
    Options o = Options.parse(new String[] {"-i", dir.toString(), "-o", root.resolve("p").toString()});
    OrientationIO.Model m = OrientationIO.read(out, "inpho", Map.of(), o);
    check(m.views.size() == 2, "two photos read back");
    for (var v : m.views.values()) {
      boolean west = v.image().startsWith("west");
      Calibration c = v.calibration();
      Camera p = v.pose();
      check(c.width == w && c.height == h && Math.abs(c.fx - 100.5 / 0.0052) < 1e-6, "calibration read back: " + c.width + "x" + c.height);
      double[] centre = p.center(), nadir = pixel(c, p, centre[0], centre[1]);
      check(Math.abs(nadir[0] - c.cx) < 1 && Math.abs(nadir[1] - c.cy) < 1, "nadir at the image centre: " + Arrays.toString(nadir));
      double[] e = pixel(c, p, centre[0] + 500, centre[1]), n = pixel(c, p, centre[0], centre[1] + 500);
      int s = west ? -1 : 1;
      direction(e[0] - nadir[0], e[1] - nadir[1], s * east[0], s * east[1], expected + (west ? " west" : " east") + " flight, east");
      direction(n[0] - nadir[0], n[1] - nadir[1], s * north[0], s * north[1], expected + (west ? " west" : " east") + " flight, north");
    }
  }

  static void direction(double dc, double dr, int col, int row, String text) {
    double len = Math.hypot(dc, dr);
    check(len > 100 && Math.abs(dc / len - col) < 1e-6 && Math.abs(dr / len - row) < 1e-6, text + ": (" + dc + ", " + dr + ") expected (" + col + ", " + row + ")");
  }

  static double[] pixel(Calibration c, Camera p, double x, double y) {
    double[] v = p.view(new double[] {x, y, 100});
    return new double[] {c.fx * v[0] / v[2] + c.cx, c.fy * v[1] / v[2] + c.cy};
  }

  /**
   * Several cameras into one project: a pan camera and a multispectral one (own definitions, both
   * read back by POSE as two camera groups), shared and numbered IDs from one report, and the
   * rejections (an image in two cameras, a given ID for two calibrations, a camera without report).
   */
  static void multiCamera(Path root, Path pdf, Path eo, Path portrait) throws Exception {
    Path eo2 = root.resolve("eo2.txt");
    Files.writeString(eo2, "cam2_0001 359600.0 6769800.0 7880.0 0.0 0.0 0.0\ncam2_0002 348000.0 6809000.0 7880.0 0.0 0.0 200.0\n");
    Path ms2 = images(root.resolve("ms2"), 6670, 4360, "cam2_0001.tif", "cam2_0002.tif");
    Path out = root.resolve("multi.prj");
    UltraCamInpho.Settings s = new UltraCamInpho.Settings(
        List.of(
            new UltraCamInpho.CameraInput(eo, pdf, portrait, ".tif", "auto", "pan", null),
            new UltraCamInpho.CameraInput(eo2, pdf, ms2, ".tif", "auto", "ms", null)),
        out, "gon", null);
    UltraCamInpho.Result r = UltraCamInpho.convert(s);
    check(r.photos() == 4 && r.cameras().size() == 2, "two cameras, four photos: " + r.photos());
    check(r.cameras().get(0).layout().equals("along-track") && r.cameras().get(1).layout().equals("up"), "layout per camera");
    String text = Files.readString(out);
    check(count(text, "$CAMERA_DEFINITION") == 2 && text.contains("$ID : UC-E-1-60715585-f100_Rev09.00\n")
        && text.contains("$ID : UC-E-1-60715585-f100_Rev09.00_ms\n"), "one definition per camera");
    check(text.indexOf("$CAMERA_DEFINITION", text.indexOf("$CAMERA_DEFINITION") + 1) < text.indexOf("$PHOTO\n"), "definitions before the photos");
    check(text.contains("$CAMERA_ID : UC-E-1-60715585-f100_Rev09.00_ms\n  $") && count(text, "$CAMERA_ID : UC-E-1-60715585-f100_Rev09.00_ms\n") == 2, "photos refer to their camera");
    check(r.summary().contains("of 2 cameras") && r.summary().contains("Camera 2: UC-E-1-60715585-f100_Rev09.00_ms, 2 photos from eo2.txt"), "summary per camera: " + r.summary());

    Options o = Options.parse(new String[] {"-i", portrait.toString(), "-o", root.resolve("p").toString()});
    OrientationIO.Model m = OrientationIO.read(out, "inpho", Map.of(), o);
    check(m.views.size() == 4, "four photos read back: " + m.views.size());
    Set<String> groups = new HashSet<>();
    for (var v : m.views.values()) {
      groups.add(v.camera());
      Calibration c = v.calibration();
      boolean ms = v.image().startsWith("cam2");
      check(ms ? c.width == 6670 && c.height == 4360 && Math.abs(c.fx - 100.5 / 0.0156) < 1e-6 : c.width == 13080 && Math.abs(c.fx - 100.5 / 0.0052) < 1e-6,
          "calibration of " + v.image() + ": " + c.width + "x" + c.height + " fx " + c.fx);
      double[] centre = v.pose().center(), nadir = pixel(c, v.pose(), centre[0], centre[1]);
      check(Math.abs(nadir[0] - c.cx) < 1 && Math.abs(nadir[1] - c.cy) < 1, "nadir at the centre of " + v.image());
      double[] e = pixel(c, v.pose(), centre[0] + 500, centre[1]);
      int sign = v.image().contains("0002") ? -1 : 1; // the second photo of each camera flies west
      if (ms) direction(e[0] - nadir[0], e[1] - nadir[1], 0, -sign, "ms camera " + v.image() + " east");
      else direction(e[0] - nadir[0], e[1] - nadir[1], sign, 0, "pan camera " + v.image() + " east");
    }
    check(groups.size() == 2 && !groups.contains(null), "two camera groups: " + groups);

    // One report, same layout: one shared definition; another layout: a numbered ID and a warning.
    Path eo3 = root.resolve("eo3.txt");
    Files.writeString(eo3, "cam3_0001 359600.0 6769800.0 7880.0 0.0 0.0 0.0\n");
    UltraCamInpho.Result shared = UltraCamInpho.convert(new UltraCamInpho.Settings(
        List.of(new UltraCamInpho.CameraInput(eo, pdf, null, ".tif", "along-track", "pan", null), new UltraCamInpho.CameraInput(eo3, pdf, null, ".tif", "along-track", "pan", null)),
        root.resolve("shared.prj"), "gon", null));
    check(count(Files.readString(shared.output()), "$CAMERA_DEFINITION") == 1 && shared.photos() == 3, "same camera twice: one definition");
    UltraCamInpho.Result numbered = UltraCamInpho.convert(new UltraCamInpho.Settings(
        List.of(new UltraCamInpho.CameraInput(eo, pdf, null, ".tif", "along-track", "pan", null), new UltraCamInpho.CameraInput(eo3, pdf, null, ".tif", "up", "pan", null)),
        root.resolve("numbered.prj"), "gon", null));
    String nt = Files.readString(numbered.output());
    check(count(nt, "$CAMERA_DEFINITION") == 2 && nt.contains("$ID : UC-E-1-60715585-f100_Rev09.00_2\n") && numbered.cameras().get(1).cameraId().endsWith("_2")
        && numbered.warnings().stream().anyMatch(w -> w.startsWith("camera 2 (eo3.txt): camera ID")), "another layout: numbered ID " + numbered.warnings());
    rejects(() -> UltraCamInpho.convert(new UltraCamInpho.Settings(
        List.of(new UltraCamInpho.CameraInput(eo, pdf, null, ".tif", "along-track", "pan", "cam"), new UltraCamInpho.CameraInput(eo3, pdf, null, ".tif", "up", "pan", "cam")),
        root.resolve("x.prj"), "gon", null)), "one given camera ID for two calibrations");
    rejects(() -> UltraCamInpho.convert(new UltraCamInpho.Settings(
        List.of(new UltraCamInpho.CameraInput(eo, pdf, null, ".tif", "auto", "pan", null), new UltraCamInpho.CameraInput(eo, pdf, null, ".tif", "auto", "ms", null)),
        root.resolve("x.prj"), "gon", null)), "an image in two cameras");
    boolean labelled = false;
    try {
      UltraCamInpho.convert(new UltraCamInpho.Settings(
          List.of(new UltraCamInpho.CameraInput(eo, pdf, null, ".tif", "auto", "pan", null), new UltraCamInpho.CameraInput(eo3, pdf, root.resolve("missing"), ".tif", "auto", "pan", null)),
          root.resolve("x.prj"), "gon", null));
    } catch (IllegalArgumentException e) {
      labelled = e.getMessage().startsWith("camera 2 (eo3.txt): image folder not found");
    }
    check(labelled, "errors name the camera");

    // Command line: one --exteriors/--calibration-report group per camera, in either order; --images carries over.
    String[] args = {"--exteriors", eo.toString(), "--calibration-report", pdf.toString(), "-i", portrait.toString(), "--output", root.resolve("cli-multi.prj").toString(),
        "--calibration-report", pdf.toString(), "--exteriors", eo3.toString(), "--layout", "up", "--angle-unit", "gon"};
    UltraCamInpho.Settings parsed = UltraCamInpho.parse(args);
    check(parsed.cameras().size() == 2 && parsed.cameras().get(1).exteriors().equals(eo3) && parsed.cameras().get(1).layout().equals("up")
        && parsed.cameras().get(0).layout().equals("auto"), "two camera groups parsed");
    check(portrait.equals(parsed.cameras().get(1).images()), "image folder carried to the next camera");
    check(UltraCamInpho.main(new String[] {"--exteriors", eo.toString(), "--calibration-report", pdf.toString(), "--exteriors", eo2.toString(), "--calibration-report", pdf.toString(),
        "--images", ms2.toString(), "--sensor", "ms", "--output", root.resolve("cli-multi.prj").toString()}) == 0
        && count(Files.readString(root.resolve("cli-multi.prj")), "$PHOTO\n") == 4, "command line with two cameras");
    check(UltraCamInpho.main(new String[] {"--exteriors", eo.toString(), "--calibration-report", pdf.toString(), "--exteriors", eo3.toString(), "--output", root.resolve("y.prj").toString()}) == 2,
        "a camera without report exits 2");
    check(UltraCamInpho.main(new String[] {"--exteriors", eo.toString(), "--calibration-report", pdf.toString(), "--output", "a.prj", "--output", "b.prj"}) == 2, "two outputs exit 2");
    check(PoseUltraCamDialog.folderNote(UltraCamInpho.parse(new String[] {"--exteriors", eo.toString(), "--calibration-report", pdf.toString(), "--images", portrait.toString(),
        "--exteriors", eo2.toString(), "--calibration-report", pdf.toString(), "--images", ms2.toString(), "--output", "z.prj"})) != null
        && PoseUltraCamDialog.folderNote(parsed) == null, "GUI note when the cameras' images are in several folders");
  }

  static int count(String text, String part) {
    int n = 0;
    for (int i = text.indexOf(part); i >= 0; i = text.indexOf(part, i + part.length())) n++;
    return n;
  }

  /** A folder with header-only TIFFs (width/height tags) for the two photos. */
  static Path images(Path dir, int w, int h) throws IOException {
    return images(dir, w, h, "east_0001.tif", "west_0002.tif");
  }

  static Path images(Path dir, int w, int h, String... names) throws IOException {
    Files.createDirectories(dir);
    ByteBuffer b = ByteBuffer.allocate(8 + 2 + 2 * 12 + 4).order(ByteOrder.LITTLE_ENDIAN);
    b.put((byte) 'I').put((byte) 'I').putShort((short) 42).putInt(8).putShort((short) 2);
    b.putShort((short) 256).putShort((short) 4).putInt(1).putInt(w);
    b.putShort((short) 257).putShort((short) 4).putInt(1).putInt(h);
    b.putInt(0);
    for (String n : names) Files.write(dir.resolve(n), b.array());
    return dir;
  }

  // ----- synthetic report PDF -----

  static byte[] reportPdf() throws IOException {
    String page1 = "BT /F1 11 Tf 1 0 0 1 72 700 Tm (Camera: UltraCam Eagle Mark 1 f100) Tj ET\n"
        + "BT /F1 11 Tf 1 0 0 1 72 680 Tm [(Serial: UC-E-1-)-20(60715585-f100)] TJ ET\n"
        + "BT /F1 11 Tf 1 0 0 1 72 660 Tm (Date of Report:) Tj 1 0 0 1 250 660 Tm (Jun-21-2019) Tj ET\n"
        + "BT /F1 11 Tf 1 0 0 1 72 640 Tm (Camera Revision:) Tj 1 0 0 1 250 640 Tm (Rev09.00) Tj ET\n";
    StringBuilder t = new StringBuilder();
    double y = 760;
    t.append(line(72, y, "Large Format Panchromatic Output Image"));
    // Drawn out of order on purpose: layout must sort by position.
    t.append(line(72, y - 40, "cross track 104.052 mm 20010 pixel"));
    t.append(line(72, y - 20, "long track 68.016 mm 13080 pixel"));
    t.append("BT /F1 10 Tf 1 0 0 1 72 680 Tm (Pixel Size 5.200 ) Tj /F2 10 Tf <0001> Tj /F1 10 Tf (m*5.200 ) Tj /F2 10 Tf <0001> Tj /F1 10 Tf (m) Tj ET\n");
    t.append("BT /F1 10 Tf 1 0 0 1 72 660 Tm [(Focal Length ck 100.)20(500 mm )] TJ (\\261 0.002 mm) Tj ET\n");
    t.append(line(72, 640, "X_ppa 0.000 mm"));
    t.append(line(72, 620, "Y_ppa 0.010 mm"));
    t.append(line(72, 560, "Medium Format Multispectral Output Image"));
    t.append(line(72, 540, "long track 68.016 mm 4360 pixel"));
    t.append(line(72, 520, "cross track 104.052 mm 6670 pixel"));
    t.append(line(72, 500, "Pixel Size 15.600 um*15.600 um"));
    t.append(line(72, 480, "Focal Length ck 100.500 mm"));

    String cmap = "/CIDInit /ProcSet findresource begin 12 dict begin begincmap\n1 begincodespacerange <0000> <FFFF> endcodespacerange\n"
        + "1 beginbfchar <0001> <00B5> endbfchar\nendcmap CMapName currentdict /CMap defineresource pop end end\n";
    // Objects 4 and 9 (fonts) live in an object stream (12).
    String font1 = "<</Type/Font/Subtype/TrueType/BaseFont/Calibri/Encoding/WinAnsiEncoding/FirstChar 32/LastChar 32/Widths[226]>>";
    String font2 = "<</Type/Font/Subtype/Type0/BaseFont/SymbolMT/Encoding/Identity-H/DescendantFonts[10 0 R]/ToUnicode 11 0 R>>";
    String objStmBody = "4 0 9 " + font1.length() + " " + font1 + font2;
    int first = ("4 0 9 " + font1.length() + " ").length();

    ByteArrayOutputStream out = new ByteArrayOutputStream();
    write(out, "%PDF-1.7\n%âã\n");
    write(out, "1 0 obj\n<</Type/Catalog/Pages 2 0 R>>\nendobj\n");
    write(out, "2 0 obj\n<</Type/Pages/Count 2/Kids[ 3 0 R 7 0 R] >>\nendobj\n");
    write(out, "3 0 obj\n<</Type/Page/Parent 2 0 R/Resources<</Font<</F1 4 0 R>>>>/Contents 5 0 R/MediaBox[0 0 595 842]>>\nendobj\n");
    stream(out, 5, "", page1.getBytes(StandardCharsets.ISO_8859_1), true);
    write(out, "6 0 obj\n<</Font<</F1 4 0 R/F2 9 0 R>>>>\nendobj\n");
    write(out, "7 0 obj\n<</Type/Page/Parent 2 0 R/Resources 6 0 R/Contents[ 8 0 R]/MediaBox[0 0 595 842]>>\nendobj\n");
    stream(out, 8, "", t.toString().getBytes(StandardCharsets.ISO_8859_1), true);
    write(out, "10 0 obj\n<</Type/Font/Subtype/CIDFontType2/BaseFont/SymbolMT/DW 1000/W[ 1[ 576]]>>\nendobj\n");
    stream(out, 11, "", cmap.getBytes(StandardCharsets.ISO_8859_1), true);
    stream(out, 12, "/Type/ObjStm/N 2/First " + first, objStmBody.getBytes(StandardCharsets.ISO_8859_1), true);
    write(out, "trailer\n<</Root 1 0 R/Size 13>>\n%%EOF\n");
    return out.toByteArray();
  }

  static String line(double x, double y, String text) {
    return "BT /F1 10 Tf 1 0 0 1 " + x + " " + y + " Tm (" + text + ") Tj ET\n";
  }

  static void write(ByteArrayOutputStream out, String s) {
    out.writeBytes(s.getBytes(StandardCharsets.ISO_8859_1));
  }

  static void stream(ByteArrayOutputStream out, int id, String dict, byte[] data, boolean flate) {
    byte[] body = data;
    if (flate) {
      Deflater d = new Deflater();
      d.setInput(data);
      d.finish();
      byte[] buf = new byte[data.length + 64];
      int n = d.deflate(buf);
      d.end();
      body = Arrays.copyOf(buf, n);
    }
    write(out, id + " 0 obj\n<<" + dict + (flate ? "/Filter/FlateDecode" : "") + "/Length " + body.length + ">>\nstream\n");
    out.writeBytes(body);
    write(out, "\nendstream\nendobj\n");
  }
}
