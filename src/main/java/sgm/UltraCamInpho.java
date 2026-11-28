package sgm;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.FileChannel;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.*;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Stream;

/**
 * `photogrammetry ultracam-inpho`: a Vexcel UltraCam exterior orientation list (name X Y Z omega
 * phi kappa) and the camera's calibration report PDF become one INPHO project file that POSE
 * (--orientation-input / --poses-format inpho) and Agisoft Metashape (Import Cameras, INPHO
 * project) read.
 *
 * <p>Conventions: R = Rx(omega) Ry(phi) Rz(kappa) maps photogrammetric camera axes (x right, y up,
 * z back) to the world; $EXT_ORI stores its transpose (world to camera), as INPHO writes it. The
 * report's image frame has x along the flight (long track). When the stored image pixels are turned
 * against that frame (20010 columns cross track), the quarter turn is folded into kappa so that the
 * pixel affine stays axis-aligned: POSE rejects rotated CCD_INTERIOR_ORIENTATION and ignores
 * $CAMERA_MOUNT_ROTATION. The report's remaining distortion (below 2 um) is not modelled: Level-2/3
 * UltraCam images are already corrected with the calibration's correction grid.
 */
final class UltraCamInpho {
  private UltraCamInpho() {}

  /** One output image format of the report (panchromatic or multispectral). */
  record Sensor(
      String name, double focalMm, double ppaXmm, double ppaYmm, double pixelMm, int alongPx, int crossPx) {}

  record Report(String camera, String serial, String revision, String reportDate, Sensor pan, Sensor ms) {
    Sensor sensor(String which) {
      Sensor s = which.equals("ms") ? ms : pan;
      if (s == null)
        throw new IllegalArgumentException(
            "the calibration report has no " + (which.equals("ms") ? "multispectral" : "panchromatic") + " camera table");
      return s;
    }
  }

  record Exterior(String name, double x, double y, double z, double omega, double phi, double kappa) {}

  static final List<String> LAYOUTS = List.of("auto", "along-track", "along-track-reversed", "up", "down");

  /**
   * One camera: its exterior orientation list and calibration report, with its own image folder,
   * layout, sensor format and camera ID. Several cameras go into one project file.
   */
  record CameraInput(
      Path exteriors, Path report, Path images, String imageExtension, String layout, String sensor, String cameraId) {}

  /** Conversion settings; the GUI fills the same record. */
  record Settings(List<CameraInput> cameras, Path output, String angleUnit, Double terrainHeight) {
    Settings {
      if (cameras == null || cameras.isEmpty()) throw new IllegalArgumentException("no camera (exterior file + calibration report) given");
      cameras = List.copyOf(cameras);
    }

    /** One camera. */
    Settings(Path exteriors, Path report, Path output, Path images, String imageExtension, String layout, String sensor,
        String angleUnit, Double terrainHeight, String cameraId) {
      this(List.of(new CameraInput(exteriors, report, images, imageExtension, layout, sensor, cameraId)), output, angleUnit, terrainHeight);
    }
  }

  /** What one camera contributed. */
  record CameraResult(
      CameraInput input, String cameraId, int photos, String layout, String layoutReason, int columns, int rows, Report report, Sensor sensor) {}

  /** What was written, for the console or the window. */
  record Result(Path output, int photos, List<CameraResult> cameras, List<String> warnings) {
    CameraResult first() {
      return cameras.get(0);
    }

    String layout() {
      return first().layout;
    }

    int columns() {
      return first().columns;
    }

    int rows() {
      return first().rows;
    }

    String summary() {
      StringBuilder s = new StringBuilder();
      s.append("Wrote ").append(photos).append(" photos").append(cameras.size() > 1 ? " of " + cameras.size() + " cameras" : "")
          .append(" to ").append(output).append('\n');
      int n = 0;
      for (CameraResult c : cameras) {
        n++;
        Report report = c.report;
        Sensor sensor = c.sensor;
        if (cameras.size() > 1)
          s.append('\n').append("Camera ").append(n).append(": ").append(c.cameraId).append(", ").append(c.photos).append(" photos from ")
              .append(c.input.exteriors().getFileName()).append('\n');
        s.append(String.format(Locale.ROOT, "Camera: %s, serial %s, %s (report %s)%n", report.camera, report.serial, report.revision, report.reportDate));
        s.append(String.format(Locale.ROOT, "Sensor: %s, focal %.3f mm, principal point (%.3f, %.3f) mm, pixel %.1f um, %d x %d px%n",
            sensor.name, sensor.focalMm, sensor.ppaXmm, sensor.ppaYmm, sensor.pixelMm * 1000, c.columns, c.rows));
        s.append("Image layout: ").append(c.layout).append(" (").append(c.layoutReason).append(")\n");
      }
      if (!warnings.isEmpty() && cameras.size() > 1) s.append('\n');
      for (String w : warnings) s.append("Warning: ").append(w).append('\n');
      return s.toString();
    }
  }

  // ----- calibration report -----

  static Report readReport(Path pdf) throws IOException {
    return parseReport(String.join("\n", PdfText.pages(pdf)));
  }

  static Report parseReport(String text) {
    String camera = first(text, "Camera:\\s*(.+)"), serial = first(text, "Serial:\\s*(\\S+)");
    String revision = first(text, "Camera Revision:\\s*(\\S+)"), date = first(text, "Date of Report:\\s*(\\S+)");
    Sensor pan = sensor("panchromatic", section(text, "Large Format Panchromatic", "Medium Format Multispectral"));
    Sensor ms = sensor("multispectral", section(text, "Medium Format Multispectral", null));
    if (pan == null && ms == null)
      throw new IllegalArgumentException(
          "no UltraCam camera table (Image Format, Pixel Size, Focal Length ck, X_ppa/Y_ppa) found in the calibration report;"
              + " is it a Vexcel calibration report with text (not a scan)?");
    return new Report(
        camera == null ? "UltraCam" : camera.strip(),
        serial == null ? "unknown" : serial,
        revision == null ? "" : revision,
        date == null ? "" : date,
        pan,
        ms);
  }

  private static String section(String text, String start, String end) {
    int a = text.indexOf(start);
    if (a < 0) return null;
    int b = end == null ? -1 : text.indexOf(end, a + start.length());
    return text.substring(a, b < 0 ? text.length() : b);
  }

  private static final String NUM = "([-+]?\\d+(?:[.,]\\d+)?)";

  private static Sensor sensor(String name, String s) {
    if (s == null) return null;
    String focal = first(s, "Focal Length\\s*ck\\s*" + NUM + "\\s*mm");
    String pixel = first(s, "Pixel Size\\s*" + NUM + "\\s*(?:\u00b5|u|micro)m");
    String along = first(s, "long track\\s*" + NUM + "\\s*mm\\s*(\\d+)\\s*pixel", 2);
    String cross = first(s, "cross track\\s*" + NUM + "\\s*mm\\s*(\\d+)\\s*pixel", 2);
    String x = first(s, "X_ppa\\s*" + NUM + "\\s*mm"), y = first(s, "Y_ppa\\s*" + NUM + "\\s*mm");
    List<String> missing = new ArrayList<>();
    if (focal == null) missing.add("Focal Length ck");
    if (pixel == null) missing.add("Pixel Size");
    if (along == null) missing.add("long track pixels");
    if (cross == null) missing.add("cross track pixels");
    if (!missing.isEmpty())
      throw new IllegalArgumentException("the " + name + " table of the calibration report has no " + String.join(", ", missing));
    return new Sensor(
        name,
        number(focal),
        x == null ? 0 : number(x),
        y == null ? 0 : number(y),
        number(pixel) / 1000,
        Integer.parseInt(along),
        Integer.parseInt(cross));
  }

  private static double number(String s) {
    return Double.parseDouble(s.replace(',', '.'));
  }

  private static String first(String text, String regex) {
    return first(text, regex, 1);
  }

  private static String first(String text, String regex, int group) {
    if (text == null) return null;
    Matcher m = Pattern.compile(regex).matcher(text);
    return m.find() ? m.group(group) : null;
  }

  // ----- exteriors -----

  static List<Exterior> readExteriors(Path file, List<String> warnings) throws IOException {
    List<Exterior> out = new ArrayList<>();
    Set<String> names = new HashSet<>();
    int skipped = 0, line = 0;
    for (String raw : Files.readAllLines(file, StandardCharsets.ISO_8859_1)) {
      line++;
      String t = raw.strip();
      if (t.isEmpty() || t.startsWith("#") || t.startsWith(";")) continue;
      String[] f = t.split("[\\s,;]+");
      Exterior e = null;
      if (f.length >= 7)
        try {
          e = new Exterior(f[0], d(f[1]), d(f[2]), d(f[3]), d(f[4]), d(f[5]), d(f[6]));
        } catch (NumberFormatException ignored) {
          // A header or other text line.
        }
      if (e == null) {
        if (out.isEmpty() && skipped == 0) {
          skipped++;
          continue;
        }
        throw new IllegalArgumentException(
            file.getFileName() + " line " + line + ": expected 'name X Y Z omega phi kappa', got: " + t);
      }
      if (!names.add(e.name)) throw new IllegalArgumentException(file.getFileName() + ": image " + e.name + " is listed twice");
      out.add(e);
    }
    if (out.isEmpty()) throw new IllegalArgumentException(file + " has no exterior orientations (name X Y Z omega phi kappa)");
    if (skipped > 0) warnings.add("skipped the header line of " + file.getFileName());
    return out;
  }

  private static double d(String s) {
    double v = Double.parseDouble(s);
    if (!Double.isFinite(v)) throw new NumberFormatException(s);
    return v;
  }

  // ----- conversion -----

  /** One camera made ready for writing. */
  private record Prepared(
      CameraInput in, Report report, Sensor sensor, List<Exterior> exteriors, Map<String, Path> files, String ext,
      String layout, String reason, int columns, int rows, double ppx, double ppy, double kappaShift, String cameraId) {}

  static Result convert(Settings s) throws IOException {
    double toRad =
        switch (s.angleUnit == null || s.angleUnit.isBlank() ? "gon" : s.angleUnit) {
          case "gon", "grad" -> Math.PI / 200;
          case "deg", "degrees" -> Math.PI / 180;
          case "rad", "radians" -> 1;
          default -> throw new IllegalArgumentException("angle unit must be gon, deg or rad: " + s.angleUnit);
        };
    boolean several = s.cameras.size() > 1;
    List<String> warnings = new ArrayList<>();
    List<Prepared> cameras = new ArrayList<>();
    Map<String, String> definitions = new LinkedHashMap<>(), imageOwner = new HashMap<>();
    for (int i = 0; i < s.cameras.size(); i++) {
      CameraInput in = s.cameras.get(i);
      String label = several ? "camera " + (i + 1) + " (" + in.exteriors.getFileName() + "): " : "";
      List<String> own = new ArrayList<>();
      Prepared p;
      try {
        p = prepare(in, toRad, own);
      } catch (IllegalArgumentException e) {
        throw new IllegalArgumentException(label + e.getMessage(), e);
      }
      for (String w : own) warnings.add(label + w);
      for (Exterior e : p.exteriors) {
        String before = imageOwner.putIfAbsent(e.name.toLowerCase(Locale.ROOT), in.exteriors.getFileName().toString());
        if (before != null)
          throw new IllegalArgumentException(
              "image " + e.name + " is listed for two cameras (" + before + " and " + in.exteriors.getFileName()
                  + "): image names must be unique in one project");
      }
      // Cameras with the same ID share one definition when it is the same; a default ID of a different
      // definition gets a number, a given one must be unique.
      String id = p.cameraId, body = definitionBody(p);
      boolean given = in.cameraId != null && !in.cameraId.isBlank();
      for (int k = 2; definitions.containsKey(id) && !definitions.get(id).equals(body); k++) {
        if (given)
          throw new IllegalArgumentException(label + "camera ID " + id + " is already used by a camera with another calibration or image layout");
        id = p.cameraId + "_" + k;
      }
      if (!id.equals(p.cameraId)) {
        warnings.add(label + "camera ID " + p.cameraId + " is taken by a camera with another calibration or image layout; written as " + id);
        p = new Prepared(p.in, p.report, p.sensor, p.exteriors, p.files, p.ext, p.layout, p.reason, p.columns, p.rows, p.ppx, p.ppy, p.kappaShift, id);
      }
      definitions.putIfAbsent(id, body);
      cameras.add(p);
    }

    String text = write(s, cameras, definitions, toRad);
    Path parent = s.output.toAbsolutePath().getParent();
    if (parent != null) Files.createDirectories(parent);
    Files.writeString(s.output, text, StandardCharsets.UTF_8);
    List<CameraResult> results = new ArrayList<>();
    int photos = 0;
    for (Prepared p : cameras) {
      results.add(new CameraResult(p.in, p.cameraId, p.exteriors.size(), p.layout, p.reason, p.columns, p.rows, p.report, p.sensor));
      photos += p.exteriors.size();
    }
    return new Result(s.output, photos, results, warnings);
  }

  private static Prepared prepare(CameraInput s, double toRad, List<String> warnings) throws IOException {
    if (s.exteriors == null) throw new IllegalArgumentException("no exterior orientation file");
    if (s.report == null) throw new IllegalArgumentException("no calibration report for " + s.exteriors.getFileName());
    Report report = readReport(s.report);
    Sensor sensor = report.sensor(s.sensor == null || s.sensor.isBlank() ? "pan" : s.sensor);
    List<Exterior> exteriors = readExteriors(s.exteriors, warnings);
    String ext = s.imageExtension == null || s.imageExtension.isBlank() ? ".tif" : s.imageExtension.startsWith(".") ? s.imageExtension : "." + s.imageExtension;

    // Image files: the given extension, else any common one with the same base name.
    Map<String, Path> files = new LinkedHashMap<>();
    if (s.images != null) {
      if (!Files.isDirectory(s.images)) throw new IllegalArgumentException("image folder not found: " + s.images);
      Map<String, Path> byBase = new HashMap<>();
      try (Stream<Path> list = Files.list(s.images)) {
        list.filter(Files::isRegularFile).forEach(p -> {
          String n = p.getFileName().toString();
          int dot = n.lastIndexOf('.');
          if (dot > 0 && Set.of("tif", "tiff", "jpg", "jpeg", "png", "jp2").contains(n.substring(dot + 1).toLowerCase(Locale.ROOT)))
            byBase.merge(n.substring(0, dot).toLowerCase(Locale.ROOT), p, (a, b) -> a.getFileName().toString().toLowerCase(Locale.ROOT).endsWith(ext.toLowerCase(Locale.ROOT)) ? a : b);
        });
      }
      int missing = 0;
      for (Exterior e : exteriors) {
        Path p = byBase.get(e.name.toLowerCase(Locale.ROOT));
        if (p == null) missing++;
        else files.put(e.name, p);
      }
      if (missing == exteriors.size())
        throw new IllegalArgumentException("none of the " + exteriors.size() + " images (e.g. " + exteriors.get(0).name + ext + ") is in " + s.images);
      if (missing > 0) warnings.add(missing + " of " + exteriors.size() + " images are not in " + s.images + " (written with " + ext + " names)");
    }

    // Layout of the stored pixels against the flight direction.
    String layout = s.layout == null || s.layout.isBlank() ? "auto" : s.layout;
    if (!LAYOUTS.contains(layout)) throw new IllegalArgumentException("layout must be one of " + LAYOUTS + ": " + layout);
    String reason = "set by --layout";
    if (layout.equals("auto")) {
      int[] size = null;
      Path sample = null;
      for (Path p : files.values()) {
        try {
          size = imageSize(p);
          sample = p;
          break;
        } catch (IOException e) {
          warnings.add("cannot read the size of " + p.getFileName() + ": " + e.getMessage());
        }
      }
      if (size == null) {
        layout = "along-track";
        reason = "no image to look at; the report's frame, " + sensor.alongPx + " columns along the flight. Give the image folder to check";
        warnings.add("image layout not checked against an image; assumed " + sensor.alongPx + " x " + sensor.crossPx + " px with the flight towards the right");
      } else if (size[0] == sensor.alongPx && size[1] == sensor.crossPx) {
        layout = "along-track";
        reason = sample.getFileName() + " is " + size[0] + " x " + size[1] + ": flight towards the right";
      } else if (size[0] == sensor.crossPx && size[1] == sensor.alongPx) {
        layout = "up";
        reason = sample.getFileName() + " is " + size[0] + " x " + size[1] + ": flight assumed towards the image top; use layout 'down' if the result is turned half round";
      } else
        throw new IllegalArgumentException(
            sample.getFileName() + " is " + size[0] + " x " + size[1] + " px, but the " + sensor.name + " camera of the report is "
                + sensor.crossPx + " x " + sensor.alongPx + " px" + (sensor.name.equals("panchromatic") ? "; multispectral images need sensor ms" : ""));
    }
    double kappaShift;
    int columns, rows;
    switch (layout) {
      case "along-track" -> { kappaShift = 0; columns = sensor.alongPx; rows = sensor.crossPx; }
      case "along-track-reversed" -> { kappaShift = Math.PI; columns = sensor.alongPx; rows = sensor.crossPx; }
      case "up" -> { kappaShift = -Math.PI / 2; columns = sensor.crossPx; rows = sensor.alongPx; }
      default -> { kappaShift = Math.PI / 2; columns = sensor.crossPx; rows = sensor.alongPx; }
    }
    // The principal point offset turns with the pixel frame.
    double ppx = sensor.ppaXmm, ppy = sensor.ppaYmm, c = Math.round(Math.cos(kappaShift)), sn = Math.round(Math.sin(kappaShift));
    double px = c * ppx + sn * ppy, py = -sn * ppx + c * ppy;

    // Return strips have kappa near 200 gon; in degrees that would be near 180.
    if (toRad == Math.PI / 200 && exteriors.stream().allMatch(e -> Math.abs(e.kappa) <= 360) && exteriors.stream().anyMatch(e -> Math.abs(Math.abs(e.kappa) - 180) < 5)
        && exteriors.stream().noneMatch(e -> Math.abs(Math.abs(e.kappa) - 200) < 5))
      warnings.add("kappa values cluster near 180: are the angles in degrees? (angle unit is gon)");

    String cameraId = s.cameraId == null || s.cameraId.isBlank() ? defaultCameraId(report, sensor) : s.cameraId.strip();
    if (cameraId.matches(".*\\s.*")) throw new IllegalArgumentException("camera ID cannot contain spaces: " + cameraId);
    return new Prepared(s, report, sensor, exteriors, files, ext, layout, reason, columns, rows, px, py, kappaShift, cameraId);
  }

  static String defaultCameraId(Report r, Sensor s) {
    String id = (r.serial + (r.revision.isEmpty() ? "" : "_" + r.revision) + (s.name.equals("multispectral") ? "_ms" : "")).replaceAll("[^A-Za-z0-9._-]", "_");
    return id.isEmpty() ? "UltraCam" : id;
  }

  /** The camera definition after its $ID line; equal bodies mean one shared definition. */
  private static String definitionBody(Prepared p) {
    double scale = 1 / p.sensor.pixelMm;
    return new StringBuilder()
        .append("  $MAKER : Vexcel\n")
        .append("  $MODEL : ").append(p.report.camera.replaceAll("\\s+", "-")).append('\n')
        .append("  $SN : ").append(p.report.serial).append('\n')
        .append("  $KIND : CCDFrame\n")
        .append("  $CCD_COLUMNS : ").append(p.columns).append('\n')
        .append("  $CCD_ROWS : ").append(p.rows).append('\n')
        .append("  $PIXEL_REFERENCE : CenterTopLeft\n")
        .append("  $CAMERA_MOUNT_ROTATION :     0.000000\n")
        .append("  $ACTIVE_CALIBRATION : 1\n")
        .append("  $CALIBRATION_SET :\n")
        .append("    $ID : 1\n")
        .append("    $MODE : manual\n")
        .append("    $DATE : 00:00:00 ").append(reportDate(p.report.reportDate)).append('\n')
        .append("    $CCD_INTERIOR_ORIENTATION :\n")
        .append(String.format(Locale.ROOT, "     %16.10f %16.10f %16.10f%n", scale, 0.0, (p.columns - 1) / 2.0))
        .append(String.format(Locale.ROOT, "     %16.10f %16.10f %16.10f%n", 0.0, -scale, (p.rows - 1) / 2.0))
        .append(String.format(Locale.ROOT, "    $FOCAL_LENGTH : %12.6f%n", p.sensor.focalMm))
        .append(String.format(Locale.ROOT, "    $PRINCIPAL_POINT_PPA : %12.6f %12.6f%n", p.ppx, p.ppy))
        .append("  $END\n")
        .append("$END\n")
        .toString();
  }

  private static String write(Settings s, List<Prepared> cameras, Map<String, String> definitions, double toRad) {
    StringBuilder b = new StringBuilder();
    b.append("$PROJECT 5.0.0\n");
    for (Prepared p : cameras)
      b.append("# Written by POSE ultracam-inpho from ").append(p.in.exteriors().getFileName()).append(" and ").append(p.in.report().getFileName()).append('\n');
    b.append("  $PROJECT_NAME : ").append(stem(s.output)).append('\n')
        .append("  $IMAGE_TYPE : Aerial\n")
        .append("  $LINEAR_UNITS_OF_OBJECT : m\n")
        .append("  $LINEAR_UNITS_OF_IMAGE : mm\n")
        .append("  $ANGULAR_UNITS : grad\n")
        .append("  $REFRACT_CORR_DEFAULT : on\n")
        .append("  $CURV_CORR_DEFAULT : on\n")
        .append("$END\n");
    for (var d : definitions.entrySet()) b.append("$CAMERA_DEFINITION\n  $ID : ").append(d.getKey()).append('\n').append(d.getValue());
    for (Prepared p : cameras)
      for (Exterior e : p.exteriors) {
        double[] r = cameraToWorld(e.omega * toRad, e.phi * toRad, e.kappa * toRad + p.kappaShift);
        Path file = p.files.get(e.name);
        Path folder = p.in.images();
        String photoFile = file != null ? file.toAbsolutePath().toString() : folder != null ? folder.resolve(e.name + p.ext).toAbsolutePath().toString() : e.name + p.ext;
        b.append("$PHOTO\n")
            .append("  $PHOTO_NUM : ").append(e.name).append('\n')
            .append("  $PHOTO_FILE : ").append(photoFile).append('\n')
            .append("  $CAMERA_ID : ").append(p.cameraId).append('\n');
        if (s.terrainHeight != null)
          b.append(String.format(Locale.ROOT, "  $TERRAIN_HEIGHT : %.6f%n", s.terrainHeight))
              .append(String.format(Locale.ROOT, "  $OBJECT_DISTANCE : %.6f%n", e.z - s.terrainHeight));
        b.append("  $ORI_STAT : AbsoluteAdjusted\n")
            .append("  $EXT_ORI : 00:00:00 ").append(reportDate(p.report.reportDate)).append('\n')
            .append(String.format(Locale.ROOT, "     %12.5f %15.5f %15.5f %15.5f%n", p.sensor.focalMm, e.x, e.y, e.z));
        for (int k = 0; k < 3; k++)
          b.append(String.format(Locale.ROOT, "     %20.12f %20.12f %20.12f%n", r[k], r[3 + k], r[6 + k]));
        b.append("$END\n");
      }
    return b.toString();
  }

  /** "Jun-21-2019" as INPHO's dd/mm/yyyy; anything unreadable becomes 01/01/2000. */
  static String reportDate(String date) {
    Matcher m = Pattern.compile("([A-Za-z]{3})[a-z]*-(\\d{1,2})-(\\d{4})").matcher(date == null ? "" : date);
    if (m.find()) {
      int month = List.of("jan", "feb", "mar", "apr", "may", "jun", "jul", "aug", "sep", "oct", "nov", "dec").indexOf(m.group(1).toLowerCase(Locale.ROOT)) + 1;
      if (month > 0) return String.format(Locale.ROOT, "%02d/%02d/%s", Integer.parseInt(m.group(2)), month, m.group(3));
    }
    return "01/01/2000";
  }

  /** Row-major Rx(omega) Ry(phi) Rz(kappa): photogrammetric camera axes to world. */
  static double[] cameraToWorld(double om, double ph, double ka) {
    double co = Math.cos(om), so = Math.sin(om), cp = Math.cos(ph), sp = Math.sin(ph), ck = Math.cos(ka), sk = Math.sin(ka);
    return new double[] {
      cp * ck, -cp * sk, sp,
      co * sk + so * sp * ck, co * ck - so * sp * sk, -so * cp,
      so * sk - co * sp * ck, so * ck + co * sp * sk, co * cp
    };
  }

  private static String stem(Path p) {
    String n = p.getFileName().toString();
    int dot = n.lastIndexOf('.');
    return dot > 0 ? n.substring(0, dot) : n;
  }

  /** Width and height from the image header (ImageIO), with a TIFF/BigTIFF fallback. */
  static int[] imageSize(Path p) throws IOException {
    try {
      return ResolutionPlan.originalSize(p);
    } catch (IOException | RuntimeException e) {
      return tiffSize(p);
    }
  }

  static int[] tiffSize(Path p) throws IOException {
    try (FileChannel ch = FileChannel.open(p)) {
      ByteBuffer b = ByteBuffer.allocate(16);
      ch.read(b, 0);
      if (b.position() < 8) throw new IOException("not a TIFF file");
      b.order(b.get(0) == 'I' ? ByteOrder.LITTLE_ENDIAN : ByteOrder.BIG_ENDIAN);
      int magic = b.getShort(2) & 0xffff;
      boolean big = magic == 43;
      if (magic != 42 && !big) throw new IOException("not a TIFF file");
      long ifd = big ? b.getLong(8) : b.getInt(4) & 0xffffffffL;
      ByteBuffer c = ByteBuffer.allocate(big ? 8 : 2).order(b.order());
      ch.read(c, ifd);
      long count = big ? c.getLong(0) : c.getShort(0) & 0xffff;
      if (count <= 0 || count > 4096) throw new IOException("bad TIFF directory");
      int entry = big ? 20 : 12;
      ByteBuffer e = ByteBuffer.allocate((int) count * entry).order(b.order());
      ch.read(e, ifd + (big ? 8 : 2));
      int w = -1, h = -1;
      for (int i = 0; i < count; i++) {
        int o = i * entry, tag = e.getShort(o) & 0xffff, type = e.getShort(o + 2) & 0xffff, at = o + (big ? 12 : 8);
        int value = type == 3 ? e.getShort(at) & 0xffff : e.getInt(at);
        if (tag == 256) w = value;
        if (tag == 257) h = value;
      }
      if (w <= 0 || h <= 0) throw new IOException("TIFF without width/height");
      return new int[] {w, h};
    }
  }

  // ----- command line -----

  static final String HELP =
      """
      photogrammetry ultracam-inpho --exteriors FILE --calibration-report PDF [camera options]
                                    [--exteriors FILE --calibration-report PDF [camera options]]...
                                    --output FILE.prj [options]
        Convert UltraCam exterior orientations and the Vexcel calibration report into an
        INPHO project file for POSE (--orientation-input FILE.prj) and Metashape (Import
        Cameras, INPHO project). Several cameras: give an exterior file and a calibration
        report for each; they are written into one project file, one camera definition each.
        Camera options (each camera; a new camera starts when --exteriors or another camera
        option is given a second time):
        --exteriors FILE            text file, one image per line: name X Y Z omega phi kappa
                                    (more columns are ignored; spaces, tabs, commas or semicolons)
        --calibration-report PDF    Vexcel field/lab calibration report (focal length, principal
                                    point, pixel size and image format are read from it)
        --images DIR                image folder (optional): file names and the image layout
                                    are taken from the images found there; a camera without
                                    one uses the folder of the camera before it
        --image-extension EXT       extension used when an image is not found (default .tif;
                                    carried over like --images)
        --layout L                  stored pixels against the flight direction:
                                      auto (default): from the first image's size; without
                                        images the report's frame (along-track)
                                      along-track: long-track side is the image width, flight
                                        towards the right (kappa as given)
                                      along-track-reversed: flight towards the left (+200 gon)
                                      up: cross-track side is the width, flight towards the
                                        image top (-100 gon)
                                      down: flight towards the image bottom (+100 gon)
        --sensor pan|ms             panchromatic/RGB (default) or multispectral image format
        --camera-id ID              camera ID (default: serial and revision from the report;
                                    cameras with the same ID and calibration share one definition)
        Options for the whole project:
        --output FILE.prj           project file to write
        --angle-unit gon|deg|rad    unit of omega, phi, kappa (default gon)
        --terrain-height M          optional mean terrain height written per photo
        Image names must be unique over all cameras.
      """;

  static final List<String> CAMERA_OPTIONS =
      List.of("--exteriors", "--calibration-report", "--images", "--image-extension", "--layout", "--sensor", "--camera-id");
  static final List<String> PROJECT_OPTIONS = List.of("--output", "--angle-unit", "--terrain-height");

  static int main(String[] args) {
    if (args.length == 0 || Arrays.asList(args).contains("-h") || Arrays.asList(args).contains("--help")) {
      System.out.print(HELP);
      return args.length == 0 ? 2 : 0;
    }
    try {
      Result r = convert(parse(args));
      System.out.print(r.summary());
      return 0;
    } catch (IllegalArgumentException | IOException e) {
      System.err.println("photogrammetry ultracam-inpho: " + e.getMessage());
      return 2;
    }
  }

  /** Command-line options as settings; the GUI builds the same argument list. */
  static Settings parse(String[] args) {
    List<Map<String, String>> cameras = new ArrayList<>();
    Map<String, String> project = new HashMap<>();
    Map<String, String> current = null;
    for (int i = 0; i < args.length; i++) {
      String key = switch (args[i]) {
        case "-o" -> "--output";
        case "-i" -> "--images";
        default -> args[i];
      };
      if (!CAMERA_OPTIONS.contains(key) && !PROJECT_OPTIONS.contains(key))
        throw new IllegalArgumentException("unknown option " + args[i] + " (see ultracam-inpho --help)");
      if (i + 1 >= args.length) throw new IllegalArgumentException(args[i] + " needs a value");
      String value = args[++i];
      if (PROJECT_OPTIONS.contains(key)) {
        if (project.put(key, value) != null) throw new IllegalArgumentException(key + " is given twice");
        continue;
      }
      if (current == null || current.containsKey(key)) cameras.add(current = new HashMap<>());
      current.put(key, value);
    }
    if (cameras.isEmpty()) throw new IllegalArgumentException("--exteriors and --calibration-report are required (see ultracam-inpho --help)");
    if (!project.containsKey("--output")) throw new IllegalArgumentException("--output is required (see ultracam-inpho --help)");
    return settings(cameras, project);
  }

  static Settings settings(List<Map<String, String>> cameraValues, Map<String, String> v) {
    List<CameraInput> cameras = new ArrayList<>();
    String images = null, extension = ".tif";
    for (int n = 0; n < cameraValues.size(); n++) {
      Map<String, String> c = cameraValues.get(n);
      String which = cameraValues.size() > 1 ? "camera " + (n + 1) + ": " : "";
      if (!c.containsKey("--exteriors")) throw new IllegalArgumentException(which + "--exteriors is required (see ultracam-inpho --help)");
      if (!c.containsKey("--calibration-report"))
        throw new IllegalArgumentException(which + "--calibration-report is required for " + c.get("--exteriors") + " (see ultracam-inpho --help)");
      Path ext = Path.of(c.get("--exteriors")), report = Path.of(c.get("--calibration-report"));
      if (!Files.isRegularFile(ext)) throw new IllegalArgumentException(which + "exterior orientation file not found: " + ext);
      if (!Files.isRegularFile(report)) throw new IllegalArgumentException(which + "calibration report not found: " + report);
      String sensor = c.getOrDefault("--sensor", "pan");
      if (!sensor.equals("pan") && !sensor.equals("ms")) throw new IllegalArgumentException(which + "--sensor must be pan or ms");
      images = c.getOrDefault("--images", images);
      extension = c.getOrDefault("--image-extension", extension);
      cameras.add(new CameraInput(ext, report, images == null ? null : Path.of(images), extension, c.getOrDefault("--layout", "auto"), sensor, c.get("--camera-id")));
    }
    String terrain = v.get("--terrain-height");
    Double height;
    try {
      height = terrain == null || terrain.isBlank() ? null : Double.parseDouble(terrain.strip());
    } catch (NumberFormatException e) {
      throw new IllegalArgumentException("--terrain-height must be a number: " + terrain);
    }
    return new Settings(cameras, Path.of(v.get("--output")), v.getOrDefault("--angle-unit", "gon"), height);
  }
}
