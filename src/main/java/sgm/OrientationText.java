package sgm;

import static sgm.OrientationIO.*;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.*;
import java.util.regex.*;

/** CSV, Bundler, COLMAP text/binary and digital-pinhole INPHO PRJ 5/8 adapters. */
final class OrientationText {
  static Model read(Path path, String format, Map<String, int[]> sizes, Options o)
      throws IOException {
    return switch (format) {
      case "native" -> nativeFile(path, sizes, o);
      case "csv", "csv-opk" -> csv(path, format, o);
      case "bundler" -> bundler(path, sizes, o);
      case "colmap" -> colmap(path, o);
      case "inpho" -> inpho(path, sizes, o);
      default -> throw new IllegalArgumentException("Unknown text orientation format");
    };
  }

  private static Model nativeFile(Path path, Map<String, int[]> sizes, Options o)
      throws IOException {
    String raw = Files.readString(path).replaceFirst("^\ufeff", "");
    if (raw.stripLeading().startsWith("{")) return fromJson(JsonRead.read(path));
    if (raw.startsWith(PoseIO.HEADER)) return csv(path, "csv", o);
    int[] size = sizes.isEmpty() ? new int[] {1, 1} : sizes.values().iterator().next();
    Options copy = o.copy();
    copy.calibration = path;
    Model m = new Model();
    m.add(new View("*", Calibration.load(copy, size[0], size[1]), null));
    return m;
  }

  private static Model csv(Path path, String format, Options o) throws IOException {
    List<String> lines = Files.readAllLines(path);
    int first = 0;
    while (first < lines.size() && (lines.get(first).isBlank() || lines.get(first).startsWith("#")))
      first++;
    if (first == lines.size()) throw new IllegalArgumentException("Empty orientation CSV");
    List<String> header = PoseIO.csv(lines.get(first).replace("\ufeff", ""));
    Map<String, Integer> columns = new LinkedHashMap<>();
    for (int i = 0; i < header.size(); i++)
      columns.put(header.get(i).strip().toLowerCase(Locale.ROOT), i);
    Model model = new Model();
    for (int line = first + 1; line < lines.size(); line++) {
      if (lines.get(line).isBlank() || lines.get(line).startsWith("#")) continue;
      List<String> row = PoseIO.csv(lines.get(line));
      if (row.size() != header.size())
        throw new IllegalArgumentException("CSV column count differs at line " + (line + 1));
      Map<String, String> values = new HashMap<>();
      for (var e : columns.entrySet()) values.put(e.getKey(), row.get(e.getValue()));
      if (values.containsKey("registered") && !Boolean.parseBoolean(values.get("registered")))
        continue;
      String image = values.getOrDefault("image", values.getOrDefault("filename", "*"));
      Calibration c = null;
      Camera pose = null;
      if (values.containsKey("fx")) {
        double[] d = {
          get(values, "k1", 0),
          get(values, "k2", 0),
          get(values, "p1", 0),
          get(values, "p2", 0),
          get(values, "k3", 0)
        };
        if (get(values, "skew", 0) != 0)
          throw new IllegalArgumentException("Nonzero camera skew unsupported");
        c =
            calibration(
                get(values, "fx", Double.NaN),
                get(values, "fy", Double.NaN),
                get(values, "cx", Double.NaN),
                get(values, "cy", Double.NaN),
                d,
                (int) get(values, "width", 0),
                (int) get(values, "height", 0));
      }
      if (values.containsKey("r00")) {
        double[] r = new double[9];
        for (int i = 0; i < 9; i++) r[i] = get(values, "r" + (i / 3) + (i % 3), Double.NaN);
        pose =
            pose(
                r,
                new double[] {
                  get(values, "tx", Double.NaN),
                  get(values, "ty", Double.NaN),
                  get(values, "tz", Double.NaN)
                });
      } else if (values.containsKey("qw")) {
        double[] r =
            quaternion(
                new double[] {
                  get(values, "qw", Double.NaN),
                  get(values, "qx", Double.NaN),
                  get(values, "qy", Double.NaN),
                  get(values, "qz", Double.NaN)
                });
        pose =
            pose(
                r,
                new double[] {
                  get(values, "tx", Double.NaN),
                  get(values, "ty", Double.NaN),
                  get(values, "tz", Double.NaN)
                });
      } else if (format.equals("csv-opk") || values.containsKey("omega")) {
        double factor = o.orientationAngleUnit.equals("degrees") ? Math.PI / 180 : 1;
        double[] r =
            euler(
                get(values, "omega", 0) * factor,
                get(values, "phi", 0) * factor,
                get(values, "kappa", 0) * factor);
        r = flipRows(Math3.transpose(r));
        pose =
            centered(
                r,
                new double[] {
                  get(values, "x", get(values, "center_x", Double.NaN)),
                  get(values, "y", get(values, "center_y", Double.NaN)),
                  get(values, "z", get(values, "center_z", Double.NaN))
                });
      }
      model.add(new View(image, c, pose));
    }
    return model;
  }

  private static double get(Map<String, String> m, String k, double def) {
    String value = m.get(k);
    return value == null || value.isBlank() ? def : Double.parseDouble(value);
  }

  static double[] quaternion(double[] q) {
    double norm = 0;
    for (double v : q) norm += v * v;
    if (!(norm > 0)) throw new IllegalArgumentException("Zero quaternion");
    norm = Math.sqrt(norm);
    double w = q[0] / norm, x = q[1] / norm, y = q[2] / norm, z = q[3] / norm;
    return new double[] {
      1 - 2 * (y * y + z * z),
      2 * (x * y - z * w),
      2 * (x * z + y * w),
      2 * (x * y + z * w),
      1 - 2 * (x * x + z * z),
      2 * (y * z - x * w),
      2 * (x * z - y * w),
      2 * (y * z + x * w),
      1 - 2 * (x * x + y * y)
    };
  }

  static double[] quaternion(Camera c) {
    double[] r = c.rotation;
    double w, x, y, z;
    double trace = r[0] + r[4] + r[8];
    if (trace > 0) {
      double s = Math.sqrt(trace + 1) * 2;
      w = .25 * s;
      x = (r[7] - r[5]) / s;
      y = (r[2] - r[6]) / s;
      z = (r[3] - r[1]) / s;
    } else if (r[0] > r[4] && r[0] > r[8]) {
      double s = Math.sqrt(1 + r[0] - r[4] - r[8]) * 2;
      w = (r[7] - r[5]) / s;
      x = .25 * s;
      y = (r[1] + r[3]) / s;
      z = (r[2] + r[6]) / s;
    } else if (r[4] > r[8]) {
      double s = Math.sqrt(1 + r[4] - r[0] - r[8]) * 2;
      w = (r[2] - r[6]) / s;
      x = (r[1] + r[3]) / s;
      y = .25 * s;
      z = (r[5] + r[7]) / s;
    } else {
      double s = Math.sqrt(1 + r[8] - r[0] - r[4]) * 2;
      w = (r[3] - r[1]) / s;
      x = (r[2] + r[6]) / s;
      y = (r[5] + r[7]) / s;
      z = .25 * s;
    }
    return new double[] {w, x, y, z};
  }

  static double[] flipRows(double[] r) {
    r = r.clone();
    for (int i = 3; i < 9; i++) r[i] = -r[i];
    return r;
  }

  static double[] flip(double[] v) {
    return new double[] {v[0], -v[1], -v[2]};
  }

  static double[] euler(double o, double p, double k) {
    double so = Math.sin(o),
        co = Math.cos(o),
        sp = Math.sin(p),
        cp = Math.cos(p),
        sk = Math.sin(k),
        ck = Math.cos(k);
    return new double[] {
      ck * cp,
      ck * sp * so - sk * co,
      ck * sp * co + sk * so,
      sk * cp,
      sk * sp * so + ck * co,
      sk * sp * co - ck * so,
      -sp,
      cp * so,
      cp * co
    };
  }

  static double[] opk(Camera c) {
    double[] r = Math3.transpose(flipRows(c.rotation));
    double p = Math.asin(Math.max(-1, Math.min(1, -r[6])));
    if (Math.abs(Math.cos(p)) < 1e-8)
      throw new IllegalArgumentException(
          "OPK Euler export is singular near phi=90 degrees; choose matrix/quaternion format");
    return new double[] {Math.atan2(r[7], r[8]), p, Math.atan2(r[3], r[0])};
  }

  private static Model bundler(Path input, Map<String, int[]> sizes, Options o) throws IOException {
    Path file = Files.isDirectory(input) ? input.resolve("bundle.out") : input;
    Path list =
        o.orientationImageList != null ? o.orientationImageList : file.resolveSibling("list.txt");
    if (!Files.isRegularFile(list))
      throw new IllegalArgumentException(
          "Bundler requires --orientation-image-list or sibling list.txt");
    List<String> names = new ArrayList<>();
    for (String line : Files.readAllLines(list))
      if (!line.isBlank() && !line.startsWith("#"))
        names.add(
            line.strip().startsWith("\"")
                ? PoseIO.csv(line).getFirst()
                : line.strip().split("\\s+")[0]);
    String text = Files.readString(file).replaceAll("(?m)^#.*$", "");
    try (Scanner scan = new Scanner(text).useLocale(Locale.ROOT)) {
      int count = scan.nextInt();
      scan.nextInt();
      if (count != names.size())
        throw new IllegalArgumentException("Bundler image-list count differs from camera count");
      Model m = new Model();
      for (String image : names) {
        double f = scan.nextDouble(), k1 = scan.nextDouble(), k2 = scan.nextDouble();
        double[] r = new double[9], t = new double[3];
        for (int i = 0; i < 9; i++) r[i] = scan.nextDouble();
        for (int i = 0; i < 3; i++) t[i] = scan.nextDouble();
        if (f <= 0) continue;
        String key = name(image);
        int[] size = sizes.get(key);
        if (size == null) {
          Path images = file.resolveSibling("images.csv");
          if (Files.exists(images)) {
            Model metadata = csv(images, "csv", o);
            View v = metadata.views.get(key);
            if (v != null) size = new int[] {v.calibration().width, v.calibration().height};
          }
        }
        if (size == null)
          throw new IllegalArgumentException(
              "Bundler camera dimensions require input images or sibling images.csv");
        m.add(
            new View(
                key,
                calibration(
                    f, f, size[0] * .5, size[1] * .5, new double[] {k1, k2}, size[0], size[1]),
                pose(flipRows(r), flip(t))));
      }
      return m;
    }
  }

  private static Calibration colmapCalibration(String model, int w, int h, double[] v) {
    return switch (model) {
      case "SIMPLE_PINHOLE" -> calibration(v[0], v[0], v[1] - .5, v[2] - .5, new double[5], w, h);
      case "PINHOLE" -> calibration(v[0], v[1], v[2] - .5, v[3] - .5, new double[5], w, h);
      case "SIMPLE_RADIAL" ->
          calibration(v[0], v[0], v[1] - .5, v[2] - .5, new double[] {v[3]}, w, h);
      case "RADIAL" ->
          calibration(v[0], v[0], v[1] - .5, v[2] - .5, new double[] {v[3], v[4]}, w, h);
      case "OPENCV" ->
          calibration(
              v[0], v[1], v[2] - .5, v[3] - .5, new double[] {v[4], v[5], v[6], v[7]}, w, h);
      case "FULL_OPENCV" -> {
        if (v[9] != 0 || v[10] != 0 || v[11] != 0)
          throw new IllegalArgumentException(
              "COLMAP rational denominator distortion is unsupported");
        yield calibration(
            v[0], v[1], v[2] - .5, v[3] - .5, new double[] {v[4], v[5], v[6], v[7], v[8]}, w, h);
      }
      default -> throw new IllegalArgumentException("Unsupported COLMAP camera model: " + model);
    };
  }

  private static Model colmap(Path input, Options o) throws IOException {
    Path dir = Files.isDirectory(input) ? input : input.toAbsolutePath().getParent();
    Map<Integer, Calibration> cameras = new LinkedHashMap<>();
    Model m = new Model();
    if (Files.exists(dir.resolve("cameras.txt"))) {
      for (String line : Files.readAllLines(dir.resolve("cameras.txt"))) {
        if (line.isBlank() || line.startsWith("#")) continue;
        String[] t = line.strip().split("\\s+");
        cameras.put(
            Integer.parseInt(t[0]),
            colmapCalibration(
                t[1],
                Integer.parseInt(t[2]),
                Integer.parseInt(t[3]),
                numbers(String.join(" ", Arrays.copyOfRange(t, 4, t.length)))));
      }
      List<String> lines = Files.readAllLines(dir.resolve("images.txt"));
      for (int i = 0; i < lines.size(); i++) {
        String line = lines.get(i);
        if (line.isBlank() || line.startsWith("#")) continue;
        String[] t = line.strip().split("\\s+", 10);
        if (t.length != 10) throw new IllegalArgumentException("Malformed COLMAP image record");
        double[]
            q =
                {
                  Double.parseDouble(t[1]),
                  Double.parseDouble(t[2]),
                  Double.parseDouble(t[3]),
                  Double.parseDouble(t[4])
                },
            v = {Double.parseDouble(t[5]), Double.parseDouble(t[6]), Double.parseDouble(t[7])};
        m.add(
            new View(
                t[9],
                Objects.requireNonNull(
                    cameras.get(Integer.parseInt(t[8])), "Unknown COLMAP camera ID"),
                pose(quaternion(q), v)));
        i++;
      }
    } else {
      String[] models = {
        "SIMPLE_PINHOLE",
        "PINHOLE",
        "SIMPLE_RADIAL",
        "RADIAL",
        "OPENCV",
        "OPENCV_FISHEYE",
        "FULL_OPENCV"
      };
      int[] parameters = {3, 4, 4, 5, 8, 8, 12};
      try (DataInputStream in =
          new DataInputStream(
              new BufferedInputStream(Files.newInputStream(dir.resolve("cameras.bin"))))) {
        long count = leLong(in);
        if (count < 0 || count > 1000000) throw new IOException("Invalid COLMAP camera count");
        for (long i = 0; i < count; i++) {
          int id = leInt(in), model = leInt(in);
          int w = Math.toIntExact(leLong(in)), h = Math.toIntExact(leLong(in));
          if (model < 0 || model >= models.length)
            throw new IOException("Unsupported COLMAP binary camera model");
          double[] p = new double[parameters[model]];
          for (int j = 0; j < p.length; j++) p[j] = leDouble(in);
          cameras.put(id, colmapCalibration(models[model], w, h, p));
        }
      }
      try (DataInputStream in =
          new DataInputStream(
              new BufferedInputStream(Files.newInputStream(dir.resolve("images.bin"))))) {
        long count = leLong(in);
        if (count < 0 || count > 1000000) throw new IOException("Invalid COLMAP image count");
        for (long i = 0; i < count; i++) {
          leInt(in);
          double[] q = new double[4], t = new double[3];
          for (int j = 0; j < 4; j++) q[j] = leDouble(in);
          for (int j = 0; j < 3; j++) t[j] = leDouble(in);
          int camera = leInt(in);
          ByteArrayOutputStream name = new ByteArrayOutputStream();
          for (int c; (c = in.readUnsignedByte()) != 0; ) {
            name.write(c);
            if (name.size() > 65536) throw new IOException("Invalid COLMAP image name");
          }
          long points = leLong(in);
          if (points < 0 || points > Integer.MAX_VALUE)
            throw new IOException("Invalid COLMAP feature count");
          in.skipNBytes(Math.multiplyExact(points, 24));
          m.add(
              new View(
                  name.toString(StandardCharsets.UTF_8),
                  Objects.requireNonNull(cameras.get(camera)),
                  pose(quaternion(q), t)));
        }
      }
    }
    return m;
  }

  private static int leInt(DataInputStream in) throws IOException {
    return Integer.reverseBytes(in.readInt());
  }

  private static long leLong(DataInputStream in) throws IOException {
    return Long.reverseBytes(in.readLong());
  }

  private static double leDouble(DataInputStream in) throws IOException {
    return Double.longBitsToDouble(leLong(in));
  }

  private static Map<String, String> tags(String text) {
    Map<String, String> m = new HashMap<>();
    Matcher matcher = Pattern.compile("(?m)^\\s*\\$([A-Z_]+)\\s*:[ \\t]*(.*)$").matcher(text);
    while (matcher.find()) m.put(matcher.group(1), matcher.group(2).strip());
    return m;
  }

  private static String activeCamera(String block) {
    int start = block.indexOf("$CALIBRATION_SET");
    if (start < 0) return block;
    String header = block.substring(0, start),
        active = tags(header).get("ACTIVE_CALIBRATION"),
        selected = null;
    Matcher sets =
        Pattern.compile("(?ms)^\\s*\\$CALIBRATION_SET\\s*:?[^\\r\\n]*\\R(.*?)(?=^\\s*\\$END\\s*$)")
            .matcher(block);
    int count = 0;
    while (sets.find()) {
      count++;
      String set = sets.group(1);
      if (Objects.equals(active, tags(set).get("ID")) || active == null && selected == null)
        selected = set;
    }
    if (selected == null || active == null && count > 1)
      throw new IllegalArgumentException("INPHO active calibration set is missing or ambiguous");
    return header + "\n" + selected;
  }

  private static void checkInphoDistortion(String text, Model m, Options o) {
    Matcher coefficients =
        Pattern.compile("(?m)^\\s*\\$(?:RADIAL_COEFFS|DECENTRE_COEFFS)\\s*:[ \\t]*\\R?([^$]+)")
            .matcher(text);
    boolean nonzero = text.contains("$DISTORTION_GRID");
    while (coefficients.find())
      for (double v : numbers(coefficients.group(1))) if (v != 0) nonzero = true;
    if (nonzero)
      loss(
          m,
          o,
          "INPHO polynomial/grid distortion is not equivalent to the supported OpenCV model;"
              + " distortion is dropped");
  }

  private static Model inpho(Path file, Map<String, int[]> sizes, Options o) throws IOException {
    String text = Files.readString(file);
    Model m = new Model();
    m.units = tags(text).getOrDefault("LINEAR_UNITS_OF_OBJECT", "project_units");
    Map<String, String> definitions = new HashMap<>();
    Matcher definitionsMatcher =
        Pattern.compile(
                "(?ms)^\\s*\\$CAMERA_DEFINITION\\s*\\R(.*?)(?=^\\s*\\$(?:PHOTO|CAMERA_DEFINITION)\\s*$|\\z)")
            .matcher(text);
    while (definitionsMatcher.find()) {
      String block = definitionsMatcher.group(1);
      Matcher id = Pattern.compile("(?m)^\\s*\\$ID\\s*:[ \\t]*(.*)$").matcher(block);
      if (id.find()) definitions.put(id.group(1).strip(), activeCamera(block));
    }
    Matcher photos =
        Pattern.compile("(?ms)^\\s*\\$PHOTO\\s*\\R(.*?)(?=^\\s*\\$END\\s*$)").matcher(text);
    while (photos.find()) {
      String block = photos.group(1);
      Map<String, String> tag = tags(block);
      String image = name(tag.getOrDefault("PHOTO_FILE", tag.getOrDefault("PHOTO_NUM", "")));
      int[] size = sizes.get(image);
      String definition = definitions.get(tag.get("CAMERA_ID"));
      Map<String, String> cameraTags = definition == null ? Map.of() : tags(definition);
      checkInphoDistortion((definition == null ? "" : definition) + "\n" + block, m, o);
      if (size == null && cameraTags.containsKey("CCD_COLUMNS"))
        size =
            new int[] {
              Integer.parseInt(cameraTags.get("CCD_COLUMNS")),
              Integer.parseInt(cameraTags.get("CCD_ROWS"))
            };
      if (size == null && tag.containsKey("IMAGE_SIZE")) {
        double[] xy = numbers(tag.get("IMAGE_SIZE"));
        size = new int[] {(int) xy[0], (int) xy[1]};
      }
      Matcher eo = Pattern.compile("(?m)^\\s*\\$EXT_ORI[^\\r\\n]*\\R\\s*([^$]+)").matcher(block);
      if (!eo.find()) continue;
      double[] values = numbers(eo.group(1));
      if (values.length < 13)
        throw new IllegalArgumentException("INPHO EXT_ORI needs focal, centre and 3x3 rotation");
      double[] rotation = Arrays.copyOfRange(values, 4, 13);
      if (!o.orientationRotation.equals("world-to-camera")) rotation = Math3.transpose(rotation);
      if (!o.orientationAxes.equals("opencv")) rotation = flipRows(rotation);
      Camera pose = centered(rotation, Arrays.copyOfRange(values, 1, 4));
      double pixel =
          Double.isFinite(o.orientationPixelSize)
              ? o.orientationPixelSize
              : Double.parseDouble(tag.getOrDefault("PIXEL_SIZE", "NaN"));
      Calibration calibration = null;
      Matcher affine =
          Pattern.compile("(?m)^\\s*\\$CCD_INTERIOR_ORIENTATION[^\\r\\n]*\\R\\s*([^$]+)")
              .matcher(definition == null ? "" : definition);
      double[] interior = affine.find() ? numbers(affine.group(1)) : null;
      if (interior != null && interior.length != 6)
        throw new IllegalArgumentException("INPHO CCD interior must contain six affine terms");
      if (interior != null
          && (interior[1] != 0 || interior[3] != 0 || interior[0] <= 0 || interior[4] >= 0))
        throw new IllegalArgumentException(
            "INPHO rotated/skewed/reflected sensor requires image resampling before import");
      if (size != null && (pixel > 0 || interior != null)) {
        if (interior != null) pixel = 1 / interior[0];
        double cx = interior == null ? (size[0] - 1) * .5 : interior[2],
            cy = interior == null ? (size[1] - 1) * .5 : interior[5];
        if (cameraTags.getOrDefault("PIXEL_REFERENCE", "CenterTopLeft").equals("TopLeftTopLeft")) {
          cx -= .5;
          cy -= .5;
        }
        if (cameraTags.containsKey("PRINCIPAL_POINT_PPA")) {
          double[] principal = numbers(cameraTags.get("PRINCIPAL_POINT_PPA"));
          cx += principal[0] / pixel;
          cy -= principal[1] / pixel;
        }
        if (tag.containsKey("PRINCIPAL_POINT_PIXELS")) {
          double[] principal = numbers(tag.get("PRINCIPAL_POINT_PIXELS"));
          cx = principal[0];
          cy = principal[1];
        }
        calibration =
            calibration(
                values[0] / pixel,
                interior == null ? values[0] / pixel : -values[0] * interior[4],
                cx,
                cy,
                new double[5],
                size[0],
                size[1]);
      }
      m.add(new View(image, calibration, pose));
    }
    m.notes.add(
        "INPHO digital pinhole EXT_ORI; default camera-to-world photogrammetric axes; pixel size in"
            + " mm; nonzero scanner/interior-affine rotations require conversion");
    return m;
  }

  static void write(Model m, Path dir, String format, Options o) throws IOException {
    if (format.equals("colmap")) {
      writeColmap(m, dir);
      return;
    }
    if (format.equals("bundler")) {
      writeBundler(m, dir, o);
      return;
    }
    if (format.equals("inpho")) {
      writeInpho(m, dir, o);
      return;
    }
    try (BufferedWriter io = Files.newBufferedWriter(dir.resolve("interiors.csv"));
        BufferedWriter eo = Files.newBufferedWriter(dir.resolve("exteriors.csv"));
        BufferedWriter both = Files.newBufferedWriter(dir.resolve("orientations.csv"))) {
      String ih = "image,width,height,fx,fy,cx,cy,k1,k2,p1,p2,k3",
          eh = format.equals("csv-opk") ? "image,x,y,z,omega,phi,kappa" : PoseIO.HEADER;
      io.write(ih + "\n");
      eo.write(eh + "\n");
      both.write(
          ih
              + ","
              + (format.equals("csv-opk") ? "x,y,z,omega,phi,kappa" : PoseIO.HEADER.substring(6))
              + "\n");
      for (View v : m.views.values()) {
        Calibration c = Objects.requireNonNull(v.calibration(), "Export requires intrinsics");
        String image = PoseIO.quote(v.image());
        String interior =
            image + "," + c.width + "," + c.height + "," + c.fx + "," + c.fy + "," + c.cx + ","
                + c.cy;
        for (double d : c.distortion) interior += "," + d;
        io.write(interior + "\n");
        if (v.pose() != null) {
          String pose = "";
          if (format.equals("csv-opk")) {
            double[] center = v.pose().center(), angles = opk(v.pose());
            for (double d : center) pose += "," + d;
            for (double d : angles)
              pose += "," + (o.orientationAngleUnit.equals("degrees") ? Math.toDegrees(d) : d);
          } else {
            for (double d : v.pose().rotation) pose += "," + d;
            for (double d : v.pose().translation) pose += "," + d;
          }
          eo.write(image + pose + "\n");
          both.write(interior + pose + "\n");
        }
      }
    }
  }

  private static void writeColmap(Model m, Path dir) throws IOException {
    try (BufferedWriter cameras = Files.newBufferedWriter(dir.resolve("cameras.txt"));
        BufferedWriter images = Files.newBufferedWriter(dir.resolve("images.txt"))) {
      int id = 0;
      for (View v : m.views.values()) {
        id++;
        Calibration c = v.calibration();
        if (c == null) throw new IllegalArgumentException("COLMAP export requires intrinsics");
        String model = c.distortion[4] == 0 ? "OPENCV" : "FULL_OPENCV";
        cameras.write(
            id
                + " "
                + model
                + " "
                + c.width
                + " "
                + c.height
                + " "
                + c.fx
                + " "
                + c.fy
                + " "
                + (c.cx + .5)
                + " "
                + (c.cy + .5));
        for (int j = 0; j < 4; j++) cameras.write(" " + c.distortion[j]);
        if (model.equals("FULL_OPENCV")) cameras.write(" " + c.distortion[4] + " 0 0 0");
        cameras.newLine();
        if (v.pose() != null) {
          images.write(Integer.toString(id));
          for (double q : quaternion(v.pose())) images.write(" " + q);
          for (double t : v.pose().translation) images.write(" " + t);
          images.write(" " + id + " " + v.image() + "\n\n");
        }
      }
    }
    Files.writeString(
        dir.resolve("points3D.txt"), "# Orientations only; no sparse observations exported\n");
  }

  private static void writeBundler(Model m, Path dir, Options o) throws IOException {
    try (BufferedWriter out = Files.newBufferedWriter(dir.resolve("bundle.out"));
        BufferedWriter list = Files.newBufferedWriter(dir.resolve("list.txt"))) {
      out.write("# Bundle file v0.3\n" + m.views.size() + " 0\n");
      for (View v : m.views.values()) {
        Calibration c = v.calibration();
        Camera p = v.pose();
        if (c == null || p == null)
          throw new IllegalArgumentException(
              "Bundler export requires complete interior/exterior data");
        if (c.fx != c.fy
            || Math.abs(c.cx - c.width * .5) > 1e-8
            || Math.abs(c.cy - c.height * .5) > 1e-8
            || c.distortion[2] != 0
            || c.distortion[3] != 0
            || c.distortion[4] != 0)
          loss(
              m,
              o,
              "Bundler has one focal length, centred principal point, k1/k2 only; other intrinsic"
                  + " terms are approximated");
        out.write((c.fx + c.fy) * .5 + " " + c.distortion[0] + " " + c.distortion[1] + "\n");
        double[] r = flipRows(p.rotation), t = flip(p.translation);
        for (int i = 0; i < 3; i++)
          out.write(r[i * 3] + " " + r[i * 3 + 1] + " " + r[i * 3 + 2] + "\n");
        out.write(t[0] + " " + t[1] + " " + t[2] + "\n");
        if (v.image().contains(" "))
          throw new IllegalArgumentException(
              "Bundler image names containing whitespace are ambiguous");
        list.write(v.image() + "\n");
      }
    }
    write(m, dir, "csv", o);
    Files.copy(dir.resolve("interiors.csv"), dir.resolve("images.csv"));
  }

  private static void writeInpho(Model m, Path dir, Options o) throws IOException {
    if (!(o.orientationPixelSize > 0))
      throw new IllegalArgumentException(
          "INPHO export needs --orientation-pixel-size MM; physical sensor pitch cannot be"
              + " guessed");
    if (m.crs == null && !Set.of("m", "ft", "mm", "metres").contains(m.units))
      throw new IllegalArgumentException(
          "INPHO export requires declared --orientation-world-units m|ft|mm for an unreferenced"
              + " model");
    try (BufferedWriter out = Files.newBufferedWriter(dir.resolve("orientations.prj"))) {
      out.write(
          "$PROJECT 5.0.0\n  $LINEAR_UNITS_OF_OBJECT : "
              + (m.crs == null ? m.units : "m")
              + "\n  $LINEAR_UNITS_OF_IMAGE : mm\n  $ANGULAR_UNITS : deg\n$END\n");
      int id = 0;
      for (View v : m.views.values()) {
        id++;
        Calibration c = v.calibration();
        Camera p = v.pose();
        if (c == null || p == null)
          throw new IllegalArgumentException("INPHO export requires interiors and exteriors");
        if (c.fx != c.fy || Arrays.stream(c.distortion).anyMatch(d -> d != 0))
          loss(
              m,
              o,
              "INPHO pinhole export supports square pixels and distortion-free imagery; choose"
                  + " processed pixels for lossless distortion-free export");
        double f = c.fx * o.orientationPixelSize;
        double[] center = p.center(), r = Math3.transpose(flipRows(p.rotation));
        out.write(
            "$CAMERA_DEFINITION\n  $ID : camera_"
                + id
                + "\n  $KIND : CCDFrame\n  $CCD_COLUMNS : "
                + c.width
                + "\n  $CCD_ROWS : "
                + c.height
                + "\n"
                + "  $PIXEL_REFERENCE : CenterTopLeft\n"
                + "  $ACTIVE_CALIBRATION : 1\n"
                + "  $CALIBRATION_SET :\n"
                + "    $ID : 1\n"
                + "    $MODE : manual\n"
                + "    $FOCAL_LENGTH : "
                + f
                + "\n    $PRINCIPAL_POINT_PPA : 0 0\n    $CCD_INTERIOR_ORIENTATION :\n      "
                + 1 / o.orientationPixelSize
                + " 0 "
                + c.cx
                + "\n      0 "
                + (-1 / o.orientationPixelSize)
                + " "
                + c.cy
                + "\n  $END\n$END\n");
        out.write(
            "$PHOTO\n  $PHOTO_NUM : "
                + id
                + "\n  $PHOTO_FILE : "
                + v.image()
                + "\n  $CAMERA_ID : camera_"
                + id
                + "\n  $PIXEL_SIZE : "
                + o.orientationPixelSize
                + "\n  $SCAN_POS : 0\n  $IO_PARS : 2 2\n  $EXT_ORI : 00:00:00 01/01/2000\n    "
                + f
                + " "
                + center[0]
                + " "
                + center[1]
                + " "
                + center[2]
                + "\n");
        for (int row = 0; row < 3; row++)
          out.write("    " + r[row * 3] + " " + r[row * 3 + 1] + " " + r[row * 3 + 2] + "\n");
        out.write("$END\n");
      }
    }
  }
}
