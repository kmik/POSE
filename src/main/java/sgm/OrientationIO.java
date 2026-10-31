package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;

/** Canonical pixel-centre, x-right/y-down/z-forward orientations and explicit format conversion. */
final class OrientationIO {
  static final String FORMATS =
      "native,csv,csv-opk,bundler,colmap,openmvg,opensfm,metashape,micmac,opencv,inpho";

  record View(String image, Calibration calibration, Camera pose) {}

  static final class Model {
    final LinkedHashMap<String, View> views = new LinkedHashMap<>();
    final List<String> notes = new ArrayList<>();
    String crs = null, units = "project_units";

    void add(View v) {
      String name = name(v.image);
      if (views.putIfAbsent(name, new View(name, v.calibration, v.pose)) != null)
        throw new IllegalArgumentException("Duplicate orientation image: " + name);
      if (v.pose != null) validate(v.pose);
    }
  }

  static String name(String s) {
    String normalized = s.replace('\\', '/');
    return normalized.substring(normalized.lastIndexOf('/') + 1);
  }

  static double[] numbers(String text) {
    String[] tokens = text.trim().split("[\\s,;]+");
    double[] values = new double[tokens.length];
    for (int i = 0; i < values.length; i++) {
      values[i] = Double.parseDouble(tokens[i]);
      if (!Double.isFinite(values[i]))
        throw new IllegalArgumentException("Non-finite orientation parameter");
    }
    return values;
  }

  static Camera pose(double[] R, double[] t) {
    if (R.length != 9 || t.length != 3)
      throw new IllegalArgumentException("Expected 3x3 rotation and 3-vector translation");
    Camera c = new Camera(1, 1, 0, 0);
    c.rotation = R;
    c.translation = t;
    c.registered = true;
    validate(c);
    return c;
  }

  static Camera centered(double[] R, double[] center) {
    return pose(R, Math3.times(Math3.mv(R, center), -1));
  }

  static void validate(Camera c) {
    for (double v : c.rotation)
      if (!Double.isFinite(v)) throw new IllegalArgumentException("Non-finite rotation");
    for (double v : c.translation)
      if (!Double.isFinite(v)) throw new IllegalArgumentException("Non-finite translation");
    double[] orth = Math3.mul(c.rotation, Math3.transpose(c.rotation));
    for (int i = 0; i < 9; i++)
      if (Math.abs(orth[i] - (i % 4 == 0 ? 1 : 0)) > 1e-5)
        throw new IllegalArgumentException("Orientation rotation is not orthonormal");
    double[] r = c.rotation;
    double det =
        r[0] * (r[4] * r[8] - r[5] * r[7])
            - r[1] * (r[3] * r[8] - r[5] * r[6])
            + r[2] * (r[3] * r[7] - r[4] * r[6]);
    if (det < 0) throw new IllegalArgumentException("Orientation rotation is reflected");
  }

  static Calibration calibration(
      double fx, double fy, double cx, double cy, double[] distortion, int w, int h) {
    if (distortion.length > 5)
      for (int i = 5; i < distortion.length; i++)
        if (distortion[i] != 0)
          throw new IllegalArgumentException(
              "Nonzero distortion beyond OpenCV k1,k2,p1,p2,k3 is unsupported");
    return new Calibration(fx, fy, cx, cy, Arrays.copyOf(distortion, 5), w, h);
  }

  static String detect(Path path) throws IOException {
    if (Files.isDirectory(path)) {
      if (Files.exists(path.resolve("cameras.txt")) && Files.exists(path.resolve("images.txt")))
        return "colmap";
      if (Files.exists(path.resolve("cameras.bin")) && Files.exists(path.resolve("images.bin")))
        return "colmap";
      if (Files.exists(path.resolve("orientations.json"))) return "native";
      if (Files.exists(path.resolve("cameras.xml"))) return detect(path.resolve("cameras.xml"));
      if (Files.exists(path.resolve("orientations.prj"))) return "inpho";
      if (Files.exists(path.resolve("orientations.csv"))) return "csv";
      if (Files.exists(path.resolve("reconstruction.json"))) return "opensfm";
      if (Files.exists(path.resolve("sfm_data.json"))) return "openmvg";
      if (Files.exists(path.resolve("bundle.out"))) return "bundler";
      try (var files = Files.list(path)) {
        if (files.anyMatch(
            p ->
                p.getFileName().toString().startsWith("Orientation-")
                    && p.toString().endsWith(".xml"))) return "micmac";
      }
      throw new IllegalArgumentException(
          "Cannot detect orientation directory format; specify its format");
    }
    String text = Files.readString(path).replaceFirst("^\ufeff", ""), trim = text.stripLeading();
    if (trim.startsWith("# Bundle")) return "bundler";
    if (trim.startsWith("$PROJECT")
        || trim.startsWith("$PHOTO")
        || trim.contains("$CAMERA_DEFINITION")) return "inpho";
    if (trim.startsWith("%YAML") || trim.contains("<opencv_storage>")) return "opencv";
    if (trim.contains("<OrientationConique")) return "micmac";
    if (trim.contains("<document") || trim.contains("<calibration")) return "metashape";
    if (trim.startsWith("{") || trim.startsWith("[")) {
      Object value = JsonRead.parse(text);
      if (value instanceof List) return "opensfm";
      Map<String, Object> m = JsonRead.object(value);
      if (m.containsKey("sfm_data_version")) return "openmvg";
      if (m.containsKey("shots")) return "opensfm";
      return "native";
    }
    if (text.contains("fx=") || trim.matches("(?s)[+\\-0-9.eE,\\s]+")) return "native";
    return "csv";
  }

  static Model read(Path path, String format, Map<String, int[]> sizes, Options o)
      throws IOException {
    String type = format.equals("auto") ? detect(path) : format;
    if (Files.isDirectory(path))
      path =
          switch (type) {
            case "native" -> path.resolve("orientations.json");
            case "csv", "csv-opk" -> path.resolve("orientations.csv");
            case "inpho" -> path.resolve("orientations.prj");
            case "metashape" -> path.resolve("cameras.xml");
            default -> path;
          };
    Model model =
        switch (type) {
          case "native", "csv", "csv-opk", "bundler", "colmap", "inpho" ->
              OrientationText.read(path, type, sizes, o);
          case "openmvg", "opensfm" -> OrientationJson.read(path, type, sizes, o);
          case "metashape", "micmac", "opencv" -> OrientationXml.read(path, type, sizes, o);
          default -> throw new IllegalArgumentException("Unsupported orientation format: " + type);
        };
    if (model.views.isEmpty())
      throw new IllegalArgumentException("No supported cameras in orientation input: " + path);
    model.notes.add("Imported " + type + " from " + path.toAbsolutePath());
    return model;
  }

  static void prepare(Options o, List<Path> images) throws IOException {
    boolean customCalibration =
        o.calibration != null
            && !(Files.isRegularFile(o.calibration)
                && !Files.readString(o.calibration)
                    .replaceFirst("^\ufeff", "")
                    .stripLeading()
                    .startsWith("{")
                && (o.calibrationFormat.equals("native")
                    || o.calibrationFormat.equals("auto")
                        && detect(o.calibration).equals("native")));
    boolean customPoses =
        o.poses != null
            && !((o.posesFormat.equals("auto") || o.posesFormat.equals("native"))
                && Files.isRegularFile(o.poses)
                && Files.readString(o.poses).startsWith(PoseIO.HEADER));
    boolean custom = o.orientationInput != null || customCalibration || customPoses;
    if (!custom) return;
    Map<String, int[]> sizes = new LinkedHashMap<>();
    for (Path p : images) sizes.put(p.getFileName().toString(), ResolutionPlan.originalSize(p));
    Model combined = new Model();
    if (o.orientationInput != null)
      merge(combined, read(o.orientationInput, o.orientationInputFormat, sizes, o));
    if (customCalibration) merge(combined, read(o.calibration, o.calibrationFormat, sizes, o));
    if (customPoses) merge(combined, read(o.poses, o.posesFormat, sizes, o));
    Map<String, Calibration> calibration = new LinkedHashMap<>();
    Map<String, Camera> poses = new LinkedHashMap<>();
    View shared = combined.views.get("*");
    if (shared != null && shared.pose != null)
      throw new IllegalArgumentException(
          "Shared orientation has a pose but no image name; supply per-image exteriors");
    for (Path image : images) {
      String name = image.getFileName().toString();
      View view = combined.views.get(name);
      Calibration c =
          view != null && view.calibration != null
              ? view.calibration
              : shared == null ? null : shared.calibration;
      if (c != null) calibration.put(name, c);
      if (view != null && view.pose != null) poses.put(name, view.pose);
    }
    if (poses.isEmpty() && combined.views.values().stream().anyMatch(v -> v.pose != null))
      throw new IllegalArgumentException(
          "No exterior-orientation filenames match the input photographs");
    if (calibration.isEmpty()
        && combined.views.values().stream().anyMatch(v -> v.calibration != null))
      throw new IllegalArgumentException(
          "No interior-orientation filenames match the input photographs");
    o.orientationNotes = List.copyOf(combined.notes);
    Path directory = o.output.resolve("00_inputs/imported_orientations");
    Files.createDirectories(directory);
    if (!calibration.isEmpty()) {
      if (calibration.size() != images.size())
        throw new IllegalArgumentException("Orientation input lacks intrinsics for some images");
      Calibration first = calibration.values().iterator().next();
      if (!o.intrinsicsMode.equals("fixed")
          && calibration.values().stream().anyMatch(c -> !same(first, c)))
        throw new IllegalArgumentException(
            "Distinct imported camera calibrations currently require --intrinsics-mode fixed");
      o.orientationCalibrations =
          calibration.values().stream().allMatch(c -> same(first, c))
              ? Map.of()
              : Collections.unmodifiableMap(calibration);
      o.calibration = directory.resolve("camera.properties");
      first.write(o.calibration, "Imported orientation reference calibration");
      // Properties.store adds a wall-clock comment; normalized imports must retain stable cache
      // keys.
      Files.write(
          o.calibration,
          Files.readAllLines(o.calibration).stream()
              .filter(line -> !line.startsWith("#"))
              .sorted()
              .toList());
    }
    if (!poses.isEmpty()) {
      if (o.rectified || o.seedPair != null)
        throw new IllegalArgumentException(
            "Imported exterior orientations cannot be combined with --rectified or --seed-pair");
      o.poses = directory.resolve("poses.csv");
      try (BufferedWriter writer = Files.newBufferedWriter(o.poses)) {
        writer.write(PoseIO.HEADER + "\n");
        for (var entry : poses.entrySet()) {
          writer.write(PoseIO.quote(entry.getKey()));
          for (double v : entry.getValue().rotation) writer.write("," + v);
          for (double v : entry.getValue().translation) writer.write("," + v);
          writer.newLine();
        }
      }
    }
    if (o.orientationCrs == null && combined.crs != null) o.orientationCrs = combined.crs;
    String units = o.orientationUnits.equals("auto") ? combined.units : o.orientationUnits;
    o.orientationSourceUnits = units;
    if (o.orientationCrs != null && !Set.of("project_units", "m", "metres").contains(units))
      throw new IllegalArgumentException(
          "Projected orientation CRS requires metre coordinates; convert "
              + units
              + " coordinates before import");
    Json.write(
        directory.resolve("import.json"),
        Json.object(
            "source_units",
            combined.units,
            "declared_crs",
            combined.crs,
            "notes",
            combined.notes,
            "imported_intrinsics",
            calibration.keySet(),
            "imported_exteriors",
            poses.keySet(),
            "coordinate_transform_applied",
            false));
  }

  private static boolean same(Calibration a, Calibration b) {
    return a.width == b.width
        && a.height == b.height
        && a.fx == b.fx
        && a.fy == b.fy
        && a.cx == b.cx
        && a.cy == b.cy
        && Arrays.equals(a.distortion, b.distortion);
  }

  private static void merge(Model into, Model other) {
    for (var e : other.views.entrySet()) {
      View old = into.views.get(e.getKey()), v = e.getValue();
      into.views.put(
          e.getKey(),
          old == null
              ? v
              : new View(
                  v.image,
                  v.calibration == null ? old.calibration : v.calibration,
                  v.pose == null ? old.pose : v.pose));
    }
    into.notes.addAll(other.notes);
    if (other.crs != null) into.crs = other.crs;
    into.units = other.units;
  }

  static Model fromFrames(List<ImageFrame> frames, String pixels, String crs) {
    Model model = new Model();
    model.crs = crs;
    model.units = crs == null ? "project_units" : "metres";
    for (ImageFrame f : frames) {
      if (!f.camera.registered) continue;
      Calibration c;
      if (pixels.equals("processed"))
        c =
            new Calibration(
                f.camera.fx,
                f.camera.fy,
                f.camera.cx,
                f.camera.cy,
                new double[5],
                f.width,
                f.height);
      else {
        Camera source = f.calibration.camera(f.sourceWidth, f.sourceHeight);
        c =
            new Calibration(
                source.fx,
                source.fy,
                source.cx,
                source.cy,
                f.calibration.distortion.clone(),
                f.sourceWidth,
                f.sourceHeight);
      }
      model.add(new View(f.path.getFileName().toString(), c, f.camera.copy()));
    }
    model.notes.add(
        "Pixels: " + pixels + "; OpenCV camera x right, y down, z forward; X_camera=R*X_world+t");
    return model;
  }

  static Object json(Model model) {
    List<Object> views = new ArrayList<>();
    for (View v : model.views.values()) {
      Calibration c = v.calibration;
      Camera p = v.pose;
      views.add(
          Json.object(
              "image",
              v.image,
              "interior",
              c == null
                  ? null
                  : Json.object(
                      "width",
                      c.width,
                      "height",
                      c.height,
                      "fx",
                      c.fx,
                      "fy",
                      c.fy,
                      "cx",
                      c.cx,
                      "cy",
                      c.cy,
                      "distortion",
                      c.distortion),
              "R",
              p == null ? null : p.rotation,
              "t",
              p == null ? null : p.translation));
    }
    return Json.object(
        "schema",
        "sgm-orientations-1",
        "pixel_origin",
        "upper-left pixel centre is (0,0)",
        "axes",
        "x right, y down, z forward; world-to-camera R,t",
        "crs",
        model.crs,
        "world_units",
        model.units,
        "notes",
        model.notes,
        "views",
        views);
  }

  static Model fromJson(Object object) {
    Map<String, Object> root = JsonRead.object(object);
    Model m = new Model();
    m.crs = (String) root.get("crs");
    m.units = Objects.toString(root.get("world_units"), "project_units");
    if (root.get("notes") != null)
      for (Object note : JsonRead.array(root.get("notes"))) m.notes.add(Objects.toString(note));
    for (Object value : JsonRead.array(root.get("views"))) {
      Map<String, Object> v = JsonRead.object(value);
      Calibration c = null;
      Camera p = null;
      if (v.get("interior") != null) {
        Map<String, Object> i = JsonRead.object(v.get("interior"));
        c =
            calibration(
                num(i, "fx"),
                num(i, "fy"),
                num(i, "cx"),
                num(i, "cy"),
                JsonRead.numbers(i.get("distortion")),
                (int) num(i, "width"),
                (int) num(i, "height"));
      }
      if (v.get("R") != null) p = pose(JsonRead.numbers(v.get("R")), JsonRead.numbers(v.get("t")));
      m.add(new View(v.get("image").toString(), c, p));
    }
    return m;
  }

  static double num(Map<String, Object> m, String key) {
    return JsonRead.number(m.get(key));
  }

  static void archive(List<ImageFrame> frames, Options o, String crs) throws IOException {
    Model source = fromFrames(frames, "source", crs),
        processed = fromFrames(frames, "processed", crs);
    source.notes.addAll(o.orientationNotes);
    processed.notes.addAll(o.orientationNotes);
    if (crs == null)
      source.units =
          processed.units =
              o.orientationUnits.equals("auto") ? o.orientationSourceUnits : o.orientationUnits;
    Json.write(
        o.output.resolve("04_geometry/orientation_model.json"),
        Json.object("source", json(source), "processed", json(processed)));
    if (!o.orientationOutputFormat.equals("none")
        || !o.interiorOutputFormat.equals("none")
        || !o.exteriorOutputFormat.equals("none"))
      export(o.orientationPixels.equals("source") ? source : processed, o);
  }

  static Model readRun(Path run, Options o) throws IOException {
    Path archive = run.resolve("04_geometry/orientation_model.json");
    if (Files.isRegularFile(archive))
      return fromJson(JsonRead.object(JsonRead.read(archive)).get(o.orientationPixels));
    // Older projects contain enough information for source or processed cameras without original
    // photos.
    Path intrinsics = run.resolve("01_calibration/reference_camera.properties"),
        images = run.resolve("00_inputs/images.csv"),
        poses = run.resolve("04_geometry/cameras.csv");
    if (!Files.exists(poses)) poses = run.resolve("cameras.csv");
    if (!Files.exists(intrinsics)) intrinsics = run.resolve("camera.txt");
    if (!Files.isRegularFile(intrinsics)
        || !Files.isRegularFile(images)
        || !Files.isRegularFile(poses))
      throw new IllegalArgumentException(
          "Run lacks orientation archive/reference calibration/image inventory/poses");
    Model calibration = OrientationText.read(intrinsics, "native", Map.of(), o),
        pose = OrientationText.read(poses, "csv", Map.of(), o);
    Calibration reference = calibration.views.get("*").calibration;
    Map<String, int[]> dimensions = new LinkedHashMap<>();
    List<String> rows = Files.readAllLines(images);
    for (int i = 1; i < rows.size(); i++) {
      List<String> v = PoseIO.csv(rows.get(i));
      dimensions.put(v.get(1), new int[] {Integer.parseInt(v.get(5)), Integer.parseInt(v.get(6))});
    }
    Map<String, Calibration> processed = new HashMap<>();
    Path file = run.resolve("01_calibration/dense_interior.csv");
    if (!Files.exists(file)) file = run.resolve("01_calibration/processed_interior.csv");
    if (Files.exists(file)) {
      Model m = OrientationText.read(file, "csv", dimensions, o);
      for (View v : m.views.values()) processed.put(v.image, v.calibration);
    }
    Model model = new Model();
    for (View v : pose.views.values()) {
      int[] size = dimensions.get(v.image);
      if (size == null)
        throw new IllegalArgumentException("Missing image inventory entry: " + v.image);
      Camera camera = reference.camera(size[0], size[1]);
      Calibration c =
          new Calibration(
              camera.fx, camera.fy, camera.cx, camera.cy, reference.distortion, size[0], size[1]);
      if (o.orientationPixels.equals("processed")) {
        c = processed.get(v.image);
        if (c == null)
          throw new IllegalArgumentException("Missing processed interior for " + v.image);
      }
      model.add(new View(v.image, c, v.pose));
    }
    Path coordinates = run.resolve("coordinates.json");
    if (Files.exists(coordinates)) {
      Object crs = JsonRead.object(JsonRead.read(coordinates)).get("horizontal_crs");
      if (crs != null) {
        model.crs = crs.toString();
        model.units = "metres";
      }
    }
    model.notes.add("Recovered from legacy run; no reconstruction rerun");
    return model;
  }

  static void exportRun(Options o) throws IOException {
    Model m = readRun(o.exportOrientations, o);
    if (o.orientationCrs != null) {
      new Crs(o.orientationCrs);
      m.crs = o.orientationCrs;
      m.units = "metres";
      m.notes.add("CRS explicitly declared at export; coordinates were not transformed");
    }
    if (o.orientationOutput == null)
      o.orientationOutput = o.exportOrientations.resolve("orientation_exports");
    if (o.orientationOutputFormat.equals("none")
        && o.interiorOutputFormat.equals("none")
        && o.exteriorOutputFormat.equals("none")) o.orientationOutputFormat = "csv";
    export(m, o);
  }

  static void export(Model model, Options o) throws IOException {
    if (!o.orientationUnits.equals("auto")) model.units = o.orientationUnits;
    if (model.crs != null && !Set.of("metres", "m").contains(model.units))
      throw new IllegalArgumentException(
          "CRS-tagged orientation export requires metre coordinates; unit declarations do not"
              + " rescale");
    Path directory =
        o.orientationOutput == null ? o.output.resolve("08_orientations") : o.orientationOutput;
    Files.createDirectories(directory);
    Set<String> formats = new LinkedHashSet<>();
    for (String s :
        new String[] {o.orientationOutputFormat, o.interiorOutputFormat, o.exteriorOutputFormat})
      if (!s.equals("none")) formats.addAll(Arrays.asList(s.split(",")));
    for (String format : formats)
      if (Files.exists(directory.resolve(format)))
        throw new IllegalArgumentException(
            "Orientation export already exists; choose a new --orientation-output directory: "
                + directory.resolve(format));
    Path staging = Files.createTempDirectory(directory, ".orientation-export-");
    List<Object> outputs = new ArrayList<>();
    try {
      for (String format : formats) {
        Path target = staging.resolve(format);
        Files.createDirectories(target);
        switch (format) {
          case "native" -> Json.write(target.resolve("orientations.json"), json(model));
          case "csv", "csv-opk", "bundler", "colmap", "inpho" ->
              OrientationText.write(model, target, format, o);
          case "openmvg", "opensfm" -> OrientationJson.write(model, target, format, o);
          case "metashape", "micmac", "opencv" -> OrientationXml.write(model, target, format, o);
          default -> throw new IllegalArgumentException("Unsupported export format: " + format);
        }
        Json.write(
            target.resolve("conventions.json"),
            Json.object(
                "format",
                format,
                "source_pixels",
                o.orientationPixels,
                "crs",
                model.crs,
                "world_units",
                model.units,
                "notes",
                model.notes,
                "lossy_conversions_allowed",
                o.orientationAllowLossy));
        outputs.add(
            Json.object(
                "format", format, "path", directory.resolve(format).toAbsolutePath().toString()));
      }
      for (String format : formats) Files.move(staging.resolve(format), directory.resolve(format));
      Json.write(
          directory.resolve("export_manifest.json"),
          Json.object("views", model.views.size(), "outputs", outputs));
    } finally {
      try (var files = Files.walk(staging)) {
        for (Path path : files.sorted(Comparator.reverseOrder()).toList())
          Files.deleteIfExists(path);
      }
    }
    System.out.println("Orientation export: " + directory.toAbsolutePath());
  }

  static void loss(Model model, Options o, String message) {
    if (!o.orientationAllowLossy)
      throw new IllegalArgumentException(
          message + "; use --orientation-allow-lossy to accept this conversion");
    model.notes.add("LOSSY: " + message);
  }
}
