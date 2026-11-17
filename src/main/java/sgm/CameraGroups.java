package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;

/**
 * Camera groups: the images that share one interior orientation (one camera body, lens and image
 * size). Each group has its own calibration in source pixels and its own intrinsics policy.
 *
 * <p>Grouping, first rule that applies wins (--camera-grouping auto):
 *
 * <ol>
 *   <li>camera IDs of an imported orientation (INPHO $CAMERA_ID, Metashape sensor_id, COLMAP
 *       CAMERA_ID, openMVG id_intrinsic, OpenSfM camera key). A file that gives every photo its own
 *       camera ID (the one-camera-per-photo layout many exporters write) carries no grouping and
 *       falls through to the next rule;
 *   <li>--camera-groups FILE (CSV image,camera);
 *   <li>EXIF: make, model, body serial when present, focal length in mm rounded to 0.1.
 * </ol>
 *
 * Images of different pixel sizes are always in separate groups: one calibration never serves two
 * image sizes. --camera-grouping single puts all images of one size into one group;
 * --camera-grouping per-image gives every image its own group. Group 0 is the group of the first
 * image; its calibration is the run's reference camera (camera.txt).
 */
final class CameraGroups {
  static final List<String> POLICIES = List.of("fixed", "prior", "initial", "free");
  private static final int MAGIC = 0x43414d47; // "CAMG", trailer of state/calibration.bin

  static final class Group {
    final String id;
    final int width, height;
    String label = "", make = "", model = "", serial = "", source = "";
    double focalMm = Double.NaN, pixelSizeMm = Double.NaN;
    /** Calibration in this camera's source pixels; null until the calibrate step. */
    Calibration calibration;
    /** True when the calibration came from a file or an imported orientation. */
    boolean supplied;
    /** How the initial calibration was obtained (supplied, focal, exif, bootstrap ...). */
    String initialization = "";
    final List<String> images = new ArrayList<>();

    Group(String id, int width, int height) {
      this.id = id;
      this.width = width;
      this.height = height;
    }

    String size() {
      return width + "x" + height;
    }
  }

  final List<Group> groups;
  private final Map<String, Integer> byImage = new HashMap<>();

  CameraGroups(List<Group> groups) {
    this.groups = List.copyOf(groups);
    for (int g = 0; g < groups.size(); g++)
      for (String image : groups.get(g).images) byImage.put(image, g);
  }

  int size() {
    return groups.size();
  }

  boolean multiple() {
    return groups.size() > 1;
  }

  Group get(int g) {
    return groups.get(g);
  }

  /** Group index of an image (by file name); 0 for an image that is not in the inventory. */
  int index(String image) {
    return byImage.getOrDefault(OrientationIO.name(image), 0);
  }

  int index(Path image) {
    return index(image.getFileName().toString());
  }

  Group of(String image) {
    return groups.get(index(image));
  }

  int indexOf(String id) {
    for (int g = 0; g < groups.size(); g++) if (groups.get(g).id.equals(id)) return g;
    return -1;
  }

  /** Intrinsics policy of a group: --intrinsics-mode CAMERA=mode, else the global mode. */
  static String policy(Group g, Options o) {
    return o.intrinsicsModes.getOrDefault(g.id, o.intrinsicsMode);
  }

  /** Lens parameters of the group may be refined (not a supplied calibration held fixed). */
  static boolean refinable(Group g, Options o) {
    return !(g.supplied && policy(g, o).equals("fixed"));
  }

  /** The groups of a run made before camera groups existed: every image in one group. */
  static CameraGroups legacy(List<String> images, Calibration reference) {
    Group g =
        new Group(
            "camera_1",
            reference == null ? 0 : reference.width,
            reference == null ? 0 : reference.height);
    g.images.addAll(images);
    g.calibration = reference;
    g.source = "legacy_run";
    g.label = "camera_1";
    return new CameraGroups(List.of(g));
  }

  // ---- Grouping.

  /**
   * Groups the input images. {@code imported}: camera ID per image name from an imported
   * orientation (empty when none). {@code importedPixel}: pixel size in mm per camera ID, when the
   * import states one.
   */
  static CameraGroups resolve(
      Options o, List<Path> paths, Map<String, String> imported, Map<String, Double> importedPixel)
      throws IOException {
    String mode = o.cameraGrouping;
    if (o.cameraGroupsFile != null && !mode.equals("auto"))
      throw new IllegalArgumentException(
          "--camera-groups FILE applies with --camera-grouping auto, not " + mode);
    Map<String, String> csv = o.cameraGroupsFile == null ? Map.of() : readCsv(o.cameraGroupsFile);
    boolean importRule = false;
    if (mode.equals("auto") && !imported.isEmpty()) {
      Set<String> distinct = new HashSet<>(imported.values());
      // One camera per photo (how many tools export) says nothing about which photos share a lens.
      importRule = !(paths.size() > 1 && distinct.size() == paths.size());
      if (importRule) {
        List<String> missing = new ArrayList<>();
        for (Path p : paths)
          if (!imported.containsKey(p.getFileName().toString()))
            missing.add(p.getFileName().toString());
        if (!missing.isEmpty())
          throw new IllegalArgumentException(
              "The imported orientation names a camera for some photos but not for "
                  + summary(missing)
                  + "; give every photo a camera, or group them with --camera-groups FILE");
      }
    }
    String rule =
        mode.equals("single")
            ? "single"
            : mode.equals("per-image")
                ? "per-image"
                : importRule ? "import" : !csv.isEmpty() ? "file" : "exif";
    if (rule.equals("file")) {
      List<String> missing = new ArrayList<>();
      for (Path p : paths) if (!csv.containsKey(p.getFileName().toString())) missing.add(p.getFileName().toString());
      if (!missing.isEmpty())
        throw new IllegalArgumentException(
            "--camera-groups " + o.cameraGroupsFile + " does not list " + summary(missing));
    }
    Map<String, Group> byKey = new LinkedHashMap<>();
    Map<String, Integer> sizesPerName = new HashMap<>();
    Set<String> usedIds = new HashSet<>();
    int exifCount = 0;
    for (Path p : paths) {
      String image = p.getFileName().toString();
      int[] size = ResolutionPlan.originalSize(p);
      Exif.Metadata m = Exif.read(p);
      String name =
          switch (rule) {
            case "import" -> imported.get(image);
            case "file" -> csv.get(image);
            case "per-image" -> stem(image);
            case "single" -> "camera";
            default ->
                m.make.trim()
                    + "|"
                    + m.model.trim()
                    + "|"
                    + m.serial.trim()
                    + "|"
                    + (Double.isFinite(m.focalMm)
                        ? String.format(Locale.ROOT, "%.1f", m.focalMm)
                        : "");
          };
      String key = name + "|" + size[0] + "x" + size[1];
      Group g = byKey.get(key);
      if (g == null) {
        int seen = sizesPerName.merge(name, 1, Integer::sum);
        String id =
            switch (rule) {
              case "import", "file", "per-image" ->
                  seen == 1 ? name : name + "_" + size[0] + "x" + size[1];
              default -> "camera_" + (++exifCount);
            };
        while (!usedIds.add(id)) id = id + "_";
        g = new Group(id, size[0], size[1]);
        g.make = m.make.trim();
        g.model = m.model.trim();
        g.serial = m.serial.trim();
        g.focalMm = m.focalMm;
        g.source = rule;
        if (rule.equals("import")) {
          Double pixel = importedPixel.get(name);
          if (pixel != null) g.pixelSizeMm = pixel;
        }
        byKey.put(key, g);
      }
      g.images.add(image);
    }
    List<Group> groups = new ArrayList<>(byKey.values());
    for (Group g : groups) {
      Double pixel = o.orientationPixelSizes.get(g.id);
      if (pixel != null) g.pixelSizeMm = pixel;
      else if (!Double.isFinite(g.pixelSizeMm) && Double.isFinite(o.orientationPixelSize))
        g.pixelSizeMm = o.orientationPixelSize;
      g.label = label(g);
    }
    CameraGroups result = new CameraGroups(groups);
    result.checkCameraOptions(o);
    return result;
  }

  /**
   * Imported intrinsics that differ between images of one camera can only be kept as they are:
   * refining needs one calibration per camera. A camera with policy free ignores its imports.
   */
  void checkImported(Options options) {
    if (options.importedCalibrations.isEmpty()) return;
    for (Group g : groups) {
      String policy = policy(g, options);
      Calibration first = null;
      for (String image : g.images) {
        Calibration c = options.importedCalibrations.get(image);
        if (c == null) continue;
        if (first == null) first = c;
        else if (!SharedCalibration.same(first, c) && !policy.equals("fixed") && !policy.equals("free"))
          throw new IllegalArgumentException(
              "Camera "
                  + g.id
                  + ": its images carry different imported calibrations ("
                  + g.images.getFirst()
                  + ", "
                  + image
                  + "); refining needs one calibration per camera. Keep them with --intrinsics-mode "
                  + (multiple() ? g.id + "=fixed" : "fixed")
                  + ", or name the cameras with --camera-groups FILE");
      }
    }
  }

  /** Every CAMERA= key of a per-camera option must name a group. */
  void checkCameraOptions(Options o) {
    List<String> ids = groups.stream().map(g -> g.id).toList();
    Map<String, Set<String>> keys = new LinkedHashMap<>();
    keys.put("--intrinsics-mode", o.intrinsicsModes.keySet());
    keys.put("--calibration", o.calibrationFiles.keySet());
    keys.put("--orientation-pixel-size", o.orientationPixelSizes.keySet());
    keys.put("--focal", o.focalByCamera.keySet());
    keys.put("--s_w", o.sensorWidthByCamera.keySet());
    for (var e : keys.entrySet())
      for (String id : e.getValue())
        if (!ids.contains(id))
          throw new IllegalArgumentException(
              e.getKey() + " " + id + "=...: no camera '" + id + "'; the cameras are "
                  + groups.stream().map(g -> g.id + " (" + g.label + ")").toList());
  }

  private static String label(Group g) {
    StringBuilder s = new StringBuilder();
    if (!g.make.isEmpty()) s.append(g.make);
    if (!g.model.isEmpty()) s.append(s.length() > 0 ? " " : "").append(g.model);
    if (!g.serial.isEmpty()) s.append(" #").append(g.serial);
    if (Double.isFinite(g.focalMm))
      s.append(String.format(Locale.ROOT, "%s%.1f mm", s.length() > 0 ? ", " : "", g.focalMm));
    s.append(s.length() > 0 ? ", " : "").append(g.size()).append(" px");
    return s.toString();
  }

  private static String stem(String image) {
    int dot = image.lastIndexOf('.');
    return dot > 0 ? image.substring(0, dot) : image;
  }

  private static String summary(List<String> images) {
    return images.size() <= 5
        ? String.join(", ", images)
        : String.join(", ", images.subList(0, 5)) + " and " + (images.size() - 5) + " more";
  }

  /** --camera-groups: CSV image,camera (header optional; image may contain a path). */
  static Map<String, String> readCsv(Path file) throws IOException {
    Map<String, String> out = new LinkedHashMap<>();
    int line = 0;
    for (String row : Files.readAllLines(file)) {
      line++;
      row = row.replaceFirst("^﻿", "");
      if (row.isBlank() || row.stripLeading().startsWith("#")) continue;
      List<String> v = PoseIO.csv(row);
      if (v.size() < 2 || v.get(1).isBlank())
        throw new IllegalArgumentException(
            "--camera-groups " + file + " line " + line + ": expected image,camera");
      String image = OrientationIO.name(v.get(0).strip()), camera = v.get(1).strip();
      if (line == 1 && image.equalsIgnoreCase("image") && camera.equalsIgnoreCase("camera"))
        continue;
      String old = out.put(image, camera);
      if (old != null && !old.equals(camera))
        throw new IllegalArgumentException(
            "--camera-groups " + file + ": " + image + " is in two cameras (" + old + ", " + camera + ")");
    }
    if (out.isEmpty()) throw new IllegalArgumentException("--camera-groups " + file + " lists no images");
    return out;
  }

  // ---- Group calibrations from the frames (after an adjustment refitted lenses).

  /** Takes each group's calibration from its first frame. */
  void refresh(List<ImageFrame> frames) {
    boolean[] done = new boolean[groups.size()];
    for (ImageFrame f : frames)
      if (f.group < groups.size() && !done[f.group]) {
        groups.get(f.group).calibration = f.calibration;
        done[f.group] = true;
      }
  }

  // ---- Persistence: a trailer of state/calibration.bin, and 01_calibration/cameras.json.

  void write(DataOutputStream out) throws IOException {
    out.writeInt(MAGIC);
    out.writeInt(1);
    out.writeInt(groups.size());
    for (Group g : groups) {
      out.writeUTF(g.id);
      out.writeInt(g.width);
      out.writeInt(g.height);
      out.writeUTF(g.label);
      out.writeUTF(g.make);
      out.writeUTF(g.model);
      out.writeUTF(g.serial);
      out.writeUTF(g.source);
      out.writeUTF(g.initialization);
      out.writeDouble(g.focalMm);
      out.writeDouble(g.pixelSizeMm);
      out.writeBoolean(g.supplied);
      out.writeBoolean(g.calibration != null);
      if (g.calibration != null) PipelineState.writeCalibration(out, g.calibration);
      out.writeInt(g.images.size());
      for (String image : g.images) out.writeUTF(image);
    }
  }

  /** The groups after the calibration map of calibration.bin, or null for a run without them. */
  static CameraGroups read(DataInputStream in) throws IOException {
    int magic;
    try {
      magic = in.readInt();
    } catch (EOFException e) {
      return null;
    }
    if (magic != MAGIC) throw new IOException("Corrupt camera groups in calibration.bin");
    if (in.readInt() != 1) throw new IOException("Unsupported camera groups version in calibration.bin");
    int n = in.readInt();
    List<Group> groups = new ArrayList<>();
    for (int i = 0; i < n; i++) {
      Group g = new Group(in.readUTF(), in.readInt(), in.readInt());
      g.label = in.readUTF();
      g.make = in.readUTF();
      g.model = in.readUTF();
      g.serial = in.readUTF();
      g.source = in.readUTF();
      g.initialization = in.readUTF();
      g.focalMm = in.readDouble();
      g.pixelSizeMm = in.readDouble();
      g.supplied = in.readBoolean();
      if (in.readBoolean()) g.calibration = PipelineState.readCalibration(in);
      int images = in.readInt();
      for (int k = 0; k < images; k++) g.images.add(in.readUTF());
      groups.add(g);
    }
    return new CameraGroups(groups);
  }

  Object json(Options o) {
    List<Object> list = new ArrayList<>();
    for (Group g : groups) {
      Calibration c = g.calibration;
      list.add(
          Json.object(
              "id", g.id,
              "label", g.label,
              "make", g.make,
              "model", g.model,
              "serial", g.serial,
              "focal_mm_exif", g.focalMm,
              "width", g.width,
              "height", g.height,
              "pixel_size_mm", g.pixelSizeMm,
              "grouping_source", g.source,
              "policy", policy(g, o),
              "supplied_calibration", g.supplied,
              "initialization", g.initialization,
              "images", g.images.size(),
              "calibration",
              c == null
                  ? null
                  : Json.object(
                      "width", c.width, "height", c.height, "fx", c.fx, "fy", c.fy, "cx", c.cx,
                      "cy", c.cy, "distortion", c.distortion),
              "image_names", g.images));
    }
    return Json.object(
        "schema", "pose-camera-groups-1",
        "grouping", o.cameraGrouping,
        "camera_groups_file", o.cameraGroupsFile == null ? null : o.cameraGroupsFile.toString(),
        "note",
        "Calibrations are in each camera's source pixels (pixel centre 0,0; OpenCV k1,k2,p1,p2,k3).",
        "cameras", list);
  }

  /** 01_calibration/cameras.json and, with two or more cameras, camera_ID.txt in the run root. */
  void export(Options o, Path run) throws IOException {
    Json.write(run.resolve("01_calibration/cameras.json"), json(o));
    if (!multiple()) return;
    for (Group g : groups)
      if (g.calibration != null)
        g.calibration.write(
            run.resolve("camera_" + safe(g.id) + ".txt"),
            "Camera " + g.id + " (" + g.label + "); " + g.images.size() + " images; source pixels");
  }

  static String safe(String id) {
    return id.replaceAll("[^A-Za-z0-9._-]", "_");
  }
}
