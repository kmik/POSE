package sgm;

import static sgm.OrientationIO.*;

import java.io.*;
import java.nio.file.*;
import java.util.*;

/** OpenMVG sfm_data JSON and OpenSfM perspective/Brown reconstruction JSON. */
final class OrientationJson {
  static Model read(Path path, String format, Map<String, int[]> sizes, Options o)
      throws IOException {
    if (Files.isDirectory(path))
      path = path.resolve(format.equals("openmvg") ? "sfm_data.json" : "reconstruction.json");
    Object root = JsonRead.read(path);
    return format.equals("openmvg") ? openmvg(JsonRead.object(root), o) : opensfm(root, o);
  }

  private static double get(Map<String, Object> m, String name, double fallback) {
    return m.containsKey(name) ? JsonRead.number(m.get(name)) : fallback;
  }

  private static Model opensfm(Object value, Options o) {
    Map<String, Object> root;
    if (value instanceof List) {
      List<Object> components = JsonRead.array(value);
      if (o.orientationComponent >= components.size())
        throw new IllegalArgumentException("OpenSfM component index out of range");
      root = JsonRead.object(components.get(o.orientationComponent));
    } else root = JsonRead.object(value);
    Model model = new Model();
    Map<String, Calibration> cameras = new HashMap<>();
    for (var item : JsonRead.object(root.get("cameras")).entrySet()) {
      Map<String, Object> c = JsonRead.object(item.getValue());
      int w = (int) num(c, "width"), h = (int) num(c, "height"), scale = Math.max(w, h);
      String projection = Objects.toString(c.get("projection_type"));
      if (!projection.equals("perspective") && !projection.equals("brown"))
        throw new IllegalArgumentException("Unsupported OpenSfM projection: " + projection);
      double fx = get(c, "focal_x", get(c, "focal", Double.NaN)) * scale,
          fy = get(c, "focal_y", get(c, "focal", Double.NaN)) * scale;
      double cx = (w - 1) * .5 + get(c, "c_x", 0) * scale,
          cy = (h - 1) * .5 + get(c, "c_y", 0) * scale;
      cameras.put(
          item.getKey(),
          calibration(
              fx,
              fy,
              cx,
              cy,
              new double[] {
                get(c, "k1", 0), get(c, "k2", 0), get(c, "p1", 0), get(c, "p2", 0), get(c, "k3", 0)
              },
              w,
              h));
    }
    for (var item : JsonRead.object(root.get("shots")).entrySet()) {
      Map<String, Object> shot = JsonRead.object(item.getValue());
      Calibration c =
          Objects.requireNonNull(
              cameras.get(shot.get("camera").toString()), "Unknown OpenSfM camera");
      model.add(
          new View(
              item.getKey(),
              c,
              pose(
                  Math3.rotation(JsonRead.numbers(shot.get("rotation"))),
                  JsonRead.numbers(shot.get("translation")))));
    }
    model.notes.add(
        "OpenSfM component "
            + o.orientationComponent
            + "; normalized focal/principal point scaled by max(width,height); reference_lla does"
            + " not automatically assign an EPSG CRS");
    return model;
  }

  private static Map<String, Object> data(Object value) {
    Map<String, Object> v = JsonRead.object(value);
    if (v.containsKey("ptr_wrapper"))
      v = JsonRead.object(JsonRead.object(v.get("ptr_wrapper")).get("data"));
    return v;
  }

  private static Model openmvg(Map<String, Object> root, Options o) {
    Model model = new Model();
    Map<Integer, Calibration> cameras = new HashMap<>();
    Map<Integer, Camera> poses = new HashMap<>();
    for (Object item : JsonRead.array(root.get("intrinsics"))) {
      Map<String, Object> entry = JsonRead.object(item),
          wrapper = JsonRead.object(entry.get("value")),
          c = data(wrapper);
      String type = Objects.toString(wrapper.get("polymorphic_name"), "pinhole");
      if (!type.contains("pinhole"))
        throw new IllegalArgumentException("Unsupported OpenMVG intrinsic model: " + type);
      double[] principal = JsonRead.numbers(c.get("principal_point")), distortion = new double[5];
      for (String key : List.of("disto_k1", "disto_k3"))
        if (c.containsKey(key)) {
          double[] d = JsonRead.numbers(c.get(key));
          distortion[0] = d[0];
          if (d.length > 1) distortion[1] = d[1];
          if (d.length > 2) distortion[4] = d[2];
        }
      if (c.containsKey("disto_t2")) {
        double[] d = JsonRead.numbers(c.get("disto_t2"));
        if (d.length != 5)
          throw new IllegalArgumentException("OpenMVG Brown disto_t2 must contain k1,k2,k3,t1,t2");
        distortion = new double[] {d[0], d[1], d[3], d[4], d[2]};
      }
      double focal = num(c, "focal_length");
      cameras.put(
          (int) num(entry, "key"),
          calibration(
              focal,
              focal,
              principal[0],
              principal[1],
              distortion,
              (int) num(c, "width"),
              (int) num(c, "height")));
    }
    for (Object item : JsonRead.array(root.get("extrinsics"))) {
      Map<String, Object> entry = JsonRead.object(item), p = JsonRead.object(entry.get("value"));
      double[] r = flatten(p.get("rotation"));
      poses.put((int) num(entry, "key"), centered(r, JsonRead.numbers(p.get("center"))));
    }
    for (Object item : JsonRead.array(root.get("views"))) {
      Map<String, Object> entry = JsonRead.object(item), v = data(entry.get("value"));
      int id = (int) get(v, "id_pose", num(entry, "key"));
      Camera pose = poses.get(id);
      if (pose == null) continue;
      String name = Objects.toString(v.get("filename"));
      model.add(
          new View(
              name,
              Objects.requireNonNull(
                  cameras.get((int) num(v, "id_intrinsic")), "Unknown OpenMVG intrinsic"),
              pose));
    }
    return model;
  }

  static double[] flatten(Object value) {
    List<Object> a = JsonRead.array(value);
    if (a.isEmpty()) return new double[0];
    if (a.getFirst() instanceof List) {
      List<Double> out = new ArrayList<>();
      for (Object row : a) for (Object n : JsonRead.array(row)) out.add(JsonRead.number(n));
      double[] v = new double[out.size()];
      for (int i = 0; i < v.length; i++) v[i] = out.get(i);
      return v;
    }
    return JsonRead.numbers(value);
  }

  private static Object rows(double[] r) {
    return new double[][] {
      Arrays.copyOfRange(r, 0, 3), Arrays.copyOfRange(r, 3, 6), Arrays.copyOfRange(r, 6, 9)
    };
  }

  static void write(Model m, Path dir, String format, Options o) throws IOException {
    if (format.equals("opensfm")) {
      Map<String, Object> cameras = new LinkedHashMap<>(), shots = new LinkedHashMap<>();
      int index = 0;
      for (View v : m.views.values()) {
        Calibration c = Objects.requireNonNull(v.calibration());
        String id = "camera_" + (index++);
        double scale = Math.max(c.width, c.height);
        cameras.put(
            id,
            Json.object(
                "projection_type",
                "brown",
                "width",
                c.width,
                "height",
                c.height,
                "focal_x",
                c.fx / scale,
                "focal_y",
                c.fy / scale,
                "c_x",
                (c.cx - (c.width - 1) * .5) / scale,
                "c_y",
                (c.cy - (c.height - 1) * .5) / scale,
                "k1",
                c.distortion[0],
                "k2",
                c.distortion[1],
                "p1",
                c.distortion[2],
                "p2",
                c.distortion[3],
                "k3",
                c.distortion[4]));
        if (v.pose() != null)
          shots.put(
              v.image(),
              Json.object(
                  "camera",
                  id,
                  "rotation",
                  Math3.rotationVector(v.pose().rotation),
                  "translation",
                  v.pose().translation,
                  "orientation",
                  1));
      }
      Json.write(
          dir.resolve("reconstruction.json"),
          List.of(Json.object("cameras", cameras, "shots", shots, "points", Map.of())));
    } else {
      List<Object> views = new ArrayList<>(),
          cameras = new ArrayList<>(),
          poses = new ArrayList<>();
      int id = 0;
      for (View v : m.views.values()) {
        Calibration c = Objects.requireNonNull(v.calibration());
        if (c.fx != c.fy) loss(m, o, "OpenMVG pinhole export has one focal; fx/fy averaged");
        Object data =
            Json.object(
                "width",
                c.width,
                "height",
                c.height,
                "focal_length",
                (c.fx + c.fy) * .5,
                "principal_point",
                new double[] {c.cx, c.cy},
                "disto_t2",
                new double[] {
                  c.distortion[0],
                  c.distortion[1],
                  c.distortion[4],
                  c.distortion[2],
                  c.distortion[3]
                });
        cameras.add(
            Json.object(
                "key",
                id,
                "value",
                Json.object(
                    "polymorphic_id",
                    2147483652L,
                    "polymorphic_name",
                    "pinhole_brown_t2",
                    "ptr_wrapper",
                    Json.object("id", 2L * id + 2147483650L, "data", data))));
        views.add(
            Json.object(
                "key",
                id,
                "value",
                Json.object(
                    "polymorphic_id",
                    1073741824,
                    "ptr_wrapper",
                    Json.object(
                        "id",
                        2L * id + 2147483649L,
                        "data",
                        Json.object(
                            "local_path",
                            "",
                            "filename",
                            v.image(),
                            "width",
                            c.width,
                            "height",
                            c.height,
                            "id_view",
                            id,
                            "id_intrinsic",
                            id,
                            "id_pose",
                            id)))));
        if (v.pose() != null)
          poses.add(
              Json.object(
                  "key",
                  id,
                  "value",
                  Json.object("rotation", rows(v.pose().rotation), "center", v.pose().center())));
        id++;
      }
      Json.write(
          dir.resolve("sfm_data.json"),
          Json.object(
              "sfm_data_version",
              "0.3",
              "root_path",
              "",
              "views",
              views,
              "intrinsics",
              cameras,
              "extrinsics",
              poses,
              "structure",
              List.of(),
              "control_points",
              List.of()));
    }
  }
}
