package sgm;

import static sgm.OrientationIO.*;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import java.util.regex.*;
import javax.xml.XMLConstants;
import javax.xml.parsers.*;
import org.w3c.dom.*;

/** Secure XML adapters; unsupported distortion, affine and projection conventions are explicit. */
final class OrientationXml {
  static Document document(Path file) throws IOException {
    try {
      DocumentBuilderFactory f = DocumentBuilderFactory.newInstance();
      f.setFeature("http://apache.org/xml/features/disallow-doctype-decl", true);
      f.setFeature("http://xml.org/sax/features/external-general-entities", false);
      f.setFeature("http://xml.org/sax/features/external-parameter-entities", false);
      f.setXIncludeAware(false);
      f.setExpandEntityReferences(false);
      f.setAttribute(XMLConstants.ACCESS_EXTERNAL_DTD, "");
      f.setAttribute(XMLConstants.ACCESS_EXTERNAL_SCHEMA, "");
      return f.newDocumentBuilder().parse(file.toFile());
    } catch (Exception e) {
      throw new IOException("Cannot parse orientation XML " + file + ": " + e.getMessage(), e);
    }
  }

  private static List<Element> children(Element root, String name) {
    List<Element> out = new ArrayList<>();
    if (root == null) return out;
    for (Node n = root.getFirstChild(); n != null; n = n.getNextSibling())
      if (n instanceof Element e && e.getTagName().equals(name)) out.add(e);
    return out;
  }

  private static Element child(Element root, String name) {
    if (root == null) return null;
    List<Element> list = children(root, name);
    return list.isEmpty() ? null : list.getFirst();
  }

  private static Element descendant(Element root, String name) {
    if (root == null) return null;
    NodeList list = root.getElementsByTagName(name);
    return list.getLength() == 0 ? null : (Element) list.item(0);
  }

  private static String text(Element root, String name, String fallback) {
    Element e = child(root, name);
    return e == null ? fallback : e.getTextContent().strip();
  }

  private static double number(Element root, String name, double fallback) {
    return Double.parseDouble(text(root, name, Double.toString(fallback)));
  }

  private static double[] vector(Element root, String name) {
    Element e = child(root, name);
    if (e == null) throw new IllegalArgumentException("Missing XML element " + name);
    Element data = child(e, "data");
    return numbers(data == null ? e.getTextContent() : data.getTextContent());
  }

  private static String attr(Element e, String name, String fallback) {
    return e.hasAttribute(name) ? e.getAttribute(name) : fallback;
  }

  private static String xml(String text) {
    return text.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace("\"", "&quot;");
  }

  private static String join(double[] v) {
    StringJoiner s = new StringJoiner(" ");
    for (double d : v) s.add(Double.toString(d));
    return s.toString();
  }

  private static String matrix4(Camera c) {
    double[] r = Math3.transpose(c.rotation), p = c.center();
    return r[0]
        + " "
        + r[1]
        + " "
        + r[2]
        + " "
        + p[0]
        + " "
        + r[3]
        + " "
        + r[4]
        + " "
        + r[5]
        + " "
        + p[1]
        + " "
        + r[6]
        + " "
        + r[7]
        + " "
        + r[8]
        + " "
        + p[2]
        + " 0 0 0 1";
  }

  static Model read(Path input, String format, Map<String, int[]> sizes, Options o)
      throws IOException {
    return switch (format) {
      case "opencv" -> opencv(Files.isDirectory(input) ? input.resolve("cameras.xml") : input, o);
      case "metashape" -> metashape(input, o);
      case "micmac" -> micmac(input, o);
      default -> throw new IllegalArgumentException("Unknown XML format");
    };
  }

  private static Calibration metashapeCalibration(
      Element c, Element sensor, Model model, Options o) {
    if (c == null) throw new IllegalArgumentException("Metashape sensor lacks calibration");
    String type = attr(c, "type", "frame");
    if (!type.equals("frame"))
      throw new IllegalArgumentException("Only Metashape frame cameras are supported");
    Element resolution = child(sensor, "resolution");
    int
        w =
            (int)
                number(
                    c,
                    "width",
                    resolution == null ? 0 : Integer.parseInt(resolution.getAttribute("width"))),
        h =
            (int)
                number(
                    c,
                    "height",
                    resolution == null ? 0 : Integer.parseInt(resolution.getAttribute("height")));
    if (number(c, "b2", 0) != 0)
      throw new IllegalArgumentException("Nonzero Metashape skew b2 unsupported");
    if (number(c, "k4", 0) != 0 || number(c, "p3", 0) != 0 || number(c, "p4", 0) != 0)
      loss(model, o, "Metashape higher-order k4/p3/p4 distortion dropped");
    double f = number(c, "f", Double.NaN);
    return calibration(
        f + number(c, "b1", 0),
        f,
        w * .5 + number(c, "cx", 0) - .5,
        h * .5 + number(c, "cy", 0) - .5,
        new double[] {
          number(c, "k1", 0),
          number(c, "k2", 0),
          number(c, "p2", 0),
          number(c, "p1", 0),
          number(c, "k3", 0)
        },
        w,
        h);
  }

  private static Model metashape(Path input, Options o) throws IOException {
    Element root = document(input).getDocumentElement();
    Model model = new Model();
    if (root.getTagName().equals("calibration")) {
      model.add(new View("*", metashapeCalibration(root, root, model, o), null));
      return model;
    }
    NodeList chunks = root.getElementsByTagName("chunk");
    if (o.orientationComponent >= chunks.getLength())
      throw new IllegalArgumentException("Metashape chunk index out of range");
    Element chunk = (Element) chunks.item(o.orientationComponent);
    Map<String, Calibration> sensors = new HashMap<>();
    Element ss = child(chunk, "sensors");
    for (Element sensor : children(ss, "sensor")) {
      List<Element> calibrations = children(sensor, "calibration");
      Element selected = null;
      for (Element c : calibrations)
        if (selected == null || attr(c, "class", "").equals("adjusted")) selected = c;
      sensors.put(sensor.getAttribute("id"), metashapeCalibration(selected, sensor, model, o));
    }
    double[] Q = Math3.identity(), T = {0, 0, 0};
    double scale = 1;
    Element transform = child(chunk, "transform");
    if (transform != null) {
      if (child(transform, "rotation") != null) Q = vector(transform, "rotation");
      if (child(transform, "translation") != null) T = vector(transform, "translation");
      scale = number(transform, "scale", 1);
      if (!(scale > 0)) throw new IllegalArgumentException("Invalid Metashape chunk scale");
    }
    Element cameraRoot = child(chunk, "cameras");
    if (cameraRoot != null) {
      NodeList cameras = cameraRoot.getElementsByTagName("camera");
      for (int i = 0; i < cameras.getLength(); i++) {
        Element c = (Element) cameras.item(i), tr = child(c, "transform");
        if (tr == null) continue;
        double[] a = numbers(tr.getTextContent());
        if (a.length != 16)
          throw new IllegalArgumentException("Metashape camera transform must be 4x4");
        double[] r = {a[0], a[1], a[2], a[4], a[5], a[6], a[8], a[9], a[10]},
            center = {a[3], a[7], a[11]};
        center = Math3.add(Math3.times(Math3.mv(Q, center), scale), T);
        r = Math3.transpose(Math3.mul(Q, r));
        String image = attr(c, "label", "");
        Element photo = child(c, "photo");
        if (photo != null) image = attr(photo, "path", image);
        model.add(
            new View(
                image,
                Objects.requireNonNull(
                    sensors.get(c.getAttribute("sensor_id")), "Unknown Metashape sensor"),
                centered(r, center)));
      }
    }
    Element crs = child(chunk, "reference");
    if (crs != null)
      model.notes.add(
          "Metashape reference CRS is not assigned to transformed camera XYZ: chunk coordinates may"
              + " be geocentric even with a projected reference CRS. Transform externally or"
              + " explicitly declare a verified --orientation-crs. Reference: "
              + crs.getTextContent().strip());
    model.notes.add(
        "Metashape: camera-to-world transforms plus chunk similarity; pixel-corner principal point"
            + " shifted -0.5; tangential p1/p2 swapped to OpenCV order");
    return model;
  }

  private static Model opencv(Path file, Options o) throws IOException {
    String raw = Files.readString(file);
    Model model = new Model();
    if (raw.stripLeading().startsWith("%YAML") || !raw.stripLeading().startsWith("<")) {
      double[] K = yamlMatrix(raw, "camera_matrix|K"),
          D = yamlMatrixOptional(raw, "distortion_coefficients|distortion_coeffs|distCoeffs|D");
      int w = (int) yamlScalar(raw, "image_width|width"),
          h = (int) yamlScalar(raw, "image_height|height");
      if (K.length != 9 || K[1] != 0 || K[3] != 0 || K[6] != 0 || K[7] != 0 || K[8] != 1)
        throw new IllegalArgumentException("OpenCV K must be a zero-skew 3x3 matrix");
      double[] R = yamlMatrixOptional(raw, "rotation_matrix|R"),
          t = yamlMatrixOptional(raw, "translation_vector|T|tvec"),
          rv = yamlMatrixOptional(raw, "rvec");
      if (R.length == 0 && rv.length > 0) R = Math3.rotation(rv);
      Camera pose = R.length == 0 ? null : pose(R, t);
      model.add(new View("*", calibration(K[0], K[4], K[2], K[5], D, w, h), pose));
      return model;
    }
    Element root = document(file).getDocumentElement(), cameras = child(root, "cameras");
    List<Element> list = cameras == null ? List.of(root) : children(cameras, "_");
    for (Element c : list) {
      String key = child(c, "camera_matrix") != null ? "camera_matrix" : "K";
      double[] K = vector(c, key),
          D =
              child(c, "distortion_coefficients") != null
                  ? vector(c, "distortion_coefficients")
                  : child(c, "distCoeffs") != null
                      ? vector(c, "distCoeffs")
                      : child(c, "D") != null ? vector(c, "D") : new double[5];
      if (K.length != 9 || K[1] != 0 || K[3] != 0 || K[6] != 0 || K[7] != 0 || K[8] != 1)
        throw new IllegalArgumentException("Unsupported OpenCV intrinsic matrix");
      int w = (int) number(c, "image_width", number(c, "width", 0)),
          h = (int) number(c, "image_height", number(c, "height", 0));
      Camera pose = null;
      if (child(c, "rotation_matrix") != null)
        pose = pose(vector(c, "rotation_matrix"), vector(c, "translation_vector"));
      else if (child(c, "rvec") != null)
        pose = pose(Math3.rotation(vector(c, "rvec")), vector(c, "tvec"));
      model.add(
          new View(text(c, "image", "*"), calibration(K[0], K[4], K[2], K[5], D, w, h), pose));
    }
    return model;
  }

  private static double yamlScalar(String text, String key) {
    Matcher m = Pattern.compile("(?m)^\\s*(?:" + key + ")\\s*:\\s*([+\\-0-9.eE]+)").matcher(text);
    if (!m.find()) throw new IllegalArgumentException("OpenCV YAML lacks " + key);
    return Double.parseDouble(m.group(1));
  }

  private static double[] yamlMatrix(String text, String key) {
    double[] v = yamlMatrixOptional(text, key);
    if (v.length == 0) throw new IllegalArgumentException("OpenCV YAML lacks " + key);
    return v;
  }

  private static double[] yamlMatrixOptional(String text, String key) {
    Matcher m =
        Pattern.compile("(?ms)^\\s*(?:" + key + ")\\s*:[^\\[]*?data\\s*:\\s*\\[([^]]+)\\]")
            .matcher(text);
    return m.find() ? numbers(m.group(1)) : new double[0];
  }

  private static Model micmac(Path input, Options o) throws IOException {
    Model model = new Model();
    List<Path> files;
    if (Files.isDirectory(input)) {
      try (var stream = Files.list(input)) {
        files =
            stream
                .filter(
                    p ->
                        p.getFileName().toString().startsWith("Orientation-")
                            && p.toString().endsWith(".xml"))
                .sorted()
                .toList();
      }
    } else files = List.of(input);
    for (Path file : files) {
      Element root = document(file).getDocumentElement();
      Element orientation =
          root.getTagName().equals("OrientationConique")
              ? root
              : descendant(root, "OrientationConique");
      if (orientation == null) {
        Element calibration =
            root.getTagName().equals("CalibrationInternConique")
                ? root
                : descendant(root, "CalibrationInternConique");
        if (calibration != null)
          model.add(new View("*", micmacCalibration(calibration, model, o), null));
        continue;
      }
      if (!text(orientation, "TypeProj", "eProjStenope").equals("eProjStenope"))
        throw new IllegalArgumentException("MicMac non-pinhole projection unsupported");
      Element internal = child(orientation, "OrIntImaM2C");
      if (internal != null
          && (!Arrays.equals(vector(internal, "I00"), new double[] {0, 0})
              || !Arrays.equals(vector(internal, "V10"), new double[] {1, 0})
              || !Arrays.equals(vector(internal, "V01"), new double[] {0, 1})))
        throw new IllegalArgumentException(
            "MicMac internal crop/scan affine must be applied before import");
      String linked = text(orientation, "FileInterne", "");
      Element calibration = child(orientation, "Interne");
      if (!linked.isBlank()) {
        Path resolved = Path.of(linked.replace('\\', '/'));
        if (!resolved.isAbsolute())
          resolved = file.toAbsolutePath().getParent().resolve(resolved).normalize();
        Element d = document(resolved).getDocumentElement();
        calibration =
            d.getTagName().equals("CalibrationInternConique")
                ? d
                : descendant(d, "CalibrationInternConique");
      }
      Element external = child(orientation, "Externe"), matrix = descendant(external, "CodageMatr");
      if (matrix == null)
        throw new IllegalArgumentException("MicMac import needs matrix-coded external orientation");
      double[] r = new double[9];
      for (int row = 0; row < 3; row++) {
        double[] line = vector(matrix, "L" + (row + 1));
        if (line.length != 3) throw new IllegalArgumentException("Invalid MicMac rotation");
        System.arraycopy(line, 0, r, row * 3, 3);
      }
      if (!o.orientationRotation.equals("world-to-camera")) r = Math3.transpose(r);
      String name = file.getFileName().toString();
      if (name.startsWith("Orientation-")) name = name.substring(12, name.length() - 4);
      else name = text(orientation, "NameIm", name);
      model.add(
          new View(
              name,
              micmacCalibration(calibration, model, o),
              centered(r, vector(external, "Centre"))));
    }
    return model;
  }

  private static Calibration micmacCalibration(Element c, Model m, Options o) {
    if (c == null) throw new IllegalArgumentException("Missing MicMac calibration");
    double[] size = vector(c, "SzIm"), pp = vector(c, "PP");
    double f = number(c, "F", Double.NaN);
    double[] d = new double[5];
    Element distortion = child(c, "CalibDistortion"), radial = descendant(distortion, "ModRad");
    if (radial != null) {
      double[] center = vector(radial, "CDist");
      if (Math.hypot(center[0] - pp[0], center[1] - pp[1]) > 1e-8)
        loss(m, o, "MicMac distortion centre differs from principal point; centre is approximated");
      List<Element> coefficients = children(radial, "CoeffDist");
      for (int i = 0; i < coefficients.size(); i++) {
        double value = Double.parseDouble(coefficients.get(i).getTextContent().strip());
        if (i < 3) d[i == 2 ? 4 : i] = value * Math.pow(f, 2 * (i + 1));
        else if (value != 0) loss(m, o, "MicMac radial terms beyond k3 dropped");
      }
    } else if (distortion != null && distortion.hasChildNodes())
      loss(m, o, "MicMac distortion model other than ModRad is dropped");
    String conv = text(c, "KnownConv", "eConvApero_DistM2C");
    if (!conv.equals("eConvApero_DistM2C") && Arrays.stream(d).anyMatch(v -> v != 0))
      throw new IllegalArgumentException(
          "MicMac inverse-distortion convention cannot be represented by forward OpenCV"
              + " coefficients");
    return calibration(f, f, pp[0], pp[1], d, (int) size[0], (int) size[1]);
  }

  static void write(Model model, Path directory, String format, Options o) throws IOException {
    if (format.equals("metashape")) {
      writeMetashape(model, directory);
      return;
    }
    if (format.equals("opencv")) {
      writeOpenCV(model, directory);
      return;
    }
    writeMicmac(model, directory, o);
  }

  private static void writeMetashape(Model model, Path dir) throws IOException {
    StringBuilder s =
        new StringBuilder(
            "<?xml version=\"1.0\"?>\n<document version=\"1.4.0\"><chunk label=\"mSGM\"><sensors>");
    int id = 0;
    for (View v : model.views.values()) {
      Calibration c = Objects.requireNonNull(v.calibration());
      s.append("<sensor id=\"")
          .append(id++)
          .append("\" type=\"frame\"><resolution width=\"")
          .append(c.width)
          .append("\" height=\"")
          .append(c.height)
          .append("\"/><calibration type=\"frame\" class=\"adjusted\"><resolution width=\"")
          .append(c.width)
          .append("\" height=\"")
          .append(c.height)
          .append("\"/>");
      s.append("<width>")
          .append(c.width)
          .append("</width><height>")
          .append(c.height)
          .append("</height><f>")
          .append(c.fy)
          .append("</f><b1>")
          .append(c.fx - c.fy)
          .append("</b1><cx>")
          .append(c.cx + .5 - c.width * .5)
          .append("</cx><cy>")
          .append(c.cy + .5 - c.height * .5)
          .append("</cy>");
      String[] keys = {"k1", "k2", "p2", "p1", "k3"};
      for (int i = 0; i < 5; i++)
        s.append('<')
            .append(keys[i])
            .append('>')
            .append(c.distortion[i])
            .append("</")
            .append(keys[i])
            .append('>');
      s.append("</calibration></sensor>");
    }
    s.append("</sensors><cameras>");
    id = 0;
    for (View v : model.views.values()) {
      if (v.pose() != null)
        s.append("<camera id=\"")
            .append(id)
            .append("\" sensor_id=\"")
            .append(id)
            .append("\" label=\"")
            .append(xml(v.image()))
            .append("\"><transform>")
            .append(matrix4(v.pose()))
            .append("</transform></camera>");
      id++;
    }
    s.append("</cameras></chunk></document>\n");
    Files.writeString(dir.resolve("cameras.xml"), s);
  }

  private static String matrix(String name, int rows, int cols, double[] values) {
    return "<"
        + name
        + " type_id=\"opencv-matrix\"><rows>"
        + rows
        + "</rows><cols>"
        + cols
        + "</cols><dt>d</dt><data>"
        + join(values)
        + "</data></"
        + name
        + ">";
  }

  private static void writeOpenCV(Model model, Path dir) throws IOException {
    StringBuilder s = new StringBuilder("<?xml version=\"1.0\"?>\n<opencv_storage><cameras>");
    for (View v : model.views.values()) {
      Calibration c = Objects.requireNonNull(v.calibration());
      s.append("<_><image>")
          .append(xml(v.image()))
          .append("</image><image_width>")
          .append(c.width)
          .append("</image_width><image_height>")
          .append(c.height)
          .append("</image_height>");
      s.append(matrix("camera_matrix", 3, 3, new double[] {c.fx, 0, c.cx, 0, c.fy, c.cy, 0, 0, 1}))
          .append(matrix("distortion_coefficients", 1, 5, c.distortion));
      if (v.pose() != null)
        s.append(matrix("rotation_matrix", 3, 3, v.pose().rotation))
            .append(matrix("translation_vector", 3, 1, v.pose().translation));
      s.append("</_>");
    }
    Files.writeString(dir.resolve("cameras.xml"), s.append("</cameras></opencv_storage>\n"));
  }

  private static void writeMicmac(Model model, Path dir, Options o) throws IOException {
    int id = 0;
    for (View v : model.views.values()) {
      id++;
      Calibration c = Objects.requireNonNull(v.calibration());
      if (c.fx != c.fy || c.distortion[2] != 0 || c.distortion[3] != 0)
        loss(model, o, "MicMac ModRad export has one focal and radial distortion only");
      double f = (c.fx + c.fy) * .5;
      String calibration = "AutoCal_" + id + ".xml";
      StringBuilder s =
          new StringBuilder(
              "<?xml"
                  + " version=\"1.0\"?><CalibrationInternConique><KnownConv>eConvApero_DistM2C</KnownConv><PP>"
                  + c.cx
                  + " "
                  + c.cy
                  + "</PP><F>"
                  + f
                  + "</F><SzIm>"
                  + c.width
                  + " "
                  + c.height
                  + "</SzIm><CalibDistortion><ModRad><CDist>"
                  + c.cx
                  + " "
                  + c.cy
                  + "</CDist>");
      double[] k = {c.distortion[0], c.distortion[1], c.distortion[4]};
      for (int i = 0; i < 3; i++)
        s.append("<CoeffDist>").append(k[i] / Math.pow(f, 2 * (i + 1))).append("</CoeffDist>");
      s.append("<PPaEqPPs>true</PPaEqPPs></ModRad></CalibDistortion></CalibrationInternConique>\n");
      Files.writeString(dir.resolve(calibration), s);
      if (v.pose() != null) {
        double[] r = Math3.transpose(v.pose().rotation);
        StringBuilder orientation =
            new StringBuilder(
                "<?xml version=\"1.0\"?><OrientationConique><OrIntImaM2C><I00>0 0</I00><V10>1"
                    + " 0</V10><V01>0"
                    + " 1</V01></OrIntImaM2C><TypeProj>eProjStenope</TypeProj><FileInterne>"
                    + calibration
                    + "</FileInterne><RelativeNameFI>true</RelativeNameFI><Externe><AltiSol>0</AltiSol><Profondeur>1</Profondeur><KnownConv>eConvApero_DistM2C</KnownConv><Centre>"
                    + join(v.pose().center())
                    + "</Centre><ParamRotation><CodageMatr>");
        for (int i = 0; i < 3; i++)
          orientation
              .append("<L")
              .append(i + 1)
              .append('>')
              .append(join(Arrays.copyOfRange(r, i * 3, i * 3 + 3)))
              .append("</L")
              .append(i + 1)
              .append('>');
        orientation.append(
            "</CodageMatr></ParamRotation></Externe><ConvOri><KnownConv>eConvApero_DistM2C</KnownConv></ConvOri></OrientationConique>\n");
        Files.writeString(dir.resolve("Orientation-" + v.image() + ".xml"), orientation);
      }
    }
  }
}
