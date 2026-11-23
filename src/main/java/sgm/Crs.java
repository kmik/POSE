package sgm;

import java.util.Locale;
import java.util.Map;
import org.locationtech.proj4j.*;

/**
 * Projected CRS for x/y, with an optional vertical CRS label for heights: "EPSG:H" or the
 * compound "EPSG:H+V" (e.g. EPSG:3067+3900 = ETRS-TM35FIN + N2000). The vertical part only labels
 * the heights in the output files; it converts nothing (see --source-geoid/--target-geoid).
 */
final class Crs {
  /** Horizontal name "EPSG:H" (also without a vertical part, so files and comparisons stay as before). */
  final String name, parameters;
  final int epsg;
  /** Vertical CRS EPSG code, or 0 when the heights are not labelled. */
  final int vertical;
  private final CoordinateTransform forward;
  private CoordinateTransform inverse;

  /** Common vertical CRSs, for messages and reports. */
  static final Map<Integer, String> VERTICAL_NAMES =
      Map.ofEntries(
          Map.entry(3900, "N2000 height"),
          Map.entry(5717, "N60 height"),
          Map.entry(5773, "EGM96 height"),
          Map.entry(3855, "EGM2008 height"),
          Map.entry(5703, "NAVD88 height"),
          Map.entry(5701, "ODN height"),
          Map.entry(7837, "DHHN2016 height"),
          Map.entry(5941, "NN2000 height"),
          Map.entry(5613, "RH2000 height"),
          Map.entry(5799, "DVR90 height"),
          Map.entry(5709, "NAP height"));

  static String normalize(String value) {
    String s = value.trim().toUpperCase(Locale.ROOT).replace(" ", "");
    if (s.startsWith("EPSG:")) s = s.substring(5);
    String[] parts = s.split("[+]", -1);
    if (parts.length > 2 || !parts[0].matches("[0-9]+") || parts.length == 2 && !parts[1].matches("(EPSG:)?[0-9]+"))
      throw new IllegalArgumentException(
          "--crs must be EPSG:CODE, a numeric EPSG code, or EPSG:CODE+VERTICAL_CODE (e.g. EPSG:3067+3900)");
    int code = code(parts[0], value);
    String name = "EPSG:" + code;
    if (parts.length == 2) {
      int v = code(parts[1].replace("EPSG:", ""), value);
      CoordinateReferenceSystem horizontal = null;
      try {
        horizontal = new CRSFactory().createFromName("EPSG:" + v);
      } catch (RuntimeException notHorizontal) {
        // expected for vertical codes: the horizontal CRS library does not know them
      }
      if (horizontal != null)
        throw new IllegalArgumentException(
            "EPSG:" + v + " in " + value + " is a horizontal CRS; the part after + must be a vertical CRS such as 3900 (N2000)");
      name += "+" + v;
    }
    return name;
  }

  private static int code(String digits, String value) {
    int code;
    try {
      code = Integer.parseInt(digits);
    } catch (NumberFormatException e) {
      throw new IllegalArgumentException("Invalid EPSG code: " + value);
    }
    if (code < 1 || code > 32766)
      throw new IllegalArgumentException("LAS 1.2 GeoTIFF output supports EPSG codes 1..32766");
    return code;
  }

  /** Vertical code of a CRS option value, 0 without one. */
  static int verticalOf(String value) {
    if (value == null) return 0;
    String n = normalize(value);
    int plus = n.indexOf('+');
    return plus < 0 ? 0 : Integer.parseInt(n.substring(plus + 1));
  }

  /** "EPSG:3900 (N2000 height)" or "EPSG:V". */
  static String verticalName(int code) {
    String known = VERTICAL_NAMES.get(code);
    return "EPSG:" + code + (known == null ? "" : " (" + known + ")");
  }

  /** Full option form: "EPSG:H" or "EPSG:H+V". */
  String spec() {
    return vertical > 0 ? name + "+" + vertical : name;
  }

  /** The same horizontal CRS with another vertical label (0: none). */
  Crs withVertical(int v) {
    return v == vertical ? this : new Crs(v > 0 ? name + "+" + v : name);
  }

  Crs(String value) {
    String full = normalize(value);
    int plus = full.indexOf('+');
    name = plus < 0 ? full : full.substring(0, plus);
    vertical = plus < 0 ? 0 : Integer.parseInt(full.substring(plus + 1));
    epsg = Integer.parseInt(name.substring(5));
    try {
      CRSFactory factory = new CRSFactory();
      CoordinateReferenceSystem target = factory.createFromName(name);
      if (target == null
          || target.isGeographic()
          || target.getParameterString().contains("+proj=geocent")
          || Math.abs(target.getProjection().getFromMetres() - 1) > 1e-12)
        throw new IllegalArgumentException(
            "Choose a projected EPSG CRS with metre units; geographic, geocentric and foot CRSs"
                + " cannot define a metric 3D similarity");
      parameters = target.getParameterString();
      forward =
          new CoordinateTransformFactory()
              .createTransform(factory.createFromName("EPSG:4326"), target);
    } catch (RuntimeException e) {
      throw new IllegalArgumentException(
          "Unsupported output CRS " + name + ": " + e.getMessage(), e);
    }
  }

  double[] project(Exif.Gps gps) {
    ProjCoordinate p =
        forward.transform(new ProjCoordinate(gps.longitude, gps.latitude), new ProjCoordinate());
    if (!Double.isFinite(p.x + p.y))
      throw new IllegalArgumentException("GPS position cannot be projected into " + name);
    return new double[] {p.x, p.y, gps.altitude};
  }

  /** {latitude, longitude} in degrees of projected easting/northing. */
  synchronized double[] geographic(double x, double y) {
    if (inverse == null) {
      CRSFactory factory = new CRSFactory();
      inverse =
          new CoordinateTransformFactory()
              .createTransform(factory.createFromName(name), factory.createFromName("EPSG:4326"));
    }
    ProjCoordinate p = inverse.transform(new ProjCoordinate(x, y), new ProjCoordinate());
    if (!Double.isFinite(p.x + p.y))
      throw new IllegalArgumentException("Coordinates " + x + ", " + y + " cannot be converted from " + name + " to latitude/longitude");
    return new double[] {p.y, p.x};
  }
}
