package sgm;

import java.util.Locale;
import org.locationtech.proj4j.*;

/** Horizontal projected CRS; heights retain the EXIF vertical reference in metres. */
final class Crs {
  final String name, parameters;
  final int epsg;
  private final CoordinateTransform forward;

  static String normalize(String value) {
    String s = value.trim().toUpperCase(Locale.ROOT);
    if (s.matches("[0-9]+")) s = "EPSG:" + s;
    if (!s.matches("EPSG:[0-9]+"))
      throw new IllegalArgumentException("--crs must be EPSG:CODE or a numeric EPSG code");
    int code;
    try {
      code = Integer.parseInt(s.substring(5));
    } catch (NumberFormatException e) {
      throw new IllegalArgumentException("Invalid EPSG code: " + value);
    }
    if (code < 1 || code > 32766)
      throw new IllegalArgumentException("LAS 1.2 GeoTIFF output supports EPSG codes 1..32766");
    return "EPSG:" + code;
  }

  Crs(String value) {
    name = normalize(value);
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
}
