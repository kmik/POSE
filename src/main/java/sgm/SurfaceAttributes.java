package sgm;

import java.io.*;
import java.nio.*;

/** Source-depth neighborhood diagnostics; never a claim of surveyed point accuracy. */
record SurfaceAttributes(
    double roughness,
    double variation,
    double planarity,
    double radius,
    int neighbors,
    int status,
    boolean trimmed) {
  static final int BYTES = 32;
  static final String[] NAMES = {
    "normal_x",
    "normal_y",
    "normal_z",
    "roughness_rms",
    "surface_variation",
    "planarity",
    "normal_radius",
    "normal_neighbors",
    "surface_status",
    "normal_trimmed"
  };
  static final String[] DESCRIPTIONS = {
    "Source-camera oriented unit X",
    "Source-camera oriented unit Y",
    "Source-camera oriented unit Z",
    "sqrt(lambda0), coordinate units",
    "lambda0 / sum(lambda)",
    "(lambda1-lambda0) / lambda2",
    "Max neighbor distance, units",
    "Robust neighborhood sample count",
    "0=valid,1=unreliable,2=normal only",
    "1 if robust fit removed samples"
  };

  static SurfaceAttributes missing(boolean normal) {
    return new SurfaceAttributes(
        Double.NaN, Double.NaN, Double.NaN, Double.NaN, 0, normal ? 2 : 1, false);
  }

  static SurfaceAttributes of(
      SurfaceGeometry.Plane plane, double[][] points, int n, double[] center, boolean trimmed) {
    double sum = plane.smallest() + plane.middle() + plane.largest(), radius = 0;
    for (int i = 0; i < n; i++) {
      double x = points[i][0] - center[0],
          y = points[i][1] - center[1],
          z = points[i][2] - center[2];
      radius = Math.max(radius, x * x + y * y + z * z);
    }
    radius = Math.sqrt(radius);
    if (!(sum > 0 && plane.largest() > 0)) return missing(true);
    return new SurfaceAttributes(
        Math.sqrt(plane.smallest()),
        plane.smallest() / sum,
        (plane.middle() - plane.smallest()) / plane.largest(),
        radius,
        n,
        0,
        trimmed);
  }

  void binary(ByteBuffer out, double[] normal, double scale) {
    for (int k = 0; k < 3; k++) out.putFloat(normal == null ? Float.NaN : (float) normal[k]);
    out.putFloat((float) (roughness * scale))
        .putFloat((float) variation)
        .putFloat((float) planarity)
        .putFloat((float) (radius * scale))
        .putShort((short) neighbors)
        .put((byte) status)
        .put((byte) (trimmed ? 1 : 0));
  }

  String csv(double scale) {
    return (roughness * scale)
        + ","
        + variation
        + ","
        + planarity
        + ","
        + (radius * scale)
        + ","
        + neighbors
        + ","
        + status
        + ","
        + trimmed;
  }

  void write(DataOutput out) throws IOException {
    out.writeByte(status);
    if (status != 0) return;
    out.writeDouble(roughness);
    out.writeDouble(variation);
    out.writeDouble(planarity);
    out.writeDouble(radius);
    out.writeInt(neighbors);
    out.writeBoolean(trimmed);
  }

  int storageBytes() {
    return status == 0 ? 38 : 1;
  }

  static SurfaceAttributes read(DataInput in) throws IOException {
    int status = in.readUnsignedByte();
    if (status == 1 || status == 2) return missing(status == 2);
    if (status != 0) throw new IOException("Invalid surface attribute status");
    var a =
        new SurfaceAttributes(
            in.readDouble(),
            in.readDouble(),
            in.readDouble(),
            in.readDouble(),
            in.readInt(),
            status,
            in.readBoolean());
    return a;
  }
}
