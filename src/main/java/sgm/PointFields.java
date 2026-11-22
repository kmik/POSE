package sgm;

import java.nio.ByteBuffer;
import java.util.*;

/**
 * {@code --point-attributes LIST}: the per-point values written into LAS Extra Bytes and PLY
 * properties. {@code surface} (the default) and {@code none} keep the writer's original layout
 * byte for byte ({@link CloudWriter} uses this class only for other lists). Fields are always
 * written in the order of {@link Field}, whatever the order of the list.
 *
 * <p>Uncertainty fields ({@code height_sigma}, {@code ray_sigma}) are 1-sigma values in output
 * units, from the fusion model sigma (triangulation angle and footprint, combined over the
 * averaged cameras) scaled by the run's variance factor: the spread of the depths fusion averaged
 * against what the model predicts (FusionOutput). They describe local precision from the images;
 * orientation and absolute errors are not included.
 */
final class PointFields {
  private PointFields() {}

  /** LAS Extra Bytes data types (ASPRS LAS 1.4 R15, table 24). */
  static final int UCHAR = 1, USHORT = 3, FLOAT = 9;

  enum Field {
    NORMAL("normal", "unit normal toward the source camera", new String[] {"normal_x", "normal_y", "normal_z"},
        new String[] {"nx", "ny", "nz"}, FLOAT, FLOAT, FLOAT),
    ROUGHNESS("roughness", "RMS of a local plane fit", new String[] {"roughness_rms"}, null, FLOAT),
    SURFACE_VARIATION("surface_variation", "smallest eigenvalue share", new String[] {"surface_variation"}, null, FLOAT),
    PLANARITY("planarity", "planarity of the neighbourhood", new String[] {"planarity"}, null, FLOAT),
    NORMAL_DIAGNOSTICS("normal_diagnostics", "normal fit radius, neighbours, status, trimmed",
        new String[] {"normal_radius", "normal_neighbors", "surface_status", "normal_trimmed"}, null, FLOAT, USHORT, UCHAR, UCHAR),
    HEIGHT_SIGMA("height_sigma", "calibrated 1-sigma of the height (local precision)", new String[] {"height_sigma"}, null, FLOAT),
    RAY_SIGMA("ray_sigma", "calibrated 1-sigma along the viewing ray (local precision)", new String[] {"ray_sigma"}, null, FLOAT),
    DEPTH_SPREAD("depth_spread", "std of the averaged cameras' depths (3+ cameras)", new String[] {"depth_spread"}, null, FLOAT),
    VIEWS("views", "cameras that confirm the point", new String[] {"views"}, null, UCHAR),
    ANGLE("angle", "largest triangulation angle among confirming cameras, degrees", new String[] {"angle"}, null, FLOAT),
    CONFIDENCE("confidence", "final point confidence 0..1", new String[] {"confidence"}, null, FLOAT),
    /** The source camera goes into the standard LAS Point Source ID: no extra bytes. */
    SOURCE("source", "source camera index (LAS Point Source ID)", new String[0], null);

    final String name, description;
    final String[] las, ply;
    final int[] types;

    Field(String name, String description, String[] las, String[] ply, int... types) {
      if (types.length != las.length) throw new IllegalStateException(name + ": one type per value");
      this.name = name;
      this.description = description;
      this.las = las;
      this.ply = ply == null ? las : ply;
      this.types = types;
    }

    int bytes() {
      int b = 0;
      for (int t : types) b += size(t);
      return b;
    }
  }

  static int size(int type) {
    return type == FLOAT ? 4 : type == USHORT ? 2 : 1;
  }

  static final EnumSet<Field> SURFACE =
      EnumSet.of(Field.NORMAL, Field.ROUGHNESS, Field.SURFACE_VARIATION, Field.PLANARITY, Field.NORMAL_DIAGNOSTICS);
  static final EnumSet<Field> UNCERTAINTY = EnumSet.of(Field.HEIGHT_SIGMA, Field.RAY_SIGMA);

  /** Names accepted in the list: fields and groups. */
  static final List<String> NAMES = names();

  private static List<String> names() {
    List<String> out = new ArrayList<>(List.of("surface", "uncertainty", "all", "none"));
    for (Field f : Field.values()) out.add(f.name);
    return List.copyOf(out);
  }

  /** The fields of a list ("surface,height_sigma"); an empty set for "none". */
  static EnumSet<Field> parse(String list) {
    EnumSet<Field> out = EnumSet.noneOf(Field.class);
    boolean none = false;
    for (String raw : list.split(",")) {
      String name = raw.strip().toLowerCase(Locale.ROOT);
      if (name.isEmpty()) continue;
      switch (name) {
        case "none" -> none = true;
        case "surface" -> out.addAll(SURFACE);
        case "uncertainty" -> out.addAll(UNCERTAINTY);
        case "all" -> out.addAll(EnumSet.allOf(Field.class));
        default -> {
          Field f = null;
          for (Field g : Field.values()) if (g.name.equals(name)) f = g;
          if (f == null)
            throw new IllegalArgumentException("--point-attributes: unknown value " + raw.strip() + " (use " + String.join(", ", NAMES) + ")");
          out.add(f);
        }
      }
    }
    if (none && !out.isEmpty()) throw new IllegalArgumentException("--point-attributes none cannot be combined with fields");
    if (!none && out.isEmpty()) throw new IllegalArgumentException("--point-attributes needs at least one value (or none)");
    return out;
  }

  /** Whether a list needs the surface neighbourhood attributes (normals, roughness ...). */
  static boolean needsSurface(EnumSet<Field> f) {
    for (Field s : SURFACE) if (f.contains(s)) return true;
    return false;
  }

  /** Whether the list is exactly one of the original layouts (surface or none). */
  static boolean legacy(EnumSet<Field> f) {
    return f.isEmpty() || f.equals(SURFACE);
  }

  /** Extra bytes per point. */
  static int bytes(EnumSet<Field> f) {
    int b = 0;
    for (Field x : f) b += x.bytes();
    return b;
  }

  /** Per-point values for the generic layout. NaN / -1 when not available. */
  static final class Values {
    double[] normal;
    SurfaceAttributes surface = SurfaceAttributes.missing(false);
    double heightSigma = Double.NaN, raySigma = Double.NaN, depthSpread = Double.NaN, angle = Double.NaN, confidence = Double.NaN;
    int views = -1, source = -1;
  }

  /**
   * The extra bytes of one point, in field order. Surface values come from SurfaceAttributes'
   * own binary form (the same bytes as the original layout), split per field.
   */
  static void write(EnumSet<Field> fields, Values v, double scale, ByteBuffer out, ByteBuffer surfaceScratch) {
    boolean surface = needsSurface(fields);
    if (surface) {
      surfaceScratch.clear();
      v.surface.binary(surfaceScratch, v.normal, scale);
    }
    for (Field f : fields) {
      switch (f) {
        case NORMAL -> out.put(surfaceScratch.array(), 0, 12);
        case ROUGHNESS -> out.put(surfaceScratch.array(), 12, 4);
        case SURFACE_VARIATION -> out.put(surfaceScratch.array(), 16, 4);
        case PLANARITY -> out.put(surfaceScratch.array(), 20, 4);
        case NORMAL_DIAGNOSTICS -> out.put(surfaceScratch.array(), 24, 8);
        case HEIGHT_SIGMA -> out.putFloat((float) v.heightSigma);
        case RAY_SIGMA -> out.putFloat((float) v.raySigma);
        case DEPTH_SPREAD -> out.putFloat((float) v.depthSpread);
        case VIEWS -> out.put((byte) Math.max(0, Math.min(255, v.views)));
        case ANGLE -> out.putFloat((float) v.angle);
        case CONFIDENCE -> out.putFloat((float) v.confidence);
        case SOURCE -> {}
      }
    }
  }

  /** Byte offsets (within the extra bytes) of the fields that the run's calibration scales. */
  static int[] calibratedOffsets(EnumSet<Field> fields) {
    List<Integer> out = new ArrayList<>();
    int at = 0;
    for (Field f : fields) {
      if (f == Field.HEIGHT_SIGMA || f == Field.RAY_SIGMA) out.add(at);
      at += f.bytes();
    }
    return out.stream().mapToInt(Integer::intValue).toArray();
  }

  /** ASPRS Extra Bytes VLR (record 4) describing the fields. */
  static byte[] extraVlr(EnumSet<Field> fields) {
    int descriptors = 0;
    for (Field f : fields) descriptors += f.las.length;
    ByteBuffer b = ByteBuffer.allocate(54 + 192 * descriptors).order(java.nio.ByteOrder.LITTLE_ENDIAN);
    b.putShort((short) 0);
    ascii(b, "LASF_Spec", 16);
    b.putShort((short) 4).putShort((short) (192 * descriptors));
    ascii(b, "POSE point attributes", 32);
    for (Field f : fields)
      for (int i = 0; i < f.las.length; i++) {
        int start = b.position();
        boolean real = f.types[i] == FLOAT;
        b.putShort((short) 0).put((byte) f.types[i]).put((byte) (real ? 1 : 0));
        ascii(b, f.las[i], 32);
        b.putInt(0);
        if (real) b.putDouble(Double.NaN);
        b.position(start + 160);
        ascii(b, f.description, 32);
      }
    return b.array();
  }

  /** PLY property lines of the fields. */
  static String plyProperties(EnumSet<Field> fields) {
    StringBuilder s = new StringBuilder();
    if (needsSurface(fields)) s.append("comment surface_status: 0=valid,1=unreliable,2=normal_only\n");
    if (fields.contains(Field.HEIGHT_SIGMA) || fields.contains(Field.RAY_SIGMA))
      s.append("comment height_sigma, ray_sigma: calibrated 1-sigma local precision in output units\n");
    for (Field f : fields)
      for (int i = 0; i < f.ply.length; i++)
        s.append("property ").append(f.types[i] == FLOAT ? "float" : f.types[i] == USHORT ? "ushort" : "uchar")
            .append(' ').append(f.ply[i]).append('\n');
    return s.toString();
  }

  private static void ascii(ByteBuffer b, String text, int length) {
    byte[] t = text.getBytes(java.nio.charset.StandardCharsets.US_ASCII);
    b.put(t, 0, Math.min(t.length, length));
    for (int i = t.length; i < length; i++) b.put((byte) 0);
  }
}
