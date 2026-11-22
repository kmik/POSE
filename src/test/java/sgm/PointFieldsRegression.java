package sgm;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.*;

/**
 * --point-attributes LIST: parsing, the original layouts unchanged (surface and none through the
 * field-list constructor give the same bytes as before), the generic layout read back from LAS and
 * PLY (Extra Bytes descriptors, field values, Point Source ID, tiles, far-point rewrite), the final
 * calibration pass, and the option rules.
 */
final class PointFieldsRegression {
  private static int checks;

  private static void check(boolean value, String why) {
    checks++;
    if (!value) throw new AssertionError(why);
  }

  public static void main(String[] args) throws Exception {
    Path root = Files.createTempDirectory(Path.of("target"), "point-fields-");
    try {
      System.out.println("PASS point fields: " + run(root));
    } finally {
      LargeFormatRegression.delete(root);
    }
  }

  static boolean refused(String list) {
    try {
      PointFields.parse(list);
      return false;
    } catch (IllegalArgumentException e) {
      return true;
    }
  }

  static boolean optionsRefused(String... extra) {
    List<String> a = new ArrayList<>(List.of("-i", ".", "-o", "fields-check"));
    a.addAll(List.of(extra));
    try {
      Options.parse(a.toArray(String[]::new));
      return false;
    } catch (IllegalArgumentException e) {
      return true;
    }
  }

  /** Deterministic test points; every 5th has NaN sigmas. */
  static List<PointFields.Values> values(int n) {
    List<PointFields.Values> out = new ArrayList<>();
    Random r = new Random(3);
    for (int i = 0; i < n; i++) {
      PointFields.Values v = new PointFields.Values();
      v.normal = new double[] {0, 0, 1};
      v.surface = SurfaceAttributes.missing(true);
      v.heightSigma = i % 5 == 0 ? Double.NaN : .01 + r.nextDouble() * .05;
      v.raySigma = i % 5 == 0 ? Double.NaN : v.heightSigma * 1.2;
      v.depthSpread = .02;
      v.views = 2 + i % 9;
      v.angle = 10 + i % 30;
      v.confidence = .5;
      v.source = i % 17;
      out.add(v);
    }
    return out;
  }

  static double[] xyz(int i) {
    return new double[] {1000 + (i % 40) * .5, 2000 + (i / 40) * .5, 100 + (i % 7) * .1};
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Files.createDirectories(root);
    // Parsing.
    check(PointFields.parse("surface").equals(PointFields.SURFACE) && PointFields.parse("none").isEmpty(), "surface / none");
    check(PointFields.parse("uncertainty,views").equals(EnumSet.of(PointFields.Field.HEIGHT_SIGMA, PointFields.Field.RAY_SIGMA, PointFields.Field.VIEWS)),
        "groups and fields combine");
    check(PointFields.parse("views, Source").contains(PointFields.Field.SOURCE), "case and spaces");
    check(PointFields.legacy(PointFields.parse("surface")) && PointFields.legacy(PointFields.parse("none"))
        && PointFields.legacy(PointFields.parse("normal,roughness,surface_variation,planarity,normal_diagnostics"))
        && !PointFields.legacy(PointFields.parse("surface,views")), "original layouts recognised");
    check(PointFields.bytes(PointFields.SURFACE) == SurfaceAttributes.BYTES, "surface group = the original 32 bytes");
    check(PointFields.bytes(PointFields.parse("all")) == 53, "all fields: " + PointFields.bytes(PointFields.parse("all")));
    check(refused("height") && refused("none,views") && refused(""), "unknown names, none with fields and empty lists are refused");
    for (PointFields.Field f : PointFields.Field.values()) check(f.types.length == f.las.length, f.name + ": one type per value");

    // The original layouts through the field-list constructor: the same bytes as before.
    for (String list : List.of("surface", "none")) {
      Path a = root.resolve(list + "-old"), b = root.resolve(list + "-new");
      boolean surface = list.equals("surface");
      try (CloudWriter old = new CloudWriter(a, 1, null, surface); CloudWriter now = new CloudWriter(b, 1, null, PointFields.parse(list))) {
        List<PointFields.Values> vs = values(500);
        for (int i = 0; i < vs.size(); i++) {
          double[] p = xyz(i);
          old.point(p[0], p[1], p[2], 10, 20, 30, vs.get(i).normal, vs.get(i).surface);
          now.point(p[0], p[1], p[2], 10, 20, 30, vs.get(i));
        }
      }
      for (String f : List.of("dense.las", "dense.ply"))
        check(Files.mismatch(a.resolve(f), b.resolve(f)) == -1, list + ": " + f + " byte-identical to the original writer");
    }

    // Generic layout: single file and PLY, calibrated by 2; tiles; far-point rewrite.
    EnumSet<PointFields.Field> fields = PointFields.parse("views,height_sigma,angle,source,ray_sigma");
    int extra = PointFields.bytes(fields);
    check(extra == 1 + 4 + 4 + 4, "bytes of views,height_sigma,angle,ray_sigma: " + extra);
    check(new ArrayList<>(fields).equals(List.of(PointFields.Field.HEIGHT_SIGMA, PointFields.Field.RAY_SIGMA, PointFields.Field.VIEWS,
        PointFields.Field.ANGLE, PointFields.Field.SOURCE)), "fields are written in a fixed order, whatever the list order");
    Path g = root.resolve("generic"), t = root.resolve("tiles"), far = root.resolve("far");
    List<PointFields.Values> vs = values(1000);
    try (CloudWriter w = new CloudWriter(g, 1, null, fields); CloudWriter tw = new CloudWriter(t, 1, null, fields);
        CloudWriter fw = new CloudWriter(far, 1, null, fields)) {
      tw.tiles(5, t.resolve("tiles"));
      fw.writePly(false);
      for (int i = 0; i < vs.size(); i++) {
        double[] p = xyz(i);
        w.point(p[0], p[1], p[2], 1, 2, 3, vs.get(i));
        tw.point(p[0], p[1], p[2], 1, 2, 3, vs.get(i));
        // The last points lie far away: the LAS is rewritten once with centred offsets.
        fw.point(p[0] + (i > 900 ? 1e6 : 0), p[1], p[2], 1, 2, 3, vs.get(i));
      }
      w.calibration(2);
      tw.calibration(2);
      fw.calibration(2);
    }
    // Field order is the enum order: height_sigma, ray_sigma, views, angle (source: Point Source ID).
    int hOff = 0, rOff = 4, vOff = 8, aOff = 9;
    for (Path las : lasFiles(g, t, far)) {
      byte[] all = Files.readAllBytes(las);
      ByteBuffer b = ByteBuffer.wrap(all).order(ByteOrder.LITTLE_ENDIAN);
      int data = b.getInt(96), len = b.getShort(105) & 0xffff;
      long n = b.getInt(107) & 0xffffffffL;
      check(len == 26 + extra, las.getFileName() + ": record length");
      String vlr = new String(all, 227, data - 227, StandardCharsets.US_ASCII);
      check(vlr.contains("height_sigma") && vlr.contains("ray_sigma") && vlr.contains("angle") && vlr.contains("views"),
          las.getFileName() + ": Extra Bytes descriptors name the fields");
      int bad = 0;
      for (int i = 0; i < n; i++) {
        int at = data + i * len, e = at + 26;
        int source = b.getShort(at + 18) & 0xffff, views = b.get(e + vOff) & 255;
        float h = b.getFloat(e + hOff), r = b.getFloat(e + rOff), angle = b.getFloat(e + aOff);
        // Recover the point's index from its views/source/angle pattern is not unique; check consistency instead.
        if (views < 2 || views > 10 || source > 16 || angle < 10 || angle >= 40) bad++;
        if (Float.isNaN(h) != Float.isNaN(r)) bad++;
        if (!Float.isNaN(h) && (h < .02 - 1e-6 || h > .12 + 1e-6 || Math.abs(r - 1.2 * h) > 1e-5)) bad++;
      }
      check(bad == 0, las.getFileName() + ": values in range, sigmas doubled by the calibration, NaN kept: " + bad);
    }
    // Exact values in the single file (point order kept).
    ByteBuffer single = ByteBuffer.wrap(Files.readAllBytes(g.resolve("dense.las"))).order(ByteOrder.LITTLE_ENDIAN);
    int data = single.getInt(96), len = single.getShort(105) & 0xffff;
    int exact = 0;
    for (int i = 0; i < vs.size(); i++) {
      int at = data + i * len, e = at + 26;
      PointFields.Values v = vs.get(i);
      boolean ok = (single.getShort(at + 18) & 0xffff) == v.source && (single.get(e + vOff) & 255) == v.views
          && single.getFloat(e + aOff) == (float) v.angle
          && (Double.isNaN(v.heightSigma) ? Float.isNaN(single.getFloat(e + hOff)) : single.getFloat(e + hOff) == (float) ((float) v.heightSigma * 2.0));
      if (ok) exact++;
    }
    check(exact == vs.size(), "single file: every point's fields exact (" + exact + "/" + vs.size() + ")");
    // PLY: same properties and the calibrated value.
    byte[] ply = Files.readAllBytes(g.resolve("dense.ply"));
    String text = new String(ply, 0, Math.min(ply.length, 4000), StandardCharsets.US_ASCII);
    int headerEnd = text.indexOf("end_header\n") + "end_header\n".length();
    check(text.contains("property float height_sigma") && text.contains("property uchar views") && text.contains("property float ray_sigma"),
        "PLY header lists the fields");
    ByteBuffer pb = ByteBuffer.wrap(ply).order(ByteOrder.LITTLE_ENDIAN);
    int plyLen = 27 + extra, wrong = 0;
    for (int i = 0; i < vs.size(); i++) {
      float h = pb.getFloat(headerEnd + i * plyLen + 27 + hOff);
      double want = vs.get(i).heightSigma;
      if (Double.isNaN(want) ? !Float.isNaN(h) : h != (float) ((float) want * 2.0)) wrong++;
    }
    check(wrong == 0, "PLY sigmas calibrated as in the LAS: " + wrong);

    // Option rules.
    check(!optionsRefused("--dense-method", "mvsgm", "--fusion-mode", "image", "--point-attributes", "all"), "all with image fusion");
    check(optionsRefused("--fusion-mode", "reference", "--point-attributes", "height_sigma"), "evidence fields need image fusion");
    check(!optionsRefused("--fusion-mode", "reference", "--point-attributes", "views,source,confidence"), "plain fields work in any fusion mode");
    check(optionsRefused("--point-attributes", "sigma"), "unknown field refused at parse time");
    GuiOptions.Field gui = GuiOptions.field("--point-attributes");
    check(gui != null && gui.kind() == GuiOptions.Kind.MULTI && gui.choices().containsAll(List.of("surface", "uncertainty", "height_sigma", "none")),
        "GUI offers the fields as checkboxes");
    for (String c : gui.choices()) if (!c.isEmpty()) check(!refused(c), "GUI choice " + c + " parses");
    return checks;
  }

  static List<Path> lasFiles(Path... dirs) throws java.io.IOException {
    List<Path> out = new ArrayList<>();
    for (Path d : dirs)
      try (var s = Files.walk(d)) {
        s.filter(p -> p.toString().endsWith(".las")).sorted().forEach(out::add);
      }
    return out;
  }
}
