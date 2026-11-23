package sgm;

import java.io.*;
import java.nio.*;
import java.nio.file.*;
import java.util.*;

/**
 * --source-geoid/--target-geoid: text and .gtx grid readers agree, bilinear interpolation,
 * missing nodes and out-of-grid cameras, the conversion arithmetic, option parsing, and no change
 * without the options.
 */
final class VerticalDatumRegression {
  private static int checks;

  private static void check(boolean value, String why) {
    checks++;
    if (!value) throw new AssertionError(why);
  }

  public static void main(String[] args) throws Exception {
    System.out.println("PASS vertical datum: " + run(Files.createTempDirectory("vertical-datum")));
  }

  /** Planar test surface: bilinear interpolation reproduces it exactly. */
  static double surface(double lat, double lon) {
    return 15 + 0.8 * (lat - 60) + 0.3 * (lon - 20);
  }

  static Options parse(String... a) {
    List<String> all = new ArrayList<>(List.of("-i", ".", "-o", "vertical-datum-check"));
    all.addAll(List.of(a));
    return Options.parse(all.toArray(String[]::new));
  }

  static boolean refused(Runnable r) {
    try {
      r.run();
      return false;
    } catch (IllegalArgumentException e) {
      return true;
    }
  }

  static int run(Path dir) throws Exception {
    checks = 0;
    Files.createDirectories(dir);
    // 5 x 9 grid: latitude 60..62 step 0.5, longitude 20..24 step 0.5.
    int rows = 5, cols = 9;
    StringBuilder text = new StringBuilder("# lat lon N\nlatitude longitude height\n");
    ByteBuffer gtx = ByteBuffer.allocate(40 + 4 * rows * cols).order(ByteOrder.BIG_ENDIAN);
    gtx.putDouble(60).putDouble(20).putDouble(.5).putDouble(.5).putInt(rows).putInt(cols);
    for (int i = 0; i < rows; i++)
      for (int j = 0; j < cols; j++) {
        double lat = 60 + .5 * i, lon = 20 + .5 * j, n = surface(lat, lon);
        text.append(String.format(Locale.ROOT, "%.4f, %.4f, %.6f%n", lat, lon, n));
        gtx.putFloat((float) n);
      }
    Path txt = dir.resolve("geoid.txt"), bin = dir.resolve("geoid.gtx");
    Files.writeString(txt, text);
    Files.write(bin, gtx.array());

    VerticalDatum.Grid a = VerticalDatum.text(txt), b = VerticalDatum.gtx(bin);
    check(a.rows == rows && a.cols == cols, "text grid size " + a.rows + "x" + a.cols);
    check(b.rows == rows && b.cols == cols, "gtx grid size");
    double[][] probes = {{60, 20}, {61.23, 21.77}, {62, 24}, {60.5, 23.999}, {61.75, 20.01}};
    for (double[] p : probes) {
      double want = surface(p[0], p[1]);
      check(Math.abs(a.at(p[0], p[1]) - want) < 1e-5, "text bilinear at " + Arrays.toString(p) + ": " + a.at(p[0], p[1]) + " vs " + want);
      check(Math.abs(b.at(p[0], p[1]) - want) < 1e-5, "gtx bilinear at " + Arrays.toString(p));
    }
    check(Double.isNaN(a.at(59.9, 21)) && Double.isNaN(a.at(61, 24.2)), "outside the grid is NaN");
    // Longitude given as 0..360 in the grid's convention still resolves (grids that start east of 180).
    check(Math.abs(b.at(61, 21 - 360) - surface(61, 21)) < 1e-5, "longitude wraps by 360");

    // A missing node (text: absent line; gtx: -88.8888) leaves its cells NaN.
    String holed = text.toString().replace(String.format(Locale.ROOT, "%.4f, %.4f, %.6f%n", 61.0, 21.0, surface(61, 21)), "");
    check(!holed.equals(text.toString()), "test removed one node");
    Path hole = dir.resolve("hole.txt");
    Files.writeString(hole, holed);
    VerticalDatum.Grid h = VerticalDatum.text(hole);
    check(Double.isNaN(h.at(61.2, 21.2)) && Double.isFinite(h.at(60.2, 22.7)), "missing node: only its cells are NaN");
    ByteBuffer holedGtx = ByteBuffer.wrap(gtx.array().clone()).order(ByteOrder.BIG_ENDIAN);
    holedGtx.putFloat(40 + 4 * (2 * cols + 2), -88.8888f);
    Path holeGtx = dir.resolve("hole.gtx");
    Files.write(holeGtx, holedGtx.array());
    check(Double.isNaN(VerticalDatum.gtx(holeGtx).at(61.2, 21.2)), "gtx -88.8888 is a missing node");

    // Irregular and truncated files are refused.
    Path irregular = dir.resolve("irregular.txt");
    Files.writeString(irregular, "60 20 1\n60 20.5 1\n60 20.7 1\n60.5 20 1\n60.5 20.5 1\n60.5 20.7 1\n");
    check(refused(() -> { try { VerticalDatum.text(irregular); } catch (IOException e) { throw new UncheckedIOException(e); } }), "irregular grid refused");
    Path shortGtx = dir.resolve("short.gtx");
    Files.write(shortGtx, Arrays.copyOf(gtx.array(), 100));
    check(refused(() -> { try { VerticalDatum.gtx(shortGtx); } catch (IOException e) { throw new UncheckedIOException(e); } }), "truncated gtx refused");

    // Options.
    Options none = parse();
    check(none.sourceGeoid == null && none.targetGeoid == null && !VerticalDatum.active(none), "no conversion by default");
    Exif.Gps camera = new Exif.Gps(61.2, 22.3, 150.0);
    check(VerticalDatum.convert(camera, none) == camera, "without the options the GPS object is unchanged");
    check(VerticalDatum.heightNote(none).equals(Georeference.HEIGHT_NOTE), "default height note unchanged");
    check(VerticalDatum.datumName(none) == null, "no vertical datum named by default");
    check(refused(() -> parse("--target-geoid", dir.resolve("missing.gtx").toString())), "missing grid file refused");
    check(refused(() -> parse("--source-geoid", " ")), "empty value refused");
    check(parse("--source-geoid", "ELLIPSOID").sourceGeoid.equals(VerticalDatum.ELLIPSOID), "ellipsoid keyword");
    check(parse("--source-geoid", "none").sourceGeoid.equals(VerticalDatum.ELLIPSOID), "none means ellipsoid");

    // Ellipsoidal (RTK) -> geoid: H = h - N.
    Options toGeoid = parse("--target-geoid", txt.toString());
    double n = surface(61.2, 22.3);
    Exif.Gps converted = VerticalDatum.convert(camera, toGeoid);
    check(Math.abs(converted.altitude - (150 - n)) < 1e-5, "ellipsoid -> geoid: " + converted.altitude);
    check(converted.latitude == camera.latitude && converted.longitude == camera.longitude && converted.datum.equals(camera.datum), "only the height changes");
    check(VerticalDatum.heightNote(toGeoid).contains("geoid.txt") && VerticalDatum.datumName(toGeoid).contains("geoid.txt"), "report names the target grid");
    // Geoid -> ellipsoid: h = H + N.
    Options toEllipsoid = parse("--source-geoid", bin.toString());
    check(Math.abs(VerticalDatum.convert(camera, toEllipsoid).altitude - (150 + n)) < 1e-4, "geoid -> ellipsoid");
    // Same surface both sides: no shift; a round trip returns the start.
    Options same = parse("--source-geoid", txt.toString(), "--target-geoid", bin.toString());
    check(Math.abs(VerticalDatum.convert(camera, same).altitude - 150) < 1e-4, "same geoid both sides: no shift");
    check(Math.abs(VerticalDatum.convert(VerticalDatum.convert(camera, toGeoid), toEllipsoid).altitude - 150) < 1e-4, "round trip");
    // Two different geoids: shift = N_source - N_target.
    StringBuilder other = new StringBuilder();
    for (int i = 0; i < rows; i++)
      for (int j = 0; j < cols; j++)
        other.append(String.format(Locale.ROOT, "%.4f %.4f %.6f%n", 60 + .5 * i, 20 + .5 * j, 30.0));
    Path flat = dir.resolve("flat30.txt");
    Files.writeString(flat, other);
    Options between = parse("--source-geoid", flat.toString(), "--target-geoid", txt.toString());
    check(Math.abs(VerticalDatum.shift(61.2, 22.3, between) - (30 - n)) < 1e-5, "geoid A -> geoid B shift");
    // Missing GPS or a NaN height passes through.
    check(VerticalDatum.convert(null, toGeoid) == null, "no GPS stays null");
    Exif.Gps nan = new Exif.Gps(61, 22, Double.NaN);
    check(VerticalDatum.convert(nan, toGeoid) == nan, "NaN height passes through");

    // A camera outside the grid stops with a clear message; swapped columns are hinted.
    try {
      VerticalDatum.convert(new Exif.Gps(65, 25, 100), toGeoid);
      check(false, "camera outside the grid accepted");
    } catch (IllegalArgumentException e) {
      check(e.getMessage().contains("outside geoid grid") && e.getMessage().contains("--target-geoid"), e.getMessage());
    }
    try {
      // Grid spans lat 60..62 / lon 20..24: a camera at lat 21, lon 61 fits only with swapped columns.
      VerticalDatum.convert(new Exif.Gps(21, 61, 100), toGeoid);
      check(false, "swapped camera accepted");
    } catch (IllegalArgumentException e) {
      check(e.getMessage().contains("longitude before latitude"), "swap hint: " + e.getMessage());
    }

    compound(dir);
    suppliedExteriors(txt);
    return checks;
  }

  /** --crs EPSG:H+V: parsing, persistence form, and the vertical key in LAS and GeoTIFF. */
  static void compound(Path dir) throws Exception {
    Crs plain = new Crs("3067");
    check(plain.name.equals("EPSG:3067") && plain.vertical == 0 && plain.spec().equals("EPSG:3067"), "plain CRS unchanged");
    for (String v : List.of("EPSG:3067+3900", "3067+3900", "epsg:3067+EPSG:3900", " EPSG:3067 + 3900 ")) {
      Crs c = new Crs(v);
      check(c.name.equals("EPSG:3067") && c.epsg == 3067 && c.vertical == 3900, "compound " + v);
      check(c.spec().equals("EPSG:3067+3900") && new Crs(c.spec()).vertical == 3900, "spec round trip " + v);
    }
    check(Crs.verticalOf("EPSG:3067") == 0 && Crs.verticalOf("EPSG:3067+5773") == 5773 && Crs.verticalOf(null) == 0, "verticalOf");
    check(new Crs("EPSG:3067+3900").withVertical(0).spec().equals("EPSG:3067"), "withVertical(0)");
    check(Crs.verticalName(3900).contains("N2000") && Crs.verticalName(9999).equals("EPSG:9999"), "vertical names");
    check(refused(() -> new Crs("3067+3067")), "horizontal code after + refused");
    check(refused(() -> new Crs("3067+3900+5773")), "three parts refused");
    check(refused(() -> new Crs("3067+")), "empty vertical refused");
    check(parse("--crs", "EPSG:3067+3900").crs.equals("EPSG:3067+3900"), "--crs keeps the compound form");
    check(parse().crs.equals("EPSG:3067"), "default --crs unchanged");
    check(refused(() -> parse("--lidar-crs", "EPSG:3067+3900")), "--lidar-crs is horizontal only");

    // Projected <-> geographic round trip (used to place supplied exteriors on the geoid grid).
    Crs tm = new Crs("EPSG:3067");
    double[] xy = tm.project(new Exif.Gps(61.2, 22.3, 0));
    double[] back = tm.geographic(xy[0], xy[1]);
    check(Math.abs(back[0] - 61.2) < 1e-8 && Math.abs(back[1] - 22.3) < 1e-8, "geographic inverse " + Arrays.toString(back));

    // LAS: VerticalCSTypeGeoKey only when labelled; the horizontal-only file is as before.
    for (int v : new int[] {0, 3900}) {
      Path out = dir.resolve("las-" + v);
      try (CloudWriter w = new CloudWriter(out, 1, new Crs(v > 0 ? "EPSG:3067+" + v : "EPSG:3067"))) {
        for (int i = 0; i < 50; i++) w.point(400000 + i, 6800000 + i, 100 + i * .1, 10, 20, 30);
      }
      List<Path> files = PointFieldsRegression.lasFiles(out);
      check(!files.isEmpty(), "LAS written");
      LasFile.Header h = LasFile.header(files.get(0));
      check(h.crs().horizontal() == 3067, "LAS horizontal key " + h.crs());
      check(h.crs().vertical() == v, "LAS vertical key " + h.crs().vertical() + " for " + v);
      byte[] bytes = Files.readAllBytes(files.get(0));
      check(new String(bytes, java.nio.charset.StandardCharsets.ISO_8859_1).contains(v > 0 ? "Projected and vertical CRS" : "Horizontal projected CRS"), "LAS projection record description");
      if (v > 0) {
        List<Path> ply = new ArrayList<>();
        try (var s = Files.walk(out)) { s.filter(p -> p.toString().endsWith(".ply")).forEach(ply::add); }
        for (Path p : ply)
          check(new String(Files.readAllBytes(p), 0, 600, java.nio.charset.StandardCharsets.ISO_8859_1).contains("Z=EPSG:3900 (N2000 height)"), "PLY header names the height system");
      }
    }

    // GeoTIFF DSM/DTM: key 4096 present only when labelled.
    float[] z = new float[16];
    Arrays.fill(z, 50f);
    Path plainTif = dir.resolve("plain.tif"), labelledTif = dir.resolve("labelled.tif");
    DsmRaster.writeTiff(plainTif, 4, 4, z, 400000, 6800000, 1, 3067);
    DsmRaster.writeTiff(labelledTif, 4, 4, z, 400000, 6800000, 1, 3067, 3900);
    check(!hasKey(Files.readAllBytes(plainTif), 4096, -1), "no vertical key by default");
    check(hasKey(Files.readAllBytes(labelledTif), 4096, 3900), "GeoTIFF VerticalCSTypeGeoKey 3900");
    check(hasKey(Files.readAllBytes(labelledTif), 4099, 9001), "GeoTIFF vertical units metre");
    check(hasKey(Files.readAllBytes(labelledTif), 3072, 3067), "GeoTIFF still has the projected key");
    DsmRaster.writeTiff(dir.resolve("plain2.tif"), 4, 4, z, 400000, 6800000, 1, 3067, 0);
    check(Arrays.equals(Files.readAllBytes(plainTif), Files.readAllBytes(dir.resolve("plain2.tif"))), "vertical 0 writes the same bytes");
    check(DsmRaster.vertical(Map.of("vertical_crs", "EPSG:3900")) == 3900 && DsmRaster.vertical(Map.of()) == 0, "vertical from coordinates.json");
  }

  /** GeoKey entry (id, 0, 1, value) in a little-endian TIFF; value -1 matches any. */
  static boolean hasKey(byte[] tiff, int id, int value) {
    ByteBuffer b = ByteBuffer.wrap(tiff).order(ByteOrder.LITTLE_ENDIAN);
    for (int i = 0; i + 8 <= tiff.length; i += 2)
      if ((b.getShort(i) & 0xffff) == id && b.getShort(i + 2) == 0 && b.getShort(i + 4) == 1
          && (value < 0 || (b.getShort(i + 6) & 0xffff) == value))
        return true;
    return false;
  }

  /** Projected supplied exteriors move by the datum shift; rotation and plan position stay. */
  static void suppliedExteriors(Path grid) throws Exception {
    Crs tm = new Crs("EPSG:3067");
    double[] xy = tm.project(new Exif.Gps(61.2, 22.3, 0));
    double angle = Math.toRadians(30);
    // Tilted camera: rotation about x, so the shift must use the rotation's third column.
    double[] r = {1, 0, 0, 0, Math.cos(angle), -Math.sin(angle), 0, Math.sin(angle), Math.cos(angle)};
    double[] centre = {xy[0], xy[1], 350};
    Camera c = new Camera(1000, 1000, 500, 500);
    c.rotation = r.clone();
    c.translation = Math3.times(Math3.mv(r, centre), -1);
    Options none = parse("--orientation-crs", "EPSG:3067");
    Camera untouched = c.copy();
    VerticalDatum.shiftPose(untouched, none);
    check(Arrays.equals(untouched.translation, c.translation), "no options: supplied pose unchanged");
    Options local = parse("--target-geoid", grid.toString());
    check(!VerticalDatum.convertsPoses(local), "local exteriors (no --orientation-crs) are not shifted");
    Options o = parse("--orientation-crs", "EPSG:3067", "--target-geoid", grid.toString());
    check(VerticalDatum.convertsPoses(o), "projected exteriors with a geoid option are shifted");
    Camera moved = c.copy();
    VerticalDatum.shiftPose(moved, o);
    double[] after = moved.center();
    double n = surface(61.2, 22.3);
    check(Math.abs(after[0] - centre[0]) < 1e-6 && Math.abs(after[1] - centre[1]) < 1e-6, "plan position kept " + Arrays.toString(after));
    check(Math.abs(after[2] - (350 - n)) < 1e-5, "height moved by -N: " + after[2]);
    check(Arrays.equals(moved.rotation, c.rotation), "rotation kept");
  }
}
