package sgm;

import java.io.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.security.MessageDigest;
import java.util.*;

/**
 * Lidar-referenced georeferencing (--lidar-reference, step 'lidar'): the LAS reader on synthetic
 * files of every point format and version, the LAZ decoder against LAS twins written by reference
 * tools (src/test/resources/lidar) and the NLS example tile when present, the CRS rules, the
 * coverage check, recovery of a known rigid transformation from synthetic tie points, the
 * fallback for weakly determined parameters, the report helpers, the options, and the step on a
 * real project (forest_tiny with its Metashape cloud as the reference).
 */
final class LidarRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Path root = Files.createTempDirectory(Path.of("target"), "lidar-");
    System.out.println("PASS lidar reference: " + run(root) + "; " + root);
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Files.createDirectories(root);
    lasFormats(root.resolve("formats"));
    lazPairs();
    nlsTile();
    crsRules(root.resolve("crs"));
    transformMath();
    registration(root.resolve("registration"));
    oldLidar(root.resolve("old-lidar"));
    flatTerrain(root.resolve("flat"));
    coverage(root.resolve("coverage"));
    reportHelpers(root.resolve("report"));
    options(root);
    realProject(root.resolve("real"));
    return checks;
  }

  // ---- A small LAS writer for synthetic files.

  record Pt(double x, double y, double z, int cls) {}

  /** Writes LAS major.minor, the given point format, GeoTIFF keys (epsg, vertical) or a WKT (VLR or EVLR). */
  static void writeLas(Path file, int minor, int format, List<Pt> pts, int epsg, int vertical, String wkt, boolean wktInEvlr, int extra) throws IOException {
    int headerSize = minor >= 4 ? 375 : minor == 3 ? 235 : 227;
    int recordLength = LasFile.BASE[format] + extra;
    ByteArrayOutputStream vlrs = new ByteArrayOutputStream();
    int nvlr = 0;
    if (epsg > 0) {
      int[] keys = vertical > 0 ? new int[] {1, 1, 0, 3, 1024, 0, 1, 1, 3072, 0, 1, epsg, 4096, 0, 1, vertical} : new int[] {1, 1, 0, 2, 1024, 0, 1, 1, 3072, 0, 1, epsg};
      ByteBuffer b = ByteBuffer.allocate(keys.length * 2).order(ByteOrder.LITTLE_ENDIAN);
      for (int k : keys) b.putShort((short) k);
      vlr(vlrs, "LASF_Projection", 34735, b.array());
      nvlr++;
    }
    if (wkt != null && !wktInEvlr) {
      vlr(vlrs, "LASF_Projection", 2112, (wkt + "\0").getBytes(StandardCharsets.US_ASCII));
      nvlr++;
    }
    long dataOffset = headerSize + vlrs.size();
    ByteBuffer h = ByteBuffer.allocate(headerSize).order(ByteOrder.LITTLE_ENDIAN);
    h.put("LASF".getBytes(StandardCharsets.US_ASCII));
    h.putShort(6, (short) (wkt != null ? 16 : 0));
    h.put(24, (byte) 1).put(25, (byte) minor);
    byte[] soft = "LidarRegression".getBytes(StandardCharsets.US_ASCII);
    for (int i = 0; i < soft.length; i++) h.put(58 + i, soft[i]);
    h.putShort(94, (short) headerSize).putInt(96, (int) dataOffset).putInt(100, nvlr);
    h.put(104, (byte) format).putShort(105, (short) recordLength).putInt(107, minor >= 4 && format >= 6 ? 0 : pts.size());
    double[] min = {1e18, 1e18, 1e18}, max = {-1e18, -1e18, -1e18};
    for (Pt p : pts) {
      double[] v = {p.x, p.y, p.z};
      for (int k = 0; k < 3; k++) {
        min[k] = Math.min(min[k], v[k]);
        max[k] = Math.max(max[k], v[k]);
      }
    }
    double[] scale = {.01, .01, .01}, offset = {Math.floor(min[0]), Math.floor(min[1]), 0};
    for (int k = 0; k < 3; k++) {
      h.putDouble(131 + 8 * k, scale[k]);
      h.putDouble(155 + 8 * k, offset[k]);
    }
    h.putDouble(179, max[0]).putDouble(187, min[0]).putDouble(195, max[1]).putDouble(203, min[1]).putDouble(211, max[2]).putDouble(219, min[2]);
    long evlrStart = dataOffset + (long) pts.size() * recordLength;
    if (minor >= 4) {
      h.putLong(235, wkt != null && wktInEvlr ? evlrStart : 0).putInt(243, wkt != null && wktInEvlr ? 1 : 0).putLong(247, pts.size());
    }
    try (OutputStream out = new BufferedOutputStream(Files.newOutputStream(file))) {
      out.write(h.array());
      vlrs.writeTo(out);
      ByteBuffer r = ByteBuffer.allocate(recordLength).order(ByteOrder.LITTLE_ENDIAN);
      for (int i = 0; i < pts.size(); i++) {
        Pt p = pts.get(i);
        Arrays.fill(r.array(), (byte) 0);
        r.putInt(0, (int) Math.round((p.x - offset[0]) / scale[0])).putInt(4, (int) Math.round((p.y - offset[1]) / scale[1]))
            .putInt(8, (int) Math.round((p.z - offset[2]) / scale[2])).putShort(12, (short) i);
        boolean withheld = i % 97 == 5;
        if (format < 6) {
          r.put(14, (byte) (1 | 2 << 3)).put(15, (byte) (p.cls | (withheld ? 128 : 0)));
          if (format == 1 || format >= 3) r.putDouble(20, i * .5);
          int c = LasFile.colourOffset(format);
          if (c >= 0) r.putShort(c, (short) (i * 7)).putShort(c + 2, (short) 1000).putShort(c + 4, (short) 2000);
        } else {
          r.put(14, (byte) (1 | 2 << 4)).put(15, (byte) (withheld ? 4 : 0)).put(16, (byte) p.cls).putShort(18, (short) -300);
          r.putDouble(22, i * .5);
          int c = LasFile.colourOffset(format);
          if (c >= 0) r.putShort(c, (short) (i * 7)).putShort(c + 2, (short) 1000).putShort(c + 4, (short) 2000);
        }
        for (int e = 0; e < extra; e++) r.put(LasFile.BASE[format] + e, (byte) (i + e));
        out.write(r.array());
      }
      if (minor >= 4 && wkt != null && wktInEvlr) {
        byte[] text = (wkt + "\0").getBytes(StandardCharsets.US_ASCII);
        ByteBuffer e = ByteBuffer.allocate(60).order(ByteOrder.LITTLE_ENDIAN);
        byte[] user = "LASF_Projection".getBytes(StandardCharsets.US_ASCII);
        for (int i = 0; i < user.length; i++) e.put(2 + i, user[i]);
        e.putShort(18, (short) 2112).putLong(20, text.length);
        out.write(e.array());
        out.write(text);
      }
    }
  }

  private static void vlr(ByteArrayOutputStream out, String user, int record, byte[] body) {
    ByteBuffer v = ByteBuffer.allocate(54).order(ByteOrder.LITTLE_ENDIAN);
    byte[] u = user.getBytes(StandardCharsets.US_ASCII);
    for (int i = 0; i < u.length; i++) v.put(2 + i, u[i]);
    v.putShort(18, (short) record).putShort(20, (short) body.length);
    out.writeBytes(v.array());
    out.writeBytes(body);
  }

  static List<Pt> grid(double x0, double y0, int n, double step) {
    List<Pt> out = new ArrayList<>();
    for (int i = 0; i < n; i++) out.add(new Pt(x0 + (i % 50) * step, y0 + (i / 50) * step, 100 + i * .01, i % 3 == 0 ? 2 : 5));
    return out;
  }

  /** Every point format 0-10 in LAS 1.2-1.4: coordinates, class, returns, withheld, colour, extra bytes. */
  static void lasFormats(Path dir) throws IOException {
    Files.createDirectories(dir);
    List<Pt> pts = grid(605000, 6937000, 1234, .5);
    for (int format = 0; format <= 10; format++)
      for (int minor : format >= 6 ? new int[] {4} : new int[] {2, 3, 4}) {
        Path f = dir.resolve("f" + format + "_1" + minor + ".las");
        writeLas(f, minor, format, pts, 3067, 0, null, false, format == 3 ? 5 : 0);
        LasFile.Header h = LasFile.header(f);
        check(h.format() == format && h.count() == pts.size() && h.version().equals("1." + minor) && !h.compressed(), "header of format " + format + " 1." + minor);
        check(h.crs().horizontal() == 3067 && h.crs().source().equals("GeoTIFF keys"), "EPSG from the GeoTIFF keys");
        try (LasFile.Reader r = LasFile.open(f)) {
          byte[] b = new byte[h.recordLength() * 100];
          int i = 0, bad = 0;
          for (int k; (k = r.read(b, 100)) > 0; )
            for (int j = 0; j < k; j++, i++) {
              int o = j * h.recordLength();
              Pt p = pts.get(i);
              if (Math.abs(h.x(LasFile.rawX(b, o)) - p.x) > .006 || Math.abs(h.y(LasFile.rawY(b, o)) - p.y) > .006 || Math.abs(h.z(LasFile.rawZ(b, o)) - p.z) > .006) bad++;
              if (LasFile.classification(format, b, o) != p.cls || LasFile.returnNumber(format, b, o) != 1 || LasFile.returnCount(format, b, o) != 2) bad++;
              if (LasFile.withheld(format, b, o) != (i % 97 == 5)) bad++;
              int c = LasFile.colourOffset(format);
              if (c >= 0 && LazDecoder.u16(b, o + c) != ((i * 7) & 0xffff)) bad++;
              if (format == 3 && (b[o + 34 + 2] & 255) != ((i + 2) & 255)) bad++;
            }
          check(i == pts.size() && bad == 0, "records of format " + format + " 1." + minor + ": " + bad + " wrong of " + i);
        }
      }
    // CRS from a WKT VLR (compound with a vertical datum) and from a WKT EVLR.
    String wkt = "COMPD_CS[\"ETRS89 / TM35FIN(E,N) + N2000 height\",PROJCS[\"ETRS89 / TM35FIN(E,N)\",GEOGCS[\"ETRS89\",DATUM[\"European_Terrestrial_Reference_System_1989\","
        + "SPHEROID[\"GRS 1980\",6378137,298.257222101,AUTHORITY[\"EPSG\",\"7019\"]],AUTHORITY[\"EPSG\",\"6258\"]],AUTHORITY[\"EPSG\",\"4258\"]],"
        + "PROJECTION[\"Transverse_Mercator\"],UNIT[\"metre\",1,AUTHORITY[\"EPSG\",\"9001\"]],AUTHORITY[\"EPSG\",\"3067\"]],"
        + "VERT_CS[\"N2000 height\",VERT_DATUM[\"Finnish Height System N2000\",2005,AUTHORITY[\"EPSG\",\"1030\"]],AUTHORITY[\"EPSG\",\"3900\"]]]";
    Path w = dir.resolve("wkt.las");
    writeLas(w, 4, 6, pts, 0, 0, wkt, false, 0);
    LasFile.CrsInfo crs = LasFile.header(w).crs();
    check(crs.horizontal() == 3067 && crs.vertical() == 3900 && crs.source().equals("WKT VLR"), "WKT VLR: projected and vertical EPSG, not the inner datum codes: " + crs);
    Path e = dir.resolve("evlr.las");
    writeLas(e, 4, 7, pts, 0, 0, wkt, true, 0);
    check(LasFile.header(e).crs().horizontal() == 3067 && LasFile.header(e).crs().source().equals("WKT EVLR"), "WKT EVLR");
    check(LasFile.wktCodes("PROJCRS[\"x\",BASEGEOGCRS[\"y\",ID[\"EPSG\",4258]],ID[\"EPSG\",3067]]")[0] == 3067, "WKT2 ID");
    check(LasFile.wktCodes("GEOGCS[\"WGS 84\",AUTHORITY[\"EPSG\",\"4326\"]]")[0] == 0, "geographic WKT has no projected code");
    Path keys = dir.resolve("keys.las");
    writeLas(keys, 2, 1, pts, 3067, 3900, null, false, 0);
    check(LasFile.header(keys).crs().vertical() == 3900 && LasFile.verticalName(3900).contains("N2000"), "vertical GeoTIFF key");
    // Refusals.
    Path bad = dir.resolve("bad.las");
    Files.write(bad, new byte[400]);
    check(refused(() -> LasFile.header(bad), "Not a LAS"), "not a LAS file");
    byte[] lazNoVlr = Files.readAllBytes(dir.resolve("f1_12.las"));
    lazNoVlr[104] = (byte) (1 | 128);
    Path nv = dir.resolve("novlr.laz");
    Files.write(nv, lazNoVlr);
    check(refused(() -> LasFile.header(nv), "laszip VLR"), "compressed without its VLR");
    byte[] truncated = Arrays.copyOf(Files.readAllBytes(dir.resolve("f0_12.las")), 2000);
    Path tr = dir.resolve("truncated.las");
    Files.write(tr, truncated);
    check(refused(() -> LasFile.open(tr).close(), "truncated"), "truncated LAS");
    check(LasFile.files(dir).size() >= 20 && LasFile.files(dir).stream().allMatch(LasFile::isPointFile), "folder listing of .las/.laz");
    check(refused(() -> LasFile.files(dir.resolve("none")), "not found"), "missing reference");
  }

  interface Action {
    void run() throws Exception;
  }

  static boolean refused(Action a, String text) {
    try {
      a.run();
      return false;
    } catch (Exception e) {
      return String.valueOf(e.getMessage()).contains(text);
    }
  }

  static String hash(Path p) throws Exception {
    try (LasFile.Reader r = LasFile.open(p)) {
      MessageDigest md = MessageDigest.getInstance("SHA-256");
      int rl = r.header().recordLength();
      byte[] b = new byte[rl * 4096];
      for (int k; (k = r.read(b, 4096)) > 0; ) md.update(b, 0, k * rl);
      return HexFormat.of().formatHex(md.digest());
    }
  }

  /** LAZ (point formats 0, 1, 2, 3 + extra bytes; 6 layered; 7 COPC) against LAS written by reference tools. */
  static void lazPairs() throws Exception {
    Path d = Path.of("src", "test", "resources", "lidar");
    if (!Files.isDirectory(d)) {
      System.out.println("LidarRegression: " + d.toAbsolutePath() + " missing; LAZ pair checks skipped");
      return;
    }
    for (String name : new String[] {"point10", "point-color", "point-time", "point-time-color", "extra-bytes", "1_4_w_evlr"}) {
      LasFile.Header h = LasFile.header(d.resolve(name + ".laz"));
      check(h.compressed() && h.laz() != null, name + " is compressed");
      check(hash(d.resolve(name + ".laz")).equals(hash(d.resolve(name + ".las"))), name + ": LAZ records equal the LAS twin byte for byte");
    }
    check(LasFile.header(d.resolve("1_4_w_evlr.laz")).format() == 6 && LasFile.header(d.resolve("1_4_w_evlr.laz")).laz().compressor() == 3, "layered POINT14 file");
    // COPC (format 7, RGB14, variable chunks, octree order): same points as simple.las in another order.
    check(rows(d.resolve("simple.copc.laz")).equals(rows(d.resolve("simple.las"))), "COPC points equal simple.las as a set");
    check(refused(() -> LasFile.header(d.resolve("point-version-1-point-wise.laz")), "version 1 is not supported"), "LASzip item version 1 refused with advice");
  }

  static List<String> rows(Path p) throws Exception {
    List<String> out = new ArrayList<>();
    try (LasFile.Reader r = LasFile.open(p)) {
      LasFile.Header h = r.header();
      int rl = h.recordLength(), f = h.format(), c = LasFile.colourOffset(f);
      byte[] b = new byte[rl * 1024];
      for (int k; (k = r.read(b, 1024)) > 0; )
        for (int i = 0; i < k; i++) {
          int o = i * rl;
          out.add(String.format(Locale.ROOT, "%.2f %.2f %.2f %d %d %d", h.x(LasFile.rawX(b, o)), h.y(LasFile.rawY(b, o)), h.z(LasFile.rawZ(b, o)),
              LazDecoder.u16(b, o + 12), LasFile.classification(f, b, o), c < 0 ? -1 : LazDecoder.u16(b, o + c)));
        }
    }
    Collections.sort(out);
    return out;
  }

  /** The NLS example tile (LAZ, format 1, 6.4 M points) decodes to the known records when present. */
  static void nlsTile() throws Exception {
    Path f = Path.of("..", "example_data", "forest", "forest_9", "lidar", "N5421D3.laz");
    if (!Files.isRegularFile(f)) {
      System.out.println("LidarRegression: " + f.toAbsolutePath() + " missing; NLS tile check skipped");
      return;
    }
    LasFile.Header h = LasFile.header(f);
    long[] byReturn = new long[8];
    double minZ = 1e9, maxZ = -1e9;
    MessageDigest md = MessageDigest.getInstance("SHA-256");
    long n = 0;
    try (LasFile.Reader r = LasFile.open(f)) {
      byte[] b = new byte[h.recordLength() * 65536];
      for (int k; (k = r.read(b, 65536)) > 0; n += k) {
        md.update(b, 0, k * h.recordLength());
        for (int i = 0; i < k; i++) {
          int o = i * h.recordLength();
          byReturn[LasFile.returnNumber(1, b, o)]++;
          double z = h.z(LasFile.rawZ(b, o));
          minZ = Math.min(minZ, z);
          maxZ = Math.max(maxZ, z);
        }
      }
    }
    // The header's "points by return" (LAStools) and extent agree with the decoded records.
    ByteBuffer head = ByteBuffer.wrap(Files.readAllBytes(f).length > 0 ? readHead(f) : new byte[0]).order(ByteOrder.LITTLE_ENDIAN);
    boolean returns = true;
    for (int r = 1; r <= 5; r++) returns &= (head.getInt(111 + 4 * (r - 1)) & 0xffffffffL) == byReturn[r];
    check(n == 6_447_049 && returns, "NLS tile: point count and points by return as in the header");
    check(Math.abs(minZ - h.min()[2]) < .006 && Math.abs(maxZ - h.max()[2]) < .006, "NLS tile: height range as in the header");
    check(HexFormat.of().formatHex(md.digest()).equals("c958e1a7ef964624678b427850debfc83eef960a0011c5a6da71b50c8978ade4"), "NLS tile: decoded records unchanged");
  }

  private static byte[] readHead(Path f) throws IOException {
    try (InputStream in = Files.newInputStream(f)) {
      return in.readNBytes(227);
    }
  }

  /** CRS: other EPSG refused, WKT without EPSG refused unless --lidar-crs, undeclared taken as --crs with a warning. */
  static void crsRules(Path dir) throws Exception {
    Files.createDirectories(dir);
    List<Pt> pts = grid(605000, 6937000, 200, 1);
    Crs run = new Crs("EPSG:3067");
    Path other = dir.resolve("other.las");
    writeLas(other, 2, 1, pts, 3879, 0, null, false, 0);
    LidarReference.Settings plain = settings(other);
    check(refused(() -> LidarReference.sources(other, run, plain), "is in EPSG:3879 but the run is in EPSG:3067"), "different EPSG refused");
    Path foot = dir.resolve("foot.las");
    writeLas(foot, 4, 6, pts, 0, 0, "PROJCS[\"Lambert\",GEOGCS[\"NAD83\",AUTHORITY[\"EPSG\",\"4269\"]],UNIT[\"foot\",0.3048]]", false, 0);
    check(refused(() -> LidarReference.sources(foot, run, settings(foot)), "--lidar-crs"), "WKT without EPSG refused with the --lidar-crs advice");
    LidarReference.Source declared = LidarReference.sources(foot, run, settings(foot, "--lidar-crs", "EPSG:3067"));
    check(declared.crsNote().contains("EPSG:3067"), "--lidar-crs accepted");
    Path none = dir.resolve("none.las");
    writeLas(none, 2, 1, pts, 0, 0, null, false, 0);
    LidarReference.Source taken = LidarReference.sources(none, run, settings(none));
    check(taken.crsNote().contains("taken to be EPSG:3067") && taken.warnings().stream().anyMatch(w -> w.contains("declare no coordinate system")), "undeclared: --crs with a warning");
    check(taken.verticalSource().equals("not declared") && taken.vertical().contains("N2000"), "undeclared heights named with the NLS hint");
    check(LidarReference.sources(none, run, settings(none, "--lidar-vertical-datum", "N2000")).vertical().equals("N2000"), "--lidar-vertical-datum");
    Path vertical = dir.resolve("vertical.las");
    writeLas(vertical, 2, 1, pts, 3067, 3900, null, false, 0);
    check(LidarReference.sources(vertical, run, settings(vertical)).vertical().contains("EPSG:3900"), "declared vertical datum");
    check(refused(() -> LidarReference.sources(vertical, run, settings(vertical, "--lidar-crs", "EPSG:3879")), "differs from the run"), "--lidar-crs must equal --crs");
  }

  static LidarReference.Settings settings(Path ref, String... extra) {
    List<String> a = new ArrayList<>(List.of("-i", ".", "--lidar-reference", ref.toString()));
    a.addAll(List.of(extra));
    return LidarReference.Settings.of(Options.parse(a.toArray(String[]::new)));
  }

  /** Increments compose; the similarity form equals the centred form; cameras keep their image coordinates. */
  static void transformMath() {
    LidarReference.Transform t = new LidarReference.Transform(new double[] {605000, 6937000, 100});
    t.update(new double[] {1, -2, 30, Math.toRadians(.3), Math.toRadians(-.5), Math.toRadians(.8), 0});
    t.update(new double[] {.1, .1, -.5, 0, 0, Math.toRadians(.1), 2e-4});
    double[] v = t.values();
    check(Math.abs(Math.toDegrees(v[5]) - .9) < .01 && Math.abs(v[6] - 2e-4) < 1e-9, "heading and scale compose: " + Arrays.toString(v));
    Georeference.Similarity s = t.similarity();
    double[] p = {605123.4, 6937456.7, 112.3}, a = t.apply(p[0], p[1], p[2]), b = s.apply(p);
    check(Math3.norm(Math3.sub(a, b)) < 1e-6, "similarity equals the centred transformation");
    // Camera update (as in Pipeline/LidarReference.run): x_cam' = s * x_cam for every point.
    double[] rc = LidarReference.rodrigues(.1, -.2, .3), tc = {5, -3, 40};
    double[] r2 = Math3.mul(rc, Math3.transpose(s.rotation)), t2 = Math3.sub(Math3.times(tc, s.scale), Math3.mv(r2, s.translation));
    double[] before = Math3.add(Math3.mv(rc, p), tc), after = Math3.add(Math3.mv(r2, b), t2);
    check(Math3.norm(Math3.sub(Math3.times(before, s.scale), after)) < 1e-6, "camera frame coordinates scale only: reprojection unchanged");
    double[] ang = new LidarReference.Transform(new double[3]).angles();
    check(ang[0] == 0 && ang[1] == 0 && ang[2] == 0, "identity angles");
  }

  // ---- Synthetic forest: terrain, cone crowns, ALS points and photogrammetric tie points.

  static final double W = 400, X0 = 605000, Y0 = 6937000;

  static double terrain(double x, double y) {
    return 100 + 8 * Math.sin(x / 40.0) + 6 * Math.cos(y / 55.0) + .03 * x + 3 * Math.sin((x + 2 * y) / 23.0);
  }

  record Tree(double x, double y, double r, double h) {}

  static List<Tree> trees(long seed, double cover) {
    Random rnd = new Random(seed);
    List<Tree> out = new ArrayList<>();
    int n = (int) (cover * W * W / (Math.PI * 7));
    for (int i = 0; i < n; i++) out.add(new Tree(rnd.nextDouble() * W, rnd.nextDouble() * W, 1.5 + rnd.nextDouble() * 1.5, 10 + rnd.nextDouble() * 12));
    return out;
  }

  /** Crown height above terrain at (x, y), 0 outside crowns (local coordinates). */
  static double crown(List<Tree> trees, Map<Long, List<Tree>> index, double x, double y) {
    double best = 0;
    long key = (long) Math.floor(x / 10) * 100000 + (long) Math.floor(y / 10);
    for (long dx = -1; dx <= 1; dx++)
      for (long dy = -1; dy <= 1; dy++)
        for (Tree t : index.getOrDefault(key + dx * 100000 + dy, List.of())) {
          double d = Math.hypot(x - t.x, y - t.y);
          if (d < t.r) best = Math.max(best, t.h * (1 - .35 * d / t.r));
        }
    return best;
  }

  static Map<Long, List<Tree>> index(List<Tree> trees) {
    Map<Long, List<Tree>> m = new HashMap<>();
    for (Tree t : trees) m.computeIfAbsent((long) Math.floor(t.x / 10) * 100000 + (long) Math.floor(t.y / 10), k -> new ArrayList<>()).add(t);
    return m;
  }

  /** ALS tile over [0, W]^2 (local), about 1 point per m2; points inside {@code hole} (cx, cy, r) dropped (water). */
  static Path als(Path file, List<Tree> trees, double flat, double[] hole, double xMax) throws IOException {
    Random rnd = new Random(7);
    Map<Long, List<Tree>> idx = index(trees);
    List<Pt> pts = new ArrayList<>();
    for (int i = 0; i < W * xMax; i++) {
      double x = rnd.nextDouble() * xMax, y = rnd.nextDouble() * W;
      if (hole != null && Math.hypot(x - hole[0], y - hole[1]) < hole[2]) continue;
      double g = flat > 0 ? flat + .002 * x : terrain(x, y), c = crown(trees, idx, x, y);
      if (c > 0 && rnd.nextDouble() < .75) pts.add(new Pt(X0 + x, Y0 + y, g + c, 5));
      else pts.add(new Pt(X0 + x, Y0 + y, g + .02 * rnd.nextGaussian(), 2));
    }
    writeLas(file, 2, 1, pts, 3067, 0, null, false, 0);
    return file;
  }

  /** True surface tie points over [lo, hi]^2 (local): crowns and ground in gaps, noise and 5 % blunders. */
  static List<double[]> tiePoints(List<Tree> trees, double flat, double lo, double hi, int n, long seed) {
    Random rnd = new Random(seed);
    Map<Long, List<Tree>> idx = index(trees);
    List<double[]> out = new ArrayList<>();
    while (out.size() < n) {
      double x = lo + rnd.nextDouble() * (hi - lo), y = lo + rnd.nextDouble() * (hi - lo);
      double g = flat > 0 ? flat + .002 * x : terrain(x, y), c = crown(trees, idx, x, y);
      double z = (c > 0 && rnd.nextDouble() < .8 ? g + c - .3 * rnd.nextDouble() : g) + .1 * rnd.nextGaussian();
      if (rnd.nextDouble() < .05) z += 8 * (rnd.nextDouble() - .5);
      out.add(new double[] {X0 + x, Y0 + y, z});
    }
    return out;
  }

  /** The GPS-frame points: the inverse of X_true = c + R (X_gps - c) + t. */
  static List<double[]> toGps(List<double[]> truth, double[] c, double[] t, double rollDeg, double pitchDeg, double headingDeg) {
    double[] r = rotation(rollDeg, pitchDeg, headingDeg), rt = Math3.transpose(r);
    List<double[]> out = new ArrayList<>();
    for (double[] p : truth) {
      double[] d = Math3.mv(rt, new double[] {p[0] - c[0] - t[0], p[1] - c[1] - t[1], p[2] - c[2] - t[2]});
      out.add(new double[] {c[0] + d[0], c[1] + d[1], c[2] + d[2]});
    }
    return out;
  }

  static double[] rotation(double roll, double pitch, double heading) {
    double a = Math.toRadians(roll), b = Math.toRadians(pitch), g = Math.toRadians(heading);
    double[] rx = {1, 0, 0, 0, Math.cos(a), -Math.sin(a), 0, Math.sin(a), Math.cos(a)};
    double[] ry = {Math.cos(b), 0, Math.sin(b), 0, 1, 0, -Math.sin(b), 0, Math.cos(b)};
    double[] rz = {Math.cos(g), -Math.sin(g), 0, Math.sin(g), Math.cos(g), 0, 0, 0, 1};
    return Math3.mul(rz, Math3.mul(ry, rx));
  }

  static LidarReference.Outcome fit(Path las, List<double[]> gps, String... extra) throws Exception {
    LidarReference.Settings set = settings(las, extra);
    LidarReference.Points p = LidarReference.points(gps);
    LidarReference.Source src = LidarReference.sources(las, new Crs("EPSG:3067"), set);
    LidarReference.Area area = LidarReference.area(p, set.margin());
    LidarReference.Surfaces s = LidarReference.read(src, area, set.cell(), null);
    check(LidarReference.coverage(area, s, src, set.margin()).complete(), "synthetic block covered");
    return LidarReference.estimate(p, s, area, set, null);
  }

  /** A known rigid transformation is recovered from forest-like tie points; blunders are rejected. */
  static void registration(Path dir) throws Exception {
    Files.createDirectories(dir);
    List<Tree> trees = trees(3, .55);
    Path las = als(dir.resolve("als.las"), trees, 0, null, W);
    List<double[]> truth = tiePoints(trees, 0, 110, 290, 12000, 11);
    double[] c = {X0 + 200, Y0 + 200, 110}, t = {3.2, -2.1, 41.5};
    List<double[]> gps = toGps(truth, c, t, .4, -.6, .9);
    long t0 = System.nanoTime();
    LidarReference.Outcome o = fit(las, gps);
    LidarReference.Fit f = o.fit();
    double worst = 0;
    for (int i = 0; i < gps.size(); i += 7) {
      double[] q = f.transform.apply(gps.get(i)[0], gps.get(i)[1], gps.get(i)[2]), p = truth.get(i);
      worst = Math.max(worst, Math.max(Math.hypot(q[0] - p[0], q[1] - p[1]), Math.abs(q[2] - p[2])));
    }
    double[] v = f.transform.values();
    check(worst < .25, "known transformation recovered: largest error " + worst + " m; " + LidarReference.attemptText(f, 90));
    check(Math.abs(Math.toDegrees(v[3]) - .4) < .05 && Math.abs(Math.toDegrees(v[4]) + .6) < .05 && Math.abs(Math.toDegrees(v[5]) - .9) < .1,
        "roll, pitch, heading recovered: " + Arrays.toString(v));
    check(f.active[0] && f.active[5] && f.active[3] && !f.active[6], "all six parameters estimated, scale off by default: " + o.fixed());
    check(f.outliers > 0.02 * f.observations && f.search != null && !f.search.ambiguous(), "blunders rejected, coarse search clear (contrast " + f.search.contrast() + ")");
    for (int k = 0; k < 6; k++) check(f.sigma[k] > 0 && f.sigma[k] >= f.sigmaFormal[k], "standard deviation of " + LidarReference.PARAMETERS[k]);
    check(f.groups.size() >= 12, "jackknife groups recorded: " + f.groups.size());
    System.out.printf(Locale.ROOT, "LidarRegression: synthetic forest fit %.1f s, largest error %.3f m%n", (System.nanoTime() - t0) / 1e9, worst);
    // Scale on: a known scale is recovered too.
    List<double[]> scaled = new ArrayList<>();
    for (double[] p : gps) scaled.add(new double[] {c[0] + (p[0] - c[0]) / 1.002, c[1] + (p[1] - c[1]) / 1.002, c[2] + (p[2] - c[2]) / 1.002});
    LidarReference.Fit fs = fit(las, scaled, "--lidar-scale", "on").fit();
    check(fs.active[6] && Math.abs(fs.transform.values()[6] - .002) < 6e-4, "scale recovered: " + fs.transform.values()[6]);
    // Deformation measure: a bowl added to the tie points is reported, a rigid block is not.
    Map<String, Object> flat = LidarReference.deformation(LidarReference.points(gps), f, LidarReference.area(LidarReference.points(gps), 10), settings(las));
    check(((Number) flat.get("cell_median_range_metres")).doubleValue() < .3, "rigid block: little deformation " + flat.get("cell_median_range_metres"));
    List<double[]> bowl = new ArrayList<>();
    for (double[] p : gps) bowl.add(new double[] {p[0], p[1], p[2] + 1.2 * (Math.pow((p[0] - c[0]) / 90, 2) + Math.pow((p[1] - c[1]) / 90, 2))});
    LidarReference.Fit fb = fit(las, bowl).fit();
    Map<String, Object> bent = LidarReference.deformation(LidarReference.points(bowl), fb, LidarReference.area(LidarReference.points(bowl), 10), settings(las));
    check(((Number) bent.get("cell_median_range_metres")).doubleValue() > .6 && ((Number) bent.get("curvature_range_metres")).doubleValue() > .6,
        "bent block: deformation reported " + bent.get("cell_median_range_metres") + " / " + bent.get("curvature_range_metres"));
  }

  // ---- An old lidar: the forest years before the images.

  /** Stand-wise height growth since the lidar (local coordinates): 1 to 5 m in 100 m stands. */
  static double growth(double x, double y) {
    int stand = (int) Math.floor(x / 100) * 7 + (int) Math.floor(y / 100) * 3;
    return 1 + Math.floorMod(stand, 5);
  }

  static boolean felled(double x, double y) {
    return x > 220 && x < 280 && y > 120 && y < 200; // trees in the lidar, gone in the images
  }

  static boolean regrown(double x, double y) {
    return x > 120 && x < 180 && y > 220 && y < 280; // open in the lidar, 1.5-3 m saplings in the images
  }

  static boolean building(double x, double y) {
    return x > 150 && x < 170 && y > 140 && y < 156; // new 6 m building in the images
  }

  /** ALS of the forest as it was: lower trees, the felled stand still there, no saplings, no building. */
  static Path oldAls(Path file, List<Tree> trees) throws IOException {
    Random rnd = new Random(17);
    Map<Long, List<Tree>> idx = index(trees);
    List<Pt> pts = new ArrayList<>();
    for (int i = 0; i < W * W; i++) {
      double x = rnd.nextDouble() * W, y = rnd.nextDouble() * W;
      double g = terrain(x, y), c = regrown(x, y) ? 0 : crown(trees, idx, x, y);
      if (c > 0) c = Math.max(1, c - growth(x, y));
      if (c > 0 && rnd.nextDouble() < .75) pts.add(new Pt(X0 + x, Y0 + y, g + c, 5));
      else pts.add(new Pt(X0 + x, Y0 + y, g + .02 * rnd.nextGaussian(), 2));
    }
    writeLas(file, 2, 1, pts, 3067, 0, null, false, 0);
    return file;
  }

  /** Tie points of today's forest: grown crowns, ground where the stand was felled, saplings, the building. */
  static List<double[]> newTiePoints(List<Tree> trees, int n, long seed, double crownShare) {
    Random rnd = new Random(seed);
    Map<Long, List<Tree>> idx = index(trees);
    List<double[]> out = new ArrayList<>();
    while (out.size() < n) {
      double x = 110 + rnd.nextDouble() * 180, y = 110 + rnd.nextDouble() * 180;
      double g = terrain(x, y), c = felled(x, y) || regrown(x, y) ? 0 : crown(trees, idx, x, y), z;
      if (building(x, y)) z = g + 6;
      else if (regrown(x, y)) z = g + 1.5 + 1.5 * Math.abs(Math.sin(x * 1.3) * Math.cos(y * 1.7)) - (rnd.nextDouble() < .15 ? 1.5 : 0);
      else z = c > 0 && rnd.nextDouble() < crownShare ? g + c - .3 * rnd.nextDouble() : g;
      z += .1 * rnd.nextGaussian();
      if (rnd.nextDouble() < .05) z += 8 * (rnd.nextDouble() - .5);
      out.add(new double[] {X0 + x, Y0 + y, z});
    }
    return out;
  }


  /** {largest horizontal error, largest height error, mean height error} of the fit on 'old lidar' data. */
  static double[] oldLidarErrors(Path dir, double cover, double crownShare, long seed) throws Exception {
    Files.createDirectories(dir);
    List<Tree> trees = trees(seed, cover);
    Path las = oldAls(dir.resolve("old.las"), trees);
    List<double[]> truth = newTiePoints(trees, 12000, seed + 9, crownShare);
    double[] c = {X0 + 200, Y0 + 200, 110}, t = {3.2, -2.1, 41.5};
    List<double[]> gps = toGps(truth, c, t, .4, -.6, .9);
    LidarReference.Fit f = fit(las, gps).fit();
    double h = 0, v = 0, mean = 0;
    int n = 0;
    for (int i = 0; i < gps.size(); i += 7) {
      double[] q = f.transform.apply(gps.get(i)[0], gps.get(i)[1], gps.get(i)[2]), p = truth.get(i);
      h = Math.max(h, Math.hypot(q[0] - p[0], q[1] - p[1]));
      v = Math.max(v, Math.abs(q[2] - p[2]));
      mean += q[2] - p[2];
      n++;
    }
    System.out.println("LidarRegression: old lidar: " + LidarReference.attemptText(f, 90));
    return new double[] {h, v, mean / n};
  }

  /**
   * The lidar years older than the images (crowns 1-5 m lower by stand, a stand felled since, a
   * clear-cut regrown with saplings, a new building): the known transformation is still
   * recovered, also in a closed canopy where the images see no ground through the crowns.
   * Before the old-lidar handling (canopy height free per patch, photogrammetric ground test,
   * change detection on ground only, clipped correlation) the closed canopy failed by 7 m.
   */
  static void oldLidar(Path dir) throws Exception {
    for (double[] scene : new double[][] {{1.6, .8, 3}, {2.5, 1.0, 3}, {2.5, 1.0, 22}}) {
      double[] e = oldLidarErrors(dir.resolve("scene-" + scene[0] + "-" + scene[1] + "-" + (long) scene[2]), scene[0], scene[1], (long) scene[2]);
      check(e[0] < .3 && e[1] < .1, String.format(Locale.ROOT, "old lidar, cover %.1f, crowns %.0f%%, seed %d: largest errors %.3f m horizontal, %.3f m height",
          scene[0], 100 * scene[1], (long) scene[2], e[0], e[1]));
    }
  }

  /** Flat, featureless open terrain: the shifts and heading are weak and fixed at GPS, height and tilt estimated. */
  static void flatTerrain(Path dir) throws Exception {
    Files.createDirectories(dir);
    Path las = als(dir.resolve("flat.las"), List.of(), 80, null, W);
    List<double[]> truth = tiePoints(List.of(), 80, 120, 280, 6000, 5);
    double[] c = {X0 + 200, Y0 + 200, 80};
    List<double[]> gps = toGps(truth, c, new double[] {0, 0, 12}, .3, 0, 0);
    LidarReference.Outcome o = fit(las, gps);
    LidarReference.Fit f = o.fit();
    check(f.active[2] && !f.active[0] && !f.active[1] && !f.active[5], "flat terrain: horizontal shift and heading fixed: " + LidarReference.estimatedNames(f.active) + "; " + LidarReference.attemptText(f, 80) + "; search contrast " + f.search.contrast());
    check(o.fixed().stream().anyMatch(s -> s.startsWith("horizontal shift")), "fallback is explained (no distinct match, or drifted out of the search range): " + o.fixed());
    check(Math.abs(f.transform.values()[2] - 12) < .05 && Math.abs(Math.toDegrees(f.transform.values()[3]) - .3) < .03, "height and roll still recovered");
  }

  /** The tie-point area must be covered: a missing strip stops with where it is; water inside the tile is fine. */
  static void coverage(Path dir) throws Exception {
    Files.createDirectories(dir);
    List<Tree> trees = trees(4, .4);
    List<double[]> gps = tiePoints(trees, 0, 100, 300, 5000, 3);
    Path partial = als(dir.resolve("partial.las"), trees, 0, null, 250);
    LidarReference.Settings set = settings(partial);
    LidarReference.Points p = LidarReference.points(gps);
    LidarReference.Source src = LidarReference.sources(partial, new Crs("EPSG:3067"), set);
    LidarReference.Area area = LidarReference.area(p, set.margin());
    LidarReference.Coverage cov = LidarReference.coverage(area, LidarReference.read(src, area, set.cell(), null), src, set.margin());
    check(!cov.complete() && cov.message().contains("does not cover") && cov.where().contains("eastern") && cov.missingBox()[0] >= X0 + 249,
        "missing eastern strip found: " + cov.message());
    check(cov.missingM2() > 0.2 * cov.areaM2() && cov.missingM2() < 0.5 * cov.areaM2(), "missing share plausible: " + cov.missingM2() / cov.areaM2());
    Path lake = als(dir.resolve("lake.las"), trees, 0, new double[] {200, 200, 20}, W);
    LidarReference.Source s2 = LidarReference.sources(lake, new Crs("EPSG:3067"), settings(lake));
    LidarReference.Coverage c2 = LidarReference.coverage(area, LidarReference.read(s2, area, 2, null), s2, 10);
    check(c2.complete() && c2.gapM2() > 500 && c2.message().contains("no lidar returns"), "a lake inside the tile counts as covered, reported as a gap: " + c2.message());
    LidarReference.coverageImage(cov, area, dir.resolve("coverage.png"));
    check(Files.size(dir.resolve("coverage.png")) > 100, "coverage map written");
    // Two adjacent tiles in a folder cover the block together.
    Path tiles = dir.resolve("tiles");
    Files.createDirectories(tiles);
    Random rnd = new Random(2);
    List<Pt> west = new ArrayList<>(), east = new ArrayList<>();
    for (int i = 0; i < 160000; i++) {
      double x = rnd.nextDouble() * W, y = rnd.nextDouble() * W;
      (x < 200 ? west : east).add(new Pt(X0 + x, Y0 + y, terrain(x, y), 2));
    }
    writeLas(tiles.resolve("a.las"), 2, 1, west, 3067, 0, null, false, 0);
    writeLas(tiles.resolve("b.las"), 2, 1, east, 3067, 0, null, false, 0);
    LidarReference.Source s3 = LidarReference.sources(tiles, new Crs("EPSG:3067"), settings(tiles));
    LidarReference.Surfaces sf = LidarReference.read(s3, area, 2, null);
    check(s3.files().size() == 2 && sf.filesRead == 2 && LidarReference.coverage(area, sf, s3, 10).complete(), "a folder of tiles");
    check(refused(() -> LidarReference.area(LidarReference.points(gps.subList(0, 10)), 10), "Too few tie points"), "too few tie points refused");
  }

  /** Comparison statistics by kind, the median resampling, flags and the summary line. */
  static void reportHelpers(Path dir) throws Exception {
    Files.createDirectories(dir);
    float[] kinds = {1, 1, 4, 4, 3, 0}, diff = {.1f, .3f, -2, 1, .5f, 9};
    LidarComparison.Raster kind = new LidarComparison.Raster(3, 2, kinds, 0, 2, 1);
    Map<String, Object> s = LidarComparison.byKind(diff, kind, "test");
    check(((Number) JsonRead.object(s.get("open_ground")).get("points")).intValue() == 2 && ((Number) JsonRead.object(s.get("total")).get("points")).intValue() == 5,
        "statistics by kind, no-data cells left out");
    float[] fine = new float[16];
    for (int i = 0; i < 16; i++) fine[i] = i < 8 ? (i % 2 == 0 ? 1 : 5) : 3;
    LidarComparison.Raster photo = new LidarComparison.Raster(4, 4, fine, 0, 4, 1), coarse = new LidarComparison.Raster(2, 2, new float[4], 0, 4, 2);
    float[] med = LidarComparison.medianOnGrid(photo, coarse);
    check(med[0] == 5 && med[2] == 3, "median of the fine cells in each coarse cell: " + Arrays.toString(med));
    check(LasFile.gpsDate(0, 1).toString().equals("2011-09-14") && LasFile.gpsDate(0, 0) == null && LasFile.gpsTimeOffset(1) == 20 && LasFile.gpsTimeOffset(2) == -1,
        "lidar acquisition date from adjusted standard GPS time only");
    check(LidarComparison.diverging(0) == 0xf7f7f7 && LidarComparison.diverging(-1) == 0x2166ac && LidarComparison.diverging(5) == 0xb2182b, "diverging colours");
    Map<String, Object> ref = Json.object(
        "parameters", List.of(Json.object("name", "shift_up", "unit", "metres", "estimated", true, "value", 50.04),
            Json.object("name", "pitch", "unit", "degrees", "estimated", true, "value", 4.277), Json.object("name", "heading", "unit", "degrees", "estimated", false)),
        "residuals_after", Json.object("stable_within_limit", Json.object("rms_metres", .52)),
        "fallback", true,
        "warnings", List.of("heading: weakly determined (x); kept at the GPS value"),
        "coarse_search", Json.object("ambiguous", true, "contrast", 2.0),
        "deformation", Json.object("cell_median_range_metres", 1.9, "grid_cell_metres", 35.0),
        "lidar", Json.object("vertical_datum_source", "not declared"));
    String line = PdfReport.lidarLine(ref);
    check(line.contains("U +50.04 m") && line.contains("pitch +4.277 deg") && !line.contains("heading") && line.contains("fewer parameters"), "summary line: " + line);
    List<String> flags = PdfReport.lidarFlags(Json.object("reference", ref, "comparison", Json.object("dsm", Json.object("stable", Json.object("median_metres", .8, "points", 90)))));
    check(flags.size() == 6, "lidar flags: weak parameter, weak search, stable RMS, deformation, height offset, undeclared datum: " + flags);
    check(PdfReport.lidarFlags(Map.of()).isEmpty(), "no lidar: no flags");
  }

  static void options(Path root) throws IOException {
    Path file = root.resolve("o.las");
    writeLas(file, 2, 1, grid(605000, 6937000, 10, 1), 3067, 0, null, false, 0);
    Options d = Options.parse(new String[] {"-i", "."});
    check(d.lidarReference == null && !d.lidarScale && d.lidarSearchShift == 10 && d.lidarSearchAngle == 3 && d.lidarCell == 2 && d.lidarCanopyWeight == .1
        && d.lidarOutlier == 2 && d.lidarWeakLimit == .25 && d.lidarMargin == 10, "defaults");
    check(Pipeline.notRequested(Pipeline.Step.LIDAR, d) && Pipeline.optional(Pipeline.Step.LIDAR), "optional step, not requested by default");
    Options o = Options.parse(new String[] {"-i", ".", "--lidar-reference", file.toString(), "--lidar-scale", "on", "--lidar-search-shift", "4", "--lidar-crs", "3067"});
    check(!Pipeline.notRequested(Pipeline.Step.LIDAR, o) && o.lidarScale && o.lidarSearchShift == 4 && o.lidarCrs.equals("EPSG:3067"), "parsed");
    check(Pipeline.Step.LIDAR.ordinal() == Pipeline.Step.GEOREFERENCE.ordinal() + 1 && Pipeline.Step.GCP.ordinal() == Pipeline.Step.LIDAR.ordinal() + 1,
        "between georeference and gcp");
    for (String[] bad : new String[][] {{"--lidar-reference", root.resolve("missing.las").toString()}, {"--lidar-search-shift", "-1"}, {"--lidar-cell", "0.1"},
        {"--lidar-canopy-weight", "2"}, {"--lidar-weak-limit", "0"}, {"--lidar-scale", "maybe"}, {"--lidar-crs", "x"}})
      check(refused(() -> Options.parse(new String[] {"-i", ".", bad[0], bad[1]}), ""), "refused: " + String.join(" ", bad));
    check(GuiOptions.field("--lidar-reference") != null && GuiOptions.field("--lidar-reference").kind() == GuiOptions.Kind.FILE
        && GuiOptions.field("--lidar-reference").section().equals(GuiOptions.INPUT), "GUI field in Input and project");
    check(Options.help().contains("--lidar-reference FILE|FOLDER") && Options.help().contains("--lidar-weak-limit"), "help");
  }

  /**
   * forest_tiny: the steps up to georeference, then lidar with its Metashape cloud as the reference
   * (unclassified: crude terrain): applied, reprojection unchanged, coordinates.json in the lidar
   * datum; a reference that covers only part of the block stops the step with a message; without
   * the option the step is skipped.
   */
  static void realProject(Path dir) throws Exception {
    Path images = Path.of("..", "example_data", "forest", "forest_tiny");
    Path metashape = images.resolve("metashape.las");
    if (!Files.isRegularFile(metashape)) {
      System.out.println("LidarRegression: forest_tiny missing; real project skipped");
      return;
    }
    // The Metashape cloud covers exactly what the four images see, so the tie-point area reaches
    // past it; the reference is every 4th of its points plus a flat 2 m filler band 40 m wide around it.
    Files.createDirectories(dir);
    Path reference = dir.resolve("reference.las");
    Path project = dir.resolve("project");
    List<Pt> refPts = new ArrayList<>();
    LasFile.Header mh = LasFile.header(metashape);
    check(mh.crs().horizontal() == 3067 || mh.crs().horizontal() == 0, "metashape.las CRS " + mh.crs());
    try (LasFile.Reader r = LasFile.open(metashape)) {
      byte[] b = new byte[mh.recordLength() * 65536];
      long i = 0;
      for (int k; (k = r.read(b, 65536)) > 0; )
        for (int j = 0; j < k; j++, i++)
          if (i % 4 == 0) {
            int at = j * mh.recordLength();
            refPts.add(new Pt(mh.x(LasFile.rawX(b, at)), mh.y(LasFile.rawY(b, at)), mh.z(LasFile.rawZ(b, at)), 1));
          }
    }
    double fillZ = (mh.min()[2] + mh.max()[2]) / 2;
    for (double x = mh.min()[0] - 40; x <= mh.max()[0] + 40; x += 2)
      for (double y = mh.min()[1] - 40; y <= mh.max()[1] + 40; y += 2)
        if (x < mh.min()[0] || x > mh.max()[0] || y < mh.min()[1] || y > mh.max()[1]) refPts.add(new Pt(x, y, fillZ, 1));
    writeLas(reference, 2, 1, refPts, 3067, 0, null, false, 0);
    String o = project.toString();
    check(Main.run(new String[] {"inputs", "-i", images.toString(), "-o", o, "--quality", "low", "--cores", "4"}) == 0, "inputs");
    check(Main.run(new String[] {"calibrate", "match", "align", "georeference", "-o", o}) == 0, "steps up to georeference");
    String base = ProjectFiles.runIds(project).get(0);
    check(Main.run(new String[] {"lidar", "-o", o, "--run", base, "--branch", "--lidar-reference", reference.toString(), "--lidar-vertical-datum", "test heights"}) == 0, "lidar step");
    String lidarRun = ProjectFiles.runIds(project).get(ProjectFiles.runIds(project).size() - 1);
    Path run = project.resolve("runs").resolve(lidarRun);
    Map<String, Object> rep = JsonRead.object(JsonRead.read(run.resolve("04_geometry/lidar/lidar_reference.json")));
    check("applied".equals(rep.get("status")), "applied");
    var rms = JsonRead.object(rep.get("reprojection_rms_pixels"));
    check(Math.abs(((Number) rms.get("before")).doubleValue() - ((Number) rms.get("after")).doubleValue()) < 1e-6, "reprojection RMS unchanged");
    check(Boolean.FALSE.equals(JsonRead.object(rep.get("lidar")).get("ground_classified")), "unclassified reference handled");
    Map<String, Object> coords = JsonRead.object(JsonRead.read(run.resolve("coordinates.json")));
    check("lidar reference".equals(coords.get("datum")) && "test heights".equals(coords.get("vertical_datum")), "coordinates.json: lidar datum and heights");
    check(ProjectFiles.run(project, lidarRun).step("lidar").status().equals("complete") && ProjectFiles.run(project, base).step("lidar").status().equals("pending"),
        "the branch did the lidar step, the source run not");
    for (String f : List.of("als_dsm.tif", "als_dtm.tif", "als_kind.tif", "als_grid.json", "coverage.png", "residuals.csv"))
      check(Files.isRegularFile(run.resolve("04_geometry/lidar/" + f)), "written: " + f);
    // A reference covering only the western half of the block: stopped, with the reason.
    List<Pt> part = new ArrayList<>();
    double cut = ((List<?>) JsonRead.object(rep.get("area")).get("centre")).stream().mapToDouble(v -> ((Number) v).doubleValue()).findFirst().orElse(0);
    try (LasFile.Reader r = LasFile.open(reference)) {
      LasFile.Header h = r.header();
      byte[] b = new byte[h.recordLength() * 65536];
      long i = 0;
      for (int k; (k = r.read(b, 65536)) > 0; )
        for (int j = 0; j < k; j++, i++) {
          int at = j * h.recordLength();
          double x = h.x(LasFile.rawX(b, at));
          if (i % 20 == 0 && x < cut) part.add(new Pt(x, h.y(LasFile.rawY(b, at)), h.z(LasFile.rawZ(b, at)), 1));
        }
    }
    Path west = dir.resolve("west.las");
    writeLas(west, 2, 1, part, 3067, 0, null, false, 0);
    check(Main.run(new String[] {"lidar", "-o", o, "--run", base, "--branch", "--lidar-reference", west.toString()}) != 0, "partial reference stops the step");
    String failed = ProjectFiles.runIds(project).get(ProjectFiles.runIds(project).size() - 1);
    Map<String, Object> fr = JsonRead.object(JsonRead.read(project.resolve("runs").resolve(failed).resolve("04_geometry/lidar/lidar_reference.json")));
    check(String.valueOf(fr.get("status")).contains("incomplete lidar coverage")
        && String.valueOf(JsonRead.object(fr.get("coverage")).get("missing_where")).contains("east"), "the failure says what is missing: " + fr.get("coverage"));
    check(ProjectFiles.run(project, failed).step("lidar").status().equals("failed"), "step failed");
    // Without the option the step is skipped when a later step runs (gcp is optional too; dense is not run here).
    check(Main.run(new String[] {"gcp", "-o", o, "--run", base, "--branch"}) == 0
        && ProjectFiles.run(project, ProjectFiles.runIds(project).get(ProjectFiles.runIds(project).size() - 1)).step("lidar").status().equals("skipped"), "not asked for: skipped");
  }
}
