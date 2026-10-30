package sgm;

import java.lang.foreign.*;
import java.nio.*;
import java.nio.file.*;
import java.util.*;

final class FollowupRegression {
  private static int checks;

  private static void check(boolean yes, String message) {
    checks++;
    if (!yes) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    Path root = Files.createTempDirectory(Path.of("target"), "followup checks ");
    System.out.println("PASS followup checks=" + run(root) + "; fixtures=" + root);
    GpuSgm.closeSession();
  }

  static int run(Path root) throws Exception {
    checks = 0;
    attributes(root);
    pages(root);
    stereo(root);
    return checks;
  }

  private static void attributes(Path root) throws Exception {
    double[][] points = new double[70][];
    int n = 0;
    for (int x = -3; x <= 3; x++)
      for (int y = -2; y <= 2; y++)
        for (int z : new int[] {-1, 1}) points[n++] = new double[] {x, y, .02 * z};
    SurfaceGeometry.Plane plane = SurfaceGeometry.plane(points, n);
    SurfaceAttributes a = SurfaceAttributes.of(plane, points, n, new double[3], true);
    check(
        Math.abs(a.roughness() - .02) < 1e-12,
        "Independent orthogonal distribution has 2-cm RMS roughness");
    check(
        Math.abs(a.variation() - .0004 / 6.0004) < 1e-12,
        "Surface variation uses sum of eigenvalues");
    check(
        Math.abs(a.planarity() - (2 - .0004) / 4) < 1e-12,
        "Planarity distinguishes planar from linear neighborhoods");
    check(
        a.neighbors() == 70 && a.status() == 0 && a.trimmed(),
        "Neighborhood support/status retained");
    Path dir = Files.createDirectories(root.resolve("attribute cloud"));
    try (CloudWriter w = new CloudWriter(dir, 3, null, true)) {
      w.point(1, 2, 3, 10, 20, 30, new double[] {0, 0, -1}, a);
      w.point(4, 5, 6, 40, 50, 60, null, SurfaceAttributes.missing(false));
    }
    ByteBuffer las =
        ByteBuffer.wrap(Files.readAllBytes(dir.resolve("dense.las")))
            .order(ByteOrder.LITTLE_ENDIAN);
    int offset = las.getInt(96), length = Short.toUnsignedInt(las.getShort(105));
    check(
        length == 58 && las.getInt(107) == 2 && las.limit() == offset + 2 * length,
        "LAS header/count/extra record size agree");
    check(
        las.getInt(100) == 1 && Short.toUnsignedInt(las.getShort(227 + 18)) == 4,
        "Extra Bytes uses LASF_Spec record 4");
    for (int i = 0; i < 10; i++) {
      int d = 227 + 54 + 192 * i;
      check((las.get(d + 2) & 255) == (i < 7 ? 9 : i == 7 ? 3 : 1), "Extra Bytes scalar type " + i);
      if (i < 7)
        check(
            Double.isNaN(las.getDouble(d + 40)) && las.get(d + 3) == 1,
            "NaN nodata upcast to float64 " + i);
    }
    check(
        Math.abs(las.getFloat(offset + 26 + 12) - .06) < 1e-7,
        "Roughness scales into exported coordinate units");
    check(
        las.getFloat(offset + 26 + 8) == -1 && las.getShort(offset + 26 + 28) == 70,
        "Normal orientation and support round trip");
    check(
        Float.isNaN(las.getFloat(offset + length + 26)) && las.get(offset + length + 26 + 30) == 1,
        "Unreliable normals remain NaN with status");
    byte[] ply = Files.readAllBytes(dir.resolve("dense.ply"));
    String text = new String(ply, java.nio.charset.StandardCharsets.ISO_8859_1);
    int header = text.indexOf("end_header\n") + 11;
    check(
        text.contains("property float nx\n") && text.contains("property float planarity\n"),
        "PLY normals and diagnostics are discoverable properties");
    check(ply.length == header + 2 * 59, "PLY property sizes match payload");
    for (int i = 0; i < 2; i++)
      check(
          Arrays.equals(
              Arrays.copyOfRange(ply, header + i * 59 + 27, header + (i + 1) * 59),
              Arrays.copyOfRange(las.array(), offset + i * 58 + 26, offset + (i + 1) * 58)),
          "PLY/LAS extra attributes are point aligned " + i);
    check(
        SurfaceDiagnostics.sample(dir.resolve("dense.ply"), 100, null).size() == 2,
        "Surface analysis accepts attribute PLY");
    Path crs = Files.createDirectories(root.resolve("attribute CRS"));
    try (CloudWriter w = new CloudWriter(crs, 1, new Crs("3067"), true)) {
      w.point(490000, 6700000, -12, 1, 2, 3, new double[] {0, 0, -1}, a);
    }
    las =
        ByteBuffer.wrap(Files.readAllBytes(crs.resolve("dense.las")))
            .order(ByteOrder.LITTLE_ENDIAN);
    check(
        las.getInt(100) == 2 && las.limit() == las.getInt(96) + 58,
        "CRS VLR and attribute VLR coexist");
    try {
      Options.parse(new String[] {"--point-attributes", "bogus"});
      check(false, "Unknown attribute mode rejected");
    } catch (IllegalArgumentException expected) {
      check(true, "Unknown attribute mode rejected");
    }
  }

  private static void pages(Path root) throws Exception {
    Options o = new Options();
    o.output = root;
    o.tileSize = 1024;
    o.disparities = 1;
    o.memoryMb = 8;
    for (boolean disk : new boolean[] {false, true})
      try (PagedVolume v = new PagedVolume(9000, 2, 0, disk, o, "short-transfer-");
          Arena a = Arena.ofConfined()) {
        MemorySegment in = a.allocate(6200), out = a.allocate(6200);
        for (int i = 0; i < 3100; i++)
          in.set(java.lang.foreign.ValueLayout.JAVA_SHORT, i * 2L, (short) (65535 - i));
        v.transfer(900, in, true);
        for (int i = 8999; i >= 0; i--) v.get(i);
        v.transfer(900, out, false);
        check(
            in.mismatch(out) == -1,
            "Unsigned-16 bulk transfer spans pages and evictions; disk=" + disk);
        check(v.get(900) == 65535 && v.get(3999) == 62436, "Unsigned values do not sign extend");
      }
  }

  private static void stereo(Path root) throws Exception {
    Options o = TestSupport.options();
    o.output = root;
    o.p2 = 7679;
    check(
        StereoCost.aggregateBound(o) == 65528 && StereoCost.aggregateBytes(o) == 2,
        "Largest eight-path 5x5 ushort bound");
    o.p2 = 7680;
    check(
        StereoCost.aggregateBound(o) == 65536 && StereoCost.aggregateBytes(o) == 4,
        "First unsafe bound automatically selects int32");
    o.sgmAggregate = "16";
    try {
      StereoCost.aggregateBytes(o);
      check(false, "Unsafe forced ushort rejected");
    } catch (IllegalArgumentException expected) {
      check(true, "Unsafe forced ushort rejected");
    }
    int w = 192, h = 65;
    byte[] a = new byte[w * h], b = new byte[a.length];
    new Random(988).nextBytes(a);
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) b[y * w + x] = a[y * w + Math.min(w - 1, x + 11)];
    for (String cost : List.of("census", "ad-census", "census-gradient", "zncc")) {
      o = TestSupport.options();
      o.output = root;
      o.disparities = 80;
      o.minDisparity = -4;
      o.matchingCost = cost;
      o.speckleSize = 0;
      o.median = 1;
      o.directions = 4;
      o.census = 11;
      o.sgmAggregate = "32";
      CensusSgm.Result expected = new CensusSgm(o).compute(a, b, w, h);
      o.sgmAggregate = "16";
      o.tiled = true;
      CensusSgm.Result actual = new CensusSgm(o).compute(a, b, w, h);
      check(
          Arrays.equals(expected.disparity, actual.disparity)
              && expected.stats.csv().equals(actual.stats.csv()),
          cost + ": paged ushort matches resident int32 exactly");
      if (!GpuSgm.devices().isEmpty())
        for (String storage : List.of("16", "32")) {
          o.compute = "gpu";
          o.gpuDevice = Integer.getInteger("sgm.test.gpu", 0);
          o.gpuSgm = "streamed";
          o.gpuStripRows = 7;
          o.gpuBankMb = 1;
          o.sgmAggregate = storage;
          actual = new CensusSgm(o).compute(a, b, w, h);
          check(
              Arrays.equals(expected.disparity, actual.disparity)
                  && expected.stats.csv().equals(actual.stats.csv()),
              cost + ": streamed banked " + storage + " matches CPU int32");
        }
    }
    o = TestSupport.options();
    o.output = root;
    o.disparities = 48;
    o.p2 = 1000000;
    o.census = 11;
    o.speckleSize = 0;
    o.sgmAggregate = "32";
    CensusSgm.Result reference = new CensusSgm(o).compute(a, b, w, h);
    o.sgmAggregate = "auto";
    check(StereoCost.aggregateBytes(o) == 4, "Large scaled penalties require int32");
    if (!GpuSgm.devices().isEmpty()) {
      o.compute = "gpu";
      o.gpuDevice = Integer.getInteger("sgm.test.gpu", 0);
      o.gpuSgm = "streamed";
      o.gpuStripRows = 1;
      CensusSgm.Result actual = new CensusSgm(o).compute(a, b, w, h);
      check(
          Arrays.equals(reference.disparity, actual.disparity),
          "Large-penalty automatic int32 streamed output matches CPU");
    }
  }
}
