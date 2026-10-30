package sgm;

import java.io.*;
import java.nio.*;
import java.nio.file.*;
import java.util.*;
import org.locationtech.proj4j.*;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

/** Independent geometric controls and full automatic-calibration/GPS export regressions. */
final class GeoCalibrationRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  private static void near(double x, double y, double tolerance, String message) {
    check(Math.abs(x - y) <= tolerance, message + ": " + x + " vs " + y);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Core.setNumThreads(1);
    Core.setRNGSeed(1);
    Path root = Files.createTempDirectory(Paths.get("target"), "geo calibration ");
    System.out.println("PASS new calibration/GPS checks: " + run(root) + "; fixtures=" + root);
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Path dir = Files.createDirectory(root.resolve("georeference calibration"));
    testFocal();
    testProjection();
    testMetadata(dir);
    testRobustFit();
    testLas(dir);
    Path images = render(dir);
    Options o = TestSupport.options();
    o.input = images;
    o.output = dir.resolve("estimate");
    Files.createDirectories(o.output);
    o.features = 6000;
    o.neighbors = 3;
    o.maxDimension = 640;
    o.cores = 1;
    Calibration c = AutomaticCalibration.estimate(o, Main.images(o), 640, 480, null);
    near(
        c.fx,
        420,
        45,
        "Focal length recovered from rotated non-planar scene without EXIF or camera input");
    check(
        Files.readString(o.output.resolve("01_calibration/self_calibration.json"))
            .contains("image_matches_shared_focal"),
        "Scene geometry, not a guessed focal length, supplies calibration");
    o.output = dir.resolve("unobservable");
    Files.createDirectories(o.output);
    boolean failed = false;
    try {
      AutomaticCalibration.estimate(
          o, Arrays.asList(images.resolve("0.png"), images.resolve("0.png")), 640, 480, null);
    } catch (IllegalArgumentException e) {
      failed = e.getMessage().contains("could not determine focal length");
    }
    check(
        failed,
        "Unobservable image geometry without metadata fails instead of inventing calibration");
    testGpsPipeline(dir, images);
    return checks;
  }

  private static double[] cross(double[] t) {
    return new double[] {0, -t[2], t[1], t[2], 0, -t[0], -t[1], t[0], 0};
  }

  private static void testFocal() {
    double focal = .63;
    double[] e =
        Math3.mul(cross(new double[] {-.9, .2, .3}), Math3.rotation(new double[] {.13, -.19, .06}));
    double[] f = e.clone(), k = {focal, focal, 1};
    for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) f[r * 3 + c] /= k[r] * k[c];
    near(
        AutomaticCalibration.focalFromFundamental(f),
        focal,
        1e-6,
        "Exact shared-focal recovery from known essential geometry");
    double[] translation = cross(new double[] {1, 0, 0});
    near(
        AutomaticCalibration.essentialError(translation, .2),
        AutomaticCalibration.essentialError(translation, 2),
        1e-12,
        "Pure lateral translation has no focal constraint");
  }

  private static void testProjection() {
    double[] xy = new Crs("3067").project(new Exif.Gps(60, 27, 123));
    near(xy[0], 500000, .001, "TM35FIN central meridian easting");
    // Independent GRS80 meridional arc at 60N, k0=.9996 (not obtained from Proj4J).
    near(xy[1], 6651411.1902, .02, "TM35FIN northing at 60N");
    near(xy[2], 123, 0, "Altitude retained without invented geoid correction");
    double[] merc = new Crs("EPSG:3857").project(new Exif.Gps(0, 1, 4));
    near(merc[0], 6378137 * Math.PI / 180, .0001, "User-selected EPSG:3857 projection");
    boolean rejected = false;
    try {
      new Crs("EPSG:4326");
    } catch (IllegalArgumentException e) {
      rejected = true;
    }
    check(rejected, "Geographic degrees rejected for metric 3D alignment");
    rejected = false;
    try {
      new Crs("EPSG:9999");
    } catch (IllegalArgumentException e) {
      rejected = true;
    }
    check(rejected, "Unknown EPSG is an explicit error");
    for (String code : new String[] {"EPSG:4978", "EPSG:2227"}) {
      rejected = false;
      try {
        new Crs(code);
      } catch (IllegalArgumentException failure) {
        rejected = true;
      }
      check(rejected, "Non-projected/geocentric or non-metre CRS rejected: " + code);
    }
  }

  private static void testMetadata(Path dir) throws IOException {
    for (boolean little : new boolean[] {true, false}) {
      Path tif = dir.resolve("metadata_" + little + ".tif");
      Files.write(tif, tiff(new Exif.Gps(-60.25, -24.75, -12.5), 28, little, true));
      Exif.Metadata m = Exif.read(tif);
      near(m.gps.latitude, -60.25, 1e-8, "TIFF latitude and hemisphere");
      near(m.gps.longitude, -24.75, 1e-8, "TIFF longitude and hemisphere");
      near(m.gps.altitude, -12.5, 1e-8, "Below-sea-level altitude reference");
      near(m.focal35Mm, 28, 0, "EXIF equivalent focal");
      near(m.focalMm, 3.6, 1e-8, "EXIF rational focal");
      check(m.gps.usable(), "WGS-84 map datum recognized");
    }
    Path missing = dir.resolve("no-altitude.tif");
    Files.write(missing, tiff(new Exif.Gps(60, 25, 0), 28, true, false));
    check(
        Double.isNaN(Exif.gps(missing).altitude) && !Exif.gps(missing).usable(),
        "Absent altitude remains unknown, not silently zero");
    Path broken = dir.resolve("bad-exif.jpg");
    Files.write(
        broken,
        new byte[] {
          (byte) 255,
          (byte) 216,
          (byte) 255,
          (byte) 225,
          0,
          16,
          69,
          120,
          105,
          102,
          0,
          0,
          73,
          73,
          42,
          0
        });
    check(Exif.read(broken).gps == null, "Truncated metadata ignored safely");
  }

  private static void testRobustFit() {
    List<double[]> source = new ArrayList<>(), target = new ArrayList<>();
    double[] r = Math3.rotation(new double[] {.2, -.4, .3}), t = {500000, 7000000, 150};
    Random random = new Random(104);
    for (int i = 0; i < 12; i++) {
      double[] p = {random.nextDouble() * 5, random.nextDouble() * 4, random.nextDouble() * 2};
      source.add(p);
      target.add(Math3.add(Math3.times(Math3.mv(r, p), 3), t));
    }
    target.set(9, new double[] {10000, 20000, 30000});
    target.set(11, new double[] {100000, 20000, 300});
    Georeference.Similarity s = Georeference.robustFit(source, target, .05);
    near(s.scale, 3, 1e-8, "GPS outliers do not distort metric scale");
    near(
        Math3.norm(Math3.sub(s.apply(source.get(0)), target.get(0))),
        0,
        1e-7,
        "Robust projected GPS fit");
    List<double[]> line =
        Arrays.asList(new double[] {0, 0, 0}, new double[] {1, 0, 0}, new double[] {2, 0, 0});
    boolean rejected = false;
    try {
      Georeference.robustFit(line, line, 1);
    } catch (IllegalArgumentException e) {
      rejected = true;
    }
    check(rejected, "Collinear GPS cannot define a 3D rotation");
  }

  private static void testLas(Path dir) throws IOException {
    Path out = dir.resolve("crs-las");
    try (CloudWriter w = new CloudWriter(out, 1, new Crs("3067"))) {
      w.point(500000.1234, 6651411.19, 111.2, 12, 34, 56);
    }
    ByteBuffer b =
        ByteBuffer.wrap(Files.readAllBytes(out.resolve("dense.las")))
            .order(ByteOrder.LITTLE_ENDIAN);
    check(b.getInt(100) == 1, "LAS contains a projection VLR");
    int offset = b.getInt(96);
    check(offset == 321 && b.capacity() == offset + 26, "Point data starts after complete CRS VLR");
    check(
        (b.getShort(245) & 65535) == 34735 && (b.getShort(247) & 65535) == 40,
        "GeoKeyDirectoryTag ID and payload size");
    check(
        (b.getShort(281) & 65535) == 1 && (b.getShort(287) & 65535) == 4,
        "GeoTIFF directory header");
    Map<Integer, Integer> keys = new HashMap<>();
    for (int i = 0; i < 4; i++) {
      int p = 289 + 8 * i;
      check(b.getShort(p + 2) == 0 && b.getShort(p + 4) == 1, "GeoTIFF inline scalar key");
      keys.put(b.getShort(p) & 65535, b.getShort(p + 6) & 65535);
    }
    check(
        keys.get(1024) == 1 && keys.get(3072) == 3067 && keys.get(3076) == 9001,
        "LAS projected EPSG and metre units independently decoded");
    near(
        b.getInt(offset) * b.getDouble(131) + b.getDouble(155),
        500000.1234,
        .00011,
        "CRS-tagged LAS X round trip");
    check(
        new String(
                Files.readAllBytes(out.resolve("dense.ply")),
                java.nio.charset.StandardCharsets.ISO_8859_1)
            .contains("horizontal_crs EPSG:3067"),
        "PLY declares horizontal CRS");
  }

  static Path render(Path dir) throws IOException {
    return render(dir, 4);
  }

  static Path render(Path dir, int cameraCount) throws IOException {
    Path images = Files.createDirectory(dir.resolve("rotated scene"));
    int w = 640, h = 480, tw = 1400, th = 1000;
    byte[][] textures = new byte[3][];
    for (int k = 0; k < 3; k++) {
      Mat texture = new Mat(th, tw, CvType.CV_8UC3);
      byte[] v = new byte[tw * th * 3];
      new Random(331 + k).nextBytes(v);
      texture.put(0, 0, v);
      Imgproc.GaussianBlur(texture, texture, new Size(3, 3), .6);
      texture.get(0, 0, v);
      textures[k] = v;
      texture.release();
    }
    for (int i = 0; i < cameraCount; i++) {
      Camera camera = truth(i);
      double[] rt = Math3.transpose(camera.rotation), center = camera.center();
      byte[] color = new byte[w * h * 3];
      for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
          double[] ray =
              Math3.mv(
                  rt, new double[] {(x - camera.cx) / camera.fx, (y - camera.cy) / camera.fy, 1});
          for (int k = 0; k < 3; k++) {
            double depth = (new double[] {4, 7, 11}[k] - center[2]) / ray[2],
                wx = center[0] + depth * ray[0],
                wy = center[1] + depth * ray[1];
            if (depth <= 0
                || (k == 0 && wx >= -.8)
                || (k == 1 && (wx < -.8 || wx > .8))
                || (k == 2 && wx <= .8)) continue;
            double textureScale = new double[] {90, 50, 32}[k];
            double tx = wx * textureScale + tw / 2., ty = wy * textureScale + th / 2.;
            int ix = (int) Math.floor(tx), iy = (int) Math.floor(ty);
            if (ix < 0 || iy < 0 || ix >= tw - 1 || iy >= th - 1) break;
            double ax = tx - ix, ay = ty - iy;
            byte[] texture = textures[k];
            for (int c = 0; c < 3; c++) {
              int p = (iy * tw + ix) * 3 + c;
              double v =
                  (texture[p] & 255) * (1 - ax) * (1 - ay)
                      + (texture[p + 3] & 255) * ax * (1 - ay)
                      + (texture[p + tw * 3] & 255) * (1 - ax) * ay
                      + (texture[p + tw * 3 + 3] & 255) * ax * ay;
              color[(y * w + x) * 3 + c] = (byte) Math.round(v);
            }
            break;
          }
        }
      Mat image = new Mat(h, w, CvType.CV_8UC3);
      image.put(0, 0, color);
      try {
        if (!Imgcodecs.imwrite(images.resolve(i + ".png").toString(), image))
          throw new IOException("Cannot save fixture");
      } finally {
        image.release();
      }
    }
    return images;
  }

  private static Camera truth(int i) {
    double[][] centers = {
      {-.9, 0, 0}, {0, .28, .15}, {.8, -.35, .28}, {.2, .7, -.15}, {.4, -.7, .1}, {-.6, .6, .3}
    };
    double[][] rotations = {
      {.03, -.10, .01},
      {-.04, .02, -.03},
      {.07, .13, .02},
      {-.09, -.06, .04},
      {.11, -.12, .03},
      {-.1, .15, -.04}
    };
    Camera c = new Camera(420, 420, 319.5, 239.5);
    c.rotation = Math3.rotation(rotations[i]);
    c.translation = Math3.times(Math3.mv(c.rotation, centers[i]), -1);
    return c;
  }

  private static void testGpsPipeline(Path dir, Path pngs) throws Exception {
    Path images = Files.createDirectory(dir.resolve("GPS images"));
    CRSFactory factory = new CRSFactory();
    CoordinateTransform inverse =
        new CoordinateTransformFactory()
            .createTransform(
                factory.createFromName("EPSG:3067"), factory.createFromName("EPSG:4326"));
    double[] rotation = Math3.rotation(new double[] {.1, -.15, .4}),
        translation = {500000, 6651411.1902, 130};
    List<double[]> targets = new ArrayList<>();
    Path poses = dir.resolve("known-local-poses.csv");
    try (BufferedWriter w = Files.newBufferedWriter(poses)) {
      w.write("image,r00,r01,r02,r10,r11,r12,r20,r21,r22,tx,ty,tz\n");
      for (int i = 0; i < 4; i++) {
        Camera c = truth(i);
        double[] target = Math3.add(Math3.times(Math3.mv(rotation, c.center()), 10), translation);
        targets.add(target);
        ProjCoordinate ll =
            inverse.transform(new ProjCoordinate(target[0], target[1]), new ProjCoordinate());
        Mat m = Imgcodecs.imread(pngs.resolve(i + ".png").toString());
        Path jpeg = images.resolve(i + ".jpg");
        try {
          Imgcodecs.imwrite(jpeg.toString(), m);
        } finally {
          m.release();
        }
        inject(jpeg, tiff(new Exif.Gps(ll.y, ll.x, target[2]), 23, true, true));
        w.write(i + ".jpg");
        for (double v : c.rotation) w.write("," + v);
        for (double v : c.translation) w.write("," + v);
        w.write("\n");
      }
    }
    Path calibration = dir.resolve("known-camera.txt");
    new Calibration(420, 420, 319.5, 239.5, new double[5], 640, 480)
        .write(calibration, "Known synthetic camera");
    Path out = dir.resolve("GPS project");
    int result =
        TestSupport.run(
            new String[] {
              "-i",
              images.toString(),
              "-o",
              out.toString(),
              "--calibration",
              calibration.toString(),
              "--poses",
              poses.toString(),
              "--max-dimension",
              "640",
              "--disparities",
              "128",
              "--step",
              "6",
              "--cores",
              "1"
            });
    check(result == 0, "Automatic GPS alignment and dense pipeline complete without --gps");
    check(
        Files.readString(out.resolve("coordinates.json")).contains("\"epsg\":3067"),
        "Default EPSG:3067 exported in result sidecar");
    List<String> cameras = Files.readAllLines(out.resolve("cameras.csv"));
    for (int i = 0; i < 4; i++) {
      List<String> row = PoseIO.csv(cameras.get(i + 1));
      // PoseIO CSV includes camera centers after its 12 R/t entries.
      double[] r = new double[9], t = new double[3];
      for (int k = 0; k < 9; k++) r[k] = Double.parseDouble(row.get(k + 1));
      for (int k = 0; k < 3; k++) t[k] = Double.parseDouble(row.get(k + 10));
      near(
          Math3.norm(Math3.sub(Math3.times(Math3.mv(Math3.transpose(r), t), -1), targets.get(i))),
          0,
          .002,
          "Exported camera center has projected GPS coordinates");
    }
    ByteBuffer las =
        ByteBuffer.wrap(Files.readAllBytes(out.resolve("dense.las")))
            .order(ByteOrder.LITTLE_ENDIAN);
    check(
        las.getInt(100) == 1 && las.getInt(107) > 500,
        "Georeferenced dense LAS has CRS and a non-empty cloud");
    int offset = las.getInt(96);
    double e = las.getInt(offset) * las.getDouble(131) + las.getDouble(155),
        n = las.getInt(offset + 4) * las.getDouble(139) + las.getDouble(163);
    check(
        Math.abs(e - 500000) < 500 && Math.abs(n - 6651411) < 500,
        "Dense cloud is actually transformed, not merely tagged with EPSG");
    check(
        Files.readString(out.resolve("report.txt")).contains("vertical datum is unspecified"),
        "Vertical reference is reported honestly");

    Options diagnostic = TestSupport.options();
    diagnostic.input = images;
    diagnostic.output = Files.createDirectory(dir.resolve("GPS diagnostics"));
    diagnostic.calibration = calibration;
    diagnostic.maxDimension = 320;
    List<ImageFrame> frames = new ArrayList<>();
    try {
      Calibration c = Calibration.load(diagnostic, 640, 480);
      for (Path p : Main.images(diagnostic))
        frames.add(new ImageFrame(frames.size(), p, c, diagnostic));
      PoseIO.read(poses, frames);
      check(
          Georeference.align(frames.subList(0, 2), Collections.emptyList(), diagnostic).crs == null,
          "Two GPS positions leave automatic output unreferenced");
      diagnostic.gps = true;
      boolean rejected = false;
      try {
        Georeference.align(frames.subList(0, 2), Collections.emptyList(), diagnostic);
      } catch (IllegalArgumentException failure) {
        rejected = true;
      }
      check(rejected, "--gps requires successful alignment");
      diagnostic.gps = false;
      diagnostic.noGps = true;
      check(
          Georeference.align(frames, Collections.emptyList(), diagnostic).crs == null,
          "--no-gps disables alignment despite usable metadata");
      diagnostic.noGps = false;
      diagnostic.scale = 2;
      rejected = false;
      try {
        Georeference.align(frames, Collections.emptyList(), diagnostic);
      } catch (IllegalArgumentException failure) {
        rejected = failure.getMessage().contains("--scale 1");
      }
      check(rejected, "Arbitrary coordinate scaling cannot corrupt an assigned EPSG CRS");
    } finally {
      for (ImageFrame f : frames) f.close();
    }

    Options fallback = TestSupport.options();
    fallback.input = images;
    fallback.output = Files.createDirectory(dir.resolve("metadata fallback"));
    fallback.features = 1000;
    Calibration exif =
        AutomaticCalibration.estimate(
            fallback,
            Arrays.asList(images.resolve("0.jpg"), images.resolve("0.jpg")),
            640,
            480,
            null);
    near(
        exif.fx,
        23 / Math.hypot(36, 24) * 800,
        1e-8,
        "EXIF fallback uses diagonal equivalent focal without a sensor database");
    check(
        Files.readString(fallback.output.resolve("01_calibration/self_calibration.json"))
            .contains("exif_35mm_equivalent"),
        "Metadata fallback is explicitly identified");

    Path automatic = dir.resolve("automatic SfM GPS project");
    result =
        TestSupport.run(
            new String[] {
              "-i",
              images.toString(),
              "-o",
              automatic.toString(),
              "--epsg",
              "32635",
              "--gps",
              "--features",
              "6000",
              "--max-dimension",
              "640",
              "--disparities",
              "128",
              "--step",
              "8",
              "--cores",
              "1"
            });
    check(
        result == 0,
        "Full image-only SfM, automatic calibration, automatic GPS, and selectable CRS complete"
            + " together");
    check(
        Files.readString(automatic.resolve("coordinates.json")).contains("\"epsg\":32635"),
        "User-selected EPSG is retained");
    Options reload = TestSupport.options();
    reload.calibration = automatic.resolve("camera.txt");
    near(
        Calibration.load(reload, 640, 480).fx,
        420,
        45,
        "Published estimated camera is reusable as --calibration input");
  }

  static byte[] tiff(Exif.Gps gps, int equiv, boolean little, boolean altitude) {
    ByteBuffer b =
        ByteBuffer.allocate(400).order(little ? ByteOrder.LITTLE_ENDIAN : ByteOrder.BIG_ENDIAN);
    b.putShort((short) (little ? 0x4949 : 0x4d4d)).putShort((short) 42).putInt(8);
    b.position(8);
    b.putShort((short) 2);
    entry(b, 0x8769, 4, 1, 40);
    entry(b, 0x8825, 4, 1, 64);
    b.putInt(0);
    b.position(40);
    b.putShort((short) 2);
    entry(b, 0x920a, 5, 1, 300);
    inline(b, 0xa405, 3, 1, equiv);
    b.putInt(0);
    b.position(64);
    b.putShort((short) (altitude ? 7 : 5));
    asciiRef(b, 1, gps.latitude < 0 ? 'S' : 'N');
    entry(b, 2, 5, 3, 320);
    asciiRef(b, 3, gps.longitude < 0 ? 'W' : 'E');
    entry(b, 4, 5, 3, 344);
    if (altitude) {
      inline(b, 5, 1, 1, gps.altitude < 0 ? 1 : 0);
      entry(b, 6, 5, 1, 368);
    }
    entry(b, 18, 2, 7, 376);
    b.putInt(0);
    b.position(300);
    b.putInt(3600).putInt(1000);
    dms(b, 320, Math.abs(gps.latitude));
    dms(b, 344, Math.abs(gps.longitude));
    b.position(368);
    b.putInt((int) Math.round(Math.abs(gps.altitude) * 10000)).putInt(10000);
    b.position(376);
    b.put(new byte[] {'W', 'G', 'S', '-', '8', '4', 0});
    return b.array();
  }

  private static void entry(ByteBuffer b, int tag, int type, int count, int value) {
    b.putShort((short) tag).putShort((short) type).putInt(count).putInt(value);
  }

  private static void inline(ByteBuffer b, int tag, int type, int count, int value) {
    b.putShort((short) tag).putShort((short) type).putInt(count);
    if (type == 3) b.putShort((short) value).putShort((short) 0);
    else b.put((byte) value).put(new byte[3]);
  }

  private static void asciiRef(ByteBuffer b, int tag, char ref) {
    b.putShort((short) tag).putShort((short) 2).putInt(2).put((byte) ref).put(new byte[3]);
  }

  private static void dms(ByteBuffer b, int offset, double degrees) {
    int d = (int) degrees, m = (int) ((degrees - d) * 60);
    double s = ((degrees - d) * 60 - m) * 60;
    b.position(offset);
    b.putInt(d).putInt(1).putInt(m).putInt(1).putInt((int) Math.round(s * 1000000)).putInt(1000000);
  }

  static void inject(Path jpg, byte[] tiff) throws IOException {
    byte[] image = Files.readAllBytes(jpg);
    ByteArrayOutputStream out = new ByteArrayOutputStream();
    out.write(image, 0, 2);
    DataOutputStream d = new DataOutputStream(out);
    d.writeShort(0xffe1);
    d.writeShort(tiff.length + 8);
    d.write(new byte[] {'E', 'x', 'i', 'f', 0, 0});
    d.write(tiff);
    d.write(image, 2, image.length - 2);
    Files.write(jpg, out.toByteArray());
  }
}
