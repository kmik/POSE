package sgm;

import java.awt.image.*;
import java.nio.file.*;
import java.util.*;
import java.util.concurrent.*;
import javax.imageio.*;
import org.opencv.core.*;

/**
 * Pre-dense speed revision: direct 8-bit sRGB pixel access equals getRGB; parallel and single-pass
 * windowed decoding equal the serial region decoder byte for byte; per-pair seeded matching is
 * independent of thread history; reused searches equal recomputed ones; ordered pair execution
 * delivers results in input order.
 */
final class PreDenseSpeedRegression {
  private static int checks;

  private static void check(boolean ok, String why) {
    checks++;
    if (!ok) throw new AssertionError(why);
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Files.createDirectories(root);
    pixels();
    windowed(root);
    matching(root);
    ordered();
    return checks;
  }

  private static void pixels() {
    int w = 37, h = 23;
    for (int type :
        new int[] {
          BufferedImage.TYPE_3BYTE_BGR,
          BufferedImage.TYPE_4BYTE_ABGR,
          BufferedImage.TYPE_INT_RGB,
          BufferedImage.TYPE_BYTE_GRAY,
          BufferedImage.TYPE_USHORT_GRAY
        }) {
      BufferedImage image = new BufferedImage(w, h, type);
      fill(image);
      compare(image, "type " + type);
    }
    // Four interleaved 8-bit bands with unassociated alpha, as large RGB+NIR aerial TIFFs decode.
    ColorModel cm =
        new ComponentColorModel(
            java.awt.color.ColorSpace.getInstance(java.awt.color.ColorSpace.CS_sRGB),
            true,
            false,
            java.awt.Transparency.TRANSLUCENT,
            DataBuffer.TYPE_BYTE);
    WritableRaster raster = cm.createCompatibleWritableRaster(w, h);
    BufferedImage rgba = new BufferedImage(cm, raster, false, null);
    Random random = new Random(3);
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++)
        for (int b = 0; b < 4; b++) raster.setSample(x, y, b, random.nextInt(256));
    compare(rgba, "custom RGBA");
    // A translated child raster exercises sample-model offsets.
    BufferedImage parent = new BufferedImage(w + 9, h + 5, BufferedImage.TYPE_3BYTE_BGR);
    fill(parent);
    compare(parent.getSubimage(5, 3, w, h), "subimage");
  }

  private static void compare(BufferedImage image, String label) {
    int w = image.getWidth(), h = image.getHeight();
    int[] expected = image.getRGB(3, 2, w - 5, h - 4, null, 0, w - 5);
    int[] actual = WindowedImage.pixels(image, 3, 2, w - 5, h - 4);
    check(Arrays.equals(expected, actual), "Direct pixels equal getRGB: " + label);
  }

  private static void fill(BufferedImage image) {
    for (int y = 0; y < image.getHeight(); y++)
      for (int x = 0; x < image.getWidth(); x++)
        image.setRGB(
            x,
            y,
            0xff000000
                | ((x * 31 + y * 7) & 255) << 16
                | ((x * 11 + y * 19) & 255) << 8
                | ((x * 17 + y * 43) & 255));
  }

  private static void windowed(Path root) throws Exception {
    int w = 613, h = 389;
    BufferedImage image = new BufferedImage(w, h, BufferedImage.TYPE_3BYTE_BGR);
    Random random = new Random(9);
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++)
        image.setRGB(
            x,
            y,
            ((x * 3 + random.nextInt(40)) & 255) << 16
                | ((y * 5 + random.nextInt(40)) & 255) << 8
                | ((x + y + random.nextInt(40)) & 255));
    List<Path> inputs = new ArrayList<>();
    for (String format : new String[] {"jpg", "png", "tif"}) {
      Path file = root.resolve("speed-source." + format);
      if (format.equals("tif")) {
        // Tiled TIFF: the format that keeps true region reads.
        ImageWriter writer = null;
        for (var it = ImageIO.getImageWritersByFormatName("TIFF"); it.hasNext(); ) {
          ImageWriter candidate = it.next();
          if (writer == null && candidate.getDefaultWriteParam().canWriteTiles()) writer = candidate;
          else candidate.dispose();
        }
        check(writer != null, "Tiled TIFF writer available");
        try (var out = ImageIO.createImageOutputStream(file.toFile())) {
          writer.setOutput(out);
          ImageWriteParam param = writer.getDefaultWriteParam();
          param.setTilingMode(ImageWriteParam.MODE_EXPLICIT);
          param.setTiling(64, 64, 0, 0);
          writer.write(null, new IIOImage(image, null, null), param);
        } finally {
          writer.dispose();
        }
      } else check(ImageIO.write(image, format, file.toFile()), format + " fixture writer");
      inputs.add(file);
    }
    for (Path input : inputs)
      for (int[] size : new int[][] {{w, h}, {301, 190}, {97, 61}})
        for (int window : new int[] {48, 200}) {
          Mat serial = WindowedImage.read(input, w, h, size[0], size[1], window, 1, 0);
          try {
            for (int threads : new int[] {2, 5})
              for (long whole : new long[] {0, Long.MAX_VALUE}) {
                Mat other =
                    WindowedImage.read(input, w, h, size[0], size[1], window, threads, whole);
                try {
                  check(
                      Core.norm(serial, other, Core.NORM_INF) == 0,
                      "Windowed decode independent of threads/single pass: "
                          + input.getFileName()
                          + " "
                          + Arrays.toString(size)
                          + " window "
                          + window);
                } finally {
                  other.release();
                }
              }
          } finally {
            serial.release();
          }
        }
    // Row order seen by sinks is unchanged by parallel tiles.
    List<String> serialRows = new ArrayList<>(), parallelRows = new ArrayList<>();
    WindowedImage.readTo(
        inputs.get(2), w, h, 301, 190, 48, 1, 0, (x, y, row) -> serialRows.add(x + ":" + y));
    WindowedImage.readTo(
        inputs.get(2), w, h, 301, 190, 48, 4, 0, (x, y, row) -> parallelRows.add(x + ":" + y));
    check(serialRows.equals(parallelRows), "Parallel windowed decode keeps sink order");
  }

  private static ImageFrame frame(Path root, int id, Mat descriptors) throws Exception {
    Path image = root.resolve("frame.png");
    if (!Files.exists(image))
      check(
          ImageIO.write(new BufferedImage(64, 48, BufferedImage.TYPE_3BYTE_BGR), "png", image.toFile()),
          "frame fixture writer");
    Calibration c = new Calibration(60, 60, 31.5, 23.5, new double[5], 64, 48);
    ImageFrame f = new ImageFrame(id, image, c, TestSupport.options());
    f.descriptors.release();
    f.descriptors = descriptors;
    return f;
  }

  private static String rng() {
    Mat r = new Mat(1, 4, CvType.CV_32F);
    Core.randu(r, 0, 1);
    float[] v = new float[4];
    r.get(0, 0, v);
    r.release();
    return Arrays.toString(v);
  }

  private static void matching(Path root) throws Exception {
    Options o = TestSupport.options();
    o.descriptorMatcher = "flann";
    o.pairMatching = "parallel";
    Mat a = random(11, 1500), noise = random(12, 1500), b = new Mat(), c = random(13, 1500);
    Core.scaleAdd(noise, 0.15, a, b);
    ImageFrame fa = frame(root, 0, a), fb = frame(root, 1, b), fc = frame(root, 2, c);
    fa.descriptorDigest = "a";
    fb.descriptorDigest = "b";
    fc.descriptorDigest = "c";
    PairMatching.clear();
    ExecutorService pool = Executors.newFixedThreadPool(2);
    try {
      String clean = pool.submit(() -> search(o, fa, fb, false)).get();
      String afterOther =
          pool.submit(
                  () -> {
                    Core.setRNGSeed(77);
                    search(o, fb, fc, false);
                    search(o, fa, fc, false);
                    return search(o, fa, fb, false);
                  })
              .get();
      check(clean.equals(afterOther), "Seeded pair search is independent of thread history");
      o.pairMatching = "serial";
      String s1 = pool.submit(() -> { Core.setRNGSeed(5); return search(o, fa, fb, false); }).get();
      String s2 = pool.submit(() -> { Core.setRNGSeed(6); return search(o, fa, fb, false); }).get();
      check(!s1.equals(s2), "FLANN search depends on the RNG state (seeding is meaningful)");
      o.pairMatching = "parallel";
      String stored = pool.submit(() -> search(o, fa, fb, true)).get();
      check(PairMatching.remembered() == 1, "Search remembered for reuse");
      boolean[] reused = new boolean[1];
      String restored =
          pool.submit(
                  () -> {
                    List<MatOfDMatch> fw = new ArrayList<>();
                    Map<Integer, Integer> reverse = new HashMap<>();
                    PairMatching.reciprocal(o, fa, fb, fw, reverse, s -> {}, false, reused);
                    return digest(fw, reverse) + "/" + rng();
                  })
              .get();
      check(reused[0], "Identical descriptors reuse the earlier search");
      String recomputed = pool.submit(() -> search(o, fa, fb, false) + "/" + rng()).get();
      check(stored.equals(clean), "Remembering does not change the search");
      check(
          restored.substring(0, restored.lastIndexOf('/')).equals(clean),
          "Reused search equals recomputed search");
      fb.descriptorDigest = "b-changed";
      boolean[] again = new boolean[1];
      pool.submit(
              () -> {
                List<MatOfDMatch> fw = new ArrayList<>();
                PairMatching.reciprocal(o, fa, fb, fw, new HashMap<>(), s -> {}, false, again);
                for (Mat m : fw) m.release();
                return null;
              })
          .get();
      check(!again[0], "Changed descriptors are not served from the reuse store");
      check(recomputed.equals(restored), "Reused search leaves the same RNG state for RANSAC");
    } finally {
      pool.shutdown();
      PairMatching.clear();
    }
  }

  private static String search(Options o, ImageFrame a, ImageFrame b, boolean remember) {
    List<MatOfDMatch> fw = new ArrayList<>();
    Map<Integer, Integer> reverse = new HashMap<>();
    PairMatching.reciprocal(o, a, b, fw, reverse, s -> {}, remember, null);
    return digest(fw, reverse);
  }

  private static String digest(List<MatOfDMatch> fw, Map<Integer, Integer> reverse) {
    StringBuilder s = new StringBuilder();
    long h = 1;
    for (MatOfDMatch row : fw) {
      for (DMatch d : row.toArray())
        h = h * 31 + d.queryIdx * 131L + d.trainIdx * 7L + d.imgIdx + Float.floatToIntBits(d.distance);
      row.release();
    }
    s.append(Long.toHexString(h)).append(':').append(new TreeMap<>(reverse));
    return s.toString();
  }

  private static Mat random(int seed, int rows) {
    Mat m = new Mat(rows, 128, CvType.CV_32F);
    Core.setRNGSeed(seed);
    Core.randu(m, 0, 64);
    return m;
  }

  private static void ordered() throws Exception {
    List<int[]> edges = new ArrayList<>();
    for (int i = 0; i < 40; i++) edges.add(new int[] {i, i + 1});
    List<Integer> seen = new ArrayList<>();
    PairMatching.ordered(
        edges,
        6,
        edge -> {
          java.util.concurrent.locks.LockSupport.parkNanos(((edge[0] * 7) % 5) * 1_000_000L);
          return edge[0] * 10;
        },
        (edge, value) -> {
          check(value == edge[0] * 10, "Ordered result belongs to its edge");
          seen.add(edge[0]);
        });
    List<Integer> expected = new ArrayList<>();
    for (int i = 0; i < 40; i++) expected.add(i);
    check(seen.equals(expected), "Ordered pair execution consumes results in input order");
    boolean failed = false;
    try {
      PairMatching.ordered(
          edges,
          4,
          edge -> {
            if (edge[0] == 17) throw new java.io.IOException("expected");
            return edge[0];
          },
          (edge, value) -> {});
    } catch (java.io.IOException e) {
      failed = e.getMessage().equals("expected");
    }
    check(failed, "Worker failures propagate to the caller");
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    System.out.println(
        "PASS pre-dense speed checks=" + run(Path.of("target/gpu-review/pre-dense-speed")));
  }
}
