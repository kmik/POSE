package sgm;

import java.awt.image.*;
import java.nio.file.*;
import java.util.*;
import javax.imageio.*;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

final class ImageResourceRegression {
  private static int checks;

  private static void check(boolean ok, String why) {
    checks++;
    if (!ok) throw new AssertionError(why);
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Files.createDirectories(root);
    int w = 431, h = 277;
    for (int type : new int[] {BufferedImage.TYPE_3BYTE_BGR, BufferedImage.TYPE_USHORT_GRAY}) {
      BufferedImage image = new BufferedImage(w, h, type);
      for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
          if (type == BufferedImage.TYPE_USHORT_GRAY)
            image.getRaster().setSample(x, y, 0, (x * 113 + y * 241) & 65535);
          else
            image.setRGB(
                x,
                y,
                ((x * 31 + y * 7) & 255) << 16
                    | ((x * 11 + y * 19) & 255) << 8
                    | ((x * 17 + y * 43) & 255));
        }
      Path input = root.resolve("source-" + type + ".tif");
      check(ImageIO.write(image, "TIFF", input.toFile()), "TIFF fixture writer");
      BufferedImage decoded = ImageIO.read(input.toFile());
      int[] packed = decoded.getRGB(0, 0, w, h, null, 0, w);
      byte[] bgr = new byte[w * h * 3];
      for (int p = 0; p < packed.length; p++) {
        bgr[p * 3] = (byte) packed[p];
        bgr[p * 3 + 1] = (byte) (packed[p] >>> 8);
        bgr[p * 3 + 2] = (byte) (packed[p] >>> 16);
      }
      Mat full = new Mat(h, w, CvType.CV_8UC3);
      full.put(0, 0, bgr);
      try {
        for (int[] size : new int[][] {{w, h}, {233, 149}, {37, 23}}) {
          Mat expected = new Mat(), actual = WindowedImage.read(input, w, h, size[0], size[1], 64);
          try {
            Imgproc.resize(full, expected, new Size(size[0], size[1]), 0, 0, Imgproc.INTER_AREA);
            check(
                Core.norm(expected, actual, Core.NORM_INF) <= 1,
                "Window seams/global area weights and 8/16-bit color conversion");
          } finally {
            expected.release();
            actual.release();
          }
        }
      } finally {
        full.release();
        image.flush();
        decoded.flush();
      }
    }
    BufferedImage big = new BufferedImage(73, 51, BufferedImage.TYPE_3BYTE_BGR);
    for (int y = 0; y < 51; y++) for (int x = 0; x < 73; x++) big.setRGB(x, y, 0x123456);
    var bigWriters = ImageIO.getImageWritersByFormatName("BigTIFF");
    check(bigWriters.hasNext(), "BigTIFF provider installed");
    var bigWriter = bigWriters.next();
    Path bigFile = root.resolve("small-bigtiff.tif");
    try (var output = ImageIO.createImageOutputStream(bigFile.toFile())) {
      bigWriter.setOutput(output);
      bigWriter.write(big);
    } finally {
      bigWriter.dispose();
      big.flush();
    }
    check(
        Arrays.equals(ResolutionPlan.originalSize(bigFile), new int[] {73, 51}),
        "BigTIFF header dimensions");
    Mat bigDecoded = WindowedImage.read(bigFile, 73, 51, 37, 26, 64);
    try {
      double[] p = bigDecoded.get(13, 20);
      check(p[0] == 0x56 && p[1] == 0x34 && p[2] == 0x12, "BigTIFF windowed color decoding");
    } finally {
      bigDecoded.release();
    }
    checkLongOffset();
    Options o = TestSupport.options();
    o.output = root;
    o.rectified = true;
    o.imageReader = "windowed";
    o.imageWindow = 64;
    o.imageMemoryMb = 2;
    o.descriptorMemoryMb = 1;
    o.memoryMb = 16;
    Calibration c = new Calibration(400, 400, 215, 138, new double[5], w, h);
    List<ImageFrame> frames = new ArrayList<>();
    try {
      for (int i = 0; i < 6; i++) {
        ImageFrame f =
            new ImageFrame(
                i, root.resolve("source-" + BufferedImage.TYPE_3BYTE_BGR + ".tif"), c, o, true);
        frames.add(f);
        check(f.gray.empty() && f.width == w && f.height == h, "Metadata-only frame creation");
        f.ensureRasters();
        check(RasterCache.bytes() <= 2 * 1048576, "Raster cache obeys native byte budget");
        f.descriptors.create(700, 128, CvType.CV_32F);
        float[] row = new float[128];
        for (int y = 0; y < 700; y++) {
          for (int x = 0; x < 128; x++) row[x] = (i * 1000 + y + x * .125f);
          f.descriptors.put(y, 0, row);
        }
        DescriptorCache.retain(f);
        check(DescriptorCache.residentBytes() <= 1048576, "Descriptor LRU obeys byte budget");
      }
      check(
          frames.get(0).gray.empty() && frames.get(0).descriptors.empty(),
          "Old rasters/descriptors evicted");
      RasterCache.pair(frames.get(0), frames.get(1));
      DescriptorCache.pair(frames.get(0), frames.get(1));
      for (int i = 0; i < 2; i++) {
        float[] row = new float[128];
        frames.get(i).descriptors.get(433, 0, row);
        boolean exact = true;
        for (int x = 0; x < 128; x++)
          exact &=
              Float.floatToRawIntBits(row[x])
                  == Float.floatToRawIntBits(i * 1000 + 433 + x * .125f);
        check(exact, "Floating descriptor spill/reload is exact");
        check(!frames.get(i).gray.empty(), "Both active pair rasters remain resident");
      }
    } finally {
      for (ImageFrame f : frames) f.close();
    }
    check(
        DescriptorCache.residentBytes() == 0 && RasterCache.bytes() == 0, "Run resources released");
    try (var files = Files.list(root)) {
      check(
          files.noneMatch(p -> p.getFileName().toString().startsWith("feature-descriptors-")),
          "Descriptor spools cleaned");
    }
    return checks;
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    System.out.println(
        "PASS image resource checks=" + run(Path.of("target/scaling/image-resources")));
  }

  private static void checkLongOffset() throws Exception {
    long high = 0x1_0000_1000L;
    byte[] pixels = new byte[73 * 51 * 3];
    for (int p = 0; p < pixels.length; p += 3) {
      pixels[p] = 0x12;
      pixels[p + 1] = 0x34;
      pixels[p + 2] = 0x56;
    }
    var fields =
        java.util.List.of(
            new com.twelvemonkeys.imageio.metadata.tiff.TIFFEntry(256, (short) 4, 73L),
            new com.twelvemonkeys.imageio.metadata.tiff.TIFFEntry(257, (short) 4, 51L),
            new com.twelvemonkeys.imageio.metadata.tiff.TIFFEntry(
                258, (short) 3, new short[] {8, 8, 8}),
            new com.twelvemonkeys.imageio.metadata.tiff.TIFFEntry(259, (short) 3, (short) 1),
            new com.twelvemonkeys.imageio.metadata.tiff.TIFFEntry(262, (short) 3, (short) 2),
            new com.twelvemonkeys.imageio.metadata.tiff.TIFFEntry(273, (short) 16, high),
            new com.twelvemonkeys.imageio.metadata.tiff.TIFFEntry(277, (short) 3, (short) 3),
            new com.twelvemonkeys.imageio.metadata.tiff.TIFFEntry(278, (short) 4, 51L),
            new com.twelvemonkeys.imageio.metadata.tiff.TIFFEntry(
                279, (short) 16, (long) pixels.length));
    var bytes = new java.io.ByteArrayOutputStream();
    try (var output = new javax.imageio.stream.MemoryCacheImageOutputStream(bytes)) {
      new com.twelvemonkeys.imageio.metadata.tiff.TIFFWriter(8).write(fields, output);
      output.flush();
    }
    byte[] header = bytes.toByteArray();
    try (var source =
        new javax.imageio.stream.ImageInputStreamImpl() {
          public long length() {
            return high + pixels.length;
          }

          public int read() throws java.io.IOException {
            byte[] b = new byte[1];
            return read(b, 0, 1) < 0 ? -1 : b[0] & 255;
          }

          public int read(byte[] b, int off, int len) throws java.io.IOException {
            if (streamPos >= length()) return -1;
            int n = (int) Math.min(len, length() - streamPos);
            if (streamPos < header.length) {
              n = Math.min(n, header.length - (int) streamPos);
              System.arraycopy(header, (int) streamPos, b, off, n);
            } else if (streamPos >= high)
              System.arraycopy(pixels, (int) (streamPos - high), b, off, n);
            else {
              n = (int) Math.min(n, high - streamPos);
              Arrays.fill(b, off, off + n, (byte) 0);
            }
            streamPos += n;
            return n;
          }
        }) {
      var decoder = new BigTiffWindows(source);
      BufferedImage tile = decoder.read(new java.awt.Rectangle(60, 40, 10, 10));
      check(
          (tile.getRGB(8, 8) & 0xffffff) == 0x123456,
          "BigTIFF block beyond 4 GiB is read using long offsets without allocating the gap");
      tile.flush();
    }
  }
}
