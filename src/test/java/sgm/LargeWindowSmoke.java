package sgm;

import java.awt.image.BufferedImage;
import java.nio.file.*;
import javax.imageio.*;
import javax.imageio.stream.FileImageOutputStream;

/** Run with -Xmx64m: the 72 MB source RGB raster cannot fit as one heap BufferedImage. */
public final class LargeWindowSmoke {
  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Path root = Files.createDirectories(Path.of("target/scaling/large-window"));
    Path file = root.resolve("large.tif");
    ImageWriter writer = null;
    var writers = ImageIO.getImageWritersByFormatName("TIFF");
    while (writers.hasNext()) {
      var candidate = writers.next();
      if (candidate.getClass().getName().startsWith("com.sun.imageio.plugins.tiff.")) {
        writer = candidate;
        break;
      }
      candidate.dispose();
    }
    if (writer == null) throw new AssertionError("No TIFF empty-image fixture writer");
    try {
      try (var output = new FileImageOutputStream(file.toFile())) {
        writer.setOutput(output);
        ImageWriteParam p = writer.getDefaultWriteParam();
        if (p.canWriteTiles()) {
          p.setTilingMode(ImageWriteParam.MODE_EXPLICIT);
          p.setTiling(256, 256, 0, 0);
        }
        writer.prepareWriteEmpty(
            null,
            ImageTypeSpecifier.createFromBufferedImageType(BufferedImage.TYPE_3BYTE_BGR),
            6000,
            4000,
            null,
            null,
            p);
        writer.endWriteEmpty();
        writer.prepareReplacePixels(0, new java.awt.Rectangle(0, 0, 6000, 4000));
        BufferedImage tile = new BufferedImage(256, 256, BufferedImage.TYPE_3BYTE_BGR);
        ImageWriteParam replace = writer.getDefaultWriteParam();
        for (int y = 0; y < 4000; y += 256)
          for (int x = 0; x < 6000; x += 256) {
            replace.setDestinationOffset(new java.awt.Point(x, y));
            replace.setSourceRegion(
                new java.awt.Rectangle(0, 0, Math.min(256, 6000 - x), Math.min(256, 4000 - y)));
            writer.replacePixels(tile, replace);
          }
        writer.endReplacePixels();
        tile.flush();
      } finally {
        writer.dispose();
      }
      Options o = TestSupport.options();
      o.output = root;
      o.imageReader = "windowed";
      o.maxDimension = 1600;
      var calibration = new Calibration(4000, 4000, 2999.5, 1999.5, new double[5], 6000, 4000);
      Path bigFile = root.resolve("large-big.tif");
      convertToBig(file, bigFile);
      for (Path source : new Path[] {file, bigFile})
        for (boolean nativeSize : new boolean[] {false, true}) {
          o.quality = nativeSize ? "ultra" : "custom";
          try (var f = new ImageFrame(0, source, calibration, o, true)) {
            f.ensureRasters();
            if (f.width != (nativeSize ? 6000 : 1600)
                || f.height != (nativeSize ? 4000 : 1067)
                || f.gray.empty()) throw new AssertionError("Large windowed image dimensions");
            System.out.println(
                "PASS large source window decoding under heap="
                    + Runtime.getRuntime().maxMemory()
                    + "; file="
                    + source.getFileName()
                    + "; source=6000x4000; processed="
                    + f.width
                    + "x"
                    + f.height);
          }
        }
    } finally {
      Files.deleteIfExists(file);
      Files.deleteIfExists(root.resolve("large-big.tif"));
    }
  }

  private static void convertToBig(Path source, Path destination) throws Exception {
    java.util.List<com.twelvemonkeys.imageio.metadata.Entry> fields = new java.util.ArrayList<>();
    try (var input = new javax.imageio.stream.FileImageInputStream(source.toFile())) {
      var directory = new com.twelvemonkeys.imageio.metadata.tiff.TIFFReader().read(input);
      for (var e : directory) fields.add(e);
    }
    Files.copy(source, destination, java.nio.file.StandardCopyOption.REPLACE_EXISTING);
    try (var output = new FileImageOutputStream(destination.toFile())) {
      output.setByteOrder(java.nio.ByteOrder.BIG_ENDIAN);
      output.seek(Files.size(destination));
      long ifd = new com.twelvemonkeys.imageio.metadata.tiff.TIFFWriter(8).writeIFD(fields, output);
      output.seek(0);
      output.writeByte('M');
      output.writeByte('M');
      output.writeShort(43);
      output.writeShort(8);
      output.writeShort(0);
      output.writeLong(ifd);
    }
  }
}
