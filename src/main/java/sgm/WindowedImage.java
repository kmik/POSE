package sgm;

import java.awt.Rectangle;
import java.awt.image.BufferedImage;
import java.io.*;
import java.nio.file.*;
import java.util.*;
import javax.imageio.*;
import javax.imageio.stream.*;
import org.opencv.core.*;

/** Source-region decoding and global pixel-area resampling, without a full source raster. */
final class WindowedImage {
  static Mat read(Path path, int sourceWidth, int sourceHeight, int width, int height, int window)
      throws IOException {
    if (width > sourceWidth || height > sourceHeight)
      throw new IllegalArgumentException("Windowed image loading does not upsample");
    Mat out = new Mat(height, width, CvType.CV_8UC3);
    try (ImageInputStream input = new FileImageInputStream(path.toFile())) {
      Iterator<ImageReader> readers = ImageIO.getImageReaders(input);
      if (!readers.hasNext()) throw new IOException("No source-window reader for " + path);
      ImageReader reader = readers.next();
      BigTiffWindows big = BigTiffWindows.isBig(path) ? new BigTiffWindows(input) : null;
      if (reader.getFormatName().equalsIgnoreCase("TIFF")) {
        reader.dispose();
        reader = BigTiffWindows.jdkTiff();
      }
      try {
        reader.setInput(input, false, true);
        double sx = sourceWidth / (double) width, sy = sourceHeight / (double) height;
        int tw = Math.max(1, (int) Math.min(window, Math.floor((window - 2) / sx))),
            th = Math.max(1, (int) Math.min(window, Math.floor((window - 2) / sy)));
        long decoded = 0;
        int largestWindow = 0;
        for (int y = 0; y < height; y += th)
          for (int x = 0; x < width; x += tw) {
            int cw = Math.min(tw, width - x), ch = Math.min(th, height - y);
            double[] sums = new double[Math.multiplyExact(Math.multiplyExact(cw, ch), 3)];
            int left = (int) Math.floor(x * sx),
                top = (int) Math.floor(y * sy),
                right = Math.min(sourceWidth, (int) Math.ceil((x + cw) * sx)),
                bottom = Math.min(sourceHeight, (int) Math.ceil((y + ch) * sy));
            for (int py = top; py < bottom; py += window)
              for (int px = left; px < right; px += window) {
                int rw = Math.min(window, right - px), rh = Math.min(window, bottom - py);
                ImageReadParam param = reader.getDefaultReadParam();
                param.setSourceRegion(new Rectangle(px, py, rw, rh));
                BufferedImage tile =
                    big == null ? reader.read(0, param) : big.read(new Rectangle(px, py, rw, rh));
                if (tile.getWidth() != rw || tile.getHeight() != rh)
                  throw new IOException("Image reader ignored requested source window: " + path);
                int components = tile.getColorModel().getNumColorComponents();
                if (components != 1 && components != 3)
                  throw new IOException(
                      "Source must be grayscale or RGB; unsupported color components: "
                          + components);
                int[] rgb = tile.getRGB(0, 0, rw, rh, null, 0, rw);
                decoded += (long) rw * rh;
                largestWindow = Math.max(largestWindow, rw * rh);
                for (int yy = 0; yy < rh; yy++) {
                  int gy = py + yy,
                      dy0 = Math.max(y, (int) Math.floor(gy / sy)),
                      dy1 = Math.min(y + ch - 1, (int) Math.floor((gy + 1 - 1e-9) / sy));
                  for (int xx = 0; xx < rw; xx++) {
                    int gx = px + xx,
                        dx0 = Math.max(x, (int) Math.floor(gx / sx)),
                        dx1 = Math.min(x + cw - 1, (int) Math.floor((gx + 1 - 1e-9) / sx));
                    int value = rgb[yy * rw + xx];
                    for (int dy = dy0; dy <= dy1; dy++)
                      for (int dx = dx0; dx <= dx1; dx++) {
                        double weight =
                            Math.max(0, Math.min(gx + 1, (dx + 1) * sx) - Math.max(gx, dx * sx))
                                * Math.max(
                                    0, Math.min(gy + 1, (dy + 1) * sy) - Math.max(gy, dy * sy));
                        int at = ((dy - y) * cw + dx - x) * 3;
                        sums[at] += (value & 255) * weight;
                        sums[at + 1] += ((value >>> 8) & 255) * weight;
                        sums[at + 2] += ((value >>> 16) & 255) * weight;
                      }
                  }
                }
                tile.flush();
              }
            byte[] row = new byte[cw * 3];
            double area = sx * sy;
            for (int yy = 0; yy < ch; yy++) {
              for (int i = 0; i < row.length; i++)
                row[i] =
                    (byte)
                        Math.max(0, Math.min(255, (int) Math.rint(sums[yy * cw * 3 + i] / area)));
              out.put(y + yy, x, row);
            }
          }
        System.out.printf(
            Locale.ROOT,
            "Source windows: %s; %dx%d -> %dx%d; max decoded window %d pixels; color=8-bit sRGB%n",
            path.getFileName(),
            sourceWidth,
            sourceHeight,
            width,
            height,
            largestWindow);
        return out;
      } finally {
        reader.dispose();
      }
    } catch (IOException | RuntimeException | Error e) {
      out.release();
      throw e;
    }
  }
}
