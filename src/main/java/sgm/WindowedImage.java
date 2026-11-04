package sgm;

import java.awt.Rectangle;
import java.awt.image.*;
import java.io.*;
import java.nio.file.*;
import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.*;
import javax.imageio.*;
import javax.imageio.stream.*;
import org.opencv.core.*;

/**
 * Source-region decoding and global pixel-area resampling, without a full source raster.
 *
 * <p>Output tiles are independent, so they are resampled on up to {@code threads} workers, each
 * with its own reader, and delivered to the sink in the original tile order. Sequential codecs
 * (JPEG, PNG and other non-TIFF formats) cannot decode a region without decoding every row above
 * it; when such a source fits {@code wholeBytes} it is decoded once and windows are sliced from
 * it. Pixel values, window boundaries and accumulation order are unchanged in every mode.
 */
final class WindowedImage {
  private static final AtomicInteger ACTIVE = new AtomicInteger();

  static Mat read(Path path, int sourceWidth, int sourceHeight, int width, int height, int window)
      throws IOException {
    return read(path, sourceWidth, sourceHeight, width, height, window, 1, 0);
  }

  static Mat read(
      Path path,
      int sourceWidth,
      int sourceHeight,
      int width,
      int height,
      int window,
      int threads,
      long wholeBytes)
      throws IOException {
    Mat out = new Mat(height, width, CvType.CV_8UC3);
    try {
      readTo(
          path,
          sourceWidth,
          sourceHeight,
          width,
          height,
          window,
          threads,
          wholeBytes,
          (x, y, row) -> out.put(y, x, row));
      return out;
    } catch (IOException | RuntimeException | Error e) {
      out.release();
      throw e;
    }
  }

  interface RowSink {
    void write(int x, int y, byte[] bgr) throws IOException;
  }

  static void readTo(
      Path path, int sourceWidth, int sourceHeight, int width, int height, int window, RowSink sink)
      throws IOException {
    readTo(path, sourceWidth, sourceHeight, width, height, window, 1, 0, sink);
  }

  /** Bytes a sequential source may occupy when decoded once instead of once per window row. */
  static long wholeDecodeBytes(Options o) {
    return RasterCache.limit(o) / Math.max(1, o.cores);
  }

  static void readTo(
      Path path,
      int sourceWidth,
      int sourceHeight,
      int width,
      int height,
      int window,
      int threads,
      long wholeBytes,
      RowSink sink)
      throws IOException {
    if (width > sourceWidth || height > sourceHeight)
      throw new IllegalArgumentException("Windowed image loading does not upsample");
    // Concurrent decodes (for example parallel image preloading) share the requested threads.
    int active = ACTIVE.incrementAndGet();
    List<Source> sources = Collections.synchronizedList(new ArrayList<>());
    ExecutorService pool = null;
    try {
      Source first = new Source(path);
      sources.add(first);
      BufferedImage whole = null;
      if (!first.tiff && 4L * sourceWidth * sourceHeight <= wholeBytes) {
        whole = first.reader.read(0);
        if (whole.getWidth() != sourceWidth || whole.getHeight() != sourceHeight)
          throw new IOException("Decoded source size differs from its header: " + path);
        components(whole);
      }
      double sx = sourceWidth / (double) width, sy = sourceHeight / (double) height;
      int tw = Math.max(1, (int) Math.min(window, Math.floor((window - 2) / sx))),
          th = Math.max(1, (int) Math.min(window, Math.floor((window - 2) / sy)));
      int columns = (width + tw - 1) / tw;
      int workers = Math.max(1, Math.min(columns, threads / Math.max(1, active)));
      AtomicInteger largestWindow =
          new AtomicInteger(whole == null ? 0 : sourceWidth * sourceHeight);
      ThreadLocal<Source> local =
          ThreadLocal.withInitial(
              () -> {
                try {
                  Source s = new Source(path);
                  sources.add(s);
                  return s;
                } catch (IOException e) {
                  throw new UncheckedIOException(e);
                }
              });
      if (workers > 1)
        pool =
            Executors.newFixedThreadPool(
                workers,
                r -> {
                  Thread t = new Thread(r, "windowed-image");
                  t.setDaemon(true);
                  return t;
                });
      BufferedImage shared = whole;
      ExecutorService executor = pool;
      for (int y = 0; y < height; y += th) {
        int ch = Math.min(th, height - y), ty = y;
        List<Future<byte[]>> row = new ArrayList<>();
        List<byte[]> inline = new ArrayList<>();
        for (int x = 0; x < width; x += tw) {
          int cw = Math.min(tw, width - x), tx = x;
          Callable<byte[]> task =
              () ->
                  tile(
                      shared != null ? null : executor == null ? first : local.get(),
                      shared,
                      tx,
                      ty,
                      cw,
                      ch,
                      sx,
                      sy,
                      sourceWidth,
                      sourceHeight,
                      window,
                      largestWindow);
          if (executor == null) inline.add(call(task));
          else row.add(executor.submit(task));
        }
        int i = 0;
        for (int x = 0; x < width; x += tw, i++) {
          int cw = Math.min(tw, width - x);
          byte[] values = executor == null ? inline.get(i) : join(row.get(i));
          byte[] bgr = new byte[cw * 3];
          for (int yy = 0; yy < ch; yy++) {
            System.arraycopy(values, yy * bgr.length, bgr, 0, bgr.length);
            sink.write(x, y + yy, bgr);
          }
        }
      }
      System.out.printf(
          Locale.ROOT,
          "Source windows: %s; %dx%d -> %dx%d; max decoded window %d pixels; color=8-bit sRGB%s%n",
          path.getFileName(),
          sourceWidth,
          sourceHeight,
          width,
          height,
          largestWindow.get(),
          whole == null ? "" : "; sequential codec decoded once");
    } finally {
      ACTIVE.decrementAndGet();
      if (pool != null) pool.shutdownNow();
      synchronized (sources) {
        for (Source s : sources) s.close();
      }
    }
  }

  private static byte[] tile(
      Source source,
      BufferedImage whole,
      int x,
      int y,
      int cw,
      int ch,
      double sx,
      double sy,
      int sourceWidth,
      int sourceHeight,
      int window,
      AtomicInteger largestWindow)
      throws IOException {
    double[] sums = new double[Math.multiplyExact(Math.multiplyExact(cw, ch), 3)];
    int left = (int) Math.floor(x * sx),
        top = (int) Math.floor(y * sy),
        right = Math.min(sourceWidth, (int) Math.ceil((x + cw) * sx)),
        bottom = Math.min(sourceHeight, (int) Math.ceil((y + ch) * sy));
    for (int py = top; py < bottom; py += window)
      for (int px = left; px < right; px += window) {
        int rw = Math.min(window, right - px), rh = Math.min(window, bottom - py);
        int[] rgb;
        if (whole != null) rgb = pixels(whole, px, py, rw, rh);
        else {
          rgb = source.window(px, py, rw, rh);
          largestWindow.accumulateAndGet(rw * rh, Math::max);
        }
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
                        * Math.max(0, Math.min(gy + 1, (dy + 1) * sy) - Math.max(gy, dy * sy));
                int at = ((dy - y) * cw + dx - x) * 3;
                sums[at] += (value & 255) * weight;
                sums[at + 1] += ((value >>> 8) & 255) * weight;
                sums[at + 2] += ((value >>> 16) & 255) * weight;
              }
          }
        }
      }
    byte[] out = new byte[sums.length];
    double area = sx * sy;
    for (int i = 0; i < out.length; i++)
      out[i] = (byte) Math.max(0, Math.min(255, (int) Math.rint(sums[i] / area)));
    return out;
  }

  /**
   * The packed values of {@link BufferedImage#getRGB}, read directly for interleaved 8-bit sRGB
   * rasters (JPEG, RGB/RGBA TIFF). Other color models keep the per-pixel color-model conversion.
   */
  static int[] pixels(BufferedImage image, int x, int y, int w, int h) {
    ColorModel cm = image.getColorModel();
    WritableRaster raster = image.getRaster();
    boolean direct =
        cm instanceof ComponentColorModel
            && cm.getColorSpace().isCS_sRGB()
            && cm.getNumColorComponents() == 3
            && !cm.isAlphaPremultiplied()
            && cm.getTransferType() == DataBuffer.TYPE_BYTE
            && raster.getSampleModel() instanceof PixelInterleavedSampleModel
            && raster.getDataBuffer() instanceof DataBufferByte buffer
            && buffer.getNumBanks() == 1;
    if (direct)
      for (int i = 0; i < cm.getNumComponents(); i++) direct &= cm.getComponentSize(i) == 8;
    if (!direct) {
      synchronized (image) {
        return image.getRGB(x, y, w, h, null, 0, w);
      }
    }
    PixelInterleavedSampleModel sm = (PixelInterleavedSampleModel) raster.getSampleModel();
    DataBufferByte buffer = (DataBufferByte) raster.getDataBuffer();
    byte[] data = buffer.getData();
    int[] bands = sm.getBandOffsets();
    int stride = sm.getPixelStride(), scan = sm.getScanlineStride();
    int r = bands[0], g = bands[1], b = bands[2], a = cm.hasAlpha() ? bands[3] : -1;
    int[] out = new int[w * h];
    for (int yy = 0; yy < h; yy++) {
      int at =
          buffer.getOffset()
              + (y + yy - raster.getSampleModelTranslateY()) * scan
              + (x - raster.getSampleModelTranslateX()) * stride;
      for (int xx = 0; xx < w; xx++, at += stride)
        out[yy * w + xx] =
            (a < 0 ? 0xff000000 : (data[at + a] & 255) << 24)
                | (data[at + r] & 255) << 16
                | (data[at + g] & 255) << 8
                | (data[at + b] & 255);
    }
    return out;
  }

  private static void components(BufferedImage image) throws IOException {
    int components = image.getColorModel().getNumColorComponents();
    if (components != 1 && components != 3)
      throw new IOException(
          "Source must be grayscale or RGB; unsupported color components: " + components);
  }

  private static byte[] call(Callable<byte[]> task) throws IOException {
    try {
      return task.call();
    } catch (IOException | RuntimeException e) {
      throw e;
    } catch (Exception e) {
      throw new IOException(e);
    }
  }

  private static byte[] join(Future<byte[]> task) throws IOException {
    try {
      return task.get();
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      throw new IOException("Windowed decode interrupted", e);
    } catch (ExecutionException e) {
      Throwable cause = e.getCause();
      if (cause instanceof UncheckedIOException u) throw u.getCause();
      if (cause instanceof IOException io) throw io;
      if (cause instanceof RuntimeException r) throw r;
      if (cause instanceof Error error) throw error;
      throw new IOException(cause);
    }
  }

  /** One reader and stream; ImageIO readers are not thread-safe. */
  private static final class Source implements Closeable {
    final Path path;
    final ImageInputStream input;
    final ImageReader reader;
    final BigTiffWindows big;
    final boolean tiff;

    Source(Path path) throws IOException {
      this.path = path;
      input = new FileImageInputStream(path.toFile());
      ImageReader opened = null;
      try {
        Iterator<ImageReader> readers = ImageIO.getImageReaders(input);
        if (!readers.hasNext()) throw new IOException("No source-window reader for " + path);
        opened = readers.next();
        big = BigTiffWindows.isBig(path) ? new BigTiffWindows(input) : null;
        tiff = opened.getFormatName().equalsIgnoreCase("TIFF");
        if (tiff) {
          opened.dispose();
          opened = null;
          opened = BigTiffWindows.jdkTiff();
        }
        opened.setInput(input, false, true);
        reader = opened;
      } catch (IOException | RuntimeException | Error e) {
        if (opened != null) opened.dispose();
        input.close();
        throw e;
      }
    }

    int[] window(int px, int py, int rw, int rh) throws IOException {
      ImageReadParam param = reader.getDefaultReadParam();
      param.setSourceRegion(new Rectangle(px, py, rw, rh));
      BufferedImage tile =
          big == null ? reader.read(0, param) : big.read(new Rectangle(px, py, rw, rh));
      if (tile.getWidth() != rw || tile.getHeight() != rh)
        throw new IOException("Image reader ignored requested source window: " + path);
      components(tile);
      int[] rgb = pixels(tile, 0, 0, rw, rh);
      tile.flush();
      return rgb;
    }

    @Override
    public void close() {
      reader.dispose();
      try {
        input.close();
      } catch (IOException ignored) {
      }
    }
  }
}
