package sgm;

import java.io.*;
import java.util.*;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

/** Run-owned, bounded rasters. Coordinates always refer to the complete processed image. */
final class WindowedRasters implements AutoCloseable {
  final Options options;
  final PagedVolume.Pool pool;
  final List<PagedVolume> files = new ArrayList<>();

  WindowedRasters(Options o) {
    if (!MemorySafety.admits(
        (Math.max(4, Math.min(64, o.memoryMb / 8)) + 96) * 1048576L, o.memoryMb * 1048576L))
      throw new IllegalArgumentException(
          "Windowed raster cache/codec/native staging exceeds safe heap/OS commit allowance");
    options = o.copy();
    options.tileSize = 1024;
    options.disparities = 16;
    pool = new PagedVolume.Pool(Math.max(4, Math.min(64, o.memoryMb / 8)) * 1048576L);
  }

  PagedVolume volume(long n, int initial) throws IOException {
    var v = new PagedVolume(n, 4, initial, true, options, "windowed-", true, pool);
    files.add(v);
    return v;
  }

  final class Image implements DisparityRange.Pixels {
    final int width, height;
    final PagedVolume color, gray, mask;

    Image(int w, int h) throws IOException {
      width = w;
      height = h;
      long n = (long) w * h;
      color = volume(n, 0);
      gray = volume(n, 0);
      mask = volume(n, 0);
    }

    byte[] bytes(PagedVolume v, int y, int rows, int channels) {
      byte[] data = new byte[Math.multiplyExact(width * rows, channels)];
      for (int p = 0; p < width * rows; p++) {
        int value = v.get((long) y * width + p);
        for (int c = 0; c < channels; c++) data[p * channels + c] = (byte) (value >>> (8 * c));
      }
      return data;
    }

    public int width() {
      return width;
    }

    public int height() {
      return height;
    }

    public int mask(int x, int y) {
      return mask.get((long) y * width + x);
    }

    public double zncc(DisparityRange.Pixels other, int x, int y, double disparity, int radius) {
      if (y - radius < 0 || y + radius >= height) return -1;
      var b = (Image) other;
      return CensusSgm.zncc(
          bytes(gray, y - radius, 2 * radius + 1, 1),
          b.bytes(b.gray, y - radius, 2 * radius + 1, 1),
          width,
          2 * radius + 1,
          x,
          radius,
          disparity,
          radius,
          1);
    }

    public void resize(Mat out, Size size, boolean validity) {
      // The bounded preview fast path also retains OpenCV's exact area-rounding behavior.
      if ((long) width * height <= 262144) {
        Mat input = new Mat(height, width, CvType.CV_8U);
        try {
          input.put(0, 0, bytes(validity ? mask : gray, 0, height, 1));
          Imgproc.resize(
              input, out, size, 0, 0, validity ? Imgproc.INTER_NEAREST : Imgproc.INTER_AREA);
        } finally {
          input.release();
        }
        return;
      }
      int dw = (int) size.width, dh = (int) size.height;
      byte[] data = new byte[dw * dh];
      double sx = width / (double) dw, sy = height / (double) dh;
      for (int y = 0; y < dh; y++)
        for (int x = 0; x < dw; x++) {
          if (validity) {
            data[y * dw + x] = (byte) mask.get((long) (int) (y * sy) * width + (int) (x * sx));
            continue;
          }
          double sum = 0;
          for (int yy = (int) Math.floor(y * sy); yy < Math.ceil((y + 1) * sy); yy++)
            for (int xx = (int) Math.floor(x * sx); xx < Math.ceil((x + 1) * sx); xx++)
              if (xx >= 0 && yy >= 0 && xx < width && yy < height)
                sum +=
                    gray.get((long) yy * width + xx)
                        * Math.max(0, Math.min(xx + 1, (x + 1) * sx) - Math.max(xx, x * sx))
                        * Math.max(0, Math.min(yy + 1, (y + 1) * sy) - Math.max(yy, y * sy));
          data[y * dw + x] = (byte) Math.max(0, Math.min(255, (int) Math.rint(sum / (sx * sy))));
        }
      out.create(dh, dw, CvType.CV_8U);
      out.put(0, 0, data);
    }

    void discard() throws IOException {
      for (var v : new PagedVolume[] {color, gray, mask}) {
        v.close();
        files.remove(v);
      }
    }
  }

  Image source(ImageFrame frame) throws IOException {
    Image raw = new Image(frame.width, frame.height);
    try (var work =
        WorkProgress.start("Windowed source " + frame.path.getFileName(), frame.height, "rows")) {
      WindowedImage.readTo(
          frame.path,
          frame.sourceWidth,
          frame.sourceHeight,
          frame.width,
          frame.height,
          Math.min(1024, options.imageWindow),
          options.cores,
          WindowedImage.wholeDecodeBytes(options),
          (x, y, row) -> {
            Mat color = new Mat(1, row.length / 3, CvType.CV_8UC3), gray = new Mat();
            try {
              color.put(0, 0, row);
              Imgproc.cvtColor(color, gray, Imgproc.COLOR_BGR2GRAY);
              byte[] g = new byte[row.length / 3];
              gray.get(0, 0, g);
              for (int i = 0; i < g.length; i++) {
                long at = (long) y * frame.width + x + i;
                raw.color.set(
                    at,
                    (row[i * 3] & 255)
                        | (row[i * 3 + 1] & 255) << 8
                        | (row[i * 3 + 2] & 255) << 16);
                raw.gray.set(at, g[i] & 255);
                raw.mask.set(at, 255);
              }
              work.update(y);
            } finally {
              color.release();
              gray.release();
            }
          });
      work.done();
    }
    if (options.rectified || Arrays.stream(frame.calibration.distortion).allMatch(x -> x == 0))
      return raw;
    double[] p = {
      frame.camera.fx, 0, frame.camera.cx, 0, 0, frame.camera.fy, frame.camera.cy, 0, 0, 0, 1, 0
    };
    Image undistorted =
        warp(raw, frame.camera, frame.calibration.distortion, Math3.identity(), p, false, true);
    raw.discard();
    return undistorted;
  }

  Image warp(
      Image input,
      Camera camera,
      double[] distortion,
      double[] rotation,
      double[] projection,
      boolean transpose,
      boolean undistortion)
      throws IOException {
    Image out =
        new Image(transpose ? input.height : input.width, transpose ? input.width : input.height);
    Mat k = camera.intrinsic(), d = new MatOfDouble(distortion), r = Math3.mat(rotation, 3, 3);
    try (var work =
        WorkProgress.start(
            undistortion ? "Windowed undistortion" : "Windowed rectification",
            input.height,
            "rows")) {
      for (int y = 0; y < input.height; y += 128) {
        for (int x = 0; x < input.width; x += 128)
          tile(
              input,
              out,
              k,
              d,
              r,
              projection,
              x,
              y,
              Math.min(128, input.width - x),
              Math.min(128, input.height - y),
              transpose,
              undistortion);
        work.update(Math.min(input.height, y + 128));
      }
      work.done();
    } finally {
      k.release();
      d.release();
      r.release();
    }
    return out;
  }

  private void tile(
      Image input,
      Image out,
      Mat k,
      Mat d,
      Mat r,
      double[] projection,
      int x,
      int y,
      int w,
      int h,
      boolean transpose,
      boolean undistortion)
      throws IOException {
    double[] local = projection.clone();
    local[2] -= x;
    local[6] -= y;
    Mat p = Math3.mat(local, 3, 4), mx = new Mat(), my = new Mat();
    Mat src = new Mat(),
        sg = new Mat(),
        sm = new Mat(),
        dst = new Mat(),
        dg = new Mat(),
        dm = new Mat();
    try {
      if (undistortion) {
        // Calib3d.undistort uses fixed-point maps. Quantize directly from doubles, before
        // converting to float; rounding a float map first changes half-bin edge pixels.
        Mat xy = new Mat(), fraction = new Mat();
        try {
          Calib3d.initUndistortRectifyMap(
              k, d, r, p, new Size(w, h), CvType.CV_16SC2, xy, fraction);
          Imgproc.convertMaps(xy, fraction, mx, my, CvType.CV_32FC1);
        } finally {
          xy.release();
          fraction.release();
        }
      } else Calib3d.initUndistortRectifyMap(k, d, r, p, new Size(w, h), CvType.CV_32FC1, mx, my);
      float[] xs = new float[w * h], ys = new float[w * h];
      mx.get(0, 0, xs);
      my.get(0, 0, ys);
      int x0 = input.width, y0 = input.height, x1 = -1, y1 = -1;
      for (int i = 0; i < xs.length; i++) {
        if (!Float.isFinite(xs[i] + ys[i])
            || xs[i] < -2
            || ys[i] < -2
            || xs[i] >= input.width + 1
            || ys[i] >= input.height + 1) continue;
        x0 = Math.min(x0, Math.max(0, (int) Math.floor(xs[i]) - 1));
        y0 = Math.min(y0, Math.max(0, (int) Math.floor(ys[i]) - 1));
        x1 = Math.max(x1, Math.min(input.width - 1, (int) Math.ceil(xs[i]) + 1));
        y1 = Math.max(y1, Math.min(input.height - 1, (int) Math.ceil(ys[i]) + 1));
      }
      if (x1 < x0 || y1 < y0) return;
      int sw = x1 - x0 + 1, sh = y1 - y0 + 1;
      if (5L * sw * sh > Math.max(1048576L, options.memoryMb * 1048576L / 32)) {
        if (w == 1 && h == 1) throw new IOException("Unbounded rectification footprint");
        if (w >= h && w > 1) {
          int half = w / 2;
          tile(input, out, k, d, r, projection, x, y, half, h, transpose, undistortion);
          tile(input, out, k, d, r, projection, x + half, y, w - half, h, transpose, undistortion);
        } else {
          int half = h / 2;
          tile(input, out, k, d, r, projection, x, y, w, half, transpose, undistortion);
          tile(input, out, k, d, r, projection, x, y + half, w, h - half, transpose, undistortion);
        }
        return;
      }
      byte[] rgb = new byte[sw * sh * 3], gray = new byte[sw * sh], mask = new byte[sw * sh];
      for (int yy = 0; yy < sh; yy++)
        for (int xx = 0; xx < sw; xx++) {
          int at = yy * sw + xx;
          long source = (long) (y0 + yy) * input.width + x0 + xx;
          int color = input.color.get(source);
          rgb[at * 3] = (byte) color;
          rgb[at * 3 + 1] = (byte) (color >>> 8);
          rgb[at * 3 + 2] = (byte) (color >>> 16);
          gray[at] = (byte) input.gray.get(source);
          mask[at] = (byte) input.mask.get(source);
        }
      for (int i = 0; i < xs.length; i++) {
        xs[i] -= x0;
        ys[i] -= y0;
      }
      mx.put(0, 0, xs);
      my.put(0, 0, ys);
      src.create(sh, sw, CvType.CV_8UC3);
      sg.create(sh, sw, CvType.CV_8U);
      sm.create(sh, sw, CvType.CV_8U);
      src.put(0, 0, rgb);
      sg.put(0, 0, gray);
      sm.put(0, 0, mask);
      Imgproc.remap(src, dst, mx, my, Imgproc.INTER_LINEAR);
      if (undistortion) Imgproc.cvtColor(dst, dg, Imgproc.COLOR_BGR2GRAY);
      else Imgproc.remap(sg, dg, mx, my, Imgproc.INTER_LINEAR);
      Imgproc.remap(sm, dm, mx, my, Imgproc.INTER_LINEAR);
      rgb = new byte[w * h * 3];
      gray = new byte[w * h];
      mask = new byte[w * h];
      dst.get(0, 0, rgb);
      dg.get(0, 0, gray);
      dm.get(0, 0, mask);
      for (int yy = 0; yy < h; yy++)
        for (int xx = 0; xx < w; xx++) {
          int at = yy * w + xx;
          long target =
              transpose
                  ? (long) (x + xx) * out.width + y + yy
                  : (long) (y + yy) * out.width + x + xx;
          out.color.set(
              target,
              (rgb[at * 3] & 255) | (rgb[at * 3 + 1] & 255) << 8 | (rgb[at * 3 + 2] & 255) << 16);
          out.gray.set(target, gray[at] & 255);
          out.mask.set(target, (mask[at] & 255) > 254 ? 255 : 0);
        }
    } finally {
      for (Mat m : new Mat[] {p, mx, my, src, sg, sm, dst, dg, dm}) m.release();
    }
  }

  void erode(Image image) throws IOException {
    var original = volume((long) image.width * image.height, 0);
    for (long p = 0; p < (long) image.width * image.height; p++) original.set(p, image.mask.get(p));
    int r = (Math.max(options.census, options.correlationWindow) + 2) / 2;
    for (int y = 0; y < image.height; y++)
      for (int x = 0; x < image.width; x++) {
        boolean valid = x >= r && y >= r && x + r < image.width && y + r < image.height;
        for (int yy = y - r; valid && yy <= y + r; yy++)
          for (int xx = x - r; xx <= x + r; xx++)
            if (original.get((long) yy * image.width + xx) == 0) {
              valid = false;
              break;
            }
        image.mask.set((long) y * image.width + x, valid ? 255 : 0);
      }
    original.close();
    files.remove(original);
  }

  public void close() throws IOException {
    IOException failure = null;
    for (var v : files)
      try {
        v.close();
      } catch (IOException e) {
        if (failure == null) failure = e;
        else failure.addSuppressed(e);
      }
    pool.closeReaders();
    if (failure != null) throw failure;
  }
}
