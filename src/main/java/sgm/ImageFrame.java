package sgm;

import java.io.*;
import java.nio.file.*;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

final class ImageFrame implements AutoCloseable {
  final int id;
  final Path path;
  final Camera camera;
  final Mat color, gray, validMask;
  Mat descriptors = new Mat();
  KeyPoint[] keypoints = new KeyPoint[0];
  double keypointPixelScale = 1;
  final Exif.Gps gps;
  final int sourceWidth, sourceHeight;
  final Options options;
  Calibration calibration;

  final int width, height;
  final boolean managedRasters;
  final boolean bigTiff;
  Path descriptorFile;
  long descriptorBytes;
  // SHA-256 of the retained descriptor rows; identifies identical matching inputs across stages.
  String descriptorDigest;
  String pixelFingerprint;
  byte[] featureColors;

  ImageFrame(int id, Path path, Calibration calibration, Options options) throws IOException {
    this(id, path, calibration, options, false);
  }

  ImageFrame(int id, Path path, Calibration calibration, Options options, boolean managed)
      throws IOException {
    calibration =
        options.orientationCalibrations.getOrDefault(path.getFileName().toString(), calibration);
    this.calibration = calibration;
    this.id = id;
    this.path = path;
    this.options = options;
    this.managedRasters = managed;
    int[] size = ResolutionPlan.originalSize(path);
    sourceWidth = size[0];
    sourceHeight = size[1];
    bigTiff = BigTiffWindows.isBig(path);
    int[] processed = options.processedSize(sourceWidth, sourceHeight);
    width = processed[0];
    height = processed[1];
    camera = calibration.camera(width, height);
    color = new Mat();
    gray = new Mat();
    validMask = new Mat();
    gps = Exif.gps(path);
    if (!managed) loadRasters();
  }

  /** Preserve measured source pixels when changing the pinhole/undistortion frame. */
  void applyCalibration(Calibration next) {
    if (SharedCalibration.same(calibration, next)) return;
    Camera target = next.camera(width, height);
    Point[] raw = new Point[keypoints.length];
    for (int i = 0; i < raw.length; i++)
      raw[i] = SharedCalibration.distorted(camera, calibration.distortion, keypoints[i].pt);
    Point[] corrected = new Point[raw.length];
    double[] d = next.distortion;
    for (int i = 0; i < raw.length; i++) {
      double xd = (raw[i].x - target.cx) / target.fx, yd = (raw[i].y - target.cy) / target.fy;
      double x = xd, y = yd;
      for (int iter = 0; iter < 50; iter++) {
        double r = x * x + y * y, a = 1 + d[0] * r + d[1] * r * r + d[4] * r * r * r;
        double slope = d[0] + 2 * d[1] * r + 3 * d[4] * r * r;
        double u = x * a + 2 * d[2] * x * y + d[3] * (r + 2 * x * x) - xd;
        double v = y * a + d[2] * (r + 2 * y * y) + 2 * d[3] * x * y - yd;
        double xx = a + 2 * x * x * slope + 2 * d[2] * y + 6 * d[3] * x;
        double xy = 2 * x * y * slope + 2 * d[2] * x + 2 * d[3] * y;
        double yy = a + 2 * y * y * slope + 6 * d[2] * y + 2 * d[3] * x;
        double det = xx * yy - xy * xy;
        if (!(det > 1e-12))
          throw new IllegalArgumentException("Non-invertible calibration at tie point");
        double dx = (yy * u - xy * v) / det, dy = (xx * v - xy * u) / det;
        x -= dx;
        y -= dy;
        if (Math.hypot(dx, dy) < 1e-13) break;
      }
      corrected[i] = new Point(target.fx * x + target.cx, target.fy * y + target.cy);
      Point check = SharedCalibration.distorted(target, d, corrected[i]);
      if (!Double.isFinite(check.x + check.y)
          || Math.hypot(check.x - raw[i].x, check.y - raw[i].y) > 1e-6)
        throw new IllegalArgumentException("Calibration inversion did not converge at tie point");
    }
    for (int i = 0; i < keypoints.length; i++) keypoints[i].pt = corrected[i];
    camera.fx = target.fx;
    camera.fy = target.fy;
    camera.cx = target.cx;
    camera.cy = target.cy;
    calibration = next;
    pixelFingerprint = null;
    RasterCache.forget(this);
    releaseRasters();
  }

  /** Colour 3 + gray 1 bytes per pixel, plus 1 for the valid mask when there is one. */
  long rasterBytes() {
    return ((options.rectified || zeroDistortion(calibration)) ? 4L : 5L) * width * height;
  }

  // Set while rasters are loaded without a mask image because every pixel is valid.
  private boolean maskFull;

  /**
   * True when the loaded rasters have no mask image because every pixel is valid (no distortion,
   * or --rectified input): validMask is then empty. Use mask()/maskTo()/maskRow().
   */
  boolean fullMask() {
    return maskFull;
  }

  /** The valid mask; a temporary all-255 image when fullMask() (give it back to releaseMask). */
  Mat mask() {
    return maskFull ? new Mat(height, width, CvType.CV_8U, new Scalar(255)) : validMask;
  }

  void releaseMask(Mat m) {
    if (m != validMask) m.release();
  }

  void maskTo(Mat out) {
    if (!maskFull) {
      validMask.copyTo(out);
      return;
    }
    out.create(height, width, CvType.CV_8U);
    out.setTo(new Scalar(255));
  }

  /** Row y of the valid mask into row[0, width). */
  void maskRow(int y, byte[] row) {
    if (maskFull) java.util.Arrays.fill(row, 0, width, (byte) -1);
    else validMask.get(y, 0, row);
  }

  String sourceReader() {
    return options.imageReader.equals("windowed")
            || options.imageReader.equals("auto")
                && (bigTiff || 3L * sourceWidth * sourceHeight > 64L * 1048576)
        ? "imageio_source_windows"
        : "opencv_full_decode";
  }

  void ensureRasters() throws IOException {
    RasterCache.pair(this, this);
  }

  void releaseRasters() {
    color.release();
    gray.release();
    validMask.release();
    maskFull = false;
  }

  void loadRasters() throws IOException {
    if (!gray.empty()) return;
    Mat resized = new Mat(), loaded = new Mat();
    try (WorkProgress work =
        WorkProgress.start("Decode/resize/undistort " + path.getFileName(), 0, "pixels")) {
      String reader = sourceReader();
      boolean windowed = reader.equals("imageio_source_windows");
      Mat cached = ProcessedImageCache.read(this, reader);
      if (cached != null) {
        resized.release();
        resized = cached;
      } else if (windowed) {
        resized.release();
        resized =
            WindowedImage.read(
                path,
                sourceWidth,
                sourceHeight,
                width,
                height,
                options.imageWindow,
                options.cores,
                WindowedImage.wholeDecodeBytes(options));
      } else {
        loaded =
            Imgcodecs.imread(
                path.toString(), Imgcodecs.IMREAD_COLOR | Imgcodecs.IMREAD_IGNORE_ORIENTATION);
        if (loaded.empty()) throw new IOException("Cannot decode " + path);
        Imgproc.resize(loaded, resized, new Size(width, height), 0, 0, Imgproc.INTER_AREA);
        loaded.release();
      }
      if (cached == null) ProcessedImageCache.write(this, reader, resized);
      Mat k = camera.intrinsic();
      MatOfDouble d = new MatOfDouble(calibration.distortion);
      Mat ones = new Mat();
      try {
        if (options.rectified || zeroDistortion(calibration)) {
          // Every pixel is valid: no mask image is kept (see fullMask()).
          resized.copyTo(color);
          maskFull = true;
        } else {
          ones = new Mat(height, width, CvType.CV_8U, new Scalar(255));
          Calib3d.undistort(resized, color, k, d);
          Calib3d.undistort(ones, validMask, k, d);
          Imgproc.threshold(validMask, validMask, 254, 255, Imgproc.THRESH_BINARY);
        }
      } finally {
        k.release();
        d.release();
        ones.release();
      }
      Imgproc.cvtColor(color, gray, Imgproc.COLOR_BGR2GRAY);
      work.done();
    } catch (RuntimeException | IOException | Error e) {
      releaseRasters();
      throw e;
    } finally {
      loaded.release();
      resized.release();
    }
  }

  void captureFeatureColors() throws IOException {
    ensureRasters();
    featureColors = new byte[keypoints.length * 3];
    byte[] value = new byte[3];
    for (int i = 0; i < keypoints.length; i++) {
      Point p = keypoints[i].pt;
      color.get(
          Math.max(0, Math.min(height - 1, (int) Math.round(p.y))),
          Math.max(0, Math.min(width - 1, (int) Math.round(p.x))),
          value);
      System.arraycopy(value, 0, featureColors, i * 3, 3);
    }
  }

  private static boolean zeroDistortion(Calibration calibration) {
    for (double value : calibration.distortion) if (value != 0) return false;
    return true;
  }

  void extract(int maximum) {
    DescriptorCache.prepare(this);
    try {
      ensureRasters();
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
    if (options.featureGrid > 0) {
      extractGrid(maximum);
      return;
    }
    org.opencv.features2d.Feature2D sift = FeatureAlgorithms.detector(options, maximum);
    MatOfKeyPoint kp = new MatOfKeyPoint();
    Mat mask = new Mat();
    try {
      sift.detectAndCompute(gray, mask, kp, descriptors);
      keypoints = kp.toArray();
      if (keypoints.length > maximum) {
        Integer[] ids = new Integer[keypoints.length];
        for (int i = 0; i < ids.length; i++) ids[i] = i;
        java.util.Arrays.sort(
            ids, (a, b) -> Float.compare(keypoints[b].response, keypoints[a].response));
        KeyPoint[] selected = new KeyPoint[maximum];
        Mat reduced = new Mat(maximum, descriptors.cols(), descriptors.type());
        for (int i = 0; i < maximum; i++) {
          selected[i] = keypoints[ids[i]];
          Mat from = descriptors.row(ids[i]), to = reduced.row(i);
          try {
            from.copyTo(to);
          } finally {
            from.release();
            to.release();
          }
        }
        descriptors.release();
        descriptors = reduced;
        keypoints = selected;
      }
    } finally {
      kp.release();
      mask.release();
      // OpenCV's Java wrappers delete their native object in finalize(), and detectAndCompute
      // passes only the native handle, so without this fence the GC may finalize the detector
      // while the native SIFT is still running (use-after-free; SIGSEGV seen on Linux at ~1,400
      // of 3,000 images).
      java.lang.ref.Reference.reachabilityFence(sift);
    }
  }

  /**
   * --feature-grid: detect every candidate (no strongest-N cap), choose maximum of them spread
   * over the grid (FeatureGrid), then describe only the chosen ones.
   */
  private void extractGrid(int maximum) {
    int candidates = options.matchingAlgorithm.equals("orb") ? Math.multiplyExact(maximum, 4) : 0;
    org.opencv.features2d.Feature2D detector = FeatureAlgorithms.detector(options, candidates);
    MatOfKeyPoint kp = new MatOfKeyPoint();
    Mat mask = new Mat();
    try {
      detector.detect(gray, kp, mask);
      KeyPoint[] chosen =
          FeatureGrid.select(kp.toArray(), width, height, options.featureGrid, maximum);
      kp.fromArray(chosen);
      detector.compute(gray, kp, descriptors);
      keypoints = kp.toArray();
    } finally {
      kp.release();
      mask.release();
      java.lang.ref.Reference.reachabilityFence(detector); // see extract()
    }
  }

  void inheritKeypoints(ImageFrame alignment) {
    double sx = width / (double) alignment.width, sy = height / (double) alignment.height;
    keypointPixelScale = alignment.keypointPixelScale * Math.max(sx, sy);
    keypoints = new KeyPoint[alignment.keypoints.length];
    for (int i = 0; i < keypoints.length; i++) {
      KeyPoint p = alignment.keypoints[i];
      keypoints[i] =
          new KeyPoint(
              (float) ((p.pt.x + .5) * sx - .5),
              (float) ((p.pt.y + .5) * sy - .5),
              (float) (p.size * Math.sqrt(sx * sy)),
              p.angle,
              p.response,
              p.octave,
              p.class_id);
    }
  }

  @Override
  public void close() {
    RasterCache.forget(this);
    releaseRasters();
    try {
      DescriptorCache.forget(this);
    } finally {
      descriptors.release();
    }
  }
}
