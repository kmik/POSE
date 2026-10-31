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
  final Calibration calibration;

  final int width, height;
  final boolean managedRasters;
  final boolean bigTiff;
  Path descriptorFile;
  long descriptorBytes;
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

  long rasterBytes() {
    return 5L * width * height;
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
  }

  void loadRasters() throws IOException {
    if (!gray.empty()) return;
    Mat resized = new Mat(), loaded = new Mat();
    try {
      boolean windowed = sourceReader().equals("imageio_source_windows");
      if (windowed) {
        resized.release();
        resized =
            WindowedImage.read(path, sourceWidth, sourceHeight, width, height, options.imageWindow);
      } else {
        loaded =
            Imgcodecs.imread(
                path.toString(), Imgcodecs.IMREAD_COLOR | Imgcodecs.IMREAD_IGNORE_ORIENTATION);
        if (loaded.empty()) throw new IOException("Cannot decode " + path);
        Imgproc.resize(loaded, resized, new Size(width, height), 0, 0, Imgproc.INTER_AREA);
      }
      Mat k = camera.intrinsic();
      MatOfDouble d = new MatOfDouble(calibration.distortion);
      Mat ones = new Mat(height, width, CvType.CV_8U, new Scalar(255));
      try {
        if (options.rectified) {
          resized.copyTo(color);
          ones.copyTo(validMask);
        } else {
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

  void extract(int maximum) {
    DescriptorCache.prepare(this);
    try {
      ensureRasters();
    } catch (IOException e) {
      throw new UncheckedIOException(e);
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
