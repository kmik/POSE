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

  ImageFrame(int id, Path path, Calibration calibration, Options options) throws IOException {
    calibration =
        options.orientationCalibrations.getOrDefault(path.getFileName().toString(), calibration);
    this.calibration = calibration;
    this.id = id;
    this.path = path;
    this.options = options;
    Mat loaded =
        Imgcodecs.imread(
            path.toString(), Imgcodecs.IMREAD_COLOR | Imgcodecs.IMREAD_IGNORE_ORIENTATION);
    if (loaded.empty()) {
      loaded.release();
      throw new IOException("Cannot decode " + path);
    }
    Mat resized = new Mat();
    sourceWidth = loaded.cols();
    sourceHeight = loaded.rows();
    color = new Mat();
    gray = new Mat();
    validMask = new Mat();
    try {
      int[] dimensions = options.processedSize(loaded.cols(), loaded.rows());
      int w = dimensions[0], h = dimensions[1];
      Imgproc.resize(loaded, resized, new Size(w, h), 0, 0, Imgproc.INTER_AREA);
      camera = calibration.camera(w, h);
      Mat k = camera.intrinsic();
      MatOfDouble d = new MatOfDouble(calibration.distortion);
      Mat ones = new Mat(h, w, CvType.CV_8U, new Scalar(255));
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
      gps = Exif.gps(path);
    } catch (RuntimeException | IOException e) {
      color.release();
      gray.release();
      validMask.release();
      descriptors.release();
      throw e;
    } finally {
      loaded.release();
      resized.release();
    }
  }

  void extract(int maximum) {
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
    double sx = gray.cols() / (double) alignment.gray.cols(),
        sy = gray.rows() / (double) alignment.gray.rows();
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
    color.release();
    gray.release();
    validMask.release();
    descriptors.release();
  }
}
