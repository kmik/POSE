package sgm;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.*;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

final class Calibration {
  final double fx, fy, cx, cy;
  final double[] distortion;
  final int width, height;

  Calibration(double fx, double fy, double cx, double cy, double[] d, int width, int height) {
    if (!(fx > 0 && fy > 0) || !Double.isFinite(fx + fy + cx + cy) || width < 1 || height < 1)
      throw new IllegalArgumentException("Invalid intrinsic calibration");
    for (double v : d)
      if (!Double.isFinite(v))
        throw new IllegalArgumentException("Non-finite distortion coefficient");
    this.fx = fx;
    this.fy = fy;
    this.cx = cx;
    this.cy = cy;
    this.distortion = d;
    this.width = width;
    this.height = height;
  }

  static Calibration load(Options o, int width, int height) throws IOException {
    if (o.calibration == null) {
      double focalPixels = o.focal / o.sensorWidth * width;
      return new Calibration(
          focalPixels,
          focalPixels,
          (width - 1) / 2.0,
          (height - 1) / 2.0,
          new double[5],
          width,
          height);
    }
    String text =
        Files.readString(o.calibration, StandardCharsets.UTF_8).replaceFirst("^\ufeff", "");
    if (text.contains("=")) {
      Properties p = new Properties();
      p.load(new StringReader(text));
      double[] d = new double[5];
      String[] keys = {"k1", "k2", "p1", "p2", "k3"};
      for (int i = 0; i < 5; i++) d[i] = Double.parseDouble(p.getProperty(keys[i], "0"));
      return new Calibration(
          Double.parseDouble(p.getProperty("fx")),
          Double.parseDouble(p.getProperty("fy")),
          Double.parseDouble(p.getProperty("cx")),
          Double.parseDouble(p.getProperty("cy")),
          d,
          Integer.parseInt(p.getProperty("width", String.valueOf(width))),
          Integer.parseInt(p.getProperty("height", String.valueOf(height))));
    }
    String[] tokens = text.trim().split("[\\s,]+");
    if (tokens.length != 14)
      throw new IllegalArgumentException(
          "Calibration must be properties or legacy 14 numbers: 5 OpenCV distortion coefficients,"
              + " then row-major K");
    double[] v = new double[14];
    for (int i = 0; i < 14; i++) v[i] = Double.parseDouble(tokens[i]);
    if (Math.abs(v[6]) + Math.abs(v[8]) + Math.abs(v[11]) + Math.abs(v[12]) > 1e-9
        || Math.abs(v[13] - 1) > 1e-9)
      throw new IllegalArgumentException("Legacy K must have zero skew and last row 0 0 1");
    return new Calibration(v[5], v[9], v[7], v[10], Arrays.copyOf(v, 5), width, height);
  }

  Camera camera(int w, int h) {
    double sx = (double) w / width, sy = (double) h / height;
    if (Math.abs(sx - sy) > Math.max(sx, sy) * .01)
      throw new IllegalArgumentException("Calibration/image aspect ratios differ");
    // Imgproc.resize maps destination pixel centres back through (u+.5)/scale-.5.
    return new Camera(fx * sx, fy * sy, (cx + .5) * sx - .5, (cy + .5) * sy - .5);
  }

  void write(Path path, String comment) throws IOException {
    Properties p = new Properties();
    p.setProperty("width", String.valueOf(width));
    p.setProperty("height", String.valueOf(height));
    p.setProperty("fx", String.valueOf(fx));
    p.setProperty("fy", String.valueOf(fy));
    p.setProperty("cx", String.valueOf(cx));
    p.setProperty("cy", String.valueOf(cy));
    String[] keys = {"k1", "k2", "p1", "p2", "k3"};
    for (int i = 0; i < 5; i++) p.setProperty(keys[i], String.valueOf(distortion[i]));
    try (Writer w = Files.newBufferedWriter(path)) {
      p.store(w, comment);
    }
  }

  static void calibrate(Options o) throws IOException {
    calibrate(o, null);
  }

  static void calibrate(Options o, Project project) throws IOException {
    if (project != null) {
      project.stage("00_inputs", "running", "Calibration image inventory");
      project.stage("01_calibration", "running", "Chessboard detections and intrinsic fitting");
    }
    List<Mat> objects = new ArrayList<>(), observations = new ArrayList<>();
    Mat k = Mat.eye(3, 3, CvType.CV_64F);
    MatOfDouble distortion = new MatOfDouble();
    List<Mat> rotations = new ArrayList<>(), translations = new ArrayList<>();
    Size size = null;
    List<Integer> detectedIds = new ArrayList<>();
    int imageId = 0;
    try {
      Point3[] board = new Point3[o.boardColumns * o.boardRows];
      for (int y = 0; y < o.boardRows; y++)
        for (int x = 0; x < o.boardColumns; x++)
          board[y * o.boardColumns + x] = new Point3(x * o.squareSize, y * o.squareSize, 0);
      for (Path p : Main.images(o)) {
        Mat image =
            Imgcodecs.imread(
                p.toString(), Imgcodecs.IMREAD_GRAYSCALE | Imgcodecs.IMREAD_IGNORE_ORIENTATION);
        MatOfPoint2f corners = new MatOfPoint2f();
        try {
          if (image.empty()) throw new IOException("Cannot decode " + p);
          if (size == null) size = image.size();
          if (image.cols() != size.width || image.rows() != size.height)
            throw new IllegalArgumentException("Calibration images must have equal dimensions");
          boolean detected =
              Calib3d.findChessboardCorners(
                  image,
                  new Size(o.boardColumns, o.boardRows),
                  corners,
                  Calib3d.CALIB_CB_ADAPTIVE_THRESH | Calib3d.CALIB_CB_NORMALIZE_IMAGE);
          if (detected) {
            Imgproc.cornerSubPix(
                image,
                corners,
                new Size(11, 11),
                new Size(-1, -1),
                new TermCriteria(TermCriteria.COUNT | TermCriteria.EPS, 40, .001));
            objects.add(new MatOfPoint3f(board));
            observations.add(corners.clone());
            detectedIds.add(imageId);
          }
          if (project != null)
            project.data.calibrationView(imageId, p, image, corners, detected, board);
          imageId++;
        } finally {
          image.release();
          corners.release();
        }
      }
      if (objects.size() < 3)
        throw new IllegalArgumentException(
            "At least 3 detected chessboards required; use diverse views (10+ recommended)");
      double rms =
          Calib3d.calibrateCamera(
              objects, observations, size, k, distortion, rotations, translations);
      double[] a = Math3.array(k), d = distortion.toArray();
      if (project != null)
        project.data.calibrationResult(
            detectedIds, objects, observations, rotations, translations, k, distortion, size, rms);
      Properties p = new Properties();
      p.setProperty("width", String.valueOf((int) size.width));
      p.setProperty("height", String.valueOf((int) size.height));
      p.setProperty("fx", String.valueOf(a[0]));
      p.setProperty("fy", String.valueOf(a[4]));
      p.setProperty("cx", String.valueOf(a[2]));
      p.setProperty("cy", String.valueOf(a[5]));
      String[] names = {"k1", "k2", "p1", "p2", "k3"};
      for (int i = 0; i < 5; i++) p.setProperty(names[i], String.valueOf(d[i]));
      try (Writer w = Files.newBufferedWriter(o.output.resolve("camera.txt"))) {
        p.store(w, "OpenCV calibration; RMS pixels=" + rms);
      }
      Files.writeString(
          o.output.resolve("report.txt"),
          "Calibration views=" + objects.size() + "\nRMS pixels=" + rms + "\n");
      System.out.printf(Locale.ROOT, "Calibrated %d views; RMS %.4f pixels%n", objects.size(), rms);
      if (project != null) {
        project.stage("00_inputs", "complete", imageId + " calibration images inventoried");
        project.stage("01_calibration", "complete", objects.size() + " board views; RMS=" + rms);
        project.stage("07_results", "running", "Calibrated camera and report");
      }
    } finally {
      for (Mat m : objects) m.release();
      for (Mat m : observations) m.release();
      for (Mat m : rotations) m.release();
      for (Mat m : translations) m.release();
      k.release();
      distortion.release();
    }
  }
}
