package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.features2d.DescriptorMatcher;

/** Shared-focal self calibration from uncalibrated epipolar geometry, with EXIF fallback. */
final class AutomaticCalibration {
  static final double LOW = .12, HIGH = 4;
  static final String ASSUMPTIONS =
      "Shared fixed-focus camera, square pixels, zero skew, centered principal point. Guarded joint"
          + " f,k1,k2 refinement follows focal initialization; p1,p2,k3 stay zero. Rejected"
          + " refinement retains initial focal and zero distortion; see refinement/result.json.";

  static Calibration estimate(Options o, List<Path> paths, int width, int height, Project project)
      throws IOException {
    Core.setRNGSeed(1);
    Path dir =
        project == null
            ? o.output.resolve("01_calibration")
            : project.data.directory("01_calibration");
    Files.createDirectories(dir);
    if (project != null)
      project.stage(
          "01_calibration", "running", "Uncalibrated feature matching and shared focal estimation");
    List<Double> priors = new ArrayList<>();
    Set<String> cameras = new HashSet<>();
    List<Object> metadata = new ArrayList<>();
    for (Path path : paths) {
      Exif.Metadata m = Exif.read(path);
      if (!m.model.isBlank()) cameras.add(m.make.trim() + "/" + m.model.trim());
      if (Double.isFinite(m.focal35Mm) && m.focal35Mm > 0)
        priors.add(m.focal35Mm / Math.hypot(36, 24));
      metadata.add(
          Json.object(
              "filename",
              path.getFileName().toString(),
              "make",
              m.make,
              "model",
              m.model,
              "focal_mm",
              m.focalMm,
              "focal_35mm_equivalent_mm",
              m.focal35Mm));
    }
    double prior = priors.isEmpty() ? Double.NaN : median(priors);
    if (cameras.size() > 1)
      throw new IllegalArgumentException(
          "Automatic shared calibration requires one camera model; separate the camera groups or"
              + " provide calibration.");
    for (double p : priors)
      if (Math.abs(Math.log(p / prior)) > .08)
        throw new IllegalArgumentException(
            "EXIF indicates changing focal lengths. Separate cameras/zoom settings or supply an"
                + " appropriate shared calibration.");
    Json.write(dir.resolve("exif_intrinsics.json"), metadata);
    List<Model> models = new ArrayList<>();
    List<ImageFrame> frames = new ArrayList<>();
    List<Object> pairReports = new ArrayList<>();
    double diagonal = Math.hypot(width, height);
    try {
      Calibration initial =
          new Calibration(
              diagonal,
              diagonal,
              (width - 1) / 2.,
              (height - 1) / 2.,
              new double[5],
              width,
              height);
      for (Path p : paths) {
        ImageFrame f = new ImageFrame(frames.size(), p, initial, o.alignmentOptions());
        frames.add(f);
        if (project != null) project.data.source(f.id, p, f.sourceWidth, f.sourceHeight, f.gps);
        f.extract(o.features);
        System.out.println(
            "Auto calibration features "
                + frames.size()
                + "/"
                + paths.size()
                + ": "
                + f.keypoints.length);
      }
      for (int a = 0; a < frames.size(); a++)
        for (int b = a + 1; b < Math.min(frames.size(), a + o.neighbors + 1); b++) {
          Model m = match(frames.get(a), frames.get(b), o, dir);
          pairReports.add(m.report());
          if (m.accepted) models.add(m);
        }
    } finally {
      for (ImageFrame f : frames) f.close();
    }
    Json.write(dir.resolve("self_calibration_pairs.json"), pairReports);
    double ratio = Double.NaN;
    String method, note;
    if (!models.isEmpty()) {
      List<Double> values = new ArrayList<>();
      for (Model m : models) values.add(m.focal);
      double middle = median(values);
      List<Model> consistent = new ArrayList<>();
      for (Model m : models) if (Math.abs(Math.log(m.focal / middle)) < .25) consistent.add(m);
      if (consistent.size() * 2 < models.size()) models.clear();
      else ratio = minimize(q -> combined(consistent, q));
    }
    if (Double.isFinite(ratio) && (Double.isNaN(prior) || Math.abs(Math.log(ratio / prior)) < .5)) {
      method = "image_matches_shared_focal";
      note =
          "Focal length estimated by enforcing equal nonzero singular values of K^T F K across"
              + " non-planar image pairs. EXIF is a consistency check when available.";
    } else if (Double.isFinite(prior)) {
      ratio = prior;
      method = "exif_35mm_equivalent";
      note =
          "Image geometry did not provide a reliable focal estimate; used the EXIF 35mm-equivalent"
              + " focal length and image diagonal. Raw focal length in mm alone cannot determine"
              + " sensor dimensions.";
    } else {
      Json.write(
          dir.resolve("self_calibration.json"),
          Json.object(
              "status",
              "unobservable",
              "assumptions",
              ASSUMPTIONS,
              "usable_pairs",
              models.size(),
              "reason",
              "No reliable focal constraint and no EXIF 35mm-equivalent focal length"));
      throw new IllegalArgumentException(
          "Automatic calibration could not determine focal length (weak/planar/pure-translation"
              + " geometry or insufficient matches). Supply --calibration FILE or --focal MM --s_w"
              + " MM, or use more varied overlapping views.");
    }
    double focal = ratio * diagonal;
    Calibration initial =
        new Calibration(
            focal, focal, (width - 1) / 2., (height - 1) / 2., new double[5], width, height);
    System.out.printf(Locale.ROOT, "Initial automatic focal: %.3f pixels; %s%n", focal, method);
    Calibration result = SelfCalibration.refine(initial, paths, o, project);
    Json.write(
        dir.resolve("self_calibration.json"),
        Json.object(
            "status",
            "estimated",
            "method",
            result == initial ? method : "joint_self_calibration",
            "initial_method",
            method,
            "reference_width",
            width,
            "reference_height",
            height,
            "focal_pixels",
            result.fx,
            "initial_focal_pixels",
            focal,
            "distortion",
            result.distortion,
            "focal_over_diagonal",
            result.fx / diagonal,
            "initial_focal_over_diagonal",
            ratio,
            "exif_focal_over_diagonal",
            prior,
            "usable_pairs",
            models.size(),
            "assumptions",
            ASSUMPTIONS,
            "note",
            note));
    result.write(
        o.output.resolve("camera.txt"), "Automatic calibration: " + method + "; " + ASSUMPTIONS);
    System.out.printf(
        Locale.ROOT,
        "Automatic calibration: f=%.3f pixels; k1=%.7f, k2=%.7f; initial method=%s%n%s%n",
        result.fx,
        result.distortion[0],
        result.distortion[1],
        method,
        ASSUMPTIONS);
    return result;
  }

  private static Model match(ImageFrame a, ImageFrame b, Options o, Path dir) throws IOException {
    Model out = new Model(a.id, b.id);
    if (a.descriptors.empty() || b.descriptors.rows() < 2 || a.descriptors.rows() < 2) {
      out.reason = "missing_descriptors";
      return out;
    }
    DescriptorMatcher matcher = FeatureAlgorithms.matcher(o, a.descriptors);
    List<MatOfDMatch> fw = new ArrayList<>(), bw = new ArrayList<>();
    MatOfPoint2f x = new MatOfPoint2f(), y = new MatOfPoint2f();
    Mat f = new Mat(), mask = new Mat(), h = new Mat(), hMask = new Mat();
    try {
      matcher.knnMatch(a.descriptors, b.descriptors, fw, 2);
      matcher.knnMatch(b.descriptors, a.descriptors, bw, 2);
      Map<Integer, Integer> reverse = new HashMap<>();
      for (MatOfDMatch row : bw) {
        DMatch[] v = row.toArray();
        if (v.length >= 2 && v[0].distance < o.ratio * v[1].distance)
          reverse.put(v[0].queryIdx, v[0].trainIdx);
      }
      List<Point> p = new ArrayList<>(), q = new ArrayList<>();
      for (MatOfDMatch row : fw) {
        DMatch[] v = row.toArray();
        if (v.length < 2
            || v[0].distance >= o.ratio * v[1].distance
            || !Objects.equals(reverse.get(v[0].trainIdx), v[0].queryIdx)) continue;
        p.add(centered(a, a.keypoints[v[0].queryIdx].pt));
        q.add(centered(b, b.keypoints[v[0].trainIdx].pt));
      }
      out.matches = p.size();
      if (p.size() < Math.max(24, o.minMatches)) {
        out.reason = "insufficient_matches";
        return out;
      }
      x.fromList(p);
      y.fromList(q);
      double threshold = Math.min(o.reprojection, 1) / Math.hypot(a.gray.cols(), a.gray.rows());
      f.release();
      f = Calib3d.findFundamentalMat(x, y, Calib3d.FM_RANSAC, threshold, .999, 5000, mask);
      if (f.empty() || f.rows() != 3) {
        out.reason = "fundamental_failed";
        return out;
      }
      out.inliers = Core.countNonZero(mask);
      List<Point> ip = new ArrayList<>(), iq = new ArrayList<>();
      byte[] flags = new byte[(int) mask.total()];
      mask.get(0, 0, flags);
      Path pair = dir.resolve(ProjectArtifacts.pairId(a.id, b.id));
      Files.createDirectories(pair);
      try (BufferedWriter w = Files.newBufferedWriter(pair.resolve("uncalibrated_matches.csv"))) {
        w.write(
            "a_centered_x_over_diagonal,a_centered_y_over_diagonal,b_centered_x_over_diagonal,b_centered_y_over_diagonal,fundamental_inlier\n");
        for (int i = 0; i < p.size(); i++) {
          if (flags[i] != 0) {
            ip.add(p.get(i));
            iq.add(q.get(i));
          }
          w.write(
              p.get(i).x
                  + ","
                  + p.get(i).y
                  + ","
                  + q.get(i).x
                  + ","
                  + q.get(i).y
                  + ","
                  + (flags[i] != 0)
                  + "\n");
        }
      }
      if (out.inliers < Math.max(24, o.minMatches)) {
        out.reason = "insufficient_fundamental_inliers";
        return out;
      }
      x.fromList(ip);
      y.fromList(iq);
      // Refitting only RANSAC inliers reduces focal bias from a single minimal hypothesis.
      f.release();
      f = Calib3d.findFundamentalMat(x, y, Calib3d.FM_8POINT);
      if (f.empty()) {
        out.reason = "fundamental_refit_failed";
        return out;
      }
      out.fundamental = Math3.array(f);
      h.release();
      h = Calib3d.findHomography(x, y, Calib3d.RANSAC, threshold * 2, hMask, 3000, .999);
      out.homographyFraction = h.empty() ? 0 : Core.countNonZero(hMask) / (double) ip.size();
      out.focal = focalFromFundamental(out.fundamental);
      out.score = essentialError(out.fundamental, out.focal);
      out.curvature =
          Math.min(
                  essentialError(out.fundamental, out.focal * .75),
                  essentialError(out.fundamental, out.focal / .75))
              - out.score;
      out.accepted =
          out.homographyFraction < .9
              && out.focal > LOW * 1.05
              && out.focal < HIGH / 1.05
              && out.curvature > Math.max(1e-6, out.score * .25)
              && out.score < .0025;
      out.reason = out.accepted ? "accepted" : "planar_or_weak_focal_constraint";
      Json.write(pair.resolve("fundamental.json"), out.report());
      try (BufferedWriter w = Files.newBufferedWriter(pair.resolve("focal_search.csv"))) {
        w.write("focal_over_diagonal,essential_singular_value_error\n");
        for (int i = 0; i <= 100; i++) {
          double r = LOW * Math.pow(HIGH / LOW, i / 100.);
          w.write(r + "," + essentialError(out.fundamental, r) + "\n");
        }
      }
      return out;
    } finally {
      for (Mat m : fw) m.release();
      for (Mat m : bw) m.release();
      matcher.clear();
      x.release();
      y.release();
      f.release();
      mask.release();
      h.release();
      hMask.release();
    }
  }

  private static Point centered(ImageFrame f, Point p) {
    double d = Math.hypot(f.sourceWidth, f.sourceHeight);
    return new Point(
        (p.x * f.sourceWidth / f.gray.cols() - (f.sourceWidth - 1) / 2.) / d,
        (p.y * f.sourceHeight / f.gray.rows() - (f.sourceHeight - 1) / 2.) / d);
  }

  static double focalFromFundamental(double[] f) {
    return minimize(q -> essentialError(f, q));
  }

  static double essentialError(double[] f, double focal) {
    double[] e = f.clone(), k = {focal, focal, 1};
    for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) e[3 * r + c] *= k[r] * k[c];
    Mat m = Math3.mat(e, 3, 3), w = new Mat(), u = new Mat(), vt = new Mat();
    try {
      Core.SVDecomp(m, w, u, vt);
      double[] s = Math3.array(w);
      return Math.pow((s[0] - s[1]) / Math.max(1e-30, s[0] + s[1]), 2);
    } finally {
      m.release();
      w.release();
      u.release();
      vt.release();
    }
  }

  private static double combined(List<Model> models, double q) {
    double sum = 0, weight = 0;
    for (Model m : models) {
      double w = Math.min(500, m.inliers);
      sum += w * essentialError(m.fundamental, q);
      weight += w;
    }
    return sum / weight;
  }

  private interface Objective {
    double at(double q);
  }

  private static double minimize(Objective objective) {
    double lo = Math.log(LOW),
        hi = Math.log(HIGH),
        best = lo,
        value = Double.POSITIVE_INFINITY,
        step = (hi - lo) / 100;
    for (int i = 0; i <= 100; i++) {
      double x = lo + i * step, e = objective.at(Math.exp(x));
      if (e < value) {
        value = e;
        best = x;
      }
    }
    double a = Math.max(lo, best - step), b = Math.min(hi, best + step);
    for (int i = 0; i < 45; i++) {
      double x = a + (b - a) * .38196601125, y = a + (b - a) * .61803398875;
      if (objective.at(Math.exp(x)) < objective.at(Math.exp(y))) b = y;
      else a = x;
    }
    return Math.exp((a + b) / 2);
  }

  private static double median(List<Double> values) {
    List<Double> sorted = new ArrayList<>(values);
    Collections.sort(sorted);
    int n = sorted.size();
    return n % 2 == 1 ? sorted.get(n / 2) : (sorted.get(n / 2 - 1) + sorted.get(n / 2)) / 2;
  }

  private static final class Model {
    final int a, b;
    int matches, inliers;
    boolean accepted;
    double focal, score, curvature, homographyFraction;
    double[] fundamental;
    String reason;

    Model(int a, int b) {
      this.a = a;
      this.b = b;
    }

    Object report() {
      return Json.object(
          "image_a",
          a,
          "image_b",
          b,
          "reciprocal_matches",
          matches,
          "fundamental_inliers",
          inliers,
          "F_centered_diagonal_coordinates",
          fundamental,
          "focal_over_diagonal",
          focal,
          "essential_error",
          score,
          "curvature_25_percent",
          curvature,
          "homography_inlier_fraction",
          homographyFraction,
          "accepted",
          accepted,
          "reason",
          reason);
    }
  }
}
