package sgm;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.*;
import java.util.zip.GZIPOutputStream;
import org.opencv.core.*;
import org.opencv.features2d.*;
import org.opencv.imgproc.Imgproc;

/** Estimates a horizontal matching interval from reciprocal rectified-image feature matches. */
final class DisparityRange {
  final int minimum, count, sampleCount;
  final double observedHigh;
  final double observedLow;
  final String method;
  final Object diagnostics;
  final List<Sample> samples;

  record Sample(double disparity, double x, double y, String source, boolean anchor) {}

  record Bounds(int minimum, int count) {
    int maximum() {
      return minimum + count - 1;
    }
  }

  DisparityRange(int minimum, int count, int sampleCount, double high, String method) {
    this(minimum, count, sampleCount, Double.NaN, high, method, null, List.of());
  }

  private DisparityRange(
      int minimum,
      int count,
      int sampleCount,
      double low,
      double high,
      String method,
      Object diagnostics,
      List<Sample> samples) {
    this.minimum = minimum;
    this.count = count;
    this.sampleCount = sampleCount;
    this.observedHigh = high;
    this.observedLow = low;
    this.method = method;
    this.diagnostics = diagnostics;
    this.samples = List.copyOf(samples);
  }

  static DisparityRange estimate(Mat left, Mat right, Mat maskLeft, Mat maskRight, Options o) {
    return estimate(left, right, maskLeft, maskRight, o, List.of());
  }

  static DisparityRange estimate(
      Mat left, Mat right, Mat maskLeft, Mat maskRight, Options o, List<Sample> extra) {
    if (o.disparitiesExplicit)
      return new DisparityRange(o.minDisparity, o.disparities, 0, Double.NaN, "explicit");
    Mat a = new Mat(),
        b = new Mat(),
        ma = new Mat(),
        mb = new Mat(),
        da = new Mat(),
        db = new Mat();
    MatOfKeyPoint ka = new MatOfKeyPoint(), kb = new MatOfKeyPoint();
    List<MatOfDMatch> fw = new ArrayList<>(), bw = new ArrayList<>();
    org.opencv.features2d.Feature2D sift = FeatureAlgorithms.detector(o, 2500);
    DescriptorMatcher matcher = null;
    List<Sample> samples = new ArrayList<>(extra);
    try {
      double scale = Math.min(1, 640. / Math.max(left.cols(), left.rows()));
      Size size =
          new Size(
              Math.max(32, Math.round(left.cols() * scale)),
              Math.max(32, Math.round(left.rows() * scale)));
      Imgproc.resize(left, a, size, 0, 0, Imgproc.INTER_AREA);
      Imgproc.resize(right, b, size, 0, 0, Imgproc.INTER_AREA);
      Imgproc.resize(maskLeft, ma, size, 0, 0, Imgproc.INTER_NEAREST);
      Imgproc.resize(maskRight, mb, size, 0, 0, Imgproc.INTER_NEAREST);
      sift.detectAndCompute(a, ma, ka, da);
      sift.detectAndCompute(b, mb, kb, db);
      if (da.rows() >= 2 && db.rows() >= 2) {
        matcher = FeatureAlgorithms.matcher(o, da);
        matcher.knnMatch(da, db, fw, 2);
        matcher.knnMatch(db, da, bw, 2);
        Map<Integer, Integer> reverse = new HashMap<>();
        for (MatOfDMatch row : bw) {
          DMatch[] m = row.toArray();
          if (m.length >= 2 && m[0].distance < .75 * m[1].distance)
            reverse.put(m[0].queryIdx, m[0].trainIdx);
        }
        KeyPoint[] x = ka.toArray(), y = kb.toArray();
        for (MatOfDMatch row : fw) {
          DMatch[] m = row.toArray();
          if (m.length < 2
              || m[0].distance >= .75 * m[1].distance
              || !Objects.equals(reverse.get(m[0].trainIdx), m[0].queryIdx)) continue;
          Point p = x[m[0].queryIdx].pt, q = y[m[0].trainIdx].pt;
          double d = (p.x - q.x) * left.cols() / a.cols();
          if (Math.abs(p.y - q.y) <= Math.max(1.25, 1.5 * o.reprojection * a.rows() / left.rows())
              && d > o.minDisparity
              && d < left.cols() - o.census)
            samples.add(
                new Sample(
                    d,
                    p.x * left.cols() / a.cols(),
                    p.y * left.rows() / a.rows(),
                    "rectified_preview",
                    false));
        }
      }
      DisparityRange initial = fromEvidence(samples, left.cols(), left.rows(), o);
      Probe guard = guard(left, right, maskLeft, maskRight, o, initial);
      samples.addAll(guard.samples);
      DisparityRange combined = fromEvidence(samples, left.cols(), left.rows(), o);
      boolean denseFallback =
          initial.method.startsWith("weak_evidence")
              && guard.samples.size() >= 24
              && combined.method.equals("pair_keypoints_and_sparse_geometry_two_sided");
      Bounds union =
          bounds(
              denseFallback ? combined.minimum : Math.min(initial.minimum, combined.minimum),
              Math.max(initial.minimum + initial.count - 1, combined.minimum + combined.count - 1),
              left.cols(),
              o);
      return new DisparityRange(
          union.minimum,
          union.count,
          combined.sampleCount,
          combined.observedLow,
          combined.observedHigh,
          denseFallback ? "coarse_validated_two_sided_fallback" : combined.method,
          Json.object(
              "keypoint_estimate",
              initial.diagnostics,
              "combined_evidence",
              combined.diagnostics,
              "coarse_guard",
              guard.report,
              "lower_bound_from_dense_fallback",
              denseFallback,
              "initial_interval",
              new int[] {initial.minimum, initial.minimum + initial.count - 1},
              "guard_widened",
              union.minimum < initial.minimum
                  || union.maximum() > initial.minimum + initial.count - 1),
          combined.samples);
    } finally {
      for (Mat m : fw) m.release();
      for (Mat m : bw) m.release();
      a.release();
      b.release();
      ma.release();
      mb.release();
      da.release();
      db.release();
      ka.release();
      kb.release();
      if (matcher != null) matcher.clear();
    }
  }

  static boolean pinnedMinimum(Options o) {
    return o.minDisparityExplicit || o.minDisparity != 0;
  }

  /** Spatial coverage is relative to the feasible overlap, including narrow high-parallax pairs. */
  static DisparityRange fromEvidence(List<Sample> input, int width, int height, Options o) {
    List<Sample> valid = new ArrayList<>();
    Map<String, Integer> sources = new TreeMap<>();
    for (Sample s : input)
      if (Double.isFinite(s.disparity)
          && s.disparity > o.minDisparity
          && s.disparity < width - o.census
          && Double.isFinite(s.x)
          && Double.isFinite(s.y)
          && s.x >= 0
          && s.x < width
          && s.y >= 0
          && s.y < height) {
        valid.add(s);
        sources.merge(s.source, 1, Integer::sum);
      }
    valid.sort(Comparator.comparingDouble(Sample::disparity));
    double low = quantile(valid, .01), high = quantile(valid, .99), median = quantile(valid, .5);
    double overlap = Math.max(1, width - Math.abs(Double.isFinite(median) ? median : 0));
    double origin = Math.max(0, Double.isFinite(median) ? median : 0);
    double xmin = Double.POSITIVE_INFINITY, xmax = -xmin, ymin = xmin, ymax = -xmin;
    Set<Integer> cells = new HashSet<>();
    for (Sample s : valid) {
      xmin = Math.min(xmin, s.x);
      xmax = Math.max(xmax, s.x);
      ymin = Math.min(ymin, s.y);
      ymax = Math.max(ymax, s.y);
      int x = Math.max(0, Math.min(3, (int) (4 * (s.x - origin) / overlap))),
          y = Math.max(0, Math.min(2, (int) (3 * s.y / height)));
      cells.add(y * 4 + x);
    }
    double cx = valid.isEmpty() ? 0 : Math.min(1, (xmax - xmin) / overlap),
        cy = valid.isEmpty() ? 0 : (ymax - ymin) / height;
    boolean reliable = valid.size() >= 12 && cells.size() >= 4 && cx >= .2 && cy >= .2;
    // Quantiles reject isolated mismatches. Credible minor depth layers are preserved,
    // including single well-constrained 3+ view points and spatially supported tails.
    double credibleLow = low, credibleHigh = high;
    for (Sample s : valid)
      if (s.anchor || s.disparity < low || s.disparity > high) {
        if (s.anchor || supported(s, valid)) {
          credibleLow = Math.min(credibleLow, s.disparity);
          credibleHigh = Math.max(credibleHigh, s.disparity);
        }
      }
    double margin =
        Double.isFinite(credibleHigh)
            ? Math.max(
                16,
                Math.max(
                    o.reprojection * 4,
                    Math.max(
                        width / 640.0 * 3,
                        .02 * Math.max(Math.abs(credibleLow), Math.abs(credibleHigh)))))
            : 16;
    Bounds range;
    if (reliable)
      range =
          bounds(
              pinnedMinimum(o) ? o.minDisparity : (int) Math.floor(credibleLow - margin),
              (int) Math.ceil(credibleHigh + margin),
              width,
              o);
    else {
      List<Double> unlocated = new ArrayList<>();
      for (Sample s : valid) unlocated.add(s.disparity);
      DisparityRange broad = fromSamples(unlocated, width, o);
      range = new Bounds(broad.minimum, broad.count);
    }
    Object info =
        Json.object(
            "samples",
            valid.size(),
            "sources",
            sources,
            "occupied_cells_4x3",
            cells.size(),
            "overlap_normalized_x_coverage",
            cx,
            "y_coverage",
            cy,
            "spatially_reliable",
            reliable,
            "confidence_reason",
            reliable
                ? "distributed_pair_evidence"
                : "insufficient_or_clustered_evidence_broad_fallback",
            "observed_p01",
            low,
            "observed_p99",
            high,
            "credible_low",
            credibleLow,
            "credible_high",
            credibleHigh,
            "margin_pixels",
            margin,
            "reprojection_tolerance_pixels",
            o.reprojection,
            "minimum_pinned",
            pinnedMinimum(o));
    return new DisparityRange(
        range.minimum,
        range.count,
        valid.size(),
        low,
        high,
        reliable
            ? "pair_keypoints_and_sparse_geometry_two_sided"
            : "weak_evidence_broad_adaptive_probe",
        info,
        valid);
  }

  private static double quantile(List<Sample> values, double fraction) {
    return values.isEmpty()
        ? Double.NaN
        : values.get((int) Math.floor((values.size() - 1) * fraction)).disparity;
  }

  private static boolean supported(Sample sample, List<Sample> values) {
    Set<Long> positions = new HashSet<>();
    double radius = Math.max(3, .005 * Math.abs(sample.disparity));
    for (Sample s : values)
      if (Math.abs(s.disparity - sample.disparity) <= radius) {
        long x = Math.round(s.x / 4), y = Math.round(s.y / 4);
        positions.add((x << 32) ^ (y & 0xffffffffL));
        if (positions.size() >= 3) return true;
      }
    return false;
  }

  static Bounds bounds(int minimum, int maximum, int width, Options o) {
    int lo = Math.max(o.minDisparity, minimum), hi = Math.min(width - 1, maximum);
    if (pinnedMinimum(o)) lo = o.minDisparity;
    if (lo > width - 3)
      throw new IllegalArgumentException(
          "Minimum disparity leaves fewer than three usable hypotheses");
    hi = Math.max(lo + 2, hi);
    long span = (long) hi - lo + 1;
    if (span > 4096)
      throw new IllegalArgumentException(
          "Observed disparity interval exceeds 4096 hypotheses; restrict the requested range or"
              + " lower dense resolution");
    // Arbitrary counts are supported by both backends. Rounding to sixteen can
    // needlessly exceed a device's allocation limit even when this envelope fits.
    int count = (int) Math.min((long) width - lo, Math.max(16, span));
    return new Bounds(lo, count);
  }

  static Bounds expansion(
      Options original, Options current, CensusSgm.Stats stats, int width, int attempt) {
    if (original.disparitiesExplicit) return null;
    boolean lower =
        stats.lowerClipped()
            && !pinnedMinimum(original)
            && current.minDisparity > original.minDisparity;
    boolean upper =
        stats.upperClipped() && current.minDisparity + current.disparities - 1 < width - 1;
    if (!lower && !upper) return null;
    int grow = Math.max(16, ((current.disparities / 2 + 15) / 16) * 16);
    int lo = current.minDisparity, hi = lo + current.disparities - 1;
    if (lower) lo = Math.max(original.minDisparity, lo - grow);
    if (upper) hi = Math.min(width - 1, hi + grow);
    if (attempt >= 3) {
      lo = original.minDisparity;
      hi = width - 1;
    }
    Bounds next = bounds(lo, hi, width, original);
    return next.minimum == current.minDisparity && next.count == current.disparities ? null : next;
  }

  static Point rectifyPoint(Camera camera, Point point, double[] r, double[] p, boolean vertical) {
    double[] ray =
        Math3.mv(
            r,
            new double[] {(point.x - camera.cx) / camera.fx, (point.y - camera.cy) / camera.fy, 1});
    double z = p[8] * ray[0] + p[9] * ray[1] + p[10] * ray[2];
    if (!(z > 0)) return null;
    double x = (p[0] * ray[0] + p[1] * ray[1] + p[2] * ray[2]) / z;
    double y = (p[4] * ray[0] + p[5] * ray[1] + p[6] * ray[2]) / z;
    return vertical ? new Point(y, x) : new Point(x, y);
  }

  void evidence(Path directory, Options o) throws IOException {
    if (!o.storage.equals("debug") || samples.isEmpty()) return;
    try (BufferedWriter out =
        new BufferedWriter(
            new OutputStreamWriter(
                new GZIPOutputStream(
                    Files.newOutputStream(directory.resolve("range_samples.csv.gz"))),
                StandardCharsets.UTF_8))) {
      out.write("disparity,left_rectified_x,left_rectified_y,source,three_view_anchor\n");
      for (Sample s : samples)
        out.write(s.disparity + "," + s.x + "," + s.y + "," + s.source + "," + s.anchor + "\n");
    }
  }

  private record Probe(List<Sample> samples, Object report) {}

  /**
   * Coarse whole-domain search guards against surfaces absent from sparse keypoints. Proposed
   * out-of-range depths must also pass a full-resolution ZNCC check.
   */
  private static Probe guard(
      Mat left, Mat right, Mat maskA, Mat maskB, Options original, DisparityRange proposed) {
    Options o = original.copy();
    o.compute = original.compute.equals("cpu") ? "cpu" : "auto";
    o.pyramidLevels = 1;
    o.tiled = false;
    o.censusMode = "gray";
    o.census = Math.min(5, o.census);
    o.memoryMb = Math.min(original.memoryMb, 128);
    o.speckleSize = 8;
    o.speckleRange = 1.5;
    o.uniqueness = Math.max(.05, o.uniqueness);
    o.correlation = Math.max(.5, o.correlation);
    int w = left.cols(), h = left.rows(), longest = 128, pw = 0, ph = 0;
    double scale = 1;
    while (longest >= 32) {
      scale = Math.min(1, longest / (double) Math.max(w, h));
      pw = (int) Math.round(w * scale);
      ph = (int) Math.round(h * scale);
      if (Math.min(pw, ph) < o.census + 2)
        return new Probe(List.of(), Json.object("status", "probe_too_thin"));
      scale = pw / (double) w;
      o.minDisparity = Math.max(1 - pw, (int) Math.floor(original.minDisparity * scale));
      o.disparities = pw - o.minDisparity;
      if (o.disparities < 3)
        return new Probe(List.of(), Json.object("status", "minimum_outside_probe"));
      if (CensusSgm.memoryEstimate(pw, ph, o.disparities, o)
          <= Math.min(o.memoryMb * 1048576L, Runtime.getRuntime().maxMemory() * .65)) break;
      longest = longest * 3 / 4;
    }
    if (longest < 32) return new Probe(List.of(), Json.object("status", "probe_memory_limit"));
    Mat a = new Mat(), b = new Mat(), ma = new Mat(), mb = new Mat();
    try {
      Size size = new Size(pw, ph);
      Imgproc.resize(left, a, size, 0, 0, Imgproc.INTER_AREA);
      Imgproc.resize(right, b, size, 0, 0, Imgproc.INTER_AREA);
      Imgproc.resize(maskA, ma, size, 0, 0, Imgproc.INTER_NEAREST);
      Imgproc.resize(maskB, mb, size, 0, 0, Imgproc.INTER_NEAREST);
      byte[] av = new byte[pw * ph],
          bv = new byte[av.length],
          am = new byte[av.length],
          bm = new byte[av.length];
      a.get(0, 0, av);
      b.get(0, 0, bv);
      ma.get(0, 0, am);
      mb.get(0, 0, bm);
      System.out.printf(
          Locale.ROOT, "Disparity range validation: %dx%d whole-domain probe%n", pw, ph);
      var result = new CensusSgm(o).compute(av, bv, null, null, pw, ph, am, bm);
      int stride = Math.max(1, (int) Math.ceil(Math.sqrt(result.stats.valid / 512.)));
      byte[] fullA = null, fullB = null, fullMaskA = null, fullMaskB = null;
      List<Sample> outside = new ArrayList<>();
      int checked = 0;
      double uncertainty = 1.5 / scale;
      for (int y = 0; y < ph; y += stride)
        for (int x = 0; x < pw; x += stride) {
          float d = result.disparity[y * pw + x];
          if (!Float.isFinite(d)) continue;
          double predicted = d / scale;
          if (!proposed.method.startsWith("weak_evidence")
              && predicted - uncertainty >= proposed.minimum
              && predicted + uncertainty <= proposed.minimum + proposed.count - 1) continue;
          if (fullA == null) {
            fullA = new byte[w * h];
            fullB = new byte[w * h];
            fullMaskA = new byte[w * h];
            fullMaskB = new byte[w * h];
            left.get(0, 0, fullA);
            right.get(0, 0, fullB);
            maskA.get(0, 0, fullMaskA);
            maskB.get(0, 0, fullMaskB);
          }
          int fx = (int) Math.round((x + .5) * w / pw - .5),
              fy = (int) Math.round((y + .5) * h / ph - .5);
          if (fullMaskA[fy * w + fx] == 0) continue;
          checked++;
          int lo = Math.max(original.minDisparity + 1, (int) Math.floor(predicted - uncertainty)),
              hi = Math.min(w - original.census, (int) Math.ceil(predicted + uncertainty));
          double best = -1;
          int winner = lo;
          double[] scores = new double[Math.max(0, hi - lo + 1)];
          Arrays.fill(scores, -1);
          for (int k = lo; k <= hi; k++) {
            int target = fx - k;
            if (target < 0 || target >= w || fullMaskB[fy * w + target] == 0) continue;
            double score =
                CensusSgm.zncc(fullA, fullB, w, h, fx, fy, k, original.correlationWindow / 2, 1);
            scores[k - lo] = score;
            if (score > best) {
              best = score;
              winner = k;
            }
          }
          double second = -1;
          for (int k = lo; k <= hi; k++)
            if (Math.abs(k - winner) > 1) second = Math.max(second, scores[k - lo]);
          if (best < o.correlation || second >= best - .03) continue;
          int rightX = fx - winner, reverseWinner = winner;
          double reverseBest = -1;
          for (int k = lo; k <= hi; k++) {
            int target = rightX + k;
            if (target < 0 || target >= w || fullMaskA[fy * w + target] == 0) continue;
            double score =
                CensusSgm.zncc(
                    fullB, fullA, w, h, rightX, fy, -k, original.correlationWindow / 2, 1);
            if (score > reverseBest) {
              reverseBest = score;
              reverseWinner = k;
            }
          }
          if (reverseBest >= o.correlation
              && Math.abs(reverseWinner - winner) <= 1
              && (proposed.method.startsWith("weak_evidence")
                  || winner < proposed.minimum
                  || winner > proposed.minimum + proposed.count - 1))
            outside.add(new Sample(winner, fx, fy, "coarse_guard", false));
        }
      return new Probe(
          outside,
          Json.object(
              "status",
              "checked",
              "size",
              new int[] {pw, ph},
              "coarse_valid_pixels",
              result.stats.valid,
              "full_resolution_checks",
              checked,
              "sampled_whole_domain",
              proposed.method.startsWith("weak_evidence"),
              "full_resolution_confirmed_samples",
              outside.size(),
              "out_of_range_confirmed_samples",
              outside.stream()
                  .filter(
                      s ->
                          s.disparity < proposed.minimum
                              || s.disparity > proposed.minimum + proposed.count - 1)
                  .count(),
              "reciprocal_tolerance_pixels",
              1,
              "minimum_zncc_peak_gap",
              .03,
              "full_resolution_zncc_minimum",
              o.correlation));
    } finally {
      a.release();
      b.release();
      ma.release();
      mb.release();
    }
  }

  static DisparityRange fromSamples(List<Double> input, int width, Options o) {
    List<Double> values = new ArrayList<>();
    for (double x : input)
      if (Double.isFinite(x) && x > o.minDisparity && x < width - o.census) values.add(x);
    Collections.sort(values);
    double high =
        values.isEmpty() ? Double.NaN : values.get((int) Math.floor((values.size() - 1) * .99));
    int desired =
        values.size() >= 8
            ? (int) Math.ceil(high + Math.max(16, .2 * high) - o.minDisparity + 1)
            : Math.min(width, 128);
    int maximum = Math.min(4096, Math.max(3, width - o.minDisparity));
    int count = Math.min(maximum, Math.max(16, ((desired + 15) / 16) * 16));
    return new DisparityRange(
        o.minDisparity,
        count,
        values.size(),
        high,
        values.size() >= 8
            ? "rectified_reciprocal_" + o.matchingAlgorithm + "_p99_with_margin"
            : "insufficient_features_adaptive_probe");
  }

  static int memoryLimit(int width, int height, Options o) {
    int gpu = GpuSgm.disparityLimit(width, height, o);
    long host = (long) width * height * (96 + 32 * ((o.census * o.census - 1 + 63) / 64));
    long budget = Math.min(o.memoryMb * 1048576L, (long) (Runtime.getRuntime().maxMemory() * .65));
    if (host > budget) gpu = 0;
    if (o.pyramidLevels != 1) return cpuMemoryLimit(width, height, o);
    if (o.compute.equals("gpu")) return gpu;
    return Math.max(gpu, cpuMemoryLimit(width, height, o));
  }

  private static int cpuMemoryLimit(int width, int height, Options o) {
    if (o.pyramidLevels != 1) return Math.min(4096, (Integer.MAX_VALUE - 8) / Math.max(1, width));
    if (o.tiled) {
      long
          budget = Math.min(o.memoryMb * 1048576L, (long) (Runtime.getRuntime().maxMemory() * .65)),
          fixed = CensusSgm.tiledMemoryEstimate(width, height, 0, o.census, o.memoryMb);
      return (int)
          Math.max(
              0,
              Math.min(
                  4096,
                  Math.min(
                      (Integer.MAX_VALUE - 8L) / Math.max(1, width),
                      (budget - fixed) / Math.max(1, 8L * width))));
    }
    long budget = Math.min(o.memoryMb * 1048576L, (long) (Runtime.getRuntime().maxMemory() * .65));
    long fixed = CensusSgm.memoryEstimate(width, height, 0, o),
        per = CensusSgm.memoryEstimate(width, height, 1, o) - fixed;
    return (int)
        Math.max(
            0,
            Math.min(
                4096,
                Math.min(
                    (Integer.MAX_VALUE - 8L) / ((long) width * height), (budget - fixed) / per)));
  }
}
