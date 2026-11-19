package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;

/**
 * Shared Brown camera blocks in the Schur pose/structure bundle solve, one 13-parameter block per
 * camera group (CameraGroups): every camera couples only to its own group's block. Observations
 * stay in the original distorted pixel frame throughout each staged fit. With one camera group
 * there is one block and everything is as before camera groups existed.
 */
final class SharedCalibration {
  // Beyond the lens (0-7): 8 rolling-shutter readout time (s, positive when rows are read top to
  // bottom), 9 GPS time offset (s), 10-12 GPS lever arm (camera axes, m). They are never part of
  // the staged lens refinement; only the GPS-weighted adjustment activates them.
  static final int SIZE = 13, LENS = 8, READOUT = 8, TIME = 9, LEVER = 10;
  static final String[] NAMES = {
    "f", "k1", "k2", "cx", "cy", "p1", "p2", "k3", "rs", "dt", "lx", "ly", "lz"
  };
  /** Group 0's initial calibration (the reference camera). */
  final Calibration initial;
  /** Initial calibration of every group (its first frame's). */
  final Calibration[] initials;
  final int groups;
  final Options options;
  /**
   * Measured (distorted) image points: observed[frame id] holds x, y of keypoint i at 2 i and
   * 2 i + 1, the exact doubles of {@link #distorted}. Null after {@link #releaseObservations}.
   */
  double[][] observed;
  final boolean fixedPoses;
  /** Parameter k of group g is index g * SIZE + k in active, values and prior. */
  final boolean[] active;
  boolean adaptivePriors, dataOnly;
  double[] values;
  final double[] prior;
  // GPS-weighted adjustment priors: log focal (NaN = none) per group with log sigma; readout sigma (s).
  final double[] focalPriors;
  double focalPriorSigma = .03, readoutSigma = .1;
  /**
   * Held-out residual gate (squared pixels) per held-out set and camera scope, fixed at the
   * first score of that set (the starting model). Lens model selection scores capped squared
   * residuals so mismatched tie points do not dominate the comparison.
   */
  private final Map<List<?>, Map<Integer, Double>> heldOutGates = new IdentityHashMap<>();
  /** Gate: GATE_MEDIAN_FACTOR times the median squared residual, at least GATE_FLOOR_PIXELS. */
  static final double GATE_MEDIAN_FACTOR = 10, GATE_FLOOR_PIXELS = 4;

  SharedCalibration(List<ImageFrame> frames) {
    options = frames.get(0).options;
    groups = options.cameraGroups == null ? 1 : Math.max(1, options.cameraGroups.size());
    initials = new Calibration[groups];
    initials[0] = frames.get(0).calibration;
    for (ImageFrame f : frames)
      if (f.group < groups && initials[f.group] == null) initials[f.group] = f.calibration;
    // A group without frames here (never the case in a run) keeps a block that nothing uses.
    for (int g = 0; g < groups; g++) if (initials[g] == null) initials[g] = initials[0];
    initial = initials[0];
    fixedPoses = options.poses != null && options.extrinsicsMode.equals("fixed");
    active = new boolean[SIZE * groups];
    values = new double[SIZE * groups];
    for (int g = 0; g < groups; g++) {
      Calibration c = initials[g];
      double[] block = {
        Math.log(c.fx),
        c.distortion[0],
        c.distortion[1],
        c.cx / c.width,
        c.cy / c.height,
        c.distortion[2],
        c.distortion[3],
        c.distortion[4],
        0,
        0,
        0,
        0,
        0
      };
      System.arraycopy(block, 0, values, g * SIZE, SIZE);
    }
    prior = values.clone();
    focalPriors = new double[groups];
    Arrays.fill(focalPriors, Double.NaN);
    long measuredPoints = 0;
    for (ImageFrame f : frames) measuredPoints += f.keypoints.length;
    // Two doubles per point plus one array header and reference per frame.
    MemorySafety.requireHeap(measuredPoints * 16 + frames.size() * 32L, options, "Shared-calibration measured image points");
    observed = new double[frames.size()][];
    for (ImageFrame f : frames) {
      double[] measured = observed[f.id] = new double[2 * f.keypoints.length];
      for (int i = 0; i < f.keypoints.length; i++) {
        Point pixel = f.keypoints[i].pt;
        distorted(f.camera, f.calibration.distortion, pixel.x, pixel.y, measured, 2 * i);
      }
    }
  }

  /**
   * Drops the measured points once no fit or validation of this model follows (reports may
   * still use the parameters). Any later use fails instead of reading stale points.
   */
  void releaseObservations() {
    observed = null;
  }

  /** Width of the lens part of the bundle system: 13 per camera group. */
  int size() {
    return SIZE * groups;
  }

  /** First index of a frame's lens block. */
  private int base(ImageFrame frame) {
    return frame.group < groups ? frame.group * SIZE : 0;
  }

  Calibration initialOf(ImageFrame frame) {
    return initials[frame.group < groups ? frame.group : 0];
  }

  /** Group ID for reports (camera_1 ... ); empty with one camera. */
  String id(int g) {
    return options.cameraGroups != null && g < options.cameraGroups.size() ? options.cameraGroups.get(g).id : "camera_" + (g + 1);
  }

  static Point distorted(Camera c, double[] d, Point pixel) {
    double[] xy = new double[2];
    distorted(c, d, pixel.x, pixel.y, xy, 0);
    return new Point(xy[0], xy[1]);
  }

  /** {@link #distorted(Camera, double[], Point)} of (px, py) into out[at], out[at + 1]. */
  static void distorted(Camera c, double[] d, double px, double py, double[] out, int at) {
    double x = (px - c.cx) / c.fx, y = (py - c.cy) / c.fy, r = x * x + y * y;
    double radial = 1 + d[0] * r + d[1] * r * r + d[4] * r * r * r;
    out[at] = c.fx * (x * radial + 2 * d[2] * x * y + d[3] * (r + 2 * x * x)) + c.cx;
    out[at + 1] = c.fy * (y * radial + d[2] * (r + 2 * y * y) + 2 * d[3] * x * y) + c.cy;
  }

  /** Group 0's calibration (one camera: the shared calibration). */
  Calibration calibration() {
    return calibration(0);
  }

  Calibration calibration(int g) {
    Calibration initial = initials[g];
    int o = g * SIZE;
    double f = values[o] == prior[o] ? initial.fx : Math.exp(values[o]);
    return new Calibration(
        f,
        values[o] == prior[o] ? initial.fy : f * initial.fy / initial.fx,
        values[o + 3] == prior[o + 3] ? initial.cx : values[o + 3] * initial.width,
        values[o + 4] == prior[o + 4] ? initial.cy : values[o + 4] * initial.height,
        new double[] {values[o + 1], values[o + 2], values[o + 5], values[o + 6], values[o + 7]},
        initial.width,
        initial.height);
  }

  /** True when camera position priors are GPS antenna fixes: C + R^T L + V dt (indices 9-12). */
  boolean antennaModel() {
    for (int g = 0; g < groups; g++)
      for (int k = TIME; k < SIZE; k++) if (active[g * SIZE + k] || values[g * SIZE + k] != 0) return true;
    return false;
  }

  /** GPS antenna position of a camera of group g: C + R^T L + V dt. */
  double[] antenna(Camera c, int g) {
    int o = g * SIZE;
    double[] p =
        Math3.add(
            c.center(),
            Math3.mv(
                Math3.transpose(c.rotation),
                new double[] {values[o + LEVER], values[o + LEVER + 1], values[o + LEVER + 2]}));
    return c.velocity == null ? p : Math3.add(p, Math3.times(c.velocity, values[o + TIME]));
  }

  /** Signed offset of an observed row from the mid-frame row, in frame heights; 0 if unmodelled. */
  private static double rowFraction(ImageFrame frame, double row) {
    return frame.camera.velocity == null || !Double.isFinite(row) ? 0 : row / frame.height - .5;
  }

  /**
   * Camera-frame point at the exposure time of the observed row. Rows are read sequentially, so
   * row y sees the camera at C + V s with s = readout * (y / height - 1/2): q = R (X - C - V s).
   * Translation only; the gimbal keeps rotation rates small between turns.
   */
  double[] view(ImageFrame frame, double[] point, double row) {
    double[] q = new double[3];
    viewInto(frame, point, row, q);
    return q;
  }

  /**
   * {@link #view} into q (3 values), allocation-free: Camera.viewInto has Camera.view's
   * arithmetic, and R V is formed as Math3.mv forms it.
   */
  void viewInto(ImageFrame frame, double[] point, double row, double[] q) {
    Camera c = frame.camera;
    c.viewInto(point[0], point[1], point[2], q);
    double s = values[base(frame) + 8] * rowFraction(frame, row);
    if (s != 0) {
      double[] r = c.rotation, v = c.velocity;
      double rv0 = r[0] * v[0] + r[1] * v[1] + r[2] * v[2];
      double rv1 = r[3] * v[0] + r[4] * v[1] + r[5] * v[2];
      double rv2 = r[6] * v[0] + r[7] * v[1] + r[8] * v[2];
      q[0] -= s * rv0;
      q[1] -= s * rv1;
      q[2] -= s * rv2;
    }
  }

  double[] project(ImageFrame frame, double[] point) {
    return project(frame, point, Double.NaN);
  }

  double[] project(ImageFrame frame, double[] point, double row) {
    double[] q = view(frame, point, row);
    return q[2] > 1e-8 ? projectView(frame, q) : null;
  }

  /**
   * {@link #project} with caller-owned scratch q (3) and uv (2): false where project returns null
   * (point not in front of the camera); uv is then unspecified.
   */
  boolean projectInto(ImageFrame frame, double[] point, double row, double[] q, double[] uv) {
    viewInto(frame, point, row, q);
    if (!(q[2] > 1e-8)) return false;
    projectViewInto(frame, q, uv);
    return true;
  }

  private double[] projectView(ImageFrame frame, double[] q) {
    double[] uv = new double[2];
    projectViewInto(frame, q, uv);
    return uv;
  }

  /** Distorted pixel of the camera-frame point q into uv (2 values). */
  void projectViewInto(ImageFrame frame, double[] q, double[] uv) {
    int o = base(frame);
    Calibration initial = initialOf(frame);
    double fx = Math.exp(values[o]) * frame.width / initial.width;
    double fy = Math.exp(values[o]) * initial.fy / initial.fx * frame.height / initial.height;
    double x = q[0] / q[2], y = q[1] / q[2], r = x * x + y * y;
    double radial = 1 + values[o + 1] * r + values[o + 2] * r * r + values[o + 7] * r * r * r;
    uv[0] = fx * (x * radial + 2 * values[o + 5] * x * y + values[o + 6] * (r + 2 * x * x))
        + (values[o + 3] * initial.width + .5) * frame.width / initial.width
        - .5;
    uv[1] = fy * (y * radial + values[o + 5] * (r + 2 * y * y) + 2 * values[o + 6] * x * y)
        + (values[o + 4] * initial.height + .5) * frame.height / initial.height
        - .5;
  }

  double[][] lensJacobian(ImageFrame frame, double[] point) {
    return lensJacobian(frame, point, Double.NaN);
  }

  double[][] lensJacobian(ImageFrame frame, double[] point, double row) {
    double[] q = view(frame, point, row), uv = projectView(frame, q), jq = new double[6];
    spatialViewInto(frame, q, jq);
    int n = size();
    double[] flat = new double[2 * n];
    lensJacobianInto(frame, q, uv, row, jq, flat, 0);
    double[][] j = new double[2][];
    j[0] = Arrays.copyOfRange(flat, 0, n);
    j[1] = Arrays.copyOfRange(flat, n, 2 * n);
    return j;
  }

  /**
   * {@link #lensJacobian} of the camera-frame point q (from {@link #viewInto} with the same row),
   * given its projection uv ({@link #projectViewInto}) and spatial Jacobian jq
   * ({@link #spatialViewInto}), into out[at, at + size()) (row 0) and out[at + size(), at + 2
   * size()) (row 1). Every element of both rows is written.
   */
  void lensJacobianInto(ImageFrame frame, double[] q, double[] uv, double row, double[] jq, double[] out, int at) {
    int o = base(frame), n = size();
    Calibration initial = initialOf(frame);
    double x = q[0] / q[2], y = q[1] / q[2], r = x * x + y * y;
    double fx = Math.exp(values[o]) * frame.width / initial.width;
    double fy = Math.exp(values[o]) * initial.fy / initial.fx * frame.height / initial.height;
    double cx = (values[o + 3] * initial.width + .5) * frame.width / initial.width - .5;
    double cy = (values[o + 4] * initial.height + .5) * frame.height / initial.height - .5;
    Arrays.fill(out, at, at + 2 * n, 0);
    int a = at + o, b = at + n + o;
    out[a] = uv[0] - cx;
    out[a + 1] = fx * x * r;
    out[a + 2] = fx * x * r * r;
    out[a + 3] = frame.width;
    out[a + 4] = 0;
    out[a + 5] = 2 * fx * x * y;
    out[a + 6] = fx * (r + 2 * x * x);
    out[a + 7] = fx * x * r * r * r;
    out[b] = uv[1] - cy;
    out[b + 1] = fy * y * r;
    out[b + 2] = fy * y * r * r;
    out[b + 3] = 0;
    out[b + 4] = frame.height;
    out[b + 5] = fy * (r + 2 * y * y);
    out[b + 6] = 2 * fy * x * y;
    out[b + 7] = fy * y * r * r * r;
    double fraction = rowFraction(frame, row);
    if (active[o + 8] && fraction != 0) {
      // d q / d readout = -fraction * R V.
      double[] rot = frame.camera.rotation, v = frame.camera.velocity;
      double rv0 = rot[0] * v[0] + rot[1] * v[1] + rot[2] * v[2];
      double rv1 = rot[3] * v[0] + rot[4] * v[1] + rot[5] * v[2];
      double rv2 = rot[6] * v[0] + rot[7] * v[1] + rot[8] * v[2];
      for (int i = 0; i < 2; i++) {
        int e = at + i * n + o + 8;
        out[e] -= jq[i * 3] * fraction * rv0;
        out[e] -= jq[i * 3 + 1] * fraction * rv1;
        out[e] -= jq[i * 3 + 2] * fraction * rv2;
      }
    }
    for (int k = 0; k < n; k++)
      if (!active[k]) {
        out[at + k] = 0;
        out[at + n + k] = 0;
      }
  }

  double[][] spatialJacobian(ImageFrame frame, double[] point) {
    return spatialJacobian(frame, point, Double.NaN);
  }

  /** Derivative of the projection with respect to the (rolling-shutter) camera-frame point. */
  double[][] spatialJacobian(ImageFrame frame, double[] point, double row) {
    return spatialView(frame, view(frame, point, row));
  }

  private double[][] spatialView(ImageFrame frame, double[] q) {
    double[] jq = new double[6];
    spatialViewInto(frame, q, jq);
    return new double[][] {{jq[0], jq[1], jq[2]}, {jq[3], jq[4], jq[5]}};
  }

  /** {@link #spatialJacobian} of the camera-frame point q into jq: row 0 at 0-2, row 1 at 3-5. */
  void spatialViewInto(ImageFrame frame, double[] q, double[] jq) {
    int o = base(frame);
    Calibration initial = initialOf(frame);
    double x = q[0] / q[2], y = q[1] / q[2], r = x * x + y * y;
    double fx = Math.exp(values[o]) * frame.width / initial.width;
    double fy = Math.exp(values[o]) * initial.fy / initial.fx * frame.height / initial.height;
    double a = 1 + values[o + 1] * r + values[o + 2] * r * r + values[o + 7] * r * r * r;
    double slope = values[o + 1] + 2 * values[o + 2] * r + 3 * values[o + 7] * r * r;
    double ux = fx * (a + 2 * x * x * slope + 2 * values[o + 5] * y + 6 * values[o + 6] * x);
    double uy = fx * (2 * x * y * slope + 2 * values[o + 5] * x + 2 * values[o + 6] * y);
    double vx = fy * (2 * x * y * slope + 2 * values[o + 5] * x + 2 * values[o + 6] * y);
    double vy = fy * (a + 2 * y * y * slope + 6 * values[o + 5] * y + 2 * values[o + 6] * x);
    jq[0] = ux / q[2];
    jq[1] = uy / q[2];
    jq[2] = -(ux * x + uy * y) / q[2];
    jq[3] = vx / q[2];
    jq[4] = vy / q[2];
    jq[5] = -(vx * x + vy * y) / q[2];
  }

  void move(double[] delta, int offset) {
    for (int k = 0; k < size(); k++) if (active[k]) values[k] += delta[offset + k];
  }

  private double sigma(int k) {
    int local = k % SIZE;
    return local == 0 ? options.focalSigma : local == 3 || local == 4 ? .02 : options.radialSigma;
  }

  /** The group's calibration came from a file or an imported orientation. */
  private boolean supplied(int g) {
    if (groups == 1 || options.cameraGroups == null) return options.calibration != null;
    return options.cameraGroups.get(g).supplied;
  }

  private String policy(int g) {
    if (options.cameraGroups == null || g >= options.cameraGroups.size()) return options.intrinsicsMode;
    return CameraGroups.policy(options.cameraGroups.get(g), options);
  }

  /** Prior {target, sigma} of an active parameter, or null when it has none. */
  private double[] priorOf(int k) {
    if (dataOnly) return null;
    int g = k / SIZE, local = k % SIZE;
    if (local == READOUT) return new double[] {0, readoutSigma};
    if (local == TIME) return new double[] {0, GpsAdjustment.TIME_SIGMA};
    if (local >= LEVER) return new double[] {0, GpsAdjustment.LEVER_SIGMA};
    if (local == 0 && Double.isFinite(focalPriors[g])) return new double[] {focalPriors[g], focalPriorSigma};
    // Automatic cameras get adaptive priors; a supplied camera only with its prior policy.
    boolean adaptive = groups == 1 ? adaptivePriors : adaptivePriors && !supplied(g);
    return adaptive || supplied(g) && policy(g).equals("prior")
        ? new double[] {prior[k], sigma(k)}
        : null;
  }

  double priorCost() {
    double cost = 0;
    for (int k = 0; k < size(); k++) {
      double[] p = active[k] ? priorOf(k) : null;
      if (p != null) cost += .5 * Math.pow((values[k] - p[0]) / p[1], 2);
    }
    return cost;
  }

  void priorNormal(double[] normal, double[] rhs) {
    int n = size();
    for (int k = 0; k < n; k++) {
      if (!active[k]) {
        normal[k * n + k] += 1;
        rhs[k] = 0;
        continue;
      }
      double[] p = priorOf(k);
      if (p != null) {
        double weight = 1 / (p[1] * p[1]);
        normal[k * n + k] += weight;
        rhs[k] -= weight * (values[k] - p[0]);
      }
    }
  }

  private boolean withinPriorBounds() {
    for (int k = 0; k < size(); k++)
      if (k % SIZE < LENS
          && active[k]
          && (!Double.isFinite(values[k]) || Math.abs(values[k] - prior[k]) > 4 * sigma(k)))
        return false;
    return true;
  }

  boolean physical() {
    for (double x : values) if (!Double.isFinite(x)) return false;
    for (int g = 0; g < groups; g++) if (!physical(g)) return false;
    return true;
  }

  private boolean physical(int g) {
    int o = g * SIZE;
    Calibration initial = initials[g];
    if (Math.abs(values[o] - prior[o]) > Math.log(2)
        || Math.abs(values[o + 1]) > 2
        || Math.abs(values[o + 2]) > 3
        || Math.abs(values[o + 7]) > 5
        || Math.abs(values[o + 3] - .5) > .25
        || Math.abs(values[o + 4] - .5) > .25
        || Math.abs(values[o + 5]) > .1
        || Math.abs(values[o + 6]) > .1
        || Math.abs(values[o + 8]) > .25
        || Math.abs(values[o + TIME]) > 2
        || Math.abs(values[o + LEVER]) > 2
        || Math.abs(values[o + LEVER + 1]) > 2
        || Math.abs(values[o + LEVER + 2]) > 2) return false;
    // Reject folding anywhere on a conservative grid spanning the source image.
    double f = Math.exp(values[o]), fy = f * initial.fy / initial.fx;
    for (int iy = 0; iy <= 8; iy++)
      for (int ix = 0; ix <= 8; ix++) {
        double x = (ix * initial.width / 8. - values[o + 3] * initial.width) / f;
        double y = (iy * initial.height / 8. - values[o + 4] * initial.height) / fy;
        double r = x * x + y * y,
            a = 1 + values[o + 1] * r + values[o + 2] * r * r + values[o + 7] * r * r * r;
        double slope = values[o + 1] + 2 * values[o + 2] * r + 3 * values[o + 7] * r * r;
        double xx = a + 2 * x * x * slope + 2 * values[o + 5] * y + 6 * values[o + 6] * x;
        double xy = 2 * x * y * slope + 2 * values[o + 5] * x + 2 * values[o + 6] * y;
        double yy = a + 2 * y * y * slope + 6 * values[o + 5] * y + 2 * values[o + 6] * x;
        if (xx < .2 || yy < .2 || xx * yy - xy * xy < .04) return false;
      }
    return true;
  }

  /** One coverage for every group (one camera). */
  boolean constrained(double[] sigma, double coverage) {
    double[] all = new double[groups];
    Arrays.fill(all, coverage);
    return constrained(sigma, all);
  }

  /** coverage: training-observation P90 radius fraction per group. */
  boolean constrained(double[] sigma, double[] coverage) {
    if (sigma == null) return false;
    for (int g = 0; g < groups; g++) {
      boolean any = false;
      for (int k = 0; k < LENS; k++) any |= active[g * SIZE + k];
      if (any && (!Double.isFinite(coverage[g]) || coverage[g] <= 0)) return false;
    }
    for (int g = 0; g < groups; g++) {
      int o = g * SIZE;
      Calibration initial = initials[g];
      // Judge supported radial corrections over the measured P90 radius. Applying the same bound
      // at unobserved corners conflates fit identifiability with extrapolation. Full-image
      // physical checks still apply, and both observed and extrapolated uncertainty are exported
      // separately.
      double r2 =
          (Math.pow(initial.width / 2., 2) + Math.pow(initial.height / 2., 2))
              / Math.exp(2 * values[o])
              * Math.pow(Math.min(1, coverage[g]), 2);
      for (int k = 0; k < LENS; k++)
        if (active[o + k]) {
          // Radial terms need measured image-radius coverage. Other parameters are judged by their
          // marginalized information, so central coverage alone is not a blanket refusal.
          if ((k == 1 || k == 2 || k == 7) && coverage[g] < .55) return false;
          double s = sigma[o + k];
          if (!Double.isFinite(s) || s < 0) return false;
          double effect =
              k == 0 || k == 3 || k == 4
                  ? s
                  : k == 7 ? s * r2 * r2 * r2 : k == 2 ? s * r2 * r2 : s * r2;
          if (effect >= (k == 0 ? .08 : .02)) return false;
        }
    }
    return true;
  }

  static boolean enabled(Options o) {
    if (!(o.alignmentCalibration && o.selfCalibration && !o.rectified)) return false;
    CameraGroups g = o.cameraGroups;
    if (g == null || !g.multiple())
      return o.calibration == null
          || !(g == null ? o.intrinsicsMode : CameraGroups.policy(g.get(0), o)).equals("fixed");
    for (CameraGroups.Group group : g.groups) if (CameraGroups.refinable(group, o)) return true;
    return false;
  }

  /** Number of camera groups of the frames' options (1 without groups). */
  static int groupCount(Options o) {
    return o.cameraGroups == null ? 1 : Math.max(1, o.cameraGroups.size());
  }

  /** The lens of group g may be refitted: not a supplied calibration held fixed. */
  static boolean refinable(Options o, int g) {
    if (groupCount(o) == 1)
      return !(o.calibration != null
          && (o.cameraGroups == null ? o.intrinsicsMode : CameraGroups.policy(o.cameraGroups.get(0), o))
              .equals("fixed"));
    return CameraGroups.refinable(o.cameraGroups.get(g), o);
  }

  /**
   * Every camera group's frames share one calibration. The lens blocks model a group by its first
   * frame's calibration, so a group with per-image calibrations cannot be in the lens model.
   */
  static boolean homogeneous(List<ImageFrame> frames) {
    Map<Integer, Calibration> first = new HashMap<>();
    for (ImageFrame f : frames) {
      Calibration c = first.putIfAbsent(f.group, f.calibration);
      if (c != null && !same(c, f.calibration)) return false;
    }
    return true;
  }

  /** Which groups the staged refinement may change, and why not (null = refinable). */
  private static String[] refusals(List<ImageFrame> frames, Options o, int groups) {
    String[] reason = new String[groups];
    Calibration[] first = new Calibration[groups];
    for (ImageFrame f : frames) {
      int g = Math.min(f.group, groups - 1);
      if (first[g] == null) first[g] = f.calibration;
      else if (reason[g] == null && !same(first[g], f.calibration))
        reason[g] = "its images keep different supplied calibrations";
    }
    if (o.cameraGroups != null && o.cameraGroups.multiple())
      for (int g = 0; g < groups; g++)
        if (reason[g] == null && !CameraGroups.refinable(o.cameraGroups.get(g), o))
          reason[g] = "supplied calibration is fixed (--intrinsics-mode)";
    return reason;
  }

  static void refine(
      List<ImageFrame> frames,
      List<FeatureTracks.Track> tracks,
      int iterations,
      double huber,
      Project project,
      int anchor,
      int scaleCamera)
      throws IOException {
    if (frames.isEmpty()) return;
    Options o = frames.get(0).options;
    if (!enabled(o) || o.alignmentCalibrationDone) return;
    o.alignmentCalibrationDone = true;
    Path report = o.output.resolve("04_geometry/bundle/shared_calibration.json");
    Files.createDirectories(report.getParent());
    int groupCount = o.cameraGroups == null ? 1 : Math.max(1, o.cameraGroups.size());
    boolean multiple = groupCount > 1;
    String[] refused = refusals(frames, o, groupCount);
    if (multiple && !homogeneous(frames)) {
      // The lens blocks model a camera by its first image's calibration; per-image calibrations
      // of one camera would be projected with the wrong one during every trial.
      Json.write(
          report,
          Json.object(
              "accepted",
              false,
              "reason",
              "A camera keeps different per-image calibrations; its lens cannot be modelled as one"
                  + " block, so no camera is refined"));
      return;
    }
    if (!multiple && refused[0] != null) {
      Json.write(
          report,
          Json.object(
              "accepted",
              false,
              "reason",
              "Shared alignment refinement requires one calibration group; heterogeneous imported"
                  + " cameras retained"));
      return;
    }
    // --calibration-trial-method: full (parameter-wise trials on the whole block), subset (the
    // same trials on a compact camera subset, then one whole-block fit of the chosen model) or
    // joint (one fit of all data-supported parameters together).
    String method = o.calibrationTrialMethod;
    int blockViews = CalibrationSubset.registered(frames);
    boolean overLimit = CalibrationSubset.overLimit(frames, groupCount, o);
    if (method.equals("full") && overLimit) {
      // Every trial would be a whole-block bundle adjustment ending in a rejection (and an exact
      // restore): the observability assessment cannot handle this many cameras. Same result,
      // without the trials.
      String why = blockViews + " registered cameras exceed the observability assessment ("
          + CalibrationSubset.OBSERVABILITY_UNKNOWNS + " unknowns, about "
          + CalibrationSubset.limit(groupCount) + " free cameras), so no lens parameter can be accepted;"
          + " trials skipped. --calibration-trial-method subset or joint refines the lens of a large block";
      Json.write(report, Json.object("accepted", false, "trial_method", method, "reason", why,
          "registered_views", blockViews));
      System.out.println("Shared alignment calibration: " + why);
      return;
    }
    // subset: trials on the subset whenever the block has more cameras than it; joint: the
    // subset only for the observability assessments a whole block cannot have.
    CalibrationSubset subset = method.equals("subset") || method.equals("joint") && overLimit
        ? CalibrationSubset.choose(frames, o.calibrationTrialCameras, groupCount) : null;
    // The block before any trial: the subset method refits the whole block from it.
    Camera[] startCameras = frames.stream().map(f -> f.camera.copy()).toArray(Camera[]::new);
    double[][] startPoints = tracks.stream().map(t -> t.point).toArray(double[][]::new);
    boolean inSubset = subset != null && method.equals("subset");
    if (inSubset) {
      subset.enter(frames);
      System.out.println("Shared alignment calibration: trials on " + subset.members.size() + " of "
          + blockViews + " cameras (--calibration-trial-method subset)");
    }
    int trialAnchor = inSubset ? subset.anchor : anchor, trialScale = inSubset ? subset.scale : scaleCamera;
    var split = CalibrationDiagnostics.split(frames, tracks);
    List<FeatureTracks.Track> train = split.train(), held = split.first(), second = split.second();
    Set<Integer> views = new HashSet<>();
    for (var t : train)
      for (var ob : t.observations) if (frames.get(ob.image).camera.registered) views.add(ob.image);
    if (views.size() < 2 || train.size() < 80 || held.size() < 20 || second.size() < 20) {
      if (inSubset) subset.exit(frames);
      Json.write(
          report,
          Json.object(
              "accepted",
              false,
              "reason",
              "Need a stereo baseline, 80 training and two groups of 20 held-out"
                  + " tracks"));
      return;
    }
    SharedCalibration lens = new SharedCalibration(frames);
    double[] startLens = lens.values.clone();
    // Automatic cameras get adaptive priors (with several cameras: those without a supplied one).
    lens.adaptivePriors = multiple || o.calibration == null;
    // A repeatable track split; validation points may move, but never influence cameras/lens.
    Set<String> selected = new HashSet<>(Arrays.asList(o.refineIntrinsics.split(",")));
    List<Object> stages = new ArrayList<>();
    boolean acceptedAny = false;
    boolean[] changedGroup = new boolean[groupCount];
    double[] coverage = new double[groupCount];
    for (int g = 0; g < groupCount; g++) {
      Calibration base = lens.initials[g];
      List<Double> radii = new ArrayList<>();
      double corner = Math.hypot(base.width / 2., base.height / 2.);
      for (var t : train)
        for (var ob : t.observations) {
          ImageFrame f = frames.get(ob.image);
          if (!f.camera.registered || multiple && f.group != g) continue;
          double[] p = lens.observed[f.id];
          radii.add(
              Math.hypot(
                      (p[2 * ob.feature] - f.camera.cx) * base.width / f.width,
                      (p[2 * ob.feature + 1] - f.camera.cy) * base.height / f.height)
                  / corner);
        }
      Collections.sort(radii);
      coverage[g] = radii.isEmpty() ? 0 : radii.get((radii.size() - 1) * 9 / 10);
    }
    Path diagnostics = report.getParent().resolve("calibration_diagnostics");
    if (lens.options.diagnostics)
      CalibrationDiagnostics.residuals(diagnostics, "initial", lens, frames, train, held, second);
    double[] initialScore = lens.heldOutRms(frames, held, -1),
        initialSecondScore = lens.heldOutRms(frames, second, -1);
    double initialRms = initialScore[0];
    boolean[] retained = new boolean[lens.size()];
    int skippedUnchanged = 0;
    List<Object> cameraSections = new ArrayList<>();
    for (int g = 0; g < groupCount; g++) {
      int scope = multiple ? g : -1, base = g * SIZE;
      Map<String, Object> section = new LinkedHashMap<>();
      section.put("camera", lens.id(g));
      section.put("radial_coverage_p90_fraction", coverage[g]);
      if (multiple) {
        // Per-camera guards: a weak camera keeps its calibration and does not block the others.
        String why = refused[g];
        if (why == null) {
          Set<Integer> groupViews = new HashSet<>();
          long trainTracks = 0, heldTracks = 0, secondTracks = 0;
          for (var t : train) {
            boolean touches = false;
            for (var ob : t.observations) {
              ImageFrame f = frames.get(ob.image);
              if (f.camera.registered && f.group == g) {
                groupViews.add(ob.image);
                touches = true;
              }
            }
            if (touches) trainTracks++;
          }
          for (var t : held) if (touches(frames, t, g)) heldTracks++;
          for (var t : second) if (touches(frames, t, g)) secondTracks++;
          if (groupViews.size() < 2 || trainTracks < 80 || heldTracks < 20 || secondTracks < 20)
            why =
                "too few observations of this camera (registered training views "
                    + groupViews.size()
                    + ", training/held-out tracks "
                    + trainTracks
                    + "/"
                    + heldTracks
                    + "/"
                    + secondTracks
                    + "; need 2, 80/20/20)";
        }
        if (why != null) {
          section.put("accepted", false);
          section.put("reason", "Camera kept at its calibration: " + why);
          section.put("calibration", lens.calibrationValues(g));
          cameraSections.add(section);
          System.out.println("Shared alignment calibration: camera " + lens.id(g) + " kept: " + why);
          continue;
        }
      }
      double[] currentScore = multiple ? lens.heldOutRms(frames, held, scope) : initialScore,
          currentSecondScore = multiple ? lens.heldOutRms(frames, second, scope) : initialSecondScore;
      double currentRms = currentScore[0], currentSecondRms = currentSecondScore[0];
      section.put("initial_validation_rms_pixels", currentRms);
      section.put("initial_unweighted_validation_rms_pixels", currentScore[1]);
      section.put("held_out_gate_pixels", new double[] {currentScore[2], currentSecondScore[2]});
      int modelVersion = 0;
      int[] triedVersion = new int[LENS];
      Arrays.fill(triedVersion, -1);
      long firstObservations = held.stream().flatMap(t -> t.observations.stream())
          .filter(ob -> frames.get(ob.image).camera.registered
              && (scope < 0 || frames.get(ob.image).group == scope)).count();
      long secondObservations = second.stream().flatMap(t -> t.observations.stream())
          .filter(ob -> frames.get(ob.image).camera.registered
              && (scope < 0 || frames.get(ob.image).group == scope)).count();
      int[] order = {0, 1, 2, 3, 4, 5, 6, 7};
      if (method.equals("joint")) {
        double[] result = joint(frames, tracks, train, held, second, iterations, huber, anchor, scaleCamera,
            lens, selected, retained, coverage, g, scope, currentRms, currentSecondRms,
            firstObservations, secondObservations, subset, stages);
        if (result[0] > 0) {
          acceptedAny = true;
          changedGroup[g] = true;
          currentRms = result[1];
          currentSecondRms = result[2];
        }
      } else
      // Revisit earlier parameters after other corrections have been supported. Every proposal
      // adds exactly one parameter; a rejected focal estimate must not block a supported k1 fit.
      for (int sweep = 0; sweep < 2; sweep++) {
        boolean changed = false;
        for (int parameter : order) {
          int k = base + parameter;
          if (!selected.contains(NAMES[parameter]) || retained[k]) continue;
          // Retry low-order terms after principal-point/tangential corrections before permitting
          // k3 to explain their residual. A high-order term must earn its additional complexity.
          if (sweep == 0 && parameter == 7 && selected.size() > 1) continue;
          // Rejected fits restore the exact model, poses and points. Retrying that same
          // proposal cannot add evidence; k3 still gets its first trial in sweep two.
          // Focal is an exception: its first sweep selects among three starts, whereas
          // the second tests the nominal start alone (a different proposal).
          if (parameter != 0 && triedVersion[parameter] == modelVersion) {
            skippedUnchanged++;
            continue;
          }
          triedVersion[parameter] = modelVersion;
          long stageStarted = System.nanoTime();
          System.arraycopy(retained, 0, lens.active, 0, lens.size());
          lens.active[k] = true;
          Camera[] saved = new Camera[frames.size()];
          for (ImageFrame f : frames) saved[f.id] = f.camera.copy();
          double[][] points = new double[tracks.size()][];
          for (int i = 0; i < tracks.size(); i++) points[i] = tracks.get(i).point;
          double[] beforeLens = lens.values.clone();
          double before = currentRms;
          double secondBefore = currentSecondRms;
          if (sweep == 0 && parameter == 0)
            lens.focalStarts(
                frames, tracks, train, held, second, iterations, huber, trialAnchor, trialScale, g, scope);
          else
            BundleAdjuster.optimizeTrial(
                frames, train, iterations, huber, trialAnchor, trialScale, lens);
          double[] afterScore = lens.heldOutRms(frames, held, scope),
              secondAfterScore = lens.heldOutRms(frames, second, scope);
          double after = afterScore[0];
          double secondAfter = secondAfterScore[0];
          double proposalAfter = after, proposalSecondAfter = secondAfter;
          double proposalNoise = Math.max(.35, Math.max(after, secondAfter));
          // Spend a full convergence solve only on a promising, bounded short proposal.
          // Acceptance is always reassessed after this solve, including both holdouts and
          // data-only observability. Held-out tracks still never fit poses or intrinsics.
          boolean polished = o.calibrationTrialIterations > 0 && lens.physical()
              && lens.heldOutImproved(before, after, secondBefore, secondAfter)
              && lens.withinPriorBounds()
              && (lens.relaxed()
                  || validationEvidence(before, after, firstObservations, proposalNoise)
                      && validationEvidence(secondBefore, secondAfter, secondObservations, proposalNoise));
          if (polished) {
            BundleAdjuster.optimizeImpl(frames, train, Math.max(12, iterations), huber,
                null, trialAnchor, trialScale, lens);
            afterScore = lens.heldOutRms(frames, held, scope);
            secondAfterScore = lens.heldOutRms(frames, second, scope);
            after = afterScore[0];
            secondAfter = secondAfterScore[0];
          }
          if (lens.options.diagnostics)
            CalibrationDiagnostics.residuals(
                diagnostics,
                (multiple ? CameraGroups.safe(lens.id(g)) + "-" : "") + "sweep-" + sweep + "-" + NAMES[parameter],
                lens, frames, train, held, second);
          boolean improvement =
              lens.physical() && lens.heldOutImproved(before, after, secondBefore, secondAfter);
          double noise = Math.max(.35, Math.max(after, secondAfter));
          // relaxed: no complexity penalty; the parameter's change must be significant instead.
          boolean evidenceGain = lens.relaxed()
              || validationEvidence(before, after, firstObservations, noise)
                  && validationEvidence(secondBefore, secondAfter, secondObservations, noise);
          boolean bounded = lens.withinPriorBounds();
          CalibrationObservability.Assessment assessment =
              improvement && evidenceGain && bounded && Double.isFinite(noise)
                  ? CalibrationObservability.assess(frames, train, huber, lens, noise)
                  : new CalibrationObservability.Assessment(null, 0,
                      "Assessment skipped: candidate already failed validation or prior bounds");
          double[][] covariance = assessment.covariance();
          double[] uncertainty = assessment.sigma();
          boolean constrained = lens.constrained(uncertainty, coverage);
          Double changeSigma = uncertainty == null || !(uncertainty[k] > 0) ? null
              : Math.abs(lens.values[k] - beforeLens[k]) / uncertainty[k];
          boolean significant = !lens.relaxed() || changeSigma != null && changeSigma >= 3;
          boolean accept = improvement && evidenceGain && constrained && bounded && significant;
          Integer trialBudget = null;
          if (o.calibrationTrialIterations > 0) trialBudget = o.calibrationTrialIterations;
          else if (o.bundleIterationsExplicit) trialBudget = Math.max(12, iterations);
          Map<String, Object> stage =
              Json.object(
                  "parameter",
                  NAMES[parameter],
                  "sweep",
                  sweep,
                  "parameters",
                  lens.parameters(),
                  "elapsed_seconds",
                  (System.nanoTime() - stageStarted) / 1e9,
                  "iteration_budget_per_trial",
                  trialBudget,
                  "iteration_mode", o.calibrationTrialIterations > 0 ? "hard trial cap"
                      : o.bundleIterationsExplicit ? "legacy fixed budget" : "legacy automatic",
                  "final_refinement_performed", polished,
                  "proposal_validation_rms_pixels", proposalAfter,
                  "proposal_second_validation_rms_pixels", proposalSecondAfter,
                  "focal_start_scales",
                  sweep == 0 && parameter == 0 ? new double[] {1, .9, 1.1} : new double[] {1},
                  "accepted",
                  accept,
                  "validation_rms_before_pixels",
                  before,
                  "validation_rms_after_pixels",
                  after,
                  "second_validation_rms_before_pixels",
                  secondBefore,
                  "second_validation_rms_after_pixels",
                  secondAfter,
                  "unweighted_validation_rms_after_pixels",
                  new double[] {afterScore[1], secondAfterScore[1]},
                  "pixel_correction_uncertainty",
                  CalibrationDiagnostics.correctionUncertainty(lens, covariance, coverage[g], g),
                  "candidate",
                  lens.calibrationValues(g),
                  "data_only_standard_deviations",
                  uncertainty,
                  "minimum_relative_information",
                  assessment.minimumRelativeInformation(),
                  "validation_complexity_gain_passed",
                  evidenceGain,
                  "observability",
                  assessment.reason(),
                  "reason",
                  accept
                      ? "Withheld-track improvement and data-supported bounded parameter"
                      : !bounded ? "Parameter exceeds four prior standard deviations"
                      : !constrained ? assessment.covariance() == null ? assessment.reason()
                          : "Data uncertainty or parameter-specific image coverage is insufficient"
                      : improvement && !evidenceGain ? "Withheld-track improvement does not justify additional model complexity"
                      : improvement && !significant ? "Parameter changed by less than three data-only standard deviations"
                      : lens.relaxed() ? "Retained preceding calibration; a withheld-track group got worse"
                      : "Retained preceding calibration; insufficient withheld-track improvement");
          if (lens.relaxed()) stage.put("change_in_standard_deviations", changeSigma);
          if (multiple) stage.put("camera", lens.id(g));
          stages.add(stage);
          if (accept) {
            acceptedAny = changed = true;
            changedGroup[g] = true;
            retained[k] = true;
            modelVersion++;
            currentRms = after;
            currentSecondRms = secondAfter;
          } else {
            lens.values = beforeLens;
            for (ImageFrame f : frames) {
              f.camera.rotation = saved[f.id].rotation;
              f.camera.translation = saved[f.id].translation;
              f.camera.fixed = saved[f.id].fixed;
            }
            for (int i = 0; i < tracks.size(); i++) tracks.get(i).point = points[i];
            System.arraycopy(retained, 0, lens.active, 0, lens.size());
          }
        }
        if (!changed && (!selected.contains("k3") || selected.size() == 1)) break;
      }
      if (multiple) {
        List<String> names = new ArrayList<>();
        for (int k = 0; k < LENS; k++) if (retained[base + k]) names.add(NAMES[k]);
        section.put("accepted", changedGroup[g]);
        section.put("accepted_parameters", names);
        section.put("final_validation_rms_pixels", currentRms);
        cameraSections.add(section);
      }
    }
    Map<String, Object> wholeBlock = null;
    if (inSubset) {
      // Back to the whole block as it was before the trials (the subset's poses and points moved
      // in accepted trials); the chosen model is fitted once to every camera and must improve
      // both held-out groups of the whole block, else the initial calibration stays.
      subset.exit(frames);
      double[] chosen = lens.values.clone();
      restore(frames, tracks, startCameras, startPoints);
      if (acceptedAny) {
        long started = System.nanoTime();
        var full = CalibrationDiagnostics.split(frames, tracks);
        lens.values = startLens.clone();
        Arrays.fill(lens.active, false);
        double[][] before = new double[groupCount][], after = new double[groupCount][];
        for (int g = 0; g < groupCount; g++) {
          int scope = multiple ? g : -1;
          before[g] = new double[] {lens.heldOutScore(frames, full.first(), scope),
              lens.heldOutScore(frames, full.second(), scope)};
        }
        lens.values = chosen;
        System.arraycopy(retained, 0, lens.active, 0, lens.size());
        BundleAdjuster.optimizeImpl(frames, full.train(), Math.max(12, iterations), huber, null, anchor, scaleCamera, lens);
        boolean keep = lens.physical();
        for (int g = 0; g < groupCount; g++) {
          int scope = multiple ? g : -1;
          after[g] = new double[] {lens.heldOutScore(frames, full.first(), scope),
              lens.heldOutScore(frames, full.second(), scope)};
          boolean better = lens.relaxed()
              ? after[g][0] <= before[g][0] && after[g][1] <= before[g][1]
              : after[g][0] < before[g][0] && after[g][1] < before[g][1];
          if (changedGroup[g] && !better) keep = false;
        }
        wholeBlock = Json.object(
            "accepted", keep,
            "cameras", blockViews,
            "training_tracks", full.train().size(),
            "validation_rms_before_pixels", before[0][0],
            "validation_rms_after_pixels", after[0][0],
            "second_validation_rms_before_pixels", before[0][1],
            "second_validation_rms_after_pixels", after[0][1],
            "elapsed_seconds", (System.nanoTime() - started) / 1e9,
            "rule", "the subset's model, fitted once to the whole block, must " + (lens.relaxed() ? "not raise" : "lower") + " both held-out"
                + " RMS values of the whole block (of every changed camera)");
        if (multiple) {
          List<Object> perCamera = new ArrayList<>();
          for (int g = 0; g < groupCount; g++)
            perCamera.add(Json.object("camera", lens.id(g), "changed", changedGroup[g],
                "validation_rms_before_pixels", before[g][0], "validation_rms_after_pixels", after[g][0],
                "second_validation_rms_before_pixels", before[g][1], "second_validation_rms_after_pixels", after[g][1]));
          wholeBlock.put("cameras_detail", perCamera);
        }
        System.out.println("Shared alignment calibration: subset model on the whole block "
            + (keep ? "accepted" : "rejected") + String.format(Locale.ROOT,
                "; held-out RMS %.4f -> %.4f px, %.4f -> %.4f px", before[0][0], after[0][0], before[0][1], after[0][1]));
        if (!keep) {
          restore(frames, tracks, startCameras, startPoints);
          lens.values = startLens.clone();
          Arrays.fill(retained, false);
          Arrays.fill(changedGroup, false);
          acceptedAny = false;
          for (Object s : cameraSections) {
            Map<String, Object> section = JsonRead.object(s);
            section.put("accepted", false);
            section.put("accepted_parameters", List.of());
            section.put("reason", "The subset's model did not improve the whole block");
          }
        }
      }
    }
    // The pre-trial block is only needed by the subset refit above.
    startCameras = null;
    startPoints = null;
    System.arraycopy(retained, 0, lens.active, 0, lens.size());
    if (acceptedAny) {
      // Fit all structure conditionally, including points omitted by the sampling cap. This is
      // needed for fixed poses too, where no subsequent pose/point bundle is performed.
      lens.validationRms(frames, tracks, true);
      // Last use of the measured points: the report below needs only the parameters, and the
      // final pose/point bundle uses the frames' keypoints.
      lens.releaseObservations();
      Calibration result = lens.calibration();
      if (!multiple) for (ImageFrame frame : frames) frame.applyCalibration(result);
      else
        // Only cameras with accepted parameters change (others may keep per-image calibrations).
        for (ImageFrame frame : frames)
          if (changedGroup[frame.group]) frame.applyCalibration(lens.calibration(frame.group));
      if (multiple)
        for (int g = 0; g < groupCount; g++) {
          Map<String, Object> section = JsonRead.object(cameraSections.get(g));
          section.put("calibration", lens.calibrationValues(g));
        }
      result.write(
          o.output.resolve("camera.txt"),
          "Adaptive shared calibration from data-supported bounded parameters");
      if (!o.orientationCalibrations.isEmpty()) {
        Map<String, Calibration> updated = new LinkedHashMap<>();
        for (ImageFrame f : frames)
          updated.put(f.path.getFileName().toString(), multiple ? f.calibration : result);
        o.orientationCalibrations = updated;
      }
      if (!lens.fixedPoses)
        BundleAdjuster.optimizeImpl(
            frames, tracks, iterations, huber, project, anchor, scaleCamera, null);
    } else if (multiple)
      for (int g = 0; g < groupCount; g++) {
        Map<String, Object> section = JsonRead.object(cameraSections.get(g));
        section.putIfAbsent("calibration", lens.calibrationValues(g));
      }
    Map<String, Object> out =
        Json.object(
            "accepted",
            acceptedAny,
            "model",
            multiple
                ? "Brown f,cx,cy,k1,k2,k3,p1,p2 per camera group; fixed focal aspect ratio"
                : "Shared Brown f,cx,cy,k1,k2,k3,p1,p2; fixed focal aspect ratio",
            "selected_parameters",
            o.refineIntrinsics,
            "accepted_parameters",
            lens.parameters(),
            "selection_policy",
            "One parameter added per trial; retry only after an accepted model change; data-only marginal information, physical map, four-sigma prior bound and two withheld-track validation groups with a noise-scaled complexity penalty",
            "unchanged_model_trials_skipped", skippedUnchanged,
            "fit_prior_sigmas",
            Json.object("log_focal", o.focalSigma, "distortion", o.radialSigma, "principal_point_fraction", .02),
            "parameter_order",
            NAMES,
            "solver_parameter_units",
            new String[] {
              "log_pixels",
              "dimensionless",
              "dimensionless",
              "fraction_of_source_width",
              "fraction_of_source_height",
              "dimensionless",
              "dimensionless",
              "dimensionless"
            },
            "uncertainty_scope",
            "Undamped data-only lens information after marginalizing structure and poses; scale gauge projected out. Excludes lens priors and model error; conditional on fixed parameters",
            "uncertainty_acceptance_radius",
            "Training observation P90 radius; radial/tangential effects below 0.02, log-focal sigma"
                + " below 0.08, principal point fractions below 0.02. Uncovered image corners are"
                + " reported as extrapolation; physical map checks cover the full image.",
            "sampling",
            split.coverage(),
            "validation_policy",
            "Two disjoint spatially stratified holdouts; neither fits poses or intrinsics. Both"
                + " must improve by 0.5% and 0.002 processed pixels. Scores are capped RMS: each"
                + " squared residual is capped at a gate fixed from the starting model (10 times the"
                + " median squared residual, at least 4 px), so gross mismatches count the same before"
                + " and after; unweighted RMS is reported alongside. Focal trials selected by"
                + " combined holdout RMS; these sets perform model selection and are not an"
                + " independent accuracy test.",
            "second_held_out_tracks",
            second.size(),
            "training_tracks",
            train.size(),
            "held_out_tracks",
            held.size(),
            "registered_views",
            views.size(),
            "radial_coverage_p90_fraction",
            coverage[0],
            "initial_validation_rms_pixels",
            initialRms,
            "initial_unweighted_validation_rms_pixels",
            new double[] {initialScore[1], initialSecondScore[1]},
            "held_out_gate_pixels",
            new double[] {initialScore[2], initialSecondScore[2]},
            "calibration",
            lens.calibrationValues(0),
            "stages",
            stages,
            "minimum_relative_held_out_improvement",
            .005,
            "minimum_pixel_held_out_improvement",
            .002,
            "principal_point_prior_sigma_fraction",
            .02);
    if (multiple) {
      out.put(
          "camera_note",
          "One lens block per camera group; each camera is staged and validated on its own"
              + " observations. Top-level calibration and coverage describe the first camera.");
      out.put("cameras", cameraSections);
    }
    if (lens.relaxed())
      out.put("acceptance", Json.object("mode", "relaxed",
          "rule", "neither held-out group may get worse (capped RMS) and every fitted parameter must"
              + " have moved by at least three data-only standard deviations; no complexity penalty;"
              + " observability, coverage, prior bound and physical checks as in strict"));
    if (!method.equals("full")) {
      out.put("trial_method", method);
      out.put("block_registered_views", blockViews);
      if (subset != null) out.put("subset", subset.json(frames));
      if (method.equals("subset"))
        out.put("selection_policy", inSubset
            ? "Parameter-wise trials (as full) on the camera subset; the chosen model is then fitted once to the whole block and kept only if it improves the whole block's held-out groups"
            : "Parameter-wise trials (as full): the block fits the observability assessment, so the subset is the whole block");
      else
        out.put("selection_policy", "Joint: parameters without data support (observability"
            + (subset != null ? " on the camera subset" : "") + ", four-sigma bound, image coverage) are dropped"
            + " first; the rest are fitted together; parameters changed by less than three data-only"
            + " standard deviations are dropped and the rest refitted once; accepted with the two held-out"
            + " tests (complexity penalty per parameter),"
            + " else once more with their f,k1,k2");
      if (wholeBlock != null) out.put("whole_block_check", wholeBlock);
    }
    Json.write(report, out);
    System.out.println(
        "Shared alignment calibration: "
            + (acceptedAny ? "accepted parameter-wise refinement" : "retained initial calibration")
            + "; "
            + report);
  }

  /** Drop order when an assessment gives no covariance: highest-order terms first. */
  private static final int[] JOINT_DROP_ORDER = {7, 6, 5, 4, 3, 2, 1, 0};

  /**
   * --calibration-trial-method joint for camera group g: drop the candidate parameters the data
   * do not support (observability, on the camera subset of a large block; four-sigma limits;
   * radial coverage), the worst first, then fit the rest together. Parameters that moved by less
   * than three data-only standard deviations are dropped and the rest refitted once (backward
   * elimination); accepted with the two held-out tests, the complexity penalty per parameter. A
   * failed set is retried once with its f, k1, k2.
   * Returns {accepted ? 1 : 0, held-out RMS, second held-out RMS}.
   */
  private static double[] joint(
      List<ImageFrame> frames, List<FeatureTracks.Track> tracks, List<FeatureTracks.Track> train,
      List<FeatureTracks.Track> held, List<FeatureTracks.Track> second, int iterations, double huber,
      int anchor, int scaleCamera, SharedCalibration lens, Set<String> selected, boolean[] retained,
      double[] coverage, int g, int scope, double currentRms, double currentSecondRms,
      long firstObservations, long secondObservations, CalibrationSubset subset, List<Object> stages)
      throws IOException {
    int base = g * SIZE;
    boolean multiple = lens.groups > 1;
    List<Integer> candidates = new ArrayList<>();
    for (int p = 0; p < LENS; p++) if (selected.contains(NAMES[p]) && !retained[base + p]) candidates.add(p);
    double[] unchanged = {0, currentRms, currentSecondRms};
    if (candidates.isEmpty()) return unchanged;
    long started = System.nanoTime();
    // Assessment tracks of the subset (the whole block's train tracks when there is no subset).
    List<FeatureTracks.Track> assessTracks = train;
    if (subset != null) {
      subset.enter(frames);
      try {
        assessTracks = CalibrationDiagnostics.split(frames, tracks).train();
      } finally {
        subset.exit(frames);
      }
    }
    List<Object> pruned = new ArrayList<>();
    double noise = Math.max(.35, Math.max(currentRms, currentSecondRms));
    while (!candidates.isEmpty()) {
      lens.setActive(retained, base, candidates);
      CalibrationObservability.Assessment a = assess(frames, assessTracks, huber, lens, noise, subset);
      double[] sigma = a.sigma();
      if (sigma != null && lens.constrained(sigma, coverage)) break;
      int drop = -1;
      double worst = -1;
      if (sigma != null)
        for (int p : JOINT_DROP_ORDER)
          if (candidates.contains(p)) {
            double ratio = lens.limitRatio(sigma, coverage[g], g, p);
            if (ratio > worst) {
              worst = ratio;
              drop = p;
            }
          }
      if (drop < 0) for (int p : JOINT_DROP_ORDER) if (candidates.contains(p)) {
        drop = p;
        break;
      }
      pruned.add(Json.object("parameter", NAMES[drop],
          "reason", sigma == null ? a.reason() : String.format(Locale.ROOT,
              "uncertainty or image coverage beyond its limit (%.2f x the limit)", worst)));
      candidates.remove((Integer) drop);
    }
    System.arraycopy(retained, 0, lens.active, 0, lens.size());
    if (candidates.isEmpty()) {
      Map<String, Object> stage = Json.object("parameter", "joint", "accepted", false, "pruned", pruned,
          "elapsed_seconds", (System.nanoTime() - started) / 1e9,
          "reason", "No candidate parameter is supported by the data");
      if (multiple) stage.put("camera", lens.id(g));
      stages.add(stage);
      return unchanged;
    }
    // Fit the supported candidates together. After the first fit, parameters whose change is
    // below three data-only standard deviations are dropped (backward elimination) and the rest
    // refitted; a set that fails validation is retried once with its f, k1, k2.
    List<Integer> set = candidates;
    boolean eliminated = false, fallback = false;
    while (!set.isEmpty()) {
      long attemptStarted = System.nanoTime();
      Camera[] saved = frames.stream().map(f -> f.camera.copy()).toArray(Camera[]::new);
      double[][] points = tracks.stream().map(t -> t.point).toArray(double[][]::new);
      double[] beforeLens = lens.values.clone();
      lens.setActive(retained, base, set);
      BundleAdjuster.optimizeImpl(frames, train, Math.max(12, iterations), huber, null, anchor, scaleCamera, lens);
      double[] afterScore = lens.heldOutRms(frames, held, scope),
          secondAfterScore = lens.heldOutRms(frames, second, scope);
      double after = afterScore[0];
      double secondAfter = secondAfterScore[0];
      double fitNoise = Math.max(.35, Math.max(after, secondAfter));
      CalibrationObservability.Assessment assessment = lens.physical()
          ? assess(frames, assessTracks, huber, lens, fitNoise, subset)
          : new CalibrationObservability.Assessment(null, 0, "Assessment skipped: unphysical lens");
      double[] sigma = assessment.sigma();
      List<Integer> significant = new ArrayList<>();
      List<Object> zScores = new ArrayList<>();
      if (sigma != null)
        for (int p : set) {
          double z = Math.abs(lens.values[base + p] - beforeLens[base + p]) / sigma[base + p];
          zScores.add(Json.object("parameter", NAMES[p], "change_in_standard_deviations", z));
          if (z >= 3) significant.add(p);
        }
      boolean improvement = lens.physical()
          && lens.heldOutImproved(currentRms, after, currentSecondRms, secondAfter);
      // Complexity penalty per fitted parameter (one trial adds one parameter); relaxed relies on
      // the three-sigma elimination instead.
      boolean evidenceGain = lens.relaxed()
          || validationEvidence(currentRms, after, firstObservations, fitNoise, set.size())
              && validationEvidence(currentSecondRms, secondAfter, secondObservations, fitNoise, set.size());
      boolean bounded = lens.withinPriorBounds();
      boolean constrained = lens.constrained(sigma, coverage);
      boolean eliminate = !eliminated && sigma != null && significant.size() < set.size();
      boolean allSignificant = !lens.relaxed() || sigma != null && significant.size() == set.size();
      boolean accept = !eliminate && improvement && evidenceGain && bounded && constrained && allSignificant;
      List<String> names = new ArrayList<>();
      for (int p : set) names.add(NAMES[p]);
      Map<String, Object> stage = Json.object(
          "parameter", "joint",
          "parameters", lens.parameters(),
          "fitted_together", names,
          "pruned", pruned,
          "change_significance", zScores,
          "elapsed_seconds", (System.nanoTime() - attemptStarted) / 1e9,
          "accepted", accept,
          "validation_rms_before_pixels", currentRms,
          "validation_rms_after_pixels", after,
          "second_validation_rms_before_pixels", currentSecondRms,
          "second_validation_rms_after_pixels", secondAfter,
          "unweighted_validation_rms_after_pixels", new double[] {afterScore[1], secondAfterScore[1]},
          "candidate", lens.calibrationValues(g),
          "data_only_standard_deviations", sigma,
          "validation_complexity_gain_passed", evidenceGain,
          "observability", assessment.reason(),
          "reason", accept ? "Withheld-track improvement and data-supported, significant bounded parameters"
              : eliminate ? "Parameters changed by less than three standard deviations are dropped; the rest refitted"
              : !improvement ? (lens.relaxed() ? "A withheld-track group got worse" : "Insufficient withheld-track improvement")
              : !evidenceGain ? "Withheld-track improvement does not justify the number of parameters"
              : !bounded ? "A parameter exceeds four prior standard deviations"
              : !allSignificant ? "A parameter changed by less than three standard deviations after the refit"
              : "Data uncertainty or image coverage is insufficient after the fit");
      if (multiple) stage.put("camera", lens.id(g));
      stages.add(stage);
      if (accept) {
        for (int p : set) retained[base + p] = true;
        System.arraycopy(retained, 0, lens.active, 0, lens.size());
        return new double[] {1, after, secondAfter};
      }
      lens.values = beforeLens;
      restore(frames, tracks, saved, points);
      System.arraycopy(retained, 0, lens.active, 0, lens.size());
      if (eliminate) {
        eliminated = true;
        set = significant;
        continue;
      }
      if (fallback) break;
      fallback = true;
      List<Integer> core = new ArrayList<>(set);
      core.retainAll(List.of(0, 1, 2));
      if (core.isEmpty() || core.size() == set.size()) break;
      set = core;
    }
    return unchanged;
  }

  /** Observability on the camera subset when there is one (the whole block's is too large). */
  private static CalibrationObservability.Assessment assess(List<ImageFrame> frames,
      List<FeatureTracks.Track> tracks, double huber, SharedCalibration lens, double noise,
      CalibrationSubset subset) {
    if (subset == null) return CalibrationObservability.assess(frames, tracks, huber, lens, noise);
    subset.enter(frames);
    try {
      return CalibrationObservability.assess(frames, tracks, huber, lens, noise);
    } finally {
      subset.exit(frames);
    }
  }

  /** Retained parameters plus the given ones of group g active. */
  private void setActive(boolean[] retained, int base, List<Integer> parameters) {
    System.arraycopy(retained, 0, active, 0, size());
    for (int p : parameters) active[base + p] = true;
  }

  /**
   * How far parameter k of group g is from the acceptance limits of {@link #constrained}: its
   * effect over the covered image radius divided by the allowed effect (infinite when a radial
   * term lacks radius coverage or its uncertainty is unusable). Above 1 it fails.
   */
  double limitRatio(double[] sigma, double coverage, int g, int k) {
    if ((k == 1 || k == 2 || k == 7) && coverage < .55) return Double.POSITIVE_INFINITY;
    double s = sigma[g * SIZE + k];
    if (!Double.isFinite(s) || s < 0) return Double.POSITIVE_INFINITY;
    Calibration initial = initials[g];
    double r2 = (Math.pow(initial.width / 2., 2) + Math.pow(initial.height / 2., 2))
        / Math.exp(2 * values[g * SIZE]) * Math.pow(Math.min(1, coverage), 2);
    double effect = k == 0 || k == 3 || k == 4 ? s : k == 7 ? s * r2 * r2 * r2 : k == 2 ? s * r2 * r2 : s * r2;
    return effect / (k == 0 ? .08 : .02);
  }

  private static boolean touches(List<ImageFrame> frames, FeatureTracks.Track t, int g) {
    for (var ob : t.observations) {
      ImageFrame f = frames.get(ob.image);
      if (f.camera.registered && f.group == g) return true;
    }
    return false;
  }

  static boolean validationEvidence(double before, double after, long observations, double noise) {
    return Double.isFinite(before) && Double.isFinite(after) && observations > 0
        && (before * before - after * after) * observations
            > noise * noise * Math.log(Math.max(2, observations));
  }

  /** validationEvidence for k parameters fitted together: k times the one-parameter penalty. */
  static boolean validationEvidence(double before, double after, long observations, double noise, int k) {
    return Double.isFinite(before) && Double.isFinite(after) && observations > 0
        && (before * before - after * after) * observations
            > k * noise * noise * Math.log(Math.max(2, observations));
  }

  private void focalStarts(
      List<ImageFrame> frames,
      List<FeatureTracks.Track> tracks,
      List<FeatureTracks.Track> train,
      List<FeatureTracks.Track> first,
      List<FeatureTracks.Track> second,
      int iterations,
      double huber,
      int anchor,
      int scaleCamera,
      int g,
      int scope)
      throws IOException {
    Camera[] original = frames.stream().map(f -> f.camera.copy()).toArray(Camera[]::new);
    double[][] originalPoints = tracks.stream().map(t -> t.point).toArray(double[][]::new);
    double[] originalLens = values.clone(), bestLens = values.clone();
    Camera[] best = original;
    double[][] bestPoints = originalPoints;
    double bestScore = Double.POSITIVE_INFINITY;
    for (double scale : new double[] {1, .9, 1.1}) {
      try (WorkProgress work =
          WorkProgress.start("Calibration focal start " + scale, 0, "trials")) {
        values = originalLens.clone();
        values[g * SIZE] += Math.log(scale);
        restore(frames, tracks, original, originalPoints);
        BundleAdjuster.optimizeTrial(
            frames, train, iterations, huber, anchor, scaleCamera, this);
        double score = heldOutScore(frames, first, scope) + heldOutScore(frames, second, scope);
        if (physical() && score < bestScore) {
          bestScore = score;
          bestLens = values.clone();
          best = frames.stream().map(f -> f.camera.copy()).toArray(Camera[]::new);
          bestPoints = tracks.stream().map(t -> t.point).toArray(double[][]::new);
        }
        work.done();
      }
    }
    values = bestLens;
    restore(frames, tracks, best, bestPoints);
  }

  private static void restore(
      List<ImageFrame> frames,
      List<FeatureTracks.Track> tracks,
      Camera[] cameras,
      double[][] points) {
    for (var f : frames) {
      f.camera.rotation = cameras[f.id].rotation.clone();
      f.camera.translation = cameras[f.id].translation.clone();
      f.camera.fixed = cameras[f.id].fixed;
    }
    for (int i = 0; i < tracks.size(); i++) tracks.get(i).point = points[i];
  }

  /** Active parameter names; with several cameras prefixed by the camera ID ("camera_2:f"). */
  List<String> parameters() {
    List<String> names = new ArrayList<>();
    for (int k = 0; k < size(); k++)
      if (active[k]) names.add(groups == 1 ? NAMES[k] : id(k / SIZE) + ":" + NAMES[k % SIZE]);
    return names;
  }

  /** Active parameter names of one group (no prefix). */
  List<String> parameters(int g) {
    List<String> names = new ArrayList<>();
    for (int k = 0; k < SIZE; k++) if (active[g * SIZE + k]) names.add(NAMES[k]);
    return names;
  }

  Object calibrationValues(int g) {
    Calibration c = calibration(g);
    int o = g * SIZE;
    return Json.object(
        "fx", c.fx, "fy", c.fy, "cx", c.cx, "cy", c.cy, "k1", values[o + 1], "k2", values[o + 2], "p1",
        values[o + 5], "p2", values[o + 6], "k3", values[o + 7], "rolling_shutter_readout_seconds",
        values[o + 8]);
  }

  static boolean same(Calibration a, Calibration b) {
    return a.width == b.width
        && a.height == b.height
        && a.fx == b.fx
        && a.fy == b.fy
        && a.cx == b.cx
        && a.cy == b.cy
        && Arrays.equals(a.distortion, b.distortion);
  }

  double validationRms(List<ImageFrame> frames, List<FeatureTracks.Track> held) {
    return validationRms(frames, held, false);
  }

  double validationRms(List<ImageFrame> frames, List<FeatureTracks.Track> held, boolean commit) {
    return validationRms(frames, held, commit, -1);
  }

  /** scope: only observations of that camera group count (points are fitted to all); -1 all. */
  double validationRms(
      List<ImageFrame> frames, List<FeatureTracks.Track> held, boolean commit, int scope) {
    // Conditional point fits are independent. Keep the final observation additions in
    // their original order: flat calibration directions are sensitive to rounding.
    double sum = 0;
    long n = 0;
    for (int begin = 0; begin < held.size(); ) {
      int end = BundleAdjuster.linearizationBatchEnd(held, begin, BundleAdjuster.linearizationObservationLimit(options, true));
      final int first = begin, size = end - begin;
      double[][] residuals = new double[size][];
      int chunks = Math.min(Math.max(1, options.cores), Math.max(1, size));
      BundleAdjuster.parallel(chunks, chunk -> {
        int from = (int) ((long) size * chunk / chunks);
        int to = (int) ((long) size * (chunk + 1) / chunks);
        for (int i = from; i < to; i++) residuals[i] = validationTrack(frames, held.get(first + i), commit, scope);
      });
      for (double[] row : residuals) if (row != null)
        for (double error : row) {
          sum += error;
          n++;
        }
      begin = end;
    }
    return Math.sqrt(sum / Math.max(1, n));
  }

  /**
   * Lens model-selection score of a held-out set: {capped RMS, plain RMS, gate in pixels}.
   * Squared residuals are capped at a gate fixed at this set's first score (the starting model),
   * so a gross mismatch counts the same before and after a trial and the comparison measures the
   * inliers, like the robust training cost. Plain RMS is validationRms (same order, same sum).
   */
  double[] heldOutRms(List<ImageFrame> frames, List<FeatureTracks.Track> held, int scope) {
    double[] errors = new double[1024];
    int n = 0;
    for (int begin = 0; begin < held.size(); ) {
      int end = BundleAdjuster.linearizationBatchEnd(held, begin, BundleAdjuster.linearizationObservationLimit(options, true));
      final int first = begin, size = end - begin;
      double[][] residuals = new double[size][];
      int chunks = Math.min(Math.max(1, options.cores), Math.max(1, size));
      BundleAdjuster.parallel(chunks, chunk -> {
        int from = (int) ((long) size * chunk / chunks);
        int to = (int) ((long) size * (chunk + 1) / chunks);
        for (int i = from; i < to; i++) residuals[i] = validationTrack(frames, held.get(first + i), false, scope);
      });
      for (double[] row : residuals) if (row != null)
        for (double error : row) {
          if (n == errors.length) errors = Arrays.copyOf(errors, n * 2);
          errors[n++] = error;
        }
      begin = end;
    }
    Map<Integer, Double> byScope = heldOutGates.computeIfAbsent(held, k -> new HashMap<>());
    Double gate = byScope.get(scope);
    if (gate == null) {
      double[] sorted = Arrays.copyOf(errors, n);
      Arrays.sort(sorted);
      double median = n == 0 ? 0 : n % 2 == 1 ? sorted[n / 2] : .5 * (sorted[n / 2 - 1] + sorted[n / 2]);
      gate = Double.isFinite(median)
          ? Math.max(GATE_MEDIAN_FACTOR * median, GATE_FLOOR_PIXELS * GATE_FLOOR_PIXELS)
          : Double.POSITIVE_INFINITY;
      byScope.put(scope, gate);
    }
    double capped = 0, plain = 0;
    for (int i = 0; i < n; i++) {
      plain += errors[i];
      capped += Math.min(errors[i], gate);
    }
    int count = Math.max(1, n);
    return new double[] {Math.sqrt(capped / count), Math.sqrt(plain / count), Math.sqrt(gate)};
  }

  /** --calibration-acceptance relaxed (alignment lens only). */
  boolean relaxed() {
    return "relaxed".equals(options.calibrationAcceptance);
  }

  /**
   * Held-out test of a lens fit. strict: both groups improve by 0.5% and 0.002 px; relaxed:
   * neither group gets worse (the parameters must then be significant, checked by the caller).
   */
  boolean heldOutImproved(double before, double after, double secondBefore, double secondAfter) {
    if (!Double.isFinite(after)) return false;
    if (relaxed()) return Double.isFinite(secondAfter) && after <= before && secondAfter <= secondBefore;
    return after < before * .995 && before - after > .002
        && secondAfter < secondBefore * .995 && secondBefore - secondAfter > .002;
  }

  /** The capped held-out RMS of heldOutRms. */
  double heldOutScore(List<ImageFrame> frames, List<FeatureTracks.Track> held, int scope) {
    return heldOutRms(frames, held, scope)[0];
  }

  private double[] validationTrack(
      List<ImageFrame> frames, FeatureTracks.Track t, boolean commit, int scope) {
      if (t.point == null) return null;
      // This track's projection scratch (one task owns a track): camera-frame point, pixel,
      // spatial Jacobian and point Jacobian, overwritten per observation.
      double[] q = new double[3], uv = new double[2], jq = new double[6], jp = new double[6];
      double[] p = t.point.clone();
      for (int iter = 0; iter < 25; iter++) {
        double[][] h = new double[3][3];
        double[] b = new double[3];
        for (var ob : t.observations) {
          ImageFrame frame = frames.get(ob.image);
          if (!frame.camera.registered) continue;
          double[] measured = observed[frame.id];
          double mx = measured[2 * ob.feature], my = measured[2 * ob.feature + 1];
          if (!projectInto(frame, p, my, q, uv)) return new double[] {Double.POSITIVE_INFINITY};
          double r0 = uv[0] - mx, r1 = uv[1] - my;
          double weight = Math.min(1, 2 / Math.max(1e-12, Math.hypot(r0, r1)));
          // spatialJacobian(frame, p, my) is the spatial Jacobian of this same q.
          spatialViewInto(frame, q, jq);
          for (int row = 0; row < 2; row++)
            for (int j = 0; j < 3; j++) {
              jp[row * 3 + j] = 0;
              for (int k = 0; k < 3; k++)
                jp[row * 3 + j] += jq[row * 3 + k] * frame.camera.rotation[k * 3 + j];
            }
          for (int j = 0; j < 3; j++) {
            b[j] -= weight * jp[j] * r0;
            b[j] -= weight * jp[3 + j] * r1;
            for (int k = 0; k < 3; k++) {
              h[j][k] += weight * jp[j] * jp[k];
              h[j][k] += weight * jp[3 + j] * jp[3 + k];
            }
          }
        }
        for (int j = 0; j < 3; j++) h[j][j] += 1e-6 * Math.max(1, h[j][j]);
        double[] delta = Math3.solve(h, b);
        if (delta == null) break;
        double previous = pointCost(frames, t, p, q, uv), factor = 1;
        boolean moved = false;
        while (factor >= 1. / 128) {
          double[] trial = Math3.add(p, Math3.times(delta, factor));
          if (pointCost(frames, t, trial, q, uv) < previous) {
            p = trial;
            moved = true;
            break;
          }
          factor *= .5;
        }
        if (!moved || Math3.norm(delta) * factor < 1e-9) break;
      }
      if (commit) t.point = p;
      double[] errors = new double[t.observations.size()];
      int n = 0;
      for (var ob : t.observations) {
        ImageFrame frame = frames.get(ob.image);
        if (!frame.camera.registered) continue;
        double[] measured = observed[frame.id];
        double mx = measured[2 * ob.feature], my = measured[2 * ob.feature + 1];
        if (!projectInto(frame, p, my, q, uv)) return new double[] {Double.POSITIVE_INFINITY};
        if (scope >= 0 && frame.group != scope) continue;
        errors[n++] = Math.pow(uv[0] - mx, 2) + Math.pow(uv[1] - my, 2);
      }
      return n == errors.length ? errors : Arrays.copyOf(errors, n);
  }

  /** Robust cost of the track's observations at p; q and uv are the caller's projection scratch. */
  private double pointCost(List<ImageFrame> frames, FeatureTracks.Track t, double[] p, double[] q, double[] uv) {
    double cost = 0;
    for (var ob : t.observations) {
      ImageFrame f = frames.get(ob.image);
      if (!f.camera.registered) continue;
      double[] measured = observed[f.id];
      double ox = measured[2 * ob.feature], oy = measured[2 * ob.feature + 1];
      if (!projectInto(f, p, oy, q, uv)) return Double.POSITIVE_INFINITY;
      double e = Math.hypot(uv[0] - ox, uv[1] - oy);
      cost += e <= 2 ? .5 * e * e : 2 * (e - 1);
    }
    return cost;
  }
}
