package sgm;

import java.util.*;

/** Fixed-camera, one-dimensional inverse-depth fit. Each measured camera contributes once. */
final class ReferenceReprojectionDepth {
  record Observation(Camera camera, double u, double v) {}

  record Fit(double depth, double relativeSigma, double before, double after) {}

  /**
   * Align a nearby depth-map sample to the base pixel before treating it as a correspondence.
   * Without this step, subpixel raster lookup offsets are mistaken for parallax/depth error. Uses a
   * locally constant base-camera Z patch; depth edges are checked separately by fusion.
   */
  static Observation link(Camera anchor, Camera camera, double[] point, double[] evidence) {
    double z = anchor.view(point)[2], measuredZ = anchor.view(evidence)[2];
    double[] center = anchor.center(), ray = Math3.times(Math3.sub(point, center), 1 / z);
    double[] d = Math3.mv(camera.rotation, ray), t = camera.view(center);
    double x = d[0] + t[0] * (1 / measuredZ),
        y = d[1] + t[1] * (1 / measuredZ),
        q = d[2] + t[2] * (1 / measuredZ);
    return new Observation(camera, camera.fx * x / q + camera.cx, camera.fy * y / q + camera.cy);
  }

  static Fit fit(
      Camera anchor,
      double[] point,
      double initialDepth,
      List<Observation> observations,
      double huber) {
    double z = anchor.view(point)[2];
    double[] center = anchor.center();
    double[] ray = Math3.times(Math3.sub(point, center), 1 / z);
    int n = observations.size();
    double[][] directions = new double[n][], offsets = new double[n][];
    for (int i = 0; i < n; i++) {
      Camera c = observations.get(i).camera;
      directions[i] = Math3.mv(c.rotation, ray);
      offsets[i] = c.view(center);
    }
    double rho = 1 / z;
    double before = evaluate(rho, observations, directions, offsets, huber)[0];
    double trial = 1 / initialDepth;
    if (trial > 0 && evaluate(trial, observations, directions, offsets, huber)[0] < before)
      rho = trial;
    for (int iteration = 0; iteration < 6; iteration++) {
      double[] f = evaluate(rho, observations, directions, offsets, huber);
      if (!(f[2] > 1e-20) || !Double.isFinite(f[1])) break;
      double step = Math.max(-rho * .25, Math.min(rho * .25, -f[1] / f[2]));
      boolean improved = false;
      for (int line = 0; line < 8; line++) {
        trial = rho + step;
        if (trial > 0 && evaluate(trial, observations, directions, offsets, huber)[0] < f[0]) {
          rho = trial;
          improved = true;
          break;
        }
        step *= .5;
      }
      if (!improved || Math.abs(step) < rho * 1e-10) break;
    }
    double[] end = evaluate(rho, observations, directions, offsets, huber);
    // Approximate conditional stereo uncertainty, never camera-calibration/pose accuracy.
    double sigma =
        Math.max(.5, Math.sqrt(end[3] / Math.max(1, 2 * n - 1)))
            / Math.sqrt(Math.max(1e-30, end[2]))
            / rho;
    return new Fit(1 / rho, sigma, before, end[0]);
  }

  private static double[] evaluate(
      double rho,
      List<Observation> observations,
      double[][] directions,
      double[][] offsets,
      double huber) {
    double objective = 0, gradient = 0, curvature = 0, residual = 0;
    for (int i = 0; i < observations.size(); i++) {
      Observation o = observations.get(i);
      double[] d = directions[i], t = offsets[i];
      double x = d[0] + t[0] * rho, y = d[1] + t[1] * rho, z = d[2] + t[2] * rho;
      if (!(z > 1e-12)) return new double[] {Double.POSITIVE_INFINITY, 0, 0, 0};
      double du = o.camera.fx * x / z + o.camera.cx - o.u;
      double dv = o.camera.fy * y / z + o.camera.cy - o.v;
      double ju = o.camera.fx * (t[0] * z - x * t[2]) / (z * z);
      double jv = o.camera.fy * (t[1] * z - y * t[2]) / (z * z);
      double r = Math.hypot(du, dv), w = r <= huber ? 1 : huber / r;
      objective += r <= huber ? .5 * r * r : huber * (r - .5 * huber);
      gradient += w * (ju * du + jv * dv);
      curvature += w * (ju * ju + jv * jv);
      residual += w * r * r;
    }
    return new double[] {objective, gradient, curvature, residual};
  }
}
