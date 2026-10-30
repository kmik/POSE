package sgm;

import org.opencv.core.*;

final class Camera {
  double[] rotation = Math3.identity(), translation = {0, 0, 0};
  boolean registered, fixed;
  double fx, fy, cx, cy;
  Camera reference;
  double positionSigma = 10, rotationSigma = Math.toRadians(10);

  Camera(double fx, double fy, double cx, double cy) {
    this.fx = fx;
    this.fy = fy;
    this.cx = cx;
    this.cy = cy;
  }

  double[] center() {
    return Math3.times(Math3.mv(Math3.transpose(rotation), translation), -1);
  }

  double[] view(double[] p) {
    return Math3.add(Math3.mv(rotation, p), translation);
  }

  double[] project(double[] p) {
    double[] q = view(p);
    if (q[2] <= 1e-8) return null;
    return new double[] {fx * q[0] / q[2] + cx, fy * q[1] / q[2] + cy};
  }

  Mat intrinsic() {
    return Math3.mat(new double[] {fx, 0, cx, 0, fy, cy, 0, 0, 1}, 3, 3);
  }

  Mat projection() {
    double[] p = new double[12];
    for (int i = 0; i < 3; i++) {
      System.arraycopy(rotation, 3 * i, p, 4 * i, 3);
      p[4 * i + 3] = translation[i];
    }
    return Math3.mat(p, 3, 4);
  }

  Camera copy() {
    Camera c = new Camera(fx, fy, cx, cy);
    c.rotation = rotation.clone();
    c.translation = translation.clone();
    c.registered = registered;
    c.fixed = fixed;
    c.reference = reference;
    c.positionSigma = positionSigma;
    c.rotationSigma = rotationSigma;
    return c;
  }
}
