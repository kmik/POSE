package sgm;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;

/** Small double-precision geometry helpers. Row-major world-to-camera convention. */
final class Math3 {
  private Math3() {}

  static double[] identity() {
    return new double[] {1, 0, 0, 0, 1, 0, 0, 0, 1};
  }

  static double[] mul(double[] a, double[] b) {
    double[] out = new double[9];
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        for (int k = 0; k < 3; k++) out[3 * i + j] += a[3 * i + k] * b[3 * k + j];
    return out;
  }

  static double[] transpose(double[] a) {
    return new double[] {a[0], a[3], a[6], a[1], a[4], a[7], a[2], a[5], a[8]};
  }

  static double[] mv(double[] a, double[] b) {
    return new double[] {
      a[0] * b[0] + a[1] * b[1] + a[2] * b[2],
      a[3] * b[0] + a[4] * b[1] + a[5] * b[2],
      a[6] * b[0] + a[7] * b[1] + a[8] * b[2]
    };
  }

  static double[] add(double[] a, double[] b) {
    return new double[] {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
  }

  static double[] sub(double[] a, double[] b) {
    return new double[] {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
  }

  static double[] times(double[] a, double k) {
    return new double[] {a[0] * k, a[1] * k, a[2] * k};
  }

  static double norm(double[] a) {
    return Math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  }

  static Mat mat(double[] a, int rows, int columns) {
    Mat m = new Mat(rows, columns, CvType.CV_64F);
    m.put(0, 0, a);
    return m;
  }

  static double[] array(Mat m) {
    double[] a = new double[(int) (m.total() * m.channels())];
    m.get(0, 0, a);
    return a;
  }

  static double[] rotation(double[] rvec) {
    Mat r = mat(rvec, 3, 1), out = new Mat();
    try {
      Calib3d.Rodrigues(r, out);
      return array(out);
    } finally {
      r.release();
      out.release();
    }
  }

  static double[] rotationVector(double[] rotation) {
    Mat r = mat(rotation, 3, 3), out = new Mat();
    try {
      Calib3d.Rodrigues(r, out);
      return array(out);
    } finally {
      r.release();
      out.release();
    }
  }

  static double[] solve(double[][] a, double[] b) {
    int n = b.length;
    double[][] m = new double[n][n + 1];
    for (int i = 0; i < n; i++) {
      System.arraycopy(a[i], 0, m[i], 0, n);
      m[i][n] = b[i];
    }
    for (int k = 0; k < n; k++) {
      int pivot = k;
      for (int i = k + 1; i < n; i++) if (Math.abs(m[i][k]) > Math.abs(m[pivot][k])) pivot = i;
      if (Math.abs(m[pivot][k]) < 1e-14) return null;
      double[] swap = m[k];
      m[k] = m[pivot];
      m[pivot] = swap;
      double v = m[k][k];
      for (int j = k; j <= n; j++) m[k][j] /= v;
      for (int i = 0; i < n; i++)
        if (i != k) {
          v = m[i][k];
          for (int j = k; j <= n; j++) m[i][j] -= v * m[k][j];
        }
    }
    double[] x = new double[n];
    for (int i = 0; i < n; i++) x[i] = m[i][n];
    return x;
  }
}
