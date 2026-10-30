package sgm;

/** Small symmetric eigensolver; coordinates are centered before forming covariance. */
final class SurfaceGeometry {
  record Plane(double[] normal, double[] center, double smallest, double middle, double largest) {}

  static Plane plane(double[][] points, int n) {
    if (n < 6) return null;
    double[] origin = points[0], mean = new double[3];
    for (int i = 0; i < n; i++) for (int k = 0; k < 3; k++) mean[k] += points[i][k] - origin[k];
    for (int k = 0; k < 3; k++) mean[k] /= n;
    double[][] a = new double[3][3], v = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    for (int i = 0; i < n; i++) {
      double[] d = new double[3];
      for (int k = 0; k < 3; k++) d[k] = points[i][k] - origin[k] - mean[k];
      for (int k = 0; k < 3; k++) for (int j = 0; j < 3; j++) a[k][j] += d[k] * d[j] / n;
    }
    for (int iteration = 0; iteration < 24; iteration++) {
      int p = 0, q = 1;
      if (Math.abs(a[0][2]) > Math.abs(a[p][q])) {
        p = 0;
        q = 2;
      }
      if (Math.abs(a[1][2]) > Math.abs(a[p][q])) {
        p = 1;
        q = 2;
      }
      if (Math.abs(a[p][q]) < 1e-15 * Math.max(1e-20, a[0][0] + a[1][1] + a[2][2])) break;
      double angle = .5 * Math.atan2(2 * a[p][q], a[q][q] - a[p][p]);
      double c = Math.cos(angle), s = Math.sin(angle), ap = a[p][p], aq = a[q][q], cross = a[p][q];
      for (int k = 0; k < 3; k++)
        if (k != p && k != q) {
          double x = a[k][p], y = a[k][q];
          a[k][p] = a[p][k] = c * x - s * y;
          a[k][q] = a[q][k] = s * x + c * y;
        }
      a[p][p] = c * c * ap - 2 * s * c * cross + s * s * aq;
      a[q][q] = s * s * ap + 2 * s * c * cross + c * c * aq;
      a[p][q] = a[q][p] = 0;
      for (int k = 0; k < 3; k++) {
        double x = v[k][p], y = v[k][q];
        v[k][p] = c * x - s * y;
        v[k][q] = s * x + c * y;
      }
    }
    int low = 0, high = 0;
    for (int k = 1; k < 3; k++) {
      if (a[k][k] < a[low][low]) low = k;
      if (a[k][k] > a[high][high]) high = k;
    }
    if (low == high) return null;
    int mid = 3 - low - high;
    return new Plane(
        new double[] {v[0][low], v[1][low], v[2][low]},
        Math3.add(origin, mean),
        Math.max(0, a[low][low]),
        Math.max(0, a[mid][mid]),
        Math.max(0, a[high][high]));
  }

  static double dot(double[] a, double[] b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  }
}
