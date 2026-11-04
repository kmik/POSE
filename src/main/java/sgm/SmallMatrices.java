package sgm;

/** One Cholesky factorization, multiple right-hand sides, and a residual certificate. */
final class SmallMatrices {
  static double[] inverse(double[] a, int n) {
    double[] l = new double[n * n], inverse = new double[n * n], work = new double[n];
    for (int i = 0; i < n; i++)
      for (int j = 0; j <= i; j++) {
        double sum = a[i * n + j];
        for (int k = 0; k < j; k++) sum -= l[i * n + k] * l[j * n + k];
        if (i == j) {
          if (!(sum > 0) || !Double.isFinite(sum)) return null;
          l[i * n + j] = Math.sqrt(sum);
        } else l[i * n + j] = sum / l[j * n + j];
      }
    for (int column = 0; column < n; column++) {
      for (int i = 0; i < n; i++) {
        double v = i == column ? 1 : 0;
        for (int k = 0; k < i; k++) v -= l[i * n + k] * work[k];
        work[i] = v / l[i * n + i];
      }
      for (int i = n - 1; i >= 0; i--) {
        double v = work[i];
        for (int k = i + 1; k < n; k++) v -= l[k * n + i] * inverse[k * n + column];
        inverse[i * n + column] = v / l[i * n + i];
      }
    }
    double worst = 0;
    for (int i = 0; i < n; i++)
      for (int j = 0; j < n; j++) {
        double sum = 0;
        for (int k = 0; k < n; k++) sum += a[i * n + k] * inverse[k * n + j];
        worst = Math.max(worst, Math.abs(sum - (i == j ? 1 : 0)));
      }
    return Double.isFinite(worst) && worst < 1e-8 ? inverse : null;
  }
}
