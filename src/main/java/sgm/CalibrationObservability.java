package sgm;

import java.util.*;
import org.opencv.core.*;

/** Data-only local lens information after marginalizing structure and camera poses. */
final class CalibrationObservability {
  record Assessment(double[][] covariance, double minimumRelativeInformation, String reason) {
    double[] sigma() {
      if (covariance == null) return null;
      double[] s = new double[SharedCalibration.SIZE];
      for (int k = 0; k < s.length; k++) s[k] = Math.sqrt(covariance[k][k]);
      return s;
    }
  }

  static Assessment assess(List<ImageFrame> frames, List<FeatureTracks.Track> tracks,
      double huber, SharedCalibration lens, double noise) {
    var data = BundleAdjuster.calibrationDataInformation(frames, tracks, huber, lens);
    if (data == null) return new Assessment(null, 0, "Singular structure or observability workspace limit");
    double[] h = data.matrix();
    int n = (int) Math.round(Math.sqrt(h.length)), cameras = n - SharedCalibration.SIZE;
    int[] keys = java.util.stream.IntStream.range(0, SharedCalibration.LENS)
        .filter(k -> lens.active[k]).toArray();
    int m = keys.length;
    if (m == 0) return new Assessment(null, 0, "No active lens parameters");
    double[] marginal = new double[m * m], scale = new double[m];
    for (int i = 0; i < m; i++) {
      scale[i] = Math.sqrt(Math.max(0, data.observationDiagonal()[cameras + keys[i]]));
      if (!(scale[i] > 0)) return new Assessment(null, 0, "No image sensitivity to parameter");
      for (int j = 0; j < m; j++)
        marginal[i * m + j] = h[(cameras + keys[i]) * n + cameras + keys[j]];
    }
    if (cameras > 0) {
      double[] cs = new double[cameras], block = new double[cameras * cameras];
      // Normalize before elimination. A gauge's Schur diagonal can be tiny negative roundoff;
      // normalizing by that cancelled diagonal would magnify it into spurious information.
      for (int i = 0; i < cameras; i++) cs[i] = Math.sqrt(Math.max(1e-30, data.observationDiagonal()[i]));
      for (int i = 0; i < cameras; i++)
        for (int j = 0; j < cameras; j++) block[i * cameras + j] = h[i * n + j] / (cs[i] * cs[j]);
      Eigen e = eigen(block, cameras);
      if (e == null || e.values[0] <= 0)
        return new Assessment(null, 0, "Invalid camera information");
      for (int k = 0; k < cameras; k++) {
        if (e.values[k] < -e.values[0] * 1e-8)
          return new Assessment(null, 0, "Indefinite camera information");
        // The baseline scale gauge has no image information. A pseudoinverse removes that
        // nuisance direction without inventing information through LM damping.
        if (e.values[k] <= e.values[0] * 1e-10) continue;
        double[] cross = new double[m];
        for (int i = 0; i < m; i++)
          for (int j = 0; j < cameras; j++)
            cross[i] += h[(cameras + keys[i]) * n + j] * e.vectors[k * cameras + j] / cs[j];
        for (int i = 0; i < m; i++)
          for (int j = 0; j < m; j++) marginal[i * m + j] -= cross[i] * cross[j] / e.values[k];
      }
    }
    double[] normalized = new double[m * m];
    for (int i = 0; i < m; i++)
      for (int j = 0; j < m; j++)
        normalized[i * m + j] = .5 * (marginal[i * m + j] + marginal[j * m + i]) / (scale[i] * scale[j]);
    Eigen e = eigen(normalized, m);
    double minimum = e == null ? 0 : e.values[m - 1];
    // Normalize against the original lens sensitivity, not the marginal diagonal: a parameter
    // almost entirely absorbed by poses/points must not appear well conditioned after rescaling.
    if (e == null || minimum <= 1e-9)
      return new Assessment(null, minimum, "Lens change is absorbed by poses, structure or other active parameters");
    double[][] covariance = new double[SharedCalibration.SIZE][SharedCalibration.SIZE];
    for (int i = 0; i < m; i++)
      for (int j = 0; j < m; j++) {
        double v = 0;
        for (int k = 0; k < m; k++) v += e.vectors[k * m + i] * e.vectors[k * m + j] / e.values[k];
        covariance[keys[i]][keys[j]] = noise * noise * v / (scale[i] * scale[j]);
      }
    return new Assessment(covariance, minimum, "Data-only local information; conditional on retained fixed parameters");
  }

  private record Eigen(double[] values, double[] vectors) {}

  private static Eigen eigen(double[] matrix, int size) {
    Mat h = new Mat(size, size, CvType.CV_64F), values = new Mat(), vectors = new Mat();
    try {
      h.put(0, 0, matrix);
      if (!Core.eigen(h, values, vectors)) return null;
      double[] v = new double[size], q = new double[size * size];
      values.get(0, 0, v);
      vectors.get(0, 0, q);
      for (double x : v) if (!Double.isFinite(x)) return null;
      for (double x : q) if (!Double.isFinite(x)) return null;
      return new Eigen(v, q);
    } finally {
      h.release(); values.release(); vectors.release();
    }
  }
}
