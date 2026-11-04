package sgm;

import java.awt.image.BufferedImage;

/**
 * Marker refinement by patch matching (GCP8). A patch around a pinned marker is warped into
 * another image through the local ground-plane geometry at the GCP (a 2x2 map from the two
 * projections of a small cross on the plane) and searched around the predicted position by
 * normalized cross-correlation, with a parabolic subpixel peak. Only clear peaks are returned:
 * correlation at least {@link #MIN_SCORE}, not on the search border, and no other peak within 0.05
 * more than 3 px away (repetitive texture). Pixels are original (distorted) image pixels with the
 * top-left pixel centre at (0, 0), as the markers.
 */
final class GcpRefine {
  private GcpRefine() {}

  static final double MIN_SCORE = .7;
  static final int RADIUS = 20, SEARCH = 40;

  record Match(double u, double v, double score) {}

  /** Grey values (luma of 8-bit RGB) of a crop of a photograph; coordinates stay in the photograph. */
  static final class Gray {
    final int ox, oy, width, height;
    final float[] value;

    Gray(BufferedImage image, int x0, int y0, int w, int h) {
      ox = Math.max(0, x0);
      oy = Math.max(0, y0);
      width = Math.max(0, Math.min(image.getWidth(), x0 + w) - ox);
      height = Math.max(0, Math.min(image.getHeight(), y0 + h) - oy);
      value = new float[width * height];
      int[] row = new int[Math.max(1, width)];
      for (int y = 0; y < height; y++) {
        image.getRGB(ox, oy + y, width, 1, row, 0, width);
        for (int x = 0; x < width; x++) {
          int c = row[x];
          value[y * width + x] = .299f * (c >> 16 & 255) + .587f * (c >> 8 & 255) + .114f * (c & 255);
        }
      }
    }

    double at(double u, double v) {
      int x = (int) Math.floor(u) - ox, y = (int) Math.floor(v) - oy;
      if (x < 0 || y < 0 || x + 1 >= width || y + 1 >= height) return Double.NaN;
      double fx = u - Math.floor(u), fy = v - Math.floor(v);
      int k = y * width + x;
      return (1 - fy) * ((1 - fx) * value[k] + fx * value[k + 1]) + fy * ((1 - fx) * value[k + width] + fx * value[k + width + 1]);
    }
  }

  /**
   * Finds the GCP of pinned marker (ua, va) in image fa again in image fb. {@code point} is its
   * current 3D location; {@code zUp} selects a horizontal ground plane (georeferenced block),
   * otherwise the plane faces camera a.
   */
  static Match refine(ImageFrame fa, double ua, double va, BufferedImage photoA, ImageFrame fb, BufferedImage photoB, double[] point, boolean zUp) {
    double[] normal;
    if (zUp) normal = new double[] {0, 0, 1};
    else {
      double[] axis = Math3.mv(Math3.transpose(fa.camera.rotation), new double[] {0, 0, 1});
      normal = Math3.times(axis, -1 / Math3.norm(axis));
    }
    double[] helper = Math.abs(normal[0]) < .9 ? new double[] {1, 0, 0} : new double[] {0, 1, 0};
    double[] e1 = cross(normal, helper);
    e1 = Math3.times(e1, 1 / Math3.norm(e1));
    double[] e2 = cross(normal, e1);
    double depth = Math3.norm(Math3.sub(point, fa.camera.center()));
    double s = 8 * depth / fa.calibration.fx; // about 8 pixels in image a
    double[][] ja = jacobian(fa, point, e1, e2, s), jb = jacobian(fb, point, e1, e2, s);
    if (ja == null || jb == null) return null;
    double det = jb[0][0] * jb[1][1] - jb[0][1] * jb[1][0];
    if (!(Math.abs(det) > 1e-9)) return null;
    double[][] inv = {{jb[1][1] / det, -jb[0][1] / det}, {-jb[1][0] / det, jb[0][0] / det}};
    // b pixel offset -> a pixel offset.
    double[][] m = {
      {ja[0][0] * inv[0][0] + ja[0][1] * inv[1][0], ja[0][0] * inv[0][1] + ja[0][1] * inv[1][1]},
      {ja[1][0] * inv[0][0] + ja[1][1] * inv[1][0], ja[1][0] * inv[0][1] + ja[1][1] * inv[1][1]}
    };
    double scale = Math.sqrt(Math.abs(m[0][0] * m[1][1] - m[0][1] * m[1][0]));
    if (!(scale > .25 && scale < 4)) return null;
    double[] centre = GroundControl.project(fb, point);
    if (centre == null) return null;
    int reach = (int) Math.ceil(RADIUS * (Math.abs(m[0][0]) + Math.abs(m[0][1]) + Math.abs(m[1][0]) + Math.abs(m[1][1]))) + 3;
    Gray a = new Gray(photoA, (int) Math.floor(ua) - reach, (int) Math.floor(va) - reach, 2 * reach + 2, 2 * reach + 2);
    int cx = (int) Math.round(centre[0]), cy = (int) Math.round(centre[1]), span = 2 * SEARCH + 1, half = SEARCH + RADIUS + 1;
    Gray b = new Gray(photoB, cx - half, cy - half, 2 * half + 1, 2 * half + 1);
    int side = 2 * RADIUS + 1;
    double[] template = new double[side * side];
    double sum = 0, square = 0;
    for (int dy = -RADIUS; dy <= RADIUS; dy++)
      for (int dx = -RADIUS; dx <= RADIUS; dx++) {
        double v = a.at(ua + m[0][0] * dx + m[0][1] * dy, va + m[1][0] * dx + m[1][1] * dy);
        if (Double.isNaN(v)) return null;
        template[(dy + RADIUS) * side + dx + RADIUS] = v;
        sum += v;
        square += v * v;
      }
    int count = side * side;
    double mean = sum / count, spread = square - sum * sum / count;
    if (spread < count * 4) return null; // no texture to match
    for (int i = 0; i < count; i++) template[i] -= mean;
    double[] score = new double[span * span];
    double best = -2;
    int bx = 0, by = 0;
    for (int oy = -SEARCH; oy <= SEARCH; oy++)
      for (int ox = -SEARCH; ox <= SEARCH; ox++) {
        int x0 = cx + ox - RADIUS - b.ox, y0 = cy + oy - RADIUS - b.oy;
        double ncc = -2;
        if (x0 >= 0 && y0 >= 0 && x0 + side <= b.width && y0 + side <= b.height) {
          double sb = 0, sbb = 0, sab = 0;
          for (int dy = 0; dy < side; dy++) {
            int row = (y0 + dy) * b.width + x0, t = dy * side;
            for (int dx = 0; dx < side; dx++) {
              double v = b.value[row + dx];
              sb += v;
              sbb += v * v;
              sab += template[t + dx] * v;
            }
          }
          double varB = sbb - sb * sb / count;
          if (varB > count * 4) ncc = sab / Math.sqrt(spread * varB);
        }
        score[(oy + SEARCH) * span + ox + SEARCH] = ncc;
        if (ncc > best) {
          best = ncc;
          bx = ox;
          by = oy;
        }
      }
    if (best < MIN_SCORE || Math.abs(bx) == SEARCH || Math.abs(by) == SEARCH) return null;
    double rival = -2;
    for (int oy = -SEARCH; oy <= SEARCH; oy++)
      for (int ox = -SEARCH; ox <= SEARCH; ox++)
        if (Math.max(Math.abs(ox - bx), Math.abs(oy - by)) > 3)
          rival = Math.max(rival, score[(oy + SEARCH) * span + ox + SEARCH]);
    if (rival > best - .05) return null;
    double l = score[(by + SEARCH) * span + bx - 1 + SEARCH], r = score[(by + SEARCH) * span + bx + 1 + SEARCH];
    double up = score[(by - 1 + SEARCH) * span + bx + SEARCH], down = score[(by + 1 + SEARCH) * span + bx + SEARCH];
    return new Match(cx + bx + parabola(l, best, r), cy + by + parabola(up, best, down), best);
  }

  private static double parabola(double left, double centre, double right) {
    double d = left - 2 * centre + right;
    return d < 0 ? Math.clamp(.5 * (left - right) / d, -.5, .5) : 0;
  }

  private static double[][] jacobian(ImageFrame f, double[] point, double[] e1, double[] e2, double s) {
    double[] p0 = GroundControl.project(f, point),
        p1 = GroundControl.project(f, Math3.add(point, Math3.times(e1, s))),
        p2 = GroundControl.project(f, Math3.add(point, Math3.times(e2, s)));
    if (p0 == null || p1 == null || p2 == null) return null;
    return new double[][] {{(p1[0] - p0[0]) / s, (p2[0] - p0[0]) / s}, {(p1[1] - p0[1]) / s, (p2[1] - p0[1]) / s}};
  }

  private static double[] cross(double[] a, double[] b) {
    return new double[] {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
  }
}
