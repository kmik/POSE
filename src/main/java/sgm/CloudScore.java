package sgm;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.*;
import java.util.stream.IntStream;

/**
 * Scores a dense cloud for parameter studies: against a reference cloud (Metashape, lidar) and on its
 * own. Kept apart from the pipeline; run as {@code java -cp ... sgm.CloudScore --help}.
 *
 * <p>The comparison separates where the cloud is from what shape it has. A rigid (or similarity)
 * transform onto the reference is fitted on stable surfaces only (reference cells that are locally
 * planar, or lidar ground) by trimmed point-to-plane ICP; its parameters are the position result.
 * Shape, deformation and completeness are measured after that transform. The transform and the
 * scored area can be saved as a frame ({@code --write-frame}) and reused unchanged for other clouds
 * ({@code --frame}), so clouds made from the same orientation are compared in one fixed frame and a
 * refit cannot hide a warp.
 */
final class CloudScore {
  private CloudScore() {}

  static final String HELP =
      """
      sgm.CloudScore --cloud A.las [--reference R.las|R.laz] [options]

        --cloud FILE           cloud to score (LAS/LAZ)
        --reference FILE       reference cloud in the same CRS (LAS/LAZ); without it only the
                               reference-free metrics (count, area, double sheets, far outliers)
        --fit rigid|similarity|none   transform fitted onto the reference (default rigid)
        --frame FILE           reuse a saved frame (transform + scored area); implies --fit none
        --write-frame FILE     save the frame of this comparison
        --stable planar|ground|all    surfaces used for fitting and stable metrics (default planar:
                               reference cells with a plane RMS <= --stable-rms; ground: lidar class 2)
        --stable-rms M         planarity limit of a stable 1 m cell (default 0.05)
        --area-margin M        scored area = reference cells within M of the cloud (default 10)
        --samples N            points sampled for distance metrics (default 400000)
        --tile M               tile size of the deformation metric (default 25)
        --region x0,y0,x1,y1   score only this rectangle
        --json FILE            write the result as JSON (always printed as text)
      """;

  // ---------------------------------------------------------------------------------------------
  // Clouds

  /** Points as float offsets from a shared origin (keeps millimetres over tens of kilometres). */
  static final class Cloud {
    final double ox, oy, oz;
    float[] x = new float[1 << 16], y = new float[1 << 16], z = new float[1 << 16];
    byte[] cls = new byte[1 << 16];
    int n;

    Cloud(double ox, double oy, double oz) {
      this.ox = ox;
      this.oy = oy;
      this.oz = oz;
    }

    void add(double px, double py, double pz, int c) {
      if (n == x.length) {
        int m = (int) Math.min(Integer.MAX_VALUE - 8, 2L * n);
        x = Arrays.copyOf(x, m);
        y = Arrays.copyOf(y, m);
        z = Arrays.copyOf(z, m);
        cls = Arrays.copyOf(cls, m);
      }
      x[n] = (float) (px - ox);
      y[n] = (float) (py - oy);
      z[n] = (float) (pz - oz);
      cls[n++] = (byte) c;
    }

    float[] bounds() {
      float[] b = {Float.MAX_VALUE, Float.MAX_VALUE, -Float.MAX_VALUE, -Float.MAX_VALUE};
      for (int i = 0; i < n; i++) {
        b[0] = Math.min(b[0], x[i]);
        b[1] = Math.min(b[1], y[i]);
        b[2] = Math.max(b[2], x[i]);
        b[3] = Math.max(b[3], y[i]);
      }
      return b;
    }
  }

  /** Reads a LAS/LAZ file; {@code box} (world x0,y0,x1,y1) limits the points kept, or null. */
  static Cloud read(Path path, double ox, double oy, double oz, double[] box) throws IOException {
    Cloud c = new Cloud(ox, oy, oz);
    try (LasFile.Reader r = LasFile.open(path)) {
      LasFile.Header h = r.header();
      int len = h.recordLength();
      byte[] buf = new byte[len * 65536];
      for (int got; (got = r.read(buf, 65536)) > 0; )
        for (int i = 0; i < got; i++) {
          int at = i * len;
          if (LasFile.withheld(h.format(), buf, at)) continue;
          double px = h.x(LasFile.rawX(buf, at)), py = h.y(LasFile.rawY(buf, at));
          if (box != null && (px < box[0] || py < box[1] || px > box[2] || py > box[3])) continue;
          c.add(px, py, h.z(LasFile.rawZ(buf, at)), LasFile.classification(h.format(), buf, at));
        }
    }
    return c;
  }

  // ---------------------------------------------------------------------------------------------
  // 2D grid index (cells sorted by row, then column)

  static final class Grid {
    final float x0, y0, cell;
    final int w, h;
    final int[] start, idx;

    Grid(Cloud c, float[] b, float cell) {
      this.cell = cell;
      x0 = b[0] - cell;
      y0 = b[1] - cell;
      w = (int) Math.min(1 << 20, (b[2] - x0) / cell + 2);
      h = (int) Math.min(1 << 20, (b[3] - y0) / cell + 2);
      if ((long) w * h > 400_000_000L) throw new IllegalArgumentException("Grid too large: " + w + " x " + h);
      start = new int[w * h + 1];
      int[] at = new int[c.n];
      for (int i = 0; i < c.n; i++) {
        int k = of(c.x[i], c.y[i]);
        at[i] = k;
        if (k >= 0) start[k + 1]++;
      }
      for (int k = 0; k < w * h; k++) start[k + 1] += start[k];
      idx = new int[start[w * h]];
      int[] fill = Arrays.copyOf(start, w * h);
      for (int i = 0; i < c.n; i++) if (at[i] >= 0) idx[fill[at[i]]++] = i;
    }

    int col(float x) {
      return (int) Math.floor((x - x0) / cell);
    }

    int row(float y) {
      return (int) Math.floor((y - y0) / cell);
    }

    int of(float x, float y) {
      int c = col(x), r = row(y);
      return c < 0 || r < 0 || c >= w || r >= h ? -1 : r * w + c;
    }

    int count(int k) {
      return start[k + 1] - start[k];
    }
  }

  // ---------------------------------------------------------------------------------------------
  // Local geometry

  /** Plane through points: centroid, unit normal (z >= 0) and RMS orthogonal residual. */
  record Plane(double cx, double cy, double cz, double nx, double ny, double nz, double rms, int count) {
    double distance(double x, double y, double z) {
      return (x - cx) * nx + (y - cy) * ny + (z - cz) * nz;
    }
  }

  /** PCA plane from moments about (ax, ay, az). */
  static Plane plane(double ax, double ay, double az, double n, double sx, double sy, double sz, double sxx, double sxy,
      double sxz, double syy, double syz, double szz) {
    double mx = sx / n, my = sy / n, mz = sz / n;
    double[][] c = {
      {sxx / n - mx * mx, sxy / n - mx * my, sxz / n - mx * mz},
      {sxy / n - mx * my, syy / n - my * my, syz / n - my * mz},
      {sxz / n - mx * mz, syz / n - my * mz, szz / n - mz * mz}
    };
    double[] values = new double[3];
    double[][] vectors = new double[3][3];
    eigen(c, values, vectors);
    int k = 0;
    for (int i = 1; i < 3; i++) if (values[i] < values[k]) k = i;
    double nx = vectors[0][k], ny = vectors[1][k], nz = vectors[2][k];
    if (nz < 0) {
      nx = -nx;
      ny = -ny;
      nz = -nz;
    }
    return new Plane(ax + mx, ay + my, az + mz, nx, ny, nz, Math.sqrt(Math.max(0, values[k])), (int) n);
  }

  /** Jacobi eigen decomposition of a symmetric 3x3 matrix (columns of v are the vectors). */
  static void eigen(double[][] a0, double[] d, double[][] v) {
    double[][] a = {a0[0].clone(), a0[1].clone(), a0[2].clone()};
    for (int i = 0; i < 3; i++) {
      Arrays.fill(v[i], 0);
      v[i][i] = 1;
    }
    for (int sweep = 0; sweep < 50; sweep++) {
      double off = Math.abs(a[0][1]) + Math.abs(a[0][2]) + Math.abs(a[1][2]);
      if (off < 1e-18) break;
      for (int p = 0; p < 2; p++)
        for (int q = p + 1; q < 3; q++) {
          if (Math.abs(a[p][q]) < 1e-30) continue;
          double theta = (a[q][q] - a[p][p]) / (2 * a[p][q]);
          double t = Math.signum(theta) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
          if (theta == 0) t = 1;
          double c = 1 / Math.sqrt(t * t + 1), s = t * c;
          for (int k = 0; k < 3; k++) {
            double akp = a[k][p], akq = a[k][q];
            a[k][p] = c * akp - s * akq;
            a[k][q] = s * akp + c * akq;
          }
          for (int k = 0; k < 3; k++) {
            double apk = a[p][k], aqk = a[q][k];
            a[p][k] = c * apk - s * aqk;
            a[q][k] = s * apk + c * aqk;
          }
          for (int k = 0; k < 3; k++) {
            double vkp = v[k][p], vkq = v[k][q];
            v[k][p] = c * vkp - s * vkq;
            v[k][q] = s * vkp + c * vkq;
          }
        }
    }
    for (int i = 0; i < 3; i++) d[i] = a[i][i];
  }

  /**
   * Distance from (x, y, z) to the reference: point-to-plane when at least 6 reference points lie
   * within {@code radius} (signed, positive = above the reference surface), else the nearest
   * reference point within 1 m horizontally (signed by height). NaN when the reference has no point
   * within 1 m horizontally (outside the reference).
   */
  static double distance(Cloud ref, Grid g, double x, double y, double z, double radius, int[] scratch) {
    float fx = (float) x, fy = (float) y;
    double hr = Math.max(1.0, radius);
    int reach = (int) Math.ceil(hr / g.cell);
    int c0 = g.col(fx), r0 = g.row(fy), found = 0;
    double r2 = radius * radius, best = Double.MAX_VALUE, bestDz = Double.NaN;
    for (int r = r0 - reach; r <= r0 + reach; r++) {
      if (r < 0 || r >= g.h) continue;
      for (int c = c0 - reach; c <= c0 + reach; c++) {
        if (c < 0 || c >= g.w) continue;
        int k = r * g.w + c;
        for (int a = g.start[k]; a < g.start[k + 1]; a++) {
          int i = g.idx[a];
          double dx = ref.x[i] - x, dy = ref.y[i] - y, dz = ref.z[i] - z, h2 = dx * dx + dy * dy;
          if (h2 > hr * hr) continue;
          double d2 = h2 + dz * dz;
          if (d2 < best) {
            best = d2;
            bestDz = -dz;
          }
          if (d2 <= r2 && found < scratch.length) scratch[found++] = i;
        }
      }
    }
    if (best == Double.MAX_VALUE) return Double.NaN;
    if (found >= 6) {
      double sx = 0, sy = 0, sz = 0, sxx = 0, sxy = 0, sxz = 0, syy = 0, syz = 0, szz = 0;
      for (int a = 0; a < found; a++) {
        int i = scratch[a];
        double px = ref.x[i] - x, py = ref.y[i] - y, pz = ref.z[i] - z;
        sx += px;
        sy += py;
        sz += pz;
        sxx += px * px;
        sxy += px * py;
        sxz += px * pz;
        syy += py * py;
        syz += py * pz;
        szz += pz * pz;
      }
      Plane p = plane(x, y, z, found, sx, sy, sz, sxx, sxy, sxz, syy, syz, szz);
      return p.distance(x, y, z);
    }
    return Math.copySign(Math.sqrt(best), bestDz);
  }

  // ---------------------------------------------------------------------------------------------
  // Transform: T(p) = centre + t + s * R * (p - centre), in origin-relative coordinates

  static final class Transform {
    double[] centre = new double[3], t = new double[3], r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    double s = 1;

    void apply(double x, double y, double z, double[] out) {
      double a = x - centre[0], b = y - centre[1], c = z - centre[2];
      out[0] = centre[0] + t[0] + s * (r[0] * a + r[1] * b + r[2] * c);
      out[1] = centre[1] + t[1] + s * (r[3] * a + r[4] * b + r[5] * c);
      out[2] = centre[2] + t[2] + s * (r[6] * a + r[7] * b + r[8] * c);
    }

    /** Rotation angles about x, y, z in degrees (small-angle readout of R). */
    double[] anglesDegrees() {
      return new double[] {
        Math.toDegrees(Math.atan2(r[7], r[8])), Math.toDegrees(-Math.asin(Math.max(-1, Math.min(1, r[6])))),
        Math.toDegrees(Math.atan2(r[3], r[0]))
      };
    }
  }

  /** Rodrigues rotation matrix of a rotation vector. */
  static double[] rotation(double wx, double wy, double wz) {
    double th = Math.sqrt(wx * wx + wy * wy + wz * wz);
    if (th < 1e-15) return new double[] {1, 0, 0, 0, 1, 0, 0, 0, 1};
    double kx = wx / th, ky = wy / th, kz = wz / th, c = Math.cos(th), s = Math.sin(th), v = 1 - c;
    return new double[] {
      c + kx * kx * v, kx * ky * v - kz * s, kx * kz * v + ky * s,
      ky * kx * v + kz * s, c + ky * ky * v, ky * kz * v - kx * s,
      kz * kx * v - ky * s, kz * ky * v + kx * s, c + kz * kz * v
    };
  }

  static double[] multiply(double[] a, double[] b) {
    double[] m = new double[9];
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) m[3 * i + j] = a[3 * i] * b[j] + a[3 * i + 1] * b[3 + j] + a[3 * i + 2] * b[6 + j];
    return m;
  }

  /** Result of the transform fit. */
  record Fit(int used, int iterations, double residualRms, double[] sigmaShift, String note) {}

  /**
   * Trimmed point-to-plane ICP of the sample points {@code idx} of {@code cloud} onto the
   * reference, gates 2, 1, 0.5, 0.25 m, Levenberg damping. Updates {@code tf}.
   */
  static Fit fit(Cloud cloud, int[] idx, Cloud ref, Grid g, Transform tf, boolean similarity, double planeRms, double radius, boolean low) {
    int m = idx.length, params = similarity ? 7 : 6;
    if (m < 50) return new Fit(m, 0, Double.NaN, new double[3], "too few stable samples to fit");
    double[] cx = new double[3];
    for (int i : idx) {
      cx[0] += cloud.x[i];
      cx[1] += cloud.y[i];
      cx[2] += cloud.z[i];
    }
    for (int k = 0; k < 3; k++) tf.centre[k] = cx[k] / m;
    // Start: median height offset to the nearest reference point within 1 m horizontally.
    double[] dz0 = new double[m];
    int[] scratch = new int[256];
    double[] q = new double[3];
    int z0 = 0;
    for (int i : idx) {
      tf.apply(cloud.x[i], cloud.y[i], cloud.z[i], q);
      double d = distance(ref, g, q[0], q[1], q[2], radius, scratch);
      if (!Double.isNaN(d)) dz0[z0++] = d;
    }
    if (z0 < 50) return new Fit(z0, 0, Double.NaN, new double[3], "cloud and reference do not overlap");
    double[] head = Arrays.copyOf(dz0, z0);
    Arrays.sort(head);
    // Ground mode: the low end (canopy lies above the ground the reference describes).
    tf.t[2] -= head[low ? z0 / 10 : z0 / 2];
    double[] gates = {2, 1, .5, .25};
    int iterations = 0, used = 0;
    double rms = Double.NaN;
    double[][] normal = new double[params][params];
    double sigma0 = 0;
    for (double gate : gates)
      for (int it = 0; it < 15; it++) {
        iterations++;
        double[] res = new double[m];
        double[][] rows = new double[m][];
        int[] ok = new int[m];
        double[] o = {tf.centre[0] + tf.t[0], tf.centre[1] + tf.t[1], tf.centre[2] + tf.t[2]};
        IntStream.range(0, m).parallel().forEach(j -> {
          int i = idx[j];
          double[] p = new double[3];
          tf.apply(cloud.x[i], cloud.y[i], cloud.z[i], p);
          Plane pl = localPlane(ref, g, p[0], p[1], p[2], radius, new int[256]);
          if (pl == null || pl.rms() > planeRms) return;
          double d = pl.distance(p[0], p[1], p[2]);
          if (Math.abs(d) > gate) return;
          double ax = p[0] - o[0], ay = p[1] - o[1], az = p[2] - o[2];
          double[] row = new double[params];
          row[0] = ay * pl.nz() - az * pl.ny();
          row[1] = az * pl.nx() - ax * pl.nz();
          row[2] = ax * pl.ny() - ay * pl.nx();
          row[3] = pl.nx();
          row[4] = pl.ny();
          row[5] = pl.nz();
          if (params == 7) row[6] = ax * pl.nx() + ay * pl.ny() + az * pl.nz();
          res[j] = d;
          rows[j] = row;
          ok[j] = 1;
        });
        // Trim: |d| within 3 robust sigmas (at least 2 cm).
        double[] abs = new double[m];
        int k = 0;
        for (int j = 0; j < m; j++) if (ok[j] == 1) abs[k++] = Math.abs(res[j]);
        if (k < 50) return new Fit(k, iterations, Double.NaN, new double[3], "too few samples near the reference surface");
        double[] sorted = Arrays.copyOf(abs, k);
        Arrays.sort(sorted);
        double limit = Math.max(.02, 3 * 1.4826 * sorted[k / 2]);
        for (double[] row : normal) Arrays.fill(row, 0);
        double[] rhs = new double[params];
        double ss = 0;
        used = 0;
        for (int j = 0; j < m; j++) {
          if (ok[j] == 0 || Math.abs(res[j]) > limit) continue;
          double[] row = rows[j];
          for (int a = 0; a < params; a++) {
            rhs[a] -= row[a] * res[j];
            for (int b = 0; b < params; b++) normal[a][b] += row[a] * row[b];
          }
          ss += res[j] * res[j];
          used++;
        }
        rms = Math.sqrt(ss / Math.max(1, used));
        sigma0 = rms;
        double[][] damped = new double[params][];
        for (int a = 0; a < params; a++) {
          damped[a] = normal[a].clone();
          damped[a][a] += 1e-6 * Math.max(1e-12, normal[a][a]) + 1e-9;
        }
        double[] dx = solve(damped, rhs);
        if (dx == null) return new Fit(used, iterations, rms, new double[3], "singular fit");
        double[] dr = rotation(dx[0], dx[1], dx[2]);
        tf.r = multiply(dr, tf.r);
        double ds = params == 7 ? dx[6] : 0;
        tf.s *= 1 + ds;
        for (int a = 0; a < 3; a++) tf.t[a] += dx[3 + a];
        double step = Math.sqrt(dx[3] * dx[3] + dx[4] * dx[4] + dx[5] * dx[5]);
        double turn = Math.sqrt(dx[0] * dx[0] + dx[1] * dx[1] + dx[2] * dx[2]);
        if (step < 1e-4 && turn < 1e-7 && Math.abs(ds) < 1e-7) break;
      }
    // Shift precision: sigma0 * sqrt(diag(N^-1)) of the translation block.
    double[] sigmaShift = new double[3];
    double[][] inv = invert(normal);
    if (inv != null) for (int a = 0; a < 3; a++) sigmaShift[a] = sigma0 * Math.sqrt(Math.max(0, inv[3 + a][3 + a]));
    String note = "";
    if (inv == null) note = "fit is singular";
    else if (Math.max(sigmaShift[0], sigmaShift[1]) > .05) note = "horizontal shift weakly determined (flat stable surfaces)";
    return new Fit(used, iterations, rms, sigmaShift, note);
  }

  /** Reference plane near (x, y, z): at least 6 points within radius, else null. */
  static Plane localPlane(Cloud ref, Grid g, double x, double y, double z, double radius, int[] scratch) {
    float fx = (float) x, fy = (float) y;
    int reach = (int) Math.ceil(radius / g.cell), c0 = g.col(fx), r0 = g.row(fy), found = 0;
    double r2 = radius * radius;
    for (int r = r0 - reach; r <= r0 + reach; r++) {
      if (r < 0 || r >= g.h) continue;
      for (int c = c0 - reach; c <= c0 + reach; c++) {
        if (c < 0 || c >= g.w) continue;
        int k = r * g.w + c;
        for (int a = g.start[k]; a < g.start[k + 1] && found < scratch.length; a++) {
          int i = g.idx[a];
          double dx = ref.x[i] - x, dy = ref.y[i] - y, dz = ref.z[i] - z;
          if (dx * dx + dy * dy + dz * dz <= r2) scratch[found++] = i;
        }
      }
    }
    if (found < 6) return null;
    double sx = 0, sy = 0, sz = 0, sxx = 0, sxy = 0, sxz = 0, syy = 0, syz = 0, szz = 0;
    for (int a = 0; a < found; a++) {
      int i = scratch[a];
      double px = ref.x[i] - x, py = ref.y[i] - y, pz = ref.z[i] - z;
      sx += px;
      sy += py;
      sz += pz;
      sxx += px * px;
      sxy += px * py;
      sxz += px * pz;
      syy += py * py;
      syz += py * pz;
      szz += pz * pz;
    }
    return plane(x, y, z, found, sx, sy, sz, sxx, sxy, sxz, syy, syz, szz);
  }

  static double[] solve(double[][] a0, double[] b0) {
    int n = b0.length;
    double[][] a = new double[n][];
    for (int i = 0; i < n; i++) a[i] = a0[i].clone();
    double[] b = b0.clone();
    for (int c = 0; c < n; c++) {
      int p = c;
      for (int r = c + 1; r < n; r++) if (Math.abs(a[r][c]) > Math.abs(a[p][c])) p = r;
      if (Math.abs(a[p][c]) < 1e-300) return null;
      double[] tr = a[c];
      a[c] = a[p];
      a[p] = tr;
      double tb = b[c];
      b[c] = b[p];
      b[p] = tb;
      for (int r = c + 1; r < n; r++) {
        double f = a[r][c] / a[c][c];
        for (int k = c; k < n; k++) a[r][k] -= f * a[c][k];
        b[r] -= f * b[c];
      }
    }
    double[] x = new double[n];
    for (int r = n - 1; r >= 0; r--) {
      double s = b[r];
      for (int k = r + 1; k < n; k++) s -= a[r][k] * x[k];
      x[r] = s / a[r][r];
    }
    return x;
  }

  static double[][] invert(double[][] a) {
    int n = a.length;
    double[][] out = new double[n][n];
    for (int c = 0; c < n; c++) {
      double[] e = new double[n];
      e[c] = 1;
      double[] col = solve(a, e);
      if (col == null) return null;
      for (int r = 0; r < n; r++) out[r][c] = col[r];
    }
    return out;
  }

  // ---------------------------------------------------------------------------------------------
  // Frame: transform + scored area, reusable across clouds

  static final class Frame {
    double ox, oy, oz;
    Transform tf = new Transform();
    float ax0, ay0, acell;
    int aw, ah;
    BitSet area;

    void save(Path p) throws IOException {
      Properties pr = new Properties();
      pr.setProperty("origin", ox + "," + oy + "," + oz);
      pr.setProperty("centre", join(tf.centre));
      pr.setProperty("shift", join(tf.t));
      pr.setProperty("rotation", join(tf.r));
      pr.setProperty("scale", Double.toString(tf.s));
      pr.setProperty("area", ax0 + "," + ay0 + "," + acell + "," + aw + "," + ah);
      pr.setProperty("area.bits", Base64.getEncoder().encodeToString(area.toByteArray()));
      try (Writer w = Files.newBufferedWriter(p, StandardCharsets.UTF_8)) {
        pr.store(w, "CloudScore frame: transform (origin-relative) and scored area");
      }
    }

    static Frame load(Path p) throws IOException {
      Properties pr = new Properties();
      try (Reader r = Files.newBufferedReader(p, StandardCharsets.UTF_8)) {
        pr.load(r);
      }
      Frame f = new Frame();
      double[] o = numbers(pr.getProperty("origin"));
      f.ox = o[0];
      f.oy = o[1];
      f.oz = o[2];
      f.tf.centre = numbers(pr.getProperty("centre"));
      f.tf.t = numbers(pr.getProperty("shift"));
      f.tf.r = numbers(pr.getProperty("rotation"));
      f.tf.s = Double.parseDouble(pr.getProperty("scale"));
      double[] a = numbers(pr.getProperty("area"));
      f.ax0 = (float) a[0];
      f.ay0 = (float) a[1];
      f.acell = (float) a[2];
      f.aw = (int) a[3];
      f.ah = (int) a[4];
      f.area = BitSet.valueOf(Base64.getDecoder().decode(pr.getProperty("area.bits")));
      return f;
    }

    boolean inArea(double x, double y) {
      int c = (int) Math.floor((x - ax0) / acell), r = (int) Math.floor((y - ay0) / acell);
      return c >= 0 && r >= 0 && c < aw && r < ah && area.get(r * aw + c);
    }
  }

  static String join(double[] v) {
    StringBuilder b = new StringBuilder();
    for (int i = 0; i < v.length; i++) b.append(i == 0 ? "" : ",").append(v[i]);
    return b.toString();
  }

  static double[] numbers(String s) {
    String[] p = s.split(",");
    double[] v = new double[p.length];
    for (int i = 0; i < p.length; i++) v[i] = Double.parseDouble(p[i].strip());
    return v;
  }

  // ---------------------------------------------------------------------------------------------
  // Settings and result

  static final class Settings {
    Path cloud, reference, frame, writeFrame, json;
    String fit = "rigid", stable = "planar";
    double stableRms = .05, areaMargin = 10, tile = 25, stableCell = Double.NaN, planeRadius = Double.NaN;
    int samples = 400_000;
    double[] region;

    static Settings parse(String[] a) {
      Settings s = new Settings();
      for (int i = 0; i < a.length; i++) {
        String k = a[i];
        if (k.equals("--help") || k.equals("-h")) throw new IllegalArgumentException(HELP);
        if (i + 1 >= a.length) throw new IllegalArgumentException("Missing value for " + k + "\n" + HELP);
        String v = a[++i];
        switch (k) {
          case "--cloud" -> s.cloud = Path.of(v);
          case "--reference" -> s.reference = Path.of(v);
          case "--frame" -> s.frame = Path.of(v);
          case "--write-frame" -> s.writeFrame = Path.of(v);
          case "--json" -> s.json = Path.of(v);
          case "--fit" -> s.fit = choice(k, v, "rigid", "similarity", "none");
          case "--stable" -> s.stable = choice(k, v, "planar", "ground", "all");
          case "--stable-rms" -> s.stableRms = Double.parseDouble(v);
          case "--stable-cell" -> s.stableCell = Double.parseDouble(v);
          case "--plane-radius" -> s.planeRadius = Double.parseDouble(v);
          case "--area-margin" -> s.areaMargin = Double.parseDouble(v);
          case "--tile" -> s.tile = Double.parseDouble(v);
          case "--samples" -> s.samples = Integer.parseInt(v);
          case "--region" -> {
            s.region = numbers(v);
            if (s.region.length != 4) throw new IllegalArgumentException("--region needs x0,y0,x1,y1");
          }
          default -> throw new IllegalArgumentException("Unknown option " + k + "\n" + HELP);
        }
      }
      if (s.cloud == null) throw new IllegalArgumentException("--cloud is required\n" + HELP);
      if (s.frame != null && s.reference == null) throw new IllegalArgumentException("--frame needs --reference");
      if (s.frame != null) s.fit = "none";
      // Ground mode is for airborne lidar: sparse ground returns need larger cells and planes.
      boolean ground = s.stable.equals("ground");
      if (Double.isNaN(s.stableCell)) s.stableCell = ground ? 6 : 1;
      if (Double.isNaN(s.planeRadius)) s.planeRadius = ground ? 3 : .5;
      return s;
    }

    static String choice(String k, String v, String... allowed) {
      for (String a : allowed) if (a.equals(v)) return v;
      throw new IllegalArgumentException(k + " must be one of " + String.join(", ", allowed));
    }
  }

  /** Ordered key -> value result (numbers, strings, nested maps). */
  static final class Result extends LinkedHashMap<String, Object> {
    Result section(String name) {
      return (Result) computeIfAbsent(name, k -> new Result());
    }

    String json(int indent) {
      StringBuilder b = new StringBuilder("{\n");
      int i = 0;
      for (var e : entrySet()) {
        b.append("  ".repeat(indent + 1)).append('"').append(e.getKey()).append("\": ");
        Object v = e.getValue();
        if (v instanceof Result r) b.append(r.json(indent + 1));
        else if (v instanceof String s) b.append('"').append(s.replace("\\", "\\\\").replace("\"", "\\\"")).append('"');
        else if (v instanceof double[] d) {
          b.append('[');
          for (int k = 0; k < d.length; k++) b.append(k == 0 ? "" : ", ").append(number(d[k]));
          b.append(']');
        } else if (v instanceof Double d) b.append(number(d));
        else b.append(v);
        b.append(++i < size() ? ",\n" : "\n");
      }
      return b.append("  ".repeat(indent)).append('}').toString();
    }

    static String number(double d) {
      if (Double.isNaN(d) || Double.isInfinite(d)) return "null";
      return String.format(Locale.ROOT, "%.6g", d);
    }

    String text(String prefix) {
      StringBuilder b = new StringBuilder();
      for (var e : entrySet()) {
        Object v = e.getValue();
        if (v instanceof Result r) b.append(prefix).append(e.getKey()).append(":\n").append(r.text(prefix + "  "));
        else {
          String s = v instanceof double[] d ? Arrays.stream(d).mapToObj(Result::number).toList().toString()
              : v instanceof Double d ? number(d) : String.valueOf(v);
          b.append(prefix).append(e.getKey()).append(" = ").append(s).append('\n');
        }
      }
      return b.toString();
    }
  }

  // ---------------------------------------------------------------------------------------------
  // Scoring

  public static void main(String[] args) throws Exception {
    Settings s;
    try {
      s = Settings.parse(args);
    } catch (IllegalArgumentException e) {
      System.err.println(e.getMessage());
      System.exit(args.length == 0 || Arrays.asList(args).contains("--help") ? 0 : 2);
      return;
    }
    Result r = score(s);
    System.out.print(r.text(""));
    if (s.json != null) Files.writeString(s.json, r.json(0) + "\n");
  }

  static Result score(Settings s) throws IOException {
    Result out = new Result();
    LasFile.Header ch = LasFile.header(s.cloud);
    Frame frame = s.frame != null ? Frame.load(s.frame) : null;
    double ox = frame != null ? frame.ox : Math.floor(ch.min()[0]), oy = frame != null ? frame.oy : Math.floor(ch.min()[1]),
        oz = frame != null ? frame.oz : Math.floor(ch.min()[2]);
    Cloud cloud = read(s.cloud, ox, oy, oz, s.region);
    out.put("cloud", s.cloud.toString());
    out.put("points", (double) cloud.n);
    if (cloud.n == 0) {
      out.put("error", "no points");
      return out;
    }
    float[] cb = cloud.bounds();
    selfMetrics(cloud, cb, out.section("artefacts"));
    if (s.reference == null) return out;

    double margin = Math.max(s.areaMargin, 5) + 5;
    double[] box = {cb[0] + ox - margin, cb[1] + oy - margin, cb[2] + ox + margin, cb[3] + oy + margin};
    if (s.region != null) box = s.region.clone();
    Cloud ref = read(s.reference, ox, oy, oz, box);
    out.put("reference", s.reference.toString());
    out.put("reference_points", (double) ref.n);
    if (ref.n < 100) {
      out.put("error", "the reference has fewer than 100 points around the cloud (same CRS?)");
      return out;
    }
    float[] rb = ref.bounds();
    Grid g = new Grid(ref, rb, .5f);

    // Stable reference cells (planar: 1 m; ground: lidar class 2, 6 m by default).
    Grid g1 = new Grid(ref, rb, (float) s.stableCell);
    BitSet stable = stableCells(ref, g1, s.stable, s.stableRms, out);
    boolean ground = out.section("position").get("stable_mode").equals("ground");
    // Ground mode fits and measures the ground against the lidar ground points only.
    Cloud gref = ground ? subset(ref, 2) : ref;
    Grid gg = ground ? new Grid(gref, rb, .5f) : g;

    // Transform.
    Transform tf = frame != null ? frame.tf : new Transform();
    Result position = out.section("position");
    position.put("fit", s.fit);
    if (!s.fit.equals("none")) {
      // Planar mode: any point whose local reference patch is planar (walls and sloped roofs fix the
      // horizontal shift; vegetation fails the planarity test). Ground/all: the stable cells.
      boolean planar = ((Result) out.get("position")).get("stable_mode").equals("planar");
      int[] fitIdx = stride(cloud, s.samples / 4, i -> planar || stableAt(g1, stable, cloud.x[i], cloud.y[i]));
      Fit f = fit(cloud, fitIdx, gref, gg, tf, s.fit.equals("similarity"), planar ? s.stableRms : Double.POSITIVE_INFINITY, s.planeRadius, ground);
      position.put("fit_samples", (double) f.used());
      position.put("fit_iterations", (double) f.iterations());
      position.put("fit_residual_rms_m", f.residualRms());
      position.put("shift_sigma_m", f.sigmaShift());
      if (!f.note().isEmpty()) position.put("note", f.note());
    }
    position.put("shift_m", tf.t.clone());
    position.put("rotation_deg", tf.anglesDegrees());
    position.put("scale_ppm", (tf.s - 1) * 1e6);

    // Apply the transform to the cloud in place.
    double[] q = new double[3];
    for (int i = 0; i < cloud.n; i++) {
      tf.apply(cloud.x[i], cloud.y[i], cloud.z[i], q);
      cloud.x[i] = (float) q[0];
      cloud.y[i] = (float) q[1];
      cloud.z[i] = (float) q[2];
    }
    cb = cloud.bounds();

    // Scored area.
    if (frame == null) {
      frame = new Frame();
      frame.ox = ox;
      frame.oy = oy;
      frame.oz = oz;
      frame.tf = tf;
      // Sparse lidar leaves most 0.5 m cells empty: ground mode scores completeness on 2 m cells.
      Grid ga = ground ? new Grid(ref, rb, 2f) : g;
      frame.acell = ga.cell;
      frame.ax0 = ga.x0;
      frame.ay0 = ga.y0;
      frame.aw = ga.w;
      frame.ah = ga.h;
      frame.area = area(cloud, ref, ga, s.areaMargin);
    }
    if (s.writeFrame != null) frame.save(s.writeFrame);

    shapeMetrics(cloud, ref, g, gref, gg, ground, g1, stable, frame, s, out);
    return out;
  }

  interface IntTest {
    boolean test(int i);
  }

  /** Up to {@code max} evenly strided point indices passing the test. */
  static int[] stride(Cloud c, int max, IntTest test) {
    int count = 0;
    for (int i = 0; i < c.n; i++) if (test.test(i)) count++;
    int step = Math.max(1, (count + max - 1) / Math.max(1, max)), seen = 0, k = 0;
    int[] out = new int[Math.min(count, max) + 1];
    for (int i = 0; i < c.n && k < out.length; i++)
      if (test.test(i) && seen++ % step == 0) out[k++] = i;
    return Arrays.copyOf(out, k);
  }

  static boolean stableAt(Grid g1, BitSet stable, float x, float y) {
    int k = g1.of(x, y);
    return k >= 0 && stable.get(k);
  }

  /** Reference 1 m cells usable for fitting and stable metrics. */
  static BitSet stableCells(Cloud ref, Grid g1, String mode, double rmsLimit, Result out) {
    BitSet stable = new BitSet(g1.w * g1.h);
    boolean ground = mode.equals("ground");
    if (ground) {
      int classified = 0;
      for (int i = 0; i < ref.n; i++) if (ref.cls[i] == 2) classified++;
      if (classified < 100) {
        out.section("position").put("stable_note", "reference has no ground class; planar cells used");
        ground = false;
      }
    }
    boolean all = mode.equals("all"), groundOnly = ground;
    int cells = g1.w * g1.h;
    IntStream.range(0, cells).parallel().forEach(k -> {
      int n = 0;
      double sx = 0, sy = 0, sz = 0, sxx = 0, sxy = 0, sxz = 0, syy = 0, syz = 0, szz = 0;
      int i0 = g1.start[k] < g1.start[k + 1] ? g1.idx[g1.start[k]] : -1;
      if (i0 < 0) return;
      double ax = ref.x[i0], ay = ref.y[i0], az = ref.z[i0];
      for (int a = g1.start[k]; a < g1.start[k + 1]; a++) {
        int i = g1.idx[a];
        if (groundOnly && ref.cls[i] != 2) continue;
        double px = ref.x[i] - ax, py = ref.y[i] - ay, pz = ref.z[i] - az;
        n++;
        sx += px;
        sy += py;
        sz += pz;
        sxx += px * px;
        sxy += px * py;
        sxz += px * pz;
        syy += py * py;
        syz += py * pz;
        szz += pz * pz;
      }
      if (n < 8) return;
      if (all) {
        synchronized (stable) {
          stable.set(k);
        }
        return;
      }
      Plane p = plane(ax, ay, az, n, sx, sy, sz, sxx, sxy, sxz, syy, syz, szz);
      if (p.rms() <= (groundOnly ? 3 * rmsLimit : rmsLimit) && p.nz() >= .5)
        synchronized (stable) {
          stable.set(k);
        }
    });
    Result pos = out.section("position");
    pos.put("stable_mode", groundOnly ? "ground" : all ? "all" : "planar");
    pos.put("stable_cells", (double) stable.cardinality());
    return stable;
  }

  /** Reference 0.5 m cells within {@code margin} of the cloud (cloud 2 m cells, dilated). */
  static BitSet area(Cloud cloud, Cloud ref, Grid g, double margin) {
    float c2 = 2f;
    int w2 = (int) ((g.w * g.cell) / c2) + 2, h2 = (int) ((g.h * g.cell) / c2) + 2;
    BitSet near = new BitSet(w2 * h2);
    for (int i = 0; i < cloud.n; i++) {
      int c = (int) Math.floor((cloud.x[i] - g.x0) / c2), r = (int) Math.floor((cloud.y[i] - g.y0) / c2);
      if (c >= 0 && r >= 0 && c < w2 && r < h2) near.set(r * w2 + c);
    }
    int reach = (int) Math.ceil(margin / c2);
    BitSet grown = new BitSet(w2 * h2);
    for (int k = near.nextSetBit(0); k >= 0; k = near.nextSetBit(k + 1)) {
      int r = k / w2, c = k % w2;
      for (int dr = -reach; dr <= reach; dr++)
        for (int dc = -reach; dc <= reach; dc++) {
          int rr = r + dr, cc = c + dc;
          if (rr >= 0 && cc >= 0 && rr < h2 && cc < w2) grown.set(rr * w2 + cc);
        }
    }
    BitSet area = new BitSet(g.w * g.h);
    for (int k = 0; k < g.w * g.h; k++) {
      if (g.count(k) == 0) continue;
      float x = g.x0 + (k % g.w + .5f) * g.cell, y = g.y0 + (k / g.w + .5f) * g.cell;
      int c = (int) Math.floor((x - g.x0) / c2), r = (int) Math.floor((y - g.y0) / c2);
      if (c >= 0 && r >= 0 && c < w2 && r < h2 && grown.get(r * w2 + c)) area.set(k);
    }
    return area;
  }

  /** Points of one LAS class (same origin). */
  static Cloud subset(Cloud c, int cls) {
    Cloud out = new Cloud(c.ox, c.oy, c.oz);
    for (int i = 0; i < c.n; i++) if (c.cls[i] == cls) out.add(c.x[i] + c.ox, c.y[i] + c.oy, c.z[i] + c.oz, cls);
    return out;
  }

  static void shapeMetrics(Cloud cloud, Cloud ref, Grid g, Cloud gref, Grid gg, boolean ground, Grid g1, BitSet stable, Frame frame, Settings s, Result out) {
    int[] idx = stride(cloud, s.samples, i -> true);
    int m = idx.length;
    // d: to the whole reference. sd: on stable cells, the stable-surface distance (ground mode: to
    // the lidar ground, kept only for ground hits within 1.5 m, so canopy does not count as error).
    double[] d = new double[m], sd = new double[m];
    boolean[] onStable = new boolean[m];
    IntStream.range(0, m).parallel().forEach(j -> {
      int i = idx[j];
      d[j] = distance(ref, g, cloud.x[i], cloud.y[i], cloud.z[i], s.planeRadius, new int[256]);
      onStable[j] = stableAt(g1, stable, cloud.x[i], cloud.y[i]);
      sd[j] = Double.NaN;
      if (onStable[j]) {
        double v = ground ? distance(gref, gg, cloud.x[i], cloud.y[i], cloud.z[i], s.planeRadius, new int[256]) : d[j];
        sd[j] = ground && Math.abs(v) > 1.5 ? Double.NaN : v;
      }
    });
    Result shape = out.section("shape");
    double[] all = new double[m], st = new double[m], signed = new double[m];
    int na = 0, ns = 0, outside = 0, stableSamples = 0;
    for (int j = 0; j < m; j++) {
      if (onStable[j]) stableSamples++;
      if (!Double.isNaN(sd[j])) {
        st[ns] = Math.abs(sd[j]);
        signed[ns++] = sd[j];
      }
      if (Double.isNaN(d[j])) {
        outside++;
        continue;
      }
      all[na++] = Math.abs(d[j]);
    }
    shape.put("samples", (double) m);
    shape.put("outside_reference_fraction", m == 0 ? Double.NaN : outside / (double) m);
    distances(Arrays.copyOf(all, na), shape.section("all"));
    distances(Arrays.copyOf(st, ns), shape.section("stable"));
    double[] sg = Arrays.copyOf(signed, ns);
    Arrays.sort(sg);
    shape.section("stable").put("bias_m", ns == 0 ? Double.NaN : sg[ns / 2]);
    if (ground) shape.section("stable").put("ground_hit_fraction", stableSamples == 0 ? Double.NaN : ns / (double) stableSamples);

    // Deformation: per-tile median signed distance on stable surfaces.
    Map<Long, List<Double>> tiles = new HashMap<>();
    Map<Long, double[]> centres = new HashMap<>();
    for (int j = 0; j < m; j++) {
      if (Double.isNaN(sd[j]) || Math.abs(sd[j]) > 1) continue;
      int i = idx[j];
      long key = ((long) Math.floor(cloud.x[i] / s.tile) << 32) ^ (Math.floorMod((long) Math.floor(cloud.y[i] / s.tile), 1L << 32));
      tiles.computeIfAbsent(key, k -> new ArrayList<>()).add(sd[j]);
      double[] c = centres.computeIfAbsent(key, k -> new double[3]);
      c[0] += cloud.x[i];
      c[1] += cloud.y[i];
      c[2]++;
    }
    List<double[]> medians = new ArrayList<>();
    for (var e : tiles.entrySet()) {
      List<Double> v = e.getValue();
      if (v.size() < 20) continue;
      Collections.sort(v);
      double[] c = centres.get(e.getKey());
      medians.add(new double[] {c[0] / c[2], c[1] / c[2], v.get(v.size() / 2)});
    }
    Result def = out.section("deformation");
    def.put("tile_m", s.tile);
    def.put("tiles", (double) medians.size());
    if (medians.size() >= 3) {
      double mean = medians.stream().mapToDouble(t -> t[2]).average().orElse(0);
      double var = medians.stream().mapToDouble(t -> (t[2] - mean) * (t[2] - mean)).sum() / medians.size();
      double[] v = medians.stream().mapToDouble(t -> t[2]).sorted().toArray();
      def.put("tile_offset_std_m", Math.sqrt(var));
      def.put("tile_offset_p5_p95_m", v[(int) (.95 * (v.length - 1))] - v[(int) (.05 * (v.length - 1))]);
      // Planar trend z = a + b x + c y over the tile medians; the rest is warp (doming, bending).
      double[][] n = new double[3][3];
      double[] rhs = new double[3];
      double mx = medians.stream().mapToDouble(t -> t[0]).average().orElse(0), my = medians.stream().mapToDouble(t -> t[1]).average().orElse(0);
      for (double[] t : medians) {
        double[] row = {1, t[0] - mx, t[1] - my};
        for (int a = 0; a < 3; a++) {
          rhs[a] += row[a] * t[2];
          for (int b = 0; b < 3; b++) n[a][b] += row[a] * row[b];
        }
      }
      double[] coef = solve(n, rhs);
      if (coef != null) {
        def.put("tilt_mm_per_100m", 1e5 * Math.hypot(coef[1], coef[2]));
        double rss = 0;
        for (double[] t : medians) {
          double e = t[2] - (coef[0] + coef[1] * (t[0] - mx) + coef[2] * (t[1] - my));
          rss += e * e;
        }
        def.put("warp_std_m", Math.sqrt(rss / medians.size()));
      }
    }

    // Flat noise: the cloud's own plane RMS in stable cells (all its points there). Not in ground
    // mode: 6 m forest cells are not flat surfaces.
    int cells = ground ? 0 : g1.w * g1.h;
    double[][] mom = new double[cells][];
    for (int i = 0; i < cloud.n && !ground; i++) {
      int k = g1.of(cloud.x[i], cloud.y[i]);
      if (k < 0 || !stable.get(k)) continue;
      double[] a = mom[k];
      if (a == null) {
        a = mom[k] = new double[13];
        a[10] = cloud.x[i];
        a[11] = cloud.y[i];
        a[12] = cloud.z[i];
      }
      double px = cloud.x[i] - a[10], py = cloud.y[i] - a[11], pz = cloud.z[i] - a[12];
      a[0]++;
      a[1] += px;
      a[2] += py;
      a[3] += pz;
      a[4] += px * px;
      a[5] += px * py;
      a[6] += px * pz;
      a[7] += py * py;
      a[8] += py * pz;
      a[9] += pz * pz;
    }
    List<Double> rms = new ArrayList<>();
    for (double[] a : mom) {
      if (a == null || a[0] < 8) continue;
      Plane p = plane(a[10], a[11], a[12], a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8], a[9]);
      rms.add(p.rms());
    }
    Collections.sort(rms);
    Result flat = out.section("flat_noise");
    flat.put("cells", (double) rms.size());
    flat.put("median_rms_m", rms.isEmpty() ? Double.NaN : rms.get(rms.size() / 2));
    flat.put("p90_rms_m", rms.isEmpty() ? Double.NaN : rms.get((int) (.9 * (rms.size() - 1))));

    // Completeness over the scored area (0.5 m reference cells).
    BitSet covered = new BitSet(frame.aw * frame.ah);
    long extra = 0;
    for (int i = 0; i < cloud.n; i++) {
      int c = (int) Math.floor((cloud.x[i] - frame.ax0) / frame.acell), r = (int) Math.floor((cloud.y[i] - frame.ay0) / frame.acell);
      if (c < 0 || r < 0 || c >= frame.aw || r >= frame.ah) {
        extra++;
        continue;
      }
      int k = r * frame.aw + c;
      if (frame.area.get(k)) covered.set(k);
      else extra++;
    }
    Result comp = out.section("completeness");
    int areaCells = frame.area.cardinality();
    comp.put("area_m2", areaCells * frame.acell * (double) frame.acell);
    comp.put("covered_fraction", areaCells == 0 ? Double.NaN : covered.cardinality() / (double) areaCells);
    comp.put("points_outside_area_fraction", extra / (double) cloud.n);
  }

  static void distances(double[] a, Result r) {
    r.put("count", (double) a.length);
    if (a.length == 0) return;
    Arrays.sort(a);
    double ss = 0;
    int capped = 0, far = 0;
    for (double v : a) {
      if (v > 1) far++;
      if (v <= 1) {
        ss += v * v;
        capped++;
      }
    }
    r.put("median_m", a[a.length / 2]);
    r.put("p75_m", a[(int) (.75 * (a.length - 1))]);
    r.put("p90_m", a[(int) (.9 * (a.length - 1))]);
    r.put("p95_m", a[(int) (.95 * (a.length - 1))]);
    r.put("rms_within_1m_m", capped == 0 ? Double.NaN : Math.sqrt(ss / capped));
    r.put("within_5cm", fraction(a, .05));
    r.put("within_10cm", fraction(a, .10));
    r.put("within_25cm", fraction(a, .25));
    r.put("far_over_1m", far / (double) a.length);
  }

  static double fraction(double[] sorted, double limit) {
    int lo = 0, hi = sorted.length;
    while (lo < hi) {
      int mid = (lo + hi) >>> 1;
      if (sorted[mid] <= limit) lo = mid + 1;
      else hi = mid;
    }
    return lo / (double) sorted.length;
  }

  // ---------------------------------------------------------------------------------------------
  // Reference-free metrics

  /**
   * Area (0.5 m cells), double thin sheets (3 m cells: two height clusters of at least 25% each,
   * IQR <= 0.4 m, empty gap >= 1.5 m) and far outliers (20 m cells: points more than 20 m below
   * the cell's 10th or 40 m above its 90th height percentile).
   */
  static void selfMetrics(Cloud c, float[] b, Result out) {
    long aw = (long) ((b[2] - b[0]) / .5f) + 1, ah = (long) ((b[3] - b[1]) / .5f) + 1;
    long occupied;
    if (aw * ah < Integer.MAX_VALUE - 64) {
      BitSet cells = new BitSet((int) (aw * ah));
      for (int i = 0; i < c.n; i++) cells.set((int) ((long) ((c.y[i] - b[1]) / .5f) * aw + (long) ((c.x[i] - b[0]) / .5f)));
      occupied = cells.cardinality();
    } else {
      HashSet<Long> cells = new HashSet<>();
      for (int i = 0; i < c.n; i++) cells.add((long) ((c.y[i] - b[1]) / .5f) * aw + (long) ((c.x[i] - b[0]) / .5f));
      occupied = cells.size();
    }
    out.put("area_m2", occupied * .25);
    out.put("points_per_m2", c.n / Math.max(1e-9, occupied * .25));

    Grid s = new Grid(c, b, 3f);
    long eligible = 0, doubles = 0;
    float[] t = new float[4096];
    for (int k = 0; k < s.w * s.h; k++) {
      int m = s.count(k);
      if (m < 12) continue;
      eligible++;
      if (m > t.length) t = new float[m];
      for (int a = 0; a < m; a++) t[a] = c.z[s.idx[s.start[k] + a]];
      Arrays.sort(t, 0, m);
      int lo = (int) Math.ceil(.25 * m), hi = m - lo, at = -1;
      float best = 0;
      for (int i = lo - 1; i < hi; i++) {
        float gap = t[i + 1] - t[i];
        if (gap > best) {
          best = gap;
          at = i;
        }
      }
      if (at < 0 || best < 1.5f) continue;
      int nl = at + 1, nu = m - nl;
      float iqrL = t[(3 * nl) / 4] - t[nl / 4], iqrU = t[nl + (3 * nu) / 4] - t[nl + nu / 4];
      if (iqrL <= .4f && iqrU <= .4f) doubles++;
    }
    out.put("double_sheet_cells_fraction", eligible == 0 ? Double.NaN : doubles / (double) eligible);
    s = null;

    Grid o = new Grid(c, b, 20f);
    long below = 0, above = 0;
    for (int k = 0; k < o.w * o.h; k++) {
      int m = o.count(k);
      if (m < 16) continue;
      int step = Math.max(1, m / 256), n = 0;
      float[] z = new float[Math.min(m, 257)];
      for (int a = 0; a < m && n < z.length; a += step) z[n++] = c.z[o.idx[o.start[k] + a]];
      Arrays.sort(z, 0, n);
      float p10 = z[(int) (.1 * (n - 1))], p90 = z[(int) (.9 * (n - 1))];
      for (int a = o.start[k]; a < o.start[k + 1]; a++) {
        float v = c.z[o.idx[a]];
        if (v < p10 - 20) below++;
        else if (v > p90 + 40) above++;
      }
    }
    out.put("far_below_fraction", below / (double) c.n);
    out.put("far_above_fraction", above / (double) c.n);
  }
}
