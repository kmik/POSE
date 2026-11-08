package sgm;

import java.io.*;
import java.nio.*;
import java.nio.channels.FileChannel;
import java.nio.file.*;
import java.util.*;

/**
 * Second-pass disparity search guided by frozen, independently generated depth anchors. Only the
 * run-owned candidate scratch stream is rewritten; no cost volumes are retained.
 */
final class MultiViewDisparityRefinement {
  interface Rectifier {
    DenseReconstruction.Rectified open(int a, int b, int pair) throws IOException;
  }

  record Outcome(long count, long[] starts, Map<String, Object> report) {}

  record Plane(double a, double b, double c, double sigma, double[] hull) {
    double inverse(double u, double v) {
      return a * u + b * v + c;
    }

    boolean contains(double u, double v) {
      return inside(hull, u, v);
    }
  }

  record Search(double disparity, double photo, double lr, double margin, double sharpness) {}

  record Decision(
      int pair,
      int pixel,
      double[] original,
      double[] point,
      String reason,
      double oldDisparity,
      double priorDisparity) {}

  java.util.function.Consumer<Decision> observer;
  private String lastReason;
  private final Map<String, Long> modelFailures = new TreeMap<>();
  private static final int SLOTS = 2, BYTES_PER_CELL = 40;
  private static final List<String> REASONS =
      List.of(
          "not_a_candidate",
          "reliable_geometry_unchanged",
          "no_independent_surface",
          "anchor_views_disagree_or_occluded",
          "within_surface_precision_unchanged",
          "directional_structure_preserved",
          "refined_independent_surface",
          "unresolved_surface_conflict_rejected",
          "photometric_foreground_preserved",
          "uncertain_wide_baseline_preserved");
  private final Options o;
  private final Camera[] cameras;
  private final int[] widths, heights;
  private final int[][] pairs;
  private final double[][] centers;
  private final Grid[] grids;
  private final double anchorAngle;
  private final int fields, record, spacing;
  private long sampled, projected, seen, changed, rejected;
  private final Map<String, Long> reasons = new TreeMap<>();
  private final List<Object> pairReports = new ArrayList<>();

  private final class Grid {
    final int w, h, step;
    final float[] z, u, v, strength;
    final int[] pair;

    Grid(int width, int height, int step) {
      this.step = step;
      w = Math.ceilDiv(width, step);
      h = Math.ceilDiv(height, step);
      int n = Math.multiplyExact(Math.multiplyExact(w, h), SLOTS);
      z = new float[n];
      u = new float[n];
      v = new float[n];
      strength = new float[n];
      pair = new int[n];
      Arrays.fill(pair, -1);
    }

    void copy(int from, int to) {
      z[to] = z[from];
      u[to] = u[from];
      v[to] = v[from];
      strength[to] = strength[from];
      pair[to] = pair[from];
    }

    void put(double x, double y, double depth, int source, double weight) {
      int cx = (int) x / step, cy = (int) y / step;
      if (cx < 0 || cy < 0 || cx >= w || cy >= h) return;
      int p = (cy * w + cx) * SLOTS;
      double band = .005 * depth;
      int at;
      if (z[p] <= 0) {
        at = p;
      } else if (depth < z[p] - band) {
        copy(p, p + 1);
        at = p;
      } else if (Math.abs(depth - z[p]) <= band && weight > strength[p]) {
        if (disjoint(pair[p], source)) copy(p, p + 1);
        else if (z[p + 1] > 0 && !disjoint(pair[p + 1], source)) {
          z[p + 1] = 0;
          pair[p + 1] = -1;
        }
        at = p;
      } else if (pair[p] != source
          && (z[p + 1] <= 0
              || disjoint(source, pair[p]) && !disjoint(pair[p + 1], pair[p])
              || disjoint(source, pair[p]) == disjoint(pair[p + 1], pair[p])
                  && (depth < z[p + 1] - band
                      || Math.abs(depth - z[p + 1]) <= band && weight > strength[p + 1])))
        at = p + 1;
      else return;
      z[at] = (float) depth;
      u[at] = (float) x;
      v[at] = (float) y;
      strength[at] = (float) weight;
      pair[at] = source;
    }
  }

  MultiViewDisparityRefinement(
      Options o, List<ImageFrame> frames, int[][] pairs, double anchorAngle, long memoryBytes) {
    this.o = o;
    this.pairs = pairs;
    this.anchorAngle = anchorAngle;
    fields = PointConfidence.bidirectional(o) ? 6 : 5;
    record = 35 + fields * 4;
    cameras = frames.stream().map(f -> f.camera).toArray(Camera[]::new);
    widths = frames.stream().mapToInt(f -> f.width).toArray();
    heights = frames.stream().mapToInt(f -> f.height).toArray();
    centers = Arrays.stream(cameras).map(Camera::center).toArray(double[][]::new);
    int step = 8;
    while (bytes(widths, heights, step) > memoryBytes && step < 16384) step *= 2;
    if (bytes(widths, heights, step) > memoryBytes)
      throw new IllegalArgumentException("Insufficient disparity anchor memory");
    spacing = step;
    grids = new Grid[cameras.length];
    for (int c = 0; c < cameras.length; c++)
      if (cameras[c].registered && !cameras[c].denseExcluded)
        grids[c] = new Grid(widths[c], heights[c], step);
  }

  static long bytes(int[] widths, int[] heights, int step) {
    long n = 0;
    for (int c = 0; c < widths.length; c++)
      n =
          Math.addExact(
              n,
              (long) Math.ceilDiv(widths[c], step)
                  * Math.ceilDiv(heights[c], step)
                  * BYTES_PER_CELL);
    return n;
  }

  private boolean independent(int p, int a, int b) {
    int[] q = pairs[p];
    return q[0] != a && q[1] != a && q[0] != b && q[1] != b;
  }

  private boolean disjoint(int p, int q) {
    int[] a = pairs[p], b = pairs[q];
    return a[0] != b[0] && a[0] != b[1] && a[1] != b[0] && a[1] != b[1];
  }

  private void reason(String name) {
    lastReason = name;
    reasons.merge(name, 1L, Long::sum);
  }

  static boolean reliable(double angle, float confidence, float reverse, double minimumAngle) {
    return angle >= minimumAngle
        && Float.isFinite(confidence)
        && Float.isFinite(reverse)
        && Math.min(confidence, reverse) >= .35;
  }

  private Plane failed(String name) {
    modelFailures.merge(name, 1L, Long::sum);
    return null;
  }

  private ByteBuffer buffer() {
    return ByteBuffer.allocateDirect(record * 8192);
  }

  private static void read(FileChannel file, ByteBuffer data, long position, int count)
      throws IOException {
    data.clear();
    data.limit(count);
    while (data.hasRemaining()) {
      int n = file.read(data, position);
      if (n < 0) throw new EOFException("Truncated refinement input");
      if (n == 0) throw new IOException("Cannot read refinement input");
      position += n;
    }
    data.flip();
  }

  /** Sparse deterministic sampling bounds projection work independently of export order. */
  void anchors(Path input, long[] starts) throws IOException {
    try (var file = FileChannel.open(input);
        var progress = WorkProgress.start("Independent disparity anchors", pairs.length, "pairs")) {
      if (file.size() != Math.multiplyExact(starts[starts.length - 1], record))
        throw new IOException("Candidate layout/anchor input mismatch");
      ByteBuffer data = buffer();
      for (int p = 0; p < pairs.length; p++) {
        progress.update(p);
        int a = pairs[p][0], b = pairs[p][1];
        for (long at = starts[p]; at < starts[p + 1]; ) {
          int count = (int) Math.min(8192, starts[p + 1] - at);
          read(file, data, at * record, count * record);
          for (int i = 0; i < count; i++) {
            int pos = data.position();
            double[] point = {data.getDouble(), data.getDouble(), data.getDouble()};
            data.position(pos + 27);
            int source = data.getInt(), pixel = data.getInt();
            float confidence = data.getFloat();
            float reverse = fields == 6 ? data.getFloat(pos + 35 + 5 * 4) : confidence;
            data.position(pos + record);
            if (source != p) throw new IOException("Candidate pair order mismatch");
            if ((Integer.rotateLeft(pixel * 0x9e3779b9, 13) & 31) != 0
                || !Float.isFinite(confidence)
                || !Float.isFinite(reverse)
                || Math.min(confidence, reverse) < .35) continue;
            double angle = DenseReconstruction.triangulationAngle(point, centers[a], centers[b]);
            if (!reliable(angle, confidence, reverse, anchorAngle)) continue;
            sampled++;
            double weight = Math.min(confidence, reverse) * Math.sin(Math.toRadians(angle));
            for (int c = 0; c < cameras.length; c++)
              if (grids[c] != null) {
                Camera cam = cameras[c];
                double[] view = cam.view(point);
                if (view[2] <= 0) continue;
                double u = cam.fx * view[0] / view[2] + cam.cx,
                    v = cam.fy * view[1] / view[2] + cam.cy;
                if (u < 0 || v < 0 || u >= widths[c] || v >= heights[c]) continue;
                grids[c].put(u, v, view[2], p, weight);
                projected++;
              }
          }
          at += count;
        }
      }
      progress.done();
    }
  }

  /** A cache belongs to one target camera and producing pair: excluded views cannot leak in. */
  final class Models {
    final int camera, a, b;
    final LinkedHashMap<Integer, Plane> cache =
        new LinkedHashMap<>(4096, .75f, true) {
          protected boolean removeEldestEntry(Map.Entry<Integer, Plane> e) {
            return size() > 4096;
          }
        };

    Models(int camera, int a, int b) {
      this.camera = camera;
      this.a = a;
      this.b = b;
    }

    Plane at(double u, double v) {
      Grid g = grids[camera];
      if (g == null || u < 0 || v < 0 || u >= widths[camera] || v >= heights[camera]) return null;
      int x = (int) u / g.step, y = (int) v / g.step, key = y * g.w + x;
      if (!cache.containsKey(key)) cache.put(key, fit(g, x, y, camera, a, b));
      Plane plane = cache.get(key);
      return plane != null && plane.contains(u, v) && plane.inverse(u, v) > 0 ? plane : null;
    }
  }

  private Plane fit(Grid g, int cx, int cy, int camera, int a, int b) {
    double[] x = new double[98], y = new double[98], q = new double[98];
    int[] provenance = new int[98];
    int n = 0;
    for (int dy = -3; dy <= 3; dy++)
      for (int dx = -3; dx <= 3; dx++) {
        int xx = cx + dx, yy = cy + dy;
        if (xx < 0 || yy < 0 || xx >= g.w || yy >= g.h) continue;
        for (int k = 0; k < SLOTS; k++) {
          int p = (yy * g.w + xx) * SLOTS + k;
          if (g.z[p] <= 0 || !independent(g.pair[p], a, b)) continue;
          x[n] = g.u[p];
          y[n] = g.v[p];
          q[n] = 1.0 / g.z[p];
          provenance[n++] = g.pair[p];
        }
      }
    if (n < 12) return failed("sparse_independent_anchors");
    double[] sorted = Arrays.copyOf(q, n);
    Arrays.sort(sorted);
    double mid = sorted[n / 2], tol = 3 * mid / Math.min(cameras[camera].fx, cameras[camera].fy);
    int best = 0;
    double ba = 0, bb = 0, bc = 0;
    for (int t = 0; t < 64; t++) {
      int i = t * 17 % n, j = (t * 31 + n / 3) % n, k = (t * 43 + 2 * n / 3) % n;
      double ux = x[j] - x[i],
          uy = y[j] - y[i],
          vx = x[k] - x[i],
          vy = y[k] - y[i],
          det = ux * vy - uy * vx;
      if (Math.abs(det) < g.step * g.step) continue;
      double aa = ((q[j] - q[i]) * vy - uy * (q[k] - q[i])) / det,
          ab = (ux * (q[k] - q[i]) - (q[j] - q[i]) * vx) / det,
          ac = q[i] - aa * x[i] - ab * y[i];
      int support = 0;
      for (int z = 0; z < n; z++) if (Math.abs(q[z] - aa * x[z] - ab * y[z] - ac) <= tol) support++;
      if (support > best && support >= 12) {
        var supportPoints = new ArrayList<double[]>();
        for (int z = 0; z < n; z++)
          if (Math.abs(q[z] - aa * x[z] - ab * y[z] - ac) <= tol)
            supportPoints.add(new double[] {x[z], y[z]});
        double[] boundary = hull(supportPoints);
        if (boundary.length >= 6 && inside(boundary, (cx + .5) * g.step, (cy + .5) * g.step)) {
          best = support;
          ba = aa;
          bb = ab;
          bc = ac;
        }
      }
    }
    if (best < Math.max(12, Math.ceil(n * .25))) return failed("no_dominant_anchor_surface");
    double mx = 0, my = 0, mq = 0;
    int count = 0;
    Set<Integer> sources = new HashSet<>();
    for (int i = 0; i < n; i++)
      if (Math.abs(q[i] - ba * x[i] - bb * y[i] - bc) <= tol) {
        mx += x[i];
        my += y[i];
        mq += q[i];
        count++;
        sources.add(provenance[i]);
      }
    boolean separate = false;
    for (int p : sources) for (int r : sources) if (disjoint(p, r)) separate = true;
    if (!separate) return failed("correlated_anchor_sources");
    mx /= count;
    my /= count;
    mq /= count;
    double xx = 0, xy = 0, yy = 0, xq = 0, yq = 0;
    for (int i = 0; i < n; i++)
      if (Math.abs(q[i] - ba * x[i] - bb * y[i] - bc) <= tol) {
        double u = x[i] - mx, v = y[i] - my, d = q[i] - mq;
        xx += u * u;
        xy += u * v;
        yy += v * v;
        xq += u * d;
        yq += v * d;
      }
    double det = xx * yy - xy * xy;
    if (det < g.step * g.step * g.step * g.step) return failed("ill_conditioned_anchor_surface");
    double aa = (xq * yy - yq * xy) / det,
        ab = (yq * xx - xq * xy) / det,
        ac = mq - aa * mx - ab * my;
    List<double[]> points = new ArrayList<>();
    Map<Integer, Integer> supportByPair = new HashMap<>();
    double[] residual = new double[n];
    int m = 0;
    for (int i = 0; i < n; i++) {
      double error = Math.abs(q[i] - aa * x[i] - ab * y[i] - ac);
      if (error <= tol) {
        points.add(new double[] {x[i], y[i]});
        residual[m++] = error;
        supportByPair.merge(provenance[i], 1, Integer::sum);
      }
    }
    separate = false;
    for (int p : supportByPair.keySet())
      for (int r : supportByPair.keySet())
        if (supportByPair.get(p) >= 3 && supportByPair.get(r) >= 3 && disjoint(p, r))
          separate = true;
    if (!separate) return failed("correlated_anchor_refit");
    if (points.size() < Math.max(12, Math.ceil(n * .25))) return failed("unstable_anchor_refit");
    Arrays.sort(residual, 0, m);
    double sigma =
        Math.max(
            mid / Math.min(cameras[camera].fx, cameras[camera].fy),
            1.4826 * residual[m / 2] / Math.sqrt(Math.min(sources.size(), 4)));
    double[] hull = hull(points);
    return hull.length >= 6 ? new Plane(aa, ab, ac, sigma, hull) : null;
  }

  static double[] disparityPlane(Plane plane, Camera camera, DenseReconstruction.Rectified r) {
    double[] n =
        Math3.mv(
            r.r1,
            new double[] {
              plane.a * camera.fx,
              plane.b * camera.fy,
              plane.a * camera.cx + plane.b * camera.cy + plane.c
            });
    double ax = n[0] / r.q[14],
        ay = n[1] / r.q[14],
        c = (n[0] * r.q[3] + n[1] * r.q[7] + n[2] * r.q[11] - r.q[15]) / r.q[14];
    return r.vertical ? new double[] {ay, ax, c} : new double[] {ax, ay, c};
  }

  static double[] world(DenseReconstruction.Rectified r, int x, int y, double d) {
    double xx = r.vertical ? y : x, yy = r.vertical ? x : y;
    double[] q = r.q;
    double divisor = q[12] * xx + q[13] * yy + q[14] * d + q[15];
    if (!Double.isFinite(divisor) || Math.abs(divisor) < 1e-12) return null;
    double[] view = {
      (q[0] * xx + q[1] * yy + q[2] * d + q[3]) / divisor,
      (q[4] * xx + q[5] * yy + q[6] * d + q[7]) / divisor,
      (q[8] * xx + q[9] * yy + q[10] * d + q[11]) / divisor
    };
    view = Math3.mv(Math3.transpose(r.r1), view);
    return Math3.mv(Math3.transpose(r.a.camera.rotation), Math3.sub(view, r.a.camera.translation));
  }

  private static double disparity(DenseReconstruction.Rectified r, double[] p) {
    double z = Math3.mv(r.r1, r.a.camera.view(p))[2];
    return (r.q[11] / z - r.q[15]) / r.q[14];
  }

  private static boolean agrees(Plane p, Camera c, double[] world) {
    double[] v = c.view(world);
    if (v[2] <= 0) return false;
    double u = c.fx * v[0] / v[2] + c.cx, w = c.fy * v[1] / v[2] + c.cy, q = p.inverse(u, w);
    return p.contains(u, w)
        && q > 0
        && Math.abs(1 / v[2] - q) <= Math.max(3 * p.sigma, 3 * q / Math.min(c.fx, c.fy));
  }

  private static boolean nearer(Plane p, Camera c, double[] world) {
    double[] v = c.view(world), uv = c.projectView(v);
    if (uv == null) return false;
    double q = p.inverse(uv[0], uv[1]);
    return q > 0 && 1 / v[2] > q + Math.max(3 * p.sigma, 3 * q / Math.min(c.fx, c.fy));
  }

  /** Strong evidence for a nearer occluder must not be replaced by a background prior. */
  static boolean foregroundPreferred(
      byte[] left,
      byte[] right,
      int w,
      int h,
      int x,
      int y,
      double original,
      double predicted,
      Options o) {
    int radius = o.correlationWindow / 2;
    double before = photo(left, right, w, h, x, x - original, y, radius);
    if (before < Math.max(.75, o.correlation)) return false;
    double proposed = -1;
    for (int k = -4; k <= 4; k++)
      proposed =
          Math.max(proposed, photo(left, right, w, h, x, x - predicted - k * .125, y, radius));
    return before > proposed + .02;
  }

  /**
   * Reads a frozen disparity layer. Directional structures are never erased by a background prior.
   */
  static boolean thin(float[] raw, int w, int h, int x, int y, double[] plane) {
    int r = 3;
    if (x < r || y < r || x >= w - r || y >= h - r) return true;
    int p = y * w + x;
    double residual = raw[p] - plane[0] * x - plane[1] * y - plane[2];
    boolean[] seen = new boolean[49];
    int[] queue = new int[49];
    int head = 0, tail = 1;
    queue[0] = 24;
    seen[24] = true;
    while (head < tail) {
      int t = queue[head++], tx = t % 7, ty = t / 7;
      for (int dy = -1; dy <= 1; dy++)
        for (int dx = -1; dx <= 1; dx++) {
          int xx = tx + dx, yy = ty + dy;
          if (xx < 0 || yy < 0 || xx >= 7 || yy >= 7) continue;
          int k = yy * 7 + xx;
          if (seen[k]) continue;
          double d = raw[(y + yy - r) * w + x + xx - r];
          if (Double.isFinite(d)
              && Math.abs(
                      d - plane[0] * (x + xx - r) - plane[1] * (y + yy - r) - plane[2] - residual)
                  <= .5) {
            seen[k] = true;
            queue[tail++] = k;
          }
        }
    }
    if (tail < 4) return false;
    double xx = 0, xy = 0, yy = 0;
    boolean reaches = false;
    for (int i = 1; i < tail; i++) {
      int dx = queue[i] % 7 - r, dy = queue[i] / 7 - r;
      xx += dx * dx;
      xy += dx * dy;
      yy += dy * dy;
      if (Math.max(Math.abs(dx), Math.abs(dy)) == r) reaches = true;
    }
    double large = (xx + yy + Math.hypot(xx - yy, 2 * xy)) * .5;
    return reaches && large > 0 && xx + yy - large < large * .12;
  }

  private static boolean patch(byte[] mask, int w, int h, double x, double y, int r) {
    int xx = (int) Math.floor(x), yy = (int) Math.floor(y);
    if (xx - r < 0 || yy - r < 0 || xx + r + 1 >= w || yy + r + 1 >= h) return false;
    for (int dy = -r; dy <= r + 1; dy++)
      for (int dx = -r; dx <= r + 1; dx++) if (mask[(yy + dy) * w + xx + dx] == 0) return false;
    return true;
  }

  static double photo(byte[] a, byte[] b, int w, int h, double ax, double bx, int y, int radius) {
    int ia = (int) Math.floor(ax), ib = (int) Math.floor(bx);
    double fa = ax - ia, fb = bx - ib;
    if (ia - radius < 0
        || ib - radius < 0
        || ia + radius + 1 >= w
        || ib + radius + 1 >= w
        || y - radius < 0
        || y + radius >= h) return -1;
    double sa = 0, sb = 0, saa = 0, sbb = 0, sab = 0;
    int n = (radius * 2 + 1) * (radius * 2 + 1);
    for (int dy = -radius; dy <= radius; dy++)
      for (int dx = -radius; dx <= radius; dx++) {
        int p = (y + dy) * w;
        double av = (1 - fa) * (a[p + ia + dx] & 255) + fa * (a[p + ia + dx + 1] & 255),
            bv = (1 - fb) * (b[p + ib + dx] & 255) + fb * (b[p + ib + dx + 1] & 255);
        sa += av;
        sb += bv;
        saa += av * av;
        sbb += bv * bv;
        sab += av * bv;
      }
    double va = saa - sa * sa / n, vb = sbb - sb * sb / n;
    return va < n * 4 || vb < n * 4
        ? -1
        : (sab - sa * sb / n) / Math.sqrt(Math.max(1e-12, va * vb));
  }

  static Search search(
      byte[] left,
      byte[] right,
      byte[] lm,
      byte[] rm,
      int w,
      int h,
      int x,
      int y,
      double predicted,
      double original,
      Options o) {
    int radius = o.correlationWindow / 2;
    if (!patch(lm, w, h, x, y, radius)) return null;
    double best = -1, chosen = Double.NaN;
    int slot = -1;
    double[] scores = new double[9];
    for (int k = 0; k < 9; k++) {
      double d = predicted + (k - 4) * .125;
      scores[k] =
          (!o.disparitiesExplicit || d > o.minDisparity && d < o.minDisparity + o.disparities - 1)
                  && patch(rm, w, h, x - d, y, radius)
              ? photo(left, right, w, h, x, x - d, y, radius)
              : -1;
      if (scores[k] > best) {
        best = scores[k];
        chosen = d;
        slot = k;
      }
    }
    if (slot <= 0 || slot >= 8 || best < o.correlation) return null;
    double curvature = 2 * best - scores[slot - 1] - scores[slot + 1];
    if (curvature > 1e-8) {
      double
          delta =
              Math.clamp(.0625 * (scores[slot + 1] - scores[slot - 1]) / curvature, -.0625, .0625),
          candidate = chosen + delta;
      double score = photo(left, right, w, h, x, x - candidate, y, radius);
      if (score > best
          && (!o.disparitiesExplicit
              || candidate > o.minDisparity && candidate < o.minDisparity + o.disparities - 1)) {
        best = score;
        chosen = candidate;
      }
    }
    double before = photo(left, right, w, h, x, x - original, y, radius);
    if (best < before - .01) return null;
    double reverse = -1, offset = Double.NaN;
    for (int k = -6; k <= 6; k++) {
      double dx = k * .25, score = photo(left, right, w, h, x + dx, x - chosen, y, radius);
      if (score > reverse) {
        reverse = score;
        offset = dx;
      }
    }
    double minus = photo(left, right, w, h, x, x - chosen - 1, y, radius),
        plus = photo(left, right, w, h, x, x - chosen + 1, y, radius);
    double
        margin =
            PointConfidence.clamp(
                (best - Math.max(minus, plus)) / Math.max(1e-12, 1 - Math.max(minus, plus))),
        sharpness =
            PointConfidence.clamp((2 * best - minus - plus) / Math.max(1e-12, 2 - minus - plus));
    return Math.abs(offset) <= Math.min(.5, o.lrThreshold)
            && reverse >= o.correlation
            && margin > 0
            && sharpness > 0
        ? new Search(chosen, best, Math.abs(offset), margin, sharpness)
        : null;
  }

  Outcome refine(Path input, long[] starts, Rectifier rectifier, Set<Integer> selected)
      throws IOException {
    long[] next = new long[starts.length];
    long write = 0;
    ByteBuffer data = buffer(), output = buffer();
    try (var file = FileChannel.open(input, StandardOpenOption.READ, StandardOpenOption.WRITE);
        var progress =
            WorkProgress.start("Multi-view disparity refinement", pairs.length, "pairs")) {
      for (int source = 0; source < pairs.length; source++) {
        progress.update(source);
        next[source] = write / record;
        long pairSeen = seen, pairChanged = changed, pairRejected = rejected;
        Map<String, Long> before = new TreeMap<>(reasons);
        int a = pairs[source][0], b = pairs[source][1];
        if (starts[source] == starts[source + 1]) continue;
        if (selected != null && !selected.contains(source)) {
          for (long at = starts[source]; at < starts[source + 1]; ) {
            int n = (int) Math.min(8192, starts[source + 1] - at);
            read(file, data, at * record, n * record);
            while (data.hasRemaining()) {
              int written = file.write(data, write);
              if (written <= 0) throw new IOException("Cannot copy unselected refinement pair");
              write += written;
            }
            at += n;
          }
          continue;
        }
        try (var r = rectifier.open(a, b, source)) {
          // Positive-disparity rectification can swap the stored pair endpoints.
          // Matching pixels and R1 belong to the actual rectified reference camera.
          a = r.a.id;
          b = r.b.id;
          int w = r.leftGray.cols(), h = r.leftGray.rows();
          byte[] left = new byte[w * h],
              right = new byte[w * h],
              lm = new byte[w * h],
              rm = new byte[w * h];
          r.leftGray.get(0, 0, left);
          r.rightGray.get(0, 0, right);
          r.leftMask.get(0, 0, lm);
          r.rightMask.get(0, 0, rm);
          float[] raw = new float[w * h];
          Arrays.fill(raw, Float.NaN);
          for (long at = starts[source]; at < starts[source + 1]; ) {
            int n = (int) Math.min(8192, starts[source + 1] - at);
            read(file, data, at * record, n * record);
            for (int i = 0; i < n; i++) {
              int pos = i * record, pixel = data.getInt(pos + 31);
              if (pixel < 0 || pixel >= raw.length)
                throw new IOException("Invalid refinement matching pixel");
              raw[pixel] =
                  (float)
                      disparity(
                          r,
                          new double[] {
                            data.getDouble(pos), data.getDouble(pos + 8), data.getDouble(pos + 16)
                          });
            }
            at += n;
          }
          float[] finalMap = o.diagnostics ? raw.clone() : null,
              reasonMap = o.diagnostics ? new float[w * h] : null;
          Models ma = new Models(a, a, b), mb = new Models(b, a, b);
          for (long at = starts[source]; at < starts[source + 1]; ) {
            int n = (int) Math.min(8192, starts[source + 1] - at);
            read(file, data, at * record, n * record);
            output.clear();
            for (int i = 0; i < n; i++) {
              int pos = i * record, pixel = data.getInt(pos + 31), x = pixel % w, y = pixel / w;
              double[]
                  point = {data.getDouble(pos), data.getDouble(pos + 8), data.getDouble(pos + 16)},
                  original = point;
              seen++;
              boolean keep = true;
              Search found = null;
              double priorD = Double.NaN;
              if (selected != null && !selected.contains(source)) {
                reason("unselected_pair");
              } else if (reliable(
                  DenseReconstruction.triangulationAngle(point, centers[a], centers[b]),
                  data.getFloat(pos + 35),
                  fields == 6 ? data.getFloat(pos + 55) : data.getFloat(pos + 35),
                  anchorAngle)) {
                reason("reliable_geometry_unchanged");
              } else if (DenseReconstruction.triangulationAngle(point, centers[a], centers[b])
                  >= anchorAngle) {
                reason("uncertain_wide_baseline_preserved");
              } else {
                double[] uv = cameras[a].project(point);
                Plane pa = uv == null ? null : ma.at(uv[0], uv[1]);
                if (pa == null) {
                  reason("no_independent_surface");
                } else {
                  double[] dp = disparityPlane(pa, cameras[a], r);
                  double predicted = dp[0] * x + dp[1] * y + dp[2];
                  priorD = predicted;
                  double[] target = world(r, x, y, predicted),
                      bv = target == null ? null : cameras[b].project(target);
                  Plane pb = bv == null ? null : mb.at(bv[0], bv[1]);
                  if (pb == null || !agrees(pb, cameras[b], target)) {
                    reason("anchor_views_disagree_or_occluded");
                  } else if (agrees(pa, cameras[a], point) && agrees(pb, cameras[b], point)) {
                    reason("within_surface_precision_unchanged");
                  } else if (thin(raw, w, h, x, y, dp)) {
                    reason("directional_structure_preserved");
                  } else if (nearer(pa, cameras[a], point)
                      && nearer(pb, cameras[b], point)
                      && foregroundPreferred(left, right, w, h, x, y, raw[pixel], predicted, o)) {
                    reason("photometric_foreground_preserved");
                  } else {
                    found = search(left, right, lm, rm, w, h, x, y, predicted, raw[pixel], o);
                    double[] refined = found == null ? null : world(r, x, y, found.disparity);
                    if (refined != null
                        && agrees(pa, cameras[a], refined)
                        && agrees(pb, cameras[b], refined)
                        && DenseReconstruction.triangulationAngle(refined, centers[a], centers[b])
                            >= o.minAngle) {
                      point = refined;
                      changed++;
                      reason("refined_independent_surface");
                    } else {
                      keep = false;
                      rejected++;
                      reason("unresolved_surface_conflict_rejected");
                    }
                  }
                }
              }
              if (observer != null)
                observer.accept(
                    new Decision(
                        source,
                        pixel,
                        original,
                        keep ? point : null,
                        lastReason,
                        raw[pixel],
                        priorD));
              if (finalMap != null) {
                finalMap[pixel] =
                    !keep ? Float.NaN : found == null ? raw[pixel] : (float) found.disparity;
                reasonMap[pixel] = REASONS.indexOf(lastReason);
              }
              if (keep) {
                int dst = output.position();
                for (int k = 0; k < record; k++) output.put(data.get(pos + k));
                if (found != null) {
                  for (int k = 0; k < 3; k++) output.putDouble(dst + k * 8, point[k]);
                  output.putFloat(dst + 35 + 3 * 4, (float) found.photo);
                  output.putFloat(dst + 35 + 4 * 4, (float) found.lr);
                  // A new local photometric score cannot increase the original SGM confidence.
                  float old = data.getFloat(pos + 35);
                  double adjusted =
                      PointConfidence.score(found.photo, found.margin, found.sharpness, found.lr);
                  output.putFloat(dst + 35, (float) Math.min(old, adjusted));
                  output.putFloat(dst + 39, (float) found.margin);
                  output.putFloat(dst + 43, (float) found.sharpness);
                  if (fields == 6)
                    output.putFloat(dst + 55, (float) Math.min(data.getFloat(pos + 55), adjusted));
                }
              }
            }
            output.flip();
            while (output.hasRemaining()) {
              int written = file.write(output, write);
              if (written <= 0) throw new IOException("Cannot compact refinement candidates");
              write += written;
            }
            at += n;
          }
          if (finalMap != null) {
            Path directory =
                o.output.resolve(String.format(Locale.ROOT, "05_stereo/pair_%04d_%04d", a, b));
            Files.createDirectories(directory);
            raster(directory.resolve("refined_candidate_disparity.pfm"), finalMap, w, h);
            raster(directory.resolve("disparity_refinement_reason.pfm"), reasonMap, w, h);
          }
        }
        var decisions = new TreeMap<String, Long>();
        for (var entry : reasons.entrySet()) {
          long difference = entry.getValue() - before.getOrDefault(entry.getKey(), 0L);
          if (difference > 0) decisions.put(entry.getKey(), difference);
        }
        var pairReport =
            Json.object(
                "pair",
                source,
                "reference",
                a,
                "secondary",
                b,
                "examined",
                seen - pairSeen,
                "refined",
                changed - pairChanged,
                "rejected",
                rejected - pairRejected,
                "decisions",
                decisions,
                "reason_codes",
                REASONS);
        pairReports.add(pairReport);
        Path directory =
            o.output.resolve(String.format(Locale.ROOT, "05_stereo/pair_%04d_%04d", a, b));
        Files.createDirectories(directory);
        Json.write(directory.resolve("disparity_refinement.json"), pairReport);
      }
      next[next.length - 1] = write / record;
      file.truncate(write);
      progress.done();
    }
    Map<String, Object> report =
        Json.object(
            "mode",
            "multiview",
            "version",
            1,
            "anchor_minimum_angle_degrees",
            anchorAngle,
            "anchor_minimum_matching_confidence",
            .35,
            "anchor_bidirectional_confidence_available",
            fields == 6,
            "anchor_grid_spacing_pixels",
            spacing,
            "anchor_grid_bytes",
            bytes(widths, heights, spacing),
            "sampled_anchor_points",
            sampled,
            "projected_anchor_samples",
            projected,
            "examined",
            seen,
            "refined",
            changed,
            "rejected",
            rejected,
            "retained",
            write / record,
            "decisions",
            reasons,
            "anchor_model_failures",
            modelFailures,
            "pairs",
            pairReports,
            "independence",
            "Two disjoint anchor pairs; all four anchor cameras exclude both producing cameras",
            "reciprocity",
            "Local subpixel photometric reverse search; initial SGM reciprocity already applied",
            "reliable_disparities_moved",
            false,
            "cost_volumes_retained",
            false);
    return new Outcome(write / record, next, report);
  }

  private static void raster(Path file, float[] values, int width, int height) throws IOException {
    try (var out = new DataOutputStream(new BufferedOutputStream(Files.newOutputStream(file)))) {
      out.writeBytes("Pf\n" + width + " " + height + "\n1.0\n");
      for (int y = height - 1; y >= 0; y--)
        for (int x = 0; x < width; x++) out.writeFloat(values[y * width + x]);
    }
  }

  private static double cross(double[] a, double[] b, double[] c) {
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
  }

  private static double[] hull(List<double[]> points) {
    points.sort(Comparator.<double[]>comparingDouble(p -> p[0]).thenComparingDouble(p -> p[1]));
    double[][] h = new double[points.size() * 2][];
    int n = 0;
    for (var p : points) {
      while (n >= 2 && cross(h[n - 2], h[n - 1], p) <= 0) n--;
      h[n++] = p;
    }
    int k = n + 1;
    for (int i = points.size() - 2; i >= 0; i--) {
      var p = points.get(i);
      while (n >= k && cross(h[n - 2], h[n - 1], p) <= 0) n--;
      h[n++] = p;
    }
    if (n > 1) n--;
    double[] out = new double[n * 2];
    for (int i = 0; i < n; i++) {
      out[2 * i] = h[i][0];
      out[2 * i + 1] = h[i][1];
    }
    return out;
  }

  private static boolean inside(double[] h, double x, double y) {
    for (int i = 0; i < h.length; i += 2) {
      int j = (i + 2) % h.length;
      if ((h[j] - h[i]) * (y - h[i + 1]) - (h[j + 1] - h[i + 1]) * (x - h[i]) < -1e-5) return false;
    }
    return true;
  }
}
