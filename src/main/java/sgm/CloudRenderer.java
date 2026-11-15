package sgm;

import java.lang.invoke.MethodHandles;
import java.lang.invoke.VarHandle;
import java.util.*;
import java.util.stream.IntStream;

/**
 * Software point renderer of the 3D view (windowless, so SelfTest can check it). Every point is
 * projected (in parallel) into a per-pixel 64-bit value: depth bits high, point id low; the
 * nearest point wins by an atomic minimum, so the picture does not depend on thread order and the
 * buffer doubles as the pick index. Colours are resolved per pixel afterwards; eye-dome lighting
 * (shading by the depth of the neighbouring pixels) gives the cloud its relief without normals.
 */
final class CloudRenderer {
  private CloudRenderer() {}

  private static final VarHandle LONGS = MethodHandles.arrayElementVarHandle(long[].class);
  static final long EMPTY = Long.MAX_VALUE;
  /** Point ids: dense index, SPARSE | sparse index, or LIDAR | lidar reference index. */
  static final int SPARSE = 1 << 30, LIDAR = 1 << 29;
  /** Colour of the lidar reference points. */
  static final int LIDAR_COLOUR = 0xff9800;

  enum Colour { RGB, HEIGHT, CLASS, ATTRIBUTE }

  /**
   * An orbit view: target (cloud coordinates), yaw (degrees, clockwise from north), pitch (degrees
   * below the horizon, 90 = straight down), distance, vertical field of view; orthographic for
   * a map-like top view.
   */
  record View(double[] target, double yaw, double pitch, double distance, double fov, boolean ortho) {
    double[] forward() {
      double y = Math.toRadians(yaw), p = Math.toRadians(pitch);
      return new double[] {Math.cos(p) * Math.sin(y), Math.cos(p) * Math.cos(y), -Math.sin(p)};
    }

    double[] right() {
      double y = Math.toRadians(yaw);
      return new double[] {Math.cos(y), -Math.sin(y), 0};
    }

    double[] up() {
      double[] f = forward(), r = right();
      return new double[] {r[1] * f[2] - r[2] * f[1], r[2] * f[0] - r[0] * f[2], r[0] * f[1] - r[1] * f[0]};
    }

    double[] eye() {
      double[] f = forward();
      return new double[] {target[0] - f[0] * distance, target[1] - f[1] * distance, target[2] - f[2] * distance};
    }

    View orbit(double dYaw, double dPitch) {
      return new View(target, ((yaw + dYaw) % 360 + 360) % 360, Math.max(-89, Math.min(90, pitch + dPitch)), distance, fov, ortho);
    }

    View zoom(double factor) {
      return new View(target, yaw, pitch, Math.max(1e-3, distance * factor), fov, ortho);
    }

    /** Moves the target by screen pixels (dx right, dy down) at the target's depth. */
    View pan(double dx, double dy, int height) {
      double perPixel = 2 * distance * Math.tan(Math.toRadians(fov) / 2) / Math.max(1, height);
      double[] r = right(), u = up(), t = target.clone();
      for (int k = 0; k < 3; k++) t[k] += (-dx * r[k] + dy * u[k]) * perPixel;
      return new View(t, yaw, pitch, distance, fov, ortho);
    }

    View at(double[] t) {
      return new View(t.clone(), yaw, pitch, distance, fov, ortho);
    }

    View withOrtho(boolean o) {
      return new View(target, yaw, pitch, distance, fov, o);
    }

    View withAngles(double y, double p) {
      return new View(target, y, p, distance, fov, ortho);
    }
  }

  /** Screen projection of a view on a w x h image: {sx, sy, depth} or null when behind. */
  static final class Projector {
    final double ex, ey, ez, rx, ry, rz, ux, uy, uz, fx, fy, fz, focal, scale, cx, cy;
    final boolean ortho;

    Projector(View v, int w, int h) {
      double[] e = v.eye(), r = v.right(), u = v.up(), f = v.forward();
      ex = e[0]; ey = e[1]; ez = e[2];
      rx = r[0]; ry = r[1]; rz = r[2];
      ux = u[0]; uy = u[1]; uz = u[2];
      fx = f[0]; fy = f[1]; fz = f[2];
      double half = Math.tan(Math.toRadians(v.fov()) / 2);
      focal = h / 2.0 / half;
      scale = h / (2 * v.distance() * half);
      cx = w / 2.0;
      cy = h / 2.0;
      ortho = v.ortho();
    }

    /** Fills out {sx, sy, depth}; false when the point is behind the eye. */
    boolean project(double x, double y, double z, double[] out) {
      double dx = x - ex, dy = y - ey, dz = z - ez;
      double depth = dx * fx + dy * fy + dz * fz;
      if (!(depth > 1e-6)) return false;
      double a = dx * rx + dy * ry + dz * rz, b = dx * ux + dy * uy + dz * uz;
      if (ortho) {
        out[0] = cx + a * scale;
        out[1] = cy - b * scale;
      } else {
        out[0] = cx + focal * a / depth;
        out[1] = cy - focal * b / depth;
      }
      out[2] = depth;
      return true;
    }
  }

  /** What to draw. step > 1 draws every step-th dense point (the fast preview while moving). */
  record Request(CloudData data, View view, int width, int height, Colour colour, int attribute, int pointSize, boolean edl,
                 boolean dense, boolean sparse, int step, float lowValue, float highValue) {}

  /** The picture and its per-pixel nearest-point buffer (for picking). */
  record Frame(int width, int height, int[] argb, long[] buffer, Request request) {
    /** Nearest point id within radius pixels of (px, py), or -1. */
    int pick(int px, int py, int radius) {
      long best = EMPTY;
      for (int y = Math.max(0, py - radius); y <= Math.min(height - 1, py + radius); y++)
        for (int x = Math.max(0, px - radius); x <= Math.min(width - 1, px + radius); x++) best = Math.min(best, buffer[y * width + x]);
      return best == EMPTY ? -1 : (int) (best & 0xffffffffL);
    }
  }

  static Frame render(Request q) {
    int w = q.width(), h = q.height();
    long[] buffer = new long[w * h];
    Arrays.fill(buffer, EMPTY);
    CloudData d = q.data();
    Projector pr = new Projector(q.view(), w, h);
    int size = Math.max(1, q.pointSize());
    if (q.dense() && d.n > 0) {
      int step = Math.max(1, q.step()), chunks = Math.max(1, Math.min(256, d.n / step / 50_000 + 1));
      IntStream.range(0, chunks).parallel().forEach(c -> {
        double[] s = new double[3];
        long count = (d.n + (long) step - 1) / step;
        for (long k = count * c / chunks; k < count * (c + 1) / chunks; k++) {
          int i = (int) (k * step);
          if (pr.project(d.x[i], d.y[i], d.z[i], s)) splat(buffer, w, h, s, size, i);
        }
      });
    }
    if (d.showLidar && d.lx.length > 0) {
      int step = Math.max(1, q.step());
      double[] s = new double[3];
      for (int i = 0; i < d.lx.length; i += step) if (pr.project(d.lx[i], d.ly[i], d.lz[i], s)) splat(buffer, w, h, s, size, LIDAR | i);
    }
    if (q.sparse() && d.sx.length > 0) {
      double[] s = new double[3];
      int sparseSize = Math.max(2, size + 1);
      boolean[] hidden = d.shidden, big = d.sbig;
      for (int i = 0; i < d.sx.length; i++) {
        if (hidden != null && hidden[i]) continue;
        if (pr.project(d.sx[i], d.sy[i], d.sz[i], s)) splat(buffer, w, h, s, big != null && big[i] ? sparseSize + 1 : sparseSize, SPARSE | i);
      }
    }
    int[] argb = new int[w * h];
    float[] attr = q.colour() == Colour.ATTRIBUTE && q.attribute() >= 0 && q.attribute() < d.attributes.size() ? d.attributes.get(q.attribute()).values() : null;
    float lo = q.lowValue(), hi = q.highValue(), range = hi > lo ? hi - lo : 1;
    int sparseMax = 2;
    for (int o : d.sobs) sparseMax = Math.max(sparseMax, Math.min(20, o));
    final int sm = sparseMax;
    IntStream.range(0, h).parallel().forEach(y -> {
      for (int x = 0; x < w; x++) {
        int p = y * w + x;
        long v = buffer[p];
        if (v == EMPTY) {
          argb[p] = background(y, h);
          continue;
        }
        int id = (int) (v & 0xffffffffL);
        int c;
        if ((id & SPARSE) != 0) {
          int i = id & ~SPARSE;
          // Tie points: bright end of the ramp (blue few images, red many), not shaded.
          int[] own = d.scolour;
          c = own != null ? own[i] & 0xffffff : ramp(.3f + .7f * Math.min(1f, (Math.min(20, d.sobs[i]) - 2f) / Math.max(1, sm - 2)));
          argb[p] = 0xff000000 | c;
          continue;
        } else if ((id & LIDAR) != 0) c = LIDAR_COLOUR;
        else
          c = switch (q.colour()) {
            case RGB -> d.rgb[id];
            case HEIGHT -> ramp((d.z[id] - lo) / range);
            case CLASS -> classColour(d.cls[id]);
            case ATTRIBUTE -> attr == null || Float.isNaN(attr[id]) ? 0x808080 : ramp((attr[id] - lo) / range);
          };
        if (q.edl()) c = shade(c, edl(buffer, w, h, x, y, v));
        argb[p] = 0xff000000 | c;
      }
    });
    return new Frame(w, h, argb, buffer, q);
  }

  private static void splat(long[] buffer, int w, int h, double[] s, int size, int id) {
    int x0 = (int) Math.floor(s[0] - (size - 1) / 2.0), y0 = (int) Math.floor(s[1] - (size - 1) / 2.0);
    if (x0 >= w || y0 >= h || x0 + size <= 0 || y0 + size <= 0) return;
    long packed = (long) Float.floatToIntBits((float) s[2]) << 32 | (id & 0xffffffffL);
    for (int y = Math.max(0, y0); y < Math.min(h, y0 + size); y++)
      for (int x = Math.max(0, x0); x < Math.min(w, x0 + size); x++) {
        int p = y * w + x;
        long old = (long) LONGS.getVolatile(buffer, p);
        while (packed < old && !LONGS.compareAndSet(buffer, p, old, packed)) old = (long) LONGS.getVolatile(buffer, p);
      }
  }

  static float depth(long v) {
    return Float.intBitsToFloat((int) (v >>> 32));
  }

  /** Eye-dome lighting: how much nearer the neighbouring points are (log depth), 0 = none. */
  static float edl(long[] buffer, int w, int h, int x, int y, long v) {
    double centre = Math.log(depth(v)) / Math.log(2), sum = 0;
    int[][] around = {{-2, 0}, {2, 0}, {0, -2}, {0, 2}, {-1, -1}, {1, -1}, {-1, 1}, {1, 1}};
    for (int[] o : around) {
      int nx = x + o[0], ny = y + o[1];
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      long n = buffer[ny * w + nx];
      if (n == EMPTY) continue;
      double d = centre - Math.log(depth(n)) / Math.log(2);
      if (d > 0) sum += d;
    }
    return (float) sum;
  }

  static int shade(int rgb, float response) {
    double k = Math.exp(-12 * response);
    int r = (int) ((rgb >> 16 & 255) * k), g = (int) ((rgb >> 8 & 255) * k), b = (int) ((rgb & 255) * k);
    return r << 16 | g << 8 | b;
  }

  static int background(int y, int h) {
    int top = 0x2a, bottom = 0x14, v = top + (bottom - top) * y / Math.max(1, h - 1);
    return 0xff000000 | v << 16 | v << 8 | (v + 6);
  }

  /** Turbo-like colour ramp over 0..1. */
  static int ramp(float t) {
    double x = Math.max(0, Math.min(1, Float.isNaN(t) ? 0 : t));
    double r = 0.13572138 + x * (4.61539260 + x * (-42.66032258 + x * (132.13108234 + x * (-152.94239396 + x * 59.28637943))));
    double g = 0.09140261 + x * (2.19418839 + x * (4.84296658 + x * (-14.18503333 + x * (4.27729857 + x * 2.82956604))));
    double b = 0.10667330 + x * (12.64194608 + x * (-60.58204836 + x * (110.36276771 + x * (-89.90310912 + x * 27.34824973))));
    return clamp(r) << 16 | clamp(g) << 8 | clamp(b);
  }

  private static int clamp(double v) {
    return (int) Math.max(0, Math.min(255, Math.round(v * 255)));
  }

  /** ASPRS classes: ground brown, vegetation greens, building orange, noise magenta, water blue. */
  static int classColour(int c) {
    return switch (c & 255) {
      case 2 -> 0xa0703c;
      case 3 -> 0x9fd36b;
      case 4 -> 0x4caf50;
      case 5 -> 0x1b7a2e;
      case 6 -> 0xf08c2a;
      case 7, 18 -> 0xe040fb;
      case 9 -> 0x3b82f6;
      default -> 0xb8b8b8;
    };
  }

  /** The frustum lines of a camera: apex then four corners at size s (cloud coordinates). */
  static double[][] frustum(CloudData.Cam c, double s) {
    double hw = c.width() / 2.0 / c.fx() * s, hh = c.height() / 2.0 / c.fx() * s;
    double[][] local = {{0, 0, 0}, {-hw, -hh, s}, {hw, -hh, s}, {hw, hh, s}, {-hw, hh, s}};
    double[][] out = new double[5][];
    double[] r = c.r();
    for (int i = 0; i < 5; i++) {
      double[] p = local[i];
      // world = R^T p + C
      out[i] = new double[] {
        r[0] * p[0] + r[3] * p[1] + r[6] * p[2] + c.center()[0],
        r[1] * p[0] + r[4] * p[1] + r[7] * p[2] + c.center()[1],
        r[2] * p[0] + r[5] * p[1] + r[8] * p[2] + c.center()[2]};
    }
    return out;
  }

  /** A view that shows everything: looking down at 55 degrees from the south, all points inside. */
  static View home(CloudData d) {
    // The points' 1%..99% range (stray tie points far outside the block would shrink the scene to
    // a dot), then the cameras near it (a camera thrown far away by a failed alignment too).
    double[] lo = {Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE}, hi = {-Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE};
    boolean any = false;
    float[][] axes = d.n > 0 ? new float[][] {d.x, d.y, d.z} : new float[][] {d.sx, d.sy, d.sz};
    int count = d.n > 0 ? d.n : d.sx.length;
    if (count > 0) {
      any = true;
      for (int k = 0; k < 3; k++) {
        lo[k] = CloudData.quantile(axes[k], count, .01);
        hi[k] = CloudData.quantile(axes[k], count, .99);
      }
    }
    if (!d.cameras.isEmpty()) {
      double[] c0 = any ? new double[] {(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2} : null;
      double reach = any ? 3 * Math.max(1, Math.sqrt(Math.pow(hi[0] - lo[0], 2) + Math.pow(hi[1] - lo[1], 2) + Math.pow(hi[2] - lo[2], 2))) : Double.MAX_VALUE;
      for (CloudData.Cam c : d.cameras) {
        double[] p = c.center();
        if (c0 != null && Math.sqrt(Math.pow(p[0] - c0[0], 2) + Math.pow(p[1] - c0[1], 2) + Math.pow(p[2] - c0[2], 2)) > reach) continue;
        any = true;
        grow(lo, hi, p[0], p[1], p[2]);
      }
    }
    if (!any) return new View(new double[3], 0, 55, 100, 45, false);
    double[] t = {(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2};
    double radius = Math.max(1, Math.sqrt(Math.pow(hi[0] - lo[0], 2) + Math.pow(hi[1] - lo[1], 2) + Math.pow(hi[2] - lo[2], 2)) / 2);
    return new View(t, 0, 55, radius / Math.tan(Math.toRadians(22.5)) * 1.05, 45, false);
  }

  private static void grow(double[] lo, double[] hi, double x, double y, double z) {
    lo[0] = Math.min(lo[0], x); lo[1] = Math.min(lo[1], y); lo[2] = Math.min(lo[2], z);
    hi[0] = Math.max(hi[0], x); hi[1] = Math.max(hi[1], y); hi[2] = Math.max(hi[2], z);
  }

  /** A sensible frustum size: a third of the median distance to the nearest other camera. */
  static double cameraSize(CloudData d) {
    List<CloudData.Cam> c = d.cameras;
    if (c.size() < 2) return 5;
    double[] nearest = new double[c.size()];
    for (int i = 0; i < c.size(); i++) {
      double best = Double.MAX_VALUE;
      for (int j = 0; j < c.size(); j++)
        if (i != j) best = Math.min(best, Math.sqrt(Math.pow(c.get(i).center()[0] - c.get(j).center()[0], 2) + Math.pow(c.get(i).center()[1] - c.get(j).center()[1], 2) + Math.pow(c.get(i).center()[2] - c.get(j).center()[2], 2)));
      nearest[i] = best;
    }
    Arrays.sort(nearest);
    return Math.max(1e-3, nearest[nearest.length / 2] / 3);
  }
}
