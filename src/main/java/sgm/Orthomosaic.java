package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;

/**
 * Full-resolution true orthomosaic (--orthomosaic; ORTHOMOSAIC.md): the original photographs are
 * projected onto a surface model of the final point cloud and written as a tiled, Deflate
 * compressed RGBA GeoTIFF (BigTIFF above 4 GiB).
 *
 * <ul>
 *   <li>Surface: the highest cloud point per cell of a north-up grid (cell from the point
 *       spacing), gaps filled from their neighbours. Larger gaps stay unknown and become
 *       transparent pixels.
 *   <li>Each pixel takes its colour from the camera that sees its surface point closest to the
 *       image centre, among the cameras whose ray to that point is not blocked by the surface
 *       (so roofs and crowns stay at their true position and never cover the ground beside them;
 *       ground hidden behind anything solid is transparent).
 *   <li>Canopy is rough and porous, and that test is too tight for it: cells that are rough on
 *       their upper side are SOFT, and a pixel that no camera passes the test for is painted by
 *       the least obstructed camera that has only SOFT cells in its way. A roof is never SOFT.
 *   <li>The mosaic is produced in blocks of 2,048 pixels: the heap holds the surface grid, one
 *       block and a bounded cache of decoded photographs; large TIFF sources are read by window.
 * </ul>
 *
 * No seam blending and no radiometric balancing. The result depends only on the inputs, not on
 * the thread count.
 */
final class Orthomosaic {
  private Orthomosaic() {}

  static final int TILE = 512, BLOCK = 2048;

  /** A source of cloud points (PlyStream.scan). */
  interface Cloud {
    void scan(String label, PlyStream.Consumer consumer) throws IOException;
  }

  /** RGB pixels (0xRRGGBB, row-major) of the window x, y, w, h of a view's photograph. */
  interface Photos {
    int[] window(int view, int x, int y, int w, int h) throws IOException;
  }

  /** Receives the compressed tiles (TiledGeoTiff.write). */
  interface TileSink {
    void write(int tx, int ty, byte[] compressed) throws IOException;
  }

  /** A calibrated, oriented photograph. Pose in reconstruction units; world = scale * those. */
  static final class View {
    final String name;
    final double[] rotation, translation, distortion, centre;
    final double fx, fy, cx, cy, scale;
    final int width, height;
    // Ground rectangle (world x0, y0, x1, y1) outside which the view sees nothing.
    double[] reach;

    View(String name, Camera pose, Calibration calibration, double scale) {
      this.name = name;
      rotation = pose.rotation.clone();
      translation = pose.translation.clone();
      distortion = Arrays.copyOf(calibration.distortion, 5);
      fx = calibration.fx;
      fy = calibration.fy;
      cx = calibration.cx;
      cy = calibration.cy;
      width = calibration.width;
      height = calibration.height;
      this.scale = scale;
      centre = pose.center();
      for (int k = 0; k < 3; k++) centre[k] *= scale;
    }

    /** Source pixel of a world point into uv; false when behind the camera or far outside. */
    boolean project(double x, double y, double z, double[] uv) {
      double px = x / scale, py = y / scale, pz = z / scale;
      double q2 = rotation[6] * px + rotation[7] * py + rotation[8] * pz + translation[2];
      if (!(q2 > 1e-9)) return false;
      double xn = (rotation[0] * px + rotation[1] * py + rotation[2] * pz + translation[0]) / q2;
      double yn = (rotation[3] * px + rotation[4] * py + rotation[5] * pz + translation[1]) / q2;
      // The distortion polynomial is only valid near the image: reject rays far outside it.
      double pu = fx * xn + cx, pv = fy * yn + cy;
      if (pu < -.25 * width || pv < -.25 * height || pu > 1.25 * width || pv > 1.25 * height) return false;
      double r = xn * xn + yn * yn;
      double radial = 1 + distortion[0] * r + distortion[1] * r * r + distortion[4] * r * r * r;
      uv[0] = fx * (xn * radial + 2 * distortion[2] * xn * yn + distortion[3] * (r + 2 * xn * xn)) + cx;
      uv[1] = fy * (yn * radial + distortion[2] * (r + 2 * yn * yn) + 2 * distortion[3] * xn * yn) + cy;
      return true;
    }

    /** World ray direction through source pixel (u, v), undistorted iteratively. */
    double[] ray(double u, double v) {
      double xd = (u - cx) / fx, yd = (v - cy) / fy, x = xd, y = yd;
      double[] d = distortion;
      for (int j = 0; j < 20; j++) {
        double r = x * x + y * y, a = 1 + d[0] * r + d[1] * r * r + d[4] * r * r * r;
        x = (xd - 2 * d[2] * x * y - d[3] * (r + 2 * x * x)) / a;
        y = (yd - d[2] * (r + 2 * y * y) - 2 * d[3] * x * y) / a;
      }
      return new double[] {
        rotation[0] * x + rotation[3] * y + rotation[6],
        rotation[1] * x + rotation[4] * y + rotation[7],
        rotation[2] * x + rotation[5] * y + rotation[8]
      };
    }

    /**
     * The ground rectangle the image can reach between heights low and high: border rays
     * intersected with both planes; a ray that does not descend reaches everywhere (limit).
     */
    void reach(double low, double high, double[] limit) {
      double[] box = {Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY};
      boolean open = false;
      for (int i = 0; i <= 8 && !open; i++)
        for (int side = 0; side < 4 && !open; side++) {
          double u = side < 2 ? i / 8. * (width - 1) : side == 2 ? 0 : width - 1;
          double v = side < 2 ? (side == 0 ? 0 : height - 1) : i / 8. * (height - 1);
          double[] ray = ray(u, v);
          for (double plane : new double[] {low, high}) {
            double t = (plane - centre[2]) / ray[2];
            if (!(t > 0) || !Double.isFinite(t) || Math.abs(ray[2]) < .05 * Math.hypot(ray[0], ray[1])) {
              open = true;
              break;
            }
            double gx = centre[0] + t * ray[0], gy = centre[1] + t * ray[1];
            box[0] = Math.min(box[0], gx);
            box[1] = Math.min(box[1], gy);
            box[2] = Math.max(box[2], gx);
            box[3] = Math.max(box[3], gy);
          }
        }
      if (open) reach = limit.clone();
      else {
        double margin = .05 * Math.max(box[2] - box[0], box[3] - box[1]);
        reach = new double[] {box[0] - margin, box[1] - margin, box[2] + margin, box[3] + margin};
      }
    }
  }

  /**
   * Surface heights on a north-up grid: cell (col, row) covers x0 + col * cell .. and
   * yTop - row * cell downwards; NaN = unknown.
   */
  static final class Surface {
    final int w, h;
    final double x0, yTop, cell;
    final float[] z;
    /** Per cell after finish(): FILLED = no cloud point (gap filling); SOFT = rough on its upper side (canopy). */
    static final byte FILLED = 1, SOFT = 2;
    byte[] kind;
    float top = Float.NEGATIVE_INFINITY, bottom = Float.POSITIVE_INFINITY;
    long known, filled, soft;

    Surface(int w, int h, double x0, double yTop, double cell) {
      this.w = w;
      this.h = h;
      this.x0 = x0;
      this.yTop = yTop;
      this.cell = cell;
      z = new float[Math.multiplyExact(w, h)];
      Arrays.fill(z, Float.NaN);
    }

    /** The highest point per cell of the cloud inside the grid. */
    void add(Cloud cloud, String label) throws IOException {
      cloud.scan(label, (x, y, v) -> {
        if (!Double.isFinite(x + y + v)) return;
        int col = (int) Math.floor((x - x0) / cell), row = (int) Math.floor((yTop - y) / cell);
        if (col < 0 || row < 0 || col >= w || row >= h) return;
        int p = row * w + col;
        if (!(z[p] >= v)) z[p] = (float) v;
      });
    }

    /**
     * Fills unknown cells up to reach cells away from a known one (8-neighbour steps), classifies
     * the cells (SOFT) and records the height range. Gaps are flooded from their lowest solid rim
     * first (cells taken in order of height, each passing its height on), so a gap beside a roof
     * or a crown continues the ground and the object does not grow sideways. What that leaves,
     * gaps with canopy (SOFT cells) all around, is flooded from the nearest rim cell: continuing
     * the lowest crown cell would dig pits between the crowns that no camera sees into. The
     * filled cells are then smoothed among
     * themselves and their known neighbours of similar height (within two cells), which turns the
     * flat fill into a ramp on sloping ground. Only cells enclosed by cloud points are kept (no
     * growth beyond the outer edge of the cloud). Deterministic: ties go to the lower cell index.
     */
    void finish(int reach) {
      finish(reach, 1);
    }

    void finish(int reach, int workers) {
      reach = Math.min(reach, Short.MAX_VALUE);
      kind = new byte[z.length];
      int unknown = 0;
      for (float v : z) if (Float.isNaN(v)) unknown++;
      if (unknown > 0 && unknown < z.length && reach > 0) {
        short[] steps = new short[z.length];
        classify(workers);
        // Two floods: from solid rims in order of height (unsigned keys: negative heights first),
        // then from canopy rims in order of distance. Solid rims go first whatever their height:
        // where ground is within reach the gap continues the ground.
        java.util.PriorityQueue<Long> queue = new java.util.PriorityQueue<>(Long::compareUnsigned), canopy = new java.util.PriorityQueue<>();
        for (int p = 0; p < z.length; p++) {
          if (Float.isNaN(z[p])) continue;
          int x = p % w, y = p / w;
          boolean rim = false;
          for (int dy = -1; dy <= 1 && !rim; dy++)
            for (int dx = -1; dx <= 1 && !rim; dx++) {
              int xx = x + dx, yy = y + dy;
              rim = xx >= 0 && yy >= 0 && xx < w && yy < h && Float.isNaN(z[yy * w + xx]);
            }
          if (!rim) continue;
          if ((kind[p] & SOFT) != 0) canopy.add((long) p);
          else queue.add(key(z[p], p));
        }
        int[] list = new int[16];
        int count = 0;
        while (!queue.isEmpty() || !canopy.isEmpty()) {
          int p = (int) (long) (queue.isEmpty() ? canopy : queue).poll(), x = p % w, y = p / w;
          if (steps[p] >= reach) continue;
          for (int dy = -1; dy <= 1; dy++)
            for (int dx = -1; dx <= 1; dx++) {
              int xx = x + dx, yy = y + dy, q = yy * w + xx;
              if (xx < 0 || yy < 0 || xx >= w || yy >= h || !Float.isNaN(z[q])) continue;
              z[q] = z[p];
              kind[q] = kind[p];
              steps[q] = (short) (steps[p] + 1);
              if (count == list.length) list = Arrays.copyOf(list, (int) Math.min(z.length, 2L * count));
              list[count++] = q;
              if ((kind[q] & SOFT) != 0) canopy.add((long) steps[q] << 32 | q);
              else queue.add(key(z[q], q));
            }
        }
        // Keep only interpolation: a filled cell stays when, on at least two of the four axes
        // through it, cells with cloud points lie within reach on both sides. Cells beyond the
        // outer edge of the cloud are dropped again.
        int kept = 0;
        int[][] axes = {{1, 0}, {0, 1}, {1, 1}, {1, -1}};
        boolean[] drop = new boolean[count];
        for (int i = 0; i < count; i++) {
          int p = list[i], x = p % w, y = p / w, enclosed = 0;
          for (int[] axis : axes) {
            boolean both = true;
            for (int sign = -1; sign <= 1 && both; sign += 2) {
              boolean found = false;
              for (int k = 1; k <= reach && !found; k++) {
                int xx = x + sign * k * axis[0], yy = y + sign * k * axis[1];
                if (xx < 0 || yy < 0 || xx >= w || yy >= h) break;
                int q = yy * w + xx;
                found = steps[q] == 0 && !Float.isNaN(z[q]);
              }
              both = found;
            }
            if (both) enclosed++;
          }
          drop[i] = enclosed < 2;
        }
        for (int i = 0; i < count; i++)
          if (drop[i]) z[list[i]] = Float.NaN;
          else list[kept++] = list[i];
        count = kept;
        filled = count;
        float[] next = new float[count];
        for (int pass = 0; pass < Math.min(reach, 32); pass++) {
          for (int i = 0; i < count; i++) {
            int p = list[i], x = p % w, y = p / w, n = 0;
            double sum = 0;
            for (int dy = -1; dy <= 1; dy++)
              for (int dx = -1; dx <= 1; dx++) {
                int xx = x + dx, yy = y + dy;
                if (xx < 0 || yy < 0 || xx >= w || yy >= h) continue;
                float v = z[yy * w + xx];
                if (Math.abs(v - z[p]) <= 2 * cell) {
                  sum += v;
                  n++;
                }
              }
            next[i] = (float) (sum / n);
          }
          for (int i = 0; i < count; i++) z[list[i]] = next[i];
        }
        Arrays.fill(kind, (byte) 0);
        for (int i = 0; i < count; i++) kind[list[i]] = FILLED | SOFT;
      }
      known = 0;
      for (float v : z)
        if (!Float.isNaN(v)) {
          known++;
          top = Math.max(top, v);
          bottom = Math.min(bottom, v);
        }
      classify(workers);
      soft = 0;
      for (byte k : kind) if ((k & SOFT) != 0) soft++;
    }

    static final int ROUGH_RADIUS = 10, ROUGH_DEPTH = 10, ROUGH_PAIRS = 6;
    static final double ROUGH_FRACTION = .1;

    /**
     * Marks the cells that are rough on their upper side as SOFT. For a cell, the cells of its
     * neighbourhood (ROUGH_RADIUS) that are not more than ROUGH_DEPTH cells below it are its own
     * object seen from above (a roof for a roof-edge cell, never the ground beside it); among the
     * pairs of adjacent cells of that set, height steps of more than two cells are counted. A
     * roof, a wall top or a car has almost none; a crown is full of them. SOFT: a share of steps
     * of at least ROUGH_FRACTION, or fewer than ROUGH_PAIRS pairs (tips of crowns, poles).
     * Depends on the heights only (rows in parallel, each cell on its own).
     */
    private void classify(int workers) {
      int radius = ROUGH_RADIUS;
      float depth = (float) (ROUGH_DEPTH * cell), step = (float) (2 * cell);
      Parallel.forEach(h, workers, y -> {
        int ya = Math.max(0, y - radius), yb = Math.min(h - 1, y + radius);
        for (int x = 0; x < w; x++) {
          int p = y * w + x;
          if (Float.isNaN(z[p])) continue;
          float floor = z[p] - depth;
          int xa = Math.max(0, x - radius), xb = Math.min(w - 1, x + radius), pairs = 0, steps = 0;
          for (int yy = ya; yy <= yb; yy++)
            for (int xx = xa, q = yy * w + xa; xx <= xb; xx++, q++) {
              float a = z[q];
              if (!(a >= floor)) continue;
              if (xx < xb) {
                float b = z[q + 1];
                if (b >= floor) {
                  pairs++;
                  if (Math.abs(a - b) > step) steps++;
                }
              }
              if (yy < yb) {
                float b = z[q + w];
                if (b >= floor) {
                  pairs++;
                  if (Math.abs(a - b) > step) steps++;
                }
              }
            }
          if ((kind[p] & FILLED) == 0 && (pairs < ROUGH_PAIRS || steps >= ROUGH_FRACTION * pairs)) kind[p] |= SOFT;
        }
      });
    }

    /** Heap key: height first (ascending), then cell index. */
    private static long key(float v, int p) {
      int bits = Float.floatToIntBits(v);
      return (bits >= 0 ? (long) bits + 0x80000000L : (long) (~bits & 0x7fffffff)) << 32 | (p & 0xffffffffL);
    }

    /**
     * Height at a world position: bilinear between the four nearest cell centres where the
     * surface is continuous (their heights within two cells of each other), the nearest cell at a
     * height step or next to unknown cells; NaN when the nearest cell is unknown.
     */
    double height(double x, double y) {
      double gx = (x - x0) / cell - .5, gy = (yTop - y) / cell - .5;
      int ix = (int) Math.floor(gx), iy = (int) Math.floor(gy);
      double fx = gx - ix, fy = gy - iy;
      int nx = Math.max(0, Math.min(w - 1, fx < .5 ? ix : ix + 1)), ny = Math.max(0, Math.min(h - 1, fy < .5 ? iy : iy + 1));
      float nearest = z[ny * w + nx];
      if (Float.isNaN(nearest) || ix < 0 || iy < 0 || ix >= w - 1 || iy >= h - 1) return nearest;
      float a = z[iy * w + ix], b = z[iy * w + ix + 1], c = z[(iy + 1) * w + ix], d = z[(iy + 1) * w + ix + 1];
      float low = Math.min(Math.min(a, b), Math.min(c, d)), high = Math.max(Math.max(a, b), Math.max(c, d));
      // Math.min/max return NaN if any corner is unknown.
      if (!(high - low <= 2 * cell)) return nearest;
      return (1 - fy) * ((1 - fx) * a + fx * b) + fy * ((1 - fx) * c + fx * d);
    }

    /**
     * Whether the straight line from the surface point (x, y, v) to a camera centre stays above
     * the surface: sampled every three quarters of a cell from one and a half cells onwards (the
     * point's own cell never blocks it) until the line is above the highest cell. Near the point
     * a cell blocks only when it is well above the line (one and a half cells: rough ground does
     * not hide itself); the allowance shrinks with distance and from four and a half cells on a
     * cell blocks as soon as it is above the line. lenient keeps the near allowance all the way
     * (for surface points no camera sees strictly, e.g. inside rough canopy).
     */
    boolean visible(double[] camera, double x, double y, double v, boolean lenient) {
      double dx = camera[0] - x, dy = camera[1] - y, dz = camera[2] - v, length = Math.sqrt(dx * dx + dy * dy);
      if (length < cell || !(dz > 0)) return dz > 0;
      double end = Math.min(length, length * (top - v) / dz), step = .75 * cell;
      for (double s = 2 * step; s < end; s += step) {
        double t = s / length;
        int col = (int) Math.floor((x + dx * t - x0) / cell), row = (int) Math.floor((yTop - y - dy * t) / cell);
        if (col < 0 || row < 0 || col >= w || row >= h) continue;
        float surface = z[row * w + col];
        double tolerance = lenient ? 1.5 * cell : Math.max(0, 1.5 * cell - .5 * (s - 2 * step));
        if (surface > v + dz * t + tolerance) return false;
      }
      return true;
    }

    /**
     * How far the surface rises into the line of sight of visible(lenient): the largest height of
     * a cell above the line and its allowance; 0 when the line is free. POSITIVE_INFINITY when the
     * camera is not above the point, when the obstruction exceeds limit, or when a cell that is
     * not SOFT is in the way (a roof hides what is behind it; canopy and filled gaps are not
     * taken as tight occluders).
     */
    double obstruction(double[] camera, double x, double y, double v, double limit) {
      double dx = camera[0] - x, dy = camera[1] - y, dz = camera[2] - v, length = Math.sqrt(dx * dx + dy * dy);
      if (!(dz > 0)) return Double.POSITIVE_INFINITY;
      if (length < cell) return 0;
      double end = Math.min(length, length * (top - v) / dz), step = .75 * cell, worst = 0;
      for (double s = 2 * step; s < end; s += step) {
        double t = s / length;
        int col = (int) Math.floor((x + dx * t - x0) / cell), row = (int) Math.floor((yTop - y - dy * t) / cell);
        if (col < 0 || row < 0 || col >= w || row >= h) continue;
        double over = z[row * w + col] - (v + dz * t + 1.5 * cell);
        if (!(over > 0)) continue;
        if ((kind[row * w + col] & SOFT) == 0) return Double.POSITIVE_INFINITY;
        if (over > worst) {
          worst = over;
          if (worst > limit) return Double.POSITIVE_INFINITY;
        }
      }
      return worst;
    }
  }

  /** Counts of a rendered mosaic. */
  static final class Result {
    long painted, unknown, unseen, relaxed;
    long[] pixels;
  }

  /**
   * Renders the mosaic of width x height pixels of size cell, top-left outer corner (x0, yTop),
   * and passes its compressed tiles (TILE pixels, row-major R G B A, alpha 0 = no data) to sink
   * in row-major order. occlusion false skips the visibility test (a conventional orthophoto:
   * tall objects lean and cover the ground; for tests). canopy false leaves a pixel that no
   * camera passes the test for transparent also where only canopy is in the way
   * (--orthomosaic-strict).
   */
  static Result render(
      Surface surface, List<View> views, Photos photos, double x0, double yTop, double cell, int width, int height,
      int workers, boolean occlusion, TileSink sink)
      throws IOException {
    return render(surface, views, photos, x0, yTop, cell, width, height, workers, occlusion, true, sink);
  }

  static Result render(
      Surface surface, List<View> views, Photos photos, double x0, double yTop, double cell, int width, int height,
      int workers, boolean occlusion, boolean canopy, TileSink sink)
      throws IOException {
    return render(surface, views, photos, x0, yTop, cell, width, height, workers, occlusion, canopy, null, sink);
  }

  /** gpu: the camera choice on the GPU where it applies (same result), or null. */
  static Result render(
      Surface surface, List<View> views, Photos photos, double x0, double yTop, double cell, int width, int height,
      int workers, boolean occlusion, boolean canopy, GpuOrthomosaic gpu, TileSink sink)
      throws IOException {
    Result result = new Result();
    result.pixels = new long[views.size()];
    double[] limit = {x0, yTop - height * cell, x0 + width * cell, yTop};
    for (View v : views) if (v.reach == null) v.reach(surface.bottom, surface.top, limit);
    int blocksAcross = Math.ceilDiv(width, BLOCK), blocksDown = Math.ceilDiv(height, BLOCK);
    try (WorkProgress work = WorkProgress.start("Orthomosaic blocks", (long) blocksAcross * blocksDown, "blocks")) {
      long done = 0;
      for (int by = 0; by < height; by += BLOCK)
        for (int bx = 0; bx < width; bx += BLOCK) {
          int bw = Math.min(BLOCK, width - bx), bh = Math.min(BLOCK, height - by);
          block(surface, views, photos, x0, yTop, cell, bx, by, bw, bh, workers, occlusion, canopy, gpu, sink, result);
          work.update(++done);
        }
      work.done();
    }
    return result;
  }

  /** State of a pixel after the choice of its camera. */
  static final byte PAINTED = 0, UNKNOWN = 1, UNSEEN = 2, RELAXED = 3;

  /**
   * The surface height and the camera of each pixel of row row of a block (left, upper: its outer
   * corner; bw pixels of size cell), written from index at: chosen (index into views or -1),
   * heights and state. GpuOrthomosaic repeats this operation for operation.
   */
  static void chooseRow(
      Surface surface, List<View> views, int[] candidates, double left, double upper, double cell, int bw, int row,
      boolean occlusion, boolean relax, short[] chosen, float[] heights, byte[] state, int at) {
    double[] uv = new double[2], excess = new double[candidates.length], scores = new double[candidates.length], pending = new double[candidates.length];
    double y = upper - (row + .5) * cell;
    for (int col = 0; col < bw; col++) {
      int p = at + col;
      chosen[p] = -1;
      state[p] = PAINTED;
      double x = left + (col + .5) * cell, v = surface.height(x, y);
      heights[p] = (float) v;
      if (Double.isNaN(v)) {
        state[p] = UNKNOWN;
        continue;
      }
      // Nearest the image centre among the cameras with a strictly free line of sight; if there
      // is none, among those that pass the lenient test. Cameras are tried from the nearest
      // outwards (equal scores: the earlier camera), so the first that passes is the answer.
      for (int k = 0; k < candidates.length; k++) {
        View view = views.get(candidates[k]);
        scores[k] = Double.POSITIVE_INFINITY;
        if (view.project(x, y, v, uv) && uv[0] >= 0 && uv[1] >= 0 && uv[0] <= view.width - 1 && uv[1] <= view.height - 1) {
          double a = (uv[0] - view.cx) / view.fx, b = (uv[1] - view.cy) / view.fy;
          scores[k] = a * a + b * b;
        }
        pending[k] = scores[k];
      }
      int lenient = -1;
      while (true) {
        int k = -1;
        double best = Double.POSITIVE_INFINITY;
        for (int j = 0; j < candidates.length; j++)
          if (pending[j] < best) {
            best = pending[j];
            k = j;
          }
        if (k < 0) break;
        pending[k] = Double.POSITIVE_INFINITY;
        View view = views.get(candidates[k]);
        if (!occlusion || surface.visible(view.centre, x, y, v, false)) {
          chosen[p] = (short) candidates[k];
          break;
        }
        if (lenient < 0 && surface.visible(view.centre, x, y, v, true)) lenient = candidates[k];
      }
      if (chosen[p] < 0) chosen[p] = (short) lenient;
      if (chosen[p] < 0 && relax) {
        // No camera passes the test. Canopy is rough and porous, and the test is too tight for
        // it: cameras with only canopy (SOFT cells) in the way remain candidates, never those
        // behind anything solid. Of the least obstructed ones, the nearest its image centre.
        double least = Double.POSITIVE_INFINITY;
        for (int k = 0; k < candidates.length; k++) {
          excess[k] = Double.POSITIVE_INFINITY;
          if (scores[k] == Double.POSITIVE_INFINITY) continue;
          excess[k] = surface.obstruction(views.get(candidates[k]).centre, x, y, v, 1.25 * least + 4 * surface.cell);
          least = Math.min(least, excess[k]);
        }
        if (least < Double.POSITIVE_INFINITY) {
          double limit = 1.25 * least + 4 * surface.cell, best = Double.POSITIVE_INFINITY;
          for (int k = 0; k < candidates.length; k++)
            if (excess[k] <= limit && scores[k] < best) {
              best = scores[k];
              chosen[p] = (short) candidates[k];
            }
          state[p] = RELAXED;
        }
      }
      if (chosen[p] < 0) state[p] = UNSEEN;
    }
  }

  private static void block(
      Surface surface, List<View> views, Photos photos, double x0, double yTop, double cell, int bx, int by, int bw, int bh,
      int workers, boolean occlusion, boolean canopy, GpuOrthomosaic gpu, TileSink sink, Result result)
      throws IOException {
    double left = x0 + bx * cell, right = left + bw * cell, upper = yTop - by * cell, lower = upper - bh * cell;
    List<Integer> near = new ArrayList<>();
    for (int i = 0; i < views.size(); i++) {
      double[] r = views.get(i).reach;
      if (r[0] <= right && r[2] >= left && r[1] <= upper && r[3] >= lower) near.add(i);
    }
    int[] candidates = near.stream().mapToInt(Integer::intValue).toArray();
    // 1. Per pixel: the surface height and the camera that sees it best.
    short[] chosen = new short[bw * bh];
    float[] heights = new float[bw * bh];
    byte[] state = new byte[bw * bh];
    boolean relax = occlusion && canopy && surface.kind != null;
    if (gpu == null || !gpu.choose(views, candidates, left, upper, cell, bw, bh, workers, occlusion, relax, chosen, heights, state))
      Parallel.forEach(bh, workers, row -> chooseRow(surface, views, candidates, left, upper, cell, bw, row, occlusion, relax, chosen, heights, state, row * bw));
    long[] states = new long[4];
    for (byte s : state) states[s]++;
    result.unknown += states[UNKNOWN];
    result.unseen += states[UNSEEN];
    result.relaxed += states[RELAXED];
    // 2. Colours, one camera at a time: its source window, then bilinear samples.
    int[] rgba = new int[bw * bh];
    boolean[] used = new boolean[views.size()];
    for (short c : chosen) if (c >= 0) used[c] = true;
    for (int i : candidates) {
      if (!used[i]) continue;
      View view = views.get(i);
      int[][] box = new int[bh][];
      Parallel.forEach(bh, workers, row -> {
        double[] uv = new double[2];
        int[] b = {Integer.MAX_VALUE, Integer.MAX_VALUE, -1, -1};
        double y = upper - (row + .5) * cell;
        for (int col = 0; col < bw; col++) {
          int p = row * bw + col;
          if (chosen[p] != i || !view.project(left + (col + .5) * cell, y, heights[p], uv)) continue;
          b[0] = Math.min(b[0], (int) uv[0]);
          b[1] = Math.min(b[1], (int) uv[1]);
          b[2] = Math.max(b[2], (int) uv[0] + 1);
          b[3] = Math.max(b[3], (int) uv[1] + 1);
        }
        box[row] = b;
      });
      int u0 = Integer.MAX_VALUE, v0 = Integer.MAX_VALUE, u1 = -1, v1 = -1;
      for (int[] b : box) {
        u0 = Math.min(u0, b[0]);
        v0 = Math.min(v0, b[1]);
        u1 = Math.max(u1, b[2]);
        v1 = Math.max(v1, b[3]);
      }
      if (u1 < 0) continue;
      u1 = Math.min(u1, view.width - 1);
      v1 = Math.min(v1, view.height - 1);
      int ww = u1 - u0 + 1, wh = v1 - v0 + 1, ou = u0, ov = v0;
      int[] window = photos.window(i, u0, v0, ww, wh);
      long[] counts = new long[bh];
      Parallel.forEach(bh, workers, row -> {
        double[] uv = new double[2];
        double y = upper - (row + .5) * cell;
        for (int col = 0; col < bw; col++) {
          int p = row * bw + col;
          if (chosen[p] != i || !view.project(left + (col + .5) * cell, y, heights[p], uv)) continue;
          double u = uv[0] - ou, v = uv[1] - ov;
          int iu = Math.min((int) u, ww - 2), iv = Math.min((int) v, wh - 2);
          if (ww < 2) iu = 0;
          if (wh < 2) iv = 0;
          double fu = u - iu, fv = v - iv;
          int a = window[iv * ww + iu], b = window[iv * ww + Math.min(ww - 1, iu + 1)];
          int c = window[Math.min(wh - 1, iv + 1) * ww + iu], d = window[Math.min(wh - 1, iv + 1) * ww + Math.min(ww - 1, iu + 1)];
          int colour = 0xff000000;
          for (int shift = 16; shift >= 0; shift -= 8) {
            double upperRow = (1 - fu) * ((a >> shift) & 255) + fu * ((b >> shift) & 255);
            double lowerRow = (1 - fu) * ((c >> shift) & 255) + fu * ((d >> shift) & 255);
            colour |= (int) Math.round((1 - fv) * upperRow + fv * lowerRow) << shift;
          }
          rgba[p] = colour;
          counts[row]++;
        }
      });
      for (long n : counts) {
        result.pixels[i] += n;
        result.painted += n;
      }
    }
    // 3. Tiles of this block, compressed in parallel, delivered in row-major order.
    int tilesAcross = Math.ceilDiv(bw, TILE), tilesDown = Math.ceilDiv(bh, TILE);
    byte[][] packed = new byte[tilesAcross * tilesDown][];
    Parallel.forEach(packed.length, workers, t -> {
      int tx = t % tilesAcross, ty = t / tilesAcross;
      byte[] samples = new byte[TILE * TILE * 4];
      for (int y = 0; y < TILE && ty * TILE + y < bh; y++)
        for (int x = 0; x < TILE && tx * TILE + x < bw; x++) {
          int value = rgba[(ty * TILE + y) * bw + tx * TILE + x], at = 4 * (y * TILE + x);
          samples[at] = (byte) (value >> 16);
          samples[at + 1] = (byte) (value >> 8);
          samples[at + 2] = (byte) value;
          samples[at + 3] = (byte) (value >>> 24);
        }
      packed[t] = TiledGeoTiff.compress(samples);
    });
    for (int t = 0; t < packed.length; t++) sink.write(bx / TILE + t % tilesAcross, by / TILE + t / tilesAcross, packed[t]);
  }

  /**
   * Photographs of a run: whole images up to wholePixels are decoded once and kept (least
   * recently used first out) within cacheBytes; larger ones are read by source window.
   */
  static final class RunPhotos implements Photos, Closeable {
    private final List<Path> files;
    private final long cacheBytes, wholePixels;
    private final LinkedHashMap<Integer, int[]> whole = new LinkedHashMap<>(16, .75f, true);
    private final LinkedHashMap<Integer, WindowedImage.Source> open = new LinkedHashMap<>(16, .75f, true);
    private final int[] widths, heights;
    private long cached;
    long decoded, windows;

    RunPhotos(List<Path> files, int[] widths, int[] heights, long cacheBytes) {
      this.files = files;
      this.widths = widths;
      this.heights = heights;
      this.cacheBytes = cacheBytes;
      wholePixels = cacheBytes / 16;
    }

    @Override
    public int[] window(int view, int x, int y, int w, int h) throws IOException {
      int width = widths[view], height = heights[view];
      if ((long) width * height > wholePixels) {
        windows++;
        return source(view).window(x, y, w, h);
      }
      int[] image = whole.get(view);
      if (image == null) {
        try (WindowedImage.Source source = new WindowedImage.Source(files.get(view))) {
          image = source.window(0, 0, width, height);
        }
        decoded++;
        cached += 4L * image.length;
        whole.put(view, image);
        var oldest = whole.entrySet().iterator();
        while (cached > cacheBytes && whole.size() > 1) {
          var entry = oldest.next();
          if (entry.getKey() == view) continue;
          cached -= 4L * entry.getValue().length;
          oldest.remove();
        }
      }
      if (x == 0 && y == 0 && w == width && h == height) return image;
      int[] out = new int[w * h];
      for (int row = 0; row < h; row++) System.arraycopy(image, (y + row) * width + x, out, row * w, w);
      return out;
    }

    private WindowedImage.Source source(int view) throws IOException {
      WindowedImage.Source source = open.get(view);
      if (source == null) {
        if (open.size() >= 8) {
          var oldest = open.entrySet().iterator();
          oldest.next().getValue().close();
          oldest.remove();
        }
        source = new WindowedImage.Source(files.get(view));
        open.put(view, source);
      }
      return source;
    }

    @Override
    public void close() {
      for (WindowedImage.Source source : open.values()) source.close();
      open.clear();
      whole.clear();
    }
  }

  /** --export-orthomosaic RUN_DIRECTORY [--orthomosaic-output DIRECTORY]. */
  static void export(Options o) throws IOException {
    Path run = o.exportOrthomosaic.toAbsolutePath().normalize();
    if (!Files.isRegularFile(run.resolve("run.json")))
      throw new IllegalArgumentException("--export-orthomosaic needs a project run directory (with run.json): " + run);
    Path out = o.orthomosaicOutput == null ? run.resolve("07_results") : o.orthomosaicOutput.toAbsolutePath().normalize();
    Map<String, Object> result = create(run, out, o);
    if (!"complete".equals(result.get("status")))
      throw new IllegalArgumentException("Orthomosaic unavailable: " + result.get("reason"));
  }

  private static Map<String, Object> unavailable(Path out, String reason) throws IOException {
    Map<String, Object> result = Json.object("status", "unavailable", "reason", reason);
    Json.write(out.resolve("orthomosaic.json"), result);
    System.out.println("Orthomosaic unavailable: " + reason);
    return result;
  }

  /**
   * Writes out/orthomosaic.tif, .tfw and .json for a finished run (its final cloud, archived
   * orientations and the original photographs). Returns the metadata; status "unavailable" with
   * a reason when something needed is missing.
   */
  static Map<String, Object> create(Path run, Path out, Options o) throws IOException {
    Files.createDirectories(out);
    for (String name : List.of("orthomosaic.tif", "orthomosaic.tfw", "orthomosaic.json")) Files.deleteIfExists(out.resolve(name));
    long clock = System.nanoTime();
    Map<String, Object> coordinates = ProcessingReport.map(run.resolve("07_results/coordinates.json"));
    if (coordinates.isEmpty()) coordinates = ProcessingReport.map(run.resolve("coordinates.json"));
    int epsg = coordinates.get("epsg") instanceof Number n ? n.intValue() : 0;
    boolean projected = Boolean.TRUE.equals(coordinates.get("georeferenced")) && epsg > 0;
    if (!projected && !o.reportLocalDsm)
      return unavailable(out, "no projected coordinate frame; use --report-local-dsm when the local XY/Z frame is meaningful (Z up)");
    double scale = 1;
    Path settings = run.resolve("logs/run-settings.properties");
    if (Files.isRegularFile(settings)) {
      Properties properties = new Properties();
      try (Reader reader = Files.newBufferedReader(settings)) {
        properties.load(reader);
      }
      try {
        scale = Double.parseDouble(properties.getProperty("scale", "1"));
      } catch (NumberFormatException ignored) {
        scale = 1;
      }
    }
    if (!(scale > 0 && Double.isFinite(scale))) scale = 1;
    Options sourceOptions = o.copy();
    sourceOptions.orientationPixels = "source";
    OrientationIO.Model model;
    try {
      model = OrientationIO.readRun(run, sourceOptions);
    } catch (IOException | IllegalArgumentException missing) {
      return unavailable(out, "camera orientations unavailable: " + missing.getMessage());
    }
    Map<String, Path> sources = ReportMaps.sourceFiles(run);
    List<View> views = new ArrayList<>();
    List<Path> files = new ArrayList<>();
    List<String> skipped = new ArrayList<>();
    for (var v : model.views.values()) {
      if (v.pose() == null || v.calibration() == null) continue;
      Path file = sources.get(v.image());
      String problem = null;
      if (file == null || !Files.isRegularFile(file)) problem = "photograph not found";
      else
        try (WindowedImage.Source source = new WindowedImage.Source(file)) {
          if (source.width() != v.calibration().width || source.height() != v.calibration().height)
            problem = "photograph is " + source.width() + " x " + source.height() + ", calibration " + v.calibration().width + " x " + v.calibration().height;
        } catch (IOException unreadable) {
          problem = "unreadable: " + unreadable.getMessage();
        }
      if (problem != null) {
        skipped.add(v.image() + " (" + problem + ")");
        continue;
      }
      views.add(new View(v.image(), v.pose(), v.calibration(), scale));
      files.add(file);
    }
    if (views.isEmpty())
      return unavailable(out, "no oriented photograph is readable" + (skipped.isEmpty() ? "" : ": " + skipped.get(0)));
    if (views.size() > Short.MAX_VALUE) return unavailable(out, "more than 32,767 photographs");
    Path path = run.resolve("07_results/dense.ply");
    if (!Files.isRegularFile(path)) path = run.resolve("dense.ply");
    PlyStream cloud;
    if (Files.isRegularFile(path)) cloud = new PlyStream(path);
    else {
      Path las = run.resolve("07_results/dense.las");
      if (!Files.isRegularFile(las)) las = run.resolve("dense.las");
      if (!Files.isRegularFile(las)) return unavailable(out, "no final PLY/LAS cloud");
      cloud = PlyStream.las(las);
    }
    // Extent and mean height of the cloud.
    double[] bounds = {Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY};
    double[] sum = new double[2];
    cloud.scan("Orthomosaic: cloud extent", (x, y, z) -> {
      if (!Double.isFinite(x + y + z)) return;
      bounds[0] = Math.min(bounds[0], x);
      bounds[1] = Math.min(bounds[1], y);
      bounds[2] = Math.max(bounds[2], x);
      bounds[3] = Math.max(bounds[3], y);
      sum[0] += z;
      sum[1]++;
    });
    if (!(sum[1] > 0) || !(bounds[2] > bounds[0]) || !(bounds[3] > bounds[1]))
      return unavailable(out, "the final cloud has no extent");
    double meanHeight = sum[0] / sum[1];
    // Pixel size: the median ground sample distance of the photographs, unless given.
    double[] gsd = new double[views.size()];
    for (int i = 0; i < gsd.length; i++) gsd[i] = Math.abs(views.get(i).centre[2] - meanHeight) / Math.min(views.get(i).fx, views.get(i).fy);
    Arrays.sort(gsd);
    double sourceGsd = gsd[gsd.length / 2];
    double cell = o.orthomosaicResolution > 0 ? o.orthomosaicResolution : twoDigits(sourceGsd);
    if (!(cell > 0) || !Double.isFinite(cell)) return unavailable(out, "no usable pixel size (camera heights above the cloud)");
    double x0 = Math.floor(bounds[0] / cell) * cell, yTop = Math.ceil(bounds[3] / cell) * cell;
    long wl = Math.max(1, (long) Math.ceil((bounds[2] - x0) / cell)), hl = Math.max(1, (long) Math.ceil((yTop - bounds[1]) / cell));
    if (wl > 2_000_000 || hl > 2_000_000 || wl * hl > 200_000_000_000L)
      return unavailable(out, "a mosaic of " + wl + " x " + hl + " pixels is too large; raise --orthomosaic-resolution (pixel size " + cell + ")");
    int width = (int) wl, height = (int) hl;
    // Surface cell: twice the point spacing (from the occupied area on a coarse grid), not finer
    // than a pixel, and within an eighth of the heap.
    double area = (bounds[2] - bounds[0]) * (bounds[3] - bounds[1]);
    double surfaceCell = o.orthomosaicSurfaceCell;
    long maxCells = Math.max(1 << 20, Runtime.getRuntime().maxMemory() / 8 / 8);
    double spacing = Double.NaN;
    if (!(surfaceCell > 0)) {
      double trial = Math.max(cell, Math.sqrt(area / 4e6));
      Surface coarse = grid(bounds, trial);
      coarse.add(cloud::scan, "Orthomosaic: point spacing");
      long occupied = 0;
      for (float v : coarse.z) if (!Float.isNaN(v)) occupied++;
      spacing = Math.sqrt(occupied * trial * trial / sum[1]);
      surfaceCell = Math.max(cell, 2 * spacing);
    }
    surfaceCell = Math.max(surfaceCell, Math.sqrt(area / maxCells));
    Surface surface = grid(bounds, surfaceCell);
    surface.add(cloud::scan, "Orthomosaic: surface");
    int fill = o.orthomosaicFill < 0 ? 64 : (int) Math.ceil(o.orthomosaicFill / surfaceCell);
    surface.finish(fill, Math.max(1, o.cores));
    if (surface.known == 0) return unavailable(out, "the surface is empty");
    boolean big = TiledGeoTiff.needsBig(width, height);
    System.out.printf(
        Locale.ROOT,
        "Orthomosaic: %,d x %,d pixels of %s (source ground sample distance %s), surface cells of %s (%,d x %,d), %d photographs%s%n",
        width, height, units(cell), units(sourceGsd), units(surfaceCell), surface.w, surface.h, views.size(), big ? ", BigTIFF" : "");
    int[] widths = new int[views.size()], heights = new int[views.size()];
    for (int i = 0; i < views.size(); i++) {
      widths[i] = views.get(i).width;
      heights[i] = views.get(i).height;
    }
    long cache = Math.max(256L << 20, Runtime.getRuntime().maxMemory() / 3);
    Result result;
    Path file = out.resolve("orthomosaic.tif");
    RunPhotos photos = new RunPhotos(files, widths, heights, cache);
    GpuOrthomosaic gpu = GpuOrthomosaic.open(o, surface, (int) Math.min((long) BLOCK * BLOCK, (long) width * height));
    try (photos; gpu; TiledGeoTiff tiff = new TiledGeoTiff(file, width, height, TILE, x0, yTop, cell, projected ? epsg : 0, big)) {
      result = render(surface, views, photos, x0, yTop, cell, width, height, Math.max(1, o.cores), true, !o.orthomosaicStrict, gpu, tiff::write);
    } catch (IOException | RuntimeException | Error failure) {
      Files.deleteIfExists(file);
      throw failure;
    }
    Files.writeString(
        out.resolve("orthomosaic.tfw"),
        String.format(Locale.ROOT, "%.12f%n0%n0%n%.12f%n%.12f%n%.12f%n", cell, -cell, x0 + cell / 2, yTop - cell / 2));
    int contributing = 0;
    for (long n : result.pixels) if (n > 0) contributing++;
    long total = (long) width * height;
    Map<String, Object> metadata =
        Json.object(
            "status", "complete",
            "file", "orthomosaic.tif",
            "format", (big ? "BigTIFF" : "TIFF") + ", 8-bit RGBA tiles of " + TILE + " pixels, Deflate; alpha 0 = no data",
            "width", width,
            "height", height,
            "cell_size", cell,
            "cell_size_basis", o.orthomosaicResolution > 0 ? "--orthomosaic-resolution" : "median ground sample distance of the photographs, two significant digits",
            "source_ground_sample_distance", sourceGsd,
            "extent", new double[] {x0, yTop - height * cell, x0 + width * cell, yTop},
            "epsg", projected ? epsg : null,
            "units", coordinates.getOrDefault("xyz_units", "local output units"),
            "surface_cell_size", surfaceCell,
            "surface_point_spacing", Double.isNaN(spacing) ? null : spacing,
            "surface_width", surface.w,
            "surface_height", surface.h,
            "surface_cells_known", surface.known,
            "surface_cells_filled", surface.filled,
            "surface_cells_canopy", surface.soft,
            "surface_fill_cells", fill,
            "source_points", cloud.count,
            "photographs_used", views.size(),
            "photographs_contributing", contributing,
            "photographs_skipped", skipped,
            "painted_pixels", result.painted,
            "pixels_without_surface", result.unknown,
            "pixels_seen_by_no_camera", result.unseen,
            "pixels_painted_through_canopy", result.relaxed,
            "strict", o.orthomosaicStrict,
            "gpu_blocks", gpu == null ? 0 : gpu.blocks,
            "gpu_device_bytes", gpu == null ? 0 : gpu.deviceBytes,
            "painted_fraction", result.painted / (double) total,
            "file_bytes", Files.size(file),
            "photographs_decoded_whole", photos.decoded,
            "source_window_reads", photos.windows,
            "seconds", (System.nanoTime() - clock) / 1e9,
            "method",
            "Original photographs projected onto the highest-point surface of the final cloud. Each pixel takes the"
                + " camera that sees its surface point nearest the image centre among the cameras whose ray is not"
                + " blocked by the surface (true orthomosaic). Where no camera passes and only canopy (rough surface)"
                + " is in the way, the least obstructed camera paints the pixel unless strict; solid surfaces always"
                + " hide. Pixels without a surface or seen by no camera are transparent. No seam blending, no"
                + " radiometric balancing, no overviews.");
    Json.write(out.resolve("orthomosaic.json"), metadata);
    System.out.printf(
        Locale.ROOT,
        "Orthomosaic: %s, %,d MiB; %.1f%% of the pixels painted from %d of %d photographs (%.1f%% by the least obstructed camera; %.1f%% no surface, %.1f%% seen by no camera); %.1f s%n",
        file, Files.size(file) >> 20, 100. * result.painted / total, contributing, views.size(), 100. * result.relaxed / total, 100. * result.unknown / total,
        100. * result.unseen / total, (System.nanoTime() - clock) / 1e9);
    return metadata;
  }

  private static Surface grid(double[] bounds, double cell) {
    double x0 = Math.floor(bounds[0] / cell) * cell, yTop = Math.ceil(bounds[3] / cell) * cell;
    int w = (int) Math.max(1, Math.ceil((bounds[2] - x0) / cell) + 1), h = (int) Math.max(1, Math.ceil((yTop - bounds[1]) / cell) + 1);
    return new Surface(w, h, x0, yTop, cell);
  }

  /** v rounded up to two significant digits. */
  static double twoDigits(double v) {
    if (!(v > 0) || !Double.isFinite(v)) return v;
    double power = Math.pow(10, Math.floor(Math.log10(v)) - 1);
    return Math.ceil(v / power - 1e-9) * power;
  }

  private static String units(double v) {
    return String.format(Locale.ROOT, "%.4g", v);
  }
}
