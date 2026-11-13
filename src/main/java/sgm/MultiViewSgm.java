package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import java.util.concurrent.*;
import java.util.stream.IntStream;

/**
 * {@code --dense-method mvsgm}: multi-view plane-sweep matching, one depth map per camera
 * (MULTIVIEW_SGM.md).
 *
 * <p>Per reference camera c:
 *
 * <ol>
 *   <li>Neighbours: the cameras sharing the most sparse points with c at useful triangulation
 *       angles.
 *   <li>Labels: inverse depths q = 1/z of fronto-parallel planes in c, spaced so that one label
 *       moves a pixel by at most about one pixel in the widest-baseline neighbour; the range comes
 *       from the sparse points c sees.
 *   <li>Each neighbour is resampled once into a virtual camera at its own centre with c's rotation
 *       and intrinsics, and its census is computed there. A plane at q then maps a reference pixel
 *       p to {@code (p + q K t') / (1 + q t'z)} in that virtual image (t' = R_c (C_c - C_j)), a
 *       pure epipolar shift, so the neighbour census is looked up, never recomputed per label, and
 *       is comparable to c's even when the neighbour is rotated (opposite flight lines).
 *   <li>Cost per pixel and label: the mean of the k smallest neighbour Hamming costs (occlusion
 *       robust); missing neighbours count as a fixed penalty.
 * </ol>
 */
final class MultiViewSgm {
  /** Fixed-point cost scale: costs are Hamming distances times 16. */
  static final int SCALE = 16;

  final Options o;
  final int neighbors, k, census;
  final boolean bilinear;
  final double labelPixels;
  final int maxLabels;
  final Map<Integer, Object> neighborSelectionReports = new ConcurrentHashMap<>();

  MultiViewSgm(Options o) {
    this.o = o;
    neighbors = Math.max(1, Math.min(8, Integer.getInteger("sgm.mvsgm.neighbors", o.mvsgmNeighbors)));
    k = Integer.getInteger("sgm.mvsgm.k", o.mvsgmBestK);
    census = Integer.getInteger("sgm.mvsgm.census", o.mvsgmCensus);
    bilinear = true;
    labelPixels = Double.parseDouble(System.getProperty("sgm.mvsgm.labelPixels", "1.0"));
    maxLabels = Integer.getInteger("sgm.mvsgm.maxLabels", 1024);
    if (census < 3 || census > 7 || census % 2 == 0)
      throw new IllegalArgumentException("mvsgm census window must be 3, 5 or 7");
  }

  // ---------------------------------------------------------------------------------------------
  // Planning: neighbours and labels

  /** One neighbour of a reference camera. */
  record Neighbor(int id, double score, int shared, double medianAngle) {}

  /**
   * Neighbour score: the sum over shared sparse points of a view-selection weight of the
   * triangulation angle (peak at 10 degrees; falls off below with sigma 4, above with sigma 12;
   * angles under 1 degree count nothing).
   */
  static List<Neighbor> neighbours(
      List<ImageFrame> frames, List<FeatureTracks.Track> tracks, int c, int count) {
    Camera ref = frames.get(c).camera;
    double[] centre = ref.center();
    int n = frames.size();
    double[] score = new double[n];
    int[] shared = new int[n];
    List<List<Double>> angles = new ArrayList<>();
    for (int j = 0; j < n; j++) angles.add(new ArrayList<>());
    double[][] centres = new double[n][];
    for (int j = 0; j < n; j++)
      if (usable(frames.get(j).camera)) centres[j] = frames.get(j).camera.center();
    for (FeatureTracks.Track t : tracks) {
      if (t.point == null || Geometry.observation(t, c) == null) continue;
      double[] u = Math3.sub(t.point, centre);
      for (FeatureTracks.Observation ob : t.observations) {
        int j = ob.image;
        if (j == c || j >= n || centres[j] == null) continue;
        double[] v = Math3.sub(t.point, centres[j]);
        double cos = (u[0] * v[0] + u[1] * v[1] + u[2] * v[2]) / (Math3.norm(u) * Math3.norm(v));
        double angle = Math.toDegrees(Math.acos(Math.max(-1, Math.min(1, cos))));
        shared[j]++;
        angles.get(j).add(angle);
        if (angle < 1) continue;
        double s = angle <= 10 ? 4 : 12, d = angle - 10;
        score[j] += Math.exp(-d * d / (2 * s * s));
      }
    }
    List<Neighbor> out = new ArrayList<>();
    for (int j = 0; j < n; j++) {
      if (j == c || centres[j] == null || score[j] <= 0 || shared[j] < 12) continue;
      List<Double> a = angles.get(j);
      Collections.sort(a);
      out.add(new Neighbor(j, score[j], shared[j], a.get(a.size() / 2)));
    }
    out.sort(Comparator.comparingDouble(Neighbor::score).reversed().thenComparingInt(Neighbor::id));
    if (out.size() > count) out = new ArrayList<>(out.subList(0, count));
    double best = out.isEmpty() ? 0 : out.get(0).score;
    out.removeIf(x -> x.score < .05 * best);
    return out;
  }

  static boolean usable(Camera camera) {
    return camera.registered && !camera.denseExcluded;
  }

  private void annotateNeighborSelection(int camera, Map<String, Object> report) {
    if (o.mvsgmOptimalNeighbors)
      report.put("neighbor_selection", neighborSelectionReports.get(camera));
  }

  /** The inverse-depth labels of a reference camera: q(l) = low + l * step. */
  record Labels(double low, double step, int count, double sparseLow, double sparseHigh, int samples) {
    double q(double label) {
      return low + label * step;
    }
  }

  /**
   * Inverse-depth range from the sparse points the camera sees (as DisparityRange.fromEvidence
   * does for pairs): 1% and 99% quantiles, widened to credible tails (points seen in 3+ views),
   * plus a margin of max(16 labels, 2% of the largest inverse depth). The step moves a pixel by
   * {@code labelPixels} in the neighbour with the largest image motion per unit q.
   */
  Labels labels(ImageFrame ref, List<FeatureTracks.Track> tracks, List<ImageFrame> frames, double[][] shifts) {
    Camera cam = ref.camera;
    List<double[]> q = new ArrayList<>();
    for (FeatureTracks.Track t : tracks) {
      if (t.point == null) continue;
      double[] v = cam.view(t.point);
      if (v[2] <= 1e-9) continue;
      double u = cam.fx * v[0] / v[2] + cam.cx, w = cam.fy * v[1] / v[2] + cam.cy;
      if (u < 0 || w < 0 || u >= ref.width || w >= ref.height) continue;
      int views = 0;
      for (var ob : t.observations) if (ob.image < frames.size() && usable(frames.get(ob.image).camera)) views++;
      q.add(new double[] {1 / v[2], views >= 3 ? 1 : 0});
    }
    if (q.size() < 12)
      throw new IllegalArgumentException(
          "mvsgm needs at least 12 sparse points visible in image " + ref.id);
    return labels(q, ref.width, ref.height, shifts);
  }

  /** Labels from inverse-depth samples {q, anchor (1 = 3+ views)} of an image of the given size. */
  Labels labels(List<double[]> q, int width, int height, double[][] shifts) {
    q.sort(Comparator.comparingDouble(a -> a[0]));
    double low = q.get((int) Math.floor(.01 * (q.size() - 1)))[0];
    double high = q.get((int) Math.ceil(.99 * (q.size() - 1)))[0];
    for (double[] s : q)
      if (s[1] > 0) {
        low = Math.min(low, s[0]);
        high = Math.max(high, s[0]);
      }
    // Largest pixel motion per unit q over the neighbours, at the image corners and centre.
    double motion = 0;
    for (double[] t : shifts) {
      double[] cx = {0, width - 1, 0, width - 1, width / 2.0};
      double[] cy = {0, 0, height - 1, height - 1, height / 2.0};
      for (int i = 0; i < cx.length; i++) {
        double qm = .5 * (low + high), d = 1 + qm * t[2];
        double mx = (t[0] - cx[i] * t[2]) / (d * d), my = (t[1] - cy[i] * t[2]) / (d * d);
        motion = Math.max(motion, Math.hypot(mx, my));
      }
    }
    if (!(motion > 0)) throw new IllegalArgumentException("mvsgm: no baseline between the neighbours");
    double step = labelPixels / motion;
    double margin = Math.max(16 * step, .02 * high);
    double lo = Math.max(step, low - margin), hi = high + margin;
    int count = (int) Math.ceil((hi - lo) / step) + 1;
    if (count > maxLabels) {
      step = (hi - lo) / (maxLabels - 1);
      count = maxLabels;
    }
    return new Labels(lo, step, count, low, high, q.size());
  }

  // ---------------------------------------------------------------------------------------------
  // Images

  /** A reference image: census, textured-and-valid mask. */
  static final class Reference {
    final int w, h, window;
    final byte[] gray, valid;
    volatile long[] census;
    final boolean deferredCensus;
    final boolean[] use;

    Reference(int w, int h, byte[] gray, byte[] valid, int window) {
      this(w, h, gray, valid, window, false);
    }

    Reference(int w, int h, byte[] gray, byte[] valid, int window, boolean defer) {
      this.w = w;
      this.h = h;
      this.window = window;
      deferredCensus = defer;
      this.gray = gray;
      this.valid = valid;
      if (!defer) census = CensusSgm.census(gray, w, h, window)[0];
      boolean[] textured = CensusSgm.textureMask(gray, w, h, window);
      boolean[] whole = windowValid(valid, w, h, window / 2);
      use = new boolean[w * h];
      for (int p = 0; p < use.length; p++) use[p] = textured[p] && whole[p];
    }

    synchronized long[] cpuCensus() {
      if (census == null) census = CensusSgm.census(gray, w, h, window)[0];
      return census;
    }
  }

  /**
   * A neighbour resampled into the reference camera's rotation and intrinsics, at the neighbour's
   * centre, on a canvas whose pixel (0,0) is virtual pixel (x0, y0). shift = K_c t'.
   */
  static final class Warped {
    final int id, x0, y0, w, h, window;
    final double[] shift;
    volatile long[] census;
    final boolean deferredCensus;
    final boolean[] valid;
    // Resampled gray and its validity before the census-window erosion (for coarser levels).
    final byte[] gray, raw;

    Warped(int id, int x0, int y0, int w, int h, double[] shift, long[] census, boolean[] valid, byte[] gray, byte[] raw) {
      this(id, x0, y0, w, h, shift, census, valid, gray, raw, 5, false);
    }

    Warped(int id, int x0, int y0, int w, int h, double[] shift, long[] census, boolean[] valid,
        byte[] gray, byte[] raw, int window, boolean defer) {
      this.id = id;
      this.x0 = x0;
      this.y0 = y0;
      this.w = w;
      this.h = h;
      this.shift = shift;
      this.census = census;
      this.valid = valid;
      this.gray = gray;
      this.raw = raw;
      this.window = window;
      deferredCensus = defer;
    }

    synchronized long[] cpuCensus() {
      if (census == null) census = CensusSgm.census(gray, w, h, window)[0];
      return census;
    }

    /** The 2x2-downsampled canvas (TsgmSearch.downsample), census recomputed once. */
    Warped coarser(int window) {
      int lw = w / 2, lh = h / 2;
      byte[] g = new byte[lw * lh], v = new byte[lw * lh];
      TsgmSearch.downsample(gray, raw, w, lw, lh, g, v);
      return new Warped(id, x0, y0, lw, lh, shift,
          deferredCensus ? null : CensusSgm.census(g, lw, lh, window)[0],
          windowValid(v, lw, lh, window / 2), g, v, window, deferredCensus);
    }
  }

  /** K_c t' with t' = R_c (C_c - C_j): the virtual neighbour's homogeneous shift per unit q. */
  static double[] shift(Camera ref, Camera other) {
    double[] t = Math3.mv(ref.rotation, Math3.sub(ref.center(), other.center()));
    return new double[] {ref.fx * t[0] + ref.cx * t[2], ref.fy * t[1] + ref.cy * t[2], t[2]};
  }

  static Warped warp(
      Camera ref, int rw, int rh, Labels labels, int id, Camera other, byte[] gray, byte[] mask, int ow, int oh, int window) {
    return warp(ref, rw, rh, labels, id, other, gray, mask, ow, oh, window, false);
  }

  static Warped warp(
      Camera ref, int rw, int rh, Labels labels, int id, Camera other, byte[] gray, byte[] mask,
      int ow, int oh, int window, boolean defer) {
    int[] canvas = canvas(ref, other, 0, 0, rw, rh, labels, window);
    if ((long) canvas[2] * canvas[3] > 4L * rw * rh)
      throw new IllegalArgumentException("mvsgm: neighbour canvas too large for image ");
    return warp(ref, other, id, canvas, gray, mask, 0, 0, ow, oh, window, defer);
  }

  /**
   * The canvas {x0, y0, w, h} (virtual pixels) that reference pixels [u0, u0 + rw) x [v0, v0 + rh)
   * can reach within the label range, with a census border.
   */
  static int[] canvas(Camera ref, Camera other, int u0, int v0, int rw, int rh, Labels labels, int window) {
    double[] s = shift(ref, other);
    double minX = Double.POSITIVE_INFINITY, minY = minX, maxX = -minX, maxY = -minX;
    for (double q : new double[] {labels.q(0), labels.q(labels.count() - 1)})
      for (double u : new double[] {u0, u0 + rw - 1})
        for (double v : new double[] {v0, v0 + rh - 1}) {
          double d = 1 + q * s[2];
          double x = (u + q * s[0]) / d, y = (v + q * s[1]) / d;
          minX = Math.min(minX, x);
          maxX = Math.max(maxX, x);
          minY = Math.min(minY, y);
          maxY = Math.max(maxY, y);
        }
    int r = window / 2 + 2;
    int x0 = (int) Math.floor(minX) - r, y0 = (int) Math.floor(minY) - r;
    return new int[] {x0, y0, (int) Math.ceil(maxX) + r - x0 + 1, (int) Math.ceil(maxY) + r - y0 + 1};
  }

  /** Virtual (reference-rotation) pixel -> neighbour pixel: K_j R_j R_c^T K_c^-1. */
  static double[] homography(Camera ref, Camera other) {
    double[] kinv = {1 / ref.fx, 0, -ref.cx / ref.fx, 0, 1 / ref.fy, -ref.cy / ref.fy, 0, 0, 1};
    double[] kj = {other.fx, 0, other.cx, 0, other.fy, other.cy, 0, 0, 1};
    return Math3.mul(kj, Math3.mul(Math3.mul(other.rotation, Math3.transpose(ref.rotation)), kinv));
  }

  /**
   * The neighbour-image rectangle {x0, y0, w, h} a canvas samples (bilinear, one-pixel margin),
   * clipped to the image; null if the canvas maps behind the neighbour or outside it.
   */
  static int[] sourceRegion(double[] hm, int[] canvas, int ow, int oh) {
    double minX = Double.POSITIVE_INFINITY, minY = minX, maxX = -minX, maxY = -minX;
    for (int e = 0; e <= 32; e++)
      for (int side = 0; side < 4; side++) {
        double t = e / 32.0;
        double vx = canvas[0] + (side == 0 ? t * (canvas[2] - 1) : side == 1 ? canvas[2] - 1 : side == 2 ? (1 - t) * (canvas[2] - 1) : 0);
        double vy = canvas[1] + (side == 0 ? 0 : side == 1 ? t * (canvas[3] - 1) : side == 2 ? canvas[3] - 1 : (1 - t) * (canvas[3] - 1));
        double hz = hm[6] * vx + hm[7] * vy + hm[8];
        if (hz <= 1e-12) return new int[] {0, 0, ow, oh};
        double sx = (hm[0] * vx + hm[1] * vy + hm[2]) / hz, sy = (hm[3] * vx + hm[4] * vy + hm[5]) / hz;
        minX = Math.min(minX, sx);
        maxX = Math.max(maxX, sx);
        minY = Math.min(minY, sy);
        maxY = Math.max(maxY, sy);
      }
    int x0 = Math.max(0, (int) Math.floor(minX) - 2), y0 = Math.max(0, (int) Math.floor(minY) - 2);
    int x1 = Math.min(ow, (int) Math.ceil(maxX) + 3), y1 = Math.min(oh, (int) Math.ceil(maxY) + 3);
    if (x1 - x0 < 2 || y1 - y0 < 2) return null;
    return new int[] {x0, y0, x1 - x0, y1 - y0};
  }

  /**
   * Resamples the neighbour into the canvas from a region of its image: gray/mask hold rows
   * [gy0, gy0 + gh) x columns [gx0, gx0 + gw) of the processed image.
   */
  static Warped warp(
      Camera ref, Camera other, int id, int[] canvas, byte[] gray, byte[] mask, int gx0, int gy0, int gw, int gh, int window) {
    return warp(ref, other, id, canvas, gray, mask, gx0, gy0, gw, gh, window, false);
  }

  static Warped warp(
      Camera ref, Camera other, int id, int[] canvas, byte[] gray, byte[] mask,
      int gx0, int gy0, int gw, int gh, int window, boolean defer) {
    double[] s = shift(ref, other);
    int x0 = canvas[0], y0 = canvas[1], w = canvas[2], h = canvas[3];
    double[] hm = homography(ref, other);
    byte[] out = new byte[w * h], valid = new byte[w * h];
    IntStream.range(0, h).parallel().forEach(yy -> {
      double vy = yy + y0;
      for (int xx = 0; xx < w; xx++) {
        double vx = xx + x0;
        double hz = hm[6] * vx + hm[7] * vy + hm[8];
        if (hz <= 1e-12) continue;
        double sx = (hm[0] * vx + hm[1] * vy + hm[2]) / hz - gx0, sy = (hm[3] * vx + hm[4] * vy + hm[5]) / hz - gy0;
        int ix = (int) Math.floor(sx), iy = (int) Math.floor(sy);
        if (ix < 0 || iy < 0 || ix + 1 >= gw || iy + 1 >= gh) continue;
        int p = iy * gw + ix;
        if (mask[p] == 0 || mask[p + 1] == 0 || mask[p + gw] == 0 || mask[p + gw + 1] == 0) continue;
        double fx = sx - ix, fy = sy - iy;
        double g =
            (1 - fy) * ((1 - fx) * (gray[p] & 255) + fx * (gray[p + 1] & 255))
                + fy * ((1 - fx) * (gray[p + gw] & 255) + fx * (gray[p + gw + 1] & 255));
        out[yy * w + xx] = (byte) Math.min(255, (int) Math.round(g));
        valid[yy * w + xx] = 1;
      }
    });
    long[] c = defer ? null : CensusSgm.census(out, w, h, window)[0];
    boolean[] whole = windowValid(valid, w, h, window / 2);
    return new Warped(id, x0, y0, w, h, s, c, whole, out, valid, window, defer);
  }

  /** Pixels whose whole (2r+1)^2 window is valid (separable erosion). */
  static boolean[] windowValid(byte[] valid, int w, int h, int r) {
    int[] row = new int[w * h];
    IntStream.range(0, h).parallel().forEach(y -> {
      int run = 0;
      // Horizontal: count of consecutive valid pixels ending at x, then window check.
      int[] ok = new int[w];
      for (int x = 0; x < w; x++) {
        run = valid[y * w + x] != 0 ? run + 1 : 0;
        ok[x] = run;
      }
      for (int x = 0; x < w; x++) row[y * w + x] = x + r < w && ok[x + r] >= 2 * r + 1 ? 1 : 0;
    });
    boolean[] out = new boolean[w * h];
    IntStream.range(0, w).parallel().forEach(x -> {
      int run = 0;
      int[] ok = new int[h];
      for (int y = 0; y < h; y++) {
        run = row[y * w + x] != 0 ? run + 1 : 0;
        ok[y] = run;
      }
      for (int y = 0; y < h; y++) out[y * w + x] = y + r < h && ok[y + r] >= 2 * r + 1;
    });
    return out;
  }

  // ---------------------------------------------------------------------------------------------
  // Cost (shared integer definition of the CPU reference and the GPU kernel)

  /** Cost units: mean best-k Hamming distance times COST_SCALE (at most 8 * 48 = 384 < 512). */
  static final int COST_SCALE = 8, INVALID = 512;

  /**
   * Per label and neighbour, the float mapping of a reference pixel into the neighbour canvas:
   * canvas x = x * a + bx, canvas y = y * a + by (a = 1/(1 + q t'z); b = q K t' a - canvas origin).
   * Computed once in double, rounded to float; the kernel and the CPU reference evaluate the same
   * float products and sums (no contraction), so their costs are identical.
   */
  static float[] table(Labels labels, Warped[] views) {
    return table(labels, views, 1, labels.count());
  }

  /**
   * The table of a pyramid level with downsampling factor f (level label j = full label j f): a
   * level pixel x_l is full pixel (x_l + .5) f - .5, and a full canvas position c is level canvas
   * position (c + .5) / f - .5, so b_l = .5 a - .5 a / f + b / f + .5 / f - .5.
   */
  static float[] table(Labels labels, Warped[] views, int f, int count) {
    return table(labels, views, f, count, 0, 0);
  }

  /** As above for a tile whose pixel (0, 0) is reference pixel (tx, ty). */
  static float[] table(Labels labels, Warped[] views, int f, int count, int tx, int ty) {
    int V = views.length;
    float[] t = new float[3 * count * V];
    for (int l = 0; l < count; l++) {
      double q = labels.q((double) l * f);
      for (int v = 0; v < V; v++) {
        double[] s = views[v].shift;
        double a = 1 / (1 + q * s[2]);
        double bx = q * s[0] * a - views[v].x0, by = q * s[1] * a - views[v].y0;
        if (tx != 0 || ty != 0) {
          bx += tx * a;
          by += ty * a;
        }
        if (f > 1) {
          double c = .5 * a - .5 * a / f + .5 / f - .5;
          bx = bx / f + c;
          by = by / f + c;
        }
        int at = 3 * (l * V + v);
        t[at] = (float) a;
        t[at + 1] = (float) bx;
        t[at + 2] = (float) by;
      }
    }
    return t;
  }

  /** Half the census bits, the cost of a neighbour that cannot see the pixel. */
  int penalty() {
    return COST_SCALE * (census * census - 1) / 2;
  }

  /**
   * Cost of label l at reference pixel (x, y): per neighbour, the bilinear interpolation of the four
   * Hamming distances around the mapped position (fixed point, rounded half up), then the mean of
   * the k smallest (missing neighbours count {@link #penalty()}; INVALID if none sees it).
   */
  static int cost(long c, Warped[] views, float[] table, int l, int x, int y, int k, int penalty, int[] each) {
    int V = views.length, valid = 0;
    float fxp = x, fyp = y;
    for (int v = 0; v < V; v++) {
      Warped nb = views[v];
      int at = 3 * (l * V + v);
      float a = table[at];
      float sx = fxp * a + table[at + 1], sy = fyp * a + table[at + 2];
      // Inside [0, size - 1) before any conversion (an out-of-range float -> int is undefined in
      // OpenCL); then floor + 1 is inside the canvas.
      if (!(sx >= 0f) || !(sy >= 0f) || !(sx < (float) (nb.w - 1)) || !(sy < (float) (nb.h - 1))) continue;
      float flx = (float) Math.floor(sx), fly = (float) Math.floor(sy);
      int ix = (int) flx, iy = (int) fly;
      int b = iy * nb.w + ix;
      if (!nb.valid[b] || !nb.valid[b + 1] || !nb.valid[b + nb.w] || !nb.valid[b + nb.w + 1]) continue;
      float fx = sx - flx, fy = sy - fly;
      long[] nc = nb.census;
      if (nc == null) nc = nb.cpuCensus();
      float h00 = Long.bitCount(c ^ nc[b]), h10 = Long.bitCount(c ^ nc[b + 1]);
      float h01 = Long.bitCount(c ^ nc[b + nb.w]), h11 = Long.bitCount(c ^ nc[b + nb.w + 1]);
      float top = (1f - fx) * h00 + fx * h10, bottom = (1f - fx) * h01 + fx * h11;
      float hv = (1f - fy) * top + fy * bottom;
      int h = (int) Math.floor(hv * COST_SCALE + .5f);
      int i = valid++;
      while (i > 0 && each[i - 1] > h) {
        each[i] = each[i - 1];
        i--;
      }
      each[i] = h;
    }
    if (valid == 0) return INVALID;
    int kk = Math.min(k, V), sum = 0;
    for (int i = 0; i < kk; i++) sum += i < valid ? each[i] : penalty;
    return (sum + kk / 2) / kk;
  }

  // ---------------------------------------------------------------------------------------------
  // One camera's matching problem

  /** Everything a camera's solve needs; built on the CPU, uploaded once to the GPU. */
  static final class Plan {
    final int camera;
    final List<Neighbor> neighbours;
    final Labels labels;
    final Reference ref;
    final Warped[] views;
    final float[] table;
    // Pyramid level: downsampling factor and label count (level label j = full label j * factor).
    final int factor, count;
    // Tile origin in the reference image (0, 0 for a whole image).
    final int tx, ty;

    Plan(int camera, List<Neighbor> neighbours, Labels labels, Reference ref, Warped[] views) {
      this(camera, neighbours, labels, ref, views, 1, labels.count(), table(labels, views), 0, 0);
    }

    /** One tile of a large image: ref/views cover the tile; (tx, ty) is its origin. */
    Plan(int camera, List<Neighbor> neighbours, Labels labels, Reference ref, Warped[] views, int tx, int ty) {
      this(camera, neighbours, labels, ref, views, 1, labels.count(), table(labels, views, 1, labels.count(), tx, ty), tx, ty);
    }

    /** Neighbours and labels only (a tiled image builds per-tile plans later). */
    static Plan header(int camera, List<Neighbor> neighbours, Labels labels) {
      return new Plan(camera, neighbours, labels, null, null, 1, labels.count(), null, 0, 0);
    }

    boolean tiled() {
      return ref == null;
    }

    private Plan(int camera, List<Neighbor> neighbours, Labels labels, Reference ref, Warped[] views,
        int factor, int count, float[] table, int tx, int ty) {
      this.camera = camera;
      this.neighbours = neighbours;
      this.labels = labels;
      this.ref = ref;
      this.views = views;
      this.factor = factor;
      this.count = count;
      this.table = table;
      this.tx = tx;
      this.ty = ty;
    }

    /**
     * The next coarser level: reference and canvases 2x2-downsampled (census recomputed), every
     * second label. The table is computed from the full-resolution geometry (fullViews).
     */
    Plan coarser(int window, Warped[] fullViews) {
      int lw = ref.w / 2, lh = ref.h / 2, f = factor * 2;
      byte[] g = new byte[lw * lh], v = new byte[lw * lh];
      TsgmSearch.downsample(ref.gray, ref.valid, ref.w, lw, lh, g, v);
      Reference r = new Reference(lw, lh, g, v, window, ref.deferredCensus);
      Warped[] vs = new Warped[views.length];
      for (int i = 0; i < vs.length; i++) vs[i] = views[i].coarser(window);
      int c = (labels.count() - 1) / f + 1;
      return new Plan(camera, neighbours, labels, r, vs, f, c, table(labels, fullViews, f, c, tx, ty), tx, ty);
    }

    long canvasPixels() {
      long n = 0;
      for (Warped v : views) n += (long) v.w * v.h;
      return n;
    }
  }

  /** Why a camera got no plan (no depth map); reported per camera. */
  final Map<Integer, String> skipped = new ConcurrentHashMap<>();

  Plan plan(List<ImageFrame> frames, List<FeatureTracks.Track> tracks, int c) throws IOException {
    Plan header = headerPlan(frames, tracks, c);
    return header == null ? null : preparePlan(frames, header);
  }

  /** Geometry only: decides map existence and labels before any raster or temporary map exists. */
  Plan headerPlan(List<ImageFrame> frames, List<FeatureTracks.Track> tracks, int c) {
    ImageFrame rf = frames.get(c);
    List<Neighbor> chosen;
    if (o.mvsgmOptimalNeighbors) {
      var selection = OptimalMvsgmNeighbors.select(frames, tracks, c, neighbors,
          Math.max(k, o.mvsgmConsistentViews));
      chosen = selection.neighbors();
      neighborSelectionReports.put(c, selection.report());
    } else chosen = neighbours(frames, tracks, c, neighbors);
    if (chosen.isEmpty()) {
      skipped.put(c, "no neighbour camera shares at least 12 sparse tie points with it at a usable angle");
      return null;
    }
    double[][] shifts = new double[chosen.size()][];
    for (int i = 0; i < shifts.length; i++) shifts[i] = shift(rf.camera, frames.get(chosen.get(i).id()).camera);
    Labels labels;
    try {
      labels = labels(rf, tracks, frames, shifts);
    } catch (IllegalArgumentException e) {
      skipped.put(c, e.getMessage());
      return null;
    }
    return Plan.header(c, chosen, labels);
  }

  Plan preparePlan(List<ImageFrame> frames, Plan header) throws IOException {
    int c = header.camera;
    ImageFrame rf = frames.get(c);
    List<Neighbor> chosen = header.neighbours;
    Labels labels = header.labels;
    if (tileSize(rf) > 0) return header;
    byte[][] r = rasters(rf);
    boolean defer = GpuMvsgmCensus.enabled(o);
    Reference ref = new Reference(rf.width, rf.height, r[0], r[1], census, defer);
    Warped[] views = new Warped[chosen.size()];
    for (int i = 0; i < views.length; i++) {
      ImageFrame of = frames.get(chosen.get(i).id());
      byte[][] g = rasters(of);
      views[i] = warp(rf.camera, rf.width, rf.height, labels, of.id, of.camera, g[0], g[1], of.width, of.height, census, defer);
    }
    return new Plan(c, chosen, labels, ref, views);
  }

  /** SGM penalties in cost units: --p1/--p2 are 5x5-census units, scaled to this census. */
  int p1() {
    String forced = System.getProperty("sgm.mvsgm.p1");
    double v = forced != null ? Double.parseDouble(forced) : o.p1 * (census * census - 1) / 24.0;
    return Math.max(1, (int) Math.round(v * COST_SCALE));
  }

  int p2() {
    String forced = System.getProperty("sgm.mvsgm.p2");
    double v = forced != null ? Double.parseDouble(forced) : o.p2 * (census * census - 1) / 24.0;
    return Math.max(p1() + 1, (int) Math.round(v * COST_SCALE));
  }

  /** Aggregate bytes: 16 bit when directions * (512 + P2) fits, as for pair SGM. */
  int sumBytes() {
    return o.directions * (512L + p2()) <= 65535 ? 2 : 4;
  }

  /** Packed costs of the bands (CPU reference). */
  short[] costs(Plan plan, DisparityBands bands) {
    Reference ref = plan.ref;
    long[] referenceCensus = ref.cpuCensus();
    for (Warped view : plan.views) view.cpuCensus();
    int w = ref.w, h = ref.h;
    int[] offset = bands.offsets();
    short[] cost = new short[(int) bands.total()];
    int pen = penalty();
    parallel(h, y -> {
      int[] each = new int[plan.views.length + 1];
      for (int x = 0; x < w; x++) {
        int p = y * w + x, at = offset[p];
        for (int j = 0; j < bands.count[p]; j++)
          cost[at + j] =
              (short) (ref.use[p] ? cost(referenceCensus[p], plan.views, plan.table, bands.low[p] + j, x, y, k, pen, each) : INVALID);
      }
    });
    return cost;
  }

  /**
   * Integer reference of the GPU solve: packed costs, the continuous 4/8-path recurrence of
   * BandedSgm.cpuSummary (same order and arithmetic as GpuBandedStereo's bpaths), winners.
   * Summary layout per pixel: best label, value, second (labels more than 1 away), before, after,
   * flags (1 = best on the lower band edge inside the range, 2 = on the upper; 4 = range edge).
   */
  int[] cpuSummary(Plan plan, DisparityBands bands) {
    Reference ref = plan.ref;
    int w = ref.w, h = ref.h, n = w * h, L = plan.count;
    int[] offset = bands.offsets(), low = bands.low, count = bands.count;
    long total = bands.total();
    if (total * 6 + 24L * n > (long) (Runtime.getRuntime().maxMemory() * .6))
      throw new IllegalArgumentException("mvsgm CPU SGM needs " + (total * 6 >> 20) + " MiB of heap");
    short[] cost = costs(plan, bands);
    int[] sum = new int[(int) total];
    int p1 = p1(), p2 = p2();
    int[][] dirs = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, 1}, {1, -1}, {-1, -1}};
    int maxRow = 0;
    for (int y = 0; y < h; y++) {
      long cells = (y + 1 < h ? offset[(y + 1) * w] : total) - offset[y * w];
      maxRow = (int) Math.max(maxRow, cells);
    }
    boolean[] ta = ref.use;
    for (int path = 0; path < o.directions; path++) {
      int dx = dirs[path][0], dy = dirs[path][1];
      int[] previous = new int[maxRow], row = new int[maxRow];
      int[] previousMin = new int[w], rowMin = new int[w];
      int yStart = dy < 0 ? h - 1 : 0, yStep = dy < 0 ? -1 : 1;
      int xStart = dx < 0 ? w - 1 : 0, xStep = dx < 0 ? -1 : 1;
      for (int yi = 0, y = yStart; yi < h; yi++, y += yStep) {
        int rowBase = offset[y * w], prevRowBase = dy == 0 ? rowBase : (y - dy >= 0 && y - dy < h ? offset[(y - dy) * w] : 0);
        for (int xi = 0, x = xStart; xi < w; xi++, x += xStep) {
          int p = y * w + x, px = x - dx, py = y - dy, lo = low[p], cnt = count[p], at = offset[p];
          int q = py * w + px;
          boolean has = px >= 0 && px < w && py >= 0 && py < h && ta[q] && count[q] > 0;
          int[] prev = dy == 0 ? row : previous;
          int plo = has ? low[q] : 0, pn = has ? count[q] : 0;
          int prevBase = has ? offset[q] - prevRowBase : 0;
          int minPrev = has ? (dy == 0 ? rowMin : previousMin)[px] : 0;
          int localP2 = has ? StereoCost.pathP2(p1, p2, o.sgmEdgeScale, ref.gray, p, q) : p2;
          int currentMin = Integer.MAX_VALUE, dest = at - rowBase;
          for (int j = 0; j < cnt; j++) {
            int value = cost[at + j];
            if (has) {
              int kk = lo + j - plo, t = minPrev + localP2;
              if (kk >= 0 && kk < pn) t = Math.min(t, prev[prevBase + kk]);
              if (kk - 1 >= 0 && kk - 1 < pn) t = Math.min(t, prev[prevBase + kk - 1] + p1);
              if (kk + 1 >= 0 && kk + 1 < pn) t = Math.min(t, prev[prevBase + kk + 1] + p1);
              value += t - minPrev;
            }
            row[dest + j] = value;
            currentMin = Math.min(currentMin, value);
            sum[at + j] += value;
          }
          rowMin[x] = currentMin;
        }
        int[] swap = previous;
        previous = row;
        row = swap;
        swap = previousMin;
        previousMin = rowMin;
        rowMin = swap;
      }
    }
    int[] summary = new int[n * 6];
    for (int p = 0; p < n; p++) {
      int lo = low[p], cnt = count[p], at = offset[p];
      int best = -1, value = Integer.MAX_VALUE, second = Integer.MAX_VALUE, before = -1, after = -1;
      for (int j = 0; j < cnt; j++)
        if (cost[at + j] != INVALID && sum[at + j] < value) {
          best = j;
          value = sum[at + j];
        }
      if (best >= 0) {
        for (int j = 0; j < cnt; j++)
          if (Math.abs(j - best) > 1 && cost[at + j] != INVALID) second = Math.min(second, sum[at + j]);
        if (best > 0 && cost[at + best - 1] != INVALID) before = sum[at + best - 1];
        if (best + 1 < cnt && cost[at + best + 1] != INVALID) after = sum[at + best + 1];
      }
      int s = p * 6;
      summary[s] = best < 0 ? -1 : lo + best;
      summary[s + 1] = value;
      summary[s + 2] = second;
      summary[s + 3] = before;
      summary[s + 4] = after;
      summary[s + 5] = flags(best, lo, cnt, L);
    }
    return summary;
  }

  /** Winner flags; identical in the kernel. */
  static int flags(int best, int lo, int cnt, int labels) {
    if (best < 0) return 0;
    int label = lo + best;
    return (best == 0 && lo > 0 ? 1 : 0)
        | (best == cnt - 1 && lo + cnt < labels ? 2 : 0)
        | (label == 0 || label == labels - 1 ? 4 : 0);
  }

  // ---------------------------------------------------------------------------------------------
  // Solve: GPU with CPU fallback and a first-use self-check

  // First-use checks per process: one for a solve with full bands, one with narrowed bands.
  static volatile boolean gpuVerifiedFull, gpuVerifiedBanded, gpuDisabled;
  String lastBackend = "cpu";

  int[] summary(Plan plan, DisparityBands bands, GpuMultiViewSgm.Session[] session) {
    if (!gpuDisabled && GpuMultiViewSgm.use(o, this, plan, bands)) {
      try {
        if (session[0] == null || session[0].plan != plan)
          throw new IllegalStateException("mvsgm: session must be opened for this plan");
        boolean full = bands.total() == count(plan.ref.use) * (long) plan.count;
        if (full ? !gpuVerifiedFull : !gpuVerifiedBanded) selfCheck(plan, session[0], bands, full);
        if (!gpuDisabled) {
          int[] s = session[0].summary(bands);
          lastBackend = "gpu";
          return s;
        }
      } catch (RuntimeException e) {
        if (o.compute.equals("gpu")) throw new IllegalArgumentException("GPU execution required: " + e.getMessage(), e);
        System.out.println("mvsgm: GPU solve failed (" + e.getMessage() + "); using the CPU reference for image " + plan.camera);
      }
    }
    lastBackend = "cpu";
    return cpuSummary(plan, bands);
  }

  /**
   * First GPU solve of each kind (full bands, narrowed bands) in a process: the same bands on a
   * 96-row crop of the image are solved on the GPU and by the CPU reference; every summary integer
   * must agree, otherwise the GPU path is disabled for the process and the CPU reference is used.
   */
  void selfCheck(Plan plan, GpuMultiViewSgm.Session session, DisparityBands bands, boolean full) {
    synchronized (MultiViewSgm.class) {
      if ((full ? gpuVerifiedFull : gpuVerifiedBanded) || gpuDisabled) return;
      Reference ref = plan.ref;
      int y0 = Math.max(0, ref.h / 2 - 48), y1 = Math.min(ref.h, y0 + 96);
      DisparityBands crop = new DisparityBands(ref.w, ref.h, plan.count);
      for (int p = y0 * ref.w; p < y1 * ref.w; p++) {
        crop.low[p] = bands.low[p];
        crop.count[p] = bands.count[p];
      }
      int[] gpu = session.summary(crop).clone(), cpu = cpuSummary(plan, crop);
      if (Arrays.equals(gpu, cpu)) {
        if (full) gpuVerifiedFull = true;
        else gpuVerifiedBanded = true;
        System.out.printf(Locale.ROOT, "mvsgm GPU: %dx%d-row %s check (%,d cells) identical to the CPU reference%n",
            ref.w, y1 - y0, full ? "full-band" : "narrow-band", crop.total());
      } else {
        gpuDisabled = true;
        int differ = 0;
        for (int i = 0; i < gpu.length; i++) if (gpu[i] != cpu[i]) differ++;
        System.out.println("WARNING: mvsgm GPU solve differs from the CPU reference (" + differ
            + " summary values); using the CPU for the rest of this process");
      }
    }
  }

  // ---------------------------------------------------------------------------------------------
  // Coarse-to-fine banding (stage 3; the TSGM idea in inverse depth)

  /** Pyramid levels below full resolution: -Dsgm.mvsgm.levels, default --tsgm-levels (3). */
  int levels() {
    return Integer.getInteger("sgm.mvsgm.levels", o.mvsgmLevels >= 0 ? o.mvsgmLevels : o.tsgmLevels);
  }

  record Solved(int[] summary, DisparityBands bands, List<Object> levels, long peakDeviceBytes) {}

  /** One level's solve on the GPU (own session, released afterwards) or the CPU reference. */
  int[] solveLevel(Plan plan, DisparityBands bands, long[] peak) {
    GpuMultiViewSgm.Session[] session = new GpuMultiViewSgm.Session[1];
    try {
      return solveLevel(plan, bands, peak, session);
    } finally {
      if (session[0] != null) session[0].close();
    }
  }

  /** The finest retry retains input buffers when its larger reservation can be acquired now. */
  private int[] solveLevel(Plan plan, DisparityBands bands, long[] peak, GpuMultiViewSgm.Session[] session) {
    if (!gpuDisabled && GpuMultiViewSgm.use(o, this, plan, bands)) {
      try {
        long bytes = GpuMultiViewSgm.deviceBytes(this, plan, bands.total());
        if (session[0] != null && (session[0].plan != plan || !session[0].reserve(bytes))) {
          session[0].close();
          session[0] = null;
        }
        if (session[0] == null) session[0] = new GpuMultiViewSgm.Session(o, this, plan, bytes);
        peak[0] = Math.max(peak[0], bytes);
      } catch (RuntimeException e) {
        if (o.compute.equals("gpu")) throw e;
        if (session[0] != null) {
          session[0].close();
          session[0] = null;
        }
        System.out.println("mvsgm: GPU session failed (" + e.getMessage() + "); CPU reference for image " + plan.camera);
      }
    } else if (session[0] != null) {
      session[0].close();
      session[0] = null;
    }
    if (session[0] == null) {
      if (GpuMvsgmCensus.enabled(o) && plan.ref.census == null) GpuMvsgmCensus.memoryFallback();
      lastBackend = "cpu";
      return cpuSummary(plan, bands);
    }
    return summary(plan, bands, session).clone();
  }

  /**
   * Coarsest level: every label; finer levels: per-pixel bands from the parent level's unique
   * winners (TsgmSearch.derive: 3x3 parent min/max doubled, margin --tsgm-margin, doubled at depth
   * edges, then 9/33/129-wide parent windows, then the full range), widened around sparse points.
   * At full resolution, winners on a band edge (not a range edge) widen their band and a 1-pixel
   * halo once and the image is solved again; still on a band edge afterwards = rejected.
   */
  Solved solve(Plan full, double[] seeds) {
    List<Plan> plans = new ArrayList<>(List.of(full));
    for (int l = 0; l < levels(); l++) {
      Plan last = plans.get(plans.size() - 1);
      if (Math.min(last.ref.w / 2, last.ref.h / 2) < 48 || (last.labels.count() - 1) / (last.factor * 2) + 1 < 8) break;
      plans.add(last.coarser(census, full.views));
    }
    long[] peak = new long[1];
    List<Object> report = new ArrayList<>();
    float[] parent = null;
    int[] summary = null;
    DisparityBands bands = null;
    for (int l = plans.size() - 1; l >= 0; l--) {
      long started = System.nanoTime();
      Plan p = plans.get(l);
      long fallback = 0, seeded = 0, retried = 0;
      if (parent == null) bands = DisparityBands.full(p.ref.w, p.ref.h, p.count, p.ref.use);
      else {
        Plan up = plans.get(l + 1);
        bands = new DisparityBands(p.ref.w, p.ref.h, p.count);
        fallback = TsgmSearch.derive(o, parent, up.ref.w, up.ref.h, bands, p.ref.use, 0, 1, false);
        seeded = seed(bands, seeds, p.factor, p.ref.use);
        bands.changed();
      }
      GpuMultiViewSgm.Session[] session = new GpuMultiViewSgm.Session[1];
      try {
        summary = solveLevel(p, bands, peak, session);
        if (l == 0 && parent != null) {
          long[] touched = new long[1];
          DisparityBands widened = widen(bands, summary, touched);
          if (widened != null) {
            retried = touched[0];
            bands = widened;
            summary = solveLevel(p, bands, peak, session);
          }
        }
      } finally {
        if (session[0] != null) session[0].close();
      }
      report.add(Json.object(
          "factor", p.factor, "width", p.ref.w, "height", p.ref.h, "labels", p.count,
          "packed_cells", bands.total(), "mean_band", bands.total() / (double) Math.max(1, count(p.ref.use)),
          "full_range_fallback_pixels", fallback, "seed_widened_pixels", seeded, "band_edge_retry_pixels", retried,
          "backend", lastBackend, "seconds", (System.nanoTime() - started) / 1e9));
      if (l > 0) parent = parentMap(p, summary);
    }
    return new Solved(summary, bands, report, peak[0]);
  }

  /** A coarse level's winners usable as parents: unique, not on a range edge; NaN elsewhere. */
  float[] parentMap(Plan p, int[] summary) {
    int n = p.ref.w * p.ref.h;
    float[] out = new float[n];
    Arrays.fill(out, Float.NaN);
    for (int i = 0; i < n; i++) {
      int s = 6 * i, best = summary[s], value = summary[s + 1], second = summary[s + 2];
      if (best < 0 || (summary[s + 5] & 7) != 0) continue;
      if (second != Integer.MAX_VALUE && second - value <= Math.max(1, value * o.uniqueness)) continue;
      out[i] = best;
    }
    return out;
  }

  /** Sparse points the camera sees: x, y and full-resolution label, triples. */
  static double[] seeds(ImageFrame ref, List<FeatureTracks.Track> tracks, Labels labels) {
    Camera cam = ref.camera;
    double[] out = new double[3 * 1024];
    int n = 0;
    for (FeatureTracks.Track t : tracks) {
      if (t.point == null) continue;
      double[] v = cam.view(t.point);
      if (v[2] <= 1e-9) continue;
      double x = cam.fx * v[0] / v[2] + cam.cx, y = cam.fy * v[1] / v[2] + cam.cy;
      double label = (1 / v[2] - labels.low()) / labels.step();
      if (x < 0 || y < 0 || x >= ref.width || y >= ref.height || label < 0 || label > labels.count() - 1) continue;
      if (3 * n + 3 > out.length) out = Arrays.copyOf(out, 2 * out.length);
      out[3 * n] = x;
      out[3 * n + 1] = y;
      out[3 * n + 2] = label;
      n++;
    }
    return Arrays.copyOf(out, 3 * n);
  }

  /** Widens bands within 4 level pixels of each seed to include seed +- (margin + 1) labels. */
  long seed(DisparityBands bands, double[] seeds, int factor, boolean[] use) {
    int w = bands.width, h = bands.height, k = o.tsgmMargin, radius = 4;
    long widened = 0;
    for (int s = 0; s + 2 < seeds.length; s += 3) {
      int cx = (int) Math.round((seeds[s] + .5) / factor - .5), cy = (int) Math.round((seeds[s + 1] + .5) / factor - .5);
      double label = seeds[s + 2] / factor;
      int lo = (int) Math.floor(label) - k - 1, hi = (int) Math.ceil(label) + k + 1;
      for (int y = Math.max(0, cy - radius); y <= Math.min(h - 1, cy + radius); y++)
        for (int x = Math.max(0, cx - radius); x <= Math.min(w - 1, cx + radius); x++) {
          int p = y * w + x;
          if (!use[p] || bands.count[p] == 0) continue;
          int before = bands.count[p];
          bands.include(p, lo);
          bands.include(p, hi);
          if (bands.count[p] != before) widened++;
        }
    }
    return widened;
  }

  /**
   * Band-edge winners (flags 1/2) and a one-pixel halo widen by max(2 * margin, band) labels on
   * the offending side (as BandedSgm.widen). Null when no pixel needs it.
   */
  DisparityBands widen(DisparityBands bands, int[] summary, long[] touched) {
    int w = bands.width, h = bands.height, n = w * h, grow = Math.max(2, 2 * o.tsgmMargin);
    DisparityBands out = null;
    long widened = 0;
    for (int p = 0; p < n; p++) {
      int flags = summary[6 * p + 5];
      if ((flags & 3) == 0) continue;
      if (out == null) {
        out = new DisparityBands(w, h, bands.labels);
        System.arraycopy(bands.low, 0, out.low, 0, n);
        System.arraycopy(bands.count, 0, out.count, 0, n);
      }
      boolean lower = (flags & 1) != 0;
      int x = p % w, y = p / w;
      for (int yy = Math.max(0, y - 1); yy <= Math.min(h - 1, y + 1); yy++)
        for (int xx = Math.max(0, x - 1); xx <= Math.min(w - 1, x + 1); xx++) {
          int q = yy * w + xx;
          if (out.count[q] == 0) continue;
          int amount = Math.max(grow, bands.count[q]), before = out.count[q];
          if (lower) out.include(q, out.low[q] - amount);
          else out.include(q, out.low[q] + out.count[q] - 1 + amount);
          if (out.count[q] != before) widened++;
        }
    }
    touched[0] = widened;
    if (out != null) out.changed();
    return out;
  }

  // ---------------------------------------------------------------------------------------------
  // Large images in tiles (STREAMED_PROCESSING.md)

  /** Images above this many pixels are solved in tiles; -Dsgm.mvsgm.tile=N forces N-pixel tiles. */
  static final long TILE_THRESHOLD = 16L << 20;
  static final int TILE_SIZE = 2048, TILE_OVERLAP = 64;

  int tileSize(ImageFrame f) {
    Integer forced = Integer.getInteger("sgm.mvsgm.tile");
    if (forced != null) return Math.max(0, forced);
    return (long) f.width * f.height > TILE_THRESHOLD ? TILE_SIZE : 0;
  }

  /** Rows [y0, y0 + h) x columns [x0, x0 + w) of a frame's gray image and valid mask. */
  static byte[][] region(ImageFrame f, int x0, int y0, int w, int h) throws IOException {
    synchronized (RasterCache.class) {
      RasterCache.pair(f, f);
      org.opencv.core.Mat g = f.gray.submat(y0, y0 + h, x0, x0 + w).clone(),
          m = f.validMask.submat(y0, y0 + h, x0, x0 + w).clone();
      try {
        return new byte[][] {bytes(g), bytes(m)};
      } finally {
        g.release();
        m.release();
      }
    }
  }

  /**
   * One large image solved tile by tile: each tile (TILE_SIZE plus TILE_OVERLAP on every side, so
   * SGM paths and filters start outside the tile core) gets its own reference crop and, per
   * neighbour, a canvas covering only what the tile can reach, resampled from only the neighbour
   * region it needs. Tile cores are written into the image's map files; nothing image-sized is
   * kept in the heap. Results differ from an untiled solve only near tile borders (paths and
   * speckle components are cut at the overlap edge).
   */
  Map<String, Object> tiledDepth(List<ImageFrame> frames, List<FeatureTracks.Track> tracks, Plan header, Path dir)
      throws IOException {
    int c = header.camera;
    ImageFrame rf = frames.get(c);
    int W = rf.width, H = rf.height, T = tileSize(rf), O = TILE_OVERLAP;
    String[] kinds = {".depth", ".margin", ".sharp", ".photo"};
    for (String k : kinds) DepthMapFile.create(dir.resolve("camera_" + c + k), rf.camera, W, H);
    double[] seeds = seeds(rf, tracks, header.labels);
    long valid = 0, cells = 0, peak = 0;
    int tiles = 0;
    for (int ty0 = 0; ty0 < H; ty0 += T)
      for (int tx0 = 0; tx0 < W; tx0 += T) {
        int cx1 = Math.min(W, tx0 + T), cy1 = Math.min(H, ty0 + T);
        int ex0 = Math.max(0, tx0 - O), ey0 = Math.max(0, ty0 - O), ex1 = Math.min(W, cx1 + O), ey1 = Math.min(H, cy1 + O);
        int ew = ex1 - ex0, eh = ey1 - ey0;
        byte[][] r = region(rf, ex0, ey0, ew, eh);
        boolean defer = GpuMvsgmCensus.enabled(o);
        Reference ref = new Reference(ew, eh, r[0], r[1], census, defer);
        if (count(ref.use) == 0) continue;
        Warped[] views = new Warped[header.neighbours.size()];
        for (int i = 0; i < views.length; i++) {
          ImageFrame of = frames.get(header.neighbours.get(i).id());
          int[] cv = canvas(rf.camera, of.camera, ex0, ey0, ew, eh, header.labels, census);
          int[] src = sourceRegion(homography(rf.camera, of.camera), cv, of.width, of.height);
          byte[][] g = src == null ? new byte[][] {new byte[4], new byte[4]} : region(of, src[0], src[1], src[2], src[3]);
          if (src == null) src = new int[] {0, 0, 2, 2};
          views[i] = warp(rf.camera, of.camera, of.id, cv, g[0], g[1], src[0], src[1], src[2], src[3], census, defer);
        }
        Plan p = new Plan(c, header.neighbours, header.labels, ref, views, ex0, ey0);
        double[] local = new double[seeds.length];
        int n = 0;
        for (int s = 0; s + 2 < seeds.length; s += 3)
          if (seeds[s] >= ex0 && seeds[s] < ex1 && seeds[s + 1] >= ey0 && seeds[s + 1] < ey1) {
            local[n++] = seeds[s] - ex0;
            local[n++] = seeds[s + 1] - ey0;
            local[n++] = seeds[s + 2];
          }
        Solved solved = solve(p, Arrays.copyOf(local, n));
        DepthMap m = select(p, solved.summary());
        float[][] maps = {m.depth(), m.margin(), m.sharpness(), m.photometric()};
        for (int k = 0; k < kinds.length; k++)
          DepthMapFile.writeRegion(dir.resolve("camera_" + c + kinds[k]), W, tx0, ty0, cx1 - tx0, cy1 - ty0,
              maps[k], ew, tx0 - ex0, ty0 - ey0);
        for (int y = ty0; y < cy1; y++)
          for (int x = tx0; x < cx1; x++) if (m.depth()[(y - ey0) * ew + x - ex0] > 0) valid++;
        cells += solved.bands().total();
        peak = Math.max(peak, solved.peakDeviceBytes());
        tiles++;
      }
    Map<String, Object> report = new LinkedHashMap<>(Json.object(
        "tiled", true, "tile_size", T, "tile_overlap", O, "tiles", tiles, "packed_cells", cells,
        "peak_device_bytes", peak, "backend", lastBackend, "valid", valid,
        "neighbours", header.neighbours.stream().map(x -> x.id()).toList()));
    annotateNeighborSelection(c, report);
    return report;
  }

  // ---------------------------------------------------------------------------------------------
  // Selection and filters

  /**
   * A camera's result: depth (0 = none), and the per-pixel evidence written with its candidates:
   * margin = (second - best) / second aggregated cost (labels more than one away), sharpness =
   * rise of the aggregated cost to the lower neighbouring label over directions x P1 (capped at 1),
   * photometric = 1 - aggregated cost / (directions x half the census bits) (capped to 0..1).
   */
  record DepthMap(float[] depth, float[] margin, float[] sharpness, float[] photometric, Map<String, Object> report) {}

  DepthMap select(Plan plan, int[] summary) {
    Reference ref = plan.ref;
    int w = ref.w, h = ref.h, n = w * h;
    Labels labels = plan.labels;
    float[] label = new float[n], margin = new float[n], sharp = new float[n], photo = new float[n];
    Arrays.fill(label, Float.NaN);
    double rise = (double) o.directions * p1(), worst = o.directions * (double) penalty();
    long none = 0, boundary = 0, ambiguous = 0, selected = 0;
    for (int p = 0; p < n; p++) {
      int s = 6 * p, best = summary[s], value = summary[s + 1], second = summary[s + 2];
      if (!ref.use[p]) continue;
      if (best < 0) {
        none++;
        continue;
      }
      if ((summary[s + 5] & 7) != 0) {
        boundary++;
        continue;
      }
      if (second != Integer.MAX_VALUE && second - value <= Math.max(1, value * o.uniqueness)) {
        ambiguous++;
        continue;
      }
      int before = summary[s + 3], after = summary[s + 4];
      double offset = before >= 0 && after >= 0 ? StereoCost.subpixel(o, before, value, after) : 0;
      label[p] = (float) (best + offset);
      margin[p] = second == Integer.MAX_VALUE ? 1 : (float) ((second - value) / (double) second);
      int side = before >= 0 && after >= 0 ? Math.min(before, after) : Math.max(before, after);
      sharp[p] = side < 0 ? 0 : (float) Math.min(1, (side - value) / rise);
      photo[p] = (float) Math.max(0, Math.min(1, 1 - value / worst));
      selected++;
    }
    long median = 0;
    if (o.median > 1 && !o.medianMode.equals("off")) {
      float[] before = label;
      label = o.medianMode.equals("classic")
          ? CensusSgm.medianValid(label, w, h, o.median)
          : CensusSgm.medianEdgePreserving(label, w, h, o.median, o.medianMaxChange, ref.gray, o.medianEdgeThreshold);
      for (int p = 0; p < n; p++) if (!(label[p] == before[p]) && Float.isFinite(before[p])) median++;
    }
    long speckles = CensusSgm.removeSpeckles(label, w, h, o.speckleSize, 1.0, o.speckleConnectivity);
    float[] depth = new float[n];
    long valid = 0;
    for (int p = 0; p < n; p++)
      if (Float.isFinite(label[p])) {
        depth[p] = (float) (1 / labels.q(label[p]));
        valid++;
      } else margin[p] = sharp[p] = photo[p] = 0;
    Map<String, Object> report = Json.object(
        "used_pixels", count(ref.use), "no_winner", none, "range_or_band_edge_rejected", boundary,
        "uniqueness_rejected", ambiguous, "selected", selected, "median_changed", median,
        "speckle_rejected", speckles, "valid", valid);
    return new DepthMap(depth, margin, sharp, photo, report);
  }

  static long count(boolean[] b) {
    long n = 0;
    for (boolean v : b) if (v) n++;
    return n;
  }

  void parallel(int n, java.util.function.IntConsumer action) {
    ForkJoinPool pool = new ForkJoinPool(Math.max(1, o.cores));
    try {
      pool.submit(() -> IntStream.range(0, n).parallel().forEach(action)).get();
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      throw new IllegalStateException(e);
    } catch (ExecutionException e) {
      if (e.getCause() instanceof RuntimeException r) throw r;
      if (e.getCause() instanceof Error r) throw r;
      throw new IllegalStateException(e.getCause());
    } finally {
      pool.shutdown();
    }
  }

  // ---------------------------------------------------------------------------------------------
  // Driver

  /** Phase-1 facts phase 2 needs: neighbours (best first) and the inverse-depth label step. */
  record CameraInfo(int camera, int[] neighbours, double step, double near, double far, boolean tiled) {}

  /** --export-disparity-images or --diagnostics: per-camera depth previews (PFM + PNG + JSON). */
  boolean exportImages() {
    return o.exportDisparityImages || o.diagnostics;
  }

  /**
   * Writes 05_stereo/mvsgm/image_<id>_<name>/<kind>.pfm (depth in scene units, NaN = none),
   * <kind>.png (linear grayscale over the camera's depth range: near dark, far bright; 0 = none)
   * and <kind>.json, as the pair path's disparity export. With {@code keep}, only pixels whose
   * keep value is not NaN (the consistent ones) are written.
   */
  void exportDepth(ImageFrame f, String kind, float[] z, float[] keep, double near, double far) throws IOException {
    float[] values = new float[z.length];
    for (int p = 0; p < z.length; p++)
      values[p] = z[p] > 0 && (keep == null || !Float.isNaN(keep[p])) ? z[p] : Float.NaN;
    String stem = f.path.getFileName().toString().replaceFirst("[.][^.]*$", "").replaceAll("[^A-Za-z0-9_.-]", "_");
    ProjectArtifacts.scalar(
        o.output.resolve("05_stereo/mvsgm/image_" + f.id + "_" + stem), kind, values, f.width, f.height, near, far);
  }

  /**
   * exportDepth for a large (tiled) image without image-sized heap arrays: the PFM is written at
   * full resolution row by row from the memory-mapped map; the PNG is downscaled so its longer
   * side is at most 4,096 pixels (each preview pixel = mean of the valid depths in its block;
   * none = 0) and the JSON records the scale.
   */
  void exportDepthStreamed(ImageFrame f, String kind, java.lang.foreign.MemorySegment z,
      java.lang.foreign.MemorySegment keep, double near, double far) throws IOException {
    int w = f.width, h = f.height, scale = Math.max(1, Math.ceilDiv(Math.max(w, h), 4096));
    String stem = f.path.getFileName().toString().replaceFirst("[.][^.]*$", "").replaceAll("[^A-Za-z0-9_.-]", "_");
    Path dir = o.output.resolve("05_stereo/mvsgm/image_" + f.id + "_" + stem);
    Files.createDirectories(dir);
    float[] row = new float[w];
    java.util.function.IntConsumer read = y -> {
      for (int x = 0; x < w; x++) {
        long p = (long) y * w + x;
        float v = z.getAtIndex(MappedFiles.FLOAT, p);
        row[x] = v > 0 && (keep == null || !Float.isNaN(keep.getAtIndex(MappedFiles.FLOAT, p))) ? v : Float.NaN;
      }
    };
    try (OutputStream out = new BufferedOutputStream(Files.newOutputStream(dir.resolve(kind + ".pfm")), 1 << 20)) {
      out.write(("Pf\n" + w + " " + h + "\n-1.0\n").getBytes(java.nio.charset.StandardCharsets.US_ASCII));
      java.nio.ByteBuffer b = java.nio.ByteBuffer.allocate(4 * w).order(java.nio.ByteOrder.LITTLE_ENDIAN);
      for (int y = h - 1; y >= 0; y--) {
        read.accept(y);
        b.clear();
        b.asFloatBuffer().put(row);
        out.write(b.array());
      }
    }
    int pw = Math.ceilDiv(w, scale), ph = Math.ceilDiv(h, scale);
    byte[] display = new byte[pw * ph];
    double[] sum = new double[pw];
    int[] count = new int[pw];
    for (int py = 0; py < ph; py++) {
      Arrays.fill(sum, 0);
      Arrays.fill(count, 0);
      for (int y = py * scale; y < Math.min(h, (py + 1) * scale); y++) {
        read.accept(y);
        for (int x = 0; x < w; x++)
          if (Float.isFinite(row[x])) {
            sum[x / scale] += row[x];
            count[x / scale]++;
          }
      }
      for (int px = 0; px < pw; px++)
        if (count[px] > 0)
          display[py * pw + px] = (byte) Math.max(1, Math.min(255, 1 + 254 * (sum[px] / count[px] - near) / Math.max(1e-12, far - near)));
    }
    org.opencv.core.Mat preview = new org.opencv.core.Mat(ph, pw, org.opencv.core.CvType.CV_8U);
    try {
      preview.put(0, 0, display);
      if (!org.opencv.imgcodecs.Imgcodecs.imwrite(dir.resolve(kind + ".png").toString(), preview))
        throw new IOException("Cannot write " + dir.resolve(kind + ".png"));
    } finally {
      preview.release();
    }
    Json.write(dir.resolve(kind + ".json"), Json.object(
        "width", w, "height", h, "preview_range", new double[] {near, far}, "preview_zero", "invalid/NaN",
        "numeric_data", kind + ".pfm", "preview_scale", scale,
        "preview", "linear grayscale, valid values clamped to 1..255; downscaled by preview_scale (block mean of"
            + " valid depths); preview clipping does not alter numeric PFM"));
  }

  final Map<Integer, CameraInfo> info = new TreeMap<>();
  final Map<String, Object> phaseTwo = new LinkedHashMap<>();

  /**
   * {@code --dense-method mvsgm}: phase 1 computes every camera's depth map (written to
   * 05_stereo/mvsgm, deleted afterwards unless --diagnostics or -Dsgm.mvsgm.keepMaps=true); phase
   * 2 keeps a depth only where at least --mvsgm-consistent-views of the camera's neighbour maps
   * agree when the point is reprojected (within --mvsgm-consistency-labels of that neighbour's
   * inverse-depth label steps; the multi-view analogue of the left/right check), and streams the
   * survivors as candidates. Each camera is one fusion pair (camera, best neighbour), marked single-
   * view: image fusion splats it into the camera only (pass-1 merge is one sample per pixel), the
   * neighbour gives the triangulation-angle weight. Pairs and candidates are in camera order.
   */
  long run(
      List<ImageFrame> frames,
      List<FeatureTracks.Track> tracks,
      DepthFusion fusion,
      DataOutputStream out,
      List<Long> pairStarts)
      throws IOException {
    long started = System.nanoTime();
    if (o.step > 1)
      System.out.println(
          "Note: --step " + o.step + " does not thin --dense-method mvsgm depth maps; every consistent"
              + " pixel goes to fusion (sparse samples of a per-camera map are removed as speckles)");
    List<Object> cameraReports;
    Path dir = o.output.resolve("05_stereo/mvsgm");
    // Own even partially written phase-1 files: DenseReconstruction closes this hand-off on
    // failure too. Dense resumes recompute these maps from the saved geometry.
    handoff = new MvsgmMaps(dir, o.diagnostics || Boolean.getBoolean("sgm.mvsgm.keepMaps"));
    for (ImageFrame frame : frames)
      if (usable(frame.camera))
        handoff.add(new MvsgmMaps.Camera(frame.id, -1, 0, 0, null, false));
    try (GpuMemoryPool.Scope pool =
        o.compute.equals("cpu") ? null : GpuMemoryPool.install(gpuBudget())) {
      cameraReports = depthMaps(frames, tracks, null);
    }
    RunTelemetry.record("Multi-view SGM depth maps", started);
    if (info.isEmpty())
      throw new IllegalArgumentException(
          "--dense-method mvsgm produced no depth map: " + (skipped.isEmpty() ? "no usable camera"
              : "every camera was skipped, e.g. image " + skipped.keySet().iterator().next() + ": "
                  + skipped.values().iterator().next()) + " (05_stereo/mvsgm/mvsgm.json lists each camera)");
    long phase2 = System.nanoTime();
    long count = 0, considered = 0, kept = 0, inconsistent = 0;
    int minViews = o.mvsgmConsistentViews;
    double tolerance = o.mvsgmConsistencyLabels;
    // Depth maps are memory-mapped (never read whole into the heap): the camera's own map and its
    // neighbours' maps, read where the reprojections fall. Colours are copied in row bands.
    try (WorkProgress progress = WorkProgress.start("Multi-view consistency", info.size(), "cameras")) {
      int done = 0;
      for (CameraInfo ci : info.values()) {
        progress.update(done++);
        int c = ci.camera();
        int pair = fusion.addPair(c, ci.neighbours()[0]);
        fusion.markSingleView(pair);
        while (pairStarts.size() <= pair) pairStarts.add(count);
        Consistency result = consistentCamera(frames, ci, pair);
        considered += result.considered();
        kept += result.kept();
        inconsistent += result.considered() - result.kept();
        count += result.kept();
      }
      progress.done();
    }
    RunTelemetry.record("Multi-view consistency", phase2);
    phaseTwo.putAll(Json.object(
        "consistent_views_required", minViews, "tolerance_labels", tolerance,
        "depths", considered, "consistent", kept, "inconsistent_rejected", inconsistent, "candidates", count,
        "handoff", "maps"));
    Json.write(dir.resolve("mvsgm.json"), Json.object(
        "census_window", census, "best_k", k, "neighbours", neighbors, "label_pixels", labelPixels,
        "levels", levels(), "consistency", phaseTwo, "cameras", cameraReports));
    System.out.printf(Locale.ROOT, "mvsgm: %,d of %,d depths consistent; %,d candidates (maps handed to fusion)%n", kept, considered, count);
    return count;
  }

  record Consistency(long considered, long kept) {}

  /** Reads raw neighbours only; no neighbour's consistency or fusion result is an input. */
  Consistency consistentCamera(List<ImageFrame> frames, CameraInfo ci, int pair) throws IOException {
    int c = ci.camera(), minViews = o.mvsgmConsistentViews;
    double tolerance = o.mvsgmConsistencyLabels;
    Path dir = o.output.resolve("05_stereo/mvsgm");
    try (MappedFiles mapped = new MappedFiles()) {
      ImageFrame rf = frames.get(c);
      int w = rf.width, h = rf.height;
      long bytes = 4L * w * h;
      java.lang.foreign.MemorySegment z = mapped.map(dir.resolve("camera_" + c + ".depth"), DepthMapFile.HEADER, bytes, false);
      java.lang.foreign.MemorySegment[] others = new java.lang.foreign.MemorySegment[ci.neighbours().length];
      for (int i = 0; i < others.length; i++) {
        CameraInfo oi = info.get(ci.neighbours()[i]);
        if (oi != null) {
          ImageFrame of = frames.get(oi.camera());
          others[i] = mapped.map(dir.resolve("camera_" + oi.camera() + ".depth"), DepthMapFile.HEADER, 4L * of.width * of.height, false);
        }
      }
      Path lrFile = dir.resolve("camera_" + c + ".lr"), rgbFile = dir.resolve("camera_" + c + ".rgb");
      DepthMapFile.create(lrFile, rf.camera, w, h);
      DepthMapFile.create(rgbFile, rf.camera, w, h);
      java.lang.foreign.MemorySegment lr = mapped.map(lrFile, DepthMapFile.HEADER, bytes, true);
      java.lang.foreign.MemorySegment rgb = mapped.map(rgbFile, DepthMapFile.HEADER, bytes, true);
      Camera cam = rf.camera;
      double[] rt = Math3.transpose(cam.rotation);
      // lr = best agreement in labels; NaN = no depth or rejected (fewer than minViews agreeing
      // neighbours). A candidate is a pixel with depth > 0 and lr not NaN.
      long[] rows = new long[2 * h];
      parallel(h, y -> {
        double[] v = new double[3];
        for (int x = 0; x < w; x++) {
          int p = y * w + x;
          float zp = z.getAtIndex(MappedFiles.FLOAT, p);
          if (zp <= 0) {
            lr.setAtIndex(MappedFiles.FLOAT, p, Float.NaN);
            continue;
          }
          rows[2 * y]++;
          double[] X = world(cam, rt, x, y, zp);
          int agree = 0;
          double best = Double.POSITIVE_INFINITY;
          for (int i = 0; i < others.length && minViews > 0; i++) {
            CameraInfo oi = info.get(ci.neighbours()[i]);
            if (oi == null || others[i] == null) continue;
            ImageFrame of = frames.get(oi.camera());
            of.camera.viewInto(X[0], X[1], X[2], v);
            if (v[2] <= 0) continue;
            int u = (int) Math.round(of.camera.fx * v[0] / v[2] + of.camera.cx),
                t = (int) Math.round(of.camera.fy * v[1] / v[2] + of.camera.cy);
            if (u < 0 || t < 0 || u >= of.width || t >= of.height) continue;
            float d = others[i].getAtIndex(MappedFiles.FLOAT, (long) t * of.width + u);
            if (d <= 0) continue;
            double labelsOff = Math.abs(1 / v[2] - 1 / (double) d) / oi.step();
            if (labelsOff <= tolerance) {
              agree++;
              best = Math.min(best, labelsOff);
            }
          }
          if (agree >= minViews) {
            lr.setAtIndex(MappedFiles.FLOAT, p, minViews == 0 ? 0 : (float) best);
            rows[2 * y + 1]++;
          } else lr.setAtIndex(MappedFiles.FLOAT, p, Float.NaN);
        }
      });
      long camConsidered = 0, camKept = 0;
      long[] rowStart = new long[h + 1];
      for (int y = 0; y < h; y++) {
        camConsidered += rows[2 * y];
        camKept += rows[2 * y + 1];
        rowStart[y + 1] = camKept;
      }
      // Packed colour r << 16 | g << 8 | b (as fusion stores it), copied in bands of rows.
      int band = Math.max(1, (1 << 22) / Math.max(1, w));
      for (int y0 = 0; y0 < h; y0 += band) {
        int y1 = Math.min(h, y0 + band);
        byte[] colors;
        synchronized (RasterCache.class) {
          RasterCache.pair(rf, rf);
          org.opencv.core.Mat sub = rf.color.submat(y0, y1, 0, w).clone();
          try {
            colors = bytes(sub);
          } finally {
            sub.release();
          }
        }
        long base = (long) y0 * w;
        for (int i = 0; i < (y1 - y0) * w; i++)
          rgb.setAtIndex(MappedFiles.INT, base + i,
              (colors[i * 3 + 2] & 255) << 16 | (colors[i * 3 + 1] & 255) << 8 | colors[i * 3] & 255);
      }
      // Every consistent pixel goes to fusion: the maps are dense per camera, and a thinned
      // (--step > 1) map falls apart in fusion's 4-connected speckle filter.
      handoff.add(new MvsgmMaps.Camera(c, pair, w, h, rowStart, ci.tiled()));
      if (exportImages() && ci.tiled()) exportDepthStreamed(rf, "consistent_depth", z, lr, ci.near(), ci.far());
      else if (exportImages()) {
        float[] zz = DepthMapFile.read(dir.resolve("camera_" + c + ".depth")).z();
        float[] ll = new float[w * h];
        java.lang.foreign.MemorySegment.copy(lr, MappedFiles.FLOAT, 0, ll, 0, w * h);
        exportDepth(rf, "consistent_depth", zz, ll, ci.near(), ci.far());
      }
      System.out.printf(Locale.ROOT, "mvsgm image %d: %,d of %,d depths consistent with %d+ neighbour maps%n",
          c, camKept, camConsidered, minViews);
        return new Consistency(camConsidered, camKept);
    }
  }

  /** Phase 2's result for image fusion (null before run); the caller closes it after fusion. */
  MvsgmMaps handoff;

  long gpuBudget() {
    try {
      return GpuSgm.budget(GpuSgm.selectedDevice(o), o);
    } catch (RuntimeException e) {
      return 0;
    }
  }

  static double[] world(Camera cam, double[] rt, double x, double y, double z) {
    double lx = (x - cam.cx) / cam.fx * z - cam.translation[0], ly = (y - cam.cy) / cam.fy * z - cam.translation[1],
        lz = z - cam.translation[2];
    return new double[] {
      rt[0] * lx + rt[1] * ly + rt[2] * lz, rt[3] * lx + rt[4] * ly + rt[5] * lz, rt[6] * lx + rt[7] * ly + rt[8] * lz
    };
  }


  static byte[] bytes(org.opencv.core.Mat m) {
    byte[] b = new byte[(int) (m.total() * m.channels())];
    m.get(0, 0, b);
    return b;
  }

  /** Gray and valid mask of a frame (undistorted, processed size). */
  static byte[][] rasters(ImageFrame f) throws IOException {
    synchronized (RasterCache.class) {
      RasterCache.pair(f, f);
      return new byte[][] {bytes(f.gray), bytes(f.validMask)};
    }
  }

  /**
   * Depth maps for the given cameras (all usable ones when null), written to 05_stereo/mvsgm as
   * DepthMapFile (.depth, .margin). Returns the per-camera reports.
   */
  /** One camera solve, shared by the sequential driver and the overlap-window producer. */
  Object computeCamera(List<ImageFrame> frames, List<FeatureTracks.Track> tracks, int c, Plan plan, long t0)
      throws IOException {
    Path dir = o.output.resolve("05_stereo/mvsgm");
    if (plan.tiled()) {
      ImageFrame rf = frames.get(c);
      long t1 = System.nanoTime();
      Map<String, Object> tiled = tiledDepth(frames, tracks, plan, dir);
      double near = 1 / plan.labels.q(plan.labels.count() - 1), far = 1 / plan.labels.low();
      info.put(c, new CameraInfo(c, plan.neighbours.stream().mapToInt(Neighbor::id).toArray(), plan.labels.step(), near, far, true));
      if (exportImages())
        try (MappedFiles mapped = new MappedFiles()) {
          exportDepthStreamed(rf, "depth", mapped.map(dir.resolve("camera_" + c + ".depth"), DepthMapFile.HEADER,
              4L * rf.width * rf.height, false), null, near, far);
        }
      tiled.put("image", c);
      tiled.put("labels", plan.labels.count());
      tiled.put("wait_seconds", (t1 - t0) / 1e9);
      tiled.put("seconds", (System.nanoTime() - t1) / 1e9);

      RunTelemetry.record("Multi-view SGM camera " + c, t0);
      System.out.printf(Locale.ROOT, "mvsgm image %d: %dx%d in %s tiles, %d labels, %.1f s, %,d valid%n",
          c, rf.width, rf.height, tiled.get("tiles"), plan.labels.count(), (System.nanoTime() - t1) / 1e9, tiled.get("valid"));
      return tiled;
    }
    long t1 = System.nanoTime();
    Solved solved = solve(plan, seeds(frames.get(c), tracks, plan.labels));
    int[] summary = solved.summary();
    DisparityBands bands = solved.bands();
    long t2 = System.nanoTime();
    DepthMap map = select(plan, summary);
    long t3 = System.nanoTime();
    ImageFrame rf = frames.get(c);
    DepthMapFile.write(dir.resolve("camera_" + c + ".depth"), rf.camera, rf.width, rf.height, map.depth());
    DepthMapFile.write(dir.resolve("camera_" + c + ".margin"), rf.camera, rf.width, rf.height, map.margin());
    DepthMapFile.write(dir.resolve("camera_" + c + ".sharp"), rf.camera, rf.width, rf.height, map.sharpness());
    DepthMapFile.write(dir.resolve("camera_" + c + ".photo"), rf.camera, rf.width, rf.height, map.photometric());
    int[] ids = plan.neighbours.stream().mapToInt(Neighbor::id).toArray();
    double near = 1 / plan.labels.q(plan.labels.count() - 1), far = 1 / plan.labels.low();
    info.put(c, new CameraInfo(c, ids, plan.labels.step(), near, far, false));
    if (exportImages()) exportDepth(rf, "depth", map.depth(), null, near, far);
    List<Object> nb = new ArrayList<>();
    for (Neighbor x : plan.neighbours)
      nb.add(Json.object("image", x.id(), "score", x.score(), "shared_points", x.shared(), "median_angle_deg", x.medianAngle()));
    Map<String, Object> entry = Json.object(
        "image", c, "neighbours", nb, "labels", plan.labels.count(), "inverse_depth_low", plan.labels.low(),
        "inverse_depth_step", plan.labels.step(), "depth_near", 1 / plan.labels.q(plan.labels.count() - 1),
        "depth_far", 1 / plan.labels.low(), "sparse_samples", plan.labels.samples(),
        "packed_cells", bands.total(), "full_range_cells", (long) count(plan.ref.use) * plan.labels.count(),
        "peak_device_bytes", solved.peakDeviceBytes(), "levels", solved.levels(),
        "backend", lastBackend, "p1", p1(), "p2", p2(),
        "prepare_wait_seconds", (t1 - t0) / 1e9, "sgm_seconds", (t2 - t1) / 1e9, "select_seconds", (t3 - t2) / 1e9,
        "selection", map.report());

    annotateNeighborSelection(c, entry);
    RunTelemetry.record("Multi-view SGM camera " + c, t0);
    System.out.printf(Locale.ROOT, "mvsgm image %d: %d neighbours, %d labels, %s, waited %.2f s for preparation, SGM %.2f s, %,d valid%n",
        c, plan.views.length, plan.labels.count(), lastBackend, (t1 - t0) / 1e9, (t2 - t1) / 1e9, map.report().get("valid"));
    return entry;
  }

  List<Object> depthMaps(List<ImageFrame> frames, List<FeatureTracks.Track> tracks, int[] cameras) throws IOException {
    long points = tracks.stream().filter(t -> t.point != null).count();
    if (points < 12)
      throw new IllegalArgumentException(
          "--dense-method mvsgm needs sparse tie points (neighbour choice and depth ranges), but this"
              + " project has " + points + " triangulated tracks. With supplied fixed poses, rerun from"
              + " the match step so tie points are matched (or use --dense-method sgm).");
    Path dir = o.output.resolve("05_stereo/mvsgm");
    Files.createDirectories(dir);
    List<Object> report = new ArrayList<>();
    if (cameras == null)
      cameras = frames.stream().filter(f -> usable(f.camera)).mapToInt(f -> f.id).toArray();
    // The next camera's plan (CPU: neighbour resampling, census) is prepared on one thread while the
    // current camera is solved; at most two plans are resident. Plans do not depend on each other.
    ExecutorService preparer = Executors.newSingleThreadExecutor(r -> {
      Thread t = new Thread(r, "mvsgm-prepare");
      t.setDaemon(true);
      return t;
    });
    final int[] list = cameras;
    // Row workers (CensusSgm, TsgmSearch: independent rows, the same results on any number of
    // threads) for the plan thread and for this thread; without a scope those loops are serial.
    final int rowCores = Math.max(1, o.cores / 2);
    java.util.function.IntFunction<Future<Plan>> prepare = i -> preparer.submit(() -> {
      try (var rows = StereoRows.open(rowCores)) {
        return plan(frames, tracks, list[i]);
      }
    });
    try (var rows = StereoRows.open(rowCores);
        WorkProgress progress = WorkProgress.start("Multi-view SGM cameras", cameras.length, "cameras")) {
      Future<Plan> next = cameras.length > 0 ? prepare.apply(0) : null;
      for (int i = 0; i < cameras.length; i++) {
        int c = cameras[i];
        progress.update(i);
        long t0 = System.nanoTime();
        Plan plan;
        try {
          plan = next.get();
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
          throw new IOException("mvsgm interrupted", e);
        } catch (ExecutionException e) {
          if (e.getCause() instanceof IOException io) throw io;
          if (e.getCause() instanceof RuntimeException r) throw r;
          if (e.getCause() instanceof Error r) throw r;
          throw new IOException(e.getCause());
        }
        next = i + 1 < cameras.length ? prepare.apply(i + 1) : null;
        if (plan == null) {
          String reason = skipped.getOrDefault(c, "no neighbour");
          report.add(Json.object("image", c, "skipped", reason));
          System.out.println("WARNING: mvsgm image " + c + " (" + frames.get(c).path.getFileName()
              + ") has no depth map: " + reason);
          continue;
        }
        report.add(computeCamera(frames, tracks, c, plan, t0));
      }
      progress.done();
    } finally {
      preparer.shutdownNow();
    }
    Json.write(dir.resolve("mvsgm.json"), Json.object(
        "census_window", census, "best_k", k, "neighbours", neighbors, "label_pixels", labelPixels, "cameras", report));
    return report;
  }

  // ---------------------------------------------------------------------------------------------
  // Stage-1 prototype (winner-take-all on the raw or box-averaged cost; MULTIVIEW_SGM.md)

  void prototype(List<ImageFrame> frames, List<FeatureTracks.Track> tracks, int[] cameras) throws IOException {
    Path dir = o.output.resolve("05_stereo/mvsgm");
    Files.createDirectories(dir);
    int box = Integer.getInteger("sgm.mvsgm.box", 0);
    for (int c : cameras) {
      Plan plan = plan(frames, tracks, c);
      Reference ref = plan.ref;
      int w = ref.w, h = ref.h, L = plan.labels.count();
      DisparityBands bands = DisparityBands.full(w, h, L, ref.use);
      short[] cost = costs(plan, bands);
      int[] offset = bands.offsets();
      float[] depth = new float[w * h], margin = new float[w * h];
      parallel(h, y -> {
        int[] cv = new int[L];
        for (int x = 0; x < w; x++) {
          int p = y * w + x;
          if (!ref.use[p]) continue;
          for (int l = 0; l < L; l++) {
            int sum = 0, nn = 0;
            for (int yy = Math.max(0, y - box); yy <= Math.min(h - 1, y + box); yy++)
              for (int xx = Math.max(0, x - box); xx <= Math.min(w - 1, x + box); xx++) {
                int q = yy * w + xx;
                if (bands.count[q] == 0 || cost[offset[q] + l] == INVALID) continue;
                sum += cost[offset[q] + l];
                nn++;
              }
            cv[l] = nn == 0 ? -1 : sum * 64 / nn;
          }
          int l1 = -1, c1 = Integer.MAX_VALUE, c2 = Integer.MAX_VALUE;
          for (int l = 0; l < L; l++) if (cv[l] >= 0 && cv[l] < c1) { c1 = cv[l]; l1 = l; }
          if (l1 <= 0 || l1 >= L - 1) continue;
          for (int l = 0; l < L; l++) if (Math.abs(l - l1) > 1 && cv[l] >= 0) c2 = Math.min(c2, cv[l]);
          double off = cv[l1 - 1] >= 0 && cv[l1 + 1] >= 0 ? StereoCost.subpixel(o, cv[l1 - 1], c1, cv[l1 + 1]) : 0;
          depth[p] = (float) (1 / plan.labels.q(l1 + off));
          margin[p] = c2 == Integer.MAX_VALUE ? 1 : (float) ((c2 - c1) / (double) Math.max(1, c2));
        }
      });
      ImageFrame rf = frames.get(c);
      DepthMapFile.write(dir.resolve("camera_" + c + ".depth"), rf.camera, w, h, depth);
      DepthMapFile.write(dir.resolve("camera_" + c + ".margin"), rf.camera, w, h, margin);
      float[] lab = new float[w * h];
      for (int p = 0; p < lab.length; p++)
        lab[p] = depth[p] > 0 ? (float) ((1 / depth[p] - plan.labels.low()) / plan.labels.step()) : Float.NaN;
      CensusSgm.removeSpeckles(lab, w, h, Math.max(1, o.speckleSize), 1.0, 8);
      float[] filtered = depth.clone();
      for (int p = 0; p < filtered.length; p++) if (!Float.isFinite(lab[p])) filtered[p] = 0;
      DepthMapFile.write(dir.resolve("camera_" + c + ".speckle"), rf.camera, w, h, filtered);
    }
  }
}
