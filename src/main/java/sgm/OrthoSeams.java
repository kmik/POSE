package sgm;

import java.awt.image.BufferedImage;
import java.io.IOException;
import java.nio.file.Path;
import java.util.*;
import javax.imageio.*;
import javax.imageio.stream.ImageInputStream;

/**
 * Seams of the mosaic orthophoto (--orthomosaic-mode mosaic): which photograph paints which part
 * of the mosaic, and the colour gain of each photograph.
 *
 * <ol>
 *   <li>A coarse seam grid (cells of {@link #cellPixels} mosaic pixels) over the mosaic. Per cell,
 *       the photographs that see its surface point, up to {@link #CANDIDATES} of them, the most
 *       vertical first, each with its colour there (from a box-filtered reduced copy of the
 *       photograph).
 *   <li>Gains: one per photograph and colour channel, least squares over the cells two photographs
 *       share (Brown and Lowe), so neighbouring photographs agree in brightness and colour.
 *   <li>Labels: every cell gets one photograph by minimising, over the whole grid, the obliquity of
 *       the chosen views (tan^2 of the angle from the vertical) plus, for every pair of adjacent
 *       cells with different photographs, a seam cost: a constant plus the colour difference of
 *       the two photographs there. Seams therefore run where the photographs agree (open ground,
 *       shadows, gaps between crowns), not through crowns or roofs, and each photograph paints a
 *       contiguous region around its nadir. Alpha-expansion with graph cuts, photographs in a fixed
 *       order: deterministic.
 *   <li>Blend weights: the share of each photograph among the labels within {@link #radius} cells,
 *       up to four photographs per cell; the renderer interpolates them bilinearly between cells.
 * </ol>
 */
final class OrthoSeams {
  static final int CANDIDATES = 12, SHARES = 4, MAX_CELLS = 4_000_000;
  /** Obliquity weight (per tan^2 of the angle from the vertical) and seam constant, in colour levels. */
  static final double OBLIQUITY = 120, SEAM = 3, OUTSIDE = 255;
  static final int CYCLES = 3;

  final int cellPixels, gw, gh, radius;
  final double x0, yTop, pixel;
  /** CSR candidates per cell: views, tan^2 of the angle from the vertical, colours (0xRRGGBB). */
  int[] start;
  short[] view;
  float[] obliquity;
  int[] colour;
  /** Label (view index) per cell, -1 = none. */
  int[] label;
  /** Gain per view and channel (R, G, B). */
  final double[][] gain;
  /** Blend shares per cell: up to SHARES views, -1 = unused. */
  short[] shareView;
  float[] shareWeight;
  long cells, candidatesTotal, expansionsChanged, seamEdges;
  double energyBefore, energyAfter;

  private OrthoSeams(int cellPixels, int gw, int gh, double x0, double yTop, double pixel, int radius, int views) {
    this.cellPixels = cellPixels;
    this.gw = gw;
    this.gh = gh;
    this.x0 = x0;
    this.yTop = yTop;
    this.pixel = pixel;
    this.radius = radius;
    gain = new double[views][];
    for (int i = 0; i < views; i++) gain[i] = new double[] {1, 1, 1};
  }

  /** Seam cell size in mosaic pixels for a mosaic of width x height pixels. */
  static int cellPixels(int width, int height) {
    return (int) Math.max(8, Math.ceil(Math.sqrt((double) width * height / MAX_CELLS)));
  }

  /**
   * Builds the seams. thumbnails supplies each view's reduced photograph for colours (step:
   * mosaic pixels per thumbnail pixel, roughly); maxTan2 limits the obliquity (POSITIVE_INFINITY:
   * none); feather is the blend half-width in mosaic pixels.
   */
  static OrthoSeams build(
      Orthomosaic.Surface surface, List<Orthomosaic.View> views, Thumbnails thumbnails, double x0, double yTop, double pixel,
      int width, int height, double maxTan2, double feather, int workers) throws IOException {
    int s = cellPixels(width, height);
    int gw = Math.ceilDiv(width, s), gh = Math.ceilDiv(height, s);
    int radius = (int) Math.max(0, Math.round(feather / s));
    OrthoSeams seams = new OrthoSeams(s, gw, gh, x0, yTop, pixel, radius, views.size());
    seams.candidates(surface, views, maxTan2, workers);
    seams.colours(views, thumbnails, workers);
    seams.gains(views.size());
    seams.applyGains();
    seams.labels();
    seams.shares(workers);
    return seams;
  }

  /** World centre of seam cell (i, j). */
  double cx(int i) {
    return x0 + (i + .5) * cellPixels * pixel;
  }

  double cy(int j) {
    return yTop - (j + .5) * cellPixels * pixel;
  }

  private void candidates(Orthomosaic.Surface surface, List<Orthomosaic.View> views, double maxTan2, int workers) {
    heights = new float[gw * gh];
    int[][] rowViews = new int[gh][];
    float[][] rowCosts = new float[gh][];
    int[][] rowCounts = new int[gh][];
    Parallel.forEach(gh, workers, j -> {
      double y = cy(j);
      List<Integer> strip = new ArrayList<>();
      for (int v = 0; v < views.size(); v++) {
        double[] r = views.get(v).reach;
        if (r[1] <= y && r[3] >= y) strip.add(v);
      }
      int[] ids = new int[gw * CANDIDATES];
      float[] costs = new float[gw * CANDIDATES];
      int[] counts = new int[gw];
      double[] uv = new double[2];
      int[] bestView = new int[CANDIDATES];
      double[] bestCost = new double[CANDIDATES];
      for (int i = 0; i < gw; i++) {
        double x = cx(i), z = surface.height(x, y);
        heights[j * gw + i] = (float) z;
        if (Double.isNaN(z)) continue;
        int n = 0;
        for (int v : strip) {
          Orthomosaic.View view = views.get(v);
          double[] r = view.reach;
          if (r[0] > x || r[2] < x) continue;
          if (!view.project(x, y, z, uv) || uv[0] < 0 || uv[1] < 0 || uv[0] > view.width - 1 || uv[1] > view.height - 1) continue;
          double dx = x - view.centre[0], dy = y - view.centre[1], dz = view.centre[2] - z;
          if (!(dz > 0)) continue;
          double t2 = (dx * dx + dy * dy) / (dz * dz);
          if (!(t2 <= maxTan2)) continue;
          // Insert into the sorted best list (ties: the lower view index stays first).
          if (n == CANDIDATES && !(t2 < bestCost[n - 1])) continue;
          int at = Math.min(n, CANDIDATES - 1);
          if (n < CANDIDATES) n++;
          while (at > 0 && bestCost[at - 1] > t2) {
            bestCost[at] = bestCost[at - 1];
            bestView[at] = bestView[at - 1];
            at--;
          }
          bestCost[at] = t2;
          bestView[at] = v;
        }
        // Stored by view index so that lookups are a scan of a short sorted list.
        Integer[] boxed = new Integer[n];
        for (int k = 0; k < n; k++) boxed[k] = k;
        Arrays.sort(boxed, Comparator.comparingInt(k -> bestView[k]));
        for (int k = 0; k < n; k++) {
          ids[i * CANDIDATES + k] = bestView[boxed[k]];
          costs[i * CANDIDATES + k] = (float) bestCost[boxed[k]];
        }
        counts[i] = n;
      }
      rowViews[j] = ids;
      rowCosts[j] = costs;
      rowCounts[j] = counts;
    });
    int total = gw * gh;
    start = new int[total + 1];
    for (int j = 0; j < gh; j++)
      for (int i = 0; i < gw; i++) start[j * gw + i + 1] = start[j * gw + i] + rowCounts[j][i];
    view = new short[start[total]];
    obliquity = new float[start[total]];
    for (int j = 0; j < gh; j++)
      for (int i = 0; i < gw; i++) {
        int c = j * gw + i, at = start[c];
        for (int k = 0; k < rowCounts[j][i]; k++) {
          view[at + k] = (short) rowViews[j][i * CANDIDATES + k];
          obliquity[at + k] = rowCosts[j][i * CANDIDATES + k];
        }
      }
    cells = 0;
    for (int c = 0; c < total; c++) if (start[c + 1] > start[c]) cells++;
    candidatesTotal = start[total];
  }

  /** Reduced, box-filtered photographs for the seam colours. */
  interface Thumbnails {
    /** RGB pixels of view's width x height photograph reduced by step: ceil(width / step) x ceil(height / step), or null. */
    int[] load(int view, int step, int width, int height) throws IOException;
  }

  private void colours(List<Orthomosaic.View> views, Thumbnails thumbnails, int workers) throws IOException {
    colour = new int[view.length];
    // Slots of each view (inverse of the candidate lists), in cell order.
    int[] count = new int[views.size() + 1];
    for (short v : view) count[v + 1]++;
    for (int v = 0; v < views.size(); v++) count[v + 1] += count[v];
    int[] slots = new int[view.length], fill = count.clone();
    for (int c = 0; c < gw * gh; c++)
      for (int k = start[c]; k < start[c + 1]; k++) slots[fill[view[k]]++] = k;
    int[] cellOf = new int[view.length];
    for (int c = 0; c < gw * gh; c++) for (int k = start[c]; k < start[c + 1]; k++) cellOf[k] = c;
    IOException[] failure = new IOException[1];
    Parallel.forEach(views.size(), Math.min(workers, 8), v -> {
      if (count[v + 1] == count[v]) return;
      Orthomosaic.View photo = views.get(v);
      // Thumbnail pixel about a quarter of a seam cell; colours are means over a seam cell.
      int step = Math.max(1, cellPixels / 4);
      int[] small;
      try {
        small = thumbnails.load(v, step, photo.width, photo.height);
      } catch (IOException e) {
        synchronized (failure) {
          failure[0] = e;
        }
        return;
      }
      if (small == null) return;
      int tw = Math.ceilDiv(photo.width, step), th = Math.ceilDiv(photo.height, step);
      int box = Math.max(1, (int) Math.round(cellPixels / (2.0 * step)));
      long[][] integral = integral(small, tw, th);
      double[] uv = new double[2];
      for (int k = count[v]; k < count[v + 1]; k++) {
        int slot = slots[k], c = cellOf[slot], i = c % gw, j = c / gw;
        double x = cx(i), y = cy(j);
        double z = heightOf(c);
        if (!photo.project(x, y, z, uv)) continue;
        int u = (int) Math.floor(uv[0] / step), w = (int) Math.floor(uv[1] / step);
        colour[slot] = boxMean(integral, tw, th, u - box, w - box, u + box, w + box);
      }
    });
    if (failure[0] != null) throw failure[0];
  }

  /** Surface heights at the cell centres, kept from candidates() for colours(). */
  private float[] heights;

  private double heightOf(int c) {
    return heights[c];
  }

  private static long[][] integral(int[] rgb, int w, int h) {
    long[][] sum = new long[3][(w + 1) * (h + 1)];
    for (int y = 0; y < h; y++)
      for (int ch = 0; ch < 3; ch++) {
        long row = 0;
        int shift = 16 - 8 * ch;
        for (int x = 0; x < w; x++) {
          row += (rgb[y * w + x] >> shift) & 255;
          sum[ch][(y + 1) * (w + 1) + x + 1] = sum[ch][y * (w + 1) + x + 1] + row;
        }
      }
    return sum;
  }

  private static int boxMean(long[][] sum, int w, int h, int xa, int ya, int xb, int yb) {
    xa = Math.max(0, xa);
    ya = Math.max(0, ya);
    xb = Math.min(w - 1, xb);
    yb = Math.min(h - 1, yb);
    if (xa > xb || ya > yb) return 0;
    long n = (long) (xb - xa + 1) * (yb - ya + 1);
    int out = 0;
    for (int ch = 0; ch < 3; ch++) {
      long[] s = sum[ch];
      long v = s[(yb + 1) * (w + 1) + xb + 1] - s[ya * (w + 1) + xb + 1] - s[(yb + 1) * (w + 1) + xa] + s[ya * (w + 1) + xa];
      out |= (int) Math.min(255, Math.round((double) v / n)) << (16 - 8 * ch);
    }
    return out;
  }

  /**
   * Gains per view and channel minimising, over pairs of views sharing a cell, (g_a I_a - g_b
   * I_b)^2 / SIGMA_N^2 + (1 - g)^2 / SIGMA_G^2 (Brown and Lowe 2007), by Gauss-Seidel sweeps in
   * view order (deterministic).
   */
  static final double SIGMA_N = 10, SIGMA_G = .1;

  private void gains(int views) {
    // Pair statistics: count and colour sums of a in cells shared with b, keyed by (a, b).
    HashMap<Long, double[]> pairs = new HashMap<>();
    for (int c = 0; c < gw * gh; c++) {
      int a0 = start[c], a1 = Math.min(start[c + 1], a0 + 6);
      for (int p = a0; p < a1; p++)
        for (int q = a0; q < a1; q++) {
          if (p == q) continue;
          long key = (long) view[p] << 32 | (view[q] & 0xffffffffL);
          double[] s = pairs.computeIfAbsent(key, k -> new double[4]);
          s[0]++;
          for (int ch = 0; ch < 3; ch++) s[1 + ch] += (colour[p] >> (16 - 8 * ch)) & 255;
        }
    }
    // Normal equations per view: sum over b of N_ab [(I_ab^2 + ...) ] -> Gauss-Seidel.
    List<Long> keys = new ArrayList<>(pairs.keySet());
    Collections.sort(keys);
    int[] begin = new int[views + 1];
    for (long k : keys) begin[(int) (k >>> 32) + 1]++;
    for (int v = 0; v < views; v++) begin[v + 1] += begin[v];
    for (int ch = 0; ch < 3; ch++) {
      double[] g = new double[views];
      Arrays.fill(g, 1);
      for (int sweep = 0; sweep < 100; sweep++) {
        double change = 0;
        for (int a = 0; a < views; a++) {
          if (begin[a + 1] == begin[a]) continue;
          double diagonal = 0, rhs = 0;
          for (int k = begin[a]; k < begin[a + 1]; k++) {
            long key = keys.get(k);
            int b = (int) key;
            double[] ab = pairs.get(key), ba = pairs.get((long) b << 32 | (a & 0xffffffffL));
            double n = ab[0], iab = ab[1 + ch] / n, iba = ba[1 + ch] / ba[0];
            diagonal += n * (2 * iab * iab / (SIGMA_N * SIGMA_N) + 1 / (SIGMA_G * SIGMA_G));
            rhs += n * (2 * iab * iba * g[b] / (SIGMA_N * SIGMA_N) + 1 / (SIGMA_G * SIGMA_G));
          }
          double next = diagonal > 0 ? rhs / diagonal : 1;
          change = Math.max(change, Math.abs(next - g[a]));
          g[a] = next;
        }
        if (change < 1e-6) break;
      }
      for (int v = 0; v < views; v++) gain[v][ch] = g[v];
    }
  }

  private void applyGains() {
    for (int k = 0; k < view.length; k++) colour[k] = scale(colour[k], gain[view[k]]);
  }

  static int scale(int rgb, double[] g) {
    int out = 0;
    for (int ch = 0; ch < 3; ch++) {
      int shift = 16 - 8 * ch;
      out |= (int) Math.min(255, Math.max(0, Math.round(((rgb >> shift) & 255) * g[ch]))) << shift;
    }
    return out;
  }

  /** Slot of view v among cell c's candidates, or -1. */
  private int slot(int c, int v) {
    for (int k = start[c]; k < start[c + 1]; k++) {
      if (view[k] == v) return k;
      if (view[k] > v) return -1;
    }
    return -1;
  }

  private static int difference(int a, int b) {
    return Math.abs(((a >> 16) & 255) - ((b >> 16) & 255)) + Math.abs(((a >> 8) & 255) - ((b >> 8) & 255)) + Math.abs((a & 255) - (b & 255));
  }

  /** Seam cost between adjacent cells p and q labelled a and b. */
  private double seam(int p, int q, int a, int b) {
    if (a == b) return 0;
    double sum = 0;
    int n = 0;
    for (int c : new int[] {p, q}) {
      int sa = slot(c, a), sb = slot(c, b);
      if (sa < 0 || sb < 0) continue;
      sum += difference(colour[sa], colour[sb]) / 3.0;
      n++;
    }
    return SEAM + (n == 0 ? OUTSIDE : sum / n);
  }

  private double data(int c, int v) {
    int k = slot(c, v);
    return k < 0 ? Double.POSITIVE_INFINITY : OBLIQUITY * obliquity[k];
  }

  private double energy() {
    double e = 0;
    for (int c = 0; c < gw * gh; c++) {
      if (label[c] < 0) continue;
      e += data(c, label[c]);
      int i = c % gw;
      if (i + 1 < gw && label[c + 1] >= 0) e += seam(c, c + 1, label[c], label[c + 1]);
      if (c + gw < gw * gh && label[c + gw] >= 0) e += seam(c, c + gw, label[c], label[c + gw]);
    }
    return e;
  }

  private void labels() {
    int total = gw * gh;
    label = new int[total];
    for (int c = 0; c < total; c++) {
      label[c] = -1;
      double best = Double.POSITIVE_INFINITY;
      for (int k = start[c]; k < start[c + 1]; k++)
        if (obliquity[k] < best) {
          best = obliquity[k];
          label[c] = view[k];
        }
    }
    energyBefore = energy();
    int views = gain.length;
    // Cells per view (where the view is a candidate), in cell order.
    int[] count = new int[views + 1];
    for (short v : view) count[v + 1]++;
    for (int v = 0; v < views; v++) count[v + 1] += count[v];
    int[] cellsOf = new int[view.length], fill = count.clone();
    for (int c = 0; c < total; c++) for (int k = start[c]; k < start[c + 1]; k++) cellsOf[fill[view[k]]++] = c;
    int[] node = new int[total];
    Arrays.fill(node, -1);
    for (int cycle = 0; cycle < CYCLES; cycle++) {
      long changed = 0;
      for (int alpha = 0; alpha < views; alpha++) {
        int n = count[alpha + 1] - count[alpha];
        if (n == 0) continue;
        for (int k = 0; k < n; k++) node[cellsOf[count[alpha] + k]] = k;
        changed += expand(alpha, cellsOf, count[alpha], n, node);
        for (int k = 0; k < n; k++) node[cellsOf[count[alpha] + k]] = -1;
      }
      expansionsChanged += changed;
      if (changed == 0) break;
    }
    energyAfter = energy();
  }

  /** One alpha-expansion over the cells where alpha is a candidate; returns the cells changed. */
  private long expand(int alpha, int[] cellsOf, int offset, int n, int[] node) {
    GraphCut cut = new GraphCut(n, 2 * n);
    int[][] steps = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
    for (int k = 0; k < n; k++) {
      int c = cellsOf[offset + k], i = c % gw, j = c / gw, f = label[c];
      double keep = f < 0 ? OUTSIDE * 4 : data(c, f), take = data(c, alpha);
      if (f == alpha) keep = take;
      cut.unary(k, keep, take);
      for (int[] d : steps) {
        int ii = i + d[0], jj = j + d[1];
        if (ii < 0 || jj < 0 || ii >= gw || jj >= gh) continue;
        int q = jj * gw + ii, g = label[q];
        if (node[q] >= 0) {
          // Both variables: each pair once (towards +x and +y).
          if (d[0] < 0 || d[1] < 0) continue;
          double a = f < 0 || g < 0 ? 0 : seam(c, q, f, g), b = f < 0 ? 0 : seam(c, q, f, alpha), cc = g < 0 ? 0 : seam(c, q, alpha, g), dd = 0;
          if (a + dd > b + cc) a = b + cc - dd;
          cut.pairwise(k, node[q], a, b, cc, dd);
        } else if (g >= 0) {
          // Fixed neighbour.
          cut.unary(k, f < 0 ? 0 : seam(c, q, f, g), seam(c, q, alpha, g));
        }
      }
    }
    cut.maxflow();
    long changed = 0;
    for (int k = 0; k < n; k++) {
      int c = cellsOf[offset + k];
      if (cut.label(k) == 1 && label[c] != alpha) {
        label[c] = alpha;
        changed++;
      }
    }
    return changed;
  }

  /** Blend shares: label counts within radius cells, the SHARES largest, normalised. */
  private void shares(int workers) {
    int total = gw * gh;
    shareView = new short[total * SHARES];
    shareWeight = new float[total * SHARES];
    Arrays.fill(shareView, (short) -1);
    Parallel.forEach(gh, workers, j -> {
      int[] ids = new int[(2 * radius + 1) * (2 * radius + 1)], counts = new int[ids.length];
      for (int i = 0; i < gw; i++) {
        int c = j * gw + i;
        if (label[c] < 0) continue;
        int n = 0;
        for (int jj = Math.max(0, j - radius); jj <= Math.min(gh - 1, j + radius); jj++)
          for (int ii = Math.max(0, i - radius); ii <= Math.min(gw - 1, i + radius); ii++) {
            int l = label[jj * gw + ii];
            if (l < 0 || slot(c, l) < 0) continue;
            int at = 0;
            while (at < n && ids[at] != l) at++;
            if (at == n) {
              ids[n] = l;
              counts[n++] = 0;
            }
            counts[at]++;
          }
        // Largest counts first; equal: lower view index.
        for (int a = 0; a < Math.min(n, SHARES); a++) {
          int best = a;
          for (int b = a + 1; b < n; b++)
            if (counts[b] > counts[best] || (counts[b] == counts[best] && ids[b] < ids[best])) best = b;
          int ti = ids[a], tc = counts[a];
          ids[a] = ids[best];
          counts[a] = counts[best];
          ids[best] = ti;
          counts[best] = tc;
        }
        int kept = Math.min(n, SHARES), sum = 0;
        for (int a = 0; a < kept; a++) sum += counts[a];
        for (int a = 0; a < kept; a++) {
          shareView[c * SHARES + a] = (short) ids[a];
          shareWeight[c * SHARES + a] = counts[a] / (float) sum;
        }
      }
    });
    long edges = 0;
    for (int c = 0; c < total; c++) {
      if (c % gw + 1 < gw && label[c] >= 0 && label[c + 1] >= 0 && label[c] != label[c + 1]) edges++;
      if (c + gw < total && label[c] >= 0 && label[c + gw] >= 0 && label[c] != label[c + gw]) edges++;
    }
    seamEdges = edges;
  }

  /** Weight of view v at mosaic pixel (px, py) (pixel centres), interpolated between seam cells. */
  double weight(int v, int px, int py) {
    double gx = (px + .5) / cellPixels - .5, gy = (py + .5) / cellPixels - .5;
    int ix = (int) Math.floor(gx), iy = (int) Math.floor(gy);
    double fx = gx - ix, fy = gy - iy, sum = 0;
    for (int dy = 0; dy <= 1; dy++)
      for (int dx = 0; dx <= 1; dx++) {
        int i = Math.max(0, Math.min(gw - 1, ix + dx)), j = Math.max(0, Math.min(gh - 1, iy + dy));
        double w = (dx == 0 ? 1 - fx : fx) * (dy == 0 ? 1 - fy : fy);
        if (w == 0) continue;
        int c = j * gw + i;
        for (int a = 0; a < SHARES; a++) {
          int s = shareView[c * SHARES + a];
          if (s < 0) break;
          if (s == v) {
            sum += w * shareWeight[c * SHARES + a];
            break;
          }
        }
      }
    return sum;
  }

  /** Debug image of the seam grid: one colour per photograph, seams black (-Dsgm.orthomosaic.seams=PNG). */
  void writeDebug(Path png) throws IOException {
    BufferedImage image = new BufferedImage(gw, gh, BufferedImage.TYPE_INT_RGB);
    for (int j = 0; j < gh; j++)
      for (int i = 0; i < gw; i++) {
        int c = j * gw + i, l = label[c];
        int rgb = l < 0 ? 0xff00ff : (int) ((l * 2654435761L) & 0xffffff) | 0x404040;
        boolean edge = l >= 0 && ((i + 1 < gw && label[c + 1] >= 0 && label[c + 1] != l) || (j + 1 < gh && label[c + gw] >= 0 && label[c + gw] != l));
        image.setRGB(i, j, edge ? 0 : rgb);
      }
    ImageIO.write(image, "png", png.toFile());
  }

  /** Views with a share in any of the cells covering mosaic pixels [px0, px1) x [py0, py1). */
  boolean[] used(int px0, int py0, int px1, int py1, int views) {
    boolean[] used = new boolean[views];
    int i0 = Math.max(0, px0 / cellPixels - 1), i1 = Math.min(gw - 1, (px1 - 1) / cellPixels + 1);
    int j0 = Math.max(0, py0 / cellPixels - 1), j1 = Math.min(gh - 1, (py1 - 1) / cellPixels + 1);
    for (int j = j0; j <= j1; j++)
      for (int i = i0; i <= i1; i++)
        for (int a = 0; a < SHARES; a++) {
          int s = shareView[(j * gw + i) * SHARES + a];
          if (s < 0) break;
          used[s] = true;
        }
    return used;
  }

  /** Thumbnails of the run's photographs (subsampled decode, then box mean of step x step). */
  static Thumbnails files(List<Path> files) {
    return (v, step, width, height) -> {
      Path file = files.get(v);
      int sub = Math.max(1, step / 2);
      BufferedImage image = null;
      try (ImageInputStream input = ImageIO.createImageInputStream(file.toFile())) {
        Iterator<ImageReader> readers = input == null ? Collections.emptyIterator() : ImageIO.getImageReaders(input);
        if (readers.hasNext()) {
          ImageReader reader = readers.next();
          try {
            reader.setInput(input, true, true);
            ImageReadParam param = reader.getDefaultReadParam();
            param.setSourceSubsampling(sub, sub, 0, 0);
            image = reader.read(0, param);
          } catch (IOException | RuntimeException unreadable) {
            image = null;
          } finally {
            reader.dispose();
          }
        }
      }
      int[] base;
      int bw, bh;
      if (image != null) {
        bw = image.getWidth();
        bh = image.getHeight();
        base = image.getRGB(0, 0, bw, bh, null, 0, bw);
      } else {
        try (WindowedImage.Source source = new WindowedImage.Source(file)) {
          int fullW = source.width();
          int fullH = source.height();
          int[] all = source.window(0, 0, fullW, fullH);
          bw = Math.ceilDiv(fullW, sub);
          bh = Math.ceilDiv(fullH, sub);
          base = new int[bw * bh];
          for (int y = 0; y < bh; y++) for (int x = 0; x < bw; x++) base[y * bw + x] = all[y * sub * fullW + x * sub];
        }
      }
      // Thumbnail pixel (x, y) = mean of the base pixels whose full-resolution position falls in
      // [x * step, (x + 1) * step) x [y * step, (y + 1) * step).
      int tw = Math.ceilDiv(width, step), th = Math.ceilDiv(height, step);
      long[] sums = new long[4 * tw * th];
      for (int yy = 0; yy < bh; yy++) {
        int ty = Math.min(th - 1, yy * sub / step);
        for (int xx = 0; xx < bw; xx++) {
          int tx = Math.min(tw - 1, xx * sub / step), p = base[yy * bw + xx], at = 4 * (ty * tw + tx);
          sums[at] += (p >> 16) & 255;
          sums[at + 1] += (p >> 8) & 255;
          sums[at + 2] += p & 255;
          sums[at + 3]++;
        }
      }
      int[] out = new int[tw * th];
      for (int t = 0; t < out.length; t++) {
        long n = sums[4 * t + 3];
        if (n > 0) out[t] = (int) (sums[4 * t] / n) << 16 | (int) (sums[4 * t + 1] / n) << 8 | (int) (sums[4 * t + 2] / n);
      }
      return out;
    };
  }
}
