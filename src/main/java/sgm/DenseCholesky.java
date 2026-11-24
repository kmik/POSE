package sgm;

import java.util.*;

/**
 * In-place tiled Cholesky factorization A = L L^T of a symmetric positive definite matrix stored
 * row-major in one array (--bundle-direct-limit). Only the lower triangle is read; it is
 * overwritten with L, the strict upper triangle is left as it was.
 *
 * <p>Right-looking over tile columns: factor the diagonal tile, solve the tiles below it, then
 * update the trailing tiles. Tiles of L that are zero and receive no fill are skipped (the reduced
 * camera matrix of a long block is sparse). Every element receives its updates in one fixed order
 * (tile column by tile column, inside a tile column by column), whatever the number of threads:
 * the parallel tasks own disjoint tiles. The factor is therefore identical for any --cores.
 */
final class DenseCholesky {
  private DenseCholesky() {}

  static final int TILE = 64;

  /** Smallest accepted pivot of a matrix scaled to a unit diagonal (OpenCV's Cholesky test). */
  static final double PIVOT = 2.220446049250313e-16;

  /** Largest order whose n^2 elements fit one Java array. */
  static final int MAX_ORDER = 46340;

  /** Per-thread transposed-tile scratch of {@link #update} (fully written before it is read). */
  private static final ThreadLocal<double[]> SCRATCH = ThreadLocal.withInitial(() -> new double[TILE * TILE]);

  /**
   * Factors a[0 .. n^2) in place. Parallel over tiles when parallel is true. False when a pivot
   * is below {@link #PIVOT} or not finite (the matrix is then partly overwritten).
   */
  static boolean factor(double[] a, int n, boolean parallel) {
    if (n <= 0 || n > MAX_ORDER) return false;
    int tiles = (n + TILE - 1) / TILE;
    // nonzero[i][j] (i >= j): the tile holds a nonzero (or not yet known zero) lower element.
    boolean[][] nonzero = new boolean[tiles][];
    for (int i = 0; i < tiles; i++) {
      nonzero[i] = new boolean[i + 1];
      for (int j = 0; j <= i; j++) nonzero[i][j] = i == j || anyNonzero(a, n, i, j);
    }
    int[] pending = new int[tiles];
    int[][] pairs = new int[tiles * (tiles + 1) / 2][];
    for (int k = 0; k < tiles; k++) {
      int k0 = k * TILE, k1 = Math.min(n, k0 + TILE);
      if (!factorDiagonal(a, n, k0, k1)) return false;
      int below = 0;
      for (int i = k + 1; i < tiles; i++) if (nonzero[i][k]) pending[below++] = i;
      final int panels = below;
      run(parallel, panels, task -> {
        int i = pending[task], i0 = i * TILE, i1 = Math.min(n, i0 + TILE);
        solvePanel(a, n, i0, i1, k0, k1);
      });
      int count = 0;
      for (int x = 0; x < panels; x++)
        for (int y = 0; y <= x; y++) pairs[count++] = new int[] {pending[x], pending[y]};
      for (int x = 0; x < count; x++) nonzero[pairs[x][0]][pairs[x][1]] = true;
      final int updates = count;
      run(parallel, updates, task -> {
        int i = pairs[task][0], j = pairs[task][1];
        double[] t = SCRATCH.get();
        update(a, n, i * TILE, Math.min(n, i * TILE + TILE), j * TILE, Math.min(n, j * TILE + TILE),
            k0, k1, t, i == j);
      });
    }
    return true;
  }

  private static void run(boolean parallel, int n, java.util.function.IntConsumer body) {
    if (parallel) BundleAdjuster.parallel(n, body);
    else for (int i = 0; i < n; i++) body.accept(i);
  }

  private static boolean anyNonzero(double[] a, int n, int ti, int tj) {
    int i0 = ti * TILE, i1 = Math.min(n, i0 + TILE), j0 = tj * TILE, j1 = Math.min(n, j0 + TILE);
    for (int r = i0; r < i1; r++)
      for (int c = j0, row = r * n; c < j1; c++) if (a[row + c] != 0) return true;
    return false;
  }

  /** Unblocked Cholesky of the diagonal tile [k0, k1) (its earlier updates already applied). */
  private static boolean factorDiagonal(double[] a, int n, int k0, int k1) {
    for (int c = k0; c < k1; c++) {
      int crow = c * n;
      double s = a[crow + c];
      for (int m = k0; m < c; m++) s -= a[crow + m] * a[crow + m];
      if (!(s >= PIVOT) || !Double.isFinite(s)) return false;
      double d = Math.sqrt(s);
      a[crow + c] = d;
      for (int r = c + 1; r < k1; r++) {
        int row = r * n;
        double v = a[row + c];
        for (int m = k0; m < c; m++) v -= a[row + m] * a[crow + m];
        a[row + c] = v / d;
      }
    }
    return true;
  }

  /** Rows [i0, i1) of tile column [k0, k1): A_ik L_kk^-T, row by row (forward substitution). */
  private static void solvePanel(double[] a, int n, int i0, int i1, int k0, int k1) {
    for (int r = i0; r < i1; r++) {
      int row = r * n;
      for (int c = k0; c < k1; c++) {
        int crow = c * n;
        double v = a[row + c];
        for (int m = k0; m < c; m++) v -= a[row + m] * a[crow + m];
        a[row + c] = v / a[crow + c];
      }
    }
  }

  /**
   * A_ij -= A_ik A_jk^T for rows [i0, i1) and columns [j0, j1) (lower part only on a diagonal
   * tile), products subtracted one at a time in increasing m. t: TILE^2 scratch.
   */
  private static void update(double[] a, int n, int i0, int i1, int j0, int j1, int k0, int k1,
      double[] t, boolean diagonal) {
    int kb = k1 - k0, jb = j1 - j0;
    for (int c = 0; c < jb; c++) {
      int row = (j0 + c) * n + k0;
      for (int m = 0; m < kb; m++) t[m * jb + c] = a[row + m];
    }
    for (int r = i0; r < i1; r++) {
      int row = r * n, end = diagonal ? r - j0 + 1 : jb, out = row + j0;
      for (int m = 0; m < kb; m++) {
        double v = a[row + k0 + m];
        if (v == 0) continue;
        int tb = m * jb;
        for (int c = 0; c < end; c++) a[out + c] -= v * t[tb + c];
      }
    }
  }

  /** x = (L L^T)^-1 b for the factor of {@link #factor}; x may be b. Serial. */
  static void solve(double[] l, int n, double[] b, double[] x) {
    for (int i = 0; i < n; i++) {
      int row = i * n;
      double s = b[i];
      for (int m = 0; m < i; m++) s -= l[row + m] * x[m];
      x[i] = s / l[row + i];
    }
    // L^T x = y, column-oriented: row i of L is contiguous.
    for (int i = n - 1; i >= 0; i--) {
      int row = i * n;
      double v = x[i] / l[row + i];
      x[i] = v;
      for (int m = 0; m < i; m++) x[m] -= l[row + m] * v;
    }
  }

  /**
   * Diagonal of (L L^T)^-1: entry i is the sum over columns j of (L^-1)_ij^2. Column j of L^-1
   * solves L z = e_j from row j on. Columns are processed in fixed chunks (by n, never by the
   * thread count), each summing into its own array, and the chunks are added in order: the
   * result does not depend on the number of threads.
   */
  static double[] inverseDiagonal(double[] l, int n, boolean parallel) {
    int chunks = Math.max(1, Math.min(64, n / 32));
    double[][] partial = new double[chunks][n];
    run(parallel, chunks, chunk -> {
      double[] z = new double[n], sum = partial[chunk];
      int from = (int) ((long) n * chunk / chunks), to = (int) ((long) n * (chunk + 1) / chunks);
      for (int j = from; j < to; j++) {
        // Forward substitution from row j (rows above are zero).
        for (int i = j; i < n; i++) {
          int row = i * n;
          double s = i == j ? 1 : 0;
          for (int m = j; m < i; m++) s -= l[row + m] * z[m];
          z[i] = s / l[row + i];
          sum[i] += z[i] * z[i];
        }
      }
    });
    double[] out = new double[n];
    for (double[] p : partial) for (int i = 0; i < n; i++) out[i] += p[i];
    return out;
  }

  /**
   * (L_tt L_tt^T)^-1 for the trailing w x w block of a factor: the inverse of the Schur
   * complement of the leading variables, i.e. the marginal covariance block of the last w
   * variables of the factored matrix. Null when it is singular.
   */
  static double[] trailingInverse(double[] l, int n, int w) {
    int o = n - w;
    double[] block = new double[w * w];
    for (int i = 0; i < w; i++)
      for (int j = 0; j <= i; j++) block[i * w + j] = l[(o + i) * n + o + j];
    double[] inverse = new double[w * w], e = new double[w];
    for (int k = 0; k < w; k++) {
      Arrays.fill(e, 0);
      e[k] = 1;
      solve(block, w, e, e);
      for (int i = 0; i < w; i++) {
        if (!Double.isFinite(e[i])) return null;
        inverse[i * w + k] = e[i];
      }
    }
    return inverse;
  }
}
