package sgm;

import java.util.*;

/**
 * 14 November 2026 exact fusion fixes: the selection used for the surface-patch medians returns
 * the same value as a full sort (random arrays with many duplicates). The merge agreement matrix
 * and the pair array are covered by the byte-identical fusion outputs of the other suites.
 */
final class CrossCheckFixesRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) {
    System.out.println("PASS cross-check fixes=" + run());
  }

  static int run() {
    checks = 0;
    Random r = new Random(14);
    int same = 0, trials = 20000;
    for (int t = 0; t < trials; t++) {
      int n = 1 + r.nextInt(90), k = r.nextInt(n);
      double[] v = new double[n];
      for (int i = 0; i < n; i++)
        v[i] = r.nextInt(4) == 0 ? (r.nextInt(5) + 1) * .25 : r.nextDouble() * 10 + 1e-9;
      double[] sorted = v.clone();
      Arrays.sort(sorted);
      if (Double.doubleToRawLongBits(SurfacePatchGrid.kth(v, n, k)) == Double.doubleToRawLongBits(sorted[k]))
        same++;
    }
    check(same == trials, "kth selection equals the sorted element: " + same + "/" + trials);
    check(SurfacePatchGrid.kth(new double[] {3}, 1, 0) == 3, "kth of one value");
    check(SurfacePatchGrid.kth(new double[] {2, 2, 2, 1}, 4, 2) == 2, "kth with ties");
    return checks;
  }
}
