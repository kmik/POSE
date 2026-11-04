package sgm;

/**
 * Shared fixed-point cost units. Costs span 0..(census area-1); penalties use 5x5 reference units.
 */
final class StereoCost {
  static int code(Options o) {
    return switch (o.matchingCost) {
      case "census" -> 0;
      case "ad-census" -> 1;
      case "census-gradient" -> 2;
      case "zncc" -> 3;
      default -> throw new IllegalArgumentException("Unknown matching cost");
    };
  }

  static int penalty(double value, Options o) {
    return (int) Math.round(value * (o.census * o.census - 1) / 24.);
  }

  static int p2(Options o) {
    return Math.max(penalty(o.p1, o) + 1, penalty(o.p2, o));
  }

  /**
   * Classic normalized recurrence guarantees 0 <= L <= max(C) + P2. All current costs are <= 120 or
   * the invalid sentinel 512; adaptation never increases P2. Sum is nonnegative and bounded by
   * directions * (512 + effective P2). Carry/local path arithmetic remains signed int for both
   * storage widths.
   */
  static long aggregateBound(Options o) {
    return o.directions * (512L + p2(o));
  }

  static int aggregateBytes(Options o) {
    boolean fits = aggregateBound(o) <= 65535;
    if (o.sgmAggregate.equals("16") && !fits)
      throw new IllegalArgumentException(
          "16-bit SGM aggregate cannot hold proved bound " + aggregateBound(o));
    return !o.sgmAggregate.equals("32") && fits ? 2 : 4;
  }

  /**
   * Subpixel offset in [-0.5, 0.5] from the winner's aggregated cost and its two neighbours. The
   * default parabola is the original formula; equiangular fits a symmetric V (Shimizu/Okutomi).
   * Both GPU and CPU selection call this, so they agree by construction.
   */
  static double subpixel(Options o, double before, double value, double after) {
    if (o.subpixelFit.equals("equiangular")) {
      double rise = Math.max(before, after) - value;
      return rise > 0 ? Math.max(-.5, Math.min(.5, .5 * (before - after) / rise)) : 0;
    }
    double curvature = before + after - 2.0 * value;
    return curvature > 0 ? Math.max(-.5, Math.min(.5, .5 * (before - after) / curvature)) : 0;
  }

  /** Integer edge-aware jump penalty, identical for CPU/GPU and strip-boundary predecessors. */
  static int pathP2(int p1, int p2, int scale, byte[] gray, int current, int previous) {
    if (scale == 0) return p2;
    int difference = Math.abs((gray[current] & 255) - (gray[previous] & 255));
    return Math.max(p1 + 1, (int) ((long) p2 * scale / (scale + difference)));
  }

  static int value(
      Options o, long[][] a, long[][] b, byte[] grayA, byte[] grayB, int p, int q, int width) {
    int hamming = 0;
    for (int word = 0; word < a.length; word++) hamming += Long.bitCount(a[word][p] ^ b[word][q]);
    int mode = code(o), bits = o.census * o.census - 1;
    if (mode == 0) return hamming;
    if (mode == 1 || mode == 2) {
      int difference, cap;
      if (mode == 1) {
        difference = Math.abs((grayA[p] & 255) - (grayB[q] & 255));
        cap = o.adTruncation;
      } else {
        int ax = (grayA[p + 1] & 255) - (grayA[p - 1] & 255),
            ay = (grayA[p + width] & 255) - (grayA[p - width] & 255);
        int bx = (grayB[q + 1] & 255) - (grayB[q - 1] & 255),
            by = (grayB[q + width] & 255) - (grayB[q - width] & 255);
        difference = Math.abs(ax - bx) + Math.abs(ay - by);
        cap = o.gradientTruncation;
      }
      int weight = (int) Math.round(o.censusWeight * 1000), denominator = 1000 * cap;
      return (weight * hamming * cap
              + (1000 - weight) * Math.min(difference, cap) * bits
              + denominator / 2)
          / denominator;
    }
    int radius = o.census / 2, n = o.census * o.census;
    long sa = 0, sb = 0, saa = 0, sbb = 0, sab = 0;
    for (int dy = -radius; dy <= radius; dy++)
      for (int dx = -radius; dx <= radius; dx++) {
        int x = grayA[p + dy * width + dx] & 255, y = grayB[q + dy * width + dx] & 255;
        sa += x;
        sb += y;
        saa += x * x;
        sbb += y * y;
        sab += x * y;
      }
    long va = n * saa - sa * sa, vb = n * sbb - sb * sb;
    if (va <= 0 || vb <= 0) return 512;
    double correlation = (n * sab - sa * sb) / Math.sqrt((double) va * vb);
    return (int) Math.round((1 - Math.max(-1, Math.min(1, correlation))) * .5 * bits);
  }
}
