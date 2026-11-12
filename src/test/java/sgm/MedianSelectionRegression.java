package sgm;

import java.util.*;

/**
 * CensusSgm.middle against Arrays.sort, bit for bit: every window size the median filters use
 * (1 to 49 values), negative values, both zeros, repeated values and infinities excluded as the
 * filters exclude them; then medianValid and medianEdgePreserving on whole maps against a
 * sort-based copy of the former code.
 */
final class MedianSelectionRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) {
    System.out.println("PASS median selection: " + run());
    if (args.length > 0) timing();
  }

  private static float value(Random random, int kind) {
    return switch (kind) {
      case 0 -> (float) (random.nextGaussian() * 20);
      case 1 -> random.nextInt(5) - 2;
      case 2 -> random.nextBoolean() ? 0f : -0f;
      default -> switch (random.nextInt(6)) {
        case 0 -> 0f;
        case 1 -> -0f;
        case 2 -> Float.MIN_VALUE;
        case 3 -> -Float.MIN_VALUE;
        default -> (float) random.nextGaussian();
      };
    };
  }

  static int run() {
    checks = 0;
    Random random = new Random(7);
    long differ = 0, zeros = 0;
    for (int n = 1; n <= 49; n++)
      for (int trial = 0; trial < 4000; trial++) {
        float[] a = new float[n];
        int kind = trial % 4;
        for (int i = 0; i < n; i++) a[i] = value(random, kind);
        float[] sorted = a.clone();
        Arrays.sort(sorted);
        float expected = sorted[n / 2], got = CensusSgm.middle(a, n);
        if (Float.floatToRawIntBits(expected) != Float.floatToRawIntBits(got)) differ++;
        if (expected == 0) zeros++;
      }
    check(differ == 0, differ + " middle values differ from Arrays.sort");
    check(zeros > 10_000, "too few zero medians exercised: " + zeros);
    int w = 310, h = 190;
    for (int window : new int[] {3, 5, 7})
      for (int seed = 0; seed < 3; seed++) {
        Random r = new Random(seed * 13 + window);
        float[] map = new float[w * h];
        byte[] gray = new byte[w * h];
        for (int y = 0; y < h; y++)
          for (int x = 0; x < w; x++) {
            int p = y * w + x;
            boolean upper = x + y / 2 > 200;
            double v = (upper ? 12 : -3) + .02 * x + r.nextGaussian() * (seed == 0 ? .1 : 1.5);
            if (seed == 2) v = Math.round(v) * (r.nextBoolean() ? 0.0 : 1.0) * (r.nextBoolean() ? -1 : 1);
            map[p] = r.nextDouble() < .1 ? Float.NaN : r.nextDouble() < .01 ? Float.POSITIVE_INFINITY : (float) v;
            gray[p] = (byte) ((upper ? 170 : 50) + r.nextInt(30));
          }
        check(same(CensusSgm.medianValid(map, w, h, window), referenceValid(map, w, h, window)),
            "medianValid differs (window " + window + ", seed " + seed + ")");
        check(same(CensusSgm.medianEdgePreserving(map, w, h, window, .5, gray, seed == 1 ? 0 : 24),
                referenceEdge(map, w, h, window, .5, gray, seed == 1 ? 0 : 24)),
            "medianEdgePreserving differs (window " + window + ", seed " + seed + ")");
      }
    return checks;
  }

  private static boolean same(float[] a, float[] b) {
    for (int i = 0; i < a.length; i++) if (Float.floatToRawIntBits(a[i]) != Float.floatToRawIntBits(b[i])) return false;
    return a.length == b.length;
  }

  /** The former medianValid: full sort per window. */
  private static float[] referenceValid(float[] a, int w, int h, int window) {
    float[] out = a.clone(), values = new float[window * window];
    int r = window / 2;
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) {
        if (!Float.isFinite(a[y * w + x])) continue;
        int n = 0;
        for (int yy = Math.max(0, y - r); yy <= Math.min(h - 1, y + r); yy++)
          for (int xx = Math.max(0, x - r); xx <= Math.min(w - 1, x + r); xx++) {
            float v = a[yy * w + xx];
            if (Float.isFinite(v)) values[n++] = v;
          }
        Arrays.sort(values, 0, n);
        out[y * w + x] = values[n / 2];
      }
    return out;
  }

  /** The former medianEdgePreserving. */
  private static float[] referenceEdge(float[] a, int w, int h, int window, double maximumChange, byte[] gray, int edgeThreshold) {
    float[] out = referenceValid(a, w, h, window), values = new float[window * window];
    int r = window / 2;
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) {
        float center = a[y * w + x];
        if (!Float.isFinite(center)) continue;
        if (Math.abs(out[y * w + x] - center) < maximumChange) continue;
        int n = 0;
        for (int yy = Math.max(0, y - r); yy <= Math.min(h - 1, y + r); yy++)
          for (int xx = Math.max(0, x - r); xx <= Math.min(w - 1, x + r); xx++) {
            float value = a[yy * w + xx];
            if (Float.isFinite(value)
                && Math.abs(value - center) <= maximumChange
                && (edgeThreshold == 0 || Math.abs((gray[yy * w + xx] & 255) - (gray[y * w + x] & 255)) <= edgeThreshold))
              values[n++] = value;
          }
        if (n > 0) {
          Arrays.sort(values, 0, n);
          out[y * w + x] = values[n / 2];
        }
      }
    return out;
  }

  /** Milliseconds per 2000 x 1125 map on one thread, former and current code (not a check). */
  private static void timing() {
    int w = 2000, h = 1125;
    Random r = new Random(1);
    float[] map = new float[w * h];
    for (int i = 0; i < map.length; i++) map[i] = r.nextDouble() < .05 ? Float.NaN : (float) (30 + .01 * (i % w) + r.nextGaussian());
    for (int window : new int[] {3, 5})
      for (int round = 0; round < 3; round++) {
        long t0 = System.nanoTime();
        for (int i = 0; i < 5; i++) referenceValid(map, w, h, window);
        long t1 = System.nanoTime();
        for (int i = 0; i < 5; i++) CensusSgm.medianValid(map, w, h, window);
        long t2 = System.nanoTime();
        System.out.printf(Locale.ROOT, "window %d: sort %.0f ms, current %.0f ms per map%n", window, (t1 - t0) / 5e6, (t2 - t1) / 5e6);
      }
  }
}
