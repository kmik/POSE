package sgm;

import java.util.*;

/**
 * GPU edge-aware depth outlier decisions against the CPU code, every pixel: planes with a depth
 * edge, noise levels from flat to rough, isolated outliers, small blobs, a thin line, holes, gray
 * edges, radii 2..5, with and without a left/right error raster, and a core strip (windowed use).
 * Also checks that an auxiliary pool reservation never waits while the thread holds pool memory.
 * Skipped without a usable OpenCL FP64 device.
 */
final class GpuDepthOutlierRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) {
    nu.pattern.OpenCV.loadLocally();
    System.out.println("PASS GPU depth outlier filter: " + run());
  }

  static int run() {
    checks = 0;
    pool();
    int w = 760, h = 420;
    Options probe = TestSupport.options();
    probe.compute = "auto";
    if (!GpuDepthOutlier.applies(probe, w, h)) {
      System.out.println("GpuDepthOutlierRegression skipped: GPU outlier filter not applicable");
      return checks;
    }
    long decided = 0;
    for (int seed = 0; seed < 3; seed++)
      for (int radius = 2; radius <= GpuDepthOutlier.MAX_RADIUS; radius++)
        for (int withLr = 0; withLr < 2; withLr++) {
          Random random = new Random(seed * 31 + radius * 7 + withLr);
          float[] d = new float[w * h], lr = withLr == 0 ? null : new float[w * h];
          byte[] gray = new byte[w * h];
          double noise = seed == 0 ? .2 : seed == 1 ? .6 : 1.5;
          for (int y = 0; y < h; y++)
            for (int x = 0; x < w; x++) {
              int p = y * w + x;
              boolean upper = x + y / 3 > 380 + 40 * Math.sin(y / 40.0);
              double v = (upper ? 40 + .01 * x - .02 * y : 25 + .03 * x + .005 * y) + random.nextGaussian() * noise;
              if (random.nextDouble() < .03) v += (random.nextBoolean() ? 1 : -1) * (2 + 10 * random.nextDouble());
              if ((x / 37 + y / 23) % 11 == 0 && random.nextDouble() < .5) v += 6;
              if (Math.abs(x - 150 - y / 4) <= 1) v += 5;
              d[p] = random.nextDouble() < .08 ? Float.NaN : (float) v;
              gray[p] = (byte) ((upper ? 160 : 60) + random.nextInt(seed == 1 ? 40 : 8));
              if (lr != null) lr[p] = random.nextDouble() < .1 ? Float.NaN : (float) (random.nextDouble() * 5);
            }
          Options o = TestSupport.options();
          o.compute = "auto";
          o.depthOutlierFilter = "edge-aware";
          o.depthOutlierRadius = radius;
          o.depthOutlierDeviation = seed == 1 ? .8 : 1.5;
          int from = seed == 2 ? 40 : 0, to = seed == 2 ? h - 50 : h;
          byte[] gpu = GpuDepthOutlier.codes(o, d, gray, w, h, from, to, lr);
          if (gpu == null) {
            System.out.println("GpuDepthOutlierRegression skipped: no OpenCL FP64 device");
            return checks;
          }
          byte[] cpu = DepthOutlierFilter.codes(o, d, gray, w, h, from, to, lr, 1);
          int differ = 0;
          for (int i = 0; i < cpu.length; i++) {
            if (cpu[i] != gpu[i]) differ++;
            if (cpu[i] > 1) decided++;
          }
          check(differ == 0, differ + " GPU outlier codes differ (seed " + seed + ", radius " + radius + ")");
        }
    check(decided > 1_000_000, "too few decisions exercised: " + decided);
    return checks;
  }

  /** reserveAuxiliary: waits only when the thread holds no pool memory; never exceeds the pool. */
  private static void pool() {
    try (GpuMemoryPool.Scope scope = GpuMemoryPool.install(1000)) {
      try (GpuMemoryPool.Reservation held = GpuMemoryPool.reserve(800)) {
        check(GpuMemoryPool.reserveAuxiliary(300) == null, "auxiliary reservation must not wait while holding");
        try (GpuMemoryPool.Reservation small = GpuMemoryPool.reserveAuxiliary(200)) {
          check(small != null && small.bytes == 200, "free auxiliary bytes must be granted");
        }
        // A session reservation never waits while the thread holds pool memory (it would wait for
        // itself): granted if free now, otherwise an error the caller turns into a CPU solve.
        try (GpuMemoryPool.Reservation more = GpuMemoryPool.reserve(150)) {
          check(more.bytes == 150, "free bytes are granted to a holding thread");
        }
        boolean refused = false;
        try (GpuMemoryPool.Reservation more = GpuMemoryPool.reserve(300)) {
        } catch (IllegalStateException expected) {
          refused = true;
        }
        check(refused, "a holding thread must not wait for pool memory");
      }
      check(GpuMemoryPool.reserveAuxiliary(2000) == null, "auxiliary reservation above the pool");
      try (GpuMemoryPool.Reservation all = GpuMemoryPool.reserveAuxiliary(1000)) {
        check(all != null, "a thread holding nothing waits for (and gets) the bytes");
      }
    }
  }
}
