package sgm;

import java.util.*;

/**
 * GPU spike filter marks against the CPU code, every pixel: smooth and rough surfaces with a depth
 * edge, isolated spikes, thin lines, holes, sparse regions, NaN and negative depths, odd and even
 * neighbour counts, whole maps and row ranges (strips with halo rows), and maps sent in several
 * bands. Also the memory rules: a 1 MiB budget, --compute cpu and -Dsgm.cpuSpikeFilter leave the
 * map to the CPU. Skipped without a usable OpenCL FP64 device.
 */
final class GpuSpikeFilterRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) {
    if (args.length > 0) timing();
    else System.out.println("PASS GPU spike filter: " + run());
  }

  private static float[] map(int w, int h, int seed) {
    Random random = new Random(seed);
    float[] z = new float[w * h];
    double noise = seed % 3 == 0 ? .002 : seed % 3 == 1 ? .02 : .2;
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) {
        boolean upper = x + y / 3 > w / 2 + 40 * Math.sin(y / 40.0);
        double v = (upper ? 40 + .01 * x - .02 * y : 25 + .03 * x + .005 * y) + random.nextGaussian() * noise;
        if (random.nextDouble() < .02) v += (random.nextBoolean() ? 1 : -1) * (.2 + 3 * random.nextDouble());
        if (Math.abs(x - 150 - y / 4) <= (seed & 1)) v -= 5;
        if (y % 97 == 13) v -= 4;
        double hole = random.nextDouble();
        boolean sparse = (x / 61 + y / 47) % 5 == 0;
        z[y * w + x] = hole < (sparse ? .55 : .06) ? 0 : hole > .998 ? Float.NaN : hole > .996 ? -1 : (float) v;
      }
    return z;
  }

  private static byte[] cpu(float[] z, int w, int h, int from, int to, double focal) {
    byte[] marks = new byte[(to - from) * w];
    Parallel.forEach(to - from, Runtime.getRuntime().availableProcessors(), row ->
        ImageFusion.spikeRow(z, w, h, focal, from + row, new float[24], new float[24], marks, row * w));
    return marks;
  }

  static int run() {
    checks = 0;
    int w = 1100, h = 640;
    Options o = TestSupport.options();
    o.compute = "auto";
    check(GpuSpikeFilter.bandRows(w, h, 1 << 20, 1L << 30) == 0, "a 1 MiB budget holds no band");
    check(GpuSpikeFilter.bandRows(w, h, 1L << 30, 1L << 30) == h, "a small map is one band");
    check(GpuSpikeFilter.bandRows(w, h, GpuSpikeFilter.bytes(w, 300), 1L << 30) == 300, "bands fill the budget");
    check(GpuSpikeFilter.bandRows(w, h, 1L << 30, 4L * w * 304) == 300, "bands respect the largest allocation");
    check(GpuSpikeFilter.bandRows(20000, 9000, 1L << 33, 1L << 31) * 20000L <= GpuSpikeFilter.BAND_PIXELS, "band pixel limit");
    float[] first = map(w, h, 0);
    Options cpuOnly = TestSupport.options();
    cpuOnly.compute = "cpu";
    check(GpuSpikeFilter.marks(cpuOnly, first, w, h, 0, h, 1500) == null, "--compute cpu: CPU path");
    System.setProperty("sgm.cpuSpikeFilter", "true");
    try {
      check(GpuSpikeFilter.marks(o, first, w, h, 0, h, 1500) == null, "-Dsgm.cpuSpikeFilter: CPU path");
    } finally {
      System.clearProperty("sgm.cpuSpikeFilter");
    }
    if (GpuSpikeFilter.marks(o, first, w, h, 0, h, 1500) == null) {
      System.out.println("GpuSpikeFilterRegression skipped: no OpenCL FP64 device");
      return checks;
    }
    try {
      Options tiny = TestSupport.options();
      tiny.compute = "auto";
      tiny.gpuMemoryMb = 1;
      long before = GpuSpikeFilter.cpuMaps;
      check(GpuSpikeFilter.marks(tiny, first, w, h, 0, h, 1500) == null && GpuSpikeFilter.cpuMaps == before + 1,
          "a map beyond a 1 MiB GPU budget is left to the CPU");
      long spikes = 0, valid = 0;
      for (int seed = 0; seed < 6; seed++) {
        float[] z = map(w, h, seed);
        double focal = seed < 3 ? 1500 : 4000;
        int from = seed % 3 == 2 ? 37 : 0, to = seed % 3 == 2 ? h - 51 : h;
        Options use = TestSupport.options();
        use.compute = "auto";
        // Seeds 3-5: a budget that splits the map into bands of 377 rows.
        if (seed >= 3) use.gpuMemoryMb = 2;
        long bands = GpuSpikeFilter.bands;
        byte[] gpu = GpuSpikeFilter.marks(use, z, w, h, from, to, focal);
        check(gpu != null, "GPU marks available (seed " + seed + ")");
        if (seed >= 3) check(GpuSpikeFilter.bands - bands >= 2, "a 2 MiB budget sends the map in bands");
        byte[] cpu = cpu(z, w, h, from, to, focal);
        int differ = 0;
        for (int i = 0; i < cpu.length; i++) {
          if (cpu[i] != gpu[i]) differ++;
          spikes += cpu[i];
        }
        for (float v : z) if (v > 0) valid++;
        check(differ == 0, differ + " GPU spike marks differ (seed " + seed + ")");
      }
      check(spikes > 20_000 && valid > 2_000_000, "too few decisions exercised: " + spikes + " spikes, " + valid + " depths");
    } finally {
      GpuSpikeFilter.close();
    }
    return checks;
  }

  /** Seconds per 2000 x 1125 map, CPU and GPU (not part of the regression). */
  private static void timing() {
    int w = 2000, h = 1125;
    float[] z = map(w, h, 1);
    Options o = TestSupport.options();
    o.compute = "auto";
    for (int open = 0; open < 2; open++) {
      long t = System.nanoTime();
      GpuSpikeFilter.marks(o, z, w, h, 0, h, 1500);
      System.out.printf(Locale.ROOT, "first map after opening the device: %.0f ms%n", (System.nanoTime() - t) / 1e6);
      GpuSpikeFilter.close();
    }
    for (int round = 0; round < 3; round++) {
      long t0 = System.nanoTime();
      for (int i = 0; i < 10; i++) cpu(z, w, h, 0, h, 1500);
      long t1 = System.nanoTime();
      for (int i = 0; i < 10; i++) GpuSpikeFilter.marks(o, z, w, h, 0, h, 1500);
      long t2 = System.nanoTime();
      System.out.printf(Locale.ROOT, "per map: CPU %.1f ms, GPU %.1f ms%n", (t1 - t0) / 1e7, (t2 - t1) / 1e7);
    }
    GpuSpikeFilter.close();
  }
}
