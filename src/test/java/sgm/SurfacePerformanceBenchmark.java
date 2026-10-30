package sgm;

import java.nio.*;
import java.nio.file.*;
import java.security.*;
import java.util.*;

/**
 * Reproducible consolidation-only load; no image/SGM time is included. No measured-field accuracy
 * claim.
 */
public final class SurfacePerformanceBenchmark {
  public static void main(String[] args) throws Exception {
    long count = args.length > 1 ? Long.parseLong(args[1]) : 6503432;
    Path directory = Files.createDirectories(Path.of(args[0]));
    boolean device = args.length > 2 && args[2].equals("gpu");
    int width = count > 1000000 ? 900 : 240;
    int height = Math.max(80, (int) Math.ceil((double) count / (8 * width)));
    Camera[] cameras = new Camera[16];
    int[] widths = new int[16], heights = new int[16];
    for (int i = 0; i < cameras.length; i++) {
      cameras[i] = new Camera(900, 900, width / 2., height / 2.);
      cameras[i].translation = new double[] {(i - 7.5) * .015, 0, 0};
      cameras[i].registered = true;
      widths[i] = width;
      heights[i] = height;
    }
    Options o = new Options();
    o.output = directory;
    o.memoryMb = 1024;
    o.surfaceMemoryMb = 128;
    o.compute = device ? "gpu" : "cpu";
    o.fusionCompute = o.compute;
    o.gpuMemoryMb = 1500;
    o.diskDepth = false;
    o.fusionRelative = .01;
    o.fusionPixels = 1.25;
    o.maxDepthUncertainty = 1;
    MessageDigest digest = MessageDigest.getInstance("SHA-256");
    ByteBuffer hash = ByteBuffer.allocate(64).order(ByteOrder.LITTLE_ENDIAN);
    long[] emitted = {0};
    double[] squared = {0};
    long started = System.nanoTime(), finish;
    try (DepthFusion fusion = new DepthFusion(cameras, widths, heights, o)) {
      for (int pair = 0; pair < 8; pair++) {
        fusion.addPair(pair * 2, pair * 2 + 1);
        for (int y = 3; y < height - 3; y++)
          for (int x = 3; x < width - 3; x++) fusion.add(pair, point(pair, x, y, width, height));
      }
      fusion.prepare(count);
      try (SurfaceConsolidator surface = new SurfaceConsolidator(o, fusion)) {
        long id = 0;
        for (int pair = 0; pair < 8 && id < count; pair++)
          for (int y = 0; y < height && id < count; y++)
            for (int x = 0; x < width && id < count; x++, id++) {
              double[] p = point(pair, x, y, width, height);
              surface.add(
                  new SurfaceConsolidator.Sample(
                      id,
                      pair,
                      y * width + x,
                      0x789abc,
                      new DepthFusion.Result(p, 16, 7, .001, 0, "accepted"),
                      new DepthFusion.SurfaceSample(new double[] {0, 0, -1}, 10. / 900, .003)));
            }
        finish = System.nanoTime();
        surface.finish(
            new SurfaceConsolidator.Sink() {
              public void point(SurfaceConsolidator.Sample p) {
                emitted[0]++;
                squared[0] += Math.pow(p.point[2] - 10, 2);
                hash.clear();
                hash.putLong(p.id);
                for (double v : p.point) hash.putDouble(v);
                hash.putInt(p.views).putInt(p.pairs).putInt(p.merged);
                digest.update(hash.array(), 0, hash.position());
              }

              public void merged(SurfaceConsolidator.Sample p, long representative) {
                hash.clear();
                hash.putLong(p.id).putLong(representative);
                digest.update(hash.array(), 0, hash.position());
              }
            });
      }
      Json.write(directory.resolve("backend.json"), fusion.backendReport());
    }
    Json.write(
        directory.resolve("benchmark.json"),
        Json.object(
            "input_samples",
            count,
            "output_samples",
            emitted[0],
            "decision_geometry_sha256",
            HexFormat.of().formatHex(digest.digest()),
            "plane_rms",
            Math.sqrt(squared[0] / emitted[0]),
            "finish_seconds",
            (System.nanoTime() - finish) / 1e9,
            "total_seconds",
            (System.nanoTime() - started) / 1e9,
            "synthetic",
            true,
            "scope",
            "Consolidation-only synthetic load; excludes images/SGM, not user city workload"));
    GpuSgm.closeSession();
    System.out.println("BENCHMARK COMPLETE " + directory);
  }

  private static double[] point(int pair, int x, int y, int width, int height) {
    long h = ((long) y * width + x + pair * 1234567L) * 0x9e3779b97f4a7c15L;
    double noise = ((h >>> 11) * 0x1.0p-53 - .5) * .008;
    return new double[] {(x - width / 2.) * 10 / 900, (y - height / 2.) * 10 / 900, 10 + noise};
  }
}
