package sgm;

import java.nio.*;
import java.nio.file.*;
import java.security.*;
import java.util.*;

/**
 * Disk-evidence audit benchmark; synthetic known plane, separate from source-image reconstruction.
 */
public final class FusionIoBenchmark {
  public static void main(String[] args) throws Exception {
    Path out = Files.createDirectories(Path.of(args[0]));
    int w = 640,
        h = 480,
        cameraCount = 16,
        pairCount = 32,
        n = args.length > 1 ? Integer.parseInt(args[1]) : 131072;
    Options o = new Options();
    o.output = out;
    o.projectDirectory = out;
    o.compute = "gpu";
    o.fusionCompute = "gpu";
    o.gpuMemoryMb = 1500;
    o.memoryMb = 2048;
    o.maxDepthUncertainty = 1;
    Camera[] cameras = new Camera[cameraCount];
    int[] widths = new int[cameraCount], heights = new int[cameraCount];
    for (int i = 0; i < cameraCount; i++) {
      cameras[i] = new Camera(700, 700, w / 2., h / 2.);
      cameras[i].translation = new double[] {(i - 8) * .06, 0, 0};
      cameras[i].registered = true;
      widths[i] = w;
      heights[i] = h;
    }
    long began = System.nanoTime();
    MessageDigest hash = MessageDigest.getInstance("SHA-256");
    long accepted = 0;
    ByteBuffer bytes = ByteBuffer.allocate(48).order(ByteOrder.LITTLE_ENDIAN);
    try (DepthFusion fusion = new DepthFusion(cameras, widths, heights, o)) {
      for (int pair = 0; pair < pairCount; pair++) {
        fusion.addPair(pair % cameraCount, (pair * 3 + 1) % cameraCount);
        for (int y = 0; y < h; y++)
          for (int x = 0; x < w; x++)
            fusion.add(pair, new double[] {(x - w / 2.) * 10 / 700., (y - h / 2.) * 10 / 700., 10});
      }
      fusion.prepare(n);
      long start = System.nanoTime();
      int chunk = 4096;
      try {
        chunk = Options.class.getField("fusionBatchSize").getInt(o);
      } catch (NoSuchFieldException ignored) {
      }
      for (int base = 0; base < n; base += chunk) {
        int length = Math.min(chunk, n - base);
        int[] sources = new int[length];
        double[][] p = new double[length][];
        for (int i = 0; i < length; i++) {
          long id = base + i;
          int x = 30 + (int) ((id * 127) % 580), y = 30 + (int) ((id * 173) % 420);
          sources[i] = 0;
          p[i] = new double[] {(x + .13 - w / 2.) * 10 / 700., (y + .19 - h / 2.) * 10 / 700., 10};
        }
        DepthFusion.Result[] results = fusion.fuseBatch(sources, p, length);
        for (var result : results) {
          if (result.accepted()) accepted++;
          bytes.clear();
          bytes
              .putInt(result.status.hashCode())
              .putInt(result.views)
              .putInt(result.independentPairs);
          for (double coordinate : result.point) bytes.putLong(Math.round(coordinate * 1e8));
          bytes.putLong(Math.round(result.relativeSigma * 1e10));
          hash.update(bytes.array(), 0, bytes.position());
        }
      }
      Json.write(
          out.resolve("benchmark.json"),
          Json.object(
              "synthetic",
              true,
              "candidates",
              n,
              "accepted",
              accepted,
              "fusion_seconds",
              (System.nanoTime() - start) / 1e9,
              "total_seconds",
              (System.nanoTime() - began) / 1e9,
              "decision_geometry_sha256",
              HexFormat.of().formatHex(hash.digest()),
              "backend",
              fusion.backendReport(),
              "scope",
              "GPU fusion with all CPU audits and disk-backed evidence; quantized geometry hash at"
                  + " 1e-8 coordinate units and sigma 1e-10"));
    } finally {
      GpuSgm.closeSession();
    }
  }
}
