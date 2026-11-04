package sgm;

import java.nio.*;
import java.nio.file.*;
import java.security.*;
import java.util.*;

/** Logged-size 3000x4000 / 174-label exact storage comparison; run each mode in a fresh JVM. */
final class NativeHybridBenchmark {
  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    org.opencv.core.Core.setNumThreads(10);
    int w = 3000, h = 4000, n = w * h;
    byte[] a = new byte[n], b = new byte[n];
    new Random(86231).nextBytes(a);
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) b[y * w + x] = a[y * w + Math.min(w - 1, x + 270)];
    Options o = TestSupport.options();
    o.compute = "gpu";
    o.gpuDevice = 0;
    o.gpuMemoryMb = 8000;
    o.memoryMb = 16000;
    o.gpuSgm = args[0];
    o.gpuAggregate = args[0].equals("streamed") ? "ram" : "auto";
    o.gpuStripRows = 64;
    o.minDisparity = 192;
    o.disparities = 174;
    o.speckleSize = 0;
    o.median = 1;
    Path output = Path.of(args[1]);
    Files.createDirectories(output);
    o.output = output;
    Object plan = GpuStereo.report(w, h, o);
    long started = System.nanoTime();
    var result = new CensusSgm(o).compute(a, b, w, h);
    double seconds = (System.nanoTime() - started) / 1e9;
    Map<String, Object> report =
        Json.object(
            "mode",
            args[0],
            "width",
            w,
            "height",
            h,
            "disparities",
            174,
            "seconds",
            seconds,
            "plan",
            plan,
            "disparity_sha256",
            hash(result.disparity),
            "margin_sha256",
            hash(result.costMargin),
            "sharpness_sha256",
            hash(result.costSharpness),
            "correlation_sha256",
            hash(result.correlation),
            "lr_sha256",
            hash(result.leftRightError),
            "stats",
            result.stats.csv());
    Json.write(output.resolve("benchmark.json"), report);
    System.out.println(Json.string(report));
    GpuSgm.closeSession();
  }

  private static String hash(float[] values) throws Exception {
    if (values == null) return "unavailable";
    MessageDigest digest = MessageDigest.getInstance("SHA-256");
    ByteBuffer b = ByteBuffer.allocate(65536);
    for (float value : values) {
      if (b.remaining() < 4) {
        digest.update(b.array(), 0, b.position());
        b.clear();
      }
      b.putInt(Float.floatToIntBits(value));
    }
    digest.update(b.array(), 0, b.position());
    return HexFormat.of().formatHex(digest.digest());
  }
}
