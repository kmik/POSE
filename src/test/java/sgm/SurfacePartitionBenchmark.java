package sgm;

import java.nio.*;
import java.nio.file.*;
import java.security.*;
import java.util.*;

public final class SurfacePartitionBenchmark {
  public static void main(String[] args) throws Exception {
    Path out = Files.createDirectories(Path.of(args[0]));
    List<SurfaceDiagnostics.Point> data =
        SurfaceDiagnostics.sample(Path.of(args[1]), Integer.parseInt(args[2]), null);
    Options o = new Options();
    o.output = out;
    o.compute = "cpu";
    o.surfaceValidation = "cpu";
    o.memoryMb = 1024;
    o.surfaceMemoryMb = 128;
    Camera[] cameras = {new Camera(900, 900, 400, 400), new Camera(900, 900, 400, 400)};
    cameras[1].translation = new double[] {-1, 0, 0};
    MessageDigest hash = MessageDigest.getInstance("SHA-256");
    ByteBuffer b = ByteBuffer.allocate(32).order(ByteOrder.LITTLE_ENDIAN);
    long started = System.nanoTime(), finish;
    try (DepthFusion fusion =
        new DepthFusion(cameras, new int[] {800, 800}, new int[] {800, 800}, o)) {
      fusion.addPair(0, 1);
      try (SurfaceConsolidator surface = new SurfaceConsolidator(o, fusion)) {
        long id = 0;
        for (var q : data) {
          double[] point = {q.x(), q.y(), q.z()};
          var sample =
              new SurfaceConsolidator.Sample(
                  id++,
                  0,
                  0,
                  0x123456,
                  new DepthFusion.Result(point, 3, 1, .001, 0, "accepted"),
                  new DepthFusion.SurfaceSample(null, .01, .001));
          sample.attributes = new SurfaceAttributes(.01, .001, .8, .1, 25, 0, false);
          surface.add(sample);
        }
        finish = System.nanoTime();
        surface.finish(
            p -> {
              b.clear();
              b.putLong(p.id);
              for (double v : p.point) b.putDouble(v);
              hash.update(b.array());
            });
      }
    }
    Json.write(
        out.resolve("benchmark.json"),
        Json.object(
            "source",
            "Read-only sample of completed real forest cloud; not the original pre-consolidation"
                + " evidence",
            "samples",
            data.size(),
            "finish_seconds",
            (System.nanoTime() - finish) / 1e9,
            "total_seconds",
            (System.nanoTime() - started) / 1e9,
            "sha256",
            HexFormat.of().formatHex(hash.digest()),
            "scope",
            "Partition routing and ordered passthrough of unreliable-normal samples; excludes"
                + " neighborhood/refinement work"));
  }
}
