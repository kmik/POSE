package sgm;

import java.io.*;
import java.nio.*;
import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;

/**
 * Opt-in full reported workload; reads saved rasters and compares complete post-filtered outputs.
 */
public final class LargeScalingBenchmark {
  public static void main(String[] args) throws Exception {
    if (args.length != 3)
      throw new IllegalArgumentException("RASTER_DIRECTORY ram|recompute|esgm RESULT_DIRECTORY");
    nu.pattern.OpenCV.loadLocally();
    Core.setNumThreads(1);
    Path input = Path.of(args[0]), output = Files.createDirectories(Path.of(args[2]));
    String mode = args[1];
    byte[][] images = new byte[4][];
    int w = 0, h = 0, j = 0;
    for (String name : List.of("leftGray", "rightGray", "leftMask", "rightMask")) {
      Mat image = Imgcodecs.imread(input.resolve(name + ".png").toString(), 0);
      try {
        if (image.empty()) throw new IOException("Cannot read " + name);
        if (j == 0) {
          w = image.cols();
          h = image.rows();
        }
        if (w != image.cols() || h != image.rows())
          throw new IOException("Mismatched saved rasters");
        images[j] = new byte[w * h];
        image.get(0, 0, images[j++]);
      } finally {
        image.release();
      }
    }
    Options o = TestSupport.options();
    o.output = output;
    o.compute = "gpu";
    o.gpuMemoryMb = 8000;
    o.gpuDevice = Integer.getInteger("sgm.test.gpu", 0);
    o.minDisparity = 131;
    o.disparities = 1037;
    o.memoryMb = mode.equals("ram") ? 16000 : 2048;
    o.gpuSgm = mode.equals("esgm") ? "esgm" : "streamed";
    o.gpuAggregate = mode.equals("esgm") ? "auto" : mode;
    o.esgmCandidates = Integer.getInteger("sgm.test.candidates", 2);
    o.stereoReportDirectory = output;
    long start = System.nanoTime();
    var result =
        new CensusSgm(o).compute(images[0], images[1], null, null, w, h, images[2], images[3]);
    double seconds = (System.nanoTime() - start) / 1e9;
    Path disparity = output.resolve("disparity.bin");
    try (var out =
        new DataOutputStream(new BufferedOutputStream(Files.newOutputStream(disparity)))) {
      out.writeInt(w);
      out.writeInt(h);
      for (float f : result.disparity) out.writeInt(Float.floatToIntBits(f));
    }
    Path reference = input.resolve("exact-reference.bin");
    boolean compared = false;
    if (Files.exists(reference)) {
      try (var in = new DataInputStream(new BufferedInputStream(Files.newInputStream(reference)))) {
        if (in.readInt() != w || in.readInt() != h)
          throw new AssertionError("Reference dimensions");
        for (int p = 0; p < result.disparity.length; p++)
          if (in.readInt() != Float.floatToIntBits(result.disparity[p]))
            throw new AssertionError("Full filtered disparity differs at pixel " + p);
        compared = true;
      }
    }
    long valid = 0;
    for (float f : result.disparity) if (Float.isFinite(f)) valid++;
    Json.write(
        output.resolve("result.json"),
        Json.object(
            "mode",
            mode,
            "width",
            w,
            "height",
            h,
            "disparities",
            o.disparities,
            "seconds",
            seconds,
            "valid",
            valid,
            "stats",
            result.stats.csv(),
            "disparity_sha256",
            ReuseCache.hash(disparity),
            "compared_exact_reference",
            compared,
            "gpu_plan",
            GpuStereo.report(w, h, o),
            "device",
            GpuSgm.selectedDevice(o).name()));
    System.out.println(
        "PASS large "
            + mode
            + ": "
            + seconds
            + " s; valid="
            + valid
            + "; exact reference="
            + compared);
    GpuSgm.closeSession();
  }
}
