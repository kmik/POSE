package sgm;

import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;

/** Separate exploratory benchmark: neither matcher configuration is installed as a default. */
public final class AdaptiveSearchBenchmark {
  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Core.setNumThreads(1);
    Path root = Path.of(System.getProperty("sgm.research.output", "target/esgm-research/adaptive"));
    Files.createDirectories(root);
    List<Object> reports = new ArrayList<>();
    try {
      for (var f :
          List.of(
              EsgmInvestigation.scene("plane", 512, 256, 0, 128, 19, false, false),
              EsgmInvestigation.scene("fences-and-layers", 512, 256, 0, 128, 19, true, false),
              EsgmInvestigation.scene("repeated-texture", 512, 256, 0, 128, 19, true, true))) {
        Options o = TestSupport.options();
        o.output = root;
        o.compute = "gpu";
        o.gpuSgm = "resident";
        o.gpuDevice = Integer.getInteger("sgm.test.gpu", 0);
        o.gpuMemoryMb = 6000;
        o.memoryMb = 4096;
        o.disparities = f.count();
        o.minDisparity = f.min();
        o.tileSize = 128;
        o.tileOverlap = 16;
        o.median = 1;
        o.speckleSize = 0;
        o.pyramidLevels = 1;
        Map<String, Object> result =
            Json.object(
                "fixture", f.name(), "width", f.w(), "height", f.h(), "disparities", f.count());
        float[] reference = null;
        for (String mode : List.of("ordinary", "pyramid", "adaptive")) {
          o.pyramidLevels = mode.equals("ordinary") ? 1 : 3;
          o.stereoReportDirectory = Files.createDirectories(root.resolve(f.name()).resolve(mode));
          long start = System.nanoTime();
          CensusSgm.Result r =
              mode.equals("adaptive")
                  ? AdaptiveStereoInvestigation.compute(
                      o, f.a(), f.b(), null, null, f.w(), f.h(), null, null, null)
                  : DenseMatcher.compute(
                      o, f.a(), f.b(), null, null, f.w(), f.h(), null, null, null);
          double seconds = (System.nanoTime() - start) / 1e9;
          if (reference == null) reference = r.disparity;
          int common = 0, different = 0, extra = 0, lost = 0;
          double change = 0;
          for (int p = 0; p < reference.length; p++) {
            boolean a = Float.isFinite(reference[p]), b = Float.isFinite(r.disparity[p]);
            if (a && b) {
              common++;
              double d = Math.abs(reference[p] - r.disparity[p]);
              change += d;
              if (d > 1) different++;
            } else if (b) extra++;
            else if (a) lost++;
          }
          result.put(
              mode,
              Json.object(
                  "seconds",
                  seconds,
                  "metrics",
                  EsgmInvestigation.metrics(r.disparity, f),
                  "common_with_ordinary",
                  common,
                  "common_difference_over_1_px",
                  different,
                  "mean_common_difference_px",
                  common == 0 ? Double.NaN : change / common,
                  "extra_valid",
                  extra,
                  "lost_valid",
                  lost,
                  "stats",
                  r.stats.csv()));
        }
        reports.add(result);
        System.out.println("RESULT " + Json.string(result));
      }
      Json.write(
          root.resolve("results.json"),
          Json.object("status", "research_not_production", "results", reports));
      System.out.println("Research adaptive benchmark complete: " + root);
    } finally {
      GpuSgm.closeSession();
    }
  }
}
