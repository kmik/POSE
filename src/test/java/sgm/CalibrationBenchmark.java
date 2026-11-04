package sgm;

import java.nio.file.*;
import java.util.*;
import org.opencv.core.Core;

/** Optional real-image alignment benchmark, kept separate from deterministic numerical tests. */
public final class CalibrationBenchmark {
  public static void main(String[] args) throws Exception {
    if (args.length != 4)
      throw new IllegalArgumentException(
          "images, Metashape XML, new output directory, initial source focal pixels");
    nu.pattern.OpenCV.loadLocally();
    Core.setNumThreads(4);
    Core.setRNGSeed(1);
    Options o = TestSupport.options();
    o.input = Path.of(args[0]);
    o.output = Path.of(args[2]);
    Files.createDirectory(o.output);
    o.alignmentCalibration = true;
    o.maxDimension = 1600;
    o.features = 56000;
    o.descriptorMatcher = "flann";
    o.neighbors = 5;
    o.storage = "minimal";
    o.cache = false;
    List<Path> images = Main.images(o);
    int[] size = ResolutionPlan.originalSize(images.getFirst());
    Calibration initial =
        new Calibration(
            Double.parseDouble(args[3]),
            Double.parseDouble(args[3]),
            (size[0] - 1) * .5,
            (size[1] - 1) * .5,
            new double[5],
            size[0],
            size[1]);
    List<ImageFrame> frames = new ArrayList<>();
    long started = System.nanoTime();
    try {
      for (Path p : images) frames.add(new ImageFrame(frames.size(), p, initial, o, true));
      FeatureTracks features = new FeatureTracks();
      features.build(frames, o);
      Geometry.reconstruct(frames, features, o, null);
      Calibration result = frames.getFirst().calibration;
      result.write(
          o.output.resolve("camera.properties"), "Controlled real-image alignment benchmark");
      var reference =
          OrientationXml.read(Path.of(args[1]), "metashape", Map.of(), o)
              .views
              .values()
              .iterator()
              .next()
              .calibration();
      CalibrationDiagnostics.compare(o.output.resolve("metashape-comparison"), result, reference);
      Json.write(
          o.output.resolve("benchmark.json"),
          Json.object(
              "images",
              images.size(),
              "maximum_dimension",
              1600,
              "initial_source_focal_pixels",
              initial.fx,
              "initial_distortion",
              "zero",
              "matcher",
              "flann",
              "feature_limit",
              o.features,
              "seconds",
              (System.nanoTime() - started) / 1e9,
              "tracks",
              features.tracks.size(),
              "scope",
              "Alignment only; bootstrap and dense reconstruction excluded; Metashape used only"
                  + " after fitting for model comparison"));
    } finally {
      for (var frame : frames) frame.close();
    }
  }
}
