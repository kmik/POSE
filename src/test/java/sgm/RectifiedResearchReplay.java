package sgm;

import java.lang.reflect.*;
import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;

/**
 * Recreate a saved pair using the production image loading/rectification path; read-only inputs.
 */
public final class RectifiedResearchReplay {
  public static void main(String[] args) throws Exception {
    if (args.length != 4)
      throw new IllegalArgumentException("RUN_DIRECTORY IMAGE_DIRECTORY REFERENCE SECONDARY");
    nu.pattern.OpenCV.loadLocally();
    Core.setNumThreads(1);
    Path run = Path.of(args[0]), images = Path.of(args[1]);
    Options o = TestSupport.options();
    o.quality = "high";
    o.compute = "cpu";
    o.output =
        Path.of(System.getProperty("sgm.research.output", "target/esgm-research/forest-pair"));
    Files.createDirectories(o.output);
    o.calibration = run.resolve("01_calibration/reference_camera.properties");
    Calibration calibration = Calibration.load(o, 1, 1);
    try (ImageFrame a = new ImageFrame(0, images.resolve(args[2]), calibration, o);
        ImageFrame b = new ImageFrame(1, images.resolve(args[3]), calibration, o)) {
      List<ImageFrame> frames = List.of(a, b);
      PoseIO.read(run.resolve("cameras.csv"), frames, true);
      DenseReconstruction reconstruction = new DenseReconstruction(o, frames);
      Field fusion = DenseReconstruction.class.getDeclaredField("fusion");
      fusion.setAccessible(true);
      Method rectify =
          DenseReconstruction.class.getDeclaredMethod(
              "rectify", ImageFrame.class, ImageFrame.class, boolean.class, int.class);
      rectify.setAccessible(true);
      try (AutoCloseable pair = (AutoCloseable) rectify.invoke(reconstruction, a, b, false, 0)) {
        Map<String, Object> matrices = new LinkedHashMap<>();
        for (String name : List.of("r1", "r2", "p1", "p2", "q", "vertical")) {
          Field field = pair.getClass().getDeclaredField(name);
          field.setAccessible(true);
          matrices.put(name, field.get(pair));
        }
        Json.write(o.output.resolve("rectification_replay.json"), matrices);
        for (String name : List.of("leftGray", "rightGray", "leftMask", "rightMask")) {
          Field field = pair.getClass().getDeclaredField(name);
          field.setAccessible(true);
          Mat value = (Mat) field.get(pair);
          if (!Imgcodecs.imwrite(o.output.resolve(name + ".png").toString(), value))
            throw new IllegalStateException("Failed to write research raster");
          System.out.println(name + ": " + value.cols() + "x" + value.rows());
        }
      } finally {
        ((DepthFusion) fusion.get(reconstruction)).close();
      }
    }
  }
}
