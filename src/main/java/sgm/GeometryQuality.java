package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import org.opencv.core.Point;

/** Sparse reprojection residuals diagnose geometry; they are not surveyed accuracy estimates. */
final class GeometryQuality {
  static String write(Path output, List<ImageFrame> frames, List<FeatureTracks.Track> tracks)
      throws IOException {
    List<List<Double>> residuals = new ArrayList<>();
    for (ImageFrame f : frames) residuals.add(new ArrayList<>());
    double total = 0;
    int observations = 0;
    for (FeatureTracks.Track track : tracks)
      if (track.point != null)
        for (FeatureTracks.Observation o : track.observations) {
          ImageFrame f = frames.get(o.image);
          if (!f.camera.registered) continue;
          double[] predicted = f.camera.project(track.point);
          if (predicted == null) continue;
          Point actual = f.keypoints[o.feature].pt;
          double e = Math.hypot(predicted[0] - actual.x, predicted[1] - actual.y);
          residuals.get(o.image).add(e);
          total += e * e;
          observations++;
        }
    List<String> rows = new ArrayList<>();
    rows.add("image,observations,reprojection_median_pixels,reprojection_p95_pixels");
    for (ImageFrame f : frames) {
      List<Double> e = residuals.get(f.id);
      Collections.sort(e);
      rows.add(
          PoseIO.quote(f.path.getFileName().toString())
              + ","
              + e.size()
              + ","
              + (e.isEmpty() ? "" : e.get(e.size() / 2))
              + ","
              + (e.isEmpty() ? "" : e.get((int) (.95 * (e.size() - 1)))));
    }
    Files.write(output.resolve("geometry-quality.csv"), rows);
    return "Sparse reprojection RMS pixels="
        + (observations == 0
            ? "not available (supplied geometry)"
            : Math.sqrt(total / observations));
  }
}
