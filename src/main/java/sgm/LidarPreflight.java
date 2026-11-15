package sgm;

import java.awt.image.BufferedImage;
import java.io.IOException;
import java.nio.file.Path;
import java.util.*;

/**
 * Lidar coverage check before a run (the New run tab's "Check lidar coverage"): the images'
 * GPS positions and estimated ground footprints instead of tie points, then the same coverage
 * test as the lidar step ({@link LidarReference#coverage}). Footprint: a circle through the image
 * corners at nadir, radius = height above ground x half the diagonal of a 36 x 24 mm frame / the
 * 35 mm equivalent focal length (DJI relative altitude; 100 m and 24 mm assumed when missing).
 * Windowless.
 */
final class LidarPreflight {
  private LidarPreflight() {}

  static final double ASSUMED_HEIGHT = 100, ASSUMED_FOCAL_35 = 24, SAMPLE = 5;

  record Result(boolean complete, String message, int images, int withGps, List<String> assumptions, BufferedImage map, double seconds) {}

  /** Footprints {x, y, radius} from the images' EXIF (GPS, relative altitude, 35 mm focal length). */
  static List<double[]> footprints(List<Path> images, Crs crs, List<String> assumptions) {
    List<double[]> out = new ArrayList<>();
    int noHeight = 0, noFocal = 0;
    for (Path p : images) {
      Exif.Metadata m;
      try {
        m = Exif.read(p);
      } catch (IOException | RuntimeException e) {
        continue;
      }
      if (m.gps == null || !m.gps.usable()) continue;
      double[] xy;
      try {
        xy = crs.project(m.gps);
      } catch (IllegalArgumentException e) {
        continue;
      }
      double h = m.relativeAltitude;
      if (!(h > 1)) {
        h = ASSUMED_HEIGHT;
        noHeight++;
      }
      double f = m.focal35Mm;
      if (!(f > 5)) {
        f = ASSUMED_FOCAL_35;
        noFocal++;
      }
      out.add(new double[] {xy[0], xy[1], Math.min(1000, h * Math.hypot(18, 12) / f)});
    }
    if (noHeight > 0) assumptions.add(noHeight + " images without a height above ground (DJI relative altitude): " + (int) ASSUMED_HEIGHT + " m assumed");
    if (noFocal > 0) assumptions.add(noFocal + " images without a 35 mm focal length: " + (int) ASSUMED_FOCAL_35 + " mm assumed");
    return out;
  }

  /** Coverage of the footprints (points every 5 m inside each circle) by the lidar. */
  static Result check(List<double[]> footprints, Path lidar, Crs crs, LidarReference.Settings set, int images, List<String> assumptions) throws IOException {
    long t0 = System.nanoTime();
    if (footprints.isEmpty())
      return new Result(false, "No image has a usable GPS position, so the area cannot be estimated before alignment.", images, 0, assumptions, null, 0);
    List<double[]> pts = new ArrayList<>();
    for (double[] f : footprints)
      for (double dx = -f[2]; dx <= f[2]; dx += SAMPLE)
        for (double dy = -f[2]; dy <= f[2]; dy += SAMPLE)
          if (dx * dx + dy * dy <= f[2] * f[2]) pts.add(new double[] {f[0] + dx, f[1] + dy, 0});
    LidarReference.Source src = LidarReference.sources(lidar, crs, set);
    LidarReference.Points p = LidarReference.points(pts);
    LidarReference.Area area = LidarReference.area(p, set.margin());
    LidarReference.Surfaces s = LidarReference.read(src, area, Math.max(set.cell(), 5), null);
    LidarReference.Coverage c = LidarReference.coverage(area, s, src, set.margin());
    String message = (c.complete()
            ? "The lidar covers the area of all " + footprints.size() + " images (estimated footprints"
                + String.format(Locale.ROOT, ", %.2f ha with the %.0f m margin)", c.areaM2() / 1e4, set.margin())
                + (c.gapM2() > 0 ? String.format(Locale.ROOT, "; %.1f%% has no lidar returns inside the tiles (water or gaps)", 100 * c.gapM2() / c.areaM2()) : "")
                + "."
            : c.message().replace("of the tie-point area", "of the images' estimated footprints").replace("overlap the tie-point area", "overlap the images' estimated footprints"))
        + (src.warnings().isEmpty() ? "" : "\n" + String.join("\n", src.warnings()));
    return new Result(c.complete(), message, images, footprints.size(), assumptions, LidarReference.coverageMap(c, area), (System.nanoTime() - t0) / 1e9);
  }

  /** The whole check from the New run tab's values: image folder, lidar file or folder, CRS, margin. */
  static Result check(Path imageFolder, Path lidar, String crsName, double margin) throws IOException {
    Options o = Options.parse(new String[] {"-i", imageFolder.toString(), "--lidar-reference", lidar.toString(), "--lidar-margin", String.valueOf(margin)});
    List<Path> images = Main.images(o);
    Crs crs = new Crs(crsName == null || crsName.isBlank() ? o.crs : crsName);
    List<String> assumptions = new ArrayList<>();
    List<double[]> f = footprints(images, crs, assumptions);
    return check(f, lidar, crs, LidarReference.Settings.of(o), images.size(), assumptions);
  }
}
