package sgm;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.nio.file.*;
import java.util.*;
import java.util.List;
import javax.imageio.ImageIO;

/** Identical plotting windows for before/after QA; cropped points are counted explicitly. */
public final class CloudComparison {
  public static void main(String[] args) throws Exception {
    List<double[]> before = CloudDiagnostics.readLas(Paths.get(args[0])),
        after = CloudDiagnostics.readLas(Paths.get(args[1]));
    double[] min = {Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY},
        max = {Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY};
    for (double[] p : after)
      for (int i = 0; i < 3; i++) {
        min[i] = Math.min(min[i], p[i]);
        max[i] = Math.max(max[i], p[i]);
      }
    for (int i = 0; i < 3; i++) {
      double margin = (max[i] - min[i]) * .3;
      min[i] -= margin;
      max[i] += margin;
    }
    BufferedImage image = new BufferedImage(1400, 1000, BufferedImage.TYPE_INT_RGB);
    Graphics2D g = image.createGraphics();
    g.setColor(new Color(17, 22, 29));
    g.fillRect(0, 0, 1400, 1000);
    g.setFont(new Font(Font.SANS_SERIF, Font.PLAIN, 16));
    draw(image, g, before, "Before", 0, 0, 0, 1, min, max);
    draw(image, g, after, "Corrected", 700, 0, 0, 1, min, max);
    draw(image, g, before, "Before", 0, 500, 0, 2, min, max);
    draw(image, g, after, "Corrected", 700, 500, 0, 2, min, max);
    g.dispose();
    ImageIO.write(image, "png", Paths.get(args[2]).toFile());
  }

  private static void draw(
      BufferedImage image,
      Graphics2D g,
      List<double[]> points,
      String label,
      int ox,
      int oy,
      int a,
      int b,
      double[] min,
      double[] max) {
    int dropped = 0;
    for (double[] p : points) {
      int x = ox + 65 + (int) (590 * (p[a] - min[a]) / (max[a] - min[a])),
          y = oy + 430 - (int) (350 * (p[b] - min[b]) / (max[b] - min[b]));
      if (x < ox + 65 || x > ox + 655 || y < oy + 80 || y > oy + 430) {
        dropped++;
        continue;
      }
      image.setRGB(x, y, (int) p[3]);
    }
    g.setColor(Color.WHITE);
    g.drawString(
        label + ": " + points.size() + " points; outside this window: " + dropped,
        ox + 25,
        oy + 30);
    g.drawString(
        a == 0 && b == 1
            ? "XY in seed-camera coordinates"
            : "XZ; camera Z is distance away from the seed camera",
        ox + 25,
        oy + 55);
    g.setColor(new Color(130, 145, 160));
    g.drawRect(ox + 65, oy + 80, 590, 350);
    for (int i = 0; i <= 4; i++) {
      double x = min[a] + (max[a] - min[a]) * i / 4, y = min[b] + (max[b] - min[b]) * i / 4;
      g.drawString(String.format(Locale.ROOT, "%.1f", x), ox + 50 + 590 * i / 4, oy + 455);
      g.drawString(String.format(Locale.ROOT, "%.1f", y), ox + 10, oy + 435 - 350 * i / 4);
    }
  }
}
