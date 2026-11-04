package sgm;

import java.nio.file.*;

/**
 * Runs --classify-ground on a finished run without touching it: dense.las, dense.ply and
 * point-confidence.bin are copied to OUT_DIR and processed there.
 *
 * <p>Usage: GroundReplay RUN_DIR IMAGE_DIR OUT_DIR; -Dsgm.pairCorrection=true and
 * -Dsgm.groundAwareFusion=true enable GS1 correction and GS2.
 */
final class GroundReplay {
  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Path run = Path.of(args[0]), images = Path.of(args[1]), out = Path.of(args[2]);
    Files.createDirectories(out);
    Path las = out.resolve("dense.las"), sidecar = out.resolve("point-confidence.bin");
    Files.copy(run.resolve("07_results/dense.las"), las, StandardCopyOption.REPLACE_EXISTING);
    if (Files.isRegularFile(run.resolve("07_results/dense.ply")))
      Files.copy(run.resolve("07_results/dense.ply"), out.resolve("dense.ply"), StandardCopyOption.REPLACE_EXISTING);
    Files.copy(run.resolve("07_results/point-confidence.bin"), sidecar, StandardCopyOption.REPLACE_EXISTING);
    Options o = TestSupport.options();
    o.output = out;
    o.groundPairCorrection = Boolean.getBoolean("sgm.pairCorrection");
    o.groundAwareFusion = Boolean.getBoolean("sgm.groundAwareFusion");
    var fixture = GpsAdjustmentReplay.load(run.resolve("04_geometry/final"), images, o);
    long started = System.nanoTime();
    var result = GroundClassification.classify(o, fixture.frames(), las, sidecar, out, new Crs(o.crs).epsg);
    Json.write(out.resolve("ground.json"), result);
    System.out.printf("GROUND REPLAY %.1f s%n", (System.nanoTime() - started) / 1e9);
    for (ImageFrame f : fixture.frames()) f.close();
    profiles(las, out);
  }

  /** Cross-sections (2 m wide bands) through the middle: ground green, other grey, noise red. */
  static void profiles(Path las, Path out) throws Exception {
    var cloud = GroundClassification.read(las, Path.of("missing"));
    byte[] classes = new byte[cloud.n];
    try (var ch = java.nio.channels.FileChannel.open(las)) {
      var b = java.nio.ByteBuffer.allocate(cloud.recordLength * 65536);
      for (long start = 0; start < cloud.n; start += 65536) {
        int records = (int) Math.min(65536, cloud.n - start);
        b.clear().limit(records * cloud.recordLength);
        while (b.hasRemaining()) ch.read(b, cloud.dataOffset + start * cloud.recordLength + b.position());
        for (int r = 0; r < records; r++) classes[(int) (start + r)] = b.get(r * cloud.recordLength + 15);
      }
    }
    float minX = Float.MAX_VALUE, maxX = -Float.MAX_VALUE, minY = Float.MAX_VALUE, maxY = -Float.MAX_VALUE;
    float minZ = Float.MAX_VALUE, maxZ = -Float.MAX_VALUE;
    for (int i = 0; i < cloud.n; i++) {
      minX = Math.min(minX, cloud.x[i]);
      maxX = Math.max(maxX, cloud.x[i]);
      minY = Math.min(minY, cloud.y[i]);
      maxY = Math.max(maxY, cloud.y[i]);
      minZ = Math.min(minZ, cloud.z[i]);
      maxZ = Math.max(maxZ, cloud.z[i]);
    }
    for (int axis = 0; axis < 2; axis++) {
      double centre = axis == 0 ? (minY + maxY) / 2 : (minX + maxX) / 2;
      double lo = axis == 0 ? minX : minY, hi = axis == 0 ? maxX : maxY;
      int w = 2000, h = 500;
      var image = new java.awt.image.BufferedImage(w, h, java.awt.image.BufferedImage.TYPE_INT_RGB);
      var gfx = image.createGraphics();
      gfx.setColor(java.awt.Color.WHITE);
      gfx.fillRect(0, 0, w, h);
      gfx.dispose();
      for (int pass = 0; pass < 2; pass++)
        for (int i = 0; i < cloud.n; i++) {
          double across = axis == 0 ? cloud.y[i] : cloud.x[i];
          if (Math.abs(across - centre) > 1) continue;
          boolean ground = classes[i] == 2;
          if ((pass == 1) != ground) continue;
          double along = axis == 0 ? cloud.x[i] : cloud.y[i];
          int px = (int) ((along - lo) / (hi - lo) * (w - 1)), py = (int) ((maxZ - cloud.z[i]) / (maxZ - minZ) * (h - 1));
          image.setRGB(px, py, classes[i] == 7 ? 0xdc2626 : ground ? 0x16a34a : 0x9ca3af);
        }
      javax.imageio.ImageIO.write(image, "png", out.resolve("profile-" + (axis == 0 ? "east" : "north") + ".png").toFile());
    }
  }

  /** Zoomed ground-band profile: 20 m long, 0.5 m wide, heights magnified (layering view). */
  static void zoom(Path las, Path out, double alongFraction) throws Exception {
    var cloud = GroundClassification.read(las, Path.of("missing"));
    float minX = Float.MAX_VALUE, maxX = -Float.MAX_VALUE, minY = Float.MAX_VALUE, maxY = -Float.MAX_VALUE;
    for (int i = 0; i < cloud.n; i++) {
      minX = Math.min(minX, cloud.x[i]);
      maxX = Math.max(maxX, cloud.x[i]);
      minY = Math.min(minY, cloud.y[i]);
      maxY = Math.max(maxY, cloud.y[i]);
    }
    double y0 = (minY + maxY) / 2, x0 = minX + (maxX - minX) * alongFraction;
    java.util.List<double[]> pts = new java.util.ArrayList<>();
    for (int i = 0; i < cloud.n; i++)
      if (Math.abs(cloud.y[i] - y0) < .25 && cloud.x[i] >= x0 && cloud.x[i] < x0 + 20) pts.add(new double[] {cloud.x[i] - x0, cloud.z[i]});
    double low = Double.MAX_VALUE;
    for (double[] p : pts) low = Math.min(low, p[1]);
    int w = 2000, h = 400;
    var image = new java.awt.image.BufferedImage(w, h, java.awt.image.BufferedImage.TYPE_INT_RGB);
    var gfx = image.createGraphics();
    gfx.setColor(java.awt.Color.WHITE);
    gfx.fillRect(0, 0, w, h);
    gfx.dispose();
    // 20 m across 2000 px, 4 m of height across 400 px.
    for (double[] p : pts) {
      int px = (int) (p[0] / 20 * (w - 1)), py = (int) (h - 1 - (p[1] - low) / 4 * (h - 1));
      if (py >= 0 && py < h) image.setRGB(px, py, 0x111827);
    }
    javax.imageio.ImageIO.write(image, "png", out.resolve("zoom.png").toFile());
  }
}
