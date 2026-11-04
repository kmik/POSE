package sgm;

import java.io.*;
import java.nio.*;
import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

/** Known-depth scene with a false stereo surface seen in only two of three images. */
public final class AdversarialSceneTest {
  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Core.setNumThreads(1);
    Path root = Paths.get(args[0]);
    Files.createDirectories(root);
    int minimumViews = args.length > 2 ? Integer.parseInt(args[2]) : 3;
    Metrics clean = run(root.resolve("clean"), false, minimumViews),
        corrupt = run(root.resolve("corrupt"), true, minimumViews);
    String report =
        "case,points,outside_true_plane,phantom_depth_points,plane_inliers\n"
            + clean.row("clean")
            + corrupt.row("corrupt");
    Files.writeString(root.resolve("metrics.csv"), report);
    System.out.print(report);
    if (args.length > 1 && args[1].equals("strict")) {
      if (corrupt.phantom > Math.max(10, .001 * corrupt.count))
        throw new AssertionError("Two-view phantom surface survived independent fusion");
      if (corrupt.outside > .01 * corrupt.count)
        throw new AssertionError("Known-plane outlier fraction exceeds 1%");
      if (corrupt.inliers < .5 * clean.inliers)
        throw new AssertionError("Noise rejection erased too much valid surface");
    }
  }

  private static Metrics run(Path root, boolean corrupt, int minimumViews) throws IOException {
    Path images = root.resolve("images"), output = root.resolve("result");
    Files.createDirectories(images);
    int w = 320, h = 240, tw = 400;
    byte[] background = texture(tw, h, 34), phantom = texture(tw, h, 107);
    for (int camera = 0; camera < 3; camera++) {
      byte[] pixels = new byte[w * h * 3];
      for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
          boolean fake = corrupt && camera < 2 && x >= 90 && x < 210 && y >= 70 && y < 170;
          int d = fake ? 20 : 10;
          byte[] source = fake ? phantom : background;
          System.arraycopy(source, (y * tw + x + camera * d) * 3, pixels, (y * w + x) * 3, 3);
        }
      Mat image = new Mat(h, w, CvType.CV_8UC3);
      try {
        image.put(0, 0, pixels);
        Imgcodecs.imwrite(images.resolve(camera + ".png").toString(), image);
      } finally {
        image.release();
      }
    }
    Path calibration = root.resolve("camera.txt"), poses = root.resolve("poses.csv");
    Files.writeString(calibration, "width=320\nheight=240\nfx=240\nfy=240\ncx=159.5\ncy=119.5\n");
    StringBuilder csv = new StringBuilder(PoseIO.HEADER + "\n");
    for (int c = 0; c < 3; c++) csv.append(c + ".png,1,0,0,0,1,0,0,0,1," + (-.25 * c) + ",0,0\n");
    Files.writeString(poses, csv);
    int code =
        TestSupport.run(
            new String[] {
              "-i",
              images.toString(),
              "-o",
              output.toString(),
              "--calibration",
              calibration.toString(),
              "--poses",
              poses.toString(),
              "--disparities",
              "48",
              "--step",
              "2",
              "--min-views",
              Integer.toString(minimumViews),
              "--cores",
              "1"
            });
    if (code != 0) throw new AssertionError("Synthetic pipeline failed with status " + code);
    byte[] bytes = Files.readAllBytes(output.resolve("dense.las"));
    ByteBuffer b = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN);
    int n = b.getInt(107);
    Metrics metrics = new Metrics();
    metrics.count = n;
    for (int i = 0; i < n; i++) {
      double z =
          b.getInt(b.getInt(96) + i * Short.toUnsignedInt(b.getShort(105)) + 8) * b.getDouble(147)
              + b.getDouble(171);
      if (Math.abs(z - 6) > .3) metrics.outside++;
      else metrics.inliers++;
      if (Math.abs(z - 3) < .3) metrics.phantom++;
    }
    return metrics;
  }

  private static byte[] texture(int width, int height, long seed) {
    byte[] pixels = new byte[width * height * 3];
    new Random(seed).nextBytes(pixels);
    Mat m = new Mat(height, width, CvType.CV_8UC3);
    try {
      m.put(0, 0, pixels);
      Imgproc.GaussianBlur(m, m, new Size(3, 3), .6);
      m.get(0, 0, pixels);
      return pixels;
    } finally {
      m.release();
    }
  }

  private static final class Metrics {
    int count, outside, phantom, inliers;

    String row(String name) {
      return name + "," + count + "," + outside + "," + phantom + "," + inliers + "\n";
    }
  }
}
