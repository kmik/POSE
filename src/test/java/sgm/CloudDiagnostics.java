package sgm;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.*;
import java.nio.*;
import java.nio.file.*;
import java.util.*;
import java.util.List;
import javax.imageio.ImageIO;

/** Read-only diagnostics of exported clouds. Comparisons to sparse tracks are not ground truth. */
public final class CloudDiagnostics {
  public static void main(String[] args) throws Exception {
    Path las = Paths.get(args[0]), sparse = Paths.get(args[1]), out = Paths.get(args[2]);
    Files.createDirectories(out);
    List<double[]> dense = readLas(las), tracks = new ArrayList<>();
    for (String line : Files.readAllLines(sparse).subList(1, Files.readAllLines(sparse).size())) {
      String[] v = line.split(",");
      tracks.add(
          new double[] {
            Double.parseDouble(v[0]), Double.parseDouble(v[1]), Double.parseDouble(v[2]), 0xFFFFFF
          });
    }
    StringBuilder text =
        new StringBuilder(
            "Dense points=" + dense.size() + "\nSparse points=" + tracks.size() + "\n");
    double[][] bound = new double[3][2];
    for (int axis = 0; axis < 3; axis++) {
      double[] values = new double[dense.size()], s = new double[tracks.size()];
      for (int i = 0; i < values.length; i++) values[i] = dense.get(i)[axis];
      for (int i = 0; i < s.length; i++) s[i] = tracks.get(i)[axis];
      Arrays.sort(values);
      Arrays.sort(s);
      text.append(
          "axis="
              + axis
              + " dense min,p01,p50,p95,p99,max="
              + quantiles(values)
              + " sparse="
              + quantiles(s)
              + "\n");
      bound[axis][0] = percentile(values, .005);
      bound[axis][1] = percentile(values, .995);
    }
    double[] nearest = new double[dense.size()];
    for (int i = 0; i < dense.size(); i++) {
      double[] p = dense.get(i);
      double minimum = Double.POSITIVE_INFINITY;
      for (double[] s : tracks) {
        double dx = p[0] - s[0], dy = p[1] - s[1], dz = p[2] - s[2];
        minimum = Math.min(minimum, dx * dx + dy * dy + dz * dz);
      }
      nearest[i] = Math.sqrt(minimum);
    }
    Arrays.sort(nearest);
    text.append("Nearest sparse distance min,p01,p50,p95,p99,max=" + quantiles(nearest) + "\n");
    BufferedImage image = new BufferedImage(1500, 550, BufferedImage.TYPE_INT_RGB);
    Graphics2D g = image.createGraphics();
    g.setColor(Color.BLACK);
    g.fillRect(0, 0, 1500, 550);
    int[][] axes = {{0, 1}, {0, 2}, {1, 2}};
    for (int panel = 0; panel < 3; panel++) {
      int a = axes[panel][0], b = axes[panel][1];
      for (double[] p : dense) {
        int x = panel * 500 + 20 + (int) (460 * (p[a] - bound[a][0]) / (bound[a][1] - bound[a][0]));
        int y = 510 - (int) (460 * (p[b] - bound[b][0]) / (bound[b][1] - bound[b][0]));
        if (x >= panel * 500 + 20 && x < panel * 500 + 480 && y >= 50 && y <= 510)
          image.setRGB(x, y, (int) p[3]);
      }
      g.setColor(Color.WHITE);
      g.drawString("axis " + a + " vs " + b + "; display .5%..99.5%", panel * 500 + 20, 25);
    }
    g.dispose();
    ImageIO.write(image, "png", out.resolve("cloud-projections.png").toFile());
    Path parent = las.getParent();
    Properties settings = new Properties();
    Path settingsPath = parent.resolve("run-settings.properties");
    if (Files.exists(settingsPath))
      try (Reader reader = Files.newBufferedReader(settingsPath)) {
        settings.load(reader);
      }
    int minimum = Integer.parseInt(settings.getProperty("minDisparity", "0")),
        number =
            Integer.parseInt(settings.getProperty("disparities", args.length > 3 ? args[3] : "96"));
    try (java.util.stream.Stream<Path> files = Files.list(parent)) {
      for (Path p : (Iterable<Path>) files.filter(f -> f.toString().endsWith(".pfm"))::iterator)
        text.append(p.getFileName() + " " + disparityStats(p, minimum, number) + "\n");
    }
    Files.writeString(out.resolve("statistics.txt"), text.toString());
    System.out.print(text);
  }

  static List<double[]> readLas(Path path) throws IOException {
    byte[] bytes = Files.readAllBytes(path);
    ByteBuffer b = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN);
    int offset = b.getInt(96), length = b.getShort(105) & 65535, count = b.getInt(107);
    List<double[]> out = new ArrayList<>(count);
    double[] scale = {b.getDouble(131), b.getDouble(139), b.getDouble(147)},
        origin = {b.getDouble(155), b.getDouble(163), b.getDouble(171)};
    for (int i = 0; i < count; i++) {
      int at = offset + i * length;
      int rgb =
          ((b.getShort(at + 20) & 65535) / 257 << 16)
              | ((b.getShort(at + 22) & 65535) / 257 << 8)
              | ((b.getShort(at + 24) & 65535) / 257);
      out.add(
          new double[] {
            b.getInt(at) * scale[0] + origin[0],
            b.getInt(at + 4) * scale[1] + origin[1],
            b.getInt(at + 8) * scale[2] + origin[2],
            rgb
          });
    }
    return out;
  }

  private static String disparityStats(Path p, int minimum, int number) throws IOException {
    byte[] bytes = Files.readAllBytes(p);
    int offset = 0;
    for (int n = 0; n < 3; n++) {
      while (bytes[offset++] != '\n') {}
    }
    ByteBuffer b = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN);
    int total = (bytes.length - offset) / 4;
    double[] valid = new double[total];
    int n = 0, low = 0, high = 0;
    for (int at = offset; at < bytes.length; at += 4) {
      float d = b.getFloat(at);
      if (Float.isFinite(d)) {
        valid[n++] = d;
        if (d <= minimum + 1) low++;
        if (d >= minimum + number - 2) high++;
      }
    }
    valid = Arrays.copyOf(valid, n);
    Arrays.sort(valid);
    return "valid="
        + n
        + "/"
        + total
        + " <="
        + (minimum + 1)
        + "="
        + low
        + " >="
        + (minimum + number - 2)
        + "="
        + high
        + " quantiles="
        + quantiles(valid);
  }

  private static String quantiles(double[] a) {
    return String.format(
        Locale.ROOT,
        "[%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]",
        percentile(a, 0),
        percentile(a, .01),
        percentile(a, .5),
        percentile(a, .95),
        percentile(a, .99),
        percentile(a, 1));
  }

  private static double percentile(double[] a, double p) {
    return a.length == 0 ? Double.NaN : a[(int) Math.round(p * (a.length - 1))];
  }
}
