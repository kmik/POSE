package sgm;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.*;
import java.nio.*;
import java.nio.channels.FileChannel;
import java.nio.file.*;
import java.util.*;
import java.util.List;
import java.util.zip.GZIPInputStream;
import javax.imageio.ImageIO;

/**
 * Reduced-resolution experiment review; reference plots are centered, not registered accuracy
 * tests.
 */
final class TwoViewExperimentAnalysis {
  record P(long id, double x, double y, double z, int views, int rgb, String canonical) {}

  record Cloud(String name, List<P> points, boolean reference) {}

  public static void main(String[] args) throws Exception {
    Path out = Path.of(args[0]);
    Files.createDirectories(out);
    Cloud base = new Cloud("Legacy: two-view orange / 3+ view blue", read(Path.of(args[2])), false);
    List<Cloud> plots = new ArrayList<>();
    plots.add(
        new Cloud("Metashape high: centered structural reference", las(Path.of(args[1])), true));
    plots.add(base);
    Map<Long, P> indexed = new HashMap<>();
    for (P p : base.points) indexed.put(p.id, p);
    long core = base.points.stream().filter(p -> p.views >= 3).count();
    List<Object> comparisons = new ArrayList<>();
    ply(
        out.resolve("trusted-three-plus.ply"),
        base.points.stream().filter(p -> p.views >= 3).toList(),
        0x196489);
    ply(
        out.resolve("baseline-two-view.ply"),
        base.points.stream().filter(p -> p.views == 2).toList(),
        0xe58b1e);
    for (int i = 3; i < args.length; i++) {
      Path run = Path.of(args[i]);
      List<P> points = read(run);
      Set<Long> retained = new HashSet<>();
      int trusted = 0, extra = 0;
      for (P p : points) {
        P original = indexed.get(p.id);
        if (original == null || !original.canonical.equals(p.canonical))
          throw new AssertionError("Retained candidate coordinates/attributes changed: " + p.id);
        if (!retained.add(p.id)) throw new AssertionError("Duplicate candidate ID");
        if (p.views >= 3) trusted++;
        else extra++;
      }
      if (trusted != core)
        throw new AssertionError("Trusted core lost points: " + trusted + " != " + core);
      String name = run.getParent().getParent().getFileName().toString();
      List<P> removed = base.points.stream().filter(p -> !retained.contains(p.id)).toList();
      if (removed.stream().anyMatch(p -> p.views != 2))
        throw new AssertionError("Non-two-view point removed");
      comparisons.add(
          Json.object(
              "run",
              run.toString(),
              "points",
              points.size(),
              "three_plus_unchanged",
              trusted,
              "two_view_retained",
              extra,
              "two_view_removed",
              removed.size(),
              "retained_all_non_index_csv_fields_exact",
              true));
      ply(
          out.resolve(name + "-two-view.ply"),
          points.stream().filter(p -> p.views == 2).toList(),
          0xe58b1e);
      ply(out.resolve(name + "-removed.ply"), removed, 0xc04048);
      plots.add(new Cloud(name + ": two-view orange / 3+ view blue", points, false));
      if (!removed.isEmpty()) plots.add(new Cloud("Removed: " + name + " (red)", removed, false));
    }
    Json.write(
        out.resolve("comparison.json"),
        Json.object(
            "baseline",
            args[2],
            "baseline_points",
            base.points.size(),
            "trusted_three_plus",
            core,
            "comparisons",
            comparisons,
            "reference_scope",
            "Metashape is a model reference, not surveyed truth. Plots use original XYZ axes,"
                + " independently centered reference and shared experimental core center. No"
                + " registration or accuracy distances claimed."));
    render(plots, out.resolve("structural-comparison.png"));
    System.out.println(
        Json.string(
            Json.object(
                "baseline", base.points.size(), "trusted_core", core, "comparisons", comparisons)));
  }

  private static List<P> read(Path run) throws IOException {
    List<P> points = new ArrayList<>();
    try (var in =
        new BufferedReader(
            new InputStreamReader(
                new GZIPInputStream(
                    Files.newInputStream(run.resolve("07_results/point-confidence.csv.gz"))),
                java.nio.charset.StandardCharsets.UTF_8))) {
      in.readLine();
      String line;
      while ((line = in.readLine()) != null) {
        String[] p = line.split(",", 14);
        long id = Long.parseLong(p[1]);
        points.add(
            new P(
                id,
                Double.parseDouble(p[3]),
                Double.parseDouble(p[4]),
                Double.parseDouble(p[5]),
                Integer.parseInt(p[9]),
                Integer.parseInt(p[6]) << 16 | Integer.parseInt(p[7]) << 8 | Integer.parseInt(p[8]),
                line.substring(line.indexOf(',') + 1)));
      }
    }
    return points;
  }

  static List<P> las(Path file) throws IOException {
    List<P> points = new ArrayList<>();
    try (FileChannel in = FileChannel.open(file)) {
      ByteBuffer h = ByteBuffer.allocate(375).order(ByteOrder.LITTLE_ENDIAN);
      while (h.hasRemaining() && in.read(h) > 0) {}
      if (h.getInt(0) != 0x4653414c || (h.get(104) & 128) != 0)
        throw new IOException("Expected uncompressed LAS");
      int stride = Short.toUnsignedInt(h.getShort(105));
      long count = Integer.toUnsignedLong(h.getInt(107));
      if (count == 0 && Short.toUnsignedInt(h.getShort(94)) >= 375) count = h.getLong(247);
      int step = (int) Math.max(1, (count + 199999) / 200000);
      double sx = h.getDouble(131),
          sy = h.getDouble(139),
          sz = h.getDouble(147),
          ox = h.getDouble(155),
          oy = h.getDouble(163),
          oz = h.getDouble(171);
      in.position(Integer.toUnsignedLong(h.getInt(96)));
      ByteBuffer b = ByteBuffer.allocateDirect(stride * 8192).order(ByteOrder.LITTLE_ENDIAN);
      for (long first = 0; first < count; first += 8192) {
        int n = (int) Math.min(8192, count - first);
        b.clear();
        b.limit(n * stride);
        while (b.hasRemaining()) if (in.read(b) < 0) throw new EOFException();
        for (int j = 0; j < n; j++)
          if ((first + j) % step == 0) {
            int p = j * stride;
            points.add(
                new P(
                    first + j,
                    b.getInt(p) * sx + ox,
                    b.getInt(p + 4) * sy + oy,
                    b.getInt(p + 8) * sz + oz,
                    0,
                    0x666666,
                    ""));
          }
      }
    }
    return points;
  }

  private static void ply(Path file, List<P> points, int rgb) throws IOException {
    try (var out = new BufferedOutputStream(Files.newOutputStream(file))) {
      out.write(
          ("ply\nformat binary_little_endian 1.0\nelement vertex "
                  + points.size()
                  + "\n"
                  + "property double x\n"
                  + "property double y\n"
                  + "property double z\n"
                  + "property uchar red\n"
                  + "property uchar green\n"
                  + "property uchar blue\n"
                  + "end_header\n")
              .getBytes(java.nio.charset.StandardCharsets.US_ASCII));
      ByteBuffer b = ByteBuffer.allocate(27).order(ByteOrder.LITTLE_ENDIAN);
      for (P p : points) {
        b.clear();
        b.putDouble(p.x)
            .putDouble(p.y)
            .putDouble(p.z)
            .put((byte) (rgb >> 16))
            .put((byte) (rgb >> 8))
            .put((byte) rgb);
        out.write(b.array());
      }
    }
  }

  private static double[] center(List<P> points) {
    double[] c = new double[3];
    for (P p : points) {
      c[0] += p.x;
      c[1] += p.y;
      c[2] += p.z;
    }
    for (int j = 0; j < 3; j++) c[j] /= Math.max(1, points.size());
    return c;
  }

  private static void render(List<Cloud> clouds, Path file) throws IOException {
    render(
        clouds,
        file,
        "Centered structural views. Blue: unchanged 3+ view core; orange: retained two-view; red:"
            + " removed two-view. Density is not accuracy.");
  }

  static void render(List<Cloud> clouds, Path file, String legend) throws IOException {
    render(
        clouds,
        file,
        legend,
        "Experimental 800-pixel reconstruction; reference high. Centering only, no registration.");
  }

  static void render(List<Cloud> clouds, Path file, String legend, String title)
      throws IOException {
    int W = 1350, H = clouds.size() * 310 + 70;
    BufferedImage im = new BufferedImage(W, H, BufferedImage.TYPE_INT_RGB);
    Graphics2D g = im.createGraphics();
    g.setColor(Color.WHITE);
    g.fillRect(0, 0, W, H);
    g.setFont(new Font(Font.SANS_SERIF, Font.PLAIN, 15));
    g.setColor(Color.DARK_GRAY);
    g.drawString(title + " Axes remain original XYZ.", 18, 24);
    g.drawString(legend, 18, 46);
    double[] common = center(clouds.get(1).points.stream().filter(p -> p.views >= 3).toList());
    double[] reference = center(clouds.get(0).points);
    double[] bounds = new double[6];
    Arrays.fill(bounds, 0, 3, Double.POSITIVE_INFINITY);
    Arrays.fill(bounds, 3, 6, Double.NEGATIVE_INFINITY);
    for (int c = 0; c < 2; c++) {
      List<P> points = clouds.get(c).points;
      double[] origin = c == 0 ? reference : common;
      for (int axis = 0; axis < 3; axis++) {
        final int a = axis;
        double[] values =
            points.stream()
                .mapToDouble(p -> (a == 0 ? p.x : a == 1 ? p.y : p.z) - origin[a])
                .sorted()
                .toArray();
        if (values.length > 0) {
          bounds[axis] = Math.min(bounds[axis], values[(int) (values.length * .005)]);
          bounds[axis + 3] =
              Math.max(
                  bounds[axis + 3],
                  values[Math.min(values.length - 1, (int) (values.length * .995))]);
        }
      }
    }
    int[][] axes = {{0, 1}, {0, 2}, {1, 2}};
    for (int row = 0; row < clouds.size(); row++) {
      Cloud cloud = clouds.get(row);
      double[] origin = cloud.reference ? reference : common;
      int top = 70 + row * 310;
      g.setColor(Color.DARK_GRAY);
      g.drawString(
          cloud.name + "; " + cloud.points.size() + " plotted/input samples", 18, top + 17);
      for (int col = 0; col < 3; col++) {
        int pw = 420,
            ph = 260,
            xx = 18 + col * 445,
            yy = top + 30,
            a = axes[col][0],
            b = axes[col][1];
        double scale =
            Math.min(
                pw / Math.max(.001, bounds[a + 3] - bounds[a]),
                ph / Math.max(.001, bounds[b + 3] - bounds[b]));
        double midA = (bounds[a] + bounds[a + 3]) / 2, midB = (bounds[b] + bounds[b + 3]) / 2;
        int[] core = new int[pw * ph], extra = new int[pw * ph];
        int clipped = 0,
            step =
                cloud.reference ? 1 : Math.max(1, (clouds.get(1).points.size() + 249999) / 250000);
        for (int i = 0; i < cloud.points.size(); i++) {
          P p = cloud.points.get(i);
          if (Math.floorMod(p.id, step) != 0) continue;
          double va = (a == 0 ? p.x : a == 1 ? p.y : p.z) - origin[a],
              vb = (b == 0 ? p.x : b == 1 ? p.y : p.z) - origin[b];
          int x = (int) Math.round(pw / 2. + (va - midA) * scale),
              y = (int) Math.round(ph / 2. - (vb - midB) * scale);
          if (x < 0 || x >= pw || y < 0 || y >= ph) {
            clipped++;
            continue;
          }
          if (p.views >= 3) core[y * pw + x]++;
          else extra[y * pw + x]++;
        }
        int maximum = 1;
        for (int i = 0; i < core.length; i++) maximum = Math.max(maximum, core[i] + extra[i]);
        for (int i = 0; i < core.length; i++)
          if (core[i] + extra[i] > 0) {
            double total = core[i] + extra[i],
                alpha = .2 + .8 * Math.log1p(total) / Math.log1p(maximum),
                f = core[i] / total;
            boolean removed = cloud.name.startsWith("Removed:");
            int r = cloud.reference ? 80 : removed ? 192 : (int) (25 * f + 229 * (1 - f)),
                green = cloud.reference ? 80 : removed ? 64 : (int) (100 * f + 139 * (1 - f)),
                blue = cloud.reference ? 80 : removed ? 72 : (int) (137 * f + 30 * (1 - f));
            g.setColor(
                new Color(
                    (int) (255 * (1 - alpha) + r * alpha),
                    (int) (255 * (1 - alpha) + green * alpha),
                    (int) (255 * (1 - alpha) + blue * alpha)));
            g.fillRect(xx + i % pw, yy + i / pw, 1, 1);
          }
        g.setColor(Color.LIGHT_GRAY);
        g.drawRect(xx, yy, pw, ph);
        g.setColor(Color.DARK_GRAY);
        g.drawString(
            "XYZ".charAt(a) + " / " + "XYZ".charAt(b) + "; outside frame: " + clipped,
            xx + 8,
            yy + ph - 7);
      }
    }
    g.dispose();
    ImageIO.write(im, "png", file.toFile());
    im.flush();
  }
}
