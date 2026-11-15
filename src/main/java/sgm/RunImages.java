package sgm;

import java.awt.image.BufferedImage;
import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.*;
import java.util.stream.IntStream;

/**
 * The photographs of a run for the image window, without a window: the image list
 * (00_inputs/images.csv), each image's final interior and exterior orientation
 * (04_geometry/orientation_model.json, source pixels, X_camera = R X_world + t), camera-screening
 * flags, the surface the photographs are projected onto (07_results/dsm.tif, else a plane at the
 * median sparse tie-point height), and rendering of photographs onto a map in world coordinates.
 * Read only.
 */
final class RunImages {
  /** Interior orientation in source pixels; distortion k1, k2, p1, p2, k3 (OpenCV order). */
  record Interior(int width, int height, double fx, double fy, double cx, double cy, double[] d) {}

  /** A camera: X_camera = R X_world + t; z forward, x right, y down. */
  record View(Interior k, double[] r, double[] t) {
    double[] center() {
      return new double[] {
        -(r[0] * t[0] + r[3] * t[1] + r[6] * t[2]), -(r[1] * t[0] + r[4] * t[1] + r[7] * t[2]), -(r[2] * t[0] + r[5] * t[1] + r[8] * t[2])
      };
    }

    /** Source pixel (u, v) of a world point (same model as GroundControl.project), or null behind. */
    double[] project(double x, double y, double z) {
      double qx = r[0] * x + r[1] * y + r[2] * z + t[0], qy = r[3] * x + r[4] * y + r[5] * z + t[1], qz = r[6] * x + r[7] * y + r[8] * z + t[2];
      if (!(qz > 1e-9)) return null;
      double a = qx / qz, b = qy / qz, rr = a * a + b * b, d[] = k.d;
      double radial = 1 + d[0] * rr + d[1] * rr * rr + d[4] * rr * rr * rr;
      double xd = a * radial + 2 * d[2] * a * b + d[3] * (rr + 2 * a * a), yd = b * radial + d[2] * (rr + 2 * b * b) + 2 * d[3] * a * b;
      return new double[] {k.fx * xd + k.cx, k.fy * yd + k.cy};
    }

    /** World direction of the ray through source pixel (u, v) (distortion removed iteratively). */
    double[] ray(double u, double v) {
      double xd = (u - k.cx) / k.fx, yd = (v - k.cy) / k.fy, a = xd, b = yd;
      for (int i = 0; i < 10; i++) {
        double rr = a * a + b * b, d[] = k.d, radial = 1 + d[0] * rr + d[1] * rr * rr + d[4] * rr * rr * rr;
        double dx = 2 * d[2] * a * b + d[3] * (rr + 2 * a * a), dy = d[2] * (rr + 2 * b * b) + 2 * d[3] * a * b;
        a = (xd - dx) / radial;
        b = (yd - dy) / radial;
      }
      // R^T (a, b, 1)
      return new double[] {r[0] * a + r[3] * b + r[6], r[1] * a + r[4] * b + r[7], r[2] * a + r[5] * b + r[8]};
    }
  }

  record Image(int id, String name, Path path, int width, int height, View view, String flag, Double rms) {
    boolean oriented() {
      return view != null;
    }
  }

  /** Heights for the projection. */
  interface Surface {
    double z(double x, double y);

    /** A typical height (for footprints). */
    double reference();

    String description();
  }

  final Path run;
  final List<Image> images;
  final String crs;
  final Surface surface;

  RunImages(Path run, List<Image> images, String crs, Surface surface) {
    this.run = run;
    this.images = images;
    this.crs = crs;
    this.surface = surface;
  }

  /** True when the photographs can be put on a map (orientations exist). */
  boolean mappable() {
    return surface != null && images.stream().anyMatch(Image::oriented);
  }

  static RunImages load(Path run) {
    Map<String, View> views = new HashMap<>();
    String crs = null;
    Map<String, Object> model = ProjectFiles.map(ProjectFiles.map(run.resolve("04_geometry/orientation_model.json")).get("source"));
    if (model.get("crs") != null) crs = String.valueOf(model.get("crs"));
    for (Object o : ProjectFiles.list(model.get("views"))) {
      Map<String, Object> v = ProjectFiles.map(o), in = ProjectFiles.map(v.get("interior"));
      double[] d = new double[5], r = numbers(v.get("R"), 9), t = numbers(v.get("t"), 3), dist = numbers(in.get("distortion"), -1);
      System.arraycopy(dist, 0, d, 0, Math.min(5, dist.length));
      if (r == null || t == null) continue;
      Interior k = new Interior((int) num(in.get("width")), (int) num(in.get("height")), num(in.get("fx")), num(in.get("fy")), num(in.get("cx")), num(in.get("cy")), d);
      views.put(String.valueOf(v.get("image")), new View(k, r, t));
    }
    Map<String, String> flags = screening(run);
    Map<String, Double> rms = reprojection(run);
    List<Image> images = new ArrayList<>();
    for (String[] row : csv(run.resolve("00_inputs/images.csv"))) {
      if (row.length < 7 || row[0].equals("image_id")) continue;
      try {
        images.add(new Image(Integer.parseInt(row[0]), row[1], Path.of(row[2]), Integer.parseInt(row[5]), Integer.parseInt(row[6]), views.get(row[1]), flags.get(row[1]), rms.get(row[1])));
      } catch (RuntimeException e) {
        // a malformed row: skipped
      }
    }
    return new RunImages(run, List.copyOf(images), crs, views.isEmpty() ? null : surface(run, views.values()));
  }

  private static double num(Object o) {
    return o instanceof Number n ? n.doubleValue() : Double.NaN;
  }

  private static double[] numbers(Object o, int n) {
    List<Object> l = ProjectFiles.list(o);
    if (n >= 0 && l.size() != n) return n < 0 ? new double[0] : null;
    double[] out = new double[l.size()];
    for (int i = 0; i < out.length; i++) out[i] = num(l.get(i));
    return out;
  }

  /** Camera-screening flags by image name (04_geometry/camera_screening.json), if any. */
  static Map<String, String> screening(Path run) {
    Map<String, String> out = new HashMap<>();
    Map<String, Object> s = ProjectFiles.map(run.resolve("04_geometry/camera_screening.json"));
    for (Object o : ProjectFiles.list(s.get("cameras"))) {
      Map<String, Object> c = ProjectFiles.map(o);
      Object reasons = c.get("flags");
      boolean flagged = reasons instanceof List<?> l && !l.isEmpty();
      if (flagged && c.get("image") != null) out.put(String.valueOf(c.get("image")), reasons instanceof List<?> l ? String.join(", ", l.stream().map(String::valueOf).toList()) : "flagged");
    }
    return out;
  }

  /** Reprojection RMS per image in pixels (camera_screening.json), if any. */
  static Map<String, Double> reprojection(Path run) {
    Map<String, Double> out = new HashMap<>();
    for (Object o : ProjectFiles.list(ProjectFiles.map(run.resolve("04_geometry/camera_screening.json")).get("cameras"))) {
      Map<String, Object> c = ProjectFiles.map(o);
      if (c.get("image") != null && c.get("reprojection_rms_pixels") instanceof Number n) out.put(String.valueOf(c.get("image")), n.doubleValue());
    }
    return out;
  }

  /** Simple CSV rows (quoted fields with commas allowed). */
  static List<String[]> csv(Path file) {
    List<String[]> out = new ArrayList<>();
    if (!Files.isRegularFile(file)) return out;
    try (BufferedReader r = Files.newBufferedReader(file, StandardCharsets.UTF_8)) {
      String line;
      while ((line = r.readLine()) != null) {
        List<String> cells = new ArrayList<>();
        StringBuilder cur = new StringBuilder();
        boolean quoted = false;
        for (int i = 0; i < line.length(); i++) {
          char c = line.charAt(i);
          if (c == '"') {
            if (quoted && i + 1 < line.length() && line.charAt(i + 1) == '"') {
              cur.append('"');
              i++;
            } else quoted = !quoted;
          } else if (c == ',' && !quoted) {
            cells.add(cur.toString());
            cur.setLength(0);
          } else cur.append(c);
        }
        cells.add(cur.toString());
        out.add(cells.toArray(String[]::new));
      }
    } catch (IOException | UncheckedIOException e) {
      // what was read
    }
    return out;
  }

  /**
   * The run's report DSM when it exists (heights in output units, divided by the coordinate
   * multiplier), with gaps filled by the median tie-point height; else that plane.
   */
  static Surface surface(Path run, Collection<View> views) {
    double plane = sparseMedian(run.resolve("04_geometry/sparse.csv"));
    if (!Double.isFinite(plane)) {
      // No tie points: a plane 80 m below the median camera (only for footprints).
      double[] z = views.stream().mapToDouble(v -> v.center()[2]).sorted().toArray();
      plane = z.length == 0 ? 0 : z[z.length / 2] - 80;
    }
    double flat = plane;
    Map<String, Object> dsm = ProjectFiles.map(run.resolve("07_results/dsm.json"));
    Path tif = run.resolve("07_results/dsm.tif");
    double multiplier = num(ProjectFiles.map(run.resolve("07_results/coordinates.json")).get("coordinate_multiplier"));
    if (!(multiplier > 0)) multiplier = 1;
    List<Object> extent = ProjectFiles.list(dsm.get("extent"));
    if (Files.isRegularFile(tif) && extent.size() == 4 && dsm.get("cell_size") instanceof Number cell)
      try {
        DsmRaster.Heights h = DsmRaster.readTiff(tif);
        double m = multiplier, x0 = num(extent.get(0)) / m, top = num(extent.get(3)) / m, size = cell.doubleValue() / m;
        float[] zs = h.z();
        double median = medianFinite(zs) / m;
        double ref = Double.isFinite(median) ? median : flat;
        return new Surface() {
          public double z(double x, double y) {
            double fc = (x - x0) / size - .5, fr = (top - y) / size - .5;
            int c = (int) Math.floor(fc), r = (int) Math.floor(fr);
            if (c < -1 || r < -1 || c >= h.width() || r >= h.height()) return ref;
            double tx = fc - c, ty = fr - r, sum = 0, weight = 0;
            for (int j = 0; j < 2; j++)
              for (int i = 0; i < 2; i++) {
                int cc = c + i, rr = r + j;
                if (cc < 0 || rr < 0 || cc >= h.width() || rr >= h.height()) continue;
                float value = zs[rr * h.width() + cc];
                if (!Float.isFinite(value)) continue;
                double w = (i == 0 ? 1 - tx : tx) * (j == 0 ? 1 - ty : ty);
                sum += w * value / m;
                weight += w;
              }
            return weight > 1e-9 ? sum / weight : ref;
          }

          public double reference() {
            return ref;
          }

          public String description() {
            return "the run's surface model (07_results/dsm.tif); gaps at the median height";
          }
        };
      } catch (IOException e) {
        // fall back to the plane
      }
    return new Surface() {
      public double z(double x, double y) {
        return flat;
      }

      public double reference() {
        return flat;
      }

      public String description() {
        return String.format(Locale.ROOT, "a plane at the median tie-point height (%.1f); no surface model yet", flat);
      }
    };
  }

  static double medianFinite(float[] z) {
    float[] f = new float[z.length];
    int n = 0;
    for (float v : z) if (Float.isFinite(v)) f[n++] = v;
    if (n == 0) return Double.NaN;
    Arrays.sort(f, 0, n);
    return f[n / 2];
  }

  static double sparseMedian(Path sparse) {
    List<Double> z = new ArrayList<>();
    for (String[] row : csv(sparse)) {
      if (row.length < 3) continue;
      try {
        double v = Double.parseDouble(row[2]);
        if (Double.isFinite(v)) z.add(v);
      } catch (NumberFormatException e) {
        // header
      }
    }
    if (z.isEmpty()) return Double.NaN;
    Collections.sort(z);
    return z.get(z.size() / 2);
  }

  /**
   * World XY outline of an image: its border pixels' rays meeting the surface's reference height
   * ({@code perEdge} samples per side), or null when the camera does not see that height.
   */
  double[][] footprint(Image im, int perEdge) {
    if (im.view == null || surface == null) return null;
    View v = im.view;
    double[] c = v.center();
    double z = surface.reference();
    List<double[]> out = new ArrayList<>();
    int w = v.k.width, h = v.k.height;
    for (int side = 0; side < 4; side++)
      for (int i = 0; i < perEdge; i++) {
        double s = i / (double) perEdge;
        double u = switch (side) {
          case 0 -> s * w;
          case 1 -> w;
          case 2 -> (1 - s) * w;
          default -> 0;
        };
        double vv = switch (side) {
          case 0 -> 0;
          case 1 -> s * h;
          case 2 -> h;
          default -> (1 - s) * h;
        };
        double[] d = v.ray(u, vv);
        if (!(Math.abs(d[2]) > 1e-12)) return null;
        double k = (z - c[2]) / d[2];
        if (!(k > 0)) return null;
        out.add(new double[] {c[0] + k * d[0], c[1] + k * d[1]});
      }
    return out.toArray(double[][]::new);
  }

  /** minX, minY, maxX, maxY of the footprints of these images, or null. */
  double[] extent(Collection<Image> chosen) {
    double[] e = {Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY};
    for (Image im : chosen) {
      double[][] f = footprint(im, 8);
      if (f == null) continue;
      for (double[] p : f) {
        e[0] = Math.min(e[0], p[0]);
        e[1] = Math.min(e[1], p[1]);
        e[2] = Math.max(e[2], p[0]);
        e[3] = Math.max(e[3], p[1]);
      }
    }
    return Double.isFinite(e[0]) ? e : null;
  }

  /** A map raster: world point of its upper-left corner and metres (units) per pixel. */
  record MapView(double left, double top, double unitsPerPixel, int width, int height) {
    double x(double column) {
      return left + column * unitsPerPixel;
    }

    double y(double row) {
      return top - row * unitsPerPixel;
    }
  }

  /**
   * Draws the photographs onto the map in order (later ones on top), each pixel's world height
   * from the surface, colour from the photograph ({@code bitmaps} hold each photograph at any
   * scale), with the given opacity. Pixels no photograph covers stay transparent.
   */
  BufferedImage render(List<Image> order, Map<Integer, BufferedImage> bitmaps, MapView m, float opacity) {
    int[] out = new int[m.width() * m.height()];
    int alpha = Math.round(Math.max(0, Math.min(1, opacity)) * 255);
    for (Image im : order) {
      BufferedImage b = bitmaps.get(im.id());
      if (im.view == null || b == null) continue;
      int bw = b.getWidth(), bh = b.getHeight();
      int[] px = b.getRGB(0, 0, bw, bh, null, 0, bw);
      double sx = bw / (double) im.view.k.width, sy = bh / (double) im.view.k.height;
      // Only the rows/columns inside the footprint's box (with a margin for relief).
      int c0 = 0, c1 = m.width(), r0 = 0, r1 = m.height();
      double[][] f = footprint(im, 8);
      if (f != null) {
        double minX = Double.MAX_VALUE, minY = Double.MAX_VALUE, maxX = -Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
        for (double[] p : f) {
          minX = Math.min(minX, p[0]);
          maxX = Math.max(maxX, p[0]);
          minY = Math.min(minY, p[1]);
          maxY = Math.max(maxY, p[1]);
        }
        double margin = .25 * Math.max(maxX - minX, maxY - minY);
        c0 = (int) Math.max(0, Math.floor((minX - margin - m.left()) / m.unitsPerPixel()));
        c1 = (int) Math.min(m.width(), Math.ceil((maxX + margin - m.left()) / m.unitsPerPixel()));
        r0 = (int) Math.max(0, Math.floor((m.top() - maxY - margin) / m.unitsPerPixel()));
        r1 = (int) Math.min(m.height(), Math.ceil((m.top() - minY + margin) / m.unitsPerPixel()));
      }
      if (c0 >= c1 || r0 >= r1) continue;
      final int fc0 = c0, fc1 = c1;
      View v = im.view;
      IntStream.range(r0, r1).parallel().forEach(row -> {
        double y = m.y(row + .5);
        for (int col = fc0; col < fc1; col++) {
          double x = m.x(col + .5);
          double[] uv = v.project(x, y, surface.z(x, y));
          if (uv == null) continue;
          int iu = (int) Math.floor(uv[0] * sx), iv = (int) Math.floor(uv[1] * sy);
          if (iu < 0 || iv < 0 || iu >= bw || iv >= bh) continue;
          int src = px[iv * bw + iu] & 0xffffff, p = row * m.width() + col, under = out[p];
          if (alpha >= 255 || (under >>> 24) == 0) out[p] = (alpha << 24) | src;
          else out[p] = 0xff000000 | blend(under & 0xffffff, src, alpha);
        }
      });
    }
    BufferedImage image = new BufferedImage(m.width(), m.height(), BufferedImage.TYPE_INT_ARGB);
    image.setRGB(0, 0, m.width(), m.height(), out, 0, m.width());
    return image;
  }

  static int blend(int under, int over, int alpha) {
    int r = (((over >> 16) & 255) * alpha + ((under >> 16) & 255) * (255 - alpha)) / 255;
    int g = (((over >> 8) & 255) * alpha + ((under >> 8) & 255) * (255 - alpha)) / 255;
    int b = ((over & 255) * alpha + (under & 255) * (255 - alpha)) / 255;
    return (r << 16) | (g << 8) | b;
  }

  /**
   * A photograph decoded at reduced size (longest side about {@code longest} pixels; JPEG and TIFF
   * subsampling while reading, so a large file is never fully expanded in memory).
   */
  static BufferedImage decode(Path file, int longest) throws IOException {
    try (javax.imageio.stream.ImageInputStream in = javax.imageio.ImageIO.createImageInputStream(file.toFile())) {
      if (in == null) throw new IOException("Cannot open " + file);
      Iterator<javax.imageio.ImageReader> readers = javax.imageio.ImageIO.getImageReaders(in);
      if (!readers.hasNext()) throw new IOException("No reader for " + file);
      javax.imageio.ImageReader reader = readers.next();
      try {
        reader.setInput(in, true, true);
        int w = reader.getWidth(0), h = reader.getHeight(0);
        int step = Math.max(1, Math.max(w, h) / Math.max(1, longest));
        javax.imageio.ImageReadParam p = reader.getDefaultReadParam();
        p.setSourceSubsampling(step, step, 0, 0);
        BufferedImage image = reader.read(0, p);
        // Plain RGB (some readers give odd colour models).
        BufferedImage rgb = new BufferedImage(image.getWidth(), image.getHeight(), BufferedImage.TYPE_INT_RGB);
        rgb.getGraphics().drawImage(image, 0, 0, null);
        return rgb;
      } finally {
        reader.dispose();
      }
    }
  }
}
