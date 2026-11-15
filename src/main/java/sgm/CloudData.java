package sgm;

import java.io.*;
import java.nio.*;
import java.nio.channels.FileChannel;
import java.nio.file.*;
import java.util.*;
import java.util.function.Consumer;
import java.util.stream.Stream;

/**
 * What the 3D view shows of a run, read from its files (nothing is written): the dense cloud
 * (07_results/dense.las or LAS tiles, every k-th point up to a budget), its per-point surface
 * attributes (LAS extra bytes), views and confidence (07_results/point-confidence.bin, loaded on
 * request), the sparse tie points (04_geometry/sparse.csv) and the cameras (orientation model or
 * cameras.csv, with camera-screening flags). Coordinates are output units relative to
 * {@link #origin} (floats keep millimetres over kilometres).
 */
final class CloudData {
  /** The point attributes the view can colour by (besides RGB, height and classification). */
  record Attribute(String name, String label, float[] values) {}

  /** A camera: centre (relative to the origin), rotation R (X_camera = R X + t), field of view, screening flag. */
  record Cam(String name, double[] center, double[] r, double fx, int width, int height, String flag, Double rms) {}

  final double[] origin = new double[3];
  // Dense cloud (subsampled).
  int n;
  long total;
  int stride = 1;
  float[] x = new float[0], y = new float[0], z = new float[0];
  int[] rgb = new int[0];
  byte[] cls = new byte[0];
  final List<Attribute> attributes = new ArrayList<>();
  final List<Path> lasFiles = new ArrayList<>();
  // Sparse tie points.
  float[] sx = new float[0], sy = new float[0], sz = new float[0];
  /** Optional per-tie-point colour (RGB; null: by the number of images), visibility and emphasis (drawn larger). */
  volatile int[] scolour;
  volatile boolean[] shidden, sbig;
  int[] sobs = new int[0];
  final List<Cam> cameras = new ArrayList<>();
  double multiplier = 1;
  String crs;

  /** Dense clouds of a run: dense.las, else LAS tiles (07_results/**​/tile_*.las). */
  static List<Path> lasFiles(Path run) {
    Path dense = run.resolve("07_results/dense.las");
    if (Files.isRegularFile(dense)) return List.of(dense);
    List<Path> out = new ArrayList<>();
    Path results = run.resolve("07_results");
    if (Files.isDirectory(results))
      try (Stream<Path> s = Files.walk(results, 3)) {
        s.filter(p -> {
          String f = p.getFileName().toString();
          return f.startsWith("tile_") && f.endsWith(".las");
        }).sorted().forEach(out::add);
      } catch (IOException e) {
        // none
      }
    return out;
  }

  /** Loads a run: the dense cloud with at most {@code budget} points, sparse points and cameras. */
  static CloudData load(Path run, long budget, Consumer<String> progress) throws IOException {
    CloudData d = new CloudData();
    Object m = ProjectFiles.map(run.resolve("07_results/coordinates.json")).get("coordinate_multiplier");
    if (!(m instanceof Number mm && mm.doubleValue() > 0)) m = ProjectFiles.map(run.resolve("coordinates.json")).get("coordinate_multiplier");
    d.multiplier = m instanceof Number mm && mm.doubleValue() > 0 ? mm.doubleValue() : 1;
    d.lasFiles.addAll(lasFiles(run));
    RunImages images = RunImages.load(run);
    d.crs = images.crs;
    // Origin: centre of the dense cloud's bounds, else of the cameras / sparse points.
    double[] bounds = null;
    for (Path f : d.lasFiles) bounds = union(bounds, lasBounds(f));
    List<double[]> sparse = readSparse(run.resolve("04_geometry/sparse.csv"));
    if (sparse.isEmpty()) sparse = readSparse(run.resolve("sparse.csv"));
    List<Cam> cams = cameras(run, images, d.multiplier);
    if (bounds != null) for (int k = 0; k < 3; k++) d.origin[k] = Math.floor((bounds[k] + bounds[k + 3]) / 2);
    else if (!sparse.isEmpty() || !cams.isEmpty()) {
      double[] lo = {Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE}, hi = {-Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE};
      for (double[] p : sparse) for (int k = 0; k < 3; k++) { lo[k] = Math.min(lo[k], p[k] * d.multiplier); hi[k] = Math.max(hi[k], p[k] * d.multiplier); }
      for (Cam c : cams) for (int k = 0; k < 3; k++) { lo[k] = Math.min(lo[k], c.center[k]); hi[k] = Math.max(hi[k], c.center[k]); }
      for (int k = 0; k < 3; k++) d.origin[k] = Math.floor((lo[k] + hi[k]) / 2);
    }
    d.sx = new float[sparse.size()];
    d.sy = new float[sparse.size()];
    d.sz = new float[sparse.size()];
    d.sobs = new int[sparse.size()];
    for (int i = 0; i < sparse.size(); i++) {
      double[] p = sparse.get(i);
      d.sx[i] = (float) (p[0] * d.multiplier - d.origin[0]);
      d.sy[i] = (float) (p[1] * d.multiplier - d.origin[1]);
      d.sz[i] = (float) (p[2] * d.multiplier - d.origin[2]);
      d.sobs[i] = (int) p[3];
    }
    for (Cam c : cams)
      d.cameras.add(new Cam(c.name, new double[] {c.center[0] - d.origin[0], c.center[1] - d.origin[1], c.center[2] - d.origin[2]}, c.r, c.fx, c.width, c.height, c.flag, c.rms));
    if (!d.lasFiles.isEmpty()) d.readLas(budget, progress);
    return d;
  }

  /** {minX, minY, minZ, maxX, maxY, maxZ} from a LAS header. */
  static double[] lasBounds(Path las) throws IOException {
    try (FileChannel ch = FileChannel.open(las, StandardOpenOption.READ)) {
      ByteBuffer h = ByteBuffer.allocate(CloudWriter.HEADER_12).order(ByteOrder.LITTLE_ENDIAN);
      ch.read(h, 0);
      if (h.get(0) != 'L' || h.get(1) != 'A' || h.get(2) != 'S' || h.get(3) != 'F') throw new IOException("Not a LAS file: " + las);
      return new double[] {h.getDouble(187), h.getDouble(203), h.getDouble(219), h.getDouble(179), h.getDouble(195), h.getDouble(211)};
    }
  }

  private static double[] union(double[] a, double[] b) {
    if (a == null) return b;
    return new double[] {Math.min(a[0], b[0]), Math.min(a[1], b[1]), Math.min(a[2], b[2]), Math.max(a[3], b[3]), Math.max(a[4], b[4]), Math.max(a[5], b[5])};
  }

  /** LAS extra-bytes layout: names and byte offsets / types within the extra part of a record. */
  record Extra(String name, int offset, int type) {}

  static List<Extra> extras(FileChannel ch, ByteBuffer header) throws IOException {
    List<Extra> out = new ArrayList<>();
    int headerSize = header.getShort(94) & 0xffff;
    long vlrs = header.getInt(100) & 0xffffffffL;
    long at = headerSize;
    for (long v = 0; v < vlrs; v++) {
      ByteBuffer vh = ByteBuffer.allocate(54).order(ByteOrder.LITTLE_ENDIAN);
      ch.read(vh, at);
      String user = new String(vh.array(), 2, 16, java.nio.charset.StandardCharsets.US_ASCII).trim().replace("\0", "");
      int record = vh.getShort(18) & 0xffff, length = vh.getShort(20) & 0xffff;
      if (user.equals("LASF_Spec") && record == 4) {
        ByteBuffer body = ByteBuffer.allocate(length).order(ByteOrder.LITTLE_ENDIAN);
        ch.read(body, at + 54);
        int offset = 0;
        for (int i = 0; i + 192 <= length; i += 192) {
          int type = body.get(i + 2) & 255;
          String name = new String(body.array(), i + 4, 32, java.nio.charset.StandardCharsets.US_ASCII).replace("\0", "").trim();
          out.add(new Extra(name, offset, type));
          offset += switch (type) { case 1, 2 -> 1; case 3, 4 -> 2; case 5, 6, 9 -> 4; case 7, 8, 10 -> 8; default -> 0; };
        }
      }
      at += 54 + length;
    }
    return out;
  }

  /** Every stride-th record of the dense LAS file(s), at most budget points in all. */
  void readLas(long budget, Consumer<String> progress) throws IOException {
    long count = 0;
    for (Path f : lasFiles)
      try (FileChannel ch = FileChannel.open(f, StandardOpenOption.READ)) {
        ByteBuffer h = ByteBuffer.allocate(CloudWriter.HEADER_14).order(ByteOrder.LITTLE_ENDIAN);
        ch.read(h, 0);
        count += CloudWriter.lasPointCount(h);
      }
    total = count;
    stride = (int) Math.max(1, (count + budget - 1) / Math.max(1, budget));
    int capacity = (int) Math.min(Integer.MAX_VALUE - 8, (count + stride - 1) / stride + lasFiles.size());
    x = new float[capacity];
    y = new float[capacity];
    z = new float[capacity];
    rgb = new int[capacity];
    cls = new byte[capacity];
    List<Extra> layout = null;
    float[][] extraValues = null;
    long seen = 0;
    for (Path f : lasFiles)
      try (FileChannel ch = FileChannel.open(f, StandardOpenOption.READ)) {
        ByteBuffer h = ByteBuffer.allocate(CloudWriter.HEADER_14).order(ByteOrder.LITTLE_ENDIAN);
        ch.read(h, 0);
        long dataOffset = h.getInt(96) & 0xffffffffL;
        int format = h.get(104) & 255, recordLength = h.getShort(105) & 0xffff;
        long points = CloudWriter.lasPointCount(h);
        double[] scale = {h.getDouble(131), h.getDouble(139), h.getDouble(147)}, offset = {h.getDouble(155), h.getDouble(163), h.getDouble(171)};
        int colour = format == 2 ? 20 : format == 3 ? 28 : format == 7 ? 30 : format == 8 ? 30 : -1;
        int classAt = format >= 6 ? 16 : 15, base = format <= 5 ? new int[] {20, 28, 26, 34, 57, 63}[format] : new int[] {30, 36, 38, 59, 67}[Math.min(4, format - 6)];
        if (layout == null) {
          layout = extras(ch, h);
          extraValues = new float[layout.size()][];
          for (int k = 0; k < layout.size(); k++) if (useful(layout.get(k))) extraValues[k] = new float[capacity];
        }
        // Records through the shared LAS/LAZ reader (LasFile): the same raw layout either way.
        byte[] raw = new byte[recordLength * 65536];
        ByteBuffer block = ByteBuffer.wrap(raw).order(ByteOrder.LITTLE_ENDIAN);
        LasFile.Reader reader = LasFile.open(f);
        try {
        for (long start = 0; start < points; start += 65536) {
          int records = reader.read(raw, (int) Math.min(65536, points - start));
          if (records <= 0) throw new EOFException(f.toString());
          for (int r = 0; r < records; r++) {
            long index = seen + start + r;
            if (index % stride != 0 || n >= capacity) continue;
            int p = r * recordLength;
            x[n] = (float) (block.getInt(p) * scale[0] + offset[0] - origin[0]);
            y[n] = (float) (block.getInt(p + 4) * scale[1] + offset[1] - origin[1]);
            z[n] = (float) (block.getInt(p + 8) * scale[2] + offset[2] - origin[2]);
            cls[n] = (byte) (format >= 6 ? block.get(p + classAt) : block.get(p + classAt) & 31);
            if (colour >= 0)
              rgb[n] = (block.getShort(p + colour) & 0xffff) >> 8 << 16 | (block.getShort(p + colour + 2) & 0xffff) >> 8 << 8 | (block.getShort(p + colour + 4) & 0xffff) >> 8;
            else rgb[n] = 0xb0b0b0;
            for (int k = 0; k < layout.size(); k++)
              if (extraValues[k] != null) {
                Extra e = layout.get(k);
                int at = p + base + e.offset();
                if (at + 4 > p + recordLength && e.type() == 9) continue;
                extraValues[k][n] = switch (e.type()) {
                  case 1 -> block.get(at) & 255;
                  case 3 -> block.getShort(at) & 0xffff;
                  case 9 -> block.getFloat(at);
                  default -> Float.NaN;
                };
              }
            n++;
          }
          if (progress != null && (start / 65536) % 64 == 0)
            progress.accept(String.format(Locale.ROOT, "Reading %s: %.0f%%", f.getFileName(), 100.0 * (seen + start) / Math.max(1, total)));
        }
        } finally {
          reader.close();
        }
        seen += points;
      }
    if (layout != null)
      for (int k = 0; k < layout.size(); k++)
        if (extraValues[k] != null) attributes.add(new Attribute(layout.get(k).name(), label(layout.get(k).name()), extraValues[k]));
  }

  // Lidar reference of a lidar-referenced run (--lidar-reference), drawn as its own layer.
  float[] lx = new float[0], ly = new float[0], lz = new float[0];
  volatile boolean showLidar;
  long lidarInExtent;

  /** The LAS/LAZ files of the run's lidar reference (04_geometry/lidar/lidar_reference.json), or none. */
  static List<Path> lidarFiles(Path run) {
    Map<String, Object> ref = ProjectFiles.map(run.resolve(LidarReference.FOLDER + "/lidar_reference.json"));
    Object source = ref.get("settings") instanceof Map<?, ?> settings ? settings.get("source") : null;
    if (!(source instanceof String s)) return List.of();
    try {
      return LasFile.files(Path.of(s));
    } catch (IOException | RuntimeException e) {
      return List.of();
    }
  }

  /**
   * Lidar points within the loaded cloud's extent (plus 10 m), noise and withheld points left out,
   * every k-th up to {@code budget}; coordinates relative to the origin like the dense cloud.
   */
  void readLidar(List<Path> files, long budget, Consumer<String> progress) throws IOException {
    float[][] ext = n > 0 ? new float[][] {x, y} : new float[][] {sx, sy};
    int count = n > 0 ? n : sx.length;
    if (count == 0 || files.isEmpty()) return;
    double minX = Double.MAX_VALUE, maxX = -Double.MAX_VALUE, minY = Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
    for (int i = 0; i < count; i++) {
      minX = Math.min(minX, ext[0][i]);
      maxX = Math.max(maxX, ext[0][i]);
      minY = Math.min(minY, ext[1][i]);
      maxY = Math.max(maxY, ext[1][i]);
    }
    minX += origin[0] - 10;
    maxX += origin[0] + 10;
    minY += origin[1] - 10;
    maxY += origin[1] + 10;
    // Expected points in the extent from the headers (uniform density), for the stride.
    double expected = 0;
    List<LasFile.Header> used = new ArrayList<>();
    for (Path f : files) {
      LasFile.Header h = LasFile.header(f);
      double ox = Math.min(maxX, h.max()[0]) - Math.max(minX, h.min()[0]), oy = Math.min(maxY, h.max()[1]) - Math.max(minY, h.min()[1]);
      if (ox <= 0 || oy <= 0) continue;
      double area = Math.max(1, (h.max()[0] - h.min()[0]) * (h.max()[1] - h.min()[1]));
      expected += h.count() * Math.min(1, ox * oy / area);
      used.add(h);
    }
    long step = Math.max(1, (long) Math.ceil(expected / Math.max(1, budget)));
    int cap = (int) Math.min(Math.max(1, budget), Integer.MAX_VALUE - 8);
    float[] ax = new float[Math.min(cap, 1 << 16)], ay = new float[ax.length], az = new float[ax.length];
    int m = 0;
    long inside = 0;
    for (LasFile.Header h : used)
      try (LasFile.Reader r = LasFile.open(h.path())) {
        int rl = h.recordLength(), fmt = h.format();
        byte[] b = new byte[rl * 65536];
        long done = 0;
        for (int k; (k = r.read(b, 65536)) > 0; ) {
          for (int j = 0; j < k; j++) {
            int o = j * rl;
            double px = h.x(LasFile.rawX(b, o)), py = h.y(LasFile.rawY(b, o));
            if (px < minX || px > maxX || py < minY || py > maxY) continue;
            int cls = LasFile.classification(fmt, b, o);
            if (cls == 7 || cls == 18 || LasFile.withheld(fmt, b, o)) continue;
            if (inside++ % step != 0 || m >= cap) continue;
            if (m == ax.length) {
              int grown = (int) Math.min(cap, 2L * ax.length);
              ax = Arrays.copyOf(ax, grown);
              ay = Arrays.copyOf(ay, grown);
              az = Arrays.copyOf(az, grown);
            }
            ax[m] = (float) (px - origin[0]);
            ay[m] = (float) (py - origin[1]);
            az[m] = (float) (h.z(LasFile.rawZ(b, o)) - origin[2]);
            m++;
          }
          done += k;
          if (progress != null && (done >> 16) % 32 == 0)
            progress.accept(String.format(Locale.ROOT, "Reading lidar %s: %.0f%%", h.path().getFileName(), 100.0 * done / Math.max(1, h.count())));
        }
      }
    lx = Arrays.copyOf(ax, m);
    ly = Arrays.copyOf(ay, m);
    lz = Arrays.copyOf(az, m);
    lidarInExtent = inside;
  }

  private static boolean useful(Extra e) {
    return Set.of("normal_z", "roughness_rms", "surface_variation", "planarity").contains(e.name()) && (e.type() == 9 || e.type() == 1 || e.type() == 3);
  }

  static String label(String name) {
    return switch (name) {
      case "normal_z" -> "Normal (vertical part)";
      case "roughness_rms" -> "Roughness";
      case "surface_variation" -> "Surface variation";
      case "planarity" -> "Planarity";
      case "views" -> "Views per point";
      case "confidence" -> "Confidence";
      default -> name;
    };
  }

  /** Views and confidence from 07_results/point-confidence.bin (cloud order), for the loaded points. */
  boolean loadConfidence(Path run, Consumer<String> progress) throws IOException {
    Path file = run.resolve("07_results/point-confidence.bin");
    if (!Files.isRegularFile(file) || lasFiles.size() != 1) return false;
    float[] views = new float[n], confidence = new float[n];
    Arrays.fill(views, Float.NaN);
    Arrays.fill(confidence, Float.NaN);
    try (PointRecords.Reader reader = new PointRecords.Reader(file)) {
      long i = 0;
      for (PointRecords.Row row; (row = reader.next()) != null; i++) {
        long id = row.id();
        if (id % stride != 0) continue;
        long k = id / stride;
        if (k >= n) continue;
        views[(int) k] = row.sample().views;
        confidence[(int) k] = (float) row.score();
        if (progress != null && (i & 0xfffff) == 0) progress.accept(String.format(Locale.ROOT, "Reading point confidence: %.0f%%", 100.0 * i / Math.max(1, total)));
      }
    }
    attributes.removeIf(a -> a.name().equals("views") || a.name().equals("confidence"));
    attributes.add(new Attribute("views", label("views"), views));
    attributes.add(new Attribute("confidence", label("confidence"), confidence));
    return true;
  }

  static List<double[]> readSparse(Path file) {
    List<double[]> out = new ArrayList<>();
    if (!Files.isRegularFile(file)) return out;
    try (BufferedReader r = Files.newBufferedReader(file)) {
      String line = r.readLine();
      while ((line = r.readLine()) != null) {
        String[] c = line.split(",");
        if (c.length < 3) continue;
        try {
          out.add(new double[] {Double.parseDouble(c[0]), Double.parseDouble(c[1]), Double.parseDouble(c[2]), c.length > 3 ? Double.parseDouble(c[3]) : 0});
        } catch (NumberFormatException e) {
          // skipped
        }
      }
    } catch (IOException e) {
      return out;
    }
    return out;
  }

  /** Cameras in output units: the orientation model (with the lens), else cameras.csv (pose only). */
  static List<Cam> cameras(Path run, RunImages images, double multiplier) {
    List<Cam> out = new ArrayList<>();
    for (RunImages.Image im : images.images) {
      if (!im.oriented()) continue;
      RunImages.View v = im.view();
      double[] c = v.center();
      out.add(new Cam(im.name(), new double[] {c[0] * multiplier, c[1] * multiplier, c[2] * multiplier}, v.r(), v.k().fx(), v.k().width(), v.k().height(), im.flag(), im.rms()));
    }
    if (!out.isEmpty()) return out;
    Map<String, String> flags = RunImages.screening(run);
    Map<String, Double> rms = RunImages.reprojection(run);
    for (Path f : List.of(run.resolve("04_geometry/cameras.csv"), run.resolve("cameras.csv"))) {
      if (!Files.isRegularFile(f)) continue;
      for (String[] row : RunImages.csv(f)) {
        if (row.length < 13 || row[0].equals("image")) continue;
        try {
          double[] r = new double[9], t = new double[3];
          for (int i = 0; i < 9; i++) r[i] = Double.parseDouble(row[1 + i]);
          for (int i = 0; i < 3; i++) t[i] = Double.parseDouble(row[10 + i]);
          double[] c = {-(r[0] * t[0] + r[3] * t[1] + r[6] * t[2]), -(r[1] * t[0] + r[4] * t[1] + r[7] * t[2]), -(r[2] * t[0] + r[5] * t[1] + r[8] * t[2])};
          String name = row[0].replace("\"", "");
          out.add(new Cam(name, new double[] {c[0] * multiplier, c[1] * multiplier, c[2] * multiplier}, r, 1.0, 4, 3, flags.get(name), rms.get(name)));
        } catch (NumberFormatException e) {
          // skipped
        }
      }
      break;
    }
    return out;
  }

  /** Quantile of a float array (NaN ignored), for colour ranges. */
  static float quantile(float[] v, int n, double q) {
    int step = Math.max(1, n / 200_000);
    float[] s = new float[(n + step - 1) / step];
    int m = 0;
    for (int i = 0; i < n; i += step) if (!Float.isNaN(v[i])) s[m++] = v[i];
    if (m == 0) return 0;
    Arrays.sort(s, 0, m);
    return s[(int) Math.min(m - 1, Math.max(0, Math.round(q * (m - 1))))];
  }
}
