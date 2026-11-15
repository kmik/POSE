package sgm;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.*;

/**
 * Tie-point cleanup by quality (the cleanup step, --tie-point-cleanup): like Metashape's gradual
 * selection. Each triangulated tie point gets three measures: its largest reprojection error,
 * the number of registered images that see it and its largest intersection angle (small angles
 * mean an uncertain depth). Points beyond the chosen limits are removed, worst first, up to a
 * share of all points and never below {@link #MIN_SUPPORT} points in any camera; the block is
 * then adjusted again. The selection works on the per-point table alone, so the POSE window's
 * preview (TiePointWindow) and the step remove exactly the same points.
 */
final class TiePointCleanup {
  private TiePointCleanup() {}

  /** Fewest tie points any registered camera keeps (its pose stays determined). */
  static final int MIN_SUPPORT = 15;
  static final String FOLDER = "04_geometry/tie_points";

  /** One triangulated tie point: largest and RMS reprojection error (processed pixels), views, largest angle (degrees), images. */
  record Point(int track, float error, float rms, int views, float angle, int[] cameras, double x, double y, double z) {
    /** Without a position (only the measures). */
    Point(int track, float error, float rms, int views, float angle, int[] cameras) {
      this(track, error, rms, views, angle, cameras, Double.NaN, Double.NaN, Double.NaN);
    }
  }

  /** A camera of the alignment (for the window's 3D view): centre, rotation R (X_camera = R X + t), lens size. */
  record Camera(String name, double[] center, double[] r, double fx, int width, int height) {}

  /** Limits: largest error (px), fewest views, smallest largest-angle (deg), most removed (% of points). */
  record Thresholds(double maxError, int minViews, double minAngle, double maxRemovedPercent) {
    Map<String, Object> json() {
      return Json.object("max_error_pixels", maxError, "min_views", minViews, "min_angle_degrees", minAngle,
          "max_removed_percent", maxRemovedPercent);
    }
  }

  /** What a selection removes. */
  record Selection(boolean[] remove, int removed, int byError, int byViews, int byAngle, int protectedBySupport, int capped) {}

  // ---------------------------------------------------------------------------------------------
  // Measures

  static List<Point> measure(List<ImageFrame> frames, List<FeatureTracks.Track> tracks) {
    List<Point> out = new ArrayList<>();
    for (int i = 0; i < tracks.size(); i++) {
      FeatureTracks.Track t = tracks.get(i);
      if (t.point == null || t.prior != null) continue; // control points are not tie points
      double max = 0, sum = 0;
      List<Integer> cams = new ArrayList<>();
      List<double[]> rays = new ArrayList<>();
      for (FeatureTracks.Observation ob : t.observations) {
        ImageFrame f = frames.get(ob.image);
        if (!f.camera.registered) continue;
        double[] uv = f.camera.project(t.point);
        org.opencv.core.Point m = f.keypoints[ob.feature].pt;
        double e = uv == null ? 1e6 : Math.hypot(uv[0] - m.x, uv[1] - m.y);
        max = Math.max(max, e);
        sum += e * e;
        cams.add(ob.image);
        double[] r = Math3.sub(t.point, f.camera.center());
        double n = Math3.norm(r);
        if (n > 0) rays.add(new double[] {r[0] / n, r[1] / n, r[2] / n});
      }
      if (cams.size() < 2) continue;
      double angle = 0;
      for (int a = 0; a < rays.size(); a++)
        for (int b = a + 1; b < rays.size(); b++) {
          double c = rays.get(a)[0] * rays.get(b)[0] + rays.get(a)[1] * rays.get(b)[1] + rays.get(a)[2] * rays.get(b)[2];
          angle = Math.max(angle, Math.toDegrees(Math.acos(Math.max(-1, Math.min(1, c)))));
        }
      out.add(new Point(i, (float) max, (float) Math.sqrt(sum / cams.size()), cams.size(), (float) angle,
          cams.stream().mapToInt(Integer::intValue).toArray(), t.point[0], t.point[1], t.point[2]));
    }
    return out;
  }

  /** The automatic largest error: three times the median of the points' largest errors (at least 0.5 px). */
  static double autoMaxError(List<Point> points) {
    if (points.isEmpty()) return 0;
    float[] e = new float[points.size()];
    for (int i = 0; i < e.length; i++) e[i] = points.get(i).error();
    Arrays.sort(e);
    return Math.max(.5, Math.round(3 * e[e.length / 2] * 100) / 100.0);
  }

  /** The automatic limits (--tie-point-cleanup auto): largest error 3x the median, 2 images, 1 degree, 20%. */
  static Thresholds automatic(List<Point> points) {
    return new Thresholds(autoMaxError(points), 2, 1, 20);
  }

  /** The limits of --tie-point-* (custom; a largest error of 0 means automatic). */
  static Thresholds thresholds(Options o, List<Point> points) {
    return new Thresholds(o.tiePointMaxError > 0 ? o.tiePointMaxError : autoMaxError(points), o.tiePointMinViews,
        o.tiePointMinAngle, o.tiePointMaxRemoved);
  }

  // ---------------------------------------------------------------------------------------------
  // Selection

  static Selection select(List<Point> points, int cameras, Thresholds t) {
    int n = points.size();
    boolean[] remove = new boolean[n];
    int[] support = new int[Math.max(cameras, 1)];
    for (Point p : points) for (int c : p.cameras()) if (c < support.length) support[c]++;
    int byError = 0, byViews = 0, byAngle = 0;
    List<Integer> candidates = new ArrayList<>();
    int[] failed = new int[n];
    for (int i = 0; i < n; i++) {
      Point p = points.get(i);
      boolean e = t.maxError() > 0 && p.error() > t.maxError(), v = p.views() < t.minViews(), a = p.angle() < t.minAngle();
      if (e) byError++;
      if (v) byViews++;
      if (a) byAngle++;
      failed[i] = (e ? 1 : 0) + (v ? 1 : 0) + (a ? 1 : 0);
      if (failed[i] > 0) candidates.add(i);
    }
    // Worst first: most limits failed, then the largest error, then the smallest angle.
    candidates.sort(Comparator.<Integer>comparingInt(i -> -failed[i])
        .thenComparingDouble(i -> -points.get(i).error())
        .thenComparingDouble(i -> points.get(i).angle())
        .thenComparingInt(i -> points.get(i).track()));
    long cap = (long) Math.floor(n * Math.max(0, Math.min(100, t.maxRemovedPercent())) / 100.0);
    int removed = 0, protectedCount = 0, capped = 0;
    for (int i : candidates) {
      if (removed >= cap) {
        capped++;
        continue;
      }
      boolean safe = true;
      for (int c : points.get(i).cameras()) if (c < support.length && support[c] - 1 < MIN_SUPPORT) safe = false;
      if (!safe) {
        protectedCount++;
        continue;
      }
      for (int c : points.get(i).cameras()) if (c < support.length) support[c]--;
      remove[i] = true;
      removed++;
    }
    return new Selection(remove, removed, byError, byViews, byAngle, protectedCount, capped);
  }

  /** RMS of the points' RMS errors weighted by their views, without the removed ones. */
  static double rms(List<Point> points, boolean[] remove) {
    double sum = 0;
    long n = 0;
    for (int i = 0; i < points.size(); i++) {
      if (remove != null && remove[i]) continue;
      Point p = points.get(i);
      sum += (double) p.rms() * p.rms() * p.views();
      n += p.views();
    }
    return Math.sqrt(sum / Math.max(1, n));
  }

  // ---------------------------------------------------------------------------------------------
  // Files: the per-point table (for the window) and the window's choice

  static void write(Path file, List<Point> points) throws IOException {
    Files.createDirectories(file.getParent());
    Path tmp = file.resolveSibling(file.getFileName() + ".tmp");
    try (BufferedWriter w = Files.newBufferedWriter(tmp, StandardCharsets.UTF_8)) {
      // Position (alignment frame, before georeferencing) for the window's 3D view; images last.
      w.write("track,max_error_px,rms_error_px,views,max_angle_deg,x,y,z,images\n");
      for (Point p : points) {
        StringBuilder s = new StringBuilder();
        for (int c : p.cameras()) s.append(s.isEmpty() ? "" : ";").append(c);
        w.write(String.format(Locale.ROOT, "%d,%.4f,%.4f,%d,%.3f,%.9g,%.9g,%.9g,%s%n", p.track(), p.error(), p.rms(), p.views(), p.angle(),
            p.x(), p.y(), p.z(), s));
      }
    }
    Files.move(tmp, file, StandardCopyOption.REPLACE_EXISTING);
  }

  static List<Point> read(Path file) throws IOException {
    List<Point> out = new ArrayList<>();
    try (BufferedReader r = Files.newBufferedReader(file, StandardCharsets.UTF_8)) {
      String line = r.readLine();
      boolean positions = line != null && line.contains(",x,y,z,");
      while ((line = r.readLine()) != null) {
        if (line.isBlank()) continue;
        String[] c = line.split(",", -1);
        String list = c[positions ? 8 : 5];
        String[] ids = list.isEmpty() ? new String[0] : list.split(";");
        int[] cams = new int[ids.length];
        for (int i = 0; i < ids.length; i++) cams[i] = Integer.parseInt(ids[i]);
        out.add(new Point(Integer.parseInt(c[0]), Float.parseFloat(c[1]), Float.parseFloat(c[2]), Integer.parseInt(c[3]),
            Float.parseFloat(c[4]), cams, positions ? Double.parseDouble(c[5]) : Double.NaN, positions ? Double.parseDouble(c[6]) : Double.NaN,
            positions ? Double.parseDouble(c[7]) : Double.NaN));
      }
    }
    return out;
  }

  /**
   * The tie points and cameras as 3D view data (the window's view), in display order of
   * {@code points} (index i is point i). The alignment frame has no "up" before georeferencing:
   * the data is turned so that the cameras' mean viewing direction points down (a nadir flight
   * looks as in the georeferenced 3D view) and centred on the points' median.
   */
  static CloudData cloud(List<Point> points, List<Camera> cameras) {
    double[] up = {0, 0, 1};
    if (!cameras.isEmpty()) {
      double[] f = new double[3];
      for (Camera c : cameras) for (int k = 0; k < 3; k++) f[k] += c.r()[6 + k]; // R^T (0, 0, 1)
      double n = Math.sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
      if (n > 0) up = new double[] {-f[0] / n, -f[1] / n, -f[2] / n};
    }
    double[] ex = Math.abs(up[0]) < .9 ? new double[] {1, 0, 0} : new double[] {0, 1, 0};
    double dot = ex[0] * up[0] + ex[1] * up[1] + ex[2] * up[2];
    double[] x = {ex[0] - dot * up[0], ex[1] - dot * up[1], ex[2] - dot * up[2]};
    double xn = Math.sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
    for (int k = 0; k < 3; k++) x[k] /= xn;
    double[] y = {up[1] * x[2] - up[2] * x[1], up[2] * x[0] - up[0] * x[2], up[0] * x[1] - up[1] * x[0]};
    double[][] q = {x, y, up};
    double[] centre = new double[3];
    for (int k = 0; k < 3; k++) {
      int axis = k;
      double[] v = points.stream().mapToDouble(p -> axis == 0 ? p.x() : axis == 1 ? p.y() : p.z()).filter(Double::isFinite).sorted().toArray();
      centre[k] = v.length == 0 ? 0 : v[v.length / 2];
    }
    CloudData d = new CloudData();
    int n = points.size();
    d.sx = new float[n];
    d.sy = new float[n];
    d.sz = new float[n];
    d.sobs = new int[n];
    boolean[] hidden = new boolean[n];
    for (int i = 0; i < n; i++) {
      Point p = points.get(i);
      d.sobs[i] = p.views();
      if (!Double.isFinite(p.x())) {
        hidden[i] = true;
        continue;
      }
      double[] a = {p.x() - centre[0], p.y() - centre[1], p.z() - centre[2]};
      d.sx[i] = (float) (q[0][0] * a[0] + q[0][1] * a[1] + q[0][2] * a[2]);
      d.sy[i] = (float) (q[1][0] * a[0] + q[1][1] * a[1] + q[1][2] * a[2]);
      d.sz[i] = (float) (q[2][0] * a[0] + q[2][1] * a[1] + q[2][2] * a[2]);
    }
    d.shidden = hidden;
    for (Camera c : cameras) {
      double[] a = {c.center()[0] - centre[0], c.center()[1] - centre[1], c.center()[2] - centre[2]};
      double[] center = {q[0][0] * a[0] + q[0][1] * a[1] + q[0][2] * a[2], q[1][0] * a[0] + q[1][1] * a[1] + q[1][2] * a[2],
          q[2][0] * a[0] + q[2][1] * a[1] + q[2][2] * a[2]};
      // X_camera = R X = R Q^T X' (X' = Q X): the rotation in the turned frame is R Q^T.
      double[] r = c.r(), rq = new double[9];
      for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) rq[3 * i + j] = r[3 * i] * q[j][0] + r[3 * i + 1] * q[j][1] + r[3 * i + 2] * q[j][2];
      d.cameras.add(new CloudData.Cam(c.name(), center, rq, c.fx(), c.width(), c.height(), null, null));
    }
    return d;
  }

  static final int REMOVED_COLOUR = 0xff3b30, KEPT_COLOUR = 0xc8ced8;

  /** Colours the points of a selection: removed red (drawn larger), kept grey or hidden. */
  static void colour(CloudData d, List<Point> points, boolean[] remove, boolean showKept) {
    int n = points.size();
    int[] c = new int[n];
    boolean[] hidden = new boolean[n], big = new boolean[n];
    for (int i = 0; i < n; i++) {
      boolean r = remove != null && remove[i];
      c[i] = r ? REMOVED_COLOUR : KEPT_COLOUR;
      big[i] = r;
      hidden[i] = !Double.isFinite(points.get(i).x()) || !r && !showKept;
    }
    d.scolour = c;
    d.sbig = big;
    d.shidden = hidden;
  }

  /** The registered cameras of the alignment (04_geometry/tie_points/cameras.csv). */
  static void writeCameras(Path file, List<ImageFrame> frames) throws IOException {
    Files.createDirectories(file.getParent());
    try (BufferedWriter w = Files.newBufferedWriter(file, StandardCharsets.UTF_8)) {
      w.write("image_id,cx,cy,cz,r00,r01,r02,r10,r11,r12,r20,r21,r22,fx,width,height,image\n");
      for (ImageFrame f : frames) {
        if (!f.camera.registered) continue;
        double[] c = f.camera.center(), r = f.camera.rotation;
        StringBuilder s = new StringBuilder().append(f.id);
        for (double v : c) s.append(',').append(String.format(Locale.ROOT, "%.9g", v));
        for (double v : r) s.append(',').append(String.format(Locale.ROOT, "%.9g", v));
        s.append(String.format(Locale.ROOT, ",%.6g,%d,%d,", f.camera.fx, f.width, f.height)).append(f.path == null ? "" : f.path.getFileName().toString().replace(",", "_"));
        w.write(s + "\n");
      }
    }
  }

  static List<Camera> readCameras(Path file) throws IOException {
    List<Camera> out = new ArrayList<>();
    if (!Files.isRegularFile(file)) return out;
    for (String line : Files.readAllLines(file, StandardCharsets.UTF_8)) {
      String[] c = line.split(",", -1);
      if (c.length < 17 || c[0].equals("image_id")) continue;
      double[] center = new double[3], r = new double[9];
      for (int i = 0; i < 3; i++) center[i] = Double.parseDouble(c[1 + i]);
      for (int i = 0; i < 9; i++) r[i] = Double.parseDouble(c[4 + i]);
      out.add(new Camera(c[16], center, r, Double.parseDouble(c[13]), Integer.parseInt(c[14]), Integer.parseInt(c[15])));
    }
    return out;
  }

  /** The window's choice: apply these limits, or skip the cleanup. */
  record Choice(boolean apply, Thresholds thresholds) {}

  static Path choiceFile(Path run) {
    return run.resolve(FOLDER).resolve("selection.json");
  }

  static void writeChoice(Path run, Choice c) throws IOException {
    Map<String, Object> v = new LinkedHashMap<>();
    v.put("action", c.apply() ? "apply" : "skip");
    if (c.thresholds() != null) v.putAll(c.thresholds().json());
    Files.createDirectories(choiceFile(run).getParent());
    Json.write(choiceFile(run), v);
  }

  static Choice readChoice(Path run) throws IOException {
    Path f = choiceFile(run);
    if (!Files.isRegularFile(f)) return null;
    Map<String, Object> v = JsonRead.object(JsonRead.read(f));
    boolean apply = "apply".equals(v.get("action"));
    if (!apply) return new Choice(false, null);
    return new Choice(true, new Thresholds(number(v, "max_error_pixels"), (int) number(v, "min_views"),
        number(v, "min_angle_degrees"), number(v, "max_removed_percent")));
  }

  private static double number(Map<String, Object> v, String key) {
    Object o = v.get(key);
    if (!(o instanceof Number n)) throw new IllegalArgumentException(key + " missing in the tie-point cleanup choice");
    return n.doubleValue();
  }

  /** The cameras' count of tie points after a selection (registered cameras only appear). */
  static int weakestSupport(List<Point> points, boolean[] remove, int cameras) {
    int[] support = new int[Math.max(1, cameras)];
    for (int i = 0; i < points.size(); i++)
      if (remove == null || !remove[i]) for (int c : points.get(i).cameras()) if (c < support.length) support[c]++;
    int min = Integer.MAX_VALUE;
    for (int s : support) if (s > 0) min = Math.min(min, s);
    return min == Integer.MAX_VALUE ? 0 : min;
  }
}
