package sgm;

import java.io.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.file.*;
import java.util.*;

/**
 * CloudScore on a synthetic scene (sloped ground, flat-roofed blocks with walls, so a horizontal
 * shift is observable): the rigid fit recovers a known shift and rotation, a saved frame gives the
 * same numbers again, and each metric reacts to the defect it is meant to catch (bowl -> warp,
 * missing strip -> completeness, copied sheet -> double sheets, sunken points -> far outliers).
 */
final class CloudScoreRegression {
  private static int checks;

  private static void check(boolean value, String why) {
    checks++;
    if (!value) throw new AssertionError(why);
  }

  public static void main(String[] args) throws Exception {
    Path root = Files.createTempDirectory(Path.of("target"), "cloud-score-");
    try {
      System.out.println("PASS cloud score: " + run(root));
    } finally {
      LargeFormatRegression.delete(root);
    }
  }

  static final double X0 = 400_000, Y0 = 6_900_000, Z0 = 100;

  /** Height of the scene's top surface at (x, y) (local metres), and whether it is a roof. */
  static double ground(double x, double y) {
    return .02 * x + .01 * y;
  }

  static final double[][] BLOCKS = {{8, 8, 18, 16, 6}, {30, 10, 42, 22, 9}, {12, 34, 24, 48, 7}, {36, 36, 50, 50, 11}};

  /** Samples the scene on a jittered 0.12 m grid (top surfaces) plus walls; local coordinates. */
  static List<double[]> scene(long seed, double noise) {
    Random r = new Random(seed);
    List<double[]> out = new ArrayList<>();
    double step = .12;
    for (double x = 0; x < 60; x += step)
      for (double y = 0; y < 60; y += step) {
        double px = x + r.nextDouble() * step, py = y + r.nextDouble() * step, z = ground(px, py);
        for (double[] b : BLOCKS) if (px >= b[0] && px < b[2] && py >= b[1] && py < b[3]) z = ground(b[0], b[1]) + b[4];
        out.add(new double[] {px, py, z + r.nextGaussian() * noise});
      }
    for (double[] b : BLOCKS) {
      double base = ground(b[0], b[1]);
      for (double h = .1; h < b[4]; h += step)
        for (double s = 0; s < 1; s += step / Math.max(b[2] - b[0], b[3] - b[1])) {
          double[][] walls = {
            {b[0] + s * (b[2] - b[0]), b[1]}, {b[0] + s * (b[2] - b[0]), b[3]}, {b[0], b[1] + s * (b[3] - b[1])}, {b[2], b[1] + s * (b[3] - b[1])}
          };
          for (double[] w : walls) {
            double gz = ground(w[0], w[1]);
            if (base + h < gz) continue;
            out.add(new double[] {w[0] + r.nextGaussian() * noise, w[1] + r.nextGaussian() * noise, base + h});
          }
        }
    }
    return out;
  }

  /** Applies p -> c + t + R(a) (p - c) with R about z by angle a (radians), in local metres. */
  static List<double[]> moved(List<double[]> in, double[] t, double angle) {
    double c = Math.cos(angle), s = Math.sin(angle), cx = 30, cy = 30;
    List<double[]> out = new ArrayList<>(in.size());
    for (double[] p : in) {
      double x = p[0] - cx, y = p[1] - cy;
      out.add(new double[] {cx + t[0] + c * x - s * y, cy + t[1] + s * x + c * y, p[2] + t[2]});
    }
    return out;
  }

  /** Minimal LAS 1.2 point format 0 writer (local coordinates + X0/Y0/Z0, 1 mm scale). */
  static void writeLas(Path p, List<double[]> pts, int classification) throws IOException {
    double[] min = {Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE}, max = {-Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE};
    for (double[] q : pts)
      for (int k = 0; k < 3; k++) {
        double v = q[k] + (k == 0 ? X0 : k == 1 ? Y0 : Z0);
        min[k] = Math.min(min[k], v);
        max[k] = Math.max(max[k], v);
      }
    ByteBuffer h = ByteBuffer.allocate(227).order(ByteOrder.LITTLE_ENDIAN);
    h.put("LASF".getBytes(), 0, 4);
    h.put(24, (byte) 1).put(25, (byte) 2);
    h.putShort(94, (short) 227).putInt(96, 227).putInt(100, 0);
    h.put(104, (byte) 0).putShort(105, (short) 20).putInt(107, pts.size()).putInt(111, pts.size());
    h.putDouble(131, .001).putDouble(139, .001).putDouble(147, .001);
    h.putDouble(155, X0).putDouble(163, Y0).putDouble(171, Z0);
    h.putDouble(179, max[0]).putDouble(187, min[0]).putDouble(195, max[1]).putDouble(203, min[1]).putDouble(211, max[2]).putDouble(219, min[2]);
    ByteBuffer b = ByteBuffer.allocate(20 * pts.size()).order(ByteOrder.LITTLE_ENDIAN);
    for (double[] q : pts) {
      b.putInt((int) Math.round(q[0] / .001)).putInt((int) Math.round(q[1] / .001)).putInt((int) Math.round(q[2] / .001));
      b.putShort((short) 0).put((byte) 0x11).put((byte) classification).put((byte) 0).put((byte) 0).putShort((short) 0);
    }
    try (OutputStream o = Files.newOutputStream(p)) {
      o.write(h.array());
      o.write(b.array());
    }
  }

  static CloudScore.Result score(String... args) throws IOException {
    return CloudScore.score(CloudScore.Settings.parse(args));
  }

  static double num(CloudScore.Result r, String... path) {
    Object v = r;
    for (String k : path) v = ((CloudScore.Result) v).get(k);
    return v instanceof Double d ? d : Double.NaN;
  }

  static double[] vec(CloudScore.Result r, String... path) {
    Object v = r;
    for (String k : path) v = ((CloudScore.Result) v).get(k);
    return (double[]) v;
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Files.createDirectories(root);
    Path ref = root.resolve("reference.las"), base = root.resolve("cloud.las"), frame = root.resolve("frame.properties");
    writeLas(ref, scene(1, .005), 2);
    List<double[]> truth = scene(2, .01);
    // The cloud sits 0.40 / -0.30 / 0.50 m and 0.05 deg away from the reference; the fit must undo it.
    double[] shift = {.40, -.30, .50};
    double angle = Math.toRadians(.05);
    writeLas(base, moved(truth, shift, angle), 0);

    CloudScore.Result r = score("--cloud", base.toString(), "--reference", ref.toString(), "--write-frame", frame.toString(), "--samples", "200000");
    double[] t = vec(r, "position", "shift_m"), rot = vec(r, "position", "rotation_deg");
    check(Math.abs(t[0] + shift[0]) < .02 && Math.abs(t[1] + shift[1]) < .02 && Math.abs(t[2] + shift[2]) < .01,
        "Rigid fit recovers the shift: " + Arrays.toString(t));
    check(Math.abs(rot[2] + .05) < .01, "Rigid fit recovers the rotation about z: " + Arrays.toString(rot));
    check(num(r, "shape", "stable", "median_m") < .02, "Stable median distance after the fit is noise level: " + num(r, "shape", "stable", "median_m"));
    check(Math.abs(num(r, "shape", "stable", "bias_m")) < .01, "No vertical bias after the fit");
    check(num(r, "completeness", "covered_fraction") > .97, "Full cloud covers the area: " + num(r, "completeness", "covered_fraction"));
    check(num(r, "flat_noise", "median_rms_m") > .005 && num(r, "flat_noise", "median_rms_m") < .02,
        "Flat noise measures the 1 cm noise: " + num(r, "flat_noise", "median_rms_m"));
    double warp = num(r, "deformation", "warp_std_m");
    check(warp < .01, "Rigid cloud has no warp: " + warp);
    check(num(r, "artefacts", "double_sheet_cells_fraction") < .01, "Clean cloud has no double sheets");
    check(num(r, "artefacts", "far_below_fraction") == 0, "Clean cloud has no far outliers");

    // Same cloud in the saved frame: identical shape numbers, nothing refitted.
    CloudScore.Result again = score("--cloud", base.toString(), "--reference", ref.toString(), "--frame", frame.toString(), "--samples", "200000");
    check(num(again, "shape", "all", "p90_m") == num(r, "shape", "all", "p90_m"), "Saved frame reproduces the result");
    check("none".equals(((CloudScore.Result) again.get("position")).get("fit")), "Saved frame is not refitted");

    // Ground mode (lidar class 2; the synthetic reference is all class 2): 6 m cells, 3 m planes.
    CloudScore.Result gr = score("--cloud", base.toString(), "--reference", ref.toString(), "--stable", "ground", "--samples", "200000");
    double[] gt = vec(gr, "position", "shift_m");
    check("ground".equals(((CloudScore.Result) gr.get("position")).get("stable_mode")) && num(gr, "position", "stable_cells") > 20,
        "Ground mode finds stable ground cells: " + num(gr, "position", "stable_cells"));
    check(Math.abs(gt[2] + shift[2]) < .03, "Ground mode recovers the vertical shift: " + Arrays.toString(gt));
    check(num(gr, "shape", "stable", "ground_hit_fraction") > .5 && Double.isNaN(num(gr, "flat_noise", "median_rms_m")),
        "Ground mode reports ground hits and skips flat noise");
    check(num(gr, "completeness", "covered_fraction") > .9, "Ground mode completeness on 2 m cells: " + num(gr, "completeness", "covered_fraction"));

    // Bowl: +0.4 m at the corners relative to the centre; the frozen frame keeps it as warp.
    List<double[]> bowl = new ArrayList<>();
    for (double[] p : moved(truth, shift, angle)) {
      double dx = p[0] - 30, dy = p[1] - 30;
      bowl.add(new double[] {p[0], p[1], p[2] + .4 * (dx * dx + dy * dy) / 1800});
    }
    Path bowlLas = root.resolve("bowl.las");
    writeLas(bowlLas, bowl, 0);
    CloudScore.Result b = score("--cloud", bowlLas.toString(), "--reference", ref.toString(), "--frame", frame.toString(), "--samples", "200000", "--tile", "10");
    check(num(b, "deformation", "warp_std_m") > .05, "Bowl shows as warp: " + num(b, "deformation", "warp_std_m"));

    // Missing strip (x > 40 of 60): completeness drops to about two thirds.
    List<double[]> strip = new ArrayList<>();
    for (double[] p : moved(truth, shift, angle)) if (p[0] < 40) strip.add(p);
    Path stripLas = root.resolve("strip.las");
    writeLas(stripLas, strip, 0);
    CloudScore.Result s = score("--cloud", stripLas.toString(), "--reference", ref.toString(), "--frame", frame.toString(), "--samples", "200000");
    double cov = num(s, "completeness", "covered_fraction");
    check(cov > .6 && cov < .72, "Missing third lowers completeness: " + cov);

    // A copied sheet 5 m above the ground (x < 20) and 1% of points 50 m down.
    List<double[]> bad = new ArrayList<>(moved(truth, shift, angle));
    int n = bad.size();
    for (int i = 0; i < n; i++) {
      double[] p = bad.get(i);
      if (p[0] < 20 && Math.abs(p[2] - ground(p[0], p[1]) - shift[2]) < .2) bad.add(new double[] {p[0], p[1], p[2] + 5});
    }
    for (int i = 0; i < n; i += 100) bad.get(i)[2] -= 50;
    Path badLas = root.resolve("bad.las");
    writeLas(badLas, bad, 0);
    CloudScore.Result a = score("--cloud", badLas.toString());
    check(num(a, "artefacts", "double_sheet_cells_fraction") > .1, "Copied sheet shows as double sheets: " + num(a, "artefacts", "double_sheet_cells_fraction"));
    double below = num(a, "artefacts", "far_below_fraction");
    check(below > .005 && below < .015, "Sunken points show as far outliers: " + below);
    check(!a.containsKey("shape"), "Without a reference only reference-free metrics");
    return checks;
  }
}
