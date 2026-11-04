package sgm;

import java.nio.*;
import java.nio.channels.FileChannel;
import java.nio.file.*;
import java.util.*;

/**
 * Synthetic terrain (slope plus bumps) with conical trees and a closed-canopy block: ground recall
 * and precision, DTM accuracy where ground is observed, the free-space ray bound, the slope
 * envelope at a cloud edge, and the in-place LAS class update.
 */
final class GroundClassificationRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Path root = Files.createTempDirectory(Path.of("target"), "ground-");
    System.out.println("PASS ground classification: " + run(root) + "; " + root);
  }

  static boolean tall(double x, double y) {
    return x > 10 && x < 50 && y > 70 && y < 110;
  }

  static double terrain(double x, double y) {
    return 90 + .08 * x - .03 * y + .4 * Math.sin(x / 7) * Math.cos(y / 9);
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Files.createDirectories(root);
    Random random = new Random(77);
    List<double[]> points = new ArrayList<>();
    List<Boolean> truth = new ArrayList<>();
    // Trees: cones 6-18 m tall, 2-4 m crown radius; ground visible between them.
    List<double[]> trees = new ArrayList<>();
    for (int t = 0; t < 120; t++)
      trees.add(new double[] {random.nextDouble() * 120, random.nextDouble() * 120, 6 + random.nextDouble() * 12, 2 + random.nextDouble() * 2});
    for (int i = 0; i < 400_000; i++) {
      double x = random.nextDouble() * 120, y = random.nextDouble() * 120, ground = terrain(x, y);
      double top = Double.NEGATIVE_INFINITY;
      for (double[] t : trees) {
        double d = Math.hypot(x - t[0], y - t[1]);
        if (d < t[3]) top = Math.max(top, ground + t[2] * (1 - d / t[3] * .6));
      }
      // Closed canopy in one 25 m block: only crowns visible there.
      // Closed stand wider than the filter window, crowns with 3 m deep depressions between them.
      boolean closed = x > 65 && x < 110 && y > 15 && y < 60;
      if (closed)
        top = Math.max(top, ground + 14 + 3 * Math.abs(Math.sin(x / 1.5) * Math.cos(y / 1.5)));
      // Tall stand (canopy 24-27 m) with small lower crowns 10-12 m up seen through gaps: the
      // lowest visible points, far below the canopy top, but metres above the terrain.
      if (tall(x, y)) {
        double gx = x - 7 * Math.floor(x / 7) - 3.5, gy = y - 7 * Math.floor(y / 7) - 3.5;
        top = Math.hypot(gx, gy) < 1.3 ? ground + 10 + 2 * Math.abs(Math.sin(x)) : ground + 24 + 3 * Math.abs(Math.sin(x / 1.7) * Math.cos(y / 1.7));
      }
      boolean isGround = top == Double.NEGATIVE_INFINITY;
      double z = isGround ? ground + random.nextGaussian() * .04 : top + random.nextGaussian() * .1;
      points.add(new double[] {610000 + x, 6930000 + y, z});
      truth.add(isGround);
    }
    Path las = root.resolve("dense.las");
    try (CloudWriter writer = new CloudWriter(root, 1, new Crs("EPSG:3067"))) {
      for (double[] p : points) writer.point(p[0], p[1], p[2], 100, 120, 90);
    }
    Options o = TestSupport.options();
    o.output = root;
    Map<String, Object> result =
        GroundClassification.classify(o, List.of(), las, root.resolve("none.bin"), root, 3067);
    check("complete".equals(result.get("status")), "classification completes");
    byte[] classes = classes(las, points.size());
    long tp = 0, fp = 0, fn = 0;
    for (int i = 0; i < points.size(); i++) {
      boolean labelled = classes[i] == 2;
      if (labelled && truth.get(i)) tp++;
      else if (labelled) fp++;
      else if (truth.get(i)) fn++;
    }
    double recall = tp / (double) (tp + fn), precision = tp / (double) (tp + fp);
    check(recall > .95, "ground recall " + recall);
    check(precision > .98, "ground precision " + precision);
    long insideStand = 0;
    for (int i = 0; i < points.size(); i++) {
      double x = points.get(i)[0] - 610000, y = points.get(i)[1] - 6930000;
      if (classes[i] == 2 && x > 67 && x < 108 && y > 17 && y < 58) insideStand++;
    }
    check(insideStand < points.size() / 2000, "no crown depressions taken as ground inside the closed stand: " + insideStand);
    long insideTall = 0;
    for (int i = 0; i < points.size(); i++) {
      double x = points.get(i)[0] - 610000, y = points.get(i)[1] - 6930000;
      if (classes[i] == 2 && x > 12 && x < 48 && y > 72 && y < 108) insideTall++;
    }
    check(insideTall < points.size() / 2000, "no lower crowns taken as ground inside the tall stand: " + insideTall);
    boolean known = true;
    for (byte b : classes) known &= b == 1 || b == 2 || b == 7;
    check(known, "only classes 1, 2, 7 written");
    // DTM accuracy where ground is observed, and interpolation under the closed block.
    float[] dtm = readTiff(root.resolve("dtm.tif"));
    var checksJson = ProcessingReport.object(result.get("checks"));
    check(
        ((Number) checksJson.get("split_half_median_abs_difference")).doubleValue() < .05,
        "independent halves agree: " + checksJson.get("split_half_median_abs_difference"));
    double cell = ((Number) result.get("cell_size")).doubleValue();
    var cloud = GroundClassification.read(las, root.resolve("none.bin"));
    var grid = GroundClassification.grid(cloud, cell);
    double sum = 0, closedSum = 0, tallSum = 0;
    int n = 0, closedN = 0, tallN = 0;
    for (int r = 0; r < grid.h(); r++)
      for (int c = 0; c < grid.w(); c++) {
        double x = grid.x0() + (c + .5) * cell + cloud.ox - 610000, y = grid.y1() - (r + .5) * cell + cloud.oy - 6930000;
        float v = dtm[r * grid.w() + c];
        if (!Float.isFinite(v) || x < 2 || y < 2 || x > 118 || y > 118) continue;
        double e = v - terrain(x, y);
        if (x > 70 && x < 105 && y > 20 && y < 55) {
          closedSum += e * e;
          closedN++;
        } else if (x > 5 && x < 55 && y > 65 && y < 115) {
          tallSum += e * e;
          tallN++;
        } else {
          sum += e * e;
          n++;
        }
      }
    check(Math.sqrt(sum / n) < .25, "DTM RMSE where ground is visible: " + Math.sqrt(sum / n));
    check(Math.sqrt(closedSum / closedN) < 1.5, "interpolated DTM under closed canopy: " + Math.sqrt(closedSum / closedN));
    check(Math.sqrt(tallSum / tallN) < 1.5, "interpolated DTM under the tall stand: " + Math.sqrt(tallSum / tallN));

    // Free-space bound: a ray from a camera to a ground point caps the cells it crosses.
    GroundClassification.Cloud c = new GroundClassification.Cloud();
    c.n = 1;
    c.x = new float[] {10};
    c.y = new float[] {10};
    c.z = new float[] {0};
    c.camA = new int[] {0};
    c.camB = new int[] {-1};
    var g = new GroundClassification.Grid(40, 40, .5, 0, 20);
    float[][] bound =
        GroundClassification.carve(c, new double[][] {{20, 10, 100}}, g, new boolean[] {true}, 20);
    int k = g.index(10.75, 10);
    check(Float.isFinite(bound[0][k]) && Math.abs(bound[0][k] - 6.6) < 1.2, "ray height recorded near the point: " + bound[0][k]);
    check(Float.isNaN(bound[1][k]), "one ray alone does not set the robust bound");
    check(Float.isNaN(bound[0][g.index(10, 15)]), "cells off the ray stay unbounded");

    // Slope envelope: a column standing 10 m above flat ground is removed, the flat ground kept.
    float[] z = new float[100];
    boolean[] ground = new boolean[100];
    Arrays.fill(ground, true);
    z[55] = 10;
    var line = new GroundClassification.Grid(10, 10, .5, 0, 5);
    GroundClassification.slopeEnvelope(z, ground, line, Math.tan(Math.toRadians(35)), .5);
    check(!ground[55], "steep spike rejected by the slope envelope");
    int kept = 0;
    for (boolean b : ground) if (b) kept++;
    check(kept == 99, "flat ground kept");
    return checks;
  }

  static byte[] classes(Path las, int n) throws Exception {
    var cloud = GroundClassification.read(las, Path.of("none"));
    byte[] out = new byte[n];
    try (FileChannel ch = FileChannel.open(las)) {
      ByteBuffer b = ByteBuffer.allocate(1);
      for (int i = 0; i < n; i++) {
        b.clear();
        ch.read(b, cloud.dataOffset + (long) i * cloud.recordLength + 15);
        out[i] = b.get(0);
      }
    }
    return out;
  }

  /** Reads the single-strip Float32 TIFF written by DsmRaster.writeTiff. */
  static float[] readTiff(Path file) throws Exception {
    ByteBuffer b = ByteBuffer.wrap(Files.readAllBytes(file)).order(ByteOrder.LITTLE_ENDIAN);
    int ifd = b.getInt(4), entries = b.getShort(ifd) & 0xffff, w = 0, h = 0, offset = 0;
    for (int e = 0; e < entries; e++) {
      int p = ifd + 2 + 12 * e, tag = b.getShort(p) & 0xffff, value = b.getInt(p + 8);
      if (tag == 256) w = value;
      if (tag == 257) h = value;
      if (tag == 273) offset = value;
    }
    float[] out = new float[w * h];
    for (int i = 0; i < out.length; i++) out[i] = b.getFloat(offset + 4 * i);
    return out;
  }
}
