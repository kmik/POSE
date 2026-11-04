package sgm;

import java.nio.*;
import java.nio.channels.FileChannel;
import java.nio.file.*;
import java.util.*;

/**
 * GS1 and GS2 on a synthetic block: four stereo pairs see the same terrain and trees, each with a
 * known vertical error (pair 1 +0.6 m, pair 2 -0.2 m with a 4 mm/m tilt, pair 3 +0.1 m). GS1
 * must recover the relative offsets and tilt and remove the two-sheet cells; GS2 must collapse the
 * ground to one thin sheet, merge duplicates, leave the trees alone and keep LAS, PLY and the
 * confidence sidecar aligned.
 */
final class GroundStuffRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Path root = Files.createTempDirectory(Path.of("target"), "ground-stuff-");
    System.out.println("PASS ground pair correction / ground-aware fusion: " + run(root) + "; " + root);
  }

  static final double[] OFFSET = {0, .6, -.2, .1};
  static final double TILT = .004;

  static double terrain(double x, double y) {
    return 90 + .05 * x - .02 * y + .3 * Math.sin(x / 6) * Math.cos(y / 8);
  }

  static double error(int pair, double x) {
    return OFFSET[pair] + (pair == 2 ? TILT * (x - 40) : 0);
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Files.createDirectories(root);
    Random random = new Random(91);
    List<double[]> trees = new ArrayList<>();
    for (int t = 0; t < 40; t++)
      trees.add(new double[] {random.nextDouble() * 80, random.nextDouble() * 80, 8 + random.nextDouble() * 8, 2 + random.nextDouble() * 1.5});
    List<double[]> points = new ArrayList<>();
    List<Integer> pairs = new ArrayList<>();
    List<Boolean> tree = new ArrayList<>();
    for (int pair = 0; pair < 4; pair++)
      for (int i = 0; i < 80_000; i++) {
        double x = random.nextDouble() * 80, y = random.nextDouble() * 80, ground = terrain(x, y);
        double top = Double.NEGATIVE_INFINITY;
        for (double[] t : trees) {
          double d = Math.hypot(x - t[0], y - t[1]);
          if (d < t[3]) top = Math.max(top, ground + t[2] * (1 - d / t[3] * .6));
        }
        boolean isTree = top != Double.NEGATIVE_INFINITY;
        double z = (isTree ? top + random.nextGaussian() * .1 : ground + random.nextGaussian() * .03) + error(pair, x);
        points.add(new double[] {610000 + x, 6930000 + y, z});
        pairs.add(pair);
        tree.add(isTree);
      }
    Path las = root.resolve("dense.las"), sidecar = root.resolve("point-confidence.bin");
    try (CloudWriter writer = new CloudWriter(root, 1, new Crs("EPSG:3067"));
        PointRecords.Writer records = new PointRecords.Writer(sidecar)) {
      for (int i = 0; i < points.size(); i++) {
        double[] p = points.get(i);
        writer.point(p[0], p[1], p[2], 120, 110, 90);
        SurfaceConsolidator.Sample s = sample(i, pairs.get(i), p, tree.get(i));
        records.point(i, s, 1, .5);
      }
    }
    Options o = TestSupport.options();
    o.output = root;
    o.groundPairCorrection = true;
    o.groundAwareFusion = true;
    Map<String, Object> result = GroundClassification.classify(o, List.of(), las, sidecar, root, 3067);
    var pairsReport = ProcessingReport.object(result.get("pair_offsets"));
    Map<Integer, double[]> fitted = new HashMap<>();
    for (Object row : (List<?>) pairsReport.get("pairs")) {
      var r = ProcessingReport.object(row);
      fitted.put(((Number) r.get("pair")).intValue(), new double[] {
        ((Number) r.get("offset_metres")).doubleValue(), ((Number) r.get("tilt_x_per_m")).doubleValue()});
    }
    check(fitted.size() == 4, "all four pairs fitted: " + fitted.keySet());
    for (int p = 1; p < 4; p++) {
      // Offsets are relative (block mean removed); compare differences at x = 40 (pair 2 centre).
      double measured = fitted.get(p)[0] - fitted.get(0)[0], truth = OFFSET[p] - OFFSET[0];
      check(Math.abs(measured - truth) < .06, "pair " + p + " offset " + measured + " vs " + truth);
    }
    check(Math.abs(fitted.get(2)[1] - TILT) < .0015, "pair 2 tilt " + fitted.get(2)[1]);
    var after = ProcessingReport.object(pairsReport.get("after_correction"));
    check(((Number) pairsReport.get("two_sheet_cells_before")).doubleValue() > .5, "layering present before");
    check(((Number) after.get("two_sheet_cells_before")).doubleValue() < .05, "pair correction removes the two-sheet cells: " + after.get("two_sheet_cells_before"));
    var sheet = ProcessingReport.object(result.get("ground_aware_fusion"));
    check("complete".equals(sheet.get("status")), "ground-aware fusion completes");
    double thicknessAfter = ((Number) sheet.get("near_ground_thickness_after_metres")).doubleValue();
    check(thicknessAfter < .12, "one thin ground sheet: " + thicknessAfter);
    long kept = ((Number) sheet.get("points_after")).longValue();
    check(kept < points.size() && ((Number) sheet.get("removed_duplicate_points")).longValue() > 0, "duplicate ground copies merged");
    // Files stay aligned.
    var cloud = GroundClassification.read(las, sidecar);
    check(cloud.n == kept && cloud.sidecar, "LAS and sidecar aligned after the rewrite");
    try (FileChannel ply = FileChannel.open(root.resolve("dense.ply"))) {
      long header = GroundClassification.plyHeaderLength(ply);
      ByteBuffer b = ByteBuffer.allocate((int) header);
      ply.read(b, 0);
      check(new String(b.array(), java.nio.charset.StandardCharsets.US_ASCII).contains("element vertex " + kept), "PLY vertex count updated");
      check((ply.size() - header) % kept == 0, "PLY body matches the count");
    }
    // Trees untouched: count of points well above the terrain is unchanged.
    long treesBefore = 0, treesAfter = 0;
    double sum = 0;
    long ground = 0;
    for (int i = 0; i < points.size(); i++) if (points.get(i)[2] - terrain(points.get(i)[0] - 610000, points.get(i)[1] - 6930000) > 3) treesBefore++;
    byte[] classes = GroundClassificationRegression.classes(las, cloud.n);
    for (int i = 0; i < cloud.n; i++) {
      double t = terrain(cloud.x[i] + cloud.ox - 610000, cloud.y[i] + cloud.oy - 6930000);
      if (cloud.z[i] - t > 3) treesAfter++;
      if (classes[i] == 2) {
        sum += Math.pow(cloud.z[i] - t - (OFFSET[0] + OFFSET[1] + OFFSET[2] + OFFSET[3]) / 4, 2);
        ground++;
      }
    }
    check(Math.abs(treesAfter - treesBefore) <= treesBefore / 100, "tree points kept: " + treesBefore + " -> " + treesAfter);
    check(Math.sqrt(sum / ground) < .1, "ground sheet follows the terrain: " + Math.sqrt(sum / ground));

    // GS2 alone (no pair correction): the sheet merge itself collapses the layers.
    Path alone = root.resolve("sheet-only");
    Files.createDirectories(alone);
    Path las2 = alone.resolve("dense.las"), sidecar2 = alone.resolve("point-confidence.bin");
    try (CloudWriter writer = new CloudWriter(alone, 1, new Crs("EPSG:3067"));
        PointRecords.Writer records = new PointRecords.Writer(sidecar2)) {
      for (int i = 0; i < points.size(); i++) {
        double[] p = points.get(i);
        writer.point(p[0], p[1], p[2], 120, 110, 90);
        records.point(i, sample(i, pairs.get(i), p, tree.get(i)), 1, .5);
      }
    }
    o.output = alone;
    o.groundPairCorrection = false;
    Map<String, Object> second = GroundClassification.classify(o, List.of(), las2, sidecar2, alone, 3067);
    var only = ProcessingReport.object(second.get("ground_aware_fusion"));
    double before = ((Number) only.get("near_ground_thickness_before_metres")).doubleValue();
    double afterOnly = ((Number) only.get("near_ground_thickness_after_metres")).doubleValue();
    check(((Number) only.get("layered_cells_before")).doubleValue() > .5, "layered before the sheet merge");
    check(before > .5, "layered ground is thick before: " + before);
    check(afterOnly < .12, "sheet merge alone leaves one thin sheet: " + afterOnly);
    check(((Number) only.get("two_sheet_cells_before")).doubleValue() > .5, "two-sheet ground cells before the merge");
    check(((Number) only.get("two_sheet_cells_after")).doubleValue() < .02, "no two-sheet ground cells after: " + only.get("two_sheet_cells_after"));
    check(!Boolean.TRUE.equals(ProcessingReport.object(second.get("pair_offsets")).get("applied")), "pair correction off unless requested");
    return checks;
  }

  static SurfaceConsolidator.Sample sample(long id, int pair, double[] p, boolean tree) {
    DepthFusion.Result r = new DepthFusion.Result(p.clone(), 3, 1, .001, 0, "accepted");
    SurfaceConsolidator.Sample s = new SurfaceConsolidator.Sample(id, pair, 0, 0x786e5a, r, new DepthFusion.SurfaceSample(null, 0, 0));
    s.normal = tree ? new double[] {Double.NaN, Double.NaN, Double.NaN} : new double[] {0, 0, 1};
    return s;
  }
}
