package sgm;

import java.nio.file.*;
import java.util.*;

/**
 * --dense-method mvsgm hands each camera's map to fusion as pair (camera, best neighbour). The
 * neighbour only weights the triangulation angle, so it must not count as a supporting view where
 * it does not see the point (2026-11-21: very_large edge bands seen by two cameras were recorded as
 * three views and passed --min-views 3, leaving stacked sheets one depth label apart).
 *
 * Three cameras in a row over a plane at depth 10, footprints [-4.75, 4.75], [1.25, 10.75],
 * [7.25, 16.75]; partners 0->1, 1->2, 2->0. With --min-views 2 only the two overlaps may emit.
 */
final class MvsgmPartnerViewsRegression {
  private static int checks;

  private static void check(boolean value, String why) {
    checks++;
    if (!value) throw new AssertionError(why);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Path root = Files.createTempDirectory(Path.of("target"), "partner-views-");
    try {
      System.out.println("PASS mvsgm partner views: " + run(root));
    } finally {
      LargeFormatRegression.delete(root);
    }
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Options o = new Options();
    o.output = Files.createDirectories(root);
    o.projectDirectory = o.output;
    o.denseMethod = "mvsgm";
    o.fusionMode = "image";
    o.minViews = 2;
    o.compute = "cpu";
    o.cores = 2;
    int n = 3, w = 96, h = 40;
    Camera[] cameras = new Camera[n];
    int[] widths = new int[n], heights = new int[n];
    for (int c = 0; c < n; c++) {
      widths[c] = w;
      heights[c] = h;
      cameras[c] = new Camera(100, 100, (w - 1) / 2., (h - 1) / 2.);
      cameras[c].translation[0] = -6 * c; // centre at x = 6c
      cameras[c].registered = true;
    }
    try (var maps = new MvsgmMaps(Files.createDirectories(o.output.resolve("source")), false, false);
        var fusion = new DepthFusion(cameras, widths, heights, o)) {
      long[] starts = new long[n + 1];
      for (int c = 0; c < n; c++) {
        int pair = fusion.addPair(c, (c + 1) % n);
        fusion.markSingleView(pair);
        long[] rows = new long[h + 1];
        for (int y = 0; y <= h; y++) rows[y] = (long) y * w;
        maps.add(new MvsgmMaps.Camera(c, pair, w, h, rows, false));
        starts[c + 1] = starts[c] + (long) w * h;
        for (int k = 0; k < MvsgmMaps.KINDS.length; k++) {
          float[] values = new float[w * h];
          Arrays.fill(values, k == MvsgmMaps.DEPTH ? 10 : k == MvsgmMaps.RGB ? Float.intBitsToFloat(0x2277AA) : k == MvsgmMaps.LR ? 0 : .9f);
          DepthMapFile.write(maps.file(c, k), cameras[c], w, h, values);
        }
      }
      try (FusionOutput output = new FusionOutput(o, fusion, null, null)) {
        Path candidates = Files.createFile(o.output.resolve("empty-candidates"));
        ImageFusion.run(o, fusion, candidates, starts, output, new TwoViewConfidence.Diagnostics(o), maps);
        output.finish();
      }
    }
    List<double[]> points = CloudDiagnostics.readLas(o.output.resolve("dense.las"));
    int inOverlap = 0, single = 0;
    for (double[] p : points) {
      double x = p[0];
      // Overlaps [1.25, 4.75] and [7.25, 10.75], widened by one 0.1 m pixel.
      if (x >= 1.15 && x <= 4.85 || x >= 7.15 && x <= 10.85) inOverlap++;
      else single++;
    }
    check(inOverlap > 0, "Overlaps seen by two cameras emit points");
    check(single == 0, "Pixels seen only by one camera (partner outside) are rejected at --min-views 2: " + single);
    return checks;
  }
}
