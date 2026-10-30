package sgm;

import java.nio.file.*;
import java.util.*;

final class SurfaceRegression {
  static int checks;

  static void check(boolean yes, String text) {
    checks++;
    if (!yes) throw new AssertionError(text);
  }

  static int run(Path root) throws Exception {
    checks = 0;
    List<SurfaceDiagnostics.Point> samples = new ArrayList<>();
    Random random = new Random(91);
    Calibration calibration = new Calibration(240, 240, 199.5, 149.5, new double[5], 400, 300);
    Camera half = calibration.camera(200, 150);
    org.opencv.core.Mat ramp = new org.opencv.core.Mat(300, 400, org.opencv.core.CvType.CV_32F),
        resized = new org.opencv.core.Mat();
    try {
      float[] row = new float[400];
      for (int x = 0; x < 400; x++) row[x] = x;
      for (int y = 0; y < 300; y++) ramp.put(y, 0, row);
      org.opencv.imgproc.Imgproc.resize(
          ramp,
          resized,
          new org.opencv.core.Size(200, 150),
          0,
          0,
          org.opencv.imgproc.Imgproc.INTER_AREA);
      double sourceU = resized.get(20, 0)[0];
      check(Math.abs(sourceU - .5) < 1e-9, "Independent resize ramp locates source pixel centre");
      check(
          Math.abs((sourceU - calibration.cx) / calibration.fx - (0 - half.cx) / half.fx) < 1e-12,
          "Resized camera ray matches actual resampled source ray");
      check(
          half.cx == 99.5 && half.cy == 74.5,
          "Centred principal point remains centred at half resolution");
    } finally {
      ramp.release();
      resized.release();
    }
    Path run = Files.createDirectories(root.resolve("surface-control/07_results"));
    try (CloudWriter cloud = new CloudWriter(run, 1)) {
      for (int y = 0; y < 60; y++)
        for (int x = 0; x < 60; x++) {
          double X = 400000 + x * .2,
              Y = 7200000 + y * .2,
              Z =
                  -100
                      + .03 * x * .2
                      - .07 * y * .2
                      + random.nextGaussian() * .005; // negative altitudes are valid
          samples.add(new SurfaceDiagnostics.Point(samples.size(), X, Y, Z));
          cloud.point(X, Y, Z, 80, 100, 120);
        }
      for (int i = 0; i < 200; i++)
        samples.add(
            new SurfaceDiagnostics.Point(
                samples.size(),
                400000 + random.nextDouble() * 12,
                7200000 + random.nextDouble() * 12,
                -103 - random.nextDouble() * 2));
    }
    var plane = SurfaceDiagnostics.fit(samples, .03, 20);
    check(
        plane != null && Math.abs(plane.a() - .03) < .001 && Math.abs(plane.b() + .07) < .001,
        "Robust local plane preserves slope despite below-plane outliers");
    Options o = TestSupport.options();
    o.analyzeRun = run.getParent();
    o.analysisOutput = root.resolve("surface-report");
    o.analysisCellSize = 2;
    o.analysisSamples = 5000;
    o.analysisTolerance = .03;
    String before = ReuseCache.hash(run.resolve("dense.ply"));
    SurfaceDiagnostics.analyze(o);
    check(
        before.equals(ReuseCache.hash(run.resolve("dense.ply"))),
        "Surface diagnostics never modify cloud");
    Map<String, Object> report =
        JsonRead.object(JsonRead.read(o.analysisOutput.resolve("surface_summary.json")));
    check(
        JsonRead.number(report.get("median_cell_thickness_p90_p10")) < .02,
        "Controlled thin sloped surface remains thin");
    check(
        Files.isRegularFile(o.analysisOutput.resolve("surface_thickness.svg")),
        "Spatial thickness report produced");
    return checks;
  }
}
