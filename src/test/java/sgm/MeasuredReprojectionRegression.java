package sgm;

import java.nio.file.*;

final class MeasuredReprojectionRegression {
  public static void main(String[] args) throws Exception {
    System.out.println("PASS measured-view reprojection checks=" + run());
    GpuSgm.closeSession();
  }

  static int run() throws Exception {
    Options o = TestSupport.options();
    o.output = Files.createTempDirectory(Path.of("target"), "measured reprojection ");
    o.fusionRelative = .01;
    o.fusionPixels = 1.25;
    o.maxDepthUncertainty = 1;
    Camera a = new Camera(120, 120, 200, 100), b = a.copy(), c = new Camera(240, 240, 400, 100);
    b.translation[0] = -1;
    c.translation[0] = -10;
    Camera[] cameras = {a, b, c};
    int[] w = {400, 400, 800}, h = {200, 200, 200};
    double[] candidate = {0, 0, 10}, independent = {-.07, 0, 10.095};
    try (ReferenceDepthFusion old = new ReferenceDepthFusion(cameras, w, h, o);
        DepthFusion cpu = new DepthFusion(cameras, w, h, o)) {
      int source = old.addPair(0, 1);
      old.addPair(0, 2);
      cpu.addPair(0, 1);
      cpu.addPair(0, 2);
      old.add(source, candidate);
      old.add(1, independent);
      cpu.add(source, candidate);
      cpu.add(1, independent);
      var legacy = old.fuse(source, candidate);
      var actual = cpu.fuse(source, candidate);
      System.out.println(
          "Measured-view regression: old="
              + legacy.status
              + "/"
              + legacy.views
              + " new="
              + actual.status
              + "/"
              + actual.views);
      if (!legacy.accepted() || legacy.views != 3)
        throw new AssertionError("Fixture must reproduce the original false support");
      if (!actual.status.equals("refinement_support_rejected") || actual.views != 2)
        throw new AssertionError("Refined point must lose the invalid measured-camera support");
      if (!GpuSgm.devices().isEmpty()) {
        Options go = o.copy();
        go.compute = "gpu";
        try (DepthFusion gpu = new DepthFusion(cameras, w, h, go)) {
          gpu.addPair(0, 1);
          gpu.addPair(0, 2);
          gpu.add(0, candidate);
          gpu.add(1, independent);
          gpu.prepare(1);
          var result = gpu.fuseBatch(new int[] {0}, new double[][] {candidate}, 1)[0];
          if (!DepthFusion.equivalent(actual, result))
            throw new AssertionError("GPU measured-camera support differs from CPU");
        }
      }
    }
    return GpuSgm.devices().isEmpty() ? 2 : 3;
  }
}
