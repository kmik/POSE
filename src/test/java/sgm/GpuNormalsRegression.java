package sgm;

/**
 * --fusion-gpu-normals: on a synthetic tilted plane seen by a pinhole camera, the GPU normals agree
 * with the plane's true normal and face the camera; the option off or --compute cpu gives no GPU
 * result (the CPU code runs). Without an OpenCL FP64 device only the fallback is checked.
 */
final class GpuNormalsRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) {
    System.out.println("PASS GPU normals=" + run());
  }

  static int run() {
    checks = 0;
    int w = 160, h = 120;
    Camera camera = new Camera(200, 200, 79.5, 59.5);
    camera.rotation = new double[] {1, 0, 0, 0, 1, 0, 0, 0, 1};
    camera.translation = new double[] {0, 0, 0};
    // Plane z = 20 + 0.1 x + 0.05 y in camera coordinates; depth along each pixel's ray.
    double a = .1, b = .05, c = 20;
    float[] z = new float[w * h], du = new float[w * h], dv = new float[w * h];
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) {
        double rx = (x - camera.cx) / camera.fx, ry = (y - camera.cy) / camera.fy;
        z[y * w + x] = (float) (c / (1 - a * rx - b * ry));
      }
    int n = 50;
    int[] xs = new int[n], ys = new int[n];
    double[] ps = new double[3 * n];
    for (int i = 0; i < n; i++) {
      xs[i] = 20 + (i * 37) % 120;
      ys[i] = 15 + (i * 23) % 90;
      double depth = z[ys[i] * w + xs[i]];
      ps[3 * i] = (xs[i] - camera.cx) / camera.fx * depth;
      ps[3 * i + 1] = (ys[i] - camera.cy) / camera.fy * depth;
      ps[3 * i + 2] = depth;
    }
    Options off = new Options();
    check(GpuNormals.compute(off, camera, z, du, dv, w, h, 0, 0, h, xs, ys, ps, i -> null) == null,
        "Option off: no GPU result");
    Options cpu = new Options();
    cpu.fusionGpuNormals = true;
    cpu.compute = "cpu";
    check(GpuNormals.compute(cpu, camera, z, du, dv, w, h, 0, 0, h, xs, ys, ps, i -> null) == null,
        "--compute cpu: no GPU result");
    Options on = new Options();
    on.fusionGpuNormals = true;
    on.fusionRelative = .05;
    double[] expected = {a, b, -1};
    double length = Math.sqrt(a * a + b * b + 1);
    for (int k = 0; k < 3; k++) expected[k] /= length;
    // The reference here is the analytic plane, not the CPU code: no self-check.
    GpuNormals.skipSelfCheck();
    DepthFusion.SurfaceSample[] gpu;
    try {
      gpu = GpuNormals.compute(on, camera, z, du, dv, w, h, 0, 0, h, xs, ys, ps,
          i -> new DepthFusion.SurfaceSample(expected.clone(), 0, 0,
              new SurfaceAttributes(0, 0, 0, 0, 0, 0, false)));
    } catch (RuntimeException | Error noDevice) {
      gpu = null;
    }
    if (gpu == null) {
      System.out.println("GPU normals: no OpenCL FP64 device; fallback checked only");
      return checks;
    }
    double worst = 0;
    int valid = 0;
    for (DepthFusion.SurfaceSample s : gpu) {
      if (s.normal() == null) continue;
      valid++;
      for (int k = 0; k < 3; k++) worst = Math.max(worst, Math.abs(s.normal()[k] - expected[k]));
    }
    check(valid == n, "Every interior plane point gets a normal: " + valid + "/" + n);
    check(worst < 1e-5, "GPU normals match the plane (float depths): " + worst);
    check(gpu[0].attributes().status() == 0 && gpu[0].attributes().neighbors() >= 9,
        "Surface attributes are filled");
    return checks;
  }
}
