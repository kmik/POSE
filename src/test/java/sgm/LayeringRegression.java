package sgm;

import java.nio.file.*;
import java.util.*;
import org.opencv.core.Core;

/** Separates real depth layers from conflicting stereo estimates of one surface. */
final class LayeringRegression {
  private static int checks;

  private static void check(boolean value, String message) {
    checks++;
    if (!value) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Core.setNumThreads(1);
    System.out.println("PASS layer/consensus checks=" + run());
  }

  private static Camera[] rig(double... centers) {
    Camera[] out = new Camera[centers.length];
    for (int i = 0; i < centers.length; i++) {
      out[i] = new Camera(400, 400, 200, 80);
      out[i].translation[0] = -centers[i];
    }
    return out;
  }

  private static int[] dimensions(int n, int value) {
    int[] a = new int[n];
    Arrays.fill(a, value);
    return a;
  }

  static int run() {
    checks = 0;
    Camera[] cameras = rig(-3, -2, -1, 0, 1, 2, 3);
    int[] widths = dimensions(7, 400), heights = dimensions(7, 160);
    Options oldOptions = TestSupport.options();
    oldOptions.fusionRelative = .02;
    oldOptions.fusionPixels = 1.5;
    LegacyLayerFusion old = new LegacyLayerFusion(cameras, widths, heights, oldOptions);
    DepthFusion fusion = new DepthFusion(cameras, widths, heights, TestSupport.options());
    int ghost = -1, truth = -1;
    for (int a = 0; a < 7; a++)
      for (int b = a + 1; b < 7; b++) {
        int id = fusion.addPair(a, b);
        old.addPair(a, b);
        boolean biased = a < 3 && b < 3;
        double[] p = {0, 0, biased ? 20.25 : 20};
        fusion.add(id, p);
        old.add(id, p);
        if (a == 0 && b == 1) ghost = id;
        if (a == 0 && b == 3) truth = id;
      }
    double[] falseLayer = {0, 0, 20.25}, ground = {0, 0, 20};
    check(
        old.consistent(ghost, falseLayer),
        "Legacy 2-percent/any-pair acceptance reproduces a second layer above one true surface");
    check(
        !fusion.fuse(ghost, falseLayer).accepted(),
        "Per-ray majority consensus rejects the minority false depth layer");
    check(fusion.fuse(truth, ground).accepted(), "The dominant true surface is retained");

    Camera[] c = rig(-2, -1, 0, 1, 2);
    DepthFusion noisy =
        new DepthFusion(c, dimensions(5, 400), dimensions(5, 160), TestSupport.options());
    List<Integer> ids = new ArrayList<>();
    List<Double> raw = new ArrayList<>();
    for (int a = 0; a < 5; a++)
      for (int b = a + 1; b < 5; b++) {
        int id = noisy.addPair(a, b);
        double z = 20 + ((a + b) % 2 == 0 ? .02 : -.02);
        noisy.add(id, new double[] {0, 0, z});
        ids.add(id);
        raw.add(z);
      }
    double original = 0,
        refined = 0,
        min = Double.POSITIVE_INFINITY,
        max = Double.NEGATIVE_INFINITY;
    int accepted = 0;
    for (int i = 0; i < ids.size(); i++) {
      double z = raw.get(i);
      DepthFusion.Result result = noisy.fuse(ids.get(i), new double[] {0, 0, z});
      if (result.accepted()) {
        accepted++;
        original += (z - 20) * (z - 20);
        refined += Math.pow(result.point[2] - 20, 2);
        min = Math.min(min, result.point[2]);
        max = Math.max(max, result.point[2]);
      }
    }
    check(
        accepted >= 8,
        "Depth refinement preserves at least 80 percent of valid mildly noisy estimates");
    check(
        refined < original * .64,
        "Multi-pair depth refinement reduces error against a known surface");
    check(max - min < .02, "Refined surface is thinner than the two raw 0.04-unit-separated bands");
    System.out.printf(
        Locale.ROOT,
        "Controlled plane: accepted=%d, raw RMS=%.6f, fused RMS=%.6f, fused thickness=%.6f%n",
        accepted,
        Math.sqrt(original / accepted),
        Math.sqrt(refined / accepted),
        max - min);

    c = rig(-3, -1, 1, 3);
    DepthFusion layers =
        new DepthFusion(c, dimensions(4, 400), dimensions(4, 160), TestSupport.options());
    for (int a = 0; a < 4; a++)
      for (int b = a + 1; b < 4; b++) {
        int id = layers.addPair(a, b);
        layers.add(id, new double[] {0, 0, 16});
        layers.add(id, new double[] {0, 0, 20});
      }
    DepthFusion.Result front = layers.fuse(0, new double[] {0, 0, 16}),
        back = layers.fuse(0, new double[] {0, 0, 20});
    check(
        front.accepted() && back.accepted(),
        "Genuine stacked surfaces visible in different rays are both retained");
    check(
        Math.abs(front.point[2] - 16) < 1e-4 && Math.abs(back.point[2] - 20) < 1e-4,
        "Distinct surfaces are not averaged together or flattened to ground");

    c = rig(0, .25, 100);
    DepthFusion limited =
        new DepthFusion(c, dimensions(3, 400), dimensions(3, 160), TestSupport.options());
    int source = limited.addPair(0, 1), independent = limited.addPair(0, 2);
    limited.add(source, new double[] {0, 0, 5});
    limited.add(independent, new double[] {0, 0, 5});
    check(
        limited.fuse(source, new double[] {0, 0, 5}).views == 2,
        "Agreement in one endpoint does not grant credit to the unobserved other camera");
    LegacyLayerFusion oldLimited =
        new LegacyLayerFusion(c, dimensions(3, 400), dimensions(3, 160), oldOptions);
    int oldSource = oldLimited.addPair(0, 1), oldIndependent = oldLimited.addPair(0, 2);
    oldLimited.add(oldSource, new double[] {0, 0, 5});
    oldLimited.add(oldIndependent, new double[] {0, 0, 5});
    check(
        oldLimited.consistent(oldSource, new double[] {0, 0, 5}),
        "Legacy endpoint union falsely claimed a third camera for the same off-image case");
    c = rig(-.005, 0, .005);
    DepthFusion uncertain =
        new DepthFusion(c, dimensions(3, 400), dimensions(3, 160), TestSupport.options());
    for (int a = 0; a < 3; a++)
      for (int b = a + 1; b < 3; b++) {
        int id = uncertain.addPair(a, b);
        uncertain.add(id, new double[] {0, 0, 20});
      }
    check(
        uncertain.fuse(0, new double[] {0, 0, 20}).status.equals("depth_uncertainty_rejected"),
        "Numerically agreeing depths with negligible parallax still fail uncertainty limit");
    int packed = DepthFusion.pack(.42312, -.3678);
    check(
        Math.abs(DepthFusion.offsetX(packed) - .42312) < 1.6e-5
            && Math.abs(DepthFusion.offsetY(packed) + .3678) < 1.6e-5,
        "Subpixel evidence coordinates survive compact storage");
    return checks;
  }
}
