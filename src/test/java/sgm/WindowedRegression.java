package sgm;

import java.nio.file.*;
import java.util.*;

/** Small bounded controls; never runs native-resolution photographs. */
public final class WindowedRegression {
  static long checks;

  static void check(boolean condition, String message) {
    checks++;
    if (!condition) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    org.opencv.core.Core.setNumThreads(1);
    Path root = Files.createDirectories(Path.of("target/windowed-revision/controls"));
    boolean gpu = args.length > 0 && args[0].equals("gpu");
    if (args.length > 0 && args[0].equals("images")) {
      images(root);
      return;
    }
    for (String cost : new String[] {"census", "ad-census", "census-gradient", "zncc"})
      for (int rows : new int[] {5, 17}) stereo(root, cost, rows, gpu);
    for (String variant : new String[] {"color", "photometric", "four-path-wide"})
      stereo(root, variant, 7, gpu);
    memory();
    System.out.println("PASS windowed controls: " + checks);
    GpuSgm.closeSession();
  }

  static Object field(Object object, String name) throws Exception {
    var f = object.getClass().getDeclaredField(name);
    f.setAccessible(true);
    return f.get(object);
  }

  static void compareImage(
      org.opencv.core.Mat mat,
      WindowedRasters.Image image,
      PagedVolume values,
      int channels,
      String label) {
    byte[] data = new byte[(int) mat.total() * channels];
    mat.get(0, 0, data);
    long different = 0;
    int maximum = 0;
    for (int p = 0; p < mat.total(); p++) {
      int value = values.get(p);
      for (int c = 0; c < channels; c++) {
        int d = Math.abs((data[p * channels + c] & 255) - (value >>> (8 * c) & 255));
        if (d != 0) different++;
        maximum = Math.max(maximum, d);
      }
    }
    System.out.println(label + " different=" + different + " maximum=" + maximum);
    check(different == 0, label + " exact raster parity");
  }

  static void images(Path root) throws Exception {
    Options o = new Options();
    o.output = Files.createDirectories(root.resolve("images"));
    o.quality = "custom";
    o.maxDimension = 400;
    o.imageReader = "windowed";
    o.memoryMb = 512;
    var model =
        OrientationIO.read(
            Path.of("target/two-view-experiment/frozen-cameras.json"), "native", Map.of(), o);
    List<ImageFrame> frames = new ArrayList<>();
    for (var v : model.views.values()) {
      var f =
          new ImageFrame(
              frames.size(),
              Path.of("../example_data/forest/forest_tiny").resolve(v.image()),
              v.calibration(),
              o);
      f.camera.rotation = v.pose().rotation.clone();
      f.camera.translation = v.pose().translation.clone();
      f.camera.registered = true;
      frames.add(f);
      if (frames.size() == 2) break;
    }
    try (var store = new WindowedRasters(o)) {
      for (var f : frames) {
        var image = store.source(f);
        compareImage(f.color, image, image.color, 3, "source color " + f.id);
        compareImage(f.gray, image, image.gray, 1, "source gray " + f.id);
        compareImage(f.validMask, image, image.mask, 1, "source mask " + f.id);
        image.discard();
      }
      var dense = new DenseReconstruction(o, frames);
      var method =
          DenseReconstruction.class.getDeclaredMethod(
              "rectify", ImageFrame.class, ImageFrame.class, boolean.class, int.class);
      method.setAccessible(true);
      Object rectified = method.invoke(dense, frames.get(0), frames.get(1), false, 0);
      try {
        boolean vertical = (boolean) field(rectified, "vertical");
        for (String side : new String[] {"left", "right"}) {
          ImageFrame f = (ImageFrame) field(rectified, side.equals("left") ? "a" : "b");
          var source = store.source(f);
          var image =
              store.warp(
                  source,
                  f.camera,
                  new double[5],
                  (double[]) field(rectified, side.equals("left") ? "r1" : "r2"),
                  (double[]) field(rectified, side.equals("left") ? "p1" : "p2"),
                  vertical,
                  false);
          store.erode(image);
          compareImage(
              (org.opencv.core.Mat) field(rectified, side + "Color"),
              image,
              image.color,
              3,
              side + " rectified color");
          compareImage(
              (org.opencv.core.Mat) field(rectified, side + "Gray"),
              image,
              image.gray,
              1,
              side + " rectified gray");
          compareImage(
              (org.opencv.core.Mat) field(rectified, side + "Mask"),
              image,
              image.mask,
              1,
              side + " rectified mask");
        }
      } finally {
        ((AutoCloseable) rectified).close();
      }
    } finally {
      for (var f : frames) f.close();
    }
  }

  static void memory() {
    long m = 1048576L;
    check(MemorySafety.arrayBytes(5221920, 4, 16 * m) == 32 * m, "G1 directory rounding");
    check(
        CameraEvidence.allocation(5221920, 0, 16 * m) >= 64 * m, "Two rounded camera directories");
    var low = new MemorySafety.Snapshot(10 * m, 16 * m, 1024 * m, 100 * m, 100 * m, m);
    check(low.allowance(800 * m) == 0, "Low native commit is refused before allocation");
    var ample = new MemorySafety.Snapshot(10 * m, 16 * m, 1024 * m, 10L * 1024 * m, 1024 * m, m);
    check(ample.allowance(100 * m) == 100 * m, "Safe modest allocation admitted");
    check(
        new MemorySafety.Snapshot(10 * m, 16 * m, 1024 * m, 0, 100 * m, m).allowance(800 * m) == 0,
        "Zero available commit is not unknown");
    List<List<DepthFusion.Endpoint>> endpoints = new ArrayList<>();
    int[] widths = new int[89], heights = new int[89];
    for (int i = 0; i < 89; i++) {
      endpoints.add(List.of(new DepthFusion.Endpoint(i, i, null)));
      widths[i] = 2640;
      heights[i] = 1978;
    }
    check(
        CameraEvidence.build(endpoints, widths, heights, 32 * m, Long.MAX_VALUE, true) == null,
        "89-camera directories rejected before reading maps or allocating masks");
    var live = MemorySafety.snapshot();
    if (System.getProperty("os.name").toLowerCase(Locale.ROOT).contains("win"))
      check(
          live.availableCommit() > 0 && live.availableCommit() < 1L << 50,
          "Live Windows commit reading has a plausible byte scale");
  }

  static void stereo(Path root, String cost, int rows, boolean gpu) throws Exception {
    int w = 81, h = 59, n = w * h;
    Options o = new Options();
    o.output = Files.createDirectories(root.resolve(cost + "-" + rows + (gpu ? "-gpu" : "-cpu")));
    o.compute = gpu ? "gpu" : "cpu";
    o.gpuDevice = Integer.getInteger("sgm.test.gpu", 0);
    o.memoryMb = 256;
    o.gpuMemoryMb = 256;
    o.disparities = 15;
    o.minDisparity = -3;
    o.census = 5;
    o.matchingCost = cost;
    o.gpuStripRows = rows;
    o.median = 3;
    o.speckleSize = 7;
    o.correlation = .2;
    o.twoViewConfidenceThreshold = .5;
    if (cost.equals("color")) {
      o.matchingCost = "census";
      o.censusMode = "color-distance";
      o.census = 9;
    }
    if (cost.equals("photometric")) {
      o.matchingCost = "census";
      o.census = 7;
      o.subpixelRefinement = "zncc";
    }
    if (cost.equals("four-path-wide")) {
      o.matchingCost = "census";
      o.census = 11;
      o.directions = 4;
      o.p2 = 20000;
    }
    byte[] a = new byte[n], b = new byte[n], ma = new byte[n], mb = new byte[n];
    new Random(733).nextBytes(a);
    new Random(33).nextBytes(b);
    Arrays.fill(ma, (byte) 255);
    Arrays.fill(mb, (byte) 255);
    for (int y = 0; y < h; y++) for (int x = 0; x < w - 5; x++) b[y * w + x] = a[y * w + x + 5];
    for (int i = 0; i < n; i++)
      if (i % 79 == 0) ma[i] = 0;
      else if (i % 89 == 0) mb[i] = 0;
    byte[] ac = new byte[n * 3], bc = new byte[n * 3];
    for (int i = 0; i < n; i++)
      for (int channel = 0; channel < 3; channel++) {
        ac[i * 3 + channel] = a[i];
        bc[i * 3 + channel] = b[i];
      }
    var reference = new CensusSgm(o).compute(a, b, ac, bc, w, h, ma, mb);
    PointConfidence.stereo(reference, a, b);
    try (var store = new WindowedRasters(o)) {
      var left = store.new Image(w, h);
      var right = store.new Image(w, h);
      for (int i = 0; i < n; i++) {
        left.gray.set(i, a[i] & 255);
        right.gray.set(i, b[i] & 255);
        left.mask.set(i, ma[i] & 255);
        right.mask.set(i, mb[i] & 255);
        left.color.set(i, (a[i] & 255) * 0x010101);
        right.color.set(i, (b[i] & 255) * 0x010101);
      }
      try (var matcher = new WindowedStereo(store, left, right, o)) {
        matcher.compute();
        float[][] values = {
          reference.disparity,
          reference.costMargin,
          reference.costSharpness,
          reference.correlation,
          reference.leftRightError,
          reference.confidence,
          reference.rightConfidence
        };
        int[] fields = {
          WindowedStereo.FINAL,
          WindowedStereo.LM,
          WindowedStereo.LS,
          WindowedStereo.PHOTO,
          WindowedStereo.LR,
          WindowedStereo.CONFIDENCE,
          WindowedStereo.REVERSE
        };
        for (int f = 0; f < fields.length; f++)
          for (int p = 0; p < n; p++) {
            float expected = values[f][p], actual = matcher.get(p, fields[f]);
            check(
                Float.floatToIntBits(expected) == Float.floatToIntBits(actual),
                cost
                    + " rows="
                    + rows
                    + " field="
                    + fields[f]
                    + " pixel="
                    + p
                    + " expected="
                    + expected
                    + " actual="
                    + actual);
          }
        check(reference.stats.valid == matcher.stats.valid, "Final membership count");
      }
    }
    try (var paths = Files.list(o.output)) {
      check(paths.noneMatch(p -> p.toString().endsWith(".tiles")), "Owned scratch removed");
    }
  }
}
