package sgm;

import java.io.IOException;
import java.nio.file.*;
import java.util.*;

/** Conservative frusta, raw-consistency readers, weighted lifetimes and disk refusal. */
final class CameraWindowRegression {
  private static int checks;
  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Files.createDirectories(root);
    Options options = new Options();
    options.denseMethod = "mvsgm";
    options.fusionMode = "image";
    check(MvsgmWindow.enabled(options), "auto enables the overlap window");
    options.streamedProcessing = "off";
    check(!MvsgmWindow.enabled(options), "off preserves separate phases");
    options.streamedProcessing = "auto";
    String oldWindow = System.getProperty("sgm.mvsgm.window");
    try {
      System.setProperty("sgm.mvsgm.window", "false");
      check(!MvsgmWindow.enabled(options), "window property restores the sequential reference");
    } finally {
      if (oldWindow == null) System.clearProperty("sgm.mvsgm.window");
      else System.setProperty("sgm.mvsgm.window", oldWindow);
    }
    CameraWindow small = block(20, false), large = block(40, false);
    check(small.peakCameras == 5 && large.peakCameras == 5, "map window stays five cameras as block grows");
    check(small.sourcePeakBytes == large.sourcePeakBytes, "source-map disk bound does not grow with block length");
    long[] a = new long[20], b = new long[40];
    Arrays.fill(a, 123456);
    Arrays.fill(b, 123456);
    check(small.peakBytes(a, false) == 5 * 123456L && large.peakBytes(b, false) == 5 * 123456L,
        "weighted map-memory bound depends on the window");
    check(block(40, true).peakCameras == 40, "precision revisits/retention require all maps");
    check(blockWithLateRawReader().rawLast[0] == 19, "later consistency reader extends raw depth lifetime");
    check(blockWithLateRawReader().sourcePeakBytes > small.sourcePeakBytes,
        "disk bound includes raw maps still needed after fusion retirement");
    CameraWindow distant = makeBlock(40, false, 2);
    long oneSource = 24L * 320 * 240 + 6L * DepthMapFile.HEADER;
    check(distant.peakCameras == 5 && distant.sourcePeakBytes == 6 * oneSource,
        "a distant raw input adds one retained map, not every intervening camera");
    CameraWindow twoStrips = flight(2, 10), sixStrips = flight(6, 10);
    CameraWindow tenStrips = flight(10, 10);
    check(twoStrips.peakCameras == 20, "two overlapping serpentine strips require the whole small block");
    check(sixStrips.peakCameras < 60, "additional flight strips do not retain the entire block");
    check(tenStrips.peakCameras == sixStrips.peakCameras && tenStrips.sourcePeakBytes == sixStrips.sourcePeakBytes,
        "flight window and source-map bound stay fixed when more strips are added");
    System.out.printf(Locale.ROOT, "Serpentine fixture: 2 strips %d/20 merged cameras, 6 strips %d/60;"
        + " 10 strips %d/100; source-map bounds %d, %d and %d cameras%n", twoStrips.peakCameras, sixStrips.peakCameras,
        tenStrips.peakCameras, twoStrips.sourcePeakBytes / oneSource, sixStrips.sourcePeakBytes / oneSource,
        tenStrips.sourcePeakBytes / oneSource);

    MultiViewSgm.Labels labels = new MultiViewSgm.Labels(.015, .0001, 101, .015, .025, 100);
    Random random = new Random(9931);
    for (int i = 0; i < 200; i++) {
      Camera c = camera(random, 20), other = camera(random, 30);
      boolean possible = CameraWindow.maySee(c, 320, 240, labels, other, 320, 240);
      for (int p = 0; p < 40; p++) {
        double x = random.nextDouble() * 322 - 1, y = random.nextDouble() * 242 - 1;
        double z = 1 / labels.q(random.nextDouble() * 100);
        double[] point = MultiViewSgm.world(c, Math3.transpose(c.rotation), x, y, z);
        double[] v = other.view(point);
        if (v[2] <= 0) continue;
        double u = other.fx * v[0] / v[2] + other.cx, t = other.fy * v[1] / v[2] + other.cy;
        if (u >= 0 && t >= 0 && u < 320 && t < 240)
          check(possible, "geometric window contains every visible sample");
      }
    }
    Camera bad = new Camera(600, 600, 160, 120);
    bad.rotation[0] = Double.NaN;
    check(CameraWindow.maySee(bad, 320, 240, labels, camera(new Random(1), 1), 320, 240),
        "indeterminate geometry retains the dependency");
    String available = System.getProperty("sgm.mvsgm.diskAvailableMb");
    try {
      System.setProperty("sgm.mvsgm.diskAvailableMb", "1");
      boolean refused = false;
      try {
        MvsgmWindow.checkDisk(root, 2L << 20, 1L << 20, 5, 40);
      } catch (IOException e) {
        refused = e.getMessage().contains("available 1 MiB") && e.getMessage().contains("no temporary maps");
      }
      check(refused, "disk shortage is refused before map creation");
    } finally {
      if (available == null) System.clearProperty("sgm.mvsgm.diskAvailableMb");
      else System.setProperty("sgm.mvsgm.diskAvailableMb", available);
    }
    return checks;
  }

  private static Camera camera(Random random, double distance) {
    Camera c = new Camera(600, 610, 160, 120);
    double roll = random.nextDouble() * 6, pitch = random.nextDouble() - .5;
    double cr = Math.cos(roll), sr = Math.sin(roll), cp = Math.cos(pitch), sp = Math.sin(pitch);
    c.rotation = new double[] {cr * cp, -sr, cr * sp, sr * cp, cr, sr * sp, -sp, 0, cp};
    c.translation = new double[] {distance * (random.nextDouble() - .5), distance * (random.nextDouble() - .5), 0};
    return c;
  }

  private static CameraWindow block(int count, boolean retain) {
    return makeBlock(count, retain, 0);
  }

  private static CameraWindow blockWithLateRawReader() {
    return makeBlock(20, false, 1);
  }

  private static CameraWindow makeBlock(int n, boolean retain, int special) {
    Camera[] cameras = new Camera[n];
    int[] widths = new int[n], heights = new int[n];
    MultiViewSgm.Plan[] plans = new MultiViewSgm.Plan[n];
    for (int c = 0; c < n; c++) {
      cameras[c] = new Camera(600, 600, 160, 120);
      cameras[c].translation[0] = -(1000 * (c / 5) + 4 * (c % 5));
      widths[c] = 320;
      heights[c] = 240;
      List<MultiViewSgm.Neighbor> neighbours = new ArrayList<>();
      for (int j = c / 5 * 5; j < c / 5 * 5 + 5; j++) if (j != c)
        neighbours.add(new MultiViewSgm.Neighbor(j, 1, 100, 4));
      if (special == 1 && c == 19) neighbours.add(new MultiViewSgm.Neighbor(0, 1, 100, 4));
      if (special == 2 && c == 0) neighbours.add(new MultiViewSgm.Neighbor(n - 1, 1, 100, 4));
      plans[c] = MultiViewSgm.Plan.header(c, neighbours,
          new MultiViewSgm.Labels(.015, .0001, 101, .015, .025, 100));
    }
    return new CameraWindow(cameras, widths, heights, plans, retain);
  }

  private static CameraWindow flight(int strips, int length) {
    int n = strips * length;
    Camera[] cameras = new Camera[n];
    int[] widths = new int[n], heights = new int[n];
    MultiViewSgm.Plan[] plans = new MultiViewSgm.Plan[n];
    for (int c = 0; c < n; c++) {
      int strip = c / length, row = c % length;
      cameras[c] = new Camera(600, 600, 160, 120);
      cameras[c].translation = new double[] {-27 * strip, -15 * (strip % 2 == 0 ? row : length - 1 - row), 0};
      widths[c] = 320;
      heights[c] = 240;
    }
    for (int c = 0; c < n; c++) {
      final int own = c;
      List<Integer> nearby = new ArrayList<>();
      for (int j = 0; j < n; j++) if (j != c) nearby.add(j);
      nearby.sort(Comparator.comparingDouble(j -> Math3.norm(Math3.sub(cameras[own].translation, cameras[j].translation))));
      List<MultiViewSgm.Neighbor> selected = nearby.subList(0, Math.min(6, nearby.size())).stream()
          .map(j -> new MultiViewSgm.Neighbor(j, 1, 100, 4)).toList();
      plans[c] = MultiViewSgm.Plan.header(c, selected,
          new MultiViewSgm.Labels(.015, .0001, 101, .015, .025, 100));
    }
    return new CameraWindow(cameras, widths, heights, plans, false);
  }

  public static void main(String[] args) throws Exception {
    Path root = Files.createTempDirectory(Files.createDirectories(Path.of("target")), "camera-window-");
    try {
      System.out.println("PASS camera window: " + run(root));
    } finally {
      LargeFormatRegression.delete(root);
    }
  }
}
