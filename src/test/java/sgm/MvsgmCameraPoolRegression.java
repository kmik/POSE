package sgm;

import java.io.IOException;
import java.nio.file.*;
import java.util.*;
import java.util.concurrent.atomic.AtomicInteger;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;

/** --mvsgm-cameras-in-flight: pool behaviour, option handling, and maps equal to the serial path. */
final class MvsgmCameraPoolRegression {
  private static int checks;
  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally(); Core.setNumThreads(1);
    Path root = Files.createTempDirectory(Path.of("target"), "camera-pool-");
    System.out.println("PASS mvsgm cameras in flight=" + run(root));
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Files.createDirectories(root);
    options(root);
    pool();
    maps(root);
    return checks;
  }

  private static void options(Path root) {
    check(new Options().mvsgmCamerasInFlight == 1, "Default is one camera at a time");
    String[] base = {"-i", root.toString(), "--mvsgm-cameras-in-flight"};
    Options three = Options.parse(new String[] {base[0], base[1], base[2], "3"});
    check(three.mvsgmCamerasInFlight == 3 && three.copy().mvsgmCamerasInFlight == 3, "Value parsed and copied");
    check(Options.parse(new String[] {base[0], base[1], base[2], "auto"}).mvsgmCamerasInFlight == 0, "auto is 0");
    for (String bad : new String[] {"17", "-1"}) {
      boolean refused = false;
      try {
        Options.parse(new String[] {base[0], base[1], "--dense-method", "mvsgm", "--fusion-mode", "image", base[2], bad});
      } catch (IllegalArgumentException e) {
        refused = e.getMessage().contains("--mvsgm-cameras-in-flight");
      }
      check(refused, "Value " + bad + " is refused");
    }
  }

  private static void pool() throws Exception {
    int[] order = {0, 1, 2, 3, 4, 5, 6, 7};
    AtomicInteger running = new AtomicInteger(), most = new AtomicInteger(), calls = new AtomicInteger();
    MvsgmCameraPool.Task slow = camera -> {
      most.accumulateAndGet(running.incrementAndGet(), Math::max);
      calls.incrementAndGet();
      try {
        Thread.sleep(30);
      } catch (InterruptedException e) {
        throw new IOException(e);
      } finally {
        running.decrementAndGet();
      }
      return "camera " + camera;
    };
    try (MvsgmCameraPool p = new MvsgmCameraPool(3, 6, order, slow)) {
      check(p.rowCores == 2, "Row workers are shared between the cameras in flight");
      for (int c : order) check(p.await(c).equals("camera " + c), "Result belongs to its camera");
      check(most.get() > 1 && most.get() <= 3, "Several cameras ran at once, never more than requested");
      check(calls.get() == order.length, "Every camera computed once");
      var report = p.report();
      check((int) report.get("peak_cameras_ahead") <= 3 && (int) report.get("requests_outside_predicted_order") == 0,
          "Bounded read-ahead in the predicted order");
      boolean twice = false;
      try {
        p.await(0);
      } catch (IllegalStateException e) {
        twice = true;
      }
      check(twice, "A camera cannot be taken twice");
    }
    // Requests outside the predicted order start their camera at once; the bound still holds.
    most.set(0);
    try (MvsgmCameraPool p = new MvsgmCameraPool(2, 4, order, slow)) {
      check(p.await(7).equals("camera 7") && p.await(5).equals("camera 5"), "Unpredicted requests are served");
      for (int c : new int[] {0, 1, 2, 3, 4, 6}) check(p.await(c).equals("camera " + c), "Remaining cameras follow");
      check((int) p.report().get("requests_outside_predicted_order") >= 1, "Unpredicted requests are counted");
      check(most.get() <= 3, "One unpredicted request adds at most one camera");
    }
    // Out of heap in a worker: drained, then one at a time on the consumer thread, same results.
    AtomicInteger failures = new AtomicInteger();
    Thread consumer = Thread.currentThread();
    List<Integer> onConsumer = Collections.synchronizedList(new ArrayList<>());
    try (MvsgmCameraPool p = new MvsgmCameraPool(3, 3, order, camera -> {
      if (Thread.currentThread() == consumer) onConsumer.add(camera);
      else if (camera == 2 && failures.getAndIncrement() == 0) throw new OutOfMemoryError("test");
      return camera * 10;
    })) {
      for (int c : order) check(p.await(c).equals(c * 10), "Result after the heap fallback");
      check(onConsumer.contains(2) && onConsumer.contains(7), "Failed and later cameras run on the consumer thread");
      check((int) p.report().get("cameras_computed_one_at_a_time") == onConsumer.size(), "Fallback is reported");
    }
    // Other failures reach the consumer with their camera, unchanged.
    try (MvsgmCameraPool p = new MvsgmCameraPool(2, 2, order, camera -> {
      if (camera == 1) throw new IOException("camera 1 failed");
      return camera;
    })) {
      check(p.await(0).equals(0), "Camera before the failure");
      boolean io = false;
      try {
        p.await(1);
      } catch (IOException e) {
        io = e.getMessage().equals("camera 1 failed");
      }
      check(io, "IOException is passed on unchanged");
    }
  }

  /** Five rendered cameras: every map file with three cameras in flight equals the serial files. */
  private static void maps(Path root) throws Exception {
    Options o = TestSupport.options();
    o.compute = "cpu"; o.cores = 4; o.mvsgmLevels = 1; o.mvsgmNeighbors = 4; o.mvsgmBestK = 2;
    int W = MultiViewSgmRegression.W, H = MultiViewSgmRegression.H;
    List<ImageFrame> frames = new ArrayList<>();
    List<FeatureTracks.Track> tracks = new ArrayList<>();
    float[] far = MultiViewSgmRegression.texture(1), near = MultiViewSgmRegression.texture(2);
    try {
      for (int i = 0; i < 5; i++) {
        Camera c = MultiViewSgmRegression.camera(2.5 * i - 5, 0, false);
        Mat m = new Mat(H, W, CvType.CV_8U);
        Path path = root.resolve("camera" + i + ".png");
        try { m.put(0, 0, MultiViewSgmRegression.render(c, far, near, null)); Imgcodecs.imwrite(path.toString(), m); }
        finally { m.release(); }
        ImageFrame f = new ImageFrame(i, path, new Calibration(c.fx, c.fy, c.cx, c.cy, new double[5], W, H), o, true);
        f.camera.rotation = c.rotation; f.camera.translation = c.translation;
        f.camera.registered = true;
        frames.add(f);
      }
      // Sparse points on the two rendered depths, seen by every camera that has them in its image.
      for (int i = 0; i < 400; i++) {
        double x = -14 + (i % 20) * 1.4, y = -7 + (i / 20) * .7, z = MultiViewSgmRegression.inNear(x, y) ? 44 : 50;
        var t = new FeatureTracks.Track();
        t.point = new double[] {x * z / 44, y * z / 44, z};
        for (ImageFrame f : frames) {
          double[] v = f.camera.view(t.point);
          double u = f.camera.fx * v[0] / v[2] + f.camera.cx, w = f.camera.fy * v[1] / v[2] + f.camera.cy;
          if (v[2] > 0 && u >= 4 && w >= 4 && u < W - 4 && w < H - 4) t.observations.add(new FeatureTracks.Observation(f.id, i));
        }
        if (t.observations.size() >= 2) tracks.add(t);
      }
      Path[] out = new Path[2];
      int[] inFlight = {1, 3};
      List<List<Object>> reports = new ArrayList<>();
      for (int mode = 0; mode < 2; mode++) {
        Options run = o.copy();
        run.output = Files.createDirectories(root.resolve(mode == 0 ? "serial" : "in-flight"));
        run.mvsgmCamerasInFlight = inFlight[mode];
        MultiViewSgm mv = new MultiViewSgm(run);
        check(mv.camerasInFlight(frames, frames.size()) == inFlight[mode], "Explicit value is used when the heap allows");
        reports.add(mv.depthMaps(frames, tracks, null));
        check((mv.parallelReport != null) == (mode == 1), "Only the concurrent path reports the pool");
        out[mode] = run.output.resolve("05_stereo/mvsgm");
      }
      int maps = 0;
      for (int c = 0; c < frames.size(); c++)
        for (String kind : new String[] {".depth", ".margin", ".sharp", ".photo"}) {
          Path a = out[0].resolve("camera_" + c + kind), b = out[1].resolve("camera_" + c + kind);
          check(Files.exists(a) == Files.exists(b), "Same map files exist for camera " + c);
          if (!Files.exists(a)) continue;
          check(Files.mismatch(a, b) == -1, "camera_" + c + kind + " equals the serial file");
          maps++;
        }
      check(maps >= 12, "At least three cameras produced maps (" + maps / 4 + ")");
      check(reports.get(0).size() == reports.get(1).size(), "One report entry per camera in both paths");
      for (int i = 0; i < reports.get(0).size(); i++)
        check(((Map<?, ?>) reports.get(0).get(i)).get("image").equals(((Map<?, ?>) reports.get(1).get(i)).get("image")),
            "Reports stay in camera order");
      Options auto = o.copy();
      auto.mvsgmCamerasInFlight = 0;
      check(new MultiViewSgm(auto).camerasInFlight(frames, frames.size()) == 1, "auto stays serial with --compute cpu");
      auto.compute = "auto"; auto.cores = 16;
      check(new MultiViewSgm(auto).camerasInFlight(frames, frames.size()) == 4, "auto: one camera per four cores");
      auto.cores = 64;
      check(new MultiViewSgm(auto).camerasInFlight(frames, frames.size()) == 5, "Never more than the cameras");
      check(new MultiViewSgm(auto).camerasInFlight(frames, 1) == 1, "A single camera is serial");
    } finally { for (ImageFrame f : frames) f.close(); GpuSgm.closeSession(); }
  }
}
