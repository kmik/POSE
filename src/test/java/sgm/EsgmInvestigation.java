package sgm;

import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;

/**
 * Standalone opt-in research harness; deliberately excluded from SelfTest and production routing.
 */
public final class EsgmInvestigation {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  record Fixture(
      String name,
      int w,
      int h,
      int min,
      int count,
      byte[] a,
      byte[] b,
      float[] truth,
      boolean[] visible,
      byte[] maskA,
      byte[] maskB) {
    Fixture(
        String name,
        int w,
        int h,
        int min,
        int count,
        byte[] a,
        byte[] b,
        float[] truth,
        boolean[] visible) {
      this(name, w, h, min, count, a, b, truth, visible, null, null);
    }
  }

  record Oracle(
      short[] candidates, int[] aggregate, int[] costs, int[] summary, int[] outsideBounds) {}

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Core.setNumThreads(1);
    Path output = Path.of(System.getProperty("sgm.research.output", "target/esgm-investigation"));
    Files.createDirectories(output);
    Options o = TestSupport.options();
    o.compute = "gpu";
    o.gpuDevice = Integer.getInteger("sgm.test.gpu", 0);
    o.gpuMemoryMb = Long.getLong("sgm.research.vram", 6000);
    o.memoryMb = 4096;
    o.output = output;
    o.gpuSgm = "resident";
    o.gpuStripRows = 7;
    o.median = 1;
    o.speckleSize = 0;
    o.uniqueness = .05;
    List<Object> results = new ArrayList<>();
    try {
      GpuSgm.Device device = GpuSgm.selectedDevice(o);
      if (args.length == 4 && args[0].equals("pair")) {
        Path folder = Path.of(args[1]);
        Mat a = Imgcodecs.imread(folder.resolve("leftGray.png").toString(), 0),
            b = Imgcodecs.imread(folder.resolve("rightGray.png").toString(), 0),
            ma = Imgcodecs.imread(folder.resolve("leftMask.png").toString(), 0),
            mb = Imgcodecs.imread(folder.resolve("rightMask.png").toString(), 0);
        try {
          check(
              !a.empty() && !b.empty() && !ma.empty() && !mb.empty(),
              "Saved pair and masks decoded");
          check(
              a.size().equals(b.size()) && a.size().equals(ma.size()) && a.size().equals(mb.size()),
              "Saved pair dimensions agree");
          byte[] aa = new byte[(int) a.total()],
              bb = new byte[aa.length],
              am = new byte[aa.length],
              bm = new byte[aa.length];
          a.get(0, 0, aa);
          b.get(0, 0, bb);
          ma.get(0, 0, am);
          mb.get(0, 0, bm);
          o.gpuStripRows = 64;
          int min = Integer.parseInt(args[2]), count = Integer.parseInt(args[3]);
          results.add(
              evaluate(
                  new Fixture(
                      "saved-forest-pair",
                      a.cols(),
                      a.rows(),
                      min,
                      count,
                      aa,
                      bb,
                      null,
                      null,
                      am,
                      bm),
                  o,
                  false));
          for (int y : new int[] {0, a.rows() / 2 - 64, a.rows() - 128}) {
            int from = y * a.cols(), to = (y + 128) * a.cols();
            results.add(
                evaluate(
                    new Fixture(
                        "forest-strip-y" + y,
                        a.cols(),
                        128,
                        min,
                        count,
                        Arrays.copyOfRange(aa, from, to),
                        Arrays.copyOfRange(bb, from, to),
                        null,
                        null,
                        Arrays.copyOfRange(am, from, to),
                        Arrays.copyOfRange(bm, from, to)),
                    o,
                    true));
          }
        } finally {
          a.release();
          b.release();
          ma.release();
          mb.release();
        }
      } else if (args.length > 0 && args[0].equals("large")) {
        Fixture f = scene("large-planar", 1978, 2640, 131, 1037, 350, false, false);
        o.gpuStripRows = 64;
        results.add(evaluate(f, o, false));
      } else {
        for (int directions : new int[] {4, 8})
          for (boolean reverse : new boolean[] {false, true}) oracleCheck(o, directions, reverse);
        for (Fixture f :
            List.of(
                scene("plane", 256, 128, -3, 65, 19, false, false),
                scene("fences-and-layers", 320, 160, 0, 96, 18, true, false),
                scene("repeated-texture", 256, 128, 0, 65, 19, true, true),
                scene("negative-disparity", 256, 128, -48, 65, -19, false, false)))
          results.add(evaluate(f, o, true));
        if (args.length == 2) {
          Mat a = Imgcodecs.imread(args[0], Imgcodecs.IMREAD_GRAYSCALE),
              b = Imgcodecs.imread(args[1], Imgcodecs.IMREAD_GRAYSCALE);
          try {
            check(
                !a.empty() && !b.empty() && a.size().equals(b.size()),
                "Real rectified input sizes");
            byte[] aa = new byte[(int) a.total()], bb = new byte[aa.length];
            a.get(0, 0, aa);
            b.get(0, 0, bb);
            results.add(
                evaluate(
                    new Fixture("real-rectified", a.cols(), a.rows(), 0, 128, aa, bb, null, null),
                    o,
                    true));
          } finally {
            a.release();
            b.release();
          }
        }
      }
      Json.write(
          output.resolve("results.json"),
          Json.object(
              "status",
              "research_not_production",
              "device",
              device.name(),
              "checks",
              checks,
              "results",
              results,
              "limitations",
              List.of(
                  "Census/16-bit only; two sweeps instead of optimized three-pass schedule",
                  "Candidate-only uniqueness is not equivalent to full-range uniqueness",
                  "Timing includes prototype descriptor creation; production reference descriptors"
                      + " are prepared before timing",
                  "Raw selection and independent LR comparison; downstream"
                      + " photometric/speckle/fusion stages are not replayed")));
      System.out.println(
          "PASS research checks=" + checks + "; report=" + output.resolve("results.json"));
    } finally {
      GpuSgm.closeSession();
    }
  }

  static Fixture scene(
      String name, int w, int h, int min, int count, int shift, boolean layers, boolean repeated) {
    int n = w * h;
    byte[] a = new byte[n], b = new byte[n];
    new Random(461).nextBytes(a);
    new Random(916).nextBytes(b);
    float[] truth = new float[n];
    boolean[] visible = new boolean[n];
    int[] owner = new int[n];
    Arrays.fill(owner, -1);
    if (repeated)
      for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) a[y * w + x] = (byte) (((x % 12) * 19 + y % 3 * 7) & 255);
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) {
        int d = shift;
        if (layers) {
          if (x > w / 3 && x < w * 2 / 3 && y > h / 4 && y < h * 3 / 4) d = shift + 23;
          if (x % 61 >= 30 && x % 61 <= 32)
            d = shift + 39; // three-pixel fence, including coarse-scale losses
        }
        int p = y * w + x, q = y * w + x - d;
        truth[p] = d;
        if (x - d >= 0 && x - d < w && (owner[q] < 0 || truth[owner[q]] < d)) owner[q] = p;
      }
    for (int q = 0; q < n; q++)
      if (owner[q] >= 0) {
        int p = owner[q];
        b[q] = a[p];
        visible[p] = true;
      }
    return new Fixture(name, w, h, min, count, a, b, truth, visible);
  }

  private static Object evaluate(Fixture f, Options base, boolean reference) throws Exception {
    Options o = base.copy();
    o.directions = 8;
    o.disparities = f.count;
    o.minDisparity = f.min;
    boolean[] ma = EsgmPrototype.texture(f.a, f.w, f.h, o.census),
        mb = EsgmPrototype.texture(f.b, f.w, f.h, o.census);
    if (f.maskA != null)
      for (int p = 0; p < ma.length; p++) {
        ma[p] &= f.maskA[p] != 0;
        mb[p] &= f.maskB[p] != 0;
      }
    long[][] a = CensusSgm.census(f.a, f.w, f.h, o.census),
        b = CensusSgm.census(f.b, f.w, f.h, o.census);
    float[] cpu = null, cpuRight = null;
    double residentSeconds = Double.NaN, residentRightSeconds = Double.NaN;
    if (reference) {
      long start = System.nanoTime();
      cpu =
          GpuStereo.execute(
              o, a, b, f.a, f.b, ma, mb, f.w, f.h, false, new CensusSgm.Stats(), null, null);
      residentSeconds = (System.nanoTime() - start) / 1e9;
      start = System.nanoTime();
      cpuRight =
          GpuStereo.execute(
              o, b, a, f.b, f.a, mb, ma, f.w, f.h, true, new CensusSgm.Stats(), null, null);
      residentRightSeconds = (System.nanoTime() - start) / 1e9;
    }
    var left = EsgmPrototype.run(o, f.a, f.b, ma, mb, f.w, f.h, false, false);
    var right = EsgmPrototype.run(o, f.b, f.a, mb, ma, f.w, f.h, true, false);
    float[] selected = EsgmPrototype.select(left.summary(), o.disparities, o),
        reverse = EsgmPrototype.select(right.summary(), o.disparities, o);
    float[] guarded = EsgmPrototype.guarded(left, o),
        guardedRight = EsgmPrototype.guarded(right, o);
    float[] guardedLr = lr(guarded, guardedRight, f.w, o.lrThreshold),
        referenceLr = cpu == null ? null : lr(cpu, cpuRight, f.w, o.lrThreshold);
    float[] lr = selected.clone();
    int changed = 0, common = 0, extra = 0, lost = 0, lrRejected = 0;
    double sumChange = 0;
    for (int p = 0; p < selected.length; p++) {
      float d = selected[p];
      if (Float.isFinite(d)) {
        int x = p % f.w, xr = Math.round(x - d);
        if (xr < 0
            || xr >= f.w
            || !Float.isFinite(reverse[p / f.w * f.w + xr])
            || Math.abs(d - reverse[p / f.w * f.w + xr]) > o.lrThreshold) {
          lr[p] = Float.NaN;
          lrRejected++;
        }
      }
      if (cpu != null) {
        if (Float.isFinite(d) && Float.isFinite(cpu[p])) {
          common++;
          sumChange += Math.abs(d - cpu[p]);
          if (Math.abs(d - cpu[p]) > .001) changed++;
        } else if (Float.isFinite(d)) extra++;
        else if (Float.isFinite(cpu[p])) lost++;
      }
    }
    Map<String, Object> report =
        Json.object(
            "fixture",
            f.name,
            "width",
            f.w,
            "height",
            f.h,
            "min_disparity",
            f.min,
            "disparities",
            f.count,
            "resident_reference_seconds",
            residentSeconds,
            "resident_reference_right_seconds",
            residentRightSeconds,
            "prototype_left_seconds",
            left.seconds(),
            "prototype_right_seconds",
            right.seconds(),
            "prototype_device_bytes",
            left.deviceBytes(),
            "aggregate_scratch_bytes",
            0,
            "common_selected",
            cpu == null ? null : common,
            "changed_common_over_0_001_px",
            cpu == null ? null : changed,
            "mean_common_change_px",
            common == 0 ? Double.NaN : sumChange / common,
            "candidate_only_extra_acceptances",
            cpu == null ? null : extra,
            "candidate_only_lost_acceptances",
            cpu == null ? null : lost,
            "lr_rejected",
            lrRejected,
            "selected_metrics",
            metrics(selected, f),
            "lr_metrics",
            metrics(lr, f),
            "resident_metrics",
            cpu == null ? null : metrics(cpu, f),
            "resident_lr_metrics",
            cpu == null ? null : metrics(referenceLr, f),
            "guarded_metrics",
            metrics(guarded, f),
            "guarded_lr_metrics",
            metrics(guardedLr, f),
            "left_profile",
            left.profile(),
            "right_profile",
            right.profile());
    if (cpu != null) {
      int kept = 0, extraGuarded = 0, wrongGuarded = 0, extraGuardedLr = 0;
      for (int p = 0; p < guarded.length; p++) {
        if (Float.isFinite(guarded[p])) {
          kept++;
          if (!Float.isFinite(cpu[p])) extraGuarded++;
          else if (Float.floatToIntBits(guarded[p]) != Float.floatToIntBits(cpu[p])) wrongGuarded++;
        }
        if (Float.isFinite(guardedLr[p]) && !Float.isFinite(referenceLr[p])) extraGuardedLr++;
      }
      check(
          extraGuarded == 0 && wrongGuarded == 0 && extraGuardedLr == 0,
          f.name + ": certified accepted subset, exact subpixel and LR decisions");
      report.put("guarded_selected_total", kept);
      report.put("guarded_extra_vs_reference", extraGuarded);
      report.put("guarded_disparities_different_vs_reference", wrongGuarded);
      report.put("guarded_lr_extra_vs_reference", extraGuardedLr);
    }
    if (f.truth != null && !f.name.equals("repeated-texture")) {
      int valid = 0, correct = 0;
      for (int p = 0; p < lr.length; p++)
        if (f.visible[p] && Float.isFinite(lr[p])) {
          valid++;
          if (Math.abs(lr[p] - f.truth[p]) < 1) correct++;
        }
      check(
          valid > f.w * f.h / 8 && correct > valid * .97,
          f.name + ": meaningful coverage and known-shift accuracy");
    }
    System.out.println("RESULT " + Json.string(report));
    return report;
  }

  static Object metrics(float[] values, Fixture f) {
    int valid = 0, visible = 0, bad = 0, fenceEligible = 0;
    List<Double> errors = new ArrayList<>(), fence = new ArrayList<>();
    double square = 0;
    for (int p = 0; p < values.length; p++)
      if (f.truth == null || f.visible[p]) {
        visible++;
        if (f.truth != null && f.truth[p] > f.truth[0] + 30) fenceEligible++;
        if (!Float.isFinite(values[p])) continue;
        valid++;
        if (f.truth != null) {
          double e = Math.abs(values[p] - f.truth[p]);
          errors.add(e);
          square += e * e;
          if (e > 1) bad++;
          if (f.truth[p] > f.truth[0] + 30) fence.add(e);
        }
      }
    Collections.sort(errors);
    Collections.sort(fence);
    return Json.object(
        "valid",
        valid,
        "eligible",
        visible,
        "coverage",
        valid / (double) Math.max(1, visible),
        "bad_over_1_px",
        f.truth == null ? null : bad,
        "rms_error_px",
        errors.isEmpty() ? Double.NaN : Math.sqrt(square / errors.size()),
        "p95_abs_error_px",
        errors.isEmpty() ? Double.NaN : errors.get((errors.size() - 1) * 95 / 100),
        "fence_samples",
        fence.size(),
        "fence_eligible",
        fenceEligible,
        "fence_p95_error_px",
        fence.isEmpty() ? Double.NaN : fence.get((fence.size() - 1) * 95 / 100));
  }

  private static float[] lr(float[] left, float[] right, int width, double threshold) {
    float[] out = left.clone();
    for (int p = 0; p < out.length; p++)
      if (Float.isFinite(out[p])) {
        int x = Math.round(p % width - out[p]);
        if (x < 0
            || x >= width
            || !Float.isFinite(right[p / width * width + x])
            || Math.abs(out[p] - right[p / width * width + x]) > threshold) out[p] = Float.NaN;
      }
    return out;
  }

  private static void oracleCheck(Options base, int directions, boolean reverse) throws Exception {
    Fixture f = scene("oracle", 87, 41, -9, 37, 8, true, false);
    Options o = base.copy();
    o.directions = directions;
    o.disparities = f.count;
    o.minDisparity = f.min;
    o.gpuStripRows = 5;
    boolean[] ma = EsgmPrototype.texture(f.a, f.w, f.h, o.census),
        mb = EsgmPrototype.texture(f.b, f.w, f.h, o.census);
    for (int y = 12; y < 18; y++) for (int x = 29; x < 37; x++) ma[y * f.w + x] = false;
    byte[] aa = reverse ? f.b : f.a, bb = reverse ? f.a : f.b;
    boolean[] ta = reverse ? mb : ma, tb = reverse ? ma : mb;
    Oracle expected = oracle(o, aa, bb, ta, tb, f.w, f.h, reverse);
    var actual = EsgmPrototype.run(o, aa, bb, ta, tb, f.w, f.h, reverse, true);
    check(
        Arrays.equals(actual.candidates(), expected.candidates),
        "GPU/CPU path minima including masks, signed range, ties and strip carry");
    for (int p = 0; p < f.w * f.h; p++)
      for (int j = 0; j < directions * 3; j++) {
        int k = (expected.candidates[p * directions + j / 3] & 65535) + j % 3 - 1;
        int sum = k >= 0 && k < f.count ? expected.aggregate[p * f.count + k] : 0;
        check(
            (actual.sums()[p * directions * 3 + j] & 65535) == sum,
            "GPU/CPU candidate-neighbor aggregate");
      }
    float[] cpu = EsgmPrototype.select(expected.summary, f.count, o);
    float[] resident =
        GpuStereo.execute(
            o,
            CensusSgm.census(aa, f.w, f.h, o.census),
            CensusSgm.census(bb, f.w, f.h, o.census),
            aa,
            bb,
            ta,
            tb,
            f.w,
            f.h,
            reverse,
            new CensusSgm.Stats(),
            null,
            null);
    check(
        Arrays.equals(cpu, resident),
        "Independent full-volume CPU oracle matches resident GPU selection");
    for (int p = 0; p < f.w * f.h; p++)
      check(
          (actual.outsideBounds()[p] & 65535) == expected.outsideBounds[p],
          "Independent omitted-disparity lower bound");
    float[] guarded = EsgmPrototype.guarded(actual, o);
    for (int p = 0; p < guarded.length; p++)
      if (Float.isFinite(guarded[p]))
        check(
            Float.floatToIntBits(guarded[p]) == Float.floatToIntBits(cpu[p]),
            "Certified candidate is accepted identically by full-volume reference");
  }

  private static Oracle oracle(
      Options o, byte[] a, byte[] b, boolean[] ma, boolean[] mb, int w, int h, boolean reverse) {
    int n = w * h, D = o.disparities, C = o.directions, r = o.census / 2;
    int[] cost = new int[n * D], aggregate = new int[cost.length], summary = new int[n * 6];
    Arrays.fill(cost, 512);
    long[][] da = CensusSgm.census(a, w, h, o.census), db = CensusSgm.census(b, w, h, o.census);
    for (int p = 0; p < n; p++)
      for (int k = 0; k < D; k++) {
        int x = p % w, y = p / w, target = x + (reverse ? 1 : -1) * (o.minDisparity + k);
        if (x >= r
            && x < w - r
            && y >= r
            && y < h - r
            && ma[p]
            && target >= r
            && target < w - r
            && mb[y * w + target])
          cost[p * D + k] = StereoCost.value(o, da, db, a, b, p, y * w + target, w);
      }
    short[] candidates = new short[n * C];
    Arrays.fill(candidates, (short) -1);
    int[][] dirs = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, 1}, {1, -1}, {-1, -1}};
    int[][] allPaths = new int[C][];
    for (int direction = 0; direction < C; direction++) {
      int dx = dirs[direction][0], dy = dirs[direction][1];
      int[] path = new int[cost.length];
      allPaths[direction] = path;
      for (int yy = 0; yy < h; yy++)
        for (int xx = 0; xx < w; xx++) {
          int y = dy < 0 ? h - 1 - yy : yy,
              x = dx < 0 ? w - 1 - xx : xx,
              p = y * w + x,
              px = x - dx,
              py = y - dy;
          boolean has = px >= 0 && px < w && py >= 0 && py < h && ma[py * w + px];
          int prev = (py * w + px) * D;
          int minimum = Integer.MAX_VALUE;
          if (has) for (int k = 0; k < D; k++) minimum = Math.min(minimum, path[prev + k]);
          int localP2 =
              has
                  ? StereoCost.pathP2(
                      StereoCost.penalty(o.p1, o),
                      StereoCost.p2(o),
                      o.sgmEdgeScale,
                      a,
                      p,
                      py * w + px)
                  : 0;
          int best = -1, value = Integer.MAX_VALUE;
          for (int k = 0; k < D; k++) {
            int v = cost[p * D + k];
            if (has) {
              int t = Math.min(path[prev + k], minimum + localP2);
              if (k > 0) t = Math.min(t, path[prev + k - 1] + StereoCost.penalty(o.p1, o));
              if (k + 1 < D) t = Math.min(t, path[prev + k + 1] + StereoCost.penalty(o.p1, o));
              v += t - minimum;
            }
            path[p * D + k] = v;
            aggregate[p * D + k] += v;
            if (cost[p * D + k] != 512 && v < value) {
              best = k;
              value = v;
            }
          }
          candidates[p * C + direction] = (short) best;
        }
    }
    for (int p = 0; p < n; p++) {
      int best = -1, value = Integer.MAX_VALUE, second = Integer.MAX_VALUE, before = -1, after = -1;
      for (int k = 0; k < D; k++)
        if (cost[p * D + k] != 512 && aggregate[p * D + k] < value) {
          best = k;
          value = aggregate[p * D + k];
        }
      if (best >= 0) {
        for (int k = 0; k < D; k++)
          if (Math.abs(k - best) > 1 && cost[p * D + k] != 512)
            second = Math.min(second, aggregate[p * D + k]);
        if (best > 0 && cost[p * D + best - 1] != 512) before = aggregate[p * D + best - 1];
        if (best + 1 < D && cost[p * D + best + 1] != 512) after = aggregate[p * D + best + 1];
      }
      int at = p * 6;
      summary[at] = best;
      summary[at + 1] = value;
      summary[at + 2] = second;
      summary[at + 3] = before;
      summary[at + 4] = after;
      summary[at + 5] = (cost[p * D] != 512 ? 1 : 0) | (cost[p * D + D - 1] != 512 ? 2 : 0);
    }
    int[] outsideBounds = new int[n];
    for (int p = 0; p < n; p++)
      for (int direction = 0; direction < C; direction++) {
        int low = Integer.MAX_VALUE;
        for (int k = 0; k < D; k++)
          if (cost[p * D + k] != 512) {
            boolean covered = false;
            for (int j = 0; j < C; j++)
              if (Math.abs(k - (candidates[p * C + j] & 65535)) <= 1) {
                covered = true;
                break;
              }
            if (!covered) low = Math.min(low, allPaths[direction][p * D + k]);
          }
        outsideBounds[p] =
            low == Integer.MAX_VALUE ? 65535 : Math.min(65535, outsideBounds[p] + low);
      }
    return new Oracle(candidates, aggregate, cost, summary, outsideBounds);
  }
}
