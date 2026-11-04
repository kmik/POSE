package sgm;

import java.util.*;

/**
 * GPU stereo filters (S2) against the CPU code, bit for bit: left/right check, median (classic and
 * edge-preserving, 3 and 5), reciprocal/ZNCC check (gray and colour), left and right confidence.
 * The inputs include NaN holes, -0.0 and +0.0 disparities, half-pixel ties for rounding, flat
 * (untextured) areas where ZNCC returns -1, and masks. Skipped without a usable OpenCL device.
 */
final class GpuStereoFiltersRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) {
    nu.pattern.OpenCV.loadLocally();
    System.out.println("PASS GPU stereo filters: " + run());
  }

  static int run() {
    checks = 0;
    int w = 720, h = 400, n = w * h;
    Random random = new Random(11);
    byte[] left = new byte[n], right = new byte[n], leftBgr = new byte[3 * n], rightBgr = new byte[3 * n];
    float[] dl = new float[n], dr = new float[n];
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) {
        int p = y * w + x;
        boolean flat = x > 300 && x < 360 && y > 100 && y < 160;
        int v = flat ? 120 : (int) (128 + 60 * Math.sin(x * .37 + y * .11) + random.nextInt(40) - 20);
        left[p] = (byte) Math.max(0, Math.min(255, v));
        for (int c = 0; c < 3; c++) leftBgr[3 * p + c] = (byte) Math.max(0, Math.min(255, v + 15 * c - 10));
      }
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) {
        int p = y * w + x;
        float d = 12 + 6 * (float) Math.sin(y * .02) + (x > 400 ? 9 : 0);
        int sx = Math.max(0, Math.min(w - 1, Math.round(x + d)));
        right[p] = left[y * w + sx];
        for (int c = 0; c < 3; c++) rightBgr[3 * p + c] = leftBgr[3 * (y * w + sx) + c];
        dl[p] = d + (float) (random.nextGaussian() * .3);
        dr[p] = d + (float) (random.nextGaussian() * .3);
        int k = random.nextInt(100);
        if (k < 6) dl[p] = Float.NaN;
        else if (k < 9) dr[p] = Float.NaN;
        else if (k < 10) dl[p] = random.nextBoolean() ? -0.0f : 0.0f;
        else if (k < 12) dl[p] = (float) Math.floor(dl[p]) + .5f; // x - d at a half-pixel tie
        else if (k < 13) dl[p] = d + 20; // gross outlier
      }
    Options base = TestSupport.options();
    base.compute = "auto";
    if (!GpuStereoFilters.applies(base, w, h)) {
      System.out.println("GpuStereoFiltersRegression skipped: GPU filters not applicable");
      return checks;
    }
    int compared = 0;
    for (String mode : List.of("edge-preserving", "classic"))
      for (int median : new int[] {3, 5})
        for (int channels : new int[] {1, 3}) {
          Options o = base.copy();
          o.medianMode = mode;
          o.median = median;
          byte[] pl = channels == 3 ? leftBgr : left, pr = channels == 3 ? rightBgr : right;
          int radius = o.census / 2;
          CensusSgm.Filtered cpu = filterCpu(o, dl, dr, left, pl, pr, channels, w, h, radius, null, null, "");
          CensusSgm.Filtered gpu = GpuStereoFilters.filter(o, dl, dr, left, pl, pr, channels, w, h, radius);
          if (gpu == null) {
            System.out.println("GpuStereoFiltersRegression skipped: no OpenCL FP64 device");
            return checks;
          }
          String label = mode + " median " + median + " channels " + channels;
          check(same(cpu.checked(), gpu.checked()), "checked disparities, " + label + ": " + diff(cpu.checked(), gpu.checked()));
          check(same(cpu.correlation(), gpu.correlation()), "ZNCC, " + label + ": " + diff(cpu.correlation(), gpu.correlation()));
          check(same(cpu.lrError(), gpu.lrError()), "LR error, " + label);
          check(cpu.lrRejected() == gpu.lrRejected() && cpu.postRejected() == gpu.postRejected() && cpu.photoRejected() == gpu.photoRejected(),
              "counts, " + label + ": " + cpu + " vs " + gpu);
          check(cpu.photoRejected() > 0 && cpu.lrRejected() > 0, "the test exercises rejections");
          // Confidences on the filtered result.
          CensusSgm.Result a = result(cpu, w, h, random), b = result(cpu, w, h, new Random(0));
          b.costMargin = a.costMargin.clone();
          b.costSharpness = a.costSharpness.clone();
          PointConfidence.stereo(a, left, right);
          check(GpuStereoFilters.stereoConfidence(b, left, right, o), "GPU left confidence applied");
          check(same(a.confidence, b.confidence), "left confidence, " + label + ": " + diff(a.confidence, b.confidence));
          PointConfidence.Selection selected = new PointConfidence.Selection(n);
          for (int p = 0; p < n; p++)
            if (random.nextInt(10) > 0) {
              selected.margin[p] = random.nextFloat();
              selected.sharpness[p] = random.nextFloat() * 1.1f;
            }
          byte[] maskL = new byte[n], maskR = new byte[n];
          for (int p = 0; p < n; p++) {
            maskL[p] = (byte) (random.nextInt(500) == 0 ? 0 : 1);
            maskR[p] = (byte) (random.nextInt(500) == 0 ? 0 : 1);
          }
          for (boolean masked : new boolean[] {true, false}) {
            PointConfidence.reverseCpu(a, left, right, pl, pr, channels, dr, selected, masked ? maskL : null, masked ? maskR : null, o);
            check(GpuStereoFilters.reverseConfidence(b, left, right, pl, pr, channels, dr, selected, masked ? maskL : null, masked ? maskR : null, o),
                "GPU right confidence applied");
            check(same(a.rightConfidence, b.rightConfidence), "right confidence, " + label + (masked ? " masked" : "") + ": " + diff(a.rightConfidence, b.rightConfidence));
            int finite = 0;
            for (float v : a.rightConfidence) if (Float.isFinite(v)) finite++;
            check(finite > n / 20, "right confidence computed for many pixels: " + finite);
          }
          compared++;
        }
    check(compared == 8, "all variants compared");
    return checks;
  }

  static CensusSgm.Filtered filterCpu(
      Options o, float[] dl, float[] dr, byte[] left, byte[] pl, byte[] pr, int channels, int w, int h, int radius,
      byte[] reasons, CensusSgm.Observer observer, String size) {
    try {
      return CensusSgm.filter(o, dl, dr, left, pl, pr, channels, w, h, radius, reasons, observer, size);
    } catch (java.io.IOException e) {
      throw new java.io.UncheckedIOException(e);
    }
  }

  static CensusSgm.Result result(CensusSgm.Filtered f, int w, int h, Random random) {
    CensusSgm.Result r =
        new CensusSgm.Result(w, h, f.checked().clone(), new CensusSgm.Stats(), f.correlation().clone(), f.lrError().clone());
    int n = w * h;
    r.costMargin = new float[n];
    r.costSharpness = new float[n];
    for (int p = 0; p < n; p++) {
      r.costMargin[p] = random.nextInt(20) == 0 ? Float.NaN : random.nextFloat() * .3f;
      r.costSharpness[p] = random.nextFloat() * .3f;
    }
    return r;
  }

  static boolean same(float[] a, float[] b) {
    if (a.length != b.length) return false;
    for (int i = 0; i < a.length; i++) if (Float.floatToRawIntBits(a[i]) != Float.floatToRawIntBits(b[i])) return false;
    return true;
  }

  static String diff(float[] a, float[] b) {
    int count = 0, first = -1;
    for (int i = 0; i < a.length; i++)
      if (Float.floatToRawIntBits(a[i]) != Float.floatToRawIntBits(b[i])) {
        if (first < 0) first = i;
        count++;
      }
    return count == 0 ? "equal" : count + " differ, first at " + first + ": " + a[first] + " (" + Integer.toHexString(Float.floatToRawIntBits(a[first])) + ") vs " + b[first] + " (" + Integer.toHexString(Float.floatToRawIntBits(b[first])) + ")";
  }
}
