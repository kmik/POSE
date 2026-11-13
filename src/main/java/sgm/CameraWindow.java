package sgm;

import java.util.*;

/** Conservative geometric lifetimes; exact sampled overlap still decides every fusion vote. */
final class CameraWindow {
  final int[][] possible;
  final int[] first, last, rawLast;
  final boolean[] eligible;
  final int peakCameras;
  final long sourcePeakBytes;

  CameraWindow(Camera[] cameras, int[] widths, int[] heights, MultiViewSgm.Plan[] plans, boolean retain) {
    int n = cameras.length;
    eligible = new boolean[n];
    possible = new int[n][];
    first = new int[n];
    rawLast = new int[n];
    for (int c = 0; c < n; c++) {
      eligible[c] = plans[c] != null;
      first[c] = rawLast[c] = c;
    }
    for (int c = 0; c < n; c++) {
      List<Integer> near = new ArrayList<>();
      if (eligible[c]) {
        Interval[] box = sampleBox(cameras[c], widths[c], heights[c], plans[c].labels);
        for (int j = 0; j < n; j++)
          if (j != c && eligible[j] && maySee(box, cameras[j], widths[j], heights[j])) {
            near.add(j);
            first[j] = Math.min(first[j], c);
          }
        for (var neighbour : plans[c].neighbours)
          if (eligible[neighbour.id()]) rawLast[neighbour.id()] = Math.max(rawLast[neighbour.id()], c);
      }
      possible[c] = near.stream().mapToInt(Integer::intValue).toArray();
    }
    last = ImageFusion.lastMapUse(possible);
    long[] sourceEvents = new long[n + 1];
    int[] mapEvents = new int[n + 1];
    int[] rawBirth = new int[n];
    Arrays.fill(rawBirth, n);
    for (int c = 0; c < n; c++) {
      if (eligible[c]) {
        rawNeeded(c, c, plans, rawBirth);
        for (int j : possible[c]) rawNeeded(c, j, plans, rawBirth);
      }
    }
    for (int j = 0; j < n; j++) {
      if (!eligible[j]) continue;
      mapEvents[retain ? 0 : first[j]]++;
      mapEvents[retain ? n : last[j] + 1]--;
      long bytes = 24L * widths[j] * heights[j] + 6L * DepthMapFile.HEADER;
      sourceEvents[rawBirth[j]] += bytes;
      sourceEvents[retain ? n : Math.max(last[j], rawLast[j]) + 1] -= bytes;
    }
    long source = 0, peakSource = 0;
    int maps = 0, peak = 0;
    for (int c = 0; c < n; c++) {
      source += sourceEvents[c];
      maps += mapEvents[c];
      peakSource = Math.max(peakSource, source);
      peak = Math.max(peak, maps);
    }
    sourcePeakBytes = peakSource;
    peakCameras = peak;
  }

  private static void rawNeeded(int stage, int c, MultiViewSgm.Plan[] plans, int[] birth) {
    birth[c] = Math.min(birth[c], stage);
    for (var neighbour : plans[c].neighbours)
      if (plans[neighbour.id()] != null) birth[neighbour.id()] = Math.min(birth[neighbour.id()], stage);
  }

  /** Maximum weighted merged-map working set in unchanged pass-2 order. */
  long peakBytes(long[] bytes, boolean retain) {
    long[] events = new long[bytes.length + 1];
    for (int j = 0; j < bytes.length; j++) if (eligible[j]) {
      events[retain ? 0 : first[j]] += bytes[j];
      events[retain ? bytes.length : last[j] + 1] -= bytes[j];
    }
    long live = 0, peak = 0;
    for (long event : events) {
      live += event;
      peak = Math.max(peak, live);
    }
    return peak;
  }

  /**
   * Outward-rounded intervals enclose the raw back-projection, own-camera splat depth, merged
   * sample back-projection and the other camera's projection, in the Java expression order.
   * Bounds include an extra label, 10% depth slack and four pixels for subpixel/re-merge offsets.
   * Indeterminate arithmetic keeps the edge. No depth interval is imposed on the other camera:
   * its free-space votes can matter even when the two measured depth intervals are disjoint.
   */
  static boolean maySee(Camera c, int w, int h, MultiViewSgm.Labels labels, Camera other, int ow, int oh) {
    return maySee(sampleBox(c, w, h, labels), other, ow, oh);
  }

  private static Interval[] sampleBox(Camera c, int w, int h, MultiViewSgm.Labels labels) {
    double lo = labels.low() - labels.step(), hi = labels.q(labels.count() + 1);
    if (!(lo > 0 && hi > lo) || !regular(c)) return null;
    double near = .9 / hi, far = 1.1 / lo;
    if (!Double.isFinite(far) || far > Float.MAX_VALUE / 4) return null;
    Interval u = new Interval(-4, w + 4), v = new Interval(-4, h + 4);
    Interval[] rawWorld = world(c, u, v, new Interval(near, far));
    Interval depth = view(c, rawWorld)[2];
    if (!depth.finite()) return null;
    double minimum = Math.max(Float.MIN_VALUE, .9 * Math.nextDown((float) depth.lo));
    double maximum = 1.1 * Math.nextUp((float) depth.hi);
    if (!(maximum > 0) || !Double.isFinite(maximum)) return null;
    return world(c, u, v, new Interval(Math.min(minimum, maximum), maximum));
  }

  private static boolean maySee(Interval[] box, Camera other, int ow, int oh) {
    if (box == null || !regular(other)) return true;
    Interval[] v = view(other, box);
    if (!v[0].finite() || !v[1].finite() || !v[2].finite()) return true;
    if (v[2].hi <= 0) return false;
    if (v[2].lo <= 0) return true;
    Interval u = v[0].mul(Interval.at(other.fx)).div(v[2]).add(Interval.at(other.cx));
    Interval t = v[1].mul(Interval.at(other.fy)).div(v[2]).add(Interval.at(other.cy));
    if (!u.finite() || !t.finite()) return true;
    return !(u.hi < 0 || u.lo >= ow || t.hi < 0 || t.lo >= oh);
  }

  private static Interval[] world(Camera c, Interval u, Interval v, Interval z) {
    Interval[] local = {
      u.sub(Interval.at(c.cx)).div(Interval.at(c.fx)).mul(z),
      v.sub(Interval.at(c.cy)).div(Interval.at(c.fy)).mul(z), z
    };
    for (int k = 0; k < 3; k++) local[k] = local[k].sub(Interval.at(c.translation[k]));
    return multiply(Math3.transpose(c.rotation), local);
  }

  private static Interval[] view(Camera c, Interval[] world) {
    Interval[] v = multiply(c.rotation, world);
    for (int k = 0; k < 3; k++) v[k] = v[k].add(Interval.at(c.translation[k]));
    return v;
  }

  private static Interval[] multiply(double[] matrix, Interval[] vector) {
    Interval[] result = new Interval[3];
    for (int r = 0; r < 3; r++)
      result[r] = Interval.at(matrix[3 * r]).mul(vector[0])
          .add(Interval.at(matrix[3 * r + 1]).mul(vector[1]))
          .add(Interval.at(matrix[3 * r + 2]).mul(vector[2]));
    return result;
  }

  private record Interval(double lo, double hi) {
    static Interval at(double value) { return new Interval(value, value); }
    boolean finite() { return Double.isFinite(lo) && Double.isFinite(hi); }
    Interval add(Interval b) { return new Interval(Math.nextDown(lo + b.lo), Math.nextUp(hi + b.hi)); }
    Interval sub(Interval b) { return new Interval(Math.nextDown(lo - b.hi), Math.nextUp(hi - b.lo)); }
    Interval mul(Interval b) { return bounds(lo * b.lo, lo * b.hi, hi * b.lo, hi * b.hi); }
    Interval div(Interval b) {
      if (b.lo <= 0 && b.hi >= 0) return new Interval(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
      return bounds(lo / b.lo, lo / b.hi, hi / b.lo, hi / b.hi);
    }
    private static Interval bounds(double a, double b, double c, double d) {
      return new Interval(Math.nextDown(Math.min(Math.min(a, b), Math.min(c, d))),
          Math.nextUp(Math.max(Math.max(a, b), Math.max(c, d))));
    }
  }

  private static boolean regular(Camera c) {
    if (!(c.fx > 0 && c.fy > 0) || !Double.isFinite(c.fx + c.fy + c.cx + c.cy)) return false;
    for (double t : c.translation) if (!Double.isFinite(t)) return false;
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) {
      double dot = 0;
      for (int k = 0; k < 3; k++) dot += c.rotation[3 * i + k] * c.rotation[3 * j + k];
      if (!Double.isFinite(dot) || Math.abs(dot - (i == j ? 1 : 0)) > 1e-10) return false;
    }
    return true;
  }
}
