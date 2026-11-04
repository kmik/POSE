package sgm;

import static java.lang.foreign.ValueLayout.*;

import java.lang.foreign.*;
import java.nio.file.*;
import java.util.*;

/**
 * Research only: whole-image packed, separated absolute-disparity bands, with CPU recurrence
 * oracle. No production switch: omitted labels cannot be certified from a few photometric probes.
 * Costs are prepared on the host here; GPU cost generation/production confidence remain future
 * work.
 */
final class PackedSgmInvestigation {
  record Volume(int w, int h, int[] offsets, int[] labels, int[] costs, boolean[] textured) {}

  private static final int[][] PATHS = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, 1}, {1, -1}, {-1, -1}
  };
  private static int checks;

  private static void check(boolean condition, String message) {
    checks++;
    if (!condition) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Path output = Path.of(args.length == 0 ? "target/next-revision/packed-sgm.json" : args[0]);
    Files.createDirectories(output.toAbsolutePath().getParent());
    List<Object> measurements = new ArrayList<>();
    int total = run(measurements);
    Json.write(
        output,
        Json.object(
            "scope",
            "Research GPU aggregation only; host costs. Restricted labels are approximate. Not"
                + " production.",
            "checks",
            total,
            "measurements",
            measurements));
    System.out.println("PASS packed bands: " + total + "; " + output);
    GpuSgm.closeSession();
  }

  static int run(List<Object> measurements) throws Exception {
    checks = 0;
    for (int fixture = 0; fixture < 3; fixture++) {
      int w = 97, h = 43, n = w * h, min = -12, count = 49;
      byte[] a = new byte[n], b = new byte[n];
      new Random(180 + fixture).nextBytes(a);
      int[] truth = new int[n];
      for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
          if (fixture == 2) a[y * w + x] = (byte) (x % 9 * 21 + y % 5 * 7);
          truth[y * w + x] = fixture > 0 && x % 29 < 4 ? 22 : 7;
        }
      // Synthesis intentionally includes occluded/unsupported columns and thin alternate layers.
      Arrays.fill(b, (byte) 0);
      for (int y = 0; y < h; y++)
        for (int x = w - 1; x >= 0; x--) {
          int q = x - truth[y * w + x];
          if (q >= 0 && q < w) b[y * w + q] = a[y * w + x];
        }
      Options o = TestSupport.options();
      o.minDisparity = min;
      o.disparities = count;
      o.census = 5;
      long[][] ca = CensusSgm.census(a, w, h, 5), cb = CensusSgm.census(b, w, h, 5);
      Volume full = volume(a, b, ca, cb, w, h, min, count, null, null, 3);
      float[] prior = new float[n], second = new float[n];
      for (int i = 0; i < n; i++) {
        // Simulated coarse support: unknown remains broad. Some fine branches are absent on
        // purpose.
        prior[i] = (i % 13 == 0) ? Float.NaN : 7;
        second[i] = fixture > 0 && i % 11 < 5 ? 22 : Float.NaN;
      }
      Volume packed = volume(a, b, ca, cb, w, h, min, count, prior, second, 3);
      check(
          packed.labels.length < full.labels.length * .65,
          "packed bands save labels while unknown regions stay broad");
      for (int i = 0; i < n; i++)
        if (!Float.isFinite(prior[i]))
          check(
              packed.offsets[i + 1] - packed.offsets[i] == count,
              "missing prior is unknown, not exclusion");
      int[] oracle = oracle(packed, 4, 24, 8);
      int[] fullOracle = oracle(full, 4, 24, 8);
      float[] reference = winners(full, fullOracle), approximate = winners(packed, oracle);
      int lost = 0, changed = 0;
      for (int i = 0; i < n; i++) {
        if (Float.isFinite(reference[i]) && !Float.isFinite(approximate[i])) lost++;
        if (Float.isFinite(reference[i])
            && Float.isFinite(approximate[i])
            && reference[i] != approximate[i]) changed++;
      }
      for (int device = 0; device < GpuSgm.devices().size(); device++) {
        o.compute = "gpu";
        o.gpuDevice = device;
        long start = System.nanoTime();
        int[] actual = gpu(packed, o, 8);
        double seconds = (System.nanoTime() - start) / 1e9;
        check(
            Arrays.equals(oracle, actual),
            "packed GPU absolute-label recurrence matches independent all-to-all CPU oracle");
        check(
            Arrays.equals(fullOracle, gpu(full, o, 8)),
            "full-label packed representation matches dense recurrence");
        measurements.add(
            Json.object(
                "fixture",
                fixture,
                "device",
                GpuSgm.devices().get(device).name(),
                "dense_labels",
                full.labels.length,
                "packed_labels",
                packed.labels.length,
                "gpu_aggregation_wall_seconds",
                seconds,
                "lost_integer_winners",
                lost,
                "changed_integer_winners",
                changed,
                "certification",
                "Only full-label rerun guarantees unrestricted equivalence; not claimed here."));
      }
    }
    return checks;
  }

  /** Two supported layers, merged on overlap; unknown first prior retains the complete interval. */
  private static Volume volume(
      byte[] a,
      byte[] b,
      long[][] ca,
      long[][] cb,
      int w,
      int h,
      int min,
      int count,
      float[] prior,
      float[] second,
      int radius) {
    int n = w * h;
    int[] offsets = new int[n + 1];
    List<Integer> labels = new ArrayList<>(), costs = new ArrayList<>();
    boolean[] textured = new boolean[n];
    Options o = TestSupport.options();
    o.census = 5;
    for (int i = 0; i < n; i++) {
      int x = i % w, y = i / w;
      boolean valid = x >= 2 && x < w - 2 && y >= 2 && y < h - 2;
      textured[i] = valid;
      for (int d = min; d < min + count; d++) {
        if (prior != null
            && Float.isFinite(prior[i])
            && Math.abs(d - prior[i]) > radius
            && !(second != null && Float.isFinite(second[i]) && Math.abs(d - second[i]) <= radius))
          continue;
        labels.add(d);
        int q = x - d;
        costs.add(
            valid && q >= 2 && q < w - 2
                ? StereoCost.value(o, ca, cb, a, b, i, y * w + q, w)
                : 512);
      }
      offsets[i + 1] = labels.size();
    }
    return new Volume(
        w,
        h,
        offsets,
        labels.stream().mapToInt(i -> i).toArray(),
        costs.stream().mapToInt(i -> i).toArray(),
        textured);
  }

  /** Independent brute-force transition minimization, including the global jump over band gaps. */
  private static int[] oracle(Volume v, int p1, int p2, int directions) {
    int[] sum = new int[v.labels.length];
    for (int dir = 0; dir < directions; dir++) {
      int dx = PATHS[dir][0], dy = PATHS[dir][1];
      for (int first = 0; first < v.w * v.h; first++) {
        int x = first % v.w, y = first / v.w, px = x - dx, py = y - dy;
        if (px >= 0 && px < v.w && py >= 0 && py < v.h) continue;
        int[] previous = null;
        int predecessor = -1;
        while (x >= 0 && x < v.w && y >= 0 && y < v.h) {
          int i = y * v.w + x, base = v.offsets[i], size = v.offsets[i + 1] - base;
          int[] current = new int[size];
          boolean has = predecessor >= 0 && v.textured[predecessor];
          int minimum = has ? Arrays.stream(previous).min().orElse(0) : 0;
          for (int k = 0; k < size; k++) {
            int transition = 0;
            if (has) {
              transition = Integer.MAX_VALUE;
              for (int j = 0; j < previous.length; j++) {
                int gap = Math.abs(v.labels[base + k] - v.labels[v.offsets[predecessor] + j]);
                transition =
                    Math.min(transition, previous[j] + (gap == 0 ? 0 : gap == 1 ? p1 : p2));
              }
              transition -= minimum;
            }
            current[k] = v.costs[base + k] + transition;
            sum[base + k] += current[k];
          }
          previous = current;
          predecessor = i;
          x += dx;
          y += dy;
        }
      }
    }
    return sum;
  }

  private static float[] winners(Volume v, int[] sum) {
    float[] result = new float[v.w * v.h];
    Arrays.fill(result, Float.NaN);
    for (int i = 0; i < result.length; i++) {
      int best = -1, value = Integer.MAX_VALUE;
      for (int k = v.offsets[i]; k < v.offsets[i + 1]; k++)
        if (v.costs[k] != 512 && sum[k] < value) {
          best = k;
          value = sum[k];
        }
      if (best >= 0) result[i] = v.labels[best];
    }
    return result;
  }

  private static int[] gpu(Volume v, Options o, int directions) {
    if (v.offsets.length - 1 != v.w * v.h)
      throw new IllegalArgumentException("Invalid packed offsets");
    for (int i = 0; i < v.w * v.h; i++)
      if (v.offsets[i + 1] - v.offsets[i] > 256)
        throw new IllegalArgumentException("Research limit: 256 labels/pixel");
    long bytes = 4L * (v.offsets.length + 3L * v.labels.length) + v.textured.length;
    if (bytes > GpuSgm.budget(GpuSgm.selectedDevice(o), o))
      throw new IllegalArgumentException("Packed research buffers exceed GPU budget");
    try (var cl = new GpuSgm.Compute(o, SOURCE);
        Arena arena = Arena.ofConfined()) {
      var offsets = cl.buffer(v.offsets.length * 4L);
      var labels = cl.buffer(v.labels.length * 4L);
      var costs = cl.buffer(v.costs.length * 4L);
      var sum = cl.buffer(v.costs.length * 4L);
      var textured = cl.buffer(v.textured.length);
      cl.transfer(offsets, 0, arena.allocateFrom(JAVA_INT, v.offsets), false);
      cl.transfer(labels, 0, arena.allocateFrom(JAVA_INT, v.labels), false);
      cl.transfer(costs, 0, arena.allocateFrom(JAVA_INT, v.costs), false);
      cl.transfer(sum, 0, arena.allocateFrom(JAVA_INT, new int[v.costs.length]), false);
      byte[] mask = new byte[v.textured.length];
      for (int i = 0; i < mask.length; i++) mask[i] = (byte) (v.textured[i] ? 1 : 0);
      cl.transfer(textured, 0, arena.allocateFrom(JAVA_BYTE, mask), false);
      var path = cl.kernel("path");
      for (int dir = 0; dir < directions; dir++)
        cl.run(
            path,
            v.w * v.h,
            offsets,
            labels,
            costs,
            sum,
            textured,
            v.w,
            v.h,
            PATHS[dir][0],
            PATHS[dir][1],
            4,
            24);
      var out = arena.allocate(v.costs.length * 4L);
      cl.transfer(sum, 0, out, true);
      return out.toArray(JAVA_INT);
    }
  }

  private static final String SOURCE =
      """
int find(__private int*labels,int n,int d){int lo=0,hi=n;while(lo<hi){int m=(lo+hi)/2;if(labels[m]<d)lo=m+1;else hi=m;}return lo<n&&labels[lo]==d?lo:-1;}
__kernel void path(__global const int*offsets,__global const int*labels,__global const int*costs,
  __global int*sum,__global const uchar*textured,int W,int H,int dx,int dy,int p1,int p2){
  int id=get_global_id(0);if(id>=W*H)return;int x=id%W,y=id/W,px=x-dx,py=y-dy;
  if(px>=0&&px<W&&py>=0&&py<H)return;
  int prevD[256],prevV[256],current[256],size=0,has=0;
  while(x>=0&&x<W&&y>=0&&y<H){int i=y*W+x,base=offsets[i],n=offsets[i+1]-base,minimum=2147483647;
    if(has)for(int k=0;k<size;k++)minimum=min(minimum,prevV[k]);
    for(int k=0;k<n;k++){int d=labels[base+k],transition=0;
      if(has){transition=minimum+p2;int j=find(prevD,size,d);if(j>=0)transition=min(transition,prevV[j]);
        j=find(prevD,size,d-1);if(j>=0)transition=min(transition,prevV[j]+p1);
        j=find(prevD,size,d+1);if(j>=0)transition=min(transition,prevV[j]+p1);transition-=minimum;}
      current[k]=costs[base+k]+transition;sum[base+k]+=current[k];}
    for(int k=0;k<n;k++){prevD[k]=labels[base+k];prevV[k]=current[k];}
    size=n;has=textured[i];x+=dx;y+=dy;
  }
}
""";
}
