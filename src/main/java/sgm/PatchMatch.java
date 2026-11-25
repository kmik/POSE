package sgm;

import static java.lang.foreign.ValueLayout.*;

import java.lang.foreign.*;
import java.util.*;

/**
 * Plane-hypothesis MVS with deterministic Jacobi propagation and inverse-depth refinement. Inspired
 * by published PatchMatch MVS, not a reproduction of a proprietary implementation.
 */
final class PatchMatch {
  static final String MODEL = "patchmatch-validity-bilateral-wide-ambiguity-geometric-v4";

  // Camera blocks: relative R(9), t(3), fx/fy/cx/cy(4), width/height/offset(3), reserved(1).
  record Scene(
      int width,
      int height,
      byte[] pixels,
      byte[] masks,
      float[] cameras,
      int views,
      float minimum,
      float maximum,
      float[] geometry,
      float geometricRelative,
      float edgeSigma) {
    Scene(
        int width,
        int height,
        byte[] pixels,
        byte[] masks,
        float[] cameras,
        int views,
        float minimum,
        float maximum) {
      this(width, height, pixels, masks, cameras, views, minimum, maximum, null, .005f, 32);
    }

    Scene(
        int width,
        int height,
        byte[] pixels,
        byte[] masks,
        float[] cameras,
        int views,
        float minimum,
        float maximum,
        float[] geometry,
        float geometricRelative) {
      this(
          width,
          height,
          pixels,
          masks,
          cameras,
          views,
          minimum,
          maximum,
          geometry,
          geometricRelative,
          32);
    }
  }

  static int sweeps(Scene s, Options o) {
    return sweeps(s.width, s.height, o);
  }

  /** Schedules follow the full level raster, also for a tile of it. */
  static int sweeps(int width, int height, Options o) {
    return Math.max(o.patchmatchIterations, 1 + Integer.numberOfTrailingZeros(reach(width, height)));
  }

  private static int reach(int width, int height) {
    return Integer.highestOneBit(Math.max(1, Math.max(width, height) / 16));
  }

  private static int stride(Scene s, int iteration) {
    return stride(s.width, s.height, iteration);
  }

  static int stride(int width, int height, int iteration) {
    int reach = reach(width, height);
    return Math.max(1, reach >> (iteration % (1 + Integer.numberOfTrailingZeros(reach))));
  }

  record Map(int width, int height, float[] data, Object backend) {
    float depth(int i) {
      return data[i * 8];
    }

    int view(int i) {
      return (int) data[i * 8 + 5];
    }

    float photo(int i) {
      return 1 - data[i * 8 + 4];
    }

    float margin(int i) {
      return data[i * 8 + 6];
    }

    float sharpness(int i) {
      return data[i * 8 + 7];
    }
  }

  /**
   * A window of a reference's level raster (tile plus halo): work items cover [x, x+width) x [y,
   * y+height) of the fullWidth x fullHeight raster. crops holds per view x0, y0, width, height of
   * the raster crop stored in the scene atlas; the scene's camera blocks keep the full-raster
   * intrinsics and sizes (offsets point at the crops).
   */
  record Tile(int x, int y, int width, int height, int fullWidth, int fullHeight, int[] crops) {}

  /** Device bytes of one tiled solve (atlas, geometry, plane generations, result, crops). */
  static long tileDeviceBytes(long atlas, long pixels, boolean geometric, int views) {
    return (geometric ? 6L : 2L) * atlas + 64L * pixels + views * 96L + 8;
  }

  /**
   * One tile of a reference (GPU). Every float expression is the untiled one evaluated at the same
   * full-raster pixel with the same seed and stride schedule; the result can differ from the
   * untiled solve only where propagation reaches across the window edge (the halo absorbs this) or
   * where a sample falls outside its source crop (treated as unavailable). Returns the window's map.
   */
  static Map computeTile(Scene s, Tile t, Options o, Map previous) {
    if (s.views < 2 || s.views > 9 || !(s.minimum > 0 && s.maximum > s.minimum))
      throw new IllegalArgumentException(
          "PatchMatch needs source views and finite positive depth bounds");
    if (o.compute.equals("cpu"))
      throw new IllegalArgumentException(
          "PatchMatch tiles run on the GPU only; with --compute cpu lower the resolution, raise"
              + " --memory-mb or use --patchmatch-tile off");
    int n = Math.multiplyExact(t.width, t.height);
    long deviceBytes = tileDeviceBytes(s.pixels.length, n, s.geometry != null, s.views);
    float[] seed =
        seeds(s, previous, o.correlation, t.x, t.y, t.width, t.height, t.fullWidth, t.fullHeight);
    var device = GpuSgm.selectedDevice(o);
    if (deviceBytes > GpuSgm.budget(device, o))
      throw new IllegalArgumentException(
          "PatchMatch tile exceeds selected GPU budget; use a smaller --patchmatch-tile");
    int sweeps = sweeps(t.fullWidth, t.fullHeight, o);
    try (GpuMemoryPool.Reservation held = GpuMemoryPool.reserveAuxiliary(deviceBytes);
        var cl = new GpuSgm.Compute(o, program(TILED_SOURCE, o));
        Arena a = Arena.ofConfined()) {
      var pixels = cl.buffer(s.pixels.length);
      var masks = cl.buffer(s.masks.length);
      var cameras = cl.buffer(s.cameras.length * 4L);
      var crops = cl.buffer(t.crops.length * 4L);
      var geometry = cl.buffer(s.geometry == null ? 4 : s.geometry.length * 4L);
      var first = cl.buffer(n * 16L);
      var second = cl.buffer(n * 16L);
      var result = cl.buffer(n * 32L);
      cl.transfer(pixels, 0, a.allocateFrom(JAVA_BYTE, s.pixels), false);
      cl.transfer(masks, 0, a.allocateFrom(JAVA_BYTE, s.masks), false);
      cl.transfer(cameras, 0, a.allocateFrom(JAVA_FLOAT, s.cameras), false);
      cl.transfer(crops, 0, a.allocateFrom(JAVA_INT, t.crops), false);
      cl.transfer(
          geometry,
          0,
          a.allocateFrom(JAVA_FLOAT, s.geometry == null ? new float[] {0} : s.geometry),
          false);
      cl.transfer(first, 0, a.allocateFrom(JAVA_FLOAT, seed), false);
      var sweep = cl.kernel("propagate_tile");
      var finish = cl.kernel("finish_tile");
      for (int i = 0; i < sweeps; i++) {
        cl.run(
            sweep,
            n,
            pixels,
            masks,
            cameras,
            crops,
            geometry,
            s.geometry == null ? 0f : s.geometricRelative,
            first,
            second,
            t.fullWidth,
            t.fullHeight,
            t.x,
            t.y,
            t.width,
            t.height,
            s.views,
            s.minimum,
            s.maximum,
            i,
            stride(t.fullWidth, t.fullHeight, i),
            s.edgeSigma);
        var swap = first;
        first = second;
        second = swap;
      }
      cl.run(
          finish,
          n,
          pixels,
          masks,
          cameras,
          crops,
          geometry,
          s.geometry == null ? 0f : s.geometricRelative,
          first,
          result,
          t.fullWidth,
          t.fullHeight,
          t.x,
          t.y,
          t.width,
          t.height,
          s.views,
          s.minimum,
          s.maximum,
          (float) (1 - o.correlation),
          s.edgeSigma);
      var out = a.allocate(n * 32L);
      cl.transfer(result, 0, out, true);
      RunTelemetry.gpuProfile(
          "PatchMatch tile "
              + t.width
              + "x"
              + t.height
              + (s.geometry == null ? " photometric" : " geometric"),
          cl.profile());
      return new Map(
          t.width,
          t.height,
          out.toArray(JAVA_FLOAT),
          Json.object(
              "backend",
              "opencl_patchmatch_tiled",
              "device",
              device.name(),
              "estimated_device_bytes",
              deviceBytes,
              "estimator_model",
              MODEL,
              "effective_sweeps",
              sweeps,
              "geometric_pass",
              s.geometry != null));
    } catch (RuntimeException e) {
      throw new IllegalArgumentException(
          "Selected PatchMatch GPU failed on a tile; no SGM substitution. " + e.getMessage(), e);
    }
  }

  /**
   * The OpenCL program for these options: the original source text unless an opt-in switch
   * changes the cost, so default runs build exactly the program they always built.
   */
  static String program(String base, Options o) {
    if (o.patchmatchTopViews <= 1) return base;
    String s = "#define TOPK " + o.patchmatchTopViews + '\n' + base;
    s = once(s, "float best=INFINITY,second=INFINITY;int contradictions=0;",
        "float best=INFINITY,second=INFINITY;int contradictions=0;"
            + "float top[TOPK];for(int k=0;k<TOPK;k++)top[k]=INFINITY;");
    s = once(s, "if(val<best){second=best;best=val;}else second=fmin(second,val);",
        "if(val<best){second=best;best=val;}else second=fmin(second,val);"
            + "for(int k=0;k<TOPK;k++)if(val<top[k]){float t=top[k];top[k]=val;val=t;}");
    s = once(s, "return best+.1f*contradictions+(isfinite(second)?.25f*fmin(.4f,second-best):0);",
        "{float sum=0;int n=0;for(int k=0;k<TOPK;k++)if(isfinite(top[k])){sum+=top[k];n++;}"
            + "return n==0?INFINITY:sum/n+.1f*contradictions;}");
    return s;
  }

  private static String once(String text, String find, String replacement) {
    int at = text.indexOf(find);
    if (at < 0 || text.indexOf(find, at + 1) >= 0)
      throw new IllegalStateException("PatchMatch program variant: expected one " + find);
    return text.substring(0, at) + replacement + text.substring(at + find.length());
  }

  static Map compute(Scene s, Options o, Map previous) {
    if (s.views < 2 || s.views > 9 || !(s.minimum > 0 && s.maximum > s.minimum))
      throw new IllegalArgumentException(
          "PatchMatch needs source views and finite positive depth bounds");
    int n = Math.multiplyExact(s.width, s.height);
    if (s.width < 1
        || s.height < 1
        || s.pixels.length != s.masks.length
        || s.cameras.length != s.views * 20
        || s.geometry != null && s.geometry.length != s.pixels.length)
      throw new IllegalArgumentException("Invalid PatchMatch raster/camera/geometry atlas");
    long deviceBytes =
        (s.geometry == null ? 2L : 6L) * s.pixels.length + 64L * n + s.cameras.length * 4L + 8;
    long hostBytes = deviceBytes + 64L * n + (previous == null ? 0 : previous.data.length * 4L);
    if (hostBytes
        > Math.min(o.memoryMb * 1048576L, (long) (Runtime.getRuntime().maxMemory() * .65)))
      throw new IllegalArgumentException(
          "PatchMatch working set exceeds --memory-mb/heap; lower resolution or source-view count");
    float[] seed = seeds(s, previous, o.correlation);
    if (o.compute.equals("cpu")) return cpu(s, o, seed);
    var device = GpuSgm.selectedDevice(o);
    if (deviceBytes > GpuSgm.budget(device, o))
      throw new IllegalArgumentException(
          "PatchMatch working set exceeds selected GPU budget; lower resolution/views");
    // Parallel references share the device through the installed pool (no-op without one).
    try (GpuMemoryPool.Reservation held = GpuMemoryPool.reserveAuxiliary(deviceBytes);
        var cl = new GpuSgm.Compute(o, program(SOURCE, o));
        Arena a = Arena.ofConfined()) {
      var pixels = cl.buffer(s.pixels.length);
      var masks = cl.buffer(s.masks.length);
      var cameras = cl.buffer(s.cameras.length * 4L);
      var geometry = cl.buffer(s.geometry == null ? 4 : s.geometry.length * 4L);
      var first = cl.buffer(n * 16L);
      var second = cl.buffer(n * 16L);
      var result = cl.buffer(n * 32L);
      cl.transfer(pixels, 0, a.allocateFrom(JAVA_BYTE, s.pixels), false);
      cl.transfer(masks, 0, a.allocateFrom(JAVA_BYTE, s.masks), false);
      cl.transfer(cameras, 0, a.allocateFrom(JAVA_FLOAT, s.cameras), false);
      cl.transfer(
          geometry,
          0,
          a.allocateFrom(JAVA_FLOAT, s.geometry == null ? new float[] {0} : s.geometry),
          false);
      cl.transfer(first, 0, a.allocateFrom(JAVA_FLOAT, seed), false);
      var sweep = cl.kernel("propagate");
      var finish = cl.kernel("finish");
      for (int i = 0; i < sweeps(s, o); i++) {
        cl.run(
            sweep,
            n,
            pixels,
            masks,
            cameras,
            geometry,
            s.geometry == null ? 0f : s.geometricRelative,
            first,
            second,
            s.width,
            s.height,
            s.views,
            s.minimum,
            s.maximum,
            i,
            stride(s, i),
            s.edgeSigma);
        var swap = first;
        first = second;
        second = swap;
      }
      cl.run(
          finish,
          n,
          pixels,
          masks,
          cameras,
          geometry,
          s.geometry == null ? 0f : s.geometricRelative,
          first,
          result,
          s.width,
          s.height,
          s.views,
          s.minimum,
          s.maximum,
          (float) (1 - o.correlation),
          s.edgeSigma);
      var out = a.allocate(n * 32L);
      cl.transfer(result, 0, out, true);
      RunTelemetry.gpuProfile(
          "PatchMatch "
              + s.width
              + "x"
              + s.height
              + (s.geometry == null ? " photometric" : " geometric"),
          cl.profile());
      return new Map(
          s.width,
          s.height,
          out.toArray(JAVA_FLOAT),
          Json.object(
              "backend",
              "opencl_patchmatch",
              "device",
              device.name(),
              "estimated_device_bytes",
              deviceBytes,
              "estimated_host_bytes",
              hostBytes,
              "estimator_model",
              MODEL,
              "effective_sweeps",
              sweeps(s, o),
              "geometric_pass",
              s.geometry != null,
              "determinism",
              "fixed seed, double-buffered propagation; floating-point differences across vendors"
                  + " possible",
              "opencl_profile",
              cl.profile()));
    } catch (RuntimeException e) {
      throw new IllegalArgumentException(
          "Selected PatchMatch GPU failed; no SGM substitution. "
              + "Use --compute cpu for the reference implementation. "
              + e.getMessage(),
          e);
    }
  }

  private static float[] seeds(Scene s, Map previous, double correlation) {
    return seeds(s, previous, correlation, 0, 0, s.width, s.height, s.width, s.height);
  }

  /**
   * Seeds of the window [ox, ox+tw) x [oy, oy+th) of a W x H level raster: hashes and plane
   * transfers use full-raster pixels, so a tile gets the seeds of the same pixels untiled.
   */
  private static float[] seeds(
      Scene s, Map previous, double correlation, int ox, int oy, int tw, int th, int W, int H) {
    float[] out = new float[tw * th * 4];
    for (int ly = 0; ly < th; ly++)
      for (int lx = 0; lx < tw; lx++) {
        int x = ox + lx, y = oy + ly, i = y * W + x, o = ly * tw + lx;
        float nx = 0, ny = 0, nz = 1, z;
        int seed = hash(i + 17);
        z = 1 / (1 / s.maximum + unit(seed) * (1 / s.minimum - 1 / s.maximum));
        if (previous != null) {
          int px = Math.min(previous.width - 1, x * previous.width / W),
              py = Math.min(previous.height - 1, y * previous.height / H),
              p = (py * previous.width + px) * 8;
          if (previous.data[p] > 0
              && Float.isFinite(previous.data[p])
              && previous.data[p + 5] >= 1
              && Float.isFinite(previous.data[p + 4])
              && previous.data[p + 4] <= 1 - correlation
              && Float.isFinite(
                  previous.data[p + 1] + previous.data[p + 2] + previous.data[p + 3])) {
            nx = previous.data[p + 1];
            ny = previous.data[p + 2];
            nz = previous.data[p + 3];
            // Keep the coarse plane, not a nearest-neighbor constant depth stair step.
            float u = (px - previousCx(s, previous, W)) / previousFx(s, previous, W);
            float v = (py - previousCy(s, previous, H)) / previousFy(s, previous, H);
            float plane = previous.data[p] * (nx * u + ny * v + nz);
            float denominator =
                nx * (x - s.cameras[14]) / s.cameras[12]
                    + ny * (y - s.cameras[15]) / s.cameras[13]
                    + nz;
            z = plane / denominator;
            if (!(denominator > .05f && z >= s.minimum && z <= s.maximum)) {
              nx = ny = 0;
              nz = 1;
              z = 1 / (1 / s.maximum + unit(seed) * (1 / s.minimum - 1 / s.maximum));
            }
          }
        }
        out[o * 4] = nx;
        out[o * 4 + 1] = ny;
        out[o * 4 + 2] = nz;
        out[o * 4 + 3] =
            z
                * (nx * (x - s.cameras[14]) / s.cameras[12]
                    + ny * (y - s.cameras[15]) / s.cameras[13]
                    + nz);
      }
    return out;
  }

  private static float previousFx(Scene s, Map p, int W) {
    return s.cameras[12] * p.width / W;
  }

  private static float previousFy(Scene s, Map p, int H) {
    return s.cameras[13] * p.height / H;
  }

  private static float previousCx(Scene s, Map p, int W) {
    return (s.cameras[14] + .5f) * p.width / W - .5f;
  }

  private static float previousCy(Scene s, Map p, int H) {
    return (s.cameras[15] + .5f) * p.height / H - .5f;
  }

  static int hash(int a) {
    a ^= a >>> 16;
    a *= 0x7feb352d;
    a ^= a >>> 15;
    a *= 0x846ca68b;
    return a ^ (a >>> 16);
  }

  static float unit(int a) {
    return (a & 0xffffff) / 16777216f;
  }

  /** Scalar oracle follows the same hypotheses; photometry is evaluated in double precision. */
  private static Map cpu(Scene s, Options o, float[] seed) {
    int n = s.width * s.height;
    float[] a = seed, b = new float[n * 4];
    for (int iter = 0; iter < sweeps(s, o); iter++) {
      // Jacobi sweep: every pixel reads only generation a and writes only its own slot of b, so
      // rows run in parallel (the caller's StereoRows scope) with identical results.
      final float[] in = a, next = b;
      final int sweep = iter;
      StereoRows.rows(
          0, s.height, row -> sweepRow(s, in, next, sweep, row, o.patchmatchTopViews));
      float[] swap = a;
      a = b;
      b = swap;
    }
    float[] out = new float[n * 8];
    final float[] last = a;
    StereoRows.rows(0, s.height, row -> finishRow(s, o, last, out, row));
    return new Map(
        s.width,
        s.height,
        out,
        Json.object(
            "backend",
            "cpu_patchmatch_reference",
            "estimator_model",
            MODEL,
            "effective_sweeps",
            sweeps(s, o),
            "geometric_pass",
            s.geometry != null));
  }

  private static void sweepRow(Scene s, float[] a, float[] b, int iter, int row, int topViews) {
      for (int i = row * s.width; i < (row + 1) * s.width; i++) {
        int x = i % s.width, y = i / s.width;
        float[] best = Arrays.copyOfRange(a, i * 4, i * 4 + 4);
        double cost = cost(s, x, y, best, -1, topViews);
        int stride = stride(s, iter);
        for (int at :
            new int[] {
              x >= stride ? i - stride : -1,
              x + stride < s.width ? i + stride : -1,
              y >= stride ? i - stride * s.width : -1,
              y + stride < s.height ? i + stride * s.width : -1
            }) {
          if (at < 0) continue;
          float[] candidate = Arrays.copyOfRange(a, at * 4, at * 4 + 4);
          double c = cost(s, x, y, candidate, -1, topViews);
          if (c < cost) {
            cost = c;
            best = candidate;
          }
        }
        int state = hash(i + iter * 92821 + 71);
        for (int j = 0; j < 8; j++) {
          float z = depth(s, x, y, best),
              radius = (float) ((1 / s.minimum - 1 / s.maximum) * Math.pow(.5, iter + 1));
          state = hash(state);
          float inv =
              Math.clamp(1 / z + (unit(state) - .5f) * 2 * radius, 1 / s.maximum, 1 / s.minimum);
          state = hash(state);
          float nx = best[0] + (unit(state) - .5f) * (float) Math.pow(.5, iter);
          state = hash(state);
          float ny = best[1] + (unit(state) - .5f) * (float) Math.pow(.5, iter);
          float nz = Math.max(.3f, best[2]), norm = (float) Math.sqrt(nx * nx + ny * ny + nz * nz);
          nx /= norm;
          ny /= norm;
          nz /= norm;
          float[] candidate = {
            nx,
            ny,
            nz,
            (nx * (x - s.cameras[14]) / s.cameras[12]
                    + ny * (y - s.cameras[15]) / s.cameras[13]
                    + nz)
                / inv
          };
          double c = cost(s, x, y, candidate, -1, topViews);
          if (c < cost) {
            cost = c;
            best = candidate;
          }
        }
        System.arraycopy(best, 0, b, i * 4, 4);
      }
  }

  private static void finishRow(Scene s, Options o, float[] a, float[] out, int row) {
    for (int i = row * s.width; i < (row + 1) * s.width; i++) {
      int x = i % s.width, y = i / s.width;
      float[] p = Arrays.copyOfRange(a, i * 4, i * 4 + 4);
      double best = Double.POSITIVE_INFINITY;
      int view = -1;
      double[] costs = new double[s.views];
      for (int v = 1; v < s.views; v++) {
        costs[v] = cost(s, x, y, p, v);
        if (costs[v] < best) {
          best = costs[v];
          view = v;
        }
      }
      boolean valid =
          Double.isFinite(cost(s, x, y, p, -1, o.patchmatchTopViews)) && best <= 1 - o.correlation;
      double margin = 0, sharpness = 0, rank = -1;
      if (valid)
        for (int v = 1; v < s.views; v++) {
          if (costs[v] > 1 - o.correlation) continue;
          double[] signal = evidence(s, x, y, p, v, costs[v]);
          double score = PointConfidence.score(1 - costs[v], signal[0], signal[1], 0);
          if (score > rank || score == rank && costs[v] < best) {
            rank = score;
            view = v;
            best = costs[v];
            margin = signal[0];
            sharpness = signal[1];
          }
        }
      out[i * 8] = valid ? depth(s, x, y, p) : Float.NaN;
      System.arraycopy(p, 0, out, i * 8 + 1, 3);
      out[i * 8 + 4] = (float) best;
      out[i * 8 + 5] = valid ? view : Double.isFinite(best) && best <= 1 - o.correlation ? -2 : -1;
      out[i * 8 + 6] = (float) margin;
      out[i * 8 + 7] = (float) sharpness;
    }
  }

  private static double[] evidence(Scene s, int x, int y, float[] p, int view, double best) {
    float delta = depthStep(s, x, y, p, view);
    double minus = cost(s, x, y, shift(p, -delta), view),
        plus = cost(s, x, y, shift(p, delta), view);
    int at = view * 20;
    double baseline =
        Math.sqrt(
            s.cameras[at + 9] * s.cameras[at + 9]
                + s.cameras[at + 10] * s.cameras[at + 10]
                + s.cameras[at + 11] * s.cameras[at + 11]);
    float center = depth(s, x, y, p);
    double alt = Double.POSITIVE_INFINITY;
    int competitors = 0;
    for (int offset : new int[] {-16, -8, -4, -2, 2, 4, 8, 16}) {
      double inverse = 1 / center + offset / Math.max(.001, s.cameras[12] * baseline);
      if (!(inverse > 0)) continue;
      float[] candidate = p.clone();
      candidate[3] /= (float) (inverse * center);
      double c = cost(s, x, y, candidate, view);
      if (Double.isFinite(c)) {
        alt = Math.min(alt, c);
        competitors++;
      }
    }
    for (int k = 0; k < 4; k++) {
      float z = 1 / (1 / s.maximum + k / 3f * (1 / s.minimum - 1 / s.maximum));
      if (Math.abs(1 / z - 1 / center) * s.cameras[12] * baseline < 1) continue;
      float[] candidate = p.clone();
      candidate[3] *= z / center;
      double c = cost(s, x, y, candidate, view);
      if (Double.isFinite(c)) {
        alt = Math.min(alt, c);
        competitors++;
      }
    }
    double margin =
        competitors >= 2 ? (float) PointConfidence.clamp((alt - best) / Math.max(.1, alt)) : 0;
    double sharpness =
        Double.isFinite(minus + plus)
            ? (float) PointConfidence.clamp((minus + plus - 2 * best) / Math.max(.1, minus + plus))
            : 0;
    return new double[] {margin, sharpness};
  }

  private static float[] shift(float[] p, float delta) {
    float[] q = p.clone();
    q[3] += delta;
    return q;
  }

  private static float depth(Scene s, int x, int y, float[] p) {
    return p[3]
        / (p[0] * (x - s.cameras[14]) / s.cameras[12]
            + p[1] * (y - s.cameras[15]) / s.cameras[13]
            + p[2]);
  }

  private static float depthStep(Scene s, int x, int y, float[] p, int view) {
    int c = view * 20;
    float baseline =
        (float)
            Math.sqrt(
                s.cameras[c + 9] * s.cameras[c + 9]
                    + s.cameras[c + 10] * s.cameras[c + 10]
                    + s.cameras[c + 11] * s.cameras[c + 11]);
    float z = depth(s, x, y, p), den = p[3] / z;
    return Math.max(z * .001f, z * z / Math.max(.001f, s.cameras[12] * baseline)) * den;
  }

  static double cost(Scene s, int x, int y, float[] p, int selected) {
    return cost(s, x, y, p, selected, 1);
  }

  /** topViews > 1 (--patchmatch-view-cost topK): the combined cost is the mean of the K best views. */
  static double cost(Scene s, int x, int y, float[] p, int selected, int topViews) {
    double[] top = topViews > 1 ? new double[topViews] : null;
    if (top != null) Arrays.fill(top, Double.POSITIVE_INFINITY);
    float z = depth(s, x, y, p);
    if (!Float.isFinite(z)
        || z < s.minimum
        || z > s.maximum
        || x < 2
        || y < 2
        || x >= s.width - 2
        || y >= s.height - 2
        || !(p[3] / z > .05f)) return Double.POSITIVE_INFINITY;
    double best = Double.POSITIVE_INFINITY, second = Double.POSITIVE_INFINITY;
    int contradictions = 0;
    for (int view = 1; view < s.views; view++) {
      if (selected > 0 && selected != view) continue;
      int c = view * 20,
          w = (int) s.cameras[c + 16],
          h = (int) s.cameras[c + 17],
          off = (int) s.cameras[c + 18] * 65536 + (int) s.cameras[c + 19];
      int evidence = geometric(s, x, y, z, c, w, h, off);
      if (evidence == 2) continue;
      if (evidence == 1) contradictions++;
      if (!warpValid(s, x, y, p, c)) continue;
      double sa = 0, sb = 0, saa = 0, sbb = 0, sab = 0, sw = 0, sw2 = 0;
      int center = s.pixels[y * s.width + x] & 255;
      boolean valid = true;
      for (int dy = -2; dy <= 2 && valid; dy++)
        for (int dx = -2; dx <= 2; dx++) {
          double rx = (x + dx - s.cameras[14]) / s.cameras[12],
              ry = (y + dy - s.cameras[15]) / s.cameras[13];
          double d = p[3] / (p[0] * rx + p[1] * ry + p[2]);
          double qx =
              (s.cameras[c] * rx + s.cameras[c + 1] * ry + s.cameras[c + 2]) * d + s.cameras[c + 9];
          double qy =
              (s.cameras[c + 3] * rx + s.cameras[c + 4] * ry + s.cameras[c + 5]) * d
                  + s.cameras[c + 10];
          double qz =
              (s.cameras[c + 6] * rx + s.cameras[c + 7] * ry + s.cameras[c + 8]) * d
                  + s.cameras[c + 11];
          if (d <= 0 || qz <= 0 || !Double.isFinite(d + qx + qy + qz)) {
            valid = false;
            break;
          }
          double u = s.cameras[c + 12] * qx / qz + s.cameras[c + 14],
              v = s.cameras[c + 13] * qy / qz + s.cameras[c + 15];
          int xx = (int) Math.floor(u), yy = (int) Math.floor(v), at = (y + dy) * s.width + x + dx;
          if (xx < 0 || yy < 0 || xx + 1 >= w || yy + 1 >= h || s.masks[at] == 0) {
            valid = false;
            break;
          }
          int b = off + yy * w + xx;
          if (s.masks[b] == 0
              || s.masks[b + 1] == 0
              || s.masks[b + w] == 0
              || s.masks[b + w + 1] == 0) {
            valid = false;
            break;
          }
          double fx = u - xx, fy = v - yy, av = s.pixels[at] & 255;
          double bv =
              (1 - fy) * ((1 - fx) * (s.pixels[b] & 255) + fx * (s.pixels[b + 1] & 255))
                  + fy * ((1 - fx) * (s.pixels[b + w] & 255) + fx * (s.pixels[b + w + 1] & 255));
          double difference = av - center;
          double weight =
              s.edgeSigma <= 0
                  ? 1
                  : 1
                      / ((1 + difference * difference / (s.edgeSigma * s.edgeSigma))
                          * (1 + .25 * (dx * dx + dy * dy)));
          sw += weight;
          sw2 += weight * weight;
          sa += weight * av;
          sb += weight * bv;
          saa += weight * av * av;
          sbb += weight * bv * bv;
          sab += weight * av * bv;
        }
      double va = saa - sa * sa / sw, vb = sbb - sb * sb / sw;
      if (!valid || sw * sw < 5 * sw2 || va < 4 * sw || vb < 4 * sw) continue;
      double value = 1 - Math.clamp((sab - sa * sb / sw) / Math.sqrt(va * vb), -1, 1);
      if (value < best) {
        second = best;
        best = value;
      } else second = Math.min(second, value);
      if (top != null)
        for (int k = 0; k < top.length; k++)
          if (value < top[k]) {
            double swap = top[k];
            top[k] = value;
            value = swap;
          }
    }
    if (selected > 0) return best;
    if (contradictions >= 2) return Double.POSITIVE_INFINITY;
    if (top != null) {
      double sum = 0;
      int n = 0;
      for (double t : top)
        if (Double.isFinite(t)) {
          sum += t;
          n++;
        }
      return n == 0 ? Double.POSITIVE_INFINITY : sum / n + .1 * contradictions;
    }
    return best
        + .1 * contradictions
        + (Double.isFinite(second) ? .25 * Math.min(.4, second - best) : 0);
  }

  private static boolean warpValid(Scene s, int x, int y, float[] p, int c) {
    double rx = (x - s.cameras[14]) / s.cameras[12], ry = (y - s.cameras[15]) / s.cameras[13];
    double a = p[0] * rx + p[1] * ry + p[2];
    double qx =
        s.cameras[c] * rx + s.cameras[c + 1] * ry + s.cameras[c + 2] + s.cameras[c + 9] * a / p[3];
    double qy =
        s.cameras[c + 3] * rx
            + s.cameras[c + 4] * ry
            + s.cameras[c + 5]
            + s.cameras[c + 10] * a / p[3];
    double qz =
        s.cameras[c + 6] * rx
            + s.cameras[c + 7] * ry
            + s.cameras[c + 8]
            + s.cameras[c + 11] * a / p[3];
    if (!(qz > 0)) return false;
    double xx = (s.cameras[c] + s.cameras[c + 9] * p[0] / p[3]) / s.cameras[12];
    double xy = (s.cameras[c + 3] + s.cameras[c + 10] * p[0] / p[3]) / s.cameras[12];
    double xz = (s.cameras[c + 6] + s.cameras[c + 11] * p[0] / p[3]) / s.cameras[12];
    double yx = (s.cameras[c + 1] + s.cameras[c + 9] * p[1] / p[3]) / s.cameras[13];
    double yy = (s.cameras[c + 4] + s.cameras[c + 10] * p[1] / p[3]) / s.cameras[13];
    double yz = (s.cameras[c + 7] + s.cameras[c + 11] * p[1] / p[3]) / s.cameras[13];
    double ux = s.cameras[c + 12] * (xx * qz - qx * xz) / (qz * qz),
        uy = s.cameras[c + 12] * (yx * qz - qx * yz) / (qz * qz);
    double vx = s.cameras[c + 13] * (xy * qz - qy * xz) / (qz * qz),
        vy = s.cameras[c + 13] * (yy * qz - qy * yz) / (qz * qz);
    double det = ux * vy - uy * vx,
        ratio = s.cameras[c + 12] * s.cameras[c + 13] / (s.cameras[12] * s.cameras[13]);
    return det > .15 * ratio && det < 6 * ratio && ux * ux + uy * uy + vx * vx + vy * vy < 12 * det;
  }

  private static int geometric(Scene s, int x, int y, float z, int c, int w, int h, int off) {
    if (s.geometry == null) return -1;
    double rx = (x - s.cameras[14]) / s.cameras[12], ry = (y - s.cameras[15]) / s.cameras[13];
    double qx =
        (s.cameras[c] * rx + s.cameras[c + 1] * ry + s.cameras[c + 2]) * z + s.cameras[c + 9];
    double qy =
        (s.cameras[c + 3] * rx + s.cameras[c + 4] * ry + s.cameras[c + 5]) * z + s.cameras[c + 10];
    double qz =
        (s.cameras[c + 6] * rx + s.cameras[c + 7] * ry + s.cameras[c + 8]) * z + s.cameras[c + 11];
    if (!(qz > 0)) return -1;
    int xx = (int) Math.round(s.cameras[c + 12] * qx / qz + s.cameras[c + 14]);
    int yy = (int) Math.round(s.cameras[c + 13] * qy / qz + s.cameras[c + 15]);
    if (xx < 1 || yy < 1 || xx >= w - 1 || yy >= h - 1) return -1;
    float target = s.geometry[off + yy * w + xx];
    if (!(target > 0) || !Float.isFinite(target)) return -1;
    double tolerance = Math.max(.01, 3 * s.geometricRelative);
    int coherent = 0;
    for (int dy = -1; dy <= 1; dy++)
      for (int dx = -1; dx <= 1; dx++) {
        float other = s.geometry[off + (yy + dy) * w + xx + dx];
        if (other > 0 && Float.isFinite(other) && Math.abs(other - target) <= tolerance * target)
          coherent++;
      }
    if (coherent < 7) return -1;
    return qz < target * (1 - tolerance) ? 1 : qz > target * (1 + tolerance) ? 2 : 0;
  }

  static final String SOURCE =
      """
#pragma OPENCL FP_CONTRACT OFF
uint hash(uint a){a^=a>>16;a*=0x7feb352d;a^=a>>15;a*=0x846ca68b;return a^(a>>16);}
float unit(uint a){return (a&0xffffff)/16777216.f;}
float dep(int x,int y,float4 p,__global const float*c){return p.w/(p.x*(x-c[14])/c[12]+p.y*(y-c[15])/c[13]+p.z);}
int geometric(__global const float*c,__global const float*geom,float geoTol,int x,int y,float z,int v){
 if(geoTol<=0)return -1;__global const float*q=c+20*v;
 float rx=(x-c[14])/c[12],ry=(y-c[15])/c[13];
 float3 t=(float3)((q[0]*rx+q[1]*ry+q[2])*z+q[9],(q[3]*rx+q[4]*ry+q[5])*z+q[10],(q[6]*rx+q[7]*ry+q[8])*z+q[11]);
 if(!(t.z>0))return -1;int xx=(int)floor(q[12]*t.x/t.z+q[14]+.5f),yy=(int)floor(q[13]*t.y/t.z+q[15]+.5f);
 int w=(int)q[16],h=(int)q[17],off=(int)q[18]*65536+(int)q[19];if(xx<1||yy<1||xx>=w-1||yy>=h-1)return -1;
 float target=geom[off+yy*w+xx];if(!(target>0)||!isfinite(target))return -1;
 float tol=fmax(.01f,3*geoTol);int coherent=0;
 for(int dy=-1;dy<=1;dy++)for(int dx=-1;dx<=1;dx++){float d=geom[off+(yy+dy)*w+xx+dx];if(d>0&&isfinite(d)&&fabs(d-target)<=tol*target)coherent++;}
 if(coherent<7)return -1;return t.z<target*(1-tol)?1:t.z>target*(1+tol)?2:0;
}
int warpValid(__global const float*c,int x,int y,float4 p,int v){
 __global const float*q=c+20*v;float rx=(x-c[14])/c[12],ry=(y-c[15])/c[13],a=p.x*rx+p.y*ry+p.z;
 float3 t=(float3)(q[0]*rx+q[1]*ry+q[2]+q[9]*a/p.w,q[3]*rx+q[4]*ry+q[5]+q[10]*a/p.w,q[6]*rx+q[7]*ry+q[8]+q[11]*a/p.w);
 if(!(t.z>0))return 0;
 float3 dx=(float3)(q[0]+q[9]*p.x/p.w,q[3]+q[10]*p.x/p.w,q[6]+q[11]*p.x/p.w)/c[12];
 float3 dy=(float3)(q[1]+q[9]*p.y/p.w,q[4]+q[10]*p.y/p.w,q[7]+q[11]*p.y/p.w)/c[13];
 float ux=q[12]*(dx.x*t.z-t.x*dx.z)/(t.z*t.z),uy=q[12]*(dy.x*t.z-t.x*dy.z)/(t.z*t.z);
 float vx=q[13]*(dx.y*t.z-t.y*dx.z)/(t.z*t.z),vy=q[13]*(dy.y*t.z-t.y*dy.z)/(t.z*t.z);
 float det=ux*vy-uy*vx,ratio=q[12]*q[13]/(c[12]*c[13]);return det>.15f*ratio&&det<6*ratio&&ux*ux+uy*uy+vx*vx+vy*vy<12*det;
}
float cost(__global const uchar *im,__global const uchar *mask,__global const float*c,
 __global const float*geom,float geoTol,int W,int H,int V,
 int x,int y,float4 p,float zmin,float zmax,int selected,float edgeSigma){
 float z=dep(x,y,p,c);if(!isfinite(z)||z<zmin||z>zmax||x<2||y<2||x>=W-2||y>=H-2||!(p.w/z>.05f))return INFINITY;
 float best=INFINITY,second=INFINITY;int contradictions=0;
 for(int v=1;v<V;v++){if(selected>0&&v!=selected)continue;__global const float *q=c+20*v;
  int w=(int)q[16],h=(int)q[17],off=(int)q[18]*65536+(int)q[19];float sa=0,sb=0,saa=0,sbb=0,sab=0,sw=0,sw2=0,center=im[y*W+x];int valid=1;
  int evidence=geometric(c,geom,geoTol,x,y,z,v);if(evidence==2)continue;if(evidence==1)contradictions++;
  if(!warpValid(c,x,y,p,v))continue;
  for(int dy=-2;dy<=2&&valid;dy++)for(int dx=-2;dx<=2;dx++){
   float rx=(x+dx-c[14])/c[12],ry=(y+dy-c[15])/c[13],d=p.w/(p.x*rx+p.y*ry+p.z);
   float3 t=(float3)((q[0]*rx+q[1]*ry+q[2])*d+q[9],(q[3]*rx+q[4]*ry+q[5])*d+q[10],(q[6]*rx+q[7]*ry+q[8])*d+q[11]);
   if(d<=0||t.z<=0||!isfinite(d+t.x+t.y+t.z)){valid=0;break;}float u=q[12]*t.x/t.z+q[14],vv=q[13]*t.y/t.z+q[15];
   int xx=(int)floor(u),yy=(int)floor(vv),at=(y+dy)*W+x+dx;
   if(xx<0||yy<0||xx+1>=w||yy+1>=h||!mask[at]){valid=0;break;}int b=off+yy*w+xx;
   if(!mask[b]||!mask[b+1]||!mask[b+w]||!mask[b+w+1]){valid=0;break;}
   float fx=u-xx,fy=vv-yy,av=im[at],bv=(1-fy)*((1-fx)*im[b]+fx*im[b+1])+fy*((1-fx)*im[b+w]+fx*im[b+w+1]);
   float difference=av-center,weight=edgeSigma<=0?1:1/((1+difference*difference/(edgeSigma*edgeSigma))*(1+.25f*(dx*dx+dy*dy)));
   sw+=weight;sw2+=weight*weight;sa+=weight*av;sb+=weight*bv;saa+=weight*av*av;sbb+=weight*bv*bv;sab+=weight*av*bv;
  }
  float va=saa-sa*sa/sw,vb=sbb-sb*sb/sw;if(!valid||sw*sw<5*sw2||va<4*sw||vb<4*sw)continue;
  float val=1-clamp((sab-sa*sb/sw)/sqrt(va*vb),-1.f,1.f);
  if(val<best){second=best;best=val;}else second=fmin(second,val);
 }
 if(selected>0)return best;if(contradictions>=2)return INFINITY;
 return best+.1f*contradictions+(isfinite(second)?.25f*fmin(.4f,second-best):0);
}
__kernel void propagate(__global const uchar*im,__global const uchar*mask,__global const float*c,
 __global const float*geom,float geoTol,__global const float4*in,__global float4*out,int W,int H,int V,float zmin,float zmax,int iter,int stride,float edgeSigma){
 int i=get_global_id(0);if(i>=W*H)return;int x=i%W,y=i/W;float4 p=in[i];
 float best=cost(im,mask,c,geom,geoTol,W,H,V,x,y,p,zmin,zmax,-1,edgeSigma);
 int at[4]={x>=stride?i-stride:-1,x+stride<W?i+stride:-1,y>=stride?i-stride*W:-1,y+stride<H?i+stride*W:-1};
 for(int k=0;k<4;k++){if(at[k]<0)continue;float4 q=in[at[k]];float val=cost(im,mask,c,geom,geoTol,W,H,V,x,y,q,zmin,zmax,-1,edgeSigma);if(val<best){p=q;best=val;}}
 uint state=hash((uint)(i+iter*92821+71));
 for(int k=0;k<8;k++){
  float z=dep(x,y,p,c),radius=(1/zmin-1/zmax)*exp2((float)(-iter-1));state=hash(state);
  float inv=clamp(1/z+(unit(state)-.5f)*2*radius,1/zmax,1/zmin);state=hash(state);float nx=p.x+(unit(state)-.5f)*exp2((float)-iter);
  state=hash(state);float ny=p.y+(unit(state)-.5f)*exp2((float)-iter);float3 normal=normalize((float3)(nx,ny,fmax(.3f,p.z)));
  float4 q=(float4)(normal,(normal.x*(x-c[14])/c[12]+normal.y*(y-c[15])/c[13]+normal.z)/inv);
  float val=cost(im,mask,c,geom,geoTol,W,H,V,x,y,q,zmin,zmax,-1,edgeSigma);if(val<best){p=q;best=val;}
 }out[i]=p;
}
float2 evidence(__global const uchar*im,__global const uchar*mask,__global const float*c,__global const float*geom,float geoTol,
 int W,int H,int V,int x,int y,float4 p,float zmin,float zmax,int view,float best,float edgeSigma){
 float z=dep(x,y,p,c),margin=0,sharpness=0;
__global const float*q=c+view*20;float baseline=length((float3)(q[9],q[10],q[11]));
  float delta=fmax(z*.001f,z*z/fmax(.001f,c[12]*baseline))*p.w/z;
  float4 a=p,b=p;a.w-=delta;b.w+=delta;
  float minus=cost(im,mask,c,geom,geoTol,W,H,V,x,y,a,zmin,zmax,view,edgeSigma),plus=cost(im,mask,c,geom,geoTol,W,H,V,x,y,b,zmin,zmax,view,edgeSigma);
  int offsets[8]={-16,-8,-4,-2,2,4,8,16},competitors=0;float alt=INFINITY;
  for(int k=0;k<8;k++){float inv=1/z+offsets[k]/fmax(.001f,c[12]*baseline);if(!(inv>0))continue;float4 q=p;q.w/=inv*z;
   float val=cost(im,mask,c,geom,geoTol,W,H,V,x,y,q,zmin,zmax,view,edgeSigma);if(isfinite(val)){alt=fmin(alt,val);competitors++;}}
  for(int k=0;k<4;k++){float zz=1/(1/zmax+(k/3.f)*(1/zmin-1/zmax));if(fabs(1/zz-1/z)*c[12]*baseline<1)continue;
   float4 q=p;q.w*=zz/z;float val=cost(im,mask,c,geom,geoTol,W,H,V,x,y,q,zmin,zmax,view,edgeSigma);if(isfinite(val)){alt=fmin(alt,val);competitors++;}}
  margin=competitors>=2?clamp((alt-best)/fmax(.1f,alt),0.f,1.f):0;
  sharpness=isfinite(minus+plus)?clamp((minus+plus-2*best)/fmax(.1f,minus+plus),0.f,1.f):0;
 return (float2)(margin,sharpness);
}
__kernel void finish(__global const uchar*im,__global const uchar*mask,__global const float*c,
 __global const float*geom,float geoTol,__global const float4*in,__global float*out,int W,int H,int V,float zmin,float zmax,float maxCost,float edgeSigma){
 int i=get_global_id(0);if(i>=W*H)return;int x=i%W,y=i/W;float4 p=in[i];float best=INFINITY;int view=-1;
 float costs[9];for(int v=1;v<V;v++){costs[v]=cost(im,mask,c,geom,geoTol,W,H,V,x,y,p,zmin,zmax,v,edgeSigma);if(costs[v]<best){best=costs[v];view=v;}}
 int valid=isfinite(cost(im,mask,c,geom,geoTol,W,H,V,x,y,p,zmin,zmax,-1,edgeSigma))&&best<=maxCost;
 float margin=0,sharpness=0,rank=-1;
 if(valid)for(int v=1;v<V;v++) {
  if(costs[v]>maxCost)continue;float2 signal=evidence(im,mask,c,geom,geoTol,W,H,V,x,y,p,zmin,zmax,v,costs[v],edgeSigma);
  float score=sqrt(clamp(1-costs[v],0.f,1.f))*sqrt(signal.x*signal.y);
  if(score>rank||(score==rank&&costs[v]<best)){rank=score;view=v;best=costs[v];margin=signal.x;sharpness=signal.y;}
 }
 out+=i*8;out[0]=valid?dep(x,y,p,c):NAN;out[1]=p.x;out[2]=p.y;out[3]=p.z;out[4]=best;out[5]=valid?(float)view:(isfinite(best)&&best<=maxCost?-2:-1);out[6]=margin;out[7]=sharpness;
}

""";

  /**
   * Tiled variant of SOURCE (compiled separately, so the untiled program is unchanged). Work items
   * cover a window of the reference; x and y are full-raster pixels, so every float expression,
   * hash and bound is the untiled one. Rasters are crops: cr[4v..4v+3] = x0, y0, width, height of
   * view v's crop (view 0 = reference, at atlas offset 0); a sample outside its crop is unavailable.
   */
  static final String TILED_SOURCE =
      """
#pragma OPENCL FP_CONTRACT OFF
uint hash(uint a){a^=a>>16;a*=0x7feb352d;a^=a>>15;a*=0x846ca68b;return a^(a>>16);}
float unit(uint a){return (a&0xffffff)/16777216.f;}
float dep(int x,int y,float4 p,__global const float*c){return p.w/(p.x*(x-c[14])/c[12]+p.y*(y-c[15])/c[13]+p.z);}
int geometric(__global const float*c,__global const int*cr,__global const float*geom,float geoTol,int x,int y,float z,int v){
 if(geoTol<=0)return -1;__global const float*q=c+20*v;
 float rx=(x-c[14])/c[12],ry=(y-c[15])/c[13];
 float3 t=(float3)((q[0]*rx+q[1]*ry+q[2])*z+q[9],(q[3]*rx+q[4]*ry+q[5])*z+q[10],(q[6]*rx+q[7]*ry+q[8])*z+q[11]);
 if(!(t.z>0))return -1;int xx=(int)floor(q[12]*t.x/t.z+q[14]+.5f),yy=(int)floor(q[13]*t.y/t.z+q[15]+.5f);
 int w=(int)q[16],h=(int)q[17],off=(int)q[18]*65536+(int)q[19];if(xx<1||yy<1||xx>=w-1||yy>=h-1)return -1;
 int x0=cr[4*v],y0=cr[4*v+1],cw=cr[4*v+2],ch=cr[4*v+3];if(xx-1<x0||yy-1<y0||xx+1>=x0+cw||yy+1>=y0+ch)return -1;
 float target=geom[off+(yy-y0)*cw+xx-x0];if(!(target>0)||!isfinite(target))return -1;
 float tol=fmax(.01f,3*geoTol);int coherent=0;
 for(int dy=-1;dy<=1;dy++)for(int dx=-1;dx<=1;dx++){float d=geom[off+(yy+dy-y0)*cw+xx+dx-x0];if(d>0&&isfinite(d)&&fabs(d-target)<=tol*target)coherent++;}
 if(coherent<7)return -1;return t.z<target*(1-tol)?1:t.z>target*(1+tol)?2:0;
}
int warpValid(__global const float*c,int x,int y,float4 p,int v){
 __global const float*q=c+20*v;float rx=(x-c[14])/c[12],ry=(y-c[15])/c[13],a=p.x*rx+p.y*ry+p.z;
 float3 t=(float3)(q[0]*rx+q[1]*ry+q[2]+q[9]*a/p.w,q[3]*rx+q[4]*ry+q[5]+q[10]*a/p.w,q[6]*rx+q[7]*ry+q[8]+q[11]*a/p.w);
 if(!(t.z>0))return 0;
 float3 dx=(float3)(q[0]+q[9]*p.x/p.w,q[3]+q[10]*p.x/p.w,q[6]+q[11]*p.x/p.w)/c[12];
 float3 dy=(float3)(q[1]+q[9]*p.y/p.w,q[4]+q[10]*p.y/p.w,q[7]+q[11]*p.y/p.w)/c[13];
 float ux=q[12]*(dx.x*t.z-t.x*dx.z)/(t.z*t.z),uy=q[12]*(dy.x*t.z-t.x*dy.z)/(t.z*t.z);
 float vx=q[13]*(dx.y*t.z-t.y*dx.z)/(t.z*t.z),vy=q[13]*(dy.y*t.z-t.y*dy.z)/(t.z*t.z);
 float det=ux*vy-uy*vx,ratio=q[12]*q[13]/(c[12]*c[13]);return det>.15f*ratio&&det<6*ratio&&ux*ux+uy*uy+vx*vx+vy*vy<12*det;
}
float cost(__global const uchar *im,__global const uchar *mask,__global const float*c,__global const int*cr,
 __global const float*geom,float geoTol,int W,int H,int V,
 int x,int y,float4 p,float zmin,float zmax,int selected,float edgeSigma){
 float z=dep(x,y,p,c);if(!isfinite(z)||z<zmin||z>zmax||x<2||y<2||x>=W-2||y>=H-2||!(p.w/z>.05f))return INFINITY;
 int rx0=cr[0],ry0=cr[1],rw=cr[2];
 float best=INFINITY,second=INFINITY;int contradictions=0;
 for(int v=1;v<V;v++){if(selected>0&&v!=selected)continue;__global const float *q=c+20*v;
  int w=(int)q[16],h=(int)q[17],off=(int)q[18]*65536+(int)q[19],x0=cr[4*v],y0=cr[4*v+1],cw=cr[4*v+2],ch=cr[4*v+3];
  float sa=0,sb=0,saa=0,sbb=0,sab=0,sw=0,sw2=0,center=im[(y-ry0)*rw+x-rx0];int valid=1;
  int evidence=geometric(c,cr,geom,geoTol,x,y,z,v);if(evidence==2)continue;if(evidence==1)contradictions++;
  if(!warpValid(c,x,y,p,v))continue;
  for(int dy=-2;dy<=2&&valid;dy++)for(int dx=-2;dx<=2;dx++){
   float rx=(x+dx-c[14])/c[12],ry=(y+dy-c[15])/c[13],d=p.w/(p.x*rx+p.y*ry+p.z);
   float3 t=(float3)((q[0]*rx+q[1]*ry+q[2])*d+q[9],(q[3]*rx+q[4]*ry+q[5])*d+q[10],(q[6]*rx+q[7]*ry+q[8])*d+q[11]);
   if(d<=0||t.z<=0||!isfinite(d+t.x+t.y+t.z)){valid=0;break;}float u=q[12]*t.x/t.z+q[14],vv=q[13]*t.y/t.z+q[15];
   int xx=(int)floor(u),yy=(int)floor(vv),at=(y+dy-ry0)*rw+x+dx-rx0;
   if(xx<0||yy<0||xx+1>=w||yy+1>=h||!mask[at]){valid=0;break;}
   if(xx<x0||yy<y0||xx+1>=x0+cw||yy+1>=y0+ch){valid=0;break;}int b=off+(yy-y0)*cw+xx-x0;
   if(!mask[b]||!mask[b+1]||!mask[b+cw]||!mask[b+cw+1]){valid=0;break;}
   float fx=u-xx,fy=vv-yy,av=im[at],bv=(1-fy)*((1-fx)*im[b]+fx*im[b+1])+fy*((1-fx)*im[b+cw]+fx*im[b+cw+1]);
   float difference=av-center,weight=edgeSigma<=0?1:1/((1+difference*difference/(edgeSigma*edgeSigma))*(1+.25f*(dx*dx+dy*dy)));
   sw+=weight;sw2+=weight*weight;sa+=weight*av;sb+=weight*bv;saa+=weight*av*av;sbb+=weight*bv*bv;sab+=weight*av*bv;
  }
  float va=saa-sa*sa/sw,vb=sbb-sb*sb/sw;if(!valid||sw*sw<5*sw2||va<4*sw||vb<4*sw)continue;
  float val=1-clamp((sab-sa*sb/sw)/sqrt(va*vb),-1.f,1.f);
  if(val<best){second=best;best=val;}else second=fmin(second,val);
 }
 if(selected>0)return best;if(contradictions>=2)return INFINITY;
 return best+.1f*contradictions+(isfinite(second)?.25f*fmin(.4f,second-best):0);
}
__kernel void propagate_tile(__global const uchar*im,__global const uchar*mask,__global const float*c,__global const int*cr,
 __global const float*geom,float geoTol,__global const float4*in,__global float4*out,int W,int H,int OX,int OY,int TW,int TH,int V,float zmin,float zmax,int iter,int stride,float edgeSigma){
 int i=get_global_id(0);if(i>=TW*TH)return;int lx=i%TW,ly=i/TW,x=OX+lx,y=OY+ly;float4 p=in[i];
 float best=cost(im,mask,c,cr,geom,geoTol,W,H,V,x,y,p,zmin,zmax,-1,edgeSigma);
 int at[4]={lx>=stride?i-stride:-1,lx+stride<TW?i+stride:-1,ly>=stride?i-stride*TW:-1,ly+stride<TH?i+stride*TW:-1};
 for(int k=0;k<4;k++){if(at[k]<0)continue;float4 q=in[at[k]];float val=cost(im,mask,c,cr,geom,geoTol,W,H,V,x,y,q,zmin,zmax,-1,edgeSigma);if(val<best){p=q;best=val;}}
 uint state=hash((uint)(y*W+x+iter*92821+71));
 for(int k=0;k<8;k++){
  float z=dep(x,y,p,c),radius=(1/zmin-1/zmax)*exp2((float)(-iter-1));state=hash(state);
  float inv=clamp(1/z+(unit(state)-.5f)*2*radius,1/zmax,1/zmin);state=hash(state);float nx=p.x+(unit(state)-.5f)*exp2((float)-iter);
  state=hash(state);float ny=p.y+(unit(state)-.5f)*exp2((float)-iter);float3 normal=normalize((float3)(nx,ny,fmax(.3f,p.z)));
  float4 q=(float4)(normal,(normal.x*(x-c[14])/c[12]+normal.y*(y-c[15])/c[13]+normal.z)/inv);
  float val=cost(im,mask,c,cr,geom,geoTol,W,H,V,x,y,q,zmin,zmax,-1,edgeSigma);if(val<best){p=q;best=val;}
 }out[i]=p;
}
float2 evidence(__global const uchar*im,__global const uchar*mask,__global const float*c,__global const int*cr,__global const float*geom,float geoTol,
 int W,int H,int V,int x,int y,float4 p,float zmin,float zmax,int view,float best,float edgeSigma){
 float z=dep(x,y,p,c),margin=0,sharpness=0;
__global const float*q=c+view*20;float baseline=length((float3)(q[9],q[10],q[11]));
  float delta=fmax(z*.001f,z*z/fmax(.001f,c[12]*baseline))*p.w/z;
  float4 a=p,b=p;a.w-=delta;b.w+=delta;
  float minus=cost(im,mask,c,cr,geom,geoTol,W,H,V,x,y,a,zmin,zmax,view,edgeSigma),plus=cost(im,mask,c,cr,geom,geoTol,W,H,V,x,y,b,zmin,zmax,view,edgeSigma);
  int offsets[8]={-16,-8,-4,-2,2,4,8,16},competitors=0;float alt=INFINITY;
  for(int k=0;k<8;k++){float inv=1/z+offsets[k]/fmax(.001f,c[12]*baseline);if(!(inv>0))continue;float4 q=p;q.w/=inv*z;
   float val=cost(im,mask,c,cr,geom,geoTol,W,H,V,x,y,q,zmin,zmax,view,edgeSigma);if(isfinite(val)){alt=fmin(alt,val);competitors++;}}
  for(int k=0;k<4;k++){float zz=1/(1/zmax+(k/3.f)*(1/zmin-1/zmax));if(fabs(1/zz-1/z)*c[12]*baseline<1)continue;
   float4 q=p;q.w*=zz/z;float val=cost(im,mask,c,cr,geom,geoTol,W,H,V,x,y,q,zmin,zmax,view,edgeSigma);if(isfinite(val)){alt=fmin(alt,val);competitors++;}}
  margin=competitors>=2?clamp((alt-best)/fmax(.1f,alt),0.f,1.f):0;
  sharpness=isfinite(minus+plus)?clamp((minus+plus-2*best)/fmax(.1f,minus+plus),0.f,1.f):0;
 return (float2)(margin,sharpness);
}
__kernel void finish_tile(__global const uchar*im,__global const uchar*mask,__global const float*c,__global const int*cr,
 __global const float*geom,float geoTol,__global const float4*in,__global float*out,int W,int H,int OX,int OY,int TW,int TH,int V,float zmin,float zmax,float maxCost,float edgeSigma){
 int i=get_global_id(0);if(i>=TW*TH)return;int x=OX+i%TW,y=OY+i/TW;float4 p=in[i];float best=INFINITY;int view=-1;
 float costs[9];for(int v=1;v<V;v++){costs[v]=cost(im,mask,c,cr,geom,geoTol,W,H,V,x,y,p,zmin,zmax,v,edgeSigma);if(costs[v]<best){best=costs[v];view=v;}}
 int valid=isfinite(cost(im,mask,c,cr,geom,geoTol,W,H,V,x,y,p,zmin,zmax,-1,edgeSigma))&&best<=maxCost;
 float margin=0,sharpness=0,rank=-1;
 if(valid)for(int v=1;v<V;v++) {
  if(costs[v]>maxCost)continue;float2 signal=evidence(im,mask,c,cr,geom,geoTol,W,H,V,x,y,p,zmin,zmax,v,costs[v],edgeSigma);
  float score=sqrt(clamp(1-costs[v],0.f,1.f))*sqrt(signal.x*signal.y);
  if(score>rank||(score==rank&&costs[v]<best)){rank=score;view=v;best=costs[v];margin=signal.x;sharpness=signal.y;}
 }
 out+=i*8;out[0]=valid?dep(x,y,p,c):NAN;out[1]=p.x;out[2]=p.y;out[3]=p.z;out[4]=best;out[5]=valid?(float)view:(isfinite(best)&&best<=maxCost?-2:-1);out[6]=margin;out[7]=sharpness;
}
""";
}
