package sgm;

import java.io.*;
import java.util.*;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

/** Coarse global search, then overlapping local refinement with full-range fallback. */
final class DenseMatcher {
  static CensusSgm.Result compute(
      Options o,
      byte[] left,
      byte[] right,
      byte[] color,
      byte[] colorRight,
      int w,
      int h,
      byte[] maskA,
      byte[] maskB,
      CensusSgm.Observer observer)
      throws IOException {
    if (PointConfidence.bidirectional(o) && o.pyramidLevels != 1)
      throw new IllegalArgumentException(
          "Experimental bidirectional confidence requires ordinary SGM (--pyramid-levels 1)");
    if (o.pyramidLevels == 1)
      return DepthOutlierFilter.filter(o,new CensusSgm(o)
          .computeRecorded(left, right, color, colorRight, w, h, maskA, maskB, observer),left,observer);
    int levels = o.pyramidLevels;
    if (levels == 0) {
      levels = 1;
      long budget =
          Math.min(o.memoryMb * 1048576L, (long) (Runtime.getRuntime().maxMemory() * .65));
      while (levels < 6 && Math.min(w, h) / (1 << levels) >= 32) {
        int factor = 1 << (levels - 1),
            cw = Math.max(1, w / factor),
            ch = Math.max(1, h / factor),
            cd = Math.max(3, (o.disparities + factor - 1) / factor);
        if (cd <= 96 && CensusSgm.memoryEstimate(cw, ch, cd, o) <= budget) break;
        levels++;
      }
      System.out.println(
          "Automatic coarse-to-fine plan: " + levels + " levels; full-resolution pixels preserved");
    }
    while (levels > 1 && Math.min(w, h) / (1 << (levels - 1)) < 32) levels--;
    float[] previous = null;
    int pw = 0, ph = 0;
    CensusSgm.Result result = null;
    CensusSgm.Stats coarseStats = null;
    for (int level = levels - 1; level >= 0; level--) {
      int factor = 1 << level,
          lw = Math.max(1, (int) Math.round(w / (double) factor)),
          lh = Math.max(1, (int) Math.round(h / (double) factor));
      double scale = lw / (double) w;
      byte[] a = resize(left, w, h, lw, lh, 1, false),
          b = resize(right, w, h, lw, lh, 1, false),
          ma = resize(maskA, w, h, lw, lh, 1, true),
          mb = resize(maskB, w, h, lw, lh, 1, true);
      byte[]
          ca = o.censusMode.equals("color-distance") ? resize(color, w, h, lw, lh, 3, false) : null,
          cb =
              o.censusMode.equals("color-distance")
                  ? resize(colorRight, w, h, lw, lh, 3, false)
                  : null;
      Options current = o.copy();
      current.pyramidLevels = 1;
      current.minDisparity = (int) Math.floor(o.minDisparity * scale);
      current.disparities =
          Math.max(
              3,
              (int) Math.ceil((o.minDisparity + o.disparities - 1) * scale)
                  - current.minDisparity
                  + 1);
      current.speckleSize =
          level == 0 ? o.speckleSize : Math.max(0, o.speckleSize / (factor * factor));
      if (previous == null) {
        result = new CensusSgm(current).computeRecorded(a, b, ca, cb, lw, lh, ma, mb, null);
        coarseStats = result.stats;
      } else
        result =
            refine(
                current,
                a,
                b,
                ca,
                cb,
                lw,
                lh,
                ma,
                mb,
                previous,
                pw,
                ph,
                level == 0 ? observer : null);
      previous = result.disparity;
      if (level == 0 && coarseStats != null && coarseStats != result.stats) {
        result.stats.coarseLowerBoundaryRejected = coarseStats.lowerBoundaryRejected;
        result.stats.coarseUpperBoundaryRejected = coarseStats.upperBoundaryRejected;
        result.stats.coarseFullIntervalPixels = coarseStats.fullIntervalPixels;
      }
      pw = lw;
      ph = lh;
      if (observer != null) observer.raster("pyramid_level_" + level, result.disparity, lw, lh);
    }
    return DepthOutlierFilter.filter(o,result,left,observer);
  }

  private static CensusSgm.Result refine(
      Options o,
      byte[] a,
      byte[] b,
      byte[] ca,
      byte[] cb,
      int w,
      int h,
      byte[] ma,
      byte[] mb,
      float[] previous,
      int pw,
      int ph,
      CensusSgm.Observer observer)
      throws IOException {
    float[] out = new float[w * h], correlation = new float[w * h], lr = new float[w * h];
    float[] margin = new float[w * h], sharpness = new float[w * h];
    Arrays.fill(margin, Float.NaN);
    Arrays.fill(sharpness, Float.NaN);
    byte[] reasons = new byte[w * h];
    Arrays.fill(out, Float.NaN);
    Arrays.fill(correlation, Float.NaN);
    Arrays.fill(lr, Float.NaN);
    Arrays.fill(reasons, (byte) 1);
    CensusSgm.Stats stats = new CensusSgm.Stats();
    int fallback = 0, tiles = 0;
    List<Object> tileReports = new ArrayList<>();
    for (int y = 0; y < h; y += o.tileSize)
      for (int x = 0; x < w; x += o.tileSize) {
        int tw = Math.min(o.tileSize, w - x), th = Math.min(o.tileSize, h - y);
        List<Double> samples = new ArrayList<>();
        Set<Integer> coverage = new HashSet<>();
        int potential = 0;
        for (int yy = y; yy < y + th; yy += 4)
          for (int xx = x; xx < x + tw; xx += 4) {
            potential++;
            float v =
                previous[
                    Math.min(ph - 1, (int) (yy * (double) ph / h)) * pw
                        + Math.min(pw - 1, (int) (xx * (double) pw / w))];
            if (Float.isFinite(v)) {
              samples.add(v * (double) w / pw);
              coverage.add(
                  Math.min(3, (xx - x) * 4 / Math.max(1, tw))
                      + 4 * Math.min(3, (yy - y) * 4 / Math.max(1, th)));
            }
          }
        Collections.sort(samples);
        int min = o.minDisparity, max = o.minDisparity + o.disparities - 1;
        boolean predicted = samples.size() >= Math.max(8, potential / 5) && coverage.size() >= 4;
        if (predicted) {
          min = Math.max(min, (int) Math.floor(samples.getFirst()) - o.pyramidRadius);
          max = Math.min(max, (int) Math.ceil(samples.getLast()) + o.pyramidRadius);
        }
        Tile t = tile(o, a, b, ca, cb, w, h, ma, mb, x, y, tw, th, min, max);
        if (predicted
            && (t.boundary > Math.max(16, .01 * tw * th) || t.valid < Math.max(4, tw * th / 100))) {
          t =
              tile(
                  o,
                  a,
                  b,
                  ca,
                  cb,
                  w,
                  h,
                  ma,
                  mb,
                  x,
                  y,
                  tw,
                  th,
                  o.minDisparity,
                  o.minDisparity + o.disparities - 1);
          fallback++;
        }
        tiles++;
        tileReports.add(
            Json.object(
                "x",
                x,
                "y",
                y,
                "width",
                tw,
                "height",
                th,
                "prediction_samples",
                samples.size(),
                "occupied_cells",
                coverage.size(),
                "min_disparity",
                t.minimum,
                "disparities",
                t.count,
                "valid",
                t.valid,
                "boundary_rejected",
                t.boundary,
                "prediction_used",
                predicted));
        for (int yy = 0; yy < th; yy++)
          for (int xx = 0; xx < tw; xx++) {
            int src = (yy + t.haloY) * t.result.width + xx + t.halo, dst = (y + yy) * w + x + xx;
            float value = t.result.disparity[src];
            out[dst] = Float.isFinite(value) ? value + t.minimum : Float.NaN;
            correlation[dst] = t.result.correlation == null ? Float.NaN : t.result.correlation[src];
            lr[dst] = t.result.leftRightError == null ? Float.NaN : t.result.leftRightError[src];
            margin[dst] = t.result.costMargin == null ? Float.NaN : t.result.costMargin[src];
            sharpness[dst] =
                t.result.costSharpness == null ? Float.NaN : t.result.costSharpness[src];
            reasons[dst] = t.codes[src];
            byte flags = t.search[src];
            if ((flags & 4) != 0) stats.fullIntervalPixels++;
            if ((flags & 1) != 0 && t.minimum == o.minDisparity) stats.lowerBoundaryRejected++;
            if ((flags & 2) != 0 && t.minimum + t.count - 1 == o.minDisparity + o.disparities - 1)
              stats.upperBoundaryRejected++;
          }
      }
    for (int i = 0; i < out.length; i++) {
      if (Float.isFinite(out[i])) stats.valid++;
      switch (reasons[i]) {
        case 2 -> stats.boundaryRejected++;
        case 3 -> stats.ambiguityRejected++;
        case 4 -> stats.lrRejected++;
        case 5 -> stats.postMedianLrRejected++;
        case 6 -> stats.photometricRejected++;
        case 7 -> stats.speckleRejected++;
        case 8 -> stats.edgeErosionRejected++;
        default -> {}
      }
    }
    stats.selected =
        stats.valid
            + stats.lrRejected
            + stats.postMedianLrRejected
            + stats.photometricRejected
            + stats.speckleRejected
            + stats.edgeErosionRejected;
    if (observer != null) {
      observer.raster("05_final_stereo", out, w, h);
      observer.raster("zncc", correlation, w, h);
      observer.raster("left_right_error", lr, w, h);
      observer.reasons(reasons, w, h);
    }
    System.out.println(
        "Pyramid refinement: "
            + w
            + "x"
            + h
            + ", tiles="
            + tiles
            + ", full-range fallbacks="
            + fallback);
    if (o.stereoReportDirectory != null)
      Json.write(
          o.stereoReportDirectory.resolve("pyramid_" + w + "x" + h + ".json"),
          Json.object(
              "width",
              w,
              "height",
              h,
              "full_range_fallbacks",
              fallback,
              "tiles",
              tileReports,
              "path_scope",
              "Exact inside each refinement tile; overlapping pyramid tiles are a local"
                  + " approximation, distinct from exact strip-streamed full-image SGM"));
    CensusSgm.Result result = new CensusSgm.Result(w, h, out, stats, correlation, lr);
    result.costMargin = margin;
    result.costSharpness = sharpness;
    return result;
  }

  private record Tile(
      CensusSgm.Result result,
      byte[] codes,
      byte[] search,
      int halo,
      int haloY,
      int minimum,
      int count,
      int valid,
      int boundary) {}

  private static Tile tile(
      Options o,
      byte[] a,
      byte[] b,
      byte[] ca,
      byte[] cb,
      int w,
      int h,
      byte[] ma,
      byte[] mb,
      int x,
      int y,
      int tw,
      int th,
      int min,
      int max)
      throws IOException {
    int count = Math.max(3, max - min + 1),
        halo = Math.max(o.tileOverlap, count + o.census + o.correlationWindow + o.median),
        haloY = Math.max(o.tileOverlap, o.census + o.correlationWindow + o.median),
        width = tw + 2 * halo,
        height = th + 2 * haloY;
    Options local = o.copy();
    local.minDisparity = 0;
    local.disparities = count;
    local.pyramidLevels = 1;
    long budget = Math.min(o.memoryMb * 1048576L, (long) (Runtime.getRuntime().maxMemory() * .65));
    if (CensusSgm.memoryEstimate(width, height, count, o) > budget) local.tiled = true;
    byte[] codes = new byte[width * height];
    byte[] search = new byte[codes.length];
    CensusSgm.Observer collect =
        new CensusSgm.Observer() {
          public void raster(String n, float[] v, int w, int h) {}

          public void reasons(byte[] c, int w, int h) {
            System.arraycopy(c, 0, codes, 0, c.length);
          }

          public void searchBoundary(byte[] flags, int w, int h) {
            System.arraycopy(flags, 0, search, 0, flags.length);
          }
        };
    CensusSgm.Result r =
        new CensusSgm(local)
            .computeRecorded(
                crop(a, w, h, x - halo, y - haloY, width, height, 1),
                crop(b, w, h, x - halo - min, y - haloY, width, height, 1),
                crop(ca, w, h, x - halo, y - haloY, width, height, 3),
                crop(cb, w, h, x - halo - min, y - haloY, width, height, 3),
                width,
                height,
                crop(ma, w, h, x - halo, y - haloY, width, height, 1),
                crop(mb, w, h, x - halo - min, y - haloY, width, height, 1),
                collect);
    int valid = 0, boundary = 0;
    for (int yy = 0; yy < th; yy++)
      for (int xx = 0; xx < tw; xx++) {
        int at = (yy + haloY) * width + xx + halo;
        if (Float.isFinite(r.disparity[at])) valid++;
        if (codes[at] == 2) boundary++;
      }
    return new Tile(r, codes, search, halo, haloY, min, count, valid, boundary);
  }

  private static byte[] crop(
      byte[] in, int w, int h, int x, int y, int width, int height, int channels) {
    if (in == null) return null;
    byte[] out = new byte[Math.multiplyExact(Math.multiplyExact(width, height), channels)];
    for (int yy = 0; yy < height; yy++) {
      int sy = y + yy;
      if (sy < 0 || sy >= h) continue;
      int start = Math.max(0, x), end = Math.min(w, x + width);
      if (end > start)
        System.arraycopy(
            in,
            (sy * w + start) * channels,
            out,
            (yy * width + start - x) * channels,
            (end - start) * channels);
    }
    return out;
  }

  private static byte[] resize(
      byte[] in, int w, int h, int width, int height, int channels, boolean mask) {
    if (in == null) return null;
    if (w == width && h == height) return in;
    Mat src = new Mat(h, w, CvType.makeType(CvType.CV_8U, channels)), dst = new Mat();
    try {
      src.put(0, 0, in);
      Imgproc.resize(
          src,
          dst,
          new Size(width, height),
          0,
          0,
          mask ? Imgproc.INTER_NEAREST : Imgproc.INTER_AREA);
      byte[] out = new byte[width * height * channels];
      dst.get(0, 0, out);
      return out;
    } finally {
      src.release();
      dst.release();
    }
  }
}
