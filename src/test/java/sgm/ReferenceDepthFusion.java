package sgm;

import java.io.*;
import java.util.*;

/**
 * Per-ray depth consensus and independent multi-view refinement, without a ground-plane assumption.
 */
final class ReferenceDepthFusion implements AutoCloseable {
  private final Camera[] cameras;
  private final int[] widths, heights;
  private final Options options;
  private final List<PairDepth> pairs = new ArrayList<>();
  private final PagedVolume.Pool pagePool;

  static final class Result {
    final double[] point;
    final int views, independentPairs;
    final double relativeSigma, shift;
    final String status;

    Result(
        double[] point,
        int views,
        int independentPairs,
        double relativeSigma,
        double shift,
        String status) {
      this.point = point;
      this.views = views;
      this.independentPairs = independentPairs;
      this.relativeSigma = relativeSigma;
      this.shift = shift;
      this.status = status;
    }

    boolean accepted() {
      return status.equals("accepted");
    }
  }

  ReferenceDepthFusion(Camera[] cameras, int[] widths, int[] heights, Options options) {
    this.cameras = cameras;
    this.widths = widths;
    this.heights = heights;
    this.options = options;
    pagePool =
        new PagedVolume.Pool(
            Math.min(64 * 1048576L, Math.max(1048576L, options.memoryMb * 1048576L / 8)));
  }

  int addPair(int a, int b) {
    int id = pairs.size();
    try {
      pairs.add(
          new PairDepth(a, b, widths[a] * heights[a], widths[b] * heights[b], options, pagePool));
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
    return id;
  }

  void add(int pair, double[] world) {
    PairDepth p = pairs.get(pair);
    put(p.a, p.first, world);
    put(p.b, p.second, world);
  }

  int requiredViews() {
    int registered = 0;
    for (Camera c : cameras) if (c.registered) registered++;
    // Standalone numerical fixtures may not set registration flags.
    return Math.min(options.minViews, registered == 0 ? cameras.length : registered);
  }

  boolean consistent(int source, double[] world) {
    return fuse(source, world).accepted();
  }

  int supportingViews(int source, double[] world) {
    return fuse(source, world).views;
  }

  void export(ProjectArtifacts artifacts) throws IOException {
    artifacts.fusionConfiguration(
        Json.object(
            "filtering_preset",
            options.filtering,
            "filtering_explicit_overrides",
            options.filteringOverrides,
            "minimum_correlation",
            options.correlation,
            "minimum_speckle_component_pixels",
            options.speckleSize,
            "algorithm",
            "per-camera majority depth modes, distinct measured cameras, robust inverse-variance"
                + " depth refinement",
            "relative_depth_tolerance",
            options.fusionRelative,
            "reprojection_tolerance_pixels",
            options.fusionPixels,
            "per_ray_mode_fraction",
            options.fusionConsensus,
            "minimum_distinct_views",
            requiredViews(),
            "maximum_estimated_relative_stereo_sigma",
            options.maxDepthUncertainty,
            "disparity_sigma_assumption_pixels",
            .5,
            "refine_depth",
            options.depthRefinement,
            "uncertainty_scope",
            "stereo/parallax heuristic only; excludes camera calibration, pose and GPS error",
            "ground_assumption",
            "none; no horizontal/lowest-point clipping or terrain flattening",
            "evidence_storage",
            "per-pair camera-Z float32 plus packed subpixel x/y offsets; 8 bytes per pixel per"
                + " participating camera"));
    for (int i = 0; i < pairs.size(); i++) {
      PairDepth p = pairs.get(i);
      artifacts.pairEvidenceStream(
          i, p.a, p.first, widths[p.a], heights[p.a], p.b, p.second, widths[p.b], heights[p.b]);
    }
  }

  Result fuse(int source, double[] world) {
    PairDepth origin = pairs.get(source);
    Mode[] modes = new Mode[cameras.length];
    for (int image = 0; image < cameras.length; image++) {
      List<Measurement> measurements = new ArrayList<>();
      for (int pair = 0; pair < pairs.size(); pair++) {
        PairDepth evidence = pairs.get(pair);
        Measurement m = null;
        if (evidence.a == image) m = lookup(pair, image, evidence.first, world);
        else if (evidence.b == image) m = lookup(pair, image, evidence.second, world);
        if (m != null) measurements.add(m);
      }
      modes[image] = mode(measurements);
    }
    if (!compatible(modes[origin.a], cameras[origin.a].view(world)[2])
        || !compatible(modes[origin.b], cameras[origin.b].view(world)[2]))
      return new Result(world, 0, 0, Double.NaN, 0, "depth_consensus_rejected");
    BitSet views = new BitSet(cameras.length);
    views.set(origin.a);
    views.set(origin.b);
    Map<Integer, Measurement> independent = new LinkedHashMap<>();
    Map<Integer, Measurement> byImage = new HashMap<>();
    for (int image = 0; image < cameras.length; image++) {
      Mode mode = modes[image];
      double z = cameras[image].view(world)[2];
      if (!compatible(mode, z)) continue;
      for (Measurement m : mode.members) {
        if (m.pair == source
            || Math.abs(m.depth - z) > options.fusionRelative * Math.min(m.depth, z)
            || !backProjection(origin, world, m.world)) continue;
        // Credit only the actual image whose measurement passed, never the other pair endpoint.
        views.set(image);
        Measurement previous = independent.get(m.pair);
        if (previous == null || m.distance < previous.distance) independent.put(m.pair, m);
        Measurement old = byImage.get(image);
        if (old == null || m.weight > old.weight) byImage.put(image, m);
      }
    }
    if (views.cardinality() < requiredViews())
      return new Result(
          world,
          views.cardinality(),
          independent.size(),
          Double.NaN,
          0,
          "independent_support_rejected");
    List<WeightedDepth> depths = new ArrayList<>();
    Camera anchor = cameras[origin.a];
    double anchorZ = anchor.view(world)[2];
    depths.add(new WeightedDepth(anchorZ, weight(source, world, anchorZ)));
    for (Measurement m : independent.values()) {
      double z = anchor.view(m.world)[2];
      if (z > 0) depths.add(new WeightedDepth(z, weight(m.pair, m.world, z)));
    }
    double medianWeight = median(depths.stream().map(d -> d.weight).toArray(Double[]::new));
    for (WeightedDepth d : depths) d.weight = Math.min(d.weight, 4 * medianWeight);
    depths.sort(Comparator.comparingDouble(d -> d.depth));
    double total = 0;
    for (WeightedDepth d : depths) total += d.weight;
    double accumulated = 0, medianDepth = anchorZ;
    for (WeightedDepth d : depths) {
      accumulated += d.weight;
      if (accumulated >= total * .5) {
        medianDepth = d.depth;
        break;
      }
    }
    double sum = 0, weightSum = 0;
    for (WeightedDepth d : depths)
      if (Math.abs(d.depth - medianDepth)
          <= options.fusionRelative * Math.min(d.depth, medianDepth)) {
        sum += d.weight * d.depth;
        weightSum += d.weight;
      }
    double refinedDepth =
        options.depthRefinement && independent.size() > 0 && weightSum > 0
            ? sum / weightSum
            : anchorZ;
    double[] q = anchor.view(world), local = Math3.times(q, refinedDepth / anchorZ);
    double[] refined =
        Math3.mv(Math3.transpose(anchor.rotation), Math3.sub(local, anchor.translation));
    // No covariance gain beyond the number of distinct additional cameras: shared pairs correlate.
    double sigma =
        1
            / Math.sqrt(Math.max(1e-30, weightSum))
            / Math.max(1e-12, refinedDepth)
            * Math.sqrt(Math.max(1, depths.size() / (double) Math.max(1, views.cardinality() - 1)));
    if (!backProjection(origin, world, refined)
        || !compatible(modes[origin.a], anchor.view(refined)[2])
        || !compatible(modes[origin.b], cameras[origin.b].view(refined)[2]))
      return new Result(
          world,
          views.cardinality(),
          independent.size(),
          sigma,
          0,
          "refinement_reprojection_rejected");
    BitSet verified = new BitSet(cameras.length);
    verified.set(origin.a);
    verified.set(origin.b);
    for (Map.Entry<Integer, Measurement> entry : byImage.entrySet()) {
      int image = entry.getKey();
      Measurement m = entry.getValue();
      double z = cameras[image].view(refined)[2];
      if (compatible(modes[image], z)
          && Math.abs(m.depth - z) <= options.fusionRelative * Math.min(z, m.depth)
          && backProjection(origin, refined, m.world)) verified.set(image);
    }
    if (verified.cardinality() < requiredViews())
      return new Result(
          world,
          verified.cardinality(),
          independent.size(),
          sigma,
          0,
          "refinement_support_rejected");
    sigma *=
        Math.sqrt(
            Math.max(1, (views.cardinality() - 1.) / Math.max(1, verified.cardinality() - 1)));
    if (sigma > options.maxDepthUncertainty)
      return new Result(
          world,
          verified.cardinality(),
          independent.size(),
          sigma,
          0,
          "depth_uncertainty_rejected");
    return new Result(
        refined,
        verified.cardinality(),
        independent.size(),
        sigma,
        Math3.norm(Math3.sub(refined, world)),
        "accepted");
  }

  private boolean backProjection(PairDepth origin, double[] candidate, double[] measurement) {
    return reprojection(cameras[origin.a], candidate, measurement)
        && reprojection(cameras[origin.b], candidate, measurement);
  }

  private boolean reprojection(Camera camera, double[] a, double[] b) {
    double[] x = camera.project(a), y = camera.project(b);
    return x != null && y != null && Math.hypot(x[0] - y[0], x[1] - y[1]) <= options.fusionPixels;
  }

  private boolean compatible(Mode mode, double depth) {
    return mode != null
        && depth > 0
        && Math.abs(mode.depth - depth) <= options.fusionRelative * Math.min(mode.depth, depth);
  }

  private Mode mode(List<Measurement> input) {
    if (input.isEmpty()) return null;
    List<Measurement> best = Collections.emptyList();
    double bestWeight = 0;
    for (Measurement center : input) {
      List<Measurement> members = new ArrayList<>();
      double weights = 0;
      for (Measurement m : input)
        if (Math.abs(m.depth - center.depth)
            <= options.fusionRelative * Math.min(m.depth, center.depth)) {
          members.add(m);
          weights += m.weight;
        }
      if (members.size() > best.size() || members.size() == best.size() && weights > bestWeight) {
        best = members;
        bestWeight = weights;
      }
    }
    if (best.size() < Math.ceil(input.size() * options.fusionConsensus)) return null;
    double[] depths = new double[best.size()];
    for (int i = 0; i < best.size(); i++) depths[i] = best.get(i).depth;
    Arrays.sort(depths);
    return new Mode(depths[depths.length / 2], best);
  }

  private double weight(int pair, double[] point, double anchorDepth) {
    PairDepth p = pairs.get(pair);
    Camera a = cameras[p.a], b = cameras[p.b];
    double sine =
        Math.sin(
            Math.toRadians(DenseReconstruction.triangulationAngle(point, a.center(), b.center())));
    double f = Math.min(Math.min(a.fx, a.fy), Math.min(b.fx, b.fy));
    double sigma = anchorDepth * .5 / Math.max(1e-9, f * sine);
    return 1 / Math.max(1e-20, sigma * sigma);
  }

  private static double median(Double[] values) {
    Arrays.sort(values);
    return values[values.length / 2];
  }

  private void put(int image, DepthRaster depth, double[] world) {
    Camera c = cameras[image];
    double[] v = c.view(world);
    if (v[2] <= 0) return;
    double u = c.fx * v[0] / v[2] + c.cx, w = c.fy * v[1] / v[2] + c.cy;
    int x = (int) Math.round(u), y = (int) Math.round(w);
    if (x < 0 || x >= widths[image] || y < 0 || y >= heights[image]) return;
    int at = y * widths[image] + x;
    if (depth.get(at) == 0 || v[2] < depth.get(at)) {
      depth.put(at, (float) v[2], pack(u - x, w - y));
    }
  }

  private Measurement lookup(int pair, int image, DepthRaster map, double[] world) {
    Camera c = cameras[image];
    double[] v = c.view(world);
    if (v[2] <= 0) return null;
    double px = c.fx * v[0] / v[2] + c.cx, py = c.fy * v[1] / v[2] + c.cy;
    if (px < 0 || px > widths[image] - 1 || py < 0 || py > heights[image] - 1) return null;
    int x = (int) Math.round(px), y = (int) Math.round(py);
    double distance = Double.POSITIVE_INFINITY, u = 0, w = 0, z = 0;
    for (int dy = -1; dy <= 1; dy++)
      for (int dx = -1; dx <= 1; dx++) {
        int xx = x + dx, yy = y + dy;
        if (xx < 0 || xx >= widths[image] || yy < 0 || yy >= heights[image]) continue;
        int at = yy * widths[image] + xx;
        if (map.get(at) <= 0) continue;
        double sx = xx + offsetX(map.offset(at)),
            sy = yy + offsetY(map.offset(at)),
            d = (sx - px) * (sx - px) + (sy - py) * (sy - py);
        if (d < distance) {
          distance = d;
          u = sx;
          w = sy;
          z = map.get(at);
        }
      }
    if (z == 0 || distance > options.fusionPixels * options.fusionPixels) return null;
    double[] point =
        Math3.mv(
            Math3.transpose(c.rotation),
            Math3.sub(
                new double[] {(u - c.cx) / c.fx * z, (w - c.cy) / c.fy * z, z}, c.translation));
    return new Measurement(pair, image, z, distance, point, weight(pair, point, z));
  }

  static int pack(double x, double y) {
    return ((int) Math.round(x * 65534) & 65535) | ((int) Math.round(y * 65534) & 65535) << 16;
  }

  static double offsetX(int value) {
    return (short) (value & 65535) / 65534.;
  }

  static double offsetY(int value) {
    return (short) (value >>> 16) / 65534.;
  }

  private static final class PairDepth {
    final int a, b;
    final DepthRaster first, second;

    PairDepth(int a, int b, int na, int nb, Options options, PagedVolume.Pool pool)
        throws IOException {
      this.a = a;
      this.b = b;
      first = new DepthRaster(na, options, pool);
      second = new DepthRaster(nb, options, pool);
    }
  }

  public void close() throws IOException {
    IOException failure = null;
    for (PairDepth p : pairs) {
      try {
        p.first.close();
        p.second.close();
      } catch (IOException e) {
        failure = e;
      }
    }
    if (failure != null) throw failure;
  }

  void discardStorage() throws IOException {
    for (PairDepth p : pairs) {
      p.first.discard();
      p.second.discard();
    }
  }

  private static final class Measurement {
    final int pair, image;
    final double depth, distance, weight;
    final double[] world;

    Measurement(int pair, int image, double depth, double distance, double[] world, double weight) {
      this.pair = pair;
      this.image = image;
      this.depth = depth;
      this.distance = distance;
      this.world = world;
      this.weight = weight;
    }
  }

  private static final class Mode {
    final double depth;
    final List<Measurement> members;

    Mode(double depth, List<Measurement> members) {
      this.depth = depth;
      this.members = members;
    }
  }

  private static final class WeightedDepth {
    final double depth;
    double weight;

    WeightedDepth(double depth, double weight) {
      this.depth = depth;
      this.weight = weight;
    }
  }
}
