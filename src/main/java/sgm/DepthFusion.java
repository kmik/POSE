package sgm;

import java.io.*;
import java.util.*;

/**
 * Per-ray depth consensus and independent multi-view refinement, without a ground-plane assumption.
 */
final class DepthFusion implements AutoCloseable {
  private final Camera[] cameras;
  private final int[] widths, heights;
  private final Options options;
  private final List<PairDepth> pairs = new ArrayList<>();
  private final PagedVolume.Pool pagePool;
  private final List<List<Endpoint>> byCamera = new ArrayList<>();
  private final double[][] centers, inverseRotations;
  private GpuFusion gpu;
  private boolean prepared;
  private long gpuCandidates, cpuCandidates, guardedCandidates;
  private volatile long gpuNanos, guardNanos, auditNanos, cpuNanos;

  record Endpoint(int pair, int image, DepthRaster map) {}

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

  DepthFusion(Camera[] cameras, int[] widths, int[] heights, Options options) {
    this.cameras = cameras;
    this.widths = widths;
    this.heights = heights;
    this.options = options;
    centers = new double[cameras.length][];
    inverseRotations = new double[cameras.length][];
    for (Camera camera : cameras) {
      byCamera.add(new ArrayList<>());
      int i = byCamera.size() - 1;
      centers[i] = camera.center();
      inverseRotations[i] = Math3.transpose(camera.rotation);
    }
    pagePool =
        new PagedVolume.Pool(
            Math.max(
                    1,
                    options.fusionCacheMb > 0
                        ? options.fusionCacheMb
                        : Math.min(512, options.memoryMb / 8))
                * 1048576L,
            128);
  }

  int addPair(int a, int b) {
    if (prepared) throw new IllegalStateException("Cannot add evidence after fusion preparation");
    int id = pairs.size();
    try {
      pairs.add(
          new PairDepth(a, b, widths[a] * heights[a], widths[b] * heights[b], options, pagePool));
      PairDepth p = pairs.get(id);
      byCamera.get(a).add(new Endpoint(id, a, p.first));
      // Preserve the original if/else lookup behavior for unusual self pairs.
      if (a != b) byCamera.get(b).add(new Endpoint(id, b, p.second));
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

  int pairCount() {
    return pairs.size();
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
            "per-camera majority depth modes, distinct measured cameras, guarded multi-view"
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
            "depth_refinement_method",
            options.depthRefinementMethod,
            "surface_consolidation",
            options.surfaceConsolidation,
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
    return fuse(source, world, options.depthRefinement);
  }

  /** Revalidate a proposed surface position without moving it back onto another depth estimate. */
  Result checkSurface(int source, double[] world) {
    return fuse(source, world, false);
  }

  boolean surfaceDisplacement(int source, double[] before, double[] after) {
    PairDepth p = pairs.get(source);
    return backProjection(p, before, after);
  }

  boolean surfaceDisplacement(int source, double[] before, double[] direction, double amount) {
    return surfaceDisplacement(source, before, Math3.add(before, Math3.times(direction, amount)));
  }

  GpuSurface surfaceDevice(int capacity) {
    if (gpu == null || options.surfaceValidation.equals("cpu")) return null;
    int[][] endpoints = new int[pairs.size()][2];
    for (int i = 0; i < pairs.size(); i++)
      endpoints[i] = new int[] {pairs.get(i).a, pairs.get(i).b};
    return new GpuSurface(cameras, endpoints, capacity, options, gpu.surfaceBudget());
  }

  boolean surfaceBatching() {
    return gpu != null && !options.surfaceValidation.equals("cpu");
  }

  record SurfaceSample(
      double[] normal, double footprint, double sigma, SurfaceAttributes attributes) {
    SurfaceSample(double[] normal, double footprint, double sigma) {
      this(normal, footprint, sigma, SurfaceAttributes.missing(normal != null));
    }
  }

  SurfaceSample surfaceSample(int source, Result result) {
    PairDepth pair = pairs.get(source);
    int image = pair.a;
    Camera camera = cameras[image];
    double[] p = result.point, v = camera.view(p);
    double footprint = v[2] / Math.min(camera.fx, camera.fy);
    double sigma =
        Math.max(footprint * .1, result.relativeSigma * Math3.norm(Math3.sub(p, centers[image])));
    double[] uv = camera.project(p);
    if (uv == null || !(footprint > 0)) return new SurfaceSample(null, footprint, sigma);
    int x = (int) Math.round(uv[0]), y = (int) Math.round(uv[1]);
    for (int stride : new int[] {1, 2, 4}) {
      double[][] neighbors = new double[49][];
      int n = 0;
      for (int dy = -3; dy <= 3; dy++)
        for (int dx = -3; dx <= 3; dx++) {
          int xx = x + dx * stride, yy = y + dy * stride;
          if (xx < 0 || yy < 0 || xx >= widths[image] || yy >= heights[image]) continue;
          int at = yy * widths[image] + xx;
          double z = pair.first.get(at);
          if (z <= 0 || Math.abs(z - v[2]) > options.fusionRelative * Math.min(z, v[2])) continue;
          double u = xx + offsetX(pair.first.offset(at)), w = yy + offsetY(pair.first.offset(at));
          neighbors[n++] =
              Math3.mv(
                  inverseRotations[image],
                  Math3.sub(
                      new double[] {
                        (u - camera.cx) / camera.fx * z, (w - camera.cy) / camera.fy * z, z
                      },
                      camera.translation));
        }
      SurfaceGeometry.Plane plane = SurfaceGeometry.plane(neighbors, n);
      if (plane == null
          || n < 9
          || plane.middle() < footprint * footprint * .2
          || plane.smallest() > .12 * plane.middle()) continue;
      double[] normal = plane.normal();
      double[] residuals = new double[n];
      for (int i = 0; i < n; i++)
        residuals[i] =
            Math.abs(SurfaceGeometry.dot(Math3.sub(neighbors[i], plane.center()), normal));
      Arrays.sort(residuals);
      double band = Math.max(footprint * .5, 3 * 1.4826 * residuals[n / 2]);
      int m = 0;
      for (int i = 0; i < n; i++)
        if (Math.abs(SurfaceGeometry.dot(Math3.sub(neighbors[i], plane.center()), normal)) <= band)
          neighbors[m++] = neighbors[i];
      if (m < n * .8) return new SurfaceSample(null, footprint, sigma);
      plane = SurfaceGeometry.plane(neighbors, m);
      if (plane == null || plane.smallest() > .12 * plane.middle()) continue;
      normal = plane.normal();
      double centerError = Math.abs(SurfaceGeometry.dot(Math3.sub(p, plane.center()), normal));
      // Use observed local scatter, not the often much larger geometric sigma, to
      // distinguish a noisy surface sample from a narrow foreground center.
      if (centerError > Math.max(.75 * footprint, 2 * Math.sqrt(plane.smallest())))
        return new SurfaceSample(null, footprint, sigma);
      double[] ray = Math3.sub(p, centers[image]);
      double cosine = Math.abs(SurfaceGeometry.dot(normal, ray)) / Math.max(1e-20, Math3.norm(ray));
      if (cosine < .15) return new SurfaceSample(null, footprint, sigma);
      if (SurfaceGeometry.dot(normal, ray) > 0) normal = Math3.times(normal, -1);
      return new SurfaceSample(
          normal,
          footprint,
          Math.max(.1 * footprint, sigma * cosine),
          SurfaceAttributes.of(plane, neighbors, m, p, m < n));
    }
    return new SurfaceSample(null, footprint, sigma);
  }

  private Result fuse(int source, double[] world, boolean refineDepth) {
    PairDepth origin = pairs.get(source);
    Mode[] modes = new Mode[cameras.length];
    double[][] projected = new double[cameras.length][];
    for (int image = 0; image < cameras.length; image++) {
      projected[image] = cameras[image].view(world);
      List<Measurement> measurements = new ArrayList<>();
      for (Endpoint endpoint : byCamera.get(image)) {
        Measurement m = lookup(endpoint.pair, image, endpoint.map, projected[image]);
        if (m != null) measurements.add(m);
      }
      modes[image] = mode(measurements);
    }
    if (!compatible(modes[origin.a], projected[origin.a][2])
        || !compatible(modes[origin.b], projected[origin.b][2]))
      return new Result(world, 0, 0, Double.NaN, 0, "depth_consensus_rejected");
    BitSet views = new BitSet(cameras.length);
    views.set(origin.a);
    views.set(origin.b);
    Map<Integer, Measurement> independent = new LinkedHashMap<>();
    Map<Integer, Measurement> byImage = new HashMap<>();
    for (int image = 0; image < cameras.length; image++) {
      Mode mode = modes[image];
      double z = projected[image][2];
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
    double anchorZ = projected[origin.a][2];
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
        refineDepth && independent.size() > 0 && weightSum > 0 ? sum / weightSum : anchorZ;
    double reprojectionSigma = 0;
    if (refineDepth
        && options.depthRefinementMethod.equals("reprojection")
        && independent.size() > 0) {
      List<ReprojectionDepth.Observation> observations = new ArrayList<>();
      for (int image = 0; image < cameras.length; image++) {
        if (image == origin.a) continue;
        Measurement m = byImage.get(image);
        double[] measured = image == origin.b ? world : m == null ? null : m.world;
        if (measured == null) continue;
        observations.add(ReprojectionDepth.link(anchor, cameras[image], world, measured));
      }
      ReprojectionDepth.Fit fit =
          ReprojectionDepth.fit(
              anchor, world, refinedDepth, observations, Math.max(.25, options.fusionPixels));
      refinedDepth = fit.depth();
      reprojectionSigma = fit.relativeSigma();
    }
    double[] q = projected[origin.a], local = Math3.times(q, refinedDepth / anchorZ);
    double[] refined = Math3.mv(inverseRotations[origin.a], Math3.sub(local, anchor.translation));
    // No covariance gain beyond the number of distinct additional cameras: shared pairs correlate.
    double sigma =
        1
            / Math.sqrt(Math.max(1e-30, weightSum))
            / Math.max(1e-12, refinedDepth)
            * Math.sqrt(Math.max(1, depths.size() / (double) Math.max(1, views.cardinality() - 1)));
    sigma = Math.max(sigma, reprojectionSigma);
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
          && backProjection(origin, refined, m.world)
          && (!options.fusionMeasuredReprojection
              || reprojection(cameras[image], refined, m.world))) verified.set(image);
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
            Math.toRadians(
                DenseReconstruction.triangulationAngle(point, centers[p.a], centers[p.b])));
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

  private Measurement lookup(int pair, int image, DepthRaster map, double[] v) {
    Camera c = cameras[image];
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
            inverseRotations[image],
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
    if (gpu != null) {
      gpu.close();
      gpu = null;
    }
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

  void prepare(long count) {
    if (prepared) return;
    prepared = true;
    String compute =
        options.fusionCompute.equals("inherit") ? options.compute : options.fusionCompute;
    if (!compute.equals("cpu")
        && (compute.equals("gpu") || count * Math.max(1, pairs.size()) >= 1000000)) {
      try {
        List<Endpoint> endpoints = new ArrayList<>();
        for (List<Endpoint> list : byCamera) endpoints.addAll(list);
        int[][] pairImages = new int[pairs.size()][2];
        for (int i = 0; i < pairs.size(); i++)
          pairImages[i] = new int[] {pairs.get(i).a, pairs.get(i).b};
        gpu =
            new GpuFusion(
                cameras, widths, heights, endpoints, pairImages, options, requiredViews());
      } catch (RuntimeException e) {
        if (compute.equals("gpu"))
          throw new IllegalArgumentException("GPU fusion required: " + e.getMessage(), e);
        System.out.println("Fusion GPU unavailable; using CPU: " + e.getMessage());
      }
    }
    System.out.println(
        gpu == null
            ? "Fusion backend: CPU (indexed camera evidence)"
            : "Fusion backend: OpenCL GPU (FP64 depth lookup, consensus and refinement)");
  }

  int pipelineCapacity() {
    return options.fusionBatchSize;
  }

  Result[] fuseBatch(int[] source, double[][] points, int count) {
    return batch(source, points, count, options.depthRefinement, true);
  }

  Result[] checkSurfaceBatch(int[] source, double[][] points, int count) {
    if (options.surfaceValidation.equals("gpu") && gpu == null)
      throw new IllegalArgumentException(
          "GPU surface validation requires prepared GPU fusion evidence");
    return batch(source, points, count, false, !options.surfaceValidation.equals("cpu"));
  }

  private Result[] batch(
      int[] source, double[][] points, int count, boolean refinement, boolean device) {
    Result[] results = null;
    if (device && gpu != null) {
      try {
        long clock = System.nanoTime();
        results = gpu.fuse(source, points, count, refinement);
        gpuNanos += System.nanoTime() - clock;
        clock = System.nanoTime();
        for (int i = 0; i < count; i++)
          if (results[i] == null) {
            results[i] = fuse(source[i], points[i], refinement);
            guardedCandidates++;
          }
        guardNanos += System.nanoTime() - clock;
        clock = System.nanoTime();
        // Continuous sparse CPU audit detects driver/compiler disagreements before publishing a
        // batch.
        List<Integer> auditIndices = new ArrayList<>();
        if (refinement) {
          for (int base = 0; base < count; base += 4096) {
            int n = Math.min(4096, count - base), audits = Math.min(8, n);
            for (int j = 0; j < audits; j++)
              auditIndices.add(base + j * (n - 1) / Math.max(1, audits - 1));
          }
        } else {
          int audits = Math.min(8, Math.max(1, (count + 511) / 512));
          for (int j = 0; j < audits; j++)
            auditIndices.add(j * (count - 1) / Math.max(1, audits - 1));
        }
        for (int i : auditIndices) {
          Result cpu = fuse(source[i], points[i], refinement);
          if (!equivalent(cpu, results[i]))
            throw new IllegalStateException(
                "CPU/GPU fusion audit disagreed at batch candidate "
                    + i
                    + "; CPU="
                    + cpu.status
                    + "/"
                    + cpu.views
                    + "/"
                    + cpu.independentPairs
                    + " "
                    + Arrays.toString(cpu.point)
                    + " sigma="
                    + cpu.relativeSigma
                    + "; GPU="
                    + results[i].status
                    + "/"
                    + results[i].views
                    + "/"
                    + results[i].independentPairs
                    + " "
                    + Arrays.toString(results[i].point)
                    + " sigma="
                    + results[i].relativeSigma);
        }
        gpuCandidates += count;
        auditNanos += System.nanoTime() - clock;
      } catch (RuntimeException e) {
        gpu.close();
        gpu = null;
        String compute =
            options.fusionCompute.equals("inherit") ? options.compute : options.fusionCompute;
        if (compute.equals("gpu") || (!refinement && options.surfaceValidation.equals("gpu")))
          throw new IllegalArgumentException("GPU fusion required: " + e.getMessage(), e);
        System.out.println(
            "Fusion GPU failed; replaying unpublished batch on CPU: " + e.getMessage());
        results = null;
      }
    }
    if (results == null) {
      long clock = System.nanoTime();
      results = new Result[count];
      for (int i = 0; i < count; i++) results[i] = fuse(source[i], points[i], refinement);
      cpuCandidates += count;
      cpuNanos += System.nanoTime() - clock;
    }
    return results;
  }

  static boolean equivalent(Result a, Result b) {
    if (!a.status.equals(b.status)
        || a.views != b.views
        || a.independentPairs != b.independentPairs) return false;
    double coordinateScale = 0;
    for (double value : a.point)
      if (Double.isFinite(value)) coordinateScale = Math.max(coordinateScale, Math.abs(value));
    // A world-to-camera rotation mixes axes. Large eastings/northings also condition Z;
    // assess all coordinates against the same frame scale (local tolerance stays 20 nm).
    double coordinateTolerance = 2e-8 + 2e-12 * coordinateScale;
    for (int i = 0; i < 3; i++)
      if (Double.doubleToLongBits(a.point[i]) != Double.doubleToLongBits(b.point[i])
          && (!Double.isFinite(a.point[i])
              || !Double.isFinite(b.point[i])
              || Math.abs(a.point[i] - b.point[i]) > coordinateTolerance)) return false;
    return (Double.isNaN(a.relativeSigma) && Double.isNaN(b.relativeSigma)
            || Math.abs(a.relativeSigma - b.relativeSigma)
                <= 1e-9 * Math.max(1, Math.abs(a.relativeSigma)))
        && Math.abs(a.shift - b.shift) <= 1e-8 + 2e-12 * coordinateScale;
  }

  Object backendReport() {
    return Json.object(
        "gpu_candidates",
        gpuCandidates,
        "cpu_candidates",
        cpuCandidates,
        "numerically_guarded_cpu_candidates",
        guardedCandidates,
        "cpu_audit_samples_per_4096_candidate_block",
        8,
        "depth_cache_bytes",
        pagePool.limit,
        "depth_page_side",
        options.fusionPageSide,
        "depth_page_reads",
        pagePool.reads,
        "depth_page_writes",
        pagePool.writes,
        "depth_page_evictions",
        pagePool.evictions,
        "depth_file_opens",
        pagePool.fileOpens,
        "gpu_batch_nanoseconds",
        gpuNanos,
        "cpu_guard_nanoseconds",
        guardNanos,
        "cpu_audit_nanoseconds",
        auditNanos,
        "cpu_batch_nanoseconds",
        cpuNanos,
        "weight_math",
        options.fusionWeightMath,
        "measurement_layout",
        options.fusionLayout,
        "pipeline_enabled",
        options.fusionPipeline,
        "measured_view_reprojection",
        options.fusionMeasuredReprojection,
        "depth_refinement_method",
        options.depthRefinementMethod,
        "gpu_resources",
        gpu == null ? null : gpu.resources());
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
