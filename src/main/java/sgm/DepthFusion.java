package sgm;

import java.io.*;
import java.util.*;
import java.util.concurrent.*;
import java.util.function.IntConsumer;

/**
 * Per-ray depth consensus and independent multi-view refinement, without a ground-plane assumption.
 */
final class DepthFusion implements AutoCloseable {
  private final Camera[] cameras;
  private final int[] widths, heights;
  private final Options options;
  private final List<PairDepth> pairs = new ArrayList<>();
  private final Map<Long, Integer> pairIdentities = new HashMap<>();
  private final List<Integer> canonicalPairs = new ArrayList<>();
  private boolean duplicatePairs;
  private final List<double[]> pairBounds = new ArrayList<>();
  private final PagedVolume.Pool pagePool;
  private final List<List<Endpoint>> byCamera = new ArrayList<>();
  private final double[][] centers, inverseRotations;
  private GpuFusion gpu;
  private boolean prepared;
  /**
   * Edge-aware surface-patch votes. Set by prepare() (reference fusion), not by
   * prepareSurfaceReaders(): image-fusion consolidation keeps its previous decisions.
   */
  private boolean patchContext;
  private ExecutorService workers;
  private int cpuWorkers;
  private CameraEvidence cameraEvidence;
  private long rawEvidenceBytes;
  private String evidenceLayoutReason = "pair_requested";
  private long residentEvidenceBytes, freezeNanos;
  private long gpuCandidates, cpuCandidates, guardedCandidates;
  private volatile long gpuNanos, guardNanos, auditNanos, cpuNanos;
  private long guardReads, auditReads, cpuReads, normalReads;
  private Object memoryAdmission;
  private final List<Object> gpuFailures = Collections.synchronizedList(new ArrayList<>());
  private int consecutiveGpuFailures;
  private final java.util.concurrent.atomic.LongAdder edgeSupported=new java.util.concurrent.atomic.LongAdder(),
      edgeFreeSpace=new java.util.concurrent.atomic.LongAdder(),edgeUnknown=new java.util.concurrent.atomic.LongAdder();
  private final java.util.concurrent.atomic.LongAdder patchSuspects=new java.util.concurrent.atomic.LongAdder(),
      patchUnsupported=new java.util.concurrent.atomic.LongAdder(),patchFreeSpace=new java.util.concurrent.atomic.LongAdder();
  private final Map<DepthRaster,SurfacePatchGrid> surfaceContexts=new ConcurrentHashMap<>();

  private long reads() {
    return pagePool.reads + pagePool.frozenReads.sum();
  }

  record Endpoint(int pair, int image, DepthRaster map) {}

  static final class Result {
    final double[] point;
    final int views, independentPairs;
    final double relativeSigma, shift;
    final String status;
    BitSet supportingCameras;
    double confidence = Double.NaN, matchingConfidence = Double.NaN;
    double rightMatchingConfidence = Double.NaN, twoViewMatchingConfidence = Double.NaN;
    double costMargin = Double.NaN,
        costSharpness = Double.NaN,
        photometric = Double.NaN,
        lrError = Double.NaN;

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

  double twoViewConfidenceThreshold() {
    return PointConfidence.bidirectional(options) ? options.twoViewConfidenceThreshold : Double.NaN;
  }

  int addPair(int a, int b) {
    if (prepared) throw new IllegalStateException("Cannot add evidence after fusion preparation");
    int id = pairs.size();
    try {
      pairs.add(
          new PairDepth(a, b, widths[a] * heights[a], widths[b] * heights[b], options, pagePool));
      pairBounds.add(new double[] {Double.POSITIVE_INFINITY,Double.POSITIVE_INFINITY,Double.POSITIVE_INFINITY,
          Double.NEGATIVE_INFINITY,Double.NEGATIVE_INFINITY,Double.NEGATIVE_INFINITY});
      PairDepth p = pairs.get(id);
      long identity = ((long) Math.min(a, b) << 32) | (Math.max(a, b) & 0xffffffffL);
      canonicalPairs.add(pairIdentities.computeIfAbsent(identity, key -> id));
      duplicatePairs |= canonicalPairs.get(id) != id;
      byCamera.get(a).add(new Endpoint(id, a, p.first));
      // Preserve the original if/else lookup behavior for unusual self pairs.
      if (a != b) byCamera.get(b).add(new Endpoint(id, b, p.second));
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
    return id;
  }

  void add(int pair, double[] world) {
    if (prepared) throw new IllegalStateException("Cannot add evidence after fusion preparation");
    PairDepth p = pairs.get(pair);
    if (options.fusionCameraCulling) {
      double[] box = pairBounds.get(pair);
      for (int axis=0;axis<3;axis++) { box[axis]=Math.min(box[axis],world[axis]);box[axis+3]=Math.max(box[axis+3],world[axis]); }
    }
    put(p.a, p.first, world);
    put(p.b, p.second, world);
  }

  /**
   * Projection part of {@link #add}, without side effects (safe in parallel): per endpoint, the
   * raster index (-1 if not stored), camera depth and packed subpixel offset.
   */
  void plan(int pair, double[] world, double[] out, int at) {
    PairDepth p = pairs.get(pair);
    planEndpoint(p.a, world, out, at);
    planEndpoint(p.b, world, out, at + 3);
  }

  private void planEndpoint(int image, double[] world, double[] out, int at) {
    out[at] = -1;
    Camera c = cameras[image];
    double[] v = c.view(world);
    if (v[2] <= 0) return;
    double u = c.fx * v[0] / v[2] + c.cx, w = c.fy * v[1] / v[2] + c.cy;
    int x = (int) Math.round(u), y = (int) Math.round(w);
    if (x < 0 || x >= widths[image] || y < 0 || y >= heights[image]) return;
    out[at] = y * widths[image] + x;
    out[at + 1] = v[2];
    out[at + 2] = pack(u - x, w - y);
  }

  /** Applies a {@link #plan}; identical to add(pair, world) when called in the same order. */
  void addPlanned(int pair, double[] world, double[] plan, int at) {
    if (prepared) throw new IllegalStateException("Cannot add evidence after fusion preparation");
    PairDepth p = pairs.get(pair);
    if (options.fusionCameraCulling) {
      double[] box = pairBounds.get(pair);
      for (int axis=0;axis<3;axis++) { box[axis]=Math.min(box[axis],world[axis]);box[axis+3]=Math.max(box[axis+3],world[axis]); }
    }
    store(p.first, plan, at);
    store(p.second, plan, at + 3);
  }

  private static void store(DepthRaster depth, double[] plan, int at) {
    if (plan[at] < 0) return;
    int index = (int) plan[at];
    double z = plan[at + 1];
    if (depth.get(index) == 0 || z < depth.get(index))
      depth.put(index, (float) z, (int) plan[at + 2]);
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

  /** First (rectified-left) and second camera of a fusion pair. */
  int pairCamera(int pair, boolean second) {
    return second ? pairs.get(pair).b : pairs.get(pair).a;
  }

  Camera camera(int image) {
    return cameras[image];
  }

  int width(int image) {
    return widths[image];
  }

  int height(int image) {
    return heights[image];
  }

  /** Inverse depth variance from the pair's triangulation angle (see {@link #fuse}). */
  double pairWeight(int pair, double[] point, double depth) {
    return weight(pair, point, depth);
  }

  int cameraCount() {
    return cameras.length;
  }

  CameraEmission.Key ownerKey(SurfaceConsolidator.Sample p) {
    int owner = p.supportingCameras == null ? -1 : p.supportingCameras.nextSetBit(0);
    if (owner < 0 || owner >= cameras.length) owner = pairs.get(p.source).a;
    double[] v = cameras[owner].view(p.point), uv = cameras[owner].project(p.point);
    if (uv == null || uv[0] < 0 || uv[1] < 0 || uv[0] >= widths[owner] || uv[1] >= heights[owner])
      return new CameraEmission.Key(-1, p.id, 0);
    return new CameraEmission.Key(
        owner, (long) Math.floor(uv[1]) * widths[owner] + (long) Math.floor(uv[0]), v[2]);
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
            "dense_method",
            options.denseMethod,
            "confidence_model",
            PointConfidence.MODEL,
            "fusion_mode",
            options.fusionMode,
            "camera_outlier_model",
            options.fusionOutliers,
            "visibility_voting",
            options.fusionOutliers.equals("robust")
                ? "Distinct additional cameras; at least ceil(verified views/2) coherent free-space"
                      + " contradictions; verified supporters excluded. A single additional camera"
                      + " can still contradict two-view-only points."
                : "One additional camera can veto after two coherent pair patches",
            "confidence_threshold",
            options.confidenceThreshold,
            "visibility_checks",
            "At most eight additional cameras, four pair maps each; require two coherent 3x3"
                + " patches (7/9 valid), agreement within depth tolerance and free-space gap above"
                + " 3x tolerance. Missing depth and nearer occluders neutral.",
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
    return visibility(source, fuse(source, world, options.depthRefinement));
  }

  /** Revalidate a proposed surface position without moving it back onto another depth estimate. */
  Result checkSurface(int source, double[] world) {
    return visibility(source, fuse(source, world, false));
  }

  /** Missing depth and a nearer occluder are neutral; reliable free space is contradictory. */
  private Result visibility(int source, Result result) {
    if (!result.accepted()) return result;
    String edgeStatus=DepthOutlierFilter.enabled(options)?edgeDecision(source,result):null;
    if(edgeStatus!=null) {
      return new Result(result.point,result.views,result.independentPairs,result.relativeSigma,
          result.shift,edgeStatus);
    }
    PairDepth origin = pairs.get(source);
    int inspected = 0, contradictions = 0;
    boolean robust = options.fusionOutliers.equals("robust");
    int requiredContradictions = robust ? Math.max(1, (result.views + 1) / 2) : 1;
    for (int image = 0; image < cameras.length && inspected < 8; image++) {
      if (image == origin.a || image == origin.b || byCamera.get(image).size() < 2) continue;
      if (robust && result.supportingCameras != null && result.supportingCameras.get(image))
        continue;
      double[] v = cameras[image].view(result.point);
      if (v[2] <= 0) continue;
      double[] uv = cameras[image].project(result.point);
      if (uv == null
          || uv[0] < 2
          || uv[1] < 2
          || uv[0] >= widths[image] - 2
          || uv[1] >= heights[image] - 2) continue;
      inspected++;
      double near = Double.POSITIVE_INFINITY, far = 0;
      int reliable = 0, examined = 0;
      BitSet reliablePartners = robust ? new BitSet(cameras.length) : null;
      for (Endpoint e : byCamera.get(image)) {
        if (e.pair == source) continue;
        if (++examined > 4) break;
        var pair = pairs.get(e.pair);
        int partner = pair.a == image ? pair.b : pair.a;
        if (robust && reliablePartners.get(partner)) continue;
        int x = (int) Math.round(uv[0]), y = (int) Math.round(uv[1]), count = 0;
        double lo = Double.POSITIVE_INFINITY, hi = 0;
        for (int dy = -1; dy <= 1; dy++)
          for (int dx = -1; dx <= 1; dx++) {
            double z = e.map.get((y + dy) * widths[image] + x + dx);
            if (z > 0) {
              lo = Math.min(lo, z);
              hi = Math.max(hi, z);
              count++;
            }
          }
        // Require a coherent, densely observed patch away from foreground/background edges.
        if (count < 7 || hi - lo > options.fusionRelative * lo) continue;
        near = Math.min(near, lo);
        far = Math.max(far, hi);
        reliable++;
        if (robust) reliablePartners.set(partner);
      }
      if (reliable >= 2
          && far - near <= options.fusionRelative * near
          && v[2] < near * (1 - 3 * options.fusionRelative)
          && ++contradictions >= requiredContradictions)
        return new Result(
            result.point,
            result.views,
            result.independentPairs,
            result.relativeSigma,
            result.shift,
            "visibility_contradiction_rejected");
    }
    return result;
  }

  /** Check distinct cameras outside the generating pair, using two distinct partner maps
   * per camera. No agreement is inferred from a missing map or a foreground occluder. */
  private boolean edgeVisibility(int source,Result result) {
    return edgeDecision(source,result)!=null;
  }
  private SurfacePatchGrid context(int image,DepthRaster raster) {
    return surfaceContexts.computeIfAbsent(raster,key->new SurfacePatchGrid(widths[image],heights[image],p->key.get(p),options.fusionRelative,SurfacePatchGrid.cacheCells(options,2*pairs.size())));
  }
  private String edgeDecision(int source,Result result) {
    PairDepth origin=pairs.get(source);
    int supports=0,free=0;
    boolean suspect=false;
    if(patchContext) {
      double[] uv=cameras[origin.a].project(result.point);
      if(uv!=null) {
        int kind=context(origin.a,origin.first).predict(uv[0],uv[1],cameras[origin.a].view(result.point)[2]).kind();
        suspect=kind==2||kind==3;
      }
    }
    for(int image=0;image<cameras.length;image++) {
      if(image==origin.a || image==origin.b || byCamera.get(image).size()<2)continue;
      Camera camera=cameras[image];double[] view=camera.view(result.point),uv=camera.project(result.point);
      if(view[2]<=0 || uv==null || uv[0]<1 || uv[1]<1 || uv[0]>=widths[image]-1 || uv[1]>=heights[image]-1)continue;
      int x=(int)Math.round(uv[0]),y=(int)Math.round(uv[1]);
      if(x>=widths[image]-1 || y>=heights[image]-1)continue;
      BitSet partners=new BitSet(cameras.length);
      int agree=0,through=0;
      for(Endpoint e:byCamera.get(image)) {
        PairDepth pair=pairs.get(e.pair);int partner=pair.a==image?pair.b:pair.a;
        if(partners.get(partner))continue;
        double low=Double.POSITIVE_INFINITY,high=0;int count=0;
        for(int dy=-1;dy<=1;dy++)for(int dx=-1;dx<=1;dx++) {
          double z=e.map.get((y+dy)*widths[image]+x+dx);
          if(z>0){low=Math.min(low,z);high=Math.max(high,z);count++;}
        }
        double sine=Math.sin(Math.toRadians(triangulationAngleDegrees(result.point,centers[image],centers[partner])));
        double footprint=view[2]/Math.min(camera.fx,camera.fy);
        double measurement=footprint*.5/Math.max(1e-6,sine);
        double sigma=Math.hypot(Math.abs(result.relativeSigma*view[2]),measurement);
        int vote=DepthOutlierFilter.cameraVote(view[2],e.map.get(y*widths[image]+x),low,high,count,sigma,footprint,options.fusionRelative);
        if(vote==0&&patchContext) {
          var patch=context(image,e.map).predict(uv[0],uv[1],view[2]);
          if(patch.kind()==2) {
            suspect=true;
            double band=3*Math.hypot(Math.abs(result.relativeSigma*view[2]),Math.hypot(patch.sigma(),footprint));
            if(view[2]<patch.depth()-band){vote=2;patchFreeSpace.increment();}
          }
        }
        if(vote==1 || vote==2){partners.set(partner);if(vote==1)agree++;else through++;}
      }
      if(agree>=2)supports++;
      else if(through>=2)free++;
    }
    if(suspect)patchSuspects.increment();
    if(DepthOutlierFilter.cameraReject(supports,free)){edgeFreeSpace.increment();return "visibility_contradiction_rejected";}
    if(suspect&&supports==0){patchUnsupported.increment();return "independent_support_rejected";}
    if(supports>0)edgeSupported.increment();else edgeUnknown.increment();
    return null;
  }
  private static double triangulationAngleDegrees(double[] p,double[] a,double[] b) {
    double[] u=Math3.sub(p,a),v=Math3.sub(p,b);
    double cosine=(u[0]*v[0]+u[1]*v[1]+u[2]*v[2])/Math.max(1e-30,Math3.norm(u)*Math3.norm(v));
    return Math.toDegrees(Math.acos(Math.max(-1,Math.min(1,cosine))));
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

  /** True when parallel() actually uses CPU fusion workers. */
  boolean cpuParallel() {
    return workers != null;
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
    double[] uv = camera.projectView(v);
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
        residuals[i] = Math.abs(SurfaceGeometry.distance(neighbors[i], plane.center(), normal));
      Arrays.sort(residuals);
      double band = Math.max(footprint * .5, 3 * 1.4826 * residuals[n / 2]);
      int m = 0;
      for (int i = 0; i < n; i++)
        if (Math.abs(SurfaceGeometry.distance(neighbors[i], plane.center(), normal)) <= band)
          neighbors[m++] = neighbors[i];
      if (m < n * .8) return new SurfaceSample(null, footprint, sigma);
      plane = SurfaceGeometry.plane(neighbors, m);
      if (plane == null || plane.smallest() > .12 * plane.middle()) continue;
      normal = plane.normal();
      double centerError = Math.abs(SurfaceGeometry.distance(p, plane.center(), normal));
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
      if (duplicatePairs && options.fusionOutliers.equals("robust")) {
        Map<Integer, Measurement> unique = new LinkedHashMap<>();
        for (var m : measurements) {
          int key = canonicalPairs.get(m.pair);
          var previous = unique.get(key);
          if (previous == null || m.distance < previous.distance) unique.put(key, m);
        }
        measurements = new ArrayList<>(unique.values());
      }
      modes[image] = mode(measurements);
    }
    if (!compatible(modes[origin.a], projected[origin.a][2])
        || !compatible(modes[origin.b], projected[origin.b][2]))
      return new Result(world, 0, 0, Double.NaN, 0, "depth_consensus_rejected");
    BitSet views = new BitSet(cameras.length);
    double[] originA = cameras[origin.a].projectView(projected[origin.a]),
        originB = cameras[origin.b].projectView(projected[origin.b]);
    views.set(origin.a);
    views.set(origin.b);
    Map<Integer, Measurement> independent = new LinkedHashMap<>();
    Map<Integer, Measurement> byImage = new HashMap<>();
    for (int image = 0; image < cameras.length; image++) {
      Mode mode = modes[image];
      double z = projected[image][2];
      if (!compatible(mode, z)) continue;
      for (Measurement m : mode.members) {
        PairDepth measuredPair = pairs.get(m.pair);
        boolean producingCameras =
            (measuredPair.a == origin.a && measuredPair.b == origin.b)
                || (measuredPair.a == origin.b && measuredPair.b == origin.a);
        if (m.pair == source
            || producingCameras
            || Math.abs(m.depth - z) > options.fusionRelative * Math.min(m.depth, z)
            || !backProjection(origin, originA, originB, m.world)) continue;
        // Credit only the actual image whose measurement passed, never the other pair endpoint.
        views.set(image);
        int identity =
            options.fusionOutliers.equals("robust") ? canonicalPairs.get(m.pair) : m.pair;
        Measurement previous = independent.get(identity);
        if (previous == null || m.distance < previous.distance) independent.put(identity, m);
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
      var context = new ReprojectionDepth.Context(anchor, world, centers[origin.a]);
      for (int image = 0; image < cameras.length; image++) {
        if (image == origin.a) continue;
        Measurement m = byImage.get(image);
        double[] measured = image == origin.b ? world : m == null ? null : m.world;
        if (measured == null) continue;
        observations.add(ReprojectionDepth.link(context, cameras[image], measured));
      }
      ReprojectionDepth.Fit fit =
          ReprojectionDepth.fit(
              context, refinedDepth, observations, Math.max(.25, options.fusionPixels));
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
    double[] refinedA = anchor.view(refined), refinedB = cameras[origin.b].view(refined);
    double[] refinedUvA = anchor.projectView(refinedA),
        refinedUvB = cameras[origin.b].projectView(refinedB);
    if (!closeProjection(originA, refinedUvA)
        || !closeProjection(originB, refinedUvB)
        || !compatible(modes[origin.a], refinedA[2])
        || !compatible(modes[origin.b], refinedB[2]))
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
      double[] imageView =
          image == origin.a
              ? refinedA
              : image == origin.b ? refinedB : cameras[image].view(refined);
      double z = imageView[2];
      if (compatible(modes[image], z)
          && Math.abs(m.depth - z) <= options.fusionRelative * Math.min(z, m.depth)
          && backProjection(origin, refinedUvA, refinedUvB, m.world)
          && (!options.fusionMeasuredReprojection
              || closeProjection(
                  cameras[image].projectView(imageView), cameras[image].project(m.world))))
        verified.set(image);
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
    Result accepted =
        new Result(
            refined,
            verified.cardinality(),
            independent.size(),
            sigma,
            Math3.norm(Math3.sub(refined, world)),
            "accepted");
    accepted.supportingCameras = verified;
    return accepted;
  }

  private boolean backProjection(PairDepth origin, double[] candidate, double[] measurement) {
    return reprojection(cameras[origin.a], candidate, measurement)
        && reprojection(cameras[origin.b], candidate, measurement);
  }

  private boolean backProjection(PairDepth origin, double[] a, double[] b, double[] measurement) {
    return closeProjection(a, cameras[origin.a].project(measurement))
        && closeProjection(b, cameras[origin.b].project(measurement));
  }

  private boolean reprojection(Camera camera, double[] a, double[] b) {
    double[] x = camera.project(a), y = camera.project(b);
    return closeProjection(x, y);
  }

  private boolean closeProjection(double[] x, double[] y) {
    return x != null && y != null && within(x[0] - y[0], x[1] - y[1], options.fusionPixels);
  }

  static boolean within(double x, double y, double limit) {
    double distance = x * x + y * y, bound = limit * limit;
    if (!Double.isFinite(distance + bound)
        || Math.abs(distance - bound) <= 1e-12 * Math.max(1, bound))
      return Math.hypot(x, y) <= limit;
    return distance <= bound;
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
    double[] u = Math3.sub(point, centers[p.a]), v = Math3.sub(point, centers[p.b]);
    double denominator = Math3.norm(u) * Math3.norm(v);
    double x = u[1] * v[2] - u[2] * v[1],
        y = u[2] * v[0] - u[0] * v[2],
        z = u[0] * v[1] - u[1] * v[0];
    double sine =
        denominator == 0 ? 0 : Math.min(1, Math.sqrt(x * x + y * y + z * z) / denominator);
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
    DepthRaster first, second;

    PairDepth(int a, int b, int na, int nb, Options options, PagedVolume.Pool pool)
        throws IOException {
      this.a = a;
      this.b = b;
      first = new DepthRaster(na, options, pool);
      second = new DepthRaster(nb, options, pool);
    }
  }

  public void close() throws IOException {
    if (workers != null) {
      workers.shutdown();
      boolean interrupted = false;
      for (; ; )
        try {
          if (workers.awaitTermination(5, TimeUnit.SECONDS)) break;
        } catch (InterruptedException e) {
          interrupted = true;
        }
      if (interrupted) Thread.currentThread().interrupt();
      workers = null;
    }
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
    pagePool.closeReaders();
    if (cameraEvidence != null) cameraEvidence.clear();
    if (failure != null) throw failure;
  }

  void discardStorage() throws IOException {
    for (PairDepth p : pairs) {
      p.first.discard();
      p.second.discard();
    }
  }

  void prepare(long count) {
    prepare(count, true);
    patchContext = true;
  }

  /**
   * Image fusion never prepares this per-pair backend. Surface consolidation re-validates moved
   * points against it, so it freezes the evidence (read-only, concurrently readable) and starts
   * the CPU workers here, without a GPU backend: results equal the serial CPU reference.
   */
  void prepareSurfaceReaders(long count) {
    prepare(count, false);
  }

  private void prepare(long count, boolean allowGpu) {
    if (prepared) return;
    prepared = true;
    long freezeStarted = System.nanoTime(), evidenceBytes = 0;
    for (PairDepth p : pairs) evidenceBytes += 8L * (p.first.length + p.second.length);
    long availableHeap =
        Math.max(
            0,
            Runtime.getRuntime().maxMemory()
                - (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory()));
    long residentBudget =
        Math.min(
            options.memoryMb * 1048576L / 2,
            Math.min(Runtime.getRuntime().maxMemory() / 3, availableHeap / 2));
    if (options.fusionCacheMb > 0)
      residentBudget = Math.min(residentBudget, options.fusionCacheMb * 1048576L);
    residentBudget = MemorySafety.snapshot().allowance(residentBudget);
    var admission = MemorySafety.snapshot();
    long roundedEvidence = 0;
    for (PairDepth p : pairs) {
      roundedEvidence += MemorySafety.arrayBytes(2L * p.first.length, 4, admission.regionBytes());
      roundedEvidence += MemorySafety.arrayBytes(2L * p.second.length, 4, admission.regionBytes());
    }
    memoryAdmission =
        Json.object(
            "heap_used_bytes",
            admission.heapUsed(),
            "heap_committed_bytes",
            admission.heapCommitted(),
            "heap_maximum_bytes",
            admission.heapMaximum(),
            "available_os_commit_bytes",
            admission.availableCommit(),
            "g1_region_bytes",
            admission.regionBytes(),
            "resident_evidence_allowance_bytes",
            residentBudget,
            "pair_arrays_rounded_allocation_bytes",
            roundedEvidence);
    boolean resident = roundedEvidence <= residentBudget;
    rawEvidenceBytes = evidenceBytes;
    boolean pack =
        options.fusionEvidence.equals("camera")
            || options.fusionEvidence.equals("auto") && cameras.length > 8;
    if (pack) {
      if (pairs.stream().noneMatch(p -> p.a == p.b))
        cameraEvidence =
            CameraEvidence.build(
                byCamera,
                widths,
                heights,
                residentBudget,
                evidenceBytes,
                options.fusionEvidence.equals("auto"));
      evidenceLayoutReason =
          cameraEvidence == null ? "limits_or_no_saving" : "lossless_camera_layers";
      if (cameraEvidence == null)
        System.out.println(
            "Camera evidence: safe allocation plan unavailable; retaining bounded paged pair maps");
      if (cameraEvidence == null && options.fusionEvidence.equals("camera"))
        throw new IllegalArgumentException(
            "Camera evidence packing requires <=32 incident pairs per camera and sufficient host"
                + " budget");
      if (cameraEvidence != null)
        try {
          installCameraEvidence();
        } catch (IOException e) {
          throw new UncheckedIOException(e);
        }
    } else if (options.fusionEvidence.equals("auto")) evidenceLayoutReason = "small_block";
    try {
      for (PairDepth p : pairs) {
        p.first.freeze(resident);
        p.second.freeze(resident);
      }
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
    residentEvidenceBytes = 0;
    for (PairDepth p : pairs) {
      if (p.first.resident()) residentEvidenceBytes += 8L * p.first.length;
      if (p.second.resident()) residentEvidenceBytes += 8L * p.second.length;
    }
    if (cameraEvidence != null) residentEvidenceBytes = cameraEvidence.bytes;
    freezeNanos = System.nanoTime() - freezeStarted;
    int readers =
        cameraEvidence != null || residentEvidenceBytes == evidenceBytes
            ? options.cores
            : (int)
                Math.max(
                    1,
                    Math.min(
                        options.cores,
                        pagePool.limit / (4L * options.fusionPageSide * options.fusionPageSide)));
    cpuWorkers = Math.max(0, readers - 1);
    // Output/coordinator and prefetch threads can perform small serial checks as well.
    pagePool.readerCount(cpuWorkers + 2);
    if (cpuWorkers > 0)
      workers =
          Executors.newFixedThreadPool(
              cpuWorkers,
              r ->
                  new Thread(
                      () -> {
                        try {
                          r.run();
                        } finally {
                          releaseReader();
                        }
                      },
                      "fusion-cpu"));
    String compute =
        options.fusionCompute.equals("inherit") ? options.compute : options.fusionCompute;
    if (allowGpu && !compute.equals("cpu")
        && (compute.equals("gpu") || count * Math.max(1, pairs.size()) >= 1000000)) {
      try {
        List<Endpoint> endpoints = new ArrayList<>();
        for (List<Endpoint> list : byCamera) endpoints.addAll(list);
        int[][] pairImages = new int[pairs.size()][2];
        for (int i = 0; i < pairs.size(); i++)
          pairImages[i] = new int[] {pairs.get(i).a, pairs.get(i).b};
        gpu =
            new GpuFusion(
                cameras,
                widths,
                heights,
                endpoints,
                pairImages,
                options,
                requiredViews(),
                1L << 30,
                cameraEvidence,
                options.fusionCameraCulling
                    ? new FusionScope(cameras, widths, heights, pairImages, pairBounds, options.fusionRelative) : null);
      } catch (RuntimeException e) {
        if (compute.equals("gpu"))
          throw new IllegalArgumentException("GPU fusion required: " + e.getMessage(), e);
        System.out.println("Fusion GPU unavailable; using CPU: " + e.getMessage());
      }
    }
    releaseReader();
    System.out.println(
        gpu == null
            ? "Fusion backend: CPU (indexed camera evidence)"
            : "Fusion backend: OpenCL GPU (FP64 depth lookup, consensus and refinement)");
  }

  int pipelineCapacity() {
    return options.fusionBatchSize;
  }

  CameraEmission.Key hypothesisKey(int source, double[] point) {
    var pair = pairs.get(source);
    int owner = Math.min(pair.a, pair.b);
    if (options.fusionAssociation.equals("camera")) {
      // Geometry-only deterministic owner, independent of the producing pair or completion order.
      // This is an index, not evidence that this camera observed the surface.
      for (int image = 0; image < cameras.length; image++) {
        if (!cameras[image].registered) continue;
        double[] q = cameras[image].view(point), p = cameras[image].projectView(q);
        if (p != null && Double.isFinite(p[0] + p[1]) && q[2] > 0
            && p[0] >= 0 && p[1] >= 0 && p[0] < widths[image] && p[1] < heights[image]) {
          owner = image; break;
        }
      }
    }
    double[] v = cameras[owner].view(point), uv = cameras[owner].projectView(v);
    if (uv == null || uv[0] < 0 || uv[1] < 0 || uv[0] >= widths[owner] || uv[1] >= heights[owner])
      return new CameraEmission.Key(-1, 0, Double.NaN);
    return new CameraEmission.Key(
        owner, (long) Math.floor(uv[1]) * widths[owner] + (long) Math.floor(uv[0]), v[2]);
  }

  long producingPairKey(int source) {
    var p = pairs.get(source);
    return ((long) p.a << 32) | (p.b & 0xffffffffL);
  }

  boolean sameHypothesis(int sourceA, double[] a, int sourceB, double[] b) {
    var pa = pairs.get(sourceA);
    var pb = pairs.get(sourceB);
    for (int image : new int[] {pa.a, pa.b, pb.a, pb.b}) {
      double[] av = cameras[image].view(a), bv = cameras[image].view(b);
      if (!(av[2] > 0 && bv[2] > 0)
          || Math.abs(av[2] - bv[2])
              > Math.min(.0025, options.fusionRelative / 4) * Math.min(av[2], bv[2])) return false;
      double[] au = cameras[image].projectView(av), bu = cameras[image].projectView(bv);
      if (au == null || bu == null || Math.hypot(au[0] - bu[0], au[1] - bu[1])
          > (options.fusionAssociation.equals("camera") ? .75 : .25)) return false;
    }
    return true;
  }

  private void installCameraEvidence() throws IOException {
    List<DepthRaster> old = new ArrayList<>();
    for (PairDepth p : pairs) {
      old.add(p.first);
      old.add(p.second);
    }
    for (int image = 0; image < byCamera.size(); image++) {
      var endpoints = byCamera.get(image);
      for (int rank = 0; rank < endpoints.size(); rank++) {
        Endpoint endpoint = endpoints.get(rank);
        DepthRaster view = new DepthRaster(cameraEvidence.images[image], rank);
        endpoints.set(rank, new Endpoint(endpoint.pair, image, view));
        var pair = pairs.get(endpoint.pair);
        if (pair.a == image) pair.first = view;
        else pair.second = view;
      }
    }
    IOException failure = null;
    for (DepthRaster map : old)
      try {
        map.close();
        if (!(options.diagnostics && options.storage.equals("debug"))) map.discard();
      } catch (IOException e) {
        if (failure == null) failure = e;
        else failure.addSuppressed(e);
      }
    if (failure != null) throw failure;
  }

  void parallel(int count, IntConsumer action) {
    if (workers == null || count < 256) {
      for (int i = 0; i < count; i++) action.accept(i);
      return;
    }
    List<Future<?>> jobs = new ArrayList<>();
    int chunk =
        Math.max(64, (count + Math.max(1, cpuWorkers) * 4 - 1) / (Math.max(1, cpuWorkers) * 4));
    for (int start = 0; start < count; start += chunk) {
      final int a = start, b = Math.min(count, start + chunk);
      jobs.add(
          workers.submit(
              () -> {
                for (int i = a; i < b; i++) action.accept(i);
              }));
    }
    for (Future<?> job : jobs)
      try {
        job.get();
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
        throw new IllegalStateException("Fusion interrupted", e);
      } catch (ExecutionException e) {
        if (e.getCause() instanceof RuntimeException r) throw r;
        throw new IllegalStateException(e.getCause());
      }
  }

  private record Scheduled(int index, int source, long tile) {}

  void releaseReader() {
    pagePool.releaseReader();
  }

  private void spatial(
      List<Integer> indices, int[] sources, double[][] points, IntConsumer action) {
    List<Scheduled> schedule = new ArrayList<>(indices.size());
    for (int i : indices) {
      int camera = pairs.get(sources[i]).a;
      double[] uv = cameras[camera].project(points[i]);
      long tile =
          uv == null
                  || !Double.isFinite(uv[0] + uv[1])
                  || uv[0] < 0
                  || uv[1] < 0
                  || uv[0] >= widths[camera]
                  || uv[1] >= heights[camera]
              ? Long.MAX_VALUE
              : (long) (uv[1] / 16) * ((widths[camera] + 15L) / 16) + (long) (uv[0] / 16);
      schedule.add(new Scheduled(i, sources[i], tile));
    }
    schedule.sort(
        Comparator.comparingInt(Scheduled::source)
            .thenComparingLong(Scheduled::tile)
            .thenComparingInt(Scheduled::index));
    parallel(schedule.size(), j -> action.accept(schedule.get(j).index));
  }

  Result[] fuseBatch(int[] source, double[][] points, int count) {
    return batch(source, points, count, options.depthRefinement, true);
  }

  SurfaceSample[] surfaceBatch(int[] source, Result[] results, int count) {
    long before = reads();
    SurfaceSample[] samples = new SurfaceSample[count];
    parallel(
        count,
        i -> {
          if (results[i].accepted()) samples[i] = surfaceSample(source[i], results[i]);
        });
    normalReads += reads() - before;
    return samples;
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
        long readStarted = reads();
        List<Integer> guardedIndices = new ArrayList<>();
        for (int i = 0; i < count; i++) if (results[i] == null) guardedIndices.add(i);
        guardedCandidates += guardedIndices.size();
        final Result[] guarded = results;
        spatial(
            guardedIndices,
            source,
            points,
            i -> {
              if (guarded[i] == null)
                guarded[i] = visibility(source[i], fuse(source[i], points[i], refinement));
            });
        guardNanos += System.nanoTime() - clock;
        guardReads += reads() - readStarted;
        clock = System.nanoTime();
        readStarted = reads();
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
        final Result[] audited = results;
        spatial(
            auditIndices,
            source,
            points,
            i -> {
              Result cpu = visibility(source[i], fuse(source[i], points[i], refinement));
              if (!equivalent(cpu, audited[i]))
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
                        + audited[i].status
                        + "/"
                        + audited[i].views
                        + "/"
                        + audited[i].independentPairs
                        + " "
                        + Arrays.toString(audited[i].point)
                        + " sigma="
                        + audited[i].relativeSigma);
            });
        gpuCandidates += count;
        auditNanos += System.nanoTime() - clock;
        auditReads += reads() - readStarted;
      } catch (RuntimeException e) {
        // B2: every fallback is reported; policy "batch" keeps the GPU for later batches.
        consecutiveGpuFailures++;
        boolean keep = options.fusionGpuFailure.equals("batch") && consecutiveGpuFailures < 3;
        gpuFailures.add(
            Json.object(
                "candidates_in_batch", count,
                "batch_index_after_gpu_candidates", gpuCandidates,
                "error", String.valueOf(e.getMessage()),
                "type", e.getClass().getSimpleName(),
                "gpu_kept_for_later_batches", keep));
        RunTelemetry.recordTotal("GPU fusion fallback (" + e.getClass().getSimpleName() + ")", 0, count);
        if (!keep) {
          gpu.close();
          gpu = null;
        }
        String compute =
            options.fusionCompute.equals("inherit") ? options.compute : options.fusionCompute;
        if (compute.equals("gpu") || (!refinement && options.surfaceValidation.equals("gpu")))
          throw new IllegalArgumentException("GPU fusion required: " + e.getMessage(), e);
        System.out.println(
            "WARNING: Fusion GPU failed; replaying this batch on CPU"
                + (keep ? " and retrying the GPU next batch: " : "; GPU disabled for the rest of the run: ")
                + e.getMessage());
        results = null;
      }
      if (results != null) consecutiveGpuFailures = 0;
    }
    if (results == null) {
      long clock = System.nanoTime();
      long readStarted = reads();
      results = new Result[count];
      final Result[] cpu = results;
      parallel(count, i -> cpu[i] = visibility(source[i], fuse(source[i], points[i], refinement)));
      cpuCandidates += count;
      cpuNanos += System.nanoTime() - clock;
      cpuReads += reads() - readStarted;
    }
    return results;
  }

  static boolean equivalent(Result a, Result b) {
    if (a.supportingCameras != null
        && b.supportingCameras != null
        && !a.supportingCameras.equals(b.supportingCameras)) return false;
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
        "edge_aware_camera_checks",DepthOutlierFilter.enabled(options)?Json.object(
            "version",3,"positive_support","Precise center measurements from distinct partners; uncertainty compatibility is neutral",
            "supported_checks",edgeSupported.sum(),"free_space_rejected_checks",edgeFreeSpace.sum(),
            "unknown_or_occluded_checks",edgeUnknown.sum(),"surface_patch_suspects",patchSuspects.sum(),
            "surface_patch_without_precise_support_rejected",patchUnsupported.sum(),
            "surface_patch_free_space_votes",patchFreeSpace.sum()):null,
        "gpu_fallbacks", gpuFailures.size(),
        "gpu_fallback_details", new ArrayList<>(gpuFailures),
        "gpu_fallback_policy", options.fusionGpuFailure,
        "fusion_camera_culling", options.fusionCameraCulling,
        "fusion_association", options.fusionAssociation,
        "fusion_mode",
        options.fusionMode,
        "camera_outlier_model",
        options.fusionOutliers,
        "gpu_evidence_policy",
        options.fusionGpuEvidence,
        "memory_admission",
        memoryAdmission,
        "packed_heap_allocation_estimate_bytes",
        cameraEvidence == null ? null : cameraEvidence.allocationBytes,
        "gpu_candidates",
        gpuCandidates,
        "evidence_layout",
        cameraEvidence == null ? "pair_maps" : "camera_sparse_layers",
        "evidence_layout_reason",
        evidenceLayoutReason,
        "raw_pair_evidence_bytes",
        rawEvidenceBytes,
        "packed_evidence_bytes",
        cameraEvidence == null ? null : cameraEvidence.bytes,
        "packed_measurements",
        cameraEvidence == null ? null : cameraEvidence.measurements,
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
        pagePool.reads + pagePool.frozenReads.sum(),
        "depth_bulk_read_bytes",
        pagePool.bulkReadBytes.sum(),
        "depth_bulk_read_calls",
        pagePool.bulkReadCalls.sum(),
        "depth_page_writes",
        pagePool.writes,
        "depth_page_evictions",
        pagePool.evictions + pagePool.frozenEvictions.sum(),
        "cpu_resident_evidence_bytes",
        residentEvidenceBytes,
        "evidence_freeze_nanoseconds",
        freezeNanos,
        "cpu_workers",
        workers == null ? 1 : cpuWorkers,
        "depth_page_bytes",
        4L * options.fusionPageSide * options.fusionPageSide,
        "depth_file_opens",
        pagePool.fileOpens + pagePool.pinnedFileOpens,
        "peak_pinned_read_files",
        pagePool.peakPinnedFiles,
        "guard_depth_reads",
        guardReads,
        "audit_depth_reads",
        auditReads,
        "cpu_depth_reads",
        cpuReads,
        "normal_depth_reads",
        normalReads,
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
