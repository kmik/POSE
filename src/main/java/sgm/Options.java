package sgm;

import java.nio.file.*;
import java.util.*;

/**
 * Strict, standalone CLI. It deliberately does not feed image inputs into LAS.utils' LAS parser.
 */
public final class Options {
  public Path input, output = Paths.get("photogrammetry-output"), calibration, poses;
  public boolean help, rectified, calibrate, overwrite, gps, noGps, copyInputs, verbose;
  public boolean diagnostics, compatibilityCopies, exportDisparityImages;
  public String densePairSelection = "all", calibrationPipeline = "bootstrap";
  public String fusionEmission = "pair";
  public String fusionEvidence = "auto";
  public String fusionGpuEvidence = "auto";
  public String windowedMode = "off";
  public String depthOutlierFilter = "off";
  public int depthOutlierRadius = 3, depthOutlierEdgeThreshold = 24;
  public double depthOutlierDeviation = 1.5;
  public String densePipeline = "standard";
  public int aerialTileSize = 512, aerialOverlap = 32, aerialCoarseDimension = 1024,
      aerialDisparityMargin = 16;
  public String fusionMode = "reference", fusionOutliers = "reference";
  public String fusionAssociation = "pair";
  public boolean fusionCameraCulling;
  public boolean stereoPipeline;
  public int fusionWindowMb;
  public int densePairNeighbors = 3;
  // Image-fusion decoded candidate cache; -1 = bounded auto, 0 = reference replay.
  public int imageFusionCacheMb = -1;
  // MVSGM image-fusion windows by default; auto selects row strips only when the heap needs them.
  public String streamedProcessing = "auto";
  public Path exportConfidence;
  public boolean disparitiesExplicit, minDisparityExplicit, selfCalibration = true;
  // --quick-calibration on|off: EXIF/XMP focal initialization (joint pipeline, no bootstrap) and
  // one GPS focal start. Default on since 2026-11-17 (the bootstrap focal start is the weaker one);
  // runs saved before that continue with off (legacyQuickCalibration).
  public boolean quickCalibration; // on by default in parse(); off for programmatic Options

  /**
   * Saved run arguments with the quick-calibration choice made explicit: new runs save it (Main
   * appends the resolved value), so arguments without any --quick-calibration /
   * --no-quick-calibration come from a run started before quick calibration became the default,
   * and continue with it off, as they started.
   */
  static List<String> legacyQuickCalibration(List<String> saved) {
    if (namesQuickCalibration(saved)) return saved;
    List<String> out = new ArrayList<>(saved);
    out.add("--no-quick-calibration");
    return out;
  }

  /** The arguments give --quick-calibration (with or without a value) or --no-quick-calibration. */
  static boolean namesQuickCalibration(List<String> args) {
    for (String a : args)
      if (a.equals("--quick-calibration") || a.startsWith("--quick-calibration=") || a.equals("--no-quick-calibration")) return true;
    return false;
  }

  /** The resolved quick-calibration choice as an argument, saved with a new run. */
  String quickCalibrationArgument() {
    return quickCalibration ? "--quick-calibration" : "--no-quick-calibration";
  }
  public boolean mvsgmOptimalNeighbors;
  // --ply off / --point-confidence off: skip writing dense.ply / 07_results/point-confidence.bin.
  public boolean writePly = true, writePointConfidence = true;
  // --tiled-las-output SIZE: square LAS tiles of SIZE output units (07_results/dense_tiles) instead
  // of dense.las; 0 = one dense.las (LAS 1.4 automatically above 4,294,967,295 points).
  public double tiledLasOutput = 0;
  // --processed-image-cache auto|on|off: decoded processed images kept on disk for reuse
  // (ProcessedImageCache). denseOptions: the full options behind an alignment copy.
  public String processedImageCache = "auto";
  // --fusion-gpu-normals on: image-fusion normals and surface attributes on the GPU (GpuNormals;
  // last-bit differences from the CPU). Default off.
  public boolean fusionGpuNormals;
  Options denseOptions;
  Path processedImageCacheDir;
  Map<String, Camera> calibrationPoses = Collections.emptyMap();
  public String crs = "EPSG:3067";
  public double gpsThreshold = 10;
  // G1: final bundle adjustment with GPS camera centres as weighted priors (off|prior) and their sigma.
  public String gpsBundle = "prior";
  // G3-G5 inside the GPS-weighted adjustment: shared lens refit (+ focal prior), GPS time offset and
  // lever arm, rolling-shutter readout. estimate | off | fixed value.
  public String gpsIntrinsics = "f,k1,k2,k3,cx,cy,p1,p2", gpsFocalPrior = "auto";
  public double gpsFocalSigma = .03;
  public String gpsTimeOffset = "estimate", gpsLeverArm = "estimate", rollingShutter = "estimate";
  public int gpsAdjustmentRounds = 3;
  // Final bundles converge automatically; model-selection proposals have a separate hard cap.
  public boolean bundleIterationsExplicit;
  // --bundle-solver legacy|fast and its switches (BundleAdjuster). With every switch at its
  // legacy value (the default) bundle results are byte-identical to the previous solver; fast
  // sets each switch that was not given explicitly to its fast value (resolved after parsing).
  public String bundleSolver = "legacy";
  /** Largest reduced system for the parallel dense Cholesky; -1 = legacy direct solve (768, OpenCV). */
  public int bundleDirectLimit = -1;
  /** Preconditioner of the iterative (PCG) bundle solve: block-jacobi (legacy) or cluster-jacobi. */
  public String bundlePreconditioner = "block-jacobi";
  /** Adaptive linear tolerance (forcing sequence) instead of the fixed 1e-6. */
  public boolean bundleInexactSteps;
  /** Local bundles of each new camera and its neighbours, global bundles at twice the growth. */
  public boolean registrationLocalBundles;
  /** Shared-lens fits: objective (legacy) or also stop once parameter changes are negligible. */
  public String lensFitStop = "objective";

  /** Fast-path values of the --bundle-solver switches. */
  static final int FAST_BUNDLE_DIRECT_LIMIT = 6000;

  /** Every bundle switch at its legacy value: results byte-identical to the previous solver. */
  boolean bundleSolverLegacy() {
    return bundleDirectLimit < 0
        && bundlePreconditioner.equals("block-jacobi")
        && !bundleInexactSteps
        && !registrationLocalBundles
        && lensFitStop.equals("objective");
  }

  /** The non-legacy bundle switches as a short key (cache keys, reports); "" when legacy. */
  String bundleSolverKey() {
    if (bundleSolverLegacy()) return "";
    return "bundleSolver=direct:" + bundleDirectLimit + ",pre:" + bundlePreconditioner
        + ",inexact:" + bundleInexactSteps + ",local:" + registrationLocalBundles
        + ",lensStop:" + lensFitStop;
  }
  // C1: ground classification of the final cloud (opt-in) and its tuning.
  public boolean classifyGround;
  // GS1: correct each stereo pair's vertical ground offset; GS2: merge ground copies into one sheet.
  // Both imply --classify-ground.
  public boolean groundPairCorrection, groundAwareFusion;
  // Sheet fusion radius for --ground-aware-fusion (0 = auto: twice the terrain cell).
  public double groundFusionRadius = 0;
  public double groundCell = 0, groundMaxSlope = 35, groundMaxObject = 30;
  // Ground control points: CSV (gcp_id,x,y,z[,sx,sy,sz][,type]), optional markers to import,
  // default sigmas (m) and the marker editor port.
  public Path gcp, gcpMarkers;
  public double gcpSigma = .02, gcpHeightSigma = .03;
  public int gcpPort = 8765;
  public String gcpCheck = "none";
  public String gcpCheckIds = "";
  // Lidar-referenced georeferencing (step 'lidar', LidarReference): ALS LAS/LAZ file or folder,
  // scale on/off, search ranges, surface cell, canopy weight, residual limit, weak limit, margin,
  // declared CRS / vertical datum for files that do not state them.
  public Path lidarReference;
  public boolean lidarScale;
  public double lidarSearchShift = 10, lidarSearchAngle = 3, lidarCell = 2, lidarCanopyWeight = .1,
      lidarOutlier = 2, lidarWeakLimit = .25, lidarMargin = 10, lidarChangeLimit = 1;
  public String lidarCrs, lidarVerticalDatum;
  /** Vertical datums of camera GPS heights (null: ellipsoid); see VerticalDatum. */
  public String sourceGeoid, targetGeoid;
  // Height sigma of the GPS camera prior: auto (0.2 m for DJI barometric heights, else gpsSigma).
  public String gpsHeightSigma = "auto";
  double gpsHeightSigmaValue = 2;
  // Fitted readout time (s) of the last GPS-weighted adjustment; reporting only.
  double rollingShutterReadout;
  // T1: off | report (default) | exclude flagged cameras from dense matching.
  public String cameraScreening = "report";
  // Tie-point cleanup step (TiePointCleanup): off, auto (automatic limits), custom (limits below)
  // or pause (choose in POSE). tiePointLimitsGiven: a limit was given on the command line.
  public String tiePointCleanup = "off";
  public double tiePointMaxError = 0, tiePointMinAngle = 1, tiePointMaxRemoved = 20;
  public int tiePointMinViews = 2;
  public boolean tiePointLimitsGiven;
  // S1: stereo pairs computed concurrently; 1 = serial (default), 0 = auto.
  public int stereoPairsInFlight = 0; // 0 = auto (one per 3 cores, at most 6, memory-checked); output equals serial
  public double gpsSigma = 2;
  public double focal = Double.NaN, sensorWidth = Double.NaN, baseline = 1, scale = 1;
  public double p1 = 4, p2 = 24, ratio = .75, reprojection = 2, uniqueness = .03, voxel = 0;
  public int sgmEdgeScale = 32;
  // Opt-in depth-edge handling; 0 keeps the previous behaviour exactly.
  public int disparityEdgeErosion = 0, censusEdgeMask = 0, censusShift = 0;
  public double disparityEdgeJump = 2.0;
  public String medianMode = "edge-preserving";
  public double medianMaxChange = .5;
  public int medianEdgeThreshold = 24;
  public int speckleConnectivity = 8;
  public int minDisparity = 0, disparities = 128, census = 5, median = 3, directions = 8;
  public int maxDimension = 1600, features = 8000, minMatches = 20, neighbors = 3, iterations = 8;
  // Image-fusion agreement neighbourhood in pixels; -1 = auto (0 for owner, else max(1, step)).
  public int fusionAgreementRadius = -1;
  // Image-fusion surface strategy: owner (default, previous rule), owner-consume, graph-average.
  public String fusionStrategy = "owner";
  // Image fusion: drop per-camera depth outliers far beyond the scene's depth range (T2).
  public boolean fusionDepthOutliers = true;
  // Image fusion impulse filter on merged depth maps (salt-and-pepper spikes on smooth surfaces).
  public boolean fusionSpikeFilter = true;
  public String fusionFreeSpace = "on";
  // --thin-surfaces (image fusion; THIN_SURFACES_FUSION.md): uncertainty-scaled agreement, precise-
  // disagreement rejection, best-first seeding, narrow pairs as support, local sigma limit.
  public String fusionTolerance = "relative", fusionSeedOrder = "camera";
  public boolean fusionPrecisionCheck = false;
  // Exact GPU brute-force Hamming k=2 search for binary feature descriptors (GpuHamming).
  public boolean gpuFeatureMatching = true;
  public boolean gpuQueuedUploads = true;
  public boolean gpuPackedSummaries;
  public int calibrationTrialIterations = 12;
  // --calibration-trial-method full|subset|joint and --calibration-trial-cameras N (0 = auto):
  // see SharedCalibration.refine and CalibrationSubset.
  public String calibrationTrialMethod = "full";
  // --calibration-acceptance strict|relaxed (alignment lens only): relaxed accepts a fit that
  // leaves both held-out groups no worse and moves each fitted parameter by >= 3 sigma.
  public String calibrationAcceptance = "strict";
  public int calibrationTrialCameras;
  public double fusionSigmaK = 3, fusionSeedMinAngle = 0, fusionLocalSigma = 0;
  public int step = 2, cores = Math.max(1, Runtime.getRuntime().availableProcessors() / 2);
  public int boardColumns = 9, boardRows = 7;
  public double squareSize = .02, maxDepth = 10000;
  public long memoryMb = 2048;
  public long ramLimitMb; // optional total host-RAM planning ceiling, separate from processing buffers
  public String format = "jpg,jpeg,png,tif,tiff,bmp";
  public String seedPair;
  public String censusMode = "gray";
  public String matchingCost = "census", gpuSgm = "auto", sgmAggregate = "auto";
  public String gpuAggregate = "auto", imageReader = "auto";
  public int imageWindow = 1024, descriptorMemoryMb = 256, imageMemoryMb = 0;
  public int esgmCandidates = 2;
  long gpuHostReserve;
  PointConfidence.Selection stereoSelection;
  public String subpixelRefinement = "none";
  public String disparityRefinement = "off";
  public double disparityAnchorAngle = 4;
  public int disparityRefinementMemoryMb = 128;
  public String pairwiseSearch = "full", subpixelFit = "parabola", onPairFailure = "abort";
  public String fusionGpuFailure = "disable";
  public int tsgmLevels = 3, tsgmMargin = 3;
  /** Packed (x, y, disparity) sparse range samples for the current pair; tsgm band seeds. */
  double[] tsgmSeeds;
  /** Internal: lets small full-range SGM solves (the range guard) use the GPU under --compute auto. */
  boolean gpuSmallSolves;
  /** --help TEXT: print only the help entries that mention TEXT (not a processing option). */
  String helpTopic;
  /** --block-processing off|auto|on, --block-images N (0 = forecast), --block-overlap F (footprints). */
  String blockProcessing = "off";
  int blockImages;
  double blockOverlap = 1;
  /** --dense-method mvsgm settings (MULTIVIEW_SGM.md); recorded in 05_stereo/mvsgm.json. */
  int mvsgmNeighbors = 6, mvsgmBestK = 3, mvsgmCensus = 5, mvsgmLevels = -1, mvsgmConsistentViews = 1;
  double mvsgmConsistencyLabels = 1;
  /** --mvsgm-cameras-in-flight: cameras whose depth maps are computed at once (0 = auto; 1 = serial). */
  int mvsgmCamerasInFlight = 1;
  /** --mvsgm-tile auto|off|N: tile size for mvsgm depth maps (-1 = auto: 2048 above 16 Mpx; 0 = untiled). */
  int mvsgmTile = -1;
  /** --dense-checkpoint off|auto|MINUTES: resumable mvsgm overlap-window dense step (DenseCheckpoint). */
  public String denseCheckpoint = "auto";
  /** The open checkpoint of the running dense step (not an option). */
  transient DenseCheckpoint checkpoint;
  /** Opt-in shared accounting allowance for overlap-window host work; zero preserves legacy. */
  public long fusionHostMemoryMb;
  public int fusionHostCacheMb = 512;
  /** Independent source-map lifetime opt-in; does not change fusion memory planning. */
  public String fusionSourceMaps = "retained";
  public double censusWeight = .5;
  public int adTruncation = 30, gradientTruncation = 60, gpuStripRows = 64;
  public int gpuBankMb = 1024;
  public double minForwardOverlap = 0, minSideOverlap = 0, minPairOverlap = 0;
  public double overlapGroundZ = Double.NaN, flightHeading = Double.NaN;
  public String overlapUnknown = "keep";
  public Path orientationInput, orientationOutput, exportOrientations, orientationImageList;
  public String orientationInputFormat = "auto", calibrationFormat = "auto", posesFormat = "auto";
  public String orientationOutputFormat = "none",
      interiorOutputFormat = "none",
      exteriorOutputFormat = "none",
      orientationPixels = "source";
  public String orientationRotation = "auto",
      orientationAxes = "auto",
      orientationAngleUnit = "degrees",
      orientationCrs;
  public String orientationUnits = "auto";
  String orientationSourceUnits = "project_units";
  List<String> orientationNotes = List.of();
  public double orientationPixelSize = Double.NaN;
  public boolean orientationAllowLossy;
  public int orientationComponent = 0;
  Map<String, Calibration> orientationCalibrations = Collections.emptyMap();
  // Camera groups (CameraGroups): --camera-grouping auto|single|per-image, --camera-groups FILE
  // (CSV image,camera), and the per-camera forms CAMERA=VALUE of --intrinsics-mode, --calibration,
  // --orientation-pixel-size, --focal and --s_w.
  public String cameraGrouping = "auto";
  public Path cameraGroupsFile;
  public Map<String, String> intrinsicsModes = Map.of();
  public Map<String, Path> calibrationFiles = Map.of();
  public Map<String, Double> orientationPixelSizes = Map.of(), focalByCamera = Map.of(),
      sensorWidthByCamera = Map.of();
  CameraGroups cameraGroups;
  // Camera ID per image and pixel size (mm) per camera ID of an imported orientation.
  Map<String, String> importedCameraIds = Map.of();
  Map<String, Double> importedPixelSizes = Map.of();
  // Every imported per-image calibration (orientationCalibrations stays empty when they are all equal).
  Map<String, Calibration> importedCalibrations = Map.of();
  Path stereoReportDirectory;
  public Path analyzeRun, analysisOutput;
  public int analysisSamples = 500000;
  public double analysisCellSize = 0, analysisTolerance = 0;
  public String analysisRegion;
  public boolean analysisConfidence;
  public double correlation = .55, lrThreshold = .75, speckleRange = 1.0, minAngle = 1.0;
  public double fusionRelative = .005, fusionPixels = .75;
  public double fusionConsensus = .6, maxDepthUncertainty = .1;
  public double confidenceThreshold = 0;
  public double twoViewConfidenceThreshold = Double.NaN;
  public String denseMethod = "sgm";
  public int patchmatchIterations = 4, patchmatchViews = 4, patchmatchLevels = 3;
  public double patchmatchEdgeSigma = 32;
  public double patchmatchMinDepth = 0, patchmatchMaxDepth = 0;
  // 0 = auto. Resource option: parallel references never change a map.
  public int patchmatchReferencesInFlight = 0;
  // -1 auto (tile only where the untiled solve does not fit), 0 off, N = tile core size.
  public int patchmatchTile = -1;
  // -1 auto (the exact dependency radius, at most 256), else pixels.
  public int patchmatchTileHalo = -1;
  // Plane-consistent depth averaging radius of the final maps (0 = off, the original maps).
  public int patchmatchSmooth = 0;
  public double patchmatchSmoothTolerance = .01;
  // --patchmatch-view-cost: 1 = best view (+ second-view term), K = mean of the K best views.
  public int patchmatchTopViews = 1;
  public String patchmatchNeighbors = "nearest";
  // Source views (best one included) whose maps must confirm a depth; 1 = the best view only.
  public int patchmatchConsistentViews = 1;
  // Minimum depth-continuous region of a final PatchMatch map in pixels (0 = off).
  public int patchmatchSpeckle = 0;
  public double patchmatchSpeckleTolerance = .01;
  public boolean depthRefinement = true;
  public String depthRefinementMethod = "reprojection", surfaceConsolidation = "normal";
  public double surfaceRadius = 2, surfaceSpacing = 1, surfaceMaxShift = 2;
  /** Opt-in --surface-flatten window in footprints; 0 keeps consolidation unchanged. */
  public double surfaceFlatten = 0;
  public double surfaceNormalAngle = 30;
  public int surfaceMemoryMb = 128, surfaceBatchSize = 64;
  public String surfaceValidation = "auto", pointAttributes = "surface";

  /** The --point-attributes fields (empty for none). */
  java.util.EnumSet<PointFields.Field> pointFields() {
    return PointFields.parse(pointAttributes);
  }

  /** Whether the surface neighbourhood attributes (normals, roughness ...) are computed for output. */
  boolean surfaceAttributes() {
    return PointFields.needsSurface(pointFields());
  }

  /** Whether fusion records the per-point evidence for angle, depth spread and calibrated sigmas. */
  boolean pointEvidence() {
    var f = pointFields();
    return f.contains(PointFields.Field.HEIGHT_SIGMA) || f.contains(PointFields.Field.RAY_SIGMA)
        || f.contains(PointFields.Field.DEPTH_SPREAD) || f.contains(PointFields.Field.ANGLE);
  }
  public int correlationWindow = 7, speckleSize = 50, minViews = 3;
  public String filtering = "moderate", filteringOverrides = "";
  public String quality = "custom";
  public boolean maxDimensionExplicit, estimateMemory;
  public String matchingAlgorithm = "sift",
      descriptorMatcher = "auto",
      pairSelection = "auto",
      pairMatching = "parallel";
  public String alignmentQuality = "same", intrinsicsMode = "fixed", extrinsicsMode = "fixed";
  public String refineIntrinsics = "f,k1,k2,k3,cx,cy,p1,p2", rerunFrom = "none";
  public int alignmentMaxDimension = 0, pairCandidates = 6, tileSize = 512, tileOverlap = 32;
  public int pyramidLevels = 1, pyramidRadius = 16, cleanupPasses = 1, trackMinViews = 2;
  public double focalSigma = .05, radialSigma = .05, positionSigma = 10, rotationSigma = 10;
  public double trackMaxError = 4, trackMinAngle = .1;
  /** Joint bundle during incremental registration each time the model grew by this percent. */
  public double registrationBundleGrowth = 15;
  /** New sparse points: drop observations that disagree, or reject the whole track. */
  public String triangulationOutliers = "drop";
  /** Cells along the longer image side for spread feature selection; 0 = strongest N overall. */
  public int featureGrid = 0;
  /** Unregistered images after alignment: off, or further models merged into the main one. */
  public String registrationRecovery = "off";
  /** After matching: match weakly connected images against more candidates. */
  public String matchRepair = "off";
  public int matchRepairCandidates = 8, matchRepairMinPairs = 2;
  /** Pair selection: also match each image with its N nearest GPS neighbours (0 = off). */
  public int gpsNeighbors = 0;
  /** Incremental registration: reject PnP poses far from GPS or tilted beyond the block. */
  public String registrationPoseCheck = "off";
  boolean alignmentCalibration, alignmentCalibrationDone;
  public boolean resume, tiled, diskDepth = true;
  public String storage = "standard", compute = "auto";
  public String fusionCompute = "inherit";
  public String fusionWeightMath = "direct";
  public String fusionLayout = "coalesced";
  public boolean fusionPipeline = true;
  public boolean fusionMeasuredReprojection = true;
  public int outputCompression = 6;
  public String surfacePartition = "routed", processingReport = "pdf", reportDsmStatistic = "max";
  // PDF layout: summary (2 pages, default) or full (the earlier detailed report).
  public String reportDetail = "summary";
  public int fusionPageSide = 64, fusionCacheMb = 0, fusionBatchSize = 16384;
  public int reportDsmMaxDimension = 1024, reportDsmMinPoints = 1;
  public double reportDsmResolution = 0;
  public boolean reportLocalDsm;
  public Path exportReport, reportOutput;
  // Full-resolution true orthomosaic (ORTHOMOSAIC.md): off by default.
  public boolean orthomosaic, orthomosaicStrict;
  public double orthomosaicResolution = 0, orthomosaicSurfaceCell = 0, orthomosaicFill = -1;
  // --orthomosaic-mode true|classic|mosaic; classic: smoothed, fully filled surface, no visibility test;
  // mosaic: the same surface, contiguous photograph regions with optimised seams (OrthoSeams).
  public String orthomosaicMode = "true";
  public double orthomosaicSmooth = -1, orthomosaicFeather = 16;
  /** --orthomosaic-max-angle DEG (mosaic): views steeper than this from the vertical are not used; 0 = no limit. */
  public double orthomosaicMaxAngle = 0;
  public Path exportOrthomosaic, orthomosaicOutput;
  public boolean cache, listGpus;
  public int gpuDevice;
  public long gpuMemoryMb;
  public String rectificationAlpha = "auto";
  public double siftContrast = .04,
      siftEdgeThreshold = 10,
      akazeThreshold = .001,
      kazeThreshold = .001;
  public int orbFastThreshold = 20, briskThreshold = 30;
  java.nio.file.Path cacheDirectory;
  OutputDiskSpace outputDiskSpace;
  java.nio.file.Path projectDirectory;
  Path runFeatureDirectory;
  java.util.concurrent.atomic.AtomicLong runFeatureHits =
      new java.util.concurrent.atomic.AtomicLong();

  public static Options parse(String[] args) {
    args = Presets.expand(args);
    Options o = new Options();
    o.quickCalibration = true; // the command-line default
    o.help = args.length == 0;
    Set<String> explicitFiltering = new TreeSet<>();
    boolean cacheDisabled = false;
    boolean storageExplicit = false;
    boolean quickExplicit = false, pipelineExplicit = false, stepExplicit = false;
    Set<String> bundleExplicit = new HashSet<>();
    for (int i = 0; i < args.length; i++) {
      String key = args[i], inline = null;
      int eq = key.indexOf('=');
      if (eq > 0) {
        inline = key.substring(eq + 1);
        key = key.substring(0, eq);
      }
      switch (key) {
        case "--mvsgm-optimal-neighbors":
          o.mvsgmOptimalNeighbors = true;
          continue;
        case "--no-mvsgm-optimal-neighbors":
          o.mvsgmOptimalNeighbors = false;
          continue;
        case "--diagnostics":
          o.diagnostics = true;
          continue;
        case "--export-disparity-images":
          o.exportDisparityImages = true;
          continue;
        case "--compatibility-copies":
          o.compatibilityCopies = true;
          continue;
        case "--orthomosaic":
          o.orthomosaic = true;
          continue;
        case "--orthomosaic-strict":
          o.orthomosaicStrict = true;
          continue;
        case "--report-local-dsm":
          o.reportLocalDsm = true;
          continue;
        case "--legacy-fusion-reprojection":
          o.fusionMeasuredReprojection = false;
          continue;
        case "--analysis-confidence":
          o.analysisConfidence = true;
          continue;
        case "--no-fusion-pipeline":
          o.fusionPipeline = false;
          continue;
        case "--orientation-allow-lossy":
          o.orientationAllowLossy = true;
          continue;
        case "-h":
        case "--help":
        case "-help":
          o.help = true;
          // --help TEXT: only the option entries mentioning TEXT.
          if (inline != null) o.helpTopic = inline;
          else if (i + 1 < args.length && !args[i + 1].startsWith("-")) o.helpTopic = args[++i];
          continue;
        case "--verbose":
          o.verbose = true;
          continue;
        case "--thin-surfaces":
          // Order matters: options after it override these values.
          o.fusionTolerance = "sigma";
          o.fusionPrecisionCheck = true;
          o.fusionSeedOrder = "precision";
          o.fusionLocalSigma = 3;
          o.surfaceConsolidation = "normal";
          o.surfaceFlatten = 8;
          o.subpixelRefinement = "zncc";
          continue;
        case "--classify-ground":
          o.classifyGround = true;
          continue;
        case "--ground-pair-correction":
          o.groundPairCorrection = true;
          continue;
        case "--ground-aware-fusion":
          o.groundAwareFusion = true;
          continue;
        case "--rectified":
          o.rectified = true;
          continue;
        case "--estimate-memory":
          o.estimateMemory = true;
          continue;
        case "--resume":
          o.resume = true;
          continue;
        case "--cache":
          o.cache = true;
          cacheDisabled = false;
          continue;
        case "--no-cache":
          o.cache = false;
          cacheDisabled = true;
          continue;
        case "--list-gpus":
          o.listGpus = true;
          continue;
        case "--tiled":
          o.tiled = true;
          continue;
        case "--memory-depth":
          o.diskDepth = false;
          continue;
        case "--calib":
        case "-c":
          o.calibrate = true;
          continue;
        case "--overwrite":
          o.overwrite = true;
          continue;
        case "--gps":
          o.gps = true;
          continue;
        case "--no-gps":
          o.noGps = true;
          continue;
        case "--copy-inputs":
          o.copyInputs = true;
          continue;
        case "--no-self-calibration":
          o.selfCalibration = false;
          continue;
        case "--quick-calibration": {
          // Optional value: --quick-calibration, --quick-calibration on|off, --quick-calibration=off.
          String quick = inline != null ? inline
              : i + 1 < args.length && args[i + 1].matches("(?i)on|off|true|false") ? args[++i] : "on";
          quick = quick.toLowerCase(Locale.ROOT);
          choice(quick, "--quick-calibration", "on", "off", "true", "false");
          o.quickCalibration = quick.equals("on") || quick.equals("true");
          quickExplicit = true;
          continue;
        }
        case "--no-quick-calibration":
          o.quickCalibration = false;
          quickExplicit = true;
          continue;
        case "--no-depth-refinement":
          o.depthRefinement = false;
          continue;
        default:
          break;
      }
      String v = inline;
      if (v == null) {
        if (++i >= args.length) throw new IllegalArgumentException("Missing value for " + key);
        v = args[i];
      }
      switch (key) {
        case "--sgm-edge-scale":
          o.sgmEdgeScale = Integer.parseInt(v);
          break;
        case "--disparity-edge-erosion":
          o.disparityEdgeErosion = Integer.parseInt(v);
          break;
        case "--disparity-edge-jump":
          o.disparityEdgeJump = number(v);
          break;
        case "--census-edge-mask":
          o.censusEdgeMask = Integer.parseInt(v);
          break;
        case "--census-shift":
          o.censusShift = Integer.parseInt(v);
          break;
        case "--median-mode":
          o.medianMode = v.toLowerCase(Locale.ROOT);
          break;
        case "--median-max-change":
          o.medianMaxChange = number(v);
          break;
        case "--median-edge-threshold":
          o.medianEdgeThreshold = Integer.parseInt(v);
          break;
        case "--speckle-connectivity":
          o.speckleConnectivity = Integer.parseInt(v);
          break;
        case "--matching-cost":
          o.matchingCost = v.toLowerCase(Locale.ROOT);
          break;
        case "--subpixel-refinement":
          o.subpixelRefinement = v.toLowerCase(Locale.ROOT);
          break;
        case "--pairwise-search":
          o.pairwiseSearch = v.toLowerCase(Locale.ROOT);
          break;
        case "--subpixel-fit":
          o.subpixelFit = v.toLowerCase(Locale.ROOT);
          break;
        case "--on-pair-failure":
          o.onPairFailure = v.toLowerCase(Locale.ROOT);
          break;
        case "--fusion-gpu-failure":
          o.fusionGpuFailure = v.toLowerCase(Locale.ROOT);
          break;
        case "--tsgm-levels":
          o.tsgmLevels = Integer.parseInt(v);
          break;
        case "--tsgm-margin":
          o.tsgmMargin = Integer.parseInt(v);
          break;
        case "--census-weight":
          o.censusWeight = number(v);
          break;
        case "--ad-truncation":
          o.adTruncation = Integer.parseInt(v);
          break;
        case "--gradient-truncation":
          o.gradientTruncation = Integer.parseInt(v);
          break;
        case "--sgm-aggregate":
          o.sgmAggregate = v.toLowerCase(Locale.ROOT);
          break;
        case "--gpu-sgm":
          o.gpuSgm = v.toLowerCase(Locale.ROOT);
          break;
        case "--gpu-aggregate":
          o.gpuAggregate = v.toLowerCase(Locale.ROOT);
          break;
        case "--esgm-candidates":
          o.esgmCandidates = Integer.parseInt(v);
          break;
        case "--image-reader":
          o.imageReader = v.toLowerCase(Locale.ROOT);
          break;
        case "--depth-outlier-filter":
          o.depthOutlierFilter = v.toLowerCase(Locale.ROOT);
          break;
        case "--disparity-refinement":
          o.disparityRefinement = v.toLowerCase(Locale.ROOT);
          break;
        case "--disparity-anchor-angle":
          o.disparityAnchorAngle = number(v);
          break;
        case "--disparity-refinement-memory-mb":
          o.disparityRefinementMemoryMb = Integer.parseInt(v);
          break;
        case "--depth-outlier-radius":
          o.depthOutlierRadius = Integer.parseInt(v);
          break;
        case "--depth-outlier-deviation":
          o.depthOutlierDeviation = number(v);
          break;
        case "--depth-outlier-edge-threshold":
          o.depthOutlierEdgeThreshold = Integer.parseInt(v);
          break;
        case "--dense-pipeline":
          o.densePipeline = v.toLowerCase(Locale.ROOT);
          break;
        case "--aerial-tile-size":
          o.aerialTileSize = Integer.parseInt(v);
          break;
        case "--aerial-overlap":
          o.aerialOverlap = Integer.parseInt(v);
          break;
        case "--aerial-coarse-dimension":
          o.aerialCoarseDimension = Integer.parseInt(v);
          break;
        case "--aerial-disparity-margin":
          o.aerialDisparityMargin = Integer.parseInt(v);
          break;
        case "--windowed-mode":
          o.windowedMode = v.toLowerCase(Locale.ROOT);
          break;
        case "--image-window":
          o.imageWindow = Integer.parseInt(v);
          break;
        case "--descriptor-memory-mb":
          o.descriptorMemoryMb = Integer.parseInt(v);
          break;
        case "--image-memory-mb":
          o.imageMemoryMb = Integer.parseInt(v);
          break;
        case "--gpu-strip-rows":
          o.gpuStripRows = Integer.parseInt(v);
          break;
        case "--gpu-bank-mb":
          o.gpuBankMb = Integer.parseInt(v);
          break;
        case "--min-forward-overlap":
          o.minForwardOverlap = number(v);
          break;
        case "--min-side-overlap":
          o.minSideOverlap = number(v);
          break;
        case "--min-pair-overlap":
          o.minPairOverlap = number(v);
          break;
        case "--overlap-ground-z":
          o.overlapGroundZ = number(v);
          break;
        case "--flight-heading":
          o.flightHeading = number(v);
          break;
        case "--overlap-unknown":
          o.overlapUnknown = v.toLowerCase(Locale.ROOT);
          break;
        case "--orientation-input":
          o.orientationInput = Path.of(v);
          break;
        case "--orientation-input-format":
          o.orientationInputFormat = v.toLowerCase(Locale.ROOT);
          break;
        case "--calibration-format":
          o.calibrationFormat = v.toLowerCase(Locale.ROOT);
          break;
        case "--poses-format":
          o.posesFormat = v.toLowerCase(Locale.ROOT);
          break;
        case "--orientation-output-format":
          o.orientationOutputFormat = v.toLowerCase(Locale.ROOT);
          break;
        case "--interior-output-format":
          o.interiorOutputFormat = v.toLowerCase(Locale.ROOT);
          break;
        case "--exterior-output-format":
          o.exteriorOutputFormat = v.toLowerCase(Locale.ROOT);
          break;
        case "--orientation-output":
          o.orientationOutput = Path.of(v);
          break;
        case "--export-orientations":
          o.exportOrientations = Path.of(v);
          break;
        case "--orientation-image-list":
          o.orientationImageList = Path.of(v);
          break;
        case "--orientation-pixels":
          o.orientationPixels = v.toLowerCase(Locale.ROOT);
          break;
        case "--orientation-rotation":
          o.orientationRotation = v.toLowerCase(Locale.ROOT);
          break;
        case "--orientation-axes":
          o.orientationAxes = v.toLowerCase(Locale.ROOT);
          break;
        case "--orientation-angle-unit":
          o.orientationAngleUnit = v.toLowerCase(Locale.ROOT);
          break;
        case "--orientation-pixel-size":
          {
            String[] pair = cameraValue(v);
            if (pair == null) o.orientationPixelSize = number(v);
            else o.orientationPixelSizes = put(o.orientationPixelSizes, pair[0], number(pair[1]), "--orientation-pixel-size");
          }
          break;
        case "--camera-grouping":
          o.cameraGrouping = v.toLowerCase(Locale.ROOT);
          break;
        case "--camera-groups":
          o.cameraGroupsFile = Path.of(v);
          break;
        case "--orientation-crs":
          o.orientationCrs = v;
          break;
        case "--orientation-component":
          o.orientationComponent = Integer.parseInt(v);
          break;
        case "--orientation-world-units":
          o.orientationUnits = v.toLowerCase(Locale.ROOT);
          break;
        case "--analyze-run":
          o.analyzeRun = Path.of(v);
          break;
        case "--analysis-output":
          o.analysisOutput = Path.of(v);
          break;
        case "--analysis-samples":
          o.analysisSamples = Integer.parseInt(v);
          break;
        case "--analysis-cell-size":
          o.analysisCellSize = number(v);
          break;
        case "--analysis-plane-tolerance":
          o.analysisTolerance = number(v);
          break;
        case "--analysis-region":
          o.analysisRegion = v;
          break;
        case "--storage":
          storageExplicit = true;
          o.storage = v.toLowerCase(Locale.ROOT);
          break;
        case "--compute":
          o.compute = v.toLowerCase(Locale.ROOT);
          break;
        case "--fusion-compute":
          o.fusionCompute = v.toLowerCase(Locale.ROOT);
          break;
        case "--fusion-weight-math":
          o.fusionWeightMath = v.toLowerCase(Locale.ROOT);
          break;
        case "--fusion-layout":
          o.fusionLayout = v.toLowerCase(Locale.ROOT);
          break;
        case "--output-compression":
          o.outputCompression = Integer.parseInt(v);
          break;
        case "--export-confidence":
          o.exportConfidence = Path.of(v);
          break;
        case "--dense-pair-selection":
          o.densePairSelection = v.toLowerCase(Locale.ROOT);
          break;
        case "--dense-pair-neighbors":
          o.densePairNeighbors = Integer.parseInt(v);
          break;
        case "--streamed-processing":
          o.streamedProcessing = v.toLowerCase(Locale.ROOT);
          break;
        case "--image-fusion-cache-mb":
          o.imageFusionCacheMb = v.equalsIgnoreCase("auto") ? -1 : Integer.parseInt(v);
          break;
        case "--gpu-transfer-mode":
          choice(v, "--gpu-transfer-mode", "queued", "blocking");
          o.gpuQueuedUploads = v.equals("queued");
          break;
        case "--gpu-summary-format":
          choice(v, "--gpu-summary-format", "packed", "full");
          o.gpuPackedSummaries = v.equals("packed");
          break;
        case "--calibration-trial-iterations":
          o.calibrationTrialIterations = Integer.parseInt(v);
          break;
        case "--calibration-trial-method":
          o.calibrationTrialMethod = v.toLowerCase(Locale.ROOT);
          choice(o.calibrationTrialMethod, "--calibration-trial-method", "full", "subset", "joint");
          break;
        case "--calibration-acceptance":
          o.calibrationAcceptance = v.toLowerCase(Locale.ROOT);
          choice(o.calibrationAcceptance, "--calibration-acceptance", "strict", "relaxed");
          break;
        case "--calibration-trial-cameras":
          o.calibrationTrialCameras = v.equalsIgnoreCase("auto") ? 0 : Integer.parseInt(v);
          if (o.calibrationTrialCameras != 0 && o.calibrationTrialCameras < 4)
            throw new IllegalArgumentException("--calibration-trial-cameras must be auto or at least 4");
          break;
        case "--calibration-pipeline":
          o.calibrationPipeline = v.toLowerCase(Locale.ROOT);
          pipelineExplicit = true;
          break;
        case "--fusion-emission":
          o.fusionEmission = v.toLowerCase(Locale.ROOT);
          break;
        case "--fusion-evidence":
          o.fusionEvidence = v.toLowerCase(Locale.ROOT);
          break;
        case "--fusion-mode":
          o.fusionMode = v.toLowerCase(Locale.ROOT);
          break;
        case "--fusion-gpu-normals":
          o.fusionGpuNormals = onOff(v, "--fusion-gpu-normals");
          break;
        case "--processed-image-cache":
          o.processedImageCache = v.toLowerCase(Locale.ROOT);
          break;
        case "--tiled-las-output":
          o.tiledLasOutput = number(v);
          break;
        case "--ply":
          o.writePly = onOff(v, "--ply");
          break;
        case "--point-confidence":
          o.writePointConfidence = onOff(v, "--point-confidence");
          break;
        case "--stereo-pipeline":
          o.stereoPipeline = switch (v.toLowerCase(Locale.ROOT)) {
            case "on" -> true; case "off" -> false;
            default -> throw new IllegalArgumentException("--stereo-pipeline requires on|off");
          };
          break;
        case "--fusion-camera-culling":
          o.fusionCameraCulling =
              switch (v.toLowerCase(Locale.ROOT)) {
                case "on" -> true;
                case "off" -> false;
                default ->
                    throw new IllegalArgumentException("--fusion-camera-culling requires on|off");
              };
          break;
        case "--fusion-association":
          o.fusionAssociation = v.toLowerCase(Locale.ROOT);
          break;
        case "--fusion-outliers":
          o.fusionOutliers = v.toLowerCase(Locale.ROOT);
          break;
        case "--fusion-window-mb":
          o.fusionWindowMb = Integer.parseInt(v);
          break;
        case "--fusion-gpu-evidence":
          o.fusionGpuEvidence = v.toLowerCase(Locale.ROOT);
          break;
        case "--gpu-device":
          o.gpuDevice = Integer.parseInt(v);
          break;
        case "--gpu-memory-mb":
          o.gpuMemoryMb = Long.parseLong(v);
          break;
        case "--rectification-alpha":
          o.rectificationAlpha = v.toLowerCase(Locale.ROOT);
          break;
        case "--matching-algorithm":
          o.matchingAlgorithm = v.toLowerCase(Locale.ROOT);
          break;
        case "--sift-contrast":
          o.siftContrast = number(v);
          break;
        case "--feature-grid":
          o.featureGrid = Integer.parseInt(v);
          if (o.featureGrid != 0 && (o.featureGrid < 2 || o.featureGrid > 256))
            throw new IllegalArgumentException("--feature-grid must be 0 (off) or 2..256");
          break;
        case "--sift-edge-threshold":
          o.siftEdgeThreshold = number(v);
          break;
        case "--orb-fast-threshold":
          o.orbFastThreshold = Integer.parseInt(v);
          break;
        case "--brisk-threshold":
          o.briskThreshold = Integer.parseInt(v);
          break;
        case "--akaze-threshold":
          o.akazeThreshold = number(v);
          break;
        case "--kaze-threshold":
          o.kazeThreshold = number(v);
          break;
        case "--descriptor-matcher":
          o.descriptorMatcher = v.toLowerCase(Locale.ROOT);
          break;
        case "--pair-selection":
          o.pairSelection = v.toLowerCase(Locale.ROOT);
          break;
        case "--pair-matching":
          o.pairMatching = v.toLowerCase(Locale.ROOT);
          break;
        case "--pair-candidates":
          o.pairCandidates = Integer.parseInt(v);
          break;
        case "--alignment-quality":
          o.alignmentQuality = v.toLowerCase(Locale.ROOT);
          break;
        case "--alignment-max-dimension":
          o.alignmentMaxDimension = Integer.parseInt(v);
          break;
        case "--block-processing":
          o.blockProcessing = v.toLowerCase(Locale.ROOT);
          break;
        case "--block-images":
          o.blockImages = Integer.parseInt(v);
          break;
        case "--mvsgm-neighbors":
          o.mvsgmNeighbors = Integer.parseInt(v);
          break;
        case "--mvsgm-best-k":
          o.mvsgmBestK = Integer.parseInt(v);
          break;
        case "--mvsgm-census-window":
          o.mvsgmCensus = Integer.parseInt(v);
          break;
        case "--mvsgm-levels":
          o.mvsgmLevels = Integer.parseInt(v);
          break;
        case "--mvsgm-consistent-views":
          o.mvsgmConsistentViews = Integer.parseInt(v);
          break;
        case "--mvsgm-consistency-labels":
          o.mvsgmConsistencyLabels = number(v);
          break;
        case "--mvsgm-cameras-in-flight":
          o.mvsgmCamerasInFlight = v.equalsIgnoreCase("auto") ? 0 : Integer.parseInt(v);
          break;
        case "--mvsgm-tile":
          o.mvsgmTile = v.equalsIgnoreCase("auto") ? -1 : v.equalsIgnoreCase("off") ? 0 : Integer.parseInt(v);
          if (o.mvsgmTile != -1 && o.mvsgmTile != 0 && (o.mvsgmTile < 256 || o.mvsgmTile > 16384))
            throw new IllegalArgumentException("--mvsgm-tile must be auto, off or 256..16384 pixels");
          break;
        case "--dense-checkpoint":
          if (!v.equalsIgnoreCase("off") && !v.equalsIgnoreCase("auto") && !v.matches("[1-9][0-9]{0,5}"))
            throw new IllegalArgumentException("--dense-checkpoint must be off, auto or minutes (1..999999)");
          o.denseCheckpoint = v.toLowerCase(Locale.ROOT);
          break;
        case "--fusion-host-memory-mb":
          o.fusionHostMemoryMb = Long.parseLong(v);
          break;
        case "--fusion-host-cache-mb":
          o.fusionHostCacheMb = Integer.parseInt(v);
          break;
        case "--fusion-source-maps":
          o.fusionSourceMaps = v.toLowerCase(Locale.ROOT);
          break;
        case "--block-overlap":
          o.blockOverlap = number(v);
          break;
        case "--intrinsics-mode":
          {
            String[] pair = cameraValue(v);
            if (pair == null) o.intrinsicsMode = v.toLowerCase(Locale.ROOT);
            else o.intrinsicsModes = put(o.intrinsicsModes, pair[0], pair[1].toLowerCase(Locale.ROOT), key);
          }
          break;
        case "--extrinsics-mode":
          o.extrinsicsMode = v.toLowerCase(Locale.ROOT);
          break;
        case "--refine-intrinsics":
          o.refineIntrinsics = v.toLowerCase(Locale.ROOT);
          break;
        case "--focal-sigma":
          o.focalSigma = number(v);
          break;
        case "--radial-sigma":
          o.radialSigma = number(v);
          break;
        case "--position-sigma":
          o.positionSigma = number(v);
          break;
        case "--rotation-sigma":
          o.rotationSigma = number(v);
          break;
        case "--tile-size":
          o.tileSize = Integer.parseInt(v);
          break;
        case "--tile-overlap":
          o.tileOverlap = Integer.parseInt(v);
          break;
        case "--pyramid-levels":
          o.pyramidLevels = v.equalsIgnoreCase("auto") ? 0 : Integer.parseInt(v);
          if (o.pyramidLevels == 0 && !v.equalsIgnoreCase("auto"))
            throw new IllegalArgumentException("--pyramid-levels must be auto or 1..6");
          break;
        case "--pyramid-radius":
          o.pyramidRadius = Integer.parseInt(v);
          break;
        case "--cleanup-passes":
          o.cleanupPasses = Integer.parseInt(v);
          break;
        case "--track-min-views":
          o.trackMinViews = Integer.parseInt(v);
          break;
        case "--track-max-error":
          o.trackMaxError = number(v);
          break;
        case "--track-min-angle":
          o.trackMinAngle = number(v);
          break;
        case "--registration-bundle-growth":
          o.registrationBundleGrowth = number(v);
          if (!(o.registrationBundleGrowth >= 0) || Double.isInfinite(o.registrationBundleGrowth))
            throw new IllegalArgumentException(
                "--registration-bundle-growth must be a percentage >= 0 (0 disables)");
          break;
        case "--registration-recovery":
          if (!v.equals("off") && !v.equals("on"))
            throw new IllegalArgumentException("--registration-recovery requires off|on");
          o.registrationRecovery = v;
          break;
        case "--match-repair":
          if (!v.equals("off") && !v.equals("on"))
            throw new IllegalArgumentException("--match-repair requires off|on");
          o.matchRepair = v;
          break;
        case "--gps-neighbors":
          o.gpsNeighbors = Integer.parseInt(v);
          if (o.gpsNeighbors < 0 || o.gpsNeighbors > 64)
            throw new IllegalArgumentException("--gps-neighbors must be 0..64");
          break;
        case "--registration-pose-check":
          if (!v.equals("off") && !v.equals("on"))
            throw new IllegalArgumentException("--registration-pose-check requires off|on");
          o.registrationPoseCheck = v;
          break;
        case "--match-repair-candidates":
          o.matchRepairCandidates = Integer.parseInt(v);
          if (o.matchRepairCandidates < 1 || o.matchRepairCandidates > 200)
            throw new IllegalArgumentException("--match-repair-candidates must be 1..200");
          break;
        case "--match-repair-min-pairs":
          o.matchRepairMinPairs = Integer.parseInt(v);
          if (o.matchRepairMinPairs < 1 || o.matchRepairMinPairs > 50)
            throw new IllegalArgumentException("--match-repair-min-pairs must be 1..50");
          break;
        case "--triangulation-outliers":
          if (!v.equals("drop") && !v.equals("reject"))
            throw new IllegalArgumentException("--triangulation-outliers requires drop|reject");
          o.triangulationOutliers = v;
          break;
        case "--rerun-from":
          o.rerunFrom = v.toLowerCase(Locale.ROOT);
          o.resume = true;
          break;
        case "-i":
        case "--images":
          o.input = Paths.get(v);
          break;
        case "-o":
        case "--output":
          o.output = Paths.get(v);
          break;
        case "--calibration":
        case "-d":
        case "--dist":
          {
            String[] pair = existingPath(v) ? null : cameraValue(v);
            if (pair == null) o.calibration = Paths.get(v);
            else o.calibrationFiles = put(o.calibrationFiles, pair[0], Paths.get(pair[1]), key);
          }
          break;
        case "--poses":
          o.poses = Paths.get(v);
          break;
        case "--crs":
        case "--epsg":
          o.crs = Crs.normalize(v);
          break;
        case "--gps-threshold":
          o.gpsThreshold = number(v);
          break;
        case "--stereo-pairs-in-flight":
          o.stereoPairsInFlight = v.equalsIgnoreCase("auto") ? 0 : Integer.parseInt(v);
          break;
        case "--tie-point-cleanup":
          o.tiePointCleanup = v.toLowerCase(Locale.ROOT);
          break;
        case "--tie-point-max-error":
          o.tiePointMaxError = v.equalsIgnoreCase("auto") ? 0 : number(v);
          o.tiePointLimitsGiven = true;
          break;
        case "--tie-point-min-views":
          o.tiePointMinViews = Integer.parseInt(v);
          o.tiePointLimitsGiven = true;
          break;
        case "--tie-point-min-angle":
          o.tiePointMinAngle = number(v);
          o.tiePointLimitsGiven = true;
          break;
        case "--tie-point-max-removed":
          o.tiePointMaxRemoved = number(v);
          o.tiePointLimitsGiven = true;
          break;
        case "--camera-screening":
          o.cameraScreening = v.toLowerCase(Locale.ROOT);
          break;
        case "--gps-bundle":
          o.gpsBundle = v.toLowerCase(Locale.ROOT);
          break;
        case "--gps-sigma":
          o.gpsSigma = number(v);
          break;
        case "--gps-intrinsics":
          o.gpsIntrinsics = v.toLowerCase(Locale.ROOT).replace(" ", "");
          break;
        case "--gps-focal-prior":
          o.gpsFocalPrior = v.toLowerCase(Locale.ROOT);
          break;
        case "--gps-focal-sigma":
          o.gpsFocalSigma = number(v);
          break;
        case "--gps-time-offset":
          o.gpsTimeOffset = v.toLowerCase(Locale.ROOT);
          break;
        case "--gps-lever-arm":
          o.gpsLeverArm = v.toLowerCase(Locale.ROOT).replace(" ", "");
          break;
        case "--rolling-shutter":
          o.rollingShutter = v.toLowerCase(Locale.ROOT);
          break;
        case "--ground-fusion-radius":
          o.groundFusionRadius = v.equalsIgnoreCase("auto") ? 0 : number(v);
          break;
        case "--ground-cell":
          o.groundCell = v.equalsIgnoreCase("auto") ? 0 : number(v);
          break;
        case "--ground-max-slope":
          o.groundMaxSlope = number(v);
          break;
        case "--gcp":
          o.gcp = Path.of(v);
          break;
        case "--gcp-markers":
          o.gcpMarkers = Path.of(v);
          break;
        case "--gcp-sigma":
          o.gcpSigma = number(v);
          break;
        case "--gcp-height-sigma":
          o.gcpHeightSigma = number(v);
          break;
        case "--gcp-check":
          o.gcpCheck = v.toLowerCase(Locale.ROOT);
          break;
        case "--gcp-check-ids":
          o.gcpCheckIds = v.trim();
          break;
        case "--lidar-reference":
          o.lidarReference = Path.of(v);
          break;
        case "--lidar-scale":
          o.lidarScale = onOff(v, "--lidar-scale");
          break;
        case "--lidar-search-shift":
          o.lidarSearchShift = number(v);
          break;
        case "--lidar-search-angle":
          o.lidarSearchAngle = number(v);
          break;
        case "--lidar-cell":
          o.lidarCell = number(v);
          break;
        case "--lidar-canopy-weight":
          o.lidarCanopyWeight = number(v);
          break;
        case "--lidar-outlier":
          o.lidarOutlier = number(v);
          break;
        case "--lidar-change-limit":
          o.lidarChangeLimit = number(v);
          break;
        case "--lidar-weak-limit":
          o.lidarWeakLimit = number(v);
          break;
        case "--lidar-margin":
          o.lidarMargin = number(v);
          break;
        case "--lidar-crs":
          o.lidarCrs = Crs.normalize(v);
          if (o.lidarCrs.contains("+"))
            throw new IllegalArgumentException("--lidar-crs takes a horizontal EPSG code only; use --lidar-vertical-datum for the heights");
          break;
        case "--lidar-vertical-datum":
          o.lidarVerticalDatum = v.trim();
          break;
        case "--source-geoid":
          o.sourceGeoid = VerticalDatum.normalize(v, key);
          break;
        case "--target-geoid":
          o.targetGeoid = VerticalDatum.normalize(v, key);
          break;
        case "--gcp-port":
          o.gcpPort = Integer.parseInt(v);
          break;
        case "--ground-max-object":
          o.groundMaxObject = number(v);
          break;
        case "--gps-height-sigma":
          o.gpsHeightSigma = v.toLowerCase(Locale.ROOT);
          break;
        case "--gps-adjustment-rounds":
          o.gpsAdjustmentRounds = Integer.parseInt(v);
          break;
        case "--seed-pair":
          o.seedPair = v;
          break;
        case "--census-mode":
          o.censusMode = v;
          break;
        case "--filtering":
          o.filtering = v.trim().toLowerCase(Locale.ROOT);
          break;
        case "--quality":
          o.quality = v.trim().toLowerCase(Locale.ROOT);
          o.qualityDownsample();
          break;
        case "--min-correlation":
          o.correlation = number(v);
          explicitFiltering.add(key);
          break;
        case "--correlation-window":
          o.correlationWindow = Integer.parseInt(v);
          break;
        case "--lr-threshold":
          o.lrThreshold = number(v);
          break;
        case "--speckle-size":
          o.speckleSize = Integer.parseInt(v);
          explicitFiltering.add(key);
          break;
        case "--speckle-range":
          o.speckleRange = number(v);
          break;
        case "--min-angle":
          o.minAngle = number(v);
          break;
        case "--confidence-threshold":
          o.confidenceThreshold = number(v);
          break;
        case "--two-view-confidence-threshold":
          o.twoViewConfidenceThreshold = number(v);
          if (!Double.isFinite(o.twoViewConfidenceThreshold)
              || o.twoViewConfidenceThreshold < 0
              || o.twoViewConfidenceThreshold > 1)
            throw new IllegalArgumentException(
                "--two-view-confidence-threshold must be finite in 0..1");
          break;
        case "--dense-method":
          o.denseMethod = v.toLowerCase(Locale.ROOT);
          break;
        case "--patchmatch-iterations":
          o.patchmatchIterations = Integer.parseInt(v);
          break;
        case "--patchmatch-edge-sigma":
          o.patchmatchEdgeSigma = number(v);
          break;
        case "--patchmatch-views":
          o.patchmatchViews = Integer.parseInt(v);
          break;
        case "--patchmatch-levels":
          o.patchmatchLevels = Integer.parseInt(v);
          break;
        case "--patchmatch-min-depth":
          o.patchmatchMinDepth = number(v);
          break;
        case "--patchmatch-max-depth":
          o.patchmatchMaxDepth = number(v);
          break;
        case "--patchmatch-speckle":
          o.patchmatchSpeckle = Integer.parseInt(v);
          if (o.patchmatchSpeckle < 0 || o.patchmatchSpeckle > 1000000)
            throw new IllegalArgumentException("--patchmatch-speckle must be 0..1000000 pixels");
          break;
        case "--patchmatch-speckle-tolerance":
          o.patchmatchSpeckleTolerance = number(v);
          if (!(o.patchmatchSpeckleTolerance > 0 && o.patchmatchSpeckleTolerance <= .2))
            throw new IllegalArgumentException("--patchmatch-speckle-tolerance must be in (0, 0.2]");
          break;
        case "--patchmatch-consistent-views":
          o.patchmatchConsistentViews = Integer.parseInt(v);
          if (o.patchmatchConsistentViews < 1 || o.patchmatchConsistentViews > 8)
            throw new IllegalArgumentException("--patchmatch-consistent-views must be 1..8");
          break;
        case "--patchmatch-neighbors":
          o.patchmatchNeighbors = v.toLowerCase(Locale.ROOT);
          break;
        case "--patchmatch-view-cost":
          switch (v.toLowerCase(Locale.ROOT)) {
            case "best" -> o.patchmatchTopViews = 1;
            case "top2" -> o.patchmatchTopViews = 2;
            case "top3" -> o.patchmatchTopViews = 3;
            case "top4" -> o.patchmatchTopViews = 4;
            default -> throw new IllegalArgumentException("--patchmatch-view-cost must be best, top2, top3 or top4");
          }
          break;
        case "--patchmatch-smooth":
          o.patchmatchSmooth = Integer.parseInt(v);
          if (o.patchmatchSmooth < 0 || o.patchmatchSmooth > 8)
            throw new IllegalArgumentException("--patchmatch-smooth must be 0..8");
          break;
        case "--patchmatch-smooth-tolerance":
          o.patchmatchSmoothTolerance = number(v);
          if (!(o.patchmatchSmoothTolerance > 0 && o.patchmatchSmoothTolerance <= .2))
            throw new IllegalArgumentException("--patchmatch-smooth-tolerance must be in (0, 0.2]");
          break;
        case "--patchmatch-tile-halo":
          o.patchmatchTileHalo = v.equalsIgnoreCase("auto") ? -1 : Integer.parseInt(v);
          if (o.patchmatchTileHalo < -1 || o.patchmatchTileHalo > 4096)
            throw new IllegalArgumentException("--patchmatch-tile-halo must be auto or 0..4096");
          break;
        case "--patchmatch-tile":
          o.patchmatchTile =
              v.equalsIgnoreCase("auto") ? -1 : v.equalsIgnoreCase("off") ? 0 : Integer.parseInt(v);
          if (o.patchmatchTile > 0 && (o.patchmatchTile < 256 || o.patchmatchTile > 16384))
            throw new IllegalArgumentException("--patchmatch-tile must be auto, off or 256..16384");
          break;
        case "--patchmatch-references-in-flight":
          o.patchmatchReferencesInFlight =
              v.equalsIgnoreCase("auto") ? 0 : Integer.parseInt(v);
          break;
        case "--min-views":
          o.minViews = Integer.parseInt(v);
          break;
        case "--fusion-strategy":
          o.fusionStrategy = v.toLowerCase(Locale.ROOT);
          break;
        case "--fusion-dedupe":
          // Earlier name of --fusion-strategy (3 Nov 2026): consume = owner-consume.
          o.fusionStrategy =
              v.toLowerCase(Locale.ROOT).equals("consume")
                  ? "owner-consume"
                  : v.toLowerCase(Locale.ROOT);
          break;
        case "--gpu-feature-matching":
          if (!Set.of("on", "off").contains(v.toLowerCase(Locale.ROOT)))
            throw new IllegalArgumentException("--gpu-feature-matching must be on or off");
          o.gpuFeatureMatching = v.equalsIgnoreCase("on");
          break;
        case "--fusion-tolerance":
          o.fusionTolerance = v.toLowerCase(Locale.ROOT);
          break;
        case "--fusion-sigma-k":
          o.fusionSigmaK = number(v);
          break;
        case "--fusion-precision-check":
          if (!Set.of("on", "off").contains(v.toLowerCase(Locale.ROOT)))
            throw new IllegalArgumentException("--fusion-precision-check must be on or off");
          o.fusionPrecisionCheck = v.equalsIgnoreCase("on");
          break;
        case "--fusion-seed-order":
          o.fusionSeedOrder = v.toLowerCase(Locale.ROOT);
          break;
        case "--fusion-seed-min-angle":
          o.fusionSeedMinAngle = number(v);
          break;
        case "--fusion-local-sigma":
          o.fusionLocalSigma = number(v);
          break;
        case "--fusion-free-space":
          o.fusionFreeSpace = v.toLowerCase(Locale.ROOT);
          if (!Set.of("on", "off", "strict").contains(o.fusionFreeSpace))
            throw new IllegalArgumentException("--fusion-free-space must be on, strict or off");
          break;
        case "--fusion-spike-filter":
          if (!v.equalsIgnoreCase("on") && !v.equalsIgnoreCase("off"))
            throw new IllegalArgumentException("--fusion-spike-filter must be on or off");
          o.fusionSpikeFilter = v.equalsIgnoreCase("on");
          break;
        case "--fusion-depth-outliers":
          o.fusionDepthOutliers = !v.equalsIgnoreCase("off");
          if (!v.equalsIgnoreCase("on") && !v.equalsIgnoreCase("off"))
            throw new IllegalArgumentException("--fusion-depth-outliers must be on or off");
          break;
        case "--fusion-agreement-radius":
          o.fusionAgreementRadius = v.equals("auto") ? -1 : Integer.parseInt(v);
          break;
        case "--fusion-relative":
          o.fusionRelative = number(v);
          explicitFiltering.add(key);
          break;
        case "--fusion-pixels":
          o.fusionPixels = number(v);
          explicitFiltering.add(key);
          break;
        case "--fusion-consensus":
          o.fusionConsensus = number(v);
          explicitFiltering.add(key);
          break;
        case "--max-depth-uncertainty":
          o.maxDepthUncertainty = number(v);
          break;
        case "--depth-refinement":
          o.depthRefinementMethod = v.toLowerCase(Locale.ROOT);
          break;
        case "--surface-consolidation":
          o.surfaceConsolidation = v.toLowerCase(Locale.ROOT);
          break;
        case "--point-attributes":
          o.pointAttributes = v.toLowerCase(Locale.ROOT);
          break;
        case "--surface-validation":
          o.surfaceValidation = v.toLowerCase(Locale.ROOT);
          break;
        case "--surface-radius":
          o.surfaceRadius = number(v);
          break;
        case "--surface-spacing":
          o.surfaceSpacing = number(v);
          break;
        case "--surface-flatten":
          o.surfaceFlatten = number(v);
          break;
        case "--surface-max-shift":
          o.surfaceMaxShift = number(v);
          break;
        case "--surface-normal-angle":
          o.surfaceNormalAngle = number(v);
          break;
        case "--surface-memory-mb":
          o.surfaceMemoryMb = Integer.parseInt(v);
          break;
        case "-f":
        case "--format":
          o.format = v.replace(".", "");
          break;
        case "--focal":
          {
            String[] pair = cameraValue(v);
            if (pair == null) o.focal = number(v);
            else o.focalByCamera = put(o.focalByCamera, pair[0], number(pair[1]), key);
          }
          break;
        case "--s_w":
          {
            String[] pair = cameraValue(v);
            if (pair == null) o.sensorWidth = number(v);
            else o.sensorWidthByCamera = put(o.sensorWidthByCamera, pair[0], number(pair[1]), key);
          }
          break;
        case "--baseline":
          o.baseline = number(v);
          break;
        case "--scale":
          o.scale = number(v);
          break;
        case "--p1":
          o.p1 = number(v);
          break;
        case "--p2":
          o.p2 = number(v);
          break;
        case "--ratio":
          o.ratio = number(v);
          break;
        case "--reprojection":
          o.reprojection = number(v);
          break;
        case "--uniqueness":
          o.uniqueness = number(v);
          break;
        case "--voxel":
          o.voxel = number(v);
          break;
        case "--max-depth":
          o.maxDepth = number(v);
          break;
        case "--min-disparity":
          o.minDisparity = Integer.parseInt(v);
          o.minDisparityExplicit = true;
          break;
        case "--disparities":
          o.disparities = Integer.parseInt(v);
          o.disparitiesExplicit = true;
          break;
        case "--similarity-window":
        case "--ws_census":
          o.census = Integer.parseInt(v);
          break;
        case "--ws_mFilter":
          o.median = Integer.parseInt(v);
          break;
        case "--directions":
          o.directions = Integer.parseInt(v);
          break;
        case "--max-dimension":
          o.maxDimension = Integer.parseInt(v);
          o.maxDimensionExplicit = true;
          break;
        case "--features":
          o.features = Integer.parseInt(v);
          break;
        case "--min-matches":
          o.minMatches = Integer.parseInt(v);
          break;
        case "--neighbors":
          o.neighbors = Integer.parseInt(v);
          break;
        case "--bundle-iterations":
          o.iterations = Integer.parseInt(v);
          o.bundleIterationsExplicit = true;
          break;
        case "--bundle-solver":
          o.bundleSolver = v.toLowerCase(Locale.ROOT);
          choice(o.bundleSolver, "--bundle-solver", "legacy", "fast");
          break;
        case "--bundle-direct-limit":
          o.bundleDirectLimit = v.equalsIgnoreCase("legacy") ? -1 : Integer.parseInt(v);
          if (o.bundleDirectLimit < -1)
            throw new IllegalArgumentException("--bundle-direct-limit requires legacy or N >= 0");
          bundleExplicit.add(key);
          break;
        case "--bundle-preconditioner":
          o.bundlePreconditioner = v.toLowerCase(Locale.ROOT);
          choice(o.bundlePreconditioner, "--bundle-preconditioner", "block-jacobi", "cluster-jacobi");
          bundleExplicit.add(key);
          break;
        case "--bundle-inexact-steps":
          o.bundleInexactSteps = onOff(v, "--bundle-inexact-steps");
          bundleExplicit.add(key);
          break;
        case "--registration-local-bundles":
          o.registrationLocalBundles = onOff(v, "--registration-local-bundles");
          bundleExplicit.add(key);
          break;
        case "--lens-fit-stop":
          o.lensFitStop = v.toLowerCase(Locale.ROOT);
          choice(o.lensFitStop, "--lens-fit-stop", "objective", "parameters");
          bundleExplicit.add(key);
          break;
        case "--step":
          o.step = Integer.parseInt(v);
          stepExplicit = true;
          break;
        case "--cores":
        case "-cores":
          o.cores = Integer.parseInt(v);
          break;
        case "--surface-batch-size":
          o.surfaceBatchSize = Integer.parseInt(v);
          break;
        case "--surface-partition":
          o.surfacePartition = v;
          break;
        case "--fusion-page-side":
          o.fusionPageSide = Integer.parseInt(v);
          break;
        case "--fusion-cache-mb":
          o.fusionCacheMb = Integer.parseInt(v);
          break;
        case "--fusion-batch-size":
          o.fusionBatchSize = Integer.parseInt(v);
          break;
        case "--report":
          o.processingReport = v;
          break;
        case "--report-detail":
          o.reportDetail = v.toLowerCase(Locale.ROOT);
          break;
        case "--orthomosaic-resolution":
          o.orthomosaicResolution = v.equalsIgnoreCase("auto") ? 0 : number(v);
          break;
        case "--orthomosaic-fill":
          o.orthomosaicFill = v.equalsIgnoreCase("auto") ? -1 : number(v);
          break;
        case "--orthomosaic-mode":
          o.orthomosaicMode = v.toLowerCase(Locale.ROOT);
          break;
        case "--orthomosaic-smooth":
          o.orthomosaicSmooth = v.equalsIgnoreCase("auto") ? -1 : number(v);
          break;
        case "--orthomosaic-max-angle":
          o.orthomosaicMaxAngle = v.equalsIgnoreCase("off") ? 0 : number(v);
          break;
        case "--orthomosaic-feather":
          o.orthomosaicFeather = number(v);
          break;
        case "--orthomosaic-surface-cell":
          o.orthomosaicSurfaceCell = v.equalsIgnoreCase("auto") ? 0 : number(v);
          break;
        case "--export-orthomosaic":
          o.exportOrthomosaic = Paths.get(v);
          break;
        case "--orthomosaic-output":
          o.orthomosaicOutput = Paths.get(v);
          break;
        case "--export-report":
          o.exportReport = Paths.get(v);
          break;
        case "--report-output":
          o.reportOutput = Paths.get(v);
          break;
        case "--report-dsm-resolution":
          o.reportDsmResolution = number(v);
          break;
        case "--report-dsm-max-dimension":
          o.reportDsmMaxDimension = Integer.parseInt(v);
          break;
        case "--report-dsm-min-points":
          o.reportDsmMinPoints = Integer.parseInt(v);
          break;
        case "--report-dsm-statistic":
          o.reportDsmStatistic = v;
          break;
        case "--memory-mb":
          o.memoryMb = Long.parseLong(v);
          break;
        case "--ram-limit-mb":
          o.ramLimitMb = Long.parseLong(v);
          break;
        case "--board-columns":
          o.boardColumns = Integer.parseInt(v);
          break;
        case "--board-rows":
          o.boardRows = Integer.parseInt(v);
          break;
        case "--square-size":
          o.squareSize = number(v);
          break;
        default:
          throw new IllegalArgumentException("Unknown option: " + key);
      }
    }
    o.applyFilteringPreset(explicitFiltering);
    if (o.bundleSolver.equals("fast")) o.applyFastBundleSolver(bundleExplicit);
    // Multi-view SGM emits every consistent pixel unless --step is given: its maps are dense per
    // camera, and the step-2 default exists for pair SGM (a default thinning would quarter mvsgm clouds).
    if (!stepExplicit && o.denseMethod.equals("mvsgm")) o.step = 1;
    if (o.help) return o;
    choice(o.surfacePartition, "--surface-partition", "routed", "recursive");
    choice(o.processingReport, "--report", "pdf", "html", "none");
    choice(o.reportDetail, "--report-detail", "summary", "full");
    choice(o.reportDsmStatistic, "--report-dsm-statistic", "max", "mean");
    if (o.outputCompression < 1 || o.outputCompression > 9)
      throw new IllegalArgumentException("Output compression must be 1..9");
    if (o.cores < 1) throw new IllegalArgumentException("--cores must be positive");
    if (o.surfaceBatchSize < 1
        || o.surfaceBatchSize > 1024
        || o.fusionPageSide < 16
        || o.fusionPageSide > 512
        || (o.fusionPageSide & (o.fusionPageSide - 1)) != 0
        || o.fusionCacheMb < 0
        || o.fusionBatchSize < 32
        || o.fusionBatchSize > 65536
        || o.reportDsmResolution < 0
        || o.reportDsmMaxDimension < 16
        || o.reportDsmMaxDimension > 4096
        || o.reportDsmMinPoints < 1)
      throw new IllegalArgumentException("Invalid fusion cache/batch or report DSM settings");
    if (java.util.stream.Stream.of(
                o.exportReport, o.exportConfidence, o.analyzeRun, o.exportOrientations, o.exportOrthomosaic)
            .filter(Objects::nonNull)
            .count()
        > 1) throw new IllegalArgumentException("Choose one standalone export/analysis action");
    if (!(o.orthomosaicResolution >= 0) || !(o.orthomosaicSurfaceCell >= 0))
      throw new IllegalArgumentException("--orthomosaic-resolution and --orthomosaic-surface-cell must be auto or positive");
    if (!o.orthomosaicMode.equals("true") && !o.orthomosaicMode.equals("classic") && !o.orthomosaicMode.equals("mosaic"))
      throw new IllegalArgumentException("--orthomosaic-mode must be true, classic or mosaic");
    if (!(o.orthomosaicMaxAngle >= 0 && o.orthomosaicMaxAngle < 90))
      throw new IllegalArgumentException("--orthomosaic-max-angle must be off or 0..<90 degrees");
    if (!(o.orthomosaicSmooth >= 0 || o.orthomosaicSmooth == -1) || !(o.orthomosaicFeather >= 0))
      throw new IllegalArgumentException("--orthomosaic-smooth must be auto or >= 0, --orthomosaic-feather >= 0");
    if (o.exportOrthomosaic != null) {
      if (!Files.isDirectory(o.exportOrthomosaic))
        throw new IllegalArgumentException("Choose a valid run directory for --export-orthomosaic");
      return o;
    }
    if (o.exportReport != null) {
      if (!Files.isDirectory(o.exportReport)
          || o.analyzeRun != null
          || o.exportOrientations != null)
        throw new IllegalArgumentException("Choose a valid standalone report run directory");
      return o;
    }
    if (o.analyzeRun != null) {
      if (!Files.isDirectory(o.analyzeRun)
          || o.analysisSamples < 20
          || o.analysisCellSize < 0
          || o.analysisTolerance < 0)
        throw new IllegalArgumentException("Invalid run analysis settings");
      if (o.exportOrientations != null)
        throw new IllegalArgumentException("Choose one standalone action");
      return o;
    }
    choice(o.orientationPixels, "--orientation-pixels", "source", "processed");
    choice(
        o.orientationRotation,
        "--orientation-rotation",
        "auto",
        "world-to-camera",
        "camera-to-world");
    choice(o.orientationAxes, "--orientation-axes", "auto", "opencv", "photogrammetric");
    choice(o.orientationAngleUnit, "--orientation-angle-unit", "degrees", "radians");
    choice(o.orientationUnits, "--orientation-world-units", "auto", "m", "ft", "mm");
    if (o.orientationComponent < 0)
      throw new IllegalArgumentException("Orientation component must be nonnegative");
    List<String> supportedOrientations = Arrays.asList(OrientationIO.FORMATS.split(","));
    for (String format :
        new String[] {o.orientationInputFormat, o.calibrationFormat, o.posesFormat})
      if (!format.equals("auto") && !supportedOrientations.contains(format))
        throw new IllegalArgumentException("Unsupported orientation input format: " + format);
    for (String formats :
        new String[] {o.orientationOutputFormat, o.interiorOutputFormat, o.exteriorOutputFormat})
      for (String format : formats.split(",", -1))
        if (!format.equals("none") && !supportedOrientations.contains(format))
          throw new IllegalArgumentException("Unsupported orientation output format: " + format);
    if (!Double.isNaN(o.orientationPixelSize) && !(o.orientationPixelSize > 0))
      throw new IllegalArgumentException("Orientation physical pixel size must be positive mm");
    if (o.exportConfidence != null) {
      if (!Files.isDirectory(o.exportConfidence)
          || o.exportOrientations != null
          || o.exportReport != null
          || o.analyzeRun != null)
        throw new IllegalArgumentException(
            "Choose one valid standalone confidence export directory");
      return o;
    }
    if (o.exportOrientations != null) {
      if (!Files.isDirectory(o.exportOrientations))
        throw new IllegalArgumentException(
            "--export-orientations must name a project run directory");
      return o;
    }
    if (o.diagnostics && !storageExplicit) o.storage = "debug";
    choice(o.storage, "--storage", "minimal", "standard", "debug");
    choice(o.densePairSelection, "--dense-pair-selection", "all", "coverage");
    if (o.imageFusionCacheMb < -1)
      throw new IllegalArgumentException("--image-fusion-cache-mb must be auto or nonnegative");
    choice(o.densePipeline, "--dense-pipeline", "standard", "aerial-tiled");
    choice(o.depthOutlierFilter, "--depth-outlier-filter", "off", "edge-aware");
    choice(o.blockProcessing, "--block-processing", "off", "auto", "on");
    if (o.blockImages < 0 || o.blockImages == 1)
      throw new IllegalArgumentException("--block-images must be 0 (forecast) or at least 2");
    if (!(o.blockOverlap >= 0) || o.blockOverlap > 20)
      throw new IllegalArgumentException("--block-overlap must be 0..20 image footprints");
    choice(o.disparityRefinement, "--disparity-refinement", "off", "multiview");
    if (!Double.isFinite(o.disparityAnchorAngle)
        || o.disparityAnchorAngle <= 0
        || o.disparityAnchorAngle >= 90
        || o.disparityRefinementMemoryMb < 8
        || o.disparityRefinementMemoryMb > 4096)
      throw new IllegalArgumentException(
          "Disparity refinement requires anchor angle >0..<90 and memory 8..4096 MiB");
    if (o.disparityRefinement.equals("multiview")
        && (!o.denseMethod.equals("sgm")
            || !o.densePipeline.equals("standard")
            || !o.fusionMode.equals("image")
            || o.step != 1
            || o.pyramidLevels != 1
            || o.rectified
            || o.windowedMode.equals("on")))
      throw new IllegalArgumentException(
          "--disparity-refinement multiview requires standard SGM, --fusion-mode image"
              + " --pyramid-levels 1 --step 1, unrectified images and non-windowed pairs");
    if (o.depthOutlierRadius < 2
        || o.depthOutlierRadius > 8
        || !Double.isFinite(o.depthOutlierDeviation)
        || o.depthOutlierDeviation <= 0
        || o.depthOutlierDeviation > 32
        || o.depthOutlierEdgeThreshold < 0
        || o.depthOutlierEdgeThreshold > 255)
      throw new IllegalArgumentException(
          "Depth outlier settings require radius 2..8,"
              + " deviation >0..32 disparity pixels, edge threshold 0..255");
    if (!o.depthOutlierFilter.equals("off")
        && o.denseMethod.equals("patchmatch")
        && !o.fusionMode.equals("image"))
      throw new IllegalArgumentException(
          "--depth-outlier-filter edge-aware requires --dense-method sgm or mvsgm, or patchmatch"
              + " with --fusion-mode image");
    if (o.aerialTileSize < 64 || o.aerialTileSize > 2048
        || o.aerialOverlap < 8 || o.aerialOverlap > o.aerialTileSize / 2
        || o.aerialCoarseDimension < 128 || o.aerialCoarseDimension > 2048
        || o.aerialDisparityMargin < 4 || o.aerialDisparityMargin > 256)
      throw new IllegalArgumentException("Invalid aerial tile settings: tile 64..2048, overlap"
          + " 8..tile/2, coarse dimension 128..2048, disparity margin 4..256");
    if (o.densePipeline.equals("aerial-tiled")
        && (!o.denseMethod.equals("sgm")
            || o.compute.equals("cpu")
            || o.pyramidLevels != 1
            || !o.fusionMode.equals("reference")
            || !o.fusionEmission.equals("pair")
            || !o.surfaceConsolidation.equals("none")
            || !o.fusionStrategy.equals("owner")
            || o.voxel != 0
            || o.windowedMode.equals("on")))
      throw new IllegalArgumentException(
          "--dense-pipeline aerial-tiled requires OpenCL GPU SGM, --pyramid-levels 1, --fusion-mode"
              + " reference, --fusion-emission pair, --surface-consolidation none,"
              + " --fusion-strategy owner, --voxel 0 and --windowed-mode off|auto. It uses bounded"
              + " block fusion, not image/incremental fusion or global consolidation.");
    // An explicit --calibration-pipeline bootstrap asks for the bootstrap: it turns the default
    // quick calibration off and conflicts with an explicit --quick-calibration on.
    if (pipelineExplicit && o.calibrationPipeline.equals("bootstrap") && o.quickCalibration) {
      if (quickExplicit)
        throw new IllegalArgumentException(
            "--calibration-pipeline bootstrap conflicts with --quick-calibration on; give --quick-calibration off");
      o.quickCalibration = false;
    }
    if (o.quickCalibration) o.calibrationPipeline = "joint";
    // The ground step reads point-confidence.bin (camera support for free-space checks, pair ids for
    // ground sheets); --diagnostics converts it to CSV. Keep it in those cases.
    if (!o.writePointConfidence
        && (o.classifyGround || o.groundPairCorrection || o.groundAwareFusion || o.diagnostics)) {
      o.writePointConfidence = true;
      System.out.println(
          "Note: --point-confidence off ignored; ground classification/correction and --diagnostics"
              + " need 07_results/point-confidence.bin");
    }
    if (o.tiledLasOutput < 0 || o.tiledLasOutput > 0 && o.tiledLasOutput * 1e4 > 4e9)
      throw new IllegalArgumentException(
          "--tiled-las-output requires a tile size above 0 and at most 400000 output units (0 = off)");
    if (o.tiledLasOutput > 0 && (o.classifyGround || o.groundPairCorrection || o.groundAwareFusion))
      throw new IllegalArgumentException(
          "--tiled-las-output cannot be combined with ground classification/correction: the ground"
              + " step reads and edits the single 07_results/dense.las");
    if (o.tiledLasOutput > 0 && !o.blockProcessing.equals("off"))
      throw new IllegalArgumentException(
          "--tiled-las-output cannot be combined with --block-processing: block merging reads each"
              + " block's single dense.las");
    choice(o.processedImageCache, "--processed-image-cache", "auto", "on", "off");
    choice(o.calibrationPipeline, "--calibration-pipeline", "bootstrap", "joint");
    choice(o.fusionEmission, "--fusion-emission", "pair", "camera");
    choice(o.fusionEvidence, "--fusion-evidence", "pair", "camera", "auto");
    choice(o.fusionGpuEvidence, "--fusion-gpu-evidence", "auto", "resident", "windowed");
    choice(o.fusionMode, "--fusion-mode", "reference", "incremental", "image");
    if (o.fusionMode.equals("image") && !o.fusionEmission.equals("pair"))
      throw new IllegalArgumentException(
          "--fusion-mode image requires --fusion-emission pair");
    choice(o.fusionOutliers, "--fusion-outliers", "reference", "robust");
    choice(o.fusionAssociation, "--fusion-association", "pair", "camera");
    if (o.fusionAssociation.equals("camera") && !o.fusionMode.equals("incremental"))
      throw new IllegalArgumentException(
          "--fusion-association camera requires --fusion-mode incremental");
    choice(o.fusionSourceMaps, "--fusion-source-maps", "retained", "scoped");
    if (o.fusionSourceMaps.equals("scoped")
        && !(o.denseMethod.equals("mvsgm") && o.fusionMode.equals("image")))
      throw new IllegalArgumentException("--fusion-source-maps scoped requires --dense-method mvsgm --fusion-mode image");
    if (o.fusionHostMemoryMb < 0 || o.fusionHostMemoryMb > Long.MAX_VALUE / MemorySafety.MIB
        || o.fusionHostCacheMb < 1)
      throw new IllegalArgumentException("--fusion-host-memory-mb must be nonnegative; --fusion-host-cache-mb must be positive");
    if (o.fusionHostMemoryMb > 0 && !MvsgmWindow.enabled(o))
      throw new IllegalArgumentException("--fusion-host-memory-mb requires overlap-window MVSGM image fusion"
          + " and --streamed-processing auto|on");
    if (o.fusionWindowMb < 0)
      throw new IllegalArgumentException("--fusion-window-mb must be nonnegative");
    if (o.windowedMode.equals("on") && o.denseMethod.equals("patchmatch"))
      throw new IllegalArgumentException(
          "--windowed-mode on currently supports SGM; PatchMatch windowing is not implemented");
    if (o.fusionEmission.equals("camera") && !o.surfaceConsolidation.equals("none"))
      throw new IllegalArgumentException(
          "Experimental camera emission requires --surface-consolidation none");
    if (o.densePairNeighbors < 1 || o.densePairNeighbors > 16)
      throw new IllegalArgumentException("--dense-pair-neighbors must be 1..16");
    choice(o.compute, "--compute", "cpu", "auto", "gpu");
    choice(o.matchingCost, "--matching-cost", "census", "ad-census", "census-gradient", "zncc");
    choice(o.medianMode, "--median-mode", "edge-preserving", "classic");
    if (o.sgmEdgeScale < 0 || o.sgmEdgeScale > 255 || o.medianMaxChange < 0)
      throw new IllegalArgumentException(
          "--sgm-edge-scale must be 0..255; --median-max-change must be nonnegative pixels");
    if (o.disparityEdgeErosion < 0 || o.disparityEdgeErosion > 8)
      throw new IllegalArgumentException("--disparity-edge-erosion must be 0..8 pixels");
    if (!(o.disparityEdgeJump > 0) || !Double.isFinite(o.disparityEdgeJump))
      throw new IllegalArgumentException("--disparity-edge-jump must be positive pixels");
    if (o.censusEdgeMask < 0 || o.censusEdgeMask > 255)
      throw new IllegalArgumentException("--census-edge-mask must be 0..255");
    if (o.censusShift < 0 || o.censusShift > 2)
      throw new IllegalArgumentException("--census-shift must be 0, 1 or 2");
    if ((o.censusEdgeMask > 0 || o.censusShift > 0) && o.matchingCost.equals("zncc"))
      throw new IllegalArgumentException(
          "--census-edge-mask and --census-shift require a census-based --matching-cost");
    if (o.medianEdgeThreshold < 0
        || o.medianEdgeThreshold > 255
        || (o.speckleConnectivity != 4 && o.speckleConnectivity != 8))
      throw new IllegalArgumentException(
          "Median edge threshold must be 0..255; speckle connectivity must be 4 or 8");
    choice(o.subpixelRefinement, "--subpixel-refinement", "none", "zncc");
    choice(o.pairwiseSearch, "--pairwise-search", "full", "tsgm");
    choice(o.subpixelFit, "--subpixel-fit", "parabola", "equiangular");
    choice(o.onPairFailure, "--on-pair-failure", "abort", "skip");
    choice(o.fusionGpuFailure, "--fusion-gpu-failure", "disable", "batch");
    if (o.tsgmLevels < 0 || o.tsgmLevels > 5 || o.tsgmMargin < 1 || o.tsgmMargin > 32)
      throw new IllegalArgumentException("--tsgm-levels must be 0..5; --tsgm-margin must be 1..32");
    if (o.pairwiseSearch.equals("tsgm") && o.pyramidLevels != 1)
      throw new IllegalArgumentException("--pairwise-search tsgm requires --pyramid-levels 1");
    choice(o.sgmAggregate, "--sgm-aggregate", "auto", "16", "32");
    StereoCost.aggregateBytes(o);
    choice(o.gpuSgm, "--gpu-sgm", "auto", "resident", "hybrid", "streamed", "esgm");
    if (o.esgmCandidates < 1 || o.esgmCandidates > 4)
      throw new IllegalArgumentException("--esgm-candidates must be 1..4");
    choice(o.gpuAggregate, "--gpu-aggregate", "auto", "ram", "disk", "recompute");
    choice(o.imageReader, "--image-reader", "auto", "windowed", "opencv");
    choice(o.windowedMode, "--windowed-mode", "auto", "on", "off");
    choice(o.streamedProcessing, "--streamed-processing", "auto", "on", "off");
    if (o.windowedMode.equals("on") && o.pyramidLevels != 1)
      throw new IllegalArgumentException("--windowed-mode on requires --pyramid-levels 1");
    if (o.imageWindow < 64
        || o.imageWindow > 8192
        || o.descriptorMemoryMb < 1
        || o.imageMemoryMb < 0)
      throw new IllegalArgumentException(
          "Image window must be 64..8192; descriptor memory must be positive");
    choice(o.overlapUnknown, "--overlap-unknown", "keep", "reject");
    for (double percentage : new double[] {o.minForwardOverlap, o.minSideOverlap, o.minPairOverlap})
      if (!(percentage >= 0 && percentage <= 100))
        throw new IllegalArgumentException("Overlap thresholds must be percentages 0..100");
    if (!(o.censusWeight >= 0 && o.censusWeight <= 1)
        || o.adTruncation < 1
        || o.adTruncation > 255
        || o.gradientTruncation < 1
        || o.gradientTruncation > 1020
        || o.gpuStripRows < 1
        || o.gpuBankMb < 1
        || o.gpuBankMb > 1024)
      throw new IllegalArgumentException(
          "Invalid matching-cost weight/truncation or GPU strip size");
    choice(o.denseMethod, "--dense-method", "sgm", "patchmatch", "mvsgm");
    if (o.denseMethod.equals("mvsgm")) {
      if (!o.fusionMode.equals("image") || !o.densePipeline.equals("standard")
          || o.disparityRefinement.equals("multiview"))
        throw new IllegalArgumentException(
            "--dense-method mvsgm requires --fusion-mode image, --dense-pipeline standard and"
                + " --disparity-refinement off");
      if (o.mvsgmNeighbors < 1 || o.mvsgmNeighbors > 8
          || o.mvsgmBestK < 1 || o.mvsgmBestK > o.mvsgmNeighbors
          || (o.mvsgmCensus != 3 && o.mvsgmCensus != 5 && o.mvsgmCensus != 7)
          || o.mvsgmLevels < -1 || o.mvsgmLevels > 5
          || o.mvsgmConsistentViews < 0 || o.mvsgmConsistentViews > o.mvsgmNeighbors
          || !(o.mvsgmConsistencyLabels > 0) || o.mvsgmConsistencyLabels > 64)
        throw new IllegalArgumentException(
            "mvsgm settings require --mvsgm-neighbors 1..8, --mvsgm-best-k 1..neighbors,"
                + " --mvsgm-census-window 3|5|7, --mvsgm-levels -1..5, --mvsgm-consistent-views"
                + " 0..neighbors, --mvsgm-consistency-labels >0..64");
      if (o.mvsgmCamerasInFlight < 0 || o.mvsgmCamerasInFlight > 16)
        throw new IllegalArgumentException("--mvsgm-cameras-in-flight must be auto or 1..16");
      if (Double.isFinite(o.twoViewConfidenceThreshold)) {
        System.out.println(
            "Note: --two-view-confidence-threshold applies to pair SGM only; ignored with"
                + " --dense-method mvsgm (multi-view consistency replaces the left/right check)");
        o.twoViewConfidenceThreshold = Double.NaN;
      }
    }
    if (!Double.isNaN(o.twoViewConfidenceThreshold)
        && (!Double.isFinite(o.twoViewConfidenceThreshold)
            || o.twoViewConfidenceThreshold < 0
            || o.twoViewConfidenceThreshold > 1
            || !o.denseMethod.equals("sgm")
            || o.pyramidLevels != 1))
      throw new IllegalArgumentException(
          "Experimental --two-view-confidence-threshold requires finite 0..1, --dense-method sgm"
              + " and --pyramid-levels 1");
    if (o.confidenceThreshold < 0 || o.confidenceThreshold > 1)
      throw new IllegalArgumentException("--confidence-threshold must be in 0..1");
    if (!Double.isFinite(o.patchmatchEdgeSigma)
        || o.patchmatchEdgeSigma < 0
        || o.patchmatchEdgeSigma > 255)
      throw new IllegalArgumentException("--patchmatch-edge-sigma must be between 0 and 255");
    if (o.patchmatchIterations < 1
        || o.patchmatchIterations > 12
        || o.patchmatchViews < 1
        || o.patchmatchViews > 8
        || o.patchmatchLevels < 1
        || o.patchmatchLevels > 5
        || o.patchmatchMinDepth < 0
        || o.patchmatchMaxDepth < 0
        || ((o.patchmatchMinDepth == 0) != (o.patchmatchMaxDepth == 0))
        || o.patchmatchReferencesInFlight < 0
        || o.patchmatchReferencesInFlight > 32
        || (o.patchmatchMaxDepth > 0 && o.patchmatchMaxDepth <= o.patchmatchMinDepth))
      throw new IllegalArgumentException(
          "Invalid PatchMatch depth range or iterations/views/levels");
    choice(o.patchmatchNeighbors, "--patchmatch-neighbors", "nearest", "score");
    choice(o.fusionCompute, "--fusion-compute", "inherit", "cpu", "auto", "gpu");
    choice(o.fusionWeightMath, "--fusion-weight-math", "direct", "reference");
    choice(o.fusionLayout, "--fusion-layout", "coalesced", "reference");
    if (o.gpuDevice < 0 || o.gpuMemoryMb < 0)
      throw new IllegalArgumentException("GPU device and memory budget must be nonnegative");
    if (o.resume) {
      if (cacheDisabled)
        throw new IllegalArgumentException("--no-cache conflicts with --resume/--rerun-from");
      o.cache = true;
    }
    if (o.listGpus) return o;
    if (!(o.siftContrast > 0
            && o.siftEdgeThreshold > 0
            && o.akazeThreshold > 0
            && o.kazeThreshold > 0)
        || o.orbFastThreshold < 0
        || o.orbFastThreshold > 255
        || o.briskThreshold < 0
        || o.briskThreshold > 255)
      throw new IllegalArgumentException("Invalid feature detector thresholds");
    choice(o.matchingAlgorithm, "--matching-algorithm", "sift", "orb", "akaze", "kaze", "brisk");
    choice(o.descriptorMatcher, "--descriptor-matcher", "auto", "bruteforce", "flann");
    choice(o.pairSelection, "--pair-selection", "auto", "sequential", "overlap", "gps", "all");
    choice(o.pairMatching, "--pair-matching", "parallel", "serial");
    choice(o.fusionStrategy, "--fusion-strategy", "owner", "owner-consume", "graph-average");
    choice(o.gpsBundle, "--gps-bundle", "off", "prior");
    choice(o.cameraScreening, "--camera-screening", "off", "report", "exclude");
    choice(o.tiePointCleanup, "--tie-point-cleanup", "off", "auto", "custom", "pause");
    if (o.tiePointCleanup.equals("auto") && o.tiePointLimitsGiven)
      throw new IllegalArgumentException("--tie-point-cleanup auto uses the automatic limits; to set --tie-point-max-error,"
          + " --tie-point-min-views, --tie-point-min-angle or --tie-point-max-removed use --tie-point-cleanup custom");
    if (!(o.tiePointMaxError >= 0) || o.tiePointMinViews < 2 || !(o.tiePointMinAngle >= 0 && o.tiePointMinAngle < 90)
        || !(o.tiePointMaxRemoved > 0 && o.tiePointMaxRemoved <= 100))
      throw new IllegalArgumentException("--tie-point-max-error must be 0 (auto) or more, --tie-point-min-views at least 2,"
          + " --tie-point-min-angle 0..90 degrees and --tie-point-max-removed 0..100 percent");
    if (o.stereoPairsInFlight < 0 || o.stereoPairsInFlight > 16)
      throw new IllegalArgumentException("--stereo-pairs-in-flight must be auto or 1..16");
    if (!(o.gpsSigma > 0)) throw new IllegalArgumentException("--gps-sigma must be positive");
    validateGpsAdjustment(o);
    choice(o.rectificationAlpha, "--rectification-alpha", "auto", "0", "1");
    choice(
        o.alignmentQuality,
        "--alignment-quality",
        "same",
        "custom",
        "ultra",
        "high",
        "medium",
        "low");
    choice(o.intrinsicsMode, "--intrinsics-mode", "fixed", "prior", "initial");
    for (String mode : o.intrinsicsModes.values())
      choice(mode, "--intrinsics-mode CAMERA=MODE", "fixed", "prior", "initial", "free");
    choice(o.cameraGrouping, "--camera-grouping", "auto", "single", "per-image");
    if (o.cameraGroupsFile != null && !Files.isRegularFile(o.cameraGroupsFile))
      throw new IllegalArgumentException("--camera-groups file does not exist: " + o.cameraGroupsFile);
    for (var e : o.calibrationFiles.entrySet())
      if (!Files.exists(e.getValue()))
        throw new IllegalArgumentException(
            "--calibration " + e.getKey() + "=FILE: file does not exist: " + e.getValue());
    for (var e : o.orientationPixelSizes.entrySet())
      if (!(e.getValue() > 0))
        throw new IllegalArgumentException("--orientation-pixel-size " + e.getKey() + "=MM must be positive");
    for (String camera : o.focalByCamera.keySet())
      if (!(o.focalByCamera.get(camera) > 0)
          || !(o.sensorWidthByCamera.getOrDefault(camera, o.sensorWidth) > 0))
        throw new IllegalArgumentException(
            "--focal " + camera + "=MM needs a positive focal and --s_w " + camera + "=MM (or a plain --s_w)");
    for (String camera : o.sensorWidthByCamera.keySet())
      if (!(o.sensorWidthByCamera.get(camera) > 0)
          || !(o.focalByCamera.getOrDefault(camera, o.focal) > 0))
        throw new IllegalArgumentException(
            "--s_w " + camera + "=MM needs a positive width and --focal " + camera + "=MM (or a plain --focal)");
    choice(o.extrinsicsMode, "--extrinsics-mode", "fixed", "prior", "initial");
    choice(
        o.rerunFrom,
        "--rerun-from",
        "none",
        "calibration",
        "features",
        "matches",
        "geometry",
        "stereo",
        "fusion");
    for (String p : o.refineIntrinsics.split(","))
      choice(p, "--refine-intrinsics", "f", "k1", "k2", "k3", "cx", "cy", "p1", "p2");
    if (o.tileSize < 32
        || o.tileOverlap < 2
        || o.tileOverlap > o.tileSize
        || o.pyramidLevels < 0
        || o.pyramidLevels > 6
        || o.pyramidRadius < 4
        || o.pairCandidates < 1
        || o.cleanupPasses < 0
        || o.trackMinViews < 2
        || !(o.trackMaxError > 0)
        || o.trackMinAngle < 0
        || o.trackMinAngle >= 90
        || o.alignmentMaxDimension < 0
        || o.alignmentMaxDimension > 0 && o.alignmentMaxDimension < 32
        || !(o.focalSigma > 0 && o.radialSigma > 0 && o.positionSigma > 0 && o.rotationSigma > 0))
      throw new IllegalArgumentException(
          "Invalid alignment, uncertainty, cleanup, tiling or pyramid settings");
    if (o.rectified && (!o.intrinsicsMode.equals("fixed") || !o.extrinsicsMode.equals("fixed")))
      throw new IllegalArgumentException(
          "Already rectified input requires fixed calibration and stereo poses");
    if (o.calibrate && o.estimateMemory)
      throw new IllegalArgumentException(
          "--estimate-memory applies to reconstruction, not --calib");
    if (o.correlation < -1
        || o.correlation > 1
        || o.lrThreshold <= 0
        || o.speckleRange < 0
        || o.speckleSize < 0
        || o.minAngle < 0
        || o.minAngle >= 90
        || o.minViews < 2
        || o.fusionRelative <= 0
        || o.fusionRelative > .5
        || o.fusionPixels <= 0
        || o.fusionConsensus <= .5
        || o.fusionConsensus > 1
        || o.fusionAgreementRadius < -1
        || o.fusionAgreementRadius > 8
        || o.maxDepthUncertainty <= 0
        || o.maxDepthUncertainty > 1
        || o.correlationWindow < 3
        || o.correlationWindow > 15
        || o.correlationWindow % 2 == 0)
      throw new IllegalArgumentException("Invalid stereo confidence or fusion settings");
    if (!o.censusMode.equals("gray") && !o.censusMode.equals("color-distance"))
      throw new IllegalArgumentException("--census-mode must be gray or color-distance");
    if (o.input == null || !Files.isDirectory(o.input))
      throw new IllegalArgumentException("-i must name an image directory");
    if (o.calibration != null && !Files.exists(o.calibration))
      throw new IllegalArgumentException("Calibration file does not exist");
    if (o.poses != null && !Files.exists(o.poses))
      throw new IllegalArgumentException("Pose CSV does not exist");
    if (Double.isNaN(o.focal) != Double.isNaN(o.sensorWidth)
        || (!Double.isNaN(o.focal) && (!(o.focal > 0) || !(o.sensorWidth > 0))))
      throw new IllegalArgumentException(
          "--focal MM and --s_w SENSOR_WIDTH_MM must be supplied together and positive");
    if (o.gps && o.noGps) throw new IllegalArgumentException("--gps and --no-gps conflict");
    if (!(o.gpsThreshold > 0))
      throw new IllegalArgumentException("--gps-threshold must be positive metres");
    if (!o.noGps && !o.calibrate) new Crs(o.crs);
    if (o.rectified && o.poses != null)
      throw new IllegalArgumentException("--rectified and --poses are separate modes");
    if (o.seedPair != null && (o.rectified || o.poses != null))
      throw new IllegalArgumentException("--seed-pair applies to image-only SfM");
    if (o.p1 != Math.rint(o.p1) || o.p2 != Math.rint(o.p2))
      throw new IllegalArgumentException("SGM penalties must be integers");
    if (o.disparities < 3
        || o.disparities > 4096
        || (long) o.minDisparity + o.disparities > Integer.MAX_VALUE)
      throw new IllegalArgumentException(
          "--disparities must be 3..4096 and its upper bound must fit an integer");
    if (o.census < 3 || o.census > 11 || o.census % 2 == 0)
      throw new IllegalArgumentException("Census window must be odd, 3..11");
    if (o.median < 1 || o.median > 11 || o.median % 2 == 0)
      throw new IllegalArgumentException("Median window must be odd, 1..11");
    if (!(o.p1 >= 0 && o.p2 > o.p1 && o.p2 <= 1000000)
        || !(o.ratio > 0 && o.ratio < 1)
        || !(o.uniqueness >= 0 && o.uniqueness < 1))
      throw new IllegalArgumentException(
          "Require 0 <= p1 < p2, 0 < ratio < 1, 0 <= uniqueness < 1");
    if (o.directions != 4 && o.directions != 8)
      throw new IllegalArgumentException("Directions must be 4 or 8");
    if (!(o.baseline > 0 && o.scale > 0 && o.reprojection > 0 && o.maxDepth > 0 && o.voxel >= 0)
        || o.maxDimension < 32
        || o.features < 100
        || o.minMatches < 8
        || o.neighbors < 1
        || o.iterations < 0
        || o.calibrationTrialIterations < 0
        || o.step < 1
        || o.cores < 1
        || o.memoryMb < 16
        || o.memoryMb > Long.MAX_VALUE / 1048576L
        || o.ramLimitMb < 0 || o.ramLimitMb > Long.MAX_VALUE / 1048576L
        || o.boardColumns < 2
        || o.boardRows < 2
        || !(o.squareSize > 0))
      throw new IllegalArgumentException(
          "Invalid positive range, iteration count, or calibration board size");
    choice(o.depthRefinementMethod, "--depth-refinement", "reprojection", "weighted");
    choice(o.surfaceConsolidation, "--surface-consolidation", "normal", "none");
    PointFields.parse(o.pointAttributes);
    if (o.pointEvidence() && (!o.fusionMode.equals("image") || !o.fusionEmission.equals("pair")))
      throw new IllegalArgumentException("--point-attributes angle, depth_spread, height_sigma and ray_sigma need --fusion-mode image"
          + " with --fusion-emission pair");
    choice(o.surfaceValidation, "--surface-validation", "auto", "cpu", "gpu");
    if (!(o.surfaceRadius >= .5
            && o.surfaceRadius <= 8
            && o.surfaceSpacing > 0
            && o.surfaceSpacing <= o.surfaceRadius
            && o.surfaceMaxShift > 0
            && o.surfaceMaxShift <= 10
            && o.surfaceNormalAngle > 0
            && o.surfaceNormalAngle <= 60)
        || o.surfaceMemoryMb < 8)
      throw new IllegalArgumentException(
          "Invalid surface radius/spacing/shift/normal angle or memory budget");
    if (!(o.surfaceFlatten >= 0 && o.surfaceFlatten <= 16))
      throw new IllegalArgumentException("--surface-flatten must be 0..16 footprints");
    choice(o.fusionTolerance, "--fusion-tolerance", "relative", "sigma");
    choice(o.fusionSeedOrder, "--fusion-seed-order", "camera", "precision");
    if (!(o.fusionSigmaK >= 1 && o.fusionSigmaK <= 10)
        || !(o.fusionSeedMinAngle >= 0 && o.fusionSeedMinAngle < 60)
        || !(o.fusionLocalSigma == 0 || o.fusionLocalSigma >= 1 && o.fusionLocalSigma <= 100))
      throw new IllegalArgumentException(
          "--fusion-sigma-k must be 1..10, --fusion-seed-min-angle 0..<60 degrees,"
              + " --fusion-local-sigma 0 (off) or 1..100");
    if (o.thinSurfaceFusion() && !o.fusionMode.equals("image"))
      throw new IllegalArgumentException(
          "--thin-surfaces and --fusion-tolerance sigma, --fusion-precision-check on,"
              + " --fusion-seed-order precision, --fusion-seed-min-angle and --fusion-local-sigma"
              + " require --fusion-mode image");
    return o;
  }

  private static void validateGpsAdjustment(Options o) {
    if (!o.gpsIntrinsics.equals("none"))
      for (String name : o.gpsIntrinsics.split(","))
        if (!Set.of("f", "k1", "k2", "k3", "cx", "cy", "p1", "p2").contains(name))
          throw new IllegalArgumentException(
              "--gps-intrinsics must be none or a comma list of f,k1,k2,k3,cx,cy,p1,p2");
    if (!Set.of("auto", "none").contains(o.gpsFocalPrior) && !(number(o.gpsFocalPrior) > 0))
      throw new IllegalArgumentException("--gps-focal-prior must be auto, none or focal pixels");
    if (!(o.gpsFocalSigma > 0))
      throw new IllegalArgumentException("--gps-focal-sigma must be positive");
    if (!Set.of("estimate", "off").contains(o.gpsTimeOffset)
        && !(Math.abs(number(o.gpsTimeOffset)) <= 2))
      throw new IllegalArgumentException(
          "--gps-time-offset must be estimate, off or -2..2 seconds");
    if (!Set.of("estimate", "off").contains(o.gpsLeverArm)) {
      String[] parts = o.gpsLeverArm.split(",");
      if (parts.length != 3)
        throw new IllegalArgumentException("--gps-lever-arm must be estimate, off or x,y,z metres");
      for (String p : parts)
        if (!(Math.abs(number(p)) <= 2))
          throw new IllegalArgumentException("--gps-lever-arm components must be within 2 metres");
    }
    if (!Set.of("estimate", "off").contains(o.rollingShutter)
        && !(Math.abs(number(o.rollingShutter)) <= .25))
      throw new IllegalArgumentException(
          "--rolling-shutter must be estimate, off or a readout time of -0.25..0.25 seconds");
    if (!o.gpsHeightSigma.equals("auto") && !(number(o.gpsHeightSigma) > 0))
      throw new IllegalArgumentException("--gps-height-sigma must be auto or positive metres");
    if (o.groundFusionRadius < 0 || o.groundFusionRadius > 10)
      throw new IllegalArgumentException("--ground-fusion-radius must be auto or 0.05..10 metres");
    if (o.groundCell < 0 || o.groundCell > 10)
      throw new IllegalArgumentException("--ground-cell must be auto or 0.01..10 metres");
    if (!(o.groundMaxSlope > 0 && o.groundMaxSlope < 85))
      throw new IllegalArgumentException("--ground-max-slope must be 0..85 degrees");
    if (o.gcp != null && !Files.isRegularFile(o.gcp))
      throw new IllegalArgumentException("--gcp file not found: " + o.gcp);
    if (!(o.gcpSigma > 0) || !(o.gcpHeightSigma > 0))
      throw new IllegalArgumentException("--gcp-sigma and --gcp-height-sigma must be positive");
    if (!o.gcpCheck.matches("none|auto|[1-9][0-9]{0,5}"))
      throw new IllegalArgumentException("--gcp-check must be none, auto or a number of check points");
    if (o.gcpPort < 1 || o.gcpPort > 65535)
      throw new IllegalArgumentException("--gcp-port must be 1..65535");
    if (o.lidarReference != null && !Files.exists(o.lidarReference))
      throw new IllegalArgumentException("--lidar-reference not found: " + o.lidarReference);
    if (!(o.lidarSearchShift >= 0 && o.lidarSearchShift <= 50))
      throw new IllegalArgumentException("--lidar-search-shift must be 0..50 metres");
    if (!(o.lidarSearchAngle >= 0 && o.lidarSearchAngle <= 15))
      throw new IllegalArgumentException("--lidar-search-angle must be 0..15 degrees");
    if (!(o.lidarCell >= .25 && o.lidarCell <= 20))
      throw new IllegalArgumentException("--lidar-cell must be 0.25..20 metres");
    if (!(o.lidarCanopyWeight >= 0 && o.lidarCanopyWeight <= 1))
      throw new IllegalArgumentException("--lidar-canopy-weight must be 0..1");
    if (!(o.lidarOutlier >= .1 && o.lidarOutlier <= 50) || !(o.lidarChangeLimit >= .1 && o.lidarChangeLimit <= 50))
      throw new IllegalArgumentException("--lidar-outlier and --lidar-change-limit must be 0.1..50 metres");
    if (!(o.lidarWeakLimit > 0 && o.lidarWeakLimit <= 10))
      throw new IllegalArgumentException("--lidar-weak-limit must be above 0 and at most 10 metres");
    if (!(o.lidarMargin >= 0 && o.lidarMargin <= 1000))
      throw new IllegalArgumentException("--lidar-margin must be 0..1000 metres");
    if (!(o.groundMaxObject >= 1 && o.groundMaxObject <= 500))
      throw new IllegalArgumentException("--ground-max-object must be 1..500 metres");
    if (o.gpsAdjustmentRounds < 1 || o.gpsAdjustmentRounds > 20)
      throw new IllegalArgumentException("--gps-adjustment-rounds must be 1..20");
  }

  /** --bundle-solver fast: the fast value of every bundle switch not given explicitly. */
  void applyFastBundleSolver(Set<String> explicit) {
    if (!explicit.contains("--bundle-direct-limit")) bundleDirectLimit = FAST_BUNDLE_DIRECT_LIMIT;
    if (!explicit.contains("--bundle-preconditioner")) bundlePreconditioner = "cluster-jacobi";
    if (!explicit.contains("--bundle-inexact-steps")) bundleInexactSteps = true;
    // --registration-local-bundles and --lens-fit-stop parameters are not part of fast: on a
    // 487-image block local bundles cost time and moved weak cameras without registering more,
    // and with exact (dense) steps the lens fits converge before the parameter rule applies.
  }

  private static boolean onOff(String v, String name) {
    return switch (v.toLowerCase(Locale.ROOT)) {
      case "on" -> true;
      case "off" -> false;
      default -> throw new IllegalArgumentException(name + " requires on|off");
    };
  }

  /** v names an existing file or directory (false for text that is not a valid path here). */
  private static boolean existingPath(String v) {
    try {
      return Files.exists(Paths.get(v));
    } catch (java.nio.file.InvalidPathException e) {
      return false;
    }
  }

  /**
   * CAMERA=VALUE of the per-camera option forms: {camera, value}, or null for a plain value. The
   * camera ID may not contain path separators, so a Windows drive path is never taken for one.
   */
  static String[] cameraValue(String v) {
    int eq = v.indexOf('=');
    if (eq <= 0 || eq == v.length() - 1) return null;
    String camera = v.substring(0, eq).strip();
    if (camera.isEmpty() || camera.contains("/") || camera.contains("\\") || camera.contains(":"))
      return null;
    return new String[] {camera, v.substring(eq + 1).strip()};
  }

  private static <T> Map<String, T> put(Map<String, T> map, String camera, T value, String flag) {
    Map<String, T> out = new LinkedHashMap<>(map);
    if (out.put(camera, value) != null)
      throw new IllegalArgumentException(flag + " " + camera + "=... is given twice");
    return Collections.unmodifiableMap(out);
  }

  private static double number(String s) {
    double x = Double.parseDouble(s);
    if (!Double.isFinite(x)) throw new IllegalArgumentException("Expected a finite number: " + s);
    return x;
  }

  private static void choice(String value, String flag, String... allowed) {
    if (!Arrays.asList(allowed).contains(value))
      throw new IllegalArgumentException(flag + " must be one of " + String.join(", ", allowed));
  }

  Options alignmentOptions() {
    Options a = copy();
    a.denseOptions = denseOptions != null ? denseOptions : this;
    if (!alignmentQuality.equals("same")) {
      a.quality = alignmentQuality;
      a.maxDimension = 1600;
      a.maxDimensionExplicit = false;
    }
    if (alignmentMaxDimension > 0) {
      if (alignmentQuality.equals("same")) a.quality = "custom";
      a.maxDimension = alignmentMaxDimension;
      a.maxDimensionExplicit = true;
    }
    return a;
  }

  private void applyFilteringPreset(Set<String> explicit) {
    double relative, pixels, consensus, minimumCorrelation;
    int minimumSpeckle;
    switch (filtering) {
      case "mild":
        relative = .01;
        pixels = 1.25;
        consensus = .55;
        minimumCorrelation = .45;
        minimumSpeckle = 20;
        break;
      case "moderate":
        relative = .005;
        pixels = .75;
        consensus = .6;
        minimumCorrelation = .55;
        minimumSpeckle = 50;
        break;
      case "aggressive":
        relative = .0025;
        pixels = .5;
        consensus = .75;
        minimumCorrelation = .65;
        minimumSpeckle = 100;
        break;
      default:
        throw new IllegalArgumentException("--filtering must be mild, moderate or aggressive");
    }
    if (!explicit.contains("--fusion-relative")) fusionRelative = relative;
    if (!explicit.contains("--fusion-pixels")) fusionPixels = pixels;
    if (!explicit.contains("--fusion-consensus")) fusionConsensus = consensus;
    if (!explicit.contains("--min-correlation")) correlation = minimumCorrelation;
    if (!explicit.contains("--speckle-size")) speckleSize = minimumSpeckle;
    filteringOverrides = String.join(",", explicit);
  }

  String filteringSummary() {
    return String.format(
        Locale.ROOT,
        "Point filtering=%s; fusion-relative=%s, fusion-pixels=%s, fusion-consensus=%s,"
            + " min-correlation=%s, speckle-size=%d; explicit overrides=%s",
        filtering,
        fusionRelative,
        fusionPixels,
        fusionConsensus,
        correlation,
        speckleSize,
        filteringOverrides.isEmpty() ? "none" : filteringOverrides);
  }

  /** Whether any --thin-surfaces image-fusion part is enabled (all off reproduces the old output). */
  boolean thinSurfaceFusion() {
    return fusionTolerance.equals("sigma")
        || fusionPrecisionCheck
        || fusionSeedOrder.equals("precision")
        || fusionSeedMinAngle > 0
        || fusionLocalSigma > 0;
  }

  int qualityDownsample() {
    switch (quality) {
      case "ultra":
        return 1;
      case "high":
        return 2;
      case "medium":
        return 4;
      case "low":
        return 8;
      case "custom":
        return 1;
      default:
        throw new IllegalArgumentException("--quality must be ultra, high, medium, low or custom");
    }
  }

  int[] processedSize(int width, int height) {
    double factor = 1. / qualityDownsample();
    if (quality.equals("custom") || maxDimensionExplicit)
      factor = Math.min(factor, maxDimension / (double) Math.max(width, height));
    factor = Math.min(1, factor);
    return new int[] {
      Math.max(1, (int) Math.round(width * factor)), Math.max(1, (int) Math.round(height * factor))
    };
  }

  Options copy() {
    Options out = new Options();
    out.cacheDirectory = cacheDirectory;
    out.outputDiskSpace = outputDiskSpace;
    out.runFeatureDirectory = runFeatureDirectory;
    out.runFeatureHits = runFeatureHits;
    out.projectDirectory = projectDirectory;
    out.orientationCalibrations = orientationCalibrations;
    out.cameraGroups = cameraGroups;
    out.importedCameraIds = importedCameraIds;
    out.importedPixelSizes = importedPixelSizes;
    out.importedCalibrations = importedCalibrations;
    out.orientationSourceUnits = orientationSourceUnits;
    out.orientationNotes = orientationNotes;
    out.stereoReportDirectory = stereoReportDirectory;
    out.gpuHostReserve = gpuHostReserve;
    out.stereoSelection = stereoSelection;
    out.tsgmSeeds = tsgmSeeds;
    out.gpuSmallSolves = gpuSmallSolves;
    out.mvsgmNeighbors = mvsgmNeighbors;
    out.mvsgmBestK = mvsgmBestK;
    out.mvsgmCensus = mvsgmCensus;
    out.mvsgmLevels = mvsgmLevels;
    out.mvsgmConsistentViews = mvsgmConsistentViews;
    out.mvsgmConsistencyLabels = mvsgmConsistencyLabels;
    out.mvsgmCamerasInFlight = mvsgmCamerasInFlight;
    out.mvsgmTile = mvsgmTile;
    try {
      for (java.lang.reflect.Field field : Options.class.getFields())
        field.set(out, field.get(this));
    } catch (IllegalAccessException e) {
      throw new IllegalStateException(e);
    }
    return out;
  }

  /**
   * The full help, or with a topic only the option entries (an option line and its indented
   * description) whose text contains the topic, ignoring case.
   */
  static String help(String topic) {
    String full = help();
    if (topic == null || topic.isBlank()) return full;
    String needle = topic.toLowerCase(Locale.ROOT);
    StringBuilder out = new StringBuilder(), entry = new StringBuilder();
    int found = 0;
    for (String line : (full + "\n").split("\n", -1)) {
      boolean start = line.startsWith("  -") || line.startsWith("  photogrammetry ");
      boolean continuation = line.startsWith("      ") || line.startsWith("                ");
      if ((start || !continuation) && entry.length() > 0) {
        if (entry.toString().toLowerCase(Locale.ROOT).contains(needle)) {
          out.append(entry);
          found++;
        }
        entry.setLength(0);
      }
      if (start || (continuation && entry.length() > 0)) entry.append(line).append('\n');
    }
    return found == 0
        ? "No option mentions \"" + topic + "\". Run with --help for the full list.\n"
        : "Options mentioning \"" + topic + "\" (" + found + "; --help alone lists all):\n" + out;
  }

  public static String help() {
    return String.join(
        "",
        """
POSE - Photogrammetric Orientation and Surface Estimation (Java/OpenCL)
Usage: photogrammetry -i IMAGE_DIR [-o PROJECT_DIR] [options]
       (pose.bat / pose.sh start the same program as photogrammetry.bat / .sh)
Run with no arguments, -h, --help or -help to display this help and exit.
--help TEXT shows only the options whose description mentions TEXT (e.g. --help census).
Values accept --option VALUE or --option=VALUE. Quote paths containing spaces.
Defaults below apply unless overridden; pixel units refer to processed images.

Pipeline steps (run one by one, or chained; the state is saved after each step):
  photogrammetry -i IMAGES -o PROJECT [options]
      All steps in one process (as before).
  photogrammetry STEP [STEP ...] -o PROJECT [options]
      Only the named steps, in order: inputs, calibrate, match, align, cleanup,
      georeference, lidar, gcp, dense, ground, finish. 'inputs' starts a new run (needs
      -i); the other steps continue the latest run with the options saved by 'inputs';
      options given here are added and override them. 'finish' writes the report and
      final files. 'cleanup', 'lidar', 'gcp' and 'ground' are optional: without
      --tie-point-cleanup, --lidar-reference, --gcp or ground options they need not be
      named and are skipped when a later step runs. With --gcp, 'gcp' ends as
      'waiting' until enough markers are pinned: pin them with gcp-edit, then run gcp
      again. With
      --tie-point-cleanup pause, 'cleanup' ends as 'waiting' until the limits are chosen
      in the POSE window (or the step is run again with --tie-point-cleanup auto|off).
  --run ID
      Continue (or branch from) this run instead of the latest one.
  --branch
      Start a new run from the given run's saved state just before the first named
      step and redo from there (e.g. 'dense ground finish --branch --min-views 2').
      The source run is not changed.
  photogrammetry status -o PROJECT [--run ID]
      Show which steps of a run are done and the next one.
  photogrammetry gui [PROJECT]
      Open the POSE desktop window (also pose-gui.bat / pose-gui.sh): start runs with
      the common options as fields, follow their steps and log, manage the runs of a
      project (continue, branch, GCP markers, report again). It starts this same
      command line for the processing. Needs a display; see GUI.md.
  photogrammetry gcp-edit -o PROJECT [--run ID] [--browser] [--no-continue] [--gcp-port N]
      Open the ground control point editor in a window. Click a GCP, then click its
      exact position in an image; the other images are then refined by patch matching
      (blue markers: check and accept with A, or accept all). Images whose view of the
      GCP is blocked by the tie points (canopy, roofs) are listed as probably hidden.
      Markers are saved as you go to 04_geometry/gcp/markers.json. Done continues the
      run with the gcp step once enough markers are pinned (not with --no-continue).
      --browser (or a machine without a display) uses the local web page instead
      (http://127.0.0.1:PORT); there, press Done, then run the gcp step again.

Inputs and project output:
  -h, --help, -help
      Show this help without starting a project or loading images.
  --verbose
      Print every processing message to the terminal (the previous behaviour). Default:
      one progress bar per stage with percent, counts, elapsed and estimated remaining
      time, the current item and live RAM/CPU/GPU; a summary line as each stage ends;
      warnings and errors always shown. logs/console.log always keeps the full text.
  -i, --images DIR
      Input image directory (required for reconstruction/calibration, not standalone
      help/GPU listing/export/analysis); files sort by filename.
  -o, --output DIR
      Project directory (default: photogrammetry-output). Each invocation saves
      a new runs/<run-id>/ directory and preserves previous runs.
  -f, --format LIST
      Comma-separated input extensions (default: jpg,jpeg,png,tif,tiff,bmp).
  --copy-inputs
      Also archive full-resolution originals with EXIF (default: disabled).
      Input provenance is always saved; processed image exports require --diagnostics.
  --overwrite
      Adopt a nonempty directory as a project; unrelated files and prior runs
      are preserved. Existing POSE projects do not require this flag.

Calibration and reconstruction modes:
  --calibration FILE, -d FILE, --dist FILE
      Supplied camera properties or legacy 14-number calib.result; other formats
      are selected with --calibration-format below.
      Describes original raster intrinsics/distortion; fixed by default. See
      --intrinsics-mode to refine selected supplied parameters.
      Without calibration/focal inputs, initialize focal from images or EXIF;
      constrained parameter-wise refinement follows initialization, and alignment
      uses the same data-support assessment.
      --calibration CAMERA=FILE (repeatable) supplies one camera's calibration; see
      "several calibrated cameras" below.
  --focal MM, --focal CAMERA=MM
      Focal length in millimetres; requires --s_w. Initializes focal in pixels.
  --s_w MM, --s_w CAMERA=MM
      Physical sensor WIDTH in millimetres (not diagonal); requires --focal.
      The CAMERA= forms apply to one camera group; plain values to every group
      without its own calibration.
  --camera-grouping auto|single|per-image
      Which images share one interior orientation (default auto: camera IDs of an
      imported orientation, else --camera-groups, else EXIF make, model, serial and
      focal mm). single: one camera; per-image: every image its own camera. Images
      of different pixel sizes are always separate cameras. Groups and their
      calibrations: 01_calibration/cameras.json; with several cameras also
      camera_ID.txt.
  --camera-groups FILE
      CSV image,camera that names the camera of every image (applies with auto).
  --calibration CAMERA=FILE, --orientation-input FILE: several calibrated cameras
      Each camera keeps its own calibration.
      Camera IDs. With --camera-groups FILE they are your names; with EXIF grouping
      they are camera_1, camera_2, ... in the order of each camera's first image. A run
      prints them with make, model and focal and lists them in
      01_calibration/cameras.json; an unknown CAMERA in an option is an error that
      lists them.
      Whole orientation in one file, cameras named in it: --orientation-input FILE.
        INPHO .prj: one $CAMERA_DEFINITION per camera, each photo refers to one with
          $CAMERA_ID; pixel size from the file, else --orientation-pixel-size
          CAMERA=MM. Metashape cameras.xml: one <sensor> per camera (ID = its label
          when the labels are unique, else sensor_N). COLMAP: CAMERA_ID. openMVG:
          id_intrinsic. OpenSfM: camera key. POSE native/CSV: camera field/column.
          Bundler has no camera IDs: put camera_groups.csv (image,camera) beside it or
          give --camera-groups. A file that gives every photo its own camera (common
          in exports) carries no grouping; then EXIF or --camera-groups decides.
      One calibration file per camera: --calibration CAMERA=FILE, repeated. FILE may
        be any calibration format holding one camera: POSE camera.txt, the 14-number
        calib.result, OpenCV YAML/XML, Metashape <calibration>, MicMac, or a COLMAP,
        INPHO or other orientation file with one camera (format: auto, or
        --calibration-format; INPHO pixel size: --orientation-pixel-size CAMERA=MM).
        Each camera's file is in that camera's own pixels. Cameras without a file
        start from --focal/--s_w CAMERA=MM, else from the images (EXIF focal with
        fewer than 3 images), and are calibrated automatically.
      Example: --camera-groups cams.csv --calibration rgb=rgb.yaml
        --calibration nir=nir_camera.txt --intrinsics-mode nir=prior
      Supplied cameras are fixed by default; --intrinsics-mode CAMERA=prior|initial
      refines one, CAMERA=free ignores its supplied values and calibrates it.
  --no-self-calibration
      Disable bootstrap and alignment self-calibration; retain initialized or supplied intrinsics.
  --poses FILE
      Known poses; native CSV uses X_camera = R * X_world + t. Other formats below.
      Every input needs a pose. Fixed by default; see --extrinsics-mode. Units come
      from the translations. Cannot combine with --rectified or --seed-pair.
  --rectified
      Exactly two already-rectified images, sorted left then right. Bypasses
      undistortion/SfM; uses supplied or derived intrinsics and --baseline.
  --baseline N
      Seed camera-center distance or rectified stereo baseline (default: 1).
      Establishes local scale when measured; otherwise units are arbitrary.
  --seed-pair A.jpg,B.jpg
      Explicit image-only SfM seed; sorted filenames must form a viable pair
      within the selected pair graph. Otherwise a suitable seed is selected automatically.

Block processing (large image sets; whole-run command only, -i IMAGES -o PROJECT):
  --block-processing off|auto|on
      Split the images into overlapping blocks by GPS position and process them one after
      another as ordinary sub-projects under PROJECT/blocks (default: off). auto splits only
      when the memory forecast (alignment and image fusion, from the image count, --features,
      --quality and the RAM free on this machine) says the whole set does not fit; on always
      splits. The central block is aligned first and its final calibration is used, fixed, by
      every block. Blocks are then aligned to each other with one 3D similarity each, from the
      cameras they share, and merged: each point is kept by the block whose own camera is
      nearest in plan view. Results: PROJECT/merged/dense.las, cameras.csv, block_report.md.
      Repeat the same command to resume after an interruption (finished blocks are skipped);
      'photogrammetry status -o PROJECT' shows the blocks. Requires GPS in every image.
  --block-images N
      Core images per block (default: 0 = from the memory forecast). Each block also takes
      the images within --block-overlap footprints of its core.
  --block-overlap F
      Overlap margin around each block's core, in image ground footprints (default: 1;
      0..20). More overlap: more shared cameras for aligning the blocks, more work per block.

Resolution and memory planning:
  --preset LIST
      Tested dense and fusion settings (multi-view SGM): a platform, drone or aircraft,
      and optionally a scene, forest or city (e.g. --preset drone,forest). A scene alone
      uses drone. Options given explicitly win over the preset's. From the 2026-11-21
      parameter study; aircraft is a tested large-format setup (no aerial reference yet).
  --quality NAME
      ultra, high, medium, low: original dimensions divided by 1, 2, 4, 8.
      Default custom: fit within --max-dimension. Never upsamples. Independent
      of filtering intensity; intrinsics scale with actual processed sizes.
  --max-dimension N
      Maximum processed long side in pixels (default: 1600 for custom quality;
      minimum: 32). An explicit value also caps any named quality preset.
  --estimate-memory
      Inspect image headers, print/save resolution and memory scenarios, then
      exit without calibration or reconstruction. May report over-budget plans.
  --memory-mb N
      Processing buffer budget in MiB (default: 2048; minimum: 16). An explicit
      value raises the CLI JVM heap when needed to ceil(N/0.65)+16 MiB, on Windows
      and Linux, before creating a project. Heap sizing is capped by detected host,
      Linux cgroup/Slurm and optional --ram-limit-mb limits, reserving native headroom.
      An oversized existing CLI heap is reduced by relaunching too.
      Embedded Main.run calls cannot resize their host JVM and retain the 65% cap.
      Native images, GPU buffers and other state add memory; this is not a total
      process RAM limit. Effective heap and processing allowance are logged.
  --ram-limit-mb N
      Optional total host-RAM planning ceiling in MiB (default: 0 = detect). Includes heap
      and native-memory headroom. The CLI limits heap to at most 65% of this and
      detected host/job RAM, and guards alignment allocations. This is not an OS
      hard RSS cap; use the scheduler/cgroup limit for enforcement. Embedded callers
      must start their JVM with a suitable -Xmx; Main.run cannot resize a live JVM.
  --cores N, -cores N
      OpenCV worker threads (default: half available processors, at least 1;
      this machine: @CORES@). Does not set the JVM heap or GPU execution.
  PHOTOGRAMMETRY_HEAP (launcher environment variable, not a CLI argument)
      Initial Java maximum heap, e.g. 8g (default: 8g). Explicit --memory-mb can
      raise it. Native memory is additional; requesting memory does not create RAM.

Feature matching and final bundle adjustment:
  --features N
      Requested maximum features per image (default: 8000; minimum: 100).
  --neighbors N
      Select each image's N strongest estimated-overlap neighbors (default: 3; minimum: 1).
      Match the union once; incoming choices can increase an image's total degree.
      With --pair-selection sequential, match the next N filename-sorted images.
      Increase for broader overlap; larger values cost time and retained memory.
  --min-matches N
      Minimum geometrically verified matches for a viable pair (default: 20;
      minimum: 8); also influences the camera-registration support threshold.
  --ratio N
      Descriptor nearest/second-nearest distance ratio (default: 0.75;
      0 < N < 1). Smaller values demand more distinctive matches.
  --reprojection N
      Geometric RANSAC tolerance and final bundle Huber threshold in pixels
      (default: 2; positive). Final sparse track rejection uses twice this value.
  --bundle-iterations N
      Fixed accepted LM iteration budget per pose/point fit. Default (option omitted):
      pose/point fit iterates until two successive iterations each improve the robust
      objective by less than 1e-6 relative, up to 100 iterations. Shared lens proposal
      fits use --calibration-trial-iterations independently. 0 skips pose-only iterations;
      --no-self-calibration separately disables lens refinement. The first pose
      and baseline norm anchor image-only geometry; pose priors anchor supplied
      refinement. Fixed supplied poses and rectified mode preserve their controls.
  --bundle-solver legacy|fast
      Linear solver of every bundle adjustment (alignment, lens fits, GPS and GCP
      adjustments). legacy (default) keeps results identical to earlier versions:
      direct solve up to 768 reduced variables (about 128 cameras), above that an
      iterative solve with a fixed 1e-6 tolerance, which on large blocks often needs
      hundreds of iterations per step. fast sets each of these switches that is not
      given explicitly: --bundle-direct-limit 6000, --bundle-preconditioner
      cluster-jacobi, --bundle-inexact-steps on (the last two act only above the
      direct limit). --registration-local-bundles and --lens-fit-stop stay off unless
      given: they brought no gain on a 487-image block, where fast aligned 17 to 18
      times faster than legacy. The same input and options give the same result with
      any --cores. fast reaches the same optimum within the solver tolerances (camera
      centres about 2 cm apart there), not bit-identical to legacy.
  --bundle-direct-limit legacy|N
      Largest reduced camera system (6 per free camera, plus 13 per camera group in
      lens fits) solved directly with a parallel dense Cholesky factorization (memory
      8 N^2 bytes, also bounded by --memory-mb / 8). Larger systems use the iterative
      solve. legacy (default): the earlier direct solve up to 768 variables.
  --bundle-preconditioner block-jacobi|cluster-jacobi
      Preconditioner of the iterative solve above the direct limit. block-jacobi
      (default): each camera's 6 x 6 block. cluster-jacobi: dense blocks of up to 64
      consecutive cameras with their mutual coupling, which needs fewer iterations.
  --bundle-inexact-steps off|on
      on: solve each iterative step only as accurately as its progress needs (a
      tolerance of 0.1 far from the optimum, tightening as the gradient shrinks,
      never below 1e-6), instead of always 1e-6 (off, default).
  --lens-fit-stop objective|parameters
      Stopping rule of shared-lens fits. objective (default): stop when two successive
      iterations each improve the objective by less than the convergence threshold.
      parameters: also stop once every free camera pose parameter and fitted lens
      parameter moved by less than 0.1 of its standard deviation in the last step, and
      the movement still to come (extrapolated from how fast the steps shrink) is below
      0.1 of it too. Needs the dense direct solve (--bundle-direct-limit); iterative
      steps use the objective rule only.
  --calibration-trial-iterations N
      Hard LM iteration cap per intrinsic-parameter proposal/focal start (default 12).
      Promising bounded proposals receive a convergence fit and repeat both holdout
      checks before acceptance; rejected proposals restore the preceding model.
      Final pose/point and GPS bundles keep their own convergence controls. 0 restores
      legacy proposal budgets (automatic convergence unless --bundle-iterations is set).
      Short proposals can change model selection; validate calibration and geometry.
  --calibration-trial-method full|subset|joint
      How the lens is refined after alignment (shared alignment calibration). full
      (default) adds one parameter per trial (f from three starts, then k1, k2, cx, cy,
      p1, p2, k3; a second sweep), each trial a bundle adjustment of the whole block,
      kept only if two held-out track groups improve and the data support it. The
      support check handles about 120 cameras per camera model: above that no trial
      can be accepted, so full skips them and keeps the starting lens.
      subset runs the same trials on a compact set of cameras near the block
      centre (--calibration-trial-cameras; a block of at most that many cameras is
      treated as with full), then fits the chosen model once to the
      whole block and keeps it only if both held-out groups of the whole block improve.
      joint drops the parameters without data support first (checked on the subset
      for a large block), fits the rest together in one whole-block adjustment, drops
      those that moved by less than three standard deviations and refits the rest,
      accepts them with the same held-out tests, else retries once with f,k1,k2.
      subset and joint are for large blocks (joint checks the whole block when it is
      within the limit). joint uses one focal start.
  --calibration-trial-cameras auto|N
      Cameras in the subset of --calibration-trial-method subset|joint (default auto:
      100), at most what the support check accepts (about 120 for one camera model).
  --calibration-acceptance strict|relaxed
      How a lens fit after alignment is accepted (not the GPS-weighted adjustment).
      strict (default): both held-out track groups must improve by 0.5% and 0.002 px,
      by more than a complexity penalty per parameter. relaxed: neither held-out group
      may get worse, and every fitted parameter must have moved by at least three
      data-only standard deviations. Both keep the observability, image coverage, prior
      bound and physical lens checks. Held-out scores cap each squared residual at a
      gate fixed from the starting lens, so gross mismatches do not decide.

Dense stereo / SGM:
  --disparities N
      Explicit fixed hypothesis count, 3..4096; minimum defaults to 0 in this mode.
      Omit to estimate both bounds per pair from verified keypoints/sparse geometry,
      validate with a coarse whole-range probe, and expand either end when needed.
  --min-disparity N
      Pin the lower search bound (default: automatic per pair; signed values
      allowed). Without this option or
      --disparities, estimate a nonnegative minimum automatically for each pair.
      Explicit --min-disparity 0 keeps a zero-based search with automatic upper
      estimation. Search ends at minimum + count - 1; positive depth is required.
  --census-mode NAME
      gray (default): center-relative grayscale census; color-distance: census
      using RGB-distance texture. Select the matching cost separately below.
  --matching-cost census|ad-census|census-gradient|zncc
      Default census is descriptor Hamming distance. ad-census combines census
      with clipped absolute grayscale difference; census-gradient uses clipped
      horizontal/vertical gradient difference. zncc uses windowed zero-mean
      normalized correlation. All have the same 0..(window*window-1) cost scale
      on CPU/GPU. Low-texture ZNCC windows are invalid. This controls dense
      stereo, independently of the sparse --matching-algorithm.
  --census-weight N
      Census share in hybrid costs (default: 0.5; 0..1, quantized to 0.001).
  --ad-truncation N
      Grayscale difference saturation for ad-census (default: 30; 1..255).
  --gradient-truncation N
      Sum-of-axis gradient difference saturation (default: 60; 1..1020).
  --subpixel-refinement none|zncc
      Default none uses the aggregated-cost parabola. Optional zncc refines
      stable non-edge matches within +/-0.75 pixels using grayscale correlation,
      before left/right checks. Can reduce pixel-locking/ripple on textured planes;
      costs CPU time and does not repair inaccurate calibration or orientation.
  --pairwise-search full|tsgm
      Default full searches every pair's whole disparity range. tsgm runs coarse-to-fine
      SGM: a 2x pyramid, then per-pixel disparity bands derived from the coarser level and
      sparse tie points; paths stay continuous across the image. Faster, smaller volumes;
      the runner-up for confidence is limited to the band.
  --on-pair-failure abort|skip
      Default abort stops the run when a stereo pair fails. skip reports the pair in
      05_stereo/skipped_pairs.json and continues (still aborts if every pair fails, or if
      the pair had already written candidates).
  --fusion-gpu-failure disable|batch
      After a GPU fusion batch fails, its candidates are replayed on CPU. Default disable
      keeps the CPU for the rest of the run; batch retries the GPU on later batches and
      disables it after three consecutive failures. Fallbacks are listed in backend.json.
  --subpixel-fit parabola|equiangular
      Aggregated-cost subpixel interpolation (default: parabola). equiangular fits a
      symmetric V, which suits Census/L1-like costs and reduces pixel-locking.
  --tsgm-levels N
      Coarser pyramid levels for tsgm (default: 3; 0..5). 0 = banded engine with full bands
      (diagnostic; identical to full search).
  --tsgm-margin N
      Base band half-margin in pixels at each finer level (default: 3; 1..32).
  --similarity-window N, --ws_census N
      Dense Census/ZNCC similarity window side in pixels (default: 5; odd, 3..11). Larger windows
      gather more context but can blur depth boundaries and increase memory.
  --ws_mFilter N
      Valid-disparity median window side (default: 3; odd, 1..11; 1 disables).
  --median-mode edge-preserving|classic
      Default edge-preserving uses an ordinary median on smooth surfaces, but
      restricts support to the center depth/intensity layer when that median
      would jump layers. classic always uses every valid neighbor.
  --median-max-change N
      Maximum neighbor disparity difference for the edge-preserving median,
      and largest ordinary median shift before restricting support, in processed
      pixels (default: 0.5; nonnegative). Does not fill holes or
      bypass subsequent left/right, correlation or speckle rejection.
  --median-edge-threshold N
      Maximum grayscale difference from the center in the edge-preserving median
      (default: 24; 0..255, 0 disables the intensity gate). Preserves image edges
      even where the foreground/background disparity jump is only one pixel.
  --directions N
      SGM aggregation paths: 4 or 8 (default: 8). Four paths reduce computation.
  --p1 N
      Integer SGM penalty for a one-hypothesis disparity change (default: 4;
      nonnegative). This is a stereo penalty, not lens tangential distortion.
  --p2 N
      Integer SGM penalty for larger disparity changes (default: 24; greater
      than p1, at most 1000000). Higher penalties favor smoother disparities.
      Both penalties are specified in 5x5-census units and scaled by
      (window*window-1)/24 for other windows, consistently across all costs.
  --sgm-edge-scale N
      Intensity scale for reducing P2 across image edges (default: 32; 0..255).
      Effective P2=max(scaled P1+1, floor(scaled P2*N/(N+intensity difference))).
      N=0 restores constant P2. Applies per path to grayscale values, including
      reverse matching and streamed strip boundaries, identically on CPU/GPU.
  --census-edge-mask T
      Opt-in edge-limited ("weighted") census, default 0 = off. Only census comparisons
      whose neighbour lies within T grey levels of the window centre in the reference
      image are counted; the Hamming cost is rescaled to the full bit count. The window
      then stops at intensity edges, which reduces foreground fattening at depth jumps
      (roof eaves, walls). Windows keeping fewer than a quarter of their comparisons
      use the ordinary cost. Try 16..32. Census-based --matching-cost only; CPU/GPU.
  --census-shift S
      Opt-in shiftable windows, default 0 = off. The matching cost is the minimum over
      census windows shifted by up to S pixels (1 or 2), so a pixel next to a depth jump
      can use a window lying entirely on its own side. Costs (2S+1)^2 evaluations per
      hypothesis. Combines with --census-edge-mask. Census-based costs only; CPU/GPU.
  --disparity-edge-erosion N
      Opt-in, default 0 = off. After the left/right, median and ZNCC checks, reject
      every valid disparity that has another valid disparity within N pixels (square
      neighbourhood) differing by more than --disparity-edge-jump, and the N pixels at
      both ends of a horizontal invalid gap (an occlusion, at most the disparity
      difference + 2N wide) whose two sides differ by more than that. Removes about N
      pixels on both sides of depth jumps, where mixed roof/ground disparities fan out
      into rays. Never fills or moves values. Try 1 or 2. Counted as reason code 8.
  --disparity-edge-jump J
      Disparity difference in pixels that counts as a depth jump for
      --disparity-edge-erosion (default: 2; positive).
  --uniqueness N
      Relative best/alternative cost separation (default: 0.03; 0 <= N < 1).
      Higher values reject more ambiguous matches; a minimum cost gap remains.
  --lr-threshold N
      Maximum left/right disparity disagreement in pixels (default: 0.75;
      positive). Reciprocity is checked again after median filtering.

Point filtering and multi-view fusion:
  --fusion-page-side N
      Depth evidence page side (default: 64; powers of two, 16..512). Smaller pages
      reduce random audit/normal lookup I/O. Each page holds N*N interleaved int32
      cells (4*N*N bytes), not an N-by-N pixel tile. Depth/offset values remain exact.
  --fusion-cache-mb N
      CPU depth-page cache MiB (default: 0 = min(512, memory-mb/8), at least 1).
      Additional to GPU evidence and native image buffers; preserves all audits.
  --fusion-batch-size N
      Fusion/pipeline batch ceiling (default: 16384; 32..65536); actual GPU batches
      may shrink to fit VRAM/allocation limits. Does not change filtering rules.
  --surface-batch-size N
      GPU surface proposal look-ahead ceiling (default: 64; 1..1024), reduced
      by the surface memory allowance. Ordered CPU representatives remain intact.
      Use 64 for the previous ceiling.
  --surface-partition routed|recursive
      Default routed plans exact spatial splits using a small coordinate file,
      then compresses final partitions once. Recursive retains the reference
      repeated gzip split path. Routed may use more peak temporary disk space.
  --filtering NAME
      mild, moderate (default), aggressive: progressively stricter depth/mode,
      reprojection, correlation and speckle thresholds. Explicit overrides win
      regardless of argument order; repeated presets use the last name.
      Preset       fusion-relative  fusion-pixels  consensus  correlation  speckle
      mild         0.01             1.25           0.55       0.45         20
      moderate     0.005            0.75           0.60       0.55         50
      aggressive   0.0025           0.50           0.75       0.65         100
  --depth-outlier-filter off|edge-aware
      Default off preserves the existing output. Opt-in edge-aware rejects disparity
      outliers against a dominant local plane using local noise and stereo uncertainty.
      Protects connected directional thin features, extending surfaces and depth edges.
      Adds multiscale, separate-layer surface checks and precise independent-camera
      validation of suspect patches during fusion; uncertain compatibility is neutral.
      Diagnostics report preservation/rejection reasons. Surviving local depths never move.
      Supports standard/pyramid, windowed and aerial-tiled SGM; not PatchMatch.
  --disparity-refinement off|multiview
      Default off. Experimental second-pass search guided by independently triangulated
      surfaces; four anchor cameras exclude both producing cameras. Rechecks image
      agreement and local photometric reciprocity; unresolved surface conflicts are
      rejected. Reliable geometry and directional thin features are retained unchanged.
      Currently requires standard, non-windowed SGM, --fusion-mode image,
      --pyramid-levels 1 --step 1, and at least six registered cameras.
      Raw stereo reports precede this pass; separate refinement reports describe changes.
  --disparity-anchor-angle DEG
      Minimum anchor triangulation angle (default 4; >0..<90). --min-angle still
      controls the candidates available for recovery; use 1 to revisit weak matches.
  --disparity-refinement-memory-mb N
      Anchor-grid budget (default 128 MiB; 8..4096). Grid spacing adapts to the
      camera count/resolution; one rectified pair and ordinary fusion are additional.
  --depth-outlier-radius N
      Neighbourhood radius (default 3; 2..8 processed pixels).
  --depth-outlier-deviation N
      Minimum deviation from the fitted plane (default 1.5; >0..32 disparity pixels).
      Lower values remove smaller deviations but may lose real small features.
  --depth-outlier-edge-threshold N
      Maximum grayscale difference used for local support (default 24; 0..255).
      Larger values cross more image edges. This is an intensity, not depth, threshold.
  --min-correlation N
      Minimum zero-mean normalized cross correlation (-1..1; preset default:
      0.55 moderate). Higher values require stronger photometric agreement.
  --correlation-window N
      Correlation window side in pixels (default: 7; odd, 3..15).
  --speckle-size N
      Reject connected disparity components smaller than N pixels (preset
      default: 50 moderate; nonnegative; 0 disables speckle removal).
  --speckle-range N
      Maximum neighboring disparity difference within a speckle component,
      in pixels (default: 1; nonnegative).
  --speckle-connectivity N
      Component adjacency: 8 (default) preserves diagonal wires/rails; 4 restores
      the earlier axial-only grouping. Component size and disparity thresholds
      still apply; this does not bridge missing pixels or fill holes.
  --min-angle N
      Minimum triangulation angle in degrees (default: 1; 0 <= N < 90).
      Larger angles require stronger depth geometry.
  --confidence-threshold T
      Final normalized evidence score cutoff, inclusive 0..1 (default: 0). Points
      below T are removed; equality passes. Invalid/missing scores pass only at 0.
      Scores rank matching and geometry evidence; they are not probabilities.
      Pair-only points additionally require matching confidence >= 0.35 and
      pass --max-depth-uncertainty. The default remains --min-views 3 pending
      the two-view quality gate; confidence cannot resolve every false match.
  --dense-method sgm|patchmatch|mvsgm
      Default sgm. Experimental PatchMatch estimates camera-space depth/planes
      directly from multiple views. Uses OpenCL GPU; --compute cpu selects its
      slow reference implementation. No silent substitution with SGM.
      Opt-in mvsgm: multi-view plane-sweep SGM, one depth map per camera from
      several neighbouring views (census, mean of the best k neighbour costs,
      8-path SGM over inverse-depth labels, coarse-to-fine bands), then a
      multi-view consistency check instead of the left/right check. Requires
      --fusion-mode image. Pair-only options (--two-view-confidence-threshold,
      --pairwise-search, --similarity-window) do not apply. See MULTIVIEW_SGM.md.
  --mvsgm-neighbors N
      mvsgm: neighbour cameras per reference camera, 1..8 (default 6).
  --mvsgm-best-k K
      mvsgm: per pixel and depth label, mean of the K lowest neighbour costs
      (default 3), so occluded neighbours do not spoil a pixel.
  --mvsgm-optimal-neighbors
      Experimental: retain the strongest neighbour, then favour complementary
      sparse image coverage and baseline directions within --mvsgm-neighbors.
      Preserves angle-weighted scores and eligibility checks; deterministic greedy
      selection, not a guarantee of optimality or improved dense accuracy.
      Default off. --no-mvsgm-optimal-neighbors explicitly restores score-only selection.
  --mvsgm-census-window W
      mvsgm: census window 3, 5 or 7 (default 5).
  --mvsgm-levels N
      mvsgm: coarse-to-fine levels below full resolution, 0..5 (default -1 =
      --tsgm-levels). 0 searches every depth label at full resolution.
  --mvsgm-consistent-views N
      mvsgm: a depth is kept only if at least N neighbour depth maps agree when
      reprojected (default 1; 0 disables the check).
  --mvsgm-consistency-labels T
      mvsgm: agreement tolerance in the neighbour's inverse-depth label steps
      (about one pixel of image motion each; default 1).
  --dense-checkpoint off|auto|MINUTES
      mvsgm with overlap-window fusion: save a checkpoint every MINUTES (auto = 30)
      between two fused cameras. If the dense step is interrupted (killed, server
      restart, out of memory), run the same dense command on the same run again
      (dense -o PROJECT --run ID): it continues after the last checkpoint with the
      same result as an uninterrupted run. Live cameras' fused maps are kept under
      06_fusion/checkpoint (hard links where possible) and depth maps retired after
      a checkpoint are deleted at the next one, so temporary disk grows by up to one
      interval's maps. A checkpoint is used only with the same options (resources
      such as --cores and --memory-mb may change) and the same earlier steps;
      otherwise dense starts over. Not with --fusion-seed-order precision,
      --fusion-host-memory-mb, --voxel without surface consolidation, --fusion-emission camera
      or --diagnostics. off deletes an earlier checkpoint.
      patchmatch: checkpoints of the solve under 05_stereo/patchmatch_checkpoint (the
      pass and the references finished in it; the map files it needs stay until the
      next checkpoint). auto uses them only when the PatchMatch generations do not fit
      in memory (large blocks); MINUTES forces them. A resume after the solve reruns
      the evidence and fusion stages from the saved maps.
  --mvsgm-cameras-in-flight N|auto
      mvsgm: compute the depth maps of N cameras at once (default 1 = one at a time;
      1..16). Each camera in flight prepares its neighbour images on the CPU and solves
      on the GPU in its own session; all sessions share --gpu-memory-mb and wait for
      free device memory, and the cameras share --cores. Depth maps do not depend on
      each other and are handed to consistency and fusion in the unchanged order, so
      the output equals the serial path. auto = one per 4 cores, at most 8, lowered
      until the per-camera heap estimate fits; 1 with --compute cpu. Up to N raw depth
      maps exist ahead of fusion (extra temporary disk: 16 bytes per pixel each). If
      a camera runs out of heap, the remaining cameras are computed one at a time.
  --mvsgm-tile auto|off|N
      mvsgm: solve each depth map in tiles of N x N pixels (256..16384) with a
      64-pixel overlap on every side; images no larger than N are solved whole.
      auto (default) = 2048-pixel tiles for images above 16 Mpx, whole images
      otherwise. off = never tile (needs memory for the whole image). Larger tiles
      use more GPU and host memory per camera in flight (about 256 heap bytes per
      tile pixel) and leave fewer tile borders, where depths differ slightly from
      an untiled solve. A dense checkpoint resumes only with the same value.
  --fusion-source-maps retained|scoped
      MVSGM image-fusion source-map lifetime (default: retained, legacy behavior).
      scoped unmaps each camera's source maps after its last active copying reader
      finishes; required files remain available for later reopening. Uses ordinary
      memory-mapped reads, without a new memory budget, forced strips or stereo
      concurrency changes. Does not bound other fusion maps, heap or OS file cache.
      With --fusion-host-memory-mb enabled, its bounded page cache takes precedence.
      Saved in run settings and available in the GUI. See FUSION_SOURCE_MAPS.md.
  --fusion-host-memory-mb N
      Opt-in host working allowance for overlap-window MVSGM image fusion (default: 0,
      legacy path). Includes retained project data, native/image reserve, stereo workspace,
      fusion strips, output bands and the shared source/fusion file-page cache. Requires
      --streamed-processing auto|on. Keeps every dependency on disk, serializes stereo
      preparation and fusion, and refuses work that cannot fit. Application accounting
      is not a strict total-RAM guarantee; use a job/cgroup limit for enforcement.
      Independent of --fusion-window-mb (GPU evidence). See FUSION_HOST_MEMORY.md.
  --fusion-host-cache-mb N
      Shared source/fusion file-page cache size (default: 512 MiB), capped at 1/8 of
      --fusion-host-memory-mb and the safe heap allowance. Only used with that opt-in.
  --dense-pipeline standard|aerial-tiled
      Default standard is unchanged. Opt-in aerial-tiled requires a compatible OpenCL GPU.
      It rectifies source regions on demand, predicts tile search intervals from a coarse
      disparity map, matches overlapping tiles in VRAM, and fuses compact maps in blocks.
      Tile-local SGM is approximate; weak predictions and search boundaries trigger widening.
      Requires --fusion-mode reference --fusion-strategy owner --surface-consolidation none --voxel 0;
      image/incremental fusion and --windowed-mode on are incompatible.
  --aerial-tile-size N / --aerial-overlap N
      Core tile side (default 512; 64..2048), context and seam-check overlap (default 32).
      Tiles subdivide to fit host/device budgets; overlap must be 8..tile-size/2.
  --aerial-coarse-dimension N / --aerial-disparity-margin N
      Maximum coarse-map side (default 1024; 128..2048), full-resolution search margin
      (default 16; 4..256). Low coverage or boundary pressure restores the pair interval.
  --two-view-confidence-threshold T
      Experimental opt-in SGM gate for points supported by exactly two cameras (default: off).
      Require min(left matching confidence, corresponding right confidence) >= T;
      the existing 0.35 matching floor remains in force on both sides. Finite 0..1;
      try 0.5. Omit to retain the existing left-only gate and memory use.
      Three-or-more-view acceptance and the global --confidence-threshold are
      unchanged. Requires ordinary SGM (--pyramid-levels 1). No extra SGM pass.
      Diagnostics: 06_fusion/two_view_confidence.json. Scores are not probabilities.
  --patchmatch-iterations N
      Minimum sweeps per scale/round (default: 4; 1..12). Large rasters add
      enough sweeps for long-range propagation to reach one-pixel refinement.
      Each level has a photometric round and two frozen-map geometric rounds.
  --patchmatch-edge-sigma N
      Reference intensity scale for edge-aware weighted ZNCC (default: 32;
      0..255; 0 uses uniform patch weights). Invalid warps and inadequate
      texture/support remain unavailable, rather than confident poor matches.
  --patchmatch-views N
      Maximum source views per reference camera (default 4; 1..8).
  --patchmatch-levels N
      Coarse-to-fine levels (default 3; 1..5).
  --patchmatch-min-depth N / --patchmatch-max-depth N
      Positive camera-Z bounds in reconstruction units, specified together (default: inferred).
      Otherwise infer padded bounds from aligned tie points; fail if unavailable.
  --patchmatch-references-in-flight N|auto
      References solved at once within each PatchMatch pass (default auto: one per
      3 cores, at most 4, on the GPU; one per core with --compute cpu; 1..32).
      Lowered when the heap is short. Results are identical for any value.
  --patchmatch-tile auto|off|N
      Solve large references in tiles of N x N pixels plus a halo (GPU only).
      auto (default) tiles only a level raster whose untiled working set exceeds
      --memory-mb, the heap or the GPU budget (where an untiled solve fails); off never
      tiles; N tiles every level raster larger than N (256..16384). Tiles agree with an
      untiled solve (see --patchmatch-tile-halo). Tiled results are not cached.
  --patchmatch-tile-halo auto|N
      Halo around each PatchMatch tile in level pixels. auto (default) uses the sum of
      the propagation strides, which makes tiles identical to an untiled solve, capped at
      256 pixels for very large frames (then tiles agree except near tile edges).
  --patchmatch-neighbors nearest|score
      Source views of each PatchMatch reference (at most --patchmatch-views). nearest
      (default): the nearest camera centres among the matched pairs. score: the
      mvsgm view selection (shared tie points weighted by triangulation angle).
  --patchmatch-speckle N
      Remove small islands from the final PatchMatch depth maps (default 0 = off): regions
      of neighbouring pixels whose depths agree within --patchmatch-speckle-tolerance that
      have fewer than N pixels are dropped before fusion. Removes isolated patches of
      noise (e.g. 200-1000 for 3-12 Mpx images).
  --patchmatch-speckle-tolerance N
      Relative depth step that still connects neighbouring pixels for --patchmatch-speckle
      (default 0.01 = 1 %; (0, 0.2]).
  --patchmatch-consistent-views N
      Source views whose PatchMatch maps must confirm a depth (default 1: the best view
      only). Each confirmation is the reciprocal check (reprojection within
      --lr-threshold, depth agreement). 2 or more removes depths that only one other
      view supports, such as floating clumps under tree crowns (1..8).
  --patchmatch-view-cost best|top2|top3|top4
      How a plane hypothesis is scored over its source views. best (default): the best
      view plus a small second-view term. topK: the mean of the K best view costs, so a
      depth must fit several views (averages per-pair orientation residuals).
  --patchmatch-smooth R
      Plane-consistent depth averaging of the final PatchMatch maps (default 0 = off):
      each depth becomes the score-weighted mean of the depths that the planes of valid
      pixels within R pixels predict for it, using only predictions within
      --patchmatch-smooth-tolerance of its own depth, so other surfaces at depth edges do
      not take part. Lowers noise on smooth surfaces.
  --patchmatch-smooth-tolerance N
      Relative depth agreement for --patchmatch-smooth (default 0.01 = 1 %; (0, 0.2]).
  --min-views N
      Minimum distinct supporting cameras (default: 3; minimum: 2), capped
      at registered image count. Two-image input uses stereo checks only; allowing
      2 in a larger collection permits points without independent pair support.
  --fusion-relative N
      Relative depth agreement fraction (preset default: 0.005 moderate = 0.5%;
      0 < N <= 0.5). Larger values allow thicker/noisier depth agreement.
  --fusion-pixels N
      Positive back-projection/measurement tolerance in pixels (preset default:
      0.75 moderate). Larger values loosen cross-pair correspondence checks.
  --fusion-consensus N
      Fraction of available per-ray depth evidence required for the dominant
      mode (preset default: 0.6 moderate; 0.5 < N <= 1).
  --fusion-strategy owner|owner-consume|graph-average
      Image fusion (--fusion-mode image): how cameras' copies of one surface become
      points. owner (default): the confirming camera with the finest footprint emits
      its own depth; other copies are dropped (the earlier rule). owner-consume: owner
      plus, with cameras processed in order, a pixel is also a copy when an earlier
      agreeing camera emitted a point nearby or an emitted point marked it; fewer
      stacked sheets, but in tests also far fewer interior points (strips).
      graph-average: every unused merged pixel seeds a point that averages, along its
      ray, all agreeing depths of the overlapping cameras, weighted by their inverse
      variance (triangulation angle); member pixels are then used and seed nothing.
      Copies are combined instead of chosen, so offsets average out without thinning.
      --fusion-dedupe owner|consume is the earlier name (consume = owner-consume).
  --thin-surfaces
      Image fusion preset for thin flat surfaces (off by default; options after it
      override its parts). Sets --fusion-tolerance sigma, --fusion-precision-check on,
      --fusion-seed-order precision, --fusion-local-sigma 3,
      --surface-consolidation normal, --surface-flatten 8, --subpixel-refinement zncc.
      Requires --fusion-mode image. See THIN_SURFACES_FUSION.md.
  --fusion-tolerance relative|sigma
      Image fusion depth agreement. relative (default): |dz| <= --fusion-relative x depth
      (0.005 = 44 cm at 88 m). sigma: |dz| <= K x combined stereo sigma of both depths
      (from triangulation angles, 0.5 px disparity), at least one pixel footprint and at
      most the relative band. Precise wide-baseline depths then only confirm what agrees
      at their precision.
  --fusion-sigma-k K
      Sigma multiplier for --fusion-tolerance sigma and --fusion-precision-check (default 3).
  --fusion-precision-check on|off
      Image fusion (default off): a point that cameras with at least 2x smaller sigma see
      within the relative band but not within K sigma (a near miss, not an occlusion) is
      rejected when such cameras outnumber the precise confirmations; confirmed points are
      averaged with the precise depths along their ray (any --fusion-strategy). Counted as
      precision_rejected.
  --fusion-seed-order camera|precision
      Image fusion emission order. camera (default): camera by camera. precision: three
      passes over all cameras: precise pixels (merged sigma <= 1 footprint) first, then
      the rest, then pixels seen only by narrow pairs (--fusion-seed-min-angle), so precise
      points absorb noisier copies of the same surface. About 3x pass-2 time.
  --fusion-seed-min-angle DEG
      Image fusion (default 0 = off; not part of --thin-surfaces): pixels whose merged depth
      comes only from pairs with triangulation angle below DEG may still confirm and be
      averaged, but emit a point only when at least two cameras outside their own pairs
      confirm it. Counted as narrow_unconfirmed_rejected. In tests, 12 removed about 14% of
      vegetation and edge coverage for a marginal flatness gain.
  --fusion-local-sigma K
      Image fusion (default 0 = off): reject a point whose final sigma exceeds K x the best
      merged sigma within 4 pixels in its camera (precise evidence nearby that it did not
      join). Counted as local_sigma_rejected.
  --fusion-free-space on|strict|off
      Image fusion: reject a point that floats in front of the surface the other cameras
      see: more cameras look through it (their surface lies more than 3 pixel footprints
      behind it) than confirm it, and at least two (default on). Removes near-ground
      "salt and pepper" points that the relative --fusion-relative tolerance accepts
      (the existing visibility test only acts beyond 3 x that tolerance, 1.3 m at 90 m).
      strict also counts a camera as confirming only within 3 footprints: more floaters
      go, and a few percent of roof and crown edge points with them. off: as before.
      Counted as free_space_rejected.
  --fusion-spike-filter on|off
      Image fusion: remove isolated depth spikes ("salt and pepper" points a few
      decimetres off a smooth surface) from each camera's merged depth map before the
      cross-check (default on). Only where the 5 x 5 neighbourhood is smooth (spread at
      most one pixel footprint); vegetation and depth edges are not touched. Counted as
      spike_rejected.
  --fusion-depth-outliers on|off
      Image fusion: remove merged depths beyond 1.5x a camera's 99.5th percentile depth or
      below 0.5x its 0.5th percentile (default on): isolated points far below the ground
      or far above the scene. Counted as depth_outlier_rejected. off keeps them.
  --fusion-agreement-radius auto|N
      Image fusion: a cross-checked camera agrees when any merged depth within N
      pixels of the projection lies inside --fusion-relative (centre first); merged
      maps are sparse at step > 1. auto (default) = 0 for owner (exact previous
      output), otherwise max(1, --step). Contradictions use the centre pixel. 0..8.
  --max-depth-uncertainty N
      Maximum estimated relative stereo depth sigma (default: 0.1 = 10%;
      0 < N <= 1). Geometric heuristic, not surveyed accuracy.
  --no-depth-refinement
      Disable per-candidate depth refinement; fusion checks still apply.
      Also use --surface-consolidation none to disable subsequent surface movement.
  --depth-refinement reprojection|weighted
      Default reprojection robustly minimizes distinct measured-camera pixel errors
      along the source ray. Weighted restores the earlier depth-average estimator.
  --surface-consolidation normal|none
      Default normal combines compatible overlapping samples and refines positions
      along local surface normals. None exports per-candidate fusion results directly.
      Separate depth layers and unreliable normals are preserved; no ground assumption.
  --surface-radius N
      Tangential neighborhood radius in local candidate spacings (pixel footprint
      times --step; default: 2;
      0.5..8). Larger neighborhoods can suppress more noise but lose small detail.
  --surface-spacing N
      Representative sample spacing in local candidate spacings (default: 1;
      positive, <= surface-radius). Only geometrically compatible samples combine.
  --surface-max-shift N
      Maximum normal displacement in local pixel footprints (default: 2; 0 < N <= 10).
      Every moved result must pass multi-view checks again.
  --surface-flatten K
      Opt-in, default 0 = off (consolidation unchanged). With K > 0 footprints:
      samples without a reliable source-depth normal get a robust plane-fit normal from
      their 3D neighbours, or the agreeing neighbour normals (used only for
      consolidation; exported normals unchanged), so they are refined like the others;
      and a sample whose own layer is weak evidence (<= 2 samples, or all from one
      stereo pair) is removed when a dominant layer (>= 6 samples from >= 2 pairs, 3x
      larger and a majority) lies within K footprints along the normal. Features
      confirmed by several pairs are kept. Try 6..8. Counted as
      isolated_off_surface_removed in surface_consolidation.json.
  --surface-normal-angle N
      Maximum angle between compatible normals, degrees (default: 30; 0 < N <= 60).
  --surface-memory-mb N
      Surface working-set budget in MiB (default: 128; minimum: 8), additionally
      limited by --memory-mb. Spatial partitions spill to temporary disk files.
  --point-attributes LIST
      Values stored per point in LAS Extra Bytes and PLY properties, comma-separated.
      Default surface: source-depth normals and roughness/planarity diagnostics (32
      bytes per point; normals point toward the producing pair's first camera, invalid
      estimates are NaN, not recomputed after consolidation). none keeps compact XYZ/RGB
      clouds and avoids normal estimation when surface consolidation is also none.
      Single surface fields: normal, roughness, surface_variation, planarity,
      normal_diagnostics. Per-point evidence: views (cameras confirming the point),
      angle (largest triangulation angle, degrees), confidence (0..1), source (source
      camera in the LAS Point Source ID), depth_spread (std of the averaged cameras'
      depths, 3+ cameras). Uncertainty: height_sigma and ray_sigma, calibrated 1-sigma
      local precision (vertical, along the viewing ray) in output units: the fusion
      model's sigma scaled by the run's variance factor from the spread of the depths
      fusion averaged (image fusion only; reported in report.txt). Local precision from the
      images only, including disagreement between cameras (so larger than the scatter
      around flat surfaces); orientation and absolute errors are not included. Groups:
      uncertainty (height_sigma, ray_sigma), all. surface and none write the layout
      above unchanged; uncertainty adds a final pass over the cloud files.
  --surface-validation auto|cpu|gpu
      Default auto reuses prepared GPU fusion evidence for batched surface checks,
      with CPU guards and audits; otherwise CPU. Cpu forces reference checks.
      Gpu requires prepared GPU fusion (--fusion-compute gpu). These checks never
      repeat depth refinement. Auto times neighborhood backends and can select CPU
      neighborhoods while retaining GPU depth checks. Representatives stay ordered.
  --max-depth N
      Maximum positive rectified-camera depth before output scaling (default:
      10000; positive, in reconstruction units).
  --step N
      Export candidate every N pixels along each image axis (default: 2; with
      --dense-method mvsgm 1; minimum: 1). Does not resize images or reduce full depth-map
      storage. mvsgm keeps its maps dense for fusion and emits every N-th pixel per camera.
  --voxel N
      Optional voxel edge in final output units; keep one point per voxel
      (default: 0 disables; nonnegative).

"""
        , """
Georeferencing and output coordinates:
  --crs EPSG:CODE, --epsg CODE
      Target projected metre-based EPSG CRS (default: EPSG:3067). GPS alignment
      runs automatically with sufficient valid, non-collinear 3D camera controls;
      otherwise output remains local without a CRS tag. Z uses EXIF GPS altitude
      with unspecified vertical datum unless --source-geoid/--target-geoid are given.
      EPSG:CODE+VERTICAL (e.g. EPSG:3067+3900, ETRS-TM35FIN + N2000 heights) also
      labels the heights in LAS, DSM/DTM GeoTIFF, PLY and coordinates.json. The
      label converts nothing: combine it with --target-geoid, or use it when the
      heights already are in that system (GCPs, supplied exteriors).
  --source-geoid ellipsoid|FILE
      Vertical datum of the camera GPS heights: ellipsoid (WGS84/ETRS89 ellipsoidal
      heights, e.g. DJI RTK) or the geoid grid FILE they are measured from (e.g. EGM96
      for most non-RTK drones). Default: ellipsoid when --target-geoid is given.
  --target-geoid ellipsoid|FILE
      Vertical datum of the outputs: ellipsoid or a geoid grid FILE (e.g. FIN2005N00
      for N2000 heights in Finland). Default: ellipsoid when --source-geoid is given.
      Camera heights are converted before georeferencing, so every output is in the
      target datum. Grid files: .gtx, or text with "latitude longitude N" per line on
      a regular grid. Without either option heights are not converted. Supplied
      exteriors with --orientation-crs are converted too (--source-geoid then names
      their height datum). GCP heights are read as given (in the target datum). This
      changes the datum only: it does not fix GPS height errors (non-RTK drones are
      often tens of metres off).
  --gps
      Require successful GPS alignment instead of allowing local output.
  --no-gps
      Disable automatic GPS alignment; conflicts with --gps.
  --gps-threshold N
      GPS camera-center RANSAC inlier tolerance in metres (default: 10; positive).
  --tie-point-cleanup off|auto|custom|pause
      Tie-point cleanup by quality after alignment (step 'cleanup', like gradual
      selection): tie points with a large reprojection error, seen in too few images or
      with a small intersection angle are removed, worst first, and the block is
      adjusted again before georeferencing. off (default): no cleanup. auto: the
      automatic limits (largest error three times the median, 2 images, 1 degree, at
      most 20%), without stopping. custom: the limits given with the four options below
      (any not given keeps its default), without stopping. pause: the run stops
      ('waiting') after the measures are written (04_geometry/tie_points/metrics.csv);
      choose the limits with the histograms in the POSE window (Project tab, Clean tie
      points; it starts from the limits below), which then continues the run. Report:
      04_geometry/tie_points/cleanup.json.
  --tie-point-max-error PX|auto
      With custom (or as the pause window's start): remove tie points whose largest
      reprojection error exceeds PX processed pixels (default: auto = three times the
      median of the points' largest errors, at least 0.5).
  --tie-point-min-views N
      Remove tie points seen in fewer than N registered images (default: 2, i.e. none).
  --tie-point-min-angle DEG
      Remove tie points whose largest intersection angle between two viewing rays is
      below DEG degrees: their depth is uncertain (default: 1).
  --tie-point-max-removed PERCENT
      Remove at most this share of the tie points, worst first (most limits failed,
      then largest error; default: 20). A camera always keeps at least 15 tie points.
  --camera-screening off|report|exclude
      After alignment, flag cameras whose sparse reprojection RMS or GPS residual is far
      above the block's (>1.6x median or >4 robust deviations) or that were taken in
      turns (heading change >45 degrees). report (default): 04_geometry/camera_screening.json
      and a PDF quality flag. exclude: also leave flagged cameras out of dense matching.
  --gps-bundle off|prior
      prior (default): after the GPS similarity fit, run a final bundle adjustment in the
      projected frame with the inlier GPS camera centres as weighted position priors
      (rotations free). Limits block bending that the similarity fit cannot correct. The
      options below extend this adjustment. off: similarity fit only.
      Report: 04_geometry/georeference/gps_bundle.json.
  --gps-sigma M
      GPS camera-centre standard deviation for --gps-bundle prior (default: 2 metres,
      consumer GNSS; use about 0.05 for RTK/PPK).
  --gps-height-sigma auto|M
      Height standard deviation of that prior (default auto: 0.2 m when DJI XMP
      barometric relative altitudes are present, else --gps-sigma). Tight heights stop
      radial distortion from doming a nadir block.
  --gps-intrinsics none|LIST
      Shared lens parameters refitted in the GPS-weighted adjustment (default: all of
      f,k1,k2,k3,cx,cy,p1,p2). Skipped for fixed supplied calibrations.
  --gps-focal-prior auto|none|PIXELS
      Focal-length prior for that refit, in source pixels. auto (default): DJI XMP
      CalibratedFocalLength, else the EXIF 35 mm equivalent. Nadir blocks barely separate
      focal length from flying height, so this prior sets the ground height scale.
  --gps-focal-sigma S
      Relative standard deviation of the focal prior (default: 0.03 = 3%).
  --gps-time-offset estimate|off|SECONDS
      GPS time offset dt in gps = C + V dt + R^T L (default: estimate). Along-track GPS
      errors that change sign with flight direction. Needs camera velocities: DJI XMP
      flight speeds, or camera centres and capture times.
  --gps-lever-arm estimate|off|X,Y,Z
      GPS antenna offset L from the camera centre in camera axes (x right, y down, z
      viewing), metres (default: estimate, with a 0.5 m prior).
  --rolling-shutter estimate|off|SECONDS
      Rolling-shutter readout time fitted in the GPS-weighted adjustment (default:
      estimate; signed, positive when rows are read top to bottom). Translation only;
      needs camera velocities. Dense matching uses the mid-exposure poses.
  --gps-adjustment-rounds N
      Alternations of the GPS antenna model and the bundle (default: 3; 1..20).
  --classify-ground
      Classify ground points in the final cloud (off by default; needs georeferenced
      output). Progressive morphological filter on robust low points, bounded by camera
      rays (terrain cannot stand above a ray that reached its point); tolerances tuned
      from the cloud, no reference terrain needed. Writes LAS classes 2 (ground) and 7
      (low noise) into dense.las, plus 07_results/dtm.tif, chm.tif, ground_support.tif
      (1 observed, 2 interpolated, 3 blocks camera rays), ground_uncertainty.tif
      (split-half) and ground.json.
  --ground-pair-correction
      With ground classification (implies --classify-ground): each stereo pair's ground is
      compared with the consensus ground of all pairs and a robust offset (plus tilt for
      pairs spanning 10 m or more) is fitted, like LiDAR strip adjustment. Offsets are
      always measured and reported in ground.json (pair_offsets); this option also
      subtracts them from all points of each pair (ground and canopy) in dense.las and
      dense.ply, then classifies again. point-confidence.bin keeps the original heights.
  --ground-aware-fusion
      Merge the ground copies of all stereo pairs into one sheet (implies
      --classify-ground). Each pair's own ground level per 1 m cell is shifted onto the
      consensus, then copies merge to one point per point spacing at the view-weighted
      mean height, and the merged ground is fused into one smooth sheet (robust local
      plane over all pairs within --ground-fusion-radius). Shrubs and canopy are not
      moved. Rewrites dense.las, dense.ply and point-confidence.bin; results in
      ground.json (ground_aware_fusion).
  --ground-fusion-radius auto|M
      Sheet fusion radius (default auto: twice the terrain cell, e.g. 0.5 m). Ground
      relief smaller than this is smoothed away.
  --ground-cell auto|M
      Terrain grid cell (default auto: 3x the point spacing, 0.25..2 m).
  --ground-max-slope DEG
      Steepest terrain kept as ground (default: 35 degrees).
  --ground-max-object M
      Largest non-ground object, e.g. a tree crown (default: 30 m); sets the
      largest filter window.

Ground control points (the gcp step):
  --gcp FILE
      Ground control point CSV: gcp_id,x,y,z with optional sx,sy,sz (sigmas, metres)
      and type (control or check). Header optional; separator , ; or tab. Coordinates
      are in --crs (or the GPS CRS). Check points are not used in the adjustment and
      give an independent accuracy estimate.
  --gcp-check none|auto|N
      Hold control points back as check points without editing the GCP file (default:
      none, the types in the file). auto: about one in five, keeping at least four
      control points (nothing with fewer than five). N: that many. The control points
      kept are the ones spread widest, so check points lie between them; the choice
      depends only on the coordinates. The report lists the points held back.
  --gcp-check-ids ID,ID,...
      Use these GCPs as check points, whatever their type in the file.
  --gcp-markers FILE
      Start from these image markers (markers.json from another run) instead of
      predicted ones.
  --gcp-sigma M
      Horizontal GCP sigma when the CSV has none (default: 0.02 m).
  --gcp-height-sigma M
      Vertical GCP sigma when the CSV has none (default: 0.03 m).
  --gcp-port N
      Local port of the gcp-edit editor (default: 8765).
  The gcp step needs 3 control points with 2+ pinned images each (1 if the block
  is already GPS-georeferenced). It moves the block onto the GCP datum, adjusts it
  with the GCPs as weighted points (GPS fixes then get a datum offset) and writes
  04_geometry/gcp/gcp_report.json and gcp_residuals.csv.
  --lidar-reference FILE|FOLDER
      Georeference the block to an airborne laser scanning (ALS) reference instead of
      ground control (step 'lidar', after georeference): one LAS/LAZ file or a folder
      of tiles. After the GPS fit, a rigid transformation (roll, pitch, heading and
      three shifts about the block centre; scale with --lidar-scale on) is estimated
      that puts the tie points onto the lidar surface (coarse search from the GPS
      solution, then robust coarse-to-fine surface matching; open ground and roofs
      weigh more than canopy; changed areas are rejected). Cameras and tie points are
      moved before dense matching, so all products are in the lidar's frame and
      height datum. The step first checks that the lidar points cover the block plus
      --lidar-margin and stops with what is missing otherwise. Weakly determined
      parameters (e.g. shifts over flat terrain) are left at the GPS values and
      reported. Report: 04_geometry/lidar/lidar_reference.json; PDF report section.
  --lidar-scale on|off
      Also estimate a scale factor (default: off; rotation and shift only).
  --lidar-search-shift M
      Horizontal search range around the GPS solution (default: 10 metres; heights are
      found without a limit).
  --lidar-search-angle DEG
      Heading search range around the GPS solution (default: 3 degrees); the tilts are
      refined from the GPS values.
  --lidar-cell M
      Cell of the lidar surface grids (default: 2 metres; coarser levels 4x and 2x).
  --lidar-canopy-weight W
      Weight of tie points on canopy (lidar surface more than 2 m above its ground)
      relative to open ground and roofs (default: 0.1; 0 uses stable areas only).
  --lidar-outlier M
      Largest accepted surface distance of a tie point after the fit (default: 2 m).
  --lidar-change-limit M
      20 m patches whose ground or roof residuals (median) differ from the block's by more
      than this are treated as changed (new buildings, earthworks) and left out (default:
      1 m). Canopy growth or felling since the lidar is handled anyway: canopy height is free
      per 20 m patch and ground counts only where the images also see ground.
  --lidar-weak-limit M
      A parameter is weakly determined when its standard deviation, as a movement at
      the block edge, exceeds M (default: 0.25 m); it is then not estimated.
  --lidar-margin M
      Margin around the tie-point area that the lidar must also cover, for the GPS
      error (default: 10 m).
  --lidar-crs EPSG:CODE
      Horizontal CRS of lidar files whose own CRS is missing or has no EPSG code. It
      must equal --crs. Without it, files with no CRS at all are taken to be in --crs
      (with a warning) and files with a WKT but no EPSG code are refused. Files that
      declare a different EPSG code than --crs are always refused (reproject them first).
  --lidar-vertical-datum NAME
      Name of the lidar height datum when the files do not declare it (e.g. N2000);
      written to coordinates.json and the report.
  --scale N
      Multiply final LAS/PLY coordinates by N (default: 1; positive). Camera CSV,
      sparse CSV and --max-depth remain in pre-multiplier reconstruction units.
      Successfully GPS-georeferenced output requires scale 1.

Chessboard calibration (separate mode):
  --calib, -c
      Estimate a camera calibration from chessboard photographs instead of
      reconstructing a point cloud. Saves camera.txt and calibration diagnostics.
      Cannot combine with --estimate-memory; uses original raster resolution.
  --board-columns N
      Number of inner corners across the board (default: 9; minimum: 2).
  --board-rows N
      Number of inner corners down the board (default: 7; minimum: 2).
  --square-size N
      Physical square edge length (default: 0.02; positive). Units define board
      pose units; e.g. 0.02 metres for 2 cm squares.

Additional workflow controls:
  --storage minimal|standard|debug
      Default standard. Optional image/feature/disparity exports require --diagnostics;
      minimal omits them, standard keeps selected exports, debug keeps the full audit.
      Without diagnostics, all profiles keep normal deliverables, compact confidence
      and summaries. Working depth files are retained after success only with both
      --diagnostics and debug storage. No effect on reconstruction/filtering.
  --cache, --no-cache
      Stage cache writing is disabled by default. --cache enables it; --resume
      and --rerun-from imply it. Superseded generations of the same key are pruned.
  --compute cpu|auto|gpu
      Default auto uses a compatible OpenCL GPU for the selected matching cost and
      global SGM path aggregation when worthwhile and within VRAM limits;
      otherwise uses CPU. gpu requires GPU execution; cpu disables GPU probing.
      Fusion also uses the GPU when FP64 and depth-memory budgets permit.
      Calibration, feature extraction and stereo post-filtering remain on CPU.
  --fusion-compute inherit|cpu|auto|gpu
      Default inherit follows --compute. Override fusion independently, e.g. cpu
      with GPU stereo. auto falls back safely; gpu requires GPU fusion. Double
      precision is required; sensitive numerical cases are rechecked on CPU.
  --list-gpus
      List OpenCL GPU devices and exit; no input/project is required.
  --gpu-device N
      Zero-based OpenCL GPU index from --list-gpus (default: 0).
  --gpu-memory-mb N
      Maximum estimated GPU working set in MiB; 0 (default) uses 70% of device
      VRAM. Explicit values may use up to 95% (5% reserved for driver/display).
      Buffers are banked to respect individual allocation limits. Actual free
      VRAM and other applications can still prevent allocation; auto falls back.
  --sgm-aggregate auto|16|32
      Default auto stores CPU/GPU aggregates as unsigned 16-bit when the proved
      bound directions*(512 + effective P2) <= 65535; otherwise signed 32-bit.
      16 requires that bound; 32 forces reference storage. Arithmetic, raw costs
      (including invalid cost 512) and continuous path state are unchanged.
  --gpu-sgm auto|resident|hybrid|streamed|esgm
      Default auto tries resident GPU SGM, then a GPU-resident aggregate with
      streamed cost strips, then exact RAM-backed streaming.
      Beyond RAM, compact Census proposals plus mandatory exact verification
      are used when supported; other configurations use bounded recomputation.
      resident requires complete GPU volumes. hybrid requires the full aggregate
      on GPU, but only one cost strip; no aggregate round trips through RAM.
      streamed preserves ordinary SGM
      with continuous paths. esgm explicitly selects compact grayscale Census
      with safe 16-bit aggregates; unresolved matches are verified exactly,
      rather than discarded. Costs, signed ranges and filters are preserved.
  --gpu-aggregate auto|ram|disk|recompute
      Aggregate backing for ordinary GPU streaming (default: auto).
      auto uses native RAM if aggregate plus workspace fits --memory-mb/heap
      planning and available system memory; otherwise it recomputes bounded
      groups with continuous global path state. ram requires the full aggregate
      in RAM. disk explicitly enables contiguous temporary storage (2 or 4
      bytes*width*height*disparities); files are removed on close. recompute
      avoids the full aggregate volume at the cost of repeated GPU work.
      Host aggregate RAM is included in the plan, not allocated on top of it.
  --esgm-candidates N
      Maximum separated proposals retained per path (default: 2; 1..4).
      Planner may reduce this to fit memory. More proposals can reduce exact
      verification work but use more VRAM/compute. No confidence rule is relaxed.
  --windowed-mode off|auto|on
      Bounded disk-backed stereo rasters and continuous SGM paths (default: off).
      auto switches a pair to windowed stereo when its rasters do not fit the memory
      allowance or the GPU disparity limit; on always uses it. Windowed pairs run
      serially and are slower; results follow the same matching rules.
  --image-reader auto|windowed|opencv
      Source decoding (default: auto). Sources above 64 MiB of RGB pixels and
      BigTIFF use source windows; other small sources retain the OpenCV reader.
      windowed always uses global area resampling over source windows. TIFF windows
      are true region reads; JPEG/PNG (which cannot skip earlier rows) are decoded
      once when 4 bytes/pixel fit the image budget divided by --cores, otherwise
      per window. Tiles use up to --cores readers; pixels are identical either way.
      Decoding produces 8-bit sRGB via the image color model, including supported
      16-bit grayscale/RGB TIFFs and BigTIFF. This does not implement native
      16-bit matching. Classic TIFF windows use the JDK codec; BigTIFF maps only
      requested compressed blocks into a bounded virtual TIFF for that decoder.
      Reader/plugin support is required; unsupported formats fail explicitly.
  --image-window N
      Maximum decoded source-window side (default: 1024; 64..8192 pixels).
      Bounds decoder output windows; codec-specific internal buffers may differ.
  --image-memory-mb N
      Native processed-raster LRU cap (default: 0 = min(2048, memory-mb/4) MiB).
      At least one active pair must fit; raise this or lower resolution if needed.
      Metadata/keypoints survive eviction; active source pairs load on demand.
      Stereo/rectification buffers and codec scratch are additional to this cache.
  --processed-image-cache auto|on|off
      Keeps each decoded, resized photograph (3 bytes per processed pixel, before
      undistortion) in the run's cache/processed-images folder until the dense step
      ends, so the dense step and frames evicted from the raster cache read it back
      instead of decoding the photograph again. Identical results. Default auto
      writes only while at least 16 GiB and a tenth of the volume stay free; on
      only needs 1 GiB free; off always decodes. Only images at the dense size.
  --descriptor-memory-mb N
      Native descriptor LRU cap (default: 256 MiB), also capped at memory-mb/4.
      Both active descriptor sets must fit. Exact descriptor spills are run-local
      temporary files, removed when frames close; independent of persistent --cache.
      Feature detector scratch and retained keypoint metadata are additional.
  --gpu-bank-mb N
      Maximum size of each SGM allocation bank (default: 1024; 1..1024 MiB),
      further capped by the device allocation limit. Up to eight banks per volume.
  --gpu-strip-rows N
      Maximum streamed working-strip height (default: 64; positive). Planner
      reduces it to fit GPU/host budgets. Smaller strips increase transfer/I/O work.
  --fusion-weight-math direct|reference
      Default direct computes geometric weights without trigonometric round trips.
      reference preserves the earlier GPU expression for diagnostic comparisons.
  --fusion-layout coalesced|reference
      Default coalesced stores per-camera measurements in GPU-friendly planes.
      reference keeps the previous layout for comparison. Filtering is unchanged.
  --no-fusion-pipeline
      Disable default one-batch prefetch/validation overlapping ordered cloud output.
      Useful for profiling; output order and decisions are preserved.
  --diagnostics
      Export detailed confidence CSV, feature/match dumps and intermediate images.
      Off by default. Compact lossless point metadata, reports, numerical checks,
      calibration and cloud attributes are retained independently of this flag.
      Enables debug detail unless --storage explicitly bounds it. Without this flag,
      storage debug does not enable expensive optional exports.
  --export-disparity-images
      Keep the final per-pair disparity map without enabling --diagnostics:
      05_stereo/pair_<a>_<b>/disparity.pfm (numeric, NaN = invalid), disparity.png
      (grayscale preview over the searched range) and disparity.json. Applies to the
      standard SGM path; windowed, aerial-tiled and PatchMatch paths do not export it.
      With --dense-method mvsgm (also with --diagnostics): per image
      05_stereo/mvsgm/image_<id>_<name>/depth.* (the SGM depth map) and
      consistent_depth.* (what passes the multi-view check), each .pfm (depth, NaN =
      none), .png (grayscale over the image's depth range, near dark) and .json.
  --dense-pair-selection all|coverage
      Default all preserves all verified pairs. Experimental coverage ranks pairs
      by angle-qualified sparse coverage and predicted range cost; unknown coverage
      keeps all pairs. Camera degree and original-component connectivity are retained.
  --dense-pair-neighbors N
      Coverage-mode minimum degree target per camera (default 3; 1..16).
      Sparse coverage cannot guarantee dense branch completeness; compare quality.
  --calibration-pipeline bootstrap|joint
      joint (the default through --quick-calibration) uses a consistent EXIF focal as
      initialization and fits intrinsics in the main alignment track graph; missing
      EXIF falls back to the bootstrap. bootstrap estimates focal and radial
      distortion from image pairs first; giving it turns --quick-calibration off.
  --ply on|off
      Default on writes 07_results/dense.ply next to dense.las (same points; double
      coordinates). off skips it: less disk and write time. The report DSM then reads
      dense.las; --analyze-run needs the PLY.
  --tiled-las-output SIZE
      Default 0 writes one 07_results/dense.las: LAS 1.2, switched automatically to
      LAS 1.4 above 4,294,967,295 points. SIZE > 0 writes square LAS tiles of SIZE
      output units (metres in a projected frame) to 07_results/dense_tiles/
      tile_X_Y.las (X, Y = lower-left corner) with tiles.csv, instead of dense.las.
      dense.ply stays one file. Not with --classify-ground or --block-processing.
  --point-confidence on|off
      Default on writes 07_results/point-confidence.bin (lossless per-point metadata in
      cloud order, about 200 bytes per point). off skips it; the cloud is unchanged.
      Kept on when ground classification/correction or --diagnostics is requested (they
      read it); --export-confidence needs it.
  --quick-calibration [on|off], --no-quick-calibration
      Default on: the focal length is initialized from EXIF/XMP and the
      focal/distortion bootstrap is skipped (as --calibration-pipeline joint);
      intrinsics are still fitted parameter-wise during alignment, the
      GPS-weighted adjustment uses one focal start instead of up to two, and
      converging bundle adjustments stop when two successive iterations each
      improve the objective by less than 1e-4 (off: 1e-6). Without a consistent
      EXIF focal it falls back to the bootstrap. off (or --calibration-pipeline
      bootstrap) restores the bootstrap path, the default before 2026-11-17. Give
      it with the first step (inputs/calibrate); later steps reuse the saved
      choice, and runs started before the change continue with off.
  --streamed-processing off|auto|on
      MVSGM image fusion uses an overlap window (default: auto). auto selects row
      strips only when required by the heap; on forces strips too. off keeps the
      separate stereo/fusion phases and refuses fusion when its work cannot fit.
      Camera order and output stay unchanged.
  --image-fusion-cache-mb auto|N
      Reuse decoded image-fusion candidates across cameras and passes. Default auto
      uses at most a quarter of --memory-mb and available estimated heap headroom.
      N is an additional upper limit; 0 restores uncached replay. Geometry is unchanged.
  --gpu-transfer-mode queued|blocking
      Default queued batches banded-stereo uploads on the in-order GPU queue;
      host buffers remain alive until completion. Blocking is the comparison path.
  --gpu-summary-format packed|full
      Default full retains the six-word summary without host unpacking. Experimental
      packed transfers five words per banded-SGM pixel, losslessly packing winner/flags.
      It saves transfer bytes but may cost more CPU time on fast GPU interconnects.
  --fusion-emission pair|camera
      Default pair keeps existing sampling. Experimental camera groups accepted
      observations by owner-camera pixel and separate depth layers, retaining the
      strongest unchanged point per layer. Requires --surface-consolidation none;
      does not remove pair evidence or invent additional supporting cameras.
  --fusion-evidence auto|pair|camera
      Exact evidence layout, independent of output sampling (default: auto). Auto packs per-camera
      sparse pair layers for blocks of 9+ cameras when this saves at least 20% and
      fits safe host/commit allowance. Pair uses the reference paged maps. Camera
      explicitly requires packing (maximum 32 incident pairs per camera).
  --fusion-gpu-evidence auto|resident|windowed
      GPU depth-evidence layout for fusion (default: auto). resident keeps all pair
      evidence on the GPU; windowed uses bounded regional evidence windows (see
      --fusion-window-mb); auto uses regional windows only when full evidence cannot fit.
  --fusion-gpu-normals on|off
      Default off. on computes image-fusion surface normals and surface attributes
      (--point-attributes surface) on the GPU in batches. Not bit-identical to the
      CPU: normals and attributes differ in the last bits, and a point on a test
      threshold can rarely keep or lose its normal. The first batch is checked
      against the CPU; a larger difference, no FP64 device or --compute cpu falls
      back to the CPU.
  --stereo-pairs-in-flight N|auto
      Compute N stereo pairs concurrently (default: auto = one per 3 cores, at most 6,
      when the per-pair memory estimate fits the heap allowance; never fewer than one per
      4 cores, at most 4; 1 = serial). The pairs share --cores and the GPU memory budget;
      results are published strictly in pair order, so the output equals the serial
      path. If concurrent pairs run out of heap, the remaining pairs are computed one at
      a time (same results). Windowed stereo, --diagnostics and PatchMatch stay serial.
  --stereo-pipeline on|off
      Bounded one-pair-ahead rectification/descriptor preparation (default off).
      Keeps OpenCL and evidence publication on the coordinator. Falls back to
      serial preparation for windowed stereo, diagnostics or insufficient RAM.
  --fusion-camera-culling on|off
      Conservative per-pair camera bounds and compact GPU scratch (default off).
      Queries outside recorded bounds use exact CPU replay; no fixed camera cap.
  --fusion-association pair|camera
      Experimental incremental association ownership (default pair). camera uses
      a shared visible reference camera and <=0.75 pixel anchored layer agreement;
      does not invent camera support. Requires --fusion-mode incremental.
  --fusion-mode reference|incremental|image
      Depth-map fusion method (default: reference). reference corroborates each
      pair's candidates against the other cameras' depth evidence. incremental is
      experimental (provisional camera/pixel hypotheses). image fuses per camera in
      image space: per-pixel consensus over the camera's pairs, 2D speckle filter,
      reciprocal cross-checks against the other cameras' merged maps, one owner camera
      per surface (see --fusion-strategy). With --dense-method patchmatch, image fusion
      takes each camera's PatchMatch map as one single-view pair (like mvsgm).
  --fusion-outliers reference|robust
      Outlier rule in fusion (default: reference). robust removes a 3+ view point only
      when independent cameras contradict it (free space or a nearer surface).
  --fusion-window-mb N
      Independent of the opt-in --fusion-host-memory-mb host working allowance.
      Cap in MiB for regional GPU fusion evidence (default: 0 = at most half the GPU
      budget). Used by --fusion-gpu-evidence windowed/auto.
  --export-confidence RUN_DIRECTORY
      Export point-confidence.bin to point-confidence.csv.gz without reconstructing.
  --compatibility-copies
      Also copy large final clouds to the project root. Off by default; latest-files.csv
      identifies run outputs without duplicating cloud bytes.
  --output-compression N
      Gzip level for point-confidence output (default: 6; 1..9). Lower values
      trade larger output for less CPU compression work; point quality is unaffected.
  --legacy-fusion-reprojection
      Diagnostic comparison only: disable the corrected final measured-camera
      reprojection check. Default rechecks every credited support camera after
      refinement, preventing points from retaining geometrically invalid support.
      Fusion backend.json and output_profile.json separate GPU kernel/transfer,
      CPU guard/audit and ordered output work; GPU activity alone misses these costs.
  --intrinsics-mode fixed|prior|initial
      Supplied calibration: fixed (default) preserves it; prior refines selected
      parameters with uncertainty penalties; initial uses it without those penalties.
      Guarded fits still require usable geometry and held-out improvement.
      CAMERA=MODE (repeatable) sets one camera group's mode; it also accepts free:
      that camera's supplied or imported intrinsics are ignored and it is calibrated
      automatically.
  --extrinsics-mode fixed|prior|initial
      Supplied poses: fixed (default) preserves them; prior refines poses with position
      and rotation uncertainties; initial refines without measurement penalties.
  --refine-intrinsics LIST
      Comma-separated subset of f,k1,k2,k3,cx,cy,p1,p2 (default: all eight).
      Bootstrap and alignment assess these one parameter at a time, including with
      two images. Requires image information after marginalizing poses/points and
      improvement on two withheld-track groups; unsupported parameters stay fixed.
      f preserves supplied fx/fy ratio. Principal point priors use
      2% of source width/height; --radial-sigma also controls p1/p2 priors.
      Adaptive fitting uses these priors and four-sigma bounds. Priors and LM
      damping are excluded from the data-support assessment.
  --focal-sigma N
      Relative focal prior standard deviation (default: 0.05, approximately 5%).
  --radial-sigma N
      Radial/tangential prior standard deviation (default: 0.05; normalized units).
  --position-sigma N
      Pose-center prior standard deviation in supplied pose units (default: 10).
  --rotation-sigma N
      Pose-rotation prior standard deviation in degrees (default: 10).
      All sigmas must be positive; smaller means stronger constraints, not hard bounds.
  --alignment-quality same|custom|ultra|high|medium|low
      Alignment resolution independent of dense --quality (default: same).
  --alignment-max-dimension N
      Optional explicit alignment long-side cap; default 0 inherits the normal cap.
  --matching-algorithm sift|orb|akaze|kaze|brisk
      Keypoint detector/descriptor (default: sift). Binary descriptors use Hamming;
      floating descriptors use L2. Applies to calibration and disparity previews too.
  --descriptor-matcher auto|bruteforce|flann
      Default auto uses brute force. flann uses a KD-tree for floating descriptors;
      binary descriptors use exact Hamming brute force for compatibility.
  --gpu-feature-matching on|off
      Default on: exact Hamming brute-force searches for binary descriptors (ORB, BRISK,
      AKAZE) run on the OpenCL GPU with the CPU matcher's neighbours and tie order. The
      first search is checked against the CPU matcher; a mismatch or GPU failure falls
      back to the CPU. Ignored with --compute cpu.
  --pair-matching parallel|serial
      Default parallel matches image pairs on --cores workers. Each pair seeds the
      OpenCV RNG from its image IDs, so FLANN/RANSAC results do not depend on thread
      count or order, and stage 03 reuses identical calibration matches. serial keeps
      the earlier single RNG stream across pairs (reproduces pre-revision results).
  --sift-contrast N
      SIFT contrast threshold (default: 0.04; positive). Lower retains weaker features.
  --feature-grid N
      Spread the --features keypoints over the image: N cells along the longer side, each
      cell first gets its share of its strongest keypoints, the rest go to the strongest
      remaining ones (default: 0 = strongest N overall; 2..256, e.g. 16). Same feature count
      and matching cost; detection and description are separate passes, so extraction is
      somewhat slower.
  --sift-edge-threshold N
      SIFT edge rejection threshold (default: 10; positive). Higher retains more edges.
  --orb-fast-threshold N
      ORB FAST intensity threshold (default: 20; 0..255). Lower detects weaker corners.
  --brisk-threshold N
      BRISK detection threshold (default: 30; 0..255). Lower detects weaker features.
  --akaze-threshold N
      AKAZE detector response threshold (default: 0.001; positive).
  --kaze-threshold N
      KAZE detector response threshold (default: 0.001; positive).
      Thresholds apply to their selected algorithm, including calibration/previews.
  --pair-selection auto|overlap|gps|sequential|all
      Default auto: shared ground footprints when poses and a scene plane are known;
      else nearest GPS cameras when every image has GPS spread over at least 20 m;
      else visual search (as overlap). Visual ranking can miss the neighbours along a
      flight line over uniform texture (forest canopy); GPS ranking cannot.
      overlap ranks shared ground footprints when poses and a scene plane are known;
      otherwise ranks reciprocal visual support and spatial coverage. Large visual
      searches use an approximate global descriptor index, then exact preview scoring.
      Scores estimate overlap, not a guaranteed physical percentage.
      gps ranks nearest GPS cameras, using visual search if coordinates are missing.
      sequential uses the next filename-sorted images. all considers every pair,
      ignoring --neighbors. No filename edges are added except with sequential.
  --gps-neighbors N
      Also match each image with its N nearest GPS neighbours (default: 0 = off; 0..64), on
      top of --pair-selection. Guarantees the neighbours along each flight line are tried when
      visual ranking misses them (uniform canopy). Needs GPS on every image.
  --pair-candidates N
      Extra visual retrieval candidates before selecting --neighbors (default: 6;
      positive). Increase for broader search; N >= image count requests exhaustive
      preview scoring. Does not increase the final choices per image.
  --min-forward-overlap N, --min-side-overlap N, --min-pair-overlap N
      Minimum common footprint extents along/across flight, and common footprint
      area, respectively, in percent (0..100; all default 0 = disabled). Denominator
      is the smaller footprint extent/area. Diagonal pairs must meet both enabled
      directional thresholds; --min-pair-overlap also works for nonflight scenes.
      Applies to every candidate edge, including sequential fallback edges, and
      again to dense pairs after registration. It can disconnect the image graph.
  --overlap-ground-z N
      Explicit scene plane at world Z=N, in pose units (default: automatic). Otherwise use a sparse
      scene plane only when registered geometry is sufficiently constrained.
      Footprints are planar approximations; strong relief/oblique views can be unknown.
  --flight-heading N
      Flight heading in degrees clockwise from world +Y toward +X (default:
      inferred). Otherwise infer
      a coherent capture direction from registered camera centers. No direction
      means directional overlap is unknown; shared area may still be available.
  --overlap-unknown keep|reject
      Default keep preserves edges whose overlap cannot be estimated. reject
      requires known geometry (usually supplied poses and an explicit ground plane
      before alignment). Estimates/confidence/decisions go to 03_matches/overlap_*.json.
  --resume
      Reuse verified compatible project-local calibration, features, matches,
      geometry and stereo caches. Every invocation still creates a separate run.
  --rerun-from calibration|features|matches|geometry|stereo|fusion
      Implies resume; recompute this stage and downstream stages. Changed content,
      settings or implementation invalidate incompatible cache entries automatically.
  --cleanup-passes N
      Sparse observation/track cleanup rounds with repeat bundle adjustment
      (default: 1; 0 disables). Preserve camera support and audit all removals.
  --track-min-views N
      Minimum registered observations for sparse tracks (default: 2; minimum: 2).
  --track-max-error N
      Sparse observation cleanup threshold in processed alignment pixels (default: 4).
  --track-min-angle N
      Minimum sparse triangulation angle, degrees (default: 0.1; 0 <= N < 90).
  --registration-bundle-growth PCT
      During incremental camera registration, run a joint bundle adjustment (poses and
      points) each time the number of registered cameras has grown by PCT percent since
      the previous one, then re-triangulate tracks that had failed (default: 15; 0 disables).
      Limits drift on large sets; smaller values cost more time.
  --registration-local-bundles off|on
      on: after each newly registered camera, adjust it with its most connected
      registered neighbours (others held fixed, only their shared points), and run
      the joint bundles of --registration-bundle-growth at twice that growth. Fewer
      large bundles; the local ones keep new cameras consistent. off (default).
  --triangulation-outliers drop|reject
      When a new sparse point disagrees with some of its registered observations: drop
      those observations and keep the point supported by the most views (drop, default),
      or reject the whole track as before (reject).
  --registration-recovery off|on
      Images still unregistered after alignment (default: off). on: grow further models from
      seed pairs among them, merge each into the main model through images both contain or,
      without those, through GPS positions, then retry the rest with PnP checked against GPS
      (fewer inliers accepted when the camera lands where its GPS says). The reason for every
      unregistered image is in 04_geometry/registration.json either way.
  --registration-pose-check off|on
      Incremental registration (default: off). on, when every image has GPS: a camera solved
      by PnP is rejected (and retried later) if it lands more than max(3 x fit RMS, 3 m) from
      its GPS position under a similarity fit of the registered cameras to GPS, or if the
      block is near-nadir and its view axis is tilted more than max(10, 3 x the block's 90th
      percentile) degrees from the block's mean axis. The same checks run on every registered
      camera after each bundle adjustment during alignment: a camera the bundle moved off is
      un-registered and solved again (twice at most), and its tie points without two registered
      views are cleared. Catches weakly tied cameras that settle on a tilted, shifted pose.
      Rejections and removals are in 04_geometry/registration.json.
  --match-repair off|on
      After pair matching (default: off). on: images with fewer than --match-repair-min-pairs
      accepted pairs, and images outside the largest connected part of the match graph, are
      matched against their --match-repair-candidates nearest images not tried yet (GPS
      distance, else filename order). Details in 03_matches/match_repair.json.
  --match-repair-candidates N
      Extra pairs tried per weak image (default: 8; 1..200).
  --match-repair-min-pairs N
      An image with fewer accepted pairs is repaired (default: 2; 1..50).
  --tiled
      Use bounded disk-backed cost-volume tiles with continuous global SGM paths.
      Avoids full disparity-volume heap allocation; requires temporary disk space.
  --tile-size N
      Storage tile sizing and pyramid refinement core side (default: 512; minimum: 32).
  --tile-overlap N
      Minimum pyramid refinement halo in pixels (default: 32; 2..tile-size).
      Exact disk-tiled SGM carries path state across storage tiles instead of resetting.
  --pyramid-levels N|auto
      Coarse-to-fine matching levels (default: 1 = ordinary SGM; allowed: auto, 1..6).
      Higher levels start with a global coarse search, then refine local intervals.
      auto chooses up to six levels to bring coarse search near 96 hypotheses.
      Local searches retain all validated coarse depths with a margin and require
      spatial coverage; weak predictions/boundary pressure restore full tile ranges.
      Pyramid refinement uses independent overlapping tile paths and is approximate;
      use one level with GPU streaming or --tiled for continuous global path results.
      Per-level/tile interval and fallback counts are recorded beside the range report.
  --pyramid-radius N
      Refinement interval margin in current-level pixels (default: 16; minimum: 4).
      Weak predictions or boundary pressure trigger full-range tile fallbacks.
  --rectification-alpha auto|0|1
      Default auto preserves field of view if border-free rectification magnifies
      focal length by more than 1.5x. 0 reproduces border-free cropping; 1 keeps
      the full field of view, with invalid borders masked. Useful for convergent views.
  --memory-depth
      Retain per-pair depth evidence in heap. Default project runs use paged disk
      storage, independent of --tiled, to reduce accumulated depth-map memory.

Processing report:
  --report pdf|html|none
      Default pdf generates 07_results/processing-report.pdf directly in Java,
      with embedded fonts, orthomosaic where source imagery/surface permit,
      cameras, geometric overlap, settings and measured resource diagnostics.
      HTML and JSON companions remain available. html omits PDF; none disables
      report generation. Missing metrics/data are explicitly unavailable.
  --export-report RUN_DIRECTORY
      Regenerate a report from a completed run without reconstruction. Missing
      historical parameters/resources are labeled unavailable, never invented.
  --report-detail summary|full
      PDF layout. summary (default): the survey report: summary and quality flags,
      survey data, camera calibration, camera locations, ground control points, elevation
      models, processing parameters. full: the same followed by the detailed diagnostics
      (solver profiles, resources, OpenCL, all settings). HTML/JSON keep every detail.
  --report-output DIRECTORY
      Standalone report destination (default: RUN_DIRECTORY/07_results/report).
  --report-dsm-resolution N
      DSM cell size in final output units (default: 0 = automatic). Positive sizes
      exceeding the dimension cap are rejected; never silently coarsened.
  --report-dsm-max-dimension N
      Bound each DSM grid side (default: 1024; 16..4096). Auto resolution fits it.
  --report-dsm-statistic max|mean
      Surface-cell statistic (default: max); no ground classification or gap fill.
  --report-dsm-min-points N
      Minimum samples per DSM cell (default: 1); others remain nodata.
  --report-local-dsm
      Explicitly allow local XY/Z surface raster without a geographic CRS. Default
      produces a DSM only when archived coordinates establish a projected CRS.
Orthomosaic (ORTHOMOSAIC.md):
  --orthomosaic
      Write 07_results/orthomosaic.tif when the run finishes: a full-resolution true
      orthomosaic of the original photographs on the surface of the final cloud, as a
      tiled RGBA GeoTIFF (Deflate; BigTIFF above 4 GiB; transparent where there is no
      surface or no camera sees it), with orthomosaic.tfw and orthomosaic.json. Needs a
      projected coordinate frame (or --report-local-dsm). Default: off.
  --orthomosaic-resolution auto|M
      Pixel size in output units (default auto: the median ground sample distance of
      the photographs, two significant digits).
  --orthomosaic-surface-cell auto|M
      Cell of the surface the photographs are projected on (default auto: twice the
      point spacing of the cloud, not finer than a pixel).
  --orthomosaic-fill auto|M
      Close surface gaps up to about twice this wide: beside solid surface by continuing
      the lower neighbouring surface, so roads and ground without cloud points are still
      painted; between tree crowns with the height of the nearest crown (default auto:
      64 surface cells; 0: none). Wider gaps stay transparent.
  --orthomosaic-strict
      Leave a pixel transparent whenever no camera passes the line-of-sight test.
      Default: where only canopy (rough surface) is in the way, the least obstructed
      camera paints it, so forest is not full of holes; solid roofs always hide.
  --orthomosaic-mode true|classic|mosaic
      true (default): true orthomosaic as above, transparent where nothing is known.
      classic: conventional orthophoto without holes inside the photographed area: the
      surface is despiked, every gap is interpolated and the whole surface is smoothed;
      each pixel takes the camera nearest its image centre with no visibility test, and
      seams are feathered. Tall objects lean and may cover the ground beside them.
      mosaic: conventional orthophoto on the same gap-free surface as classic, but each
      photograph paints a contiguous region around its nadir; the seams between them are
      placed by a graph cut where neighbouring photographs agree (not through crowns or
      roofs), colours are balanced per photograph and the seams are blended over
      --orthomosaic-feather pixels. Recommended for forest blocks.
  --orthomosaic-smooth auto|M
      classic, mosaic: radius of the surface smoothing in output units (default auto: a tenth
      of the median camera height above the cloud; 0: despike and fill only). Less
      smoothing keeps crowns closer to their place but folds the photographs over
      steep canopy where the views are oblique.
  --orthomosaic-feather N
      classic, mosaic: blend photographs over about 2N pixels across each seam (default 16;
      0: hard seams).
  --orthomosaic-max-angle off|DEG
      mosaic: use only views within DEG degrees of the vertical (default off). Trims
      the stretched edge of a block that was photographed only obliquely; those parts
      become transparent.
  --export-orthomosaic RUN_DIRECTORY
      Make the orthomosaic of a finished run without reprocessing; written to the
      run's 07_results or to --orthomosaic-output DIRECTORY.
Orientation import/export (supported variants and conventions in ORIENTATIONS.md):
  --orientation-input FILE_OR_DIRECTORY
      Combined interior/exterior source. Per-image intrinsics are supported; the
      file's camera IDs become camera groups. Images of one camera with different
      intrinsics require --intrinsics-mode fixed (or CAMERA=fixed).
  --orientation-input-format NAME, --calibration-format NAME, --poses-format NAME
      Input format (default: auto). Choices: native, csv, csv-opk, inpho, bundler,
      colmap, openmvg, opensfm, metashape, micmac, opencv. --calibration and --poses
      may refer to separate sources. Unsupported models/conventions fail explicitly.
  --orientation-output-format LIST
      Comma-separated export formats from the choices above (default: none).
      Exports use original source intrinsics and final poses; no photos are rewritten.
  --interior-output-format LIST, --exterior-output-format LIST
      Additional export formats (default: none). CSV writes separate interior and
      exterior files; formats with combined camera containers export both together.
  --orientation-output DIRECTORY
      Export destination (default: run/08_orientations; standalone: run/orientation_exports).
      Existing format subdirectories are not overwritten. Each includes conventions.json.
  --export-orientations RUN_DIRECTORY
      Standalone export from an existing run; no images, matching or reconstruction
      needed. Defaults to CSV. New runs retain 04_geometry/orientation_model.json;
      supported older runs recover from inventory, calibration and final camera CSV.
  --orientation-pixels source|processed
      Default source exports original distorted image intrinsics. processed exports
      dense undistorted raster intrinsics with zero distortion. Pixel coordinates
      always refer to the declared raster size. Processed pixels do not match raw photos.
  --orientation-image-list FILE
      Bundler filename association (default: sibling list.txt). Source image headers
      or sibling images.csv are also required because Bundler omits raster dimensions.
  --orientation-component N
      Zero-based OpenSfM component or Metashape chunk index (default: 0).
  --orientation-rotation auto|world-to-camera|camera-to-world
      Matrix direction override (default: auto); auto uses world-to-camera for INPHO EXT_ORI,
      camera-to-world for MicMac. Other formats use their specified conventions.
  --orientation-axes auto|opencv|photogrammetric
      INPHO camera-axis override (default auto: x right, y up, z backward).
      Internally all formats become OpenCV x right, y down, z forward, world-to-camera.
  --orientation-angle-unit degrees|radians
      CSV omega/phi/kappa angle unit (default: degrees). R_camera_to_world_photo =
      Rz(kappa)*Ry(phi)*Rx(omega); another OPK convention requires explicit conversion.
  --orientation-pixel-size MM, --orientation-pixel-size CAMERA=MM
      Physical sensor pixel pitch for INPHO import when absent, required for INPHO
      export. No default: physical millimetres cannot be inferred from pixel focal length.
      CAMERA=MM (repeatable) sets one camera's pitch; INPHO imports keep their own.
  --orientation-world-units auto|m|ft|mm
      Declare orientation coordinate units (default auto: preserve available metadata).
      Declaration does not rescale coordinates. INPHO local export needs known units;
      CRS-tagged processing requires metre coordinates.
  --orientation-crs EPSG:CODE
      Declare supplied pose coordinates in a supported projected metre CRS; does
      not transform them. Suppresses automatic GPS realignment; conflicts with --gps.
  --orientation-allow-lossy
      Allow explicitly reported intrinsic approximations when target/source models
      cannot represent all terms. Default rejects these conversions. CRS/unit/axis
      notes are retained in sidecars; foreign programs may require separate CRS setup.

Read-only surface diagnosis:
  --analyze-run RUN_DIRECTORY
      Analyze an existing binary PLY without reconstructing or removing any points.
      Writes local plane residuals, thickness/offset CSV and SVG maps plus a summary.
      Negative residuals are candidates to inspect, not a ground classification.
  --analysis-output DIRECTORY
      Destination (default: a new surface_analysis_TIMESTAMP inside the run).
  --analysis-samples N
      Maximum uniformly sampled cloud points (default: 500000; minimum: 20).
  --analysis-region minX,minY,maxX,maxY
      Optional world XY region. Sampling is global before the region restriction.
  --analysis-cell-size N, --analysis-plane-tolerance N
      Local grid cell edge and plane-fit tolerance in cloud units (nonnegative).
      Default 0 chooses scene span/24 and cell-size*0.01, respectively.
  --analysis-confidence
      Scan compact point-confidence.bin (or a legacy CSV) to attribute sampled
      residuals to source pairs, supporting views and uncertainty. Large scans take time.

Unregistered images are skipped and reported at completion. At least two usable
cameras and a productive stereo pair are required. Each final point has a row in
07_results/point-confidence.bin, in cloud order; confidence is not surveyed accuracy.
Use --diagnostics or --export-confidence for the compatible CSV export.

Examples (from the repository directory):
  Windows reconstruction, automatic calibration and GPS when usable:
    .\\sgm_java\\photogrammetry.bat -i example_data/5x_test -o sgm_java/output/my-project --quality medium --filtering moderate
  Linux reconstruction with the same settings:
    bash sgm_java/photogrammetry.sh -i example_data/5x_test -o sgm_java/output/my-project --quality medium --filtering moderate
  Inspect memory requirements without reconstructing:
    .\\sgm_java\\photogrammetry.bat -i example_data/5x_test -o sgm_java/output/memory-plan --quality medium --estimate-memory
  Use a measured calibration:
    bash sgm_java/photogrammetry.sh -i /data/images -o /data/project --calibration /data/camera.txt --quality high

Outputs: per-run LAS/PLY clouds in 07_results/, interior/exterior orientations,
compact confidence/provenance, reports, quality summaries and logs. Detailed
keypoint/match tables, disparity rasters and decision traces require --diagnostics.
Memory plan: runs/<run-id>/00_inputs/resolution_plan.json. Consult README.md and
PROJECT_FORMAT.md for file conventions and calibration/pose formats.
""")
        .replace("@CORES@", Integer.toString(new Options().cores));
  }
}
