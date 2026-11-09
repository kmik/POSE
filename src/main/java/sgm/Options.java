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
  public Path exportConfidence;
  public boolean disparitiesExplicit, minDisparityExplicit, selfCalibration = true;
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
  // Without --bundle-iterations every bundle fit runs until it stops improving meaningfully.
  public boolean bundleIterationsExplicit;
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
  // Height sigma of the GPS camera prior: auto (0.2 m for DJI barometric heights, else gpsSigma).
  public String gpsHeightSigma = "auto";
  double gpsHeightSigmaValue = 2;
  // Fitted readout time (s) of the last GPS-weighted adjustment; reporting only.
  double rollingShutterReadout;
  // T1: off | report (default) | exclude flagged cameras from dense matching.
  public String cameraScreening = "report";
  // S1: stereo pairs computed concurrently; 1 = serial (default), 0 = auto.
  public int stereoPairsInFlight = 0; // 0 = auto (up to 4, one per 4 cores); output equals serial
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
  public boolean depthRefinement = true;
  public String depthRefinementMethod = "reprojection", surfaceConsolidation = "normal";
  public double surfaceRadius = 2, surfaceSpacing = 1, surfaceMaxShift = 2;
  /** Opt-in --surface-flatten window in footprints; 0 keeps consolidation unchanged. */
  public double surfaceFlatten = 0;
  public double surfaceNormalAngle = 30;
  public int surfaceMemoryMb = 128, surfaceBatchSize = 64;
  public String surfaceValidation = "auto", pointAttributes = "surface";
  public int correlationWindow = 7, speckleSize = 50, minViews = 3;
  public String filtering = "moderate", filteringOverrides = "";
  public String quality = "custom";
  public boolean maxDimensionExplicit, estimateMemory;
  public String matchingAlgorithm = "sift",
      descriptorMatcher = "auto",
      pairSelection = "sequential",
      pairMatching = "parallel";
  public String alignmentQuality = "same", intrinsicsMode = "fixed", extrinsicsMode = "fixed";
  public String refineIntrinsics = "f,k1,k2,k3,cx,cy,p1,p2", rerunFrom = "none";
  public int alignmentMaxDimension = 0, pairCandidates = 6, tileSize = 512, tileOverlap = 32;
  public int pyramidLevels = 1, pyramidRadius = 16, cleanupPasses = 1, trackMinViews = 2;
  public double focalSigma = .05, radialSigma = .05, positionSigma = 10, rotationSigma = 10;
  public double trackMaxError = 4, trackMinAngle = .1;
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
  java.nio.file.Path projectDirectory;
  Path runFeatureDirectory;
  java.util.concurrent.atomic.AtomicLong runFeatureHits =
      new java.util.concurrent.atomic.AtomicLong();

  public static Options parse(String[] args) {
    Options o = new Options();
    o.help = args.length == 0;
    Set<String> explicitFiltering = new TreeSet<>();
    boolean cacheDisabled = false;
    boolean storageExplicit = false;
    for (int i = 0; i < args.length; i++) {
      String key = args[i], inline = null;
      int eq = key.indexOf('=');
      if (eq > 0) {
        inline = key.substring(eq + 1);
        key = key.substring(0, eq);
      }
      switch (key) {
        case "--diagnostics":
          o.diagnostics = true;
          continue;
        case "--export-disparity-images":
          o.exportDisparityImages = true;
          continue;
        case "--compatibility-copies":
          o.compatibilityCopies = true;
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
          o.orientationPixelSize = number(v);
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
        case "--image-fusion-cache-mb":
          o.imageFusionCacheMb = v.equalsIgnoreCase("auto") ? -1 : Integer.parseInt(v);
          break;
        case "--gpu-transfer-mode":
          choice(v, "--gpu-transfer-mode", "queued", "blocking");
          o.gpuQueuedUploads = v.equals("queued");
          break;
        case "--calibration-pipeline":
          o.calibrationPipeline = v.toLowerCase(Locale.ROOT);
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
        case "--intrinsics-mode":
          o.intrinsicsMode = v.toLowerCase(Locale.ROOT);
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
          o.calibration = Paths.get(v);
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
          o.focal = number(v);
          break;
        case "--s_w":
          o.sensorWidth = number(v);
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
        case "--step":
          o.step = Integer.parseInt(v);
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
                o.exportReport, o.exportConfidence, o.analyzeRun, o.exportOrientations)
            .filter(Objects::nonNull)
            .count()
        > 1) throw new IllegalArgumentException("Choose one standalone export/analysis action");
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
    if (!o.depthOutlierFilter.equals("off") && !o.denseMethod.equals("sgm"))
      throw new IllegalArgumentException(
          "--depth-outlier-filter edge-aware currently requires --dense-method sgm");
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
    choice(o.calibrationPipeline, "--calibration-pipeline", "bootstrap", "joint");
    choice(o.fusionEmission, "--fusion-emission", "pair", "camera");
    choice(o.fusionEvidence, "--fusion-evidence", "pair", "camera", "auto");
    choice(o.fusionGpuEvidence, "--fusion-gpu-evidence", "auto", "resident", "windowed");
    choice(o.fusionMode, "--fusion-mode", "reference", "incremental", "image");
    if (o.fusionMode.equals("image")
        && (!o.fusionEmission.equals("pair") || !o.denseMethod.equals("sgm")))
      throw new IllegalArgumentException(
          "--fusion-mode image requires --dense-method sgm and --fusion-emission pair");
    choice(o.fusionOutliers, "--fusion-outliers", "reference", "robust");
    choice(o.fusionAssociation, "--fusion-association", "pair", "camera");
    if (o.fusionAssociation.equals("camera") && !o.fusionMode.equals("incremental"))
      throw new IllegalArgumentException(
          "--fusion-association camera requires --fusion-mode incremental");
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
    choice(o.denseMethod, "--dense-method", "sgm", "patchmatch");
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
        || (o.patchmatchMaxDepth > 0 && o.patchmatchMaxDepth <= o.patchmatchMinDepth))
      throw new IllegalArgumentException(
          "Invalid PatchMatch depth range or iterations/views/levels");
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
    choice(o.pairSelection, "--pair-selection", "sequential", "overlap", "gps", "all");
    choice(o.pairMatching, "--pair-matching", "parallel", "serial");
    choice(o.fusionStrategy, "--fusion-strategy", "owner", "owner-consume", "graph-average");
    choice(o.gpsBundle, "--gps-bundle", "off", "prior");
    choice(o.cameraScreening, "--camera-screening", "off", "report", "exclude");
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
    choice(o.pointAttributes, "--point-attributes", "surface", "none");
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
    if (o.gcpPort < 1 || o.gcpPort > 65535)
      throw new IllegalArgumentException("--gcp-port must be 1..65535");
    if (!(o.groundMaxObject >= 1 && o.groundMaxObject <= 500))
      throw new IllegalArgumentException("--ground-max-object must be 1..500 metres");
    if (o.gpsAdjustmentRounds < 1 || o.gpsAdjustmentRounds > 20)
      throw new IllegalArgumentException("--gps-adjustment-rounds must be 1..20");
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
    out.runFeatureDirectory = runFeatureDirectory;
    out.runFeatureHits = runFeatureHits;
    out.projectDirectory = projectDirectory;
    out.orientationCalibrations = orientationCalibrations;
    out.orientationSourceUnits = orientationSourceUnits;
    out.orientationNotes = orientationNotes;
    out.stereoReportDirectory = stereoReportDirectory;
    out.gpuHostReserve = gpuHostReserve;
    out.stereoSelection = stereoSelection;
    out.tsgmSeeds = tsgmSeeds;
    try {
      for (java.lang.reflect.Field field : Options.class.getFields())
        field.set(out, field.get(this));
    } catch (IllegalAccessException e) {
      throw new IllegalStateException(e);
    }
    return out;
  }

  public static String help() {
    return String.join(
        "",
        """
POSE - Photogrammetric Orientation and Surface Estimation (Java/OpenCL)
Usage: photogrammetry -i IMAGE_DIR [-o PROJECT_DIR] [options]
       (pose.bat / pose.sh start the same program as photogrammetry.bat / .sh)
Run with no arguments, -h, --help or -help to display this help and exit.
Values accept --option VALUE or --option=VALUE. Quote paths containing spaces.
Defaults below apply unless overridden; pixel units refer to processed images.

Pipeline steps (run one by one, or chained; the state is saved after each step):
  photogrammetry -i IMAGES -o PROJECT [options]
      All steps in one process (as before).
  photogrammetry STEP [STEP ...] -o PROJECT [options]
      Only the named steps, in order: inputs, calibrate, match, align, georeference,
      gcp, dense, ground, finish. 'inputs' starts a new run (needs -i); the other
      steps continue the latest run with the options saved by 'inputs'; options given
      here are added and override them. 'finish' writes the report and final files.
      'gcp' and 'ground' are optional: without --gcp (or without ground options) they
      need not be named and are skipped when a later step runs. With --gcp, 'gcp' ends
      as 'waiting' until enough markers are pinned: pin them with gcp-edit, then run
      gcp again.
  --run ID
      Continue (or branch from) this run instead of the latest one.
  --branch
      Start a new run from the given run's saved state just before the first named
      step and redo from there (e.g. 'dense ground finish --branch --min-views 2').
      The source run is not changed.
  photogrammetry status -o PROJECT [--run ID]
      Show which steps of a run are done and the next one.
  photogrammetry gcp-edit -o PROJECT [--run ID] [--gcp-port N]
      Open the ground control point editor in the browser (local only). Click a GCP,
      then click its exact position in an image; the other images are then refined by
      patch matching (blue markers: check and accept with A, or accept all). Images
      whose view of the GCP is blocked by the tie points (canopy, roofs) are listed as
      probably hidden. Markers are saved as you go to 04_geometry/gcp/markers.json.
      Press Done, then run the gcp step again.

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
      Without calibration/focal inputs, initialize focal from images or EXIF.
  --focal MM
      Focal length in millimetres; requires --s_w. Initializes focal in pixels.
  --s_w MM
      Physical sensor WIDTH in millimetres (not diagonal); requires --focal.
      Without a calibration file, constrained parameter-wise bootstrap refinement
      follows initialization. Alignment uses the same data-support assessment.
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

Resolution and memory planning:
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
      Optional total host-RAM planning ceiling in MiB (0 = detect). Includes heap
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
      Match each image to the next N sorted images (default: 3; minimum: 1).
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
      every fit iterates until two successive iterations each improve the robust
      objective by less than 1e-6 relative, up to 100 iterations. With N, shared lens
      calibration uses staged fits and three focal starts, each with at least 12
      iterations; progress labels identify each fit. 0 skips pose-only iterations;
      --no-self-calibration separately disables lens refinement. The first pose
      and baseline norm anchor image-only geometry; pose priors anchor supplied
      refinement. Fixed supplied poses and rectified mode preserve their controls.

Dense stereo / SGM:
  --disparities N
      Explicit fixed hypothesis count, 3..4096; minimum defaults to 0 in this mode.
      Omit to estimate both bounds per pair from verified keypoints/sparse geometry,
      validate with a coarse whole-range probe, and expand either end when needed.
  --min-disparity N
      Pin the lower search bound (signed values allowed). Without this option or
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
  --dense-method sgm|patchmatch
      Default sgm. Experimental PatchMatch estimates camera-space depth/planes
      directly from multiple views. Uses OpenCL GPU; --compute cpu selects its
      slow reference implementation. No silent substitution with SGM.
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
      Experimental opt-in SGM gate for points supported by exactly two cameras.
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
      Positive camera-Z bounds in reconstruction units, specified together.
      Otherwise infer padded bounds from aligned tie points; fail if unavailable.
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
      stacked sheets but on forest_10 also 42% fewer interior points (strips).
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
      confirm it. Counted as narrow_unconfirmed_rejected. On city2, 12 removed 1.1M points and
      ~14% of vegetation/edge coverage for a marginal flatness gain.
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
  --point-attributes surface|none
      Default surface writes source-depth normals and roughness/planarity diagnostics
      into PLY properties, LAS Extra Bytes and the point-confidence sidecar. Normals
      point toward the producing pair's first camera; invalid estimates are NaN.
      Neighborhood scale/support/status are recorded; normals are not recomputed
      after consolidation. None keeps compact XYZ/RGB clouds and avoids normal
      estimation when surface consolidation is also none. Adds 32 bytes per cloud
      point to each LAS/PLY file, plus temporary spool/diagnostic storage.
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
      Export candidate every N pixels along each image axis (default: 2;
      minimum: 1). Does not resize images or reduce full depth-map storage.
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
      with unspecified vertical datum; no geoid/N2000 conversion is applied.
  --gps
      Require successful GPS alignment instead of allowing local output.
  --no-gps
      Disable automatic GPS alignment; conflicts with --gps.
  --gps-threshold N
      GPS camera-center RANSAC inlier tolerance in metres (default: 10; positive).
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
  --image-reader auto|windowed|opencv
  --windowed-mode off|auto|on
                          Bounded disk-backed stereo rasters and continuous SGM paths (default off).
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
  --dense-pair-selection all|coverage
      Default all preserves all verified pairs. Experimental coverage ranks pairs
      by sparse coverage and predicted range cost; unknown coverage keeps all pairs.
  --dense-pair-neighbors N
      Coverage-mode minimum degree target per camera (default 3; 1..16).
      Sparse coverage cannot guarantee dense branch completeness; compare quality.
  --calibration-pipeline bootstrap|joint
      Default bootstrap retains guarded focal/radial initialization. Experimental
      joint uses a consistent EXIF focal as initialization and fits intrinsics in
      the main alignment track graph. Missing EXIF falls back to the bootstrap.
  --image-fusion-cache-mb auto|N
      Reuse decoded image-fusion candidates across cameras and passes. Default auto
      uses at most a quarter of --memory-mb and available estimated heap headroom.
      N is an additional upper limit; 0 restores uncached replay. Geometry is unchanged.
  --gpu-transfer-mode queued|blocking
      Default queued batches banded-stereo uploads on the in-order GPU queue;
      host buffers remain alive until completion. Blocking is the comparison path.
  --fusion-emission pair|camera
      Default pair keeps existing sampling. Experimental camera groups accepted
      observations by owner-camera pixel and separate depth layers, retaining the
      strongest unchanged point per layer. Requires --surface-consolidation none;
      does not remove pair evidence or invent additional supporting cameras.
  --fusion-evidence auto|pair|camera
      Exact evidence layout, independent of output sampling. Auto packs per-camera
      sparse pair layers for blocks of 9+ cameras when this saves at least 20% and
      fits safe host/commit allowance. Pair uses the reference paged maps. Camera
      explicitly requires packing (maximum 32 incident pairs per camera).
  --fusion-gpu-evidence auto|resident|windowed
                          Auto uses bounded regional GPU evidence when full evidence cannot fit.
  --stereo-pairs-in-flight N|auto
      Compute N stereo pairs concurrently (default auto = up to 4, one per 4 cores;
      1 = serial). Each pair gets its share of --cores and GPU memory and its own OpenCL
      session; results are published strictly in pair order, so the output equals the
      serial path. Windowed stereo, --diagnostics and PatchMatch stay serial.
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
                          Experimental provisional camera/pixel hypotheses; reference is default.
                          image fuses per camera in image space: per-pixel consensus over the
                          camera's pairs, 2D speckle filter, reciprocal cross-checks against the
                          other cameras' merged maps, one owner camera per surface (see P2).
  --fusion-outliers reference|robust
                          Robust requires independent contradictory cameras for 3+ view points.
  --fusion-window-mb N     Regional evidence cap; 0 chooses at most half the GPU budget.
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
  --pair-selection sequential|overlap|gps|all
      Default sequential uses filename neighbors. overlap shortlists reciprocal
      preview matches; gps adds nearby GPS cameras, falling back to previews when
      coordinates are missing. Both retain sequential edges; all considers every pair.
  --pair-candidates N
      Additional shortlist neighbors per image (default: 6; positive).
  --min-forward-overlap N, --min-side-overlap N, --min-pair-overlap N
      Minimum common footprint extents along/across flight, and common footprint
      area, respectively, in percent (0..100; all default 0 = disabled). Denominator
      is the smaller footprint extent/area. Diagonal pairs must meet both enabled
      directional thresholds; --min-pair-overlap also works for nonflight scenes.
      Applies to every candidate edge, including sequential fallback edges, and
      again to dense pairs after registration. It can disconnect the image graph.
  --overlap-ground-z N
      Explicit scene plane at world Z=N, in pose units. Otherwise use a sparse
      scene plane only when registered geometry is sufficiently constrained.
      Footprints are planar approximations; strong relief/oblique views can be unknown.
  --flight-heading N
      Flight heading in degrees clockwise from world +Y toward +X. Otherwise infer
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
Orientation import/export (supported variants and conventions in ORIENTATIONS.md):
  --orientation-input FILE_OR_DIRECTORY
      Combined interior/exterior source. Per-image intrinsics are supported;
      distinct camera calibrations currently require --intrinsics-mode fixed.
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
      Matrix direction override; auto uses world-to-camera for INPHO EXT_ORI,
      camera-to-world for MicMac. Other formats use their specified conventions.
  --orientation-axes auto|opencv|photogrammetric
      INPHO camera-axis override (default auto: x right, y up, z backward).
      Internally all formats become OpenCV x right, y down, z forward, world-to-camera.
  --orientation-angle-unit degrees|radians
      CSV omega/phi/kappa angle unit (default: degrees). R_camera_to_world_photo =
      Rz(kappa)*Ry(phi)*Rx(omega); another OPK convention requires explicit conversion.
  --orientation-pixel-size MM
      Physical sensor pixel pitch for INPHO import when absent, required for INPHO
      export. No default: physical millimetres cannot be inferred from pixel focal length.
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
