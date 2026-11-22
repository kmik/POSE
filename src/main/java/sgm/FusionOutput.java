package sgm;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.*;

/** Keeps final cloud coordinates and confidence rows synchronized through surface consolidation. */
final class FusionOutput implements AutoCloseable {
  private final Options options;
  private final Project project;
  private final CloudWriter cloud;
  private final PointRecords.Writer records;
  private final CameraEmission cameraEmission;
  private final DepthFusion fusion;
  private final BufferedWriter surfaceDecisions;
  private final SurfaceConsolidator surface;
  private final Set<Voxel> occupied;
  private long staged;
  private long voxelRejected, confidenceRejected, unavailableConfidence;
  private long cloudWriteNanos;
  // --point-attributes evidence: the run's variance factor for the calibrated sigmas. Sums of the
  // weighted residuals of the depths fusion averaged (weights = inverse model variances) and their
  // degrees of freedom, overall and per triangulation-angle class (report only).
  private final boolean evidence;
  private final EnumSet<PointFields.Field> fields;
  /** report.txt line of the --point-attributes calibration ("" without it). */
  String uncertaintySummary = "";
  private double spreadRss;
  private long spreadDof;
  private final double[] binRss = new double[ANGLE_BINS.length + 1];
  private final long[] binDof = new long[ANGLE_BINS.length + 1];
  static final double[] ANGLE_BINS = {5, 10, 20, 40};
  /** Degrees of freedom needed before the run's factor replaces the model (factor 1). */
  static final long MIN_DOF = 1000;
  private long cameraNormalNanos;

  long deferredNormalNanos() {
    return cameraNormalNanos;
  }

  private boolean finished;
  private FusionHostMemory hostMemory;
  void hostMemory(FusionHostMemory memory) { hostMemory = memory; }

  private void retainOutput(long voxels) throws IOException {
    if (hostMemory == null) return;
    // One new tile and one overflow spool can appear in the upcoming point write.
    long extra = options.tiledLasOutput > 0 ? 4096 + (1L << 20) : 0;
    hostMemory.retainOutput(Math.addExact(Math.multiplyExact(voxels, 128L), cloud.retainedMetadataBytes() + extra));
  }

  private record Voxel(long x, long y, long z) {}

  FusionOutput(Options options, DepthFusion fusion, Project project, Crs crs) throws IOException {
    this.options = options;
    evidence = options.pointEvidence();
    fields = options.pointFields();
    this.fusion = fusion;
    this.project = project;
    Path file = options.output.resolve("07_results/point-confidence.csv.gz");
    Files.createDirectories(file.getParent());
    Files.createDirectories(options.output.resolve("06_fusion"));
    cloud =
        new CloudWriter(
            options.output, options.scale, crs, options.pointFields());
    cloud.writePly(options.writePly);
    if (options.tiledLasOutput > 0)
      cloud.tiles(options.tiledLasOutput, options.output.resolve("07_results/dense_tiles"));
    cloud.diskSpace(options.outputDiskSpace);
    Map<String, Object> resumed = options.checkpoint == null ? null : options.checkpoint.resume("output");
    if (resumed != null && resumed.get("cloud") != null) cloud.restore(DenseCheckpoint.map(resumed, "cloud"));
    BufferedWriter openedDecisions = null;
    PointRecords.Writer openedRecords = null;
    SurfaceConsolidator openedSurface = null;
    CameraEmission openedCamera = null;
    try {
      if (options.writePointConfidence)
        openedRecords = resumed != null && resumed.get("records_length") != null
            ? new PointRecords.Writer(file.resolveSibling("point-confidence.bin"),
                (long) resumed.get("records_length"), (long) resumed.get("records_bytes"))
            : new PointRecords.Writer(file.resolveSibling("point-confidence.bin"));
      if (options.fusionEmission.equals("camera"))
        openedCamera = new CameraEmission(options, fusion);
      openedSurface =
          options.surfaceConsolidation.equals("normal")
              ? new SurfaceConsolidator(options, fusion)
              : null;
      openedDecisions =
          openedSurface != null && options.diagnostics && options.storage.equals("debug")
              ? new BufferedWriter(
                  new OutputStreamWriter(
                      Compression.gzip(
                          Files.newOutputStream(
                              options.output.resolve("06_fusion/surface_decisions.csv.gz")),
                          options.outputCompression),
                      StandardCharsets.UTF_8))
              : null;
      if (openedDecisions != null)
        openedDecisions.write("candidate_id,representative_candidate_id,status,surface_shift\n");
    } catch (IOException | RuntimeException failure) {
      closeOnFailure(openedDecisions, failure);
      closeOnFailure(openedSurface, failure);
      closeOnFailure(openedRecords, failure);
      closeOnFailure(openedCamera, failure);
      try {
        cloud.abort();
      } catch (IOException cleanup) {
        failure.addSuppressed(cleanup);
      }
      throw failure;
    }
    records = openedRecords;
    cameraEmission = openedCamera;
    surface = openedSurface;
    surfaceDecisions = openedDecisions;
    occupied = options.voxel > 0 ? new HashSet<>() : null;
    if (resumed != null) {
      staged = (long) resumed.get("staged");
      confidenceRejected = (long) resumed.get("confidence_rejected");
      unavailableConfidence = (long) resumed.get("unavailable_confidence");
      voxelRejected = (long) resumed.get("voxel_rejected");
      if (evidence && resumed.get("spread_rss") != null) {
        spreadRss = ((Number) resumed.get("spread_rss")).doubleValue();
        spreadDof = ((Number) resumed.get("spread_dof")).longValue();
        List<?> r = (List<?>) resumed.get("bin_rss"), d = (List<?>) resumed.get("bin_dof");
        for (int i = 0; i < binRss.length; i++) {
          binRss[i] = ((Number) r.get(i)).doubleValue();
          binDof[i] = ((Number) d.get(i)).longValue();
        }
      }
    }
  }

  /** --dense-checkpoint: the staged output so far (surface consolidation spool). */
  Map<String, Object> checkpoint() throws IOException {
    if (cameraEmission != null || surface == null && occupied != null || finished)
      throw new IOException("Dense checkpoints need pair emissions without --voxel");
    Map<String, Object> out = new LinkedHashMap<>(Json.object("staged", staged,
        "confidence_rejected", confidenceRejected, "unavailable_confidence", unavailableConfidence,
        "voxel_rejected", voxelRejected));
    if (evidence) {
      out.put("spread_rss", spreadRss);
      out.put("spread_dof", spreadDof);
      out.put("bin_rss", Arrays.stream(binRss).boxed().toList());
      out.put("bin_dof", Arrays.stream(binDof).boxed().toList());
    }
    if (surface != null) out.put("surface", surface.checkpoint());
    else {
      // Without consolidation the points are already in the cloud files and records.
      out.put("cloud", cloud.checkpoint());
      if (records != null) {
        out.put("records_length", records.checkpoint());
        out.put("records_bytes", records.bytes);
      }
    }
    return out;
  }

  void add(
      long id,
      int source,
      int pixel,
      int rgb,
      DepthFusion.Result result,
      DepthFusion.SurfaceSample geometry)
      throws IOException {
    if (geometry == null) geometry = new DepthFusion.SurfaceSample(null, 0, 0);
    SurfaceConsolidator.Sample p =
        new SurfaceConsolidator.Sample(id, source, pixel, rgb, result, geometry);
    if (evidence && result.spreadDof > 0 && Double.isFinite(result.spreadRss)) {
      spreadRss += result.spreadRss;
      spreadDof += result.spreadDof;
      int bin = angleBin(result.angle);
      binRss[bin] += result.spreadRss;
      binDof[bin] += result.spreadDof;
    }
    if (options.outputDiskSpace != null && staged % OutputDiskSpace.BATCH == 0
        && (records != null || cameraEmission != null || surface != null))
      OutputDiskSpace.require(options,
          OutputDiskSpace.BATCH * (256L + Math.ceilDiv(fusion.cameraCount(), 8)),
          "fusion staging and confidence records");
    staged++;
    if (cameraEmission != null) {
      cameraEmission.add(p);
      return;
    }
    if (surface == null) {
      long before = cloud.count();
      publish(p);
      if (project != null)
        project.data.decision(
            id,
            source,
            pixel,
            result.views,
            cloud.count() == before ? "voxel_duplicate" : "accepted");
    } else {
      surface.add(p);
      if (project != null) project.data.decision(id, source, pixel, result.views, "accepted");
    }
  }

  long count() {
    return finished
        ? cloud.count()
        : cameraEmission != null ? staged : surface == null ? cloud.count() : staged;
  }

  long stagedCount() {
    return staged;
  }

  long mergedCount() {
    return cameraEmission != null
        ? cameraEmission.merged()
        : surface == null ? 0 : surface.mergedCount();
  }

  long movedCount() {
    return surface == null ? 0 : surface.movedCount();
  }

  long confidenceRejected() {
    return confidenceRejected;
  }

  long voxelRejected() {
    return voxelRejected;
  }

  void finish() throws IOException {
    if (cameraEmission != null)
      cameraEmission.finish(
          batch -> {
            long normalStarted = System.nanoTime();
            if (options.surfaceAttributes())
              fusion.parallel(
                  batch.size(),
                  i -> {
                    var p = batch.get(i);
                    var r =
                        new DepthFusion.Result(
                            p.point, p.views, p.pairs, p.sigma, p.shift, "accepted");
                    var s = fusion.surfaceSample(p.source, r);
                    p.normal = s.normal();
                    p.attributes = s.attributes();
                    p.footprint = s.footprint();
                    p.normalSigma = s.sigma();
                  });
            cameraNormalNanos += System.nanoTime() - normalStarted;
            for (var p : batch) publish(p);
          });
    if (surface != null)
      surface.finish(
          new SurfaceConsolidator.Sink() {
            public void point(SurfaceConsolidator.Sample p) throws IOException {
              publish(p);
            }

            public void merged(SurfaceConsolidator.Sample p, long representative)
                throws IOException {
              if (surfaceDecisions != null)
                surfaceDecisions.write(p.id + "," + representative + ",merged,0\n");
            }

            public void removed(SurfaceConsolidator.Sample p) throws IOException {
              if (surfaceDecisions != null)
                surfaceDecisions.write(p.id + ",-1,isolated_off_surface_removed,0\n");
            }
          });
    if (records != null) records.flush();
    if (hostMemory != null)
      Json.write(options.output.resolve("06_fusion/host_output_memory.json"), hostMemory.report());
    Json.write(
        options.output.resolve("06_fusion/confidence.json"),
        Json.object(
            "model",
            PointConfidence.MODEL,
            "point_metadata",
            records == null
                ? "not written (--point-confidence off)"
                : "07_results/point-confidence.bin; version 2, lossless, in cloud order",
            "diagnostic_csv_enabled",
            options.diagnostics,
            "binary_write_nanoseconds",
            records == null ? 0 : records.nanos,
            "binary_record_bytes",
            records == null ? 0 : records.bytes,
            "cloud_staging_nanoseconds",
            cloudWriteNanos,
            "threshold",
            options.confidenceThreshold,
            "confidence_rejected",
            confidenceRejected,
            "unavailable_confidence",
            unavailableConfidence,
            "invalid_score_policy",
            "Retained only when threshold is zero",
            "comparison",
            "strictly below threshold is removed; equality retained",
            "two_view_matching_minimum",
            PointConfidence.TWO_VIEW_MINIMUM,
            "experimental_bidirectional_two_view_enabled",
            PointConfidence.bidirectional(options),
            "experimental_two_view_threshold",
            options.twoViewConfidenceThreshold,
            "experimental_two_view_diagnostics",
            "06_fusion/two_view_confidence.json; the extra gate does not change the final score"
                + " formula",
            "matching_cost_components",
            "SGM: normalized aggregate runner-up margin and curvature before median/subpixel"
                + " refinement, plus independent 3x3 photometric alternatives in the score."
                + " Compact SGM uses a conservative runner-up lower bound for certified proposals."
                + " PatchMatch: best-view photometric margin and curvature at +/-1 and +/-2"
                + " disparity-equivalent depth steps.",
            "propagation",
            "Representative matching evidence retained without merge-count boost; final score"
                + " recomputed with revalidated relative sigma after surface movement",
            "scope",
            "Evidence ranking heuristic, not a probability or surveyed accuracy"));
    if (evidence) uncertaintyReport();
    finished = true;
  }


  /**
   * The run's variance factor: sqrt(sum of weighted residuals / degrees of freedom) of the depths
   * fusion averaged. 1 means the model sigma matches the scatter between cameras; with too few
   * degrees of freedom the model is kept (factor 1, flagged). Written to
   * 07_results/point-uncertainty.json and given to the cloud writer for the sigma fields.
   */
  private void uncertaintyReport() throws IOException {
    boolean calibrated = spreadDof >= MIN_DOF && spreadRss > 0;
    double factor = calibrated ? Math.sqrt(spreadRss / spreadDof) : 1;
    List<Object> bins = new ArrayList<>();
    for (int i = 0; i < binRss.length; i++) {
      // Bins of angleBin: [0, 5), [5, 10), [10, 20), [20, 40), then 40 and above or unknown.
      String range = i < ANGLE_BINS.length
          ? (i == 0 ? 0 : (int) ANGLE_BINS[i - 1]) + "-" + (int) ANGLE_BINS[i] + " deg"
          : (int) ANGLE_BINS[ANGLE_BINS.length - 1] + " deg and above, or unknown";
      bins.add(Json.object("triangulation_angle", range, "degrees_of_freedom", binDof[i],
          "factor", binDof[i] > 0 && binRss[i] > 0 ? Math.sqrt(binRss[i] / binDof[i]) : null));
    }
    Json.write(options.output.resolve("07_results/point-uncertainty.json"), Json.object(
        "factor", factor, "calibrated", calibrated, "degrees_of_freedom", spreadDof,
        "minimum_degrees_of_freedom", MIN_DOF, "by_triangulation_angle", bins,
        "fields", fields.stream().map(f -> f.name).toList(),
        "meaning", "height_sigma and ray_sigma are the fusion model's sigma (triangulation angle, footprint, averaged"
            + " cameras) times this factor: 1-sigma local precision from the images. Orientation and absolute errors"
            + " are not included.",
        "factor_source", "weighted scatter of the depths fusion averaged against the model's inverse variances"));
    System.out.printf(Locale.ROOT, "Point uncertainty: variance factor %.3f from %,d degrees of freedom%s%n",
        factor, spreadDof, calibrated ? "" : " (too few: model sigma kept)");
    uncertaintySummary = String.format(Locale.ROOT,
        "%nPoint uncertainty factor=%.4f (%s, %d degrees of freedom; 07_results/point-uncertainty.json)",
        factor, calibrated ? "calibrated" : "too few to calibrate: model sigma", spreadDof).replace("\r", "");
    cloud.calibration(factor);
  }
  private void publish(SurfaceConsolidator.Sample p) throws IOException {
    double score = PointConfidence.finalScore(p.matchingConfidence, p.sigma);
    if (!PointConfidence.valid(score)) unavailableConfidence++;
    if (!PointConfidence.passes(score, options.confidenceThreshold)) {
      confidenceRejected++;
      if (project != null)
        project.data.decision(p.id, p.source, p.pixel, p.views, "confidence_threshold_rejected");
      if (surfaceDecisions != null)
        surfaceDecisions.write(p.id + ",-1,confidence_threshold_rejected,0\n");
      return;
    }
    if (occupied != null) {
      double size = options.voxel / options.scale;
      Voxel v =
          new Voxel(
              (long) Math.floor(p.point[0] / size),
              (long) Math.floor(p.point[1] / size),
              (long) Math.floor(p.point[2] / size));
      if (hostMemory != null && !occupied.contains(v)) retainOutput((long) occupied.size() + 1);
      if (!occupied.add(v)) {
        voxelRejected++;
        if (surfaceDecisions != null) surfaceDecisions.write(p.id + ",-1,voxel_duplicate,0\n");
        return;
      }
    }
    if (hostMemory != null && occupied == null && options.tiledLasOutput > 0) retainOutput(0);
    if (surfaceDecisions != null)
      surfaceDecisions.write(
          p.id + "," + p.id + ",representative," + (p.surfaceShift * options.scale) + "\n");
    int r = p.rgb >>> 16, g = p.rgb >>> 8 & 255, b = p.rgb & 255;
    if (records != null) records.point(cloud.count(), p, options.scale, score);
    long cloudStarted = System.nanoTime();
    if (PointFields.legacy(fields)) cloud.point(p.point[0], p.point[1], p.point[2], r, g, b, p.normal, p.attributes);
    else cloud.point(p.point[0], p.point[1], p.point[2], r, g, b, values(p, score));
    cloudWriteNanos += System.nanoTime() - cloudStarted;
  }

  /**
   * The generic --point-attributes values of a point. Sigmas are uncalibrated here (the model's
   * relative sigma times the range from the source camera; height: its vertical part along the
   * ray); CloudWriter scales them by the run's factor when the cloud is closed.
   */
  private PointFields.Values values(SurfaceConsolidator.Sample p, double score) {
    PointFields.Values v = new PointFields.Values();
    v.normal = p.normal;
    v.surface = p.attributes;
    v.views = p.views;
    v.confidence = score;
    v.angle = p.angle;
    v.depthSpread = p.depthSpread;
    int camera = p.source >= 0 && p.source < fusion.pairCount() ? fusion.pairCamera(p.source, false) : -1;
    v.source = camera;
    if (camera >= 0 && Double.isFinite(p.sigma)) {
      double[] c = fusion.camera(camera).center();
      double dx = p.point[0] - c[0], dy = p.point[1] - c[1], dz = p.point[2] - c[2];
      double range = Math.sqrt(dx * dx + dy * dy + dz * dz);
      if (range > 0) {
        v.raySigma = p.sigma * range;
        v.heightSigma = v.raySigma * Math.abs(dz) / range;
      }
    }
    // Output units: the writer scales coordinates; these lengths are scaled here.
    v.raySigma *= options.scale;
    v.heightSigma *= options.scale;
    v.depthSpread *= options.scale;
    return v;
  }

  static int angleBin(double angle) {
    int bin = 0;
    while (bin < ANGLE_BINS.length && Double.isFinite(angle) && angle >= ANGLE_BINS[bin]) bin++;
    return Double.isFinite(angle) ? bin : ANGLE_BINS.length;
  }

  private static void closeOnFailure(AutoCloseable resource, Throwable failure) {
    if (resource != null)
      try {
        resource.close();
      } catch (Exception cleanup) {
        failure.addSuppressed(cleanup);
      }
  }

  public void close() throws IOException {
    IOException failure = null;
    for (AutoCloseable resource :
        new AutoCloseable[] {surface, cameraEmission, surfaceDecisions, records})
      if (resource != null)
        try {
          resource.close();
        } catch (Exception e) {
          IOException io = e instanceof IOException x ? x : new IOException(e);
          if (failure == null) failure = io;
          else failure.addSuppressed(io);
        }
    try {
      if (finished && failure == null) cloud.close();
      else if (options.checkpoint != null && options.checkpoint.keepsWork()) cloud.release();
      else cloud.abort();
    } catch (IOException e) {
      if (failure == null) failure = e;
      else failure.addSuppressed(e);
    }
    if (failure != null) throw failure;
    if (finished && options.diagnostics) {
      try (var work =
          WorkProgress.start("Detailed confidence CSV export", cloud.count(), "points")) {
        PointRecords.export(options.output, options.outputCompression, options.cores);
        work.done();
      }
    }
  }
}
