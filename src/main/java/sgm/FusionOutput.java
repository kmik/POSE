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
  private final BufferedWriter confidence;
  private final BufferedWriter surfaceDecisions;
  private final SurfaceConsolidator surface;
  private final Set<Voxel> occupied;
  private long staged;
  private long voxelRejected;
  private boolean finished;

  private record Voxel(long x, long y, long z) {}

  FusionOutput(Options options, DepthFusion fusion, Project project, Crs crs) throws IOException {
    this.options = options;
    this.project = project;
    Path file = options.output.resolve("07_results/point-confidence.csv.gz");
    Files.createDirectories(file.getParent());
    cloud =
        new CloudWriter(
            options.output, options.scale, crs, options.pointAttributes.equals("surface"));
    BufferedWriter openedConfidence = null, openedDecisions = null;
    SurfaceConsolidator openedSurface = null;
    try {
      openedConfidence =
          new BufferedWriter(
              new OutputStreamWriter(
                  Compression.gzip(Files.newOutputStream(file), options.outputCompression),
                  StandardCharsets.UTF_8));
      openedConfidence.write(
          "point_id,candidate_id,fusion_pair_id,x,y,z,red,green,blue,supporting_views,independent_pairs,estimated_relative_stereo_sigma,refinement_shift,normal_x,normal_y,normal_z,surface_shift,merged_observations,roughness_rms,surface_variation,planarity,normal_radius,normal_neighbors,surface_status,normal_trimmed\n");
      openedSurface =
          options.surfaceConsolidation.equals("normal")
              ? new SurfaceConsolidator(options, fusion)
              : null;
      openedDecisions =
          openedSurface != null && options.storage.equals("debug")
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
      closeOnFailure(openedConfidence, failure);
      try {
        cloud.abort();
      } catch (IOException cleanup) {
        failure.addSuppressed(cleanup);
      }
      throw failure;
    }
    confidence = openedConfidence;
    surface = openedSurface;
    surfaceDecisions = openedDecisions;
    occupied = options.voxel > 0 ? new HashSet<>() : null;
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
    staged++;
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
    return finished ? cloud.count() : surface == null ? cloud.count() : staged;
  }

  long stagedCount() {
    return staged;
  }

  long mergedCount() {
    return surface == null ? 0 : surface.mergedCount();
  }

  long movedCount() {
    return surface == null ? 0 : surface.movedCount();
  }

  long voxelRejected() {
    return voxelRejected;
  }

  void finish() throws IOException {
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
          });
    confidence.flush();
    finished = true;
  }

  private void publish(SurfaceConsolidator.Sample p) throws IOException {
    if (occupied != null) {
      double size = options.voxel / options.scale;
      Voxel v =
          new Voxel(
              (long) Math.floor(p.point[0] / size),
              (long) Math.floor(p.point[1] / size),
              (long) Math.floor(p.point[2] / size));
      if (!occupied.add(v)) {
        voxelRejected++;
        if (surfaceDecisions != null) surfaceDecisions.write(p.id + ",-1,voxel_duplicate,0\n");
        return;
      }
    }
    if (surfaceDecisions != null)
      surfaceDecisions.write(
          p.id + "," + p.id + ",representative," + (p.surfaceShift * options.scale) + "\n");
    int r = p.rgb >>> 16, g = p.rgb >>> 8 & 255, b = p.rgb & 255;
    confidence.write(
        cloud.count()
            + ","
            + p.id
            + ","
            + p.source
            + ","
            + (p.point[0] * options.scale)
            + ","
            + (p.point[1] * options.scale)
            + ","
            + (p.point[2] * options.scale)
            + ","
            + r
            + ","
            + g
            + ","
            + b
            + ","
            + p.views
            + ","
            + p.pairs
            + ","
            + p.sigma
            + ","
            + (p.shift * options.scale)
            + ","
            + (p.normal == null
                ? "NaN,NaN,NaN"
                : p.normal[0] + "," + p.normal[1] + "," + p.normal[2])
            + ","
            + (p.surfaceShift * options.scale)
            + ","
            + p.merged
            + ","
            + p.attributes.csv(options.scale)
            + "\n");
    cloud.point(p.point[0], p.point[1], p.point[2], r, g, b, p.normal, p.attributes);
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
    for (AutoCloseable resource : new AutoCloseable[] {surface, surfaceDecisions, confidence})
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
      else cloud.abort();
    } catch (IOException e) {
      if (failure == null) failure = e;
      else failure.addSuppressed(e);
    }
    if (failure != null) throw failure;
  }
}
