package sgm;

import java.awt.image.BufferedImage;
import java.io.*;
import java.nio.*;
import java.nio.channels.FileChannel;
import java.nio.file.*;
import java.util.*;
import javax.imageio.ImageIO;

/**
 * Ground classification of the final cloud (--classify-ground, C1), with no reference terrain.
 *
 * <p>A progressive morphological filter (after SMRF, Pingel et al. 2013) runs on a grid of robust
 * low points, using cues only the reconstruction has:
 *
 * <ul>
 *   <li><b>Free space:</b> every camera ray reached its point, so terrain cannot stand above the
 *       ray anywhere along it. The second-lowest ray height per cell caps the terrain; it rejects
 *       canopy "low points" beside gaps and caps the interpolation under canopy.
 *   <li><b>Support:</b> candidates need two or more views and a normal that is not near
 *       vertical.
 *   <li><b>Self-tuning:</b> cell size from the point spacing, height tolerance from the measured
 *       spread of points about the terrain, low-outlier limits from the same spread.
 * </ul>
 *
 * Checks without a reference: split-half repeatability (terrain from even vs odd stereo pairs),
 * rays still blocked by the final terrain, points far below it, and how far every cell is from
 * observed ground. Writes LAS classes 2 (ground) and 7 (low noise) in place, dtm.tif, chm.tif,
 * ground_support.tif, ground_uncertainty.tif and ground.json in 07_results.
 */
final class GroundClassification {
  private GroundClassification() {}

  /** Points relative to an origin; per-point cues from the confidence sidecar when available. */
  static final class Cloud {
    int n;
    double ox, oy;
    float[] x, y, z, nz;
    short[] views;
    int[] camA, camB, source;
    boolean sidecar;
    // LAS layout for the in-place class update.
    long dataOffset;
    double zScale, zOffset;
    int recordLength;
  }

  record Grid(int w, int h, double cell, double x0, double y1) {
    int col(double x) {
      return (int) Math.floor((x - x0) / cell);
    }

    int row(double y) {
      return (int) Math.floor((y1 - y) / cell);
    }

    int index(double x, double y) {
      if (!Double.isFinite(x + y)) return -1;
      int c = col(x), r = row(y);
      return c < 0 || r < 0 || c >= w || r >= h ? -1 : r * w + c;
    }
  }

  static Map<String, Object> run(Options o, List<ImageFrame> frames, Crs crs) throws IOException {
    Path las = o.output.resolve("dense.las");
    if (!Files.isRegularFile(las)) las = o.output.resolve("07_results/dense.las");
    Path results = o.output.resolve("07_results"), report = results.resolve("ground.json");
    Files.createDirectories(results);
    if (crs == null) {
      Map<String, Object> skipped =
          Json.object(
              "status",
              "skipped",
              "reason",
              "Ground classification needs a georeferenced (z-up) frame; the cloud stayed in its"
                  + " local reconstruction frame");
      Json.write(report, skipped);
      System.out.println("Ground classification skipped: no georeferenced z-up frame.");
      return skipped;
    }
    try (WorkProgress work = WorkProgress.start("Ground classification", 0, "steps")) {
      Map<String, Object> result = classify(o, frames, las, results.resolve("point-confidence.bin"), results, crs.epsg);
      Json.write(report, result);
      work.done();
      return result;
    }
  }

  static Map<String, Object> classify(
      Options o, List<ImageFrame> frames, Path las, Path sidecar, Path results, int epsg)
      throws IOException {
    threads = Math.max(1, Integer.getInteger("sgm.groundThreads", o.cores));
    long started = System.nanoTime();
    Cloud cloud = read(las, sidecar);
    double[][] cameras = new double[frames.size()][];
    for (ImageFrame f : frames)
      if (f.camera.registered) {
        double[] c = f.camera.center();
        cameras[f.id] = new double[] {c[0] * o.scale - cloud.ox, c[1] * o.scale - cloud.oy, c[2] * o.scale};
      }

    // Cell size from the point spacing over the occupied 1 m footprint.
    Grid metre = grid(cloud, 1);
    BitSet occupiedMetre = new BitSet(metre.w * metre.h);
    for (int i = 0; i < cloud.n; i++) {
      int k = metre.index(cloud.x[i], cloud.y[i]);
      if (k >= 0) occupiedMetre.set(k);
    }
    double spacing = Math.sqrt(Math.max(1, occupiedMetre.cardinality()) / (double) cloud.n);
    double cell =
        o.groundCell > 0 ? o.groundCell : Math.max(.25, Math.min(2, Math.ceil(3 * spacing * 20) / 20));
    Grid g = grid(cloud, cell);
    while ((long) g.w * g.h > 16L << 20) g = grid(cloud, g.cell * 1.5);
    cell = g.cell;
    double slope = Math.tan(Math.toRadians(o.groundMaxSlope));
    int radius = Math.max(1, (int) Math.ceil(o.groundMaxObject / 2 / cell));

    Core r = core(cloud, cameras, g, slope, radius);
    // GS1: per-pair vertical offsets of the ground against the consensus; optional correction.
    Map<String, Object> pairReport = null;
    if (cloud.sidecar) {
      PairOffsets.Fit fit = PairOffsets.fit(cloud, g, r);
      pairReport = fit.report();
      if (o.groundPairCorrection && fit.pairs() > 0) {
        writeHeights(las, cloud, PairOffsets.apply(cloud, fit));
        r = core(cloud, cameras, g, slope, radius);
        PairOffsets.Fit after = PairOffsets.fit(cloud, g, r);
        pairReport.put("applied", true);
        pairReport.put("after_correction", after.summary());
        System.out.println(
            "Ground pair correction applied: " + fit.line() + "; afterwards " + after.line());
      } else {
        pairReport.put("applied", false);
        System.out.println("Ground pair offsets: " + fit.line());
      }
    }
    boolean[] usable = r.usable;
    Terrain full = r.full;
    double top = r.top, tolerance = r.tolerance, sigma = r.sigma, threshold = r.threshold;
    double lowLimit = r.lowLimit;
    List<Double> differences = r.differences;
    float[] uncertainty = r.uncertainty;
    byte[] classes = r.classes;
    long groundPoints = r.groundPoints, noisePoints = r.noisePoints;
    writeClasses(las, cloud, classes);

    // Rays the final terrain still blocks (free-space violations).
    long blocked = 0, checked = 0;
    float[] blockedCells = new float[g.w * g.h];
    if (cloud.sidecar) {
      long[] counts = violations(cloud, cameras, g, usable, top + 1, full.dtm, tolerance, blockedCells);
      blocked = counts[0];
      checked = counts[1];
    }

    // Footprint, observed vs interpolated ground, distance to observed ground.
    BitSet footprint = new BitSet(g.w * g.h);
    for (int i = 0; i < cloud.n; i++) {
      int k = g.index(cloud.x[i], cloud.y[i]);
      if (k >= 0) footprint.set(k);
    }
    footprint = dilate(footprint, g, 2);
    float[] distance = distance(full.ground, g);
    float[] support = new float[g.w * g.h], dtm = new float[g.w * g.h], chm = new float[g.w * g.h];
    float[] surface = new float[g.w * g.h];
    Arrays.fill(surface, Float.NEGATIVE_INFINITY);
    for (int i = 0; i < cloud.n; i++) {
      int k = g.index(cloud.x[i], cloud.y[i]);
      if (k >= 0 && classes[i] != 7) surface[k] = Math.max(surface[k], cloud.z[i]);
    }
    long footprintCells = 0, observedCells = 0, flaggedCells = 0;
    double maxDistance = 0;
    for (int k = 0; k < support.length; k++) {
      if (!footprint.get(k)) {
        support[k] = dtm[k] = chm[k] = Float.NaN;
        continue;
      }
      footprintCells++;
      dtm[k] = full.dtm[k];
      chm[k] = Float.isFinite(surface[k]) ? Math.max(0, surface[k] - full.dtm[k]) : Float.NaN;
      if (blockedCells[k] >= 2) {
        support[k] = 3;
        flaggedCells++;
      } else if (full.ground[k]) {
        support[k] = 1;
        observedCells++;
      } else support[k] = 2;
      maxDistance = Math.max(maxDistance, distance[k] * cell);
    }
    double x0 = g.x0 + cloud.ox, y1 = g.y1 + cloud.oy;
    DsmRaster.writeTiff(results.resolve("dtm.tif"), g.w, g.h, dtm, x0, y1, cell, epsg);
    DsmRaster.writeTiff(results.resolve("chm.tif"), g.w, g.h, chm, x0, y1, cell, epsg);
    DsmRaster.writeTiff(results.resolve("ground_support.tif"), g.w, g.h, support, x0, y1, cell, epsg);
    DsmRaster.writeTiff(
        results.resolve("ground_uncertainty.tif"), g.w, g.h, uncertainty, x0, y1, cell, epsg);
    for (String name : new String[] {"dtm", "chm", "ground_support", "ground_uncertainty"})
      Files.writeString(
          results.resolve(name + ".tfw"),
          String.format(
              Locale.ROOT, "%.12f%n0%n0%n%.12f%n%.12f%n%.12f%n", cell, -cell, x0 + cell / 2, y1 - cell / 2));
    preview(results.resolve("dtm.png"), dtm, g, false);
    preview(results.resolve("ground_support.png"), support, g, true);

    Collections.sort(differences);
    double takeoff = takeoffGround(frames);
    List<Double> terrain = new ArrayList<>();
    for (int k = 0; k < dtm.length; k++) if (support[k] == 1) terrain.add((double) dtm[k]);
    Collections.sort(terrain);
    Map<String, Object> result =
        Json.object(
            "status", "complete",
            "method",
            "Progressive morphological filter on robust low points (after SMRF), free-space"
                + " ray bound, self-tuned tolerances; no reference terrain used",
            "points", cloud.n,
            "ground_points", groundPoints,
            "ground_fraction", groundPoints / (double) cloud.n,
            "low_noise_points", noisePoints,
            "las_classes", "2 ground, 7 low noise, 1 other (written in place in dense.las)",
            "cell_size", cell,
            "point_spacing", spacing,
            "max_terrain_slope_degrees", o.groundMaxSlope,
            "max_object_size", o.groundMaxObject,
            "ground_noise_sigma", sigma,
            "ground_tolerance", threshold,
            "low_noise_limit", lowLimit,
            "free_space_bound", cloud.sidecar ? "camera rays of supporting cameras" : "unavailable (no point-confidence.bin)",
            "checks",
            Json.object(
                "footprint_cells", footprintCells,
                "observed_ground_cells", observedCells,
                "observed_ground_fraction", observedCells / (double) Math.max(1, footprintCells),
                "interpolated_cells", footprintCells - observedCells - flaggedCells,
                "flagged_cells_blocking_rays", flaggedCells,
                "max_distance_to_observed_ground", maxDistance,
                "split_half_cells", differences.size(),
                "split_half_median_abs_difference", quantile(differences, .5),
                "split_half_p95_abs_difference", quantile(differences, .95),
                "split_half_split", cloud.sidecar ? "even vs odd stereo pair" : "even vs odd point order (weaker)",
                "rays_checked", checked,
                "rays_blocked_by_terrain", blocked,
                "blocked_ray_fraction", checked == 0 ? null : blocked / (double) checked,
                "points_below_terrain_as_noise", noisePoints,
                "dji_takeoff_ground_altitude", Double.isFinite(takeoff) ? takeoff : null,
                "observed_terrain_p5_p50_p95",
                terrain.isEmpty() ? null : new double[] {quantile(terrain, .05), quantile(terrain, .5), quantile(terrain, .95)},
                "note",
                "No reference terrain is used. The takeoff altitude (EXIF absolute minus DJI relative"
                    + " altitude) is reported for orientation only: the takeoff spot may lie outside"
                    + " the mapped area."),
            "rasters",
            Json.object(
                "dtm", "dtm.tif (terrain; interpolated under canopy)",
                "chm", "chm.tif (highest point minus terrain)",
                "support", "ground_support.tif: 1 observed ground, 2 interpolated, 3 terrain blocks camera rays",
                "uncertainty", "ground_uncertainty.tif: half the split-half difference where both halves observe ground"),
            "elapsed_seconds", (System.nanoTime() - started) / 1e9);
    if (r.islands != null) result.put("canopy_islands", r.islands);
    if (pairReport != null) result.put("pair_offsets", pairReport);
    // GS2 (opt-in) last: it removes duplicate ground points, so the cloud arrays end here.
    if (o.groundAwareFusion)
      result.put("ground_aware_fusion", GroundSheet.run(las, sidecar, cloud, g, r, spacing, o.groundFusionRadius));
    System.out.printf(
        Locale.ROOT,
        "Ground classification: %,d of %,d points ground (%.1f%%), %,d low noise; observed ground"
            + " in %.1f%% of cells, split-half median |dZ| %.3f m (p95 %.3f m), %.2f%% rays blocked;"
            + " cell %.2f m, tolerance %.2f m%n",
        groundPoints,
        cloud.n,
        100. * groundPoints / cloud.n,
        noisePoints,
        100. * observedCells / Math.max(1, footprintCells),
        quantile(differences, .5),
        quantile(differences, .95),
        checked == 0 ? 0 : 100. * blocked / checked,
        cell,
        threshold);
    return result;
  }

  /** Terrain, tolerances, split-half and classes for the cloud's current heights. */
  static final class Core {
    boolean[] usable;
    Terrain full;
    double top, tolerance, sigma, threshold, lowLimit;
    List<Double> differences = new ArrayList<>();
    float[] uncertainty;
    byte[] classes;
    long groundPoints, noisePoints;
    Map<String, Object> islands;
  }

  static Core core(Cloud cloud, double[][] cameras, Grid g, double slope, int radius) {
    Core r = new Core();
    double cell = g.cell;
    boolean[] all = new boolean[cloud.n];
    Arrays.fill(all, true);
    // Pass 1: preliminary terrain without free-space carving.
    float[] candidates = candidates(cloud, g, all);
    lowOutliers(candidates, g, slope);
    boolean[] ground0 = smrf(candidates, g, slope, radius);
    slopeEnvelope(candidates, ground0, g, slope, .5);
    float[] dtm0 = interpolate(candidates, ground0, g);
    double sigma0 = spread(cloud, g, dtm0, ground0);
    // Points far below the preliminary terrain are noise; they must not carve.
    r.lowLimit = Math.max(1, 4 * sigma0);
    r.usable = new boolean[cloud.n];
    for (int i = 0; i < cloud.n; i++) {
      double t = sample(dtm0, g, cloud.x[i], cloud.y[i]);
      r.usable[i] = !(cloud.z[i] < t - r.lowLimit);
    }
    // Free space: second-lowest ray height per cell.
    r.top = Double.NEGATIVE_INFINITY;
    for (int i = 0; i < cloud.n; i++) r.top = Math.max(r.top, cloud.z[i]);
    float[][] rays = cloud.sidecar ? carve(cloud, cameras, g, r.usable, r.top + 1) : null;
    float[] bound = rays == null ? null : rays[1];
    r.tolerance = Math.max(.3, 3 * sigma0);
    // Highest point per cell: the canopy surface the island test compares ground patches with.
    float[] surface = new float[g.w * g.h];
    Arrays.fill(surface, Float.NaN);
    for (int i = 0; i < cloud.n; i++) {
      int k = g.index(cloud.x[i], cloud.y[i]);
      if (k >= 0 && r.usable[i] && !(cloud.z[i] <= surface[k])) surface[k] = cloud.z[i];
    }
    Islands islands = new Islands(surface);

    // Pass 2: final terrain, with candidates above the ray bound rejected and the bound applied.
    r.full = terrain(cloud, g, r.usable, bound, r.tolerance, slope, radius, islands);
    r.islands = islands.report;
    r.sigma = spread(cloud, g, r.full.dtm, r.full.ground);
    r.threshold = Math.max(.1, 3 * r.sigma);
    float[] gradient = gradient(r.full.dtm, g);

    // Split-half repeatability: even vs odd stereo pairs (point order without the sidecar).
    boolean[] even = new boolean[cloud.n], odd = new boolean[cloud.n];
    for (int i = 0; i < cloud.n; i++) {
      boolean a = ((cloud.sidecar ? cloud.source[i] : i) & 1) == 0;
      even[i] = r.usable[i] && a;
      odd[i] = r.usable[i] && !a;
    }
    Terrain halfA = terrain(cloud, g, even, bound, r.tolerance, slope, radius, new Islands(surface)),
        halfB = terrain(cloud, g, odd, bound, r.tolerance, slope, radius, new Islands(surface));
    r.uncertainty = new float[g.w * g.h];
    Arrays.fill(r.uncertainty, Float.NaN);
    for (int k = 0; k < r.uncertainty.length; k++)
      if (halfA.ground[k] && halfB.ground[k]) {
        double d = Math.abs(halfA.dtm[k] - halfB.dtm[k]);
        r.uncertainty[k] = (float) (d / 2);
        r.differences.add(d);
      }

    // Classes: 2 ground, 7 low noise, 1 everything else.
    r.classes = new byte[cloud.n];
    for (int i = 0; i < cloud.n; i++) {
      double t = sample(r.full.dtm, g, cloud.x[i], cloud.y[i]);
      int k = g.index(cloud.x[i], cloud.y[i]);
      double e = r.threshold + (k < 0 ? 0 : gradient[k] * cell);
      double d = cloud.z[i] - t;
      if (!Double.isFinite(t)) r.classes[i] = 1;
      else if (d < -Math.max(r.lowLimit, 3 * e)) {
        r.classes[i] = 7;
        r.noisePoints++;
      } else if (Math.abs(d) <= e) {
        r.classes[i] = 2;
        r.groundPoints++;
      } else r.classes[i] = 1;
    }
    return r;
  }

  record Terrain(float[] dtm, boolean[] ground) {}

  /**
   * Canopy islands: in closed canopy the lowest visible points are depressions between crowns,
   * and they can pass every slope test because no real ground is near. Ground cells are grouped
   * into connected patches; large smooth patches are anchors (reliable ground) and give a coarse
   * terrain trend. Every other patch that is elevated above that trend, or far from any anchor,
   * must look like a real gap: the surrounding canopy (75th percentile of the highest points in a
   * 5 m margin) must stand at least the gap depth above it. The gap depth is half the canopy
   * height measured over the anchors' trend (at least 3 m), or --ground-gap-depth. Patches that
   * fail are dropped and the terrain there is interpolated instead.
   */
  static final class Islands {
    final float[] surface;
    final Map<String, Object> report = new LinkedHashMap<>();

    Islands(float[] surface) {
      this.surface = surface;
    }

    void filter(float[] z, boolean[] ground, Grid g, double tolerance) {
      int n = ground.length;
      int[] label = new int[n];
      Arrays.fill(label, -1);
      List<int[]> patches = new ArrayList<>();
      int[] queue = new int[n];
      for (int seed = 0; seed < n; seed++) {
        if (!ground[seed] || label[seed] >= 0) continue;
        int head = 0, tail = 0, id = patches.size();
        queue[tail++] = seed;
        label[seed] = id;
        while (head < tail) {
          int k = queue[head++], r = k / g.w, c = k % g.w;
          for (int dr = -1; dr <= 1; dr++)
            for (int dc = -1; dc <= 1; dc++) {
              int rr = r + dr, cc = c + dc;
              if (rr < 0 || cc < 0 || rr >= g.h || cc >= g.w) continue;
              int q = rr * g.w + cc;
              if (ground[q] && label[q] < 0) {
                label[q] = id;
                queue[tail++] = q;
              }
            }
        }
        patches.add(Arrays.copyOf(queue, tail));
      }
      double cellArea = g.cell * g.cell, smooth = Math.max(.25, tolerance);
      boolean[] anchor = new boolean[n];
      boolean[] isAnchor = new boolean[patches.size()];
      double anchorArea = 0;
      for (int p = 0; p < patches.size(); p++) {
        int[] cells = patches.get(p);
        // Large and locally smooth: terrain may curve over a big patch, crowns are rough within 4 m.
        if (cells.length * cellArea < 25 || roughness(cells, z, g) > smooth) continue;
        isAnchor[p] = true;
        anchorArea += cells.length * cellArea;
        for (int k : cells) anchor[k] = true;
      }
      // Coarse terrain trend from the anchors (5 m cells, filled in between).
      float[] trend = null;
      if (anchorArea > 0) {
        int factor = Math.max(1, (int) Math.round(5 / g.cell));
        Grid coarse = new Grid((g.w + factor - 1) / factor, (g.h + factor - 1) / factor, g.cell * factor, g.x0, g.y1);
        Map<Integer, List<Float>> bins = new HashMap<>();
        for (int k = 0; k < n; k++)
          if (anchor[k])
            bins.computeIfAbsent((k / g.w / factor) * coarse.w + (k % g.w) / factor, q -> new ArrayList<>()).add(z[k]);
        float[] values = new float[coarse.w * coarse.h];
        Arrays.fill(values, Float.NaN);
        for (var e : bins.entrySet()) {
          List<Float> v = e.getValue();
          Collections.sort(v);
          values[e.getKey()] = v.get(v.size() / 2);
        }
        float[] filled = fillHoles(values, coarse);
        trend = new float[n];
        for (int k = 0; k < n; k++)
          trend[k] =
              (float) sample(filled, coarse, g.x0 + (k % g.w + .5) * g.cell, g.y1 - (k / g.w + .5) * g.cell);
      }
      // Per patch: suspect (elevated above the anchor trend, or far from any anchor), its level,
      // and (for the diagnostics) its depth below the surrounding canopy.
      float[] distance = anchorArea > 0 ? distance(anchor, g) : null;
      int margin = (int) Math.ceil(5 / g.cell);
      int count = patches.size();
      boolean[] suspect = new boolean[count];
      double[] level = new double[count], depthBelow = new double[count];
      double[] nearestOf = new double[count], elevationOf = new double[count], overOf = new double[count];
      Arrays.fill(overOf, Double.NaN);
      Arrays.fill(depthBelow, Double.NaN);

      for (int p = 0; p < count; p++) {
        if (isAnchor[p]) continue;
        int[] cells = patches.get(p);
        float[] own = new float[cells.length];
        double nearest = Double.POSITIVE_INFINITY;
        List<Double> above = new ArrayList<>();
        for (int i = 0; i < cells.length; i++) {
          own[i] = z[cells[i]];
          if (trend != null) above.add((double) (z[cells[i]] - trend[cells[i]]));
          if (distance != null && Float.isFinite(distance[cells[i]])) nearest = Math.min(nearest, distance[cells[i]] * g.cell);
        }
        Arrays.sort(own);
        level[p] = own[own.length / 2];
        double elevation = 0;
        if (trend != null) {
          Collections.sort(above);
          elevation = quantile(above, .5);
        }
        nearestOf[p] = nearest;
        elevationOf[p] = elevation;
        suspect[p] = trend == null || nearest > 30 || elevation > Math.max(1, 2 * tolerance);
        if (!suspect[p]) continue;
        int r0 = Integer.MAX_VALUE, r1 = -1, c0 = Integer.MAX_VALUE, c1 = -1;
        for (int k : cells) {
          r0 = Math.min(r0, k / g.w);
          r1 = Math.max(r1, k / g.w);
          c0 = Math.min(c0, k % g.w);
          c1 = Math.max(c1, k % g.w);
        }
        List<Float> ring = new ArrayList<>();
        for (int r = Math.max(0, r0 - margin); r <= Math.min(g.h - 1, r1 + margin); r++)
          for (int c = Math.max(0, c0 - margin); c <= Math.min(g.w - 1, c1 + margin); c++) {
            int k = r * g.w + c;
            if (label[k] != p && Float.isFinite(surface[k])) ring.add(surface[k]);
          }
        if (ring.isEmpty()) continue;
        Collections.sort(ring);
        depthBelow[p] = ring.get((ring.size() - 1) * 3 / 4) - level[p];

      }
      // Decision: ground cannot stand far above the terrain of the trusted ground (anchors and
      // unsuspected patches) nearby. The allowed rise grows with the distance to that ground
      // (0.25 m per metre, about 14 degrees of average slope), so terrain seen through gaps on a
      // slope survives while crowns standing metres above the surrounding terrain do not.
      boolean[] trusted = new boolean[n];
      boolean any = false;
      for (int p = 0; p < count; p++)
        if (isAnchor[p] || !suspect[p])
          for (int k : patches.get(p)) {
            trusted[k] = true;
            any = true;
          }
      float[] alternative = null, away = null;
      if (any) {
        float[] known = new float[n];
        for (int k = 0; k < n; k++) known[k] = trusted[k] ? z[k] : Float.NaN;
        alternative = fillHoles(known, g);
        away = distance(trusted, g);
      }
      double base = Math.max(1, 2 * tolerance), perMetre = .25;
      long tested = 0, rejected = 0;
      double rejectedArea = 0;
      List<Double> rejectedRise = new ArrayList<>();
      for (int p = 0; p < count; p++) {
        if (!suspect[p] || alternative == null) continue;
        tested++;
        int[] cells = patches.get(p);
        List<Double> over = new ArrayList<>(), far = new ArrayList<>();
        for (int k : cells) {
          over.add(z[k] - (double) alternative[k]);
          far.add(Float.isFinite(away[k]) ? away[k] * g.cell : 1e9);
        }
        Collections.sort(over);
        Collections.sort(far);
        double rise = quantile(over, .5), allowed = Math.max(base, perMetre * quantile(far, .5));
        overOf[p] = rise;
        if (rise <= allowed) continue;
        rejected++;
        rejectedRise.add(rise);
        rejectedArea += cells.length * cellArea;
        for (int k : cells) ground[k] = false;
      }
      Collections.sort(rejectedRise);
      String debug = System.getProperty("sgm.islandsDebug");
      if (debug != null) debugDump(Path.of(debug), patches, isAnchor, suspect, level, depthBelow, nearestOf, elevationOf, overOf, ground, z, g, perMetre);
      report.put("patches", count);
      report.put("anchor_area_m2", anchorArea);
      report.put("patches_tested", tested);
      report.put("patches_rejected_as_canopy", rejected);
      report.put("rejected_area_m2", rejectedArea);
      report.put("rejected_median_rise_m", rejectedRise.isEmpty() ? null : quantile(rejectedRise, .5));
      report.put("rise_limit", String.format(Locale.ROOT, "max(%.2f m, 0.25 x distance to trusted ground)", base));
      report.put(
          "rule",
          "Trusted ground: large patches that are smooth within 4 m tiles, and patches near them at"
              + " their terrain trend. Every other ground patch is rejected as canopy when its median"
              + " height above the terrain interpolated from the trusted ground exceeds max(1 m, 2 x"
              + " tolerance) or 0.25 m per metre of distance to the trusted ground, whichever is larger.");
    }

    private static int dumps;

    /** Diagnostics (-Dsgm.islandsDebug=FILE): one row per patch; FILE-N.csv per filter call. */
    private static void debugDump(
        Path file, List<int[]> patches, boolean[] isAnchor, boolean[] suspect, double[] level, double[] depthBelow,
        double[] nearest, double[] elevation, double[] over, boolean[] ground, float[] z, Grid g, double depth) {
      try (BufferedWriter w = Files.newBufferedWriter(Path.of(file + "-" + dumps++ + ".csv"))) {
        w.write("patch,x,y,area_m2,level,plane_rms,anchor,suspect,nearest_anchor_m,above_trend,depth_below_canopy,above_alternative,kept,rise_per_metre\n");
        for (int p = 0; p < patches.size(); p++) {
          int[] cells = patches.get(p);
          double x = 0, y = 0;
          for (int k : cells) {
            x += g.x0 + (k % g.w + .5) * g.cell;
            y += g.y1 - (k / g.w + .5) * g.cell;
          }
          w.write(String.format(Locale.ROOT, "%d,%.2f,%.2f,%.3f,%.2f,%.3f,%b,%b,%.1f,%.2f,%.2f,%.2f,%b,%.2f%n",
              p, x / cells.length, y / cells.length, cells.length * g.cell * g.cell, level[p], planeRms(cells, z, g),
              isAnchor[p], suspect[p], nearest[p], elevation[p], depthBelow[p], over[p], ground[cells[0]], depth));
        }
      } catch (IOException e) {
        System.err.println("islands debug: " + e);
      }
    }

    /**
     * Local roughness: the median plane RMS over 4 m tiles of the patch (tiles with at least a
     * quarter of their cells). Real terrain may curve over a large patch but is smooth locally;
     * crown tops are not. Falls back to the whole-patch plane for small patches.
     */
    static double roughness(int[] cells, float[] z, Grid g) {
      int side = Math.max(4, (int) Math.round(4 / g.cell));
      Map<Long, List<Integer>> tiles = new HashMap<>();
      for (int k : cells) tiles.computeIfAbsent((long) (k / g.w / side) * 1_000_003L + (k % g.w) / side, q -> new ArrayList<>()).add(k);
      List<Double> rms = new ArrayList<>();
      for (List<Integer> tile : tiles.values())
        if (tile.size() >= side * side / 4) rms.add(planeRms(tile.stream().mapToInt(Integer::intValue).toArray(), z, g));
      if (rms.isEmpty()) return planeRms(cells, z, g);
      Collections.sort(rms);
      return rms.get(rms.size() / 2);
    }

    private static double planeRms(int[] cells, float[] z, Grid g) {
      double[][] a = new double[3][3];
      double[] b = new double[3];
      double mr = 0, mc = 0;
      for (int k : cells) {
        mr += (double) (k / g.w) / cells.length;
        mc += (double) (k % g.w) / cells.length;
      }
      for (int k : cells) {
        double[] j = {1, (k % g.w - mc) * g.cell, (k / g.w - mr) * g.cell};
        for (int p = 0; p < 3; p++) {
          b[p] += j[p] * z[k];
          for (int q = 0; q < 3; q++) a[p][q] += j[p] * j[q];
        }
      }
      for (int p = 1; p < 3; p++) a[p][p] += 1e-9;
      double[] plane = Math3.solve(a, b);
      if (plane == null) return Double.POSITIVE_INFINITY;
      double sum = 0;
      for (int k : cells) {
        double e = z[k] - plane[0] - plane[1] * (k % g.w - mc) * g.cell - plane[2] * (k / g.w - mr) * g.cell;
        sum += e * e;
      }
      return Math.sqrt(sum / cells.length);
    }
  }

  /** Candidates (optionally above the ray bound rejected), SMRF, interpolation, ray cap. */
  static Terrain terrain(
      Cloud cloud,
      Grid g,
      boolean[] use,
      float[] bound,
      double tolerance,
      double slope,
      int radius,
      Islands islands) {
    float[] candidates = candidates(cloud, g, use);
    if (bound != null)
      for (int k = 0; k < candidates.length; k++)
        if (Float.isFinite(bound[k]) && candidates[k] > bound[k] + tolerance) candidates[k] = Float.NaN;
    lowOutliers(candidates, g, slope);
    boolean[] ground = smrf(candidates, g, slope, radius);
    slopeEnvelope(candidates, ground, g, slope, Math.max(.5, tolerance));
    if (islands != null) islands.filter(candidates, ground, g, tolerance);
    float[] dtm = interpolate(candidates, ground, g);
    if (bound != null)
      for (int k = 0; k < dtm.length; k++)
        if (Float.isFinite(bound[k]) && dtm[k] > bound[k] + tolerance)
          dtm[k] = (float) (bound[k] + tolerance);
    return new Terrain(dtm, ground);
  }

  /** Robust low point per cell: second lowest of the eligible points (lowest if only one). */
  static float[] candidates(Cloud c, Grid g, boolean[] use) {
    float[] lowest = new float[g.w * g.h], second = new float[g.w * g.h];
    Arrays.fill(lowest, Float.POSITIVE_INFINITY);
    Arrays.fill(second, Float.POSITIVE_INFINITY);
    for (int i = 0; i < c.n; i++) {
      if (!use[i]) continue;
      if (c.views != null && c.views[i] > 0 && c.views[i] < 2) continue;
      // A normal within 30 degrees of horizontal is a trunk or crown side, not terrain.
      if (c.nz != null && Float.isFinite(c.nz[i]) && Math.abs(c.nz[i]) < .5) continue;
      int k = g.index(c.x[i], c.y[i]);
      if (k < 0) continue;
      float z = c.z[i];
      if (z < lowest[k]) {
        second[k] = lowest[k];
        lowest[k] = z;
      } else if (z < second[k]) second[k] = z;
    }
    float[] out = new float[lowest.length];
    for (int k = 0; k < out.length; k++)
      out[k] =
          Float.isFinite(second[k]) ? second[k] : Float.isFinite(lowest[k]) ? lowest[k] : Float.NaN;
    return out;
  }

  /** Removes candidates far below the median of their 5x5 neighbourhood. */
  static void lowOutliers(float[] z, Grid g, double slope) {
    float[] copy = z.clone();
    float[] window = new float[25];
    double limit = 1 + slope * 2 * g.cell;
    for (int r = 0; r < g.h; r++)
      for (int c = 0; c < g.w; c++) {
        int k = r * g.w + c;
        if (!Float.isFinite(copy[k])) continue;
        int n = 0;
        for (int dr = -2; dr <= 2; dr++)
          for (int dc = -2; dc <= 2; dc++) {
            int rr = r + dr, cc = c + dc;
            if (rr < 0 || cc < 0 || rr >= g.h || cc >= g.w) continue;
            float v = copy[rr * g.w + cc];
            if (Float.isFinite(v)) window[n++] = v;
          }
        if (n < 5) continue;
        Arrays.sort(window, 0, n);
        if (window[n / 2] - copy[k] > limit) z[k] = Float.NaN;
      }
  }

  /**
   * Progressive opening with square windows of radius 1..radius cells. A cell is an object when
   * an opening step lowers it by more than slope x window radius x cell size. Returns the ground
   * mask (valid candidates that are not objects).
   */
  static boolean[] smrf(float[] candidates, Grid g, double slope, int radius) {
    boolean[] object = new boolean[candidates.length];
    float[] previous = fillHoles(candidates, g);
    for (int r = 1; r <= radius; r++) {
      float[] opened = dilate(erode(previous, g, r), g, r);
      double threshold = slope * r * g.cell;
      for (int k = 0; k < opened.length; k++)
        if (previous[k] - opened[k] > threshold) object[k] = true;
      previous = opened;
    }
    boolean[] ground = new boolean[candidates.length];
    for (int k = 0; k < ground.length; k++) ground[k] = Float.isFinite(candidates[k]) && !object[k];
    return ground;
  }

  /**
   * Terrain cannot rise faster than the maximum slope: a ground cell standing more than slope x
   * distance + tolerance above any other ground cell is an object (crown sides at the cloud edge,
   * canopy patches beside real ground). The lower envelope min_j(z_j + slope d_kj) is propagated
   * with a two-pass chamfer; repeated after removals.
   */
  static void slopeEnvelope(float[] z, boolean[] ground, Grid g, double slope, double tolerance) {
    float straight = (float) (slope * g.cell), diagonal = (float) (slope * g.cell * Math.sqrt(2));
    for (int round = 0; round < 5; round++) {
      float[] l = new float[z.length];
      for (int k = 0; k < l.length; k++) l[k] = ground[k] ? z[k] : Float.POSITIVE_INFINITY;
      for (int r = 0; r < g.h; r++)
        for (int c = 0; c < g.w; c++) {
          int k = r * g.w + c;
          if (c > 0) l[k] = Math.min(l[k], l[k - 1] + straight);
          if (r > 0) {
            l[k] = Math.min(l[k], l[k - g.w] + straight);
            if (c > 0) l[k] = Math.min(l[k], l[k - g.w - 1] + diagonal);
            if (c + 1 < g.w) l[k] = Math.min(l[k], l[k - g.w + 1] + diagonal);
          }
        }
      for (int r = g.h - 1; r >= 0; r--)
        for (int c = g.w - 1; c >= 0; c--) {
          int k = r * g.w + c;
          if (c + 1 < g.w) l[k] = Math.min(l[k], l[k + 1] + straight);
          if (r + 1 < g.h) {
            l[k] = Math.min(l[k], l[k + g.w] + straight);
            if (c + 1 < g.w) l[k] = Math.min(l[k], l[k + g.w + 1] + diagonal);
            if (c > 0) l[k] = Math.min(l[k], l[k + g.w - 1] + diagonal);
          }
        }
      int removed = 0;
      for (int r = 0; r < g.h; r++)
        for (int c = 0; c < g.w; c++) {
          int k = r * g.w + c;
          if (!ground[k]) continue;
          // Envelope from the neighbours, so the cell's own height does not excuse itself.
          float envelope = Float.POSITIVE_INFINITY;
          for (int dr = -1; dr <= 1; dr++)
            for (int dc = -1; dc <= 1; dc++) {
              int rr = r + dr, cc = c + dc;
              if ((dr | dc) == 0 || rr < 0 || cc < 0 || rr >= g.h || cc >= g.w) continue;
              envelope = Math.min(envelope, l[rr * g.w + cc] + (dr != 0 && dc != 0 ? diagonal : straight));
            }
          if (z[k] - envelope > tolerance) {
            ground[k] = false;
            removed++;
          }
        }
      if (removed == 0) break;
    }
  }

  /** Terrain: ground candidates kept, everything else interpolated (pyramid fill + relaxation). */
  static float[] interpolate(float[] candidates, boolean[] ground, Grid g) {
    float[] known = new float[candidates.length];
    for (int k = 0; k < known.length; k++) known[k] = ground[k] ? candidates[k] : Float.NaN;
    return fillHoles(known, g);
  }

  /** Fills NaN cells from coarser averages, then relaxes the filled cells (known cells fixed). */
  static float[] fillHoles(float[] z, Grid g) {
    float[] out = pyramid(z, g.w, g.h);
    boolean any = false;
    for (float v : z) any |= Float.isFinite(v);
    if (!any) return out;
    // Red-black relaxation: cells of one colour only read the other colour, so rows can run in
    // parallel and the result does not depend on the thread count.
    for (int iter = 0; iter < 60; iter++) {
      int colour = iter % 2;
      parallel(g.h, threads, r -> {
        for (int c = (r + colour) % 2; c < g.w; c += 2) {
          int k = r * g.w + c;
          if (Float.isFinite(z[k])) continue;
          double sum = 0;
          int n = 0;
          if (c > 0) {
            sum += out[k - 1];
            n++;
          }
          if (c + 1 < g.w) {
            sum += out[k + 1];
            n++;
          }
          if (r > 0) {
            sum += out[k - g.w];
            n++;
          }
          if (r + 1 < g.h) {
            sum += out[k + g.w];
            n++;
          }
          out[k] = (float) (sum / n);
        }
      });
    }
    return out;
  }

  private static float[] pyramid(float[] z, int w, int h) {
    float[] out = z.clone();
    boolean missing = false, any = false;
    for (float v : z) {
      if (Float.isFinite(v)) any = true;
      else missing = true;
    }
    if (!missing) return out;
    if (!any) {
      Arrays.fill(out, 0);
      return out;
    }
    int cw = (w + 1) / 2, ch = (h + 1) / 2;
    float[] coarse = new float[cw * ch];
    for (int r = 0; r < ch; r++)
      for (int c = 0; c < cw; c++) {
        double sum = 0;
        int n = 0;
        for (int dr = 0; dr < 2; dr++)
          for (int dc = 0; dc < 2; dc++) {
            int rr = 2 * r + dr, cc = 2 * c + dc;
            if (rr < h && cc < w && Float.isFinite(z[rr * w + cc])) {
              sum += z[rr * w + cc];
              n++;
            }
          }
        coarse[r * cw + c] = n == 0 ? Float.NaN : (float) (sum / n);
      }
    float[] filled = w == 1 && h == 1 ? coarse : pyramid(coarse, cw, ch);
    // Bilinear upsampling (push-pull): copying each coarse cell into a 2x2 block would leave
    // flat squares up to 2^levels cells wide in large interpolated areas.
    for (int r = 0; r < h; r++)
      for (int c = 0; c < w; c++) {
        if (Float.isFinite(out[r * w + c])) continue;
        double fy = Math.max(0, Math.min(ch - 1, (r + .5) / 2 - .5)), fx = Math.max(0, Math.min(cw - 1, (c + .5) / 2 - .5));
        int y0 = (int) fy, x0 = (int) fx, y1 = Math.min(ch - 1, y0 + 1), x1 = Math.min(cw - 1, x0 + 1);
        double ty = fy - y0, tx = fx - x0;
        out[r * w + c] =
            (float)
                ((1 - ty) * ((1 - tx) * filled[y0 * cw + x0] + tx * filled[y0 * cw + x1])
                    + ty * ((1 - tx) * filled[y1 * cw + x0] + tx * filled[y1 * cw + x1]));
      }
    return out;
  }

  static float[] erode(float[] z, Grid g, int r) {
    return separable(z, g, r, true);
  }

  static float[] dilate(float[] z, Grid g, int r) {
    return separable(z, g, r, false);
  }

  /** Square min/max filter of radius r: two 1-D monotone-deque passes, O(cells). */
  /** Worker threads for the ground step (set from --cores by classify). */
  static volatile int threads = Math.max(1, Runtime.getRuntime().availableProcessors() / 2);

  static void parallel(int count, int workers, java.util.function.IntConsumer action) {
    Parallel.forEach(count, workers, action);
  }

  private static float[] separable(float[] z, Grid g, int r, boolean min) {
    float[] rows = new float[z.length], out = new float[z.length];
    int side = Math.max(g.w, g.h);
    ThreadLocal<float[][]> buffers = ThreadLocal.withInitial(() -> new float[][] {new float[side], new float[side]});
    ThreadLocal<int[]> deques = ThreadLocal.withInitial(() -> new int[side]);
    // Rows, then columns; each line is independent, so the result does not depend on threads.
    parallel(g.h, threads, y -> {
      float[] line = buffers.get()[0], result = buffers.get()[1];
      System.arraycopy(z, y * g.w, line, 0, g.w);
      slide(line, g.w, r, min, deques.get(), result);
      System.arraycopy(result, 0, rows, y * g.w, g.w);
    });
    parallel(g.w, threads, x -> {
      float[] line = buffers.get()[0], result = buffers.get()[1];
      for (int y = 0; y < g.h; y++) line[y] = rows[y * g.w + x];
      slide(line, g.h, r, min, deques.get(), result);
      for (int y = 0; y < g.h; y++) out[y * g.w + x] = result[y];
    });
    return out;
  }

  private static void slide(float[] v, int n, int r, boolean min, int[] deque, float[] out) {
    int head = 0, tail = 0;
    for (int i = 0; i < n + r; i++) {
      if (i < n) {
        while (tail > head && (min ? v[deque[tail - 1]] >= v[i] : v[deque[tail - 1]] <= v[i])) tail--;
        deque[tail++] = i;
      }
      int centre = i - r;
      if (centre < 0) continue;
      while (deque[head] < centre - r) head++;
      out[centre] = v[deque[head]];
    }
  }

  /**
   * Free-space terrain bound: for each ray from a supporting camera to its point, the heights the
   * ray passes through (above the point by a margin) are recorded per cell. Returns {lowest,
   * second lowest}; the second needs two rays and survives a single false point.
   */
  static float[][] carve(Cloud c, double[][] cameras, Grid g, boolean[] use, double top) {
    int cells = g.w * g.h;
    double margin = Math.max(.5, 3 * g.cell);
    int stride = Math.max(1, c.n / 2_000_000);
    int samples = (c.n + stride - 1) / stride;
    // Per-thread {lowest, second} grids, merged afterwards: the two smallest heights per cell do not
    // depend on the order of the rays, so the result is the same as a serial pass. Threads are
    // limited so the per-thread grids stay within about 512 MB.
    int workers = (int) Math.max(1, Math.min(threads, (512L << 20) / Math.max(1, 8L * cells)));
    float[][][] parts = new float[workers][][];
    int chunk = (samples + workers - 1) / workers;
    parallel(workers, workers, t -> {
      float[] lowest = new float[cells], second = new float[cells];
      Arrays.fill(lowest, Float.POSITIVE_INFINITY);
      Arrays.fill(second, Float.POSITIVE_INFINITY);
      for (int s = t * chunk, end = Math.min(samples, s + chunk); s < end; s++) {
        int i = s * stride;
        if (!use[i]) continue;
        for (int cam : new int[] {c.camA[i], c.camB[i]}) {
          if (cam < 0 || cam >= cameras.length || cameras[cam] == null) continue;
          double[] o = cameras[cam];
          // One entry per cell and ray (its lowest sample there): the robust bound needs two rays.
          int[] run = {-1};
          float[] runLow = {Float.POSITIVE_INFINITY};
          RayVisitor commit =
              (k, z) -> {
                int idx = g.index(k[0], k[1]);
                if (idx != run[0] && run[0] >= 0) {
                  float h = runLow[0];
                  int at = run[0];
                  if (h < lowest[at]) {
                    second[at] = lowest[at];
                    lowest[at] = h;
                  } else if (h < second[at]) second[at] = h;
                  runLow[0] = Float.POSITIVE_INFINITY;
                }
                run[0] = idx;
                if (idx >= 0) runLow[0] = Math.min(runLow[0], (float) z);
              };
          walk(o, c.x[i], c.y[i], c.z[i], top, margin, g.cell, commit);
          commit.visit(new double[] {Double.NaN, Double.NaN}, 0);
        }
      }
      parts[t] = new float[][] {lowest, second};
    });
    float[] lowest = parts[0][0], second = parts[0][1];
    for (int t = 1; t < workers; t++) {
      float[] l = parts[t][0], s2 = parts[t][1];
      float[] low = lowest, sec = second;
      parallel(cells, threads, k -> {
        // Insert the other part's two values; ties count twice, as in the serial pass.
        for (int j = 0; j < 2; j++) {
          float h = j == 0 ? l[k] : s2[k];
          if (h < low[k]) {
            sec[k] = low[k];
            low[k] = h;
          } else if (h < sec[k]) sec[k] = h;
        }
      });
      parts[t] = null;
    }
    for (int k = 0; k < second.length; k++) {
      if (second[k] == Float.POSITIVE_INFINITY) second[k] = Float.NaN;
      if (lowest[k] == Float.POSITIVE_INFINITY) lowest[k] = Float.NaN;
    }
    return new float[][] {lowest, second};
  }

  interface RayVisitor {
    void visit(double[] xy, double z);
  }

  /** Samples a camera->point ray every half cell, from height top down to point + margin. */
  static void walk(
      double[] camera, double px, double py, double pz, double top, double margin, double cell, RayVisitor visitor) {
    double cz = camera[2];
    if (!(cz > pz + margin)) return;
    double t0 = cz > top ? (cz - top) / (cz - pz) : 0, t1 = (cz - (pz + margin)) / (cz - pz);
    if (t1 <= t0) return;
    double dx = px - camera[0], dy = py - camera[1];
    double length = (t1 - t0) * Math.hypot(dx, dy);
    int steps = (int) Math.min(800, Math.ceil(length / (cell / 2)) + 1);
    double[] xy = new double[2];
    for (int s = 0; s <= steps; s++) {
      double t = t0 + (t1 - t0) * s / Math.max(1, steps);
      xy[0] = camera[0] + t * dx;
      xy[1] = camera[1] + t * dy;
      visitor.visit(xy, cz + t * (pz - cz));
    }
  }

  /** Counts rays the terrain blocks (terrain above the ray by more than the tolerance). */
  static long[] violations(
      Cloud c, double[][] cameras, Grid g, boolean[] use, double top, float[] dtm, double tolerance, float[] cells) {
    long[] counts = new long[2];
    double margin = Math.max(.5, 3 * g.cell);
    int stride = Math.max(1, c.n / 500_000);
    for (int i = 0; i < c.n; i += stride) {
      if (!use[i]) continue;
      int cam = c.camA[i];
      if (cam < 0 || cam >= cameras.length || cameras[cam] == null) continue;
      boolean[] hit = {false};
      int[] where = {-1};
      walk(cameras[cam], c.x[i], c.y[i], c.z[i], top, margin, g.cell, (xy, z) -> {
        int idx = g.index(xy[0], xy[1]);
        if (!hit[0] && idx >= 0 && dtm[idx] > z + tolerance) {
          hit[0] = true;
          where[0] = idx;
        }
      });
      counts[1]++;
      if (hit[0]) {
        counts[0]++;
        cells[where[0]]++;
      }
    }
    return counts;
  }

  /** Robust spread (1.4826 MAD) of points within 1 m of the terrain over ground cells. */
  static double spread(Cloud c, Grid g, float[] dtm, boolean[] ground) {
    List<Double> residuals = new ArrayList<>();
    int stride = Math.max(1, c.n / 1_000_000);
    for (int i = 0; i < c.n; i += stride) {
      int k = g.index(c.x[i], c.y[i]);
      if (k < 0 || !ground[k]) continue;
      double r = c.z[i] - sample(dtm, g, c.x[i], c.y[i]);
      if (Math.abs(r) <= 1) residuals.add(r);
    }
    if (residuals.size() < 20) return .1;
    Collections.sort(residuals);
    double median = quantile(residuals, .5);
    List<Double> deviations = new ArrayList<>();
    for (double r : residuals) deviations.add(Math.abs(r - median));
    Collections.sort(deviations);
    return Math.max(.02, 1.4826 * quantile(deviations, .5));
  }

  /** Bilinear terrain height at a point (cell centres), NaN outside. */
  static double sample(float[] z, Grid g, double x, double y) {
    double fx = (x - g.x0) / g.cell - .5, fy = (g.y1 - y) / g.cell - .5;
    int c0 = (int) Math.floor(fx), r0 = (int) Math.floor(fy);
    double ax = fx - c0, ay = fy - r0, sum = 0, weight = 0;
    for (int dr = 0; dr < 2; dr++)
      for (int dc = 0; dc < 2; dc++) {
        int r = Math.max(0, Math.min(g.h - 1, r0 + dr)), c = Math.max(0, Math.min(g.w - 1, c0 + dc));
        double w = (dc == 0 ? 1 - ax : ax) * (dr == 0 ? 1 - ay : ay);
        float v = z[r * g.w + c];
        if (Float.isFinite(v) && w > 0) {
          sum += w * v;
          weight += w;
        }
      }
    return weight > 0 ? sum / weight : Double.NaN;
  }

  /** Terrain slope magnitude (rise per run) per cell. */
  static float[] gradient(float[] z, Grid g) {
    float[] out = new float[z.length];
    for (int r = 0; r < g.h; r++)
      for (int c = 0; c < g.w; c++) {
        int k = r * g.w + c;
        double dx = (z[r * g.w + Math.min(g.w - 1, c + 1)] - z[r * g.w + Math.max(0, c - 1)]) / (2 * g.cell);
        double dy = (z[Math.min(g.h - 1, r + 1) * g.w + c] - z[Math.max(0, r - 1) * g.w + c]) / (2 * g.cell);
        out[k] = Float.isFinite((float) (dx + dy)) ? (float) Math.hypot(dx, dy) : 0;
      }
    return out;
  }

  /** Chamfer distance (cells) to the nearest observed ground cell. */
  static float[] distance(boolean[] ground, Grid g) {
    float[] d = new float[ground.length];
    for (int k = 0; k < d.length; k++) d[k] = ground[k] ? 0 : Float.POSITIVE_INFINITY;
    float diagonal = (float) Math.sqrt(2);
    for (int r = 0; r < g.h; r++)
      for (int c = 0; c < g.w; c++) {
        int k = r * g.w + c;
        if (c > 0) d[k] = Math.min(d[k], d[k - 1] + 1);
        if (r > 0) {
          d[k] = Math.min(d[k], d[k - g.w] + 1);
          if (c > 0) d[k] = Math.min(d[k], d[k - g.w - 1] + diagonal);
          if (c + 1 < g.w) d[k] = Math.min(d[k], d[k - g.w + 1] + diagonal);
        }
      }
    for (int r = g.h - 1; r >= 0; r--)
      for (int c = g.w - 1; c >= 0; c--) {
        int k = r * g.w + c;
        if (c + 1 < g.w) d[k] = Math.min(d[k], d[k + 1] + 1);
        if (r + 1 < g.h) {
          d[k] = Math.min(d[k], d[k + g.w] + 1);
          if (c + 1 < g.w) d[k] = Math.min(d[k], d[k + g.w + 1] + diagonal);
          if (c > 0) d[k] = Math.min(d[k], d[k + g.w - 1] + diagonal);
        }
      }
    for (int k = 0; k < d.length; k++) if (d[k] == Float.POSITIVE_INFINITY) d[k] = Float.NaN;
    return d;
  }

  private static BitSet dilate(BitSet set, Grid g, int r) {
    BitSet out = new BitSet(g.w * g.h);
    for (int k = set.nextSetBit(0); k >= 0; k = set.nextSetBit(k + 1)) {
      int row = k / g.w, col = k % g.w;
      for (int dr = -r; dr <= r; dr++)
        for (int dc = -r; dc <= r; dc++) {
          int rr = row + dr, cc = col + dc;
          if (rr >= 0 && cc >= 0 && rr < g.h && cc < g.w) out.set(rr * g.w + cc);
        }
    }
    return out;
  }

  static Grid grid(Cloud c, double cell) {
    double minX = Double.POSITIVE_INFINITY, minY = Double.POSITIVE_INFINITY;
    double maxX = Double.NEGATIVE_INFINITY, maxY = Double.NEGATIVE_INFINITY;
    for (int i = 0; i < c.n; i++) {
      minX = Math.min(minX, c.x[i]);
      maxX = Math.max(maxX, c.x[i]);
      minY = Math.min(minY, c.y[i]);
      maxY = Math.max(maxY, c.y[i]);
    }
    int w = Math.max(1, (int) Math.ceil((maxX - minX) / cell) + 1);
    int h = Math.max(1, (int) Math.ceil((maxY - minY) / cell) + 1);
    return new Grid(w, h, cell, minX, minY + h * cell);
  }

  static double quantile(List<Double> sorted, double q) {
    if (sorted.isEmpty()) return Double.NaN;
    return sorted.get((int) Math.min(sorted.size() - 1, Math.floor(q * (sorted.size() - 1) + .5)));
  }

  /** DJI takeoff ground: EXIF GPS altitude minus XMP relative altitude (median over images). */
  static double takeoffGround(List<ImageFrame> frames) throws IOException {
    List<Double> values = new ArrayList<>();
    for (ImageFrame f : frames) {
      if (f.gps == null || !Double.isFinite(f.gps.altitude)) continue;
      double relative = Exif.read(f.path).relativeAltitude;
      if (Double.isFinite(relative)) values.add(f.gps.altitude - relative);
    }
    Collections.sort(values);
    return quantile(values, .5);
  }

  /** LAS 1.2 point format 2 (+ extra bytes) and, if aligned, the confidence sidecar. */
  static Cloud read(Path las, Path sidecar) throws IOException {
    Cloud c = new Cloud();
    double[] scale = new double[3], offset = new double[3];
    try (FileChannel ch = FileChannel.open(las, StandardOpenOption.READ)) {
      ByteBuffer h = ByteBuffer.allocate(227).order(ByteOrder.LITTLE_ENDIAN);
      ch.read(h, 0);
      if (h.get(0) != 'L' || h.get(1) != 'A' || h.get(2) != 'S' || h.get(3) != 'F')
        throw new IOException("Not a LAS file: " + las);
      c.dataOffset = h.getInt(96) & 0xffffffffL;
      int format = h.get(104) & 255;
      c.recordLength = h.getShort(105) & 0xffff;
      long count = h.getInt(107) & 0xffffffffL;
      if (format != 2 && format != 3 && format != 0 && format != 1)
        throw new IOException("Unsupported LAS point format " + format);
      if (count > Integer.MAX_VALUE - 8) throw new IOException("Cloud too large for classification");
      for (int k = 0; k < 3; k++) {
        scale[k] = h.getDouble(131 + 8 * k);
        offset[k] = h.getDouble(155 + 8 * k);
      }
      c.zScale = scale[2];
      c.zOffset = offset[2];
      c.n = (int) count;
      c.x = new float[c.n];
      c.y = new float[c.n];
      c.z = new float[c.n];
      // Origin at the first point keeps float precision at millimetres over kilometres.
      ByteBuffer first = ByteBuffer.allocate(12).order(ByteOrder.LITTLE_ENDIAN);
      ch.read(first, c.dataOffset);
      c.ox = first.getInt(0) * scale[0] + offset[0];
      c.oy = first.getInt(4) * scale[1] + offset[1];
      ByteBuffer block = ByteBuffer.allocate(c.recordLength * 65536).order(ByteOrder.LITTLE_ENDIAN);
      for (long start = 0; start < count; start += 65536) {
        int records = (int) Math.min(65536, count - start);
        block.clear().limit(records * c.recordLength);
        long position = c.dataOffset + start * c.recordLength;
        while (block.hasRemaining()) if (ch.read(block, position + block.position()) < 0) throw new EOFException();
        for (int r = 0; r < records; r++) {
          int p = r * c.recordLength, i = (int) (start + r);
          c.x[i] = (float) (block.getInt(p) * scale[0] + offset[0] - c.ox);
          c.y[i] = (float) (block.getInt(p + 4) * scale[1] + offset[1] - c.oy);
          c.z[i] = (float) (block.getInt(p + 8) * scale[2] + offset[2]);
        }
      }
    }
    if (Files.isRegularFile(sidecar)) {
      short[] views = new short[c.n];
      float[] nz = new float[c.n];
      int[] a = new int[c.n], b = new int[c.n], source = new int[c.n];
      int i = 0;
      boolean aligned = true;
      try (PointRecords.Reader reader = new PointRecords.Reader(sidecar)) {
        for (PointRecords.Row row; (row = reader.next()) != null; i++) {
          if (i >= c.n || row.id() != i) {
            aligned = false;
            break;
          }
          var s = row.sample();
          views[i] = (short) Math.min(Short.MAX_VALUE, s.views);
          nz[i] = s.normal == null ? Float.NaN : (float) s.normal[2];
          source[i] = s.source;
          a[i] = b[i] = -1;
          if (s.supportingCameras != null) {
            a[i] = s.supportingCameras.nextSetBit(0);
            b[i] = a[i] < 0 ? -1 : s.supportingCameras.nextSetBit(a[i] + 1);
          }
        }
      } catch (IOException e) {
        aligned = false;
      }
      if (aligned && i == c.n) {
        c.sidecar = true;
        c.views = views;
        c.nz = nz;
        c.camA = a;
        c.camB = b;
        c.source = source;
      }
    }
    return c;
  }

  /**
   * Adds delta to every point's height in place: LAS (quantized z, header z range) and the
   * sibling dense.ply (double z at byte 16 of each vertex).
   */
  static void writeHeights(Path las, Cloud c, float[] delta) throws IOException {
    double low = Double.POSITIVE_INFINITY, high = Double.NEGATIVE_INFINITY;
    try (FileChannel ch = FileChannel.open(las, StandardOpenOption.READ, StandardOpenOption.WRITE)) {
      ByteBuffer block = ByteBuffer.allocate(c.recordLength * 65536).order(ByteOrder.LITTLE_ENDIAN);
      for (long start = 0; start < c.n; start += 65536) {
        int records = (int) Math.min(65536, c.n - start);
        long position = c.dataOffset + start * c.recordLength;
        block.clear().limit(records * c.recordLength);
        while (block.hasRemaining()) if (ch.read(block, position + block.position()) < 0) throw new EOFException();
        for (int r = 0; r < records; r++) {
          int p = r * c.recordLength;
          double z = block.getInt(p + 8) * c.zScale + c.zOffset + delta[(int) (start + r)];
          block.putInt(p + 8, (int) Math.round((z - c.zOffset) / c.zScale));
          low = Math.min(low, z);
          high = Math.max(high, z);
        }
        block.flip();
        while (block.hasRemaining()) ch.write(block, position + block.position());
      }
      ByteBuffer range = ByteBuffer.allocate(16).order(ByteOrder.LITTLE_ENDIAN).putDouble(high).putDouble(low);
      range.flip();
      ch.write(range, 211);
    }
    Path ply = las.resolveSibling("dense.ply");
    if (!Files.isRegularFile(ply)) return;
    try (FileChannel ch = FileChannel.open(ply, StandardOpenOption.READ, StandardOpenOption.WRITE)) {
      long header = plyHeaderLength(ch);
      long record = (ch.size() - header) / c.n;
      if (record * c.n != ch.size() - header || record < 24) throw new IOException("Unexpected PLY layout: " + ply);
      ByteBuffer block = ByteBuffer.allocate((int) record * 16384).order(ByteOrder.LITTLE_ENDIAN);
      for (long start = 0; start < c.n; start += 16384) {
        int records = (int) Math.min(16384, c.n - start);
        long position = header + start * record;
        block.clear().limit((int) (records * record));
        while (block.hasRemaining()) if (ch.read(block, position + block.position()) < 0) throw new EOFException();
        for (int r = 0; r < records; r++) {
          int p = (int) (r * record) + 16;
          block.putDouble(p, block.getDouble(p) + delta[(int) (start + r)]);
        }
        block.flip();
        while (block.hasRemaining()) ch.write(block, position + block.position());
      }
    }
  }

  /** Bytes up to and including the "end_header\n" line. */
  static long plyHeaderLength(FileChannel ch) throws IOException {
    ByteBuffer b = ByteBuffer.allocate((int) Math.min(ch.size(), 1 << 16));
    ch.read(b, 0);
    String text = new String(b.array(), 0, b.position(), java.nio.charset.StandardCharsets.ISO_8859_1);
    int end = text.indexOf("end_header\n");
    if (end < 0) throw new IOException("PLY header not found");
    return end + "end_header\n".length();
  }

  /** Sets the LAS classification byte (offset 15 of each point record) in place. */
  static void writeClasses(Path las, Cloud c, byte[] classes) throws IOException {
    try (FileChannel ch = FileChannel.open(las, StandardOpenOption.READ, StandardOpenOption.WRITE)) {
      ByteBuffer block = ByteBuffer.allocate(c.recordLength * 65536).order(ByteOrder.LITTLE_ENDIAN);
      for (long start = 0; start < c.n; start += 65536) {
        int records = (int) Math.min(65536, c.n - start);
        long position = c.dataOffset + start * c.recordLength;
        block.clear().limit(records * c.recordLength);
        while (block.hasRemaining()) if (ch.read(block, position + block.position()) < 0) throw new EOFException();
        for (int r = 0; r < records; r++) block.put(r * c.recordLength + 15, classes[(int) (start + r)]);
        block.flip();
        while (block.hasRemaining()) ch.write(block, position + block.position());
      }
    }
  }

  private static void preview(Path file, float[] z, Grid g, boolean categorical) throws IOException {
    int pw = Math.min(1024, g.w), ph = Math.max(1, (int) Math.round(g.h * (pw / (double) g.w)));
    if (ph > 1024) {
      pw = Math.max(1, (int) Math.round(pw * (1024. / ph)));
      ph = 1024;
    }
    double low = Double.POSITIVE_INFINITY, high = Double.NEGATIVE_INFINITY;
    for (float v : z)
      if (Float.isFinite(v)) {
        low = Math.min(low, v);
        high = Math.max(high, v);
      }
    int[] palette = {0xe5e7eb, 0x16a34a, 0xfacc15, 0xdc2626};
    BufferedImage image = new BufferedImage(pw, ph, BufferedImage.TYPE_INT_RGB);
    for (int y = 0; y < ph; y++)
      for (int x = 0; x < pw; x++) {
        float v = z[Math.min(g.h - 1, y * g.h / ph) * g.w + Math.min(g.w - 1, x * g.w / pw)];
        int rgb = 0xffffff;
        if (Float.isFinite(v))
          rgb =
              categorical
                  ? palette[Math.max(0, Math.min(3, (int) v))]
                  : java.awt.Color.HSBtoRGB(
                      (float) ((1 - (v - low) / Math.max(1e-9, high - low)) * .67), 1, 1);
        image.setRGB(x, y, rgb);
      }
    ImageIO.write(image, "png", file.toFile());
    image.flush();
  }
}
