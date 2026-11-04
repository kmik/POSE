package sgm;

import java.nio.file.*;
import java.util.*;
import org.opencv.core.Core;

/**
 * Independent analytical camera observations and noisy surface controls, not implementation
 * mirrors.
 */
final class SurfaceFusionRegression {
  private static int checks;

  private static void check(boolean yes, String message) {
    checks++;
    if (!yes) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Core.setNumThreads(1);
    Path root = Files.createTempDirectory(Path.of("target"), "surface fusion ");
    System.out.println("PASS surface checks=" + run(root) + "; fixtures=" + root);
    GpuSgm.closeSession();
  }

  static int run(Path root) throws Exception {
    checks = 0;
    reprojection();
    plane(root, false, false);
    plane(root, true, false);
    plane(root, false, true);
    layers(root);
    opposingAndCleanup(root);
    medianTie(root);
    endToEnd(root);
    options(root);
    return checks;
  }

  private static Camera[] cameras() {
    Camera[] c = new Camera[4];
    for (int i = 0; i < c.length; i++) {
      c[i] = new Camera(200, 210, 79.5, 59.5);
      c[i].translation = new double[] {-(i - 1.5) * .45, -(i % 2) * .12, 0};
      c[i].registered = true;
    }
    return c;
  }

  private static void reprojection() {
    Camera[] c = cameras();
    double[] truth = {.2, -.1, 10}, initial = {.21, -.105, 10.5};
    List<ReprojectionDepth.Observation> obs = new ArrayList<>();
    for (int i = 1; i < c.length; i++) {
      double[] uv = c[i].project(truth);
      obs.add(new ReprojectionDepth.Observation(c[i], uv[0], uv[1]));
    }
    // Anchor the point to the exact truth ray; only depth is unknown.
    double[] origin = c[0].center();
    initial = Math3.add(origin, Math3.times(Math3.sub(truth, origin), 1.05));
    ReprojectionDepth.Fit fit = ReprojectionDepth.fit(c[0], initial, 10.4, obs, .75);
    check(
        Math.abs(fit.depth() - 10) < 1e-8,
        "Exact multi-view measurements recover analytical depth");
    check(
        fit.after() < fit.before() * 1e-8,
        "Reprojection objective decreases to its independent exact solution");
    check(
        Double.isFinite(fit.relativeSigma()) && fit.relativeSigma() > 0,
        "Conditional inverse-depth uncertainty is finite");
    double weak = fit.relativeSigma();
    Camera tiny = new Camera(200, 210, 79.5, 59.5);
    tiny.translation = new double[] {c[0].translation[0] - .0001, 0, 0};
    double[] uv = tiny.project(truth);
    ReprojectionDepth.Fit poor =
        ReprojectionDepth.fit(
            c[0],
            initial,
            10.4,
            List.of(new ReprojectionDepth.Observation(tiny, uv[0], uv[1])),
            .75);
    check(
        poor.relativeSigma() > weak * 100,
        "Weak baselines remain uncertain rather than being certified by a small residual");
    List<ReprojectionDepth.Observation> linked = new ArrayList<>();
    for (int i = 1; i < c.length; i++)
      linked.add(
          ReprojectionDepth.link(
              c[0], c[i], truth, new double[] {truth[0] + .025, truth[1] - .02, truth[2]}));
    ReprojectionDepth.Fit transported = ReprojectionDepth.fit(c[0], initial, 10.4, linked, .75);
    check(
        Math.abs(transported.depth() - 10) < 1e-8,
        "Neighboring raster samples are linked to the base ray before pixel fitting, preventing"
            + " tangential lookup offsets from biasing depth");
  }

  private static void plane(Path root, boolean wall, boolean fence) throws Exception {
    Camera[] c = cameras();
    int[] widths = {160, 160, 160, 160}, heights = {120, 120, 120, 120};
    Options o = new Options();
    o.compute = "cpu";
    o.surfaceMemoryMb = 8;
    o.memoryMb = 64;
    o.minViews = 3;
    o.output = Files.createDirectories(root.resolve("surface-" + wall + "-" + fence));
    double[] translation = {490000, 6710000, -100};
    double[] transform = wall ? new double[] {1, 0, 0, 0, 0, -1, 0, 1, 0} : Math3.identity();
    if (wall)
      for (Camera camera : c) {
        camera.translation =
            Math3.sub(
                camera.translation,
                Math3.mv(Math3.mul(camera.rotation, Math3.transpose(transform)), translation));
        camera.rotation = Math3.mul(camera.rotation, Math3.transpose(transform));
      }
    List<double[]> truth = new ArrayList<>();
    List<double[]> points = new ArrayList<>();
    List<Integer> sources = new ArrayList<>();
    try (DepthFusion fusion = new DepthFusion(c, widths, heights, o)) {
      Random random = new Random(81231);
      int[][] pair = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
      for (int id = 0; id < pair.length; id++) {
        fusion.addPair(pair[id][0], pair[id][1]);
        // Common world-grid coordinates avoid inventing a correspondence model in the tests.
        for (int y = -18; y <= 18; y++)
          for (int x = -32; x <= 32; x++) {
            double wx = x * .05, wy = y * .05, z = 10 + .035 * wx + .02 * wy;
            if (fence && x == 0) z -= .12;
            double[] t = {wx, wy, z}, p = {wx, wy, z + random.nextGaussian() * .012};
            if (wall) {
              t = Math3.add(Math3.mv(transform, t), translation);
              p = Math3.add(Math3.mv(transform, p), translation);
            }
            fusion.add(id, p);
            truth.add(t);
            points.add(p);
            sources.add(id);
          }
      }
      double raw = 0;
      int count = 0, thinBefore = 0;
      Set<String> coverageBefore = new HashSet<>();
      List<SurfaceConsolidator.Sample> accepted = new ArrayList<>();
      for (int i = 0; i < points.size(); i++) {
        DepthFusion.Result r = fusion.fuse(sources.get(i), points.get(i));
        if (!r.accepted()) continue;
        double[] local =
            wall ? Math3.mv(Math3.transpose(transform), Math3.sub(r.point, translation)) : r.point;
        double target = 10 + .035 * local[0] + .02 * local[1];
        boolean thin = fence && Math.abs(local[0]) < .02;
        if (thin) {
          target -= .12;
          thinBefore++;
        } else {
          raw += Math.pow(local[2] - target, 2);
          count++;
        }
        coverageBefore.add(Math.round(local[0] / .1) + ":" + Math.round(local[1] / .1));
        accepted.add(
            new SurfaceConsolidator.Sample(
                i, sources.get(i), i, 0x789abc, r, fusion.surfaceSample(sources.get(i), r)));
      }
      List<double[]> output = new ArrayList<>();
      long[] combined = {0}, normalCount = {0};
      if (Boolean.getBoolean("sgm.test.surface.gpu")) {
        o.fusionCompute = "gpu";
        o.gpuDevice = Integer.getInteger("sgm.test.gpu", 0);
        fusion.prepare(accepted.size());
      }
      try (SurfaceConsolidator surface = new SurfaceConsolidator(o, fusion)) {
        for (var p : accepted) {
          surface.add(p);
          if (p.normal != null) normalCount[0]++;
        }
        surface.finish(
            new SurfaceConsolidator.Sink() {
              public void point(SurfaceConsolidator.Sample p) {
                output.add(p.point.clone());
                combined[0] += p.merged;
              }
            });
      }
      double revised = 0;
      int n = 0, thinAfter = 0;
      Set<String> coverageAfter = new HashSet<>();
      for (double[] world : output) {
        double[] local =
            wall ? Math3.mv(Math3.transpose(transform), Math3.sub(world, translation)) : world;
        double target = 10 + .035 * local[0] + .02 * local[1];
        if (fence && Math.abs(local[0]) < .02) {
          if (Math.abs(local[2] - (target - .12)) < .04) thinAfter++;
        } else {
          revised += Math.pow(local[2] - target, 2);
          n++;
        }
        coverageAfter.add(Math.round(local[0] / .1) + ":" + Math.round(local[1] / .1));
      }
      double before = Math.sqrt(raw / count), after = Math.sqrt(revised / n);
      System.out.printf(
          Locale.ROOT,
          "Surface control wall=%s fence=%s: %,d -> %,d points; RMS %.6f -> %.6f; normals=%d;"
              + " thin=%d -> %d; coverage=%d/%d%n",
          wall,
          fence,
          accepted.size(),
          output.size(),
          before,
          after,
          normalCount[0],
          thinBefore,
          thinAfter,
          coverageAfter.size(),
          coverageBefore.size());
      Json.write(
          o.output.resolve("comparison.json"),
          Json.object(
              "input",
              accepted.size(),
              "output",
              output.size(),
              "before_rms",
              before,
              "after_rms",
              after,
              "normal_count",
              normalCount[0],
              "thin_before",
              thinBefore,
              "thin_after",
              thinAfter,
              "coverage_before",
              coverageBefore.size(),
              "coverage_after",
              coverageAfter.size()));
      check(
          accepted.size() > 6000,
          "Noisy analytical surface has meaningful independently validated coverage");
      check(after < before * .9, "Normal-aware consolidation reduces analytical plane noise");
      check(output.size() < accepted.size() * .9, "Redundant stereo samples are consolidated");
      check(
          combined[0] == accepted.size(),
          "Every accepted sample has exactly one representative or passthrough");
      check(
          coverageAfter.size() >= coverageBefore.size() * .98,
          "Surface extent and coverage survive consolidation");
      if (fence)
        check(
            thinAfter >= thinBefore * .85,
            "One-pixel foreground strip is preserved instead of being projected onto the ground");
      try (var files = Files.list(o.output)) {
        check(
            files.noneMatch(p -> p.getFileName().toString().startsWith("surface-work-")),
            "All temporary consolidation partitions are removed");
      }
      if (!fence) gpu(root, c, widths, heights, points, sources, pair);
    }
  }

  private static void gpu(
      Path root,
      Camera[] cameras,
      int[] widths,
      int[] heights,
      List<double[]> points,
      List<Integer> source,
      int[][] pairs)
      throws Exception {
    if (GpuSgm.devices().isEmpty()) return;
    Options cpu = new Options();
    cpu.compute = "cpu";
    cpu.output = root;
    Options device = cpu.copy();
    device.fusionCompute = "gpu";
    device.gpuDevice = Integer.getInteger("sgm.test.gpu", 0);
    try (DepthFusion a = new DepthFusion(cameras, widths, heights, cpu);
        DepthFusion b = new DepthFusion(cameras, widths, heights, device)) {
      for (int[] pair : pairs) {
        a.addPair(pair[0], pair[1]);
        b.addPair(pair[0], pair[1]);
      }
      for (int i = 0; i < points.size(); i++) {
        a.add(source.get(i), points.get(i));
        b.add(source.get(i), points.get(i));
      }
      b.prepare(points.size());
      int n = 4096;
      int[] ids = new int[n];
      double[][] p = new double[n][];
      for (int i = 0; i < n; i++) {
        int k = i * 3;
        ids[i] = source.get(k);
        p[i] = points.get(k);
      }
      DepthFusion.Result[] fast = b.fuseBatch(ids, p, n);
      for (int i = 0; i < n; i++)
        if (!DepthFusion.equivalent(a.fuse(ids[i], p[i]), fast[i]))
          throw new AssertionError("Reprojection CPU/GPU parity candidate=" + i);
      check(
          ((Number) ((Map<?, ?>) b.backendReport()).get("gpu_candidates")).longValue() == n,
          "New inverse-depth fit executes on GPU and agrees with CPU");
    }
  }

  private static void layers(Path root) throws Exception {
    Options o = new Options();
    o.compute = "cpu";
    o.output = Files.createDirectories(root.resolve("parallel layers"));
    Camera[] c = cameras();
    int[] w = {160, 160, 160, 160}, h = {120, 120, 120, 120};
    try (DepthFusion fusion = new DepthFusion(c, w, h, o);
        SurfaceConsolidator s = new SurfaceConsolidator(o, fusion)) {
      fusion.addPair(0, 1);
      fusion.addPair(0, 2);
      int input = 0;
      for (int layer = 0; layer < 2; layer++)
        for (int pair = 0; pair < 2; pair++)
          for (int y = -5; y <= 5; y++)
            for (int x = -5; x <= 5; x++) {
              double[] p = {x * .05, y * .05, 10 + layer * .09};
              fusion.add(pair, p);
              s.add(
                  new SurfaceConsolidator.Sample(
                      input++,
                      pair,
                      0,
                      0,
                      new DepthFusion.Result(p, 3, 1, .01, 0, "accepted"),
                      new DepthFusion.SurfaceSample(new double[] {0, 0, -1}, .05, .01)));
            }
      int[] counts = {0, 0};
      s.finish(
          new SurfaceConsolidator.Sink() {
            public void point(SurfaceConsolidator.Sample p) {
              double z = p.point[2];
              if (Math.abs(z - 10) < 1e-8) counts[0]++;
              else if (Math.abs(z - 10.09) < 1e-8) counts[1]++;
              else throw new AssertionError("Parallel layers blended into an intermediate surface");
            }
          });
      check(
          counts[0] > 100 && counts[1] > 100,
          "Resolved parallel layers both survive without blending");
    }
  }

  private static void options(Path root) {
    Options o =
        Options.parse(
            new String[] {
              "-i",
              root.toString(),
              "--depth-refinement",
              "weighted",
              "--surface-consolidation",
              "none",
              "--surface-radius",
              "3",
              "--surface-spacing",
              ".5",
              "--surface-max-shift",
              "1.5",
              "--surface-normal-angle",
              "20",
              "--surface-memory-mb",
              "8"
            });
    check(
        o.copy().depthRefinementMethod.equals("weighted")
            && o.copy().surfaceConsolidation.equals("none")
            && o.copy().surfaceRadius == 3,
        "New settings parse and survive option copying");
    for (String[] args :
        new String[][] {
          {"--surface-radius", "0"},
          {"--surface-normal-angle", "90"},
          {"--surface-memory-mb", "1"},
          {"--surface-consolidation", "bogus"},
          {"--depth-refinement", "bogus"},
          {"--surface-spacing", "3"}
        }) {
      boolean rejected = false;
      try {
        Options.parse(new String[] {"-i", root.toString(), args[0], args[1]});
      } catch (IllegalArgumentException e) {
        rejected = true;
      }
      check(rejected, "Invalid surface/refinement control is rejected: " + args[0]);
    }
    check(
        Options.help().contains("--surface-consolidation")
            && Options.help().contains("--depth-refinement"),
        "New defaults and controls are documented in CLI help");
  }

  private static void opposingAndCleanup(Path root) throws Exception {
    Options o = new Options();
    o.compute = "cpu";
    o.surfaceMemoryMb = 8;
    o.output = Files.createDirectories(root.resolve("surface guard controls"));
    Camera[] c = cameras();
    int[] w = {160, 160, 160, 160}, h = {120, 120, 120, 120};
    try (DepthFusion fusion = new DepthFusion(c, w, h, o)) {
      fusion.addPair(0, 1);
      fusion.addPair(0, 2);
      try (SurfaceConsolidator s = new SurfaceConsolidator(o, fusion)) {
        int id = 0;
        for (int side = 0; side < 2; side++)
          for (int pair = 0; pair < 2; pair++)
            for (int y = -3; y <= 3; y++)
              for (int x = -3; x <= 3; x++) {
                double[] p = {x * .05, y * .05, 10 + side * .02};
                fusion.add(pair, p);
                s.add(
                    new SurfaceConsolidator.Sample(
                        id++,
                        pair,
                        0,
                        0,
                        new DepthFusion.Result(p, 3, 1, .01, 0, "accepted"),
                        new DepthFusion.SurfaceSample(
                            new double[] {0, 0, side == 0 ? -1 : 1}, .05, .01)));
              }
        int[] counts = {0, 0};
        s.finish(
            new SurfaceConsolidator.Sink() {
              public void point(SurfaceConsolidator.Sample p) {
                if (Math.abs(p.point[2] - 10) < 1e-9) counts[0]++;
                else if (Math.abs(p.point[2] - 10.02) < 1e-9) counts[1]++;
                else throw new AssertionError("Opposing thin-surface sides blended");
              }
            });
        check(
            counts[0] > 20 && counts[1] > 20,
            "Opposing normals protect both sides of a thin surface, even below the layer-gap"
                + " threshold");
      }
      int[] count = {0};
      try (SurfaceConsolidator s = new SurfaceConsolidator(o, fusion)) {
        for (int i = 0; i < 12000; i++)
          s.add(
              new SurfaceConsolidator.Sample(
                  i,
                  0,
                  0,
                  0,
                  new DepthFusion.Result(new double[] {0, 0, 10}, 3, 1, .01, 0, "accepted"),
                  new DepthFusion.SurfaceSample(null, .05, .01)));
        s.finish(
            new SurfaceConsolidator.Sink() {
              public void point(SurfaceConsolidator.Sample p) {
                count[0]++;
              }
            });
      }
      check(
          count[0] == 12000,
          "A scale-degenerate partition exceeding its working-set budget passes through without"
              + " point loss");
      boolean failed = false;
      try (SurfaceConsolidator s = new SurfaceConsolidator(o, fusion)) {
        s.add(
            new SurfaceConsolidator.Sample(
                1,
                0,
                0,
                0,
                new DepthFusion.Result(new double[] {0, 0, 10}, 3, 1, .01, 0, "accepted"),
                new DepthFusion.SurfaceSample(null, .05, .01)));
        s.finish(
            new SurfaceConsolidator.Sink() {
              public void point(SurfaceConsolidator.Sample p) throws java.io.IOException {
                throw new java.io.IOException("injected output failure");
              }
            });
      } catch (java.io.IOException expected) {
        failed = expected.getMessage().equals("injected output failure");
      }
      check(failed, "Output failure is propagated rather than publishing partial surface results");
      Path broken = Files.createDirectories(o.output.resolve("broken output"));
      Files.createDirectories(broken.resolve("07_results/point-confidence.bin"));
      Options bad = o.copy();
      bad.output = broken;
      boolean rejected = false;
      try (FusionOutput output = new FusionOutput(bad, fusion, null, null)) {
      } catch (java.io.IOException expected) {
        rejected = true;
      }
      check(rejected, "Confidence initialization failure aborts cloud staging");
      try (var files = Files.list(broken)) {
        check(
            files.noneMatch(p -> p.getFileName().toString().startsWith("cloud-")),
            "Cloud spool is removed even when output construction fails");
      }
      try (var files = Files.list(o.output)) {
        check(
            files.noneMatch(p -> p.getFileName().toString().startsWith("surface-work-")),
            "Temporary partitions are removed after success, budget passthrough and output"
                + " failure");
      }
    }
  }

  private static void endToEnd(Path root) throws Exception {
    Path dir = Files.createDirectories(root.resolve("surface end to end"));
    Path images = SelfTest.scene(dir, 3, false),
        calibration = dir.resolve("camera.properties"),
        poses = dir.resolve("poses.csv");
    Files.writeString(calibration, "width=400\nheight=300\nfx=240\nfy=240\ncx=199.5\ncy=149.5\n");
    Files.writeString(
        poses,
        PoseIO.HEADER
            + "\n"
            + "00.png,1,0,0,0,1,0,0,0,1,0,0,0\n"
            + "01.png,1,0,0,0,1,0,0,0,1,-0.25,0,0\n"
            + "02.png,1,0,0,0,1,0,0,0,1,-0.5,0,0\n");
    Path[] runs = new Path[2];
    for (int version = 0; version < 2; version++) {
      Path out = dir.resolve(version == 0 ? "pipeline" : "synchronous");
      List<String> args =
          new ArrayList<>(
              List.of(
                  "-i",
                  images.toString(),
                  "-o",
                  out.toString(),
                  "--calibration",
                  calibration.toString(),
                  "--poses",
                  poses.toString(),
                  "--disparities",
                  "40",
                  "--step",
                  "4",
                  "--compute",
                  "cpu",
                  "--cores",
                  "1",
                  "--storage",
                  "debug",
                  "--diagnostics",
                  "--surface-memory-mb",
                  "8"));
      if (version == 1) args.add("--no-fusion-pipeline");
      check(
          Main.run(args.toArray(String[]::new)) == 0,
          "New default surface pipeline completes end to end, asynchronous=" + (version == 0));
      runs[version] = ProjectRegression.latest(out, "latest_successful_run");
      java.nio.ByteBuffer ply = CloudEquivalence.cloud(runs[version]);
      int count = ply.remaining() / 27, rows = 0;
      long merged = 0;
      try (var confidence = CloudEquivalence.confidence(runs[version])) {
        check(
            confidence.readLine().contains("merged_observations"),
            "Final confidence includes normals, surface shifts and consolidation multiplicity");
        for (String line; (line = confidence.readLine()) != null; ) {
          String[] f = line.split(",");
          if (Integer.parseInt(f[0]) != rows++)
            throw new AssertionError("Final confidence IDs are not contiguous");
          for (int k = 0; k < 3; k++)
            if (Math.abs(Double.parseDouble(f[k + 3]) - ply.getDouble()) > 1e-12)
              throw new AssertionError(
                  "Confidence coordinates do not describe the final published cloud");
          ply.position(ply.position() + 3);
          merged += Integer.parseInt(f[17]);
        }
      }
      check(rows == count && count > 3000, "Final confidence rows and cloud geometry agree");
      check(merged > count, "End-to-end output consolidates overlapping stereo candidates");
      Path decisions = runs[version].resolve("06_fusion/surface_decisions.csv.gz");
      check(
          Files.isRegularFile(decisions),
          "Debug output preserves a separate surface decision/provenance stream");
      // Disk-backed input makes accidental cross-thread page-cache access observable.
      try (var files = Files.list(runs[version])) {
        check(
            files.noneMatch(p -> p.getFileName().toString().startsWith("surface-work-")),
            "Successful run removes all surface work files");
      }
    }
    CloudEquivalence.main(
        new String[] {
          runs[0].toString(),
          runs[1].toString(),
          dir.resolve("pipeline-equivalence.json").toString()
        });
    check(
        true,
        "Prefetch pipeline and synchronous path produce identical final decisions and coordinates");
    Path voxel = dir.resolve("legacy voxel contract");
    check(
        Main.run(
                new String[] {
                  "-i",
                  images.toString(),
                  "-o",
                  voxel.toString(),
                  "--calibration",
                  calibration.toString(),
                  "--poses",
                  poses.toString(),
                  "--disparities",
                  "40",
                  "--step",
                  "4",
                  "--compute",
                  "cpu",
                  "--cores",
                  "1",
                  "--storage",
                  "debug",
                  "--surface-consolidation",
                  "none",
                  "--depth-refinement",
                  "weighted",
                  "--voxel",
                  ".15",
                  "--diagnostics"
                })
            == 0,
        "Voxel thinning remains usable without surface consolidation");
    Path run = ProjectRegression.latest(voxel, "latest_successful_run");
    int accepted = 0, duplicates = 0;
    try (var in =
        new java.io.BufferedReader(
            new java.io.InputStreamReader(
                new java.util.zip.GZIPInputStream(
                    Files.newInputStream(run.resolve("06_fusion/decisions.csv.gz"))),
                java.nio.charset.StandardCharsets.UTF_8))) {
      in.readLine();
      for (String line; (line = in.readLine()) != null; ) {
        String status = line.split(",")[5];
        if (status.equals("accepted")) accepted++;
        if (status.equals("voxel_duplicate")) duplicates++;
      }
    }
    int points = CloudEquivalence.cloud(run).remaining() / 27;
    check(duplicates > 100, "Legacy debug decisions retain explicit voxel-duplicate statuses");
    check(
        accepted == points,
        "Without spatial consolidation, accepted debug decisions still match final cloud count"
            + " after voxel thinning");
  }

  private static void medianTie(Path root) throws Exception {
    Options o = new Options();
    o.compute = "cpu";
    o.output = Files.createDirectories(root.resolve("balanced median"));
    int[] w = {160, 160, 160, 160}, h = {120, 120, 120, 120};
    Camera[] cameras = cameras();
    o.minViews = 3;
    try (DepthFusion f = new DepthFusion(cameras, w, h, o);
        SurfaceConsolidator s = new SurfaceConsolidator(o, f)) {
      f.addPair(0, 1);
      f.addPair(0, 2);
      int id = 0;
      for (int pair = 0; pair < 2; pair++)
        for (int y = -4; y <= 4; y++)
          for (int x = -4; x <= 4; x++) {
            double[] p = {x * .05, y * .05, 10 + (pair == 0 ? .01 : -.01)};
            f.add(pair, p);
            s.add(
                new SurfaceConsolidator.Sample(
                    id++,
                    pair,
                    0,
                    0,
                    new DepthFusion.Result(p, 3, 1, .01, 0, "accepted"),
                    new DepthFusion.SurfaceSample(new double[] {0, 0, -1}, .05, .01)));
          }
      double[] sum = {0};
      int[] n = {0};
      s.finish(
          new SurfaceConsolidator.Sink() {
            public void point(SurfaceConsolidator.Sample p) {
              sum[0] += p.point[2];
              n[0]++;
            }
          });
      check(
          n[0] > 20 && Math.abs(sum[0] / n[0] - 10) < 1e-8,
          "Balanced even-weight observations consolidate to the mid-surface without front/back tie"
              + " bias");
    }
  }
}
