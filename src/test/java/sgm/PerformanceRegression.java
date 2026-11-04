package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import java.util.concurrent.*;

/** Reduced-size controls only. Never launches the unrestricted repository suite. */
public final class PerformanceRegression {
  static int checks;

  static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    org.opencv.core.Core.setNumThreads(1);
    Path root = Files.createDirectories(Path.of("target/performance-revision/controls"));
    if (args.length > 0 && args[0].startsWith("compare")) {
      compare(
          Path.of(args[1]),
          Path.of(args[2]),
          !args[0].equals("compare"),
          args[0].equals("compare-measure"));
      return;
    }
    records(root);
    atlas(root);
    publication(root);
    pages(root);
    numerics();
    cameraEmission(root);
    selection(root);
    zncc(root);
    System.out.println("PASS performance infrastructure: " + checks);
    if (args.length > 0 && args[0].equals("fusion")) {
      System.out.println("Fusion GPU checks: " + FusionGpuRegression.run(root));
      System.out.println("Measured reprojection checks: " + MeasuredReprojectionRegression.run());
      System.out.println("Surface checks: " + RegularizationSurfaceRegression.run());
      System.out.println("Bundle checks: " + BundleQualityRegression.run(root));
      System.out.println("Schur checks: " + BundleSchurRegression.run(root));
    }
    if (args.length > 0 && args[0].equals("bundle")) {
      Path dir = Files.createTempDirectory(Path.of("target"), "pr-controls-");
      System.out.println("Bundle checks: " + BundleQualityRegression.run(dir));
      System.out.println("Schur checks: " + BundleSchurRegression.run(dir));
      System.out.println("Shared calibration checks: " + SharedCalibrationRegression.run(dir));
      System.out.println("Surface fusion checks: " + SurfaceFusionRegression.run(dir));
    }
    if (args.length > 0 && args[0].equals("gpu")) {
      Path dir = Files.createTempDirectory(Path.of("target"), "pr-gpu-");
      System.out.println("Hybrid SGM checks: " + HybridSgmRegression.run(dir));
      System.out.println("Two-view checks: " + TwoViewConfidenceRegression.run(dir));
      System.out.println("Fusion GPU checks: " + FusionGpuRegression.run(dir));
      System.out.println("Depth I/O checks: " + DepthIoRegression.run(dir));
    }
    if (args.length > 0 && args[0].equals("atlas")) {
      Path dir = Files.createTempDirectory(Path.of("target"), "pr-atlas-");
      System.out.println("Packed fusion GPU checks: " + FusionGpuRegression.run(dir, true));
    }
    GpuSgm.closeSession();
  }

  static void zncc(Path root) throws Exception {
    int w = 72, h = 46, n = w * h, D = 13;
    byte[] a = new byte[n], b = new byte[n];
    boolean[] ma = new boolean[n], mb = new boolean[n];
    Random random = new Random(717);
    random.nextBytes(a);
    random.nextBytes(b);
    for (int i = 0; i < n; i++) {
      ma[i] = i % 17 != 0;
      mb[i] = i % 19 != 0;
    }
    for (int window : new int[] {3, 5, 9, 11})
      for (boolean reverse : new boolean[] {false, true}) {
        Options o = new Options();
        o.matchingCost = "zncc";
        o.census = window;
        int radius = window / 2, minimum = -5;
        try (var costs = new PagedVolume((long) n * D, 2, 512, false, o, "zncc-control-")) {
          ZnccCosts.fill(costs, a, b, ma, mb, w, h, minimum, D, window, reverse);
          for (int y = 0; y < h; y++)
            for (int x = 0; x < w; x++)
              for (int k = 0; k < D; k++) {
                int p = y * w + x, shift = reverse ? minimum + k : -(minimum + k), qx = x + shift;
                int expected = 512;
                if (y >= radius
                    && y < h - radius
                    && x >= radius
                    && x < w - radius
                    && qx >= radius
                    && qx < w - radius
                    && ma[p]
                    && mb[p + shift])
                  expected =
                      StereoCost.value(o, new long[0][], new long[0][], a, b, p, p + shift, w);
                check(
                    costs.get((long) p * D + k) == expected,
                    "sliding ZNCC exact signed/masked cost");
              }
        }
      }
  }

  static void cameraEmission(Path root) throws Exception {
    Options o = new Options();
    o.output = Files.createDirectories(root.resolve("owner-output"));
    Files.createDirectories(o.output.resolve("06_fusion"));
    o.surfaceMemoryMb = 8;
    Camera a = new Camera(100, 100, 79.5, 59.5), b = a.copy();
    b.translation[0] = -.2;
    try (var fusion =
            new DepthFusion(new Camera[] {a, b}, new int[] {160, 160}, new int[] {120, 120}, o);
        var emission = new CameraEmission(o, fusion)) {
      fusion.addPair(0, 1);
      for (int i = 0; i < 10000; i++) {
        var p = new SurfaceConsolidator.Sample();
        p.id = i;
        p.source = 0;
        p.point = new double[] {0, 0, 10 + (i % 3) * 10};
        p.views = 2;
        p.sigma = .01;
        p.matchingConfidence = (i % 7 == 0) ? .9 : .5;
        p.supportingCameras.set(0);
        p.supportingCameras.set(1);
        emission.add(p);
      }
      List<SurfaceConsolidator.Sample> output = new ArrayList<>();
      emission.finish(output::addAll);
      check(output.size() == 3, "separate owner-camera depth layers retained");
      check(emission.merged() == 9997, "bounded multipass camera grouping");
      for (var p : output) {
        check(p.matchingConfidence == .9, "strongest original evidence retained");
        check(p.views == 2 && p.supportingCameras.cardinality() == 2, "no invented camera support");
        check(p.point[0] == 0 && p.point[1] == 0, "no coordinate averaging");
      }
    }
  }

  static void atlas(Path root) throws Exception {
    Options o = new Options();
    o.output = root.resolve("packed-bits");
    o.projectDirectory = root;
    var pool = new PagedVolume.Pool(1048576, 64);
    int w = 83, h = 57, n = w * h;
    List<List<DepthFusion.Endpoint>> endpoints = new ArrayList<>();
    try {
      for (int image = 0; image < 2; image++) {
        List<DepthFusion.Endpoint> maps = new ArrayList<>();
        endpoints.add(maps);
        for (int rank = 0; rank < 32; rank++) {
          var map = new DepthRaster(n, o, pool);
          maps.add(new DepthFusion.Endpoint(rank, image, map));
          for (int p = 0; p < n; p++)
            if ((p * 31 + rank * 7) % 19 == 0 || p == 777)
              map.put(
                  p,
                  p * .125f + .5f + rank,
                  DepthFusion.pack((rank % 5 - 2) * .1, (p % 7 - 3) * .05));
        }
      }
      var atlas =
          CameraEvidence.build(
              endpoints, new int[] {w, w}, new int[] {h, h}, 64L << 20, 2L * n * 32 * 8, false);
      check(atlas != null, "camera atlas built");
      check(atlas.images[0].mask[777] == -1, "rank 31 uses unsigned mask bit");
      for (int image = 0; image < 2; image++)
        for (int rank = 0; rank < 32; rank++)
          for (int p = 0; p < n; p++) {
            long value = atlas.images[image].value(p, rank);
            var map = endpoints.get(image).get(rank).map();
            check((int) value == Float.floatToRawIntBits(map.get(p)), "packed depth bits exact");
            check((int) (value >>> 32) == map.offset(p), "packed offset bits exact");
          }
      check(atlas.bytes < 2L * n * 32 * 8 / 2, "sparse camera layout saves memory");
      check(
          CameraEvidence.build(
                  endpoints, new int[] {w, w}, new int[] {h, h}, 1024, 2L * n * 32 * 8, false)
              == null,
          "atlas obeys host budget");
      endpoints.get(0).add(endpoints.get(0).get(0));
      check(
          CameraEvidence.build(
                  endpoints, new int[] {w, w}, new int[] {h, h}, 64L << 20, 2L * n * 32 * 8, false)
              == null,
          "unsupported degree returns reference fallback");
      endpoints.get(0).remove(32);
      atlas.clear();
    } finally {
      for (var maps : endpoints)
        for (var endpoint : maps) {
          endpoint.map().close();
          endpoint.map().discard();
        }
    }
  }

  static void publication(Path root) throws Exception {
    Path output = Files.createTempDirectory(root, "publication-");
    Options first = new Options();
    first.input = root;
    first.output = output;
    first.processingReport = "none";
    first.compatibilityCopies = true;
    Project before = Project.open(first, new String[0]);
    byte[] old = {1, 2, 3};
    Files.write(before.run.resolve("07_results/dense.las"), old);
    Files.write(before.run.resolve("07_results/dense.ply"), old);
    before.finish(true, null);
    check(
        Arrays.equals(Files.readAllBytes(output.resolve("dense.las")), old),
        "explicit compatibility copy published");
    Options second = new Options();
    second.input = root;
    second.output = output;
    second.processingReport = "none";
    Project after = Project.open(second, new String[0]);
    Files.write(after.run.resolve("07_results/dense.las"), new byte[] {4, 5, 6});
    Files.write(after.run.resolve("07_results/dense.ply"), new byte[] {4, 5, 6});
    after.finish(true, null);
    check(
        !Files.exists(output.resolve("dense.las")),
        "compact publication leaves no stale root alias");
    check(
        Arrays.equals(
            Files.readAllBytes(
                after.run.resolve("00_inputs/retired_compatibility_copies/dense.las")),
            old),
        "old root copy preserved");
    check(
        Files.readString(output.resolve("latest-files.csv"))
            .contains("runs/" + after.id + "/07_results/dense.las"),
        "manifest points to new canonical cloud");
  }

  static void selection(Path root) throws Exception {
    Path image = root.resolve("selection.png");
    var mat =
        new org.opencv.core.Mat(
            120, 160, org.opencv.core.CvType.CV_8U, new org.opencv.core.Scalar(100));
    try {
      org.opencv.imgcodecs.Imgcodecs.imwrite(image.toString(), mat);
    } finally {
      mat.release();
    }
    Options o = new Options();
    o.output = root.resolve("pair-selection");
    o.densePairSelection = "coverage";
    o.densePairNeighbors = 2;
    Calibration k = new Calibration(100, 100, 79.5, 59.5, new double[5], 160, 120);
    List<ImageFrame> frames = new ArrayList<>();
    try {
      for (int i = 0; i < 4; i++) {
        var f = new ImageFrame(i, image, k, o, true);
        f.camera.translation[0] = -.3 * i;
        f.camera.registered = true;
        frames.add(f);
      }
      List<FeatureTracks.Pair> pairs = new ArrayList<>();
      for (int i = 0; i < 4; i++)
        for (int j = i + 1; j < 4; j++) pairs.add(new FeatureTracks.Pair(i, j));
      check(
          DensePairSelection.select(frames, pairs, List.of(), o).size() == 6,
          "unknown coverage keeps original pairs");
      List<FeatureTracks.Track> tracks = new ArrayList<>();
      for (int y = -3; y <= 3; y++)
        for (int x = -4; x <= 4; x++) {
          var t = new FeatureTracks.Track();
          t.point = new double[] {x * .5, y * .5, 10 + (x + 4) * .2};
          for (int i = 0; i < 4; i++) t.observations.add(new FeatureTracks.Observation(i, 0));
          tracks.add(t);
        }
      var selected = DensePairSelection.select(frames, pairs, tracks, o);
      int[] degree = new int[4];
      for (var p : selected) {
        degree[p.a]++;
        degree[p.b]++;
      }
      for (int d : degree) check(d >= 2, "minimum camera coverage degree retained");
      check(selected.size() <= 6, "selector adds no unverified pairs");
    } finally {
      for (var f : frames) f.close();
    }
  }

  static void compare(Path before, Path after, boolean numerical, boolean measure)
      throws Exception {
    double[] maximum = new double[32];
    long[] changed = new long[32];
    try (var old =
            new BufferedReader(
                new InputStreamReader(
                    new java.util.zip.GZIPInputStream(
                        Files.newInputStream(
                            before.resolve("07_results/point-confidence.csv.gz")))));
        var records = new PointRecords.Reader(after.resolve("07_results/point-confidence.bin"))) {
      check(old.readLine().equals(PointRecords.HEADER.stripTrailing()), "CSV schema unchanged");
      PointRecords.Row row;
      long count = 0;
      while ((row = records.next()) != null) {
        String expected = old.readLine();
        String actual = row.csv().stripTrailing();
        if (!numerical) check(actual.equals(expected), "all legacy CSV fields at " + count);
        else {
          check(expected != null, "reference row exists");
          String[] a = actual.split(","), b = expected.split(",");
          check(a.length == b.length, "row schema");
          for (int k = 0; k < a.length; k++)
            if (!a[k].equals(b[k])) {
              changed[k]++;
              if (k <= 2)
                check(false, "candidate and order unchanged at " + count + " column " + k);
              if (k >= 6 && k <= 10 || k == 17 || k >= 22 && k <= 24 || k == 31) {
                if (!measure)
                  check(false, "support/selection unchanged at " + count + " column " + k);
                continue;
              }
              double x = Double.parseDouble(a[k]), y = Double.parseDouble(b[k]);
              if (Double.isNaN(x) && Double.isNaN(y)) continue;
              double difference = Math.abs(x - y);
              maximum[k] = Math.max(maximum[k], difference);
              if (!measure)
                check(
                    Double.isFinite(difference)
                        && difference
                            <= (k == 12
                                    ? 2e-8
                                    : k >= 13 && k <= 21 ? 1e-8 : k == 11 || k == 25 ? 1e-9 : 2e-8)
                                + 2e-12 * Math.max(Math.abs(x), Math.abs(y)),
                    "bounded numeric difference at " + count + " column " + k + ": " + difference);
            }
        }
        count++;
      }
      check(old.readLine() == null, "point counts equal");
      System.out.println("PASS exact metadata comparison: " + count + " rows");
      if (numerical)
        Json.write(
            Path.of("target/performance-revision/numerical-parity.json"),
            Json.object(
                "rows",
                count,
                "same_candidate_order_support",
                true,
                "maximum_absolute_difference_by_csv_column",
                maximum,
                "changed_values_by_csv_column",
                changed));
    }
  }

  static void numerics() {
    Random random = new Random(913);
    for (int i = 0; i < 10000; i++) {
      double x = random.nextGaussian(), y = random.nextGaussian(), r = Math.hypot(x, y);
      for (double limit : new double[] {r, Math.nextDown(r), Math.nextUp(r), .75})
        check(
            DepthFusion.within(x, y, limit) == (Math.hypot(x, y) <= limit),
            "squared-distance boundary fallback");
    }
    for (int trial = 0; trial < 1000; trial++) {
      double offset = trial % 3 == 0 ? 6000000 : 0;
      double[][] points = new double[49][3];
      for (int i = 0; i < 49; i++) {
        double x = random.nextGaussian(), y = random.nextGaussian();
        points[i] =
            new double[] {
              offset + x,
              offset + y,
              offset + .3 * x + .7 * y + random.nextGaussian() * (trial % 5 == 0 ? 1e-12 : .01)
            };
      }
      var expected = ReferenceSurfaceGeometry.plane(points, 49);
      var actual = SurfaceGeometry.plane(points, 49);
      check((expected == null) == (actual == null), "plane availability");
      if (actual != null) {
        check(
            Math.abs(Math.abs(SurfaceGeometry.dot(expected.normal(), actual.normal())) - 1) < 1e-10,
            "eigenvector parity");
        check(Math.abs(actual.smallest() - expected.smallest()) < 1e-10, "eigenvalue parity");
      }
      int n = trial % 2 == 0 ? 3 : 6;
      double[] m = new double[n * n], a = new double[n * n];
      for (int i = 0; i < m.length; i++) m[i] = random.nextGaussian();
      for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++) {
          for (int k = 0; k < n; k++) a[i * n + j] += m[k * n + i] * m[k * n + j];
          if (i == j) a[i * n + j] += .1;
        }
      check(SmallMatrices.inverse(a, n) != null, "SPD inverse certificate");
    }
    for (int trial = 0; trial < 500; trial++) {
      Camera anchor = new Camera(700, 700, 319.5, 239.5);
      anchor.translation = new double[] {-.2, .1, 0};
      double[] point = {
        random.nextGaussian(), random.nextGaussian(), 15 + random.nextDouble() * 15
      };
      List<ReprojectionDepth.Observation> actual = new ArrayList<>();
      List<ReferenceReprojectionDepth.Observation> expected = new ArrayList<>();
      var context = new ReprojectionDepth.Context(anchor, point, anchor.center());
      for (int i = 0; i < 4; i++) {
        Camera c = anchor.copy();
        c.translation = new double[] {(i + 1) * .5, .1, 0};
        double[] measured = point.clone();
        measured[2] += .02 * random.nextGaussian();
        actual.add(ReprojectionDepth.link(context, c, measured));
        expected.add(ReferenceReprojectionDepth.link(anchor, c, point, measured));
      }
      var old = ReferenceReprojectionDepth.fit(anchor, point, point[2] * 1.01, expected, .75);
      var revised = ReprojectionDepth.fit(context, point[2] * 1.01, actual, .75);
      check(
          Double.doubleToLongBits(old.depth()) == Double.doubleToLongBits(revised.depth()),
          "reprojection depth bit parity");
      check(
          Double.doubleToLongBits(old.relativeSigma())
              == Double.doubleToLongBits(revised.relativeSigma()),
          "reprojection sigma bit parity");
    }
  }

  static void records(Path root) throws Exception {
    Options parsed = Options.parse(new String[] {"-i", root.toString(), "--diagnostics"});
    check(parsed.diagnostics, "diagnostics parsed");
    check(!new Options().diagnostics, "diagnostics default off");
    for (String[] invalid :
        List.of(new String[] {"--cores", "0"}, new String[] {"--output-compression", "10"})) {
      boolean bad = false;
      try {
        Options.parse(
            new String[] {"--export-confidence", root.toString(), invalid[0], invalid[1]});
      } catch (IllegalArgumentException expected) {
        bad = true;
      }
      check(bad, "standalone export validates " + invalid[0]);
    }
    var p = new SurfaceConsolidator.Sample();
    p.id = 19;
    p.source = 3;
    p.point = new double[] {1.25, -3.125, 4567890.23456};
    p.rgb = 0x1357ad;
    p.views = 4;
    p.pairs = 7;
    p.sigma = .023;
    p.shift = .125;
    p.normal = new double[] {.6, .8, 0};
    p.attributes = new SurfaceAttributes(.032, .008, .9, 1.2, 29, 0, true);
    p.supportingCameras.set(1);
    p.supportingCameras.set(129);
    p.matchingConfidence = .8;
    p.costMargin = .4;
    p.costSharpness = .5;
    p.photometric = .95;
    p.lrError = .125;
    Path run = Files.createDirectories(root.resolve("metadata/07_results"));
    try (var out = new PointRecords.Writer(run.resolve("point-confidence.bin"))) {
      out.point(0, p, 2, .7);
    }
    try (var in = new PointRecords.Reader(run.resolve("point-confidence.bin"))) {
      var row = in.next();
      check(row.id() == 0, "point index");
      check(row.sample().id == 19, "candidate id");
      check(row.sample().point[2] == p.point[2] * 2, "lossless XYZ");
      check(row.sample().supportingCameras.equals(p.supportingCameras), "wide provenance");
      check(row.sample().attributes.roughness() == .064, "scaled attrs");
      check(in.next() == null, "metadata footer");
    }
    PointRecords.export(run, 1);
    byte[] a;
    try (var in =
        new java.util.zip.GZIPInputStream(
            Files.newInputStream(run.resolve("point-confidence.csv.gz")))) {
      a = in.readAllBytes();
    }
    PointRecords.export(run, 6);
    byte[] b;
    try (var in =
        new java.util.zip.GZIPInputStream(
            Files.newInputStream(run.resolve("point-confidence.csv.gz")))) {
      b = in.readAllBytes();
    }
    check(Arrays.equals(a, b), "compression does not change rows");
    byte[] damaged = Files.readAllBytes(run.resolve("point-confidence.bin"));
    Files.write(root.resolve("truncated.bin"), Arrays.copyOf(damaged, damaged.length - 4));
    boolean rejected = false;
    try (var in = new PointRecords.Reader(root.resolve("truncated.bin"))) {
      while (in.next() != null) {}
    } catch (EOFException e) {
      rejected = true;
    }
    check(rejected, "truncated metadata rejected");
  }

  static void pages(Path root) throws Exception {
    for (int side : new int[] {16, 32, 64, 128})
      for (boolean resident : new boolean[] {false, true}) {
        Options o = new Options();
        o.output = root.resolve("pages-" + side + "-" + resident);
        o.projectDirectory = root;
        o.fusionPageSide = side;
        var pool = new PagedVolume.Pool(1048576, 8);
        pool.readerCount(5);
        var map = new DepthRaster(32789, o, pool);
        for (int i = 0; i < map.length; i++)
          if (i % 5 != 0) map.put(i, i * .125f, DepthFusion.pack(.25, -.125));
        map.freeze(resident);
        boolean immutable = false;
        try {
          map.put(1, 999, 0);
        } catch (IllegalStateException expected) {
          immutable = true;
        }
        check(immutable, "frozen file evidence rejects writes");
        var executor = Executors.newFixedThreadPool(4);
        List<Future<Boolean>> jobs = new ArrayList<>();
        for (int w = 0; w < 4; w++) {
          final int seed = w;
          jobs.add(
              executor.submit(
                  () -> {
                    Random random = new Random(seed);
                    for (int j = 0; j < 12000; j++) {
                      int i = random.nextInt(map.length);
                      if (map.get(i) != (i % 5 == 0 ? 0 : i * .125f)) return false;
                      if (i % 5 != 0 && map.offset(i) != DepthFusion.pack(.25, -.125)) return false;
                    }
                    return true;
                  }));
        }
        for (var job : jobs) check(job.get(), "immutable concurrent depth and offsets");
        executor.shutdown();
        executor.awaitTermination(30, TimeUnit.SECONDS);
        if (!resident) check(pool.frozenReads.sum() > 0, "disk reader exercised");
        map.close();
        map.discard();
      }
    Options o = new Options();
    o.output = root.resolve("handle-overflow");
    o.diskDepth = false;
    try (var map = new DepthRaster(17, o)) {
      map.put(1, 7, 0);
      map.freeze(true);
      boolean immutable = false;
      try {
        map.put(1, 99, 0);
      } catch (IllegalStateException expected) {
        immutable = true;
      }
      check(immutable && map.get(1) == 7, "frozen in-memory evidence rejects writes");
    }
    o.diskDepth = true;
    o.projectDirectory = root;
    o.fusionPageSide = 16;
    var pool = new PagedVolume.Pool(65536, 8);
    pool.readerCount(1);
    List<DepthRaster> maps = new ArrayList<>();
    try {
      for (int i = 0; i < 600; i++) {
        var map = new DepthRaster(17, o, pool);
        maps.add(map);
        map.put(3, i + 1, 0);
        map.freeze(false);
      }
      for (int i = 0; i < maps.size(); i++)
        check(maps.get(i).get(3) == i + 1, "overflow handle positional read");
      check(pool.peakPinnedFiles == 512, "pinned read-handle limit exercised");
      check(pool.peakChannels <= 8, "overflow shared handle pool remains bounded");
      check(pool.frozenReads.sum() >= 600, "all overflow files read from disk");
    } finally {
      pool.releaseReader();
      pool.closeReaders();
      for (var map : maps) {
        map.close();
        map.discard();
      }
    }
  }
}
