package sgm;

import java.io.*;
import java.nio.*;
import java.nio.file.*;
import java.security.*;
import java.util.*;

final class FollowupThroughputRegression {
  private static int checks;

  static void check(boolean ok, String message) {
    if (!ok) throw new AssertionError(message);
    checks++;
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    System.out.println(
        "PASS throughput/report checks="
            + run(Files.createTempDirectory(Path.of("target"), "throughput report ")));
  }

  static int run(Path root) throws Exception {
    checks = 0;
    partition(root);
    projection(root);
    reports(root);
    return checks;
  }

  private static void partition(Path root) throws Exception {
    String reference = null;
    for (String mode : List.of("recursive", "routed")) {
      Path out = Files.createDirectories(root.resolve(mode));
      Options o = new Options();
      o.output = out;
      o.surfacePartition = mode;
      o.surfaceMemoryMb = 8;
      o.memoryMb = 128;
      o.compute = "cpu";
      o.surfaceValidation = "cpu";
      Camera[] cameras = {new Camera(100, 100, 20, 20), new Camera(100, 100, 20, 20)};
      cameras[1].translation = new double[] {-1, 0, 0};
      int[] sizes = {40, 40};
      MessageDigest hash = MessageDigest.getInstance("SHA-256");
      try (DepthFusion fusion = new DepthFusion(cameras, sizes, sizes, o)) {
        fusion.addPair(0, 1);
        try (SurfaceConsolidator surface = new SurfaceConsolidator(o, fusion)) {
          Random random = new Random(42);
          for (int i = 0; i < 23000; i++) {
            double[] p = {
              600000 + random.nextDouble() * 10,
              6800000 + random.nextDouble() * 10,
              100 + random.nextDouble()
            };
            var sample =
                new SurfaceConsolidator.Sample(
                    i,
                    0,
                    i,
                    0x123456,
                    new DepthFusion.Result(p, 3, 1, .001, 0, "accepted"),
                    new DepthFusion.SurfaceSample(null, .01, .001));
            if (i % 3 == 0)
              sample.attributes = new SurfaceAttributes(.01, .02, .8, .1, 25, 0, false);
            surface.add(sample);
          }
          surface.finish(
              p -> {
                ByteBuffer b = ByteBuffer.allocate(40).order(ByteOrder.BIG_ENDIAN);
                b.putLong(p.id);
                for (double v : p.point) b.putDouble(v);
                b.putInt(p.attributes.status());
                hash.update(b.array(), 0, b.position());
              });
        }
      }
      String result = HexFormat.of().formatHex(hash.digest());
      if (reference == null) reference = result;
      else
        check(
            reference.equals(result),
            "Routed partitions preserve exact recursive leaf/record order and attributes");
      try (var files = Files.list(out)) {
        check(
            files.noneMatch(p -> p.getFileName().toString().startsWith("surface-work-")),
            "Owned surface scratch is removed: " + mode);
      }
    }
  }

  private static void projection(Path root) throws Exception {
    Options o = new Options();
    o.output = root;
    Camera[] cameras = {new Camera(1500, 1520, 800, 700), new Camera(1500, 1520, 800, 700)};
    double[] center = {600000, 6800000, 100};
    cameras[0].rotation = Math3.rotation(new double[] {.1, -.2, .03});
    cameras[1].rotation = Math3.rotation(new double[] {.08, -.21, .03});
    for (int i = 0; i < 2; i++)
      cameras[i].translation =
          Math3.times(
              Math3.mv(
                  cameras[i].rotation, new double[] {center[0] + i, center[1], center[2] - 50}),
              -1);
    boolean same = true;
    Random random = new Random(5);
    try (DepthFusion fusion =
        new DepthFusion(cameras, new int[] {1600, 1600}, new int[] {1400, 1400}, o)) {
      int source = fusion.addPair(0, 1);
      for (int i = 0; i < 5000; i++) {
        double[]
            before =
                {
                  center[0] + random.nextDouble() * 10,
                  center[1] + random.nextDouble() * 10,
                  center[2]
                },
            direction = {random.nextDouble(), random.nextDouble(), random.nextDouble()};
        double amount = (random.nextDouble() - .5) * .1;
        double[] after = Math3.add(before, Math3.times(direction, amount));
        boolean reference = true;
        for (Camera camera : cameras) {
          double[] a = camera.project(before), b = camera.project(after);
          reference &=
              a != null && b != null && Math.hypot(a[0] - b[0], a[1] - b[1]) <= o.fusionPixels;
        }
        same &=
            fusion.surfaceDisplacement(source, before, after) == reference
                && fusion.surfaceDisplacement(source, before, direction, amount) == reference;
      }
    }
    check(same, "Projection guards match reference on 5000 large-coordinate controls");
  }

  private static void reports(Path root) throws Exception {
    Path run = Files.createDirectories(root.resolve("report-run")),
        out = Files.createDirectories(root.resolve("report-output"));
    Files.createDirectories(run.resolve("07_results"));
    Files.createDirectories(run.resolve("logs"));
    Json.write(
        run.resolve("run.json"),
        Json.object(
            "status",
            "complete",
            "run_id",
            "test<&>",
            "arguments",
            new String[] {"--quality", "high"},
            "stages",
            Json.object("05_stereo", Json.object("status", "complete", "elapsed_seconds", 1.25))));
    Json.write(
        run.resolve("07_results/coordinates.json"),
        Json.object(
            "georeferenced", true, "epsg", 3067, "xyz_units", "metres", "vertical_crs", null));
    try (CloudWriter cloud = new CloudWriter(run.resolve("07_results"), 1, new Crs("3067"))) {
      cloud.point(100, 200, 10, 0, 0, 0);
      cloud.point(100.2, 199.8, 12, 0, 0, 0);
      cloud.point(101.2, 199.8, 15, 0, 0, 0);
    }
    Options o = new Options();
    o.input = root;
    o.output = run;
    o.reportDsmResolution = 1;
    o.quality = "high";
    o.ratio = .83;
    try (RunTelemetry telemetry = new RunTelemetry(run, o)) {
      try (WorkProgress work = WorkProgress.start("measured test task", 3, "records")) {
        work.update(2);
        work.done();
      }
      telemetry.persist();
      ProcessingReport.generate(o, run, out);
    }
    Map<String, Object> report = ProcessingReport.map(out.resolve("processing-report.json"));
    Map<String, Object> parameters = ProcessingReport.object(report.get("parameters"));
    for (var field : Options.class.getFields())
      check(
          parameters.containsKey(field.getName()), "Report includes parameter " + field.getName());
    check(
        ((Number) parameters.get("ratio")).doubleValue() == .83, "Report contains actual override");
    check("high".equals(parameters.get("quality")), "Report retains selected quality");
    var dsm = ProcessingReport.object(report.get("dsm"));
    check("complete".equals(dsm.get("status")), "DSM generated");
    check(((Number) dsm.get("occupied_cells")).intValue() == 2, "DSM occupied cells counted");
    try (var input =
        new javax.imageio.stream.FileImageInputStream(out.resolve("dsm.tif").toFile())) {
      var reader = BigTiffWindows.jdkTiff();
      try {
        reader.setInput(input);
        var raster = reader.read(0).getRaster();
        check(
            raster.getWidth() == 2 && raster.getHeight() == 1,
            "Independent TIFF decoder dimensions");
        check(
            raster.getSampleFloat(0, 0, 0) == 12 && raster.getSampleFloat(1, 0, 0) == 15,
            "Max DSM elevations and north-up indexing");
      } finally {
        reader.dispose();
      }
    }
    String html = Files.readString(out.resolve("processing-report.html"));
    check(
        html.contains("test&lt;&amp;&gt;") && !html.contains("test<&>"),
        "Report escapes user text");
    check(html.contains("data:image/png;base64,"), "DSM embedded without external assets");
    check(html.contains("All processing parameter values"), "Full parameter table visible");
    o.reportDsmMinPoints = 2;
    var limited =
        DsmRaster.create(
            run, out, o, ProcessingReport.map(run.resolve("07_results/coordinates.json")));
    check(
        ((Number) limited.get("nodata_cells")).intValue() == 1,
        "Minimum point count produces nodata");
    var noFrame = DsmRaster.create(run, out, o, new LinkedHashMap<>());
    check(
        "unavailable".equals(noFrame.get("status")),
        "Local object DSM not mislabeled georeferenced");
    o.reportDsmResolution = .000001;
    boolean rejected = false;
    try {
      DsmRaster.create(
          run, out, o, ProcessingReport.map(run.resolve("07_results/coordinates.json")));
    } catch (IOException expected) {
      rejected = true;
    }
    check(rejected, "Explicit overly fine DSM resolution is rejected, not silently changed");
    Options parsed =
        Options.parse(new String[] {"--export-report", run.toString(), "--similarity-window", "7"});
    check(
        parsed.census == 7 && parsed.exportReport != null,
        "Standalone export and similarity alias parsed");
  }
}
