package sgm;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.*;
import java.nio.*;
import java.nio.file.*;
import java.util.*;
import java.util.List;
import javax.imageio.ImageIO;
import org.apache.pdfbox.Loader;
import org.apache.pdfbox.pdmodel.PDDocument;
import org.apache.pdfbox.rendering.PDFRenderer;
import org.apache.pdfbox.text.PDFTextStripper;

/** Independent projection/scale, missing-source, font, pagination and render controls. */
final class PdfReportRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    if (args.length > 0) render(Path.of(args[0]), Path.of(args[1]));
    else {
      Path root = Files.createTempDirectory(Path.of("target"), "pdf-report-");
      System.out.println("PASS PDF report: " + run(root) + "; " + root);
    }
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Map<String, Object> control =
        Json.object(
            "tasks",
            List.of(
                Json.object(
                    "task",
                    "Stage 05_stereo - test",
                    "started_offset_seconds",
                    1.5,
                    "wall_seconds",
                    1.0)),
            "resource_timeline",
            List.of(
                Json.object("seconds", 1, "cpu_percent", 99, "rss_bytes", 9999),
                Json.object(
                    "seconds",
                    2,
                    "cpu_percent",
                    20,
                    "rss_bytes",
                    1234,
                    "tracked_opencl_bytes",
                    512),
                Json.object("seconds", 3, "cpu_percent", 88, "rss_bytes", 8888)));
    var usage = ResourceTimeline.stageUsage(control, "05_stereo");
    check(
        ((Number) usage.get("mean_sampled_cpu_percent")).doubleValue() == 20,
        "stage CPU uses monotonic interval samples");
    check(
        ((Number) usage.get("peak_rss_bytes")).longValue() == 1234,
        "stage RAM excludes neighboring stages");
    check(
        usage.get("mean_sampled_device_gpu_percent") == null,
        "missing GPU measurement not invented");
    check(
        ResourceTimeline.stageUsage(control, "04_geometry").isEmpty(),
        "missing stage resource intervals unavailable");
    for (double scale : new double[] {1, 2}) {
      Path run = fixture(root.resolve("pdf-" + (int) scale), scale), out = run.resolve("report");
      Options o = TestSupport.options();
      o.reportLocalDsm = true;
      o.reportDsmResolution = .25 * scale;
      // The default survey report: summary, survey data, elevation model, processing parameters.
      Path summaryOut = run.resolve("summary");
      ProcessingReport.generate(o, run, summaryOut);
      int surveyPages;
      try (PDDocument summary = Loader.loadPDF(summaryOut.resolve("processing-report.pdf").toFile())) {
        surveyPages = summary.getNumberOfPages();
        check(surveyPages >= 3 && surveyPages <= 8, "survey report is a few pages: " + surveyPages);
        String text = new PDFTextStripper().getText(summary);
        for (String section : List.of("Summary", "Quality flags", "Survey Data", "Digital Elevation Model", "Processing Parameters"))
          check(text.contains(section), "survey report section " + section);
        check(!text.contains("Ground Control Points"), "no GCP section without ground control");
      }
      o.reportDetail = "full";
      ProcessingReport.generate(o, run, out);
      Path file = out.resolve("processing-report.pdf");
      check(Files.size(file) > 10000, "real PDF generated");
      try (PDDocument document = Loader.loadPDF(file.toFile())) {
        check(document.getNumberOfPages() > surveyPages, "full report appends the detailed pages");
        String text = new PDFTextStripper().getText(document);
        check(
            text.contains("Orthomosaic overview") && text.contains("Geometric image overlap"),
            "overview and coverage sections searchable");
        check(
            text.contains("not reconstructed depth support") && text.contains("Local"),
            "coverage and coordinate scope explicit");
        for (var page : document.getPages())
          for (var name : page.getResources().getFontNames())
            check(
                page.getResources().getFont(name).isEmbedded(),
                "fonts embedded for portable output");
        BufferedImage preview = new PDFRenderer(document).renderImageWithDPI(0, 72);
        check(preview.getWidth() > 500 && preview.getHeight() > 700, "PDF renders");
        preview.flush();
      }
      var maps = ProcessingReport.map(out.resolve("maps.json"));
      check(
          "2020-08-09T08:11:12Z".equals(maps.get("capture_first_known")),
          "source acquisition date is normalized independently of processing date");
      check("complete".equals(maps.get("status")), "orthomosaic has usable photographed surface");
      check(
          !Boolean.TRUE.equals(maps.get("north_established")), "local frame does not invent north");
      check(
          !Boolean.TRUE.equals(maps.get("acquisition_order_established")),
          "missing capture times do not invent flight order");
      BufferedImage ortho = ImageIO.read(out.resolve("orthomosaic.png").toFile());
      int rgb = ortho.getRGB(ortho.getWidth() / 2, ortho.getHeight() / 2);
      check(
          Math.abs(((rgb >> 16) & 255) - 126) <= 5 && Math.abs(((rgb >> 8) & 255) - 126) <= 5,
          "known Brown projection preserves source colors under output scaling");
      ortho.flush();
      Files.move(run.resolve("source.png"), run.resolve("source-unavailable.png"));
      ProcessingReport.generate(o, run, out);
      check(
          !Files.exists(out.resolve("orthomosaic.png")),
          "missing source cannot reuse a stale orthomosaic");
      try (PDDocument document = Loader.loadPDF(file.toFile())) {
        check(
            new PDFTextStripper().getText(document).contains("Orthomosaic unavailable"),
            "missing imagery explicitly reported");
      }
      Properties properties = new Properties();
      try (var in = Files.newBufferedReader(run.resolve("logs/run-settings.properties"))) {
        properties.load(in);
      }
      check(
          Double.parseDouble(properties.getProperty("scale")) == scale,
          "report regeneration retains archived processing parameters");
      o.processingReport = "html";
      ProcessingReport.generate(o, run, out);
      check(
          Files.isRegularFile(out.resolve("processing-report.html")),
          "HTML-only report still supported");
      check(!Files.exists(file), "HTML regeneration cannot leave a stale PDF for different inputs");
    }
    return checks;
  }

  private static Path fixture(Path run, double scale) throws Exception {
    Files.createDirectories(run.resolve("00_inputs"));
    Files.createDirectories(run.resolve("04_geometry"));
    Files.createDirectories(run.resolve("07_results"));
    Files.createDirectories(run.resolve("logs"));
    Json.write(
        run.resolve("00_inputs/acquisition.json"),
        List.of(
            Json.object(
                "filename",
                "source.png",
                "capture_time_local",
                "2020:08:09 10:11:12",
                "capture_offset",
                "+02:00",
                "camera_model",
                "control")));
    BufferedImage image = new BufferedImage(64, 64, BufferedImage.TYPE_INT_RGB);
    for (int y = 0; y < 64; y++)
      for (int x = 0; x < 64; x++) image.setRGB(x, y, (x * 4 << 16) | (y * 4 << 8) | 80);
    Path photo = run.resolve("source.png");
    ImageIO.write(image, "png", photo.toFile());
    image.flush();
    Files.writeString(
        run.resolve("00_inputs/images.csv"),
        "image_id,filename,original_path,bytes,sha256,original_width,original_height,gps_latitude_deg,gps_longitude_deg,gps_altitude_metres,archived_original\n"
            + "0,source.png,"
            + PoseIO.quote(photo.toAbsolutePath().toString())
            + ","
            + Files.size(photo)
            + ","
            + ReuseCache.hash(photo)
            + ",64,64,,,,\n");
    Calibration calibration = new Calibration(40, 40, 31.5, 31.5, new double[5], 64, 64);
    Camera camera = new Camera(40, 40, 31.5, 31.5);
    camera.rotation = new double[] {1, 0, 0, 0, -1, 0, 0, 0, -1};
    camera.translation = new double[] {-2, 2, 8};
    camera.registered = true;
    OrientationIO.Model model = new OrientationIO.Model();
    model.add(new OrientationIO.View("source.png", calibration, camera));
    Json.write(
        run.resolve("04_geometry/orientation_model.json"),
        Json.object("source", OrientationIO.json(model), "processed", OrientationIO.json(model)));
    Json.write(
        run.resolve("run.json"),
        Json.object(
            "run_id",
            "known-projection-control",
            "status",
            "complete",
            "started_utc",
            "2026-11-01T00:00:00Z",
            "updated_utc",
            "2026-11-01T00:00:02Z",
            "stages",
            Json.object(
                "00_inputs",
                Json.object("status", "complete", "elapsed_seconds", .5),
                "05_stereo",
                Json.object("status", "complete", "elapsed_seconds", 1.5))));
    Files.writeString(
        run.resolve("logs/run-settings.properties"),
        "scale=" + scale + "\nprocessingReport=none\nquality=custom\n");
    Json.write(
        run.resolve("07_results/coordinates.json"),
        Json.object("georeferenced", false, "xyz_units", "local output units"));
    String header =
        "ply\n"
            + "format binary_little_endian 1.0\n"
            + "element vertex 256\n"
            + "property double x\n"
            + "property double y\n"
            + "property double z\n"
            + "end_header\n";
    try (OutputStream out = Files.newOutputStream(run.resolve("07_results/dense.ply"))) {
      out.write(header.getBytes(java.nio.charset.StandardCharsets.US_ASCII));
      ByteBuffer point = ByteBuffer.allocate(24).order(ByteOrder.LITTLE_ENDIAN);
      for (int y = 0; y < 16; y++)
        for (int x = 0; x < 16; x++) {
          point.clear();
          point
              .putDouble((x * .25 + .125) * scale)
              .putDouble((y * .25 + .125) * scale)
              .putDouble(0);
          out.write(point.array());
        }
    }
    return run;
  }

  static void render(Path file, Path output) throws Exception {
    Files.createDirectories(output);
    try (PDDocument document = Loader.loadPDF(file.toFile())) {
      PDFRenderer renderer = new PDFRenderer(document);
      int columns = 3, width = 397, height = 562;
      BufferedImage sheet =
          new BufferedImage(
              columns * width,
              ((document.getNumberOfPages() + columns - 1) / columns) * height,
              BufferedImage.TYPE_INT_RGB);
      Graphics2D g = sheet.createGraphics();
      g.setColor(Color.LIGHT_GRAY);
      g.fillRect(0, 0, sheet.getWidth(), sheet.getHeight());
      for (int i = 0; i < document.getNumberOfPages(); i++) {
        BufferedImage page = renderer.renderImageWithDPI(i, 105);
        ImageIO.write(
            page,
            "png",
            output.resolve(String.format(Locale.ROOT, "page-%02d.png", i + 1)).toFile());
        g.drawImage(page, (i % columns) * width, (i / columns) * height, width, height, null);
        page.flush();
      }
      g.dispose();
      ImageIO.write(sheet, "png", output.resolve("contact-sheet.png").toFile());
      sheet.flush();
      System.out.println("Rendered " + document.getNumberOfPages() + " pages; " + output);
    }
  }
}
