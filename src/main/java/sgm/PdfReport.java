package sgm;

import java.awt.Color;
import java.io.*;
import java.nio.file.*;
import java.util.*;
import org.apache.pdfbox.pdmodel.*;
import org.apache.pdfbox.pdmodel.common.PDRectangle;
import org.apache.pdfbox.pdmodel.font.*;
import org.apache.pdfbox.pdmodel.graphics.image.*;

/**
 * Paginated, embedded-font PDF report produced directly in Java. The layout follows the usual
 * photogrammetry survey report: title and summary, survey data, camera calibration, camera
 * locations, ground control points, elevation models, processing parameters. {@code
 * --report-detail full} appends the detailed diagnostics (profiles, resources, hardware, every
 * setting).
 */
final class PdfReport implements AutoCloseable {
  /** POSE brand colours (from the logo): navy text, teal accent; headings in a darker teal. */
  private static final Color INK = new Color(4, 48, 88),
      ACCENT = new Color(0, 188, 176),
      TEAL = new Color(0, 122, 115),
      GRAY = new Color(84, 99, 115),
      LIGHT = new Color(224, 243, 241),
      STRIPE = new Color(246, 248, 249),
      RULE = new Color(200, 210, 214),
      ORANGE = new Color(214, 110, 24),
      BLUE = new Color(38, 96, 178),
      RED = new Color(196, 48, 43);
  private static final float LEFT = 44, WIDTH = 507;
  private final PDDocument document = new PDDocument();
  private final PDFont regular, bold;
  private PDPageContentStream content;
  private float y;
  private int page;
  private String runId, section = "Processing report";
  private Map<String, Object> maps;

  private PdfReport(String runId) throws IOException {
    this.runId = runId;
    regular = font("LiberationSans-Regular.ttf");
    bold = font("LiberationSans-Bold.ttf");
    document.getDocumentInformation().setTitle("POSE processing report - " + runId);
    document.getDocumentInformation().setCreator("POSE (Photogrammetric Orientation and Surface Estimation)");
  }

  static final String NAME = "POSE", LONG_NAME = "Photogrammetric Orientation and Surface Estimation", VERSION = "0.1";

  private PDImageXObject logo;
  private boolean logoMissing;

  /** The POSE logo (resource sgm/pose-logo.png), or null when the resource is not on the classpath. */
  private PDImageXObject logo() throws IOException {
    if (logo != null || logoMissing) return logo;
    try (InputStream in = PdfReport.class.getResourceAsStream("/sgm/pose-logo.png")) {
      java.awt.image.BufferedImage image = in == null ? null : javax.imageio.ImageIO.read(in);
      if (image == null) logoMissing = true;
      else logo = LosslessFactory.createFromImage(document, image);
    }
    return logo;
  }

  /** Draws the logo with its top-left at (x, top), the given height; text "POSE" without it. */
  private float drawLogo(float x, float top, float height) throws IOException {
    PDImageXObject image = logo();
    if (image == null) {
      text(NAME, x, top - height * .8f, height, bold, INK);
      return bold.getStringWidth(NAME) * height / 1000;
    }
    float width = height * image.getWidth() / image.getHeight();
    content.drawImage(image, x, top - height, width, height);
    return width;
  }

  private PDFont font(String name) throws IOException {
    InputStream stream =
        PDDocument.class.getResourceAsStream("/org/apache/pdfbox/resources/ttf/" + name);
    if (stream == null && name.contains("Bold"))
      stream =
          PDDocument.class.getResourceAsStream(
              "/org/apache/pdfbox/resources/ttf/LiberationSans-Regular.ttf");
    if (stream != null)
      try (InputStream input = stream) {
        return PDType0Font.load(document, input);
      }
    for (Path file :
        List.of(
            Path.of("C:/Windows/Fonts", name.contains("Bold") ? "arialbd.ttf" : "arial.ttf"),
            Path.of(
                "/usr/share/fonts/truetype/dejavu",
                name.contains("Bold") ? "DejaVuSans-Bold.ttf" : "DejaVuSans.ttf")))
      if (Files.isRegularFile(file)) return PDType0Font.load(document, file.toFile());
    throw new IOException("No embeddable report font available");
  }

  static void generate(Map<String, Object> data, Path out) throws IOException {
    generate(data, out, "full");
  }

  /**
   * {@code summary} (default): the survey report. {@code full}: the survey report followed by the
   * detailed diagnostics. The HTML and JSON companions always keep every detail.
   */
  static void generate(Map<String, Object> data, Path out, String detail) throws IOException {
    try (PdfReport pdf = new PdfReport(String.valueOf(data.get("run_id")))) {
      pdf.maps = ProcessingReport.object(data.get("maps"));
      Map<String, Object> survey = ProcessingReport.object(data.get("survey"));
      Map<String, Map<String, Object>> stages = new LinkedHashMap<>();
      if (data.get("stage_timings") instanceof List<?> list)
        for (Object item : list) {
          var stage = ProcessingReport.object(item);
          stages.put(String.valueOf(stage.get("stage")), stage);
        }
      pdf.title(data, survey, stages, out);
      pdf.surveyData(data, survey, out);
      pdf.calibration(survey);
      pdf.cameraLocations(survey);
      pdf.groundControl(survey);
      pdf.elevation(data, survey, out);
      pdf.parameters(data, survey, stages);
      if (detail.equals("full")) pdf.appendix(data, out);
      pdf.finish();
      Path file = out.resolve("processing-report.pdf"),
          temporary = out.resolve("processing-report.pdf.tmp");
      documentSave(pdf.document, temporary);
      Files.move(temporary, file, StandardCopyOption.REPLACE_EXISTING);
    }
  }

  // ---------------------------------------------------------------- sections

  private void title(
      Map<String, Object> data, Map<String, Object> survey, Map<String, Map<String, Object>> stages, Path out)
      throws IOException {
    finish();
    page++;
    PDPage p = new PDPage(PDRectangle.A4);
    document.addPage(p);
    content = new PDPageContentStream(document, p);
    drawLogo(LEFT, 802, 46);
    text(LONG_NAME, LEFT, 742, 9.5f, regular, GRAY);
    fill(LEFT, 728, WIDTH, 2, ACCENT);
    text("Processing Report", LEFT, 690, 26, bold, INK);
    String finished = String.valueOf(data.getOrDefault("processing_finished_utc", ""));
    String date = finished.length() >= 16 ? finished.substring(0, 10) + " " + finished.substring(11, 16) + " UTC" : finished;
    text(project(data) + "  |  " + date, LEFT, 668, 11, regular, GRAY);
    text("Run " + data.get("run_id") + "  |  " + data.get("run_status"), LEFT, 653, 9, regular, GRAY);
    y = 638;
    section = "Processing report";
    figure(out.resolve("orthomosaic.png"), 300);
    paragraph(
        "Orthomosaic overview. "
            + pdf(maps.getOrDefault("orthomosaic_note", "Orthomosaic unavailable: calibrated source images and a usable surface are required.")));
    heading("Summary");
    var surface = ProcessingReport.object(data.get("dsm"));
    var telemetry = ProcessingReport.object(data.get("telemetry"));
    List<String[]> facts = new ArrayList<>();
    facts.add(new String[] {"Images / aligned", count(survey.getOrDefault("images", maps.get("input_images"))) + " / " + count(survey.getOrDefault("aligned", maps.get("registered_cameras")))});
    facts.add(new String[] {"Tie points", count(survey.get("tie_points"))});
    facts.add(new String[] {"Dense points", count(surface.get("source_points"))});
    facts.add(new String[] {"Coordinate system", coordinates(ProcessingReport.object(data.get("coordinates")))});
    var gcp = ProcessingReport.object(survey.get("gcp"));
    if (!gcp.isEmpty())
      facts.add(new String[] {"Ground control", gcpLine(gcp)});
    String ground = ground(ProcessingReport.object(data.get("ground")));
    if (ground != null) facts.add(new String[] {"Ground", ground});
    facts.add(new String[] {"Total processing time", telemetry.get("elapsed_wall_seconds") instanceof Number n ? clock(n.doubleValue()) : "Unavailable"});
    for (String[] f : facts) row(f[0], f[1]);
    heading("Quality flags");
    List<String> flags = flags(data, maps, stages);
    if (flags.isEmpty()) paragraph("No issues flagged.");
    for (String flag : flags) paragraph("- " + flag);
  }

  private void surveyData(Map<String, Object> data, Map<String, Object> survey, Path out) throws IOException {
    page("Survey Data");
    String units = String.valueOf(survey.getOrDefault("units", "units"));
    figurePair(out.resolve("camera-map.png"), out.resolve("overlap.png"), 250);
    paragraph("Fig. 1. Camera locations and viewing directions (left) and geometric image overlap (right).");
    paragraph(pdf(maps.getOrDefault("overlap_note", "Coverage unavailable.")));
    List<String[]> grid = new ArrayList<>();
    grid.add(new String[] {"Number of images", count(survey.getOrDefault("images", maps.get("input_images"))), "Camera stations", count(survey.getOrDefault("aligned", maps.get("registered_cameras")))});
    grid.add(new String[] {"Flying altitude", length(survey.get("flying_altitude"), units, 1), "Tie points", count(survey.get("tie_points"))});
    grid.add(new String[] {"Ground resolution", resolution(survey.get("ground_resolution"), units), "Projections", count(survey.get("projections"))});
    grid.add(new String[] {"Coverage area", area(survey.get("coverage_area"), units), "Reprojection error", pixels(survey.get("reprojection_rms_pixels"))});
    facts(grid);
    if (survey.get("processed_ground_resolution") instanceof Number processed)
      paragraph(
          "Ground resolution is for the source images; the dense cloud was matched at "
              + resolution(processed, units)
              + " (processed images). Flying altitude is the mean camera height above the median tie point. Reprojection error is the RMS of all tie point projections in source pixels"
              + (survey.get("reprojection_max_pixels") instanceof Number max ? String.format(Locale.ROOT, " (maximum %.2f px)", max.doubleValue()) : "")
              + ".");
    var camera = ProcessingReport.object(survey.get("camera"));
    if (!camera.isEmpty()) {
      heading("Cameras");
      Object[] res = camera.get("resolution") instanceof Object[] a ? a : camera.get("resolution") instanceof List<?> l ? l.toArray() : null;
      table(
          new String[] {"Camera model", "Resolution", "Focal length", "Pixel size", "Precalibrated"},
          new float[] {150, 90, 90, 90, 87},
          List.<String[]>of(
              new String[] {
                String.valueOf(camera.getOrDefault("model", "Unknown")),
                res == null ? "-" : count(res[0]) + " x " + count(res[1]),
                camera.get("focal_mm") instanceof Number mm ? String.format(Locale.ROOT, "%.2f mm", mm.doubleValue()) : "-",
                camera.get("pixel_size_um") instanceof Number um ? String.format(Locale.ROOT, "%.2f x %.2f um", um.doubleValue(), um.doubleValue()) : "-",
                precalibrated(data)
              }));
      if (camera.get("groups") instanceof Number g && g.intValue() > 1)
        paragraph(g.intValue() + " camera models (calibration groups); see Camera Calibration.");
    }
  }

  private void calibration(Map<String, Object> survey) throws IOException {
    List<?> groups = survey.get("calibrations") instanceof List<?> l ? l : List.of();
    if (groups.isEmpty()) return;
    page("Camera Calibration");
    var residuals = ProcessingReport.object(survey.get("image_residuals"));
    if (!residuals.isEmpty()) {
      residualPlot(residuals);
      paragraph(
          "Fig. 2. Image residuals: mean tie point residual (observed minus predicted) per image cell, all"
              + " images together. A systematic pattern means the lens model does not fit.");
    }
    var gpsLens = ProcessingReport.object(survey.get("gps_intrinsics"));
    int k = 0;
    for (Object item : groups) {
      var c = ProcessingReport.object(item);
      heading(groups.size() > 1 ? "Camera model " + ++k : "Calibration coefficients");
      double w = number(c.get("width")), h = number(c.get("height"));
      double[] d = c.get("distortion") instanceof List<?> dl ? dl.stream().mapToDouble(PdfReport::number).toArray() : new double[5];
      List<String[]> rows = new ArrayList<>();
      rows.add(new String[] {"F", fixed(c.get("fx"), 3), "px", "focal length"});
      if (Math.abs(number(c.get("fy")) - number(c.get("fx"))) > 1e-6) rows.add(new String[] {"Fy", fixed(c.get("fy"), 3), "px", "vertical focal length"});
      rows.add(new String[] {"Cx", fixed(number(c.get("cx")) - (w - 1) / 2, 3), "px", "principal point offset from the image centre"});
      rows.add(new String[] {"Cy", fixed(number(c.get("cy")) - (h - 1) / 2, 3), "px", ""});
      String[] names = {"K1", "K2", "P1", "P2", "K3"};
      for (int i = 0; i < Math.min(5, d.length); i++)
        rows.add(new String[] {names[i], scientific(d[i]), "", i == 0 ? "Brown radial (K) and tangential (P)" : ""});
      table(new String[] {"Parameter", "Value", "Unit", "Note"}, new float[] {80, 120, 50, 257}, rows);
      paragraph(
          String.format(
              Locale.ROOT,
              "Source images %s x %s px.%s",
              count(c.get("width")),
              count(c.get("height")),
              gpsLens.get("refitted") instanceof List<?> r && !r.isEmpty()
                  ? " Refitted in the GPS-weighted adjustment: " + String.join(", ", r.stream().map(String::valueOf).toList())
                      + (gpsLens.get("focal_prior_pixels") instanceof Number prior
                          ? String.format(Locale.ROOT, "; focal prior %.1f px (%s)", prior.doubleValue(), gpsLens.get("focal_prior_source"))
                          : "")
                      + "."
                  : ""));
    }
    var model = ProcessingReport.object(survey.get("gps_model"));
    var shutter = ProcessingReport.object(survey.get("rolling_shutter"));
    if (!model.isEmpty() || !shutter.isEmpty()) {
      heading("GPS and camera timing model");
      List<String[]> rows = new ArrayList<>();
      if (model.get("time_offset_seconds") instanceof Number dt)
        rows.add(new String[] {"GPS time offset", String.format(Locale.ROOT, "%.3f s", dt.doubleValue()), String.valueOf(model.getOrDefault("time_offset_mode", ""))});
      double[] lever = numbers(model.get("lever_arm_camera_metres"));
      if (lever != null && lever.length == 3)
        rows.add(new String[] {"Antenna lever arm (camera x, y, z)", String.format(Locale.ROOT, "%.3f, %.3f, %.3f m", lever[0], lever[1], lever[2]), String.valueOf(model.getOrDefault("lever_arm_mode", ""))});
      if (shutter.get("readout_seconds") instanceof Number rs)
        rows.add(new String[] {"Rolling shutter readout", String.format(Locale.ROOT, "%.2f ms", 1000 * rs.doubleValue()), String.valueOf(shutter.getOrDefault("mode", ""))});
      if (survey.get("gps_position_sigma") instanceof Number s)
        rows.add(new String[] {"GPS accuracy (horizontal / vertical)", String.format(Locale.ROOT, "%.2f / %s m", s.doubleValue(), survey.get("gps_height_sigma") instanceof Number hs ? String.format(Locale.ROOT, "%.2f", hs.doubleValue()) : "-"), "weights"});
      table(new String[] {"Parameter", "Value", "Mode"}, new float[] {200, 180, 127}, rows);
    }
  }

  private void cameraLocations(Map<String, Object> survey) throws IOException {
    var locations = ProcessingReport.object(survey.get("camera_locations"));
    if (locations.isEmpty()) return;
    page("Camera Locations");
    String units = String.valueOf(survey.getOrDefault("units", "units"));
    List<double[]> at = new ArrayList<>(), error = new ArrayList<>();
    List<String> labels = new ArrayList<>();
    List<Object[]> cameras = new ArrayList<>();
    for (Object item : locations.get("cameras") instanceof List<?> l ? l : List.of()) {
      var c = ProcessingReport.object(item);
      double[] xyz = numbers(c.get("position")), e = numbers(c.get("error"));
      if (e == null) continue;
      cameras.add(new Object[] {c.get("image"), e});
      if (xyz != null) {
        at.add(xyz);
        error.add(e);
        labels.add(null);
      }
    }
    errorPlot(at, error, labels, List.of(), units);
    paragraph(
        "Fig. 3. Camera locations and error estimates. Arrows show the horizontal error, colour the"
            + " vertical error. Errors are the estimated antenna position (camera centre with the"
            + " lever arm and time offset) minus the "
            + pdf(locations.get("reference"))
            + ".");
    var rms = ProcessingReport.object(locations.get("rms"));
    heading("Camera location error (RMS)");
    table(
        new String[] {"X error", "Y error", "Z error", "XY error", "Total error"},
        new float[] {101, 101, 101, 102, 102},
        List.<String[]>of(new String[] {length(rms.get("x"), units, 3), length(rms.get("y"), units, 3), length(rms.get("z"), units, 3), length(rms.get("xy"), units, 3), length(rms.get("total"), units, 3)}));
    paragraph("X - easting, Y - northing, Z - altitude.");
    cameras.sort(Comparator.comparingDouble(c -> -norm((double[]) c[1])));
    boolean all = cameras.size() <= 40;
    heading(all ? "Cameras" : "Cameras with the largest errors");
    List<String[]> rows = new ArrayList<>();
    for (Object[] c : cameras.subList(0, Math.min(cameras.size(), all ? 40 : 25))) {
      double[] e = (double[]) c[1];
      rows.add(new String[] {String.valueOf(c[0]), length(e[0], units, 3), length(e[1], units, 3), length(e[2], units, 3), length(norm(e), units, 3)});
    }
    table(new String[] {"Label", "X error", "Y error", "Z error", "Total"}, new float[] {147, 90, 90, 90, 90}, rows);
  }

  private void groundControl(Map<String, Object> survey) throws IOException {
    var gcp = ProcessingReport.object(survey.get("gcp"));
    if (gcp.isEmpty()) return;
    page("Ground Control Points");
    String units = String.valueOf(survey.getOrDefault("units", "units"));
    List<double[]> at = new ArrayList<>(), error = new ArrayList<>();
    List<String> labels = new ArrayList<>();
    List<double[]> check = new ArrayList<>();
    List<Map<String, Object>> points = new ArrayList<>();
    for (Object item : gcp.get("points") instanceof List<?> l ? l : List.of()) {
      var p = ProcessingReport.object(item);
      points.add(p);
      double[] given = numbers(p.get("given")), e = numbers(p.get("error_xyz_metres"));
      if (given == null) continue;
      at.add(given);
      error.add(e == null ? new double[3] : e);
      labels.add(String.valueOf(p.get("id")));
      check.add(new double[] {"check".equals(p.get("type")) ? 1 : 0});
    }
    List<double[]> cameras = new ArrayList<>();
    for (Object c : survey.get("camera_positions") instanceof List<?> l ? l : List.of()) {
      double[] xyz = numbers(c);
      if (xyz != null) cameras.add(xyz);
    }
    gcpPlot(at, error, labels, check, cameras, units);
    paragraph(
        "Fig. 4. GCP locations and error estimates: control points (triangles) and check points"
            + " (circles); arrows show the horizontal error, colour the vertical error. Grey dots are"
            + " the camera stations.");
    paragraph("Datum: " + pdf(gcp.get("datum")) + ". Adjustment: " + pdf(gcp.get("adjustment")) + ".");
    if (gcp.get("gps_offset_to_gcp_datum_metres") instanceof List<?> off && off.size() == 3)
      paragraph(String.format(Locale.ROOT, "Offset of the GPS fixes to the GCP datum: %.3f, %.3f, %.3f m (X, Y, Z).", number(off.get(0)), number(off.get(1)), number(off.get(2))));
    for (String kind : new String[] {"control", "check"}) {
      var rms = ProcessingReport.object(gcp.get(kind + "_rms_metres"));
      if (rms.isEmpty()) continue;
      heading(kind.equals("control") ? "Control points" : "Check points");
      table(
          new String[] {"Count", "X error", "Y error", "Z error", "XY error", "Total"},
          new float[] {57, 90, 90, 90, 90, 90},
          List.<String[]>of(new String[] {count(rms.get("points")), length(rms.get("x"), units, 3), length(rms.get("y"), units, 3), length(rms.get("z"), units, 3), length(rms.get("xy"), units, 3), length(rms.get("total"), units, 3)}));
    }
    heading("Points");
    List<String[]> rows = new ArrayList<>();
    for (var p : points) {
      double[] e = numbers(p.get("error_xyz_metres"));
      rows.add(
          new String[] {
            String.valueOf(p.get("id")),
            String.valueOf(p.get("type")),
            e == null ? "-" : length(e[0], units, 3),
            e == null ? "-" : length(e[1], units, 3),
            e == null ? "-" : length(e[2], units, 3),
            e == null ? "-" : length(norm(e), units, 3),
            count(p.get("pinned_markers")),
            p.get("pixel_rms") instanceof Number px ? String.format(Locale.ROOT, "%.2f px", px.doubleValue()) : "-"
          });
    }
    table(new String[] {"Label", "Type", "X error", "Y error", "Z error", "Total", "Images", "Image error"}, new float[] {70, 52, 65, 65, 65, 65, 50, 75}, rows);
    paragraph(
        "Errors are estimated minus given coordinates. Check points are triangulated from their"
            + " markers with the final cameras and were not used in the adjustment, so their errors"
            + " are an independent accuracy estimate. Image error is the RMS marker residual in source pixels"
            + (gcp.get("marker_pixel_rms") instanceof Number m ? String.format(Locale.ROOT, " (all markers %.2f px)", m.doubleValue()) : "")
            + ".");
  }

  private void elevation(Map<String, Object> data, Map<String, Object> survey, Path out) throws IOException {
    var dsm = ProcessingReport.object(data.get("dsm"));
    page("Digital Elevation Model");
    String units = String.valueOf(survey.getOrDefault("units", "units"));
    figure(out.resolve("dsm.png"), 330);
    paragraph("Fig. 5. Digital surface model (highest point per cell; vegetation and roofs included).");
    List<String[]> grid = new ArrayList<>();
    grid.add(new String[] {"Resolution", dsm.get("cell_size") instanceof Number c ? length(c, units, 3) + "/pix" : "-", "Point density", survey.get("point_density") instanceof Number d ? String.format(Locale.ROOT, "%.1f points/%s", d.doubleValue(), units.equals("m") ? "m2" : "unit2") : "-"});
    grid.add(new String[] {"Elevation range", dsm.get("minimum_elevation") instanceof Number lo && dsm.get("maximum_elevation") instanceof Number hi ? String.format(Locale.ROOT, "%.2f .. %.2f %s", lo.doubleValue(), hi.doubleValue(), units) : "-", "Coverage area", area(survey.get("coverage_area"), units)});
    facts(grid);
    if (dsm.get("height_note") != null) paragraph(pdf(dsm.get("height_note")));
    var g = ProcessingReport.object(data.get("ground"));
    if ("complete".equals(g.get("status"))) {
      ensure(survey.get("dtm_preview") != null ? 300 : 120);
      heading("Ground classification and terrain model");
      if (survey.get("dtm_preview") instanceof String dtm) {
        figure(Path.of(dtm), 230);
        paragraph("Fig. 6. Digital terrain model from the ground points (interpolated under canopy).");
      }
      var checks = ProcessingReport.object(g.get("checks"));
      var islands = ProcessingReport.object(g.get("canopy_islands"));
      List<String[]> rows = new ArrayList<>();
      rows.add(new String[] {"Ground points", count(g.get("ground_points")) + String.format(Locale.ROOT, " (%.1f%%)", 100 * number(g.get("ground_fraction")))});
      rows.add(new String[] {"Low noise points", count(g.get("low_noise_points"))});
      rows.add(new String[] {"Terrain cell", length(g.get("cell_size"), units, 2)});
      rows.add(new String[] {"Terrain cells with observed ground", String.format(Locale.ROOT, "%.0f%%", 100 * number(checks.get("observed_ground_fraction")))});
      rows.add(new String[] {"Split-half difference (median / p95)", length(checks.get("split_half_median_abs_difference"), units, 2) + " / " + length(checks.get("split_half_p95_abs_difference"), units, 2)});
      if (checks.get("blocked_ray_fraction") instanceof Number b) rows.add(new String[] {"Camera rays blocked by the terrain", String.format(Locale.ROOT, "%.2f%%", 100 * b.doubleValue())});
      if (!islands.isEmpty())
        rows.add(new String[] {"Canopy patches rejected", count(islands.get("patches_rejected_as_canopy")) + " of " + count(islands.get("patches_tested")) + " tested (" + String.format(Locale.ROOT, "%.0f", number(islands.get("rejected_area_m2"))) + " m2), median " + length(islands.get("rejected_median_rise_m"), units, 1) + " above the terrain"});
      table(new String[] {"Ground", "Value"}, new float[] {230, 277}, rows);
      paragraph("LAS classes: 2 ground, 7 low noise, 1 other. Rasters: dtm.tif, chm.tif, ground_support.tif, ground_uncertainty.tif.");
    }
  }

  /** Per-step settings that matter (CLI names), stage times and software. */
  private static final String[][][] STEP_OPTIONS = {
    {{"inputs"}, {"quality", "--quality"}, {"maxDimension", "--max-dimension"}, {"calibration", "--calibration"}, {"focal", "--focal"}},
    {{"calibrate"}, {"calibrationPipeline", "--calibration-pipeline"}, {"selfCalibration", "self calibration"}, {"intrinsicsMode", "--intrinsics-mode"}, {"refineIntrinsics", "--refine-intrinsics"}},
    {{"match"}, {"matchingAlgorithm", "--matching-algorithm"}, {"features", "--features"}, {"ratio", "--ratio"}, {"minMatches", "--min-matches"}, {"pairCandidates", "--pair-candidates"}},
    {{"align"}, {"iterations", "--bundle-iterations"}, {"reprojection", "--reprojection"}, {"trackMinViews", "--track-min-views"}, {"trackMaxError", "--track-max-error"}, {"cameraScreening", "--camera-screening"}},
    {{"georeference"}, {"crs", "--crs"}, {"gpsBundle", "--gps-bundle"}, {"gpsSigma", "--gps-sigma"}, {"gpsHeightSigma", "--gps-height-sigma"}, {"gpsIntrinsics", "--gps-intrinsics"}, {"gpsFocalPrior", "--gps-focal-prior"}, {"gpsTimeOffset", "--gps-time-offset"}, {"gpsLeverArm", "--gps-lever-arm"}, {"rollingShutter", "--rolling-shutter"}},
    {{"gcp"}, {"gcp", "--gcp"}, {"gcpSigma", "--gcp-sigma"}, {"gcpHeightSigma", "--gcp-height-sigma"}},
    {{"dense"}, {"denseMethod", "--dense-method"}, {"disparities", "--disparities"}, {"correlation", "--min-correlation"}, {"minViews", "--min-views"}, {"filtering", "--filtering"}, {"fusionMode", "--fusion-mode"}},
    {{"ground"}, {"classifyGround", "--classify-ground"}, {"groundCell", "--ground-cell"}, {"groundMaxSlope", "--ground-max-slope"}, {"groundMaxObject", "--ground-max-object"}, {"groundPairCorrection", "--ground-pair-correction"}, {"groundAwareFusion", "--ground-aware-fusion"}},
    {{"finish"}, {"processingReport", "--report"}, {"reportDetail", "--report-detail"}, {"scale", "--scale"}, {"outputCompression", "--output-compression"}},
  };

  private static final Map<String, String[]> STEP_STAGES =
      Map.of(
          "inputs", new String[] {"00_inputs"},
          "calibrate", new String[] {"01_calibration"},
          "match", new String[] {"02_features", "03_matches"},
          "dense", new String[] {"05_stereo", "06_fusion"},
          "finish", new String[] {"07_results"});

  private void parameters(Map<String, Object> data, Map<String, Object> survey, Map<String, Map<String, Object>> stages)
      throws IOException {
    page("Processing Parameters");
    var parameters = ProcessingReport.object(data.get("parameters"));
    var telemetry = ProcessingReport.object(data.get("telemetry"));
    Map<String, String> status = new HashMap<>();
    for (Object item : survey.get("steps") instanceof List<?> l ? l : List.of()) {
      var s = ProcessingReport.object(item);
      status.put(String.valueOf(s.get("step")), String.valueOf(s.get("status")));
    }
    heading("General");
    List<String[]> general = new ArrayList<>();
    general.add(new String[] {"Cameras / aligned", count(survey.getOrDefault("images", maps.get("input_images"))) + " / " + count(survey.getOrDefault("aligned", maps.get("registered_cameras")))});
    general.add(new String[] {"Coordinate system", coordinates(ProcessingReport.object(data.get("coordinates")))});
    general.add(new String[] {"Arguments", String.join(" ", (data.get("arguments") instanceof List<?> a ? a : List.of()).stream().map(String::valueOf).toList())});
    table(new String[] {"Setting", "Value"}, new float[] {150, 357}, general);
    paragraph(
        "Reprojection, GPS and confidence figures describe internal consistency; only check points"
            + " measure absolute accuracy.");
    heading("Steps");
    List<String[]> rows = new ArrayList<>();
    for (String[][] step : STEP_OPTIONS) {
      String name = step[0][0];
      List<String> values = new ArrayList<>();
      boolean skipped = "skipped".equals(status.get(name));
      for (int i = 1; i < (skipped ? Math.min(2, step.length) : step.length); i++) {
        Object v = parameters.get(step[i][0]);
        if (v == null || String.valueOf(v).equals("null") || String.valueOf(v).equals("NaN")) continue;
        if (step[i][0].equals("iterations") && "false".equals(String.valueOf(parameters.get("bundleIterationsExplicit"))))
          v = "until converged";
        if (step[i][0].equals("gcp") && name.equals("gcp")) v = Path.of(String.valueOf(v)).getFileName();
        values.add(step[i][1] + " " + setting(v));
      }
      String time = "";
      double seconds = 0;
      boolean timed = false;
      for (String stage : STEP_STAGES.getOrDefault(name, new String[0]))
        if (stages.get(stage) != null && stages.get(stage).get("elapsed_seconds") instanceof Number n) {
          seconds += n.doubleValue();
          timed = true;
        }
      if (timed) time = clock(seconds);
      rows.add(new String[] {name, status.getOrDefault(name, "-"), time, String.join("; ", values)});
    }
    table(new String[] {"Step", "Status", "Time", "Main settings"}, new float[] {72, 60, 60, 315}, rows);
    if (stages.get("04_geometry") != null && stages.get("04_geometry").get("elapsed_seconds") instanceof Number geometry)
      paragraph("Alignment, georeferencing and ground control together (04_geometry): " + clock(geometry.doubleValue()) + ". All settings: processing-report.json, or --report-detail full.");
    heading("Software and hardware");
    var hardware = ProcessingReport.object(telemetry.get("hardware"));
    var device = ProcessingReport.object(telemetry.get("selected_opencl_device"));
    List<String[]> software = new ArrayList<>();
    software.add(new String[] {"Software", NAME + " " + VERSION + " (" + LONG_NAME + ")"});
    if (hardware.get("java_version") != null) software.add(new String[] {"Java", pdf(hardware.get("java_version"))});
    if (hardware.get("os") != null) software.add(new String[] {"Platform", pdf(hardware.get("os")) + " " + pdf(hardware.getOrDefault("architecture", "")) + ", " + count(hardware.get("available_processors")) + " processors"});
    if (device.get("name") != null) software.add(new String[] {"OpenCL device", pdf(device.get("name"))});
    if (telemetry.get("peak_sampled_process_resident_bytes") instanceof Number ram) software.add(new String[] {"Peak memory", String.format(Locale.ROOT, "%,.0f MiB", ram.doubleValue() / 1048576.)});
    software.add(new String[] {"Total processing time", telemetry.get("elapsed_wall_seconds") instanceof Number n ? clock(n.doubleValue()) : "Unavailable"});
    table(new String[] {"Item", "Value"}, new float[] {150, 357}, software);
  }

  /** The detailed diagnostics of --report-detail full. */
  private void appendix(Map<String, Object> data, Path out) throws IOException {
    page("Appendix: acquisition");
    heading("Geometric image overlap");
    figure(out.resolve("overlap.png"), 300);
    paragraph(pdf(maps.getOrDefault("overlap_note", "Coverage unavailable.")));
    paragraph(pdf(maps.getOrDefault("camera_note", "Camera geometry unavailable.")));
    row(
        "Acquisition (known times)",
        maps.get("capture_first_known") == null
            ? "Unavailable"
            : brief(maps.get("capture_first_known")) + " to " + brief(maps.get("capture_last_known")) + "; " + brief(maps.get("capture_time_basis")));
    if (maps.containsKey("cell_size")) row("Report ortho resolution", brief(maps.get("cell_size")) + " output units / pixel");
    page("Appendix: calibration and reconstruction");
    Map<String, Object> camera = ProcessingReport.object(data.get("camera_orientations"));
    var source = ProcessingReport.object(camera.get("source"));
    if (source.get("views") instanceof List<?> cameras) {
      heading("Registered image IDs");
      int id = 0;
      for (Object item : cameras) row(Integer.toString(id++), String.valueOf(ProcessingReport.object(item).get("image")));
    }
    List<String> keys =
        List.of(
            "method", "model", "solver", "accepted", "reason", "confidence_model", "threshold",
            "sampled_candidates", "registered_views", "training_tracks", "held_out_tracks",
            "calibration", "rejected_below_threshold");
    for (var entry : ProcessingReport.object(data.get("profiles")).entrySet()) {
      Map<String, Object> profile = ProcessingReport.object(entry.getValue());
      if (keys.stream().noneMatch(profile::containsKey)) continue;
      String k = entry.getKey();
      heading(
          k.contains("shared_calibration") ? "Shared camera calibration"
              : k.contains("patchmatch") ? "Dense matching"
              : k.contains("confidence") ? "Point confidence"
              : k.contains("refinement") ? "Calibration bootstrap"
              : k.contains("solver") ? "Bundle solver"
              : k.contains("surface") ? "Surface consolidation"
              : "Depth fusion");
      for (String key : keys) if (profile.containsKey(key)) row(key.replace('_', ' '), brief(profile.get(key)));
    }
    page("Appendix: processing time and resources");
    paragraph(
        "Nested and overlapping task times are not additive. CPU is measured for this process;"
            + " hardware GPU load, when available, is device-wide. Configured budgets are not"
            + " measured usage.");
    paragraph(brief(data.get("measurement_cutoff")));
    if (data.get("stage_timings") instanceof List<?> list)
      for (Object item : list) {
        var stage = ProcessingReport.object(item);
        var resources = ProcessingReport.object(stage.get("sampled_resources"));
        String usage = "";
        if (resources.get("samples") instanceof Number count && count.intValue() > 0)
          usage =
              "; sampled CPU " + percentage(resources.get("mean_sampled_cpu_percent"))
                  + ", GPU " + percentage(resources.get("mean_sampled_device_gpu_percent"))
                  + ", peak process RAM "
                  + (resources.get("peak_rss_bytes") instanceof Number ram ? String.format(Locale.ROOT, "%,.0f MiB", ram.doubleValue() / 1048576.) : "unavailable");
        row(String.valueOf(stage.get("stage")), stage.get("status") + " | " + seconds(stage.get("elapsed_seconds")) + usage);
      }
    var telemetry = ProcessingReport.object(data.get("telemetry"));
    for (String key :
        List.of(
            "elapsed_wall_seconds", "process_cpu_seconds_since_launch", "peak_sampled_heap_used_bytes",
            "peak_sampled_process_resident_bytes", "peak_tracked_opencl_buffer_bytes",
            "peak_device_gpu_utilization_percent", "peak_device_vram_used_bytes", "gpu_utilization_scope"))
      row(key.replace('_', ' '), key.endsWith("_bytes") && telemetry.get(key) instanceof Number n ? String.format(Locale.ROOT, "%,.1f MiB", n.doubleValue() / 1048576.) : brief(telemetry.get(key)));
    figure(out.resolve("resource-timeline.png"), 220);
    page("Appendix: hardware and compute backends");
    for (var entry : ProcessingReport.object(telemetry.get("hardware")).entrySet()) row(entry.getKey().replace('_', ' '), brief(entry.getValue()));
    var device = ProcessingReport.object(telemetry.get("selected_opencl_device"));
    if (device.isEmpty()) row("OpenCL device", "Unavailable");
    for (String key : List.of("name", "driver", "global_memory_bytes", "maximum_allocation_bytes", "effective_working_budget_bytes", "scope", "unavailable"))
      if (device.containsKey(key))
        row(key.replace('_', ' '), key.endsWith("_bytes") && device.get(key) instanceof Number n ? String.format(Locale.ROOT, "%,.1f MiB", n.doubleValue() / 1048576.) : brief(device.get(key)));
    if (!ProcessingReport.object(telemetry.get("opencl_profile_totals")).isEmpty()) heading("Measured OpenCL work and transfers");
    for (var entry : ProcessingReport.object(telemetry.get("opencl_profile_totals")).entrySet())
      row(entry.getKey(), entry.getValue() instanceof Number n && entry.getKey().endsWith("nanoseconds") ? String.format(Locale.ROOT, "%.3f s", n.doubleValue() / 1e9) : brief(entry.getValue()));
    paragraph(String.valueOf(telemetry.getOrDefault("timeline_scope", "Historical timeline unavailable.")));
    page("Appendix: effective processing settings");
    paragraph("Values below come from the archived run. Report regeneration settings are stored separately in processing-report.json.");
    for (var entry : new TreeMap<>(ProcessingReport.object(data.get("parameters"))).entrySet()) row(entry.getKey(), brief(entry.getValue()));
  }

  // ---------------------------------------------------------------- flags and one-liners

  /** Actionable findings only: incomplete stages, unregistered images, weak georeferencing. */
  static List<String> flags(
      Map<String, Object> data, Map<String, Object> maps, Map<String, Map<String, Object>> stages) {
    List<String> flags = new ArrayList<>();
    for (var e : stages.entrySet()) {
      String status = String.valueOf(e.getValue().get("status"));
      if (!status.equals("complete") && !status.equals("skipped"))
        flags.add(e.getKey() + " " + status + ": " + e.getValue().get("detail"));
    }
    if (maps.get("input_images") instanceof Number in
        && maps.get("registered_cameras") instanceof Number reg
        && reg.intValue() < in.intValue())
      flags.add((in.intValue() - reg.intValue()) + " of " + in.intValue() + " images were not registered.");
    var geometry = stages.get("04_geometry");
    if (geometry != null) {
      var m = java.util.regex.Pattern.compile("RMS=([0-9.]+) m").matcher(String.valueOf(geometry.get("detail")));
      if (m.find() && Double.parseDouble(m.group(1)) > 2)
        flags.add("GPS camera fit RMS " + m.group(1) + " m: georeferencing may be weak (consumer GPS, timing offset or bent block).");
    }
    var survey = ProcessingReport.object(data.get("survey"));
    var gcp = ProcessingReport.object(survey.get("gcp"));
    var checkRms = ProcessingReport.object(gcp.get("check_rms_metres"));
    var controlRms = ProcessingReport.object(gcp.get("control_rms_metres"));
    if (!gcp.isEmpty() && checkRms.isEmpty())
      flags.add("No check points: the ground control residuals do not measure accuracy independently. Mark some GCPs as type 'check'.");
    // Check points within a few matching pixels on the ground are normal; well beyond is not.
    double gsd = number(survey.get("processed_ground_resolution"));
    if (checkRms.get("total") instanceof Number c && controlRms.get("total") instanceof Number k) {
      double expected = Math.max(k.doubleValue(), Double.isFinite(gsd) ? gsd : .02);
      if (c.doubleValue() > 3 * expected)
        flags.add(String.format(Locale.ROOT, "Check point RMS %.3f m is over three times the control RMS or the ground pixel (%.3f m): the block may bend between the control points, or a marker is misplaced.", c.doubleValue(), expected));
    }
    if (gcp.get("marker_pixel_rms") instanceof Number px && px.doubleValue() > 2)
      flags.add(String.format(Locale.ROOT, "GCP marker RMS %.1f px: check the pinned markers (gcp-edit).", px.doubleValue()));
    var screening = ProcessingReport.object(data.get("camera_screening"));
    if (screening.get("flagged") instanceof Number flagged && flagged.intValue() > 0) {
      List<String> named = new ArrayList<>();
      if (screening.get("cameras") instanceof List<?> cameras)
        for (Object item : cameras) {
          var camera = ProcessingReport.object(item);
          if (camera.get("flags") instanceof List<?> f && !f.isEmpty())
            named.add(camera.get("image") + " (" + String.join(", ", f.stream().map(String::valueOf).toList()) + ")");
        }
      flags.add(
          flagged.intValue()
              + " cameras flagged as possibly poorly oriented"
              + ("exclude".equals(screening.get("mode")) ? " and excluded from dense matching" : "")
              + ": "
              + String.join("; ", named.subList(0, Math.min(12, named.size())))
              + (named.size() > 12 ? "; ..." : "")
              + ". See 04_geometry/camera_screening.json.");
    }
    var stereo = stages.get("05_stereo");
    if (stereo != null && String.valueOf(stereo.get("detail")).toLowerCase(Locale.ROOT).contains("skipped"))
      flags.add("Some stereo pairs were skipped: see 05_stereo/skipped_pairs.json.");
    var checks = ProcessingReport.object(ProcessingReport.object(data.get("ground")).get("checks"));
    if (checks.get("observed_ground_fraction") instanceof Number seen && seen.doubleValue() < .1)
      flags.add(
          String.format(
              Locale.ROOT,
              "Ground observed in only %.0f%% of terrain cells; the DTM is mostly interpolated (see"
                  + " ground_support.tif).",
              100 * seen.doubleValue()));
    if (checks.get("split_half_p95_abs_difference") instanceof Number p95 && p95.doubleValue() > 1)
      flags.add(
          String.format(
              Locale.ROOT,
              "Ground from independent halves of the stereo pairs disagrees by up to %.2f m (95th"
                  + " percentile); treat the DTM as uncertain there.",
              p95.doubleValue()));
    if (checks.get("blocked_ray_fraction") instanceof Number blocked && blocked.doubleValue() > .05)
      flags.add(
          String.format(
              Locale.ROOT,
              "%.1f%% of camera rays pass below the DTM; it is too high in places (cells coded 3 in"
                  + " ground_support.tif).",
              100 * blocked.doubleValue()));
    return flags;
  }

  private static String gcpLine(Map<String, Object> gcp) {
    var control = ProcessingReport.object(gcp.get("control_rms_metres"));
    var check = ProcessingReport.object(gcp.get("check_rms_metres"));
    StringBuilder s = new StringBuilder();
    s.append(count(control.getOrDefault("points", 0))).append(" control");
    if (control.get("total") instanceof Number c) s.append(String.format(Locale.ROOT, " (RMS %.3f m)", c.doubleValue()));
    s.append(", ").append(count(check.getOrDefault("points", 0))).append(" check points");
    if (check.get("total") instanceof Number c) s.append(String.format(Locale.ROOT, " (RMS %.3f m)", c.doubleValue()));
    return s.toString();
  }

  private static String project(Map<String, Object> data) {
    if (data.get("arguments") instanceof List<?> a)
      for (int i = 0; i + 1 < a.size(); i++)
        if (a.get(i).equals("-o") || a.get(i).equals("--output")) {
          Path p = Path.of(String.valueOf(a.get(i + 1))).getFileName();
          if (p != null) return p.toString();
        }
    return "Project";
  }

  private static String precalibrated(Map<String, Object> data) {
    var p = ProcessingReport.object(data.get("parameters"));
    Object c = p.get("calibration");
    return c == null || String.valueOf(c).equals("null") ? "No" : "Yes (" + p.getOrDefault("intrinsicsMode", "fixed") + ")";
  }

  /** One line: ground share, observed terrain share, split-half agreement. */
  private static String ground(Map<String, Object> g) {
    if (!"complete".equals(g.get("status"))) return g.isEmpty() ? null : "Not classified: " + g.get("reason");
    var checks = ProcessingReport.object(g.get("checks"));
    return String.format(
        Locale.ROOT,
        "%.1f%% of points; ground observed in %.0f%% of cells, rest interpolated; split-half median"
            + " %.2f m",
        100 * number(g.get("ground_fraction")),
        100 * number(checks.get("observed_ground_fraction")),
        number(checks.get("split_half_median_abs_difference")));
  }

  private static double number(Object value) {
    return value instanceof Number n ? n.doubleValue() : Double.NaN;
  }

  private static double[] numbers(Object value) {
    if (value instanceof double[] d) return d;
    if (!(value instanceof List<?> l)) return null;
    double[] v = new double[l.size()];
    for (int i = 0; i < v.length; i++) v[i] = number(l.get(i));
    return v;
  }

  private static double norm(double[] v) {
    double s = 0;
    for (double x : v) s += x * x;
    return Math.sqrt(s);
  }

  /** One readable line: CRS and units, GPS fit, height reference. */
  private static String coordinates(Map<String, Object> c) {
    if (c.isEmpty()) return "Unavailable";
    if (!Boolean.TRUE.equals(c.get("georeferenced")))
      return "Local reconstruction frame (not georeferenced)";
    StringBuilder s = new StringBuilder(String.valueOf(c.getOrDefault("horizontal_crs", "projected CRS")));
    if (c.get("xyz_units") != null) s.append(", ").append(c.get("xyz_units"));
    if (c.get("datum") != null) s.append("; datum from ").append(c.get("datum"));
    if (c.get("inlier_rms_metres") instanceof Number rms)
      s.append(String.format(Locale.ROOT, "; GPS camera fit RMS %.2f m", rms.doubleValue()));
    if (c.get("gps_inliers") instanceof Number in && c.get("gps_controls") instanceof Number all)
      s.append(String.format(Locale.ROOT, " (%d/%d cameras)", in.intValue(), all.intValue()));
    if (c.get("vertical_crs") == null)
      s.append(c.get("datum") != null ? "; heights from the GCPs" : "; heights: GPS altitude, vertical datum unspecified");
    return s.toString();
  }

  private static String count(Object v) {
    return v instanceof Number n ? String.format(Locale.ROOT, "%,d", n.longValue()) : v == null ? "-" : String.valueOf(v);
  }

  private static String fixed(Object v, int decimals) {
    return v instanceof Number n && Double.isFinite(n.doubleValue()) ? String.format(Locale.ROOT, "%." + decimals + "f", n.doubleValue()) : "-";
  }

  private static String scientific(double v) {
    return Double.isFinite(v) ? String.format(Locale.ROOT, "%.6g", v) : "-";
  }

  private static String length(Object v, String units, int decimals) {
    if (!(v instanceof Number n) || !Double.isFinite(n.doubleValue())) return "-";
    double d = n.doubleValue();
    if (Math.abs(d) < .5 * Math.pow(10, -decimals)) d = 0; // no "-0.000"
    return String.format(Locale.ROOT, "%." + decimals + "f %s", d, units);
  }

  /** A setting value: integral doubles without ".0". */
  private static String setting(Object v) {
    if (v instanceof Number n && n.doubleValue() == Math.rint(n.doubleValue()) && Math.abs(n.doubleValue()) < 1e15)
      return Long.toString((long) n.doubleValue());
    return String.valueOf(v);
  }

  private static String resolution(Object v, String units) {
    if (!(v instanceof Number n) || !Double.isFinite(n.doubleValue())) return "-";
    return units.equals("m")
        ? String.format(Locale.ROOT, "%.2f cm/pix", 100 * n.doubleValue())
        : String.format(Locale.ROOT, "%.4g units/pix", n.doubleValue());
  }

  private static String area(Object v, String units) {
    if (!(v instanceof Number n) || !Double.isFinite(n.doubleValue())) return "-";
    double a = n.doubleValue();
    if (!units.equals("m")) return String.format(Locale.ROOT, "%.4g units2", a);
    return a >= 1e6 ? String.format(Locale.ROOT, "%.3f km2", a / 1e6) : String.format(Locale.ROOT, "%.2f ha", a / 1e4);
  }

  private static String pixels(Object v) {
    return v instanceof Number n ? String.format(Locale.ROOT, "%.3f px", n.doubleValue()) : "-";
  }

  private static String pdf(Object v) {
    return v == null ? "-" : String.valueOf(v);
  }

  private static String clock(double seconds) {
    long s = Math.round(seconds);
    return s >= 3600
        ? String.format(Locale.ROOT, "%d h %02d min", s / 3600, s / 60 % 60)
        : String.format(Locale.ROOT, "%d min %02d s", s / 60, s % 60);
  }

  private static void documentSave(PDDocument document, Path path) throws IOException {
    document.save(path.toFile());
  }

  private static String seconds(Object value) {
    return value instanceof Number n ? String.format(Locale.ROOT, "%.2f s", n.doubleValue()) : "unavailable";
  }

  private static String percentage(Object value) {
    return value instanceof Number n ? String.format(Locale.ROOT, "%.1f%%", n.doubleValue()) : "unavailable";
  }

  private static String brief(Object value) {
    if (value == null) return "Unavailable";
    if (value instanceof List<?> list && list.size() > 12)
      return list.size() + " entries; see archived JSON for complete values";
    String text = value instanceof Map<?, ?> || value instanceof List<?> ? Json.string(value) : String.valueOf(value);
    return text.length() > 1600 ? text.substring(0, 1590) + " [...]" : text;
  }

  // ---------------------------------------------------------------- layout

  private void page(String title) throws IOException {
    finish();
    page++;
    section = title;
    PDPage p = new PDPage(PDRectangle.A4);
    document.addPage(p);
    content = new PDPageContentStream(document, p);
    // Header: logo left, report name right, accent rule below.
    drawLogo(LEFT, 818, 20);
    float label = regular.getStringWidth("PROCESSING REPORT") * 8.5f / 1000;
    text("PROCESSING REPORT", LEFT + WIDTH - label, 805, 8.5f, regular, GRAY);
    fill(LEFT, 790, WIDTH, 1.2f, ACCENT);
    text(title, LEFT, 756, 23, bold, INK);
    y = 727;
  }

  private void finish() throws IOException {
    if (content == null) return;
    text(NAME + " " + VERSION + " | " + runId, LEFT, 24, 8, regular, GRAY);
    text("Page " + page, 507, 24, 8, regular, GRAY);
    content.close();
    content = null;
  }

  private void ensure(float height) throws IOException {
    if (y - height < 50) page(section.endsWith("(continued)") ? section : section + " (continued)");
  }

  private void heading(String title) throws IOException {
    ensure(48);
    y -= 9;
    text(title, LEFT, y, 13, bold, TEAL);
    y -= 23;
  }

  private void paragraph(String value) throws IOException {
    for (String line : wrap(value, regular, 9.5f, WIDTH)) {
      ensure(14);
      text(line, LEFT, y, 9.5f, regular, GRAY);
      y -= 14;
    }
    y -= 6;
  }

  private void row(String key, String value) throws IOException {
    List<String> keys = wrap(key, regular, 8.5f, 170), values = wrap(value, regular, 8.5f, 325);
    int count = Math.max(keys.size(), values.size());
    for (int i = 0; i < count; i++) {
      ensure(13);
      if (i < keys.size()) text(keys.get(i), LEFT, y, 8.5f, regular, INK);
      if (i < values.size()) text(values.get(i), 226, y, 8.5f, regular, GRAY);
      y -= 13;
    }
    y -= 4;
  }

  /** Label/value pairs in a four-column grid (label, value, label, value). */
  private void facts(List<String[]> rows) throws IOException {
    float h = 17;
    ensure(rows.size() * h + 10);
    for (int i = 0; i < rows.size(); i++) {
      String[] r = rows.get(i);
      if (i % 2 == 0) fill(LEFT, y - h + 4, WIDTH, h, STRIPE);
      text(r[0], LEFT + 6, y - 8, 9, bold, INK);
      text(r[1], LEFT + 130, y - 8, 9, regular, GRAY);
      if (r.length > 3) {
        text(r[2], LEFT + 260, y - 8, 9, bold, INK);
        text(r[3], LEFT + 384, y - 8, 9, regular, GRAY);
      }
      y -= h;
    }
    y -= 10;
  }

  /** Table with a shaded header; cells wrap; the header repeats after a page break. */
  private void table(String[] header, float[] widths, List<String[]> rows) throws IOException {
    float size = 8.5f, line = 11.5f;
    ensure(2 * line + 14);
    tableHeader(header, widths);
    for (int r = 0; r < rows.size(); r++) {
      String[] cells = rows.get(r);
      List<List<String>> wrapped = new ArrayList<>();
      int lines = 1;
      for (int c = 0; c < widths.length; c++) {
        List<String> w = wrap(c < cells.length && cells[c] != null ? cells[c] : "", regular, size, widths[c] - 8);
        if (w.isEmpty()) w = List.of("");
        wrapped.add(w);
        lines = Math.max(lines, w.size());
      }
      float h = lines * line + 5;
      if (y - h < 50) {
        page(section.endsWith("(continued)") ? section : section + " (continued)");
        tableHeader(header, widths);
      }
      if (r % 2 == 1) fill(LEFT, y - h, WIDTH, h, STRIPE);
      float x = LEFT;
      for (int c = 0; c < widths.length; c++) {
        for (int i = 0; i < wrapped.get(c).size(); i++)
          text(wrapped.get(c).get(i), x + 4, y - 9.5f - i * line, size, regular, c == 0 ? INK : GRAY);
        x += widths[c];
      }
      y -= h;
    }
    rule(LEFT, y, LEFT + WIDTH, y);
    y -= 12;
  }

  private void tableHeader(String[] header, float[] widths) throws IOException {
    float h = 16;
    fill(LEFT, y - h, WIDTH, h, LIGHT);
    float x = LEFT;
    for (int c = 0; c < widths.length; c++) {
      text(header[c], x + 4, y - 11, 8.5f, bold, INK);
      x += widths[c];
    }
    y -= h;
  }

  private void figure(Path image, float maxHeight) throws IOException {
    if (!Files.isRegularFile(image)) {
      paragraph("Overview unavailable for this run.");
      return;
    }
    PDImageXObject object = PDImageXObject.createFromFile(image.toString(), document);
    float scale = Math.min(WIDTH / object.getWidth(), maxHeight / object.getHeight());
    float width = object.getWidth() * scale, height = object.getHeight() * scale;
    ensure(height + 10);
    content.drawImage(object, LEFT + (WIDTH - width) / 2, y - height, width, height);
    y -= height + 12;
    if (image.getFileName().toString().equals("orthomosaic.png") && maps.get("cell_size") instanceof Number cell) {
      float x = LEFT + (WIDTH - width) / 2;
      double units = cell.doubleValue() * object.getWidth() / width * 100;
      fill(x + 7, y + 20, 150, 30, Color.WHITE);
      content.setStrokingColor(INK);
      content.setLineWidth(1);
      content.moveTo(x + 15, y + 27);
      content.lineTo(x + 115, y + 27);
      content.stroke();
      text(String.format(Locale.ROOT, "%.3g output units", units), x + 15, y + 35, 8, regular, INK);
      float arrowX = x + width - 15, arrowY = y + height - 10;
      content.moveTo(arrowX, arrowY - 12);
      content.lineTo(arrowX, arrowY + 8);
      content.moveTo(arrowX - 4, arrowY + 3);
      content.lineTo(arrowX, arrowY + 8);
      content.lineTo(arrowX + 4, arrowY + 3);
      content.stroke();
      text(Boolean.TRUE.equals(maps.get("north_established")) ? "N (up)" : "Local +Y (up)", x + width - 90, y + height + 2, 8, regular, INK);
    }
  }

  /** Two images side by side (each half the width). */
  private void figurePair(Path left, Path right, float maxHeight) throws IOException {
    Path[] files = {left, right};
    PDImageXObject[] images = new PDImageXObject[2];
    float height = 0, half = (WIDTH - 10) / 2;
    float[] w = new float[2], h = new float[2];
    for (int i = 0; i < 2; i++)
      if (Files.isRegularFile(files[i])) {
        images[i] = PDImageXObject.createFromFile(files[i].toString(), document);
        float s = Math.min(half / images[i].getWidth(), maxHeight / images[i].getHeight());
        w[i] = images[i].getWidth() * s;
        h[i] = images[i].getHeight() * s;
        height = Math.max(height, h[i]);
      }
    if (height == 0) {
      paragraph("Camera and overlap maps unavailable for this run.");
      return;
    }
    ensure(height + 10);
    for (int i = 0; i < 2; i++)
      if (images[i] != null) content.drawImage(images[i], LEFT + i * (half + 10) + (half - w[i]) / 2, y - h[i], w[i], h[i]);
    y -= height + 12;
  }

  /** Vector field of the mean image residual per cell, in the source image frame. */
  private void residualPlot(Map<String, Object> r) throws IOException {
    int columns = (int) number(r.get("columns")), rows = (int) number(r.get("rows"));
    double iw = number(r.get("width")), ih = number(r.get("height"));
    List<?> cells = r.get("cells") instanceof List<?> l ? l : List.of();
    if (columns <= 0 || rows <= 0 || cells.size() != columns * rows) return;
    float maxH = 250, maxW = 330;
    float s = (float) Math.min(maxW / iw, maxH / ih), w = (float) (iw * s), h = (float) (ih * s);
    ensure(h + 30);
    float x0 = LEFT + (WIDTH - w) / 2, y0 = y - h;
    content.setStrokingColor(RULE);
    content.setLineWidth(.5f);
    for (int c = 0; c <= columns; c++) {
      content.moveTo(x0 + c * w / columns, y0);
      content.lineTo(x0 + c * w / columns, y0 + h);
    }
    for (int k = 0; k <= rows; k++) {
      content.moveTo(x0, y0 + k * h / rows);
      content.lineTo(x0 + w, y0 + k * h / rows);
    }
    content.stroke();
    double max = 0;
    for (Object o : cells) {
      double[] c = numbers(o);
      if (c != null && c[2] > 0) max = Math.max(max, Math.hypot(c[0], c[1]));
    }
    float cell = Math.min(w / columns, h / rows);
    double scale = max > 0 ? .9 * cell / max : 0;
    for (int k = 0; k < rows; k++)
      for (int c = 0; c < columns; c++) {
        double[] v = numbers(cells.get(k * columns + c));
        if (v == null || v[2] == 0) continue;
        float cx = x0 + (c + .5f) * w / columns, cy = y0 + h - (k + .5f) * h / rows;
        arrow(cx, cy, (float) (v[0] * scale), (float) (-v[1] * scale), INK);
      }
    content.setStrokingColor(INK);
    content.setLineWidth(.8f);
    content.addRect(x0, y0, w, h);
    content.stroke();
    y = y0 - 12;
    text(String.format(Locale.ROOT, "Arrow of one cell = %.2f px", max > 0 ? cell / scale : 0), x0, y, 8, regular, GRAY);
    text(String.format(Locale.ROOT, "%,.0f x %,.0f px", iw, ih), x0 + w - 70, y, 8, regular, GRAY);
    y -= 14;
  }

  /** Points with horizontal error arrows coloured by the vertical error (camera locations). */
  private void errorPlot(List<double[]> at, List<double[]> error, List<String> labels, List<double[]> background, String units)
      throws IOException {
    plot(at, error, labels, null, background, units);
  }

  private void gcpPlot(List<double[]> at, List<double[]> error, List<String> labels, List<double[]> check, List<double[]> cameras, String units)
      throws IOException {
    plot(at, error, labels, check, cameras, units);
  }

  private void plot(List<double[]> at, List<double[]> error, List<String> labels, List<double[]> check, List<double[]> background, String units)
      throws IOException {
    if (at.isEmpty()) return;
    double minX = Double.MAX_VALUE, maxX = -Double.MAX_VALUE, minY = Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
    for (List<double[]> list : List.of(at, background))
      for (double[] p : list) {
        minX = Math.min(minX, p[0]);
        maxX = Math.max(maxX, p[0]);
        minY = Math.min(minY, p[1]);
        maxY = Math.max(maxY, p[1]);
      }
    // The box keeps at least a 1:2 aspect; points sit 46 pt inside it so the arrows stay inside.
    double spanX = Math.max(maxX - minX, 1e-6), spanY = Math.max(maxY - minY, 1e-6);
    if (spanX < spanY / 2) {
      minX -= (spanY / 2 - spanX) / 2;
      maxX = minX + spanY / 2;
    } else if (spanY < spanX / 2) {
      minY -= (spanX / 2 - spanY) / 2;
      maxY = minY + spanX / 2;
    }
    float inset = 46, maxW = WIDTH - 70 - 2 * inset, maxH = 250 - 2 * inset;
    float s = (float) Math.min(maxW / (maxX - minX), maxH / (maxY - minY));
    float w = (float) ((maxX - minX) * s) + 2 * inset, h = (float) ((maxY - minY) * s) + 2 * inset;
    ensure(h + 40);
    float bx = LEFT + (WIDTH - 70 - w) / 2, by = y - h;
    float x0 = bx + inset, y0 = by + inset;
    content.setStrokingColor(RULE);
    content.setLineWidth(.8f);
    content.addRect(bx, by, w, h);
    content.stroke();
    double maxXY = 0, maxZ = 0;
    for (double[] e : error) {
      maxXY = Math.max(maxXY, Math.hypot(e[0], e[1]));
      maxZ = Math.max(maxZ, Math.abs(e[2]));
    }
    // Arrow length: the largest horizontal error spans 40 points.
    double arrowScale = maxXY > 0 ? 40 / maxXY : 0;
    final double fMinX = minX, fMinY = minY;
    for (double[] p : background) {
      float px = (float) (x0 + (p[0] - fMinX) * s), py = (float) (y0 + (p[1] - fMinY) * s);
      circle(px, py, 1.6f, RULE, true);
    }
    for (int i = 0; i < at.size(); i++) {
      double[] p = at.get(i), e = error.get(i);
      float px = (float) (x0 + (p[0] - fMinX) * s), py = (float) (y0 + (p[1] - fMinY) * s);
      Color colour = diverging(maxZ > 0 ? e[2] / maxZ : 0);
      boolean isCheck = check != null && check.get(i)[0] == 1;
      if (check == null) circle(px, py, 3f, colour, true);
      else if (isCheck) circle(px, py, 4f, colour, true);
      else triangle(px, py, 5f, colour);
      arrow(px, py, (float) (e[0] * arrowScale), (float) (e[1] * arrowScale), INK);
      if (labels.get(i) != null) text(labels.get(i), px + 6, py + 4, 7, regular, INK);
    }
    // Colour legend (vertical error) on the right.
    float lx = bx + w + 16, ly = by + h - 110;
    for (int k = 0; k < 20; k++) fill(lx, ly + k * 5, 10, 5, diverging(-1 + k / 9.5));
    text(length(maxZ, units, 3), lx + 13, ly + 96, 7, regular, GRAY);
    text("0", lx + 13, ly + 48, 7, regular, GRAY);
    text(length(-maxZ, units, 3), lx + 13, ly, 7, regular, GRAY);
    text("Z error", lx - 2, ly + 108, 7, bold, INK);
    y = by - 12;
    text(String.format(Locale.ROOT, "Arrow 40 pt = %s (horizontal error). Extent %.0f x %.0f %s.", length(maxXY, units, 3), maxX - minX, maxY - minY, units), bx, y, 8, regular, GRAY);
    y -= 14;
  }

  private static Color diverging(double t) {
    t = Math.max(-1, Math.min(1, t));
    Color end = t < 0 ? BLUE : RED, mid = new Color(214, 214, 214);
    double a = Math.abs(t);
    return new Color(
        (int) Math.round(mid.getRed() + a * (end.getRed() - mid.getRed())),
        (int) Math.round(mid.getGreen() + a * (end.getGreen() - mid.getGreen())),
        (int) Math.round(mid.getBlue() + a * (end.getBlue() - mid.getBlue())));
  }

  private void arrow(float x, float y, float dx, float dy, Color colour) throws IOException {
    float length = (float) Math.hypot(dx, dy);
    if (length < .3f) return;
    content.setStrokingColor(colour);
    content.setLineWidth(.7f);
    content.moveTo(x, y);
    content.lineTo(x + dx, y + dy);
    float head = Math.min(3.5f, length * .4f), ux = dx / length, uy = dy / length;
    content.moveTo(x + dx - head * (ux - .5f * uy), y + dy - head * (uy + .5f * ux));
    content.lineTo(x + dx, y + dy);
    content.lineTo(x + dx - head * (ux + .5f * uy), y + dy - head * (uy - .5f * ux));
    content.stroke();
  }

  private void circle(float x, float y, float r, Color colour, boolean filled) throws IOException {
    float k = .5523f * r;
    content.moveTo(x + r, y);
    content.curveTo(x + r, y + k, x + k, y + r, x, y + r);
    content.curveTo(x - k, y + r, x - r, y + k, x - r, y);
    content.curveTo(x - r, y - k, x - k, y - r, x, y - r);
    content.curveTo(x + k, y - r, x + r, y - k, x + r, y);
    content.closePath();
    if (filled) {
      content.setNonStrokingColor(colour);
      content.fill();
    } else {
      content.setStrokingColor(colour);
      content.stroke();
    }
  }

  private void triangle(float x, float y, float r, Color colour) throws IOException {
    content.moveTo(x, y + r);
    content.lineTo(x - .87f * r, y - .5f * r);
    content.lineTo(x + .87f * r, y - .5f * r);
    content.closePath();
    content.setNonStrokingColor(colour);
    content.fill();
  }

  private void fill(float x, float y, float w, float h, Color colour) throws IOException {
    content.setNonStrokingColor(colour);
    content.addRect(x, y, w, h);
    content.fill();
  }

  private void rule(float x1, float y1, float x2, float y2) throws IOException {
    content.setStrokingColor(RULE);
    content.setLineWidth(.6f);
    content.moveTo(x1, y1);
    content.lineTo(x2, y2);
    content.stroke();
  }

  private String safe(String text) {
    StringBuilder out = new StringBuilder();
    for (int i = 0; i < text.length(); i++) {
      char c = text.charAt(i);
      if (Character.isISOControl(c)) c = ' ';
      try {
        regular.encode(String.valueOf(c));
        out.append(c);
      } catch (IOException | IllegalArgumentException missing) {
        out.append('?');
      }
    }
    return out.toString();
  }

  private List<String> wrap(String value, PDFont font, float size, float width) throws IOException {
    String text = safe(value);
    List<String> lines = new ArrayList<>();
    StringBuilder line = new StringBuilder();
    for (String word : text.split("\\s+")) {
      if (word.isEmpty()) continue;
      if (line.length() > 0 && font.getStringWidth(line + " " + word) * size / 1000 > width) {
        lines.add(line.toString().stripTrailing());
        line.setLength(0);
      }
      for (int i = 0; i < word.length(); i++) {
        if (font.getStringWidth(line.toString() + word.charAt(i)) * size / 1000 > width && line.length() > 0) {
          lines.add(line.toString().stripTrailing());
          line.setLength(0);
        }
        line.append(word.charAt(i));
      }
      line.append(' ');
    }
    if (line.length() > 0) lines.add(line.toString().stripTrailing());
    return lines;
  }

  private void text(String value, float x, float y, float size, PDFont font, Color color) throws IOException {
    content.setNonStrokingColor(color);
    content.beginText();
    content.setFont(font, size);
    content.newLineAtOffset(x, y);
    content.showText(safe(value));
    content.endText();
  }

  public void close() throws IOException {
    finish();
    document.close();
  }
}
