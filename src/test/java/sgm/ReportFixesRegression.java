package sgm;

import java.nio.file.*;
import java.util.*;

/**
 * 14 November 2026 report fixes: the report reads DSM heights itself (ImageIO returned 1.0 for
 * every height, which shifted the overview orthomosaic in blocks), the EPSG code comes from
 * horizontal_crs when "epsg" is missing, the frame messages, and the tie-point fallback for the
 * coverage area ignores far outliers.
 */
final class ReportFixesRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    System.out.println("PASS report fixes=" + run(Files.createTempDirectory(Path.of("target"), "report-fixes-")));
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Files.createDirectories(root);
    int w = 37, h = 23;
    float[] z = new float[w * h];
    for (int i = 0; i < z.length; i++) z[i] = i % 11 == 0 ? Float.NaN : 87.5f + i * .125f;
    Path tif = root.resolve("dsm.tif");
    DsmRaster.writeTiff(tif, w, h, z, 611959.25, 6933603.5, .25, 3067);
    DsmRaster.Heights back = DsmRaster.readTiff(tif);
    check(back.width() == w && back.height() == h, "DSM size read back");
    check(Arrays.equals(back.z(), z), "DSM heights read back exactly (NaN gaps kept)");

    check(DsmRaster.epsg(Map.of("epsg", 3067)) == 3067, "EPSG from the epsg field");
    check(DsmRaster.epsg(Map.of("horizontal_crs", "EPSG:3067")) == 3067,
        "EPSG from horizontal_crs (supplied projected orientations have no epsg field)");
    check(DsmRaster.epsg(Map.of("georeferenced", false)) == 0, "No EPSG without a CRS");
    check(DsmRaster.frameProblem(Map.of()).contains("no coordinates.json"),
        "Missing coordinates.json is named as the reason");
    check(DsmRaster.frameProblem(Map.of("georeferenced", false, "reason", "too few GPS images"))
            .contains("too few GPS images"),
        "A local frame gives the georeference reason");

    Map<String, Object> data =
        Json.object("survey", Json.object("camera_locations", Json.object("rms", Json.object("total", .79))));
    check(PdfReport.adjustedCameraRms(data) == .79, "Summary uses the adjusted camera location RMS");
    check(Double.isNaN(PdfReport.adjustedCameraRms(Json.object("survey", Json.object()))),
        "Without the GPS adjustment the similarity RMS is used");

    List<double[]> ties = new ArrayList<>();
    Random r = new Random(3);
    for (int i = 0; i < 2000; i++) ties.add(new double[] {r.nextDouble() * 100, r.nextDouble() * 100});
    ties.add(new double[] {5000, 5000});
    ties.add(new double[] {-4000, 3000});
    double all = ReportSurvey.hullArea(ties), trimmed = ReportSurvey.hullArea(ReportSurvey.trimmed(ties, .01));
    check(all > 1e7, "Two far tie points stretch the full hull");
    check(trimmed > 9000 && trimmed < 10000, "The trimmed hull is the 100 x 100 block: " + trimmed);
    return checks;
  }
}
