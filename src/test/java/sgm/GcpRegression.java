package sgm;

import java.net.*;
import java.net.http.*;
import java.nio.file.*;
import java.util.*;

/**
 * Ground control points on forest_tiny (skipped without the example data). Well-observed sparse
 * points become GCPs whose coordinates are shifted by a known offset; their markers are their real
 * image positions plus noise. The gcp step must wait without pins, the editor must serve and save
 * markers, and with pins the block must move onto the GCP datum (by the offset) with small control
 * and check residuals, and the later steps must still run.
 */
final class GcpRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    Path root = Files.createTempDirectory(Path.of("target"), "gcp-");
    System.out.println("PASS ground control points: " + run(root) + "; " + root);
  }

  static final double[] OFFSET = {1.5, -0.7, 2.0};

  static int run(Path root) throws Exception {
    checks = 0;
    Path images = Path.of("..", "example_data", "forest", "forest_tiny");
    if (!Files.isDirectory(images)) {
      System.out.println("GcpRegression skipped: no " + images);
      return 0;
    }
    Path project = root.resolve("p");
    String[] base = {"-i", images.toString(), "-o", project.toString(), "--quality", "low", "--cores", "4"};
    check(Main.run(concat(new String[] {"inputs"}, base)) == 0, "inputs");
    check(Main.run(new String[] {"calibrate", "match", "align", "georeference", "-o", project.toString()}) == 0, "steps up to georeference");
    Path run = project.resolve("runs").resolve(Main.latestRun(project));
    PipelineState state = PipelineState.load(run);
    Options alignment = Options.parse(Main.continued(Main.command(new String[] {"gcp", "-o", project.toString()}))).alignmentOptions();
    alignment.alignmentCalibration = true;
    List<ImageFrame> frames = state.readFrames(alignment);
    FeatureTracks features = state.readTracks();
    // GCPs: the best-observed tracks, coordinates shifted by OFFSET; markers at their true pixels.
    List<FeatureTracks.Track> chosen = new ArrayList<>();
    features.tracks.stream()
        .filter(t -> t.point != null)
        .sorted(Comparator.comparingInt((FeatureTracks.Track t) -> -t.observations.size()))
        .forEach(
            t -> {
              for (var c : chosen) if (Math3.norm(Math3.sub(c.point, t.point)) < 3) return;
              if (chosen.size() < 8) chosen.add(t);
            });
    check(chosen.size() >= 6, "enough well-spread sparse points for GCPs");
    Random random = new Random(3);
    StringBuilder csv = new StringBuilder("gcp_id,x,y,z,sx,sy,sz,type\n");
    GroundControl.Markers markers = new GroundControl.Markers();
    double[] centreBefore = meanCentre(frames);
    for (int i = 0; i < chosen.size(); i++) {
      FeatureTracks.Track t = chosen.get(i);
      String id = "G" + (i + 1);
      double[] xyz = Math3.add(t.point, OFFSET);
      boolean check = i >= chosen.size() - 2;
      csv.append(String.format(Locale.ROOT, "%s,%.4f,%.4f,%.4f,0.02,0.02,0.03,%s%n", id, xyz[0], xyz[1], xyz[2], check ? "check" : "control"));
      for (var ob : t.observations) {
        ImageFrame f = frames.get(ob.image);
        double[] uv = GroundControl.project(f, t.point);
        if (!GroundControl.inside(f, uv)) continue;
        GroundControl.Marker m = new GroundControl.Marker(id, f.id);
        m.status = "pinned";
        m.u = uv[0] + random.nextGaussian() * .3;
        m.v = uv[1] + random.nextGaussian() * .3;
        markers.markers.add(m);
      }
    }
    Path gcps = root.resolve("gcps.csv");
    Files.writeString(gcps, csv);

    // GCP8: one pin per GCP at its true pixel; the other images are refined by patch matching
    // and must land on the true projections (the predictions start from the shifted coordinates).
    GroundControl.Markers single = new GroundControl.Markers();
    single.gcps.addAll(GroundControl.read(gcps, .02, .03));
    for (int i = 0; i < chosen.size(); i++) {
      FeatureTracks.Track t = chosen.get(i);
      ImageFrame f = frames.get(t.observations.get(0).image);
      GroundControl.Marker m = new GroundControl.Marker("G" + (i + 1), f.id);
      m.status = "pinned";
      double[] uv = GroundControl.project(f, t.point);
      m.u = uv[0];
      m.v = uv[1];
      single.markers.add(m);
    }
    GroundControl.predict(single, frames, true);
    GcpEditor.Editor refineEditor = new GcpEditor.Editor(frames, single, root.resolve("refine-markers.json"), true);
    int refined = 0, candidates = 0;
    double worst = 0, inconsistent = 0;
    for (int i = 0; i < chosen.size(); i++) {
      String id = "G" + (i + 1);
      int source = chosen.get(i).observations.get(0).image;
      refineEditor.refine(id, source);
      List<GroundControl.Marker> found = new ArrayList<>();
      for (GroundControl.Marker k : single.of(id)) {
        if (k.pinned()) continue;
        candidates++;
        if (!k.status.equals("refined")) continue;
        refined++;
        found.add(k);
        // The tie point itself is only good to about 2 source pixels (aligned at 1/8 resolution).
        double[] truth = GroundControl.project(frames.get(k.image), chosen.get(i).point);
        worst = Math.max(worst, Math.hypot(k.u - truth[0], k.v - truth[1]));
      }
      // Consistency: the refined markers and the pin meet in one 3D point.
      if (found.size() >= 1) {
        for (GroundControl.Marker k : found) k.status = "pinned";
        double[] point = GroundControl.triangulate(single, id, frames);
        for (GroundControl.Marker k : found) inconsistent = Math.max(inconsistent, GroundControl.residual(frames.get(k.image), k, point));
        for (GroundControl.Marker k : found) k.status = "refined";
      }
    }
    refineEditor.close();
    check(candidates > 0 && refined >= candidates / 2, "most markers refined by patch matching: " + refined + " of " + candidates);
    check(worst < 5, "refined markers near the tie-point projections (worst " + worst + " px)");
    check(inconsistent < 1.5, "refined markers consistent with the pin (worst residual " + inconsistent + " px)");
    System.out.printf(Locale.ROOT, "GCP refinement: %d of %d refined, worst %.2f px from the tie point, worst residual %.2f px%n", refined, candidates, worst, inconsistent);

    // GCP10: a canopy block of tie points hides a point under it from an oblique camera.
    List<FeatureTracks.Track> surface = new ArrayList<>();
    for (int x = 0; x < 40; x++)
      for (int y = 0; y < 40; y++)
        for (int k = 0; k < 4; k++) {
          FeatureTracks.Track t = new FeatureTracks.Track();
          boolean canopy = x >= 15 && x < 25;
          t.point = new double[] {x + k * .2, y + k * .2, canopy ? 15 : 0};
          surface.add(t);
        }
    GroundControl.Occlusion occlusion = GroundControl.Occlusion.of(surface);
    check(occlusion != null, "occlusion surface built");
    check(occlusion.blocked(new double[] {35, 20, 40}, new double[] {12, 20, 0}), "ray through the canopy block is blocked");
    check(!occlusion.blocked(new double[] {5, 20, 40}, new double[] {8, 20, 0}), "ray beside the block is not blocked");
    check(!occlusion.blocked(new double[] {20, 20, 60}, new double[] {20.4, 20, 15}), "a point on the canopy is visible from above");
    for (ImageFrame f : frames) f.close();

    // 0. Without --gcp (and without ground options) the optional steps need not be named.
    Path plain = root.resolve("n");
    check(Main.run(concat(new String[] {"inputs"}, new String[] {"-i", images.toString(), "-o", plain.toString(), "--quality", "low", "--cores", "4"})) == 0, "inputs (no GCP project)");
    check(Main.run(new String[] {"calibrate", "match", "align", "georeference", "-o", plain.toString()}) == 0, "steps up to georeference (no GCP project)");
    check(Main.run(new String[] {"dense", "-o", plain.toString()}) == 0, "dense runs without the gcp step when --gcp is not given");
    check(Main.run(new String[] {"finish", "-o", plain.toString()}) == 0, "finish runs without the ground step when no ground option is given");
    PipelineState plainState = PipelineState.load(plain.resolve("runs").resolve(Main.latestRun(plain)));
    check("skipped".equals(plainState.status(Pipeline.Step.GCP)) && "skipped".equals(plainState.status(Pipeline.Step.GROUND)), "optional steps recorded as skipped");
    check(Files.isRegularFile(plain.resolve("runs").resolve(Main.latestRun(plain)).resolve("07_results/processing-report.pdf")), "report written without GCPs");

    // 1. Without pins the step waits, with predicted markers written.
    Path waitingProject = root.resolve("w");
    check(Main.run(concat(new String[] {"inputs"}, new String[] {"-i", images.toString(), "-o", waitingProject.toString(), "--quality", "low", "--cores", "4"})) == 0, "inputs (waiting project)");
    check(Main.run(new String[] {"calibrate", "match", "align", "georeference", "gcp", "-o", waitingProject.toString(), "--gcp", gcps.toString()}) == 0, "gcp step runs");
    Path waitingRun = waitingProject.resolve("runs").resolve(Main.latestRun(waitingProject));
    check("waiting".equals(PipelineState.load(waitingRun).status(Pipeline.Step.GCP)), "gcp step waits for pins");
    GroundControl.Markers predicted = GroundControl.load(GroundControl.markersFile(waitingRun));
    check(predicted.markers.size() > chosen.size(), "predicted markers written for the GCPs");
    check(Main.run(new String[] {"dense", "-o", waitingProject.toString()}) != 0, "dense refused while gcp waits");

    // 2. The editor serves the state, saves a pin, stops on done.
    int port = 18000 + random.nextInt(2000);
    Thread editor =
        new Thread(
            () -> {
              try {
                GcpEditor.serve(Main.command(new String[] {"gcp-edit", "-o", waitingProject.toString(), "--gcp-port", "" + port}));
              } catch (Exception e) {
                throw new RuntimeException(e);
              }
            });
    editor.setDaemon(true);
    editor.start();
    HttpClient http = HttpClient.newHttpClient();
    String state1 = null;
    for (int attempt = 0; attempt < 50 && state1 == null; attempt++)
      try {
        state1 = http.send(HttpRequest.newBuilder(URI.create("http://127.0.0.1:" + port + "/api/state")).build(), HttpResponse.BodyHandlers.ofString()).body();
      } catch (java.io.IOException e) {
        Thread.sleep(200);
      }
    check(state1 != null && state1.contains("\"markers\""), "editor serves its state");
    GroundControl.Marker first = predicted.markers.get(0);
    String body = String.format(Locale.ROOT, "{\"gcp\":\"%s\",\"image\":%d,\"status\":\"pinned\",\"u\":%.2f,\"v\":%.2f}", first.gcp, first.image, first.u + 1, first.v);
    var posted = http.send(HttpRequest.newBuilder(URI.create("http://127.0.0.1:" + port + "/api/marker")).POST(HttpRequest.BodyPublishers.ofString(body)).build(), HttpResponse.BodyHandlers.ofString());
    check(posted.statusCode() == 200, "editor accepts a pin");
    var crop = http.send(HttpRequest.newBuilder(URI.create(String.format(Locale.ROOT, "http://127.0.0.1:%d/api/image?id=%d&cx=%.1f&cy=%.1f&size=200&out=200", port, first.image, first.u, first.v))).build(), HttpResponse.BodyHandlers.ofByteArray());
    check(crop.statusCode() == 200 && crop.body().length > 1000, "editor serves image crops");
    // The pin starts a background refinement; the state reports it until done; then accept all.
    String polled = null;
    for (int attempt = 0; attempt < 300; attempt++) {
      polled = http.send(HttpRequest.newBuilder(URI.create("http://127.0.0.1:" + port + "/api/state")).build(), HttpResponse.BodyHandlers.ofString()).body();
      if (polled.contains("\"refining\":false")) break;
      Thread.sleep(200);
    }
    check(polled != null && polled.contains("\"refining\":false"), "refinement after a pin finishes");
    var accepted =
        http.send(
            HttpRequest.newBuilder(URI.create("http://127.0.0.1:" + port + "/api/accept"))
                .POST(HttpRequest.BodyPublishers.ofString("{\"gcp\":\"" + first.gcp + "\"}"))
                .build(),
            HttpResponse.BodyHandlers.ofString());
    check(accepted.statusCode() == 200 && accepted.body().contains("\"markers\""), "accept-all endpoint");
    http.send(HttpRequest.newBuilder(URI.create("http://127.0.0.1:" + port + "/api/done")).build(), HttpResponse.BodyHandlers.ofString());
    editor.join(20000);
    check(!editor.isAlive(), "editor stops on done");
    check(GroundControl.load(GroundControl.markersFile(waitingRun)).marker(first.gcp, first.image).pinned(), "pin saved to markers.json");

    // 3. With pins: datum onto the GCPs, small residuals, later steps run.
    Path markersFile = root.resolve("markers.json");
    markers.gcps.addAll(GroundControl.read(gcps, .02, .03));
    GroundControl.save(markers, markersFile, null);
    check(Main.run(new String[] {"gcp", "-o", project.toString(), "--gcp", gcps.toString(), "--gcp-markers", markersFile.toString()}) == 0, "gcp step with pinned markers");
    state = PipelineState.load(run);
    check("complete".equals(state.status(Pipeline.Step.GCP)), "gcp step complete");
    var report = JsonRead.object(JsonRead.read(run.resolve("04_geometry/gcp/gcp_report.json")));
    double control = ((Number) JsonRead.object(report.get("control_rms_metres")).get("total")).doubleValue();
    double checkRms = ((Number) JsonRead.object(report.get("check_rms_metres")).get("total")).doubleValue();
    check(control < .15, "control RMS " + control);
    check(checkRms < .4, "check RMS " + checkRms);
    frames = state.readFrames(alignment);
    double[] moved = Math3.sub(meanCentre(frames), centreBefore);
    for (ImageFrame f : frames) f.close();
    // Horizontally the block follows the GCP offset; vertically the camera heights may also absorb
    // the datum scale (the GCPs themselves sit at their given heights, see the control RMS).
    check(Math.hypot(moved[0] - OFFSET[0], moved[1] - OFFSET[1]) < .3, "block moved onto the GCP datum: " + Arrays.toString(moved));
    check(Math.abs(moved[2] - OFFSET[2]) < 1, "camera heights follow the GCP datum: " + moved[2]);
    check(Main.run(new String[] {"dense", "ground", "finish", "-o", project.toString()}) == 0, "dense, ground and finish after the gcp step");
    check(Files.isRegularFile(run.resolve("07_results/dense.las")), "cloud written");
    try (var pdf = org.apache.pdfbox.Loader.loadPDF(run.resolve("07_results/processing-report.pdf").toFile())) {
      String text = new org.apache.pdfbox.text.PDFTextStripper().getText(pdf);
      check(text.contains("Ground Control Points") && text.contains("Check points") && text.contains("G8"), "report has the GCP section");
      check(text.contains("Camera Locations") && text.contains("Camera Calibration"), "report has camera sections");
    }
    return checks;
  }

  static double[] meanCentre(List<ImageFrame> frames) {
    double[] sum = new double[3];
    int n = 0;
    for (ImageFrame f : frames)
      if (f.camera.registered) {
        sum = Math3.add(sum, f.camera.center());
        n++;
      }
    return Math3.times(sum, 1. / n);
  }

  static String[] concat(String[] a, String[] b) {
    String[] out = Arrays.copyOf(a, a.length + b.length);
    System.arraycopy(b, 0, out, a.length, b.length);
    return out;
  }
}
