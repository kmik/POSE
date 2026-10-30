package sgm;

import java.io.*;
import java.nio.*;
import java.nio.file.*;
import java.util.*;
import java.util.zip.GZIPInputStream;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;

/**
 * Validate artifact contents/links, append history, failure isolation, and coordinate conventions.
 */
final class ProjectRegression {
  private static int checks;

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally();
    geometryAndCandidates(Path.of(args[0]));
    System.out.println("PASS project geometry/refinement checks=" + checks);
  }

  private static void check(boolean condition, String message) {
    checks++;
    if (!condition) throw new AssertionError(message);
  }

  static int run(Path fixtures) throws Exception {
    checks = 0;
    Path sfm = latest(fixtures.resolve("SfM result"), "latest_successful_run");
    inventory(sfm);
    features(sfm);
    geometryAndCandidates(sfm);
    stereo(sfm);
    history(fixtures);
    calibration(fixtures);
    return checks;
  }

  static Path latest(Path root, String key) throws IOException {
    Properties p = new Properties();
    try (Reader reader = Files.newBufferedReader(root.resolve("project.properties"))) {
      p.load(reader);
    }
    return root.resolve("runs").resolve(p.getProperty(key));
  }

  private static List<List<String>> rows(Path path) throws IOException {
    List<List<String>> rows = new ArrayList<>();
    try (BufferedReader reader =
        path.toString().endsWith(".gz")
            ? new BufferedReader(
                new InputStreamReader(
                    new GZIPInputStream(Files.newInputStream(path)),
                    java.nio.charset.StandardCharsets.UTF_8))
            : Files.newBufferedReader(path)) {
      String line;
      while ((line = reader.readLine()) != null) rows.add(PoseIO.csv(line));
    }
    return rows;
  }

  private static void inventory(Path run) throws IOException {
    List<List<String>> source = rows(run.resolve("00_inputs/images.csv"));
    check(source.size() == 4, "All three input images have provenance records");
    boolean originals = true;
    for (int i = 1; i < source.size(); i++) {
      List<String> row = source.get(i);
      originals &= Project.sha256(Paths.get(row.get(2))).equals(row.get(4));
    }
    check(originals, "Source hashes identify actual photographs");
    boolean catalog = true;
    List<List<String>> artifacts = rows(run.resolve("artifacts.csv"));
    for (int i = 1; i < artifacts.size(); i++) {
      List<String> row = artifacts.get(i);
      Path file = run.resolve(row.get(0)).normalize();
      catalog &=
          file.startsWith(run)
              && Files.isRegularFile(file)
              && Files.size(file) == Long.parseLong(row.get(1))
              && Project.sha256(file).equals(row.get(2));
    }
    check(
        catalog && artifacts.size() > 200,
        "Every indexed artifact exists with the declared size and SHA-256");
    check(
        Files.readString(run.resolve("logs/console.log")).contains("Registered 02.png"),
        "Camera-registration log was captured");
    check(
        Files.readString(run.resolve("logs/events.jsonl")).contains("run_complete"),
        "Structured lifecycle events are retained");
  }

  private static void features(Path run) throws IOException {
    List<List<String>> keys = rows(run.resolve("02_features/image_0000/keypoints.csv"));
    int count = keys.size() - 1;
    Path descriptor = run.resolve("02_features/image_0000/descriptors.f32");
    try (DataInputStream in = new DataInputStream(Files.newInputStream(descriptor))) {
      byte[] magic = new byte[8];
      in.readFully(magic);
      int rows = in.readInt(), columns = in.readInt();
      check(
          new String(magic, java.nio.charset.StandardCharsets.US_ASCII).equals("SGMDESC1")
              && rows == count
              && columns == 128,
          "SIFT descriptor rows match keypoint IDs and have 128 components");
      double norm = 0;
      for (int i = 0; i < columns; i++) {
        float v = in.readFloat();
        norm += v * v;
      }
      check(
          Math.sqrt(norm) > 100 && Math.sqrt(norm) < 1000,
          "Descriptor payload decodes as finite big-endian SIFT values");
      check(
          Files.size(descriptor) == 16L + 4L * rows * columns,
          "Descriptor binary contains all rows without padding");
    }
    List<List<String>> matches = rows(run.resolve("03_matches/pair_0000_0001/matches.csv.gz"));
    check(
        matches.size() - 1 == count,
        "All forward nearest-neighbor decisions are exported, including rejects");
    int used = 0, rejected = 0;
    boolean ids = true;
    for (int i = 1; i < matches.size(); i++) {
      List<String> m = matches.get(i);
      ids &= Integer.parseInt(m.get(0)) >= 0 && Integer.parseInt(m.get(0)) < count;
      if (Boolean.parseBoolean(m.get(10))) used++;
      else rejected++;
    }
    check(
        ids && used > 100 && rejected > 0,
        "Match exports distinguish accepted and rejected feature IDs");
    List<List<String>> graph = rows(run.resolve("03_matches/all_track_components.csv.gz"));
    int total = 0;
    for (int image = 0; image < 3; image++)
      total +=
          rows(run.resolve("02_features/" + ProjectArtifacts.imageId(image) + "/keypoints.csv"))
                  .size()
              - 1;
    check(
        graph.size() - 1 == total,
        "Track graph includes every feature, including unused components");
    List<String> kp = keys.get(1);
    check(
        Math.abs(Double.parseDouble(kp.get(1)) - Double.parseDouble(kp.get(8))) < 1e-6
            && Math.abs(Double.parseDouble(kp.get(2)) - Double.parseDouble(kp.get(9))) < 1e-6,
        "Zero-distortion, unscaled keypoints map back to their original raster coordinates");
  }

  private static void geometryAndCandidates(Path run) throws IOException {
    List<List<String>> poses = rows(run.resolve("04_geometry/final/exterior.csv")),
        interior = rows(run.resolve("04_geometry/final/interior.csv"));
    List<String> p = poses.get(1), k = interior.get(1);
    Camera camera =
        new Camera(
            Double.parseDouble(k.get(4)),
            Double.parseDouble(k.get(5)),
            Double.parseDouble(k.get(6)),
            Double.parseDouble(k.get(7)));
    for (int i = 0; i < 9; i++) camera.rotation[i] = Double.parseDouble(p.get(4 + i));
    for (int i = 0; i < 3; i++) camera.translation[i] = Double.parseDouble(p.get(13 + i));
    double[] center = camera.center();
    boolean centers = true;
    for (int i = 0; i < 3; i++)
      centers &= Math.abs(center[i] - Double.parseDouble(p.get(16 + i))) < 1e-9;
    check(centers, "Exported exterior centers satisfy C=-R^T*t");
    check(
        rows(run.resolve("04_geometry/bundle/iterations.csv")).size() >= 3,
        "Bundle objective history includes initial and refined states");
    check(
        Files.isRegularFile(run.resolve("04_geometry/seed/exterior.csv"))
            && Files.isRegularFile(run.resolve("04_geometry/pre_bundle/tracks.csv"))
            && Files.isRegularFile(
                run.resolve("04_geometry/post_bundle_before_track_filter/tracks.csv")),
        "Geometry checkpoints retain orientations and tracks before and after refinement");
    List<List<String>> candidates = rows(run.resolve("06_fusion/candidates.csv.gz")),
        decisions = rows(run.resolve("06_fusion/decisions.csv.gz"));
    check(candidates.size() == decisions.size(), "Every sampled candidate has one fusion decision");
    int kept = 0;
    boolean references = true;
    for (int i = 1; i < decisions.size(); i++) {
      List<String> d = decisions.get(i), c = candidates.get(i);
      references &=
          Long.parseLong(d.get(0)) == i - 1
              && d.get(0).equals(c.get(0))
              && d.get(1).equals(c.get(1))
              && d.get(2).equals(c.get(2))
              && d.get(3).equals(c.get(3));
      if (d.get(5).equals("accepted")) {
        kept++;
        references &= Integer.parseInt(d.get(4)) >= 3;
      }
    }
    ByteBuffer las =
        ByteBuffer.wrap(Files.readAllBytes(run.resolve("07_results/dense.las")))
            .order(ByteOrder.LITTLE_ENDIAN);
    check(
        references && kept == las.getInt(107),
        "Candidate IDs, pixel IDs, view counts, and final LAS count agree");
    List<List<String>> refined = rows(run.resolve("06_fusion/refinements.csv.gz"));
    boolean refinedExport = refined.size() == candidates.size();
    int pointIndex = 0;
    for (int i = 1; i < decisions.size(); i++) {
      List<String> d = decisions.get(i), r = refined.get(i), c = candidates.get(i);
      refinedExport &= r.get(0).equals(c.get(0)) && r.get(1).equals(c.get(1));
      for (int axis = 0; axis < 3; axis++)
        refinedExport &=
            Math.abs(Double.parseDouble(r.get(axis + 2)) - Double.parseDouble(c.get(axis + 5)))
                < 1e-8;
      if (d.get(5).equals("accepted")) {
        int offset = las.getInt(96) + pointIndex++ * Short.toUnsignedInt(las.getShort(105));
        for (int axis = 0; axis < 3; axis++) {
          double coordinate =
              las.getInt(offset + axis * 4) * las.getDouble(131 + axis * 8)
                  + las.getDouble(155 + axis * 8);
          refinedExport &= Math.abs(coordinate - Double.parseDouble(r.get(axis + 5))) < .00011;
        }
      }
    }
    check(
        refinedExport,
        "Final LAS coordinates trace to refined XYZ while candidate XYZ remains unchanged");
    List<String> first = candidates.get(1);
    int x = Integer.parseInt(first.get(2)), y = Integer.parseInt(first.get(3));
    double[] world = {
      Double.parseDouble(first.get(5)),
      Double.parseDouble(first.get(6)),
      Double.parseDouble(first.get(7))
    };
    double[] uv = camera.project(world);
    float[] mx = pfm(run.resolve("05_stereo/pair_0000_0001/left_source_x.pfm")),
        my = pfm(run.resolve("05_stereo/pair_0000_0001/left_source_y.pfm"));
    check(
        Math.abs(uv[0] - mx[y * 400 + x]) < .001 && Math.abs(uv[1] - my[y * 400 + x]) < .001,
        "A candidate reprojects through exported exterior/interior orientation to its"
            + " rectification-map source coordinate");
    List<List<String>> observations = rows(run.resolve("04_geometry/final/observations.csv"));
    check(
        observations.size() > 100 && observations.get(1).size() == observations.get(0).size(),
        "Sparse observation exports include consistent residual columns");
  }

  private static void stereo(Path run) throws IOException {
    Path dir = run.resolve("05_stereo/pair_0000_0001");
    float[] selected = pfm(dir.resolve("disparity/00_left_selected.pfm")),
        filtered = pfm(dir.resolve("disparity/05_final_stereo.pfm"));
    Mat reason =
        Imgcodecs.imread(
            dir.resolve("disparity/rejection_codes.png").toString(), Imgcodecs.IMREAD_GRAYSCALE);
    boolean codes = true;
    int rejected = 0;
    try {
      byte[] bytes = new byte[filtered.length];
      reason.get(0, 0, bytes);
      for (int i = 0; i < bytes.length; i++) {
        codes &= (bytes[i] == 0) == Float.isFinite(filtered[i]);
        if (Float.isFinite(selected[i]) && !Float.isFinite(filtered[i])) rejected++;
      }
    } finally {
      reason.release();
    }
    check(codes && rejected > 0, "Per-pixel rejection codes agree with the final disparity mask");
    check(
        Files.isRegularFile(dir.resolve("disparity/zncc.pfm"))
            && Files.isRegularFile(dir.resolve("disparity/left_right_error.pfm"))
            && Files.isRegularFile(dir.resolve("triangulation_angle_degrees.pfm")),
        "Photometric, reciprocity, and parallax evidence are exported numerically");
    Path vertical = run.getParent().getParent().getParent().resolve("vertical result");
    Path vr = latest(vertical, "latest_successful_run");
    float[] map = pfm(vr.resolve("05_stereo/pair_0000_0001/left_source_x.pfm"));
    check(
        Math.abs(map[200 * 300 + 150] - 200) < .01,
        "Vertical-pair maps follow matching raster transposition while values retain source x"
            + " coordinates");
  }

  private static void history(Path fixtures) throws Exception {
    Path project = fixtures.resolve("known poses result"),
        previous = latest(project, "latest_successful_run");
    String manifestHash = Project.sha256(previous.resolve("run.json")),
        cloudHash = Project.sha256(project.resolve("dense.las"));
    Path one = Files.createDirectory(fixtures.resolve("one image"));
    Files.copy(fixtures.resolve("input images/00.png"), one.resolve("only.png"));
    int code =
        TestSupport.run(
            new String[] {
              "-i",
              one.toString(),
              "-o",
              project.toString(),
              "--calibration",
              fixtures.resolve("camera.txt").toString()
            });
    check(code == 2, "A failed invocation is reported as failure");
    Path failed = latest(project, "latest_run");
    check(
        !failed.equals(previous) && latest(project, "latest_successful_run").equals(previous),
        "Failure preserves the latest successful run pointer");
    check(
        Project.sha256(project.resolve("dense.las")).equals(cloudHash)
            && Project.sha256(previous.resolve("run.json")).equals(manifestHash),
        "Failure never replaces prior result aliases or rewrites historical artifacts");
    check(
        Files.readString(failed.resolve("logs/console.log")).contains("At least 2 input images")
            && Files.readString(failed.resolve("logs/error.txt"))
                .contains("At least 2 input images"),
        "Failure logs preserve both the message and stack trace");
    check(
        rows(failed.resolve("stages.csv")).stream()
            .anyMatch(r -> r.get(0).equals("00_inputs") && r.get(1).equals("failed")),
        "The stage that failed is identified");
    code =
        TestSupport.run(
            new String[] {
              "-i",
              fixtures.resolve("input images").toString(),
              "-o",
              project.toString(),
              "--calibration",
              fixtures.resolve("camera.txt").toString(),
              "--poses",
              fixtures.resolve("input poses.csv").toString(),
              "--disparities",
              "40",
              "--step",
              "4",
              "--cores",
              "1",
              "--copy-inputs"
            });
    check(
        code == 0 && !latest(project, "latest_successful_run").equals(previous),
        "An existing project accepts a new successful run without destructive overwrite");
    Path latest = latest(project, "latest_successful_run");
    try (java.util.stream.Stream<Path> children = Files.list(project.resolve("runs"))) {
      check(children.count() == 3, "Successful and failed runs are retained in project history");
    }
    List<List<String>> inputs = rows(latest.resolve("00_inputs/images.csv"));
    boolean copied = true;
    for (int i = 1; i < inputs.size(); i++) {
      List<String> r = inputs.get(i);
      copied &= Project.sha256(latest.resolve("00_inputs").resolve(r.get(10))).equals(r.get(4));
    }
    check(copied, "--copy-inputs archives byte-identical original photographs");
    check(
        rows(latest.resolve("stages.csv")).stream()
            .anyMatch(r -> r.get(0).equals("02_features") && r.get(1).equals("skipped")),
        "Supplied-pose runs explicitly mark feature extraction as skipped");
    Path legacy = Files.createDirectory(fixtures.resolve("adopt existing directory"));
    Files.writeString(legacy.resolve("notes.txt"), "retain me");
    Files.writeString(legacy.resolve("readme.md"), "preserve my existing README");
    String[] args = {
      "-i",
      fixtures.resolve("rectified images").toString(),
      "-o",
      legacy.toString(),
      "--calibration",
      fixtures.resolve("camera.txt").toString(),
      "--rectified",
      "--baseline",
      ".25",
      "--disparities",
      "32",
      "--step",
      "4",
      "--scale",
      "2",
      "--cores",
      "1"
    };
    check(
        TestSupport.run(args) == 2 && !Files.exists(legacy.resolve("project.properties")),
        "An unrelated nonempty directory is not silently adopted");
    String[] adopt = Arrays.copyOf(args, args.length + 1);
    adopt[args.length] = "--overwrite";
    check(
        TestSupport.run(adopt) == 0
            && Files.readString(legacy.resolve("notes.txt")).equals("retain me"),
        "Explicit adoption preserves unrelated files");
    Path run = latest(legacy, "latest_successful_run");
    check(
        Files.readString(run.resolve("00_inputs/adopted_root_files/readme.md"))
            .equals("preserve my existing README"),
        "Adoption archives colliding reserved filenames before replacing project indexes");
    List<String> first = rows(run.resolve("06_fusion/candidates.csv.gz")).get(1);
    ByteBuffer las =
        ByteBuffer.wrap(Files.readAllBytes(legacy.resolve("dense.las")))
            .order(ByteOrder.LITTLE_ENDIAN);
    double z = las.getInt(las.getInt(96) + 8) * las.getDouble(147) + las.getDouble(171);
    check(
        Math.abs(z - 2 * Double.parseDouble(first.get(7))) < .00011,
        "Final coordinates apply --scale while candidate geometry stays in unscaled reconstruction"
            + " units");
    check(
        Project.sha256(legacy.resolve("dense.las"))
            .equals(Project.sha256(run.resolve("07_results/dense.las"))),
        "Convenience root exports are byte-identical to the canonical run results");
    List<List<String>> aliases = rows(legacy.resolve("latest-files.csv"));
    check(
        aliases.stream()
            .anyMatch(
                r ->
                    r.get(0).equals("dense.las")
                        && r.get(1).equals(run.getFileName().toString())
                        && r.get(3).equals(uncheckedHash(legacy.resolve("dense.las")))),
        "Root compatibility copies identify their exact source run and checksum");
  }

  private static void calibration(Path fixtures) throws IOException {
    Path run = latest(fixtures.resolve("calibration result"), "latest_successful_run");
    check(
        Files.isRegularFile(run.resolve("01_calibration/calibration.json"))
            && Files.isRegularFile(run.resolve("01_calibration/reprojection.csv")),
        "Calibration retains board exterior poses and corner residuals");
    check(
        rows(run.resolve("01_calibration/views/image_0000/corners.csv")).size() == 36,
        "All detected chessboard corners are retained with object-space coordinates");
    check(
        Files.isRegularFile(run.resolve("logs/run-settings.properties")),
        "Calibration-only runs also retain their effective default settings");
  }

  private static String uncheckedHash(Path path) {
    try {
      return Project.sha256(path);
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  private static float[] pfm(Path path) throws IOException {
    byte[] bytes = Files.readAllBytes(path);
    int p = 0;
    String[] header = new String[3];
    for (int i = 0; i < 3; i++) {
      int start = p;
      while (bytes[p++] != '\n') {}
      header[i] =
          new String(bytes, start, p - start - 1, java.nio.charset.StandardCharsets.US_ASCII);
    }
    String[] size = header[1].split(" ");
    int width = Integer.parseInt(size[0]), height = Integer.parseInt(size[1]);
    ByteBuffer b = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN);
    float[] values = new float[width * height];
    for (int y = height - 1; y >= 0; y--)
      for (int x = 0; x < width; x++) {
        values[y * width + x] = b.getFloat(p);
        p += 4;
      }
    return values;
  }
}
