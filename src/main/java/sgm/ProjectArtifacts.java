package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import java.util.zip.GZIPOutputStream;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

/** Lossless numerical exports and deterministic previews. Exporting never consumes OpenCV RNG. */
final class ProjectArtifacts implements AutoCloseable {
  private final Project project;
  private final Options options;
  private final List<String> sourceRows = new ArrayList<>(), pairRows = new ArrayList<>();
  private final Set<Integer> inventoriedImages = new HashSet<>();
  private BufferedWriter candidates, decisions, refinements;
  private final Map<Integer, PairRaster> rasters = new LinkedHashMap<>();
  private List<ImageFrame> frames = Collections.emptyList();
  private int pnpAttempts;
  private Calibration sourceCalibration;

  ProjectArtifacts(Project project, Options options) {
    this.project = project;
    this.options = options;
  }

  Path directory(String relative) throws IOException {
    Path p = project.run.resolve(relative).normalize();
    if (!p.startsWith(project.run)) throw new IOException("Artifact escapes its run");
    Files.createDirectories(p);
    return p;
  }

  void denseInputs(List<ImageFrame> dense) throws IOException {
    frames = dense;
    for (ImageFrame frame : dense) {
      if ((options.diagnostics && !options.storage.equals("minimal"))) frame.ensureRasters();
      Path dir = directory("00_inputs/dense/" + imageId(frame.id));
      if ((options.diagnostics && !options.storage.equals("minimal")))
        image(dir.resolve("undistorted_color.png"), frame.color);
      if ((options.diagnostics && options.storage.equals("debug"))) {
        image(dir.resolve("undistorted_gray.png"), frame.gray);
        image(dir.resolve("valid_mask.png"), frame.validMask);
      }
      Json.write(
          dir.resolve("image.json"),
          Json.object(
              "image_id",
              frame.id,
              "source_reader",
              frame.sourceReader(),
              "source_window_side",
              frame.sourceReader().equals("imageio_source_windows") ? options.imageWindow : null,
              "processed_size",
              new int[] {frame.width, frame.height},
              "fx",
              frame.camera.fx,
              "fy",
              frame.camera.fy,
              "cx",
              frame.camera.cx,
              "cy",
              frame.camera.cy,
              "pixel_frame",
              "dense processed-undistorted; alignment keypoints retain their original frame"));
    }
  }

  static String csv(Object value) {
    if (value == null) return "";
    return PoseIO.quote(String.valueOf(value));
  }

  private static void image(Path path, Mat data) throws IOException {
    if (!Imgcodecs.imwrite(path.toString(), data))
      throw new IOException("Cannot write artifact " + path);
  }

  private static BufferedWriter gzip(Path path) throws IOException {
    return new BufferedWriter(
        new OutputStreamWriter(
            new GZIPOutputStream(Files.newOutputStream(path)),
            java.nio.charset.StandardCharsets.UTF_8));
  }

  static String imageId(int id) {
    return String.format(Locale.ROOT, "image_%04d", id);
  }

  static String pairId(int a, int b) {
    return String.format(Locale.ROOT, "pair_%04d_%04d", a, b);
  }

  void source(int id, Path path, int width, int height, Exif.Gps gps) throws IOException {
    if (!inventoriedImages.add(id)) return;
    if (sourceRows.isEmpty())
      sourceRows.add(
          "image_id,filename,original_path,bytes,sha256,original_width,original_height,gps_latitude_deg,gps_longitude_deg,gps_altitude_metres,archived_original");
    String archived = "";
    if (options.copyInputs) {
      Path originals = directory("00_inputs/originals");
      String name = path.getFileName().toString();
      int dot = name.lastIndexOf('.');
      String target = imageId(id) + (dot >= 0 ? name.substring(dot) : "");
      Files.copy(path, originals.resolve(target), StandardCopyOption.REPLACE_EXISTING);
      archived = "originals/" + target;
    }
    sourceRows.add(
        id
            + ","
            + csv(path.getFileName())
            + ","
            + csv(path.toAbsolutePath().normalize())
            + ","
            + Files.size(path)
            + ","
            + Project.sha256(path)
            + ","
            + width
            + ","
            + height
            + ","
            + (gps == null ? ",," : gps.latitude + "," + gps.longitude + "," + gps.altitude)
            + ","
            + csv(archived));
    Files.write(project.run.resolve("00_inputs/images.csv"), sourceRows);
    Path acquisition = project.run.resolve("00_inputs/acquisition.json");
    List<Object> timestamps =
        Files.isRegularFile(acquisition)
            ? new ArrayList<>(JsonRead.array(JsonRead.read(acquisition)))
            : new ArrayList<>();
    Exif.Metadata metadata = Exif.read(path);
    timestamps.add(
        Json.object(
            "filename",
            path.getFileName().toString(),
            "capture_time_local",
            metadata.captureTime,
            "capture_offset",
            metadata.captureOffset,
            "camera_model",
            metadata.make + "/" + metadata.model));
    Json.write(acquisition, timestamps);
  }

  /** A later pipeline step (new process): the same in-memory context, nothing rewritten. */
  void restore(List<ImageFrame> frames, Calibration calibration) {
    this.frames = frames;
    if (calibration != null) sourceCalibration = calibration;
  }

  void inputs(List<ImageFrame> frames, Calibration calibration) throws IOException {
    this.frames = frames;
    sourceCalibration = calibration;
    for (ImageFrame frame : frames) {
      if ((options.diagnostics && !options.storage.equals("minimal"))) frame.ensureRasters();
      source(frame.id, frame.path, frame.sourceWidth, frame.sourceHeight, frame.gps);
      Path dir = directory("00_inputs/" + imageId(frame.id));
      if ((options.diagnostics && !options.storage.equals("minimal")))
        image(dir.resolve("undistorted_color.png"), frame.color);
      if ((options.diagnostics && options.storage.equals("debug"))) {
        image(dir.resolve("undistorted_gray.png"), frame.gray);
        image(dir.resolve("valid_mask.png"), frame.validMask);
      }
      Camera c = frame.camera;
      Json.write(
          dir.resolve("image.json"),
          Json.object(
              "image_id",
              frame.id,
              "filename",
              frame.path.getFileName().toString(),
              "original_size",
              new int[] {frame.sourceWidth, frame.sourceHeight},
              "source_reader",
              frame.sourceReader(),
              "source_window_side",
              frame.sourceReader().equals("imageio_source_windows") ? options.imageWindow : null,
              "processed_size",
              new int[] {frame.width, frame.height},
              "pixel_coordinates",
              "zero-based processed undistorted raster, x right, y down",
              "distortion_applied",
              !options.rectified,
              "effective_distortion",
              new double[5],
              "K",
              new double[][] {{c.fx, 0, c.cx}, {0, c.fy, c.cy}, {0, 0, 1}}));
      Mat k = c.intrinsic(),
          r = Mat.eye(3, 3, CvType.CV_64F),
          d = new MatOfDouble(frame.calibration.distortion),
          mx = new Mat(),
          my = new Mat();
      try {
        if (!options.rectified && (options.diagnostics && options.storage.equals("debug"))) {
          Calib3d.initUndistortRectifyMap(
              k, d, r, k, new Size(frame.width, frame.height), CvType.CV_32FC1, mx, my);
          pfm(dir.resolve("undistort_source_x.pfm"), mx);
          pfm(dir.resolve("undistort_source_y.pfm"), my);
        }
      } finally {
        k.release();
        r.release();
        d.release();
        mx.release();
        my.release();
      }
    }
    referenceCalibration(calibration);
    if (options.poses != null)
      Files.copy(
          options.poses,
          project.run.resolve("00_inputs/supplied_poses.csv"),
          StandardCopyOption.REPLACE_EXISTING);
  }

  void referenceCalibration(Calibration calibration) throws IOException {
    Path dir = directory("01_calibration");
    if (options.calibration != null)
      Files.copy(
          options.calibration,
          dir.resolve("supplied_calibration.txt"),
          StandardCopyOption.REPLACE_EXISTING);
    Json.write(
        dir.resolve("reference_interior.json"),
        Json.object(
            "reference_size",
            new int[] {calibration.width, calibration.height},
            "fx",
            calibration.fx,
            "fy",
            calibration.fy,
            "cx",
            calibration.cx,
            "cy",
            calibration.cy,
            "distortion_order",
            new String[] {"k1", "k2", "p1", "p2", "k3"},
            "distortion",
            calibration.distortion,
            "model",
            "OpenCV pinhole; zero skew",
            "units",
            "focal lengths and principal point in pixels; distortion dimensionless"));
    Properties p = new Properties();
    p.setProperty("width", String.valueOf(calibration.width));
    p.setProperty("height", String.valueOf(calibration.height));
    p.setProperty("fx", String.valueOf(calibration.fx));
    p.setProperty("fy", String.valueOf(calibration.fy));
    p.setProperty("cx", String.valueOf(calibration.cx));
    p.setProperty("cy", String.valueOf(calibration.cy));
    String[] keys = {"k1", "k2", "p1", "p2", "k3"};
    for (int i = 0; i < 5; i++) p.setProperty(keys[i], String.valueOf(calibration.distortion[i]));
    try (Writer w = Files.newBufferedWriter(dir.resolve("reference_camera.properties"))) {
      p.store(w, "Original calibration at its reference resolution");
    }
    interior(dir.resolve("processed_interior.csv"), frames);
  }

  private static void interior(Path path, List<ImageFrame> frames) throws IOException {
    try (BufferedWriter w = Files.newBufferedWriter(path)) {
      w.write("image_id,filename,width,height,fx,fy,cx,cy,skew,k1,k2,p1,p2,k3,pixel_frame\n");
      for (ImageFrame f : frames) {
        Camera c = f.camera;
        w.write(
            f.id
                + ","
                + csv(f.path.getFileName())
                + ","
                + f.width
                + ","
                + f.height
                + ","
                + c.fx
                + ","
                + c.fy
                + ","
                + c.cx
                + ","
                + c.cy
                + ",0,0,0,0,0,0,processed_undistorted\n");
      }
    }
  }

  void features(ImageFrame frame) throws IOException {
    if (!options.diagnostics || options.storage.equals("minimal")) return;
    Path dir = directory("02_features/" + imageId(frame.id));
    try (BufferedWriter w = Files.newBufferedWriter(dir.resolve("keypoints.csv"))) {
      w.write(
          "keypoint_id,x,y,size,angle_degrees,response,octave,class_id,original_raster_u,original_raster_v\n");
      for (int i = 0; i < frame.keypoints.length; i++) {
        KeyPoint k = frame.keypoints[i];
        double[] raw = originalPixel(frame, k.pt);
        w.write(
            i
                + ","
                + k.pt.x
                + ","
                + k.pt.y
                + ","
                + k.size
                + ","
                + k.angle
                + ","
                + k.response
                + ","
                + k.octave
                + ","
                + k.class_id
                + ","
                + raw[0]
                + ","
                + raw[1]
                + "\n");
      }
    }
    if ((options.diagnostics && options.storage.equals("debug"))) {
      try (DataOutputStream out =
          new DataOutputStream(
              new BufferedOutputStream(Files.newOutputStream(dir.resolve("descriptors.f32"))))) {
        out.writeBytes("SGMDESC1");
        out.writeInt(frame.descriptors.rows());
        out.writeInt(frame.descriptors.cols());
        Mat floating = new Mat();
        frame.descriptors.convertTo(floating, CvType.CV_32F);
        float[] row = new float[frame.descriptors.cols()];
        try {
          for (int i = 0; i < frame.descriptors.rows(); i++) {
            floating.get(i, 0, row);
            for (float v : row) out.writeFloat(v);
          }
        } finally {
          floating.release();
        }
      }
    }
    Json.write(
        dir.resolve("features.json"),
        Json.object(
            "descriptors_exported",
            (options.diagnostics && options.storage.equals("debug")),
            "detector",
            frame.options.matchingAlgorithm,
            "native_descriptor_type",
            CvType.typeToString(frame.descriptors.type()),
            "distance_metric",
            frame.descriptors.depth() == CvType.CV_8U ? "Hamming" : "L2",
            "descriptor_storage_note",
            "Binary descriptors are exported as exact byte values in float32 containers; restore"
                + " CV_8U before Hamming matching",
            "image_id",
            frame.id,
            "count",
            frame.keypoints.length,
            "pixel_frame",
            "processed undistorted image",
            "descriptor_format",
            "SGMDESC1: 8 ASCII bytes, big-endian int32 rows, int32 columns, row-major big-endian"
                + " float32; row index equals keypoint_id",
            "preview",
            "all keypoint centers; every coordinate is in keypoints.csv"));
    frame.ensureRasters();
    Mat overlay = frame.color.clone();
    try {
      for (KeyPoint k : frame.keypoints)
        Imgproc.circle(overlay, k.pt, 2, new Scalar(0, 255, 255), 1, Imgproc.LINE_AA);
      image(dir.resolve("keypoints.png"), overlay);
    } finally {
      overlay.release();
    }
  }

  void matches(
      ImageFrame a,
      ImageFrame b,
      FeatureTracks.Pair pair,
      List<MatOfDMatch> forward,
      Map<Integer, Integer> reverse,
      Set<Integer> ransac,
      Set<Integer> cheirality)
      throws IOException {
    if (!(options.diagnostics && options.storage.equals("debug"))) {
      compactMatches(a, b, pair, forward.size(), ransac.size(), cheirality.size());
      return;
    }
    Path dir = directory("03_matches/" + pairId(a.id, b.id));
    boolean used = pair.matches.size() >= options.minMatches;
    Set<Integer> ratioAccepted = new HashSet<>(), finalAccepted = new HashSet<>();
    for (FeatureTracks.Match m : pair.matches) finalAccepted.add(m.a);
    List<int[]> ratioPreview = new ArrayList<>(),
        inlierPreview = new ArrayList<>(),
        rejectedPreview = new ArrayList<>();
    try (BufferedWriter w = gzip(dir.resolve("matches.csv.gz"))) {
      w.write(
          "query_keypoint_id,best_train_keypoint_id,best_l2_distance,second_train_keypoint_id,second_l2_distance,ratio,ratio_pass,reciprocal_pass,essential_ransac_inlier,cheirality_inlier,used_for_tracks,rejection_stage\n");
      for (MatOfDMatch row : forward) {
        DMatch[] matches = row.toArray();
        if (matches.length == 0) continue;
        DMatch m = matches[0];
        boolean ratio = matches.length > 1 && m.distance < options.ratio * matches[1].distance,
            reciprocal = ratio && Objects.equals(reverse.get(m.trainIdx), m.queryIdx);
        if (reciprocal) {
          ratioAccepted.add(m.queryIdx);
          ratioPreview.add(new int[] {m.queryIdx, m.trainIdx});
        }
        boolean inlier = finalAccepted.contains(m.queryIdx);
        if (inlier) inlierPreview.add(new int[] {m.queryIdx, m.trainIdx});
        else rejectedPreview.add(new int[] {m.queryIdx, m.trainIdx});
        String reason =
            matches.length < 2
                ? "no_second_neighbor"
                : !ratio
                    ? "ratio"
                    : !reciprocal
                        ? "reciprocity"
                        : !ransac.contains(m.queryIdx)
                            ? "essential_ransac"
                            : !cheirality.contains(m.queryIdx)
                                ? "cheirality"
                                : !used ? "pair_minimum_inliers" : "accepted";
        w.write(
            m.queryIdx
                + ","
                + m.trainIdx
                + ","
                + m.distance
                + ","
                + (matches.length > 1 ? matches[1].trainIdx : "")
                + ","
                + (matches.length > 1 ? matches[1].distance : "")
                + ","
                + (matches.length > 1 && matches[1].distance > 0
                    ? m.distance / matches[1].distance
                    : "")
                + ","
                + ratio
                + ","
                + reciprocal
                + ","
                + ransac.contains(m.queryIdx)
                + ","
                + cheirality.contains(m.queryIdx)
                + ","
                + (used && inlier)
                + ","
                + reason
                + "\n");
      }
    }
    if (pairRows.isEmpty())
      pairRows.add(
          "image_a,image_b,forward_knn_queries,reciprocal_ratio_matches,essential_ransac_inliers,cheirality_inliers,used_for_reconstruction,status");
    pairRows.add(
        a.id
            + ","
            + b.id
            + ","
            + forward.size()
            + ","
            + ratioAccepted.size()
            + ","
            + ransac.size()
            + ","
            + cheirality.size()
            + ","
            + used
            + ","
            + csv(pair.reason));
    Files.write(project.run.resolve("03_matches/pairs.csv"), pairRows);
    double[]
        ka =
            {
              1 / a.camera.fx,
              0,
              -a.camera.cx / a.camera.fx,
              0,
              1 / a.camera.fy,
              -a.camera.cy / a.camera.fy,
              0,
              0,
              1
            },
        kb =
            {
              1 / b.camera.fx,
              0,
              -b.camera.cx / b.camera.fx,
              0,
              1 / b.camera.fy,
              -b.camera.cy / b.camera.fy,
              0,
              0,
              1
            };
    double[] fundamental =
        pair.essential == null
            ? null
            : Math3.mul(Math3.transpose(kb), Math3.mul(pair.essential, ka));
    Json.write(
        dir.resolve("pair.json"),
        Json.object(
            "image_a",
            a.id,
            "image_b",
            b.id,
            "status",
            pair.reason,
            "used_for_reconstruction",
            used,
            "E_normalized",
            matrix(pair.essential, 3, 3),
            "F_processed_undistorted_pixels",
            matrix(fundamental, 3, 3),
            "R_b_from_a",
            matrix(pair.rotation, 3, 3),
            "translation_b_from_a_unit_baseline",
            pair.translation,
            "preview_max_matches",
            300,
            "preview_sampling",
            "deterministic evenly spaced; numeric CSV contains all forward nearest-neighbor"
                + " decisions"));
    previewMatches(dir.resolve("ratio_matches.png"), a, b, ratioPreview, new Scalar(255, 200, 0));
    previewMatches(
        dir.resolve("geometric_inliers.png"), a, b, inlierPreview, new Scalar(0, 255, 0));
    previewMatches(
        dir.resolve("rejected_matches.png"), a, b, rejectedPreview, new Scalar(0, 0, 255));
  }

  private static void previewMatches(
      Path path, ImageFrame a, ImageFrame b, List<int[]> matches, Scalar color) throws IOException {
    RasterCache.pair(a, b);
    Mat canvas =
        new Mat(
            Math.max(a.color.rows(), b.color.rows()),
            a.color.cols() + b.color.cols(),
            CvType.CV_8UC3,
            new Scalar(0));
    Mat left = canvas.submat(0, a.color.rows(), 0, a.color.cols()),
        right = canvas.submat(0, b.color.rows(), a.color.cols(), a.color.cols() + b.color.cols());
    try {
      a.color.copyTo(left);
      b.color.copyTo(right);
      int n = Math.min(300, matches.size());
      for (int i = 0; i < n; i++) {
        int[] m = matches.get((int) ((long) i * matches.size() / n));
        Point p = a.keypoints[m[0]].pt, q = b.keypoints[m[1]].pt;
        Point shifted = new Point(q.x + a.color.cols(), q.y);
        Imgproc.line(canvas, p, shifted, color, 1, Imgproc.LINE_AA);
        Imgproc.circle(canvas, p, 2, color, 1);
        Imgproc.circle(canvas, shifted, 2, color, 1);
      }
      image(path, canvas);
    } finally {
      left.release();
      right.release();
      canvas.release();
    }
  }

  private void compactMatches(
      ImageFrame a, ImageFrame b, FeatureTracks.Pair pair, int queries, int ransac, int cheirality)
      throws IOException {
    if (pairRows.isEmpty())
      pairRows.add(
          "image_a,image_b,queries,ransac_inliers,cheirality_inliers,accepted_matches,status");
    pairRows.add(
        a.id
            + ","
            + b.id
            + ","
            + queries
            + ","
            + ransac
            + ","
            + cheirality
            + ","
            + pair.matches.size()
            + ","
            + csv(pair.reason));
    Files.write(project.run.resolve("03_matches/pairs.csv"), pairRows);
    if (!options.diagnostics || options.storage.equals("minimal")) return;
    Path dir = directory("03_matches/" + pairId(a.id, b.id));
    try (BufferedWriter out = gzip(dir.resolve("accepted_matches.csv.gz"))) {
      out.write("query_keypoint_id,train_keypoint_id\n");
      for (var match : pair.matches) out.write(match.a + "," + match.b + "\n");
    }
    List<int[]> matches = new ArrayList<>();
    for (var match : pair.matches) matches.add(new int[] {match.a, match.b});
    previewMatches(dir.resolve("accepted_matches.png"), a, b, matches, new Scalar(0, 255, 0));
    Json.write(
        dir.resolve("pair.json"),
        Json.object(
            "image_a",
            a.id,
            "image_b",
            b.id,
            "status",
            pair.reason,
            "essential",
            pair.essential,
            "R",
            pair.rotation,
            "t",
            pair.translation));
  }

  void snapshot(
      String name, List<ImageFrame> frames, List<FeatureTracks.Track> tracks, String coordinates)
      throws IOException {
    if (!(options.diagnostics && options.storage.equals("debug")) && !name.equals("final")) return;
    Path dir = directory("04_geometry/" + name);
    interior(dir.resolve("interior.csv"), frames);
    List<Object> cameras = new ArrayList<>();
    try (BufferedWriter w = Files.newBufferedWriter(dir.resolve("exterior.csv"))) {
      w.write(
          "image_id,filename,registered,fixed,r00,r01,r02,r10,r11,r12,r20,r21,r22,tx,ty,tz,center_x,center_y,center_z,rodrigues_x,rodrigues_y,rodrigues_z\n");
      for (ImageFrame f : frames) {
        Camera c = f.camera;
        double[] center = c.center(),
            rv = Math3.rotationVector(c.rotation),
            rt = Math3.transpose(c.rotation);
        w.write(f.id + "," + csv(f.path.getFileName()) + "," + c.registered + "," + c.fixed);
        for (double v : c.rotation) w.write("," + v);
        for (double v : c.translation) w.write("," + v);
        for (double v : center) w.write("," + v);
        for (double v : rv) w.write("," + v);
        w.newLine();
        double[][]
            worldToCamera =
                {
                  {c.rotation[0], c.rotation[1], c.rotation[2], c.translation[0]},
                  {c.rotation[3], c.rotation[4], c.rotation[5], c.translation[1]},
                  {c.rotation[6], c.rotation[7], c.rotation[8], c.translation[2]},
                  {0, 0, 0, 1}
                },
            cameraToWorld =
                {
                  {rt[0], rt[1], rt[2], center[0]},
                  {rt[3], rt[4], rt[5], center[1]},
                  {rt[6], rt[7], rt[8], center[2]},
                  {0, 0, 0, 1}
                };
        Mat projection = c.projection(),
            k = c.intrinsic(),
            pixelProjection = new Mat(),
            empty = new Mat();
        try {
          Core.gemm(k, projection, 1, empty, 0, pixelProjection);
          cameras.add(
              Json.object(
                  "image_id",
                  f.id,
                  "filename",
                  f.path.getFileName().toString(),
                  "registered",
                  c.registered,
                  "fixed",
                  c.fixed,
                  "R_world_to_camera",
                  matrix(c.rotation, 3, 3),
                  "t_world_to_camera",
                  c.translation,
                  "camera_center_world",
                  center,
                  "rodrigues_radians",
                  rv,
                  "world_to_camera_4x4",
                  worldToCamera,
                  "camera_to_world_4x4",
                  cameraToWorld,
                  "pixel_projection_K_Rt",
                  matrix(Math3.array(pixelProjection), 3, 4)));
        } finally {
          projection.release();
          k.release();
          pixelProjection.release();
          empty.release();
        }
      }
    }
    Json.write(
        dir.resolve("orientations.json"),
        Json.object(
            "coordinates",
            coordinates,
            "convention",
            "X_camera = R X_world + t; C_world = -transpose(R) t; camera axes x right, y down, z"
                + " forward; no omega/phi/kappa convention assumed",
            "coordinate_multiplier_applied",
            false,
            "cameras",
            cameras));
    writeTracks(dir, frames, tracks);
    cameraGeometry(dir, frames);
  }

  void tracks(
      List<FeatureTracks.Track> tracks,
      Map<Integer, FeatureTracks.Track> groups,
      int conflicts,
      int singletons)
      throws IOException {
    if (!(options.diagnostics && options.storage.equals("debug"))) return;
    Path dir = directory("03_matches");
    Json.write(
        dir.resolve("track_build.json"),
        Json.object(
            "connected_components",
            groups.size(),
            "conflicting_components_rejected",
            conflicts,
            "singleton_components_rejected",
            singletons,
            "retained_tracks",
            tracks.size(),
            "rule",
            "at most one keypoint per image in a track"));
    try (BufferedWriter w = Files.newBufferedWriter(dir.resolve("track_observations.csv"))) {
      w.write("track_id,image_id,keypoint_id\n");
      for (int i = 0; i < tracks.size(); i++) {
        FeatureTracks.Track track = tracks.get(i);
        track.id = i;
        for (FeatureTracks.Observation o : track.observations)
          w.write(i + "," + o.image + "," + o.feature + "\n");
      }
    }
    Set<FeatureTracks.Track> retained = Collections.newSetFromMap(new IdentityHashMap<>());
    retained.addAll(tracks);
    try (BufferedWriter w = gzip(dir.resolve("all_track_components.csv.gz"))) {
      w.write("component_id,retained_track_id,image_id,keypoint_id,component_status\n");
      for (Map.Entry<Integer, FeatureTracks.Track> entry : groups.entrySet()) {
        FeatureTracks.Track track = entry.getValue();
        String status =
            retained.contains(track)
                ? "retained"
                : track.observations.size() < 2 ? "singleton" : "conflicting_image_observations";
        for (FeatureTracks.Observation o : track.observations)
          w.write(
              entry.getKey()
                  + ","
                  + (retained.contains(track) ? track.id : "")
                  + ","
                  + o.image
                  + ","
                  + o.feature
                  + ","
                  + status
                  + "\n");
      }
    }
  }

  private static void writeTracks(
      Path dir, List<ImageFrame> frames, List<FeatureTracks.Track> tracks) throws IOException {
    long valid = tracks.stream().filter(t -> t.point != null).count();
    try (BufferedWriter points = Files.newBufferedWriter(dir.resolve("tracks.csv"));
        BufferedWriter observations = Files.newBufferedWriter(dir.resolve("observations.csv"));
        BufferedWriter ply = Files.newBufferedWriter(dir.resolve("sparse.ply"))) {
      points.write("track_id,triangulated,x,y,z,observations\n");
      observations.write(
          "track_id,image_id,keypoint_id,u,v,image_registered,predicted_u,predicted_v,residual_u,residual_v,residual_norm,positive_depth\n");
      ply.write(
          "ply\nformat ascii 1.0\nelement vertex "
              + valid
              + "\n"
              + "property double x\n"
              + "property double y\n"
              + "property double z\n"
              + "property uchar red\n"
              + "property uchar green\n"
              + "property uchar blue\n"
              + "end_header\n");
      for (int i = 0; i < tracks.size(); i++) {
        FeatureTracks.Track t = tracks.get(i);
        if (t.id < 0) t.id = i;
        points.write(
            t.id
                + ","
                + (t.point != null)
                + ","
                + (t.point == null ? ",," : t.point[0] + "," + t.point[1] + "," + t.point[2])
                + ","
                + t.observations.size()
                + "\n");
        if (t.point != null) {
          FeatureTracks.Observation origin = t.observations.get(0);
          ImageFrame f = frames.get(origin.image);
          Point uv = f.keypoints[origin.feature].pt;
          if (f.featureColors == null) f.captureFeatureColors();
          double[] color = {
            f.featureColors[origin.feature * 3] & 255,
            f.featureColors[origin.feature * 3 + 1] & 255,
            f.featureColors[origin.feature * 3 + 2] & 255
          };
          ply.write(
              t.point[0]
                  + " "
                  + t.point[1]
                  + " "
                  + t.point[2]
                  + " "
                  + (int) color[2]
                  + " "
                  + (int) color[1]
                  + " "
                  + (int) color[0]
                  + "\n");
        }
        for (FeatureTracks.Observation o : t.observations) {
          ImageFrame f = frames.get(o.image);
          Point uv = f.keypoints[o.feature].pt;
          double[] predicted =
              t.point != null && f.camera.registered ? f.camera.project(t.point) : null;
          observations.write(
              t.id
                  + ","
                  + o.image
                  + ","
                  + o.feature
                  + ","
                  + uv.x
                  + ","
                  + uv.y
                  + ","
                  + f.camera.registered
                  + ","
                  + (predicted == null
                      ? ",,,,"
                      : predicted[0]
                          + ","
                          + predicted[1]
                          + ","
                          + (predicted[0] - uv.x)
                          + ","
                          + (predicted[1] - uv.y)
                          + ","
                          + Math.hypot(predicted[0] - uv.x, predicted[1] - uv.y))
                  + ","
                  + (t.point != null && f.camera.registered && f.camera.view(t.point)[2] > 0)
                  + "\n");
        }
      }
    }
  }

  private static void cameraGeometry(Path dir, List<ImageFrame> frames) throws IOException {
    List<ImageFrame> active = new ArrayList<>();
    for (ImageFrame f : frames) if (f.camera.registered) active.add(f);
    double extent = 1;
    if (active.size() > 1) {
      extent = 0;
      for (int i = 1; i < active.size(); i++)
        extent +=
            Math3.norm(Math3.sub(active.get(i).camera.center(), active.get(0).camera.center()));
      extent = Math.max(1e-6, extent / (active.size() - 1) * .15);
    }
    try (BufferedWriter centers = Files.newBufferedWriter(dir.resolve("camera_centers.ply"))) {
      centers.write(
          "ply\nformat ascii 1.0\nelement vertex "
              + active.size()
              + "\n"
              + "property double x\n"
              + "property double y\n"
              + "property double z\n"
              + "property int image_id\n"
              + "end_header\n");
      for (ImageFrame f : active) {
        double[] c = f.camera.center();
        centers.write(c[0] + " " + c[1] + " " + c[2] + " " + f.id + "\n");
      }
    }
    try (BufferedWriter w = Files.newBufferedWriter(dir.resolve("camera_frustums.ply"))) {
      w.write(
          "ply\nformat ascii 1.0\ncomment frustum display depth="
              + extent
              + " in reconstruction units\nelement vertex "
              + active.size() * 5
              + "\nproperty double x\nproperty double y\nproperty double z\nelement edge "
              + active.size() * 8
              + "\nproperty int vertex1\nproperty int vertex2\nend_header\n");
      for (ImageFrame f : active) {
        double[] center = f.camera.center();
        w.write(center[0] + " " + center[1] + " " + center[2] + "\n");
        for (double[] uv :
            new double[][] {
              {0, 0},
              {f.width - 1, 0},
              {f.width - 1, f.height - 1},
              {0, f.height - 1}
            }) {
          double[] local = {
            (uv[0] - f.camera.cx) / f.camera.fx * extent,
            (uv[1] - f.camera.cy) / f.camera.fy * extent,
            extent
          };
          double[] p = Math3.add(center, Math3.mv(Math3.transpose(f.camera.rotation), local));
          w.write(p[0] + " " + p[1] + " " + p[2] + "\n");
        }
      }
      for (int i = 0; i < active.size(); i++) {
        int base = i * 5;
        for (int j = 1; j <= 4; j++) w.write(base + " " + (base + j) + "\n");
        for (int j = 1; j <= 4; j++) w.write((base + j) + " " + (base + (j % 4) + 1) + "\n");
      }
    }
  }

  void bundle(
      int iteration,
      double before,
      double after,
      List<ImageFrame> frames,
      List<FeatureTracks.Track> tracks)
      throws IOException {
    Path dir = directory("04_geometry/bundle");
    Path history = dir.resolve("iterations.csv");
    if (!Files.exists(history))
      Files.writeString(history, "iteration,robust_objective_before,robust_objective_after\n");
    Files.writeString(
        history, iteration + "," + before + "," + after + "\n", StandardOpenOption.APPEND);
    snapshot(
        String.format(Locale.ROOT, "bundle/iteration_%04d", iteration),
        frames,
        tracks,
        "pre-output-multiplier reconstruction frame");
  }

  void pnp(
      ImageFrame frame,
      List<FeatureTracks.Track> tracks,
      List<Point> pixels,
      int[] inliers,
      double[] rvec,
      double[] tvec,
      boolean registered)
      throws IOException {
    if (!options.diagnostics) return;
    Path dir =
        directory(
            String.format(
                Locale.ROOT, "04_geometry/pnp/attempt_%04d_%s", ++pnpAttempts, imageId(frame.id)));
    Set<Integer> selected = new HashSet<>();
    for (int index : inliers) selected.add(index);
    try (BufferedWriter w = Files.newBufferedWriter(dir.resolve("correspondences.csv"))) {
      w.write("correspondence_id,track_id,keypoint_id,x,y,z,u,v,pnp_inlier\n");
      for (int i = 0; i < tracks.size(); i++) {
        FeatureTracks.Track t = tracks.get(i);
        FeatureTracks.Observation observation = Geometry.observation(t, frame.id);
        Point uv = pixels.get(i);
        w.write(
            i
                + ","
                + t.id
                + ","
                + observation.feature
                + ","
                + t.point[0]
                + ","
                + t.point[1]
                + ","
                + t.point[2]
                + ","
                + uv.x
                + ","
                + uv.y
                + ","
                + selected.contains(i)
                + "\n");
      }
    }
    Json.write(
        dir.resolve("pnp.json"),
        Json.object(
            "image_id",
            frame.id,
            "registered",
            registered,
            "correspondences",
            tracks.size(),
            "inliers",
            inliers.length,
            "rodrigues_radians",
            rvec,
            "t_world_to_camera",
            tvec,
            "refinement",
            "inlier-only solvePnPRefineLM when registration succeeds",
            "coordinate_frame",
            "current pre-output-multiplier SfM frame"));
  }

  void trackFilter(FeatureTracks.Track track, List<ImageFrame> frames, boolean retained)
      throws IOException {
    if (!options.diagnostics) return;
    Path file = project.run.resolve("04_geometry/track_filter.csv");
    if (!Files.exists(file))
      Files.writeString(file, "track_id,retained,reason,maximum_reprojection_pixels\n");
    double worst = 0;
    boolean behind = false;
    if (track.point != null)
      for (FeatureTracks.Observation o : track.observations) {
        ImageFrame f = frames.get(o.image);
        if (!f.camera.registered) continue;
        double[] p = f.camera.project(track.point);
        Point actual = f.keypoints[o.feature].pt;
        if (p == null) behind = true;
        else worst = Math.max(worst, Math.hypot(p[0] - actual.x, p[1] - actual.y));
      }
    Files.writeString(
        file,
        track.id
            + ","
            + retained
            + ","
            + (track.point == null
                ? "not_triangulated"
                : retained
                    ? "retained"
                    : behind ? "nonpositive_camera_depth" : "reprojection_error")
            + ","
            + (track.point == null ? "" : worst)
            + "\n",
        StandardOpenOption.APPEND);
  }

  void rectification(
      int source,
      ImageFrame a,
      ImageFrame b,
      boolean vertical,
      double[] r1,
      double[] r2,
      double[] p1,
      double[] p2,
      double[] q,
      double[] relativeR,
      double[] relativeT,
      boolean alreadyRectified)
      throws IOException {
    Path dir = directory("05_stereo/" + pairId(a.id, b.id));
    PairRaster raster = new PairRaster();
    raster.dir = dir;
    rasters.put(source, raster);
    Json.write(
        dir.resolve("rectification.json"),
        Json.object(
            "fusion_pair_id",
            source,
            "reference_image_id",
            a.id,
            "secondary_image_id",
            b.id,
            "reference_filename",
            a.path.getFileName().toString(),
            "secondary_filename",
            b.path.getFileName().toString(),
            "already_rectified_input",
            alreadyRectified,
            "matching_raster_transposed",
            vertical,
            "matching_to_original_rectified_xy",
            vertical
                ? new double[][] {{0, 1, 0}, {1, 0, 0}, {0, 0, 1}}
                : new double[][] {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
            "disparity_convention",
            "d=x_left_matching-x_right_matching; use original rectified x,y and matching disparity"
                + " in Q*[x,y,d,1]",
            "R_secondary_from_reference",
            matrix(relativeR, 3, 3),
            "t_secondary_from_reference",
            relativeT,
            "R1_camera_to_rectified",
            matrix(r1, 3, 3),
            "R2_camera_to_rectified",
            matrix(r2, 3, 3),
            "P1",
            matrix(p1, 3, 4),
            "P2",
            matrix(p2, 3, 4),
            "Q",
            matrix(q, 4, 4),
            "K1_processed_undistorted",
            new double[][] {
              {a.camera.fx, 0, a.camera.cx}, {0, a.camera.fy, a.camera.cy}, {0, 0, 1}
            },
            "K2_processed_undistorted",
            new double[][] {
              {b.camera.fx, 0, b.camera.cx}, {0, b.camera.fy, b.camera.cy}, {0, 0, 1}
            },
            "effective_distortion",
            new double[5],
            "map_convention",
            "Each matching-raster pixel stores its source processed-undistorted x/y; source is not"
                + " the original distorted photo"));
    if (alreadyRectified && (options.diagnostics && options.storage.equals("debug"))) {
      int w = a.width, h = a.height;
      float[] x = new float[w * h], y = new float[w * h];
      for (int i = 0; i < x.length; i++) {
        x[i] = i % w;
        y[i] = i / w;
      }
      for (String side : new String[] {"left", "right"}) {
        DenseReconstruction.writeDisparity(
            dir.resolve(side + "_source_x.pfm"), new CensusSgm.Result(w, h, x));
        DenseReconstruction.writeDisparity(
            dir.resolve(side + "_source_y.pfm"), new CensusSgm.Result(w, h, y));
      }
    }
  }

  void rectificationMaps(int source, String side, Mat mx, Mat my, boolean vertical)
      throws IOException {
    if (!(options.diagnostics && options.storage.equals("debug"))) return;
    Path dir = rasters.get(source).dir;
    Mat x = new Mat(), y = new Mat();
    try {
      if (vertical) {
        Core.transpose(mx, x);
        Core.transpose(my, y);
      } else {
        mx.copyTo(x);
        my.copyTo(y);
      }
      pfm(dir.resolve(side + "_source_x.pfm"), x);
      pfm(dir.resolve(side + "_source_y.pfm"), y);
    } finally {
      x.release();
      y.release();
    }
  }

  void patchmatchPair(int source, ImageFrame reference, ImageFrame other) throws IOException {
    PairRaster raster = new PairRaster();
    raster.width = reference.width;
    raster.height = reference.height;
    raster.dir =
        directory(
            String.format(Locale.ROOT, "05_stereo/pm_ref_%04d_src_%04d", reference.id, other.id));
    if ((options.diagnostics && options.storage.equals("debug"))) {
      raster.accepted = new byte[raster.width * raster.height];
      raster.support = new float[raster.width * raster.height];
      Arrays.fill(raster.support, Float.NaN);
    }
    rasters.put(source, raster);
    Json.write(
        raster.dir.resolve("raster.json"),
        Json.object(
            "method",
            "patchmatch",
            "reference_image_id",
            reference.id,
            "source_image_id",
            other.id,
            "width",
            raster.width,
            "height",
            raster.height,
            "pixel_frame",
            "processed undistorted reference image; no rectification"));
  }

  void rectifiedImages(
      int source,
      ImageFrame a,
      ImageFrame b,
      Mat left,
      Mat right,
      Mat leftGray,
      Mat rightGray,
      Mat leftMask,
      Mat rightMask)
      throws IOException {
    PairRaster raster = rasters.get(source);
    raster.width = left.cols();
    raster.height = left.rows();
    if (!(options.diagnostics && options.storage.equals("debug"))) {
      Json.write(
          raster.dir.resolve("raster.json"),
          Json.object(
              "width",
              raster.width,
              "height",
              raster.height,
              "left_image_id",
              a.id,
              "right_image_id",
              b.id,
              "pixel_frame",
              "rectified matching raster",
              "storage",
              options.storage));
      return;
    }
    raster.accepted = new byte[raster.width * raster.height];
    raster.support = new float[raster.width * raster.height];
    Arrays.fill(raster.support, Float.NaN);
    image(raster.dir.resolve("left_color.png"), left);
    image(raster.dir.resolve("right_color.png"), right);
    image(raster.dir.resolve("left_gray.png"), leftGray);
    image(raster.dir.resolve("right_gray.png"), rightGray);
    image(raster.dir.resolve("left_valid_mask.png"), leftMask);
    image(raster.dir.resolve("right_valid_mask.png"), rightMask);
    Mat preview = new Mat();
    try {
      Core.hconcat(Arrays.asList(left, right), preview);
      for (int y = 16; y < preview.rows(); y += 32)
        Imgproc.line(
            preview, new Point(0, y), new Point(preview.cols() - 1, y), new Scalar(0, 128, 255), 1);
      image(raster.dir.resolve("epipolar_rows.png"), preview);
    } finally {
      preview.release();
    }
    Json.write(
        raster.dir.resolve("raster.json"),
        Json.object(
            "width",
            raster.width,
            "height",
            raster.height,
            "left_image_id",
            a.id,
            "right_image_id",
            b.id,
            "pixel_frame",
            "matching raster; may be transposed relative to the original processed images",
            "invalid_reason_codes",
            Json.object(
                "0",
                "stereo-valid",
                "1",
                "mask, texture, border or no valid hypothesis",
                "2",
                "search endpoint",
                "3",
                "uniqueness",
                "4",
                "initial LR",
                "5",
                "post-median LR",
                "6",
                "photometric ZNCC",
                "7",
                "speckle",
                "8",
                "disparity edge erosion (opt-in)"),
            "fusion_status_codes",
            Json.object(
                "0",
                "not sampled as a candidate",
                "1",
                "accepted into output",
                "2",
                "insufficient independent view support",
                "3",
                "voxel duplicate")));
  }

  CensusSgm.Observer stereoObserver(int source, int width, int height) {
    return stereoObserver(source, width, height, options, "disparity");
  }

  CensusSgm.Observer stereoObserver(
      int source, int width, int height, Options effective, String folder) {
    return new CensusSgm.Observer() {
      public void raster(String name, float[] values, int w, int h) throws IOException {
        Path dir =
            directory(
                "05_stereo/"
                    + project.run.relativize(rasters.get(source).dir).getFileName()
                    + "/"
                    + folder);
        scalar(
            dir,
            name,
            values,
            w,
            h,
            name.equals("zncc") ? -1 : name.equals("left_right_error") ? 0 : effective.minDisparity,
            name.equals("zncc")
                ? 1
                : name.equals("left_right_error")
                    ? options.lrThreshold
                    : effective.minDisparity + effective.disparities - 1);
      }

      public void reasons(byte[] codes, int w, int h) throws IOException {
        Path dir = directory("05_stereo/" + rasters.get(source).dir.getFileName() + "/" + folder);
        Mat raw = new Mat(h, w, CvType.CV_8U), color = new Mat(h, w, CvType.CV_8UC3);
        byte[] colors = new byte[w * h * 3];
        int[][] palette = {
          {0, 180, 0},
          {40, 40, 40},
          {0, 128, 255},
          {255, 0, 255},
          {0, 0, 255},
          {255, 128, 0},
          {0, 255, 255},
          {255, 0, 0},
          {255, 255, 255}
        };
        for (int i = 0; i < codes.length; i++)
          for (int c = 0; c < 3; c++) colors[i * 3 + c] = (byte) palette[codes[i] & 255][c];
        try {
          raw.put(0, 0, codes);
          color.put(0, 0, colors);
          image(dir.resolve("rejection_codes.png"), raw);
          image(dir.resolve("rejection_preview.png"), color);
        } finally {
          raw.release();
          color.release();
        }
      }
    };
  }

  void finalDisparityAttempt(int source, String folder) throws IOException {
    Path from = rasters.get(source).dir.resolve(folder),
        to = rasters.get(source).dir.resolve("disparity");
    Files.createDirectories(to);
    try (java.util.stream.Stream<Path> files = Files.list(from)) {
      for (Path p : (Iterable<Path>) files.filter(Files::isRegularFile)::iterator)
        Files.copy(p, to.resolve(p.getFileName()), StandardCopyOption.REPLACE_EXISTING);
    }
  }

  void finalDisparity(int source, CensusSgm.Result result, Options effective) throws IOException {
    if (!options.diagnostics || !options.storage.equals("standard")) return;
    Path dir = directory("05_stereo/" + rasters.get(source).dir.getFileName() + "/disparity");
    scalar(
        dir,
        "05_final_stereo",
        result.disparity,
        result.width,
        result.height,
        effective.minDisparity,
        effective.minDisparity + effective.disparities - 1);
  }

  static void scalar(
      Path dir, String name, float[] values, int width, int height, double low, double high)
      throws IOException {
    Files.createDirectories(dir);
    DenseReconstruction.writeDisparity(
        dir.resolve(name + ".pfm"), new CensusSgm.Result(width, height, values));
    byte[] display = new byte[values.length];
    for (int i = 0; i < values.length; i++)
      if (Float.isFinite(values[i]))
        display[i] =
            (byte)
                Math.max(
                    1, Math.min(255, 1 + 254 * (values[i] - low) / Math.max(1e-12, high - low)));
    Mat preview = new Mat(height, width, CvType.CV_8U);
    try {
      preview.put(0, 0, display);
      image(dir.resolve(name + ".png"), preview);
    } finally {
      preview.release();
    }
    Json.write(
        dir.resolve(name + ".json"),
        Json.object(
            "width",
            width,
            "height",
            height,
            "preview_range",
            new double[] {low, high},
            "preview_zero",
            "invalid/NaN",
            "numeric_data",
            name + ".pfm",
            "preview",
            "linear grayscale, valid values clamped to 1..255; preview clipping does not alter"
                + " numeric PFM"));
  }

  void triangulation(
      int source, float[] depths, float[] angles, byte[] valid, int width, int height)
      throws IOException {
    Path dir = rasters.get(source).dir;
    double max = 0;
    for (float d : depths) if (Float.isFinite(d)) max = Math.max(max, d);
    scalar(dir, "left_camera_depth", depths, width, height, 0, Math.max(1, max));
    scalar(
        dir,
        "triangulation_angle_degrees",
        angles,
        width,
        height,
        0,
        Math.max(5, options.minAngle * 5));
    Mat mask = new Mat(height, width, CvType.CV_8U);
    try {
      mask.put(0, 0, valid);
      image(dir.resolve("triangulation_valid_mask.png"), mask);
    } finally {
      mask.release();
    }
  }

  void candidate(
      long id,
      int source,
      int x,
      int y,
      float disparity,
      double[] world,
      int r,
      int g,
      int b,
      double angle,
      double correlation)
      throws IOException {
    if (!(options.diagnostics && options.storage.equals("debug"))) return;
    if (candidates == null) {
      candidates = gzip(project.run.resolve("06_fusion/candidates.csv.gz"));
      candidates.write(
          "candidate_id,fusion_pair_id,matching_x,matching_y,disparity,x,y,z,red,green,blue,angle_degrees,zncc\n");
    }
    candidates.write(
        id
            + ","
            + source
            + ","
            + x
            + ","
            + y
            + ","
            + disparity
            + ","
            + world[0]
            + ","
            + world[1]
            + ","
            + world[2]
            + ","
            + r
            + ","
            + g
            + ","
            + b
            + ","
            + angle
            + ","
            + correlation
            + "\n");
  }

  void decision(long candidate, int source, int pixel, int views, String status)
      throws IOException {
    if (!(options.diagnostics && options.storage.equals("debug"))) return;
    if (decisions == null) {
      decisions = gzip(project.run.resolve("06_fusion/decisions.csv.gz"));
      decisions.write(
          "candidate_id,fusion_pair_id,matching_x,matching_y,supporting_distinct_views,decision\n");
    }
    PairRaster raster = rasters.get(source);
    decisions.write(
        candidate
            + ","
            + source
            + ","
            + (pixel % raster.width)
            + ","
            + (pixel / raster.width)
            + ","
            + views
            + ","
            + status
            + "\n");
    raster.support[pixel] = views;
    raster.accepted[pixel] =
        (byte)
            (status.equals("accepted")
                ? 1
                : status.equals("voxel_duplicate")
                    ? 3
                    : status.equals("depth_consensus_rejected")
                        ? 4
                        : status.equals("depth_uncertainty_rejected")
                            ? 5
                            : status.startsWith("refinement_") ? 6 : 2);
  }

  void pairEvidence(
      int source, int a, float[] first, int wa, int ha, int b, float[] second, int wb, int hb)
      throws IOException {
    Path dir =
        directory("06_fusion/pair_evidence/source_" + String.format(Locale.ROOT, "%04d", source));
    float[] one = first.clone(), two = second.clone();
    for (int i = 0; i < one.length; i++) if (one[i] == 0) one[i] = Float.NaN;
    for (int i = 0; i < two.length; i++) if (two[i] == 0) two[i] = Float.NaN;
    DenseReconstruction.writeDisparity(
        dir.resolve(imageId(a) + "_camera_z.pfm"), new CensusSgm.Result(wa, ha, one));
    DenseReconstruction.writeDisparity(
        dir.resolve(imageId(b) + "_camera_z.pfm"), new CensusSgm.Result(wb, hb, two));
    Json.write(
        dir.resolve("pair.json"),
        Json.object(
            "fusion_pair_id",
            source,
            "stereo_artifacts",
            project.run.relativize(rasters.get(source).dir).toString().replace('\\', '/'),
            "image_a",
            a,
            "image_b",
            b,
            "pixel_frame",
            "each original processed-undistorted image; not the rectified matching raster",
            "values",
            "camera Z before output multiplier; NaN where no measurement; producing pair never"
                + " counts as independent evidence"));
  }

  void pairEvidenceStream(
      int source,
      int a,
      DepthRaster first,
      int wa,
      int ha,
      int b,
      DepthRaster second,
      int wb,
      int hb)
      throws IOException {
    if (!(options.diagnostics && options.storage.equals("debug"))) return;
    Path dir =
        directory("06_fusion/pair_evidence/source_" + String.format(Locale.ROOT, "%04d", source));
    DepthRaster[] maps = {first, second};
    int[] ids = {a, b}, width = {wa, wb}, height = {ha, hb};
    for (int side = 0; side < 2; side++)
      for (int channel = 0; channel < 3; channel++) {
        String suffix =
            channel == 0 ? "_camera_z.pfm" : channel == 1 ? "_offset_x.pfm" : "_offset_y.pfm";
        try (DataOutputStream out =
            new DataOutputStream(
                new BufferedOutputStream(
                    Files.newOutputStream(dir.resolve(imageId(ids[side]) + suffix))))) {
          out.writeBytes("Pf\n" + width[side] + " " + height[side] + "\n-1.0\n");
          for (int y = height[side] - 1; y >= 0; y--)
            for (int x = 0; x < width[side]; x++) {
              int at = y * width[side] + x;
              float z = maps[side].get(at);
              float value =
                  z == 0
                      ? Float.NaN
                      : channel == 0
                          ? z
                          : (float)
                              (channel == 1
                                  ? DepthFusion.offsetX(maps[side].offset(at))
                                  : DepthFusion.offsetY(maps[side].offset(at)));
              out.writeInt(Integer.reverseBytes(Float.floatToRawIntBits(value)));
            }
        }
      }
    Json.write(
        dir.resolve("pair.json"),
        Json.object(
            "fusion_pair_id",
            source,
            "image_a",
            a,
            "image_b",
            b,
            "pixel_frame",
            "each processed-undistorted image; not rectified",
            "storage",
            "exact float32 camera-Z and packed subpixel offsets; bounded paged I/O",
            "stereo_artifacts",
            project.run.relativize(rasters.get(source).dir).toString().replace('\\', '/'),
            "offsets",
            "normalized packed offsets decoded to pixel units; NaN where no measurement"));
  }

  void fusionRefinement(long id, int source, double[] original, DepthFusion.Result result)
      throws IOException {
    if (!(options.diagnostics && options.storage.equals("debug"))) return;
    if (refinements == null) {
      refinements = gzip(project.run.resolve("06_fusion/refinements.csv.gz"));
      refinements.write(
          "candidate_id,fusion_pair_id,original_x,original_y,original_z,refined_x,refined_y,refined_z,shift,independent_pairs,supporting_views,estimated_relative_stereo_sigma,geometry_decision\n");
    }
    refinements.write(
        id
            + ","
            + source
            + ","
            + original[0]
            + ","
            + original[1]
            + ","
            + original[2]
            + ","
            + result.point[0]
            + ","
            + result.point[1]
            + ","
            + result.point[2]
            + ","
            + result.shift
            + ","
            + result.independentPairs
            + ","
            + result.views
            + ","
            + result.relativeSigma
            + ","
            + result.status
            + "\n");
  }

  void fusionConfiguration(Object value) throws IOException {
    Json.write(directory("06_fusion").resolve("fusion.json"), value);
  }

  void pairEvidenceOffsets(
      int source, int a, int[] first, int wa, int ha, int b, int[] second, int wb, int hb)
      throws IOException {
    Path dir =
        directory("06_fusion/pair_evidence/source_" + String.format(Locale.ROOT, "%04d", source));
    for (int side = 0; side < 2; side++) {
      int image = side == 0 ? a : b, w = side == 0 ? wa : wb, h = side == 0 ? ha : hb;
      int[] offsets = side == 0 ? first : second;
      float[] x = new float[offsets.length], y = new float[offsets.length];
      for (int i = 0; i < offsets.length; i++) {
        x[i] = (float) DepthFusion.offsetX(offsets[i]);
        y[i] = (float) DepthFusion.offsetY(offsets[i]);
      }
      DenseReconstruction.writeDisparity(
          dir.resolve(imageId(image) + "_offset_x.pfm"), new CensusSgm.Result(w, h, x));
      DenseReconstruction.writeDisparity(
          dir.resolve(imageId(image) + "_offset_y.pfm"), new CensusSgm.Result(w, h, y));
    }
  }

  void finishFusion() throws IOException {
    close();
    if (!(options.diagnostics && options.storage.equals("debug"))) return;
    for (Map.Entry<Integer, PairRaster> item : rasters.entrySet()) {
      PairRaster raster = item.getValue();
      Path dir = directory("06_fusion/" + raster.dir.getFileName());
      scalar(
          dir,
          "supporting_views",
          raster.support,
          raster.width,
          raster.height,
          0,
          Math.max(2, frames.size()));
      Mat status = new Mat(raster.height, raster.width, CvType.CV_8U);
      Mat color = new Mat(raster.height, raster.width, CvType.CV_8UC3);
      try {
        status.put(0, 0, raster.accepted);
        image(dir.resolve("candidate_status.png"), status);
        int[][] palette = {
          {0, 0, 0},
          {0, 200, 0},
          {0, 0, 255},
          {255, 180, 0},
          {180, 0, 180},
          {0, 128, 255},
          {255, 0, 255}
        };
        byte[] pixels = new byte[raster.accepted.length * 3];
        for (int i = 0; i < raster.accepted.length; i++)
          for (int channel = 0; channel < 3; channel++)
            pixels[i * 3 + channel] = (byte) palette[raster.accepted[i] & 255][channel];
        color.put(0, 0, pixels);
        image(dir.resolve("candidate_status_preview.png"), color);
        Json.write(
            dir.resolve("fusion_raster.json"),
            Json.object(
                "pixel_frame",
                "same matching raster as the stereo pair",
                "candidate_status_codes",
                Json.object(
                    "0",
                    "unsampled/no candidate",
                    "1",
                    "accepted (green)",
                    "2",
                    "independent-support rejected (red)",
                    "3",
                    "voxel duplicate (blue)",
                    "4",
                    "per-ray depth-consensus rejected (purple)",
                    "5",
                    "depth-uncertainty rejected (orange)",
                    "6",
                    "refined-point reprojection/support rejected (magenta)"),
                "supporting_views",
                "float32 distinct camera count at sampled candidates; NaN elsewhere"));
      } finally {
        status.release();
        color.release();
      }
    }
  }

  void calibrationView(
      int id, Path path, Mat gray, MatOfPoint2f corners, boolean detected, Point3[] objectPoints)
      throws IOException {
    source(id, path, gray.cols(), gray.rows(), Exif.gps(path));
    Path dir = directory("01_calibration/views/" + imageId(id));
    Mat preview = new Mat();
    try {
      Imgproc.cvtColor(gray, preview, Imgproc.COLOR_GRAY2BGR);
      if (detected)
        Calib3d.drawChessboardCorners(
            preview, new Size(options.boardColumns, options.boardRows), corners, true);
      image(dir.resolve("corners.png"), preview);
    } finally {
      preview.release();
    }
    try (BufferedWriter w = Files.newBufferedWriter(dir.resolve("corners.csv"))) {
      w.write("corner_id,board_x,board_y,board_z,image_u,image_v\n");
      Point[] uv = corners.toArray();
      for (int i = 0; i < uv.length; i++) {
        Point3 p = objectPoints[i];
        w.write(i + "," + p.x + "," + p.y + "," + p.z + "," + uv[i].x + "," + uv[i].y + "\n");
      }
    }
    Json.write(
        dir.resolve("detection.json"),
        Json.object(
            "image_id",
            id,
            "detected",
            detected,
            "corner_count",
            corners.total(),
            "original_raster_size",
            new int[] {gray.cols(), gray.rows()},
            "board_inner_columns",
            options.boardColumns,
            "board_inner_rows",
            options.boardRows,
            "square_size",
            options.squareSize,
            "board_frame",
            "z=0; object coordinates use the units of --square-size; image pixels are original"
                + " raster coordinates"));
  }

  void calibrationResult(
      List<Integer> ids,
      List<Mat> objectPoints,
      List<Mat> observations,
      List<Mat> rotations,
      List<Mat> translations,
      Mat k,
      MatOfDouble distortion,
      Size size,
      double rms)
      throws IOException {
    double[] values = Math3.array(k);
    referenceCalibration(
        new Calibration(
            values[0],
            values[4],
            values[2],
            values[5],
            distortion.toArray(),
            (int) size.width,
            (int) size.height));
    List<Object> poses = new ArrayList<>();
    try (BufferedWriter all =
        Files.newBufferedWriter(project.run.resolve("01_calibration/reprojection.csv"))) {
      all.write(
          "image_id,corner_id,observed_u,observed_v,predicted_u,predicted_v,residual_pixels\n");
      for (int i = 0; i < ids.size(); i++) {
        MatOfPoint3f object = new MatOfPoint3f(objectPoints.get(i));
        MatOfPoint2f projected = new MatOfPoint2f(),
            observed = new MatOfPoint2f(observations.get(i));
        try {
          Calib3d.projectPoints(
              object, rotations.get(i), translations.get(i), k, distortion, projected);
          Point[] a = observed.toArray(), b = projected.toArray();
          double square = 0;
          for (int j = 0; j < a.length; j++) {
            double e = Math.hypot(a[j].x - b[j].x, a[j].y - b[j].y);
            square += e * e;
            all.write(
                ids.get(i)
                    + ","
                    + j
                    + ","
                    + a[j].x
                    + ","
                    + a[j].y
                    + ","
                    + b[j].x
                    + ","
                    + b[j].y
                    + ","
                    + e
                    + "\n");
          }
          poses.add(
              Json.object(
                  "image_id",
                  ids.get(i),
                  "rodrigues_radians",
                  Math3.array(rotations.get(i)),
                  "translation_board_to_camera",
                  Math3.array(translations.get(i)),
                  "reprojection_RMS_pixels",
                  Math.sqrt(square / a.length)));
        } finally {
          object.release();
          projected.release();
          observed.release();
        }
      }
    }
    Json.write(
        project.run.resolve("01_calibration/calibration.json"),
        Json.object(
            "RMS_pixels",
            rms,
            "views",
            poses,
            "exterior_convention",
            "Each pose maps board coordinates to that image camera; these are calibration-board"
                + " poses, not a shared flight exterior orientation"));
  }

  static double[][] matrix(double[] values, int rows, int cols) {
    if (values == null) return null;
    double[][] out = new double[rows][cols];
    for (int i = 0; i < rows; i++) System.arraycopy(values, i * cols, out[i], 0, cols);
    return out;
  }

  private double[] originalPixel(ImageFrame f, Point p) {
    Camera c = f.camera;
    double x = (p.x - c.cx) / c.fx, y = (p.y - c.cy) / c.fy, r2 = x * x + y * y;
    double[] d = f.calibration.distortion;
    double radial = 1 + d[0] * r2 + d[1] * r2 * r2 + d[4] * r2 * r2 * r2;
    double u = c.fx * (x * radial + 2 * d[2] * x * y + d[3] * (r2 + 2 * x * x)) + c.cx,
        v = c.fy * (y * radial + d[2] * (r2 + 2 * y * y) + 2 * d[3] * x * y) + c.cy;
    return new double[] {
      (u + .5) * f.sourceWidth / f.width - .5, (v + .5) * f.sourceHeight / f.height - .5
    };
  }

  static void pfm(Path path, Mat mat) throws IOException {
    Mat converted = new Mat();
    try {
      mat.convertTo(converted, CvType.CV_32F);
      float[] values = new float[(int) converted.total()];
      converted.get(0, 0, values);
      DenseReconstruction.writeDisparity(
          path, new CensusSgm.Result(mat.cols(), mat.rows(), values));
    } finally {
      converted.release();
    }
  }

  public void close() throws IOException {
    IOException error = null;
    try {
      if (candidates != null) candidates.close();
    } catch (IOException e) {
      error = e;
    } finally {
      candidates = null;
    }
    try {
      if (decisions != null) decisions.close();
    } catch (IOException e) {
      if (error == null) error = e;
      else error.addSuppressed(e);
    } finally {
      decisions = null;
    }
    try {
      if (refinements != null) refinements.close();
    } catch (IOException e) {
      if (error == null) error = e;
      else error.addSuppressed(e);
    } finally {
      refinements = null;
    }
    if (error != null) throw error;
  }

  private static final class PairRaster {
    int width, height;
    byte[] accepted;
    float[] support;
    Path dir;
  }

  static String guide() {
    return "# POSE project run\n\n"
               + "Detailed diagnostics are off by default. Orientations, final clouds, compact"
               + " lossless point-confidence.bin, reports and quality summaries remain available."
               + " --diagnostics enables optional exports and defaults storage to debug; an"
               + " explicit --storage minimal|standard|debug bounds that export scope. The table"
               + " below includes optional diagnostic artifacts, which may be absent in compact"
               + " runs. Unrequested debug artifacts are not missing processing stages. Caching is"
               + " opt-in (--cache or --resume); successful non-debug fusion removes working depth"
               + " files. GPU compute auto uses OpenCL when available and within device limits; CPU"
               + " fallback does not change image resolution.\n\n"
               + "[Run status](run.json) · [Stage status](stages.csv) · [Artifact index with"
               + " SHA-256](artifacts.csv) · [Console log](logs/console.log)\n\n"
               + "| Directory | Contents |\n"
               + "|---|---|\n"
               + "|00_inputs|Source inventory/hashes, optional archived originals, lossless"
               + " processed images, undistortion maps and masks|\n"
               + "|01_calibration|Reference and processed interior orientation; camera-calibration"
               + " observations, automatic focal search and EXIF evidence if applicable|\n"
               + "|02_features|All keypoints and selected descriptors; native type/metric and"
               + " visual overlays|\n"
               + "|03_matches|All nearest-neighbor decisions, filtering flags, accepted/rejected"
               + " previews, relative geometry and feature-track graph|\n"
               + "|04_geometry|Seed/registration/bundle/final orientations, sparse tracks and"
               + " observations, reprojection errors and camera frustums|\n"
               + "|05_stereo|Rectification images/maps/matrices, each disparity filtering stage,"
               + " confidence and rejection maps, pair depth/angle estimates|\n"
               + "|06_fusion|Per-candidate depth validation/refinement, normal-aware surface"
               + " consolidation summary; debug candidates, pre-consolidation decisions/support"
               + " maps and separate surface_decisions.csv.gz provenance|\n"
               + "|07_results|Final RGB LAS/PLY, point-confidence.bin, optional confidence CSV,"
               + " coordinates.json and report|\n"
               + "|logs|Console, events and effective settings|\n\n"
               + "Depth decisions and refinements describe the per-candidate stage. When"
               + " normal-aware consolidation is enabled, the final point-confidence.bin records"
               + " published positions, source candidate IDs, normals, neighborhood"
               + " roughness/planarity, surface shifts and merged-observation counts. Debug"
               + " surface_decisions.csv.gz maps merged candidates to their representatives. Merged"
               + " counts are not extra supporting-camera counts; camera support is rechecked for"
               + " moved points. --export-confidence RUN_DIRECTORY generates the compatible CSV"
               + " later without reconstructing. Project-root latest-files.csv references canonical"
               + " run outputs; --compatibility-copies additionally duplicates the large"
               + " clouds.\n\n"
               + "## Coordinates and formats\n\n"
               + "Image/keypoint IDs are zero-based and stable within this run. Keypoint/match"
               + " coordinates use the processed undistorted alignment raster: x right, y down,"
               + " zero-based pixel centers. Descriptors row i refers to keypoint i. Camera"
               + " extrinsics use X_camera=R*X_world+t, C_world=-R^T*t; camera axes x right, y"
               + " down, z forward. Rodrigues rotations are radians. No ambiguous omega/phi/kappa"
               + " convention is assumed. Intrinsics and distortion are explicitly separated"
               + " between original/reference and processed/undistorted images. When dense sizes"
               + " differ, 00_inputs/dense/ records their separate rasters and intrinsics.\n\n"
               + "Geometry snapshots and stereo candidates use pre-output-multiplier reconstruction"
               + " units. Final LAS/PLY apply --scale. Image-only geometry has arbitrary scale"
               + " unless anchored by a measured baseline, supplied poses or GPS. Successful"
               + " automatic GPS alignment uses the selected projected CRS (default EPSG:3067),"
               + " metres, x=easting and y=northing; --scale must be 1. Earlier geometry snapshots"
               + " retain the original reconstruction frame; the final snapshot, candidates and"
               + " clouds use the projected frame. See 04_geometry/georeference/transform.json and"
               + " 07_results/coordinates.json. Z retains EXIF altitude with unspecified vertical"
               + " datum; no N2000/geoid conversion.\n\n"
               + "PFM files are single-channel float32, little-endian, bottom-up; invalid"
               + " disparities/depths are NaN. Rectification-map values are source"
               + " undistorted-image pixel coordinates. If epipolar lines are vertical, matching"
               + " rasters are transposed; pair metadata records this and how to feed original"
               + " coordinates to Q. Descriptor .f32 uses SGMDESC1 magic (8 bytes), big-endian"
               + " int32 row/column counts, then row-major big-endian float32. Binary descriptors"
               + " retain exact byte values as floats; restore unsigned bytes for Hamming matching."
               + " CSV.gz files are ordinary UTF-8 CSV compressed with gzip. Numerical exports are"
               + " complete; match previews show at most 300 deterministic samples.\n\n"
               + "A stage marked skipped was not executed (e.g. feature matching with fixed"
               + " supplied poses), not silently lost. Intermediate stereo estimates are not final"
               + " accepted cloud points; use fusion decisions and final results for acceptance."
               + " Full SGM cost volumes and native solver workspaces are not persisted. Source"
               + " paths refer to originals unless --copy-inputs was requested. Each invocation"
               + " creates a new audit run; --resume and --rerun-from reuse verified stage payloads"
               + " from the separate project cache/v3/<storage>-<compact|diagnostics>/. Edited"
               + " audit exports are not implicitly loaded as restart state.\n";
  }
}
