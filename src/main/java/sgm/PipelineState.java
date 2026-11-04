package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import org.opencv.core.KeyPoint;

/**
 * Saved project state between pipeline steps ({@code RUN/state}). {@code state.json} holds step
 * status, arguments and small results; {@code frames.bin} and {@code tracks.bin} hold cameras,
 * calibrations, keypoints, tracks and matched pairs as exact binary values, so a step started in
 * a new process continues from exactly the numbers the previous step ended with.
 */
final class PipelineState {
  static final String FORMAT = "msgm-pipeline-state-1";
  private static final long FRAMES_MAGIC = 0x53474d4652414d31L, TRACKS_MAGIC = 0x53474d5452414b31L;

  final Path directory;
  final Map<String, Object> json;

  private PipelineState(Path directory, Map<String, Object> json) {
    this.directory = directory;
    this.json = json;
  }

  static boolean exists(Path run) {
    return Files.isRegularFile(run.resolve("state/state.json"));
  }

  static PipelineState create(Path run, String[] arguments) throws IOException {
    Path dir = run.resolve("state");
    Files.createDirectories(dir);
    Map<String, Object> json = new LinkedHashMap<>();
    json.put("format", FORMAT);
    json.put("arguments", Arrays.asList(arguments));
    json.put("started_epoch_millis", System.currentTimeMillis());
    Map<String, Object> steps = new LinkedHashMap<>();
    for (Pipeline.Step s : Pipeline.Step.values()) steps.put(s.name, Json.object("status", "pending"));
    json.put("steps", steps);
    PipelineState state = new PipelineState(dir, json);
    state.write();
    return state;
  }

  static PipelineState load(Path run) throws IOException {
    Path dir = run.resolve("state");
    Map<String, Object> json = JsonRead.object(JsonRead.read(dir.resolve("state.json")));
    if (!FORMAT.equals(json.get("format")))
      throw new IOException("Unsupported pipeline state in " + dir + ": " + json.get("format"));
    return new PipelineState(dir, new LinkedHashMap<>(json));
  }

  void write() throws IOException {
    Path temp = directory.resolve("state.json.part");
    Json.write(temp, json);
    CloudWriter.publish(temp, directory.resolve("state.json"));
  }

  /**
   * Keeps the state as it was after a step (state/after-STEP), so a branch at a later step starts
   * from exactly that point. Files unchanged since the previous snapshot are hard-linked.
   */
  void snapshot(Pipeline.Step step) throws IOException {
    Path target = directory.resolve("after-" + step.name);
    Files.createDirectories(target);
    Path previous = null;
    for (Pipeline.Step s : Pipeline.Step.values()) {
      if (s == step) break;
      Path candidate = directory.resolve("after-" + s.name);
      if (Files.isDirectory(candidate)) previous = candidate;
    }
    Path run = directory.getParent();
    for (String name :
        new String[] {
          "state.json", "frames.bin", "tracks.bin", "calibration.bin", "calibration_poses.bin", "camera.txt",
          "constraints.json"
        }) {
      // camera.txt and constraints.json are rewritten by later steps; keep this step's versions.
      Path file =
          name.equals("camera.txt")
              ? run.resolve("camera.txt")
              : name.equals("constraints.json")
                  ? run.resolve("01_calibration/constraints.json")
                  : directory.resolve(name);
      Path out = target.resolve(name);
      Files.deleteIfExists(out);
      if (!Files.isRegularFile(file)) continue;
      Path old = previous == null ? null : previous.resolve(name);
      if (old != null && Files.isRegularFile(old) && Files.mismatch(old, file) == -1) {
        try {
          Files.createLink(out, old);
          continue;
        } catch (IOException | UnsupportedOperationException e) {
          // fall through to copy
        }
      }
      Files.copy(file, out);
    }
  }

  @SuppressWarnings("unchecked")
  Map<String, Object> steps() {
    return (Map<String, Object>) json.get("steps");
  }

  String status(Pipeline.Step step) {
    Object entry = steps().get(step.name);
    // A step added after the run was created (e.g. gcp) did not exist for it: skipped.
    return entry == null ? "skipped" : String.valueOf(JsonRead.object(entry).get("status"));
  }

  void mark(Pipeline.Step step, String status, String detail) throws IOException {
    steps()
        .put(
            step.name,
            Json.object(
                "status", status, "updated_utc", java.time.Instant.now().toString(), "detail", detail));
    write();
  }

  @SuppressWarnings("unchecked")
  List<String> arguments() {
    List<String> out = new ArrayList<>();
    for (Object o : (List<Object>) json.get("arguments")) out.add(String.valueOf(o));
    return out;
  }

  void put(String key, Object value) {
    json.put(key, value);
  }

  Object get(String key) {
    return json.get(key);
  }

  String string(String key) {
    Object v = json.get(key);
    return v == null ? null : String.valueOf(v);
  }

  // ---- Frames: calibration, camera, flags, keypoints, colours, digests.

  void writeFrames(List<ImageFrame> frames) throws IOException {
    Path temp = directory.resolve("frames.bin.part");
    try (DataOutputStream out =
        new DataOutputStream(new BufferedOutputStream(Files.newOutputStream(temp), 1 << 20))) {
      out.writeLong(FRAMES_MAGIC);
      out.writeInt(frames.size());
      for (ImageFrame f : frames) {
        out.writeInt(f.id);
        out.writeUTF(f.path.toAbsolutePath().toString());
        out.writeInt(f.width);
        out.writeInt(f.height);
        writeCalibration(out, f.calibration);
        Camera c = f.camera;
        out.writeDouble(c.fx);
        out.writeDouble(c.fy);
        out.writeDouble(c.cx);
        out.writeDouble(c.cy);
        for (double v : c.rotation) out.writeDouble(v);
        for (double v : c.translation) out.writeDouble(v);
        out.writeBoolean(c.registered);
        out.writeBoolean(c.fixed);
        out.writeBoolean(c.denseExcluded);
        out.writeBoolean(c.velocity != null);
        if (c.velocity != null) for (double v : c.velocity) out.writeDouble(v);
        out.writeDouble(f.keypointPixelScale);
        out.writeInt(f.keypoints.length);
        for (KeyPoint k : f.keypoints) {
          out.writeDouble(k.pt.x);
          out.writeDouble(k.pt.y);
          out.writeFloat(k.size);
          out.writeFloat(k.angle);
          out.writeFloat(k.response);
          out.writeInt(k.octave);
          out.writeInt(k.class_id);
        }
        out.writeInt(f.featureColors == null ? -1 : f.featureColors.length);
        if (f.featureColors != null) out.write(f.featureColors);
        writeNullable(out, f.descriptorDigest);
        writeNullable(out, f.pixelFingerprint);
      }
    }
    CloudWriter.publish(temp, directory.resolve("frames.bin"));
  }

  /** Rebuilds the frames (rasters stay unloaded until a step needs them). */
  List<ImageFrame> readFrames(Options options) throws IOException {
    List<ImageFrame> frames = new ArrayList<>();
    try (DataInputStream in =
        new DataInputStream(
            new BufferedInputStream(Files.newInputStream(directory.resolve("frames.bin")), 1 << 20))) {
      if (in.readLong() != FRAMES_MAGIC) throw new IOException("Corrupt frames.bin");
      int n = in.readInt();
      try {
        for (int i = 0; i < n; i++) {
          int id = in.readInt();
          Path path = Path.of(in.readUTF());
          int width = in.readInt(), height = in.readInt();
          Calibration calibration = readCalibration(in);
          ImageFrame f = new ImageFrame(id, path, calibration, options, true);
          frames.add(f);
          if (f.width != width || f.height != height)
            throw new IllegalArgumentException(
                "Processed size of "
                    + path.getFileName()
                    + " changed ("
                    + width
                    + "x"
                    + height
                    + " saved, "
                    + f.width
                    + "x"
                    + f.height
                    + " now): an option that sets the alignment resolution differs from the earlier"
                    + " steps. Start a new run, or rerun from the match step with --branch.");
          f.calibration = calibration;
          Camera c = f.camera;
          c.fx = in.readDouble();
          c.fy = in.readDouble();
          c.cx = in.readDouble();
          c.cy = in.readDouble();
          for (int k = 0; k < 9; k++) c.rotation[k] = in.readDouble();
          for (int k = 0; k < 3; k++) c.translation[k] = in.readDouble();
          c.registered = in.readBoolean();
          c.fixed = in.readBoolean();
          c.denseExcluded = in.readBoolean();
          if (in.readBoolean()) {
            c.velocity = new double[3];
            for (int k = 0; k < 3; k++) c.velocity[k] = in.readDouble();
          }
          f.keypointPixelScale = in.readDouble();
          KeyPoint[] keys = new KeyPoint[in.readInt()];
          for (int k = 0; k < keys.length; k++) {
            double x = in.readDouble(), y = in.readDouble();
            KeyPoint p =
                new KeyPoint(
                    (float) x, (float) y, in.readFloat(), in.readFloat(), in.readFloat(), in.readInt(), in.readInt());
            // Keep the exact (possibly undistortion-remapped) double coordinates.
            p.pt.x = x;
            p.pt.y = y;
            keys[k] = p;
          }
          f.keypoints = keys;
          int colors = in.readInt();
          if (colors >= 0) {
            f.featureColors = new byte[colors];
            in.readFully(f.featureColors);
          }
          f.descriptorDigest = readNullable(in);
          f.pixelFingerprint = readNullable(in);
        }
      } catch (IOException | RuntimeException e) {
        for (ImageFrame f : frames) f.close();
        throw e;
      }
    }
    return frames;
  }

  // ---- Tracks and matched pairs.

  void writeTracks(FeatureTracks features) throws IOException {
    Path temp = directory.resolve("tracks.bin.part");
    try (DataOutputStream out =
        new DataOutputStream(new BufferedOutputStream(Files.newOutputStream(temp), 1 << 20))) {
      out.writeLong(TRACKS_MAGIC);
      out.writeInt(features.tracks.size());
      for (FeatureTracks.Track t : features.tracks) {
        out.writeInt(t.id);
        out.writeBoolean(t.point != null);
        if (t.point != null) for (double v : t.point) out.writeDouble(v);
        out.writeInt(t.observations.size());
        for (FeatureTracks.Observation o : t.observations) {
          out.writeInt(o.image);
          out.writeInt(o.feature);
        }
      }
      out.writeInt(features.pairs.size());
      for (FeatureTracks.Pair p : features.pairs) writePair(out, p);
      out.writeBoolean(features.seed != null);
      if (features.seed != null) {
        int at = features.pairs.indexOf(features.seed);
        out.writeInt(at);
        if (at < 0) writePair(out, features.seed);
      }
    }
    CloudWriter.publish(temp, directory.resolve("tracks.bin"));
  }

  FeatureTracks readTracks() throws IOException {
    FeatureTracks features = new FeatureTracks();
    Path file = directory.resolve("tracks.bin");
    if (!Files.isRegularFile(file)) return features;
    try (DataInputStream in =
        new DataInputStream(new BufferedInputStream(Files.newInputStream(file), 1 << 20))) {
      if (in.readLong() != TRACKS_MAGIC) throw new IOException("Corrupt tracks.bin");
      int n = in.readInt();
      for (int i = 0; i < n; i++) {
        FeatureTracks.Track t = new FeatureTracks.Track();
        t.id = in.readInt();
        if (in.readBoolean()) t.point = new double[] {in.readDouble(), in.readDouble(), in.readDouble()};
        int m = in.readInt();
        for (int k = 0; k < m; k++) t.observations.add(new FeatureTracks.Observation(in.readInt(), in.readInt()));
        features.tracks.add(t);
      }
      int pairs = in.readInt();
      for (int i = 0; i < pairs; i++) features.pairs.add(readPair(in));
      if (in.readBoolean()) {
        int at = in.readInt();
        features.seed = at >= 0 ? features.pairs.get(at) : readPair(in);
      }
    }
    return features;
  }

  private static void writePair(DataOutputStream out, FeatureTracks.Pair p) throws IOException {
    out.writeInt(p.a);
    out.writeInt(p.b);
    out.writeInt(p.matches.size());
    for (FeatureTracks.Match m : p.matches) {
      out.writeInt(m.a);
      out.writeInt(m.b);
    }
    writeArray(out, p.rotation);
    writeArray(out, p.translation);
    writeArray(out, p.essential);
    writeNullable(out, p.reason);
  }

  private static FeatureTracks.Pair readPair(DataInputStream in) throws IOException {
    FeatureTracks.Pair p = new FeatureTracks.Pair(in.readInt(), in.readInt());
    int m = in.readInt();
    for (int k = 0; k < m; k++) p.matches.add(new FeatureTracks.Match(in.readInt(), in.readInt()));
    p.rotation = readArray(in);
    p.translation = readArray(in);
    p.essential = readArray(in);
    p.reason = readNullable(in);
    return p;
  }

  // ---- Calibrations (also the per-image orientation calibrations of the options).

  static void writeCalibration(DataOutputStream out, Calibration c) throws IOException {
    out.writeDouble(c.fx);
    out.writeDouble(c.fy);
    out.writeDouble(c.cx);
    out.writeDouble(c.cy);
    out.writeInt(c.distortion.length);
    for (double v : c.distortion) out.writeDouble(v);
    out.writeInt(c.width);
    out.writeInt(c.height);
  }

  static Calibration readCalibration(DataInputStream in) throws IOException {
    double fx = in.readDouble(), fy = in.readDouble(), cx = in.readDouble(), cy = in.readDouble();
    double[] d = new double[in.readInt()];
    for (int k = 0; k < d.length; k++) d[k] = in.readDouble();
    return new Calibration(fx, fy, cx, cy, d, in.readInt(), in.readInt());
  }

  void writeCalibrations(String name, Calibration main, Map<String, Calibration> perImage)
      throws IOException {
    Path temp = directory.resolve(name + ".part");
    try (DataOutputStream out = new DataOutputStream(new BufferedOutputStream(Files.newOutputStream(temp)))) {
      out.writeBoolean(main != null);
      if (main != null) writeCalibration(out, main);
      out.writeInt(perImage.size());
      for (var e : perImage.entrySet()) {
        out.writeUTF(e.getKey());
        writeCalibration(out, e.getValue());
      }
    }
    CloudWriter.publish(temp, directory.resolve(name));
  }

  /** {main calibration or null, per-image map}. */
  Object[] readCalibrations(String name) throws IOException {
    Path file = directory.resolve(name);
    if (!Files.isRegularFile(file)) return new Object[] {null, new LinkedHashMap<String, Calibration>()};
    try (DataInputStream in = new DataInputStream(new BufferedInputStream(Files.newInputStream(file)))) {
      Calibration main = in.readBoolean() ? readCalibration(in) : null;
      Map<String, Calibration> map = new LinkedHashMap<>();
      int n = in.readInt();
      for (int i = 0; i < n; i++) map.put(in.readUTF(), readCalibration(in));
      return new Object[] {main, map};
    }
  }

  /** Camera poses by image name (the self-calibration bootstrap poses that seed alignment). */
  void writePoses(String name, Map<String, Camera> poses) throws IOException {
    Path temp = directory.resolve(name + ".part");
    try (DataOutputStream out = new DataOutputStream(new BufferedOutputStream(Files.newOutputStream(temp)))) {
      out.writeInt(poses.size());
      for (var e : poses.entrySet()) {
        Camera c = e.getValue();
        out.writeUTF(e.getKey());
        out.writeDouble(c.fx);
        out.writeDouble(c.fy);
        out.writeDouble(c.cx);
        out.writeDouble(c.cy);
        for (double v : c.rotation) out.writeDouble(v);
        for (double v : c.translation) out.writeDouble(v);
        out.writeBoolean(c.registered);
        out.writeBoolean(c.fixed);
      }
    }
    CloudWriter.publish(temp, directory.resolve(name));
  }

  Map<String, Camera> readPoses(String name) throws IOException {
    Path file = directory.resolve(name);
    Map<String, Camera> poses = new LinkedHashMap<>();
    if (!Files.isRegularFile(file)) return poses;
    try (DataInputStream in = new DataInputStream(new BufferedInputStream(Files.newInputStream(file)))) {
      int n = in.readInt();
      for (int i = 0; i < n; i++) {
        String key = in.readUTF();
        Camera c = new Camera(in.readDouble(), in.readDouble(), in.readDouble(), in.readDouble());
        for (int k = 0; k < 9; k++) c.rotation[k] = in.readDouble();
        for (int k = 0; k < 3; k++) c.translation[k] = in.readDouble();
        c.registered = in.readBoolean();
        c.fixed = in.readBoolean();
        poses.put(key, c);
      }
    }
    return poses;
  }

  private static void writeArray(DataOutputStream out, double[] v) throws IOException {
    out.writeInt(v == null ? -1 : v.length);
    if (v != null) for (double x : v) out.writeDouble(x);
  }

  private static double[] readArray(DataInputStream in) throws IOException {
    int n = in.readInt();
    if (n < 0) return null;
    double[] v = new double[n];
    for (int i = 0; i < n; i++) v[i] = in.readDouble();
    return v;
  }

  private static void writeNullable(DataOutputStream out, String s) throws IOException {
    out.writeBoolean(s != null);
    if (s != null) out.writeUTF(s);
  }

  private static String readNullable(DataInputStream in) throws IOException {
    return in.readBoolean() ? in.readUTF() : null;
  }
}
