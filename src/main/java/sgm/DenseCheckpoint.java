package sgm;

import java.io.*;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.FileChannel;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.security.MessageDigest;
import java.util.*;

/**
 * {@code --dense-checkpoint}: a resumable dense step for {@code --dense-method mvsgm} with
 * overlap-window image fusion. Rerunning an interrupted dense step on the same run continues
 * after the last camera that a committed checkpoint covers, with the same result as an
 * uninterrupted run.
 *
 * <p>A checkpoint is taken between two cameras of fusion pass 2, after the output of every
 * finished camera has been handed to the surface spool. It holds:
 *
 * <ul>
 *   <li>every live camera's merged fusion maps (depth, weights, local sigma: unchanged after
 *       pass 1, so saved once per camera, as a hard link where the file system allows) and its
 *       ownership marks (copied at each checkpoint);
 *   <li>the stereo window's bookkeeping, counters and report entries;
 *   <li>the surface spool length (a checkpoint closes the spool's gzip member; a resumed run cuts
 *       the file back to that length and appends a new member; readers accept concatenated
 *       members, the decoded records are identical).
 * </ul>
 *
 * <p>Raw and consistent mvsgm maps that the checkpoint still needs stay where they are: a map
 * file retired after a checkpoint is deleted only once the next checkpoint is committed (or the
 * dense step completes). The state is written to a new generation folder, then the small manifest
 * is replaced atomically; a crash at any moment leaves the previous checkpoint usable.
 *
 * <p>A checkpoint is used only by the same run with the same effective options and the same
 * saved state of the earlier steps (fingerprint); otherwise it is deleted and dense starts over.
 */
final class DenseCheckpoint {
  static final int VERSION = 1;
  static final String MANIFEST = "checkpoint.bin", MAGIC = "POSE-DENSE-CHECKPOINT";

  /**
   * Options that cannot change a result (resources, scheduling, paths); a resume may change them.
   * Everything else must equal the interrupted run's value.
   */
  static final Set<String> RESOURCE_OPTIONS = Set.of(
      "output", "cores", "memoryMb", "denseCheckpoint", "imageFusionCacheMb", "streamedProcessing",
      "fusionSourceMaps", "surfaceMemoryMb", "estimateMemory", "processedImageCache", "cache",
      "verbose", "mvsgmCamerasInFlight", "fusionHostCacheMb", "gpuMemoryMb", "imageMemoryMb");

  final Options o;
  final Path dir;
  final String fingerprint;
  final long intervalNanos;
  private final int everyCameras;
  private Map<String, Object> resume;
  private int generation;
  private long last = System.nanoTime(), writeNanos;
  private int sinceLast, written;
  private final List<Path> deferred = new ArrayList<>();
  private boolean completed;
  /** Cameras whose immutable fusion maps are in the store (maps/). */
  private final Set<Integer> stored = new TreeSet<>();
  private Map<String, Object> resumedFrom;
  private TreeMap<String, String> parts;

  private DenseCheckpoint(Options o, String fingerprint, long intervalNanos, int everyCameras) {
    this.o = o;
    this.dir = o.output.resolve("06_fusion/checkpoint");
    this.fingerprint = fingerprint;
    this.intervalNanos = intervalNanos;
    this.everyCameras = everyCameras;
  }

  /** Why checkpoints cannot be used with these options, or null. */
  static String unsupported(Options o) {
    if (!MvsgmWindow.enabled(o)) return "only --dense-method mvsgm with overlap-window image fusion";
    if (o.fusionSeedOrder.equals("precision")) return "--fusion-seed-order precision keeps every map";
    if (o.fusionHostMemoryMb > 0) return "--fusion-host-memory-mb";
    if (!o.surfaceConsolidation.equals("normal") && o.voxel > 0) return "--voxel without surface consolidation";
    if (!o.fusionEmission.equals("pair")) return "--fusion-emission camera";
    if (o.diagnostics) return "--diagnostics";
    if (Boolean.getBoolean("sgm.mvsgm.keepMaps") || Boolean.getBoolean("sgm.fusion.keepFinishedMaps"))
      return "kept maps (-Dsgm.mvsgm.keepMaps / -Dsgm.fusion.keepFinishedMaps)";
    return null;
  }

  /**
   * The checkpoint of this dense step: a resumable one when a valid checkpoint exists, a fresh one
   * otherwise, or null when checkpoints are off or not supported (a stale checkpoint is deleted).
   */
  static DenseCheckpoint open(Options o) throws IOException {
    Path dir = o.output.resolve("06_fusion/checkpoint");
    if (o.denseCheckpoint.equals("off")) {
      if (Files.isDirectory(dir)) {
        System.out.println("Dense checkpoint: off (--dense-checkpoint off); deleting the earlier checkpoint");
        deleteTree(dir);
      }
      return null;
    }
    String reason = unsupported(o);
    if (reason != null) {
      if (!o.denseCheckpoint.equals("auto"))
        System.out.println("Note: --dense-checkpoint is not available here (" + reason + "); dense runs without checkpoints");
      if (Files.isDirectory(dir)) deleteTree(dir);
      return null;
    }
    long minutes = o.denseCheckpoint.equals("auto") ? 30 : Long.parseLong(o.denseCheckpoint);
    int every = Integer.getInteger("sgm.dense.checkpointCameras", 0);
    TreeMap<String, String> parts = fingerprintParts(o);
    DenseCheckpoint cp = new DenseCheckpoint(o, digest(parts.toString()), minutes * 60_000_000_000L, every);
    cp.parts = parts;
    cp.load();
    return cp;
  }

  private void load() throws IOException {
    Path manifest = dir.resolve(MANIFEST);
    if (!Files.isRegularFile(manifest)) {
      if (Files.isDirectory(dir)) deleteTree(dir);
      return;
    }
    String problem = null;
    Map<String, Object> state = null;
    try {
      Map<String, Object> head = Codec.read(manifest);
      if (!MAGIC.equals(head.get("magic")) || ((Number) head.get("version")).intValue() != VERSION)
        problem = "written by another version";
      else if (!fingerprint.equals(head.get("fingerprint")))
        problem = "changed since it was written: " + String.join(", ", changed(map(head, "parts"), parts));
      else {
        generation = ((Number) head.get("generation")).intValue();
        state = Codec.read(generationDir(generation).resolve("state.bin"));
        problem = missingOutput(map(state, "output"));
      }
    } catch (IOException | RuntimeException e) {
      problem = "unreadable (" + e.getMessage() + ")";
    }
    if (problem != null) {
      System.out.println("Dense checkpoint ignored: " + problem + "; dense starts from the beginning");
      deleteTree(dir);
      generation = 0;
      return;
    }
    resume = state;
    resumedFrom = Json.object("generation", generation, "next_camera", state.get("next_camera"),
        "cameras_fused", state.get("cameras_fused"), "written_epoch_millis", state.get("written_epoch_millis"));
    for (Object c : (List<?>) state.get("stored")) stored.add(((Number) c).intValue());
    System.out.printf(Locale.ROOT,
        "Dense checkpoint found: resuming after camera %d (%d cameras fused; checkpoint %d)%n",
        ((Number) state.get("next_camera")).intValue() - 1, ((Number) state.get("cameras_fused")).intValue(), generation);
  }

  /** Why the checkpoint's output files cannot be continued, or null when they all exist. */
  @SuppressWarnings("unchecked")
  private String missingOutput(Map<String, Object> output) throws IOException {
    List<Object[]> needed = new ArrayList<>();
    Map<String, Object> surface = map(output, "surface");
    if (surface != null) {
      Path work = o.output.resolve((String) surface.get("directory"));
      needed.add(new Object[] {work.resolve("accepted.bin.gz"), surface.get("spool_bytes")});
      if ((long) surface.get("coordinate_bytes") > 0)
        needed.add(new Object[] {work.resolve("coordinates.bin"), surface.get("coordinate_bytes")});
    }
    Map<String, Object> cloud = map(output, "cloud");
    if (cloud != null && (boolean) cloud.get("opened")) {
      Path tiles = o.output.resolve("07_results/dense_tiles");
      for (Object entry : (List<Object>) cloud.get("files")) {
        Map<String, Object> f = (Map<String, Object>) entry;
        // Tile names are rebuilt by the writer; check the single file and the spools here.
        if (!(boolean) f.get("tile"))
          needed.add(new Object[] {o.output.resolve("dense.las.part"), f.get("part_bytes")});
        if (f.get("spool") != null)
          needed.add(new Object[] {((boolean) f.get("tile") ? tiles : o.output).resolve((String) f.get("spool")),
              f.get("spool_bytes")});
      }
      if ((long) cloud.get("ply_bytes") > 0)
        needed.add(new Object[] {o.output.resolve("dense.ply.part"), cloud.get("ply_bytes")});
      if (o.tiledLasOutput > 0) {
        long parts = 0, expected = 0;
        for (Object entry : (List<Object>) cloud.get("files")) if ((boolean) ((Map<String, Object>) entry).get("tile")) expected++;
        if (Files.isDirectory(tiles))
          try (var files = Files.list(tiles)) {
            parts = files.filter(p -> p.getFileName().toString().endsWith(".las.part")).count();
          }
        if (parts < expected) return "LAS tiles of the checkpoint are missing";
      }
    }
    if (output.get("records_length") != null)
      needed.add(new Object[] {o.output.resolve("07_results/point-confidence.bin"), output.get("records_length")});
    for (Object[] n : needed) {
      Path p = (Path) n[0];
      if (!Files.isRegularFile(p) || Files.size(p) < (long) n[1])
        return p.getFileName() + " is missing or shorter than recorded";
    }
    return null;
  }

  /** Temporary candidate files left by the interrupted process (the run is locked: no other writer). */
  void deleteStaleTemporaries(Path own) throws IOException {
    try (var files = Files.list(o.output)) {
      for (Path p : files.toList()) {
        String name = p.getFileName().toString();
        if (p.equals(own) || !name.endsWith(".bin")) continue;
        if (name.startsWith("candidates-") || name.startsWith("prepared-fusion-")) Files.deleteIfExists(p);
      }
    }
  }

  /** The loaded state to resume from, or null for a fresh dense step. */
  Map<String, Object> resume() {
    return resume;
  }

  /** Section of the resume state (null when not resuming). */
  Map<String, Object> resume(String section) {
    return resume == null ? null : map(resume, section);
  }

  /** Whether a checkpoint is due after this camera (time interval, or the test hook's count). */
  boolean due() {
    sinceLast++;
    if (everyCameras > 0) return sinceLast >= everyCameras;
    return System.nanoTime() - last >= intervalNanos;
  }

  /** The committed generation (the resumed one until the next commit). */
  Path currentGeneration() {
    return generationDir(generation);
  }

  Path generationDir(int g) {
    return dir.resolve("gen-" + g);
  }

  Path storeFile(int camera, String kind) {
    return dir.resolve("maps/camera_" + camera + "." + kind);
  }

  /** Next generation's folder, created empty. */
  Path begin() throws IOException {
    Path g = generationDir(generation + 1);
    if (Files.isDirectory(g)) deleteTree(g);
    Files.createDirectories(g);
    Files.createDirectories(dir.resolve("maps"));
    return g;
  }

  boolean stored(int camera) {
    return stored.contains(camera);
  }

  void markStored(int camera) {
    stored.add(camera);
  }

  /**
   * Commits the generation: state file, then the manifest (atomic replace), then removes what
   * only the previous checkpoint needed (its generation, store maps of retired cameras, retired
   * mvsgm maps).
   */
  void commit(Map<String, Object> state, Set<Integer> live, long started) throws IOException {
    Path g = generationDir(generation + 1);
    state.put("stored", new ArrayList<>(live));
    state.put("written_epoch_millis", System.currentTimeMillis());
    Codec.write(g.resolve("state.bin"), state);
    force(g.resolve("state.bin"));
    Path temporary = dir.resolve(MANIFEST + ".tmp");
    Codec.write(temporary, Json.object("magic", MAGIC, "version", VERSION, "fingerprint", fingerprint,
        "generation", generation + 1, "parts", parts));
    force(temporary);
    Files.move(temporary, dir.resolve(MANIFEST), StandardCopyOption.REPLACE_EXISTING, StandardCopyOption.ATOMIC_MOVE);
    Path previous = generationDir(generation);
    generation++;
    if (Files.isDirectory(previous)) deleteTree(previous);
    for (Iterator<Integer> it = stored.iterator(); it.hasNext(); ) {
      int c = it.next();
      if (live.contains(c)) continue;
      for (String kind : ImageFusion.TILE_KINDS) Files.deleteIfExists(storeFile(c, kind));
      it.remove();
    }
    deleteDeferred();
    sinceLast = 0;
    last = System.nanoTime();
    writeNanos += last - started;
    written++;
    System.out.printf(Locale.ROOT, "Dense checkpoint %d: %,d cameras fused, %d live cameras saved in %.1f s%n",
        generation, ((Number) state.get("cameras_fused")).longValue(), live.size(), (last - started) / 1e9);
  }

  /** A retired mvsgm map file: deleted at the next commit (the last checkpoint may still need it). */
  synchronized void defer(Path file) {
    deferred.add(file);
  }

  private synchronized void deleteDeferred() {
    for (Path p : deferred)
      try {
        Files.deleteIfExists(p);
      } catch (IOException e) {
        System.err.println("Warning: could not delete " + p + ": " + e);
      }
    deferred.clear();
  }

  boolean completed() {
    return completed;
  }

  /** An interrupted step keeps its work files when a committed checkpoint refers to them. */
  boolean keepsWork() {
    return !completed && generation > 0;
  }

  /** Fusion finished: the checkpoint is no longer needed. */
  void complete() throws IOException {
    completed = true;
    resume = null;
    deleteDeferred();
    if (Files.isDirectory(dir)) deleteTree(dir);
  }

  Object report() {
    return Json.object("enabled", true, "interval_minutes", intervalNanos / 60_000_000_000L,
        "checkpoints_written", written, "write_seconds", writeNanos / 1e9,
        "resumed_from", resumedFrom);
  }

  // ---- Fingerprint

  /** SHA-256 over the result-relevant options and the saved state of the earlier steps. */
  static String fingerprint(Options o) throws IOException {
    return digest(fingerprintParts(o).toString());
  }

  /** Result-relevant option values and SHA-256 of the earlier steps' saved state files. */
  static TreeMap<String, String> fingerprintParts(Options o) throws IOException {
    TreeMap<String, String> values = new TreeMap<>();
    try {
      for (Field field : Options.class.getDeclaredFields()) {
        int m = field.getModifiers();
        if (Modifier.isStatic(m) || Modifier.isTransient(m) || RESOURCE_OPTIONS.contains(field.getName())) continue;
        // Package-private paths are run-time folders (caches, temporary reuse), not settings.
        if (!Modifier.isPublic(m) && field.getType() == Path.class) continue;
        field.setAccessible(true);
        Object value = field.get(o);
        if (!plain(value)) continue;
        values.put("option " + field.getName(), value == null ? "null" : Json.string(value));
      }
      for (String name : new String[] {"frames.bin", "tracks.bin", "calibration.bin", "calibration_poses.bin"}) {
        Path file = o.output.resolve("state").resolve(name);
        if (!Files.isRegularFile(file)) {
          values.put("state " + name, "absent");
          continue;
        }
        MessageDigest digest = MessageDigest.getInstance("SHA-256");
        try (InputStream in = new BufferedInputStream(Files.newInputStream(file), 1 << 20)) {
          byte[] buffer = new byte[1 << 20];
          for (int n; (n = in.read(buffer)) > 0; ) digest.update(buffer, 0, n);
        }
        values.put("state " + name, HexFormat.of().formatHex(digest.digest()));
      }
    } catch (java.security.NoSuchAlgorithmException | IllegalAccessException e) {
      throw new IllegalStateException(e);
    }
    values.put("checkpoint version", String.valueOf(VERSION));
    return values;
  }

  private static String digest(String text) {
    try {
      return HexFormat.of().formatHex(MessageDigest.getInstance("SHA-256").digest(text.getBytes(StandardCharsets.UTF_8)));
    } catch (java.security.NoSuchAlgorithmException e) {
      throw new IllegalStateException(e);
    }
  }

  /** Names of the fingerprint parts that differ (for the message when a checkpoint is refused). */
  static List<String> changed(Map<String, Object> saved, Map<String, String> now) {
    List<String> out = new ArrayList<>();
    Set<String> keys = new TreeSet<>(now.keySet());
    keys.addAll(saved.keySet());
    for (String k : keys) if (!Objects.equals(saved.get(k), now.get(k))) out.add(k);
    return out;
  }

  /** Values with a stable text form (no object identities): primitives, text, paths, collections of them. */
  private static boolean plain(Object v) {
    if (v == null || v instanceof Number || v instanceof Boolean || v instanceof String || v instanceof Path
        || v instanceof Enum<?> || v instanceof Character) return true;
    if (v.getClass().isArray()) {
      if (v.getClass().getComponentType().isPrimitive()) return true;
      for (Object x : (Object[]) v) if (!plain(x)) return false;
      return true;
    }
    if (v instanceof Collection<?> c) {
      for (Object x : c) if (!plain(x)) return false;
      return true;
    }
    if (v instanceof Map<?, ?> m) {
      for (var e : m.entrySet()) if (!plain(e.getKey()) || !plain(e.getValue())) return false;
      return true;
    }
    return false;
  }

  // ---- Field snapshots (counters and report statistics)

  /** Primitive and primitive-array instance fields of obj (exact values). */
  static Map<String, Object> fields(Object obj) {
    Map<String, Object> out = new TreeMap<>();
    try {
      for (Field f : obj.getClass().getDeclaredFields()) {
        if (!snapshot(f)) continue;
        f.setAccessible(true);
        Object v = f.get(obj);
        out.put(f.getName(), v instanceof long[] a ? a.clone() : v);
      }
    } catch (IllegalAccessException e) {
      throw new IllegalStateException(e);
    }
    return out;
  }

  /** Restores what {@link #fields} saved (long[] fields are copied into the existing array). */
  static void restore(Object obj, Map<String, Object> values) {
    try {
      for (Field f : obj.getClass().getDeclaredFields()) {
        if (!snapshot(f) || !values.containsKey(f.getName())) continue;
        f.setAccessible(true);
        Object v = values.get(f.getName());
        Class<?> t = f.getType();
        if (t == long[].class) {
          long[] saved = (long[]) v, current = (long[]) f.get(obj);
          if (saved == null) f.set(obj, null);
          else if (current != null && current.length == saved.length && Modifier.isFinal(f.getModifiers()))
            System.arraycopy(saved, 0, current, 0, saved.length);
          else f.set(obj, saved.clone());
        } else if (t == long.class) f.setLong(obj, ((Number) v).longValue());
        else if (t == int.class) f.setInt(obj, ((Number) v).intValue());
        else if (t == double.class) f.setDouble(obj, ((Number) v).doubleValue());
        else if (t == boolean.class) f.setBoolean(obj, (Boolean) v);
      }
    } catch (IllegalAccessException e) {
      throw new IllegalStateException(e);
    }
  }

  private static boolean snapshot(Field f) {
    int m = f.getModifiers();
    if (Modifier.isStatic(m)) return false;
    Class<?> t = f.getType();
    if (t == long[].class) return true;
    return !Modifier.isFinal(m) && (t == long.class || t == int.class || t == double.class || t == boolean.class);
  }

  // ---- Raw little-endian map files

  static void writeFloats(Path file, ImageFusion.Raster r, long n) throws IOException {
    try (FileChannel ch = FileChannel.open(file, StandardOpenOption.CREATE, StandardOpenOption.TRUNCATE_EXISTING,
        StandardOpenOption.WRITE)) {
      float[] chunk = new float[1 << 18];
      ByteBuffer b = ByteBuffer.allocate(4 * chunk.length).order(ByteOrder.LITTLE_ENDIAN);
      for (long i = 0; i < n; ) {
        int k = (int) Math.min(chunk.length, n - i);
        r.read(i, chunk, 0, k);
        b.clear();
        b.asFloatBuffer().put(chunk, 0, k);
        b.limit(4 * k);
        while (b.hasRemaining()) ch.write(b);
        i += k;
      }
      ch.force(true);
    }
  }

  static void readFloats(Path file, ImageFusion.Raster r, long n) throws IOException {
    try (FileChannel ch = FileChannel.open(file, StandardOpenOption.READ)) {
      if (ch.size() != 4 * n) throw new IOException("Checkpoint map " + file + " holds " + ch.size() + " bytes, expected " + 4 * n);
      float[] chunk = new float[1 << 18];
      ByteBuffer b = ByteBuffer.allocate(4 * chunk.length).order(ByteOrder.LITTLE_ENDIAN);
      for (long i = 0; i < n; ) {
        int k = (int) Math.min(chunk.length, n - i);
        b.clear();
        b.limit(4 * k);
        while (b.hasRemaining()) if (ch.read(b) < 0) throw new EOFException("Checkpoint map truncated: " + file);
        b.flip();
        b.asFloatBuffer().get(chunk, 0, k);
        r.write(i, chunk, 0, k);
        i += k;
      }
    }
  }

  static void writeLongs(Path file, ImageFusion.Bits bits, long words) throws IOException {
    try (FileChannel ch = FileChannel.open(file, StandardOpenOption.CREATE, StandardOpenOption.TRUNCATE_EXISTING,
        StandardOpenOption.WRITE)) {
      long[] chunk = new long[1 << 17];
      ByteBuffer b = ByteBuffer.allocate(8 * chunk.length).order(ByteOrder.LITTLE_ENDIAN);
      for (long i = 0; i < words; ) {
        int k = (int) Math.min(chunk.length, words - i);
        bits.read(i, chunk, k);
        b.clear();
        b.asLongBuffer().put(chunk, 0, k);
        b.limit(8 * k);
        while (b.hasRemaining()) ch.write(b);
        i += k;
      }
      ch.force(true);
    }
  }

  static void readLongs(Path file, ImageFusion.Bits bits, long words) throws IOException {
    try (FileChannel ch = FileChannel.open(file, StandardOpenOption.READ)) {
      if (ch.size() != 8 * words) throw new IOException("Checkpoint marks " + file + " hold " + ch.size() + " bytes, expected " + 8 * words);
      long[] chunk = new long[1 << 17];
      ByteBuffer b = ByteBuffer.allocate(8 * chunk.length).order(ByteOrder.LITTLE_ENDIAN);
      for (long i = 0; i < words; ) {
        int k = (int) Math.min(chunk.length, words - i);
        b.clear();
        b.limit(8 * k);
        while (b.hasRemaining()) if (ch.read(b) < 0) throw new EOFException("Checkpoint marks truncated: " + file);
        b.flip();
        b.asLongBuffer().get(chunk, 0, k);
        bits.write(i, chunk, k);
        i += k;
      }
    }
  }

  /** Byte range [offset, offset + bytes) of src as a new file dst. */
  static void copyRange(Path src, long offset, long bytes, Path dst) throws IOException {
    try (FileChannel in = FileChannel.open(src, StandardOpenOption.READ);
        FileChannel out = FileChannel.open(dst, StandardOpenOption.CREATE, StandardOpenOption.TRUNCATE_EXISTING,
            StandardOpenOption.WRITE)) {
      for (long done = 0; done < bytes; ) {
        long n = in.transferTo(offset + done, bytes - done, out);
        if (n <= 0) throw new EOFException("Short copy from " + src);
        done += n;
      }
      out.force(true);
    }
  }

  /** dst as a hard link of src (same data, no copy), else a copy. src must not change later. */
  static void linkOrCopy(Path src, Path dst) throws IOException {
    Files.deleteIfExists(dst);
    try {
      Files.createLink(dst, src);
      return;
    } catch (IOException | UnsupportedOperationException e) {
      // Another volume or no hard links: copy.
    }
    copyRange(src, 0, Files.size(src), dst);
  }

  static void force(Path file) throws IOException {
    try (FileChannel ch = FileChannel.open(file, StandardOpenOption.WRITE)) {
      ch.force(true);
    }
  }

  static void truncate(Path file, long bytes) throws IOException {
    try (FileChannel ch = FileChannel.open(file, StandardOpenOption.WRITE)) {
      ch.truncate(bytes);
      ch.force(true);
    }
  }

  static void deleteTree(Path root) throws IOException {
    if (!Files.exists(root)) return;
    try (var walk = Files.walk(root)) {
      List<Path> all = walk.sorted(Comparator.reverseOrder()).toList();
      for (Path p : all) Files.deleteIfExists(p);
    }
  }

  @SuppressWarnings("unchecked")
  static Map<String, Object> map(Map<String, Object> m, String key) {
    return (Map<String, Object>) m.get(key);
  }

  static long[] longs(Object v) {
    return (long[]) v;
  }

  static int[] ints(Object v) {
    return (int[]) v;
  }

  static boolean[] booleans(Object v) {
    return (boolean[]) v;
  }

  // ---- Exact binary encoding of nested maps, lists, numbers, strings and primitive arrays

  static final class Codec {
    private Codec() {}

    static void write(Path file, Map<String, Object> value) throws IOException {
      try (DataOutputStream out = new DataOutputStream(new BufferedOutputStream(Files.newOutputStream(file), 1 << 16))) {
        out.writeUTF(MAGIC);
        value(out, value);
      }
    }

    @SuppressWarnings("unchecked")
    static Map<String, Object> read(Path file) throws IOException {
      try (DataInputStream in = new DataInputStream(new BufferedInputStream(Files.newInputStream(file), 1 << 16))) {
        if (!MAGIC.equals(in.readUTF())) throw new IOException("Not a dense checkpoint file: " + file);
        return (Map<String, Object>) value(in);
      }
    }

    private static void string(DataOutputStream out, String s) throws IOException {
      byte[] b = s.getBytes(StandardCharsets.UTF_8);
      out.writeInt(b.length);
      out.write(b);
    }

    private static String string(DataInputStream in) throws IOException {
      byte[] b = new byte[in.readInt()];
      in.readFully(b);
      return new String(b, StandardCharsets.UTF_8);
    }

    private static void value(DataOutputStream out, Object v) throws IOException {
      if (v == null) out.writeByte('N');
      else if (v instanceof Boolean b) { out.writeByte('Z'); out.writeBoolean(b); }
      else if (v instanceof Integer i) { out.writeByte('i'); out.writeInt(i); }
      else if (v instanceof Long l) { out.writeByte('J'); out.writeLong(l); }
      else if (v instanceof Double d) { out.writeByte('D'); out.writeDouble(d); }
      else if (v instanceof Float f) { out.writeByte('f'); out.writeFloat(f); }
      else if (v instanceof Number n) { out.writeByte('D'); out.writeDouble(n.doubleValue()); }
      else if (v instanceof String s) { out.writeByte('S'); string(out, s); }
      else if (v instanceof Path p) { out.writeByte('S'); string(out, p.toString()); }
      else if (v instanceof long[] a) { out.writeByte('L'); out.writeInt(a.length); for (long x : a) out.writeLong(x); }
      else if (v instanceof int[] a) { out.writeByte('I'); out.writeInt(a.length); for (int x : a) out.writeInt(x); }
      else if (v instanceof double[] a) { out.writeByte('F'); out.writeInt(a.length); for (double x : a) out.writeDouble(x); }
      else if (v instanceof boolean[] a) { out.writeByte('B'); out.writeInt(a.length); for (boolean x : a) out.writeBoolean(x); }
      else if (v instanceof Map<?, ?> m) {
        out.writeByte('M');
        out.writeInt(m.size());
        for (var e : m.entrySet()) {
          string(out, String.valueOf(e.getKey()));
          value(out, e.getValue());
        }
      } else if (v instanceof Collection<?> c) {
        out.writeByte('A');
        out.writeInt(c.size());
        for (Object x : c) value(out, x);
      } else if (v instanceof Object[] a) {
        out.writeByte('A');
        out.writeInt(a.length);
        for (Object x : a) value(out, x);
      } else throw new IOException("Cannot save " + v.getClass().getName() + " in a dense checkpoint");
    }

    private static Object value(DataInputStream in) throws IOException {
      int tag = in.readByte();
      switch (tag) {
        case 'N': return null;
        case 'Z': return in.readBoolean();
        case 'i': return in.readInt();
        case 'J': return in.readLong();
        case 'D': return in.readDouble();
        case 'f': return in.readFloat();
        case 'S': return string(in);
        case 'L': { long[] a = new long[in.readInt()]; for (int i = 0; i < a.length; i++) a[i] = in.readLong(); return a; }
        case 'I': { int[] a = new int[in.readInt()]; for (int i = 0; i < a.length; i++) a[i] = in.readInt(); return a; }
        case 'F': { double[] a = new double[in.readInt()]; for (int i = 0; i < a.length; i++) a[i] = in.readDouble(); return a; }
        case 'B': { boolean[] a = new boolean[in.readInt()]; for (int i = 0; i < a.length; i++) a[i] = in.readBoolean(); return a; }
        case 'M': {
          int n = in.readInt();
          Map<String, Object> m = new LinkedHashMap<>();
          for (int i = 0; i < n; i++) {
            String k = string(in);
            m.put(k, value(in));
          }
          return m;
        }
        case 'A': {
          int n = in.readInt();
          List<Object> l = new ArrayList<>(n);
          for (int i = 0; i < n; i++) l.add(value(in));
          return l;
        }
        default: throw new IOException("Corrupt dense checkpoint (tag " + tag + ")");
      }
    }
  }
}
