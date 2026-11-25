package sgm;

import java.io.IOException;
import java.nio.file.*;
import java.util.*;

/**
 * {@code --dense-checkpoint} for {@code --dense-method patchmatch} (PATCHMATCH.md): a resumable
 * PatchMatch solve. Rerunning an interrupted dense step on the same run continues at the pass and
 * reference after the last committed checkpoint; the maps, the cloud and the reports (apart from
 * timing) are the same as an uninterrupted run, because every pass reads only the previous
 * generation's map files, which a checkpoint keeps.
 *
 * <p>A checkpoint holds the stage ({@code solve}, {@code solved} after the last pass, {@code
 * filtered} after --patchmatch-smooth/--patchmatch-speckle), the level and pass, the references
 * finished in that pass, and every reference's report entries. It refers to map files in
 * 05_stereo/reference_N (the previous generation and the finished references of the current pass;
 * final or filtered maps), which therefore stay until the next commit or until fusion completes.
 * The evidence stream and fusion after the solve are not checkpointed: a resume after the solve
 * reruns them from the saved maps.
 *
 * <p>Used only when the generations are on disk: {@code auto} enables it for runs whose
 * generations do not fit in memory (the large ones), an explicit interval forces disk generations;
 * an existing valid checkpoint is always resumed. Same fingerprint as {@link DenseCheckpoint}
 * (every result-relevant option and the earlier steps' saved state); otherwise it is deleted and
 * the solve starts over. The state is written to a new generation folder, then the manifest is
 * replaced atomically.
 */
final class PatchMatchCheckpoint {
  static final int VERSION = 1;
  static final String MANIFEST = "checkpoint.bin", MAGIC = "POSE-PATCHMATCH-CHECKPOINT";

  final Path dir;
  private final String fingerprint;
  private final TreeMap<String, String> parts;
  private final long intervalNanos;
  private final int everyReferences;
  private final boolean forced;
  private Map<String, Object> resume;
  private int generation, written;
  private long last = System.nanoTime(), writeNanos;
  private int since;

  private PatchMatchCheckpoint(Path dir, TreeMap<String, String> parts, long intervalNanos, int every, boolean forced) {
    this.dir = dir;
    this.parts = parts;
    this.fingerprint = DenseCheckpoint.digest(parts.toString());
    this.intervalNanos = intervalNanos;
    this.everyReferences = every;
    this.forced = forced;
  }

  /**
   * The checkpoint of this solve, or null (off, or auto with in-memory generations and no earlier
   * checkpoint). A stale checkpoint is deleted.
   */
  static PatchMatchCheckpoint open(Options o, boolean inMemory) throws IOException {
    Path dir = o.output.resolve("05_stereo/patchmatch_checkpoint");
    if (o.denseCheckpoint.equals("off")) {
      if (Files.isDirectory(dir)) {
        System.out.println("PatchMatch checkpoint: off (--dense-checkpoint off); deleting the earlier checkpoint");
        DenseCheckpoint.deleteTree(dir);
      }
      return null;
    }
    boolean auto = o.denseCheckpoint.equals("auto");
    long minutes = auto ? 30 : Long.parseLong(o.denseCheckpoint);
    PatchMatchCheckpoint cp =
        new PatchMatchCheckpoint(dir, DenseCheckpoint.fingerprintParts(o), minutes * 60_000_000_000L,
            Integer.getInteger("sgm.dense.checkpointCameras", 0), !auto);
    cp.load();
    if (cp.resume == null && auto && inMemory) return null;
    return cp;
  }

  private void load() throws IOException {
    Path manifest = dir.resolve(MANIFEST);
    if (!Files.isRegularFile(manifest)) {
      if (Files.isDirectory(dir)) DenseCheckpoint.deleteTree(dir);
      return;
    }
    String problem = null;
    try {
      Map<String, Object> head = DenseCheckpoint.Codec.read(manifest);
      if (!MAGIC.equals(head.get("magic")) || ((Number) head.get("version")).intValue() != VERSION)
        problem = "written by another version";
      else if (!fingerprint.equals(head.get("fingerprint")))
        problem = "changed since it was written: "
            + String.join(", ", DenseCheckpoint.changed(DenseCheckpoint.map(head, "parts"), parts));
      else {
        generation = ((Number) head.get("generation")).intValue();
        resume = DenseCheckpoint.Codec.read(generationDir(generation).resolve("state.bin"));
      }
    } catch (IOException | RuntimeException e) {
      problem = "unreadable (" + e.getMessage() + ")";
    }
    if (problem != null) discard(problem);
  }

  /** Drops the checkpoint (the solve starts from the beginning). */
  void discard(String why) throws IOException {
    System.out.println("PatchMatch checkpoint ignored: " + why + "; the solve starts from the beginning");
    resume = null;
    generation = 0;
    if (Files.isDirectory(dir)) DenseCheckpoint.deleteTree(dir);
  }

  /** The state to resume from, or null. */
  Map<String, Object> resume() {
    return resume;
  }

  /** Disk generations are needed (an explicit interval, or a checkpoint being resumed). */
  boolean forcesDisk() {
    return forced || resume != null;
  }

  /** A commit exists: the files it refers to must stay until the next one. */
  boolean committed() {
    return generation > 0;
  }

  /** Counts a finished reference; true when a checkpoint is due. */
  boolean due() {
    since++;
    if (everyReferences > 0) return since >= everyReferences;
    return System.nanoTime() - last >= intervalNanos;
  }

  private Path generationDir(int g) {
    return dir.resolve("gen-" + g);
  }

  /** Writes the state and replaces the manifest atomically; the previous generation is deleted. */
  void commit(Map<String, Object> state) throws IOException {
    long started = System.nanoTime();
    Path g = generationDir(generation + 1);
    if (Files.isDirectory(g)) DenseCheckpoint.deleteTree(g);
    Files.createDirectories(g);
    state.put("written_epoch_millis", System.currentTimeMillis());
    DenseCheckpoint.Codec.write(g.resolve("state.bin"), state);
    DenseCheckpoint.force(g.resolve("state.bin"));
    Path temporary = dir.resolve(MANIFEST + ".tmp");
    DenseCheckpoint.Codec.write(temporary, Json.object("magic", MAGIC, "version", VERSION,
        "fingerprint", fingerprint, "generation", generation + 1, "parts", parts));
    DenseCheckpoint.force(temporary);
    Files.move(temporary, dir.resolve(MANIFEST), StandardCopyOption.REPLACE_EXISTING, StandardCopyOption.ATOMIC_MOVE);
    Path previous = generationDir(generation);
    generation++;
    if (Files.isDirectory(previous)) DenseCheckpoint.deleteTree(previous);
    since = 0;
    last = System.nanoTime();
    writeNanos += last - started;
    written++;
    System.out.printf(Locale.ROOT, "PatchMatch checkpoint %d: %s level %s pass %s, %d references done%n",
        generation, state.get("stage"), state.get("level"), state.get("pass"),
        ((List<?>) state.getOrDefault("done", List.of())).size());
  }

  /** The dense step finished: the checkpoint is no longer needed. */
  void complete() throws IOException {
    resume = null;
    if (Files.isDirectory(dir)) DenseCheckpoint.deleteTree(dir);
  }

  Object report(Map<String, Object> resumedFrom) {
    return Json.object("enabled", true, "interval_minutes", intervalNanos / 60_000_000_000L,
        "checkpoints_written", written, "write_seconds", writeNanos / 1e9, "resumed_from", resumedFrom);
  }
}
