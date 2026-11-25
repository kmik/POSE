package sgm;

import java.nio.file.*;
import java.util.*;

/** PatchMatchCheckpoint lifecycle: when it is used, commit/resume, fingerprint, off, complete. */
final class PatchMatchCheckpointRegression {
  private static int checks;

  private static void check(boolean ok, String text) {
    checks++;
    if (!ok) throw new AssertionError(text);
  }

  public static void main(String[] args) throws Exception {
    Path root = Files.createTempDirectory(Path.of("target"), "patchmatch-checkpoint-");
    System.out.println("PASS PatchMatch checkpoint: " + run(root) + "; " + root);
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Options o = TestSupport.options();
    o.output = Files.createDirectories(root.resolve("pm-checkpoint"));
    o.denseMethod = "patchmatch";
    o.denseCheckpoint = "auto";
    Path dir = o.output.resolve("05_stereo/patchmatch_checkpoint");
    check(PatchMatchCheckpoint.open(o, true) == null, "auto with in-memory generations and no checkpoint: none");
    PatchMatchCheckpoint cp = PatchMatchCheckpoint.open(o, false);
    check(cp != null && cp.resume() == null && !cp.committed() && !cp.forcesDisk(), "auto on disk: fresh checkpoint");
    Map<String, Object> state = new LinkedHashMap<>();
    state.put("stage", "solve");
    state.put("level", 1);
    state.put("pass", 2);
    state.put("done", List.of(3, 5));
    state.put("reports", Map.of("3", List.of(Json.object("level", 2, "phase", "photometric"))));
    Files.createDirectories(dir);
    cp.commit(state);
    check(cp.committed() && Files.isRegularFile(dir.resolve(PatchMatchCheckpoint.MANIFEST)), "commit writes the manifest");
    cp.commit(new LinkedHashMap<>(state));
    check(!Files.exists(dir.resolve("gen-1")) && Files.isDirectory(dir.resolve("gen-2")), "older generation removed");
    PatchMatchCheckpoint again = PatchMatchCheckpoint.open(o, true);
    check(again != null && again.forcesDisk(), "a saved checkpoint is resumed even if generations would fit in memory");
    check("solve".equals(again.resume().get("stage")) && ((Number) again.resume().get("pass")).intValue() == 2, "state restored");
    check(((List<?>) again.resume().get("done")).size() == 2, "finished references restored");
    o.cores = o.cores + 3;
    check(PatchMatchCheckpoint.open(o, false).resume() != null, "a resource option may change");
    o.patchmatchSmooth = 3;
    check(PatchMatchCheckpoint.open(o, false).resume() == null && !Files.exists(dir.resolve(PatchMatchCheckpoint.MANIFEST)),
        "a result option change discards the checkpoint");
    o.patchmatchSmooth = 0;
    PatchMatchCheckpoint fresh = PatchMatchCheckpoint.open(o, false);
    Files.createDirectories(dir);
    fresh.commit(new LinkedHashMap<>(state));
    o.denseCheckpoint = "off";
    check(PatchMatchCheckpoint.open(o, false) == null && !Files.exists(dir), "off deletes an earlier checkpoint");
    o.denseCheckpoint = "5";
    PatchMatchCheckpoint forced = PatchMatchCheckpoint.open(o, true);
    check(forced != null && forced.forcesDisk(), "an explicit interval forces disk generations");
    Files.createDirectories(dir);
    forced.commit(new LinkedHashMap<>(state));
    forced.complete();
    check(!Files.exists(dir), "complete removes the checkpoint");
    return checks;
  }
}
