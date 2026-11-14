package sgm;

import java.io.IOException;
import java.nio.file.*;
import java.nio.file.attribute.BasicFileAttributes;
import java.nio.file.attribute.FileTime;
import java.util.*;
import java.util.concurrent.*;

/**
 * SHA-256 of finished output files (the dense clouds), computed on one background thread right
 * after they are published, so the run catalogue does not read multi-GB clouds again at the end.
 * A digest is used only while the file's size, modification time and file key are unchanged
 * (the ground step edits dense.las in place; the catalogue then hashes it again).
 */
final class ArtifactDigests {
  private record Entry(long bytes, FileTime modified, Object key, Future<String> hash) {}

  private static final Map<Path, Entry> digests = new ConcurrentHashMap<>();
  private static ExecutorService pool;

  private ArtifactDigests() {}

  private static synchronized ExecutorService pool() {
    if (pool == null)
      pool =
          Executors.newSingleThreadExecutor(
              r -> {
                Thread t = new Thread(r, "artifact-digest");
                t.setDaemon(true);
                t.setPriority(Thread.MIN_PRIORITY);
                return t;
              });
    return pool;
  }

  static void start(Path file) {
    if (Boolean.getBoolean("sgm.noBackgroundDigest")) return;
    try {
      Path key = file.toAbsolutePath().normalize();
      BasicFileAttributes a = Files.readAttributes(key, BasicFileAttributes.class);
      Future<String> hash =
          pool()
              .submit(
                  () -> {
                    long started = System.nanoTime();
                    String h = Project.sha256(key);
                    RunTelemetry.record("Background SHA-256 " + key.getFileName(), started);
                    return h;
                  });
      digests.put(key, new Entry(a.size(), a.lastModifiedTime(), a.fileKey(), hash));
    } catch (IOException | RuntimeException e) {
      // Only an optimisation: the catalogue hashes the file itself.
    }
  }

  /** The background digest when the file is unchanged since it was started, else null. */
  static String lookup(Path file, BasicFileAttributes now) {
    Entry e = digests.get(file.toAbsolutePath().normalize());
    if (e == null
        || e.bytes != now.size()
        || !e.modified.equals(now.lastModifiedTime())
        || !Objects.equals(e.key, now.fileKey())) return null;
    try {
      return e.hash.get();
    } catch (InterruptedException x) {
      Thread.currentThread().interrupt();
      return null;
    } catch (ExecutionException x) {
      return null;
    }
  }
}
