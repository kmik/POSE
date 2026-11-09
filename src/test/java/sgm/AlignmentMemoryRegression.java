package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;

/** Linux limits, bounded geometry workspaces, graph admission and batch-order controls. */
public final class AlignmentMemoryRegression {
  private static int checks;
  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    nu.pattern.OpenCV.loadLocally(); Core.setNumThreads(1);
    System.out.println("Live host/job memory limits: " + MemorySafety.limits());
    Path root = Files.createTempDirectory(Path.of("target"), "alignment memory ");
    System.out.println("PASS alignment memory: " + run(root) + "; " + root);
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Files.createDirectories(root);
    linux(root.resolve("linux"));
    heap();
    triangulation();
    graph(root.resolve("graph"));
    batching(root.resolve("batch"));
    return checks;
  }

  private static void write(Path path, String value) throws Exception {
    Files.createDirectories(path.getParent()); Files.writeString(path, value);
  }

  private static void linux(Path root) throws Exception {
    long g = 1024L * 1024 * 1024;
    Path proc = root.resolve("proc"), cg = root.resolve("controller").toAbsolutePath();
    write(proc.resolve("meminfo"), "MemTotal: 285212672 kB\nMemAvailable: 251658240 kB\nCommitLimit: 1048576 kB\nCommitted_AS: 2097152 kB\n");
    write(proc.resolve("sys/vm/overcommit_memory"), "0");
    write(proc.resolve("self/status"), "VmRSS: 1048576 kB\n");
    check(LinuxMemory.read(proc, Map.of()).available() == 240 * g, "Heuristic overcommit does not treat Committed_AS as an allocation limit");
    write(proc.resolve("self/mountinfo"), "1 0 0:1 / " + cg.toString().replace('\\', '/').replace(" ", "\\040") + " rw - cgroup2 cgroup rw\n");
    write(proc.resolve("self/cgroup"), "0::/job/step\n");
    write(cg.resolve("memory.max"), "max");
    write(cg.resolve("job/memory.max"), "" + 228 * g);
    write(cg.resolve("job/memory.current"), "" + 100 * g);
    write(cg.resolve("job/memory.stat"), "inactive_file " + 10 * g + "\n");
    write(cg.resolve("job/step/memory.max"), "max");
    var limits = LinuxMemory.read(proc, Map.of());
    check(limits.total() == 228 * g && limits.available() == 138 * g, "v2 ancestor limit and reclaimable file cache are detected");
    write(cg.resolve("job/step/memory.max"), "" + 180 * g);
    check(LinuxMemory.read(proc, Map.of()).total() == 180 * g, "Known controller cap survives unavailable usage counters");
    check(LinuxMemory.read(proc, Map.of()).available() == 0, "Unknown finite-quota usage is not treated as free RAM");
    write(cg.resolve("job/step/memory.current"), "" + 170 * g);
    check(LinuxMemory.read(proc, Map.of()).available() == 10 * g, "Tighter leaf controller is enforced");
    write(cg.resolve("job/step/memory.current"), "" + 180 * g);
    check(LinuxMemory.read(proc, Map.of()).available() == 0, "Exhausted cgroup is zero, not unknown");
    write(cg.resolve("job/step/memory.current"), "" + 190 * g);
    write(cg.resolve("job/step/memory.stat"), "inactive_file " + 20 * g + "\n");
    check(LinuxMemory.read(proc, Map.of()).available() == 10 * g, "Reclaim estimate subtracts quota overage first");
    // Namespace root maps to the controller mount even when mountinfo names a host job root.
    write(proc.resolve("self/mountinfo"), "1 0 0:1 /host/job " + cg.toString().replace('\\', '/').replace(" ", "\\040") + " rw - cgroup2 cgroup rw\n");
    write(proc.resolve("self/cgroup"), "0::/\n");
    write(cg.resolve("memory.max"), "" + 128 * g);
    write(cg.resolve("memory.current"), "" + 32 * g);
    check(LinuxMemory.read(proc, Map.of()).total() == 128 * g, "Namespaced cgroup root is recognized");
    write(proc.resolve("self/cgroup"), "0::/child\n");
    write(cg.resolve("child/memory.max"), "" + 64 * g);
    write(cg.resolve("child/memory.current"), "" + 16 * g);
    check(LinuxMemory.read(proc, Map.of()).total() == 64 * g, "Namespaced child controller is recognized");
    write(proc.resolve("self/mountinfo"), "1 0 0:1 / " + cg.toString().replace('\\', '/').replace(" ", "\\040") + " rw - cgroup cgroup rw,memory\n");
    write(proc.resolve("self/cgroup"), "7:cpu,memory:/job\n");
    write(cg.resolve("job/memory.limit_in_bytes"), "" + 200 * g);
    write(cg.resolve("job/memory.usage_in_bytes"), "" + 60 * g);
    write(cg.resolve("job/memory.stat"), "total_inactive_file " + 5 * g + "\n");
    check(LinuxMemory.read(proc, Map.of()).available() == 145 * g, "v1 controller and ancestor usage are recognized");
    write(proc.resolve("self/mountinfo"), "");
    check(LinuxMemory.read(proc, Map.of("SLURM_JOB_ID", "7", "SLURM_MEM_PER_NODE", "65536")).total() == 64 * g,
        "Slurm node allocation is recognized without a visible controller");
    check(LinuxMemory.read(proc, Map.of("SLURM_JOB_ID", "7", "SLURM_MEM_PER_CPU", "2048", "SLURM_CPUS_ON_NODE", "16")).total() == 32 * g,
        "Slurm per-CPU allocation is converted using allocated node CPUs");
    check(LinuxMemory.read(proc, Map.of("SLURM_MEM_PER_NODE", "16")).total() == 272 * g,
        "Stale scheduler variables outside a job are ignored");
    write(proc.resolve("sys/vm/overcommit_memory"), "2");
    check(LinuxMemory.read(proc, Map.of()).available() == 0, "Strict host commit limit is enforced");
    check(LinuxMemory.read(root.resolve("missing"), Map.of()).available() == -1, "Unavailable metrics stay unknown");
  }

  private static void heap() {
    long m = MemorySafety.MIB;
    check(MemoryLauncher.requestedRamLimit(new String[] {"--ram-limit-mb", "1", "--ram-limit-mb=233472"}) == 233472,
        "Last explicit RAM ceiling wins");
    long safe = MemoryLauncher.safeHeapMiB(new MemorySafety.Limits(233472 * m, 233472 * m, "fixture"), 0);
    check(safe <= 233472 * .65 && safe > 233472 * .64, "228 GiB job leaves at least 35 percent outside heap");
    check(MemoryLauncher.safeHeapMiB(new MemorySafety.Limits(1024 * m, 0, "fixture"), 0) == 0, "Zero available RAM never becomes a minimum oversized heap");
    check(MemoryLauncher.safeHeapMiB(new MemorySafety.Limits(-1, -1, "unknown"), 0) == Long.MAX_VALUE, "Unknown RAM does not invent a cap");
    Options o = Options.parse(new String[] {"-i", ".", "--ram-limit-mb", "1024"});
    check(o.ramLimitMb == 1024 && o.copy().ramLimitMb == 1024, "RAM ceiling parses and survives option copies");
    boolean refused = false;
    try { Options.parse(new String[] {"-i", ".", "--ram-limit-mb", "-1"}); }
    catch (IllegalArgumentException expected) { refused = true; }
    check(refused, "Negative RAM ceiling rejected");
  }

  private static void triangulation() {
    Camera a = new Camera(600, 600, 320, 240), b = a.copy();
    b.rotation = Math3.rotation(new double[] {.02, -.04, .01});
    b.translation = new double[] {-1, .1, .02};
    var workspace = Geometry.triangulator();
    long matrix = workspace.matrix.nativeObj, vt = workspace.vt.nativeObj;
    Random rng = new Random(652);
    for (int i = 0; i < 100000; i++) {
      double[] truth = {rng.nextDouble() - .5, rng.nextDouble() - .5, 3 + rng.nextDouble() * 10};
      Point u = FeatureTracks.normalize(a, new Point(a.project(truth))), v = FeatureTracks.normalize(b, new Point(b.project(truth)));
      double[] actual = Geometry.triangulate(a, b, u, v);
      if (i < 1000) check(Arrays.equals(actual, legacy(a, b, u, v)), "Reused native SVD equals original per-point SVD bit for bit");
    }
    check(Geometry.triangulator() == workspace && workspace.matrix.nativeObj == matrix && workspace.vt.nativeObj == vt,
        "100,000 triangulations reuse the same native headers");
    Geometry.releaseWorkspaces();
    check(workspace.matrix.empty(), "Geometry phase releases workspace buffers");
  }

  private static double[] legacy(Camera a, Camera b, Point u, Point v) {
    Mat p = a.projection(), q = b.projection(), matrix = new Mat(4, 4, CvType.CV_64F), w = new Mat(), left = new Mat(), vt = new Mat();
    try {
      double[] pa = Math3.array(p), pb = Math3.array(q), data = new double[16];
      for (int j = 0; j < 4; j++) {
        data[j] = u.x * pa[8 + j] - pa[j]; data[4 + j] = u.y * pa[8 + j] - pa[4 + j];
        data[8 + j] = v.x * pb[8 + j] - pb[j]; data[12 + j] = v.y * pb[8 + j] - pb[4 + j];
      }
      matrix.put(0, 0, data); Core.SVDecomp(matrix, w, left, vt);
      double[] h = new double[4]; vt.get(3, 0, h);
      return new double[] {h[0] / h[3], h[1] / h[3], h[2] / h[3]};
    } finally { p.release(); q.release(); matrix.release(); w.release(); left.release(); vt.release(); }
  }

  private static void graph(Path root) throws Exception {
    PipelineState state = PipelineState.create(root, new String[0]);
    FeatureTracks data = new FeatureTracks();
    for (int i = 0; i < 20000; i++) {
      var t = new FeatureTracks.Track(); t.id = i; t.point = new double[] {i, 2, 3};
      for (int j = 0; j < 4; j++) t.observations.add(new FeatureTracks.Observation((i + j) % 3001, i / 3001));
      data.tracks.add(t);
    }
    for (int i = 0; i < 3000; i++) {
      var p = new FeatureTracks.Pair(i, i + 1); p.matches.add(new FeatureTracks.Match(2, 3)); data.pairs.add(p);
    }
    data.seed = data.pairs.get(5); state.writeTracks(data);
    long estimate = PipelineState.trackGraphBytes(state.directory.resolve("tracks.bin"));
    check(estimate == AlignmentMemory.graphBytes(0, 20000, 80000, 3000, 3000), "Graph preflight counts match 3,001-camera fixture exactly");
    Options options = TestSupport.options(); options.memoryMb = 256;
    var restored = state.readTracks(options);
    check(restored.tracks.size() == 20000 && restored.pairs.size() == 3000 && restored.seed == restored.pairs.get(5), "Guarded graph restores exact track/pair/seed identities");
    options.memoryMb = 1;
    boolean refused = false;
    try { state.readTracks(options); } catch (IllegalArgumentException e) { refused = e.getMessage().contains("Restore sparse tracks"); }
    check(refused, "Oversized graph rejected before materialization");
    Path truncated = root.resolve("truncated.bin"); Files.write(truncated, new byte[12]);
    boolean corrupt = false;
    try { PipelineState.trackGraphBytes(truncated); } catch (IOException expected) { corrupt = true; }
    check(corrupt, "Corrupt graph cannot silently bypass preflight");
  }

  private static void batching(Path root) throws Exception {
    Files.createDirectories(root);
    String previous = System.getProperty("sgm.test.bundleBatchObservations");
    try {
      for (boolean shared : new boolean[] {false, true}) {
      double[] reference = null;
      for (int budget : new int[] {32768, 17}) {
        System.setProperty("sgm.test.bundleBatchObservations", "" + budget);
        Options o = TestSupport.options(); o.output = root; o.cores = 4; o.selfCalibration = false;
        Calibration c = new Calibration(650, 650, 319.5, 239.5, new double[5], 640, 480);
        var frames = SharedCalibrationRegression.frames(root, c, o);
        try {
          var tracks = SharedCalibrationRegression.scene(frames, c);
          for (var t : tracks) t.point = Math3.add(t.point, new double[] {.01, -.01, .02});
          frames.get(0).camera.fixed = true; frames.get(1).camera.fixed = true;
          double before = BundleAdjuster.error(frames, tracks, 2);
          SharedCalibration lens = new SharedCalibration(frames);
          if (shared) {
            lens.active[1] = true; lens.active[3] = true;
            BundleAdjuster.optimizeImpl(frames, tracks, 3, 2, null, 0, 1, lens);
          } else BundleAdjuster.optimize(frames, tracks, 3, 2, null, 0, 1);
          var values = new ArrayList<Double>(); values.add(before);
          for (var f : frames) { for (double x : f.camera.rotation) values.add(x); for (double x : f.camera.translation) values.add(x); }
          for (var t : tracks) for (double x : t.point) values.add(x);
          for (double x : lens.values) values.add(x);
          values.add(lens.validationRms(frames, tracks));
          double[] actual = values.stream().mapToDouble(Double::doubleValue).toArray();
          if (reference == null) reference = actual;
          else check(Arrays.equals(reference, actual), "Changing Jacobian/residual batch size preserves poses, points and RMS bit for bit");
          int limit = BundleAdjuster.linearizationObservationLimit(o, true);
          for (int i = 0; i < tracks.size();) {
            int end = BundleAdjuster.linearizationBatchEnd(tracks, i, limit);
            long observations = 0; for (int j = i; j < end; j++) observations += tracks.get(j).observations.size();
            check(end > i && (observations <= limit || end == i + 1), "Jacobian batches remain bounded and advance"); i = end;
          }
          // An untriangulated track allocates no Jacobians, even if its observation list is long.
          FeatureTracks.Track missing = new FeatureTracks.Track();
          for (int i = 0; i < 40000; i++) missing.observations.add(new FeatureTracks.Observation(0, 0));
          o.memoryMb = 16;
          BundleAdjuster.optimizeImpl(frames, List.of(missing), 1, 2, null, 0, 1, null);
          check(missing.point == null, "Untriangulated long track does not reserve a nonexistent Jacobian payload");
        } finally { for (var f : frames) f.close(); Geometry.releaseWorkspaces(); }
      }
      }
    } finally {
      if (previous == null) System.clearProperty("sgm.test.bundleBatchObservations"); else System.setProperty("sgm.test.bundleBatchObservations", previous);
    }
  }
}
