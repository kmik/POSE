package sgm;

import java.io.*;
import java.util.*;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;

final class FeatureTracks {
  static final class Match {
    final int a, b;

    Match(int a, int b) {
      this.a = a;
      this.b = b;
    }
  }

  /**
   * The matches of one pair as packed keypoint ids (8 bytes per match instead of a Match object
   * plus a list slot, about 32 bytes). Large blocks keep tens of millions of matches until
   * alignment. Elements are created on read; Match has no identity, only its two ids.
   */
  static final class Matches extends AbstractList<Match> implements RandomAccess {
    private int[] ids = new int[16];
    private int size;

    @Override
    public Match get(int i) {
      Objects.checkIndex(i, size);
      return new Match(ids[2 * i], ids[2 * i + 1]);
    }

    @Override
    public int size() {
      return size;
    }

    @Override
    public boolean add(Match m) {
      if (2 * size + 2 > ids.length) ids = Arrays.copyOf(ids, Math.max(16, ids.length * 2));
      ids[2 * size] = m.a;
      ids[2 * size + 1] = m.b;
      size++;
      modCount++;
      return true;
    }

    @Override
    public void clear() {
      ids = new int[16];
      size = 0;
      modCount++;
    }

    /** Releases the spare capacity once the pair is complete. */
    void trim() {
      if (ids.length > 2 * size) ids = Arrays.copyOf(ids, Math.max(2, 2 * size));
    }
  }

  static final class Pair {
    final int a, b;
    final Matches matches = new Matches();
    double[] rotation, translation;
    double[] essential;
    String reason = "not_estimated";

    Pair(int a, int b) {
      this.a = a;
      this.b = b;
    }
  }

  static final class Observation {
    final int image, feature;

    Observation(int image, int feature) {
      this.image = image;
      this.feature = feature;
    }
  }

  static final class Track {
    final List<Observation> observations = new ArrayList<>();
    double[] point;
    int id = -1;
    // Optional known position (ground control point) and its per-axis sigma, in world units.
    double[] prior, priorSigma;
  }

  final List<Pair> pairs = new ArrayList<>();
  final List<Track> tracks = new ArrayList<>();
  Pair seed;

  void build(List<ImageFrame> frames, Options o) {
    try {
      build(frames, o, null);
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  /**
   * Extracts (or reuses cached) features for these frames, a few images at a time. OpenCV's SIFT
   * uses only a few cores per image, and detection is deterministic per image, so the keypoints
   * and descriptors equal the serial loop's. Everything order-dependent (colours, exports,
   * descriptor-cache accounting, counts) stays in the caller's ordered loop. A frame whose
   * concurrent extraction failed is marked false and extracted again serially by the caller.
   */
  static boolean[] extractConcurrently(List<ImageFrame> frames, Options o) {
    boolean[] done = new boolean[frames.size()];
    int threads = Math.max(1, Math.min(frames.size(), Math.min(4, o.cores / 3)));
    if (threads < 2) return done;
    java.util.concurrent.ExecutorService pool =
        java.util.concurrent.Executors.newFixedThreadPool(
            threads,
            r -> {
              Thread t = new Thread(r, "feature-extraction");
              t.setDaemon(true);
              return t;
            });
    try {
      List<java.util.concurrent.Future<?>> tasks = new ArrayList<>();
      for (int k = 0; k < frames.size(); k++) {
        int index = k;
        tasks.add(
            pool.submit(
                () -> {
                  ImageFrame f = frames.get(index);
                  try (MemoryGovernor.Admission memory =
                          MemoryGovernor.admit("feature extraction", MemoryGovernor.featureBytes(f, o), o);
                      WorkProgress work =
                      WorkProgress.start("Feature extraction " + f.path.getFileName(), 0, "keypoints")) {
                    ReuseCache.features(f, o);
                    work.done();
                  }
                  done[index] = true;
                  return null;
                }));
      }
      for (int k = 0; k < tasks.size(); k++)
        try {
          tasks.get(k).get();
        } catch (java.util.concurrent.ExecutionException e) {
          done[k] = false;
          System.err.println(
              "Warning: concurrent feature extraction of "
                  + frames.get(k).path.getFileName()
                  + " failed ("
                  + e.getCause()
                  + "); extracting it serially");
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
          throw new IllegalStateException("Feature extraction interrupted", e);
        }
    } finally {
      pool.shutdownNow();
    }
    return done;
  }

  void build(List<ImageFrame> frames, Options o, Project project) throws IOException {
    if (project != null)
      project.stage(
          "02_features",
          "running",
          o.matchingAlgorithm + " extraction on processed undistorted images");
    int[] offset = new int[frames.size() + 1];
    int chunk = Math.max(1, o.cores);
    boolean[] extracted = null;
    WorkProgress extraction = WorkProgress.start("Feature extraction", frames.size(), "images");
    for (int i = 0; i < frames.size(); i++) {
      extraction.update(i);
      // F1: decode the next chunk's rasters in parallel (identical pixels). Fingerprinting, cache
      // reuse, extraction and output below stay serial and in the original order.
      if (i % chunk == 0 && chunk > 1) {
        long preloadStarted = System.nanoTime();
        int decoded =
            RasterCache.preload(frames.subList(i, Math.min(frames.size(), i + chunk)), chunk);
        if (decoded > 0)
          RunTelemetry.record("Parallel feature image preload (" + decoded + ")", preloadStarted);
        extracted = extractConcurrently(frames.subList(i, Math.min(frames.size(), i + chunk)), o);
      }
      if (extracted == null || !extracted[i % chunk])
        try (WorkProgress work =
            WorkProgress.start(
                "Feature extraction " + frames.get(i).path.getFileName(), 0, "keypoints")) {
          ReuseCache.features(frames.get(i), o);
          work.done();
        }
      frames.get(i).captureFeatureColors();
      if (project != null) project.data.features(frames.get(i));
      DescriptorCache.retain(frames.get(i));
      offset[i + 1] = Math.addExact(offset[i], frames.get(i).keypoints.length);
      System.out.printf(
          Locale.ROOT,
          "Features %d/%d: %d%n",
          i + 1,
          frames.size(),
          frames.get(i).keypoints.length);
    }
    extraction.done();
    extraction.close();
    if (project != null && ReuseCache.matches(frames, this, o, false)) {
      PairMatching.clear();
      if (project != null) {
        project.stage(
            "02_features",
            "complete",
            "Keypoints exported; compatible feature caches reused where available");
        project.stage(
            "03_matches",
            "complete",
            "Verified cached matches, tracks and original artifacts reused");
      }
      return;
    }
    pairs.clear();
    tracks.clear();
    UnionFind uf = new UnionFind(offset[frames.size()]);
    if (project != null) {
      project.stage("02_features", "complete", "All keypoints and descriptors exported");
      project.stage("03_matches", "running", "Reciprocal matching, RANSAC and track building");
    }
    int reusable = PairMatching.remembered();
    int[] reused = {0};
    List<int[]> edges = PairSelection.select(frames, o, project);
    WorkProgress matching = WorkProgress.start("Matching image pairs", edges.size(), "pairs");
    long[] consumed = {0};
    MatchCheckpoint checkpoint = MatchCheckpoint.open(o, project);
    boolean matchingFinished = false;
    try {
    PairMatching.Task<Result> task =
        edge -> {
          if (checkpoint != null) {
            MatchCheckpoint.Record record = checkpoint.get(frames.get(edge[0]), frames.get(edge[1]));
            if (record != null) {
              Result result = new Result(record.pair());
              result.restored = record;
              return result;
            }
          }
          try (WorkProgress work =
              WorkProgress.start("Match/verify pair " + edge[0] + "-" + edge[1], 0, "matches")) {
            Result result = match(frames.get(edge[0]), frames.get(edge[1]), o, work);
            work.done();
            return result;
          }
        };
    PairMatching.Sink<Result> sink =
        (edge, result) -> {
          int a = edge[0], b = edge[1];
          Pair pair = result.pair;
          matching.update(++consumed[0]);
          matching.detail("pair " + a + "-" + b);
          try {
            // Exports and track joins stay on this thread, in the original pair order.
            if (project != null && result.restored != null)
              project.data.compactMatches(frames.get(a), frames.get(b), pair, result.restored.queries(),
                  result.restored.ransac(), result.restored.cheirality());
            else if (project != null)
              project.data.matches(
                  frames.get(a),
                  frames.get(b),
                  pair,
                  result.forward,
                  result.reverse,
                  result.ransacFlags,
                  result.cheiralityFlags);
          } finally {
            for (Mat m : result.forward) m.release();
          }
          if (checkpoint != null && result.restored == null)
            checkpoint.put(frames.get(a), frames.get(b), pair, result.forward.size(),
                result.ransacFlags.size(), result.cheiralityFlags.size());
          if (result.reused) reused[0]++;
          if (pair.matches.size() < o.minMatches) return;
          pair.matches.trim();
          pairs.add(pair);
          for (Match m : pair.matches) uf.join(offset[a] + m.a, offset[b] + m.b);
          System.out.printf(
              Locale.ROOT, "Pair %d-%d: %d geometric inliers%n", a, b, pair.matches.size());
        };
    PairMatching.ordered(edges, PairMatching.threads(o), MemoryGovernor.pairBytes(o), o, task, sink);
    if (o.matchRepair.equals("on")) repair(frames, edges, o, task, sink);
    matchingFinished = true;
    } finally {
      if (checkpoint != null) {
        if (checkpoint.restored > 0)
          System.out.printf(Locale.ROOT, "Pair matching: %,d pairs restored from the checkpoint, %,d matched%n",
              checkpoint.restored, checkpoint.written);
        checkpoint.close(matchingFinished);
      }
    }
    GpuHamming.close();
    matching.done();
    matching.close();
    if (reusable > 0)
      System.out.printf(
          Locale.ROOT,
          "Pair matching: %d descriptor searches reused from focal initialization (%d available)%n",
          reused[0],
          reusable);
    if (project != null) PairMatching.clear();
    // Unmatched keypoints form one-keypoint components that are never kept. Only the debug
    // export lists them, so other runs skip their objects (one per unmatched keypoint of every
    // image); the kept tracks and their order are the same either way.
    boolean allComponents = project != null && o.diagnostics && o.storage.equals("debug");
    int unmatched = 0;
    Map<Integer, Track> groups = new TreeMap<>();
    for (int i = 0; i < frames.size(); i++)
      for (int k = 0; k < frames.get(i).keypoints.length; k++) {
        int root = uf.find(offset[i] + k);
        if (!allComponents && uf.size[root] < 2) {
          unmatched++;
          continue;
        }
        groups.computeIfAbsent(root, r -> new Track()).observations.add(new Observation(i, k));
      }
    int conflicts = 0, singletons = unmatched;
    for (Track track : groups.values()) {
      Set<Integer> seen = new HashSet<>();
      boolean conflict = false;
      for (Observation observation : track.observations)
        if (!seen.add(observation.image)) {
          conflict = true;
          break;
        }
      if (conflict) conflicts++;
      else if (track.observations.size() < 2) singletons++;
      else {
        track.id = tracks.size();
        tracks.add(track);
      }
    }
    if (project != null) {
      project.data.tracks(tracks, groups, conflicts, singletons);
      project.stage(
          "03_matches",
          "complete",
          pairs.size() + " accepted stereo pairs; " + tracks.size() + " retained tracks");
    }
    if (project != null) ReuseCache.matches(frames, this, o, true);
  }

  /**
   * --match-repair: one extra round of matching for weak images (MatchRepair.plan). The pairs go
   * through the same verification and join the same tracks; 03_matches/match_repair.json lists
   * the weak images, the pairs tried and which were accepted.
   */
  private void repair(
      List<ImageFrame> frames,
      List<int[]> evaluated,
      Options o,
      PairMatching.Task<Result> task,
      PairMatching.Sink<Result> sink)
      throws IOException {
    long started = System.nanoTime();
    MatchRepair.Plan plan = MatchRepair.plan(frames, pairs, evaluated, o);
    Set<Long> before = new HashSet<>();
    for (Pair p : pairs) before.add(PairSelection.key(p.a, p.b));
    System.out.printf(
        Locale.ROOT,
        "Match repair: %d weak images (match graph: %d parts, largest %d of %d images); %d extra"
            + " pairs%n",
        plan.weak().size(),
        plan.components(),
        plan.largestComponent(),
        frames.size(),
        plan.edges().size());
    if (!plan.edges().isEmpty())
      try (WorkProgress work = WorkProgress.start("Match repair", plan.edges().size(), "pairs")) {
        PairMatching.ordered(plan.edges(), PairMatching.threads(o), MemoryGovernor.pairBytes(o), o, task, sink);
        work.done();
      }
    Map<Long, Integer> added = new HashMap<>();
    for (Pair p : pairs) {
      long key = PairSelection.key(p.a, p.b);
      if (!before.contains(key)) added.put(key, p.matches.size());
    }
    int[] after = MatchRepair.components(frames.size(), pairs);
    int parts = 0, largest = 0;
    for (int c : after) parts = Math.max(parts, c + 1);
    for (int c : after) if (c == 0) largest++;
    List<Object> weak = new ArrayList<>();
    for (MatchRepair.Weak w : plan.weak()) {
      List<Object> tried = new ArrayList<>();
      for (int other : w.candidates()) {
        Integer matches = added.get(PairSelection.key(w.image(), other));
        tried.add(
            Json.object(
                "image_id", other,
                "filename", frames.get(other).path.getFileName().toString(),
                "accepted", matches != null,
                "matches", matches == null ? 0 : matches));
      }
      weak.add(
          Json.object(
              "image_id", w.image(),
              "filename", frames.get(w.image()).path.getFileName().toString(),
              "reason", w.reason(),
              "accepted_pairs_before", w.acceptedPairs(),
              "component_before", w.component(),
              "candidates", tried));
    }
    double seconds = (System.nanoTime() - started) / 1e9;
    java.nio.file.Path file = o.output.resolve("03_matches/match_repair.json");
    java.nio.file.Files.createDirectories(file.getParent());
    Json.write(
        file,
        Json.object(
            "min_accepted_pairs", o.matchRepairMinPairs,
            "candidates_per_image", o.matchRepairCandidates,
            "candidate_order",
                frames.stream().allMatch(f -> f.gps != null && f.gps.usable())
                    ? "gps_horizontal_distance"
                    : "filename_order",
            "weak_images", weak,
            "extra_pairs", plan.edges().size(),
            "accepted_extra_pairs", added.size(),
            "components_before", plan.components(),
            "largest_component_before", plan.largestComponent(),
            "components_after", parts,
            "largest_component_after", largest,
            "seconds", seconds));
    System.out.printf(
        Locale.ROOT,
        "Match repair: %d of %d extra pairs accepted; match graph now %d parts, largest %d of %d"
            + " images (%.1f s)%n",
        added.size(),
        plan.edges().size(),
        parts,
        largest,
        frames.size(),
        seconds);
  }

  /** One verified pair plus the nearest-neighbor decisions its export needs. */
  private static final class Result {
    final Pair pair;
    final List<MatOfDMatch> forward = new ArrayList<>();
    final Map<Integer, Integer> reverse = new HashMap<>();
    final Set<Integer> ransacFlags = new HashSet<>(), cheiralityFlags = new HashSet<>();
    final boolean[] reusedSearch = new boolean[1];
    boolean reused;
    /** Set when the pair came from the step's checkpoint (no match lists then). */
    MatchCheckpoint.Record restored;

    Result(Pair pair) {
      this.pair = pair;
    }
  }

  private static Result match(ImageFrame a, ImageFrame b, Options o, WorkProgress work)
      throws IOException {
    DescriptorCache.acquire(a, b);
    Result result = new Result(new Pair(a.id, b.id));
    try {
      verify(a, b, o, work, result);
      result.reused = result.reusedSearch[0];
      return result;
    } catch (RuntimeException | Error e) {
      for (Mat m : result.forward) m.release();
      throw e;
    } finally {
      DescriptorCache.release(a, b);
    }
  }

  private static void verify(ImageFrame a, ImageFrame b, Options o, WorkProgress work, Result res) {
    Pair out = res.pair;
    if (a.descriptors.empty() || b.descriptors.rows() < 2) {
      out.reason = "missing_descriptors";
      return;
    }
    List<MatOfDMatch> forward = res.forward;
    Map<Integer, Integer> reverse = res.reverse;
    Set<Integer> ransacFlags = res.ransacFlags, cheiralityFlags = res.cheiralityFlags;
    Mat p1 = new Mat(),
        p2 = new Mat(),
        mask = new Mat(),
        e = new Mat(),
        r = new Mat(),
        t = new Mat();
    try {
      PairMatching.reciprocal(
          o, a, b, forward, reverse, work::detail, false, res.reusedSearch);
      work.detail("essential RANSAC and cheirality");
      List<Match> accepted = new ArrayList<>();
      List<Point> x = new ArrayList<>(), y = new ArrayList<>();
      for (MatOfDMatch row : forward) {
        DMatch[] m = row.toArray();
        if (m.length < 2 || m[0].distance >= o.ratio * m[1].distance) continue;
        DMatch best = m[0];
        if (!Objects.equals(reverse.get(best.trainIdx), best.queryIdx)) continue;
        accepted.add(new Match(best.queryIdx, best.trainIdx));
        x.add(normalize(a.camera, a.keypoints[best.queryIdx].pt));
        y.add(normalize(b.camera, b.keypoints[best.trainIdx].pt));
      }
      if (accepted.size() < o.minMatches) {
        out.reason = "insufficient_reciprocal_ratio_matches";
        return;
      }
      p1.release();
      p2.release();
      p1 = new MatOfPoint2f();
      p2 = new MatOfPoint2f();
      ((MatOfPoint2f) p1).fromList(x);
      ((MatOfPoint2f) p2).fromList(y);
      double threshold = o.reprojection / Math.max(a.camera.fx, b.camera.fx);
      e.release();
      e =
          Calib3d.findEssentialMat(
              p1, p2, 1.0, new Point(0, 0), Calib3d.RANSAC, .999, threshold, 2000, mask);
      if (e.empty() || e.rows() != 3) {
        out.reason = "essential_estimation_failed";
        return;
      }
      out.essential = Math3.array(e);
      byte[] ransacMask = new byte[(int) mask.total()];
      mask.get(0, 0, ransacMask);
      for (int i = 0; i < accepted.size(); i++)
        if (ransacMask[i] != 0) ransacFlags.add(accepted.get(i).a);
      Calib3d.recoverPose(e, p1, p2, r, t, 1.0, new Point(0, 0), mask);
      byte[] inliers = new byte[(int) mask.total()];
      mask.get(0, 0, inliers);
      for (int i = 0; i < accepted.size(); i++)
        if (inliers[i] != 0) {
          out.matches.add(accepted.get(i));
          cheiralityFlags.add(accepted.get(i).a);
        }
      out.rotation = Math3.array(r);
      out.translation = Math3.array(t);
      out.reason =
          out.matches.size() >= o.minMatches ? "accepted" : "insufficient_geometric_inliers";
    } finally {
      p1.release();
      p2.release();
      mask.release();
      e.release();
      r.release();
      t.release();
    }
  }

  static Point normalize(Camera c, Point p) {
    return new Point((p.x - c.cx) / c.fx, (p.y - c.cy) / c.fy);
  }

  private static final class UnionFind {
    final int[] parent, size;

    UnionFind(int n) {
      parent = new int[n];
      size = new int[n];
      for (int i = 0; i < n; i++) {
        parent[i] = i;
        size[i] = 1;
      }
    }

    int find(int x) {
      while (x != parent[x]) {
        parent[x] = parent[parent[x]];
        x = parent[x];
      }
      return x;
    }

    void join(int a, int b) {
      a = find(a);
      b = find(b);
      if (a == b) return;
      if (size[a] < size[b]) {
        int s = a;
        a = b;
        b = s;
      }
      parent[b] = a;
      size[a] += size[b];
    }
  }
}
