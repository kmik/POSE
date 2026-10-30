package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import org.opencv.core.Point;

/** Removes bad observations conservatively; never removes a camera's last useful support. */
final class SparseCleanup {
  static void run(
      List<ImageFrame> frames,
      List<FeatureTracks.Track> tracks,
      Options o,
      Project project,
      int anchor,
      int scale)
      throws IOException {
    Path dir = o.output.resolve("04_geometry/cleanup");
    Files.createDirectories(dir);
    boolean fresh = !Files.exists(dir.resolve("decisions.csv"));
    try (BufferedWriter out =
        Files.newBufferedWriter(
            dir.resolve("decisions.csv"), StandardOpenOption.CREATE, StandardOpenOption.APPEND)) {
      if (fresh) out.write("pass,track_id,image_id,residual_pixels,action\n");
      for (int pass = 0; pass < o.cleanupPasses; pass++) {
        int[] support = new int[frames.size()];
        for (var t : tracks)
          if (t.point != null)
            for (var ob : t.observations)
              if (frames.get(ob.image).camera.registered) support[ob.image]++;
        int removed = 0;
        Connectivity graph = new Connectivity(frames, tracks);
        for (var t : tracks)
          if (t.point != null) {
            int views = 0;
            for (var ob : t.observations) if (frames.get(ob.image).camera.registered) views++;
            Iterator<FeatureTracks.Observation> it = t.observations.iterator();
            while (it.hasNext()) {
              var ob = it.next();
              ImageFrame f = frames.get(ob.image);
              if (!f.camera.registered) continue;
              double[] uv = f.camera.project(t.point);
              Point measured = f.keypoints[ob.feature].pt;
              double error =
                  uv == null
                      ? Double.POSITIVE_INFINITY
                      : Math.hypot(uv[0] - measured.x, uv[1] - measured.y);
              if (error > o.trackMaxError) {
                boolean safe =
                    views > Math.max(2, o.trackMinViews)
                        && support[ob.image] > 12
                        && graph.removeObservation(t, ob);
                out.write(
                    pass
                        + ","
                        + t.id
                        + ","
                        + ob.image
                        + ","
                        + error
                        + ","
                        + (safe ? "removed_observation" : "kept_support_or_connectivity")
                        + "\n");
                if (safe) {
                  it.remove();
                  views--;
                  support[ob.image]--;
                  removed++;
                }
              }
            }
            double angle = 0;
            for (var a : t.observations)
              for (var b : t.observations)
                if (a.image < b.image
                    && frames.get(a.image).camera.registered
                    && frames.get(b.image).camera.registered)
                  angle =
                      Math.max(
                          angle,
                          DenseReconstruction.triangulationAngle(
                              t.point,
                              frames.get(a.image).camera.center(),
                              frames.get(b.image).camera.center()));
            if (views < o.trackMinViews || angle < o.trackMinAngle) {
              boolean safe = true;
              for (var ob : t.observations)
                if (frames.get(ob.image).camera.registered && support[ob.image] <= 12) safe = false;
              if (safe && graph.removeTrack(t)) {
                for (var ob : t.observations) support[ob.image]--;
                t.point = null;
                removed++;
                out.write(pass + "," + t.id + ",-1,,removed_weak_geometry\n");
              }
            }
          }
        if (removed == 0) break;
        // Preserve primary bundle history; cleanup has its own before/after snapshot and audit.
        BundleAdjuster.optimize(frames, tracks, o.iterations, o.reprojection, null, anchor, scale);
        if (project != null)
          project.data.snapshot(
              "cleanup/pass_" + pass,
              frames,
              tracks,
              "after sparse cleanup and joint bundle adjustment");
      }
    }
  }

  private static final class Connectivity {
    final List<ImageFrame> frames;
    final List<Map<Integer, Integer>> edges = new ArrayList<>();
    final int components;

    Connectivity(List<ImageFrame> frames, List<FeatureTracks.Track> tracks) {
      this.frames = frames;
      for (int i = 0; i < frames.size(); i++) edges.add(new HashMap<>());
      for (var t : tracks) if (t.point != null) update(t, 1);
      components = count();
    }

    void edge(int a, int b, int delta) {
      if (a == b || !frames.get(a).camera.registered || !frames.get(b).camera.registered) return;
      edges.get(a).merge(b, delta, Integer::sum);
      edges.get(b).merge(a, delta, Integer::sum);
    }

    void update(FeatureTracks.Track t, int delta) {
      for (var a : t.observations)
        for (var b : t.observations) if (a.image < b.image) edge(a.image, b.image, delta);
    }

    boolean removeObservation(FeatureTracks.Track t, FeatureTracks.Observation ob) {
      for (var other : t.observations) if (other.image != ob.image) edge(ob.image, other.image, -1);
      if (count() <= components) return true;
      for (var other : t.observations) if (other.image != ob.image) edge(ob.image, other.image, 1);
      return false;
    }

    boolean removeTrack(FeatureTracks.Track t) {
      update(t, -1);
      if (count() <= components) return true;
      update(t, 1);
      return false;
    }

    int count() {
      Set<Integer> seen = new HashSet<>();
      int components = 0;
      for (ImageFrame f : frames)
        if (f.camera.registered && seen.add(f.id)) {
          components++;
          ArrayDeque<Integer> queue = new ArrayDeque<>();
          queue.add(f.id);
          while (!queue.isEmpty()) {
            int at = queue.remove();
            for (var e : edges.get(at).entrySet())
              if (e.getValue() > 0 && seen.add(e.getKey())) queue.add(e.getKey());
          }
        }
      return components;
    }
  }
}
