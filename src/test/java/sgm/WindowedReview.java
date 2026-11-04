package sgm;

import java.nio.file.*;
import java.util.*;

/** Structural review of reduced fixed-camera experiments, not a surveyed accuracy assessment. */
public final class WindowedReview {
  static List<PointRecords.Row> read(Path run) throws Exception {
    List<PointRecords.Row> out = new ArrayList<>();
    try (var in = new PointRecords.Reader(run.resolve("07_results/point-confidence.bin"))) {
      PointRecords.Row r;
      while ((r = in.next()) != null) out.add(r);
    }
    return out;
  }

  static long key(PointRecords.Row r) {
    return ((long) r.sample().source << 32) | (r.sample().pixel & 0xffffffffL);
  }

  static TwoViewExperimentAnalysis.P plot(PointRecords.Row r) {
    var p = r.sample();
    return new TwoViewExperimentAnalysis.P(
        p.id, p.point[0], p.point[1], p.point[2], p.views, p.rgb, "");
  }

  static Set<String> cells(List<PointRecords.Row> rows, double[] lo, double edge) {
    Set<String> set = new HashSet<>();
    for (var r : rows) {
      var p = r.sample().point;
      set.add(
          (long) Math.floor((p[0] - lo[0]) / edge)
              + ":"
              + (long) Math.floor((p[1] - lo[1]) / edge)
              + ":"
              + (long) Math.floor((p[2] - lo[2]) / edge));
    }
    return set;
  }

  public static void main(String[] args) throws Exception {
    var baseline = read(Path.of(args[0]));
    var changed = read(Path.of(args[1]));
    Path out = Files.createDirectories(Path.of(args[3]));
    Map<Long, PointRecords.Row> indexed = new HashMap<>();
    for (var r : baseline) indexed.put(key(r), r);
    long common = 0, added = 0;
    double maxDistance = 0;
    long changedFields = 0;
    for (var r : changed) {
      var previous = indexed.get(key(r));
      if (previous == null) {
        added++;
        continue;
      }
      common++;
      maxDistance =
          Math.max(maxDistance, Math3.norm(Math3.sub(previous.sample().point, r.sample().point)));
      String[] a = previous.csv().split(","), b = r.csv().split(",");
      for (int k = 2; k < a.length; k++) if (!a[k].equals(b[k])) changedFields++;
    }
    double[] lo = {Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY},
        hi = {-Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE};
    for (var r : baseline)
      for (int k = 0; k < 3; k++) {
        lo[k] = Math.min(lo[k], r.sample().point[k]);
        hi[k] = Math.max(hi[k], r.sample().point[k]);
      }
    double span = Math.max(hi[0] - lo[0], Math.max(hi[1] - lo[1], hi[2] - lo[2]));
    List<Object> occupancy = new ArrayList<>();
    for (int resolution : new int[] {128, 256, 512}) {
      double edge = span / resolution;
      var a = cells(baseline, lo, edge);
      var b = cells(changed, lo, edge);
      Set<String> intersection = new HashSet<>(a);
      intersection.retainAll(b);
      occupancy.add(
          Json.object(
              "edge",
              edge,
              "baseline_cells",
              a.size(),
              "incremental_cells",
              b.size(),
              "retained_fraction",
              intersection.size() / (double) a.size()));
    }
    var external = TwoViewExperimentAnalysis.las(Path.of(args[2]));
    TwoViewExperimentAnalysis.render(
        List.of(
            new TwoViewExperimentAnalysis.Cloud(
                "Metashape high: independently centered reference", external, true),
            new TwoViewExperimentAnalysis.Cloud(
                "Reference pair candidates, 400 pixels / step 2",
                baseline.stream().map(WindowedReview::plot).toList(),
                false),
            new TwoViewExperimentAnalysis.Cloud(
                "Incremental representatives + robust cameras, matched settings",
                changed.stream().map(WindowedReview::plot).toList(),
                false)),
        out.resolve("structure.png"),
        "Blue: 3+ measured views; orange: two views. Reduced settings; density and voxel retention"
            + " do not establish branch accuracy.",
        "Experimental 400-pixel reconstruction / step 2; reference high. Centering only, no"
            + " registration.");
    Json.write(
        out.resolve("review.json"),
        Json.object(
            "baseline_points",
            baseline.size(),
            "incremental_points",
            changed.size(),
            "common_source_pixels",
            common,
            "new_source_pixels",
            added,
            "changed_common_fields_excluding_candidate_order",
            changedFields,
            "maximum_common_xyz_distance",
            maxDistance,
            "baseline_three_plus",
            baseline.stream().filter(p -> p.sample().views >= 3).count(),
            "incremental_three_plus",
            changed.stream().filter(p -> p.sample().views >= 3).count(),
            "occupied_voxels",
            occupancy,
            "scope",
            "Fixed-camera structural diagnostics only; external high-resolution cloud independently"
                + " centered. No registration, surveyed distances or claim of certified thin-branch"
                + " completeness."));
    System.out.println(
        "Structural review: "
            + baseline.size()
            + " -> "
            + changed.size()
            + "; added "
            + added
            + "; changed common fields "
            + changedFields);
  }
}
