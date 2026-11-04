package sgm;

import java.nio.file.*;
import java.util.*;

/** Frozen-frame sampling review; independently centered external reference is not ground truth. */
final class PerformanceSamplingAnalysis {
  public static void main(String[] args) throws Exception {
    Path baseline = Path.of(args[0]),
        changed = Path.of(args[1]),
        reference = Path.of(args[2]),
        out = Path.of(args[3]);
    Files.createDirectories(out);
    Map<Long, PointRecords.Row> indexed = new HashMap<>();
    List<TwoViewExperimentAnalysis.P> original = new ArrayList<>(), selected = new ArrayList<>();
    try (var records =
        new PointRecords.Reader(baseline.resolve("07_results/point-confidence.bin"))) {
      PointRecords.Row row;
      while ((row = records.next()) != null) {
        if (indexed.put(row.sample().id, row) != null)
          throw new AssertionError("Duplicate baseline candidate");
        original.add(plot(row));
      }
    }
    Set<Long> seen = new HashSet<>();
    try (var records =
        new PointRecords.Reader(changed.resolve("07_results/point-confidence.bin"))) {
      PointRecords.Row row;
      while ((row = records.next()) != null) {
        var previous = indexed.get(row.sample().id);
        if (previous == null || !seen.add(row.sample().id))
          throw new AssertionError("Unknown/duplicate winner");
        String[] a = previous.csv().stripTrailing().split(","),
            b = row.csv().stripTrailing().split(",");
        for (int k = 1; k < a.length; k++)
          if (k != 17 && !a[k].equals(b[k]))
            throw new AssertionError("Winner changed field " + k + " candidate " + row.sample().id);
        if (previous.sample().pixel != row.sample().pixel)
          throw new AssertionError("Source pixel changed");
        selected.add(plot(row));
      }
    }
    double[] lo = {Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY},
        hi = {-Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE};
    for (var p : original) {
      double[] v = {p.x(), p.y(), p.z()};
      for (int k = 0; k < 3; k++) {
        lo[k] = Math.min(lo[k], v[k]);
        hi[k] = Math.max(hi[k], v[k]);
      }
    }
    double span = Math.max(hi[0] - lo[0], Math.max(hi[1] - lo[1], hi[2] - lo[2]));
    List<Object> occupancy = new ArrayList<>();
    for (int resolution : new int[] {128, 256, 512}) {
      var a = cells(original, lo, span / resolution);
      var b = cells(selected, lo, span / resolution);
      if (!a.containsAll(b)) throw new AssertionError("Winner occupied a new voxel");
      occupancy.add(
          Json.object(
              "cells_per_longest_bbox_side",
              resolution,
              "cell_edge",
              span / resolution,
              "baseline_occupied",
              a.size(),
              "selected_occupied",
              b.size(),
              "retained_fraction",
              b.size() / (double) a.size()));
    }
    var external = TwoViewExperimentAnalysis.las(reference);
    TwoViewExperimentAnalysis.render(
        List.of(
            new TwoViewExperimentAnalysis.Cloud(
                "Metashape high: independently centered structural reference", external, true),
            new TwoViewExperimentAnalysis.Cloud("Pair emission, 800 pixels", original, false),
            new TwoViewExperimentAnalysis.Cloud("Camera emission, 800 pixels", selected, false)),
        out.resolve("sampling-structure.png"),
        "Blue: 3+ measured views; orange: two measured views. Camera emission changes membership."
            + " Density is not accuracy.");
    Json.write(
        out.resolve("sampling-review.json"),
        Json.object(
            "baseline",
            baseline.toString(),
            "selected",
            changed.toString(),
            "baseline_points",
            original.size(),
            "selected_points",
            selected.size(),
            "baseline_three_plus",
            original.stream().filter(p -> p.views() >= 3).count(),
            "selected_three_plus",
            selected.stream().filter(p -> p.views() >= 3).count(),
            "winner_numeric_fields_and_support_exact_except_output_index_and_merge_count",
            true,
            "occupied_voxels",
            occupancy,
            "reference_plot_samples",
            external.size(),
            "scope",
            "Subset and occupied-voxel retention are sampling diagnostics, not branch completeness"
                + " or surveyed accuracy. Metashape is high resolution and independently centered;"
                + " no registration or accuracy distances."));
    System.out.println(
        "PASS sampling subset/field check: "
            + selected.size()
            + " / "
            + original.size()
            + "; "
            + out);
  }

  private static TwoViewExperimentAnalysis.P plot(PointRecords.Row row) {
    var p = row.sample();
    return new TwoViewExperimentAnalysis.P(
        p.id, p.point[0], p.point[1], p.point[2], p.views, p.rgb, "");
  }

  private static Set<Long> cells(
      List<TwoViewExperimentAnalysis.P> points, double[] origin, double cell) {
    Set<Long> cells = new HashSet<>();
    for (var p : points) {
      long x = (long) Math.floor((p.x() - origin[0]) / cell),
          y = (long) Math.floor((p.y() - origin[1]) / cell),
          z = (long) Math.floor((p.z() - origin[2]) / cell);
      cells.add((x << 42) | (y << 21) | z);
    }
    return cells;
  }
}
