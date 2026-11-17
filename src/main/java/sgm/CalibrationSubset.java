package sgm;

import java.util.*;

/**
 * A compact, connected set of registered cameras for the lens trials of a large block
 * (--calibration-trial-method subset|joint). The data-only observability assessment
 * (CalibrationObservability) works on a dense camera information matrix and refuses more than
 * {@link #OBSERVABILITY_UNKNOWNS} unknowns, so on a large block no trial can ever be accepted; on
 * this subset it can. The subset is the cameras nearest the block centre, per camera group in
 * proportion to the group's size (at least 10 of each): neighbouring images overlap, so the subset
 * is one connected sub-block, and a lens is determined by its images' overlap, not by the extent
 * of the block. While entered, the other cameras are marked unregistered, so every trial, split
 * and assessment sees the subset only; exit restores the flags.
 */
final class CalibrationSubset {
  /** Largest unknown count of BundleAdjuster.calibrationDataInformation (poses + lens blocks). */
  static final int OBSERVABILITY_UNKNOWNS = Integer.getInteger("sgm.observability.unknowns", 768); // property: tests only
  /** Default subset size (cameras). */
  static final int DEFAULT_CAMERAS = 100;

  final Set<Integer> members;
  /** Gauge of the subset trials: the member nearest the centre and the member farthest from it. */
  final int anchor, scale;
  private boolean[] saved;

  private CalibrationSubset(Set<Integer> members, int anchor, int scale) {
    this.members = members;
    this.anchor = anchor;
    this.scale = scale;
  }

  /** Most free cameras an assessment of this many camera groups accepts. */
  static int limit(int groups) {
    return (OBSERVABILITY_UNKNOWNS - SharedCalibration.SIZE * groups) / 6;
  }

  static int registered(List<ImageFrame> frames) {
    int n = 0;
    for (ImageFrame f : frames) if (f.camera.registered) n++;
    return n;
  }

  /**
   * True when every observability assessment of the whole block must fail: at least all
   * registered cameras but the anchor are free in a trial (poses fixed only by a known fixed
   * orientation), and their unknowns with the lens blocks exceed the assessment's limit. A
   * parameter is then never accepted, whatever the trials find.
   */
  static boolean overLimit(List<ImageFrame> frames, int groups, Options o) {
    if (o.poses != null && o.extrinsicsMode.equals("fixed")) return false;
    return (registered(frames) - 1) * 6L + (long) SharedCalibration.SIZE * groups > OBSERVABILITY_UNKNOWNS;
  }

  /** Subset size for a request (0 = default), bounded by the assessment limit. */
  static int size(int requested, int groups) {
    return Math.min(requested > 0 ? requested : DEFAULT_CAMERAS, limit(groups));
  }

  /** The subset, or null when the registered cameras already fit. */
  static CalibrationSubset choose(List<ImageFrame> frames, int requested, int groups) {
    int size = size(requested, groups);
    List<ImageFrame> registered = new ArrayList<>();
    for (ImageFrame f : frames) if (f.camera.registered) registered.add(f);
    if (registered.size() <= size || size < 2) return null;
    // Block centre: the per-axis median of the camera centres (robust to outlying cameras).
    double[] centre = new double[3];
    for (int k = 0; k < 3; k++) {
      final int axis = k;
      double[] v = registered.stream().mapToDouble(f -> f.camera.center()[axis]).sorted().toArray();
      centre[k] = v[v.length / 2];
    }
    Map<Integer, List<ImageFrame>> byGroup = new TreeMap<>();
    for (ImageFrame f : registered) byGroup.computeIfAbsent(f.group, g -> new ArrayList<>()).add(f);
    // Quotas in proportion to the groups' sizes, at least 10 (or all) of each, total <= size.
    Map<Integer, Integer> quota = new TreeMap<>();
    for (var e : byGroup.entrySet()) {
      int n = e.getValue().size();
      quota.put(e.getKey(), Math.min(n, Math.max(Math.min(10, n), (int) Math.round((double) size * n / registered.size()))));
    }
    while (quota.values().stream().mapToInt(Integer::intValue).sum() > size) {
      int largest = quota.entrySet().stream().max(Map.Entry.comparingByValue()).get().getKey();
      if (quota.get(largest) <= 1) break;
      quota.merge(largest, -1, Integer::sum);
    }
    Set<Integer> members = new TreeSet<>();
    for (var e : byGroup.entrySet()) {
      List<ImageFrame> list = new ArrayList<>(e.getValue());
      list.sort(Comparator.comparingDouble((ImageFrame f) -> distance(f, centre)).thenComparingInt(f -> f.id));
      for (int i = 0; i < quota.get(e.getKey()); i++) members.add(list.get(i).id);
    }
    int anchor = -1, scale = -1;
    double nearest = Double.POSITIVE_INFINITY, farthest = -1;
    for (ImageFrame f : registered)
      if (members.contains(f.id) && distance(f, centre) < nearest) {
        nearest = distance(f, centre);
        anchor = f.id;
      }
    double[] origin = frames.get(anchor).camera.center();
    for (ImageFrame f : registered)
      if (members.contains(f.id) && f.id != anchor) {
        double d = Math3.norm(Math3.sub(f.camera.center(), origin));
        if (d > farthest) {
          farthest = d;
          scale = f.id;
        }
      }
    if (scale < 0 || !(farthest > 1e-9)) return null;
    return new CalibrationSubset(members, anchor, scale);
  }

  private static double distance(ImageFrame f, double[] centre) {
    return Math3.norm(Math3.sub(f.camera.center(), centre));
  }

  /** Marks the cameras outside the subset unregistered (exit restores them). */
  void enter(List<ImageFrame> frames) {
    if (saved != null) return;
    saved = new boolean[frames.size()];
    for (ImageFrame f : frames) {
      saved[f.id] = f.camera.registered;
      if (!members.contains(f.id)) f.camera.registered = false;
    }
  }

  void exit(List<ImageFrame> frames) {
    if (saved == null) return;
    for (ImageFrame f : frames) f.camera.registered = saved[f.id];
    saved = null;
  }

  Object json(List<ImageFrame> frames) {
    Map<Integer, Integer> perGroup = new TreeMap<>();
    for (int id : members) perGroup.merge(frames.get(id).group, 1, Integer::sum);
    return Json.object(
        "cameras", members.size(),
        "cameras_per_group", new ArrayList<>(perGroup.values()),
        "selection", "cameras nearest the block centre (per-axis median of camera centres), per camera"
            + " group in proportion to its size, at least 10 of each",
        "anchor_image", frames.get(anchor).path.getFileName().toString(),
        "scale_image", frames.get(scale).path.getFileName().toString());
  }
}
