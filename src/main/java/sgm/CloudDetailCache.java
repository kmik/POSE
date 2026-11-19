package sgm;

import java.util.Arrays;

/** Resident overview and one local detail sample. Navigation never needs disk reads to zoom out. */
final class CloudDetailCache {
  CloudData overview, detail;
  private CloudRenderer.View previous;
  boolean zoomingOut;

  void navigated(CloudRenderer.View view) {
    if (previous != null) {
      if (view.distance() > previous.distance() * 1.000001) zoomingOut = true;
      else if (view.distance() < previous.distance() / 1.000001
          || !Arrays.equals(view.target(), previous.target())
          || view.yaw() != previous.yaw() || view.pitch() != previous.pitch()
          || view.roll() != previous.roll() || view.ortho() != previous.ortho()) zoomingOut = false;
    }
    previous = view;
  }

  void accept(CloudData data) {
    if (overview != null && (!Arrays.equals(overview.origin, data.origin)
        || !overview.lasFiles.equals(data.lasFiles)
        || overview.sourceIndex != data.sourceIndex)) clear();
    if (data.detailRegion == null) overview = data;
    else {if(detail!=data)zoomingOut=false;detail = data;}
  }

  CloudData cached(CloudDisplay.Box requested, boolean force) {
    if (overview == null) return null;
    if ((!force && zoomingOut) || overview.n >= overview.total || coversSource(requested)) return overview;
    if (detail != null && CloudViewerControls.encloses(detail.detailRegion, requested)
        && (detail.regionTotal <= detail.n || span(requested) >= span(detail.detailRegion) * .6))
      return detail;
    return null;
  }

  private boolean coversSource(CloudDisplay.Box box) {
    if (overview.sourceIndex == null || overview.sourceIndex.root == null) return false;
    CloudDisplay.Box bounds = overview.sourceIndex.root.bounds.shift(
        -overview.origin[0], -overview.origin[1], -overview.origin[2]);
    return CloudViewerControls.encloses(box, bounds);
  }

  static double span(CloudDisplay.Box b) {
    return Math.max(b.x1() - b.x0(), Math.max(b.y1() - b.y0(), b.z1() - b.z0()));
  }

  static CloudDisplay.Box padded(CloudDisplay.Box b) {
    double x=(b.x1()-b.x0())*.125, y=(b.y1()-b.y0())*.125, z=(b.z1()-b.z0())*.125;
    return new CloudDisplay.Box(b.x0()-x,b.y0()-y,b.z0()-z,b.x1()+x,b.y1()+y,b.z1()+z);
  }

  /** Overview N + current detail N/2 + incoming detail N/2 stay within the two-sample plan. */
  static long detailBudget(long overviewBudget) { return Math.max(1, overviewBudget / 2); }

  void clearDetail() { detail = null; }
  void clear() { overview=null;detail=null;previous=null;zoomingOut=false; }
}
