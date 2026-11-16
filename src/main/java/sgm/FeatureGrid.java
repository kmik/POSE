package sgm;

import java.util.Arrays;
import org.opencv.core.KeyPoint;

/**
 * Spatially spread keypoint selection (--feature-grid N): the image is divided into N cells along
 * its longer side (square-ish cells), each cell first receives up to maximum / cells of its
 * strongest keypoints, and the remaining slots go to the strongest keypoints not yet chosen. A
 * global strongest-N choice concentrates on high-contrast areas; this keeps weak-texture areas
 * represented without changing the number of features.
 */
final class FeatureGrid {
  private FeatureGrid() {}

  /** Grid cells along the image's longer side and shorter side. */
  static int[] cells(int width, int height, int grid) {
    int longer = Math.max(width, height), shorter = Math.min(width, height);
    int across = Math.max(1, (int) Math.round(grid * (double) shorter / Math.max(1, longer)));
    return width >= height ? new int[] {grid, across} : new int[] {across, grid};
  }

  /**
   * Chosen keypoints, strongest first (ties keep detection order). Returns all keypoints, in
   * that order, when there are no more than maximum.
   */
  static KeyPoint[] select(KeyPoint[] all, int width, int height, int grid, int maximum) {
    Integer[] order = new Integer[all.length];
    for (int i = 0; i < order.length; i++) order[i] = i;
    Arrays.sort(order, (a, b) -> Float.compare(all[b].response, all[a].response));
    int count = Math.min(maximum, all.length);
    KeyPoint[] out = new KeyPoint[count];
    if (all.length <= maximum) {
      for (int i = 0; i < count; i++) out[i] = all[order[i]];
      return out;
    }
    int[] size = cells(width, height, grid);
    int columns = size[0], rows = size[1];
    int quota = Math.max(1, maximum / (columns * rows));
    int[] used = new int[columns * rows];
    boolean[] taken = new boolean[all.length];
    int n = 0;
    for (int k = 0; k < order.length && n < count; k++) {
      KeyPoint p = all[order[k]];
      int cx = Math.min(columns - 1, Math.max(0, (int) (p.pt.x * columns / width)));
      int cy = Math.min(rows - 1, Math.max(0, (int) (p.pt.y * rows / height)));
      int cell = cy * columns + cx;
      if (used[cell] < quota) {
        used[cell]++;
        taken[order[k]] = true;
        n++;
      }
    }
    for (int k = 0; k < order.length && n < count; k++)
      if (!taken[order[k]]) {
        taken[order[k]] = true;
        n++;
      }
    int j = 0;
    for (int k = 0; k < order.length; k++) if (taken[order[k]]) out[j++] = all[order[k]];
    return out;
  }
}
