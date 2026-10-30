package sgm;

import java.io.*;
import java.util.*;

/** Frozen earlier acceptance rule, retained only for regression ablations of layered surfaces. */
final class LegacyLayerFusion {
  private final Camera[] cameras;
  private final int[] widths, heights;
  private final Options options;
  private final List<PairDepth> pairs = new ArrayList<>();

  LegacyLayerFusion(Camera[] cameras, int[] widths, int[] heights, Options options) {
    this.cameras = cameras;
    this.widths = widths;
    this.heights = heights;
    this.options = options;
  }

  int addPair(int a, int b) {
    int id = pairs.size();
    pairs.add(
        new PairDepth(a, b, new float[widths[a] * heights[a]], new float[widths[b] * heights[b]]));
    return id;
  }

  void add(int pair, double[] world) {
    PairDepth p = pairs.get(pair);
    put(p.a, p.first, world);
    put(p.b, p.second, world);
  }

  int requiredViews() {
    return Math.min(options.minViews, cameras.length);
  }

  boolean consistent(int source, double[] world) {
    return supportingViews(source, world) >= requiredViews();
  }

  void export(ProjectArtifacts artifacts) throws IOException {
    for (int i = 0; i < pairs.size(); i++) {
      PairDepth p = pairs.get(i);
      artifacts.pairEvidence(
          i, p.a, p.first, widths[p.a], heights[p.a], p.b, p.second, widths[p.b], heights[p.b]);
    }
  }

  int supportingViews(int source, double[] world) {
    PairDepth origin = pairs.get(source);
    BitSet views = new BitSet(cameras.length);
    views.set(origin.a);
    views.set(origin.b);
    for (int i = 0; i < pairs.size(); i++) {
      if (i == source) continue;
      PairDepth independent = pairs.get(i);
      if (agrees(independent.a, independent.first, origin, world)
          || agrees(independent.b, independent.second, origin, world)) {
        views.set(independent.a);
        views.set(independent.b);
      }
    }
    return views.cardinality();
  }

  private void put(int id, float[] map, double[] world) {
    Camera camera = cameras[id];
    double[] v = camera.view(world);
    if (v[2] <= 0) return;
    int x = (int) Math.round(camera.fx * v[0] / v[2] + camera.cx),
        y = (int) Math.round(camera.fy * v[1] / v[2] + camera.cy);
    if (x < 0 || x >= widths[id] || y < 0 || y >= heights[id]) return;
    int at = y * widths[id] + x;
    float z = (float) v[2];
    if (map[at] == 0 || z < map[at]) map[at] = z;
  }

  private boolean agrees(int image, float[] map, PairDepth origin, double[] world) {
    Camera camera = cameras[image];
    double[] v = camera.view(world);
    if (v[2] <= 0) return false;
    double px = camera.fx * v[0] / v[2] + camera.cx, py = camera.fy * v[1] / v[2] + camera.cy;
    if (px < 0 || px > widths[image] - 1 || py < 0 || py > heights[image] - 1) return false;
    int x = (int) Math.round(px), y = (int) Math.round(py), bestX = 0, bestY = 0;
    double best = Double.POSITIVE_INFINITY;
    float depth = 0;
    // Pick the nearest measured pixel, not whichever neighbor happens to match the candidate.
    for (int dy = -1; dy <= 1; dy++)
      for (int dx = -1; dx <= 1; dx++) {
        int xx = x + dx, yy = y + dy;
        if (xx < 0 || xx >= widths[image] || yy < 0 || yy >= heights[image]) continue;
        float z = map[yy * widths[image] + xx];
        double distance = (xx - px) * (xx - px) + (yy - py) * (yy - py);
        if (z > 0 && distance < best) {
          best = distance;
          depth = z;
          bestX = xx;
          bestY = yy;
        }
      }
    if (depth == 0
        || best > 2.0
        || Math.abs(depth - v[2]) > options.fusionRelative * Math.min(depth, v[2])) return false;
    double[] local = {
      (bestX - camera.cx) / camera.fx * depth, (bestY - camera.cy) / camera.fy * depth, depth
    };
    double[] independentWorld =
        Math3.mv(Math3.transpose(camera.rotation), Math3.sub(local, camera.translation));
    return reprojectionAgrees(cameras[origin.a], world, independentWorld)
        && reprojectionAgrees(cameras[origin.b], world, independentWorld);
  }

  private boolean reprojectionAgrees(Camera camera, double[] candidate, double[] independent) {
    double[] a = camera.project(candidate), b = camera.project(independent);
    return a != null && b != null && Math.hypot(a[0] - b[0], a[1] - b[1]) <= options.fusionPixels;
  }

  private static final class PairDepth {
    final int a, b;
    final float[] first, second;

    PairDepth(int a, int b, float[] first, float[] second) {
      this.a = a;
      this.b = b;
      this.first = first;
      this.second = second;
    }
  }
}
