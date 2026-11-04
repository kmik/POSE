package sgm;

import java.util.*;

/** Conservative per-producing-pair bounds; a shortlist never asserts support or occlusion. */
final class FusionScope {
  private final double[][] input;
  private final boolean[][] cameras;
  FusionScope(Camera[] camera, int[] widths, int[] heights, int[][] pairs,
      List<double[]> bounds, double relative) {
    input = new double[pairs.length][];
    cameras = new boolean[pairs.length][camera.length];
    double scale = (1 + relative) * (1 + relative);
    for (int p = 0; p < pairs.length; p++) {
      double[] box = bounds.get(p).clone(); input[p] = box;
      if (!finite(box) || !(relative >= 0 && relative < .5)) { Arrays.fill(cameras[p], true); continue; }
      double[] center = camera[pairs[p][0]].center();
      double[] expanded = {Double.POSITIVE_INFINITY,Double.POSITIVE_INFINITY,Double.POSITIVE_INFINITY,
          Double.NEGATIVE_INFINITY,Double.NEGATIVE_INFINITY,Double.NEGATIVE_INFINITY};
      for (int corner = 0; corner < 8; corner++) for (double factor : new double[] {1 / scale, scale}) {
        for (int axis = 0; axis < 3; axis++) {
          double v = center[axis] + factor * (box[axis + (((corner >> axis) & 1) * 3)] - center[axis]);
          double error = 1e-6 + 1e-8 * Math.max(Math.abs(v), Math.abs(center[axis]));
          expanded[axis] = Math.min(expanded[axis], v - error);
          expanded[axis + 3] = Math.max(expanded[axis + 3], v + error);
        }
      }
      for (int image = 0; image < camera.length; image++)
        cameras[p][image] = intersects(camera[image], widths[image], heights[image], expanded);
      cameras[p][pairs[p][0]] = cameras[p][pairs[p][1]] = true;
    }
  }
  static boolean finite(double[] box) {
    for (double v : box) if (!Double.isFinite(v)) return false;
    return true;
  }
  static boolean intersects(Camera c, int w, int h, double[] box) {
    if (!finite(box)) return true;
    double minX=Double.POSITIVE_INFINITY,minY=minX,maxX=-minX,maxY=-minX;
    boolean allBehind = true;
    for (int corner = 0; corner < 8; corner++) {
      double[] p = new double[3];
      for (int axis = 0; axis < 3; axis++) p[axis] = box[axis + (((corner >> axis) & 1) * 3)];
      double[] v = c.view(p);
      if (!Double.isFinite(v[0]+v[1]+v[2])) return true;
      if (v[2] > -1e-6) allBehind = false;
      // Boxes touching/crossing the near plane are never culled.
      if (v[2] <= 1e-6 && !allBehind) return true;
      if (v[2] <= 1e-6) continue;
      double[] uv = c.projectView(v);
      if (uv == null || !Double.isFinite(uv[0]+uv[1])) return true;
      minX=Math.min(minX,uv[0]);maxX=Math.max(maxX,uv[0]);
      minY=Math.min(minY,uv[1]);maxY=Math.max(maxY,uv[1]);
    }
    if (allBehind) return false;
    // Check the minimum Z separately: iteration order must not affect plane-crossing handling.
    for (int corner=0;corner<8;corner++) {
      double[] p=new double[3];for(int a=0;a<3;a++)p[a]=box[a+(((corner>>a)&1)*3)];
      if(c.view(p)[2]<=1e-6)return true;
    }
    return !(maxX < -1 || maxY < -1 || minX > w || minY > h);
  }
  boolean contains(int source, double[] point) {
    double[] b = input[source];
    if (!finite(b)) return true;
    for (int a=0;a<3;a++) {
      double e=1e-6+1e-8*Math.max(Math.abs(b[a]),Math.abs(b[a+3]));
      if (!Double.isFinite(point[a]) || point[a]<b[a]-e || point[a]>b[a+3]+e) return false;
    }
    return true;
  }
  boolean keeps(int source,int image) { return cameras[source][image]; }
  int maxEndpoints(List<DepthFusion.Endpoint> endpoints) {
    int maximum=0;for(int p=0;p<cameras.length;p++){int n=0;for(var e:endpoints)if(keeps(p,e.image()))n++;maximum=Math.max(maximum,n);}return maximum;
  }
}
