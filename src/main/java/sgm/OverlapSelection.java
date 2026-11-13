package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;

/** Symmetric common-footprint overlap on an explicit or well-constrained scene plane. */
final class OverlapSelection {
  record Plane(
      double[] origin,
      double[] normal,
      double[] u,
      double[] v,
      double heading,
      String source,
      double scatter) {}

  record Estimate(double forward, double side, double area, String confidence, String reason) {}

  static boolean enabled(Options o) {
    return o.minForwardOverlap > 0 || o.minSideOverlap > 0 || o.minPairOverlap > 0;
  }

  static List<int[]> filter(
      List<ImageFrame> frames,
      List<int[]> pairs,
      List<FeatureTracks.Track> tracks,
      Options o,
      String stage)
      throws IOException {
    if (!enabled(o)) return pairs;
    Camera[] c = new Camera[frames.size()];
    int[] w = new int[c.length], h = new int[c.length];
    for (ImageFrame f : frames) {
      c[f.id] = f.camera;
      w[f.id] = f.width;
      h[f.id] = f.height;
    }
    List<double[]> points = new ArrayList<>();
    for (var t : tracks) if (t.point != null) points.add(t.point);
    Plane plane = plane(c, points, o);
    List<int[]> selected = new ArrayList<>();
    List<Object> report = new ArrayList<>();
    for (int[] pair : pairs) {
      Estimate e =
          estimate(c[pair[0]], w[pair[0]], h[pair[0]], c[pair[1]], w[pair[1]], h[pair[1]], plane);
      String reason = decision(e, o);
      boolean accept = accepted(e, o);
      if (accept) selected.add(pair);
      report.add(
          Json.object(
              "a",
              pair[0],
              "b",
              pair[1],
              "forward_percent",
              e.forward,
              "side_percent",
              e.side,
              "shared_area_percent",
              e.area,
              "confidence",
              e.confidence,
              "estimate_reason",
              e.reason,
              "accepted",
              accept,
              "decision",
              reason));
    }
    Json.write(
        o.output.resolve("03_matches/overlap_" + stage + ".json"),
        Json.object(
            "stage",
            stage,
            "thresholds_percent",
            new double[] {o.minForwardOverlap, o.minSideOverlap, o.minPairOverlap},
            "denominator",
            "smaller footprint area/axis extent; actual intersection polygon",
            "unknown_policy",
            o.overlapUnknown,
            "plane",
            plane == null
                ? null
                : Json.object(
                    "origin",
                    plane.origin,
                    "normal",
                    plane.normal,
                    "u",
                    plane.u,
                    "v",
                    plane.v,
                    "heading_radians_in_plane",
                    plane.heading,
                    "source",
                    plane.source,
                    "relative_scatter",
                    plane.scatter),
            "pairs",
            report));
    System.out.printf(
        Locale.ROOT,
        "Overlap %s: %d/%d pairs retained; geometric estimates and unknowns:"
            + " 03_matches/overlap_%s.json%n",
        stage,
        selected.size(),
        pairs.size(),
        stage);
    return selected;
  }

  static boolean accepted(Estimate e, Options o) {
    String reason = decision(e, o);
    return reason.equals("accepted") || reason.equals("unknown_keep");
  }

  static String decision(Estimate e, Options o) {
    boolean unknown =
        !Double.isFinite(e.area)
          || (o.minForwardOverlap > 0 || o.minSideOverlap > 0) && !Double.isFinite(e.forward);
    String reason =
        unknown
          ? "unknown_" + o.overlapUnknown
          : e.area + 1e-8 < o.minPairOverlap
            ? "shared_area_below_minimum"
            : e.forward + 1e-8 < o.minForwardOverlap
                ? "forward_overlap_below_minimum"
                : e.side + 1e-8 < o.minSideOverlap
                  ? "side_overlap_below_minimum"
                  : "accepted";
    return reason;
  }

  static Plane plane(Camera[] cameras, List<double[]> points, Options o) {
    List<Camera> registered = new ArrayList<>();
    for (Camera c : cameras) if (c.registered) registered.add(c);
    if (registered.size() < 2) return null;
    double[] normal = {0, 0, 0}, origin = {0, 0, 0};
    String source;
    double scatter = 0;
    if (Double.isFinite(o.overlapGroundZ)) {
      normal = new double[] {0, 0, 1};
      for (Camera c : registered) {
        double[] center = c.center();
        origin[0] += center[0] / registered.size();
        origin[1] += center[1] / registered.size();
      }
      origin[2] = o.overlapGroundZ;
      source = "explicit_world_z_plane";
    } else {
      if (points.size() < 8) return null;
      for (Camera c : registered) for (int k = 0; k < 3; k++) normal[k] += c.rotation[6 + k];
      double norm = Math3.norm(normal);
      if (norm < registered.size() * .5) return null;
      normal = Math3.times(normal, 1 / norm);
      for (double[] p : points) for (int k = 0; k < 3; k++) origin[k] += p[k] / points.size();
      double[] distances = new double[points.size()];
      for (int i = 0; i < distances.length; i++)
        distances[i] = dot(Math3.sub(points.get(i), origin), normal);
      Arrays.sort(distances);
      double median = distances[distances.length / 2];
      origin = Math3.add(origin, Math3.times(normal, median));
      double span = 0;
      for (double[] p : points) span = Math.max(span, Math3.norm(Math3.sub(p, origin)));
      double[] deviations = new double[distances.length];
      for (int i = 0; i < distances.length; i++) deviations[i] = Math.abs(distances[i] - median);
      Arrays.sort(deviations);
      scatter = deviations[deviations.length * 9 / 10] / Math.max(1e-9, span);
      if (scatter > .15) return null;
      source = "sparse_scene_median_plane_approximation";
    }
    double[] axis = Math.abs(normal[0]) < .9 ? new double[] {1, 0, 0} : new double[] {0, 1, 0};
    double[] u = Math3.sub(axis, Math3.times(normal, dot(axis, normal)));
    u = Math3.times(u, 1 / Math3.norm(u));
    double[] v = cross(normal, u);
    double heading = Double.NaN;
    if (Double.isFinite(o.flightHeading)) {
      double angle = Math.toRadians(o.flightHeading);
      double[] direction = {Math.sin(angle), Math.cos(angle), 0};
      double x = dot(direction, u), y = dot(direction, v);
      if (Math.hypot(x, y) > 1e-8) heading = Math.atan2(y, x);
    } else {
      double sx = 0, sy = 0;
      int count = 0;
      for (int i = 1; i < registered.size(); i++) {
        double[] delta = Math3.sub(registered.get(i).center(), registered.get(i - 1).center());
        double x = dot(delta, u), y = dot(delta, v);
        if (Math.hypot(x, y) < 1e-8) continue;
        double angle = Math.atan2(y, x);
        sx += Math.cos(2 * angle);
        sy += Math.sin(2 * angle);
        count++;
      }
      if (count >= 1 && Math.hypot(sx, sy) / count >= .6) heading = .5 * Math.atan2(sy, sx);
    }
    return new Plane(origin, normal, u, v, heading, source, scatter);
  }

  static Estimate estimate(Camera a, int wa, int ha, Camera b, int wb, int hb, Plane plane) {
    if (plane == null || !a.registered || !b.registered)
      return new Estimate(
          Double.NaN, Double.NaN, Double.NaN, "unknown", "missing_registration_or_scene_plane");
    List<double[]> pa = footprint(a, wa, ha, plane), pb = footprint(b, wb, hb, plane);
    if (pa == null || pb == null)
      return new Estimate(
          Double.NaN,
          Double.NaN,
          Double.NaN,
          "unknown",
          "footprint_crosses_horizon_or_behind_camera");
    List<double[]> intersection = intersection(pa, pb);
    double area = 100 * area(intersection) / Math.max(1e-20, Math.min(area(pa), area(pb)));
    double forward = Double.NaN, side = Double.NaN;
    if (Double.isFinite(plane.heading)) {
      forward = axisOverlap(pa, pb, intersection, plane.heading);
      side = axisOverlap(pa, pb, intersection, plane.heading + Math.PI / 2);
    }
    return new Estimate(
        forward,
        side,
        Math.max(0, Math.min(100, area)),
        plane.source.startsWith("explicit") ? "explicit_plane" : "estimated_plane",
        Double.isFinite(forward)
            ? "estimated_footprint"
            : "flight_direction_unknown_shared_area_available");
  }

  private static List<double[]> footprint(Camera c, int w, int h, Plane p) {
    double[] center = c.center(), rt = Math3.transpose(c.rotation);
    List<double[]> out = new ArrayList<>();
    for (double[] uv :
        new double[][] {{-.5, -.5}, {w - .5, -.5}, {w - .5, h - .5}, {-.5, h - .5}}) {
      double[] ray = Math3.mv(rt, new double[] {(uv[0] - c.cx) / c.fx, (uv[1] - c.cy) / c.fy, 1});
      double denominator = dot(ray, p.normal);
      if (Math.abs(denominator) < 1e-8) return null;
      double distance = dot(Math3.sub(p.origin, center), p.normal) / denominator;
      if (!(distance > 0)) return null;
      double[] q = Math3.sub(Math3.add(center, Math3.times(ray, distance)), p.origin);
      out.add(new double[] {dot(q, p.u), dot(q, p.v)});
    }
    if (signedArea(out) < 0) Collections.reverse(out);
    return out;
  }

  private static double axisOverlap(
      List<double[]> a, List<double[]> b, List<double[]> common, double angle) {
    if (common.isEmpty()) return 0;
    double[] ea = extent(a, angle), eb = extent(b, angle), ec = extent(common, angle);
    return Math.min(
        100, 100 * (ec[1] - ec[0]) / Math.max(1e-20, Math.min(ea[1] - ea[0], eb[1] - eb[0])));
  }

  private static double[] extent(List<double[]> p, double angle) {
    double lo = Double.POSITIVE_INFINITY, hi = -lo;
    for (double[] q : p) {
      double t = q[0] * Math.cos(angle) + q[1] * Math.sin(angle);
      lo = Math.min(lo, t);
      hi = Math.max(hi, t);
    }
    return new double[] {lo, hi};
  }

  private static List<double[]> intersection(List<double[]> subject, List<double[]> clip) {
    List<double[]> out = new ArrayList<>(subject);
    for (int i = 0; i < clip.size() && !out.isEmpty(); i++) {
      double[] a = clip.get(i), b = clip.get((i + 1) % clip.size());
      List<double[]> input = out;
      out = new ArrayList<>();
      double[] previous = input.getLast();
      double pd = side(a, b, previous);
      for (double[] current : input) {
        double cd = side(a, b, current);
        if ((pd >= -1e-9) != (cd >= -1e-9)) {
          double t = pd / (pd - cd);
          out.add(
              new double[] {
                previous[0] + t * (current[0] - previous[0]),
                previous[1] + t * (current[1] - previous[1])
              });
        }
        if (cd >= -1e-9) out.add(current);
        previous = current;
        pd = cd;
      }
    }
    return out;
  }

  private static double side(double[] a, double[] b, double[] p) {
    return (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]);
  }

  private static double signedArea(List<double[]> p) {
    double area = 0;
    for (int i = 0; i < p.size(); i++) {
      double[] a = p.get(i), b = p.get((i + 1) % p.size());
      area += a[0] * b[1] - b[0] * a[1];
    }
    return area * .5;
  }

  private static double area(List<double[]> p) {
    return Math.abs(signedArea(p));
  }

  private static double dot(double[] a, double[] b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  }

  private static double[] cross(double[] a, double[] b) {
    return new double[] {
      a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]
    };
  }
}
