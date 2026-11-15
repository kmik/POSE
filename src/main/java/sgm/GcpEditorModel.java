package sgm;

import java.util.*;

/**
 * Typed view of {@link GcpEditor.Editor#state()} for the native editor window: GCP rows, the
 * marker cards of a GCP in display order (pinned, refined, predicted, rejected; probably hidden
 * ones last), what "accept all" would pin and the status texts. No window here.
 */
final class GcpEditorModel {
  record Image(int id, String name, int width, int height, boolean registered) {}

  record Gcp(String id, String type, int pins, int markers, int refined, Double pinRms, Double pinMax) {}

  record Marker(
      String gcp, int image, double u, double v, String status, Double predictedU, Double predictedV, Double residual, Double offset, boolean hidden, Double score) {
    boolean placed() {
      return Double.isFinite(u) && Double.isFinite(v);
    }

    boolean pinned() {
      return status.equals("pinned");
    }
  }

  final List<Image> images = new ArrayList<>();
  final List<Gcp> gcps = new ArrayList<>();
  final List<Marker> markers = new ArrayList<>();
  final boolean ready, refining, georeferenced;
  final String needed;
  final double accept;

  GcpEditorModel(Map<String, Object> state) {
    for (Object o : list(state.get("images"))) {
      Map<String, Object> m = map(o);
      images.add(new Image(integer(m.get("id")), String.valueOf(m.get("name")), integer(m.get("width")), integer(m.get("height")), Boolean.TRUE.equals(m.get("registered"))));
    }
    for (Object o : list(state.get("markers"))) {
      Map<String, Object> m = map(o);
      markers.add(
          new Marker(
              String.valueOf(m.get("gcp")), integer(m.get("image")), real(m.get("u")), real(m.get("v")), String.valueOf(m.get("status")),
              boxed(m.get("predicted_u")), boxed(m.get("predicted_v")), boxed(m.get("residual")), boxed(m.get("offset")),
              Boolean.TRUE.equals(m.get("hidden")), boxed(m.get("score"))));
    }
    for (Object o : list(state.get("gcps"))) {
      Map<String, Object> m = map(o);
      String id = String.valueOf(m.get("id"));
      int all = 0, refined = 0;
      for (Marker k : markers)
        if (k.gcp.equals(id)) {
          all++;
          if (k.status.equals("refined")) refined++;
        }
      gcps.add(new Gcp(id, String.valueOf(m.get("type")), integer(m.get("pins")), all, refined, boxed(m.get("pin_rms")), boxed(m.get("pin_max"))));
    }
    ready = Boolean.TRUE.equals(state.get("ready"));
    refining = Boolean.TRUE.equals(state.get("refining"));
    georeferenced = Boolean.TRUE.equals(state.get("georeferenced"));
    needed = state.get("needed") == null ? null : String.valueOf(state.get("needed"));
    accept = state.get("accept") instanceof Number n ? n.doubleValue() : GcpEditor.Editor.ACCEPT;
  }

  Gcp gcp(String id) {
    for (Gcp g : gcps) if (g.id.equals(id)) return g;
    return null;
  }

  Marker marker(String gcp, int image) {
    for (Marker k : markers) if (k.gcp.equals(gcp) && k.image == image) return k;
    return null;
  }

  private static int order(String status) {
    return switch (status) {
      case "pinned" -> 0;
      case "refined" -> 1;
      case "predicted" -> 2;
      default -> 3;
    };
  }

  /** Placed markers of a GCP shown first: not probably hidden, or pinned; by status (stable). */
  List<Marker> shown(String gcp) {
    List<Marker> out = new ArrayList<>();
    for (Marker k : markers) if (k.gcp.equals(gcp) && k.placed() && (!k.hidden || k.pinned())) out.add(k);
    out.sort(Comparator.comparingInt(k -> order(k.status)));
    return out;
  }

  /** Placed markers whose view of the GCP is probably blocked by the tie-point surface. */
  List<Marker> hidden(String gcp) {
    List<Marker> out = new ArrayList<>();
    for (Marker k : markers) if (k.gcp.equals(gcp) && k.placed() && k.hidden && !k.pinned()) out.add(k);
    return out;
  }

  /** Browsing order (previous / next): shown, then probably hidden. */
  List<Marker> cards(String gcp) {
    List<Marker> out = shown(gcp);
    out.addAll(hidden(gcp));
    return out;
  }

  /** Registered images without a marker for this GCP (placed by hand). */
  List<Image> others(String gcp) {
    Set<Integer> seen = new HashSet<>();
    for (Marker k : markers) if (k.gcp.equals(gcp) && k.placed()) seen.add(k.image);
    List<Image> out = new ArrayList<>();
    for (Image im : images) if (im.registered && !seen.contains(im.id)) out.add(im);
    return out;
  }

  /** Refined markers that "accept all" would pin. */
  int acceptable(String gcp) {
    int n = 0;
    for (Marker k : markers) if (k.gcp.equals(gcp) && k.status.equals("refined") && k.score != null && k.score >= accept) n++;
    return n;
  }

  /** Residual grade as in the web page: good below 1 px, mid below 3 px, else bad; null if none. */
  static String grade(Double residual) {
    if (residual == null) return null;
    return residual < 1 ? "good" : residual < 3 ? "mid" : "bad";
  }

  /** Status line of a marker, e.g. "refined 0.91", "pinned, 0.42 px", "predicted, probably hidden". */
  static String info(Marker k) {
    StringBuilder t = new StringBuilder(k.status);
    if (k.status.equals("refined") && k.score != null) t.append(String.format(Locale.ROOT, " %.2f", k.score));
    if (k.residual != null) t.append(String.format(Locale.ROOT, ", %.2f px", k.residual));
    else if (k.offset != null) t.append(String.format(Locale.ROOT, ", %.1f px from prediction", k.offset));
    if (k.hidden && !k.pinned()) t.append(", probably hidden");
    return t.toString();
  }

  /** Second line of a GCP row: pin RMS and max, refined count. */
  static String summary(Gcp g) {
    List<String> parts = new ArrayList<>();
    if (g.pinRms != null) parts.add(String.format(Locale.ROOT, "pin RMS %.2f px, max %.2f px", g.pinRms, g.pinMax == null ? Double.NaN : g.pinMax));
    if (g.refined > 0) parts.add(g.refined + " refined");
    return String.join(" · ", parts);
  }

  String readiness() {
    return ready ? "Ready for the gcp step." : "Still needed: " + needed;
  }

  private static List<Object> list(Object o) {
    return o instanceof List<?> l ? new ArrayList<>(l) : List.of();
  }

  @SuppressWarnings("unchecked")
  private static Map<String, Object> map(Object o) {
    return o instanceof Map<?, ?> m ? (Map<String, Object>) m : Map.of();
  }

  private static int integer(Object o) {
    return o instanceof Number n ? n.intValue() : 0;
  }

  private static double real(Object o) {
    return o instanceof Number n ? n.doubleValue() : Double.NaN;
  }

  private static Double boxed(Object o) {
    return o instanceof Number n && Double.isFinite(n.doubleValue()) ? n.doubleValue() : null;
  }
}
