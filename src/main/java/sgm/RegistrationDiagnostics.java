package sgm;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.nio.file.*;
import java.util.*;
import java.util.List;

/**
 * Why each image was or was not registered: how it was placed (seed, incremental, retry, a
 * further model, GPS-checked PnP), and for unregistered images the first failing condition:
 * no accepted pairs, a separate part of the match graph, too few registered tie points, or a
 * rejected PnP solve (with its correspondence and inlier counts). Written to
 * 04_geometry/registration.json (per image), registration_summary.json and, with GPS,
 * registration_map.png.
 */
final class RegistrationDiagnostics {
  final int images;
  final String[] method;
  final int[] model;
  final int[] attempts, correspondences, inliers, consistent;
  /** --registration-pose-check: rejections per image and the last rejection's description. */
  final int[] poseRejections;
  final String[] poseRejection;
  final boolean[] poseRejectedLast;
  /** Summary of the pose check (null = off). */
  Map<String, Object> poseCheck;
  /** Current label for successful registrations; null = not recorded (trial models). */
  String stage = "incremental";
  int currentModel = 0;
  final List<Object> models = new ArrayList<>();
  final Map<String, Object> recovery = new LinkedHashMap<>();

  RegistrationDiagnostics(int images) {
    this.images = images;
    method = new String[images];
    model = new int[images];
    Arrays.fill(model, -1);
    attempts = new int[images];
    correspondences = new int[images];
    inliers = new int[images];
    consistent = new int[images];
    poseRejections = new int[images];
    poseRejection = new String[images];
    poseRejectedLast = new boolean[images];
  }

  void attempt(int image, int points, int ransacInliers, int consistentInliers) {
    if (stage == null) return;
    attempts[image]++;
    correspondences[image] = points;
    inliers[image] = ransacInliers;
    consistent[image] = consistentInliers;
    poseRejectedLast[image] = false;
  }

  void poseRejected(int image, String why) {
    if (stage == null) return;
    poseRejections[image]++;
    poseRejection[image] = why;
    poseRejectedLast[image] = true;
  }

  void registered(int image) {
    if (stage == null) return;
    method[image] = stage;
    model[image] = currentModel;
  }

  void registered(int image, String how, int modelId) {
    method[image] = how;
    model[image] = modelId;
  }

  static final String[] REASONS = {
    "no_accepted_pairs", "separate_match_component", "too_few_registered_tie_points", "pnp_rejected",
    "pose_check_rejected"
  };

  static String explanation(String reason) {
    return switch (reason) {
      case "no_accepted_pairs" ->
          "No image pair with this image passed matching and geometric verification.";
      case "separate_match_component" ->
          "Its accepted pairs link it only to images outside the registered part of the match"
              + " graph; more or better pairs are needed (--match-repair, --neighbors).";
      case "too_few_registered_tie_points" ->
          "It shares too few triangulated tie points with registered images for PnP.";
      case "pnp_rejected" ->
          "Enough tie points, but PnP found too few consistent inliers.";
      case "pose_check_rejected" ->
          "PnP solved a pose, but --registration-pose-check rejected it (too far from its GPS"
              + " position or tilted beyond the rest of the block); see pose_check.";
      default -> "";
    };
  }

  /** Writes the per-image file, the summary and (with GPS) the map; prints a summary. */
  void write(List<ImageFrame> frames, FeatureTracks features, Options o, Project project)
      throws IOException {
    int n = frames.size();
    int minimum = Math.max(8, o.minMatches / 2);
    int[] degree = new int[n], strongest = new int[n];
    for (FeatureTracks.Pair p : features.pairs) {
      degree[p.a]++;
      degree[p.b]++;
      strongest[p.a] = Math.max(strongest[p.a], p.matches.size());
      strongest[p.b] = Math.max(strongest[p.b], p.matches.size());
    }
    int[] component = MatchRepair.components(n, features.pairs);
    int parts = 0;
    for (int c : component) parts = Math.max(parts, c + 1);
    int[] size = new int[parts], registeredIn = new int[parts];
    for (int i = 0; i < n; i++) {
      size[component[i]]++;
      if (frames.get(i).camera.registered) registeredIn[component[i]]++;
    }
    // The registered part of the graph: components holding registered images.
    int[] tiePoints = new int[n];
    for (FeatureTracks.Track t : features.tracks)
      if (t.point != null) {
        int last = -1;
        for (FeatureTracks.Observation ob : t.observations)
          if (ob.image != last) {
            tiePoints[ob.image]++;
            last = ob.image;
          }
      }
    Map<String, Integer> reasons = new LinkedHashMap<>();
    for (String r : REASONS) reasons.put(r, 0);
    Map<String, Integer> methods = new TreeMap<>();
    List<Object> rows = new ArrayList<>();
    String[] reasonOf = new String[n];
    for (int i = 0; i < n; i++) {
      ImageFrame f = frames.get(i);
      Map<String, Object> row = new LinkedHashMap<>();
      row.put("image_id", i);
      row.put("filename", f.path.getFileName().toString());
      row.put("registered", f.camera.registered);
      if (f.camera.registered) {
        String how = method[i] == null ? "incremental" : method[i];
        row.put("registered_by", how);
        row.put("model", Math.max(0, model[i]));
        methods.merge(how, 1, Integer::sum);
      } else {
        String reason =
            degree[i] == 0
                ? "no_accepted_pairs"
                : registeredIn[component[i]] == 0
                    ? "separate_match_component"
                    : tiePoints[i] < minimum
                        ? "too_few_registered_tie_points"
                        : poseRejectedLast[i] ? "pose_check_rejected" : "pnp_rejected";
        reasonOf[i] = reason;
        reasons.merge(reason, 1, Integer::sum);
        row.put("reason", reason);
        row.put("explanation", explanation(reason));
      }
      row.put("accepted_pairs", degree[i]);
      row.put("strongest_pair_matches", strongest[i]);
      row.put("match_component", component[i]);
      row.put("match_component_size", size[component[i]]);
      row.put("registered_tie_points", tiePoints[i]);
      row.put("pnp_attempts", attempts[i]);
      if (attempts[i] > 0) {
        row.put("last_pnp_correspondences", correspondences[i]);
        row.put("last_pnp_inliers", inliers[i]);
        row.put("last_pnp_consistent_inliers", consistent[i]);
      }
      if (poseRejections[i] > 0) {
        row.put("pose_check_rejections", poseRejections[i]);
        row.put("last_pose_check_rejection", poseRejection[i]);
      }
      rows.add(row);
    }
    Path dir = o.output.resolve("04_geometry");
    Files.createDirectories(dir);
    Json.write(dir.resolve("registration.json"), rows);
    int registered = 0;
    for (ImageFrame f : frames) if (f.camera.registered) registered++;
    List<Object> componentRows = new ArrayList<>();
    for (int c = 0; c < parts; c++)
      if (size[c] > 1 || registeredIn[c] > 0 || parts <= 50)
        componentRows.add(Json.object("component", c, "images", size[c], "registered", registeredIn[c]));
    boolean gps = frames.stream().allMatch(f -> f.gps != null && f.gps.usable());
    boolean map = gps && n >= 2 && map(frames, features, reasonOf, dir.resolve("registration_map.png"));
    Map<String, Object> summary = new LinkedHashMap<>();
    summary.put("images", n);
    summary.put("registered", registered);
    summary.put("unregistered", n - registered);
    summary.put("unregistered_reasons", reasons);
    summary.put("registered_by", methods);
    summary.put("pnp_minimum_inliers", minimum);
    summary.put("match_graph_components", parts);
    summary.put("components", componentRows);
    summary.put("registration_recovery", o.registrationRecovery);
    if (!models.isEmpty()) summary.put("models", models);
    if (!recovery.isEmpty()) summary.put("recovery", recovery);
    if (poseCheck != null) summary.put("pose_check", poseCheck);
    summary.put("map", map ? "registration_map.png" : null);
    Json.write(dir.resolve("registration_summary.json"), summary);
    if (registered < n) {
      List<String> parts2 = new ArrayList<>();
      for (var e : reasons.entrySet()) if (e.getValue() > 0) parts2.add(e.getValue() + " " + e.getKey().replace('_', ' '));
      System.out.println(
          "Registration: " + registered + "/" + n + " images; unregistered: " + String.join(", ", parts2)
              + " (see 04_geometry/registration.json)");
    }
    if (project != null)
      project.event("registration_summary", Json.object("registered", registered, "images", n, "reasons", reasons));
  }

  /** Registered (teal) and unregistered (red) images at their GPS positions, accepted pairs as lines. */
  private static boolean map(
      List<ImageFrame> frames, FeatureTracks features, String[] reasonOf, Path file) {
    try {
      double[][] p = MatchRepair.positions(frames);
      double minX = Double.MAX_VALUE, maxX = -Double.MAX_VALUE, minY = Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
      for (double[] q : p) {
        minX = Math.min(minX, q[0]);
        maxX = Math.max(maxX, q[0]);
        minY = Math.min(minY, q[1]);
        maxY = Math.max(maxY, q[1]);
      }
      double span = Math.max(1e-6, Math.max(maxX - minX, maxY - minY));
      int width = 1200, height = 900, margin = 60, legend = 120;
      double scale = Math.min((width - 2 * margin) / Math.max(1e-6, maxX - minX + span * 1e-3),
          (height - 2 * margin - legend) / Math.max(1e-6, maxY - minY + span * 1e-3));
      double cx = (minX + maxX) / 2, cy = (minY + maxY) / 2;
      BufferedImage image = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);
      Graphics2D g = image.createGraphics();
      g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
      g.setColor(Color.WHITE);
      g.fillRect(0, 0, width, height);
      int[] x = new int[p.length], y = new int[p.length];
      for (int i = 0; i < p.length; i++) {
        x[i] = (int) Math.round(width / 2.0 + (p[i][0] - cx) * scale);
        y[i] = (int) Math.round(legend + (height - legend) / 2.0 - (p[i][1] - cy) * scale);
      }
      int max = 1;
      for (FeatureTracks.Pair pair : features.pairs) max = Math.max(max, pair.matches.size());
      for (FeatureTracks.Pair pair : features.pairs) {
        double strength = Math.min(1, Math.log1p(pair.matches.size()) / Math.log1p(max));
        int shade = (int) (215 - 150 * strength);
        g.setColor(new Color(shade, shade, shade));
        g.setStroke(new BasicStroke((float) (0.5 + 2 * strength)));
        g.drawLine(x[pair.a], y[pair.a], x[pair.b], y[pair.b]);
      }
      g.setStroke(new BasicStroke(2f));
      int dot = p.length > 1000 ? 3 : p.length > 200 ? 4 : 6;
      for (int i = 0; i < p.length; i++) {
        if (frames.get(i).camera.registered) {
          g.setColor(new Color(0, 120, 135));
          g.fillOval(x[i] - dot, y[i] - dot, 2 * dot, 2 * dot);
        } else {
          g.setColor(switch (reasonOf[i]) {
            case "no_accepted_pairs" -> new Color(200, 30, 30);
            case "separate_match_component" -> new Color(220, 110, 0);
            case "too_few_registered_tie_points" -> new Color(170, 60, 170);
            default -> new Color(120, 70, 20);
          });
          g.drawLine(x[i] - dot - 1, y[i] - dot - 1, x[i] + dot + 1, y[i] + dot + 1);
          g.drawLine(x[i] - dot - 1, y[i] + dot + 1, x[i] + dot + 1, y[i] - dot - 1);
        }
      }
      g.setFont(new Font(Font.SANS_SERIF, Font.PLAIN, 16));
      int row = 28;
      Object[][] keys = {
        {new Color(0, 120, 135), "registered", true},
        {new Color(200, 30, 30), "no accepted pairs", false},
        {new Color(220, 110, 0), "separate part of the match graph", false},
        {new Color(170, 60, 170), "too few registered tie points", false},
        {new Color(120, 70, 20), "PnP or pose check rejected", false}
      };
      int lx = margin;
      for (Object[] k : keys) {
        g.setColor((Color) k[0]);
        if ((Boolean) k[2]) g.fillOval(lx, row - 10, 12, 12);
        else {
          g.drawLine(lx, row - 10, lx + 12, row + 2);
          g.drawLine(lx, row + 2, lx + 12, row - 10);
        }
        g.setColor(Color.DARK_GRAY);
        g.drawString((String) k[1], lx + 18, row + 2);
        lx += 18 + g.getFontMetrics().stringWidth((String) k[1]) + 30;
        if (lx > width - 300) {
          lx = margin;
          row += 24;
        }
      }
      g.setColor(Color.DARK_GRAY);
      g.drawString(
          String.format(
              Locale.ROOT,
              "GPS positions (east/north); lines: accepted image pairs, darker = more matches (max %d). Extent %.0f m",
              max,
              span),
          margin,
          row + 34);
      g.dispose();
      javax.imageio.ImageIO.write(image, "png", file.toFile());
      return true;
    } catch (IOException | RuntimeException e) {
      System.err.println("Registration map not written: " + e);
      return false;
    }
  }
}
