package sgm;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.nio.file.*;
import java.util.*;
import java.util.List;
import javax.imageio.ImageIO;
import javax.swing.*;
import javax.swing.border.EmptyBorder;

/**
 * The camera registration of a run (Project tab, "Registration..."): how images were registered,
 * why the others were not (04_geometry/registration.json, registration_summary.json), match
 * repair and recovery results, and the GPS map; buttons redo the alignment with
 * --registration-recovery on, or the matching with --match-repair on as well. Reads files only.
 */
final class RegistrationWindow {
  private RegistrationWindow() {}

  static boolean available(ProjectFiles.Run run) {
    return run != null && Files.isRegularFile(run.dir().resolve("04_geometry/registration_summary.json"));
  }

  static void open(Component parent, ProjectFiles.Run run, Runnable redoAlign, Runnable redoMatch) {
    JFrame frame = new JFrame("Registration - run " + run.id());
    GuiSupport.brand(frame);
    frame.setContentPane(content(run, redoAlign, redoMatch, frame::dispose));
    frame.setSize(1150, 760);
    frame.setLocationRelativeTo(parent);
    frame.setVisible(true);
  }

  static JPanel content(ProjectFiles.Run run, Runnable redoAlign, Runnable redoMatch, Runnable close) {
    JEditorPane text = new JEditorPane("text/html", html(run.dir()));
    text.setEditable(false);
    text.putClientProperty(JEditorPane.HONOR_DISPLAY_PROPERTIES, Boolean.TRUE);
    text.setCaretPosition(0);
    JPanel picture = new JPanel(new BorderLayout());
    picture.setBorder(new EmptyBorder(6, 6, 6, 6));
    Path map = run.dir().resolve("04_geometry/registration_map.png");
    if (Files.isRegularFile(map))
      try {
        BufferedImage b = ImageIO.read(map.toFile());
        double s = Math.min(1, 560.0 / Math.max(1, b.getWidth()));
        picture.add(new JLabel(new ImageIcon(b.getScaledInstance(Math.max(1, (int) (b.getWidth() * s)), Math.max(1, (int) (b.getHeight() * s)), Image.SCALE_SMOOTH))), BorderLayout.NORTH);
      } catch (Exception e) {
        picture.add(new JLabel("Could not read " + map));
      }
    else picture.add(new JLabel("No map (the images have no usable GPS)."), BorderLayout.NORTH);
    JSplitPane split = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT, new JScrollPane(text), new JScrollPane(picture));
    split.setResizeWeight(.45);
    JButton align = new JButton("Redo alignment with recovery...");
    align.setToolTipText("<html>A new run from this one just before the align step, with --registration-recovery on:<br>"
        + "further models are grown from the unregistered images and merged (shared images or GPS),<br>"
        + "then GPS-checked PnP. This run is not changed.</html>");
    align.addActionListener(e -> redoAlign.run());
    JButton match = new JButton("Redo matching with repair...");
    match.setToolTipText("<html>A new run from this one just before the match step, with --match-repair on and<br>"
        + "--registration-recovery on: weakly connected images are matched against more candidates.</html>");
    match.addActionListener(e -> redoMatch.run());
    JButton closeButton = new JButton("Close");
    closeButton.addActionListener(e -> close.run());
    JPanel buttons = new JPanel(new FlowLayout(FlowLayout.RIGHT));
    buttons.add(match);
    buttons.add(align);
    buttons.add(closeButton);
    JPanel content = new JPanel(new BorderLayout(4, 4));
    content.add(split, BorderLayout.CENTER);
    content.add(buttons, BorderLayout.SOUTH);
    return content;
  }

  /** The text side, from the run's files (windowless; tested). */
  static String html(Path runDir) {
    StringBuilder h = new StringBuilder("<html><body style='font-family:sans-serif'>");
    Map<String, Object> summary;
    try {
      summary = ProcessingReport.registration(runDir, null);
    } catch (Exception e) {
      summary = null;
    }
    if (summary == null) {
      h.append("<p>No registration details in this run (made before they were recorded, or the align step has not run).</p>");
      return h.append("</body></html>").toString();
    }
    int images = number(summary.get("images")).intValue(), registered = number(summary.get("registered")).intValue();
    h.append("<h3>").append(registered).append(" of ").append(images).append(" images registered</h3>");
    var by = ProcessingReport.object(summary.get("registered_by"));
    if (!by.isEmpty()) {
      h.append("<p><b>Registered by:</b> ");
      List<String> parts = new ArrayList<>();
      for (var e : by.entrySet()) parts.add(GuiOptions.escape(e.getKey()) + " " + number(e.getValue()).intValue());
      h.append(String.join(", ", parts)).append("</p>");
    }
    h.append("<p><b>Match graph:</b> ").append(number(summary.get("match_graph_components")).intValue()).append(" connected part(s)</p>");
    var repair = ProcessingReport.object(summary.get("match_repair"));
    if (!repair.isEmpty())
      h.append("<p><b>Match repair:</b> ").append(number(repair.get("weak_image_count")).intValue()).append(" weak images; ")
          .append(number(repair.get("accepted_extra_pairs")).intValue()).append(" of ").append(number(repair.get("extra_pairs")).intValue())
          .append(" extra pairs accepted; parts ").append(number(repair.get("components_before")).intValue()).append(" &rarr; ")
          .append(number(repair.get("components_after")).intValue()).append("</p>");
    var recovery = ProcessingReport.object(summary.get("recovery"));
    if (!recovery.isEmpty())
      h.append("<p><b>Recovery:</b> ").append(number(recovery.get("registered_before")).intValue()).append(" &rarr; ")
          .append(number(recovery.get("registered_after")).intValue()).append(" registered; ")
          .append(number(recovery.get("models_merged")).intValue()).append(" models merged, ")
          .append(number(recovery.get("models_rejected")).intValue()).append(" rejected, ")
          .append(number(recovery.get("gps_checked_pnp")).intValue()).append(" by GPS-checked PnP</p>");
    if (summary.get("models") instanceof List<?> models && !models.isEmpty()) {
      h.append("<table border='1' cellspacing='0' cellpadding='3'><tr><th>Model</th><th>Images</th><th>New</th><th>Merged via</th><th>Fit RMS / limit</th><th>Merged</th></tr>");
      for (Object item : models) {
        var m = ProcessingReport.object(item);
        h.append("<tr><td>").append(number(m.get("model")).intValue()).append("</td><td>").append(number(m.get("images")).intValue())
            .append("</td><td>").append(number(m.get("new_images")).intValue()).append("</td><td>").append(GuiOptions.escape(String.valueOf(m.get("merged_via"))))
            .append("</td><td>").append(String.format(Locale.ROOT, "%.3g / %.3g %s", number(m.get("fit_rms")), number(m.get("fit_limit")), m.get("fit_units")))
            .append("</td><td>").append(Boolean.TRUE.equals(m.get("accepted")) ? "yes" : "no").append("</td></tr>");
      }
      h.append("</table>");
    }
    var reasons = ProcessingReport.object(summary.get("unregistered_reasons"));
    boolean any = false;
    for (var e : reasons.entrySet()) if (number(e.getValue()) > 0) any = true;
    if (any) {
      h.append("<h4>Why images were not registered</h4><ul>");
      for (var e : reasons.entrySet())
        if (number(e.getValue()) > 0)
          h.append("<li><b>").append(GuiOptions.escape(e.getKey().replace('_', ' '))).append("</b>: ").append(number(e.getValue()).intValue())
              .append(" &mdash; ").append(GuiOptions.escape(RegistrationDiagnostics.explanation(e.getKey()))).append("</li>");
      h.append("</ul>");
    }
    if (summary.get("unregistered_images") instanceof List<?> list && !list.isEmpty()) {
      h.append("<table border='1' cellspacing='0' cellpadding='3'><tr><th>Image</th><th>Reason</th><th>Pairs</th><th>Tie points</th><th>PnP inliers</th></tr>");
      for (Object item : list.subList(0, Math.min(200, list.size()))) {
        var r = ProcessingReport.object(item);
        h.append("<tr><td>").append(GuiOptions.escape(String.valueOf(r.get("filename")))).append("</td><td>")
            .append(GuiOptions.escape(String.valueOf(r.get("reason")).replace('_', ' '))).append("</td><td>")
            .append(number(r.get("accepted_pairs")).intValue()).append("</td><td>").append(number(r.get("registered_tie_points")).intValue())
            .append("</td><td>").append(r.get("last_pnp_inliers") == null ? "-" : String.valueOf(number(r.get("last_pnp_inliers")).intValue())).append("</td></tr>");
      }
      h.append("</table>");
      if (list.size() > 200) h.append("<p>... ").append(list.size() - 200).append(" more in 04_geometry/registration.json</p>");
    }
    return h.append("</body></html>").toString();
  }

  private static Double number(Object v) {
    return v instanceof Number n ? n.doubleValue() : 0.0;
  }
}
