package sgm;

import java.awt.*;
import java.awt.event.*;
import java.io.IOException;
import java.nio.file.*;
import java.util.*;
import java.util.List;
import java.util.function.Consumer;
import javax.swing.*;
import javax.swing.border.EmptyBorder;

/**
 * Tie-point cleanup window (--tie-point-cleanup pause): histograms of the three measures of the
 * run's tie points (04_geometry/tie_points/metrics.csv) with the limits drawn in, a live count of
 * what the limits remove (TiePointCleanup.select, the same code as the step) and Apply / Skip,
 * which hand the choice to the caller (it writes it into the run and continues the run).
 */
final class TiePointWindow {
  static final Color KEEP = new Color(0x2563eb), DROP = new Color(0xdc2626), LIMIT = new Color(0x111827);

  final JDialog dialog;
  final List<TiePointCleanup.Point> points;
  final int cameras;
  final JSpinner maxError, minViews, minAngle, maxRemoved;
  final Histogram errors, views, angles;
  final JLabel summary = new JLabel(" ");
  final javax.swing.Timer refresh;
  // 3D view of the tie points: red the ones the limits remove (updated with the limits), grey the rest.
  final CloudView view3d = new CloudView();
  final CloudData cloud;
  final JCheckBox showKept = new JCheckBox("Kept points", true), showCameras = new JCheckBox("Cameras", true);
  final JLabel viewStatus = new JLabel(" ");
  final boolean positions;
  volatile boolean[] removed;

  static void open(Component parent, ProjectFiles.Run run, Consumer<TiePointCleanup.Choice> chosen) {
    Path file = run.dir().resolve(TiePointCleanup.FOLDER).resolve("metrics.csv");
    List<TiePointCleanup.Point> points;
    List<TiePointCleanup.Camera> cameras;
    try {
      points = TiePointCleanup.read(file);
      cameras = TiePointCleanup.readCameras(file.resolveSibling("cameras.csv"));
    } catch (IOException | RuntimeException e) {
      GuiSupport.error(parent, "Tie-point cleanup", e);
      return;
    }
    new TiePointWindow(parent, run, points, cameras, chosen).dialog.setVisible(true);
  }

  TiePointWindow(Component parent, ProjectFiles.Run run, List<TiePointCleanup.Point> points, List<TiePointCleanup.Camera> cameraList,
      Consumer<TiePointCleanup.Choice> chosen) {
    this.points = points;
    cloud = TiePointCleanup.cloud(points, cameraList);
    positions = points.stream().anyMatch(p -> Double.isFinite(p.x()));
    int max = 0;
    for (TiePointCleanup.Point p : points) for (int c : p.cameras()) max = Math.max(max, c + 1);
    cameras = max;
    Window owner = parent == null ? null : SwingUtilities.getWindowAncestor(parent) != null ? SwingUtilities.getWindowAncestor(parent) : parent instanceof Window w ? w : null;
    dialog = new JDialog(owner, "Tie-point cleanup - run " + run.id(), Dialog.ModalityType.MODELESS);
    GuiSupport.brand(dialog);
    dialog.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
    dialog.addWindowListener(new WindowAdapter() {
      @Override
      public void windowClosed(WindowEvent ev) {
        view3d.dispose();
      }
    });

    // Start from the run's own --tie-point-* values (else the defaults; automatic largest error).
    Map<String, String> given = given(run.arguments());
    double e0 = parse(given.get("--tie-point-max-error"), 0);
    if (!(e0 > 0)) e0 = TiePointCleanup.autoMaxError(points);
    maxError = new JSpinner(new SpinnerNumberModel(e0, 0.0, 1000.0, 0.05));
    minViews = new JSpinner(new SpinnerNumberModel((int) parse(given.get("--tie-point-min-views"), 2), 2, 100, 1));
    minAngle = new JSpinner(new SpinnerNumberModel(parse(given.get("--tie-point-min-angle"), 1), 0.0, 89.0, 0.1));
    maxRemoved = new JSpinner(new SpinnerNumberModel(parse(given.get("--tie-point-max-removed"), 20), 0.1, 100.0, 1.0));

    float[] e = new float[points.size()], v = new float[points.size()], a = new float[points.size()];
    for (int i = 0; i < e.length; i++) {
      e[i] = points.get(i).error();
      v[i] = points.get(i).views();
      a[i] = points.get(i).angle();
    }
    errors = new Histogram("Largest reprojection error (px)", e, 0, Math.max(.5, quantile(e, .995)), 60, true, false);
    views = new Histogram("Images that see the point", v, 1.5, Math.max(6.5, Math.min(30.5, quantile(v, .999) + 1)), 0, false, true);
    angles = new Histogram("Largest intersection angle (degrees)", a, 0, Math.max(5, quantile(a, .99)), 60, false, false);
    errors.onPick = x -> maxError.setValue(Math.round(x * 100) / 100.0);
    views.onPick = x -> minViews.setValue((int) Math.max(2, Math.round(x + .5)));
    angles.onPick = x -> minAngle.setValue(Math.round(x * 10) / 10.0);

    refresh = new javax.swing.Timer(120, ev -> update());
    refresh.setRepeats(false);
    for (JSpinner s : List.of(maxError, minViews, minAngle, maxRemoved)) s.addChangeListener(ev -> refresh.restart());

    JPanel charts = new JPanel(new GridLayout(1, 3, 10, 0));
    charts.add(column(errors, "Remove above", maxError, "px",
        "Points whose largest reprojection error is above this are removed (Metashape: reprojection error)."));
    charts.add(column(views, "Remove seen in fewer than", minViews, "images",
        "Points seen in fewer images are removed (Metashape: image count). 2 removes none."));
    charts.add(column(angles, "Remove below", minAngle, "degrees",
        "Points whose rays meet at a smaller angle have an uncertain depth (Metashape: reconstruction uncertainty)."));

    JPanel limitRow = new JPanel(new FlowLayout(FlowLayout.LEFT, 6, 0));
    limitRow.add(new JLabel("Remove at most"));
    limitRow.add(maxRemoved);
    limitRow.add(new JLabel("% of the points, worst first. Every camera keeps at least " + TiePointCleanup.MIN_SUPPORT + " points."));
    maxRemoved.setToolTipText("Worst first: most limits failed, then the largest error, then the smallest angle");
    JButton auto = new JButton("Automatic limits");
    auto.setToolTipText("Largest error 3x the median, 2 images, 1 degree, at most 20% (as --tie-point-cleanup auto)");
    auto.addActionListener(ev -> {
      TiePointCleanup.Thresholds limits = TiePointCleanup.automatic(points);
      maxError.setValue(limits.maxError());
      minViews.setValue(limits.minViews());
      minAngle.setValue(limits.minAngle());
      maxRemoved.setValue(limits.maxRemovedPercent());
    });
    limitRow.add(auto);

    JButton apply = new JButton("Apply and continue the run"), skip = new JButton("Skip the cleanup and continue"), cancel = new JButton("Cancel");
    apply.addActionListener(ev -> {
      dialog.dispose();
      chosen.accept(new TiePointCleanup.Choice(true, thresholds()));
    });
    skip.addActionListener(ev -> {
      dialog.dispose();
      chosen.accept(new TiePointCleanup.Choice(false, null));
    });
    cancel.addActionListener(ev -> dialog.dispose());
    apply.setToolTipText("Removes the selected points, adjusts the block again and runs the remaining steps");
    skip.setToolTipText("Keeps all tie points and runs the remaining steps");
    cancel.setToolTipText("The run keeps waiting; open this window again from the Project tab");
    JPanel buttons = new JPanel(new FlowLayout(FlowLayout.RIGHT, 6, 0));
    buttons.add(skip);
    buttons.add(cancel);
    buttons.add(apply);
    dialog.getRootPane().setDefaultButton(apply);

    JLabel intro = new JLabel(String.format(Locale.ROOT,
        "<html>%,d tie points in %d images, measured after alignment. Red bars fail that limit. Click a histogram to"
            + " set its limit. After Apply the block is adjusted again, so the final error drops further.</html>",
        points.size(), cameras));
    summary.setBorder(new EmptyBorder(6, 0, 6, 0));
    JPanel bottom = new JPanel(new BorderLayout());
    bottom.add(limitRow, BorderLayout.NORTH);
    bottom.add(summary, BorderLayout.CENTER);
    bottom.add(buttons, BorderLayout.SOUTH);
    JPanel content = new JPanel(new BorderLayout(0, 8));
    content.setBorder(new EmptyBorder(10, 12, 10, 12));
    content.add(intro, BorderLayout.NORTH);
    content.add(split(charts), BorderLayout.CENTER);
    content.add(bottom, BorderLayout.SOUTH);
    dialog.setContentPane(content);
    update();
    dialog.setSize(1240, 960);
    view3d.setData(cloud, true);
    dialog.setLocationRelativeTo(owner);
  }

  /** The histograms above, the 3D view of the tie points below (resizable). */
  JComponent split(JComponent charts) {
    String legend = positions
        ? "Red: removed by these limits. Grey: kept."
        : "This run's tie-point table has no positions (made before the 3D view): run the cleanup step again to see them.";
    view3d.look = () -> new CloudView.Look(CloudRenderer.Colour.RGB, -1, 2, false, false, true, showCameras.isSelected(), 0, 1, legend);
    view3d.status = s -> viewStatus.setText(s == null ? "Left drag: orbit, right drag: pan, wheel: zoom, double-click: centre. Hover a point for its measures." : s);
    view3d.sparseText = i -> {
      if (i < 0 || i >= points.size()) return null;
      TiePointCleanup.Point p = points.get(i);
      boolean[] r = removed;
      return String.format(Locale.ROOT, "Tie point %d: largest error %.2f px, seen in %d images, largest angle %.1f degrees - %s",
          p.track(), p.error(), p.views(), p.angle(), r != null && i < r.length && r[i] ? "removed" : "kept");
    };
    view3d.status.accept(null);
    showKept.setToolTipText("Show the points the limits keep too (grey); off: only the removed ones (red)");
    showCameras.setToolTipText("The cameras of the alignment");
    showKept.addActionListener(ev -> update());
    showCameras.addActionListener(ev -> view3d.redraw());
    JButton home = new JButton("Home"), top = new JButton("Top");
    home.addActionListener(ev -> view3d.home());
    top.addActionListener(ev -> {
      if (view3d.view != null) view3d.setView(view3d.view.withAngles(0, 90).withOrtho(true), false);
    });
    home.setToolTipText("Everything in view, at an angle");
    top.setToolTipText("Straight down (the alignment frame is turned so that the cameras look down)");
    JToolBar bar = new JToolBar();
    bar.setFloatable(false);
    bar.add(new JLabel(" 3D view of the tie points   "));
    bar.add(showKept);
    bar.add(showCameras);
    bar.addSeparator();
    bar.add(home);
    bar.add(top);
    JPanel panel = new JPanel(new BorderLayout());
    panel.add(bar, BorderLayout.NORTH);
    view3d.setPreferredSize(new Dimension(800, 420));
    panel.add(view3d, BorderLayout.CENTER);
    viewStatus.setBorder(new EmptyBorder(2, 4, 2, 4));
    panel.add(viewStatus, BorderLayout.SOUTH);
    JSplitPane split = new JSplitPane(JSplitPane.VERTICAL_SPLIT, charts, panel);
    split.setResizeWeight(.35);
    split.setBorder(null);
    return split;
  }

  TiePointCleanup.Thresholds thresholds() {
    return new TiePointCleanup.Thresholds(((Number) maxError.getValue()).doubleValue(), ((Number) minViews.getValue()).intValue(),
        ((Number) minAngle.getValue()).doubleValue(), ((Number) maxRemoved.getValue()).doubleValue());
  }

  void update() {
    TiePointCleanup.Thresholds t = thresholds();
    TiePointCleanup.Selection s = TiePointCleanup.select(points, cameras, t);
    errors.limit = t.maxError();
    views.limit = t.minViews() - .5;
    angles.limit = t.minAngle();
    for (Histogram h : List.of(errors, views, angles)) h.repaint();
    removed = s.remove();
    TiePointCleanup.colour(cloud, points, s.remove(), showKept.isSelected());
    view3d.redraw();
    int n = points.size();
    summary.setText(String.format(Locale.ROOT,
        "<html><b>Removes %,d of %,d points (%.1f%%).</b> Failing: error %,d, images %,d, angle %,d; kept because a camera needs"
            + " them %,d; kept over the %.0f%% limit %,d.<br>RMS error of the points %.3f px now, %.3f px without the removed ones"
            + " (before the new adjustment). The camera with the fewest points keeps %,d.</html>",
        s.removed(), n, n == 0 ? 0 : 100.0 * s.removed() / n, s.byError(), s.byViews(), s.byAngle(), s.protectedBySupport(),
        t.maxRemovedPercent(), s.capped(), TiePointCleanup.rms(points, null), TiePointCleanup.rms(points, s.remove()),
        TiePointCleanup.weakestSupport(points, s.remove(), cameras)));
  }

  private static JPanel column(Histogram h, String label, JSpinner field, String unit, String tip) {
    JPanel p = new JPanel(new BorderLayout(0, 4));
    p.add(h, BorderLayout.CENTER);
    JPanel row = new JPanel(new FlowLayout(FlowLayout.LEFT, 4, 0));
    row.add(new JLabel(label));
    ((JSpinner.DefaultEditor) field.getEditor()).getTextField().setColumns(5);
    row.add(field);
    row.add(new JLabel(unit));
    row.setToolTipText(tip);
    field.setToolTipText(tip);
    h.setToolTipText(tip);
    p.add(row, BorderLayout.SOUTH);
    return p;
  }

  static Map<String, String> given(List<String> arguments) {
    Map<String, String> out = new HashMap<>();
    if (arguments == null) return out;
    for (int i = 0; i < arguments.size(); i++) {
      String a = arguments.get(i);
      if (!a.startsWith("--tie-point-")) continue;
      int eq = a.indexOf('=');
      if (eq > 0) out.put(a.substring(0, eq), a.substring(eq + 1));
      else if (i + 1 < arguments.size()) out.put(a, arguments.get(++i));
    }
    return out;
  }

  static double parse(String s, double fallback) {
    if (s == null || s.equalsIgnoreCase("auto")) return fallback == 0 ? 0 : fallback;
    try {
      return Double.parseDouble(s.trim());
    } catch (NumberFormatException e) {
      return fallback;
    }
  }

  static float quantile(float[] v, double q) {
    if (v.length == 0) return 0;
    float[] s = v.clone();
    Arrays.sort(s);
    return s[(int) Math.min(s.length - 1, Math.floor(q * (s.length - 1)))];
  }

  /** A histogram with a limit line; values past the range go into the last bin. */
  static final class Histogram extends JComponent {
    final String title;
    final double from, to;
    final int[] counts;
    final boolean above, integer;
    double limit = Double.NaN;
    java.util.function.DoubleConsumer onPick;

    /** above: values above the limit fail (else below); integer: one bar per whole value. */
    Histogram(String title, float[] values, double from, double to, int bins, boolean above, boolean integer) {
      this.title = title;
      this.from = from;
      this.to = to;
      this.above = above;
      this.integer = integer;
      int n = integer ? (int) Math.round(to - from) : bins;
      counts = new int[Math.max(1, n)];
      for (float v : values) counts[bin(v)]++;
      setPreferredSize(new Dimension(360, 260));
      addMouseListener(new MouseAdapter() {
        @Override
        public void mouseClicked(MouseEvent e) {
          if (onPick == null) return;
          Rectangle plot = plot();
          if (plot.width <= 0) return;
          double x = from + (to - from) * (e.getX() - plot.x) / plot.width;
          onPick.accept(Math.max(from, Math.min(to, x)));
        }
      });
      setCursor(Cursor.getPredefinedCursor(Cursor.CROSSHAIR_CURSOR));
    }

    int bin(double v) {
      int b = (int) Math.floor((v - from) / (to - from) * counts.length);
      return Math.max(0, Math.min(counts.length - 1, b));
    }

    Rectangle plot() {
      Insets in = getInsets();
      return new Rectangle(in.left + 44, in.top + 22, getWidth() - in.left - in.right - 54, getHeight() - in.top - in.bottom - 44);
    }

    @Override
    protected void paintComponent(Graphics g0) {
      Graphics2D g = (Graphics2D) g0.create();
      g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
      g.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
      g.setColor(getBackground() == null ? Color.WHITE : UIManager.getColor("Panel.background"));
      g.fillRect(0, 0, getWidth(), getHeight());
      Rectangle p = plot();
      FontMetrics fm = g.getFontMetrics();
      g.setColor(UIManager.getColor("Label.foreground"));
      g.drawString(title, p.x, p.y - 8);
      if (p.width <= 0 || p.height <= 0) return;
      g.setColor(Color.WHITE);
      g.fillRect(p.x, p.y, p.width, p.height);
      int max = 1;
      for (int c : counts) max = Math.max(max, c);
      // Square-root heights: the long tails stay visible next to the peak.
      double scale = p.height / Math.sqrt(max);
      double w = (double) p.width / counts.length;
      for (int i = 0; i < counts.length; i++) {
        if (counts[i] == 0) continue;
        double lo = from + (to - from) * i / counts.length, hi = from + (to - from) * (i + 1) / counts.length, mid = (lo + hi) / 2;
        boolean fail = !Double.isNaN(limit) && (above ? mid > limit : mid < limit);
        g.setColor(fail ? DROP : KEEP);
        int h = (int) Math.max(1, Math.round(Math.sqrt(counts[i]) * scale));
        int x0 = p.x + (int) Math.round(i * w), x1 = p.x + (int) Math.round((i + 1) * w);
        g.fillRect(x0, p.y + p.height - h, Math.max(1, x1 - x0 - (integer ? 2 : 0)), h);
      }
      g.setColor(Color.GRAY);
      g.drawRect(p.x, p.y, p.width, p.height);
      // Axis labels: range ends and a middle value; counts at the top.
      g.setColor(UIManager.getColor("Label.foreground"));
      String left = integer ? "" + (int) Math.round(from + .5) : fmt(from), right = (integer ? "" + (int) Math.round(to - .5) : fmt(to)) + "+";
      int base = p.y + p.height + fm.getAscent() + 2;
      // The limit's label first; a range end label is left out where it would overlap it.
      int lx0 = Integer.MIN_VALUE, lx1 = Integer.MIN_VALUE;
      if (!Double.isNaN(limit) && limit >= from && limit <= to) {
        int x = p.x + (int) Math.round((limit - from) / (to - from) * p.width);
        g.setColor(LIMIT);
        g.setStroke(new BasicStroke(2f));
        g.drawLine(x, p.y, x, p.y + p.height);
        String label = integer ? "" + (int) Math.round(limit + .5) : fmt(limit);
        lx0 = Math.min(p.x + p.width - fm.stringWidth(label), Math.max(p.x, x - fm.stringWidth(label) / 2));
        lx1 = lx0 + fm.stringWidth(label);
        g.drawString(label, lx0, base);
      }
      g.setColor(UIManager.getColor("Label.foreground"));
      int gap = 6;
      if (p.x + fm.stringWidth(left) + gap <= lx0 || p.x >= lx1 + gap) g.drawString(left, p.x, base);
      int rx = p.x + p.width - fm.stringWidth(right);
      if (rx >= lx1 + gap || rx + fm.stringWidth(right) + gap <= lx0) g.drawString(right, rx, base);
      String peak = String.format(Locale.ROOT, "%,d", max);
      g.drawString(peak, p.x - fm.stringWidth(peak) - 4, p.y + fm.getAscent());
      g.drawString("0", p.x - fm.stringWidth("0") - 4, p.y + p.height);
      g.dispose();
    }

    static String fmt(double v) {
      return v >= 10 ? String.format(Locale.ROOT, "%.0f", v) : String.format(Locale.ROOT, "%.2f", v);
    }
  }
}
