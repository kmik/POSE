package sgm;

import java.awt.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.*;
import java.util.List;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicLong;
import javax.swing.*;
import javax.swing.border.*;

/**
 * The photographs of a run in their own window: previews, one photograph large (zoom, pan), and a
 * map of the photographs projected to world coordinates through the run's final orientations and
 * surface ({@link RunImages}), each switched on or off, with opacity and outlines to see how they
 * align. Nothing is written into the project; decoded previews are cached in the settings folder.
 */
final class ImageWindow {
  private static final Map<Path, ImageWindow> OPEN = new HashMap<>();

  /** Opens (or brings to front) the window of a run. */
  static void open(Component parent, ProjectFiles.Run run) {
    ImageWindow w = OPEN.get(run.dir());
    if (w != null && w.frame.isDisplayable()) {
      w.frame.toFront();
      return;
    }
    RunImages images = RunImages.load(run.dir());
    if (images.images.isEmpty()) {
      JOptionPane.showMessageDialog(parent, "The run has no image list yet (00_inputs/images.csv).", "Images", JOptionPane.INFORMATION_MESSAGE);
      return;
    }
    OPEN.put(run.dir(), new ImageWindow(run, images, parent));
  }

  static final int PREVIEW = 1024, THUMB = 180;

  final RunImages images;
  final JFrame frame;
  final ExecutorService loader = Executors.newFixedThreadPool(3, GcpEditorWindow.daemon("image-window-decode"));
  final ExecutorService renderer = Executors.newSingleThreadExecutor(GcpEditorWindow.daemon("image-window-map"));
  final ExecutorService full = Executors.newSingleThreadExecutor(GcpEditorWindow.daemon("image-window-full"));
  final Map<Integer, BufferedImage> previews = new ConcurrentHashMap<>();
  final Set<Integer> loading = ConcurrentHashMap.newKeySet();
  final Map<Integer, JCheckBox> onMap = new LinkedHashMap<>();
  final Map<Integer, JPanel> cards = new LinkedHashMap<>();
  final JPanel grid = new JPanel(new GuiSupport.WrapLayout(6));
  final JTabbedPane views = new JTabbedPane();
  final ZoomView photo = new ZoomView();
  final JLabel photoInfo = new JLabel(" "), mapInfo = new JLabel(" ");
  final MapPanel map;
  final JSlider opacity = new JSlider(10, 100, 100);
  final JCheckBox outlines = new JCheckBox("Outlines", true), selectedOnTop = new JCheckBox("Selected on top", true);
  RunImages.Image selected;
  final Path cache = GuiOptions.Settings.user().dir.resolve("image-cache");

  ImageWindow(ProjectFiles.Run run, RunImages images, Component parent) {
    this.images = images;
    // Settings > Preferences: the preview cache is kept under its size limit (oldest first).
    long limit = (long) GuiPreferences.user().cacheMb << 20;
    Thread trim = new Thread(() -> GuiPreferences.trim(cache, limit), "pose-cache-trim");
    trim.setDaemon(true);
    trim.start();
    frame = new JFrame("Images - run " + run.id());
    GuiSupport.brand(frame);
    frame.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
    frame.addWindowListener(
        new WindowAdapter() {
          @Override
          public void windowClosed(WindowEvent e) {
            loader.shutdownNow();
            renderer.shutdownNow();
            full.shutdownNow();
            previews.clear();
            OPEN.remove(run.dir());
          }
        });
    map = new MapPanel();
    for (RunImages.Image im : images.images) grid.add(card(im));
    JScrollPane gridScroll = new JScrollPane(grid);
    gridScroll.getVerticalScrollBar().setUnitIncrement(24);
    JButton all = new JButton("All"), none = new JButton("None"), flagged = new JButton("Flagged");
    all.setToolTipText("Show every oriented photograph on the map.");
    none.setToolTipText("Hide every photograph from the map.");
    flagged.setToolTipText("Show only the photographs flagged by camera screening.");
    all.addActionListener(e -> setOnMap(im -> true));
    none.addActionListener(e -> setOnMap(im -> false));
    flagged.addActionListener(e -> setOnMap(im -> im.flag() != null));
    JPanel leftTop = new JPanel(new FlowLayout(FlowLayout.LEFT, 4, 2));
    leftTop.add(new JLabel("On map:"));
    leftTop.add(all);
    leftTop.add(none);
    leftTop.add(flagged);
    long oriented = images.images.stream().filter(RunImages.Image::oriented).count();
    JLabel count = new JLabel(images.images.size() + " photographs, " + oriented + " oriented");
    count.setBorder(new EmptyBorder(0, 6, 4, 6));
    JPanel left = new JPanel(new BorderLayout());
    JPanel north = new JPanel(new BorderLayout());
    north.add(leftTop, BorderLayout.NORTH);
    north.add(count, BorderLayout.SOUTH);
    left.add(north, BorderLayout.NORTH);
    left.add(gridScroll, BorderLayout.CENTER);

    JPanel photoPanel = new JPanel(new BorderLayout());
    photoPanel.add(photoInfo, BorderLayout.NORTH);
    photoPanel.add(photo, BorderLayout.CENTER);
    JLabel photoHelp = new JLabel("Mouse wheel zooms, drag pans, double-click fits.");
    photoPanel.add(photoHelp, BorderLayout.SOUTH);

    JPanel mapPanel = new JPanel(new BorderLayout());
    JPanel mapTools = new JPanel(new FlowLayout(FlowLayout.LEFT, 6, 2));
    mapTools.add(new JLabel("Opacity"));
    opacity.setPreferredSize(new Dimension(140, opacity.getPreferredSize().height));
    opacity.setToolTipText("Below 100 %, overlapping photographs show through each other: misaligned ones look doubled.");
    mapTools.add(opacity);
    mapTools.add(outlines);
    mapTools.add(selectedOnTop);
    JButton fit = new JButton("Fit");
    fit.addActionListener(e -> map.fit());
    mapTools.add(fit);
    opacity.addChangeListener(e -> {
      if (!opacity.getValueIsAdjusting()) map.rerender();
    });
    outlines.addActionListener(e -> map.repaint());
    selectedOnTop.addActionListener(e -> map.rerender());
    mapPanel.add(mapTools, BorderLayout.NORTH);
    if (images.mappable()) mapPanel.add(map, BorderLayout.CENTER);
    else
      mapPanel.add(
          new JLabel(
              "<html><div style='padding:20px'>No map yet: the photographs' final orientations are written when the dense step starts"
                  + " (04_geometry/orientation_model.json).</div></html>"),
          BorderLayout.CENTER);
    mapPanel.add(mapInfo, BorderLayout.SOUTH);
    mapInfo.setText(images.mappable() ? "Projected onto " + images.surface.description() + (images.crs == null ? "" : "; coordinates in " + images.crs) : " ");
    views.addTab("Map", mapPanel);
    views.addTab("Image", photoPanel);
    JSplitPane split = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT, left, views);
    split.setResizeWeight(.3);
    frame.setContentPane(split);
    Rectangle screen = GraphicsEnvironment.getLocalGraphicsEnvironment().getMaximumWindowBounds();
    frame.setSize((int) (screen.width * .85), (int) (screen.height * .85));
    frame.setLocationRelativeTo(parent);
    frame.setVisible(true);
    split.setDividerLocation(.3);
    if (!images.mappable()) views.setSelectedIndex(1);
    for (RunImages.Image im : images.images) load(im);
    SwingUtilities.invokeLater(map::fit);
  }

  interface Pick {
    boolean on(RunImages.Image im);
  }

  void setOnMap(Pick p) {
    for (RunImages.Image im : images.images) {
      JCheckBox b = onMap.get(im.id());
      if (b != null) b.setSelected(im.oriented() && p.on(im));
    }
    map.rerender();
  }

  List<RunImages.Image> mapOrder() {
    List<RunImages.Image> out = new ArrayList<>();
    for (RunImages.Image im : images.images) if (im.oriented() && onMap.get(im.id()).isSelected()) out.add(im);
    if (selectedOnTop.isSelected() && selected != null && out.remove(selected)) out.add(selected);
    return out;
  }

  JPanel card(RunImages.Image im) {
    JPanel c = new JPanel(new BorderLayout());
    JComponent thumb =
        new JComponent() {
          @Override
          protected void paintComponent(Graphics g0) {
            Graphics2D g = (Graphics2D) g0.create();
            g.setColor(Color.DARK_GRAY);
            g.fillRect(0, 0, getWidth(), getHeight());
            BufferedImage b = previews.get(im.id());
            if (b == null) {
              g.setColor(Color.LIGHT_GRAY);
              g.drawString("loading...", 8, 18);
            } else {
              double s = Math.min(getWidth() / (double) b.getWidth(), getHeight() / (double) b.getHeight());
              int w = (int) (b.getWidth() * s), h = (int) (b.getHeight() * s);
              g.setRenderingHint(RenderingHints.KEY_INTERPOLATION, RenderingHints.VALUE_INTERPOLATION_BILINEAR);
              g.drawImage(b, (getWidth() - w) / 2, (getHeight() - h) / 2, w, h, null);
            }
            g.dispose();
          }
        };
    thumb.setPreferredSize(new Dimension(THUMB, THUMB * 3 / 4));
    String info = !im.oriented() ? "<font color='#dc2626'>not oriented</font>" : im.rms() == null ? "oriented" : String.format(Locale.ROOT, "%.2f px", im.rms());
    if (im.flag() != null) info += " <font color='#b45309'>flag: " + GuiOptions.escape(im.flag()) + "</font>";
    JLabel label = new JLabel("<html><body style='width:" + (THUMB - 10) + "px'>" + GuiOptions.escape(im.name()) + "<br><small>" + info + "</small></body></html>");
    JCheckBox box = new JCheckBox("on map", im.oriented());
    box.setEnabled(im.oriented());
    box.addActionListener(e -> map.rerender());
    onMap.put(im.id(), box);
    JPanel south = new JPanel(new BorderLayout());
    south.add(label, BorderLayout.CENTER);
    south.add(box, BorderLayout.SOUTH);
    c.add(thumb, BorderLayout.CENTER);
    c.add(south, BorderLayout.SOUTH);
    c.setBorder(new LineBorder(Color.LIGHT_GRAY, 2));
    MouseAdapter click =
        new MouseAdapter() {
          @Override
          public void mouseClicked(MouseEvent e) {
            select(im, e.getClickCount() == 2);
          }
        };
    thumb.addMouseListener(click);
    label.addMouseListener(click);
    thumb.setToolTipText("Click to select (highlighted on the map), double-click to open it large.");
    cards.put(im.id(), c);
    return c;
  }

  void select(RunImages.Image im, boolean open) {
    selected = im;
    for (var e : cards.entrySet()) e.getValue().setBorder(new LineBorder(e.getKey() == im.id() ? new Color(0xf97316) : Color.LIGHT_GRAY, e.getKey() == im.id() ? 3 : 2));
    JPanel card = cards.get(im.id());
    if (card != null) card.scrollRectToVisible(new Rectangle(card.getSize()));
    map.rerender();
    if (open || !images.mappable()) showFull(im);
  }

  /** The photograph at full resolution on the Image tab. */
  void showFull(RunImages.Image im) {
    views.setSelectedIndex(1);
    photoInfo.setText(
        "<html><b>" + GuiOptions.escape(im.name()) + "</b> - " + im.width() + " x " + im.height() + " px"
            + (im.oriented() ? "" : ", not oriented") + (im.rms() == null ? "" : String.format(Locale.ROOT, ", reprojection RMS %.2f px", im.rms()))
            + (im.flag() == null ? "" : ", camera screening: " + GuiOptions.escape(im.flag())) + "<br><small>" + GuiOptions.escape(im.path().toString()) + "</small></html>");
    BufferedImage quick = previews.get(im.id());
    photo.setImage(quick, im.width(), im.height());
    full.submit(
        () -> {
          try {
            BufferedImage b = RunImages.decode(im.path(), Integer.MAX_VALUE);
            SwingUtilities.invokeLater(() -> {
              if (selected == im) photo.setImage(b, im.width(), im.height());
            });
          } catch (IOException | RuntimeException e) {
            SwingUtilities.invokeLater(() -> photoInfo.setText(photoInfo.getText().replace("</html>", "<br><font color='#dc2626'>Cannot read: " + GuiOptions.escape(String.valueOf(e.getMessage())) + "</font></html>")));
          }
        });
  }

  /** Decodes a preview (from the cache when it is there) and repaints. */
  void load(RunImages.Image im) {
    if (!loading.add(im.id())) return;
    loader.submit(
        () -> {
          try {
            BufferedImage b = cached(im);
            previews.put(im.id(), b);
            SwingUtilities.invokeLater(
                () -> {
                  JPanel c = cards.get(im.id());
                  if (c != null) c.repaint();
                  if (im.oriented() && onMap.get(im.id()).isSelected()) map.rerenderSoon();
                });
          } catch (IOException | RuntimeException e) {
            // A missing or unreadable photograph stays "loading..."; its card says where it should be.
          }
        });
  }

  BufferedImage cached(RunImages.Image im) throws IOException {
    Path file = im.path();
    String key;
    try {
      key = GuiOptions.Settings.hash(file.toAbsolutePath() + "|" + Files.size(file) + "|" + Files.getLastModifiedTime(file).toMillis()) + "-" + PREVIEW + ".jpg";
    } catch (IOException e) {
      throw e;
    }
    Path c = cache.resolve(key);
    if (Files.isRegularFile(c)) {
      BufferedImage b = javax.imageio.ImageIO.read(c.toFile());
      if (b != null) {
        GuiPreferences.touch(c);
        return b;
      }
    }
    BufferedImage b = RunImages.decode(file, PREVIEW);
    try {
      Files.createDirectories(cache);
      Path tmp = Files.createTempFile(cache, "preview", ".tmp");
      javax.imageio.ImageIO.write(b, "jpg", tmp.toFile());
      Files.move(tmp, c, StandardCopyOption.REPLACE_EXISTING);
    } catch (IOException e) {
      // The cache is a convenience.
    }
    return b;
  }

  /** A picture with zoom (mouse wheel), pan (drag) and fit (double-click), in source pixels. */
  static final class ZoomView extends JComponent {
    BufferedImage image;
    int sourceWidth, sourceHeight;
    double cx, cy, scale = 1;
    Point last;

    ZoomView() {
      MouseAdapter m =
          new MouseAdapter() {
            @Override
            public void mousePressed(MouseEvent e) {
              last = e.getPoint();
            }

            @Override
            public void mouseDragged(MouseEvent e) {
              if (last == null) return;
              cx -= (e.getX() - last.x) / scale;
              cy -= (e.getY() - last.y) / scale;
              last = e.getPoint();
              repaint();
            }

            @Override
            public void mouseClicked(MouseEvent e) {
              if (e.getClickCount() == 2) fit();
            }

            @Override
            public void mouseWheelMoved(MouseWheelEvent e) {
              double next = Math.max(.005, Math.min(32, scale * Math.pow(1.25, -e.getPreciseWheelRotation())));
              double u = cx + (e.getX() - getWidth() / 2.) / scale, v = cy + (e.getY() - getHeight() / 2.) / scale;
              cx = u - (e.getX() - getWidth() / 2.) / next;
              cy = v - (e.getY() - getHeight() / 2.) / next;
              scale = next;
              repaint();
            }
          };
      addMouseListener(m);
      addMouseMotionListener(m);
      addMouseWheelListener(m);
    }

    /** Shows a picture of a source of the given size (a preview is stretched to it). */
    void setImage(BufferedImage b, int width, int height) {
      boolean first = sourceWidth != width || sourceHeight != height || image == null;
      image = b;
      sourceWidth = width;
      sourceHeight = height;
      if (first) fit();
      repaint();
    }

    void fit() {
      if (sourceWidth == 0) return;
      cx = sourceWidth / 2.;
      cy = sourceHeight / 2.;
      int w = getWidth() > 0 ? getWidth() : 800, h = getHeight() > 0 ? getHeight() : 600;
      scale = Math.min(w / (double) sourceWidth, h / (double) sourceHeight);
      repaint();
    }

    @Override
    protected void paintComponent(Graphics g0) {
      Graphics2D g = (Graphics2D) g0.create();
      g.setColor(new Color(0x222222));
      g.fillRect(0, 0, getWidth(), getHeight());
      if (image != null) {
        double f = image.getWidth() / (double) sourceWidth;
        g.setRenderingHint(RenderingHints.KEY_INTERPOLATION, scale * (1 / f) >= 2 ? RenderingHints.VALUE_INTERPOLATION_NEAREST_NEIGHBOR : RenderingHints.VALUE_INTERPOLATION_BILINEAR);
        AffineTransform t = new AffineTransform();
        t.translate(getWidth() / 2., getHeight() / 2.);
        t.scale(scale, scale);
        t.translate(-cx, -cy);
        t.scale(1 / f, 1 / f);
        g.drawImage(image, t, null);
      }
      g.dispose();
    }
  }

  /**
   * The map: world coordinates with north up; the visible area is rendered in the background at
   * screen resolution after each change, the previous rendering is shown moved and scaled until
   * then.
   */
  final class MapPanel extends JComponent {
    double cx, cy, unitsPerPixel = 1;
    BufferedImage rendered;
    RunImages.MapView renderedView;
    final AtomicLong generation = new AtomicLong();
    final javax.swing.Timer soon = new javax.swing.Timer(300, e -> rerender());
    Point last, pressed;

    MapPanel() {
      soon.setRepeats(false);
      setFocusable(true);
      MouseAdapter m =
          new MouseAdapter() {
            @Override
            public void mousePressed(MouseEvent e) {
              last = pressed = e.getPoint();
            }

            @Override
            public void mouseDragged(MouseEvent e) {
              if (last == null) return;
              cx -= (e.getX() - last.x) * unitsPerPixel;
              cy += (e.getY() - last.y) * unitsPerPixel;
              last = e.getPoint();
              repaint();
              rerenderSoon();
            }

            @Override
            public void mouseReleased(MouseEvent e) {
              if (pressed != null && pressed.distance(e.getPoint()) < 4) pick(e.getPoint(), e.getClickCount() == 2);
              last = pressed = null;
            }

            @Override
            public void mouseMoved(MouseEvent e) {
              double x = cx + (e.getX() - getWidth() / 2.) * unitsPerPixel, y = cy - (e.getY() - getHeight() / 2.) * unitsPerPixel;
              mapInfo.setText(
                  String.format(Locale.ROOT, "x %.2f  y %.2f  z %.2f%s  |  %.3f units/pixel  |  projected onto %s", x, y, images.surface.z(x, y),
                      images.crs == null ? "" : " (" + images.crs + ")", unitsPerPixel, images.surface.description()));
            }

            @Override
            public void mouseWheelMoved(MouseWheelEvent e) {
              double next = unitsPerPixel * Math.pow(1.25, e.getPreciseWheelRotation());
              double x = cx + (e.getX() - getWidth() / 2.) * unitsPerPixel, y = cy - (e.getY() - getHeight() / 2.) * unitsPerPixel;
              cx = x - (e.getX() - getWidth() / 2.) * next;
              cy = y + (e.getY() - getHeight() / 2.) * next;
              unitsPerPixel = next;
              repaint();
              rerenderSoon();
            }
          };
      addMouseListener(m);
      addMouseMotionListener(m);
      addMouseWheelListener(m);
      addComponentListener(
          new ComponentAdapter() {
            @Override
            public void componentResized(ComponentEvent e) {
              rerenderSoon();
            }
          });
      setToolTipText("Click a photograph to select it, double-click to open it. Wheel zooms, drag pans.");
    }

    void fit() {
      List<RunImages.Image> all = new ArrayList<>();
      for (RunImages.Image im : images.images) if (im.oriented()) all.add(im);
      double[] e = images.extent(all);
      if (e == null || getWidth() == 0) return;
      cx = (e[0] + e[2]) / 2;
      cy = (e[1] + e[3]) / 2;
      unitsPerPixel = Math.max((e[2] - e[0]) / getWidth(), (e[3] - e[1]) / getHeight()) * 1.05;
      rerender();
    }

    void rerenderSoon() {
      soon.restart();
    }

    void rerender() {
      if (!images.mappable() || getWidth() == 0) {
        repaint();
        return;
      }
      double device = getGraphicsConfiguration() == null ? 1 : getGraphicsConfiguration().getDefaultTransform().getScaleX();
      int w = (int) Math.ceil(getWidth() * device), h = (int) Math.ceil(getHeight() * device);
      double upp = unitsPerPixel / device;
      RunImages.MapView view = new RunImages.MapView(cx - getWidth() / 2. * unitsPerPixel, cy + getHeight() / 2. * unitsPerPixel, upp, w, h);
      List<RunImages.Image> order = mapOrder();
      float alpha = opacity.getValue() / 100f;
      long gen = generation.incrementAndGet();
      renderer.submit(
          () -> {
            if (generation.get() != gen) return;
            BufferedImage b = images.render(order, previews, view, alpha);
            SwingUtilities.invokeLater(
                () -> {
                  if (generation.get() != gen) return;
                  rendered = b;
                  renderedView = view;
                  repaint();
                });
          });
    }

    double sx(double x) {
      return (x - cx) / unitsPerPixel + getWidth() / 2.;
    }

    double sy(double y) {
      return (cy - y) / unitsPerPixel + getHeight() / 2.;
    }

    /** Selects the topmost shown photograph that sees the clicked point. */
    void pick(Point p, boolean open) {
      double x = cx + (p.x - getWidth() / 2.) * unitsPerPixel, y = cy - (p.y - getHeight() / 2.) * unitsPerPixel, z = images.surface.z(x, y);
      List<RunImages.Image> order = mapOrder();
      for (int i = order.size() - 1; i >= 0; i--) {
        RunImages.Image im = order.get(i);
        double[] uv = im.view().project(x, y, z);
        if (uv != null && uv[0] >= 0 && uv[1] >= 0 && uv[0] < im.view().k().width() && uv[1] < im.view().k().height()) {
          select(im, open);
          return;
        }
      }
    }

    @Override
    protected void paintComponent(Graphics g0) {
      Graphics2D g = (Graphics2D) g0.create();
      g.setColor(new Color(0x2b2b2b));
      g.fillRect(0, 0, getWidth(), getHeight());
      if (rendered != null && renderedView != null) {
        // The last rendering, placed by its world corner and scale.
        double x0 = sx(renderedView.left()), y0 = sy(renderedView.top());
        double w = renderedView.width() * renderedView.unitsPerPixel() / unitsPerPixel, h = renderedView.height() * renderedView.unitsPerPixel() / unitsPerPixel;
        g.setRenderingHint(RenderingHints.KEY_INTERPOLATION, RenderingHints.VALUE_INTERPOLATION_BILINEAR);
        g.drawImage(rendered, (int) Math.round(x0), (int) Math.round(y0), (int) Math.round(w), (int) Math.round(h), null);
      }
      g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
      if (outlines.isSelected())
        for (RunImages.Image im : mapOrder()) {
          if (im == selected) continue;
          outline(g, im, new Color(255, 255, 255, 110), 1f);
        }
      if (selected != null && selected.oriented()) outline(g, selected, new Color(0xf97316), 2.5f);
      g.dispose();
    }

    void outline(Graphics2D g, RunImages.Image im, Color c, float width) {
      double[][] f = images.footprint(im, 8);
      if (f == null) return;
      Path2D path = new Path2D.Double();
      for (int i = 0; i < f.length; i++)
        if (i == 0) path.moveTo(sx(f[i][0]), sy(f[i][1]));
        else path.lineTo(sx(f[i][0]), sy(f[i][1]));
      path.closePath();
      g.setColor(c);
      g.setStroke(new BasicStroke(width));
      g.draw(path);
    }
  }
}
