package sgm;

import java.awt.*;
import java.awt.event.*;
import java.awt.image.BufferedImage;
import java.util.*;
import java.util.List;
import java.util.concurrent.*;
import javax.swing.*;
import javax.swing.border.*;

/**
 * Native (Swing) window of the GCP marker editor over {@link GcpEditor.Editor}: the GCP list with
 * pins and residuals on the left, the image crops of the selected GCP in the middle (probably
 * hidden images in their own section), and a zoomable view of one image on the right where a
 * click pins the marker. Every change is saved by the editor at once. All state comes from
 * {@link GcpEditorModel}; this class only draws and forwards clicks.
 */
final class GcpEditorWindow {
  static final Color PINNED = new Color(0x16a34a), REFINED = new Color(0x2563eb), PREDICTED = new Color(0xef4444), PREDICTION = new Color(0xfacc15);
  static final Color GOOD = new Color(0x16a34a), MID = new Color(0xca8a04), BAD = new Color(0xdc2626);

  final GcpEditor.Session session;
  final GcpEditor.Editor editor;
  final JFrame frame;
  final CountDownLatch closed = new CountDownLatch(1);
  // Marker changes run one at a time, in order, off the event thread.
  final ExecutorService actions = Executors.newSingleThreadExecutor(daemon("gcp-editor-actions"));
  // Photograph decoding for crops and the view.
  final ExecutorService loader = Executors.newFixedThreadPool(2, daemon("gcp-editor-images"));
  final Map<String, BufferedImage> thumbnails = new ConcurrentHashMap<>();
  final Set<String> loading = ConcurrentHashMap.newKeySet();
  GcpEditorModel model;
  String gcp;
  volatile boolean proceed;

  final DefaultListModel<GcpEditorModel.Gcp> gcpItems = new DefaultListModel<>();
  final JList<GcpEditorModel.Gcp> gcpList = new JList<>(gcpItems);
  final JLabel ready = new JLabel(), busy = new JLabel(" "), title = new JLabel();
  final JCheckBox continueBox = new JCheckBox("Continue the run after Done (gcp step)");
  final JButton acceptAll = new JButton();
  final JPanel cards = new JPanel(new GuiSupport.WrapLayout(8)), hiddenCards = new JPanel(new GuiSupport.WrapLayout(8)), others = new JPanel(new GuiSupport.WrapLayout(4));
  final JToggleButton hiddenToggle = new JToggleButton();
  final JLabel othersTitle = new JLabel("Other images (no prediction): click to place the marker by hand");
  final Viewer viewer = new Viewer();
  final JLabel viewerTitle = new JLabel(" "), viewerInfo = new JLabel(" ");
  final javax.swing.Timer poll = new javax.swing.Timer(1000, e -> pollRefining());

  static ThreadFactory daemon(String name) {
    return r -> {
      Thread t = new Thread(r, name);
      t.setDaemon(true);
      return t;
    };
  }

  /** Opens the window and blocks until it is closed; true when the run should continue. */
  static boolean showAndWait(GcpEditor.Session session, boolean proceed) {
    GuiSupport.lookAndFeel();
    GcpEditorWindow[] window = new GcpEditorWindow[1];
    try {
      SwingUtilities.invokeAndWait(() -> window[0] = new GcpEditorWindow(session, proceed, null));
      window[0].closed.await();
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    } catch (java.lang.reflect.InvocationTargetException e) {
      throw new IllegalStateException("Cannot open the GCP editor window: " + e.getCause(), e.getCause());
    }
    return window[0] != null && window[0].proceed;
  }

  /** Builds and shows the window (event thread). {@code onClose} runs after Done or closing. */
  GcpEditorWindow(GcpEditor.Session session, boolean proceed, Runnable onClose) {
    this.session = session;
    this.editor = session.editor();
    frame = new JFrame("GCP markers - run " + session.id());
    frame.setDefaultCloseOperation(WindowConstants.DO_NOTHING_ON_CLOSE);
    GuiSupport.brand(frame);
    frame.addWindowListener(
        new WindowAdapter() {
          @Override
          public void windowClosing(WindowEvent e) {
            finish(false, onClose);
          }
        });
    continueBox.setSelected(proceed);
    continueBox.setToolTipText("When the markers are ready, Done runs 'photogrammetry gcp' on this run, which adjusts the block onto the GCPs.");

    // Left: GCP list, readiness, Done.
    gcpList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
    gcpList.setFocusable(false); // letters are editor keys, not list search
    gcpList.setCellRenderer(new GcpRenderer());
    gcpList.addListSelectionListener(
        e -> {
          if (e.getValueIsAdjusting() || gcpList.getSelectedValue() == null) return;
          String id = gcpList.getSelectedValue().id();
          if (!id.equals(gcp)) selectGcp(id);
        });
    JButton done = new JButton("Done");
    done.setFont(done.getFont().deriveFont(Font.BOLD));
    done.addActionListener(e -> finish(true, onClose));
    JLabel help =
        new JLabel(
            "<html><body style='width:200px'>Click a crop to open it on the right. Click in the image to pin the marker there;"
                + " drag to pan, mouse wheel to zoom. After a pin, the other images are refined by patch matching (blue): check and"
                + " accept them.<br><br>Keys: Left/Right previous/next image, Up/Down previous/next GCP, + / - zoom, F fit,"
                + " A or Enter accept, R reject, U unpin.<br><br>Residuals: <font color='#16a34a'>&lt; 1 px</font>,"
                + " <font color='#ca8a04'>&lt; 3 px</font>, <font color='#dc2626'>larger</font>.</body></html>");
    help.setForeground(UIManager.getColor("Label.disabledForeground") == null ? Color.GRAY : UIManager.getColor("Label.disabledForeground").darker());
    JPanel left = new JPanel(new BorderLayout(4, 4));
    left.setBorder(new EmptyBorder(8, 8, 8, 8));
    JLabel heading = new JLabel("Ground control points");
    heading.setFont(heading.getFont().deriveFont(Font.BOLD));
    left.add(heading, BorderLayout.NORTH);
    left.add(new JScrollPane(gcpList), BorderLayout.CENTER);
    Box bottom = Box.createVerticalBox();
    for (JComponent c : List.of(ready, busy, continueBox, done, help)) {
      c.setAlignmentX(Component.LEFT_ALIGNMENT);
      bottom.add(c);
      bottom.add(Box.createVerticalStrut(4));
    }
    done.setMaximumSize(new Dimension(Integer.MAX_VALUE, done.getPreferredSize().height));
    left.add(bottom, BorderLayout.SOUTH);

    // Middle: crops of the selected GCP.
    title.setFont(title.getFont().deriveFont(Font.BOLD, title.getFont().getSize2D() * 1.3f));
    acceptAll.addActionListener(e -> act(() -> editor.acceptAll(gcp)));
    hiddenToggle.addActionListener(e -> hiddenCards.setVisible(hiddenToggle.isSelected()));
    hiddenCards.setVisible(false);
    Box middle = Box.createVerticalBox();
    JPanel head = new JPanel(new FlowLayout(FlowLayout.LEFT, 8, 4));
    head.add(title);
    head.add(acceptAll);
    for (JComponent c : List.of(head, cards, hiddenToggle, hiddenCards, othersTitle, others)) {
      c.setAlignmentX(Component.LEFT_ALIGNMENT);
      middle.add(c);
    }
    JPanel middleHolder = new JPanel(new BorderLayout());
    middleHolder.add(middle, BorderLayout.NORTH);
    JScrollPane middleScroll = new JScrollPane(middleHolder);
    middleScroll.getVerticalScrollBar().setUnitIncrement(24);
    middleScroll.addComponentListener(
        new ComponentAdapter() {
          @Override
          public void componentResized(ComponentEvent e) {
            middle.revalidate();
          }
        });

    // Right: the image view and its buttons.
    JPanel right = new JPanel(new BorderLayout(4, 4));
    right.setBorder(new EmptyBorder(6, 6, 6, 6));
    right.add(viewerTitle, BorderLayout.NORTH);
    right.add(viewer, BorderLayout.CENTER);
    JPanel buttons = new JPanel(new GuiSupport.WrapLayout(2));
    buttons.add(button("Zoom in", () -> viewer.zoom(2, null)));
    buttons.add(button("Zoom out", () -> viewer.zoom(.5, null)));
    buttons.add(button("Fit", viewer::fit));
    buttons.add(button("Accept", this::acceptCurrent));
    buttons.add(button("Reject", () -> markCurrent("rejected")));
    buttons.add(button("Unpin", () -> markCurrent("predicted")));
    buttons.add(button("Previous", () -> step(-1)));
    buttons.add(button("Next", () -> step(1)));
    JPanel south = new JPanel(new BorderLayout());
    south.add(buttons, BorderLayout.CENTER);
    south.add(viewerInfo, BorderLayout.SOUTH);
    right.add(south, BorderLayout.SOUTH);

    JSplitPane inner = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT, middleScroll, right);
    inner.setResizeWeight(.45);
    JSplitPane outer = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT, left, inner);
    outer.setResizeWeight(0);
    frame.setContentPane(outer);
    keys(frame.getRootPane());

    Rectangle screen = GraphicsEnvironment.getLocalGraphicsEnvironment().getMaximumWindowBounds();
    frame.setSize((int) (screen.width * .9), (int) (screen.height * .9));
    frame.setLocationRelativeTo(null);
    refresh(editor.state());
    outer.setDividerLocation(Math.max(260, left.getPreferredSize().width + 20));
    inner.setDividerLocation((int) ((frame.getWidth() - outer.getDividerLocation()) * .45));
    frame.setVisible(true);
    poll.start();
    SwingUtilities.invokeLater(
        () -> {
          if (gcp != null) selectGcp(gcp);
        });
  }

  JButton button(String text, Runnable action) {
    JButton b = new JButton(text);
    b.setFocusable(false);
    b.addActionListener(e -> action.run());
    return b;
  }

  void keys(JRootPane root) {
    InputMap in = root.getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW);
    ActionMap am = root.getActionMap();
    Object[][] bindings = {
      {"LEFT", (Runnable) () -> step(-1)}, {"RIGHT", (Runnable) () -> step(1)},
      {"UP", (Runnable) () -> stepGcp(-1)}, {"DOWN", (Runnable) () -> stepGcp(1)},
      {"PLUS", (Runnable) () -> viewer.zoom(2, null)}, {"ADD", (Runnable) () -> viewer.zoom(2, null)},
      {"shift EQUALS", (Runnable) () -> viewer.zoom(2, null)}, {"EQUALS", (Runnable) () -> viewer.zoom(2, null)},
      {"MINUS", (Runnable) () -> viewer.zoom(.5, null)}, {"SUBTRACT", (Runnable) () -> viewer.zoom(.5, null)},
      {"F", (Runnable) () -> viewer.fit()}, {"A", (Runnable) this::acceptCurrent}, {"ENTER", (Runnable) this::acceptCurrent},
      {"R", (Runnable) () -> markCurrent("rejected")}, {"U", (Runnable) () -> markCurrent("predicted")},
    };
    for (Object[] b : bindings) {
      String key = (String) b[0];
      Runnable run = (Runnable) b[1];
      in.put(KeyStroke.getKeyStroke(key), key);
      am.put(
          key,
          new AbstractAction() {
            @Override
            public void actionPerformed(ActionEvent e) {
              run.run();
            }
          });
    }
  }

  void finish(boolean done, Runnable onClose) {
    if (!frame.isDisplayable()) return;
    if (!done && editor.refining()) {
      // Refinement results are saved when they arrive; closing now only drops pending ones.
      int answer = JOptionPane.showConfirmDialog(frame, "Markers are still being refined. Close anyway?", "GCP markers", JOptionPane.YES_NO_OPTION);
      if (answer != JOptionPane.YES_OPTION) return;
    }
    proceed = done && continueBox.isSelected() && model != null && model.ready;
    poll.stop();
    frame.dispose();
    actions.shutdown();
    loader.shutdownNow();
    try {
      actions.awaitTermination(30, TimeUnit.SECONDS);
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    }
    if (onClose != null) onClose.run();
    closed.countDown();
  }

  /** Runs an editor action off the event thread, then shows the new state. */
  void act(Callable<Map<String, Object>> action) {
    actions.submit(
        () -> {
          try {
            Map<String, Object> state = action.call();
            SwingUtilities.invokeLater(() -> refresh(state));
          } catch (Exception e) {
            SwingUtilities.invokeLater(() -> GuiSupport.error(frame, "GCP markers", e));
          }
        });
  }

  void pollRefining() {
    if (model != null && (model.refining || editor.refining()))
      actions.submit(
          () -> {
            Map<String, Object> state = editor.state();
            SwingUtilities.invokeLater(() -> refresh(state));
          });
  }

  void refresh(Map<String, Object> state) {
    if (!frame.isDisplayable() && model != null) return;
    model = new GcpEditorModel(state);
    if (gcp == null || model.gcp(gcp) == null) gcp = model.gcps.isEmpty() ? null : model.gcps.get(0).id();
    gcpItems.clear();
    for (GcpEditorModel.Gcp g : model.gcps) gcpItems.addElement(g);
    for (int i = 0; i < gcpItems.size(); i++) if (gcpItems.get(i).id().equals(gcp)) gcpList.setSelectedIndex(i);
    ready.setText("<html><b>" + (model.ready ? "Ready for the gcp step." : "Still needed: " + model.needed) + "</b></html>");
    ready.setForeground(model.ready ? PINNED : UIManager.getColor("Label.foreground"));
    busy.setText(model.refining ? "Refining markers..." : " ");
    continueBox.setEnabled(model.ready);
    rebuildCards();
    viewer.update();
  }

  void selectGcp(String id) {
    gcp = id;
    viewer.show(-1);
    rebuildCards();
    List<GcpEditorModel.Marker> list = model.cards(gcp);
    if (!list.isEmpty()) viewer.show(list.get(0).image());
  }

  void stepGcp(int delta) {
    if (model == null || model.gcps.isEmpty()) return;
    int i = 0;
    for (int k = 0; k < model.gcps.size(); k++) if (model.gcps.get(k).id().equals(gcp)) i = k;
    i = Math.floorMod(i + delta, model.gcps.size());
    gcpList.setSelectedIndex(i);
  }

  void rebuildCards() {
    cards.removeAll();
    hiddenCards.removeAll();
    others.removeAll();
    if (gcp == null) {
      title.setText("No GCPs");
      acceptAll.setVisible(false);
      hiddenToggle.setVisible(false);
      othersTitle.setVisible(false);
      cards.revalidate();
      cards.repaint();
      return;
    }
    GcpEditorModel.Gcp g = model.gcp(gcp);
    title.setText(gcp + "  (" + g.type() + ", " + g.pins() + " pinned of " + g.markers() + ")");
    int good = model.acceptable(gcp);
    acceptAll.setText(String.format(Locale.ROOT, "Accept %d refined (score >= %.2f)", good, model.accept));
    acceptAll.setVisible(good > 0);
    for (GcpEditorModel.Marker k : model.shown(gcp)) cards.add(new Card(k));
    List<GcpEditorModel.Marker> hidden = model.hidden(gcp);
    for (GcpEditorModel.Marker k : hidden) hiddenCards.add(new Card(k));
    hiddenToggle.setVisible(!hidden.isEmpty());
    hiddenToggle.setText((hiddenToggle.isSelected() ? "Hide" : "Show") + " probably hidden images (" + hidden.size() + "): the view passes below the tie-point surface (canopy, roofs)");
    hiddenCards.setVisible(hiddenToggle.isSelected() && !hidden.isEmpty());
    List<GcpEditorModel.Image> rest = model.others(gcp);
    othersTitle.setVisible(!rest.isEmpty());
    for (GcpEditorModel.Image im : rest) {
      JButton b = new JButton(im.name());
      b.setFocusable(false);
      b.addActionListener(e -> viewer.show(im.id()));
      others.add(b);
    }
    for (JPanel p : List.of(cards, hiddenCards, others)) {
      p.revalidate();
      p.repaint();
    }
  }

  GcpEditorModel.Marker current() {
    return model == null || gcp == null || viewer.image < 0 ? null : model.marker(gcp, viewer.image);
  }

  void step(int delta) {
    if (model == null || gcp == null) return;
    List<GcpEditorModel.Marker> list = model.cards(gcp);
    if (list.isEmpty()) return;
    int i = -1;
    for (int k = 0; k < list.size(); k++) if (list.get(k).image() == viewer.image) i = k;
    int next = i < 0 ? (delta > 0 ? 0 : list.size() - 1) : Math.floorMod(i + delta, list.size());
    if (list.get(next).hidden() && !list.get(next).pinned() && !hiddenToggle.isSelected()) {
      hiddenToggle.setSelected(true);
      rebuildCards();
    }
    viewer.show(list.get(next).image());
  }

  void acceptCurrent() {
    GcpEditorModel.Marker k = current();
    if (k == null || !k.placed() || k.pinned()) return;
    String g = gcp;
    int image = k.image();
    act(() -> editor.mark(g, image, "pinned", null, null));
    step(1);
  }

  void markCurrent(String status) {
    if (current() == null && status.equals("predicted")) return;
    if (viewer.image < 0 || gcp == null) return;
    String g = gcp;
    int image = viewer.image;
    act(() -> editor.mark(g, image, status, null, null));
  }

  void pin(double u, double v) {
    if (viewer.image < 0 || gcp == null) return;
    String g = gcp;
    int image = viewer.image;
    act(() -> editor.mark(g, image, "pinned", u, v));
  }

  static Color statusColor(String status) {
    return switch (status) {
      case "pinned" -> PINNED;
      case "refined" -> REFINED;
      case "rejected" -> Color.GRAY;
      default -> PREDICTED;
    };
  }

  static Color gradeColor(Double residual) {
    String grade = GcpEditorModel.grade(residual);
    return grade == null ? null : grade.equals("good") ? GOOD : grade.equals("mid") ? MID : BAD;
  }

  static String hex(Color c) {
    return String.format(Locale.ROOT, "#%06x", c.getRGB() & 0xffffff);
  }

  /** Marker status with a coloured residual (HTML). */
  static String infoHtml(GcpEditorModel.Marker k) {
    String text = escape(GcpEditorModel.info(k));
    Color c = gradeColor(k.residual());
    if (c != null) {
      String r = String.format(Locale.ROOT, "%.2f px", k.residual());
      text = text.replace(r, "<font color='" + hex(c) + "'>" + r + "</font>");
    }
    return text;
  }

  static String escape(String s) {
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;");
  }

  final class GcpRenderer extends DefaultListCellRenderer {
    @Override
    public Component getListCellRendererComponent(JList<?> list, Object value, int index, boolean selected, boolean focus) {
      GcpEditorModel.Gcp g = (GcpEditorModel.Gcp) value;
      String summary = escape(GcpEditorModel.summary(g));
      Color c = gradeColor(g.pinRms());
      if (c != null && g.pinRms() != null && !selected) {
        String r = String.format(Locale.ROOT, "pin RMS %.2f px", g.pinRms());
        summary = summary.replace(r, "pin RMS <font color='" + hex(c) + "'>" + String.format(Locale.ROOT, "%.2f px", g.pinRms()) + "</font>");
      }
      String html =
          "<html><b>" + escape(g.id()) + "</b> <small>" + g.type() + "</small> &nbsp; " + g.pins() + " pinned / " + g.markers()
              + (summary.isEmpty() ? "" : "<br><small>" + summary + "</small>") + "</html>";
      JLabel label = (JLabel) super.getListCellRendererComponent(list, html, index, selected, false);
      label.setBorder(new EmptyBorder(4, 6, 4, 6));
      return label;
    }
  }

  /** One image crop of the selected GCP with its status. */
  final class Card extends JPanel {
    static final int SIZE = 150;
    final GcpEditorModel.Marker marker;
    final String key;

    Card(GcpEditorModel.Marker k) {
      super(new BorderLayout());
      marker = k;
      key = String.format(Locale.ROOT, "%d:%.1f:%.1f", k.image(), k.u(), k.v());
      boolean selected = k.image() == viewer.image;
      Color border = statusColor(k.status());
      setBorder(new CompoundBorder(new LineBorder(selected ? new Color(0xf97316) : border, selected ? 4 : 2), new EmptyBorder(selected ? 0 : 2, selected ? 0 : 2, selected ? 0 : 2, selected ? 0 : 2)));
      JComponent picture =
          new JComponent() {
            @Override
            protected void paintComponent(Graphics g0) {
              Graphics2D g = (Graphics2D) g0.create();
              g.setColor(Color.DARK_GRAY);
              g.fillRect(0, 0, getWidth(), getHeight());
              BufferedImage im = thumbnails.get(key);
              if (im == null) {
                g.setColor(Color.LIGHT_GRAY);
                g.drawString("loading...", 10, 20);
                load();
              } else {
                g.setRenderingHint(RenderingHints.KEY_INTERPOLATION, RenderingHints.VALUE_INTERPOLATION_BILINEAR);
                g.drawImage(im, 0, 0, getWidth(), getHeight(), null);
              }
              if (marker.status().equals("rejected") || marker.hidden() && !marker.pinned()) {
                g.setColor(new Color(255, 255, 255, marker.status().equals("rejected") ? 140 : 70));
                g.fillRect(0, 0, getWidth(), getHeight());
              }
              g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
              g.setStroke(new BasicStroke(2f));
              g.setColor(statusColor(marker.status()));
              int cx = getWidth() / 2, cy = getHeight() / 2;
              g.drawOval(cx - 7, cy - 7, 14, 14);
              g.dispose();
            }
          };
      picture.setPreferredSize(new Dimension(SIZE, SIZE));
      add(picture, BorderLayout.CENTER);
      JLabel caption = new JLabel("<html><body style='width:" + (SIZE - 10) + "px'>" + escape(model.images.get(k.image()).name()) + "<br>" + infoHtml(k) + "</body></html>");
      caption.setBorder(new EmptyBorder(2, 4, 2, 4));
      caption.setFont(caption.getFont().deriveFont(caption.getFont().getSize2D() * .9f));
      add(caption, BorderLayout.SOUTH);
      setToolTipText("Open " + model.images.get(k.image()).name());
      setCursor(Cursor.getPredefinedCursor(Cursor.HAND_CURSOR));
      MouseAdapter click =
          new MouseAdapter() {
            @Override
            public void mouseClicked(MouseEvent e) {
              viewer.show(marker.image());
            }
          };
      addMouseListener(click);
      picture.addMouseListener(click);
    }

    void load() {
      if (!loading.add(key)) return;
      double u = marker.u(), v = marker.v();
      int image = marker.image();
      loader.submit(
          () -> {
            try {
              // Twice the display size: sharp on HiDPI screens.
              thumbnails.put(key, editor.crop(image, u, v, 200, SIZE * 2));
            } catch (Exception e) {
              thumbnails.put(key, new BufferedImage(1, 1, BufferedImage.TYPE_INT_RGB));
            }
            SwingUtilities.invokeLater(Card.this::repaint);
          });
    }
  }

  /**
   * One photograph at any zoom: source pixel (cx, cy) at the centre of the component, {@code
   * scale} screen pixels per source pixel. A click (without dragging) pins the marker there.
   */
  final class Viewer extends JComponent {
    int image = -1;
    BufferedImage photo, preview;
    double cx, cy, scale = 1;
    Point press, last;
    boolean dragged;

    Viewer() {
      setFocusable(true);
      setPreferredSize(new Dimension(700, 700));
      setCursor(Cursor.getPredefinedCursor(Cursor.CROSSHAIR_CURSOR));
      MouseAdapter mouse =
          new MouseAdapter() {
            @Override
            public void mousePressed(MouseEvent e) {
              requestFocusInWindow();
              press = last = e.getPoint();
              dragged = false;
            }

            @Override
            public void mouseDragged(MouseEvent e) {
              if (last == null) return;
              if (press.distance(e.getPoint()) > 4) dragged = true;
              cx -= (e.getX() - last.x) / scale;
              cy -= (e.getY() - last.y) / scale;
              last = e.getPoint();
              repaint();
            }

            @Override
            public void mouseReleased(MouseEvent e) {
              if (!dragged && SwingUtilities.isLeftMouseButton(e) && photo != null && press != null) {
                double u = cx + (e.getX() - getWidth() / 2.) / scale, v = cy + (e.getY() - getHeight() / 2.) / scale;
                if (u >= 0 && v >= 0 && u < photo.getWidth() && v < photo.getHeight()) pin(u, v);
              }
              press = last = null;
            }

            @Override
            public void mouseWheelMoved(MouseWheelEvent e) {
              zoom(Math.pow(1.25, -e.getPreciseWheelRotation()), e.getPoint());
            }
          };
      addMouseListener(mouse);
      addMouseMotionListener(mouse);
      addMouseWheelListener(mouse);
    }

    /** Opens an image: centred on the marker with a 400 px window, or the whole photograph. */
    void show(int id) {
      image = id;
      photo = preview = null;
      if (id < 0) {
        update();
        return;
      }
      GcpEditorModel.Image im = model.images.get(id);
      GcpEditorModel.Marker k = current();
      int side = getWidth() > 0 && getHeight() > 0 ? Math.min(getWidth(), getHeight()) : 700;
      if (k != null && k.placed()) {
        cx = k.u();
        cy = k.v();
        scale = side / 400.;
      } else {
        cx = im.width() / 2.;
        cy = im.height() / 2.;
        scale = side / (double) Math.max(im.width(), im.height());
      }
      loader.submit(
          () -> {
            try {
              BufferedImage p = editor.photo(id);
              BufferedImage small = downsample(p, 2048);
              SwingUtilities.invokeLater(
                  () -> {
                    if (image == id) {
                      photo = p;
                      preview = small;
                      repaint();
                    }
                  });
            } catch (Exception e) {
              SwingUtilities.invokeLater(() -> GuiSupport.error(frame, "Cannot read the image", e));
            }
          });
      rebuildCards();
      update();
      requestFocusInWindow();
    }

    void fit() {
      if (image < 0) return;
      GcpEditorModel.Image im = model.images.get(image);
      cx = im.width() / 2.;
      cy = im.height() / 2.;
      scale = Math.min(getWidth() / (double) im.width(), getHeight() / (double) im.height());
      repaint();
      update();
    }

    /** Zooms by a factor about a screen point (the centre when null), keeping that spot fixed. */
    void zoom(double factor, Point at) {
      if (image < 0) return;
      double next = Math.max(.01, Math.min(32, scale * factor));
      if (at != null) {
        double u = cx + (at.x - getWidth() / 2.) / scale, v = cy + (at.y - getHeight() / 2.) / scale;
        cx = u - (at.x - getWidth() / 2.) / next;
        cy = v - (at.y - getHeight() / 2.) / next;
      }
      scale = next;
      repaint();
      update();
    }

    /** Title and status lines for the open image. */
    void update() {
      if (image < 0 || model == null) {
        viewerTitle.setText(" ");
        viewerInfo.setText("Select an image crop.");
        repaint();
        return;
      }
      GcpEditorModel.Marker k = current();
      viewerTitle.setText("<html><b>" + escape(gcp) + "</b> in " + escape(model.images.get(image).name()) + " - " + (k == null ? "no marker" : infoHtml(k)) + "</html>");
      viewerInfo.setText(String.format(Locale.ROOT, "zoom %.2fx; yellow ring = predicted position", scale));
      repaint();
    }

    @Override
    protected void paintComponent(Graphics g0) {
      Graphics2D g = (Graphics2D) g0.create();
      g.setColor(new Color(0x222222));
      g.fillRect(0, 0, getWidth(), getHeight());
      if (image < 0) {
        g.dispose();
        return;
      }
      if (photo == null) {
        g.setColor(Color.LIGHT_GRAY);
        g.drawString("Loading image...", 20, 30);
        g.dispose();
        return;
      }
      // Visible source rectangle, drawn with a source-rect blit (fast for large photographs).
      boolean small = scale < .5 && preview != null && preview != photo;
      BufferedImage src = small ? preview : photo;
      double f = src.getWidth() / (double) photo.getWidth();
      double u0 = cx - getWidth() / 2. / scale, v0 = cy - getHeight() / 2. / scale;
      double u1 = cx + getWidth() / 2. / scale, v1 = cy + getHeight() / 2. / scale;
      int su0 = (int) Math.floor(Math.max(0, u0) * f), sv0 = (int) Math.floor(Math.max(0, v0) * f);
      int su1 = (int) Math.ceil(Math.min(photo.getWidth(), u1) * f), sv1 = (int) Math.ceil(Math.min(photo.getHeight(), v1) * f);
      if (su1 > su0 && sv1 > sv0) {
        g.setRenderingHint(RenderingHints.KEY_INTERPOLATION, scale >= 1 ? RenderingHints.VALUE_INTERPOLATION_NEAREST_NEIGHBOR : RenderingHints.VALUE_INTERPOLATION_BILINEAR);
        int dx0 = (int) Math.round(toX(su0 / f)), dy0 = (int) Math.round(toY(sv0 / f)), dx1 = (int) Math.round(toX(su1 / f)), dy1 = (int) Math.round(toY(sv1 / f));
        g.drawImage(src, dx0, dy0, dx1, dy1, su0, sv0, su1, sv1, null);
      }
      g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
      GcpEditorModel.Marker k = current();
      if (k != null) {
        if (k.predictedU() != null && k.predictedV() != null) {
          g.setColor(PREDICTION);
          g.setStroke(new BasicStroke(1.5f));
          double x = toX(k.predictedU()), y = toY(k.predictedV());
          g.draw(new java.awt.geom.Ellipse2D.Double(x - 10, y - 10, 20, 20));
        }
        if (k.placed()) {
          g.setColor(statusColor(k.status()));
          g.setStroke(new BasicStroke(2f));
          double x = toX(k.u()), y = toY(k.v());
          g.draw(new java.awt.geom.Line2D.Double(x - 16, y, x - 4, y));
          g.draw(new java.awt.geom.Line2D.Double(x + 4, y, x + 16, y));
          g.draw(new java.awt.geom.Line2D.Double(x, y - 16, x, y - 4));
          g.draw(new java.awt.geom.Line2D.Double(x, y + 4, x, y + 16));
        }
      }
      g.dispose();
    }

    double toX(double u) {
      return (u - cx) * scale + getWidth() / 2.;
    }

    double toY(double v) {
      return (v - cy) * scale + getHeight() / 2.;
    }
  }

  static BufferedImage downsample(BufferedImage p, int longest) {
    int big = Math.max(p.getWidth(), p.getHeight());
    if (big <= longest) return p;
    double s = longest / (double) big;
    BufferedImage out = new BufferedImage(Math.max(1, (int) (p.getWidth() * s)), Math.max(1, (int) (p.getHeight() * s)), BufferedImage.TYPE_INT_RGB);
    Graphics2D g = out.createGraphics();
    g.setRenderingHint(RenderingHints.KEY_INTERPOLATION, RenderingHints.VALUE_INTERPOLATION_BILINEAR);
    g.drawImage(p, 0, 0, out.getWidth(), out.getHeight(), null);
    g.dispose();
    return out;
  }
}
