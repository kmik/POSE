package sgm;

import java.awt.*;
import java.awt.event.*;
import java.awt.geom.AffineTransform;
import java.awt.image.BufferedImage;
import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicLong;
import java.util.function.*;
import javax.swing.*;

/**
 * The 3D canvas (used by the 3D view window and the tie-point cleanup window): a CloudData drawn
 * by CloudRenderer at the screen's device resolution, camera frustums and a colour legend on top.
 * Left drag orbits, right drag (or Shift) pans, the wheel zooms towards the pointer, double-click
 * centres on a point. While moving, every k-th dense point is drawn; the full picture follows
 * when the view rests. Rendering runs on its own thread; the newest request wins.
 */
final class CloudView extends JComponent {
  static final int PREVIEW_POINTS = 350_000;

  /** How to draw: colour mode and attribute, point size, shading, layers, colour range and legend title. */
  record Look(CloudRenderer.Colour colour, int attribute, int pointSize, boolean edl, boolean dense, boolean sparse, boolean cameras,
              float lo, float hi, String legend) {}

  final ExecutorService renderer = Executors.newSingleThreadExecutor(GcpEditorWindow.daemon("pose-cloud-render"));
  final AtomicLong generation = new AtomicLong();
  final javax.swing.Timer settle;
  volatile CloudData data;
  volatile CloudRenderer.View view;
  volatile CloudRenderer.Frame frameShown;
  volatile BufferedImage image;
  Supplier<Look> look = () -> new Look(CloudRenderer.Colour.RGB, -1, 1, true, true, true, true, 0, 1, "");
  /** Hover text for the status line; null when nothing is under the pointer. */
  Consumer<String> status = s -> {};
  /** Hover text of a tie point (index), or null for the default. */
  IntFunction<String> sparseText;
  double cameraSize = 5;
  boolean moving;
  Point press;
  CloudRenderer.View pressView;
  boolean panning;

  CloudView() {
    setOpaque(true);
    settle = new javax.swing.Timer(180, e -> {
      moving = false;
      redraw();
    });
    settle.setRepeats(false);
    MouseAdapter m = new MouseAdapter() {
      @Override
      public void mousePressed(MouseEvent e) {
        press = e.getPoint();
        pressView = view;
        panning = SwingUtilities.isRightMouseButton(e) || SwingUtilities.isMiddleMouseButton(e) || e.isShiftDown();
      }

      @Override
      public void mouseDragged(MouseEvent e) {
        if (press == null || pressView == null) return;
        int dx = e.getX() - press.x, dy = e.getY() - press.y;
        if (panning) setView(pressView.pan(dx, dy, getHeight()), true);
        else setView(pressView.orbit(dx * .4, dy * .3), true);
      }

      @Override
      public void mouseReleased(MouseEvent e) {
        press = null;
      }

      @Override
      public void mouseWheelMoved(MouseWheelEvent e) {
        if (view == null) return;
        double factor = Math.pow(1.15, e.getPreciseWheelRotation());
        // Towards the pointer: the target moves by the zoom's share of the pointer's offset.
        CloudRenderer.View v = view.pan(-(e.getX() - getWidth() / 2.0) * (1 - factor), -(e.getY() - getHeight() / 2.0) * (1 - factor), getHeight());
        setView(v.zoom(factor), true);
      }

      @Override
      public void mouseClicked(MouseEvent e) {
        if (e.getClickCount() != 2 || view == null) return;
        double[] p = pointAt(e.getX(), e.getY());
        if (p != null) setView(view.at(p), false);
      }

      @Override
      public void mouseMoved(MouseEvent e) {
        hover(e.getX(), e.getY());
      }
    };
    addMouseListener(m);
    addMouseMotionListener(m);
    addMouseWheelListener(m);
    addComponentListener(new ComponentAdapter() {
      @Override
      public void componentResized(ComponentEvent e) {
        redraw();
      }
    });
  }

  /** Shows new data; the home view when asked (or when there is no view yet). */
  void setData(CloudData d, boolean home) {
    data = d;
    cameraSize = CloudRenderer.cameraSize(d);
    if (home || view == null) view = CloudRenderer.home(d);
    redraw();
  }

  void setView(CloudRenderer.View v, boolean interactive) {
    if (v == null) return;
    view = v;
    if (interactive) {
      moving = true;
      settle.restart();
    }
    redraw();
  }

  void home() {
    if (data != null) setView(CloudRenderer.home(data), false);
  }

  double deviceScale() {
    return getGraphicsConfiguration() == null ? 1 : getGraphicsConfiguration().getDefaultTransform().getScaleX();
  }

  void redraw() {
    CloudData d = data;
    CloudRenderer.View v = view;
    if (d == null || v == null) return;
    double scale = deviceScale();
    int w = Math.max(1, (int) Math.round(getWidth() * scale)), h = Math.max(1, (int) Math.round(getHeight() * scale));
    Look l = look.get();
    int step = moving ? Math.max(1, d.n / PREVIEW_POINTS) : 1;
    int size = (int) Math.round(l.pointSize() * Math.max(1, scale));
    CloudRenderer.Request q = new CloudRenderer.Request(d, v, w, h, l.colour(), l.attribute(), size, l.edl(), l.dense(), l.sparse(), step, l.lo(), l.hi());
    long g = generation.incrementAndGet();
    renderer.submit(() -> {
      if (generation.get() != g) return;
      CloudRenderer.Frame f = CloudRenderer.render(q);
      BufferedImage im = new BufferedImage(f.width(), f.height(), BufferedImage.TYPE_INT_ARGB);
      im.setRGB(0, 0, f.width(), f.height(), f.argb(), 0, f.width());
      SwingUtilities.invokeLater(() -> {
        if (generation.get() < g) return;
        frameShown = f;
        image = im;
        repaint();
      });
    });
  }

  void dispose() {
    renderer.shutdownNow();
    settle.stop();
    data = null;
    frameShown = null;
    image = null;
  }

  /** Cloud coordinates of the point under (x, y) logical pixels, or null. */
  double[] pointAt(int x, int y) {
    CloudRenderer.Frame f = frameShown;
    CloudData d = data;
    if (f == null || d == null) return null;
    double s = deviceScale();
    int id = f.pick((int) Math.round(x * s), (int) Math.round(y * s), (int) Math.round(4 * s));
    if (id < 0) return null;
    if ((id & CloudRenderer.SPARSE) != 0) {
      int i = id & ~CloudRenderer.SPARSE;
      return new double[] {d.sx[i], d.sy[i], d.sz[i]};
    }
    if ((id & CloudRenderer.LIDAR) != 0) {
      int i = id & ~CloudRenderer.LIDAR;
      return new double[] {d.lx[i], d.ly[i], d.lz[i]};
    }
    return new double[] {d.x[id], d.y[id], d.z[id]};
  }

  void hover(int x, int y) {
    CloudData d = data;
    CloudRenderer.Frame f = frameShown;
    if (d == null || f == null) return;
    if (look.get().cameras()) {
      CloudRenderer.Projector pr = new CloudRenderer.Projector(f.request().view(), getWidth(), getHeight());
      double[] s = new double[3];
      double best = 8;
      String text = null;
      for (CloudData.Cam c : d.cameras)
        if (pr.project(c.center()[0], c.center()[1], c.center()[2], s)) {
          double dist = Math.hypot(s[0] - x, s[1] - y);
          if (dist < best) {
            best = dist;
            text = "Camera " + c.name() + (c.flag() != null ? " - flagged: " + c.flag() : "")
                + (c.rms() != null ? String.format(Locale.ROOT, " - reprojection RMS %.2f px", c.rms()) : "");
          }
        }
      if (text != null) {
        status.accept(text);
        return;
      }
    }
    double sc = deviceScale();
    int id = f.pick((int) Math.round(x * sc), (int) Math.round(y * sc), (int) Math.round(3 * sc));
    if (id < 0) {
      status.accept(null);
      return;
    }
    if ((id & CloudRenderer.SPARSE) != 0) {
      int i = id & ~CloudRenderer.SPARSE;
      String own = sparseText == null ? null : sparseText.apply(i);
      status.accept(own != null ? own
          : String.format(Locale.ROOT, "Tie point %.3f, %.3f, %.3f - seen in %d images", d.sx[i] + d.origin[0], d.sy[i] + d.origin[1], d.sz[i] + d.origin[2], d.sobs[i]));
      return;
    }
    if ((id & CloudRenderer.LIDAR) != 0) {
      int i = id & ~CloudRenderer.LIDAR;
      status.accept(String.format(Locale.ROOT, "Lidar reference point %.3f, %.3f, %.3f", d.lx[i] + d.origin[0], d.ly[i] + d.origin[1], d.lz[i] + d.origin[2]));
      return;
    }
    StringBuilder s = new StringBuilder(String.format(Locale.ROOT, "Point %.3f, %.3f, %.3f - class %d", d.x[id] + d.origin[0], d.y[id] + d.origin[1], d.z[id] + d.origin[2], d.cls[id] & 255));
    for (CloudData.Attribute a : d.attributes)
      if (!Float.isNaN(a.values()[id])) s.append(String.format(Locale.ROOT, " - %s %.3g", a.label().toLowerCase(Locale.ROOT), a.values()[id]));
    status.accept(s.toString());
  }

  @Override
  protected void paintComponent(Graphics g0) {
    Graphics2D g = (Graphics2D) g0.create();
    g.setColor(new Color(0x20, 0x20, 0x26));
    g.fillRect(0, 0, getWidth(), getHeight());
    CloudRenderer.Frame f = frameShown;
    BufferedImage im = image;
    if (im != null) {
      // Drawn at device pixels (the frame was rendered at the device scale).
      AffineTransform t = g.getTransform();
      g.setTransform(new AffineTransform(1, 0, 0, 1, t.getTranslateX(), t.getTranslateY()));
      g.drawImage(im, 0, 0, null);
      g.setTransform(t);
    } else {
      g.setColor(Color.LIGHT_GRAY);
      g.drawString("Loading...", 20, 30);
    }
    CloudData d = data;
    Look l = look.get();
    if (f != null && d != null && l.cameras()) cameras(g, f, d);
    if (f != null && d != null) legend(g, f, d, l);
    g.dispose();
  }

  void cameras(Graphics2D g, CloudRenderer.Frame f, CloudData d) {
    g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
    CloudRenderer.Projector pr = new CloudRenderer.Projector(f.request().view(), getWidth(), getHeight());
    double[] s = new double[3];
    for (CloudData.Cam c : d.cameras) {
      double[][] fr = CloudRenderer.frustum(c, cameraSize);
      int[] px = new int[5], py = new int[5];
      boolean ok = true;
      for (int i = 0; i < 5 && ok; i++) {
        ok = pr.project(fr[i][0], fr[i][1], fr[i][2], s);
        px[i] = (int) Math.round(s[0]);
        py[i] = (int) Math.round(s[1]);
      }
      if (!ok) continue;
      g.setColor(c.flag() != null ? new Color(0xff4d4d) : new Color(0xf5f5f5));
      g.setStroke(new BasicStroke(c.flag() != null ? 1.6f : 1f));
      for (int i = 1; i <= 4; i++) {
        g.drawLine(px[0], py[0], px[i], py[i]);
        g.drawLine(px[i], py[i], px[i % 4 + 1], py[i % 4 + 1]);
      }
    }
  }

  void legend(Graphics2D g, CloudRenderer.Frame f, CloudData d, Look l) {
    CloudRenderer.Colour m = f.request().colour();
    g.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
    int x = 12, y = getHeight() - 16;
    FontMetrics fm = g.getFontMetrics();
    if ((m == CloudRenderer.Colour.HEIGHT || m == CloudRenderer.Colour.ATTRIBUTE) && f.request().dense() && d.n > 0) {
      int w = 200;
      for (int i = 0; i < w; i++) {
        g.setColor(new Color(CloudRenderer.ramp(i / (float) (w - 1))));
        g.drawLine(x + i, y - 12, x + i, y - 2);
      }
      g.setColor(Color.WHITE);
      double off = m == CloudRenderer.Colour.HEIGHT ? d.origin[2] : 0;
      String a = m == CloudRenderer.Colour.HEIGHT ? String.format(Locale.ROOT, "%.1f", l.lo() + off) : String.format(Locale.ROOT, "%.3g", l.lo());
      String b = m == CloudRenderer.Colour.HEIGHT ? String.format(Locale.ROOT, "%.1f", l.hi() + off) : String.format(Locale.ROOT, "%.3g", l.hi());
      g.drawString(a, x, y - 16);
      g.drawString(b, x + w - fm.stringWidth(b), y - 16);
      g.drawString(l.legend(), x + w + 10, y - 2);
    } else if (m == CloudRenderer.Colour.CLASS && f.request().dense() && d.n > 0) {
      int[] classes = {1, 2, 3, 4, 5, 6, 7};
      String[] names = {"unclassified", "ground", "low veg.", "medium veg.", "high veg.", "building", "noise"};
      for (int i = 0; i < classes.length; i++) {
        g.setColor(new Color(CloudRenderer.classColour(classes[i])));
        g.fillRect(x, y - 10, 10, 10);
        g.setColor(Color.WHITE);
        g.drawString(names[i], x + 14, y);
        x += 24 + fm.stringWidth(names[i]);
      }
    } else if (!l.legend().isEmpty() && !f.request().dense()) {
      g.setColor(Color.WHITE);
      g.drawString(l.legend(), x, y);
    }
  }
}
