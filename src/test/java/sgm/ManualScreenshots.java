package sgm;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.nio.file.*;
import java.util.*;
import java.util.List;
import javax.imageio.ImageIO;
import javax.swing.*;

/**
 * Screenshots of the POSE windows for the user manual. Opens the windows on a finished demo
 * project, paints each one into a PNG at twice the screen scale and closes it. Run with its own
 * settings folder so no personal recent projects appear:
 *
 * <pre>java -Dpose.gui.settings=DIR -cp ... sgm.ManualScreenshots PROJECT IMAGES OUTDIR</pre>
 */
final class ManualScreenshots {
  static final double SCALE = 2;

  public static void main(String[] args) throws Exception {
    Path project = Path.of(args[0]).toAbsolutePath(), images = Path.of(args[1]).toAbsolutePath(), out = Path.of(args[2]);
    Files.createDirectories(out);
    GuiSupport.lookAndFeel();
    PoseWindow[] holder = new PoseWindow[1];
    SwingUtilities.invokeAndWait(() -> holder[0] = new PoseWindow(project, () -> {}));
    PoseWindow w = holder[0];
    place(w.frame, 1500, 1000);
    pause(1500);

    // New run: image folder and the drone + forest preset.
    SwingUtilities.invokeAndWait(() -> {
      Map<String, String> v = w.launcher.values();
      v.put("-i", images.toString());
      w.launcher.load(v);
      for (int i = 0; i < w.launcher.preset.getItemCount(); i++) {
        String item = w.launcher.preset.getItemAt(i);
        if (item.contains("drone") && item.contains("forest")) { w.launcher.preset.setSelectedIndex(i); break; }
      }
      w.tabs.setSelectedIndex(0);
      JSplitPane split = find(w.frame.getContentPane(), JSplitPane.class);
      if (split != null) split.setDividerLocation(0.86);
    });
    pause(1500);
    shoot(w.frame, out.resolve("gui-new-run.png"));

    SwingUtilities.invokeAndWait(() -> w.tabs.setSelectedIndex(1));
    pause(800);
    shoot(w.frame, out.resolve("gui-advanced.png"));

    SwingUtilities.invokeAndWait(() -> {
      JSplitPane split = find(w.frame.getContentPane(), JSplitPane.class);
      if (split != null) split.setDividerLocation(0.62);
      w.tabs.setSelectedIndex(2);
      w.manager.refresh();
      if (w.manager.runTable.getRowCount() > 0) w.manager.runTable.setRowSelectionInterval(0, 0);
    });
    pause(2500);
    shoot(w.frame, out.resolve("gui-project.png"));

    ProjectFiles.Run run = w.manager.selectedRun();
    if (run == null) throw new IllegalStateException("no run in " + project);

    window(out.resolve("gui-3d-view.png"), 1200, 760, 9000, () -> CloudWindow.open(w.frame, run), null, x -> {
      CloudView v = find(x, CloudView.class);
      if (v != null && v.view != null) v.setView(v.view.withAngles(25, 38), false);
    });
    window(out.resolve("gui-images-map.png"), 1200, 800, 7000, () -> ImageWindow.open(w.frame, run), "Map", null);
    window(out.resolve("gui-tie-points.png"), 1200, 860, 6000, () -> TiePointWindow.open(w.frame, run, c -> {}), null, null);
    window(out.resolve("gui-registration.png"), 1200, 760, 4000, () -> RegistrationWindow.open(w.frame, run, () -> {}, () -> {}), null, null);

    SwingUtilities.invokeAndWait(() -> { for (Window x : Window.getWindows()) x.dispose(); });
    System.out.println("Screenshots in " + out);
    System.exit(0);
  }

  /** Opens a window, waits for it to render, optionally selects a tab, then captures it. */
  static void window(Path file, int width, int height, long wait, Runnable open, String tab, java.util.function.Consumer<Window> adjust) throws Exception {
    Set<Window> before = new HashSet<>(Arrays.asList(Window.getWindows()));
    try {
      SwingUtilities.invokeAndWait(open);
    } catch (Exception e) {
      System.out.println("skipped " + file.getFileName() + ": " + e);
      return;
    }
    pause(800);
    Window opened = null;
    for (Window x : Window.getWindows()) if (!before.contains(x) && x.isShowing()) opened = x;
    if (opened == null) {
      System.out.println("skipped " + file.getFileName() + ": no window opened");
      return;
    }
    Window target = opened;
    SwingUtilities.invokeAndWait(() -> {
      place(target, width, height);
      if (tab != null) {
        JTabbedPane tabs = find(target, JTabbedPane.class);
        if (tabs != null) for (int i = 0; i < tabs.getTabCount(); i++) if (tabs.getTitleAt(i).contains(tab)) tabs.setSelectedIndex(i);
      }
      if (adjust != null) adjust.accept(target);
    });
    pause(wait);
    shoot(target, file);
    SwingUtilities.invokeAndWait(target::dispose);
  }

  static void place(Window w, int width, int height) {
    if (w instanceof Frame f) f.setExtendedState(Frame.NORMAL);
    w.setSize(width, height);
    w.setLocation(0, 0);
    w.validate();
  }

  static <T> T find(Container c, Class<T> type) {
    for (Component child : c.getComponents()) {
      if (type.isInstance(child)) return type.cast(child);
      if (child instanceof Container inner) {
        T found = find(inner, type);
        if (found != null) return found;
      }
    }
    return null;
  }

  static void shoot(Window w, Path file) throws Exception {
    JRootPane root = w instanceof RootPaneContainer r ? r.getRootPane() : null;
    shoot(w, root != null ? root : w, file);
  }

  static void shoot(Window w, Component c, Path file) throws Exception {
    BufferedImage[] image = new BufferedImage[1];
    SwingUtilities.invokeAndWait(() -> {
      int width = c.getWidth(), height = c.getHeight();
      BufferedImage b = new BufferedImage((int) (width * SCALE), (int) (height * SCALE), BufferedImage.TYPE_INT_RGB);
      Graphics2D g = b.createGraphics();
      g.scale(SCALE, SCALE);
      c.printAll(g);
      g.dispose();
      image[0] = b;
    });
    ImageIO.write(image[0], "png", file.toFile());
    System.out.println("wrote " + file);
  }

  static void pause(long ms) throws InterruptedException {
    Thread.sleep(ms);
  }

  static List<Window> windows() {
    return Arrays.asList(Window.getWindows());
  }
}
