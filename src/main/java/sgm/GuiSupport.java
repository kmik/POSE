package sgm;

import java.awt.*;
import java.io.IOException;
import java.nio.file.*;
import java.util.*;
import java.util.List;
import javax.swing.*;

/** Small helpers shared by the POSE windows: look and feel, opening files, a wrapping layout. */
final class GuiSupport {
  private GuiSupport() {}

  /** Message for machines without a display. */
  static final String HEADLESS =
      "No display is available, so the POSE window cannot open. Use the command line instead:"
          + " 'photogrammetry --help' lists the steps and options (gcp-edit then serves its browser page).";

  /** System look and feel, font smoothing on Linux; call before the first window. */
  static void lookAndFeel() {
    GuiPreferences.applyScale();
    if (System.getProperty("awt.useSystemAAFontSettings") == null) System.setProperty("awt.useSystemAAFontSettings", "on");
    if (System.getProperty("swing.aatext") == null) System.setProperty("swing.aatext", "true");
    try {
      UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
    } catch (ReflectiveOperationException | UnsupportedLookAndFeelException e) {
      // The cross-platform look and feel stays.
    }
    ToolTipManager.sharedInstance().setDismissDelay(30_000);
  }

  private static List<Image> icons;

  /**
   * The POSE icon (resource sgm/pose-icon.png, the logo of the repository) in the sizes window
   * managers pick from: title bar, taskbar, Alt+Tab. Empty when the resource is missing.
   */
  static synchronized List<Image> icons() {
    if (icons != null) return icons;
    List<Image> out = new ArrayList<>();
    try (var in = GuiSupport.class.getResourceAsStream("/sgm/pose-icon.png")) {
      java.awt.image.BufferedImage source = in == null ? null : javax.imageio.ImageIO.read(in);
      if (source != null) for (int size : new int[] {16, 20, 24, 32, 40, 48, 64, 128, 256}) out.add(scaled(source, size));
    } catch (IOException e) {
      // No icon: the platform default stays.
    }
    icons = List.copyOf(out);
    return icons;
  }

  /** Square downscale in halving steps (bilinear), so small sizes stay sharp and smooth. */
  static java.awt.image.BufferedImage scaled(java.awt.image.BufferedImage source, int size) {
    return scaledTo(source, size, size);
  }

  /** Downscale to width x height in halving steps (premultiplied alpha, bilinear). */
  static java.awt.image.BufferedImage scaledTo(java.awt.image.BufferedImage source, int width, int height) {
    java.awt.image.BufferedImage current = source;
    int w = source.getWidth(), h = source.getHeight();
    do {
      w = Math.max(width, w / 2);
      h = Math.max(height, h / 2);
      // Premultiplied alpha: no fringe from the colour of transparent pixels.
      java.awt.image.BufferedImage next = new java.awt.image.BufferedImage(w, h, java.awt.image.BufferedImage.TYPE_INT_ARGB_PRE);
      Graphics2D g = next.createGraphics();
      g.setRenderingHint(RenderingHints.KEY_INTERPOLATION, RenderingHints.VALUE_INTERPOLATION_BILINEAR);
      g.setRenderingHint(RenderingHints.KEY_RENDERING, RenderingHints.VALUE_RENDER_QUALITY);
      g.drawImage(current, 0, 0, w, h, null);
      g.dispose();
      current = next;
    } while (w > width || h > height);
    return current;
  }

  /** Gives a window the POSE icon (and the taskbar/dock where the platform supports it). */
  static void brand(Window window) {
    List<Image> list = icons();
    if (list.isEmpty()) return;
    window.setIconImages(list);
    try {
      if (Taskbar.isTaskbarSupported() && Taskbar.getTaskbar().isSupported(Taskbar.Feature.ICON_IMAGE))
        Taskbar.getTaskbar().setIconImage(list.get(list.size() - 1));
    } catch (RuntimeException e) {
      // Not supported here; the window icons suffice.
    }
  }

  /** Fallback command that opens a file or folder with the desktop's default application. */
  static List<String> openCommand(Path path, String osName) {
    String os = osName.toLowerCase(Locale.ROOT);
    if (os.contains("win")) return Files.isDirectory(path) ? List.of("explorer.exe", path.toString()) : List.of("rundll32", "url.dll,FileProtocolHandler", path.toString());
    if (os.contains("mac")) return List.of("open", path.toString());
    return List.of("xdg-open", path.toString());
  }

  /** Opens a file or folder: Desktop.open, else xdg-open / explorer. */
  static void open(Component parent, Path path) {
    if (!Files.exists(path)) {
      JOptionPane.showMessageDialog(parent, "Not found:\n" + path, "Open", JOptionPane.WARNING_MESSAGE);
      return;
    }
    try {
      if (Desktop.isDesktopSupported() && Desktop.getDesktop().isSupported(Desktop.Action.OPEN)) {
        Desktop.getDesktop().open(path.toFile());
        return;
      }
    } catch (IOException | RuntimeException e) {
      // Try the fallback below.
    }
    try {
      new ProcessBuilder(openCommand(path, System.getProperty("os.name"))).redirectErrorStream(true).redirectOutput(ProcessBuilder.Redirect.DISCARD).start();
    } catch (IOException e) {
      JOptionPane.showMessageDialog(parent, "Cannot open " + path + ":\n" + e.getMessage(), "Open", JOptionPane.WARNING_MESSAGE);
    }
  }

  /** Selectable text (error messages, logs) with Copy and Close. */
  static void textDialog(Component parent, String title, String text) {
    JTextArea area = new JTextArea(text);
    area.setEditable(false);
    area.setFont(new Font(Font.MONOSPACED, Font.PLAIN, area.getFont().getSize()));
    area.setCaretPosition(0);
    JScrollPane scroll = new JScrollPane(area);
    scroll.setPreferredSize(new Dimension(900, 520));
    Object[] buttons = {"Copy", "Close"};
    int choice = JOptionPane.showOptionDialog(parent, scroll, title, JOptionPane.DEFAULT_OPTION, JOptionPane.PLAIN_MESSAGE, null, buttons, buttons[1]);
    if (choice == 0) copy(area.getSelectedText() == null || area.getSelectedText().isEmpty() ? text : area.getSelectedText());
  }

  static void copy(String text) {
    Toolkit.getDefaultToolkit().getSystemClipboard().setContents(new java.awt.datatransfer.StringSelection(text), null);
  }

  /** Error dialog with the message of the cause. */
  static void error(Component parent, String title, Throwable e) {
    Throwable cause = e;
    while (cause.getCause() != null && (cause instanceof java.util.concurrent.ExecutionException || cause instanceof RuntimeException && cause.getMessage() == null))
      cause = cause.getCause();
    JOptionPane.showMessageDialog(parent, String.valueOf(cause.getMessage() == null ? cause : cause.getMessage()), title, JOptionPane.ERROR_MESSAGE);
  }

  /** A FlowLayout whose preferred height follows the wrapped rows (inside a scroll pane). */
  static final class WrapLayout extends FlowLayout {
    WrapLayout(int gap) {
      super(FlowLayout.LEFT, gap, gap);
    }

    @Override
    public Dimension preferredLayoutSize(Container target) {
      return size(target, true);
    }

    @Override
    public Dimension minimumLayoutSize(Container target) {
      Dimension d = size(target, false);
      d.width -= getHgap() + 1;
      return d;
    }

    private Dimension size(Container target, boolean preferred) {
      synchronized (target.getTreeLock()) {
        Container scroll = SwingUtilities.getAncestorOfClass(JScrollPane.class, target);
        int width = scroll != null ? scroll.getWidth() : target.getWidth();
        if (width == 0) width = Integer.MAX_VALUE;
        Insets insets = target.getInsets();
        int max = width - (insets.left + insets.right + getHgap() * 2) - (scroll != null ? 24 : 0);
        int w = 0, h = 0, rowW = 0, rowH = 0;
        for (Component c : target.getComponents()) {
          if (!c.isVisible()) continue;
          Dimension d = preferred ? c.getPreferredSize() : c.getMinimumSize();
          if (rowW + d.width > max && rowW > 0) {
            w = Math.max(w, rowW);
            h += rowH + getVgap();
            rowW = 0;
            rowH = 0;
          }
          rowW += d.width + getHgap();
          rowH = Math.max(rowH, d.height);
        }
        w = Math.max(w, rowW);
        h += rowH;
        return new Dimension(w + insets.left + insets.right + getHgap() * 2, h + insets.top + insets.bottom + getVgap() * 2);
      }
    }
  }
}
