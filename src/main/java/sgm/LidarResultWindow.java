package sgm;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.nio.file.Path;
import java.util.List;
import javax.imageio.ImageIO;
import javax.swing.*;
import javax.swing.border.EmptyBorder;

/**
 * The lidar result of a run (Project tab, "Lidar result..."): the transformation table, dates,
 * warnings and distances (ProjectFiles.lidarHtml) beside the coverage map and the difference
 * maps, plus "Redo lidar with other settings...". Reads the run's files only.
 */
final class LidarResultWindow {
  private LidarResultWindow() {}

  static void open(Component parent, ProjectFiles.Run run, Runnable redo) {
    JFrame frame = new JFrame("Lidar result - run " + run.id());
    GuiSupport.brand(frame);
    frame.setContentPane(content(run, redo, frame::dispose));
    frame.setSize(1100, 760);
    frame.setLocationRelativeTo(parent);
    frame.setVisible(true);
  }

  /** The window's content: text and maps side by side, the redo and close buttons. */
  static JPanel content(ProjectFiles.Run run, Runnable redo, Runnable close) {
    JEditorPane text = new JEditorPane("text/html", ProjectFiles.lidarHtml(ProjectFiles.lidar(run)));
    text.setEditable(false);
    text.putClientProperty(JEditorPane.HONOR_DISPLAY_PROPERTIES, Boolean.TRUE);
    text.setCaretPosition(0);
    JPanel pictures = new JPanel();
    pictures.setLayout(new BoxLayout(pictures, BoxLayout.Y_AXIS));
    pictures.setBorder(new EmptyBorder(6, 6, 6, 6));
    List<Object[]> images = ProjectFiles.lidarImages(run);
    if (images.isEmpty()) pictures.add(new JLabel("No maps yet (the difference maps are written by the finish step)."));
    for (Object[] im : images) {
      JLabel caption = new JLabel("<html><b>" + GuiOptions.escape((String) im[0]) + "</b></html>");
      caption.setAlignmentX(Component.LEFT_ALIGNMENT);
      pictures.add(caption);
      try {
        BufferedImage b = ImageIO.read(((Path) im[1]).toFile());
        double s = Math.min(4, 460.0 / Math.max(1, Math.max(b.getWidth(), b.getHeight())));
        JLabel l = new JLabel(new ImageIcon(b.getScaledInstance(Math.max(1, (int) (b.getWidth() * s)), Math.max(1, (int) (b.getHeight() * s)), Image.SCALE_FAST)));
        l.setAlignmentX(Component.LEFT_ALIGNMENT);
        pictures.add(l);
      } catch (Exception e) {
        pictures.add(new JLabel("Could not read " + im[1]));
      }
      pictures.add(Box.createVerticalStrut(10));
    }
    JSplitPane split = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT, new JScrollPane(text), new JScrollPane(pictures));
    split.setResizeWeight(.5);
    JButton again = new JButton("Redo lidar with other settings...");
    again.setToolTipText("<html>A new run from this one just before the lidar step: the lidar step and the later steps run again with the<br>"
        + "options you give (e.g. --lidar-search-shift 15 --lidar-canopy-weight 0); this run is not changed.</html>");
    again.addActionListener(e -> redo.run());
    JButton closeButton = new JButton("Close");
    closeButton.addActionListener(e -> close.run());
    JPanel buttons = new JPanel(new FlowLayout(FlowLayout.RIGHT));
    buttons.add(again);
    buttons.add(closeButton);
    JPanel content = new JPanel(new BorderLayout(4, 4));
    content.add(split, BorderLayout.CENTER);
    content.add(buttons, BorderLayout.SOUTH);
    return content;
  }
}
