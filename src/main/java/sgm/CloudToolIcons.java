package sgm;

import java.awt.*;
import java.util.function.Consumer;
import javax.swing.*;
import javax.swing.border.Border;

/** Resolution-independent toolbar icons for the four mutually exclusive viewer tools. */
final class CloudToolIcons {
  private CloudToolIcons() {}

  static void addTo(JToolBar toolbar, Consumer<CloudView.Tool> select) {
    ButtonGroup group = new ButtonGroup();
    for (CloudView.Tool tool : new CloudView.Tool[] {
        CloudView.Tool.NAVIGATE, CloudView.Tool.RECTANGLE,
        CloudView.Tool.INSPECT, CloudView.Tool.MEASURE}) {
      String name = switch (tool) {
        case NAVIGATE -> "Navigate";
        case RECTANGLE -> "Clip";
        case INSPECT -> "Inspect";
        case MEASURE -> "Measure";
      };
      String help = switch (tool) {
        case NAVIGATE -> "Orbit, pan and zoom the point cloud";
        case RECTANGLE -> "Draw a rectangular clip in top view";
        case INSPECT -> "Click a point for XYZ, RGB, class and available attributes";
        case MEASURE -> "Click two points to measure distance and height difference";
      };
      JToggleButton button = new JToggleButton(new ToolIcon(tool));
      button.setToolTipText(name + " — " + help);
      button.getAccessibleContext().setAccessibleName(name);
      button.getAccessibleContext().setAccessibleDescription(help);
      button.setFocusable(false);
      button.setPreferredSize(new Dimension(36, 36));
      button.setMaximumSize(new Dimension(36, 36));
      Border normal = BorderFactory.createEmptyBorder(2, 2, 2, 2);
      Border active = BorderFactory.createLineBorder(new Color(0x2563eb), 2, true);
      button.setBorder(normal);
      button.addItemListener(e -> button.setBorder(button.isSelected() ? active : normal));
      button.addActionListener(e -> select.accept(tool));
      group.add(button);
      toolbar.add(button);
      button.setSelected(tool == CloudView.Tool.NAVIGATE);
    }
  }

  private record ToolIcon(CloudView.Tool tool) implements Icon {
    public int getIconWidth() { return 24; }
    public int getIconHeight() { return 24; }

    public void paintIcon(Component component, Graphics graphics, int x, int y) {
      Graphics2D g = (Graphics2D) graphics.create();
      try {
        g.translate(x, y);
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        boolean selected = component instanceof AbstractButton b && b.isSelected();
        g.setColor(!component.isEnabled() ? Color.GRAY
            : selected ? new Color(0x2563eb) : component.getForeground());
        g.setStroke(new BasicStroke(1.8f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
        switch (tool) {
          case NAVIGATE -> {
            Polygon arrow = new Polygon(new int[] {5, 5, 10, 14, 17, 13, 21},
                new int[] {3, 21, 16, 23, 21, 14, 14}, 7);
            g.fillPolygon(arrow);
          }
          case RECTANGLE -> {
            g.drawLine(3, 7, 17, 7);
            g.drawLine(17, 7, 17, 22);
            g.drawLine(7, 3, 7, 17);
            g.drawLine(7, 17, 22, 17);
          }
          case INSPECT -> {
            g.drawOval(3, 3, 14, 14);
            g.fillOval(8, 8, 4, 4);
            g.drawLine(16, 16, 22, 22);
          }
          case MEASURE -> {
            g.rotate(-Math.PI / 4, 12, 12);
            g.drawRoundRect(2, 7, 20, 10, 2, 2);
            for (int tick = 6; tick <= 18; tick += 4)
              g.drawLine(tick, 7, tick, tick == 10 ? 13 : 11);
          }
        }
      } finally {
        g.dispose();
      }
    }
  }
}
