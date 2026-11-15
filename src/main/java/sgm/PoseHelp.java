package sgm;

import java.awt.*;
import java.awt.event.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.List;
import javax.swing.*;
import javax.swing.border.EmptyBorder;
import javax.swing.event.DocumentEvent;
import javax.swing.event.DocumentListener;

/** The Help menu of the POSE window: user guide, command-line options, About box. */
final class PoseHelp {
  private PoseHelp() {}

  private static JFrame guide, options;

  /** The Help menu for a window's menu bar. */
  static JMenu menu(JFrame owner) {
    JMenu menu = new JMenu("Help");
    menu.setMnemonic(KeyEvent.VK_H);
    JMenuItem guideItem = new JMenuItem("User guide");
    guideItem.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_F1, 0));
    guideItem.addActionListener(e -> showGuide(owner));
    JMenuItem optionsItem = new JMenuItem("Command-line options...");
    optionsItem.addActionListener(e -> showOptions(owner, null));
    JMenuItem docsItem = new JMenuItem("Open documentation folder");
    docsItem.addActionListener(
        e -> {
          Path docs = GuiAbout.docs(GuiAbout.installRoot());
          if (docs == null) JOptionPane.showMessageDialog(owner, "The installation folder was not found.", "Documentation", JOptionPane.INFORMATION_MESSAGE);
          else GuiSupport.open(owner, docs);
        });
    JMenuItem aboutItem = new JMenuItem("About POSE...");
    aboutItem.addActionListener(e -> showAbout(owner));
    menu.add(guideItem);
    menu.add(optionsItem);
    menu.add(docsItem);
    menu.addSeparator();
    menu.add(aboutItem);
    return menu;
  }

  static JFrame helpFrame(String title, JComponent content, Component near) {
    JFrame f = new JFrame(title);
    GuiSupport.brand(f);
    f.setDefaultCloseOperation(WindowConstants.HIDE_ON_CLOSE);
    f.setContentPane(content);
    Rectangle screen = GraphicsEnvironment.getLocalGraphicsEnvironment().getMaximumWindowBounds();
    f.setSize(Math.min(screen.width, 900), (int) (screen.height * .85));
    f.setLocationRelativeTo(near);
    return f;
  }

  /** GUI.md as a page. */
  static void showGuide(JFrame owner) {
    if (guide == null) {
      Path file = GuiAbout.guide(GuiAbout.installRoot());
      String html;
      try {
        html = file == null
            ? "<html><body style='margin:12px'>The user guide (GUI.md) was not found next to the program.<br>Use Help &gt; Command-line options for the option help.</body></html>"
            : GuiAbout.html(Files.readString(file, StandardCharsets.UTF_8));
      } catch (java.io.IOException e) {
        html = "<html><body>Cannot read " + GuiAbout.escape(String.valueOf(file)) + ": " + GuiAbout.escape(e.getMessage()) + "</body></html>";
      }
      JEditorPane pane = new JEditorPane();
      pane.setEditable(false);
      pane.putClientProperty(JEditorPane.HONOR_DISPLAY_PROPERTIES, Boolean.TRUE);
      // The window's font for text; code and code blocks in a readable monospaced size.
      javax.swing.text.html.HTMLEditorKit kit = new javax.swing.text.html.HTMLEditorKit();
      javax.swing.text.html.StyleSheet css = new javax.swing.text.html.StyleSheet();
      css.addStyleSheet(kit.getStyleSheet());
      int size = Math.round(UIManager.getFont("Label.font").getSize2D());
      css.addRule("code, pre { font-family: Monospaced; font-size: " + size + "pt; }");
      css.addRule("h1 { font-size: " + (size + 8) + "pt; } h2 { font-size: " + (size + 4) + "pt; } h3 { font-size: " + (size + 2) + "pt; }");
      kit.setStyleSheet(css);
      pane.setEditorKit(kit);
      pane.setText(html);
      pane.setCaretPosition(0);
      JScrollPane scroll = new JScrollPane(pane);
      scroll.getVerticalScrollBar().setUnitIncrement(24);
      guide = helpFrame("POSE user guide", scroll, owner);
    }
    guide.setVisible(true);
    guide.toFront();
  }

  /** The full command-line help with a live search (as 'photogrammetry --help TEXT'). */
  static void showOptions(JFrame owner, String topic) {
    if (options == null) {
      JTextField search = new JTextField(30);
      JTextArea text = new JTextArea(Options.help());
      text.setEditable(false);
      text.setFont(new Font(Font.MONOSPACED, Font.PLAIN, text.getFont().getSize()));
      text.setCaretPosition(0);
      search.setToolTipText("Shows only the options whose description mentions the text (like 'photogrammetry --help TEXT').");
      search.getDocument().addDocumentListener(
          new DocumentListener() {
            void update() {
              String t = search.getText().strip();
              text.setText(t.isEmpty() ? Options.help() : Options.help(t));
              text.setCaretPosition(0);
            }

            public void insertUpdate(DocumentEvent e) {
              update();
            }

            public void removeUpdate(DocumentEvent e) {
              update();
            }

            public void changedUpdate(DocumentEvent e) {
              update();
            }
          });
      JPanel top = new JPanel(new BorderLayout(6, 0));
      top.setBorder(new EmptyBorder(8, 8, 8, 8));
      top.add(new JLabel("Search:"), BorderLayout.WEST);
      top.add(search, BorderLayout.CENTER);
      JPanel content = new JPanel(new BorderLayout());
      content.add(top, BorderLayout.NORTH);
      content.add(new JScrollPane(text), BorderLayout.CENTER);
      options = helpFrame("POSE command-line options", content, owner);
      options.getRootPane().putClientProperty("search", search);
    }
    if (topic != null) ((JTextField) options.getRootPane().getClientProperty("search")).setText(topic);
    options.setVisible(true);
    options.toFront();
  }

  /** Logo, version and environment, with a copy button for bug reports. */
  static void showAbout(JFrame owner) {
    Path root = GuiAbout.installRoot();
    JPanel panel = new JPanel(new BorderLayout(0, 10));
    try (var in = PoseHelp.class.getResourceAsStream("/sgm/pose-logo.png")) {
      if (in != null) {
        java.awt.image.BufferedImage logo = javax.imageio.ImageIO.read(in);
        int w = 360, h = logo.getHeight() * w / logo.getWidth();
        panel.add(new JLabel(new ImageIcon(GuiSupport.scaledTo(logo, w, h))), BorderLayout.NORTH);
      }
    } catch (java.io.IOException e) {
      // no logo
    }
    JPanel facts = new JPanel(new GridBagLayout());
    List<String[]> rows = GuiAbout.facts(root, "computing...");
    JLabel[] values = new JLabel[rows.size()];
    GridBagConstraints c = new GridBagConstraints();
    c.insets = new Insets(2, 4, 2, 10);
    c.anchor = GridBagConstraints.NORTHWEST;
    for (int i = 0; i < rows.size(); i++) {
      c.gridy = i;
      c.gridx = 0;
      JLabel label = new JLabel(rows.get(i)[0]);
      label.setFont(label.getFont().deriveFont(Font.BOLD));
      facts.add(label, c);
      c.gridx = 1;
      values[i] = new JLabel(rows.get(i)[1]);
      facts.add(values[i], c);
    }
    panel.add(facts, BorderLayout.CENTER);
    String[] engine = new String[1];
    // The engine hash reads the processing classes; off the event thread.
    Thread t =
        new Thread(
            () -> {
              String id;
              try {
                id = ReuseCache.engineIdentifier();
              } catch (Throwable e) {
                id = "unavailable";
              }
              engine[0] = id;
              List<String[]> done = GuiAbout.facts(root, id);
              SwingUtilities.invokeLater(
                  () -> {
                    for (int i = 0; i < values.length && i < done.size(); i++) values[i].setText(done.get(i)[1]);
                  });
            },
            "pose-about");
    t.setDaemon(true);
    t.start();
    Object[] buttons = {"Copy details", "Close"};
    int choice = JOptionPane.showOptionDialog(owner, panel, "About POSE", JOptionPane.DEFAULT_OPTION, JOptionPane.PLAIN_MESSAGE, null, buttons, buttons[1]);
    if (choice == 0)
      Toolkit.getDefaultToolkit().getSystemClipboard().setContents(new java.awt.datatransfer.StringSelection(GuiAbout.text(GuiAbout.facts(root, engine[0]))), null);
  }

  static String table(List<String[]> facts) {
    StringBuilder h = new StringBuilder("<html><body style='font-family:sans-serif'><table>");
    for (String[] f : facts) h.append("<tr><td valign='top'><b>").append(GuiAbout.escape(f[0])).append("</b></td><td>").append(GuiAbout.escape(f[1])).append("</td></tr>");
    return h.append("</table></body></html>").toString();
  }
}
