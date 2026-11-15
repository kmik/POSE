package sgm;

import java.awt.*;
import java.util.*;
import java.util.List;
import javax.swing.*;
import javax.swing.border.*;
import javax.swing.event.*;

/**
 * "Advanced options" tab: every other command-line option of a new run as a field, generated
 * from the help text ({@link GuiOptions#advanced()}), grouped by the help's sections, with a
 * search, "only the ones I set" and a counter. The values go into the New run command after the
 * common fields (the fields are registered with the launcher, so they are validated, remembered
 * and reset with it).
 */
final class PoseAdvancedPanel {
  final PoseLauncherPanel launcher;
  final JPanel panel = new JPanel(new BorderLayout(4, 4));
  final JTextField search = new JTextField(24);
  final JCheckBox onlySet = new JCheckBox("Show only the options I set");
  final JLabel count = new JLabel(" ");
  final List<Row> rows = new ArrayList<>();
  final Map<String, JPanel> sections = new LinkedHashMap<>();
  Runnable onCount;

  record Row(GuiOptions.Field field, JLabel flag, JComponent input, JButton browse, JLabel description, String text) {}

  PoseAdvancedPanel(PoseLauncherPanel launcher) {
    this.launcher = launcher;
    Box body = Box.createVerticalBox();
    for (String name : GuiOptions.advancedSections()) {
      JPanel p = new JPanel(new GridBagLayout());
      p.setBorder(new TitledBorder(name));
      p.setAlignmentX(Component.LEFT_ALIGNMENT);
      sections.put(name, p);
      body.add(p);
    }
    Font mono = new Font(Font.MONOSPACED, Font.PLAIN, UIManager.getFont("Label.font").getSize());
    Map<String, Integer> next = new HashMap<>();
    for (GuiOptions.Field f : GuiOptions.advanced()) {
      JPanel p = sections.get(f.section());
      int y = next.merge(f.section(), 1, Integer::sum) - 1;
      JLabel flag = new JLabel(f.flag());
      flag.setFont(mono);
      flag.setToolTipText(GuiOptions.tooltip(f.flag()));
      JComponent[] made = launcher.makeInput(f, false);
      JLabel description = new JLabel(f.label());
      description.setForeground(UIManager.getColor("Label.disabledForeground") == null ? Color.GRAY : UIManager.getColor("Label.disabledForeground").darker());
      description.setToolTipText(GuiOptions.tooltip(f.flag()));
      GridBagConstraints c = new GridBagConstraints();
      c.gridy = y;
      c.insets = new Insets(1, 4, 1, 4);
      c.anchor = GridBagConstraints.WEST;
      c.gridx = 0;
      p.add(flag, c);
      c.gridx = 1;
      c.fill = f.kind() == GuiOptions.Kind.FLAG ? GridBagConstraints.NONE : GridBagConstraints.HORIZONTAL;
      made[0].setPreferredSize(new Dimension(f.kind() == GuiOptions.Kind.FLAG ? made[0].getPreferredSize().width : 190, made[0].getPreferredSize().height));
      p.add(made[0], c);
      c.gridx = 2;
      c.fill = GridBagConstraints.NONE;
      if (made[1] != null) p.add(made[1], c);
      c.gridx = 3;
      c.weightx = 1;
      p.add(description, c);
      String text = (f.flag() + " " + f.section() + " " + GuiOptions.help(f.flag())).toLowerCase(Locale.ROOT);
      rows.add(new Row(f, flag, made[0], (JButton) made[1], description, text));
    }
    search.setToolTipText("Shows the options whose name or help mentions the text.");
    search.getDocument().addDocumentListener(
        new DocumentListener() {
          public void insertUpdate(DocumentEvent e) {
            filter();
          }

          public void removeUpdate(DocumentEvent e) {
            filter();
          }

          public void changedUpdate(DocumentEvent e) {
            filter();
          }
        });
    onlySet.addActionListener(e -> filter());
    JButton clear = new JButton("Clear advanced options");
    clear.setToolTipText("Back to the command-line defaults for every option on this tab (the New run fields stay).");
    clear.addActionListener(e -> clear());
    JPanel top = new JPanel(new FlowLayout(FlowLayout.LEFT, 6, 4));
    top.add(new JLabel("Search:"));
    top.add(search);
    top.add(onlySet);
    top.add(clear);
    top.add(count);
    JLabel note =
        new JLabel(
            "<html>Every other command-line option of a new run. Empty means the command-line default (shown in grey). Hover a name for its full help."
                + " These go into the command on the New run tab, after its fields; the Extra arguments there come last and win.</html>");
    note.setBorder(new EmptyBorder(0, 8, 4, 8));
    JPanel north = new JPanel(new BorderLayout());
    north.add(top, BorderLayout.NORTH);
    north.add(note, BorderLayout.SOUTH);
    JPanel holder = new JPanel(new BorderLayout());
    holder.setBorder(new EmptyBorder(4, 8, 8, 8));
    holder.add(body, BorderLayout.NORTH);
    JScrollPane scroll = new JScrollPane(holder);
    scroll.getVerticalScrollBar().setUnitIncrement(24);
    panel.add(north, BorderLayout.NORTH);
    panel.add(scroll, BorderLayout.CENTER);
    refresh();
  }

  boolean set(Row r) {
    String v = launcher.values().get(r.field().flag());
    return v != null && !v.isBlank();
  }

  /** Marks the options that are set and updates the counter (after any change or load). */
  void refresh() {
    Map<String, String> values = launcher.values();
    int n = 0;
    for (Row r : rows) {
      String v = values.get(r.field().flag());
      boolean on = v != null && !v.isBlank();
      if (on) n++;
      r.flag().setFont(r.flag().getFont().deriveFont(on ? Font.BOLD : Font.PLAIN));
      r.flag().setForeground(on ? PoseWindow.RUNNING : UIManager.getColor("Label.foreground"));
    }
    count.setText(n == 0 ? "No advanced options set." : n + (n == 1 ? " advanced option set." : " advanced options set."));
    if (onlySet.isSelected()) filter();
    if (onCount != null) onCount.run();
  }

  int setCount() {
    int n = 0;
    Map<String, String> values = launcher.values();
    for (Row r : rows) {
      String v = values.get(r.field().flag());
      if (v != null && !v.isBlank()) n++;
    }
    return n;
  }

  /** Shows the rows that match the search (and are set, when asked); hides empty sections. */
  void filter() {
    String q = search.getText().strip().toLowerCase(Locale.ROOT);
    Map<String, Boolean> any = new HashMap<>();
    for (Row r : rows) {
      boolean show = (q.isEmpty() || r.text().contains(q)) && (!onlySet.isSelected() || set(r));
      for (JComponent c : new JComponent[] {r.flag(), r.input(), r.browse(), r.description()}) if (c != null) c.setVisible(show);
      if (show) any.put(r.field().section(), true);
    }
    for (var e : sections.entrySet()) e.getValue().setVisible(any.getOrDefault(e.getKey(), false));
    panel.revalidate();
    panel.repaint();
  }

  void clear() {
    Map<String, String> keep = new HashMap<>(launcher.values());
    for (Row r : rows) keep.remove(r.field().flag());
    keep.put("advanced", launcher.advanced.getText());
    launcher.load(keep);
  }
}
