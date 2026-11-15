package sgm;

import java.awt.*;
import java.io.IOException;
import java.nio.file.*;
import java.util.*;
import java.util.List;
import javax.swing.*;
import javax.swing.border.*;
import javax.swing.event.*;

/**
 * "New run" tab: the common options as fields (tooltips are the CLI help entries), advanced
 * arguments, a preview of the exact command, validation with the real option parser, the memory
 * estimate, and Run. Values are remembered per project and as the user's defaults.
 */
final class PoseLauncherPanel {
  final PoseWindow window;
  final JPanel panel = new JPanel(new BorderLayout(8, 8));
  final Map<String, JComponent> inputs = new LinkedHashMap<>();
  final JTextArea advanced = new JTextArea(3, 40), preview = new JTextArea(3, 40);
  final JLabel validation = new JLabel(" ");
  final JButton run = new JButton("Run"), estimate = new JButton("Memory estimate"), reset = new JButton("Reset options"), copy = new JButton("Copy command");
  final javax.swing.Timer validateLater = new javax.swing.Timer(300, e -> validateNow());
  boolean loading;
  PoseAdvancedPanel advancedOptions;

  PoseLauncherPanel(PoseWindow window) {
    this.window = window;
    validateLater.setRepeats(false);
    JPanel sections = new JPanel(new GridLayout(0, 2, 12, 8));
    for (String section : List.of(GuiOptions.INPUT, GuiOptions.PROCESSING, GuiOptions.COMPUTE, GuiOptions.OUTPUT)) sections.add(section(section));
    JPanel extra = new JPanel(new BorderLayout(4, 4));
    extra.setBorder(new TitledBorder("Extra arguments (typed, quoted like a shell; every option also has a field on the Advanced options tab)"));
    advanced.setLineWrap(true);
    advanced.setToolTipText("<html>Command-line options typed by hand, e.g. <tt>--features 40000 --ratio 0.85</tt>.<br>They come after the fields and override them. Help &gt; Command-line options lists them all.</html>");
    advanced.getDocument().addDocumentListener(changed());
    extra.add(new JScrollPane(advanced), BorderLayout.CENTER);
    JPanel command = new JPanel(new BorderLayout(4, 4));
    command.setBorder(new TitledBorder("Command"));
    preview.setEditable(false);
    preview.setLineWrap(true);
    preview.setFont(new Font(Font.MONOSPACED, Font.PLAIN, preview.getFont().getSize()));
    command.add(new JScrollPane(preview), BorderLayout.CENTER);
    command.add(validation, BorderLayout.SOUTH);
    JPanel buttons = new JPanel(new FlowLayout(FlowLayout.RIGHT, 6, 0));
    for (JButton b : List.of(reset, copy, estimate, run)) buttons.add(b);
    run.setFont(run.getFont().deriveFont(Font.BOLD));
    run.setToolTipText("Start a new run in the project folder with all steps (inputs ... finish).");
    estimate.setToolTipText("<html>Read the image headers and print the resolution and memory plan<br>(--estimate-memory, in a temporary project; nothing is processed).</html>");
    reset.setToolTipText("<html>Back to the window's defaults on both tabs (the tested mvsgm setup, with cores and memory sized for this computer); the folders stay.<br>Clear advanced options on the Advanced options tab returns those to the command-line defaults.</html>");
    run.addActionListener(e -> runNow());
    estimate.addActionListener(e -> estimateMemory());
    reset.addActionListener(e -> resetOptions());
    copy.addActionListener(e -> Toolkit.getDefaultToolkit().getSystemClipboard().setContents(new java.awt.datatransfer.StringSelection(preview.getText()), null));
    JPanel south = new JPanel(new BorderLayout(4, 4));
    south.add(extra, BorderLayout.NORTH);
    south.add(command, BorderLayout.CENTER);
    south.add(buttons, BorderLayout.SOUTH);
    JPanel body = new JPanel(new BorderLayout(8, 8));
    body.setBorder(new EmptyBorder(8, 8, 8, 8));
    body.add(sections, BorderLayout.NORTH);
    body.add(south, BorderLayout.CENTER);
    JScrollPane scroll = new JScrollPane(body);
    scroll.getVerticalScrollBar().setUnitIncrement(24);
    scroll.setBorder(null);
    panel.add(scroll, BorderLayout.CENTER);
    // Every other option, on its own tab; its fields join this form.
    advancedOptions = new PoseAdvancedPanel(this);
  }

  DocumentListener changed() {
    return new DocumentListener() {
      @Override
      public void insertUpdate(DocumentEvent e) {
        changedNow();
      }

      @Override
      public void removeUpdate(DocumentEvent e) {
        changedNow();
      }

      @Override
      public void changedUpdate(DocumentEvent e) {
        changedNow();
      }
    };
  }

  void changedNow() {
    if (loading) return;
    tiePointFields();
    updatePreview();
    validateLater.restart();
    if (advancedOptions != null) advancedOptions.refresh();
  }

  JPanel section(String name) {
    JPanel p = new JPanel(new GridBagLayout());
    p.setBorder(new TitledBorder(name));
    GridBagConstraints c = new GridBagConstraints();
    c.insets = new Insets(2, 4, 2, 4);
    int row = 0;
    for (GuiOptions.Field f : GuiOptions.FIELDS) {
      if (!f.section().equals(name) || f.flag().equals("-o")) continue; // the project folder is the bar above
      JLabel label = new JLabel(f.label());
      label.setToolTipText(GuiOptions.tooltip(f.flag()));
      JComponent[] made = makeInput(f, f.flag().equals("--gcp-check"));
      JComponent input = made[0];
      JButton browse = (JButton) made[1];
      c.gridx = 0;
      c.gridy = row;
      c.weightx = 0;
      c.anchor = GridBagConstraints.WEST;
      c.fill = GridBagConstraints.NONE;
      c.gridwidth = 1;
      p.add(label, c);
      c.gridx = 1;
      c.weightx = 1;
      c.fill = f.kind() == GuiOptions.Kind.FLAG ? GridBagConstraints.NONE : GridBagConstraints.HORIZONTAL;
      c.gridwidth = browse == null ? 2 : 1;
      p.add(input, c);
      if (browse != null) {
        c.gridx = 2;
        c.weightx = 0;
        c.fill = GridBagConstraints.NONE;
        p.add(browse, c);
      }
      row++;
    }
    c.gridy = row;
    c.weighty = 1;
    p.add(Box.createVerticalGlue(), c);
    return p;
  }

  /**
   * The input of a field ({input, browse button or null}), registered under its flag so its value
   * goes into the command. Choices show "(default)" for the CLI default; editable ones (and
   * suggestion lists such as "auto" for N|auto) accept typed values.
   */
  JComponent[] makeInput(GuiOptions.Field f, boolean editable) {
    String tip = GuiOptions.tooltip(f.flag());
    JComponent input;
    JButton browse = null;
    switch (f.kind()) {
      case FLAG -> {
        JCheckBox box = new JCheckBox();
        box.addActionListener(e -> changedNow());
        input = box;
      }
      case MULTI -> {
        MultiChoice box = new MultiChoice(f.choices(), GuiOptions.defaultHint(f.flag()), this::changedNow);
        input = box;
      }
      case CHOICE, SUGGEST -> {
        String hint = GuiOptions.defaultHint(f.flag());
        String label = hint.isEmpty() ? "(default)" : "(default: " + hint + ")";
        JComboBox<String> box = new JComboBox<>(f.choices().stream().map(s -> s.isEmpty() ? label : s).toArray(String[]::new));
        box.setEditable(editable || f.kind() == GuiOptions.Kind.SUGGEST);
        box.addActionListener(e -> changedNow());
        // Typed values count at once, not only after Enter.
        if (box.isEditable() && box.getEditor().getEditorComponent() instanceof JTextField text) text.getDocument().addDocumentListener(changed());
        input = box;
      }
      default -> {
        JTextField text = new HintField(f.kind() == GuiOptions.Kind.TEXT ? 10 : 22, GuiOptions.defaultHint(f.flag()));
        text.getDocument().addDocumentListener(changed());
        input = text;
        if (f.kind() != GuiOptions.Kind.TEXT) {
          browse = new JButton("...");
          browse.setMargin(new Insets(1, 6, 1, 6));
          boolean dir = f.kind() == GuiOptions.Kind.DIR;
          browse.setToolTipText(dir ? "Choose a folder" : "Choose a file");
          browse.addActionListener(e -> choose(text, dir, f.flag()));
        }
      }
    }
    input.setToolTipText(tip);
    inputs.put(f.flag(), input);
    return new JComponent[] {input, browse};
  }

  /**
   * Several values from a fixed list: a button showing the picked ones that opens checkboxes.
   * Only the listed values can be chosen.
   */
  static final class MultiChoice extends JButton {
    final List<String> choices;
    final String empty;
    final Map<String, JCheckBox> boxes = new LinkedHashMap<>();
    final JPopupMenu popup = new JPopupMenu();
    final Runnable changed;

    MultiChoice(List<String> choices, String hint, Runnable changed) {
      this.choices = choices;
      this.changed = changed;
      empty = hint.isEmpty() ? "(default)" : "(default: " + hint + ")";
      setHorizontalAlignment(SwingConstants.LEFT);
      JPanel list = new JPanel(new GridLayout(0, 2, 8, 0));
      list.setBorder(new EmptyBorder(4, 6, 4, 6));
      for (String c : choices) {
        if (c.isEmpty()) continue;
        JCheckBox b = new JCheckBox(c);
        b.addActionListener(e -> update(true));
        boxes.put(c, b);
        list.add(b);
      }
      JButton none = new JButton("None");
      none.addActionListener(e -> {
        setValue("");
        changed.run();
      });
      JPanel all = new JPanel(new BorderLayout());
      all.add(list, BorderLayout.CENTER);
      all.add(none, BorderLayout.SOUTH);
      popup.add(all);
      addActionListener(e -> popup.show(this, 0, getHeight()));
      update(false);
    }

    String value() {
      List<String> picked = new ArrayList<>();
      for (var e : boxes.entrySet()) if (e.getValue().isSelected()) picked.add(e.getKey());
      return GuiOptions.multi(picked, choices);
    }

    void setValue(String value) {
      Set<String> picked = new HashSet<>(Arrays.asList(GuiOptions.multi(Arrays.asList(value.split(",")), choices).split(",")));
      for (var e : boxes.entrySet()) e.getValue().setSelected(picked.contains(e.getKey()));
      update(false);
    }

    void update(boolean notify) {
      String v = value();
      setText(v.isEmpty() ? empty : v.replace(",", ", ") + "  ▾");
      if (notify) changed.run();
    }
  }

  /** A text field that shows the command-line default in grey while it is empty. */
  static final class HintField extends JTextField {
    final String hint;

    HintField(int columns, String hint) {
      super(columns);
      this.hint = hint == null || hint.isEmpty() ? "" : "default: " + hint;
    }

    @Override
    protected void paintComponent(Graphics g0) {
      super.paintComponent(g0);
      if (!getText().isEmpty() || hint.isEmpty()) return;
      Graphics2D g = (Graphics2D) g0.create();
      g.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
      g.setColor(UIManager.getColor("TextField.inactiveForeground") == null ? Color.GRAY : UIManager.getColor("TextField.inactiveForeground"));
      Insets in = getInsets();
      FontMetrics fm = g.getFontMetrics(getFont());
      g.drawString(hint, in.left + 2, in.top + (getHeight() - in.top - in.bottom - fm.getHeight()) / 2 + fm.getAscent());
      g.dispose();
    }
  }

  void choose(JTextField target, boolean dir, String title) {
    String current = target.getText().strip();
    Path start = current.isEmpty() ? window.project() : Path.of(current);
    JFileChooser chooser = new JFileChooser(start != null && Files.isDirectory(start) ? start.toFile() : start != null && start.getParent() != null ? start.getParent().toFile() : null);
    chooser.setDialogTitle(title);
    // Orientation imports may be a file or a folder (COLMAP, Bundler, ...).
    chooser.setFileSelectionMode(dir ? JFileChooser.DIRECTORIES_ONLY : JFileChooser.FILES_AND_DIRECTORIES);
    // The lidar reference is one LAS/LAZ file or a folder of tiles.
    if (title.equals("--lidar-reference")) chooser.setFileFilter(new javax.swing.filechooser.FileNameExtensionFilter("LAS/LAZ point clouds (or a folder of tiles)", "las", "laz"));
    if (chooser.showOpenDialog(panel) == JFileChooser.APPROVE_OPTION) target.setText(chooser.getSelectedFile().getPath());
  }

  /** Field values (flag to text; "true"/"false" for flags; "" for the CLI default). */
  Map<String, String> values() {
    Map<String, String> out = new LinkedHashMap<>();
    for (var e : inputs.entrySet()) {
      JComponent c = e.getValue();
      String v;
      if (c instanceof MultiChoice m) v = m.value();
      else if (c instanceof JCheckBox b) v = b.isSelected() ? "true" : "";
      else if (c instanceof JComboBox<?> b) {
        Object o = b.isEditable() ? b.getEditor().getItem() : b.getSelectedItem();
        v = o == null || o.toString().startsWith("(default") ? "" : o.toString().strip();
      } else v = ((JTextField) c).getText().strip();
      out.put(e.getKey(), v);
    }
    Path p = window.project();
    out.put("-o", p == null ? "" : p.toString());
    return out;
  }

  String advancedText() {
    return advanced.getText().replace('\n', ' ').strip();
  }

  /** Puts remembered values into the fields (missing ones become the CLI default). */
  void load(Map<String, String> v) {
    loading = true;
    try {
      for (var e : inputs.entrySet()) {
        String value = v.getOrDefault(e.getKey(), "");
        JComponent c = e.getValue();
        if (c instanceof MultiChoice m) m.setValue(value);
        else if (c instanceof JCheckBox b) b.setSelected(Boolean.parseBoolean(value));
        else if (c instanceof JComboBox<?> b) {
          @SuppressWarnings("unchecked")
          JComboBox<String> box = (JComboBox<String>) b;
          if (value.isEmpty()) box.setSelectedIndex(0);
          else {
            box.setSelectedItem(value);
            // A remembered value that is not offered (fixed list) goes back to the default.
            if (!box.isEditable() && !value.equals(box.getSelectedItem())) box.setSelectedIndex(0);
          }
        } else ((JTextField) c).setText(value);
      }
      advanced.setText(v.getOrDefault("advanced", ""));
    } finally {
      loading = false;
    }
    tiePointFields();
    updatePreview();
    validateNow();
    if (advancedOptions != null) advancedOptions.refresh();
  }

  /** The cleanup limit fields are used with custom (and as the pause window's start) only. */
  void tiePointFields() {
    boolean used = GuiOptions.tiePointLimitsUsed(values());
    for (String flag : GuiOptions.TIE_POINT_LIMITS) {
      JComponent c = inputs.get(flag);
      if (c != null) c.setEnabled(used);
    }
  }

  void resetOptions() {
    // Back to the window's defaults (the tested mvsgm setup); the folders stay.
    load(window.settings.reset(values()));
  }

  List<String> arguments() {
    return GuiOptions.arguments(values(), advancedText());
  }

  void updatePreview() {
    try {
      preview.setText(PoseProcess.display(arguments(), PoseProcess.windows()));
    } catch (IllegalArgumentException e) {
      preview.setText("(" + e.getMessage() + ")");
    }
  }

  /** Null when valid; also shows the result under the command. */
  String validateNow() {
    String problem;
    try {
      problem = GuiOptions.validate(arguments());
    } catch (IllegalArgumentException e) {
      problem = e.getMessage();
    }
    validation.setText(problem == null ? "Options are valid." : problem);
    validation.setForeground(problem == null ? PoseWindow.COMPLETE : PoseWindow.FAILED);
    return problem;
  }

  void updateButtons() {
    boolean busy = window.busy();
    run.setEnabled(!busy);
    estimate.setEnabled(!busy);
  }

  void remember() {
    Map<String, String> v = new TreeMap<>(values());
    v.put("advanced", advanced.getText());
    try {
      window.settings.save(window.project(), v);
    } catch (IOException e) {
      // Settings are a convenience; the run goes on.
      System.err.println("Cannot save the launcher settings: " + e.getMessage());
    }
  }

  void runNow() {
    String problem = validateNow();
    if (problem != null) {
      JOptionPane.showMessageDialog(panel, problem, "Check the options", JOptionPane.WARNING_MESSAGE);
      return;
    }
    Path project = window.project();
    remember();
    if (window.run("New run in " + project, arguments(), project, null, null)) {
      String p = project.toString();
      if (((DefaultComboBoxModel<String>) window.projectBox.getModel()).getIndexOf(p) < 0) window.projectBox.insertItemAt(p, 0);
    }
  }

  /** --estimate-memory in a temporary project: the plan is shown, nothing is kept. */
  void estimateMemory() {
    List<String> args = arguments();
    String problem = GuiOptions.validate(args);
    if (problem != null) {
      JOptionPane.showMessageDialog(panel, problem, "Check the options", JOptionPane.WARNING_MESSAGE);
      return;
    }
    Path scratch;
    try {
      scratch = Files.createTempDirectory("pose-estimate-");
    } catch (IOException e) {
      GuiSupport.error(panel, "Memory estimate", e);
      return;
    }
    List<String> estimate = new ArrayList<>(GuiOptions.withProject(args, scratch.resolve("project")));
    estimate.add("--estimate-memory");
    estimate.add("--verbose"); // the plan lines are only printed verbosely
    window.run(
        "Memory estimate",
        estimate,
        null,
        null,
        exit -> {
          deleteTree(scratch);
          List<String> plan = GuiOptions.estimateLines(window.active.tail());
          String text = String.join("\n", plan.isEmpty() || exit != 0 ? window.active.tail() : plan);
          JTextArea area = new JTextArea(text, 24, 100);
          area.setEditable(false);
          area.setFont(new Font(Font.MONOSPACED, Font.PLAIN, area.getFont().getSize()));
          JOptionPane.showMessageDialog(panel, new JScrollPane(area), exit == 0 ? "Memory estimate" : "Memory estimate failed", exit == 0 ? JOptionPane.INFORMATION_MESSAGE : JOptionPane.WARNING_MESSAGE);
        });
  }

  static void deleteTree(Path root) {
    try (var walk = Files.walk(root)) {
      for (Path p : walk.sorted(Comparator.reverseOrder()).toList()) Files.deleteIfExists(p);
    } catch (IOException e) {
      // a temporary folder; the system cleans it eventually
    }
  }
}
