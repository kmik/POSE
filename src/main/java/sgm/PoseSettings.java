package sgm;

import java.awt.*;
import java.awt.event.*;
import java.io.IOException;
import java.nio.file.*;
import java.util.*;
import java.util.List;
import javax.swing.*;
import javax.swing.border.EmptyBorder;

/**
 * The Settings menu of the POSE window: Preferences (GuiPreferences), fitting the compute options
 * to this computer, and housekeeping of the remembered settings. The logic lives in
 * GuiPreferences; this class only shows it.
 */
final class PoseSettings {
  private PoseSettings() {}

  static JMenu menu(PoseWindow w) {
    JMenu menu = new JMenu("Settings");
    menu.setMnemonic(KeyEvent.VK_S);
    JMenuItem prefs = new JMenuItem("Preferences...");
    prefs.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_COMMA, Toolkit.getDefaultToolkit().getMenuShortcutKeyMaskEx()));
    prefs.addActionListener(e -> preferences(w));
    JMenuItem fit = new JMenuItem("Fit compute options to this computer...");
    fit.setToolTipText("Sets CPU cores, memory budget and GPU memory on the New run tab from this computer's hardware");
    fit.addActionListener(e -> fit(w));
    JMenuItem recent = new JMenuItem("Clear recent projects");
    recent.addActionListener(e -> clearRecent(w));
    JMenuItem forget = new JMenuItem("Forget saved options...");
    forget.setToolTipText("Deletes the options remembered for each project and as your defaults");
    forget.addActionListener(e -> forget(w));
    JMenuItem folder = new JMenuItem("Open settings folder");
    folder.addActionListener(e -> {
      try {
        Files.createDirectories(w.settings.dir);
      } catch (IOException ignored) {
        // GuiSupport.open reports a missing folder.
      }
      GuiSupport.open(w.frame, w.settings.dir);
    });
    menu.add(prefs);
    menu.add(fit);
    menu.addSeparator();
    menu.add(recent);
    menu.add(forget);
    menu.add(folder);
    return menu;
  }

  // ---------------------------------------------------------------------------------------------

  static final String[] SCALE_LABELS = {"Automatic (system setting)", "100%", "125%", "150%", "175%", "200%"};
  static final String[] NEW_PROJECT_LABELS = {"the last options I used", "the POSE defaults"};

  static void preferences(PoseWindow w) {
    GuiPreferences p = GuiPreferences.load(w.settings.dir);
    Path cache = w.settings.dir.resolve("image-cache");
    JComboBox<String> scale = new JComboBox<>(SCALE_LABELS);
    scale.setSelectedIndex(Math.max(0, GuiPreferences.SCALES.indexOf(p.scale)));
    JComboBox<String> newProjects = new JComboBox<>(NEW_PROJECT_LABELS);
    newProjects.setSelectedIndex(Math.max(0, GuiPreferences.NEW_PROJECTS.indexOf(p.newProjects)));
    JCheckBox openReport = new JCheckBox("Open the report when a run completes", p.openReport);
    JCheckBox toFront = new JCheckBox("Bring POSE to the front when a command ends", p.toFront);
    JCheckBox confirmClose = new JCheckBox("Ask before closing POSE while a command runs", p.confirmClose);
    JSpinner cacheMb = new JSpinner(new SpinnerNumberModel(p.cacheMb, 0, 100_000, 100));
    JLabel cacheNow = new JLabel();
    Runnable showCache = () -> cacheNow.setText(String.format(Locale.ROOT, "now %.0f MB", GuiPreferences.cacheBytes(cache) / 1048576.0));
    showCache.run();
    JButton clearCache = new JButton("Clear now");
    clearCache.addActionListener(e -> {
      GuiPreferences.trim(cache, 0);
      showCache.run();
    });

    JPanel form = new JPanel(new GridBagLayout());
    form.setBorder(new EmptyBorder(10, 12, 6, 12));
    GridBagConstraints c = new GridBagConstraints();
    c.insets = new Insets(4, 4, 4, 4);
    c.anchor = GridBagConstraints.WEST;
    int row = 0;
    row = row(form, c, row, "Interface scale:", scale, note("applies when POSE starts next time"));
    row = row(form, c, row, "New projects start with:", newProjects, note("a project you used before keeps its own options"));
    for (JCheckBox b : List.of(openReport, toFront, confirmClose)) {
      c.gridx = 0;
      c.gridy = row++;
      c.gridwidth = 3;
      form.add(b, c);
      c.gridwidth = 1;
    }
    JPanel cacheRow = new JPanel(new FlowLayout(FlowLayout.LEFT, 6, 0));
    cacheRow.add(cacheMb);
    cacheRow.add(new JLabel("MB"));
    cacheRow.add(cacheNow);
    cacheRow.add(clearCache);
    row = row(form, c, row, "Image preview cache:", cacheRow, note("oldest previews are removed first"));
    scale.setToolTipText("Size of text and controls. Automatic follows Windows' display scaling; on Linux set GDK_SCALE or choose a size here.");
    newProjects.setToolTipText("Which options the New run tab shows for a project folder POSE has not seen before");
    openReport.setToolTipText("Opens the PDF report (or HTML) of a run that finished all its steps");
    toFront.setToolTipText("Long runs end while you work in other windows; this brings POSE back to the front");
    cacheMb.setToolTipText("Decoded previews of the Images window are kept here so they open fast the next time");

    int a = JOptionPane.showConfirmDialog(w.frame, form, "Preferences", JOptionPane.OK_CANCEL_OPTION, JOptionPane.PLAIN_MESSAGE);
    if (a != JOptionPane.OK_OPTION) return;
    String oldScale = p.scale;
    p.scale = GuiPreferences.SCALES.get(scale.getSelectedIndex());
    p.newProjects = GuiPreferences.NEW_PROJECTS.get(newProjects.getSelectedIndex());
    p.openReport = openReport.isSelected();
    p.toFront = toFront.isSelected();
    p.confirmClose = confirmClose.isSelected();
    p.cacheMb = (Integer) cacheMb.getValue();
    try {
      p.save();
    } catch (IOException e) {
      GuiSupport.error(w.frame, "Preferences", e);
      return;
    }
    w.preferences = p;
    GuiPreferences.trim(cache, (long) p.cacheMb << 20);
    if (!oldScale.equals(p.scale))
      JOptionPane.showMessageDialog(w.frame, "The new interface scale applies when POSE starts next time.", "Preferences", JOptionPane.INFORMATION_MESSAGE);
  }

  private static JLabel note(String text) {
    JLabel l = new JLabel(text);
    l.setForeground(Color.GRAY);
    return l;
  }

  private static int row(JPanel form, GridBagConstraints c, int row, String label, JComponent field, JComponent note) {
    c.gridy = row;
    c.gridx = 0;
    form.add(new JLabel(label), c);
    c.gridx = 1;
    form.add(field, c);
    c.gridx = 2;
    form.add(note, c);
    return row + 1;
  }

  // ---------------------------------------------------------------------------------------------

  static void fit(PoseWindow w) {
    Map<String, String> current = w.launcher.values();
    int device = 0;
    try {
      device = Integer.parseInt(current.getOrDefault("--gpu-device", "0").trim());
    } catch (NumberFormatException ignored) {
      // Device 0.
    }
    int gpuDevice = device;
    w.frame.setCursor(Cursor.getPredefinedCursor(Cursor.WAIT_CURSOR));
    new SwingWorker<GuiPreferences.Machine, Void>() {
      @Override
      protected GuiPreferences.Machine doInBackground() {
        return GuiPreferences.machine(gpuDevice); // OpenCL enumeration can take a moment
      }

      @Override
      protected void done() {
        w.frame.setCursor(Cursor.getDefaultCursor());
        GuiPreferences.Machine m;
        try {
          m = get();
        } catch (Exception e) {
          GuiSupport.error(w.frame, "Fit compute options", e);
          return;
        }
        Map<String, String> fitted = GuiPreferences.fitted(m);
        StringBuilder s = new StringBuilder("This computer: " + m.describe() + ".\n\nSet on the New run tab:\n");
        for (var e : fitted.entrySet())
          s.append("    ").append(e.getKey()).append(' ').append(e.getValue())
              .append("    (now ").append(current.getOrDefault(e.getKey(), "").isBlank() ? "default" : current.get(e.getKey())).append(")\n");
        s.append("\nCores: three quarters of the processors; memory budget: half the RAM; GPU memory: 95% of the device (the most the command line accepts). These are also the window's defaults. Other options stay as they are.");
        int a = JOptionPane.showConfirmDialog(w.frame, s.toString(), "Fit compute options to this computer", JOptionPane.OK_CANCEL_OPTION, JOptionPane.QUESTION_MESSAGE);
        if (a != JOptionPane.OK_OPTION) return;
        Map<String, String> v = new LinkedHashMap<>(w.launcher.values());
        v.putAll(fitted);
        w.launcher.load(v);
        w.launcher.changedNow();
      }
    }.execute();
  }

  static void clearRecent(PoseWindow w) {
    try {
      GuiPreferences.clearRecent(w.settings);
    } catch (IOException e) {
      GuiSupport.error(w.frame, "Clear recent projects", e);
      return;
    }
    Object current = w.projectBox.getSelectedItem();
    ActionListener[] listeners = w.projectBox.getActionListeners();
    for (ActionListener l : listeners) w.projectBox.removeActionListener(l);
    w.projectBox.removeAllItems();
    w.projectBox.setSelectedItem(current);
    for (ActionListener l : listeners) w.projectBox.addActionListener(l);
  }

  static void forget(PoseWindow w) {
    int a = JOptionPane.showConfirmDialog(w.frame,
        "Delete the options POSE remembers for each project folder and as your defaults?\n"
            + "Projects and runs are not touched. The New run tab keeps what it shows now;\n"
            + "project folders opened later start with the POSE defaults.",
        "Forget saved options", JOptionPane.OK_CANCEL_OPTION, JOptionPane.WARNING_MESSAGE);
    if (a != JOptionPane.OK_OPTION) return;
    int n = GuiPreferences.forgetOptions(w.settings);
    JOptionPane.showMessageDialog(w.frame, n + " settings file" + (n == 1 ? "" : "s") + " deleted.", "Forget saved options", JOptionPane.INFORMATION_MESSAGE);
  }
}
