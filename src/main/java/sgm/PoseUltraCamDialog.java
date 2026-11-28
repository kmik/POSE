package sgm;

import java.awt.*;
import java.awt.event.KeyEvent;
import java.io.IOException;
import java.io.Reader;
import java.io.Writer;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.*;
import java.util.List;
import javax.swing.*;
import javax.swing.border.EmptyBorder;
import javax.swing.border.TitledBorder;
import javax.swing.filechooser.FileNameExtensionFilter;

/**
 * Tools > Convert UltraCam orientations to INPHO: the GUI of `photogrammetry ultracam-inpho`. One
 * panel per camera (exterior file + calibration report); all cameras go into one project file. The
 * conversion is a small file job, so it runs in this JVM (UltraCamInpho.convert) rather than as a
 * child process. The fields are remembered in ultracam-inpho.properties next to the GUI settings.
 */
final class PoseUltraCamDialog {
  static final String TITLE = "Convert UltraCam orientations to INPHO";

  /** The fields of one camera. */
  final class CameraFields {
    final JPanel panel = new JPanel(new GridBagLayout());
    final JTextField exteriors = new JTextField(36), report = new JTextField(36), images = new JTextField(36);
    final JComboBox<String> layout = new JComboBox<>(UltraCamInpho.LAYOUTS.toArray(String[]::new));
    final JComboBox<String> sensor = new JComboBox<>(new String[] {"pan", "ms"});
    final JButton remove = new JButton("Remove");

    CameraFields() {
      exteriors.setToolTipText("<html>Text file, one image per line: <tt>name X Y Z omega phi kappa</tt><br>(more columns are ignored; spaces, tabs, commas or semicolons)</html>");
      report.setToolTipText("Vexcel calibration report PDF of this camera: focal length, principal point, pixel size and image format are read from it");
      images.setToolTipText("<html>Optional. The image file names and the image layout (which way the flight runs in the image) are taken from the images found here.<br>"
          + "Empty: the folder of the camera above.</html>");
      layout.setToolTipText("<html>How the stored pixels sit against the flight direction:<br>"
          + "<b>auto</b>: from the first image's size (without images: along-track)<br>"
          + "<b>along-track</b>: long-track side is the width, flight to the right (kappa as given)<br>"
          + "<b>along-track-reversed</b>: flight to the left (+200 gon)<br>"
          + "<b>up</b>: cross-track side is the width, flight to the top (-100 gon)<br>"
          + "<b>down</b>: flight to the bottom (+100 gon)</html>");
      sensor.setToolTipText("pan: panchromatic / pan-sharpened RGB format; ms: multispectral format");
      remove.setToolTipText("Leave this camera out");
      remove.addActionListener(e -> removeCamera(this));
      panel.setBorder(BorderFactory.createCompoundBorder(new TitledBorder("Camera"), new EmptyBorder(2, 6, 4, 6)));
      int row = 0;
      row = fileRow(panel, row, "Exterior orientations", exteriors, () -> chooseFile(exteriors, "Exterior orientation file", false, new FileNameExtensionFilter("Text files", "txt", "csv", "dat", "eo")));
      row = fileRow(panel, row, "Calibration report (PDF)", report, () -> chooseFile(report, "Calibration report", false, new FileNameExtensionFilter("PDF files", "pdf")));
      row = fileRow(panel, row, "Image folder (optional)", images, () -> chooseImages(images));
      JPanel choices = new JPanel(new FlowLayout(FlowLayout.LEFT, 6, 0));
      choices.add(new JLabel("Layout"));
      choices.add(layout);
      choices.add(new JLabel("  Sensor"));
      choices.add(sensor);
      GridBagConstraints c = new GridBagConstraints();
      c.gridx = 0;
      c.gridy = row;
      c.gridwidth = 2;
      c.anchor = GridBagConstraints.WEST;
      c.insets = new Insets(4, 0, 0, 0);
      panel.add(choices, c);
      c.gridx = 2;
      c.gridwidth = 1;
      c.anchor = GridBagConstraints.EAST;
      panel.add(remove, c);
    }

    void set(String ext, String rep, String img, String lay, String sen) {
      exteriors.setText(ext);
      report.setText(rep);
      images.setText(img);
      layout.setSelectedItem(lay == null || lay.isBlank() ? "auto" : lay);
      sensor.setSelectedItem(sen == null || sen.isBlank() ? "pan" : sen);
    }
  }

  final JDialog dialog;
  final PoseWindow window;
  final List<CameraFields> cameras = new ArrayList<>();
  final JPanel cameraList = new JPanel();
  final JScrollPane cameraScroll;
  final JTextField output = new JTextField(36), terrain = new JTextField(8);
  final JComboBox<String> unit = new JComboBox<>(new String[] {"gon", "deg", "rad"});
  final JTextArea result = new JTextArea(12, 60);
  final JButton convert = new JButton("Convert"), use = new JButton("Use in New run"), copy = new JButton("Copy command"), close = new JButton("Close"),
      add = new JButton("Add camera");
  Path written;
  UltraCamInpho.Settings convertedWith;

  static JMenu menu(PoseWindow w) {
    JMenu menu = new JMenu("Tools");
    menu.setMnemonic(KeyEvent.VK_T);
    JMenuItem item = new JMenuItem(TITLE + "...");
    item.setToolTipText("UltraCam exterior orientations + Vexcel calibration report PDF (one pair per camera) -> INPHO .prj for POSE and Metashape");
    item.addActionListener(e -> new PoseUltraCamDialog(w).dialog.setVisible(true));
    menu.add(item);
    return menu;
  }

  PoseUltraCamDialog(PoseWindow w) {
    window = w;
    dialog = new JDialog(w == null ? null : w.frame, TITLE, false);
    GuiSupport.brand(dialog);
    dialog.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);

    output.setToolTipText("The INPHO project file (.prj) to write; it holds all cameras");
    terrain.setToolTipText("Optional mean terrain height (m), written per photo");
    unit.setToolTipText("Unit of omega, phi and kappa in the exterior files");
    add.setToolTipText("Another camera: its own exterior file and calibration report, written into the same project");
    add.addActionListener(e -> {
      addCamera();
      relayout();
    });

    cameraList.setLayout(new BoxLayout(cameraList, BoxLayout.Y_AXIS));
    cameraScroll = new JScrollPane(cameraList, ScrollPaneConstants.VERTICAL_SCROLLBAR_AS_NEEDED, ScrollPaneConstants.HORIZONTAL_SCROLLBAR_NEVER);
    cameraScroll.setBorder(BorderFactory.createEmptyBorder());
    cameraScroll.getVerticalScrollBar().setUnitIncrement(16);

    JPanel form = new JPanel(new GridBagLayout());
    form.setBorder(new EmptyBorder(10, 10, 4, 10));
    GridBagConstraints c = new GridBagConstraints();
    c.gridx = 0;
    c.gridy = 0;
    c.gridwidth = 3;
    c.fill = GridBagConstraints.HORIZONTAL;
    c.weightx = 1;
    form.add(cameraScroll, c);
    c.gridy = 1;
    c.fill = GridBagConstraints.NONE;
    c.anchor = GridBagConstraints.WEST;
    c.insets = new Insets(2, 0, 6, 0);
    form.add(add, c);
    int row = fileRow(form, 2, "Output INPHO project", output, () -> chooseFile(output, "INPHO project file", true, new FileNameExtensionFilter("INPHO project", "prj")));
    JPanel choices = new JPanel(new FlowLayout(FlowLayout.LEFT, 6, 0));
    choices.add(new JLabel("Angles"));
    choices.add(unit);
    choices.add(new JLabel("  Terrain height"));
    choices.add(terrain);
    c = new GridBagConstraints();
    c.gridx = 0;
    c.gridy = row;
    c.gridwidth = 3;
    c.anchor = GridBagConstraints.WEST;
    c.insets = new Insets(6, 0, 0, 0);
    form.add(choices, c);

    result.setEditable(false);
    result.setLineWrap(true);
    result.setWrapStyleWord(true);
    result.setFont(new Font(Font.MONOSPACED, Font.PLAIN, result.getFont().getSize()));
    result.setText("Choose the exterior orientation file and the calibration report of each camera, then Convert.\n"
        + "The .prj works as Orientations on the New run tab and in Metashape (Import Cameras, INPHO project).");
    JScrollPane scroll = new JScrollPane(result);
    scroll.setBorder(BorderFactory.createCompoundBorder(new EmptyBorder(4, 10, 4, 10), scroll.getBorder()));

    use.setEnabled(false);
    use.setToolTipText("Puts the written .prj (format inpho) and the image folder on the New run tab");
    copy.setToolTipText("Copies the equivalent photogrammetry ultracam-inpho command line");
    convert.addActionListener(e -> convert());
    use.addActionListener(e -> useInLauncher());
    copy.addActionListener(e -> GuiSupport.copy(commandLine()));
    close.addActionListener(e -> dialog.dispose());
    JPanel buttons = new JPanel(new FlowLayout(FlowLayout.RIGHT));
    buttons.add(copy);
    buttons.add(use);
    buttons.add(convert);
    buttons.add(close);

    JPanel content = new JPanel(new BorderLayout());
    content.add(form, BorderLayout.NORTH);
    content.add(scroll, BorderLayout.CENTER);
    content.add(buttons, BorderLayout.SOUTH);
    dialog.setContentPane(content);
    dialog.getRootPane().setDefaultButton(convert);
    load();
    if (cameras.isEmpty()) addCamera();
    relayout();
    dialog.pack();
    dialog.setLocationRelativeTo(w == null ? null : w.frame);
  }

  CameraFields addCamera() {
    CameraFields f = new CameraFields();
    cameras.add(f);
    cameraList.add(f.panel);
    return f;
  }

  private void removeCamera(CameraFields f) {
    if (cameras.size() <= 1) return;
    cameras.remove(f);
    cameraList.remove(f.panel);
    relayout();
  }

  /** Numbers the camera panels and shows at most two of them before scrolling. */
  private void relayout() {
    for (int i = 0; i < cameras.size(); i++) {
      CameraFields f = cameras.get(i);
      ((TitledBorder) ((javax.swing.border.CompoundBorder) f.panel.getBorder()).getOutsideBorder())
          .setTitle(cameras.size() == 1 ? "Camera" : "Camera " + (i + 1));
      f.remove.setVisible(cameras.size() > 1);
    }
    int one = cameras.get(0).panel.getPreferredSize().height, all = cameraList.getPreferredSize().height;
    cameraScroll.setPreferredSize(new Dimension(cameraList.getPreferredSize().width + 20, Math.min(all, 2 * one + 8)));
    cameraList.revalidate();
    cameraList.repaint();
    if (dialog.isShowing()) {
      Dimension before = dialog.getSize();
      dialog.pack();
      dialog.setSize(Math.max(before.width, dialog.getWidth()), dialog.getHeight());
    }
    if (!cameras.isEmpty()) SwingUtilities.invokeLater(() -> cameras.get(cameras.size() - 1).panel.scrollRectToVisible(new Rectangle(0, 0, 1, one)));
  }

  private int fileRow(JPanel form, int row, String label, JTextField field, Runnable browse) {
    GridBagConstraints c = new GridBagConstraints();
    c.gridy = row;
    c.insets = new Insets(2, 0, 2, 6);
    c.anchor = GridBagConstraints.WEST;
    form.add(new JLabel(label), c);
    c.gridx = 1;
    c.fill = GridBagConstraints.HORIZONTAL;
    c.weightx = 1;
    form.add(field, c);
    c.gridx = 2;
    c.fill = GridBagConstraints.NONE;
    c.weightx = 0;
    JButton b = new JButton("Browse...");
    b.addActionListener(e -> browse.run());
    form.add(b, c);
    return row + 1;
  }

  private Path firstExteriors() {
    for (CameraFields f : cameras) if (path(f.exteriors) != null) return path(f.exteriors);
    return null;
  }

  private void chooseFile(JTextField target, String title, boolean save, FileNameExtensionFilter filter) {
    Path start = path(target);
    if (start == null) start = firstExteriors();
    JFileChooser chooser = new JFileChooser(start == null ? null : Files.isDirectory(start) ? start.toFile() : start.getParent() == null ? null : start.getParent().toFile());
    chooser.setDialogTitle(title);
    chooser.setFileFilter(filter);
    if (save && path(target) != null) chooser.setSelectedFile(path(target).toFile());
    int answer = save ? chooser.showSaveDialog(dialog) : chooser.showOpenDialog(dialog);
    if (answer != JFileChooser.APPROVE_OPTION) return;
    String chosen = chooser.getSelectedFile().getPath();
    if (save && !chosen.toLowerCase(Locale.ROOT).endsWith(".prj")) chosen += ".prj";
    target.setText(chosen);
    if (target != output && output.getText().isBlank() && filter.getExtensions()[0].equals("txt")) output.setText(defaultOutput(Path.of(chosen)).toString());
  }

  private void chooseImages(JTextField target) {
    Path start = path(target), ext = firstExteriors();
    JFileChooser chooser = new JFileChooser(start != null ? start.toFile() : ext != null && ext.getParent() != null ? ext.getParent().toFile() : null);
    chooser.setDialogTitle("Image folder");
    chooser.setFileSelectionMode(JFileChooser.DIRECTORIES_ONLY);
    if (chooser.showOpenDialog(dialog) == JFileChooser.APPROVE_OPTION) target.setText(chooser.getSelectedFile().getPath());
  }

  static Path defaultOutput(Path exteriors) {
    String n = exteriors.getFileName().toString();
    int dot = n.lastIndexOf('.');
    return exteriors.resolveSibling((dot > 0 ? n.substring(0, dot) : n) + "_inpho.prj");
  }

  private static Path path(JTextField f) {
    String t = f.getText().strip();
    if (t.length() >= 2 && t.startsWith("\"") && t.endsWith("\"")) t = t.substring(1, t.length() - 1);
    return t.isEmpty() ? null : Path.of(t);
  }

  /** The fields as ultracam-inpho arguments (only the set ones; cameras without any file are left out). */
  List<String> args() {
    List<String> a = new ArrayList<>();
    for (CameraFields f : cameras) {
      if (path(f.exteriors) == null && path(f.report) == null && path(f.images) == null) continue;
      put(a, "--exteriors", f.exteriors);
      put(a, "--calibration-report", f.report);
      put(a, "--images", f.images);
      if (!"auto".equals(f.layout.getSelectedItem())) a.addAll(List.of("--layout", (String) f.layout.getSelectedItem()));
      if (!"pan".equals(f.sensor.getSelectedItem())) a.addAll(List.of("--sensor", (String) f.sensor.getSelectedItem()));
    }
    put(a, "--output", output);
    if (!"gon".equals(unit.getSelectedItem())) a.addAll(List.of("--angle-unit", (String) unit.getSelectedItem()));
    if (!terrain.getText().isBlank()) a.addAll(List.of("--terrain-height", terrain.getText().strip()));
    return a;
  }

  private static void put(List<String> a, String key, JTextField f) {
    Path p = path(f);
    if (p != null) a.addAll(List.of(key, p.toString()));
  }

  String commandLine() {
    StringBuilder s = new StringBuilder("photogrammetry ultracam-inpho");
    for (String v : args()) s.append(' ').append(v.startsWith("--") ? v : quote(v));
    return s.toString();
  }

  private static String quote(String s) {
    return s.matches("[A-Za-z0-9_./:\\\\+-]+") ? s : "\"" + s + "\"";
  }

  /** What is missing before a conversion, or null. */
  String missing() {
    int n = 0, used = 0;
    for (CameraFields f : cameras) {
      n++;
      boolean e = path(f.exteriors) != null, r = path(f.report) != null;
      if (!e && !r && path(f.images) == null) continue;
      used++;
      String which = cameras.size() > 1 ? " of camera " + n : "";
      if (!e) return "the exterior orientation file" + which;
      if (!r) return "the calibration report PDF" + which;
    }
    if (used == 0) return "the exterior orientation file and the calibration report";
    if (path(output) == null) return "the output .prj file";
    return null;
  }

  void convert() {
    String missing = missing();
    if (missing != null) {
      result.setText("Missing: " + missing);
      return;
    }
    UltraCamInpho.Settings settings;
    try {
      settings = UltraCamInpho.parse(args().toArray(String[]::new));
    } catch (RuntimeException e) {
      result.setText("Error: " + e.getMessage());
      return;
    }
    Path out = settings.output();
    if (Files.exists(out) && !out.equals(written)
        && JOptionPane.showConfirmDialog(dialog, out + " exists. Replace it?", TITLE, JOptionPane.YES_NO_OPTION, JOptionPane.WARNING_MESSAGE) != JOptionPane.YES_OPTION)
      return;
    save();
    convert.setEnabled(false);
    use.setEnabled(false);
    result.setText("Converting...");
    new SwingWorker<UltraCamInpho.Result, Void>() {
      @Override
      protected UltraCamInpho.Result doInBackground() throws Exception {
        return UltraCamInpho.convert(settings);
      }

      @Override
      protected void done() {
        convert.setEnabled(true);
        try {
          UltraCamInpho.Result r = get();
          written = r.output();
          convertedWith = settings;
          use.setEnabled(window != null);
          String folders = folderNote(settings);
          result.setText(r.summary() + (folders == null ? "" : "\n" + folders + "\n") + "\nCommand line:\n" + commandLine());
        } catch (Exception e) {
          Throwable cause = e.getCause() == null ? e : e.getCause();
          written = null;
          convertedWith = null;
          result.setText("Error: " + cause.getMessage());
        }
        result.setCaretPosition(0);
      }
    }.execute();
  }

  /** The image folders of all cameras (after carrying a folder over to the cameras below it). */
  static List<Path> imageFolders(UltraCamInpho.Settings s) {
    LinkedHashSet<Path> folders = new LinkedHashSet<>();
    for (UltraCamInpho.CameraInput c : s.cameras()) if (c.images() != null) folders.add(c.images().toAbsolutePath().normalize());
    return new ArrayList<>(folders);
  }

  /** POSE reads the images of a run from one folder: say so when the cameras use several. */
  static String folderNote(UltraCamInpho.Settings s) {
    List<Path> folders = imageFolders(s);
    if (folders.size() < 2) return null;
    return "Note: the cameras' images are in " + folders.size() + " folders. A POSE run reads one image folder (-i), and Use in New run puts "
        + folders.get(0) + " there; put all images into one folder (the .prj finds them by name) for a run with every camera.";
  }

  void useInLauncher() {
    if (window == null || written == null) return;
    Map<String, String> v = window.launcher.values();
    v.put("--orientation-input", written.toString());
    v.put("--orientation-input-format", "inpho");
    List<Path> folders = convertedWith == null ? List.of() : imageFolders(convertedWith);
    if (!folders.isEmpty()) v.put("-i", folders.get(0).toString());
    v.put("advanced", window.launcher.advancedText());
    window.launcher.load(v);
    window.tabs.setSelectedComponent(window.launcher.panel);
    window.frame.toFront();
  }

  // ----- remembered fields -----

  private Path store() {
    return window == null ? null : window.settings.dir.resolve("ultracam-inpho.properties");
  }

  private void load() {
    Path f = store();
    if (f == null || !Files.isRegularFile(f)) return;
    Properties p = new Properties();
    try (Reader r = Files.newBufferedReader(f, StandardCharsets.UTF_8)) {
      p.load(r);
    } catch (IOException e) {
      return;
    }
    load(p);
  }

  void load(Properties p) {
    cameras.clear();
    cameraList.removeAll();
    int n;
    try {
      n = Math.max(0, Math.min(50, Integer.parseInt(p.getProperty("cameras", "0"))));
    } catch (NumberFormatException e) {
      n = 0;
    }
    for (int i = 1; i <= n; i++) {
      String k = "camera." + i + ".";
      addCamera().set(p.getProperty(k + "exteriors", ""), p.getProperty(k + "report", ""), p.getProperty(k + "images", ""), p.getProperty(k + "layout"), p.getProperty(k + "sensor"));
    }
    if (n == 0 && p.containsKey("exteriors")) // one camera, as written before several cameras were possible
      addCamera().set(p.getProperty("exteriors", ""), p.getProperty("report", ""), p.getProperty("images", ""), p.getProperty("layout"), p.getProperty("sensor"));
    output.setText(p.getProperty("output", ""));
    terrain.setText(p.getProperty("terrain", ""));
    unit.setSelectedItem(p.getProperty("unit", "gon"));
  }

  Properties properties() {
    Properties p = new Properties();
    p.setProperty("cameras", Integer.toString(cameras.size()));
    for (int i = 0; i < cameras.size(); i++) {
      CameraFields c = cameras.get(i);
      String k = "camera." + (i + 1) + ".";
      p.setProperty(k + "exteriors", c.exteriors.getText().strip());
      p.setProperty(k + "report", c.report.getText().strip());
      p.setProperty(k + "images", c.images.getText().strip());
      p.setProperty(k + "layout", (String) c.layout.getSelectedItem());
      p.setProperty(k + "sensor", (String) c.sensor.getSelectedItem());
    }
    p.setProperty("output", output.getText().strip());
    p.setProperty("terrain", terrain.getText().strip());
    p.setProperty("unit", (String) unit.getSelectedItem());
    return p;
  }

  private void save() {
    Path f = store();
    if (f == null) return;
    try {
      Files.createDirectories(f.getParent());
      try (Writer w = Files.newBufferedWriter(f, StandardCharsets.UTF_8)) {
        properties().store(w, "POSE: Tools > " + TITLE);
      }
    } catch (IOException e) {
      // Remembering the fields is a convenience only.
    }
  }
}
