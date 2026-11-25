package sgm;

import java.awt.*;
import java.nio.file.*;
import java.util.*;
import java.util.List;
import javax.swing.*;
import javax.swing.border.*;
import javax.swing.table.*;

/**
 * "Project" tab: the runs of the project folder (project.json), the steps of the selected run with
 * status and times (state.json), actions on the run (next / remaining / selected steps, branch
 * from a step with changed options, GCP markers, report and orthomosaic again, open files) and
 * the report's summary numbers and quality flags.
 */
final class PoseProjectPanel {
  final PoseWindow window;
  final JPanel panel = new JPanel(new BorderLayout(8, 8));
  final DefaultTableModel runModel = table("Run", "Started", "Status", "Points", "Next step");
  final DefaultTableModel stepModel = table("Step", "Status", "Started", "Finished", "Time", "Detail");
  final JTable runTable = new JTable(runModel), stepTable = new JTable(stepModel);
  final JEditorPane summary = new JEditorPane("text/html", "");
  final JTextField extra = new JTextField();
  final JLabel extraCheck = new JLabel(" ");
  final JButton next = new JButton("Run next step"), remaining = new JButton("Run remaining steps"), selected = new JButton("Run selected steps"), branch = new JButton("Redo steps in a new run...");
  final JButton tiePoints = new JButton("Clean tie points...");
  final JButton gcps = new JButton("Edit GCPs"), report = new JButton("Report again"), ortho = new JButton("Orthomosaic again");
  final JButton pdf = new JButton("Report PDF"), html = new JButton("Report HTML"), results = new JButton("07_results"), folder = new JButton("Run folder"), refresh = new JButton("Refresh");
  final JButton images = new JButton("Images..."), view3d = new JButton("3D view..."), copyRun = new JButton("Copy run details");
  final JButton lidar = new JButton("Lidar result..."), copyCommand = new JButton("Copy command");
  final JButton registration = new JButton("Registration...");
  List<ProjectFiles.Run> runs = List.of();
  boolean filling;

  static DefaultTableModel table(String... columns) {
    return new DefaultTableModel(columns, 0) {
      @Override
      public boolean isCellEditable(int row, int column) {
        return false;
      }
    };
  }

  PoseProjectPanel(PoseWindow window) {
    this.window = window;
    runTable.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
    runTable.getSelectionModel().addListSelectionListener(e -> {
      if (!e.getValueIsAdjusting() && !filling) showRun();
    });
    runTable.setAutoCreateRowSorter(false);
    stepTable.setSelectionMode(ListSelectionModel.MULTIPLE_INTERVAL_SELECTION);
    stepTable.getSelectionModel().addListSelectionListener(e -> updateButtons());
    stepTable.setDefaultRenderer(Object.class, new StatusRenderer(1));
    stepTable.setToolTipText("Double-click a step for its full message (copyable); right-click for Copy.");
    stepTable.addMouseListener(
        new java.awt.event.MouseAdapter() {
          @Override
          public void mouseClicked(java.awt.event.MouseEvent e) {
            if (e.getClickCount() == 2 && stepTable.rowAtPoint(e.getPoint()) >= 0) stepDetails(stepTable.rowAtPoint(e.getPoint()));
          }

          @Override
          public void mousePressed(java.awt.event.MouseEvent e) {
            popup(e);
          }

          @Override
          public void mouseReleased(java.awt.event.MouseEvent e) {
            popup(e);
          }

          void popup(java.awt.event.MouseEvent e) {
            if (!e.isPopupTrigger()) return;
            int row = stepTable.rowAtPoint(e.getPoint());
            if (row >= 0 && !stepTable.isRowSelected(row)) stepTable.setRowSelectionInterval(row, row);
            JPopupMenu menu = new JPopupMenu();
            JMenuItem details = new JMenuItem("Show details...");
            details.addActionListener(a -> stepDetails(row));
            JMenuItem copy = new JMenuItem("Copy");
            copy.addActionListener(a -> GuiSupport.copy(selectedRowsText(stepTable)));
            menu.add(details);
            menu.add(copy);
            menu.show(stepTable, e.getX(), e.getY());
          }
        });
    runTable.setDefaultRenderer(Object.class, new StatusRenderer(2));
    widths(runTable, 230, 150, 90, 100, 100);
    widths(stepTable, 100, 80, 150, 150, 70, 400);
    summary.setEditable(false);
    summary.putClientProperty(JEditorPane.HONOR_DISPLAY_PROPERTIES, Boolean.TRUE);

    JPanel runsPanel = new JPanel(new BorderLayout(4, 4));
    runsPanel.setBorder(new TitledBorder("Runs"));
    runsPanel.add(new JScrollPane(runTable), BorderLayout.CENTER);
    JPanel runButtons = new JPanel(new FlowLayout(FlowLayout.LEFT, 4, 0));
    runButtons.add(refresh);
    runButtons.add(new JLabel("Open:"));
    for (JButton b : List.of(pdf, html, results, folder)) runButtons.add(b);
    runButtons.add(images);
    runButtons.add(view3d);
    runButtons.add(lidar);
    runButtons.add(registration);
    runsPanel.add(runButtons, BorderLayout.SOUTH);

    JPanel stepsPanel = new JPanel(new BorderLayout(4, 4));
    stepsPanel.setBorder(new TitledBorder("Steps of the selected run"));
    stepsPanel.add(new JScrollPane(stepTable), BorderLayout.CENTER);
    JPanel actions = new JPanel(new GuiSupport.WrapLayout(4));
    for (JButton b : List.of(next, remaining, selected, branch, tiePoints, gcps, report, ortho)) actions.add(b);
    JPanel options = new JPanel(new BorderLayout(4, 2));
    options.add(new JLabel("Options for these steps (optional):"), BorderLayout.WEST);
    options.add(extra, BorderLayout.CENTER);
    options.add(extraCheck, BorderLayout.SOUTH);
    extra.setToolTipText("<html>Options given to the steps run here (they also hold for later steps of the run),<br>e.g. <tt>--min-views 2 --filtering mild</tt>. For a branch they are the changed options.</html>");
    extra.getDocument().addDocumentListener(
        new javax.swing.event.DocumentListener() {
          public void insertUpdate(javax.swing.event.DocumentEvent e) {
            checkExtra();
          }

          public void removeUpdate(javax.swing.event.DocumentEvent e) {
            checkExtra();
          }

          public void changedUpdate(javax.swing.event.DocumentEvent e) {
            checkExtra();
          }
        });
    Box south = Box.createVerticalBox();
    options.setAlignmentX(Component.LEFT_ALIGNMENT);
    actions.setAlignmentX(Component.LEFT_ALIGNMENT);
    south.add(options);
    south.add(actions);
    stepsPanel.add(south, BorderLayout.SOUTH);

    JPanel summaryPanel = new JPanel(new BorderLayout());
    summaryPanel.setBorder(new TitledBorder("Summary and quality flags"));
    summaryPanel.add(new JScrollPane(summary), BorderLayout.CENTER);
    JPanel copyButtons = new JPanel(new FlowLayout(FlowLayout.LEFT, 4, 0));
    copyButtons.add(copyRun);
    copyButtons.add(copyCommand);
    summaryPanel.add(copyButtons, BorderLayout.SOUTH);

    JSplitPane lower = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT, stepsPanel, summaryPanel);
    lower.setResizeWeight(.6);
    JSplitPane split = new JSplitPane(JSplitPane.VERTICAL_SPLIT, runsPanel, lower);
    split.setResizeWeight(.35);
    panel.setBorder(new EmptyBorder(6, 6, 6, 6));
    panel.add(split, BorderLayout.CENTER);

    refresh.addActionListener(e -> refresh());
    next.setToolTipText("Run the first step that is not done yet.");
    remaining.setToolTipText("Run every step that is not done yet, in order.");
    selected.setToolTipText("Run the steps selected in the table (pipeline order; earlier steps must be done).");
    branch.setToolTipText("<html>Redo the selected step(s) with other options in a new run: the run is copied as it was<br>just before them (nothing earlier runs again); later steps stay pending. This run is not changed.</html>");
    gcps.setToolTipText("Open the native GCP marker editor; when it is closed with Done, the run continues.");
    report.setToolTipText("Write the processing report again (07_results/report) without reprocessing.");
    ortho.setToolTipText("Make the orthomosaic of the run again (07_results) without reprocessing.");
    next.addActionListener(e -> withRun(r -> continueRun(r, r.remaining().subList(0, 1))));
    remaining.addActionListener(e -> withRun(r -> continueRun(r, r.remaining())));
    selected.addActionListener(e -> withRun(r -> continueRun(r, selectedSteps())));
    branch.addActionListener(e -> withRun(this::branchRun));
    gcps.addActionListener(e -> withRun(r -> window.editGcps(window.project(), r.id())));
    tiePoints.setToolTipText("The run waits at the cleanup step (--tie-point-cleanup pause): choose the limits with the histograms, then continue.");
    tiePoints.addActionListener(e -> withRun(r -> window.cleanTiePoints(window.project(), r.id())));
    report.addActionListener(e -> withRun(r -> window.run("Report again for run " + r.id(), ProjectFiles.exportReport(r.dir()), window.project(), r.id(), null)));
    ortho.addActionListener(e -> withRun(r -> window.run("Orthomosaic again for run " + r.id(), ProjectFiles.exportOrthomosaic(r.dir()), window.project(), r.id(), null)));
    pdf.addActionListener(e -> withRun(r -> GuiSupport.open(panel, r.report("pdf"))));
    html.addActionListener(e -> withRun(r -> GuiSupport.open(panel, r.report("html"))));
    results.addActionListener(e -> withRun(r -> GuiSupport.open(panel, r.results())));
    folder.addActionListener(e -> withRun(r -> GuiSupport.open(panel, r.dir())));
    images.setToolTipText("Previews of the run's photographs, each one large, and a map of them projected to real-world coordinates.");
    images.addActionListener(e -> withRun(r -> ImageWindow.open(panel, r)));
    view3d.setToolTipText("The dense cloud, tie points and cameras of the run in 3D, coloured by RGB, height, class or point attributes.");
    view3d.addActionListener(e -> withRun(r -> CloudWindow.open(panel, r)));
    copyRun.setToolTipText("Copy the run's status, failed step, error, arguments and the end of its log (for debugging).");
    copyRun.addActionListener(e -> withRun(r -> GuiSupport.textDialog(panel, "Run " + r.id(), ProjectFiles.runDetails(r))));
    copyCommand.setToolTipText("<html>Copy the run's options as one command line (options given later replace earlier ones),<br>e.g. to run it again on a server. Adjust the paths there.</html>");
    copyCommand.addActionListener(e -> withRun(r -> {
      GuiSupport.copy(ProjectFiles.command(r));
      copyCommand.setText("Copied");
      javax.swing.Timer t = new javax.swing.Timer(1500, a -> copyCommand.setText("Copy command"));
      t.setRepeats(false);
      t.start();
    }));
    lidar.setToolTipText("The lidar referencing of the run (--lidar-reference): transformation, uncertainty, coverage and difference maps.");
    lidar.addActionListener(e -> withRun(r -> LidarResultWindow.open(panel, r, () -> branchRun(r, List.of("lidar", "finish"), ProjectFiles.lidarOptions(r)))));
    registration.setToolTipText("How the images were registered, why the others were not, and the GPS map (04_geometry/registration.json).");
    registration.addActionListener(e -> withRun(r -> RegistrationWindow.open(panel, r,
        () -> branchRun(r, List.of("align", "finish"), "--registration-recovery on"),
        () -> branchRun(r, List.of("match", "finish"), "--match-repair on --registration-recovery on"))));
    updateButtons();
  }

  static void widths(JTable t, int... w) {
    for (int i = 0; i < w.length; i++) t.getColumnModel().getColumn(i).setPreferredWidth(w[i]);
  }

  ProjectFiles.Run selectedRun() {
    int row = runTable.getSelectedRow();
    return row < 0 || row >= runs.size() ? null : runs.get(row);
  }

  void withRun(java.util.function.Consumer<ProjectFiles.Run> action) {
    ProjectFiles.Run r = selectedRun();
    if (r != null) action.accept(r);
  }

  List<String> selectedSteps() {
    List<String> out = new ArrayList<>();
    for (int row : stepTable.getSelectedRows()) out.add(ProjectFiles.STEPS.get(row));
    return out;
  }

  /** Re-reads the project folder, keeping the selected run. */
  void refresh() {
    Path root = window.project();
    String keep = selectedRun() == null ? null : selectedRun().id();
    runs = root == null || !ProjectFiles.isProject(root) ? List.of() : ProjectFiles.runs(root);
    filling = true;
    try {
      runModel.setRowCount(0);
      int select = runs.isEmpty() ? -1 : 0;
      for (int i = 0; i < runs.size(); i++) {
        ProjectFiles.Run r = runs.get(i);
        List<String> rest = r.remaining();
        runModel.addRow(new Object[] {r.id(), ProjectFiles.local(r.started()), r.status(), r.points() == null ? "" : String.format(Locale.ROOT, "%,d", r.points()), rest.isEmpty() ? "" : rest.get(0)});
        if (r.id().equals(keep)) select = i;
      }
      if (select >= 0) runTable.setRowSelectionInterval(select, select);
    } finally {
      filling = false;
    }
    showRun();
  }

  void showRun() {
    ProjectFiles.Run r = selectedRun();
    stepModel.setRowCount(0);
    if (r == null) {
      Path root = window.project();
      summary.setText(
          "<html><body>"
              + (root == null ? "Choose a project folder above." : ProjectFiles.isProject(root) ? "No runs yet." : "Not a POSE project yet: start a run on the New run tab.")
              + "</body></html>");
      updateButtons();
      return;
    }
    for (ProjectFiles.Step s : r.steps())
      stepModel.addRow(new Object[] {s.name(), s.status(), ProjectFiles.local(s.started()), ProjectFiles.local(s.finished()), ProjectFiles.duration(s.duration()), s.detail()});
    summary.setText(summaryHtml(r));
    summary.setCaretPosition(0);
    updateButtons();
  }

  /** The full message of a step (and, when it failed, the log's context) in a copyable window. */
  void stepDetails(int row) {
    ProjectFiles.Run r = selectedRun();
    if (r == null || row < 0 || row >= r.steps().size()) return;
    GuiSupport.textDialog(panel, "Step " + r.steps().get(row).name() + " of run " + r.id(), ProjectFiles.stepDetails(r, r.steps().get(row)));
  }

  /** Selected rows of a table as tab-separated text. */
  static String selectedRowsText(JTable t) {
    StringBuilder s = new StringBuilder();
    for (int row : t.getSelectedRows()) {
      for (int c = 0; c < t.getColumnCount(); c++) s.append(c == 0 ? "" : "\t").append(t.getValueAt(row, c));
      s.append('\n');
    }
    return s.toString();
  }

  /** Camera groups of a run: {ID, "label; N images; f px; initialization, policy"}; empty if unknown. */
  static List<String[]> cameraGroups(java.nio.file.Path run) {
    List<String[]> rows = new java.util.ArrayList<>();
    java.nio.file.Path file = run == null ? null : run.resolve("01_calibration/cameras.json");
    if (file == null || !java.nio.file.Files.isRegularFile(file)) return rows;
    try {
      for (Object item : JsonRead.array(JsonRead.object(JsonRead.read(file)).get("cameras"))) {
        var g = JsonRead.object(item);
        var c = g.get("calibration") == null ? null : JsonRead.object(g.get("calibration"));
        rows.add(
            new String[] {
              String.valueOf(g.get("id")),
              g.get("label")
                  + "; "
                  + ((Number) g.get("images")).intValue()
                  + " images"
                  + (c == null ? "" : String.format(java.util.Locale.ROOT, "; f %.1f px", JsonRead.number(c.get("fx"))))
                  + "; "
                  + String.valueOf(g.get("initialization")).replace('_', ' ')
                  + ", "
                  + g.get("policy")
            });
      }
    } catch (java.io.IOException | RuntimeException e) {
      rows.clear();
    }
    return rows;
  }

  static String summaryHtml(ProjectFiles.Run r) {
    StringBuilder h = new StringBuilder("<html><body style='font-family:sans-serif'>");
    h.append("<b>Run ").append(GuiOptions.escape(r.id())).append("</b> - ").append(r.status()).append("<br>");
    String error = ProjectFiles.runError(r.dir());
    if (error != null && !r.status().equals("complete")) h.append("<p><b>Error:</b> <font color='#dc2626'>").append(GuiOptions.escape(error)).append("</font></p>");
    if (r.summary().isEmpty()) h.append("<p>No processing report yet (it is written by the finish step).</p>");
    else {
      h.append("<table>");
      for (String[] row : r.summary()) h.append("<tr><td>").append(GuiOptions.escape(row[0])).append("</td><td><b>").append(GuiOptions.escape(row[1])).append("</b></td></tr>");
      h.append("</table>");
    }
    List<String[]> cameras = cameraGroups(r.dir());
    if (!cameras.isEmpty()) {
      // Read-only: the camera groups of this run (01_calibration/cameras.json).
      h.append("<p><b>Cameras</b><table>");
      for (String[] c : cameras)
        h.append("<tr><td>").append(GuiOptions.escape(c[0])).append("</td><td>").append(GuiOptions.escape(c[1])).append("</td></tr>");
      h.append("</table></p>");
    }
    h.append("<p><b>Quality flags</b>");
    if (r.flags().isEmpty()) h.append(r.summary().isEmpty() ? "<br>-" : "<br>None.");
    else {
      h.append("<ul>");
      for (String f : r.flags()) h.append("<li><font color='#b45309'>").append(GuiOptions.escape(f)).append("</font></li>");
      h.append("</ul>");
    }
    h.append("</p><p><b>Arguments</b><br><tt>").append(GuiOptions.escape(String.join(" ", r.arguments()))).append("</tt></p>");
    return h.append("</body></html>").toString();
  }

  List<String> extraArgs() {
    ProjectFiles.Run r = selectedRun();
    return GuiOptions.withImplied(
        r == null ? List.of() : r.arguments(), GuiOptions.split(extra.getText()));
  }

  /** Validates the saved arguments of the run plus the extra options, as the CLI would. */
  String checkExtra() {
    ProjectFiles.Run r = selectedRun();
    String problem = null;
    try {
      List<String> extraArgs = extraArgs();
      if (r != null && !extraArgs.isEmpty()) problem = GuiOptions.validateContinued(r.arguments(), extraArgs);
    } catch (IllegalArgumentException e) {
      problem = e.getMessage();
    }
    extraCheck.setText(problem == null ? " " : problem);
    extraCheck.setForeground(PoseWindow.FAILED);
    return problem;
  }

  void continueRun(ProjectFiles.Run r, List<String> steps) {
    if (steps.isEmpty()) return;
    String problem = checkExtra();
    if (problem != null) {
      JOptionPane.showMessageDialog(panel, problem, "Check the options", JOptionPane.WARNING_MESSAGE);
      return;
    }
    window.run("Run " + r.id() + ": " + String.join(" ", steps), ProjectFiles.continueSteps(window.project(), r.id(), steps, extraArgs()), window.project(), r.id(), null);
  }

  /**
   * Redo steps in a new run: copied from this run as it was just before the first step (nothing
   * earlier runs again), the chosen steps redone with the options given, later steps left
   * pending to be run (or not) afterwards. The source run is not changed.
   */
  void branchRun(ProjectFiles.Run r) {
    branchRun(r, selectedSteps(), extra.getText());
  }

  /** The redo dialog with these steps (from, until) and these options already filled in. */
  void branchRun(ProjectFiles.Run r, List<String> chosen, String startOptions) {
    List<String> allowed = ProjectFiles.branchable(r);
    if (allowed.isEmpty()) {
      JOptionPane.showMessageDialog(panel, "No step of this run can be redone yet: the steps before it must be complete.", "Redo steps", JOptionPane.INFORMATION_MESSAGE);
      return;
    }
    String from = !chosen.isEmpty() && allowed.contains(chosen.get(0)) ? chosen.get(0) : allowed.contains("dense") ? "dense" : allowed.get(allowed.size() - 1);
    String until = chosen.size() > 1 && ProjectFiles.STEPS.indexOf(chosen.get(chosen.size() - 1)) >= ProjectFiles.STEPS.indexOf(from) ? chosen.get(chosen.size() - 1) : from;
    JComboBox<String> fromBox = new JComboBox<>(allowed.toArray(String[]::new)), untilBox = new JComboBox<>();
    fromBox.setSelectedItem(from);
    JTextField options = new JTextField(startOptions, 40);
    JTextArea preview = new JTextArea(3, 60);
    preview.setEditable(false);
    preview.setLineWrap(true);
    preview.setFont(new Font(Font.MONOSPACED, Font.PLAIN, preview.getFont().getSize()));
    JLabel check = new JLabel(" "), note = new JLabel(" ");
    Runnable fillUntil =
        () -> {
          String keep = (String) untilBox.getSelectedItem();
          untilBox.removeAllItems();
          for (String s : ProjectFiles.STEPS.subList(ProjectFiles.STEPS.indexOf((String) fromBox.getSelectedItem()), ProjectFiles.STEPS.size())) untilBox.addItem(s);
          untilBox.setSelectedItem(keep != null && ProjectFiles.STEPS.indexOf(keep) >= ProjectFiles.STEPS.indexOf((String) fromBox.getSelectedItem()) ? keep : fromBox.getSelectedItem());
        };
    String[] problem = new String[1];
    List<String>[] command = new List[1];
    Runnable update =
        () -> {
          if (untilBox.getSelectedItem() == null) return;
          String f = (String) fromBox.getSelectedItem(), u = (String) untilBox.getSelectedItem();
          try {
            List<String> extraArgs = GuiOptions.withImplied(r.arguments(), GuiOptions.split(options.getText()));
            command[0] = ProjectFiles.branch(window.project(), r.id(), f, u, extraArgs);
            problem[0] = extraArgs.isEmpty() ? null : GuiOptions.validateContinued(r.arguments(), extraArgs);
          } catch (IllegalArgumentException e) {
            command[0] = null;
            problem[0] = e.getMessage();
          }
          preview.setText(command[0] == null ? "" : PoseProcess.display(command[0], PoseProcess.windows()));
          check.setText(problem[0] == null ? "Options are valid." : problem[0]);
          check.setForeground(problem[0] == null ? PoseWindow.COMPLETE : PoseWindow.FAILED);
          List<String> later = ProjectFiles.STEPS.subList(ProjectFiles.STEPS.indexOf(u) + 1, ProjectFiles.STEPS.size());
          note.setText(
              "<html>Steps before <b>" + f + "</b> are copied from run " + r.id() + " (not run again)."
                  + (later.isEmpty() ? "" : "<br>Afterwards " + String.join(", ", later) + " stay pending in the new run: run them later with Run remaining steps.")
                  + "<br>The run " + r.id() + " itself is not changed.</html>");
        };
    fillUntil.run();
    untilBox.setSelectedItem(until);
    fromBox.addActionListener(e -> {
      fillUntil.run();
      update.run();
    });
    untilBox.addActionListener(e -> update.run());
    options.getDocument().addDocumentListener(
        new javax.swing.event.DocumentListener() {
          public void insertUpdate(javax.swing.event.DocumentEvent e) {
            update.run();
          }

          public void removeUpdate(javax.swing.event.DocumentEvent e) {
            update.run();
          }

          public void changedUpdate(javax.swing.event.DocumentEvent e) {
            update.run();
          }
        });
    options.setToolTipText("<html>Options for the redone steps (and the later ones of the new run), e.g. <tt>--features 60000 --ratio 0.8</tt>.<br>Hover the fields of the New run tab for help, or see 'photogrammetry --help'.</html>");
    update.run();
    JPanel form = new JPanel(new GridBagLayout());
    GridBagConstraints c = new GridBagConstraints();
    c.insets = new Insets(3, 4, 3, 4);
    c.anchor = GridBagConstraints.WEST;
    Object[][] rows = {{"Redo from step:", fromBox}, {"up to step:", untilBox}, {"With options:", options}};
    for (int i = 0; i < rows.length; i++) {
      c.gridy = i;
      c.gridx = 0;
      c.fill = GridBagConstraints.NONE;
      c.weightx = 0;
      form.add(new JLabel((String) rows[i][0]), c);
      c.gridx = 1;
      c.fill = GridBagConstraints.HORIZONTAL;
      c.weightx = 1;
      form.add((Component) rows[i][1], c);
    }
    c.gridx = 0;
    c.gridwidth = 2;
    c.gridy = rows.length;
    form.add(note, c);
    c.gridy++;
    form.add(new JScrollPane(preview), c);
    c.gridy++;
    form.add(check, c);
    while (true) {
      int a = JOptionPane.showConfirmDialog(panel, form, "Redo steps of run " + r.id() + " in a new run", JOptionPane.OK_CANCEL_OPTION, JOptionPane.PLAIN_MESSAGE);
      if (a != JOptionPane.OK_OPTION) return;
      if (problem[0] == null && command[0] != null) break;
      JOptionPane.showMessageDialog(panel, problem[0], "Check the options", JOptionPane.WARNING_MESSAGE);
    }
    String f = (String) fromBox.getSelectedItem(), u = (String) untilBox.getSelectedItem();
    window.run(
        "Redo " + (f.equals(u) ? f : f + " .. " + u) + " of " + r.id() + " in a new run",
        command[0],
        window.project(),
        null,
        exit -> {
          // Show the new run with its pending steps.
          if (exit == 0 && window.consoleRun != null) select(window.consoleRun);
        });
  }

  /** Selects a run in the table by id (after a refresh). */
  void select(String id) {
    refresh();
    for (int i = 0; i < runs.size(); i++)
      if (runs.get(i).id().equals(id)) {
        runTable.setRowSelectionInterval(i, i);
        runTable.scrollRectToVisible(runTable.getCellRect(i, 0, true));
      }
  }

  void updateButtons() {
    ProjectFiles.Run r = selectedRun();
    boolean idle = !window.busy() && r != null;
    boolean pipeline = r != null && !r.status().equals("unknown");
    List<String> rest = r == null ? List.of() : r.remaining();
    boolean waiting = r != null && r.waitingForGcp();
    next.setEnabled(idle && pipeline && !rest.isEmpty() && !waiting);
    remaining.setEnabled(idle && pipeline && !rest.isEmpty() && !waiting);
    selected.setEnabled(idle && pipeline && stepTable.getSelectedRowCount() > 0);
    branch.setEnabled(idle && pipeline);
    gcps.setEnabled(idle && waiting);
    tiePoints.setEnabled(idle && r != null && r.waitingForCleanup());
    lidar.setEnabled(r != null && !ProjectFiles.lidar(r).isEmpty());
    registration.setEnabled(RegistrationWindow.available(r));
    copyCommand.setEnabled(r != null && !r.arguments().isEmpty());
    report.setEnabled(idle && r != null && Files.isRegularFile(r.dir().resolve("run.json")));
    ortho.setEnabled(idle && r != null && Files.isRegularFile(r.dir().resolve("07_results/coordinates.json")));
    pdf.setEnabled(r != null && Files.isRegularFile(r.report("pdf")));
    html.setEnabled(r != null && Files.isRegularFile(r.report("html")));
    results.setEnabled(r != null && Files.isDirectory(r.results()));
    folder.setEnabled(r != null);
    images.setEnabled(r != null);
    view3d.setEnabled(r != null);
    copyRun.setEnabled(r != null);
  }

  /** Colours the status column. */
  static final class StatusRenderer extends DefaultTableCellRenderer {
    final int column;

    StatusRenderer(int column) {
      this.column = column;
    }

    @Override
    public Component getTableCellRendererComponent(JTable t, Object value, boolean selected, boolean focus, int row, int col) {
      Component c = super.getTableCellRendererComponent(t, value, selected, focus, row, col);
      if (!selected) {
        Color color = null;
        if (col == column)
          color =
              switch (String.valueOf(value)) {
                case "complete" -> PoseWindow.COMPLETE;
                case "running" -> PoseWindow.RUNNING;
                case "waiting", "incomplete" -> PoseWindow.WAITING;
                case "failed" -> PoseWindow.FAILED;
                default -> null;
              };
        c.setForeground(color == null ? t.getForeground() : color);
      }
      return c;
    }
  }
}
