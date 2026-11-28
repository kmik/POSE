package sgm;

import java.awt.*;
import java.awt.event.*;
import java.io.IOException;
import java.nio.file.*;
import java.util.*;
import java.util.List;
import java.util.concurrent.*;
import java.util.function.IntConsumer;
import javax.swing.*;
import javax.swing.border.*;

/**
 * The POSE desktop window ({@code photogrammetry gui [PROJECT]}): a project folder bar, the "New
 * run" launcher and the "Project" manager tabs, and a console that shows the command running as a
 * child process (steps, stage progress, elapsed time, live log, Stop, results). Processing happens
 * only in the child process; this window reads its output and the run files.
 */
final class PoseWindow {
  /** Opens the window and blocks until it is closed. Exit code: 0, or 2 without a display. */
  static int start(String[] args) {
    if (GraphicsEnvironment.isHeadless()) {
      System.err.println(GuiSupport.HEADLESS);
      return 2;
    }
    Path project = null;
    for (int i = 0; i < args.length; i++)
      if ((args[i].equals("-o") || args[i].equals("--output")) && i + 1 < args.length) project = Path.of(args[++i]);
      else if (!args[i].startsWith("-")) project = Path.of(args[i]);
    GuiSupport.lookAndFeel();
    // The window's compute defaults follow this computer: detected once (a --list-gpus child,
    // about a second) before the first window, afterwards refreshed in the background for the
    // next start (a new GPU or more RAM).
    Path settingsDir = GuiOptions.Settings.user().dir;
    if (GuiPreferences.remembered(settingsDir) == null) GuiPreferences.detect(settingsDir);
    else {
      Thread refresh = new Thread(() -> GuiPreferences.detect(settingsDir), "pose-hardware");
      refresh.setDaemon(true);
      refresh.start();
    }
    CountDownLatch closed = new CountDownLatch(1);
    Path initial = project == null ? null : project.toAbsolutePath().normalize();
    try {
      SwingUtilities.invokeAndWait(() -> new PoseWindow(initial, closed::countDown));
      closed.await();
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    } catch (java.lang.reflect.InvocationTargetException e) {
      System.err.println("photogrammetry gui: " + e.getCause());
      return 1;
    }
    return 0;
  }

  static final Color RUNNING = new Color(0x2563eb), COMPLETE = new Color(0x16a34a), WAITING = new Color(0xca8a04), FAILED = new Color(0xdc2626);

  final JFrame frame = new JFrame("POSE");
  final GuiOptions.Settings settings = GuiOptions.Settings.user();
  /** Settings > Preferences; replaced when the dialog saves. */
  GuiPreferences preferences = GuiPreferences.load(settings.dir);
  final JComboBox<String> projectBox = new JComboBox<>();
  final JTabbedPane tabs = new JTabbedPane();
  final PoseLauncherPanel launcher;
  final PoseProjectPanel manager;
  // Console.
  final JLabel commandLabel = new JLabel("No command running."), elapsed = new JLabel(" ");
  final JButton stop = new JButton("Stop");
  final JProgressBar stageBar = new JProgressBar(0, 100);
  final JLabel[] stepLabels = new JLabel[ProjectFiles.STEPS.size()];
  final JTextArea log = new JTextArea();
  final JButton openPdf = new JButton("Report PDF"), openHtml = new JButton("Report HTML"), openResults = new JButton("07_results"), openProject = new JButton("Project folder"), editGcps = new JButton("Edit GCPs"), cleanTiePoints = new JButton("Clean tie points");
  final ConcurrentLinkedQueue<String> pending = new ConcurrentLinkedQueue<>();
  final javax.swing.Timer tick = new javax.swing.Timer(250, e -> tick());
  ChildRun active;
  String activeLabel;
  IntConsumer afterActive;
  /** The run the console shows (the active command's, or the last one's). */
  Path consoleProject;
  String consoleRun;
  long lastStepRead;
  /** Runs of the project before the command started: a new id is the command's new run. */
  List<String> runsBefore = List.of();

  PoseWindow(Path project, Runnable onClose) {
    frame.setDefaultCloseOperation(WindowConstants.DO_NOTHING_ON_CLOSE);
    GuiSupport.brand(frame);
    frame.setJMenuBar(new JMenuBar());
    frame.getJMenuBar().add(PoseSettings.menu(this));
    frame.getJMenuBar().add(PoseUltraCamDialog.menu(this));
    frame.getJMenuBar().add(PoseHelp.menu(frame));
    frame.addWindowListener(
        new WindowAdapter() {
          @Override
          public void windowClosing(WindowEvent e) {
            if (active != null && active.alive()) {
              if (!preferences.confirmClose) {
                active.stop();
                tick.stop();
                frame.dispose();
                onClose.run();
                return;
              }
              int a = JOptionPane.showConfirmDialog(frame, "A command is running:\n" + activeLabel + "\n\nStop it and close POSE?", "POSE", JOptionPane.YES_NO_OPTION, JOptionPane.WARNING_MESSAGE);
              if (a != JOptionPane.YES_OPTION) return;
              active.stop();
            }
            tick.stop();
            frame.dispose();
            onClose.run();
          }
        });

    // Project bar.
    projectBox.setEditable(true);
    for (String p : settings.recent()) projectBox.addItem(p);
    projectBox.setSelectedItem(project == null ? (projectBox.getItemCount() > 0 ? projectBox.getItemAt(0) : "") : project.toString());
    projectBox.addActionListener(e -> projectChanged());
    projectBox.setToolTipText(GuiOptions.tooltip("-o"));
    JButton browse = new JButton("Browse...");
    browse.addActionListener(
        e -> {
          Path p = chooseFolder("Project folder", project());
          if (p != null) projectBox.setSelectedItem(p.toString());
        });
    JButton openFolder = new JButton("Open");
    openFolder.addActionListener(e -> {
      if (project() != null) GuiSupport.open(frame, project());
    });
    JPanel bar = new JPanel(new BorderLayout(6, 0));
    bar.setBorder(new EmptyBorder(8, 8, 4, 8));
    JLabel projectLabel = new JLabel("Project folder:");
    projectLabel.setFont(projectLabel.getFont().deriveFont(Font.BOLD));
    bar.add(projectLabel, BorderLayout.WEST);
    bar.add(projectBox, BorderLayout.CENTER);
    JPanel barButtons = new JPanel(new FlowLayout(FlowLayout.LEFT, 4, 0));
    barButtons.add(browse);
    barButtons.add(openFolder);
    bar.add(barButtons, BorderLayout.EAST);

    launcher = new PoseLauncherPanel(this);
    manager = new PoseProjectPanel(this);
    tabs.addTab("New run", launcher.panel);
    tabs.addTab("Advanced options", launcher.advancedOptions.panel);
    tabs.addTab("Project", manager.panel);
    launcher.advancedOptions.onCount = () -> {
      int n = launcher.advancedOptions.setCount(), at = tabs.indexOfComponent(launcher.advancedOptions.panel);
      if (at >= 0) tabs.setTitleAt(at, n == 0 ? "Advanced options" : "Advanced options (" + n + ")");
    };
    launcher.advancedOptions.refresh();
    tabs.addChangeListener(e -> {
      if (tabs.getSelectedComponent() == manager.panel) manager.refresh();
    });

    JSplitPane split = new JSplitPane(JSplitPane.VERTICAL_SPLIT, tabs, console());
    split.setResizeWeight(.72);
    JPanel content = new JPanel(new BorderLayout());
    content.add(bar, BorderLayout.NORTH);
    content.add(split, BorderLayout.CENTER);
    frame.setContentPane(content);
    Rectangle screen = GraphicsEnvironment.getLocalGraphicsEnvironment().getMaximumWindowBounds();
    frame.setSize(Math.min(screen.width, 1300), (int) (screen.height * .92));
    frame.setLocationRelativeTo(null);
    projectChanged();
    if (ProjectFiles.isProject(project())) tabs.setSelectedComponent(manager.panel);
    frame.setVisible(true);
    // The console keeps room for the step strip and a readable log; the tabs get the rest.
    split.setDividerLocation(Math.max(220, split.getHeight() - 280));
    tick.start();
  }

  /** The project folder in the bar (absolute), or null when empty. */
  Path project() {
    Object o = projectBox.getEditor().getItem();
    String s = o == null ? "" : o.toString().strip();
    if (s.isEmpty()) return null;
    try {
      return Path.of(s).toAbsolutePath().normalize();
    } catch (InvalidPathException e) {
      return null;
    }
  }

  void projectChanged() {
    Path p = project();
    frame.setTitle(p == null ? "POSE" : "POSE - " + p);
    launcher.load(settings.load(p));
    manager.refresh();
    if (!busy()) {
      // Idle: the console shows the project's newest run (steps, report buttons).
      List<ProjectFiles.Run> runs = p != null && ProjectFiles.isProject(p) ? ProjectFiles.runs(p) : List.of();
      consoleProject = p;
      consoleRun = runs.isEmpty() ? null : runs.get(0).id();
      commandLabel.setForeground(UIManager.getColor("Label.foreground"));
      commandLabel.setText(runs.isEmpty() ? "No command running." : "Newest run: " + runs.get(0).id() + " (" + runs.get(0).status() + ")");
      showSteps(runs.isEmpty() ? null : runs.get(0));
    }
    updateButtons();
  }

  Path chooseFolder(String title, Path start) {
    JFileChooser chooser = new JFileChooser(start != null && Files.isDirectory(start) ? start.toFile() : start != null && start.getParent() != null ? start.getParent().toFile() : null);
    chooser.setDialogTitle(title);
    chooser.setFileSelectionMode(JFileChooser.DIRECTORIES_ONLY);
    return chooser.showOpenDialog(frame) == JFileChooser.APPROVE_OPTION ? chooser.getSelectedFile().toPath() : null;
  }

  JComponent console() {
    log.setEditable(false);
    log.setFont(new Font(Font.MONOSPACED, Font.PLAIN, log.getFont().getSize()));
    JScrollPane scroll = new JScrollPane(log);
    stageBar.setStringPainted(true);
    stageBar.setString(" ");
    stop.setEnabled(false);
    stop.setToolTipText("End the running command and every process it started.");
    stop.addActionListener(e -> stopActive());
    JPanel head = new JPanel(new BorderLayout(8, 0));
    commandLabel.setFont(commandLabel.getFont().deriveFont(Font.BOLD));
    head.add(commandLabel, BorderLayout.CENTER);
    JPanel right = new JPanel(new FlowLayout(FlowLayout.RIGHT, 6, 0));
    right.add(elapsed);
    right.add(stop);
    head.add(right, BorderLayout.EAST);
    JPanel steps = new JPanel(new GridLayout(1, stepLabels.length, 3, 0));
    for (int i = 0; i < stepLabels.length; i++) {
      stepLabels[i] = new JLabel(ProjectFiles.STEPS.get(i), SwingConstants.CENTER);
      stepLabels[i].setOpaque(true);
      stepLabels[i].setBorder(new EmptyBorder(3, 2, 3, 2));
      steps.add(stepLabels[i]);
    }
    showSteps(null);
    JPanel results = new JPanel(new FlowLayout(FlowLayout.LEFT, 4, 0));
    results.add(new JLabel("Open:"));
    for (JButton b : List.of(openPdf, openHtml, openResults, openProject)) results.add(b);
    results.add(editGcps);
    results.add(cleanTiePoints);
    cleanTiePoints.setToolTipText("Choose the tie-point cleanup limits with the histograms, then continue the run.");
    cleanTiePoints.addActionListener(e -> cleanTiePoints(consoleProject, consoleRun));
    openPdf.addActionListener(e -> openRun(r -> r.report("pdf")));
    openHtml.addActionListener(e -> openRun(r -> r.report("html")));
    openResults.addActionListener(e -> openRun(ProjectFiles.Run::results));
    openProject.addActionListener(e -> {
      if (consoleProject != null) GuiSupport.open(frame, consoleProject);
      else if (project() != null) GuiSupport.open(frame, project());
    });
    editGcps.setToolTipText("Pin the GCP markers in the native editor, then continue the run.");
    editGcps.addActionListener(e -> editGcps(consoleProject, consoleRun));
    Box top = Box.createVerticalBox();
    for (JComponent c : List.of(head, steps, stageBar)) {
      c.setAlignmentX(Component.LEFT_ALIGNMENT);
      top.add(c);
      top.add(Box.createVerticalStrut(4));
    }
    JPanel panel = new JPanel(new BorderLayout(0, 4));
    panel.setBorder(new CompoundBorder(new EmptyBorder(4, 8, 8, 8), new TitledBorder("Console")));
    panel.add(top, BorderLayout.NORTH);
    panel.add(scroll, BorderLayout.CENTER);
    panel.add(results, BorderLayout.SOUTH);
    return panel;
  }

  void openRun(java.util.function.Function<ProjectFiles.Run, Path> what) {
    if (consoleProject == null || consoleRun == null) return;
    GuiSupport.open(frame, what.apply(ProjectFiles.run(consoleProject, consoleRun)));
  }

  boolean busy() {
    return active != null && active.alive();
  }

  /**
   * Starts a POSE command as a child process with the console following it. {@code after} runs
   * on the event thread with the exit code (not when stopped). False when another one runs.
   */
  boolean run(String label, List<String> args, Path project, String runId, IntConsumer after) {
    if (busy()) {
      JOptionPane.showMessageDialog(frame, "Another command is running:\n" + activeLabel + "\n\nWait for it or stop it first.", "POSE", JOptionPane.INFORMATION_MESSAGE);
      return false;
    }
    log.setText("");
    append("> " + PoseProcess.display(args, PoseProcess.windows()));
    activeLabel = label;
    afterActive = after;
    consoleProject = project;
    consoleRun = runId;
    runsBefore = project == null ? List.of() : ProjectFiles.runIds(project);
    commandLabel.setForeground(UIManager.getColor("Label.foreground"));
    elapsed.setText(" ");
    showSteps(runId == null ? null : ProjectFiles.run(project, runId));
    stageBar.setValue(0);
    stageBar.setString("Starting...");
    try {
      active =
          ChildRun.start(
              args,
              new ChildRun.Listener() {
                @Override
                public void line(String line) {
                  pending.add(line);
                }

                @Override
                public void finished(int exit, boolean stopped) {
                  SwingUtilities.invokeLater(() -> done(exit, stopped));
                }
              });
    } catch (IOException e) {
      GuiSupport.error(frame, "Cannot start POSE", e);
      return false;
    }
    commandLabel.setText(label);
    stop.setEnabled(true);
    updateButtons();
    return true;
  }

  void stopActive() {
    if (!busy()) return;
    stop.setEnabled(false);
    append("Stopping...");
    ChildRun r = active;
    new Thread(r::stop, "pose-stop").start();
  }

  void done(int exit, boolean stopped) {
    tick();
    ChildRun r = active;
    if (r != null && r.runId() != null) consoleRun = r.runId();
    if (r != null && r.project() != null) consoleProject = r.project();
    if (consoleRun == null && consoleProject != null) consoleRun = ProjectFiles.newRun(runsBefore, consoleProject);
    String time = ProjectFiles.duration(java.time.Duration.ofMillis(r == null ? 0 : r.elapsedMillis()));
    append(stopped ? "Stopped after " + time + "." : exit == 0 ? "Finished in " + time + "." : "Failed (exit code " + exit + ") after " + time + ".");
    commandLabel.setText(activeLabel + (stopped ? " - stopped" : exit == 0 ? " - finished" : " - failed (exit " + exit + ")"));
    commandLabel.setForeground(stopped || exit != 0 ? FAILED : COMPLETE);
    stop.setEnabled(false);
    ProjectFiles.Run run = consoleRun == null || consoleProject == null ? null : ProjectFiles.run(consoleProject, consoleRun);
    showSteps(run);
    stageBar.setValue(exit == 0 && !stopped ? 100 : stageBar.getValue());
    stageBar.setString(stopped ? "Stopped" : exit == 0 ? "Done" : "Failed");
    IntConsumer after = afterActive;
    afterActive = null;
    updateButtons();
    manager.refresh();
    if (!stopped && preferences.toFront) {
      if ((frame.getExtendedState() & Frame.ICONIFIED) != 0) frame.setExtendedState(frame.getExtendedState() & ~Frame.ICONIFIED);
      frame.toFront();
    }
    // Settings > Preferences: the report of a run that finished all its steps (not after a GCP edit
    // that continues the run, which ends in its own command).
    if (!stopped && exit == 0 && after == null && preferences.openReport && run != null && "complete".equals(run.status())) {
      Path report = Files.isRegularFile(run.report("pdf")) ? run.report("pdf") : run.report("html");
      if (Files.isRegularFile(report)) GuiSupport.open(frame, report);
    }
    if (!stopped && after != null) after.accept(exit);
    else if (!stopped && run != null && run.waitingForCleanup())
      SwingUtilities.invokeLater(
          () -> {
            int a = JOptionPane.showConfirmDialog(frame, "The run waits for the tie-point cleanup.\nChoose the limits now?", "POSE", JOptionPane.YES_NO_OPTION);
            if (a == JOptionPane.YES_OPTION) cleanTiePoints(consoleProject, consoleRun);
          });
    else if (!stopped && run != null && run.waitingForGcp())
      SwingUtilities.invokeLater(
          () -> {
            int a = JOptionPane.showConfirmDialog(frame, "The run waits for GCP markers.\nOpen the GCP editor now?", "POSE", JOptionPane.YES_NO_OPTION);
            if (a == JOptionPane.YES_OPTION) editGcps(consoleProject, consoleRun);
          });
  }

  /**
   * GCP markers: the native editor (a child process, so its photographs stay out of this window's
   * memory); when it closes normally, the run continues from the gcp step.
   */
  void editGcps(Path project, String runId) {
    if (project == null || runId == null) return;
    run(
        "Edit GCP markers of run " + runId,
        ProjectFiles.editGcps(project, runId),
        project,
        runId,
        exit -> {
          if (exit != 0) return;
          ProjectFiles.Run r = ProjectFiles.run(project, runId);
          List<String> rest = r.remaining();
          if (rest.isEmpty() || !rest.get(0).equals("gcp")) return;
          run("Continue run " + runId + ": " + String.join(" ", rest), ProjectFiles.continueSteps(project, runId, rest, List.of()), project, runId, null);
        });
  }

  /**
   * Tie-point cleanup (--tie-point-cleanup pause): the histograms window; its choice is written
   * into the run, and the run continues from the cleanup step, which applies or skips it.
   */
  void cleanTiePoints(Path project, String runId) {
    if (project == null || runId == null) return;
    ProjectFiles.Run r = ProjectFiles.run(project, runId);
    if (r == null || !r.waitingForCleanup()) return;
    TiePointWindow.open(frame, r, choice -> {
      try {
        TiePointCleanup.writeChoice(r.dir(), choice);
      } catch (IOException e) {
        GuiSupport.error(frame, "Tie-point cleanup", e);
        return;
      }
      List<String> rest = ProjectFiles.run(project, runId).remaining();
      run("Continue run " + runId + ": " + String.join(" ", rest), ProjectFiles.continueSteps(project, runId, rest, List.of()), project, runId, null);
    });
  }

  void tick() {
    StringBuilder text = new StringBuilder();
    String line;
    while ((line = pending.poll()) != null) text.append(line).append('\n');
    if (!text.isEmpty()) appendRaw(text.toString());
    ChildRun r = active;
    if (r == null) return;
    if (r.alive()) elapsed.setText("Elapsed " + ProjectFiles.duration(java.time.Duration.ofMillis(r.elapsedMillis())));
    if (r.runId() != null && consoleRun == null) consoleRun = r.runId();
    if (r.project() != null && (consoleProject == null || !consoleProject.equals(r.project()))) consoleProject = r.project();
    ProjectFiles.Progress p = r.progress();
    if (r.alive() && p != null && p.percent() == 100 && runningStep != null) {
      // The last stage line says done; the next step has started without a progress line yet.
      stageBar.setIndeterminate(true);
      stageBar.setString("Step " + runningStep + "...");
    } else if (r.alive() && p != null) {
      if (p.percent() >= 0) {
        stageBar.setIndeterminate(false);
        stageBar.setValue(p.percent());
        stageBar.setString(String.format(Locale.ROOT, "Stage %d/%d %s  %d%%", p.stage(), p.stages(), p.name(), p.percent()));
      } else {
        stageBar.setIndeterminate(true);
        stageBar.setString(String.format(Locale.ROOT, "Stage %d/%d %s", p.stage(), p.stages(), p.name()));
      }
    } else if (r.alive() && p == null) {
      stageBar.setIndeterminate(true);
      stageBar.setString("Working...");
    } else stageBar.setIndeterminate(false);
    if (r.alive() && consoleProject != null && System.currentTimeMillis() - lastStepRead > 1500) {
      lastStepRead = System.currentTimeMillis();
      if (consoleRun == null) consoleRun = ProjectFiles.newRun(runsBefore, consoleProject);
      if (consoleRun != null) showSteps(ProjectFiles.run(consoleProject, consoleRun));
    }
  }

  /** The step marked running in the run the console shows, or null. */
  String runningStep;

  void showSteps(ProjectFiles.Run run) {
    runningStep = null;
    for (int i = 0; i < stepLabels.length; i++) {
      ProjectFiles.Step s = run == null ? null : run.steps().get(i);
      String status = s == null ? "pending" : s.status();
      if (status.equals("running")) runningStep = s.name();
      Color c =
          switch (status) {
            case "running" -> RUNNING;
            case "complete" -> COMPLETE;
            case "waiting" -> WAITING;
            case "failed" -> FAILED;
            default -> null;
          };
      JLabel l = stepLabels[i];
      l.setBackground(c == null ? (status.equals("skipped") ? new Color(0xe5e7eb) : new Color(0xf3f4f6)) : c);
      l.setForeground(c == null ? Color.DARK_GRAY : Color.WHITE);
      String time = s == null ? "" : ProjectFiles.duration(s.duration());
      l.setText("<html><center>" + ProjectFiles.STEPS.get(i) + "<br><small>" + status + (time.isEmpty() ? "" : " " + time) + "</small></center></html>");
      l.setToolTipText(s == null ? null : "<html>" + GuiOptions.escape(s.detail()) + (s.started() == null ? "" : "<br>started " + ProjectFiles.local(s.started())) + (s.finished() == null ? "" : "<br>finished " + ProjectFiles.local(s.finished())) + "</html>");
    }
  }

  void updateButtons() {
    boolean have = consoleProject != null && consoleRun != null;
    ProjectFiles.Run run = have ? ProjectFiles.run(consoleProject, consoleRun) : null;
    openPdf.setEnabled(run != null && Files.isRegularFile(run.report("pdf")));
    openHtml.setEnabled(run != null && Files.isRegularFile(run.report("html")));
    openResults.setEnabled(run != null && Files.isDirectory(run.results()));
    openProject.setEnabled(consoleProject != null || project() != null);
    editGcps.setVisible(run != null && run.waitingForGcp() && !busy());
    cleanTiePoints.setVisible(run != null && run.waitingForCleanup() && !busy());
    launcher.updateButtons();
    manager.updateButtons();
  }

  void append(String line) {
    appendRaw(line + "\n");
  }

  void appendRaw(String text) {
    boolean atEnd = log.getCaretPosition() >= log.getDocument().getLength() - 1;
    log.append(text);
    int length = log.getDocument().getLength();
    if (length > 2_000_000)
      try {
        log.getDocument().remove(0, length - 1_000_000);
      } catch (javax.swing.text.BadLocationException e) {
        // keep
      }
    if (atEnd || busy()) log.setCaretPosition(log.getDocument().getLength());
  }
}
