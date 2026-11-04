package sgm;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.util.*;

/**
 * Terminal presentation of a run. With {@code --verbose} every log line is shown, as before.
 * Otherwise the terminal shows one progress line per processing stage (named after the stage, with
 * percent, counts, elapsed and estimated remaining time, the current item and live resources),
 * a one-line summary as each stage ends, and warnings/errors above the bar. {@code
 * logs/console.log} always receives the full verbose text; only the terminal view changes.
 */
final class Terminal {
  private static final String[][] STAGES = {
    {"00_inputs", "Reading images"},
    {"01_calibration", "Calibrating camera"},
    {"02_features", "Detecting features"},
    {"03_matches", "Matching image pairs"},
    {"04_geometry", "Aligning cameras"},
    {"05_stereo", "Dense matching"},
    {"06_fusion", "Fusing depth maps"},
    {"07_results", "Writing results"}
  };

  private static volatile boolean verbose = true;
  private static volatile boolean interactive;
  private static PrintStream screen;
  private static final List<WorkProgress> active = new ArrayList<>();
  private static String stage = "", stageName = "Starting";
  private static int stageIndex;
  private static long stageStarted = System.nanoTime();
  private static int shown;
  private static long lastPlain;
  private static Thread renderer;

  private Terminal() {}

  /** Configures the view for a run; call before the log capture starts. */
  static synchronized void configure(boolean verboseOutput, PrintStream terminal) {
    verbose = verboseOutput;
    screen = terminal;
    Console console = System.console();
    interactive = console != null && console.isTerminal();
    if (!verbose && renderer == null) {
      renderer =
          new Thread(
              () -> {
                while (!Thread.currentThread().isInterrupted()) {
                  try {
                    Thread.sleep(interactive ? 500 : 1000);
                  } catch (InterruptedException e) {
                    return;
                  }
                  render();
                }
              },
              "terminal-progress");
      renderer.setDaemon(true);
      renderer.start();
    }
  }

  /** Ends the view: clears the progress line. */
  static synchronized void stop() {
    if (renderer != null) renderer.interrupt();
    renderer = null;
    clear();
    verbose = true;
  }

  static boolean verbose() {
    return verbose;
  }

  /**
   * The terminal side of the console tee. Lines are decoded as UTF-8 (the log's encoding) and
   * printed through the console's own encoding, so stage labels no longer show as "ΓÇö".
   */
  static OutputStream sink(PrintStream target, boolean errors) {
    return new OutputStream() {
      private final ByteArrayOutputStream line = new ByteArrayOutputStream();

      @Override
      public synchronized void write(int b) {
        if (b == '\n') {
          String text = line.toString(StandardCharsets.UTF_8);
          line.reset();
          if (text.endsWith("\r")) text = text.substring(0, text.length() - 1);
          Terminal.line(target, text, errors);
        } else line.write(b);
      }

      @Override
      public synchronized void write(byte[] bytes, int offset, int length) {
        for (int i = offset; i < offset + length; i++) write(bytes[i]);
      }

      @Override
      public synchronized void flush() {
        target.flush();
      }
    };
  }

  static void line(PrintStream target, String text, boolean errors) {
    String plain = ascii(text);
    if (verbose) {
      target.println(plain);
      return;
    }
    if (errors || important(text)) above(target, plain);
  }

  /** Lines worth showing without --verbose: warnings, errors, failures and fallbacks. */
  static boolean important(String text) {
    String t = text.toLowerCase(Locale.ROOT);
    if (t.contains("stopped before completion") && !t.startsWith("stage ")) return false;
    // Summaries that merely report zero failures.
    if (t.contains("failed images=[]") || t.matches(".*\\bfailed[ =:]+0\\b.*")) return false;
    return t.startsWith("warning")
        || t.contains(" warning")
        || t.contains("error")
        || t.contains("failed")
        || t.contains("exception")
        || t.contains("skipping")
        || t.contains("overlapping strips")
        || t.startsWith("camera screening")
        || t.startsWith("gps-weighted bundle")
        || t.startsWith("photogrammetry")
        || t.startsWith("project:")
        || t.startsWith("run:")
        || t.startsWith("stage ") && t.contains("stopped before completion");
  }

  static String ascii(String text) {
    return text.replace('—', '-').replace('–', '-').replace("…", "...").replace('−', '-');
  }

  static void started(WorkProgress task) {
    if (verbose) return;
    synchronized (active) {
      active.add(task);
    }
  }

  static void finished(WorkProgress task) {
    if (verbose) return;
    synchronized (active) {
      active.remove(task);
    }
  }

  /** Stage transitions from Project: running stages title the bar; ends print one summary line. */
  static synchronized void stage(String name, String status, String detail) {
    if (verbose || screen == null) return;
    int index = 0;
    String friendly = name;
    for (int i = 0; i < STAGES.length; i++)
      if (STAGES[i][0].equals(name)) {
        index = i + 1;
        friendly = STAGES[i][1];
      }
    if (status.equals("running")) {
      if (!name.equals(stage)) stageStarted = System.nanoTime();
      stage = name;
      stageName = friendly;
      stageIndex = index;
      return;
    }
    String mark = status.equals("complete") ? "done" : status;
    String time = name.equals(stage) ? "  " + clock((System.nanoTime() - stageStarted) / 1e9) : "";
    above(
        screen,
        String.format(
            Locale.ROOT,
            "[%d/%d] %-22s %s%s%s",
            index,
            STAGES.length,
            friendly,
            mark,
            time,
            detail == null || detail.isBlank() ? "" : "  - " + ascii(shorten(detail, 90))));
  }

  /** Prints a line above the progress bar (clears it first, redraws next tick). */
  private static synchronized void above(PrintStream target, String text) {
    clear();
    target.println(text);
    target.flush();
  }

  private static synchronized void clear() {
    if (screen != null && interactive && shown > 0) {
      screen.print("\r" + " ".repeat(shown) + "\r");
      screen.flush();
    }
    shown = 0;
  }

  private static synchronized void render() {
    if (verbose || screen == null) return;
    String text = progressLine(System.nanoTime());
    int width = width();
    if (text.length() > width - 1) text = text.substring(0, width - 1);
    if (interactive) {
      screen.print("\r" + text + (shown > text.length() ? " ".repeat(shown - text.length()) : ""));
      screen.flush();
      shown = text.length();
    } else if (System.nanoTime() - lastPlain > 15_000_000_000L) {
      screen.println(text);
      lastPlain = System.nanoTime();
    }
  }

  /** The bar: the latest started task with a known total sets percent, counts and ETA. */
  static String progressLine(long now) {
    WorkProgress measured = null, latest = null;
    synchronized (active) {
      for (WorkProgress w : active) {
        latest = w;
        if (w.total() > 0) measured = w;
      }
    }
    StringBuilder s = new StringBuilder();
    if (stageIndex > 0) s.append('[').append(stageIndex).append('/').append(STAGES.length).append("] ");
    s.append(stageName);
    if (measured != null) {
      long done = Math.min(measured.count(), measured.total()), total = measured.total();
      double fraction = done / (double) total, elapsed = (now - measured.startedNanos()) / 1e9;
      if (!sameTopic(measured.label(), stageName)) s.append(": ").append(ascii(measured.label()));
      s.append("  ").append(bar(fraction, 20));
      s.append(String.format(Locale.ROOT, " %3.0f%%  %,d/%,d %s", 100 * fraction, done, total, measured.units()));
      s.append("  ").append(clock((now - stageStarted) / 1e9));
      if (done > 0 && done < total) s.append("  ~").append(clock(elapsed * (total - done) / done)).append(" left");
      String detail = measured.detailText();
      if (detail != null && !detail.isBlank()) s.append("  ").append(ascii(shorten(detail, 28)));
    } else {
      s.append("  ").append(clock((now - stageStarted) / 1e9));
      if (latest != null) s.append("  ").append(ascii(shorten(latest.label(), 40)));
    }
    String live = RunTelemetry.live();
    if (!live.isEmpty()) s.append("  | ").append(live);
    return s.toString();
  }

  private static boolean sameTopic(String label, String stage) {
    return label.toLowerCase(Locale.ROOT).startsWith(stage.toLowerCase(Locale.ROOT));
  }

  static String bar(double fraction, int width) {
    int filled = (int) Math.round(Math.clamp(fraction, 0, 1) * width);
    return "[" + "#".repeat(filled) + "-".repeat(width - filled) + "]";
  }

  static String clock(double seconds) {
    long s = Math.max(0, Math.round(seconds));
    return s >= 3600
        ? String.format(Locale.ROOT, "%d:%02d:%02d", s / 3600, s / 60 % 60, s % 60)
        : String.format(Locale.ROOT, "%d:%02d", s / 60, s % 60);
  }

  private static String shorten(String text, int max) {
    return text.length() <= max ? text : text.substring(0, max - 3) + "...";
  }

  private static int width() {
    try {
      String columns = System.getenv("COLUMNS");
      if (columns != null) return Math.max(60, Integer.parseInt(columns.trim()));
    } catch (NumberFormatException ignored) {
    }
    return 120;
  }
}
