package sgm;

import java.io.*;
import java.lang.management.ManagementFactory;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.*;
import java.util.function.Consumer;

/**
 * Starts the POSE command line as a child process: the same java, the same class path and the
 * same JVM options as this process (so the heap given by the launcher script applies), entry
 * point {@code photogrammetry}, which goes through {@link MemoryLauncher} as on the command line
 * (the memory re-execution still applies). Processing code is not touched. No window here.
 */
final class PoseProcess {
  private PoseProcess() {}

  /** JVM options of this process that must not be passed on (debuggers, agents, the re-exec flag). */
  static boolean inherited(String option) {
    return !(option.startsWith("-Dsgm.heap.reexecuted=")
        || option.startsWith("-agentlib:")
        || option.startsWith("-agentpath:")
        || option.startsWith("-javaagent:")
        || option.startsWith("-Xrunjdwp")
        || option.equals("-Xdebug")
        || option.startsWith("-Dstdout.encoding=")
        || option.startsWith("-Dstderr.encoding=")
        || option.startsWith("-Dpose.gui"));
  }

  static Path java() {
    boolean windows = System.getProperty("os.name").toLowerCase(Locale.ROOT).contains("win");
    return Path.of(System.getProperty("java.home"), "bin", windows ? "java.exe" : "java");
  }

  /**
   * Full child command for the given CLI arguments. With {@code piped}, the child's console
   * streams are UTF-8 so non-ASCII paths survive the pipe.
   */
  static List<String> command(List<String> args, boolean piped) {
    List<String> command = new ArrayList<>();
    command.add(java().toString());
    for (String option : ManagementFactory.getRuntimeMXBean().getInputArguments()) if (inherited(option)) command.add(option);
    if (piped) {
      command.add("-Dstdout.encoding=UTF-8");
      command.add("-Dstderr.encoding=UTF-8");
    }
    command.add("-cp");
    command.add(System.getProperty("java.class.path"));
    command.add("photogrammetry");
    command.addAll(args);
    return command;
  }

  /**
   * Starts the CLI with these arguments. Without a sink the child shares this console; with one,
   * stdout and stderr are merged and passed to the sink line by line (on a daemon thread; the
   * sink gets null once the stream ends).
   */
  static Process start(List<String> args, Consumer<String> sink) throws IOException {
    ProcessBuilder builder = new ProcessBuilder(command(args, sink != null));
    if (sink == null) return builder.inheritIO().start();
    builder.redirectErrorStream(true);
    builder.redirectInput(ProcessBuilder.Redirect.PIPE);
    Process process = builder.start();
    process.getOutputStream().close();
    Thread reader =
        new Thread(
            () -> {
              try (BufferedReader in = new BufferedReader(new InputStreamReader(process.getInputStream(), StandardCharsets.UTF_8))) {
                lines(in, sink);
              } catch (IOException e) {
                sink.accept("[output closed: " + e.getMessage() + "]");
              } finally {
                sink.accept(null);
              }
            },
            "pose-child-output");
    reader.setDaemon(true);
    reader.start();
    return process;
  }

  /**
   * Splits console output into lines; a carriage return (progress bars redraw the line) also ends
   * a line, so the latest state of a progress line is passed on.
   */
  static void lines(Reader in, Consumer<String> sink) throws IOException {
    StringBuilder line = new StringBuilder();
    int c;
    boolean cr = false;
    while ((c = in.read()) >= 0) {
      if (c == '\n') {
        if (!cr || line.length() > 0) sink.accept(line.toString());
        line.setLength(0);
        cr = false;
      } else if (c == '\r') {
        if (line.length() > 0) sink.accept(line.toString());
        line.setLength(0);
        cr = true;
      } else {
        line.append((char) c);
        cr = false;
      }
    }
    if (line.length() > 0) sink.accept(line.toString());
  }

  /**
   * Stops a child and everything it started (the memory re-execution starts a second JVM):
   * descendants first, then the child; forcibly after {@code graceMillis}.
   */
  static void terminate(ProcessHandle root, long graceMillis) {
    List<ProcessHandle> tree = new ArrayList<>(root.descendants().toList());
    Collections.reverse(tree); // deepest first
    tree.add(root);
    for (ProcessHandle p : tree) p.destroy();
    long end = System.currentTimeMillis() + graceMillis;
    for (ProcessHandle p : tree) {
      try {
        long left = Math.max(1, end - System.currentTimeMillis());
        p.onExit().get(left, java.util.concurrent.TimeUnit.MILLISECONDS);
      } catch (Exception e) {
        // Still running: forced below.
      }
    }
    for (ProcessHandle p : tree) if (p.isAlive()) p.destroyForcibly();
  }

  /** The CLI command as a user would type it (quoted for the given shell). */
  static String display(List<String> args, boolean windows) {
    StringBuilder out = new StringBuilder("photogrammetry");
    for (String a : args) out.append(' ').append(quote(a, windows));
    return out.toString();
  }

  static String quote(String a, boolean windows) {
    if (!a.isEmpty() && a.chars().noneMatch(ch -> Character.isWhitespace(ch) || "\"'`$&|;<>()*?!#%^".indexOf(ch) >= 0)) return a;
    if (windows) return "\"" + a.replace("\"", "\\\"") + "\"";
    return "'" + a.replace("'", "'\\''") + "'";
  }

  static boolean windows() {
    return System.getProperty("os.name").toLowerCase(Locale.ROOT).contains("win");
  }
}
