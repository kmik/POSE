package sgm;

import java.io.IOException;
import java.nio.file.Path;
import java.util.*;
import java.util.function.*;

/**
 * One POSE command running as a child process for a window: output lines go to a listener, the
 * run id and project announced by the command and the latest stage progress are kept, the exit
 * code is reported once, and {@link #stop()} ends the whole process tree. No window here; the
 * listener is called from the reader thread.
 */
final class ChildRun {
  interface Listener {
    void line(String line);

    void finished(int exit, boolean stopped);
  }

  final List<String> args;
  final long startedMillis = System.currentTimeMillis();
  private final Listener listener;
  private volatile Process process;
  private volatile String runId;
  private volatile Path project;
  private volatile ProjectFiles.Progress progress;
  private volatile boolean stopped;
  private volatile Integer exit;
  private final List<String> tail = new ArrayList<>();

  private ChildRun(List<String> args, Listener listener, Function<Consumer<String>, Process> starter) {
    this.args = List.copyOf(args);
    this.listener = listener;
    this.process = starter.apply(this::accept);
  }

  /** Starts the CLI with these arguments (PoseProcess: same java, class path, launcher). */
  static ChildRun start(List<String> args, Listener listener) throws IOException {
    try {
      return new ChildRun(
          args,
          listener,
          sink -> {
            try {
              return PoseProcess.start(args, sink);
            } catch (IOException e) {
              throw new UncheckedIOException(e);
            }
          });
    } catch (UncheckedIOException e) {
      throw e.getCause();
    }
  }

  /** For tests: any command line, with the same output handling. */
  static ChildRun startCommand(List<String> command, Listener listener) throws IOException {
    try {
      return new ChildRun(
          command,
          listener,
          sink -> {
            try {
              ProcessBuilder b = new ProcessBuilder(command).redirectErrorStream(true);
              Process p = b.start();
              p.getOutputStream().close();
              Thread t =
                  new Thread(
                      () -> {
                        try (var in = new java.io.InputStreamReader(p.getInputStream(), java.nio.charset.StandardCharsets.UTF_8)) {
                          PoseProcess.lines(in, sink);
                        } catch (IOException e) {
                          // closed
                        } finally {
                          sink.accept(null);
                        }
                      });
              t.setDaemon(true);
              t.start();
              return p;
            } catch (IOException e) {
              throw new UncheckedIOException(e);
            }
          });
    } catch (UncheckedIOException e) {
      throw e.getCause();
    }
  }

  private static final class UncheckedIOException extends RuntimeException {
    UncheckedIOException(IOException e) {
      super(e);
    }

    @Override
    public synchronized IOException getCause() {
      return (IOException) super.getCause();
    }
  }

  /** Output from the reader thread; null at the end of the stream. */
  private void accept(String line) {
    if (line == null) {
      int code;
      try {
        code = waitProcess().waitFor();
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
        code = -1;
      }
      exit = code;
      listener.finished(code, stopped);
      return;
    }
    String id = ProjectFiles.runId(line);
    if (id != null) runId = id;
    String root = ProjectFiles.projectRoot(line);
    if (root != null)
      try {
        project = Path.of(root);
      } catch (RuntimeException e) {
        // not a path
      }
    ProjectFiles.Progress p = ProjectFiles.progress(line);
    if (p != null) progress = p;
    synchronized (tail) {
      tail.add(line);
      if (tail.size() > 200) tail.remove(0);
    }
    listener.line(line);
  }

  private Process waitProcess() {
    // The reader may see the end of output before the constructor stored the process.
    while (process == null) Thread.onSpinWait();
    return process;
  }

  boolean alive() {
    return exit == null;
  }

  Integer exit() {
    return exit;
  }

  String runId() {
    return runId;
  }

  Path project() {
    return project;
  }

  ProjectFiles.Progress progress() {
    return progress;
  }

  List<String> tail() {
    synchronized (tail) {
      return new ArrayList<>(tail);
    }
  }

  long elapsedMillis() {
    return System.currentTimeMillis() - startedMillis;
  }

  /** Ends the command and every process it started (the memory re-execution). */
  void stop() {
    stopped = true;
    PoseProcess.terminate(process.toHandle(), 3000);
  }

  ProcessHandle handle() {
    return process.toHandle();
  }
}
