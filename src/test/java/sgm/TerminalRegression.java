package sgm;

import java.io.*;
import java.nio.charset.StandardCharsets;

/** U1: --verbose passes every line; the default view keeps only important lines; formatting. */
final class TerminalRegression {
  private static int checks;

  private static void check(boolean ok, String why) {
    checks++;
    if (!ok) throw new AssertionError(why);
  }

  static int run() throws Exception {
    checks = 0;
    check(Terminal.important("Warning: skipping failed pair 10-14: x"), "warnings shown");
    check(Terminal.important("photogrammetry: Packed disparity bands exceed 2^31 cells"), "errors shown");
    check(Terminal.important("Banded SGM: 1 packed cells exceed ...; solving 3 overlapping strips"), "strip fallback shown");
    check(!Terminal.important("Registered cameras=4/4; failed images=[]"), "zero-failure summary hidden");
    check(!Terminal.important("Dense pair 10-13: 996,101 candidates"), "routine lines hidden");
    check(!Terminal.important("Decode/resize/undistort x.JPG: stopped before completion; 1.0 s"), "task interruptions hidden");
    check(Terminal.ascii("Stage 05_stereo — Rectification").equals("Stage 05_stereo - Rectification"), "em dash becomes ASCII");
    check(Terminal.bar(.5, 10).equals("[#####-----]"), "bar");
    check(Terminal.clock(65).equals("1:05") && Terminal.clock(3725).equals("1:02:05"), "clock");
    // Verbose: every line passes, decoded from UTF-8 and printed through the console encoding.
    ByteArrayOutputStream screen = new ByteArrayOutputStream();
    PrintStream target = new PrintStream(screen, true, StandardCharsets.UTF_8);
    Terminal.configure(true, target);
    OutputStream sink = Terminal.sink(target, false);
    sink.write("Stage 05_stereo — x: starting\nDense pair 1-2: 5 candidates\n".getBytes(StandardCharsets.UTF_8));
    String shown = screen.toString(StandardCharsets.UTF_8);
    check(shown.contains("Stage 05_stereo - x: starting") && shown.contains("Dense pair 1-2"), "verbose passes every line");
    Terminal.stop();
    return checks;
  }

  public static void main(String[] args) throws Exception {
    System.out.println("PASS terminal checks=" + run());
  }
}
