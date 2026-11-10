package sgm;

/** --help TEXT: only matching option entries, whole entries, case-insensitive; full help unchanged. */
final class HelpSearchRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) {
    System.out.println("PASS help search: " + run());
  }

  static int run() {
    checks = 0;
    check(Options.help(null).equals(Options.help()), "no topic prints the full help");
    check(Options.parse(new String[] {"--help", "census"}).helpTopic.equals("census"), "--help TEXT");
    check(Options.parse(new String[] {"--help=census"}).helpTopic.equals("census"), "--help=TEXT");
    check(Options.parse(new String[] {"--help", "-i", "x"}).helpTopic == null, "an option is not a topic");
    String census = Options.help("CENSUS-EDGE");
    check(census.contains("  --census-edge-mask T"), "entry found case-insensitively");
    check(census.contains("reduces foreground fattening"), "the whole entry is printed");
    check(!census.contains("--quick-calibration"), "non-matching entries are left out");
    check(Options.help("no-such-text-xyz").startsWith("No option mentions"), "no match message");
    // Every entry of the full help is found by its own option name.
    for (String line : Options.help().split("\n"))
      if (line.startsWith("  --")) {
        String name = line.trim().split("[ ,=]")[0];
        check(Options.help(name).contains(line), "entry not found by its name: " + name);
      }
    return checks;
  }
}
