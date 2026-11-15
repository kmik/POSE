package sgm;

import java.util.*;

/**
 * Memory use of pair matching on large blocks: the packed per-pair match list behaves like the
 * ArrayList it replaced, and the low-RAM warning appears only when the host is nearly out of
 * memory. (Unchanged outputs are covered by the project runs of the other suites.)
 */
final class MatchMemoryRegression {
  private static int checks;

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) {
    System.out.println("PASS match memory: " + run());
  }

  static int run() {
    checks = 0;
    packedMatches();
    lowMemoryWarning();
    return checks;
  }

  private static void packedMatches() {
    Random random = new Random(7);
    FeatureTracks.Matches packed = new FeatureTracks.Matches();
    List<int[]> plain = new ArrayList<>();
    check(packed.isEmpty() && packed.size() == 0, "new list is empty");
    for (int i = 0; i < 1000; i++) {
      int a = random.nextInt(Integer.MAX_VALUE), b = random.nextInt(50000);
      check(packed.add(new FeatureTracks.Match(a, b)), "add returns true");
      plain.add(new int[] {a, b});
    }
    check(packed.size() == 1000, "size after adds");
    boolean same = true;
    for (int i = 0; i < plain.size(); i++)
      same &= packed.get(i).a == plain.get(i)[0] && packed.get(i).b == plain.get(i)[1];
    check(same, "get returns the added ids in order");
    int k = 0;
    same = true;
    for (FeatureTracks.Match m : packed) {
      same &= m.a == plain.get(k)[0] && m.b == plain.get(k)[1];
      k++;
    }
    check(same && k == 1000, "iteration returns the added ids in order");
    packed.trim();
    check(packed.size() == 1000 && packed.get(999).b == plain.get(999)[1], "trim keeps the content");
    packed.add(new FeatureTracks.Match(-1, 3));
    check(packed.size() == 1001 && packed.get(1000).a == -1 && packed.get(1000).b == 3, "add after trim");
    for (int bad : new int[] {-1, 1001})
      try {
        packed.get(bad);
        check(false, "index " + bad + " must be refused");
      } catch (IndexOutOfBoundsException expected) {
        check(true, "index refused");
      }
    try {
      for (FeatureTracks.Match m : packed) packed.add(m);
      check(false, "adding while iterating must fail fast");
    } catch (ConcurrentModificationException expected) {
      check(true, "fail-fast iterator");
    }
    packed.clear();
    check(packed.isEmpty(), "clear");
    packed.trim();
    packed.add(new FeatureTracks.Match(4, 5));
    check(packed.size() == 1 && packed.get(0).a == 4, "add after clear and trim");
    FeatureTracks.Pair pair = new FeatureTracks.Pair(2, 3);
    check(pair.matches.isEmpty() && pair.matches instanceof RandomAccess, "pairs use the packed list");
  }

  private static void lowMemoryWarning() {
    long gib = 1L << 30;
    check(RunTelemetry.lowMemoryWarning(-1, gib, gib) == null, "unknown total: no warning");
    check(RunTelemetry.lowMemoryWarning(64 * gib, -1, gib) == null, "unknown available: no warning");
    check(RunTelemetry.lowMemoryWarning(64 * gib, 20 * gib, 40 * gib) == null, "plenty left");
    // 5% of 256 GB is 12.8 GB, but 3 GiB is still above the 2 GiB floor.
    check(RunTelemetry.lowMemoryWarning(256 * gib, 3 * gib, 200 * gib) == null, "large host, 3 GiB left");
    // A small machine: 1.5 GiB of 16 is under 2 GiB but above 5%.
    check(RunTelemetry.lowMemoryWarning(16 * gib, 3 * gib / 2, 10 * gib) == null, "16 GiB host, 1.5 GiB left");
    String w = RunTelemetry.lowMemoryWarning(256 * gib, gib, 240 * gib);
    check(w != null && w.startsWith("Warning: host RAM nearly exhausted"), "warning near exhaustion: " + w);
    check(w.contains("1.1 of 274.9 GB") && w.contains("257.7 GB"), "warning numbers: " + w);
    check(Terminal.important(w), "the warning is shown without --verbose");
    check(RunTelemetry.lowMemoryWarning(8 * gib, gib / 4, 6 * gib) != null, "small host near exhaustion");
  }
}
