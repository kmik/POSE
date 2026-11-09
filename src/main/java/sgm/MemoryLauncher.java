package sgm;

import java.io.*;
import java.lang.management.ManagementFactory;
import java.nio.file.*;
import java.util.*;

/**
 * The heap must be selected before reconstruction starts; CLI-only re-execution preserves
 * arguments.
 */
final class MemoryLauncher {
  static long requestedBudget(String[] args) {
    long requested = 0;
    for (int i = 0; i < args.length; i++) {
      if (Set.of("-h", "--help", "-help", "--list-gpus").contains(args[i])) return 0;
      String value = null;
      if (args[i].equals("--memory-mb") && i + 1 < args.length) value = args[++i];
      else if (args[i].startsWith("--memory-mb=")) value = args[i].substring(12);
      if (value != null)
        try {
          requested = Long.parseLong(value);
        } catch (NumberFormatException ignored) {
          return 0;
        }
    }
    return requested;
  }

  static long heapMiB(long processingMiB) {
    if (processingMiB <= 0 || processingMiB > Long.MAX_VALUE / 100)
      throw new IllegalArgumentException("Memory budget is out of range");
    // The processing budget is 65% of heap; the remainder holds Java images/tracks/other state.
    return Math.addExact(
        Math.floorDiv(Math.addExact(Math.multiplyExact(processingMiB, 100), 64), 65), 16);
  }

  static long requestedRamLimit(String[] args) {
    for (int i = args.length - 1; i >= 0; i--)
      try {
        if (args[i].startsWith("--ram-limit-mb=")) return Long.parseLong(args[i].substring(15));
        if (args[i].equals("--ram-limit-mb") && i + 1 < args.length) return Long.parseLong(args[i + 1]);
      } catch (NumberFormatException ignored) { return 0; }
    return 0;
  }

  static long safeHeapMiB(MemorySafety.Limits limits, long heapUsed) {
    long safe = Long.MAX_VALUE;
    if (limits.total() >= 0) safe = limits.total() / 100 * 65;
    if (limits.available() >= 0)
      safe = Math.min(safe, heapUsed + limits.available() / 100 * 65);
    return safe == Long.MAX_VALUE ? safe : safe / MemorySafety.MIB;
  }

  static void main(String[] args) {
    int exit;
    try {
      // A continued pipeline step uses the memory budget saved by the first step.
      String[] effective = Main.launchArguments(args);
      if (Arrays.stream(effective).anyMatch(s -> Set.of("--help", "-h", "-help", "--list-gpus").contains(s))) {
        System.exit(Main.run(args));
        return;
      }
      long budget = requestedBudget(effective);
      long current = Runtime.getRuntime().maxMemory() / MemorySafety.MIB;
      long target = Math.max(current, budget > 0 ? heapMiB(budget) : 0);
      var memory = MemorySafety.snapshot();
      var limits = MemorySafety.limits(requestedRamLimit(effective));
      long safe = safeHeapMiB(limits, memory.heapUsed());
      if (!Boolean.getBoolean("sgm.heap.reexecuted")) {
        if (target > safe) {
          System.out.printf(
              Locale.ROOT,
              "Memory: heap target capped from %,d to %,d MiB by %s (35%% host/native headroom);"
                  + " effective processing allowance will be smaller%n",
              target,
              safe, limits.source());
          target = safe;
        }
        if (target < 64) throw new IllegalArgumentException("Insufficient host/job RAM for a safe JVM heap");
      } else target = current; // one relaunch only; runtime guards recheck remaining memory
      if (target != current) {
        List<String> command = new ArrayList<>();
        command.add(
            Path.of(
                    System.getProperty("java.home"),
                    "bin",
                    System.getProperty("os.name").toLowerCase(Locale.ROOT).contains("win")
                        ? "java.exe"
                        : "java")
                .toString());
        for (String option : ManagementFactory.getRuntimeMXBean().getInputArguments())
          if (!option.startsWith("-Xmx") && !option.startsWith("-Dsgm.heap.reexecuted=")
              && !(target < current && (option.startsWith("-Xms")
                  || option.startsWith("-XX:InitialHeapSize=") || option.startsWith("-XX:SoftMaxHeapSize="))))
            command.add(option);
        command.add("-Xmx" + target + "m");
        command.add("-Dsgm.heap.reexecuted=true");
        command.add("-cp");
        command.add(System.getProperty("java.class.path"));
        command.add("photogrammetry");
        command.addAll(Arrays.asList(args));
        System.out.printf(
            Locale.ROOT,
            "Memory: requested processing budget %,d MiB; starting JVM with %,d MiB heap "
                + "(processing allowance up to 65%% of heap; native memory separate)%n",
            budget,
            target);
        Process process = new ProcessBuilder(command).inheritIO().start();
        Thread forward =
            new Thread(
                () -> {
                  if (process.isAlive()) process.destroy();
                },
                "photogrammetry-child-cleanup");
        Runtime.getRuntime().addShutdownHook(forward);
        try {
          exit = process.waitFor();
        } finally {
          Runtime.getRuntime().removeShutdownHook(forward);
        }
      } else exit = Main.run(args);
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      exit = 130;
    } catch (IOException | IllegalArgumentException | ArithmeticException e) {
      System.err.println("photogrammetry: " + e.getMessage());
      exit = 2;
    }
    System.exit(exit);
  }

  static void report(Options o) {
    long heap = Runtime.getRuntime().maxMemory(), requested = o.memoryMb * 1048576L;
    System.out.printf(
        Locale.ROOT,
        "Memory: JVM heap %.1f MiB; processing budget %.1f MiB (requested %,d MiB); native/GPU"
            + " memory separate%n",
        heap / 1048576.,
        Math.min(requested, heap * .65) / 1048576.,
        o.memoryMb);
  }
}
