package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;

/** Linux host, cgroup v1/v2 ancestor and Slurm memory limits. Paths are injectable for tests. */
final class LinuxMemory {
  record Limits(long total, long available, String source) {}

  static Limits read(Path proc, Map<String, String> environment) {
    Map<String, Long> info = values(proc.resolve("meminfo"), true);
    long total = info.getOrDefault("MemTotal", -1L);
    long available = info.getOrDefault("MemAvailable", info.getOrDefault("MemFree", -1L));
    String source = "Linux MemAvailable";
    boolean missingUsage = false;
    // CommitLimit is a hard constraint only under strict overcommit accounting.
    if (number(proc.resolve("sys/vm/overcommit_memory")) == 2) {
      long limit = info.getOrDefault("CommitLimit", -1L), used = info.getOrDefault("Committed_AS", -1L);
      if (limit >= 0 && used >= 0) available = minimum(available, Math.max(0, limit - used));
    }
    List<String> groups = lines(proc.resolve("self/cgroup"));
    for (String mount : lines(proc.resolve("self/mountinfo"))) {
      String[] halves = mount.split(" - ", 2);
      if (halves.length != 2) continue;
      String[] fields = halves[0].split(" +"), fs = halves[1].split(" +");
      if (fields.length < 5 || fs.length < 3) continue;
      boolean v2 = fs[0].equals("cgroup2");
      if (!v2 && (!fs[0].equals("cgroup") || !tokens(fs[2], "memory"))) continue;
      for (String group : groups) {
        String[] parts = group.split(":", 3);
        if (parts.length != 3 || (v2 ? !parts[1].isEmpty() : !tokens(parts[1], "memory"))) continue;
        Path boundary;
        try { boundary = Path.of(unescape(fields[4])).toAbsolutePath().normalize(); }
        catch (InvalidPathException ignored) { continue; }
        String root = unescape(fields[3]), name = parts[2];
        String relative = name.equals(root) ? ""
            : name.startsWith(root + "/") ? name.substring(root.length() + 1)
            : name.startsWith("/") ? name.substring(1) : ""; // cgroup namespace may rebase the path
        Path leaf = boundary.resolve(relative).normalize();
        if (!leaf.startsWith(boundary)) leaf = boundary;
        // Enforce every visible ancestor: a leaf may say 'max' under a limited job parent.
        for (Path dir = leaf; dir != null && dir.startsWith(boundary); dir = dir.getParent()) {
          long limit = number(dir.resolve(v2 ? "memory.max" : "memory.limit_in_bytes"));
          if (limit < 0 || limit >= (1L << 60)) continue; // v1 unlimited sentinel
          total = minimum(total, limit);
          source = "Linux host + cgroup " + (v2 ? "v2 " : "v1 ") + boundary;
          long used = number(dir.resolve(v2 ? "memory.current" : "memory.usage_in_bytes"));
          if (used < 0) {
            available = 0; // finite quota with unknown consumption is not safely available RAM
            missingUsage = true;
            continue;
          }
          Map<String, Long> stat = values(dir.resolve("memory.stat"), false);
          long reclaimable = stat.getOrDefault(v2 ? "inactive_file" : "total_inactive_file", 0L);
          long remaining = Math.max(0, limit - Math.max(0, used - Math.min(used, reclaimable)));
          available = minimum(available, remaining);
          if (dir.equals(boundary)) break;
        }
      }
    }
    // Useful when Slurm does not expose a memory controller in the process namespace.
    if (environment.containsKey("SLURM_JOB_ID") || environment.containsKey("SLURM_JOBID")) {
      long perNode = positive(environment.get("SLURM_MEM_PER_NODE"));
      long perCpu = positive(environment.get("SLURM_MEM_PER_CPU"));
      long cpus = positive(environment.get("SLURM_CPUS_ON_NODE"));
      long mib = perNode > 0 ? perNode : perCpu > 0 && cpus > 0 && perCpu <= Long.MAX_VALUE / cpus
          ? perCpu * cpus : -1;
      if (mib > 0 && mib <= Long.MAX_VALUE / MemorySafety.MIB) {
        long limit = mib * MemorySafety.MIB;
        long rss = values(proc.resolve("self/status"), true).getOrDefault("VmRSS", -1L);
        total = minimum(total, limit);
        if (rss >= 0) available = minimum(available, Math.max(0, limit - rss));
        source += " + Slurm allocation";
      }
    }
    if (missingUsage) source += " (cgroup usage unreadable; no safe headroom assumed)";
    return new Limits(total, available, source);
  }

  static long minimum(long a, long b) {
    return a < 0 ? b : b < 0 ? a : Math.min(a, b);
  }

  private static boolean tokens(String list, String token) {
    return Arrays.asList(list.split(",")).contains(token);
  }

  private static String unescape(String s) {
    return s.replace("\\040", " ").replace("\\011", "\t").replace("\\012", "\n").replace("\\134", "\\");
  }

  private static long positive(String text) {
    if (text == null) return -1;
    try { long n = Long.parseLong(text); return n > 0 ? n : -1; }
    catch (NumberFormatException ignored) { return -1; }
  }

  static long number(Path file) {
    try {
      String text = Files.readString(file).trim();
      if (text.equals("max")) return -1;
      long n = Long.parseLong(text);
      return n >= 0 ? n : -1;
    } catch (IOException | NumberFormatException | SecurityException ignored) { return -1; }
  }

  private static List<String> lines(Path file) {
    try { return Files.readAllLines(file); }
    catch (IOException | SecurityException ignored) { return List.of(); }
  }

  static Map<String, Long> values(Path file, boolean kib) {
    Map<String, Long> out = new HashMap<>();
    for (String line : lines(file)) {
      String[] fields = line.trim().split("\\s+");
      if (fields.length < 2) continue;
      try {
        long n = Long.parseLong(fields[1]);
        if (n < 0) continue;
        if (kib && fields.length > 2 && fields[2].equals("kB")) n = Math.multiplyExact(n, 1024L);
        out.put(fields[0].replace(":", ""), n);
      } catch (NumberFormatException | ArithmeticException ignored) { }
    }
    return out;
  }
}
