package sgm;

import java.io.IOException;
import java.lang.management.ManagementFactory;
import java.nio.file.*;
import java.nio.file.attribute.FileTime;
import java.util.*;
import java.util.stream.Stream;

/**
 * Preferences of the POSE windows (the Settings menu), kept beside the launcher settings in
 * {@code preferences.properties}. Nothing here changes processing: the preferences decide how the
 * windows look and behave, and "fit to this computer" only fills option fields the user can see.
 * Windowless, so that SelfTest can check it.
 */
final class GuiPreferences {
  static final String FILE = "preferences.properties";
  /** Interface scale choices; "auto" leaves it to the system (Windows display setting, GDK_SCALE). */
  static final List<String> SCALES = List.of("auto", "100", "125", "150", "175", "200");
  static final List<String> NEW_PROJECTS = List.of("last", "pose");
  static final int DEFAULT_CACHE_MB = 1000;

  final Path dir;
  /** Interface scale in percent, or "auto". Applied at the next start. */
  String scale = "auto";
  /** New projects start with the last options used ("last") or always with the POSE defaults ("pose"). */
  String newProjects = "last";
  /** Open the report (PDF, else HTML) when a run completes. */
  boolean openReport = false;
  /** Bring the window to the front when a command ends (it may have been behind other windows for hours). */
  boolean toFront = true;
  /** Ask before closing the window while a command runs. */
  boolean confirmClose = true;
  /** Size limit of the image preview cache in MB; the oldest previews go first. */
  int cacheMb = DEFAULT_CACHE_MB;

  GuiPreferences(Path dir) {
    this.dir = dir;
  }

  static GuiPreferences load(Path dir) {
    GuiPreferences p = new GuiPreferences(dir);
    Map<String, String> v = GuiOptions.Settings.read(dir.resolve(FILE));
    if (SCALES.contains(v.getOrDefault("scale", ""))) p.scale = v.get("scale");
    if (NEW_PROJECTS.contains(v.getOrDefault("new.projects", ""))) p.newProjects = v.get("new.projects");
    p.openReport = Boolean.parseBoolean(v.getOrDefault("open.report", "" + p.openReport));
    p.toFront = Boolean.parseBoolean(v.getOrDefault("to.front", "" + p.toFront));
    p.confirmClose = Boolean.parseBoolean(v.getOrDefault("confirm.close", "" + p.confirmClose));
    try {
      p.cacheMb = Math.max(0, Integer.parseInt(v.getOrDefault("cache.mb", "" + p.cacheMb).trim()));
    } catch (NumberFormatException e) {
      p.cacheMb = DEFAULT_CACHE_MB;
    }
    return p;
  }

  static GuiPreferences user() {
    return load(GuiOptions.Settings.user().dir);
  }

  void save() throws IOException {
    Map<String, String> v = new TreeMap<>();
    v.put("scale", scale);
    v.put("new.projects", newProjects);
    v.put("open.report", "" + openReport);
    v.put("to.front", "" + toFront);
    v.put("confirm.close", "" + confirmClose);
    v.put("cache.mb", "" + cacheMb);
    GuiOptions.Settings.write(dir.resolve(FILE), v);
  }

  /** The value of sun.java2d.uiScale for the scale preference, or null for "auto". */
  static String uiScale(String scale) {
    if (scale == null || scale.equals("auto")) return null;
    return String.format(Locale.ROOT, "%.2f", Integer.parseInt(scale) / 100.0);
  }

  /**
   * Sets the interface scale before the first window exists (it cannot change afterwards). A scale
   * given on the Java command line (-Dsun.java2d.uiScale) wins.
   */
  static void applyScale() {
    if (System.getProperty("sun.java2d.uiScale") != null) return;
    try {
      String s = uiScale(user().scale);
      if (s != null) System.setProperty("sun.java2d.uiScale", s);
    } catch (RuntimeException e) {
      // Unreadable preferences: the system scale stays.
    }
  }

  // ---------------------------------------------------------------------------------------------
  // This computer

  /** Cores, physical RAM and the GPU (OpenCL device) of this computer; gpuMb < 0 without a GPU. */
  record Machine(int cores, long ramMb, String gpu, long gpuMb) {
    String describe() {
      return cores + " CPU cores, " + String.format(Locale.ROOT, "%.1f", ramMb / 1024.0) + " GB RAM, "
          + (gpuMb < 0 ? "no OpenCL GPU found" : gpu + " with " + String.format(Locale.ROOT, "%.1f", gpuMb / 1024.0) + " GB");
    }
  }

  static Machine machine(int gpuDevice) {
    int cores = Runtime.getRuntime().availableProcessors();
    long ram = -1;
    try {
      ram = MemorySafety.limits().total();
    } catch (RuntimeException | LinkageError e) {
      // Below: the management bean.
    }
    if (ram <= 0 && ManagementFactory.getOperatingSystemMXBean() instanceof com.sun.management.OperatingSystemMXBean mx)
      ram = mx.getTotalMemorySize();
    // The GPUs as the command line numbers them (--list-gpus in a child process): inside a window
    // process OpenCL may enumerate differently (on hybrid-graphics laptops the display adapter's
    // toolkit changes the order), so --gpu-device N would not match.
    String gpu = null;
    long gpuMb = -1;
    List<String[]> gpus = gpus(listGpus());
    if (!gpus.isEmpty()) {
      String[] d = gpus.get(Math.max(0, Math.min(gpus.size() - 1, gpuDevice)));
      gpu = d[0];
      gpuMb = Long.parseLong(d[1]);
    }
    return new Machine(cores, Math.max(0, ram >> 20), gpu, gpuMb);
  }

  /** Output of {@code photogrammetry --list-gpus} in a child process (empty on failure). */
  static String listGpus() {
    StringBuilder out = new StringBuilder();
    try {
      Process p = PoseProcess.start(List.of("--list-gpus"), line -> {
        if (line != null) synchronized (out) { out.append(line).append('\n'); }
      });
      if (!p.waitFor(60, java.util.concurrent.TimeUnit.SECONDS)) {
        PoseProcess.terminate(p.toHandle(), 1000);
        return "";
      }
      Thread.sleep(100); // the reader thread passes the last lines
    } catch (IOException e) {
      return "";
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      return "";
    }
    synchronized (out) {
      return out.toString();
    }
  }

  /** {name, MiB} per line "N: NAME; VRAM M MiB; ..." of --list-gpus, in device order. */
  static List<String[]> gpus(String listing) {
    List<String[]> out = new ArrayList<>();
    java.util.regex.Matcher m = java.util.regex.Pattern.compile("(?m)^(\\d+): (.+?); VRAM (\\d+) MiB").matcher(listing);
    while (m.find())
      if (Integer.parseInt(m.group(1)) == out.size()) out.add(new String[] {m.group(2).trim(), m.group(3)});
    return out;
  }

  /**
   * Compute options sized for a computer, the window's defaults for them: three quarters of the
   * logical processors (the rest keeps the computer usable), a memory budget of half the physical
   * RAM (rounded down to 1000 MB, at least 2000; the launcher gives Java a heap from it), and GPU
   * memory of 95% of the device (the most the command line accepts; rounded down to 100 MB), or
   * CPU processing without a GPU. A GPU not yet detected (gpu == null) adds nothing for it, so
   * the command line's automatic GPU budget applies.
   */
  static Map<String, String> fitted(Machine m) {
    Map<String, String> v = new LinkedHashMap<>();
    v.put("--cores", "" + Math.max(1, Math.round(m.cores() * .75f)));
    if (m.ramMb() > 0) v.put("--memory-mb", "" + Math.max(2000, m.ramMb() / 2 / 1000 * 1000));
    if (m.gpuMb() > 0) v.put("--gpu-memory-mb", "" + Math.max(100, m.gpuMb() * 95 / 100 / 100 * 100));
    else if (m.gpu() != null) v.put("--compute", "cpu");
    return v;
  }

  // ---------------------------------------------------------------------------------------------
  // This computer, remembered (the window's compute defaults)

  static final String MACHINE_FILE = "machine.properties";
  /** Marks "no OpenCL GPU found" in the machine file (gpu.mb -1). */
  static final String NO_GPU = "none";

  /** The remembered hardware, or null before it was detected in this settings folder. */
  static Machine remembered(Path dir) {
    Map<String, String> v = GuiOptions.Settings.read(dir.resolve(MACHINE_FILE));
    try {
      if (!v.containsKey("cores")) return null;
      String gpu = v.getOrDefault("gpu", NO_GPU);
      return new Machine(Integer.parseInt(v.get("cores")), Long.parseLong(v.getOrDefault("ram.mb", "0")),
          gpu, Long.parseLong(v.getOrDefault("gpu.mb", "-1")));
    } catch (NumberFormatException e) {
      return null;
    }
  }

  static void remember(Path dir, Machine m) throws IOException {
    Map<String, String> v = new TreeMap<>();
    v.put("cores", "" + m.cores());
    v.put("ram.mb", "" + m.ramMb());
    v.put("gpu", m.gpu() == null ? NO_GPU : m.gpu());
    v.put("gpu.mb", "" + m.gpuMb());
    GuiOptions.Settings.write(dir.resolve(MACHINE_FILE), v);
  }

  /** Detects this computer (GPU 0 as the command line numbers it) and remembers it. */
  static Machine detect(Path dir) {
    Machine m = machine(0);
    if (m.gpu() == null) m = new Machine(m.cores(), m.ramMb(), NO_GPU, -1);
    try {
      remember(dir, m);
    } catch (IOException e) {
      // Detected again next time.
    }
    return m;
  }

  /**
   * The window's compute defaults for this computer: from the remembered hardware, or before the
   * first detection from the cores and RAM alone (no GPU budget: the command line's automatic one).
   */
  static Map<String, String> hardwareDefaults(Path dir) {
    Machine m = remembered(dir);
    if (m == null) {
      long ram = 0;
      if (ManagementFactory.getOperatingSystemMXBean() instanceof com.sun.management.OperatingSystemMXBean mx)
        ram = mx.getTotalMemorySize() >> 20;
      m = new Machine(Runtime.getRuntime().availableProcessors(), ram, null, -1);
    }
    return fitted(m);
  }

  // ---------------------------------------------------------------------------------------------
  // Preview cache

  /** Bytes of the files in the cache folder (0 when it does not exist). */
  static long cacheBytes(Path cache) {
    if (!Files.isDirectory(cache)) return 0;
    long n = 0;
    try (Stream<Path> s = Files.list(cache)) {
      for (Path p : (Iterable<Path>) s::iterator)
        if (Files.isRegularFile(p)) n += Files.size(p);
    } catch (IOException e) {
      // Partial sum.
    }
    return n;
  }

  /** Deletes the least recently used previews until the cache is at most maxBytes; returns how many. */
  static int trim(Path cache, long maxBytes) {
    if (!Files.isDirectory(cache)) return 0;
    record Entry(Path path, long size, FileTime time) {}
    List<Entry> all = new ArrayList<>();
    long total = 0;
    try (Stream<Path> s = Files.list(cache)) {
      for (Path p : (Iterable<Path>) s::iterator)
        if (Files.isRegularFile(p)) {
          long size = Files.size(p);
          all.add(new Entry(p, size, Files.getLastModifiedTime(p)));
          total += size;
        }
    } catch (IOException e) {
      return 0;
    }
    all.sort(Comparator.comparing(Entry::time).thenComparing(e -> e.path().toString()));
    int removed = 0;
    for (Entry e : all) {
      if (total <= maxBytes) break;
      try {
        Files.deleteIfExists(e.path());
        total -= e.size();
        removed++;
      } catch (IOException ignored) {
        // In use: keep it.
      }
    }
    return removed;
  }

  /** Marks a cached preview as just used, so that trimming keeps it. */
  static void touch(Path file) {
    try {
      Files.setLastModifiedTime(file, FileTime.fromMillis(System.currentTimeMillis()));
    } catch (IOException e) {
      // Only affects which previews are trimmed first.
    }
  }

  // ---------------------------------------------------------------------------------------------
  // Saved options

  /** Deletes the remembered options of every project and the user defaults; returns the files removed. */
  static int forgetOptions(GuiOptions.Settings settings) {
    int n = 0;
    try {
      if (Files.deleteIfExists(settings.defaultsFile())) n++;
      Path projects = settings.dir.resolve("projects");
      if (Files.isDirectory(projects))
        try (Stream<Path> s = Files.list(projects)) {
          for (Path p : (Iterable<Path>) s::iterator)
            if (p.getFileName().toString().endsWith(".properties") && Files.deleteIfExists(p)) n++;
        }
    } catch (IOException e) {
      // What could be deleted is gone.
    }
    return n;
  }

  /** Empties the recent-projects list. */
  static void clearRecent(GuiOptions.Settings settings) throws IOException {
    Files.deleteIfExists(settings.dir.resolve("recent.properties"));
  }
}
