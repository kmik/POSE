package sgm;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.security.MessageDigest;
import java.util.*;

/**
 * The options shown as fields in the POSE launcher window, their tooltips (the CLI help entries),
 * building the argument list, splitting the "advanced arguments" text, validation with the real
 * {@link Options} parser, and remembered settings. No window here.
 */
final class GuiOptions {
  private GuiOptions() {}

  enum Kind {
    /** Folder path. */
    DIR,
    /** File path (or folder for orientation imports). */
    FILE,
    /** Free text or number. */
    TEXT,
    /** One of the choices; the first choice "" means the CLI default (not passed). */
    CHOICE,
    /** A flag without value ("true" when set). */
    FLAG,
    /** Free value with suggested words (e.g. "auto" for N|auto). */
    SUGGEST,
    /** Any number of the choices, comma-separated (choices[0] is "" = the CLI default). */
    MULTI
  }

  /**
   * A MULTI value: the given words that are among the choices, in the choices' order, joined by
   * commas ("" when none). Anything else is dropped, so only supported values reach the command.
   */
  static String multi(Collection<String> picked, List<String> choices) {
    Set<String> want = new HashSet<>();
    for (String p : picked) want.add(p.strip().toLowerCase(Locale.ROOT));
    List<String> out = new ArrayList<>();
    for (String c : choices) if (!c.isEmpty() && want.contains(c)) out.add(c);
    return String.join(",", out);
  }

  /**
   * Options whose values come from a fixed list in the code although the help names them NAME
   * or LIST: offered only as those values.
   */
  static Field refine(Field f) {
    List<String> orientation = new ArrayList<>(List.of(""));
    orientation.addAll(Arrays.asList(OrientationIO.FORMATS.split(",")));
    return switch (f.flag) {
      case "--orientation-input-format" ->
          new Field(f.flag, "INPUT: format of the Orientations file (--orientation-input) on the New run tab (default: auto).", f.section, Kind.CHOICE, List.copyOf(orientation));
      case "--calibration-format" ->
          new Field(f.flag, "INPUT: format of the --calibration file (interior orientation) (default: auto).", f.section, Kind.CHOICE, List.copyOf(orientation));
      case "--poses-format" ->
          new Field(f.flag, "INPUT: format of the --poses file (exterior orientation) (default: auto).", f.section, Kind.CHOICE, List.copyOf(orientation));
      case "--orientation-output-format" ->
          new Field(f.flag, "OUTPUT: export the interior and exterior orientations in these formats when the run finishes (default: none).", f.section, Kind.MULTI, List.copyOf(orientation));
      case "--interior-output-format" ->
          new Field(f.flag, "OUTPUT: also export only the interior orientations (camera calibration) in these formats.", f.section, Kind.MULTI, List.copyOf(orientation));
      case "--exterior-output-format" ->
          new Field(f.flag, "OUTPUT: also export only the exterior orientations (camera poses) in these formats.", f.section, Kind.MULTI, List.copyOf(orientation));
      default -> f;
    };
  }

  record Field(String flag, String label, String section, Kind kind, List<String> choices) {
    /** CLI help entry of this option (tooltip). */
    String help() {
      return GuiOptions.help(flag);
    }
  }

  static final String INPUT = "Input and project", PROCESSING = "Processing", COMPUTE = "Compute and memory", OUTPUT = "Outputs";

  static final List<Field> FIELDS =
      List.of(
          new Field("-i", "Image folder", INPUT, Kind.DIR, List.of()),
          new Field("-o", "Project folder", INPUT, Kind.DIR, List.of()),
          new Field("--orientation-input", "Orientations (optional)", INPUT, Kind.FILE, List.of()),
          new Field("--gcp", "GCP file (optional)", INPUT, Kind.FILE, List.of()),
          new Field("--gcp-check", "GCP check points", INPUT, Kind.CHOICE, List.of("", "none", "auto", "1", "2", "3", "4")),
          new Field("--lidar-reference", "Lidar reference (LAS/LAZ)", INPUT, Kind.FILE, List.of()),
          new Field("--lidar-scale", "   Estimate scale", INPUT, Kind.CHOICE, List.of("", "off", "on")),
          new Field("--lidar-search-shift", "   Search shift (m)", INPUT, Kind.TEXT, List.of()),
          new Field("--lidar-search-angle", "   Search heading (degrees)", INPUT, Kind.TEXT, List.of()),
          new Field("--lidar-canopy-weight", "   Canopy weight", INPUT, Kind.TEXT, List.of()),
          new Field("--lidar-vertical-datum", "   Lidar height datum", INPUT, Kind.TEXT, List.of()),
          new Field("--crs", "Coordinate system (EPSG)", INPUT, Kind.TEXT, List.of()),
          new Field("--source-geoid", "   GPS height geoid (file or ellipsoid)", INPUT, Kind.FILE, List.of()),
          new Field("--target-geoid", "   Output height geoid (file or ellipsoid)", INPUT, Kind.FILE, List.of()),
          new Field("--camera-grouping", "Camera grouping", INPUT, Kind.CHOICE, List.of("", "auto", "single", "per-image")),
          new Field("--camera-groups", "   Camera of each image (CSV)", INPUT, Kind.FILE, List.of()),
          new Field("--quality", "Quality", PROCESSING, Kind.CHOICE, List.of("", "ultra", "high", "medium", "low")),
          new Field("--max-dimension", "Max image dimension (px)", PROCESSING, Kind.TEXT, List.of()),
          new Field("--dense-method", "Dense method", PROCESSING, Kind.CHOICE, List.of("", "sgm", "mvsgm", "patchmatch")),
          new Field("--filtering", "Filtering", PROCESSING, Kind.CHOICE, List.of("", "mild", "moderate", "aggressive")),
          new Field("--dense-checkpoint", "Dense checkpoint (minutes)", PROCESSING, Kind.SUGGEST, List.of("", "auto", "off")),
          new Field("--min-views", "Minimum views per point", PROCESSING, Kind.TEXT, List.of()),
          new Field("--quick-calibration", "Quick calibration", PROCESSING, Kind.FLAG, List.of()),
          new Field("--calibration-trial-method", "Calibration trials", PROCESSING, Kind.CHOICE, List.of("", "full", "subset", "joint")),
          new Field("--calibration-acceptance", "Calibration acceptance", PROCESSING, Kind.CHOICE, List.of("", "strict", "relaxed")),
          new Field("--tie-point-cleanup", "Tie-point cleanup", PROCESSING, Kind.CHOICE, List.of("", "off", "auto", "custom", "pause")),
          new Field("--tie-point-max-error", "   Largest error (px)", PROCESSING, Kind.SUGGEST, List.of("", "auto")),
          new Field("--tie-point-min-views", "   Fewest images", PROCESSING, Kind.TEXT, List.of()),
          new Field("--tie-point-min-angle", "   Smallest angle (degrees)", PROCESSING, Kind.TEXT, List.of()),
          new Field("--tie-point-max-removed", "   Most removed (%)", PROCESSING, Kind.TEXT, List.of()),
          new Field("--classify-ground", "Classify ground (DTM, CHM)", PROCESSING, Kind.FLAG, List.of()),
          new Field("--orthomosaic", "Orthomosaic", PROCESSING, Kind.FLAG, List.of()),
          new Field("--orthomosaic-mode", "   Mode", PROCESSING, Kind.CHOICE, List.of("", "true", "classic")),
          new Field("--compute", "Compute device", COMPUTE, Kind.CHOICE, List.of("", "auto", "gpu", "cpu")),
          new Field("--gpu-device", "GPU index", COMPUTE, Kind.TEXT, List.of()),
          new Field("--gpu-memory-mb", "GPU memory (MiB)", COMPUTE, Kind.TEXT, List.of()),
          new Field("--fusion-gpu-normals", "GPU normals in fusion", COMPUTE, Kind.CHOICE, List.of("", "on", "off")),
          new Field("--cores", "CPU cores", COMPUTE, Kind.TEXT, List.of()),
          new Field("--memory-mb", "Memory budget (MiB)", COMPUTE, Kind.TEXT, List.of()),
          new Field("--ram-limit-mb", "RAM limit (MiB)", COMPUTE, Kind.TEXT, List.of()),
          new Field("--ply", "Write dense.ply", OUTPUT, Kind.CHOICE, List.of("", "on", "off")),
          new Field("--tiled-las-output", "LAS tile size (m)", OUTPUT, Kind.TEXT, List.of()),
          new Field("--point-confidence", "Point confidence file", OUTPUT, Kind.CHOICE, List.of("", "on", "off")),
          new Field("--point-attributes", "Per-point values in LAS/PLY", OUTPUT, Kind.MULTI, List.of("", "surface", "uncertainty",
              "normal", "roughness", "surface_variation", "planarity", "normal_diagnostics", "height_sigma", "ray_sigma",
              "depth_spread", "views", "angle", "confidence", "source", "none")),
          new Field("--verbose", "Verbose log", OUTPUT, Kind.FLAG, List.of()));

  /**
   * The window's own defaults for a new project and for Reset options: the tested mvsgm setup
   * (the user's city/forest mvsgm script), everything except the folders and the compute sizes,
   * which come from this computer ({@link Settings#windowDefaults}, GuiPreferences.fitted). Fields
   * left out keep the command-line default.
   */
  static final Map<String, String> SCRIPT_DEFAULTS = scriptDefaults();

  /** The compute options the window sizes from the hardware. */
  static final List<String> HARDWARE = List.of("--cores", "--memory-mb", "--gpu-memory-mb", "--compute");

  /** The script's fixed compute sizes (settings version 2); saved ones still equal to these follow the hardware. */
  static final Map<String, String> SCRIPT_HARDWARE = Map.of("--cores", "12", "--memory-mb", "16000", "--gpu-memory-mb", "8000");

  private static Map<String, String> scriptDefaults() {
    String[][] pairs = {
      {"--quality", "high"}, {"--quick-calibration", "true"},
      {"--ply", "off"}, {"--point-confidence", "off"}, {"--features", "40000"}, {"--ratio", "0.85"}, {"--neighbors", "8"},
      {"--descriptor-matcher", "flann"}, {"--matching-algorithm", "sift"}, {"--descriptor-memory-mb", "1600"}, {"--min-forward-overlap", "50"},
      {"--step", "1"}, {"--filtering", "mild"}, {"--dense-method", "mvsgm"}, {"--min-views", "2"}, {"--min-angle", "4"},
      {"--confidence-threshold", "0.0"}, {"--surface-consolidation", "none"}, {"--windowed-mode", "auto"}, {"--fusion-gpu-evidence", "auto"},
      {"--fusion-mode", "image"}, {"--fusion-outliers", "robust"}, {"--subpixel-fit", "equiangular"}, {"--fusion-strategy", "graph-average"},
      {"--mvsgm-optimal-neighbors", "true"}, {"--depth-outlier-filter", "edge-aware"}, {"--mvsgm-neighbors", "8"},
      {"--mvsgm-best-k", "3"}, {"--mvsgm-census-window", "5"}, {"--mvsgm-consistent-views", "1"}, {"--mvsgm-consistency-labels", "1"},
    };
    Map<String, String> out = new LinkedHashMap<>();
    for (String[] p : pairs) out.put(p[0], p[1]);
    return Collections.unmodifiableMap(out);
  }

  /**
   * The window's Preset menu ({@link Presets}): platform, or platform and scene. Choosing one fills
   * the preset's option fields, which stay editable; the menu itself adds nothing to the command.
   */
  static final List<String> PRESET_CHOICES =
      List.of("drone", "drone, city", "drone, forest", "aircraft", "aircraft, city", "aircraft, forest");

  /**
   * Field values of a preset choice ("drone, forest"): flag to value ("true" for a bare flag), plus
   * "" (the CLI default) for every option another choice sets, so switching presets leaves nothing
   * of the previous one behind.
   */
  static Map<String, String> presetValues(String choice) {
    Map<String, String> out = new LinkedHashMap<>();
    for (String other : PRESET_CHOICES) for (String flag : presetOwnValues(other).keySet()) out.put(flag, "");
    out.putAll(presetOwnValues(choice));
    return out;
  }

  /** The options a preset choice sets itself. */
  static Map<String, String> presetOwnValues(String choice) {
    List<String> o = Presets.options(choice.replace(" ", ""));
    Map<String, String> out = new LinkedHashMap<>();
    for (int i = 0; i < o.size(); i++) {
      String flag = o.get(i);
      boolean bare = i + 1 >= o.size() || o.get(i + 1).startsWith("--");
      out.put(flag, bare ? "true" : o.get(++i));
    }
    return out;
  }

  /** Fields that name the user's files; kept by Reset options and across a defaults change. */
  static final List<String> FOLDERS = List.of("-i", "--orientation-input", "--gcp", "--lidar-reference", "--camera-groups");

  static Field field(String flag) {
    for (Field f : all()) if (f.flag.equals(flag)) return f;
    return null;
  }

  /** Help sections that are not options of a new run (other commands or modes). */
  static final Set<String> NOT_RUN_SECTIONS = Set.of("Pipeline steps", "Chessboard calibration", "Read-only surface diagnosis", "Examples");

  /** Options that are commands, set elsewhere in the window, or editor-only. */
  static final Set<String> NOT_RUN_OPTIONS =
      Set.of(
          "--run", "--branch", "--help", "--estimate-memory", "--list-gpus", "--export-confidence", "--export-report", "--report-output",
          "--export-orthomosaic", "--export-orientations", "--gcp-port", "--analyze-run", "--preset");

  /** Second names of an option (the first name is used). */
  static final Set<String> ALIASES = Set.of("-h", "-help", "-f", "-d", "--dist", "-cores", "--ws_census", "--epsg", "-c", "--images", "--output");

  private static volatile List<Field> advanced;

  /** Every option of the help that is not a common field, as fields, in help order and sections. */
  static List<Field> advanced() {
    if (advanced != null) return advanced;
    if (helpText == null) helpText = Options.help();
    Set<String> common = new HashSet<>();
    for (Field f : FIELDS) common.add(f.flag);
    List<Field> out = new ArrayList<>();
    Set<String> seen = new HashSet<>();
    String section = "";
    for (String line : helpText.split("\n", -1)) {
      if (!line.isEmpty() && !line.startsWith(" ") && line.strip().endsWith(":")) {
        section = line.strip().replaceAll("\\s*\\(.*$", "").replaceAll(":$", "");
        continue;
      }
      if (!line.startsWith("  -") || NOT_RUN_SECTIONS.contains(section)) continue;
      for (String part : line.strip().split(",\\s+(?=-)|\\s+/\\s+(?=-)")) {
        String[] tokens = part.strip().split("\\s+", 2);
        String flag = tokens[0], placeholder = tokens.length > 1 ? tokens[1].strip() : "";
        if (ALIASES.contains(flag) || NOT_RUN_OPTIONS.contains(flag) || common.contains(flag) || !seen.add(flag)) continue;
        Kind kind;
        List<String> choices = new ArrayList<>();
        if (placeholder.contains("|")) {
          List<String> words = new ArrayList<>(), others = new ArrayList<>();
          for (String t : placeholder.split("\\|")) (t.matches("[a-z0-9][a-z0-9._-]*") ? words : others).add(t);
          kind = others.isEmpty() ? Kind.CHOICE : Kind.SUGGEST;
          choices.add("");
          choices.addAll(words);
        } else if (placeholder.isEmpty()) kind = Kind.FLAG;
        else if (placeholder.equals("DIR") || placeholder.equals("DIRECTORY")) kind = Kind.DIR;
        else if (placeholder.contains("FILE")) kind = Kind.FILE;
        else kind = Kind.TEXT;
        out.add(refine(new Field(flag, shortDescription(flag), section, kind, List.copyOf(choices))));
      }
    }
    advanced = List.copyOf(out);
    return advanced;
  }

  /** The common fields and then the advanced ones. */
  static List<Field> all() {
    List<Field> out = new ArrayList<>(FIELDS);
    out.addAll(advanced());
    return out;
  }

  /** Sections of the advanced options, in help order. */
  static List<String> advancedSections() {
    LinkedHashSet<String> out = new LinkedHashSet<>();
    for (Field f : advanced()) out.add(f.section);
    return new ArrayList<>(out);
  }

  /** First sentence of an option's description (at most about 110 characters). */
  static String shortDescription(String flag) {
    String[] lines = help(flag).split("\n", 2);
    if (lines.length < 2) return "";
    String d = lines[1].replace('\n', ' ').strip();
    int end = d.indexOf(". ");
    if (end > 0) d = d.substring(0, end + 1);
    return d.length() > 110 ? d.substring(0, 107).strip() + "..." : d;
  }

  static final java.util.regex.Pattern DEFAULT = java.util.regex.Pattern.compile("(?i)\\bdefault(?::|\\s+is)?\\s+([^;)]+?)(?:[;)]|\\.\\s|\\.$|,\\s|$)");

  /** The default an option's help states ("default: 2048" gives "2048"), or "". */
  static String defaultHint(String flag) {
    String[] lines = help(flag).split("\n", 2);
    if (lines.length < 2) return "";
    String first = lines[0].strip().split("[\\s,=]", 2)[0];
    // An entry describing several options states the first one's default; the three orientation
    // input formats share theirs (auto).
    if (!first.equals(flag) && !ALIASES.contains(first) && !first.equals("--orientation-input-format")) return "";
    java.util.regex.Matcher m = DEFAULT.matcher(lines[1].replace('\n', ' '));
    if (!m.find()) return "";
    String d = m.group(1).strip();
    // "Default sgm. ..." / "Default off. on computes": the word after "default" only.
    boolean colon = lines[1].replace('\n', ' ').substring(m.start(), m.start(1)).contains(":");
    if (!colon) d = d.split("\\s+")[0].replaceAll("[.,;:]+$", "");
    return d.length() > 28 ? d.substring(0, 26).strip() + "..." : d;
  }

  private static volatile String helpText;

  /**
   * The help entry of one option: its option line and indented description from {@link
   * Options#help()} ("-i" finds "-i, --images DIR").
   */
  static String help(String flag) {
    if (helpText == null) helpText = Options.help();
    StringBuilder entry = new StringBuilder();
    boolean in = false;
    for (String line : helpText.split("\n", -1)) {
      boolean start = line.startsWith("  -") || line.startsWith("  photogrammetry ");
      if (start) {
        if (in) break;
        String head = line.trim();
        for (String name : head.split(",\\s+(?=-)|\\s+/\\s+(?=-)")) {
          String option = name.split("[\\s=]", 2)[0];
          if (option.equals(flag)) in = true;
        }
      } else if (in && !(line.startsWith("      ") || line.startsWith("                "))) break;
      if (in) entry.append(line.strip()).append('\n');
    }
    return entry.toString().strip();
  }

  /** Tooltip HTML of a help entry (first line bold, the rest wrapped). */
  static String tooltip(String flag) {
    String h = help(flag);
    if (h.isEmpty()) return null;
    String[] lines = h.split("\n", 2);
    String body = lines.length > 1 ? lines[1].replace("\n", " ") : "";
    String note = impliedNote(flag);
    return "<html><body style='width:420px'><b>" + escape(lines[0]) + "</b><br>" + escape(body)
        + (note.isEmpty() ? "" : "<br><br><i>In this window: " + escape(note) + "</i>") + "</body></html>";
  }

  static String escape(String s) {
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;");
  }

  /**
   * Arguments from field values (flag to value; "true" for a set FLAG; blank = CLI default, not
   * passed) in catalog order, then the advanced arguments.
   */
  /** The cleanup limits: used with --tie-point-cleanup custom (and as the pause window's start). */
  static final List<String> TIE_POINT_LIMITS = List.of("--tie-point-max-error", "--tie-point-min-views", "--tie-point-min-angle", "--tie-point-max-removed");

  /** Whether the cleanup mode uses the limit fields (custom or pause); otherwise they are left out. */
  /** The lidar settings: used only with a lidar reference (otherwise left out of the command). */
  static final List<String> LIDAR_OPTIONS = List.of("--lidar-scale", "--lidar-search-shift", "--lidar-search-angle", "--lidar-canopy-weight", "--lidar-vertical-datum");

  static boolean lidarUsed(Map<String, String> values) {
    return !values.getOrDefault("--lidar-reference", "").isBlank();
  }

  static boolean tiePointLimitsUsed(Map<String, String> values) {
    String mode = values.getOrDefault("--tie-point-cleanup", "");
    return mode.equals("custom") || mode.equals("pause");
  }

  /**
   * Flags that are on in the CLI by default: a cleared box passes "FLAG off", so the box always
   * shows what runs (a set box passes the bare flag, as before).
   */
  static final Set<String> ON_BY_DEFAULT = Set.of("--quick-calibration");

  static List<String> arguments(Map<String, String> values, String advanced) {
    List<String> out = new ArrayList<>();
    for (Field f : all()) {
      String v = values.get(f.flag);
      if (ON_BY_DEFAULT.contains(f.flag) && (v == null || !Boolean.parseBoolean(v.strip()))) {
        out.add(f.flag);
        out.add("off");
        continue;
      }
      if (v == null || v.isBlank()) continue;
      if (TIE_POINT_LIMITS.contains(f.flag) && !tiePointLimitsUsed(values)) continue;
      if (LIDAR_OPTIONS.contains(f.flag) && !lidarUsed(values)) continue;
      v = v.strip();
      if (f.kind == Kind.FLAG) {
        if (Boolean.parseBoolean(v)) out.add(f.flag);
      } else {
        out.add(f.flag);
        out.add(v);
      }
    }
    out.addAll(split(advanced));
    return withImplied(out);
  }

  /**
   * Options another choice needs, as {when flag, when value, needed flag, needed value}. The
   * parser requires them (e.g. --dense-method mvsgm needs --fusion-mode image; its other needs,
   * --dense-pipeline standard and --disparity-refinement off, are the defaults).
   */
  static final String[][] IMPLIED = {
    {"--dense-method", "mvsgm", "--fusion-mode", "image"},
    // The tested mvsgm settings (city/forest mvsgm runs); only those that differ from the CLI
    // defaults. Consolidation off: it is not needed with image fusion and its all-pixels disk
    // reservation refuses large blocks.
    {"--dense-method", "mvsgm", "--surface-consolidation", "none"},
    {"--dense-method", "mvsgm", "--fusion-outliers", "robust"},
    {"--dense-method", "mvsgm", "--fusion-strategy", "graph-average"},
    {"--dense-method", "mvsgm", "--subpixel-fit", "equiangular"},
    {"--dense-method", "mvsgm", "--depth-outlier-filter", "edge-aware"},
    {"--dense-method", "mvsgm", "--windowed-mode", "auto"},
    {"--dense-method", "mvsgm", "--min-angle", "4"},
    {"--dense-method", "mvsgm", "--mvsgm-neighbors", "8"},
    {"--dense-method", "mvsgm", "--mvsgm-optimal-neighbors", null},
  };

  /**
   * Adds the options a chosen value needs or is best run with, right after the choice, unless
   * the arguments set that option themselves (for a flag also its --no- form). A value given by
   * the user always wins; a conflict is then reported by the parser, as on the command line.
   */
  static List<String> withImplied(List<String> args) {
    List<String> out = new ArrayList<>(args);
    Map<String, Integer> triggers = new LinkedHashMap<>();
    for (String[] rule : IMPLIED) triggers.putIfAbsent(rule[0] + "\0" + rule[1], -1);
    for (String key : new ArrayList<>(triggers.keySet())) {
      String flag = key.split("\0")[0], value = key.split("\0")[1];
      int at = -1;
      for (int i = 0; i < out.size(); i++) {
        String a = out.get(i);
        if (a.equals(flag) && i + 1 < out.size()) {
          at = out.get(i + 1).equalsIgnoreCase(value) ? i + 2 : -1;
          i++;
        } else if (a.startsWith(flag + "=")) at = a.substring(flag.length() + 1).equalsIgnoreCase(value) ? i + 1 : -1;
      }
      if (at < 0) continue;
      List<String> add = new ArrayList<>();
      for (String[] rule : IMPLIED) {
        if (!(rule[0] + "\0" + rule[1]).equals(key) || given(out, rule[2])) continue;
        add.add(rule[2]);
        if (rule[3] != null) add.add(rule[3]);
      }
      out.addAll(at, add);
    }
    return out;
  }

  private static boolean given(List<String> args, String flag) {
    String negative = "--no-" + flag.substring(2);
    return args.stream().anyMatch(a -> a.equals(flag) || a.startsWith(flag + "=") || a.equals(negative));
  }

  /** Text for a field's tooltip about options it adds, or "". */
  static String impliedNote(String flag) {
    Map<String, List<String>> by = new LinkedHashMap<>();
    for (String[] rule : IMPLIED)
      if (rule[0].equals(flag)) by.computeIfAbsent(rule[1], k -> new ArrayList<>()).add(rule[2] + (rule[3] == null ? "" : " " + rule[3]));
    StringBuilder s = new StringBuilder();
    for (var e : by.entrySet())
      s.append(e.getKey()).append(" adds ").append(String.join(", ", e.getValue())).append(" unless you set them yourself. ");
    return s.toString().strip();
  }

  /**
   * Splits the advanced-arguments text like a shell: whitespace separates, double or single quotes
   * group (and are removed), a backslash before a quote keeps the quote. Backslashes elsewhere are
   * kept (Windows paths).
   */
  static List<String> split(String text) {
    List<String> out = new ArrayList<>();
    if (text == null) return out;
    StringBuilder cur = new StringBuilder();
    boolean any = false;
    char quote = 0;
    for (int i = 0; i < text.length(); i++) {
      char c = text.charAt(i);
      if (c == '\\' && i + 1 < text.length() && (text.charAt(i + 1) == '"' || text.charAt(i + 1) == '\'')) {
        cur.append(text.charAt(++i));
        any = true;
      } else if (quote != 0) {
        if (c == quote) quote = 0;
        else cur.append(c);
      } else if (c == '"' || c == '\'') {
        quote = c;
        any = true;
      } else if (Character.isWhitespace(c)) {
        if (any) out.add(cur.toString());
        cur.setLength(0);
        any = false;
      } else {
        cur.append(c);
        any = true;
      }
    }
    if (quote != 0) throw new IllegalArgumentException("Unclosed quote in the advanced arguments");
    if (any) out.add(cur.toString());
    return out;
  }

  /**
   * Checks the arguments of a new run with the real option parser plus what the parser leaves to
   * the run (the image folder must exist, a project folder must be given). Null when valid, else
   * the message the command line would print.
   */
  static String validate(List<String> args) {
    try {
      Options o = Options.parse(args.toArray(String[]::new));
      if (o.help) return "No arguments: give the image folder and the project folder.";
      if (!args.contains("-i") && !args.contains("--images")) return "Give the image folder.";
      if (o.input != null && !Files.isDirectory(o.input)) return "Image folder not found: " + o.input;
      if (!args.contains("-o") && !args.contains("--output")) return "Give the project folder.";
      return null;
    } catch (IllegalArgumentException e) {
      return e.getMessage();
    } catch (RuntimeException e) {
      return String.valueOf(e);
    }
  }

  /**
   * Options given to a continued or branched run: the CLI parses the run's saved arguments
   * followed by them. Null when valid, else the parser's message.
   */
  static String validateContinued(List<String> saved, List<String> extra) {
    List<String> all = new ArrayList<>(saved);
    all.addAll(extra);
    try {
      Options.parse(all.toArray(String[]::new));
      return null;
    } catch (IllegalArgumentException e) {
      return e.getMessage();
    } catch (RuntimeException e) {
      return String.valueOf(e);
    }
  }

  /**
   * The resolution and memory plan from the verbose output of an --estimate-memory command
   * (memory, quality, filtering and scenario lines; not the step and report chatter).
   */
  static List<String> estimateLines(List<String> output) {
    List<String> out = new ArrayList<>();
    for (String line : output) {
      String t = line.strip();
      if (t.isEmpty() || t.startsWith("[") || t.startsWith(">")) continue;
      if (line.startsWith("Memory") || line.startsWith("Quality=") || line.startsWith("Point filtering") || line.startsWith("Estimates ")
          || line.startsWith("OpenCL") || line.startsWith(" ") || line.startsWith("photogrammetry") || line.toLowerCase(Locale.ROOT).contains("warning")
          || line.toLowerCase(Locale.ROOT).contains("exceed") || line.toLowerCase(Locale.ROOT).contains("budget"))
        out.add(line);
    }
    return out;
  }

  /** Same arguments with the project folder replaced (memory estimate in a scratch project). */
  static List<String> withProject(List<String> args, Path project) {
    List<String> out = new ArrayList<>();
    for (int i = 0; i < args.size(); i++) {
      String a = args.get(i);
      if ((a.equals("-o") || a.equals("--output")) && i + 1 < args.size()) i++;
      else if (a.startsWith("-o=") || a.startsWith("--output=")) continue;
      else out.add(a);
    }
    out.add("-o");
    out.add(project.toString());
    return out;
  }

  /**
   * Remembered launcher settings: user defaults (last options used, without the folders) and one
   * file per project (all fields), under the user's configuration folder.
   */
  static final class Settings {
    final Path dir;

    Settings(Path dir) {
      this.dir = dir;
    }

    /** %APPDATA%\POSE on Windows, $XDG_CONFIG_HOME/pose or ~/.config/pose elsewhere. */
    static Settings user() {
      // -Dpose.gui.settings=DIR keeps the settings elsewhere (portable installs, tests).
      String override = System.getProperty("pose.gui.settings");
      if (override != null && !override.isBlank()) return new Settings(Path.of(override));
      String os = System.getProperty("os.name").toLowerCase(Locale.ROOT);
      String appdata = System.getenv("APPDATA"), xdg = System.getenv("XDG_CONFIG_HOME");
      Path base =
          os.contains("win") && appdata != null
              ? Path.of(appdata, "POSE")
              : xdg != null && !xdg.isBlank() ? Path.of(xdg, "pose") : Path.of(System.getProperty("user.home"), ".config", "pose");
      return new Settings(base.resolve("gui"));
    }

    Path defaultsFile() {
      return dir.resolve("defaults.properties");
    }

    Path projectFile(Path project) {
      return dir.resolve("projects").resolve(hash(project.toAbsolutePath().normalize().toString()) + ".properties");
    }

    /**
     * Saved settings carry this. Version 1 (none) predates the window's defaults; version 2 had
     * the script's fixed compute sizes (SCRIPT_HARDWARE); version 3 sizes them from the hardware.
     */
    static final String VERSION_KEY = "settings.version", VERSION = "3";

    /** The window's defaults: the script's settings plus compute sizes for this computer. */
    Map<String, String> windowDefaults() {
      Map<String, String> out = new LinkedHashMap<>(GuiPreferences.hardwareDefaults(dir));
      out.putAll(SCRIPT_DEFAULTS);
      return out;
    }

    /** The window's defaults with these values' folders kept (Reset options). */
    Map<String, String> reset(Map<String, String> current) {
      Map<String, String> out = windowDefaults();
      for (String f : FOLDERS) if (current.get(f) != null && !current.get(f).isBlank()) out.put(f, current.get(f));
      return out;
    }

    /**
     * Saved values brought to this version: version 1 gives way to the window's defaults (keeping
     * the folders); in version 2 the compute sizes still equal to the script's fixed ones follow the
     * hardware, the rest (also sizes the user changed) stays.
     */
    Map<String, String> upgrade(Map<String, String> saved) {
      if (VERSION.equals(saved.get(VERSION_KEY))) return saved;
      if (!"2".equals(saved.get(VERSION_KEY))) return reset(saved);
      Map<String, String> out = new LinkedHashMap<>(saved);
      Map<String, String> hardware = GuiPreferences.hardwareDefaults(dir);
      for (var e : SCRIPT_HARDWARE.entrySet())
        if (e.getValue().equals(out.get(e.getKey()))) {
          if (hardware.containsKey(e.getKey())) out.put(e.getKey(), hardware.get(e.getKey()));
          else out.remove(e.getKey());
        }
      if (hardware.containsKey("--compute") && SCRIPT_HARDWARE.get("--gpu-memory-mb").equals(saved.get("--gpu-memory-mb"))
          && out.getOrDefault("--compute", "").isBlank())
        out.put("--compute", hardware.get("--compute"));
      out.put(VERSION_KEY, VERSION);
      return out;
    }

    /** Values for a project: its own file, else the user's defaults, else the window's defaults. */
    Map<String, String> load(Path project) {
      Map<String, String> own = project == null ? Map.of() : read(projectFile(project));
      if (!own.isEmpty()) return upgrade(own);
      // Settings > Preferences: new projects may always start with the POSE defaults.
      if (GuiPreferences.load(dir).newProjects.equals("pose")) return windowDefaults();
      Map<String, String> user = read(defaultsFile());
      return user.isEmpty() ? windowDefaults() : upgrade(user);
    }

    Map<String, String> defaults() {
      return read(defaultsFile());
    }

    /** Saves the values for the project and, without the folders, as the user defaults. */
    void save(Path project, Map<String, String> values) throws IOException {
      values = new TreeMap<>(values);
      values.put(VERSION_KEY, VERSION);
      if (project != null) {
        Map<String, String> withPath = new TreeMap<>(values);
        withPath.put("project.path", project.toAbsolutePath().normalize().toString());
        write(projectFile(project), withPath);
      }
      Map<String, String> defaults = new TreeMap<>(values);
      defaults.remove("-i");
      defaults.remove("-o");
      defaults.remove("--orientation-input");
      defaults.remove("--gcp");
      defaults.remove("--lidar-reference");
      write(defaultsFile(), defaults);
      remember(project);
    }

    /** Recently used projects, newest first. */
    List<String> recent() {
      Map<String, String> r = read(dir.resolve("recent.properties"));
      List<String> out = new ArrayList<>();
      for (int i = 0; i < 20 && r.containsKey("" + i); i++) out.add(r.get("" + i));
      return out;
    }

    void remember(Path project) throws IOException {
      if (project == null) return;
      List<String> list = new ArrayList<>(recent());
      String p = project.toAbsolutePath().normalize().toString();
      list.remove(p);
      list.add(0, p);
      Map<String, String> r = new TreeMap<>();
      for (int i = 0; i < Math.min(10, list.size()); i++) r.put("" + i, list.get(i));
      write(dir.resolve("recent.properties"), r);
    }

    static Map<String, String> read(Path file) {
      Map<String, String> out = new TreeMap<>();
      if (!Files.isRegularFile(file)) return out;
      Properties p = new Properties();
      try (Reader r = Files.newBufferedReader(file, StandardCharsets.UTF_8)) {
        p.load(r);
      } catch (IOException e) {
        return out;
      }
      for (String k : p.stringPropertyNames()) out.put(k, p.getProperty(k));
      return out;
    }

    static void write(Path file, Map<String, String> values) throws IOException {
      Files.createDirectories(file.getParent());
      Properties p = new Properties();
      p.putAll(values);
      Path tmp = file.resolveSibling(file.getFileName() + ".tmp");
      try (Writer w = Files.newBufferedWriter(tmp, StandardCharsets.UTF_8)) {
        p.store(w, "POSE launcher settings");
      }
      Files.move(tmp, file, StandardCopyOption.REPLACE_EXISTING);
    }

    static String hash(String s) {
      try {
        byte[] d = MessageDigest.getInstance("SHA-256").digest(s.getBytes(StandardCharsets.UTF_8));
        return HexFormat.of().formatHex(d, 0, 10);
      } catch (java.security.NoSuchAlgorithmException e) {
        throw new IllegalStateException(e);
      }
    }
  }
}
