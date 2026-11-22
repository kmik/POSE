package sgm;

import java.util.*;

/**
 * {@code --preset LIST}: tested dense and fusion settings for a platform (drone, aircraft) and a
 * scene (forest, city). The preset's options go before the command line's, so options given
 * explicitly still win. A scene preset adds to its platform preset; without a platform, drone is
 * used. Order in the list does not matter: platform first, then scene.
 *
 * <p>Values from the 2026-11-21 parameter study (CloudScore against Metashape for city, against
 * NLS lidar for forest_9; stage A one-option screening plus a check of each preset as a whole).
 * The aircraft preset is a tested large-format setup (four-frame block); no aerial reference was available
 * to search it.
 */
final class Presets {
  private Presets() {}

  /**
   * Every preset: the alignment settings of the study's runs (the tested setup; not searched yet)
   * and the multi-view SGM pipeline. Without them a command-line preset would align with the CLI
   * defaults (8000 features, ratio 0.75, 3 neighbours, matcher auto).
   */
  static final List<String> PIPELINE =
      List.of(
          "--matching-algorithm", "sift", "--descriptor-matcher", "flann", "--features", "20000", "--ratio", "0.85",
          "--neighbors", "5", "--min-forward-overlap", "50", "--quick-calibration",
          "--dense-method", "mvsgm", "--fusion-mode", "image", "--fusion-strategy", "graph-average",
          "--fusion-outliers", "robust", "--subpixel-fit", "equiangular", "--depth-outlier-filter", "edge-aware",
          "--surface-consolidation", "none", "--filtering", "mild", "--confidence-threshold", "0", "--step", "1",
          // On in the window's mvsgm defaults; measured with the presets: no change on forest_9, on city
          // stable error -0.04 cm and p90 +0.07 cm.
          "--mvsgm-optimal-neighbors");

  static final Map<String, List<String>> PLATFORM = new LinkedHashMap<>();
  static final Map<String, List<String>> SCENE = new LinkedHashMap<>();

  static {
    PLATFORM.put("drone", List.of(
        "--min-views", "2", "--min-angle", "4", "--mvsgm-neighbors", "8", "--mvsgm-best-k", "3",
        "--mvsgm-census-window", "5", "--mvsgm-consistent-views", "1", "--mvsgm-consistency-labels", "1",
        "--fusion-tolerance", "sigma"));
    PLATFORM.put("aircraft", List.of(
        "--min-views", "3", "--min-angle", "4", "--mvsgm-neighbors", "6", "--mvsgm-best-k", "3",
        "--mvsgm-census-window", "3", "--mvsgm-consistent-views", "2", "--mvsgm-consistency-labels", "1"));
    SCENE.put("forest", List.of("--p1", "8", "--p2", "48", "--fusion-outliers", "reference"));
    SCENE.put("city", List.of("--p1", "2", "--mvsgm-census-window", "3"));
  }

  static final String NAMES = "drone, aircraft, forest, city";

  /** The options of a preset list ("drone,forest"); platform first, then scene. */
  static List<String> options(String list) {
    String platform = null, scene = null;
    for (String raw : list.split(",")) {
      String name = raw.strip().toLowerCase(Locale.ROOT);
      if (name.isEmpty()) continue;
      if (PLATFORM.containsKey(name)) {
        if (platform != null && !platform.equals(name))
          throw new IllegalArgumentException("--preset names two platforms: " + platform + " and " + name);
        platform = name;
      } else if (SCENE.containsKey(name)) {
        if (scene != null && !scene.equals(name))
          throw new IllegalArgumentException("--preset names two scenes: " + scene + " and " + name);
        scene = name;
      } else throw new IllegalArgumentException("Unknown --preset " + raw.strip() + " (use " + NAMES + ")");
    }
    if (platform == null && scene == null) throw new IllegalArgumentException("--preset needs one of " + NAMES);
    List<String> out = new ArrayList<>(PIPELINE);
    out.addAll(PLATFORM.get(platform == null ? "drone" : platform));
    if (scene != null) out.addAll(SCENE.get(scene));
    return out;
  }

  /**
   * The arguments with every {@code --preset LIST} (or {@code --preset=LIST}) replaced: the
   * presets' options first, then the remaining arguments in their order, so explicit options win.
   * A later --preset replaces an earlier one.
   */
  static String[] expand(String[] args) {
    String list = null;
    List<String> rest = new ArrayList<>(args.length);
    for (int i = 0; i < args.length; i++) {
      String a = args[i];
      if (a.equals("--preset")) {
        if (i + 1 >= args.length) throw new IllegalArgumentException("--preset needs a value (" + NAMES + ")");
        list = args[++i];
      } else if (a.startsWith("--preset=")) list = a.substring("--preset=".length());
      else rest.add(a);
    }
    if (list == null) return args;
    List<String> out = new ArrayList<>(options(list));
    out.addAll(rest);
    return out.toArray(String[]::new);
  }
}
