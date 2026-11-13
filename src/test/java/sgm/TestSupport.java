package sgm;

import java.util.*;

/**
 * Existing audit regressions freeze their original CPU/debug/cache and per-candidate output
 * contract. SurfaceFusionRegression separately exercises the new production defaults, including the
 * pipeline.
 */
final class TestSupport {
  static Options options() {
    Options o = new Options();
    o.pairSelection = "sequential";
    o.diagnostics = true;
    o.compatibilityCopies = true;
    o.compute = "cpu";
    o.pointAttributes = "none";
    o.storage = "debug";
    o.cache = true;
    o.depthRefinementMethod = "weighted";
    o.surfaceConsolidation = "none";
    // Fixed 8-iteration bundle budget, as these regressions were frozen with.
    o.bundleIterationsExplicit = true;
    return o;
  }

  static int run(String[] arguments) {
    if (arguments.length == 0) return Main.run(arguments);
    List<String> args =
        new ArrayList<>(
            List.of(
                "--pair-selection",
                "sequential",
                "--diagnostics",
                "--compatibility-copies",
                "--point-attributes",
                "none",
                "--storage",
                "debug",
                "--cache",
                "--compute",
                "cpu",
                "--depth-refinement",
                "weighted",
                "--surface-consolidation",
                "none",
                "--bundle-iterations",
                "8"));
    args.addAll(Arrays.asList(arguments));
    return Main.run(args.toArray(String[]::new));
  }
}
