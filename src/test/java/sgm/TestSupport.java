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
    o.compute = "cpu";
    o.pointAttributes = "none";
    o.storage = "debug";
    o.cache = true;
    o.depthRefinementMethod = "weighted";
    o.surfaceConsolidation = "none";
    return o;
  }

  static int run(String[] arguments) {
    if (arguments.length == 0) return Main.run(arguments);
    List<String> args =
        new ArrayList<>(
            List.of(
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
                "none"));
    args.addAll(Arrays.asList(arguments));
    return Main.run(args.toArray(String[]::new));
  }
}
