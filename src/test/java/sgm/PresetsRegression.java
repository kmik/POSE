package sgm;

import java.util.*;

/** --preset: expansion, platform/scene combination, explicit options win, errors. */
final class PresetsRegression {
  private static int checks;

  private static void check(boolean value, String why) {
    checks++;
    if (!value) throw new AssertionError(why);
  }

  public static void main(String[] args) {
    System.out.println("PASS presets: " + run());
  }

  static Options parse(String... a) {
    List<String> all = new ArrayList<>(List.of("-i", ".", "-o", "presets-check"));
    all.addAll(List.of(a));
    return Options.parse(all.toArray(String[]::new));
  }

  static boolean refused(String... a) {
    try {
      parse(a);
      return false;
    } catch (IllegalArgumentException e) {
      return true;
    }
  }

  static int run() {
    checks = 0;
    Options drone = parse("--preset", "drone");
    check(drone.denseMethod.equals("mvsgm") && drone.fusionMode.equals("image") && drone.fusionStrategy.equals("graph-average"),
        "Every preset uses the multi-view SGM pipeline");
    check(drone.mvsgmNeighbors == 8 && drone.fusionTolerance.equals("sigma") && drone.minViews == 2, "Drone values");
    for (String p : List.of("drone", "aircraft", "drone,forest", "aircraft,city")) {
      Options q = parse("--preset", p);
      check(q.matchingAlgorithm.equals("sift") && q.descriptorMatcher.equals("flann"), p + ": SIFT with FLANN matching");
      check(q.features == 20000 && q.ratio == .85 && q.neighbors == 5 && q.minForwardOverlap == 50 && q.quickCalibration, p + ": the study's alignment settings");
    }
    check(drone.step == 1 && parse("--dense-method", "mvsgm", "--fusion-mode", "image").step == 1
        && parse("--dense-method", "mvsgm", "--fusion-mode", "image", "--step", "2").step == 2
        && parse().step == 2, "mvsgm defaults to --step 1 (pair SGM keeps 2); an explicit --step applies");
    Options aircraft = parse("--preset", "aircraft");
    check(aircraft.minViews == 3 && aircraft.mvsgmConsistentViews == 2 && aircraft.mvsgmCensus == 3, "Aircraft values (very_large setup)");
    // A scene adds to its platform; alone it uses drone; list order does not matter.
    for (String scene : Presets.SCENE.keySet()) {
      List<String> alone = Presets.options(scene), withDrone = Presets.options("drone," + scene), reversed = Presets.options(scene + ",drone");
      check(alone.equals(withDrone) && withDrone.equals(reversed), scene + ": scene alone = drone + scene, in any order");
      check(!parse("--preset", "aircraft," + scene).toString().isEmpty(), scene + " combines with aircraft");
    }
    // Explicit options win, before or after --preset; the = form works.
    check(parse("--mvsgm-neighbors", "5", "--preset", "drone").mvsgmNeighbors == 5, "Explicit option before --preset wins");
    check(parse("--preset", "drone", "--mvsgm-neighbors", "5").mvsgmNeighbors == 5, "Explicit option after --preset wins");
    check(parse("--preset=aircraft").minViews == 3, "--preset=LIST form");
    check(parse("--preset", "drone", "--preset", "aircraft").minViews == 3, "A later --preset replaces an earlier one");
    check(Arrays.equals(Presets.expand(new String[] {"-i", "x"}), new String[] {"-i", "x"}), "No --preset: arguments unchanged");
    // Errors.
    check(refused("--preset", "boat"), "Unknown preset refused");
    check(refused("--preset", "drone,aircraft"), "Two platforms refused");
    check(refused("--preset", "forest,city"), "Two scenes refused");
    check(refused("--preset"), "Missing value refused");
    // Every preset combination is a valid option set.
    for (String p : Presets.PLATFORM.keySet()) {
      parse("--preset", p);
      for (String s : Presets.SCENE.keySet()) parse("--preset", p + "," + s);
      checks++;
    }
    // The window's Preset menu fills fields: every preset option has a field, choice fields offer
    // the value, and the command built from the filled fields means the same as --preset.
    check(GuiOptions.field("--preset") == null, "--preset is not a field (the menu fills the fields instead)");
    for (String choice : GuiOptions.PRESET_CHOICES) {
      Map<String, String> v = GuiOptions.presetValues(choice);
      for (var e : v.entrySet()) {
        GuiOptions.Field f = GuiOptions.field(e.getKey());
        check(f != null, choice + ": " + e.getKey() + " has a GUI field");
        if (f.kind() == GuiOptions.Kind.CHOICE && !e.getValue().isEmpty()) check(f.choices().contains(e.getValue()), choice + ": " + e.getKey() + " offers " + e.getValue());
      }
      Map<String, String> fields = new LinkedHashMap<>(v);
      fields.put("-i", ".");
      fields.put("-o", "presets-check");
      Options fromGui = Options.parse(GuiOptions.arguments(fields, "").toArray(String[]::new));
      Options fromCli = parse("--preset", choice.replace(" ", ""));
      check(fromGui.denseMethod.equals(fromCli.denseMethod) && fromGui.mvsgmNeighbors == fromCli.mvsgmNeighbors
          && fromGui.mvsgmCensus == fromCli.mvsgmCensus && fromGui.mvsgmConsistentViews == fromCli.mvsgmConsistentViews
          && fromGui.p1 == fromCli.p1 && fromGui.p2 == fromCli.p2 && fromGui.minViews == fromCli.minViews && fromGui.minAngle == fromCli.minAngle
          && fromGui.step == fromCli.step && fromGui.fusionTolerance.equals(fromCli.fusionTolerance)
          && fromGui.fusionOutliers.equals(fromCli.fusionOutliers) && fromGui.fusionStrategy.equals(fromCli.fusionStrategy)
          && fromGui.mvsgmOptimalNeighbors == fromCli.mvsgmOptimalNeighbors && fromCli.mvsgmOptimalNeighbors,
          choice + ": the GUI's filled fields give the same settings as --preset");
    }
    return checks;
  }
}
