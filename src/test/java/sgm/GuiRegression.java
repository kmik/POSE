package sgm;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.*;

/**
 * The windowless parts of the POSE GUI: child command building, output splitting, quoting, stopping
 * a process tree, opening files, the GCP editor model on a synthetic state. No window is opened.
 */
final class GuiRegression {
  private static int checks;

  /** PatchMatch: study settings implied; image-fusion-only options adapted without image fusion. */
  private static void patchmatchImplied(Path images) {
    List<String> base = List.of("-i", images.toString(), "-o", "p", "--dense-method", "patchmatch");
    List<String> plain = GuiOptions.withImplied(base);
    for (String[] kv : new String[][] {{"--fusion-mode", "reference"}, {"--patchmatch-neighbors", "score"},
        {"--patchmatch-view-cost", "top4"}, {"--patchmatch-smooth", "4"}, {"--patchmatch-views", "8"},
        {"--patchmatch-consistent-views", "2"}, {"--patchmatch-speckle", "200"}})
      check(plain.indexOf(kv[0]) > 0 && plain.get(plain.indexOf(kv[0]) + 1).equals(kv[1]), "patchmatch adds " + kv[0] + " " + kv[1] + ": " + plain);
    check(GuiOptions.validate(plain) == null, "patchmatch settings valid: " + GuiOptions.validate(plain));
    // The window defaults are tuned for mvsgm (image fusion, edge-aware filter): valid as they are.
    Map<String, String> window = new HashMap<>(GuiOptions.SCRIPT_DEFAULTS);
    window.put("-i", images.toString());
    window.put("-o", "p");
    window.put("--dense-method", "patchmatch");
    List<String> defaults = GuiOptions.arguments(window, "");
    check(GuiOptions.validate(defaults) == null, "patchmatch with the window defaults is valid: " + GuiOptions.validate(defaults));
    // Without image fusion the image-only opt-ins are switched off so the run can go.
    List<String> reference = GuiOptions.arguments(window,
        "--fusion-mode reference --thin-surfaces --fusion-seed-min-angle 2 --point-attributes surface,height_sigma,views --windowed-mode on");
    check(GuiOptions.validate(reference) == null, "patchmatch without image fusion is adapted to run: " + GuiOptions.validate(reference));
    Options o = Options.parse(reference.toArray(String[]::new));
    check(o.depthOutlierFilter.equals("off") && !o.thinSurfaceFusion() && !o.pointEvidence(), "image-only options off");
    check(o.pointFields().contains(PointFields.Field.VIEWS) && o.pointFields().contains(PointFields.Field.NORMAL), "other attributes kept");
    check(o.windowedMode.equals("auto"), "windowed mode on reset for patchmatch");
    check(o.patchmatchTopViews == 4 && o.patchmatchSmooth == 4 && o.patchmatchNeighbors.equals("score")
        && o.patchmatchViews == 8 && o.patchmatchConsistentViews == 2 && o.patchmatchSpeckle == 200, "study settings applied");
    // With image fusion nothing image-only is touched.
    List<String> image = GuiOptions.arguments(window, "--point-attributes height_sigma");
    Options io = Options.parse(image.toArray(String[]::new));
    check(io.fusionMode.equals("image") && io.depthOutlierFilter.equals("edge-aware") && io.pointEvidence(), "image fusion keeps its options");
    // A user's own values win over the implied ones.
    List<String> mine = GuiOptions.withImplied(List.of("--dense-method", "patchmatch", "--patchmatch-smooth", "0", "--patchmatch-view-cost", "best"));
    check(Collections.frequency(mine, "--patchmatch-smooth") == 1 && Collections.frequency(mine, "--patchmatch-view-cost") == 1, "own patchmatch values kept: " + mine);
    // Branch from a saved mvsgm-style run: the saved image-only settings are adapted as well.
    List<String> saved = List.of("-i", images.toString(), "-o", "p", "--fusion-mode", "image", "--depth-outlier-filter", "edge-aware", "--fusion-tolerance", "sigma");
    List<String> branch = GuiOptions.withImplied(saved, List.of("--dense-method", "patchmatch"));
    check(GuiOptions.validateContinued(saved, branch) == null, "branch to patchmatch is adapted to run: " + GuiOptions.validateContinued(saved, branch));
    check(GuiOptions.validateContinued(saved, GuiOptions.withImplied(List.of("--dense-method", "patchmatch"))) != null,
        "without the saved context the conflict would remain (context is needed)");
    // Other methods are never adapted.
    List<String> sgm = List.of("--dense-method", "sgm", "--windowed-mode", "on", "--depth-outlier-filter", "edge-aware");
    check(GuiOptions.withImplied(sgm).equals(sgm), "sgm arguments unchanged");
    check(GuiOptions.tooltip("--dense-method").contains("patchmatch adds --fusion-mode reference"), "tooltip lists the patchmatch settings");
    // Switching the dense method field: patchmatch gets reference fusion, back to mvsgm gets image.
    check(GuiOptions.followDenseMethod("mvsgm", "patchmatch", Map.of("--fusion-mode", "image")).equals(Map.of("--fusion-mode", "reference")), "to patchmatch: reference");
    check(GuiOptions.followDenseMethod("", "patchmatch", Map.of()).equals(Map.of("--fusion-mode", "reference")), "to patchmatch from an empty field: reference");
    check(GuiOptions.followDenseMethod("mvsgm", "patchmatch", Map.of("--fusion-mode", "incremental")).isEmpty(), "another chosen mode stays");
    check(GuiOptions.followDenseMethod("patchmatch", "patchmatch", Map.of("--fusion-mode", "image")).isEmpty(), "image chosen with patchmatch stays");
    check(GuiOptions.followDenseMethod("patchmatch", "mvsgm", Map.of("--fusion-mode", "reference")).equals(Map.of("--fusion-mode", "image")), "back to mvsgm: image");
    check(GuiOptions.followDenseMethod("patchmatch", "sgm", Map.of("--fusion-mode", "reference")).isEmpty(), "sgm keeps reference");
  }

  private static void check(boolean ok, String message) {
    checks++;
    if (!ok) throw new AssertionError(message);
  }

  public static void main(String[] args) throws Exception {
    if (args.length > 0 && args[0].equals("sleeper")) {
      sleeper(args.length > 1 && args[1].equals("spawn"));
      return;
    }
    Path root = Files.createTempDirectory(Path.of("target"), "gui-");
    System.out.println("PASS GUI logic: " + run(root) + " checks");
  }

  static int run(Path root) throws Exception {
    checks = 0;
    Files.createDirectories(root);
    process();
    tree();
    editorModel();
    files(root);
    launcherOptions(root);
    settings(root.resolve("settings ä"));
    preferences(root.resolve("preferences ä"));
    about(root.resolve("about"));
    imagesModel(root.resolve("images ä"));
    projectReaders(root.resolve("project ö"));
    cloudViewer(root.resolve("cloud ä"));
    childRun();
    return checks;
  }

  /** Field catalog, tooltips from the CLI help, arguments, validation with the real parser. */
  static void launcherOptions(Path root) throws IOException {
    check(GuiOptions.FIELDS.size() >= 20, "about 20 common options as fields");
    for (GuiOptions.Field f : GuiOptions.FIELDS) {
      check(!f.help().isEmpty() && f.help().contains(f.flag()), "help entry for " + f.flag());
      check(GuiOptions.tooltip(f.flag()).startsWith("<html>"), "tooltip for " + f.flag());
    }
    check(GuiOptions.help("-i").startsWith("-i, --images DIR") && GuiOptions.help("--crs").contains("EPSG"), "help entries found by any alias");
    check(GuiOptions.help("--no-such-option").isEmpty(), "unknown option has no help");
    Path images = Files.createDirectories(root.resolve("kuvat ja muut"));
    // Every choice and flag of the catalog, as the window builds it, is accepted by the option parser.
    for (GuiOptions.Field f : GuiOptions.FIELDS) {
      if (f.kind() == GuiOptions.Kind.CHOICE)
        for (String choice : f.choices()) {
          if (choice.isEmpty()) continue;
          List<String> args = GuiOptions.arguments(Map.of("-i", images.toString(), "-o", root.resolve("p").toString(), f.flag(), choice), "");
          check(GuiOptions.validate(args) == null, f.flag() + " " + choice + " accepted: " + GuiOptions.validate(args));
        }
      if (f.kind() == GuiOptions.Kind.FLAG && !f.flag().equals("--orthomosaic") && !f.flag().equals("--classify-ground"))
        check(GuiOptions.validate(List.of("-i", images.toString(), "-o", "p", f.flag())) == null, f.flag() + " accepted");
    }
    // Every other option as an advanced field, generated from the help text.
    List<GuiOptions.Field> adv = GuiOptions.advanced();
    check(adv.size() > 200, "most options are advanced fields: " + adv.size());
    Set<String> flags = new HashSet<>();
    for (GuiOptions.Field f : GuiOptions.all()) check(flags.add(f.flag()), "each option once: " + f.flag());
    for (String gone : List.of("--run", "--branch", "--help", "-h", "--export-report", "--analyze-run", "--analysis-samples", "--calib", "--board-rows", "--epsg", "-d", "--ws_census", "--images", "-i", "--gcp-port"))
      check(adv.stream().noneMatch(f -> f.flag().equals(gone)), "not an advanced field: " + gone);
    for (String there : List.of("--features", "--aerial-overlap", "--min-side-overlap", "--poses-format", "--no-cache", "--report", "--orthomosaic-resolution", "--mvsgm-cameras-in-flight"))
      check(adv.stream().anyMatch(f -> f.flag().equals(there)), "advanced field: " + there);
    check(!GuiOptions.advancedSections().contains("Pipeline steps") && GuiOptions.advancedSections().contains("Dense stereo / SGM"), "sections from the help");
    check(GuiOptions.field("--report").kind() == GuiOptions.Kind.CHOICE && GuiOptions.field("--mvsgm-cameras-in-flight").kind() == GuiOptions.Kind.SUGGEST
        && GuiOptions.field("--resume").kind() == GuiOptions.Kind.FLAG && GuiOptions.field("--calibration").kind() == GuiOptions.Kind.FILE, "field kinds from the placeholders");
    check(GuiOptions.defaultHint("--report").equals("pdf") && GuiOptions.defaultHint("--features").equals("8000") && GuiOptions.defaultHint("--dense-pipeline").equals("standard")
        && GuiOptions.defaultHint("--aerial-overlap").isEmpty(), "default hints (only for the first option of a shared entry)");
    for (GuiOptions.Field f : adv) {
      List<String> args = new ArrayList<>(List.of("-i", images.toString(), "-o", root.resolve("p").toString(), f.flag()));
      switch (f.kind()) {
        case FLAG -> {}
        case CHOICE, SUGGEST -> args.add(f.choices().size() > 1 ? f.choices().get(1) : "1");
        case FILE, DIR -> args.add(images.toString());
        default -> args.add("1");
      }
      String problem = GuiOptions.validate(args);
      check(problem == null || !problem.startsWith("Unknown option"), "the parser knows " + f.flag() + ": " + problem);
    }
    // Orientation formats: only the supported ones (OrientationIO.FORMATS), one for input, any number for output.
    List<String> formats = Arrays.asList(OrientationIO.FORMATS.split(","));
    for (String flag : List.of("--orientation-input-format", "--calibration-format", "--poses-format", "--orientation-output-format", "--interior-output-format", "--exterior-output-format")) {
      GuiOptions.Field f = GuiOptions.field(flag);
      boolean output = flag.contains("output");
      check(f.kind() == (output ? GuiOptions.Kind.MULTI : GuiOptions.Kind.CHOICE) && f.choices().get(0).isEmpty() && f.choices().subList(1, f.choices().size()).equals(formats), flag + " offers exactly the supported formats");
      for (String format : formats)
        check(GuiOptions.validate(List.of("-i", images.toString(), "-o", "p", flag, format)) == null, flag + " " + format + " accepted");
    }
    check(GuiOptions.defaultHint("--poses-format").equals("auto") && GuiOptions.defaultHint("--calibration-format").equals("auto"), "input formats default to auto");
    check(GuiOptions.multi(List.of("colmap", "bogus", "CSV", " opencv "), formats).equals("csv,colmap,opencv") && GuiOptions.multi(List.of("bogus"), formats).isEmpty(), "multi value: supported formats only, in list order");
    List<String> exports = GuiOptions.arguments(Map.of("-i", images.toString(), "-o", "p", "--orientation-output-format", "csv,colmap", "--interior-output-format", "opencv"), "");
    check(GuiOptions.validate(exports) == null && exports.contains("csv,colmap"), "several export formats in one option");
    Map<String, String> withAdvanced = new HashMap<>(Map.of("-i", images.toString(), "-o", "p", "--quality", "high", "--features", "40000", "--storage", "minimal"));
    List<String> ordered = GuiOptions.arguments(withAdvanced, "--ratio 0.8");
    check(ordered.indexOf("--quality") < ordered.indexOf("--features") && ordered.indexOf("--features") < ordered.indexOf("--ratio") && ordered.contains("--storage"), "common fields, then advanced fields, then extra arguments: " + ordered);
    check(GuiOptions.validate(ordered) == null, "advanced values parse: " + GuiOptions.validate(ordered));
    Map<String, String> ownConsolidation = new HashMap<>(Map.of("-i", images.toString(), "-o", "p", "--dense-method", "mvsgm", "--surface-consolidation", "normal"));
    List<String> keepOwn = GuiOptions.arguments(ownConsolidation, "");
    check(Collections.frequency(keepOwn, "--surface-consolidation") == 1 && keepOwn.get(keepOwn.indexOf("--surface-consolidation") + 1).equals("normal"), "an advanced field wins over the mvsgm preset");
    // mvsgm: the options it needs are added (the settings of the user's report that failed before).
    Map<String, String> mv = new HashMap<>();
    for (String[] kv : new String[][] {{"-i", images.toString()}, {"-o", root.resolve("p").toString()}, {"--quality", "high"}, {"--dense-method", "mvsgm"}, {"--filtering", "mild"},
        {"--min-views", "2"}, {"--quick-calibration", "true"}, {"--orthomosaic", "true"}, {"--compute", "auto"}, {"--gpu-memory-mb", "8000"}, {"--fusion-gpu-normals", "on"},
        {"--cores", "12"}, {"--memory-mb", "16000"}, {"--ram-limit-mb", "16000"}, {"--ply", "off"}, {"--point-confidence", "on"}, {"--verbose", "true"}}) mv.put(kv[0], kv[1]);
    List<String> mvArgs = GuiOptions.arguments(mv, "");
    int at = mvArgs.indexOf("mvsgm");
    check(at > 0 && mvArgs.subList(at + 1, at + 3).equals(List.of("--fusion-mode", "image")), "mvsgm adds --fusion-mode image after it: " + mvArgs);
    check(GuiOptions.validate(mvArgs) == null, "mvsgm settings valid: " + GuiOptions.validate(mvArgs));
    check(Collections.frequency(GuiOptions.withImplied(mvArgs), "--fusion-mode") == 1, "added once");
    check(!GuiOptions.arguments(Map.of("--dense-method", "sgm"), "").contains("--fusion-mode"), "nothing added for sgm");
    List<String> own = GuiOptions.arguments(mv, "--fusion-mode reference");
    check(Collections.frequency(own, "--fusion-mode") == 1 && GuiOptions.validate(own) != null, "an explicit --fusion-mode is kept; the parser reports the conflict");
    List<String> eq = GuiOptions.arguments(Map.of("-i", images.toString(), "-o", "p", "--dense-method", "sgm"), "--dense-method=mvsgm");
    check(eq.get(eq.indexOf("--dense-method=mvsgm") + 1).equals("--fusion-mode") && GuiOptions.validate(eq) == null, "mvsgm given in the advanced box (= form) also gets it: " + eq);
    check(!GuiOptions.withImplied(List.of("--dense-method", "mvsgm", "--dense-method", "sgm")).contains("--fusion-mode"), "the last choice counts");
    check(GuiOptions.validateContinued(List.of("-i", images.toString(), "-o", "p", "--fusion-mode", "reference"), GuiOptions.withImplied(List.of("--dense-method", "mvsgm"))) == null, "branch to mvsgm overrides a saved --fusion-mode");
    check(GuiOptions.tooltip("--dense-method").contains("mvsgm adds --fusion-mode image"), "the field's tooltip says so");
    patchmatchImplied(images);
    // The mvsgm preset: the tested settings, a user's own value always wins.
    for (String needed : List.of("--surface-consolidation", "--fusion-outliers", "--fusion-strategy", "--subpixel-fit", "--depth-outlier-filter", "--mvsgm-neighbors", "--mvsgm-optimal-neighbors"))
      check(mvArgs.contains(needed), "mvsgm preset has " + needed + ": " + mvArgs);
    check(mvArgs.get(mvArgs.indexOf("--surface-consolidation") + 1).equals("none"), "consolidation off with mvsgm");
    List<String> mine = GuiOptions.arguments(mv, "--surface-consolidation normal --no-mvsgm-optimal-neighbors --mvsgm-neighbors=6");
    check(Collections.frequency(mine, "--surface-consolidation") == 1 && mine.get(mine.indexOf("--surface-consolidation") + 1).equals("normal")
        && !mine.contains("--mvsgm-optimal-neighbors") && !mine.contains("--mvsgm-neighbors") && mine.contains("--fusion-strategy"), "own values (also --no- and = forms) win; the rest stays: " + mine);
    check(GuiOptions.validate(mine) == null, "own values parse");
    // The failure that led here: forest_10 at high quality, 86 cameras, 113 GB free. The CLI
    // defaults reserve the all-pixels worst case (consolidation records); the preset does not.
    long pixels = 450_767_087L, maps = 16_616_221_680L, free = 113_513_066_496L;
    Options cliDefaults = Options.parse(new String[] {"-i", images.toString(), "-o", "p", "--dense-method", "mvsgm", "--fusion-mode", "image", "--point-confidence", "on"});
    Options preset = Options.parse(GuiOptions.arguments(Map.of("-i", images.toString(), "-o", "p", "--dense-method", "mvsgm", "--point-confidence", "on"), "").toArray(String[]::new));
    long refused = MvsgmDiskPlan.of(pixels, 86, maps, 0, cliDefaults).startup(), admitted = MvsgmDiskPlan.of(pixels, 86, maps, 0, preset).startup();
    check(refused > free && admitted < free / 4, "disk admission: CLI defaults " + (refused >> 30) + " GiB, mvsgm preset " + (admitted >> 30) + " GiB of " + (free >> 30) + " GiB free");
    Map<String, String> values = new HashMap<>();
    values.put("-i", images.toString());
    values.put("-o", "C:/out dir");
    values.put("--quality", "high");
    values.put("--cores", " ");
    values.put("--quick-calibration", "true");
    values.put("--orthomosaic", "false");
    values.put("--ply", "off");
    List<String> args = GuiOptions.arguments(values, "--features 40000 --gpu-summary-format \"a b\"");
    check(args.equals(List.of("-i", images.toString(), "-o", "C:/out dir", "--quality", "high", "--quick-calibration", "--ply", "off", "--features", "40000", "--gpu-summary-format", "a b")), "arguments in catalog order, advanced last: " + args);
    check(GuiOptions.split("a \"b c\" 'd e' C:\\x\\y \\\"q\\\" \"\"").equals(List.of("a", "b c", "d e", "C:\\x\\y", "\"q\"", "")), "advanced arguments split like a shell");
    boolean unclosed = false;
    try {
      GuiOptions.split("--x \"open");
    } catch (IllegalArgumentException e) {
      unclosed = true;
    }
    check(unclosed, "unclosed quote refused");
    check(GuiOptions.validate(List.of("-i", images.toString(), "-o", "p", "--quality", "fantastic")) != null, "bad value reported by the parser");
    check(GuiOptions.validate(List.of("-o", "p")) != null, "image folder required: " + GuiOptions.validate(List.of("-o", "p")));
    check(GuiOptions.validate(List.of("-i", root.resolve("missing").toString(), "-o", "p")) != null, "missing image folder");
    check(GuiOptions.validate(List.of("-i", images.toString())).contains("project folder"), "project folder required");
    check(GuiOptions.validate(List.of("-i", images.toString(), "-o", "p", "--cores", "x")) != null, "non-numeric value refused");
    check(GuiOptions.withProject(List.of("-i", "a", "-o", "b", "--x"), Path.of("t")).equals(List.of("-i", "a", "--x", "-o", "t")), "memory estimate in a scratch project");
    check(GuiOptions.validateContinued(List.of("-i", images.toString(), "-o", "p"), List.of("--min-views", "2")) == null && GuiOptions.validateContinued(List.of("-i", images.toString()), List.of("--min-views", "zero")) != null, "options for continued steps checked with the saved ones");
  }

  /** About menu content: install folder, guide, release metadata, version facts, guide as HTML. */
  static void about(Path dir) throws IOException {
    Path install = GuiAbout.installRoot();
    check(install != null && Files.isDirectory(install.resolve("target").resolve("classes")), "install folder from the class path: " + install);
    Path guide = GuiAbout.guide(install);
    check(guide != null && guide.getFileName().toString().equals("GUI.md"), "user guide found next to the program");
    List<String[]> facts = GuiAbout.facts(install, "0123456789abcdef0123");
    String text = GuiAbout.text(facts);
    check(text.contains("Software: POSE " + PdfReport.VERSION) && text.contains("Processing engine: 0123456789abcdef\n") && text.contains("Java: " + System.getProperty("java.version")), "version facts: " + text);
    check(text.contains("Release: development build"), "a working copy is a development build");
    Path release = Files.createDirectories(dir.resolve("pose ä"));
    Files.writeString(release.resolve("portable-release.properties"), "version=2026.11\nplatform=windows-x64\n");
    Files.createDirectories(release.resolve("docs"));
    Files.writeString(release.resolve("docs").resolve("GUI.md"), "# x");
    check(GuiAbout.text(GuiAbout.facts(release, null)).contains("Release: 2026.11 (windows-x64)"), "portable release version shown");
    check(GuiAbout.guide(release).equals(release.resolve("docs").resolve("GUI.md")) && GuiAbout.docs(release).equals(release.resolve("docs")), "portable package: guide and docs under docs/");
    check(GuiAbout.guide(null) == null && GuiAbout.release(dir.resolve("none")).isEmpty(), "missing install folder handled");
    String html = GuiAbout.html("# Title\n\nSome **bold** and `code <x>` text\nwrapped.\n\n- one\n  - nested `a`\n- two\n  continued\n\n1. first\n2. second\n\n```\nphotogrammetry gui <P>\n```\nend *it*\n");
    check(html.contains("<h1>Title</h1>") && html.contains("<p>Some <b>bold</b> and <code>code &lt;x&gt;</code> text wrapped.</p>"), "headings, paragraphs, inline code, bold: " + html);
    check(html.contains("<ul><li>one<ul><li>nested <code>a</code></li></ul></li><li>two continued</li></ul>"), "nested list with continuation: " + html);
    check(html.contains("<ol><li>first</li><li>second</li></ol>") && html.contains("<pre>photogrammetry gui &lt;P&gt;\n</pre>") && html.contains("<i>it</i>"), "ordered list, code block, italics: " + html);
    String real = GuiAbout.html(Files.readString(guide));
    check(real.contains("<h1>POSE desktop windows</h1>") && !real.contains("**") && !real.contains("```"), "the real guide converts");
    check(count(real, "<ul>") == count(real, "</ul>") && count(real, "<li>") == count(real, "</li>") && count(real, "<pre>") == count(real, "</pre>"), "balanced lists and code blocks in the real guide");
    check(GuiSupport.scaledTo(new java.awt.image.BufferedImage(1600, 363, java.awt.image.BufferedImage.TYPE_INT_ARGB), 360, 81).getWidth() == 360, "logo scaled for the About box");
  }

  /**
   * The image window's model on a synthetic run: one nadir camera 50 m above flat ground at
   * (100, 200), a photograph red on its left (west) half and blue on its right (east) half.
   */
  static void imagesModel(Path run) throws IOException {
    Files.createDirectories(run.resolve("00_inputs"));
    Files.createDirectories(run.resolve("04_geometry"));
    Path photo = run.resolve("kuva ä.png");
    java.awt.image.BufferedImage pic = new java.awt.image.BufferedImage(400, 300, java.awt.image.BufferedImage.TYPE_INT_RGB);
    for (int y = 0; y < 300; y++) for (int x = 0; x < 400; x++) pic.setRGB(x, y, x < 200 ? 0xff0000 : 0x0000ff);
    javax.imageio.ImageIO.write(pic, "png", photo.toFile());
    Files.writeString(run.resolve("00_inputs/images.csv"),
        "image_id,filename,original_path,bytes,sha256,original_width,original_height\n0,\"kuva ä.png\",\"" + photo + "\",1,x,400,300\n1,\"lost.png\",\"" + run.resolve("lost.png") + "\",1,x,400,300\n", StandardCharsets.UTF_8);
    // Nadir: camera x = east, y = south, z = down; R rows; t = -R C with C = (100, 200, 50).
    Files.writeString(run.resolve("04_geometry/orientation_model.json"),
        "{\"source\":{\"crs\":\"EPSG:3067\",\"views\":[{\"image\":\"kuva ä.png\",\"interior\":{\"width\":400,\"height\":300,\"fx\":300,\"fy\":300,\"cx\":200,\"cy\":150,\"distortion\":[0,0,0,0,0]},"
            + "\"R\":[1,0,0,0,-1,0,0,0,-1],\"t\":[-100,200,50]}]}}", StandardCharsets.UTF_8);
    Files.writeString(run.resolve("04_geometry/sparse.csv"), "x,y,z,observations\n90,190,0,3\n110,210,0,3\n100,200,0,2\n");
    Files.writeString(run.resolve("04_geometry/camera_screening.json"), "{\"cameras\":[{\"image\":\"kuva ä.png\",\"reprojection_rms_pixels\":0.4,\"flags\":[\"turn\"]}]}", StandardCharsets.UTF_8);
    RunImages ri = RunImages.load(run);
    check(ri.images.size() == 2 && ri.images.get(0).oriented() && !ri.images.get(1).oriented() && ri.mappable() && "EPSG:3067".equals(ri.crs), "images, orientations, CRS");
    RunImages.Image im = ri.images.get(0);
    check("turn".equals(im.flag()) && im.rms() == 0.4, "camera screening flag and reprojection RMS");
    check(Math.abs(ri.surface.reference()) < 1e-9 && ri.surface.description().contains("plane"), "plane at the median tie-point height without a surface model");
    double[] c = im.view().center();
    check(Math.abs(c[0] - 100) + Math.abs(c[1] - 200) + Math.abs(c[2] - 50) < 1e-9, "camera centre from R and t");
    double[] uv = im.view().project(100, 200, 0), west = im.view().project(90, 200, 0);
    check(Math.abs(uv[0] - 200) + Math.abs(uv[1] - 150) < 1e-9 && west[0] < 200 && im.view().project(100, 200, 60) == null, "projection (nadir point to the principal point; west is left; behind is null)");
    double[] ray = im.view().ray(200, 150);
    check(Math.abs(ray[0]) + Math.abs(ray[1]) < 1e-9 && ray[2] < 0, "the principal ray points down");
    double[] e = ri.extent(List.of(im));
    check(Math.abs(e[0] - (100 - 50 * 200 / 300.)) < 1e-6 && Math.abs(e[3] - (200 + 50 * 150 / 300.)) < 1e-6, "footprint on the ground: " + Arrays.toString(e));
    // Map: 1 m pixels around the camera; red west of it, blue east, transparent outside the footprint.
    RunImages.MapView m = new RunImages.MapView(40, 260, 1, 120, 120);
    java.awt.image.BufferedImage b = RunImages.decode(photo, 100);
    check(b.getWidth() <= 200 && b.getWidth() >= 100, "decoded at reduced size: " + b.getWidth());
    java.awt.image.BufferedImage map = ri.render(List.of(im), Map.of(0, b), m, 1f);
    int redPx = map.getRGB(60 - 40 + 10, 60), bluePx = map.getRGB(60 + 20, 60), outside = map.getRGB(1, 1);
    check((redPx & 0xffffff) == 0xff0000 && (bluePx & 0xffffff) == 0x0000ff && (outside >>> 24) == 0, "map: red west, blue east, empty outside: " + Integer.toHexString(redPx) + " " + Integer.toHexString(bluePx));
    java.awt.image.BufferedImage half = ri.render(List.of(im, im), Map.of(0, b), m, .5f);
    check((half.getRGB(80, 60) >>> 24) == 255 && (half.getRGB(80, 60) & 0xff) > 0, "opacity: overlapping photographs blend");
    // With a surface model 10 units up, the footprint shrinks (40 m from the camera instead of 50).
    Files.createDirectories(run.resolve("07_results"));
    float[] z = new float[20 * 20];
    Arrays.fill(z, 10f);
    DsmRaster.writeTiff(run.resolve("07_results/dsm.tif"), 20, 20, z, 0, 400, 20, 3067);
    Files.writeString(run.resolve("07_results/dsm.json"), "{\"extent\":[0,0,400,400],\"cell_size\":20,\"width\":20,\"height\":20}");
    RunImages withDsm = RunImages.load(run);
    check(Math.abs(withDsm.surface.z(100, 200) - 10) < 1e-6 && withDsm.surface.description().contains("dsm.tif"), "the run's surface model is used");
    check(Math.abs(withDsm.extent(List.of(withDsm.images.get(0)))[0] - (100 - 40 * 200 / 300.)) < 1e-6, "footprint on the surface model");
  }

  static int count(String s, String part) {
    int n = 0;
    for (int i = s.indexOf(part); i >= 0; i = s.indexOf(part, i + 1)) n++;
    return n;
  }

  /** Settings menu: preferences file, new-project defaults, machine fit, preview cache, housekeeping. */
  static void preferences(Path dir) throws IOException {
    GuiPreferences p = GuiPreferences.load(dir);
    check(p.scale.equals("auto") && p.newProjects.equals("last") && !p.openReport && p.toFront && p.confirmClose
        && p.cacheMb == GuiPreferences.DEFAULT_CACHE_MB, "preferences: defaults without a file");
    p.scale = "150";
    p.newProjects = "pose";
    p.openReport = true;
    p.toFront = false;
    p.confirmClose = false;
    p.cacheMb = 5;
    p.save();
    GuiPreferences q = GuiPreferences.load(dir);
    check(q.scale.equals("150") && q.newProjects.equals("pose") && q.openReport && !q.toFront && !q.confirmClose && q.cacheMb == 5,
        "preferences: saved and read back");
    GuiOptions.Settings.write(dir.resolve(GuiPreferences.FILE), Map.of("scale", "333", "new.projects", "x", "cache.mb", "lots"));
    GuiPreferences bad = GuiPreferences.load(dir);
    check(bad.scale.equals("auto") && bad.newProjects.equals("last") && bad.cacheMb == GuiPreferences.DEFAULT_CACHE_MB,
        "preferences: unknown values fall back to the defaults");
    check(GuiPreferences.uiScale("auto") == null && GuiPreferences.uiScale("125").equals("1.25") && GuiPreferences.uiScale("200").equals("2.00"),
        "interface scale as sun.java2d.uiScale");
    for (String s : GuiPreferences.SCALES) check(s.equals("auto") || Integer.parseInt(s) >= 100, "scale choice " + s);
    check(PoseSettings.SCALE_LABELS.length == GuiPreferences.SCALES.size()
        && PoseSettings.NEW_PROJECT_LABELS.length == GuiPreferences.NEW_PROJECTS.size(), "dialog labels match the choices");

    // New projects: the last options used, or always the POSE defaults (a known project keeps its own).
    GuiOptions.Settings s = new GuiOptions.Settings(dir);
    Path known = dir.resolve("tunnettu");
    Map<String, String> mine = new TreeMap<>(s.windowDefaults());
    mine.put("--quality", "low");
    mine.put("-i", "C:/kuvat");
    s.save(known, mine);
    GuiPreferences last = GuiPreferences.load(dir);
    last.newProjects = "last";
    last.save();
    check("low".equals(s.load(dir.resolve("uusi")).get("--quality")), "new project with 'last options': the last options");
    last.newProjects = "pose";
    last.save();
    check(s.load(dir.resolve("uusi")).equals(s.windowDefaults()), "new project with 'POSE defaults': the window's defaults");
    check("low".equals(s.load(known).get("--quality")) && "C:/kuvat".equals(s.load(known).get("-i")), "a known project keeps its own options");
    check(GuiPreferences.forgetOptions(s) == 2 && s.defaults().isEmpty() && s.load(known).equals(s.windowDefaults()),
        "forget saved options: project file and defaults deleted");
    check(!s.recent().isEmpty(), "recent projects kept by 'forget'");
    GuiPreferences.clearRecent(s);
    check(s.recent().isEmpty(), "clear recent projects");

    // Fit to this computer: values the option parser accepts.
    Map<String, String> big = GuiPreferences.fitted(new GuiPreferences.Machine(16, 32 * 1024, "GPU", 8 * 1024));
    check(big.equals(Map.of("--cores", "12", "--memory-mb", "16000", "--gpu-memory-mb", "7700")), "fit: 16 cores, 32 GB, 8 GB GPU -> " + big);
    Map<String, String> small = GuiPreferences.fitted(new GuiPreferences.Machine(4, 3000, GuiPreferences.NO_GPU, -1));
    check(small.equals(Map.of("--cores", "3", "--memory-mb", "2000", "--compute", "cpu")), "fit: small laptop without GPU -> " + small);
    Map<String, String> unknown = GuiPreferences.fitted(new GuiPreferences.Machine(1, 0, null, -1));
    check(unknown.equals(Map.of("--cores", "1")), "fit: GPU and RAM not known -> " + unknown);
    Path images = Files.createDirectories(dir.resolve("kuvat"));
    for (Map<String, String> fit : List.of(big, small)) {
      Map<String, String> v = new LinkedHashMap<>(GuiOptions.SCRIPT_DEFAULTS);
      v.put("-i", images.toString());
      v.put("-o", dir.resolve("p").toString());
      v.putAll(fit);
      check(GuiOptions.validate(GuiOptions.arguments(v, "")) == null, "fitted options parse: " + fit);
    }
    List<String[]> parsed = GuiPreferences.gpus("Something else\n0: NVIDIA GeForce RTX 5070 Laptop GPU; VRAM 8123 MiB; max allocation 2031 MiB; driver 616.92\n"
        + "1: gfx1103; VRAM 12534 MiB; max allocation 10219 MiB; driver 3679.0 (PAL,LC)\n");
    check(parsed.size() == 2 && parsed.get(0)[0].equals("NVIDIA GeForce RTX 5070 Laptop GPU") && parsed.get(0)[1].equals("8123")
        && parsed.get(1)[0].equals("gfx1103"), "--list-gpus parsed in device order");
    check(GuiPreferences.gpus("No OpenCL GPU devices found.\n").isEmpty(), "no GPU listed");
    GuiPreferences.Machine here = GuiPreferences.machine(0);
    check(here.cores() >= 1 && here.ramMb() > 0 && here.describe().contains("CPU cores"), "this computer: " + here.describe());

    // Preview cache: size, least recently used trimmed first, cleared.
    Path cache = Files.createDirectories(dir.resolve("image-cache"));
    long now = System.currentTimeMillis();
    for (int i = 0; i < 5; i++) {
      Path f = cache.resolve("p" + i + ".jpg");
      Files.write(f, new byte[1000]);
      Files.setLastModifiedTime(f, java.nio.file.attribute.FileTime.fromMillis(now - (5 - i) * 60_000L));
    }
    GuiPreferences.touch(cache.resolve("p0.jpg"));
    check(GuiPreferences.cacheBytes(cache) == 5000, "cache size");
    check(GuiPreferences.trim(cache, 2500) == 3 && GuiPreferences.cacheBytes(cache) == 2000, "cache trimmed to the limit");
    check(Files.exists(cache.resolve("p0.jpg")) && Files.exists(cache.resolve("p4.jpg")), "recently used previews kept");
    check(GuiPreferences.trim(cache, 0) == 2 && GuiPreferences.cacheBytes(cache) == 0, "cache cleared");
    check(GuiPreferences.cacheBytes(dir.resolve("none")) == 0 && GuiPreferences.trim(dir.resolve("none"), 0) == 0, "no cache folder");
    Files.delete(dir.resolve(GuiPreferences.FILE));
  }

  static void settings(Path dir) throws IOException {
    GuiOptions.Settings s = new GuiOptions.Settings(dir);
    Path project = dir.resolve("projekti å");
    // Before any detection: cores and RAM of this computer, no GPU budget (the command line's automatic one).
    Map<String, String> undetected = s.windowDefaults();
    check(undetected.containsKey("--cores") && !undetected.containsKey("--gpu-memory-mb") && !undetected.containsKey("--compute")
        && Integer.parseInt(undetected.get("--cores")) == Math.max(1, Math.round(Runtime.getRuntime().availableProcessors() * .75f)),
        "compute defaults before detection: " + undetected);
    GuiPreferences.remember(dir, new GuiPreferences.Machine(16, 32051, "NVIDIA GeForce RTX 5070 Laptop GPU", 8123));
    check(GuiPreferences.remembered(dir).equals(new GuiPreferences.Machine(16, 32051, "NVIDIA GeForce RTX 5070 Laptop GPU", 8123)), "machine remembered");
    check(s.load(project).equals(s.windowDefaults()) && s.recent().isEmpty(), "nothing remembered yet: the window's defaults");
    // Version 2 (the script's fixed sizes): sizes still equal to the script's follow the hardware, the rest stays.
    GuiOptions.Settings.write(s.projectFile(dir.resolve("v2")), Map.of("settings.version", "2", "-i", "C:/kuvat",
        "--cores", "12", "--memory-mb", "16000", "--gpu-memory-mb", "8000", "--quality", "low"));
    Map<String, String> v2 = s.load(dir.resolve("v2"));
    check(v2.get("--gpu-memory-mb").equals("7700") && v2.get("--cores").equals("12") && v2.get("--memory-mb").equals("16000")
        && v2.get("--quality").equals("low") && v2.get("-i").equals("C:/kuvat"), "version 2: script sizes follow the hardware, rest kept: " + v2);
    GuiOptions.Settings.write(s.projectFile(dir.resolve("v2")), Map.of("settings.version", "2", "--cores", "6", "--gpu-memory-mb", "3000"));
    v2 = s.load(dir.resolve("v2"));
    check(v2.get("--cores").equals("6") && v2.get("--gpu-memory-mb").equals("3000") && !v2.containsKey("--memory-mb"), "version 2: own sizes kept");
    Files.delete(s.projectFile(dir.resolve("v2")));
    // Settings saved before the window had defaults give way to them, keeping their folders.
    GuiOptions.Settings.write(s.projectFile(dir.resolve("old")), Map.of("-i", "C:/vanhat kuvat", "--quality", "low", "--dense-method", "sgm"));
    Map<String, String> migrated = s.load(dir.resolve("old"));
    check(migrated.get("-i").equals("C:/vanhat kuvat") && migrated.get("--quality").equals("high") && migrated.get("--dense-method").equals("mvsgm"), "old settings: defaults with the old folders");
    GuiOptions.Settings.write(s.defaultsFile(), Map.of("--quality", "low"));
    check(s.load(dir.resolve("unseen")).equals(s.windowDefaults()), "old user defaults give way too");
    Files.delete(s.defaultsFile());
    Files.delete(s.projectFile(dir.resolve("old")));
    // The window's defaults: the user's tested mvsgm script, without the folders.
    String script = "--cores 12 --memory-mb 16000 --gpu-memory-mb 8000 --quality high --quick-calibration --ply off --point-confidence off --features 40000 --ratio 0.85"
        + " --neighbors 8 --descriptor-matcher flann --matching-algorithm sift --descriptor-memory-mb 1600 --min-forward-overlap 50 --step 1 --filtering mild"
        + " --dense-method mvsgm --min-views 2 --min-angle 4 --confidence-threshold 0.0 --surface-consolidation none --windowed-mode auto --fusion-gpu-evidence auto"
        + " --fusion-mode image --fusion-outliers robust --subpixel-fit equiangular --fusion-strategy graph-average --mvsgm-optimal-neighbors"
        + " --depth-outlier-filter edge-aware --mvsgm-neighbors 8 --mvsgm-best-k 3 --mvsgm-census-window 5 --mvsgm-consistent-views 1 --mvsgm-consistency-labels 1";
    Map<String, String> fromScript = new LinkedHashMap<>();
    List<String> tokens = GuiOptions.split(script);
    for (int i = 0; i < tokens.size(); i++)
      if (i + 1 < tokens.size() && !tokens.get(i + 1).startsWith("--")) fromScript.put(tokens.get(i), tokens.get(++i));
      else fromScript.put(tokens.get(i), "true");
    Map<String, String> fixed = new LinkedHashMap<>(fromScript);
    fixed.keySet().removeAll(GuiOptions.SCRIPT_HARDWARE.keySet());
    check(fixed.equals(GuiOptions.SCRIPT_DEFAULTS), "the window's defaults are the script's settings: " + GuiOptions.SCRIPT_DEFAULTS);
    for (String k : GuiOptions.SCRIPT_HARDWARE.keySet()) check(GuiOptions.SCRIPT_HARDWARE.get(k).equals(fromScript.get(k)), "script compute size " + k);
    // ...with the compute sizes from this computer (the remembered machine; here the user's laptop,
    // where they come out as the script's 12 cores and 16000 MB; GPU memory 95% of 8123 MB).
    Map<String, String> expected = new LinkedHashMap<>(GuiOptions.SCRIPT_DEFAULTS);
    expected.putAll(Map.of("--cores", "12", "--memory-mb", "16000", "--gpu-memory-mb", "7700"));
    check(s.windowDefaults().equals(expected), "window defaults with this computer's sizes: " + s.windowDefaults());
    tokens = new ArrayList<>(tokens);
    tokens.removeAll(List.of("8000"));
    for (String flag : s.windowDefaults().keySet()) check(GuiOptions.field(flag) != null, "a field for default " + flag);
    Path imgs = Files.createDirectories(dir.resolve("kuvat"));
    Map<String, String> run = new LinkedHashMap<>(s.windowDefaults());
    run.put("-i", imgs.toString());
    run.put("-o", dir.resolve("out").toString());
    List<String> cmd = GuiOptions.arguments(run, "");
    check(GuiOptions.validate(cmd) == null, "the defaults parse: " + GuiOptions.validate(cmd));
    for (String t : tokens) check(cmd.contains(t), "the default command has " + t);
    Map<String, String> reset = s.reset(Map.of("-i", "C:/kuvat", "--gcp", "g.csv", "--quality", "low", "--features", "100"));
    check(reset.get("-i").equals("C:/kuvat") && reset.get("--gcp").equals("g.csv") && reset.get("--quality").equals("high") && reset.get("--features").equals("40000"), "Reset options: the defaults, folders kept");
    Map<String, String> v = new TreeMap<>(Map.of("-i", "C:/kuvat ä", "-o", project.toString(), "--quality", "low", "advanced", "--features 1000"));
    s.save(project, v);
    Map<String, String> back = s.load(project);
    check(back.get("-i").equals("C:/kuvat ä") && back.get("--quality").equals("low") && back.get("advanced").equals("--features 1000"), "per-project settings round trip (Unicode)");
    Map<String, String> defaults = s.load(dir.resolve("another"));
    check(defaults.get("--quality").equals("low") && !defaults.containsKey("-i") && !defaults.containsKey("-o"), "user defaults without the folders for a new project");
    s.save(dir.resolve("second"), Map.of("--quality", "high"));
    check(s.recent().get(0).endsWith("second") && s.recent().get(1).endsWith("projekti å"), "recent projects newest first");
    check(s.load(project).get("--quality").equals("low"), "a project keeps its own settings");
    check(GuiOptions.Settings.user().dir.endsWith(Path.of("gui")), "user settings folder");
  }

  /** Readers on a synthetic project: runs, steps and times, statuses, summary, report choice. */
  static void projectReaders(Path root) throws IOException {
    check(!ProjectFiles.isProject(root), "not a project before files exist");
    String a = "20261101T100000000Z_aaaa", b = "20261102T100000000Z_bbbb", c = "20261103T100000000Z_cccc";
    Files.createDirectories(root);
    Files.writeString(root.resolve("project.json"), "{\"schema\":\"msgm-java-project-v1\",\"runs\":[\"" + a + "\",\"" + b + "\",\"" + c + "\"]}");
    // a: complete in one command; finished report with flags.
    writeRun(root, a, "2026-11-01T10:00:00Z", with(Map.of("inputs", "complete@2026-11-01T10:00:10Z", "calibrate", "complete@2026-11-01T10:01:00Z", "match", "complete@2026-11-01T10:02:00Z",
        "align", "complete@2026-11-01T10:03:00Z", "georeference", "complete@2026-11-01T10:03:30Z", "gcp", "skipped@2026-11-01T10:03:30Z", "dense", "complete@2026-11-01T10:10:00Z",
        "ground", "skipped@2026-11-01T10:10:00Z", "finish", "complete@2026-11-01T10:10:05Z", "cleanup", "absent"), "lidar", "absent"), List.of());
    Path results = Files.createDirectories(root.resolve("runs").resolve(a).resolve("07_results"));
    Files.writeString(results.resolve("processing-report.json"),
        "{\"quality_flags\":[\"1 camera flagged\"],\"dense\":{\"views_per_point\":{\"points\":6370945}},\"survey\":{\"images\":8,\"aligned\":8,\"tie_points\":15312,\"reprojection_rms_pixels\":0.96,\"coverage_area\":9388.0},"
            + "\"coordinates\":{\"georeferenced\":true,\"horizontal_crs\":\"EPSG:3067\",\"inlier_rms_metres\":0.54}}");
    Files.writeString(results.resolve("processing-report.pdf"), "first");
    // b: step by step; waits at gcp; the gcp step was started by a later command.
    writeRun(root, b, "2026-11-02T10:00:00Z", Map.of("inputs", "complete@2026-11-02T10:00:10Z", "calibrate", "complete@2026-11-02T10:01:00Z", "match", "complete@2026-11-02T10:02:00Z",
        "align", "complete@2026-11-02T10:03:00Z", "cleanup", "skipped@2026-11-02T10:03:00Z", "georeference", "complete@2026-11-02T10:04:00Z", "lidar", "skipped@2026-11-02T10:04:00Z", "gcp", "waiting@2026-11-02T12:00:30Z"), List.of("2026-11-02T10:00:20Z", "2026-11-02T12:00:00Z"));
    // c: failed in dense.
    writeRun(root, c, "2026-11-03T10:00:00Z", Map.of("inputs", "complete@2026-11-03T10:00:10Z", "calibrate", "complete@2026-11-03T10:01:00Z", "match", "complete@2026-11-03T10:02:00Z",
        "align", "complete@2026-11-03T10:03:00Z", "cleanup", "skipped@2026-11-03T10:03:00Z", "georeference", "complete@2026-11-03T10:04:00Z", "lidar", "skipped@2026-11-03T10:04:00Z", "gcp", "skipped@2026-11-03T10:04:00Z", "dense", "failed@2026-11-03T10:05:00Z"), List.of());
    check(ProjectFiles.isProject(root), "project recognised");
    List<ProjectFiles.Run> runs = ProjectFiles.runs(root);
    check(runs.stream().map(ProjectFiles.Run::id).toList().equals(List.of(c, b, a)), "runs newest first");
    ProjectFiles.Run ra = ProjectFiles.run(root, a), rb = ProjectFiles.run(root, b), rc = ProjectFiles.run(root, c);
    check(ra.status().equals("complete") && rb.status().equals("waiting") && rc.status().equals("failed"), "run statuses: " + ra.status() + " " + rb.status() + " " + rc.status());
    check(ra.points() == 6370945L && ra.flags().equals(List.of("1 camera flagged")), "points and quality flags from the report");
    Map<String, String> summary = new LinkedHashMap<>();
    for (String[] row : ra.summary()) summary.put(row[0], row[1]);
    check(summary.get("Images aligned").equals("8 of 8") && summary.get("Dense points").equals("6,370,945") && summary.get("Coordinate system").equals("EPSG:3067")
        && summary.get("Reprojection RMS").equals("0.96 px") && summary.get("Coverage area").equals("0.94 ha"), "summary numbers: " + summary);
    check(ProjectFiles.duration(ra.step("inputs").duration()).equals("0:10") && ProjectFiles.duration(ra.step("dense").duration()).equals("6:30"), "step times from the marks (one command)");
    check(ra.remaining().isEmpty() && rb.remaining().equals(List.of("gcp", "dense", "ground", "finish")) && rc.remaining().equals(List.of("dense", "ground", "finish")), "remaining steps");
    check(rb.waitingForGcp() && !ra.waitingForGcp(), "gcp waiting detected");
    // The gcp step of b started with the later command, not when georeference ended.
    check(ProjectFiles.duration(rb.step("gcp").duration()).equals("0:30"), "step start = start of the command that ran it: " + ProjectFiles.duration(rb.step("gcp").duration()));
    check(ProjectFiles.duration(rb.step("calibrate").duration()).equals("0:40"), "second command's first step");
    check(rb.step("dense").status().equals("pending") && rb.step("dense").duration() == null, "pending step has no time");
    check(ra.report("pdf").endsWith(Path.of("07_results", "processing-report.pdf")), "the run's report");
    Path again = Files.createDirectories(results.resolve("report")).resolve("processing-report.pdf");
    Files.writeString(again, "again");
    Files.setLastModifiedTime(again, java.nio.file.attribute.FileTime.fromMillis(System.currentTimeMillis() + 5000));
    check(ProjectFiles.run(root, a).report("pdf").equals(again), "a newer re-exported report is opened");
    // Commands of the actions, as the CLI parses them.
    Main.Command cont = Main.command(ProjectFiles.continueSteps(root, b, rb.remaining(), List.of("--min-views", "2")).toArray(String[]::new));
    check(cont.steps().size() == 4 && cont.steps().get(0) == Pipeline.Step.GCP && b.equals(cont.runId()) && !cont.branch() && Arrays.asList(cont.rest()).contains("--min-views"), "continue command");
    Main.Command br = Main.command(ProjectFiles.branch(root, a, "dense", List.of("--filtering", "mild")).toArray(String[]::new));
    check(br.branch() && a.equals(br.runId()) && br.steps().equals(List.of(Pipeline.Step.DENSE, Pipeline.Step.GROUND, Pipeline.Step.FINISH)), "branch command redoes to finish");
    // Copyable error details of a failed run.
    Files.writeString(root.resolve("runs").resolve(c).resolve("logs/console.log"), "Pipeline step: dense - Dense matching\nfine line\nphotogrammetry: mvsgm map startup reservation needs approximately 279260 MiB, available 108254 MiB\n");
    Files.writeString(root.resolve("runs").resolve(c).resolve("run.json"), "{\"run_id\":\"" + c + "\",\"status\":\"failed\",\"started_utc\":\"2026-11-03T10:00:00Z\",\"error\":\"java.io.IOException: mvsgm map startup reservation\"}");
    ProjectFiles.Run failed = ProjectFiles.run(root, c);
    String stepText = ProjectFiles.stepDetails(failed, failed.step("dense"));
    check(stepText.contains("Step: dense - failed") && stepText.contains("Run error:\njava.io.IOException: mvsgm map startup reservation")
        && stepText.contains("Errors in logs/console.log:\nphotogrammetry: mvsgm map startup reservation needs approximately 279260 MiB") && stepText.contains("End of logs/console.log:\nPipeline step: dense"), "step details carry the message, run error and log: " + stepText);
    String runText = ProjectFiles.runDetails(failed);
    check(runText.contains("Status: failed") && runText.contains("dense") && runText.contains("Arguments: -i x -o y") && runText.contains("Run error:"), "run details for a bug report");
    check(ProjectFiles.runError(root.resolve("runs").resolve(a)) == null && !ProjectFiles.stepDetails(ra, ra.step("match")).contains("Run error"), "no error text for a good step");
    check(PoseProjectPanel.summaryHtml(failed).contains("<b>Error:</b>"), "the summary shows the run's error");
    // Redo only some steps in a new run: just align, or align up to georeference; later steps pending.
    Main.Command justAlign = Main.command(ProjectFiles.branch(root, a, "align", "align", List.of("--features", "3000")).toArray(String[]::new));
    check(justAlign.branch() && justAlign.steps().equals(List.of(Pipeline.Step.ALIGN)) && Arrays.asList(justAlign.rest()).contains("--features"), "redo only align");
    check(Main.command(ProjectFiles.branch(root, a, "align", "georeference", List.of()).toArray(String[]::new)).steps().equals(List.of(Pipeline.Step.ALIGN, Pipeline.Step.CLEANUP, Pipeline.Step.GEOREFERENCE)), "redo align up to georeference");
    for (String[] bad : new String[][] {{"dense", "align"}, {"inputs", "align"}}) {
      boolean refused = false;
      try {
        ProjectFiles.branch(root, a, bad[0], bad[1], List.of());
      } catch (IllegalArgumentException e) {
        refused = true;
      }
      check(refused, "invalid redo range refused: " + bad[0] + ".." + bad[1]);
    }
    check(ProjectFiles.branchable(ProjectFiles.run(root, a)).equals(ProjectFiles.STEPS.subList(1, ProjectFiles.STEPS.size())) && ProjectFiles.run(root, a).step("cleanup").status().equals("skipped"), "every step after inputs of a finished run can be redone, also of a run made before the cleanup step existed");
    check(ProjectFiles.branchable(rb).equals(List.of("calibrate", "match", "align", "cleanup", "georeference", "lidar", "gcp")), "a waiting run can be redone up to its waiting step: " + ProjectFiles.branchable(rb));
    check(ProjectFiles.branchable(rc).equals(List.of("calibrate", "match", "align", "cleanup", "georeference", "lidar", "gcp", "dense")), "a failed run can be redone up to its failed step");
    Main.Command edit = Main.command(ProjectFiles.editGcps(root, b).toArray(String[]::new));
    check(edit.editor() && b.equals(edit.runId()) && Arrays.asList(edit.rest()).contains(GcpEditor.NO_CONTINUE), "GCP editor command leaves continuing to the window");
    Options report = Options.parse(ProjectFiles.exportReport(root.resolve("runs").resolve(a)).toArray(String[]::new));
    Options ortho = Options.parse(ProjectFiles.exportOrthomosaic(root.resolve("runs").resolve(a)).toArray(String[]::new));
    check(report.exportReport != null && ortho.exportOrthomosaic != null, "report and orthomosaic export commands parse");
    // Console lines.
    check(a.equals(ProjectFiles.runId("Run: " + a)) && ProjectFiles.runId("Running: x") == null, "run id line");
    check(ProjectFiles.projectRoot("Project: C:\\a b\\p ä").equals("C:\\a b\\p ä"), "project line");
    ProjectFiles.Progress p = ProjectFiles.progress("[5/8] Dense matching: Stereo pairs  [=====     ]  45%  12/30 pairs  1:02  ~1:10 left");
    check(p != null && p.stage() == 5 && p.stages() == 8 && p.name().equals("Dense matching") && p.percent() == 45, "progress line: " + p);
    ProjectFiles.Progress d = ProjectFiles.progress("[3/8] Matching image pairs     done  0:05  - 12 pairs");
    check(d != null && d.name().equals("Matching image pairs") && d.percent() == 100, "stage done line: " + d);
    ProjectFiles.Progress w = ProjectFiles.progress("[6/8] Fusing depth maps  0:12  Fusion");
    check(w != null && w.percent() == -1, "stage without percent");
    check(ProjectFiles.progress("Pipeline step: dense") == null, "not a progress line");
  }

  /** A copy of the marks with one more step (Map.of takes at most ten pairs). */
  static Map<String, String> with(Map<String, String> marks, String step, String mark) {
    Map<String, String> out = new HashMap<>(marks);
    out.put(step, mark);
    return out;
  }

  static void writeRun(Path root, String id, String started, Map<String, String> marks, List<String> commands) throws IOException {
    Path run = Files.createDirectories(root.resolve("runs").resolve(id));
    Files.createDirectories(run.resolve("state"));
    Files.createDirectories(run.resolve("logs"));
    // As a real run: every step from the start, pending until marked. "absent" leaves a step out
    // (a run made before that step existed).
    StringBuilder steps = new StringBuilder();
    for (String name : ProjectFiles.STEPS) {
      String mark = marks.get(name);
      if ("absent".equals(mark)) continue;
      if (!steps.isEmpty()) steps.append(',');
      if (mark == null) {
        steps.append("\"").append(name).append("\":{\"status\":\"pending\"}");
        continue;
      }
      String[] sv = mark.split("@");
      steps.append("\"").append(name).append("\":{\"status\":\"").append(sv[0]).append("\",\"updated_utc\":\"").append(sv[1]).append("\",\"detail\":\"d\"}");
      if (sv[0].equals("complete") || sv[0].equals("skipped")) Files.createDirectories(run.resolve("state/after-" + name));
    }
    Files.writeString(run.resolve("state/state.json"), "{\"format\":\"msgm-pipeline-state-1\",\"arguments\":[\"-i\",\"x\",\"-o\",\"y\"],\"steps\":{" + steps + "}}");
    Files.writeString(run.resolve("run.json"), "{\"run_id\":\"" + id + "\",\"status\":\"running\",\"started_utc\":\"" + started + "\"}");
    StringBuilder events = new StringBuilder("{\"utc\":\"" + started + "\",\"type\":\"run_started\",\"detail\":{}}\n");
    for (String t : commands) events.append("{\"utc\":\"").append(t).append("\",\"type\":\"step_started\",\"detail\":{\"arguments\":[]}}\n");
    events.append("{\"utc\":\"").append(started).append("\",\"type\":\"stage\",\"detail\":{}}\n");
    Files.writeString(run.resolve("logs/events.jsonl"), events);
  }


  /** A LAS 1.2 point format 2 file: points {x, y, z, class, rgb}, scale 0.001, offset (600000, 6900000, 0). */
  static void writeLas(Path file, double[][] points) throws IOException {
    int n = points.length;
    java.nio.ByteBuffer b = java.nio.ByteBuffer.allocate(227 + 26 * n).order(java.nio.ByteOrder.LITTLE_ENDIAN);
    b.put("LASF".getBytes(StandardCharsets.US_ASCII));
    b.put(24, (byte) 1).put(25, (byte) 2).putShort(94, (short) 227).putInt(96, 227).putInt(100, 0).put(104, (byte) 2).putShort(105, (short) 26).putInt(107, n);
    double[] scale = {.001, .001, .001}, offset = {600000, 6900000, 0}, lo = {1e18, 1e18, 1e18}, hi = {-1e18, -1e18, -1e18};
    for (double[] p : points) for (int k = 0; k < 3; k++) { lo[k] = Math.min(lo[k], p[k]); hi[k] = Math.max(hi[k], p[k]); }
    for (int k = 0; k < 3; k++) {
      b.putDouble(131 + 8 * k, scale[k]).putDouble(155 + 8 * k, offset[k]);
      b.putDouble(179 + 16 * k, hi[k]).putDouble(187 + 16 * k, lo[k]);
    }
    for (int i = 0; i < n; i++) {
      int at = 227 + 26 * i;
      double[] p = points[i];
      for (int k = 0; k < 3; k++) b.putInt(at + 4 * k, (int) Math.round((p[k] - offset[k]) / scale[k]));
      b.put(at + 15, (byte) p[3]);
      int rgb = (int) p[4];
      b.putShort(at + 20, (short) ((rgb >> 16 & 255) * 257)).putShort(at + 22, (short) ((rgb >> 8 & 255) * 257)).putShort(at + 24, (short) ((rgb & 255) * 257));
    }
    Files.createDirectories(file.getParent());
    Files.write(file, b.array());
  }

  /** The 3D view: reading a run (subsampled LAS, tie points, cameras), the home view, rendering and picking. */
  static void cloudViewer(Path root) throws IOException {
    Path run = Files.createDirectories(root.resolve("runs/r1"));
    // A 10 x 10 grid of points at heights 10 and 20 (classes 2 and 5), a red marker point at (600050, 6900050, 30).
    List<double[]> pts = new ArrayList<>();
    for (int i = 0; i < 10; i++)
      for (int j = 0; j < 10; j++) pts.add(new double[] {600000 + 10 * i + 5, 6900000 + 10 * j + 5, (i + j) % 2 == 0 ? 10 : 20, (i + j) % 2 == 0 ? 2 : 5, 0x336699});
    pts.add(new double[] {600050, 6900050, 30, 6, 0xff0000});
    writeLas(run.resolve("07_results/dense.las"), pts.toArray(double[][]::new));
    Files.writeString(run.resolve("07_results/coordinates.json"), "{\"coordinate_multiplier\":1}");
    // Tie points: two in the block and a stray one 50 km away (must not shrink the home view).
    Files.createDirectories(run.resolve("04_geometry"));
    Files.writeString(run.resolve("04_geometry/sparse.csv"), "x,y,z,observations\n600020,6900020,15,3\n600080,6900080,15,9\n650000,6950000,15,2\n");
    // One camera 100 m above the centre looking straight down (R = diag(1,-1,-1)), flagged.
    double[] c = {600050, 6900050, 110};
    double[] r = {1, 0, 0, 0, -1, 0, 0, 0, -1};
    double[] t = {-(r[0] * c[0] + r[1] * c[1] + r[2] * c[2]), -(r[3] * c[0] + r[4] * c[1] + r[5] * c[2]), -(r[6] * c[0] + r[7] * c[1] + r[8] * c[2])};
    Files.writeString(run.resolve("04_geometry/cameras.csv"), "image,r00,r01,r02,r10,r11,r12,r20,r21,r22,tx,ty,tz\n\"a.jpg\"," + Arrays.stream(r).mapToObj(String::valueOf).reduce((x, y) -> x + "," + y).get()
        + "," + t[0] + "," + t[1] + "," + t[2] + "\n");
    Files.writeString(run.resolve("04_geometry/camera_screening.json"), "{\"cameras\":[{\"image\":\"a.jpg\",\"reprojection_rms_pixels\":2.5,\"flags\":[\"gps_residual\"]}]}");

    CloudData all = CloudData.load(run, 1_000_000, null);
    check(all.n == 101 && all.total == 101 && all.stride == 1, "all points read: " + all.n);
    check(Arrays.equals(all.origin, new double[] {600050, 6900050, 20}), "origin at the centre of the LAS bounds: " + Arrays.toString(all.origin));
    check(Math.abs(all.x[100]) < 1e-3 && Math.abs(all.y[100]) < 1e-3 && Math.abs(all.z[100] - 10) < 1e-3 && all.cls[100] == 6 && all.rgb[100] == 0xff0000, "point values, class and colour");
    check(all.cls[0] == 2 && all.rgb[0] == 0x336699, "class and colour of the grid");
    check(all.sx.length == 3 && all.sobs[1] == 9 && Math.abs(all.sx[0] + 30) < 1e-3, "tie points relative to the origin");
    check(all.cameras.size() == 1 && Math.abs(all.cameras.get(0).center()[2] - 90) < 1e-6 && "gps_residual".equals(all.cameras.get(0).flag()), "camera centre and flag: " + all.cameras);
    CloudData some = CloudData.load(run, 30, null);
    check(some.stride == 4 && some.n == 26 && some.x[1] == all.x[4], "every k-th point up to the budget: stride " + some.stride + ", " + some.n);
    // Home view: the stray tie point is left out, everything else is in view.
    CloudRenderer.View home = CloudRenderer.home(all);
    check(home.distance() < 1000 && Math.abs(home.target()[0]) < 10 && Math.abs(home.target()[1]) < 10, "home view not shrunk by a stray tie point: " + home.distance());
    // Top view, orthographic: the marker point (nearest to the eye) lands in the centre, red; picking finds it.
    CloudRenderer.View top = new CloudRenderer.View(new double[] {0, 0, 10}, 0, 90, 200, 45, true);
    CloudRenderer.Frame f = CloudRenderer.render(new CloudRenderer.Request(all, top, 201, 201, CloudRenderer.Colour.RGB, -1, 1, false, true, false, 1, 0, 1));
    check((f.argb()[100 * 201 + 100] & 0xffffff) == 0xff0000 && f.pick(100, 100, 0) == 100, "marker in the centre of a top view and picked");
    // North is up and east right: the grid point at (+40, +40) is up-right of the centre.
    double[] s = new double[3];
    new CloudRenderer.Projector(top, 201, 201).project(40, 40, 0, s);
    check(s[0] > 100 && s[1] < 100, "north up, east right in the top view: " + Arrays.toString(s));
    CloudRenderer.Frame c2 = CloudRenderer.render(new CloudRenderer.Request(all, top, 201, 201, CloudRenderer.Colour.CLASS, -1, 1, true, true, true, 1, 0, 1));
    check((c2.argb()[100 * 201 + 100] & 0xffffff) == CloudRenderer.shade(CloudRenderer.classColour(6), 0) || (c2.argb()[100 * 201 + 100] & 0xffffff) != 0, "class colours with shading");
    int[] again = CloudRenderer.render(new CloudRenderer.Request(all, top, 201, 201, CloudRenderer.Colour.CLASS, -1, 1, true, true, true, 1, 0, 1)).argb();
    check(Arrays.equals(again, c2.argb()), "rendering is repeatable (parallel, order independent)");
    // Camera frustum: apex at the camera, base below it (looking down).
    double[][] fr = CloudRenderer.frustum(all.cameras.get(0), 5);
    check(Math.abs(fr[0][2] - 90) < 1e-9 && fr[1][2] < fr[0][2] - 4.9, "frustum points down from the camera");
    check(CloudRenderer.ramp(0) != CloudRenderer.ramp(1) && CloudRenderer.classColour(2) != CloudRenderer.classColour(5), "colour ramps");
    // Orbit crosses the poles without clamping; pan and zoom move as expected.
    CloudRenderer.View v = home.orbit(370, 100);
    check(v.pitch() == CloudRenderer.View.wrap(home.pitch()+100) && v.yaw() >= 0 && v.yaw() < 360, "unrestricted orbit");
    check(home.zoom(.5).distance() == home.distance() * .5, "zoom");
  }

  /** ChildRun: output lines, exit code once, stop of a running command. */
  static void childRun() throws Exception {
    List<String> lines = Collections.synchronizedList(new ArrayList<>());
    java.util.concurrent.CompletableFuture<int[]> end = new java.util.concurrent.CompletableFuture<>();
    ChildRun.Listener listener =
        new ChildRun.Listener() {
          public void line(String line) {
            lines.add(line);
          }

          public void finished(int exit, boolean stopped) {
            end.complete(new int[] {exit, stopped ? 1 : 0});
          }
        };
    ChildRun r = ChildRun.startCommand(List.of(PoseProcess.java().toString(), "-version"), listener);
    int[] result = end.get(60, java.util.concurrent.TimeUnit.SECONDS);
    check(result[0] == 0 && result[1] == 0 && !r.alive() && r.exit() == 0, "exit reported once");
    check(lines.stream().anyMatch(l -> l.contains("version")), "output lines delivered");
    java.util.concurrent.CompletableFuture<int[]> stopped = new java.util.concurrent.CompletableFuture<>();
    ChildRun s =
        ChildRun.startCommand(
            List.of(PoseProcess.java().toString(), "-cp", System.getProperty("java.class.path"), "sgm.GuiRegression", "sleeper", "spawn"),
            new ChildRun.Listener() {
              public void line(String line) {}

              public void finished(int exit, boolean wasStopped) {
                stopped.complete(new int[] {exit, wasStopped ? 1 : 0});
              }
            });
    for (int i = 0; i < 200 && s.handle().descendants().findAny().isEmpty(); i++) Thread.sleep(100);
    List<ProcessHandle> kids = s.handle().descendants().toList();
    check(s.alive() && !kids.isEmpty(), "long command running with a child");
    s.stop();
    int[] st = stopped.get(30, java.util.concurrent.TimeUnit.SECONDS);
    check(st[1] == 1 && !s.alive(), "stop reported as stopped");
    for (ProcessHandle k : kids) {
      k.onExit().get(10, java.util.concurrent.TimeUnit.SECONDS);
      check(!k.isAlive(), "stop ends the whole tree");
    }
  }

  /**
   * On a real project (the waiting GCP project of GcpRegression): the readers see the waiting gcp
   * step, and the CLI started the way the window starts it (PoseProcess, launcher, re-exec logic)
   * runs a step, announces the run and keeps the run waiting.
   */
  static int realProject(Path project) throws Exception {
    int before = checks;
    project = project.toAbsolutePath().normalize();
    List<ProjectFiles.Run> runs = ProjectFiles.runs(project);
    check(runs.size() == 1, "one run in the waiting project");
    ProjectFiles.Run run = runs.get(0);
    check(run.status().equals("waiting") && run.waitingForGcp() && run.remaining().get(0).equals("gcp"), "real run waits at gcp: " + run.status());
    check(run.steps().stream().filter(s -> s.status().equals("complete")).allMatch(s -> s.started() != null && s.finished() != null), "real steps have times");
    List<String> lines = Collections.synchronizedList(new ArrayList<>());
    java.util.concurrent.CompletableFuture<Integer> end = new java.util.concurrent.CompletableFuture<>();
    ChildRun child =
        ChildRun.start(
            ProjectFiles.continueSteps(project, run.id(), List.of("gcp"), List.of()),
            new ChildRun.Listener() {
              public void line(String line) {
                lines.add(line);
              }

              public void finished(int exit, boolean stopped) {
                end.complete(exit);
              }
            });
    int exit = end.get(300, java.util.concurrent.TimeUnit.SECONDS);
    check(exit == 0, "gcp step through the child process: exit " + exit + "\n" + String.join("\n", lines));
    check(child.runId() == null || run.id().equals(child.runId()), "announced run is the continued one: " + child.runId());
    String after = ProjectFiles.run(project, run.id()).step("gcp").status();
    check(after.equals("waiting") || after.equals("complete"), "gcp step ran in the child: " + after);
    check(ProjectFiles.newRun(List.of(), project).equals(run.id()) && ProjectFiles.newRun(List.of(run.id()), project) == null, "new run found by its id");
    // Redo only align of the run with another option, in a new run; then continue that run by one step.
    java.util.concurrent.CompletableFuture<Integer> redone = new java.util.concurrent.CompletableFuture<>();
    Map<String, String> sourceStatus = new LinkedHashMap<>();
    for (ProjectFiles.Step s : ProjectFiles.run(project, run.id()).steps()) sourceStatus.put(s.name(), s.status());
    ChildRun.Listener quiet =
        new ChildRun.Listener() {
          public void line(String line) {}

          public void finished(int code, boolean stopped) {
            redone.complete(code);
          }
        };
    ChildRun redo = ChildRun.start(ProjectFiles.branch(project, run.id(), "align", "align", List.of("--min-angle", "2")), quiet);
    check(redone.get(300, java.util.concurrent.TimeUnit.SECONDS) == 0, "redo align in a new run: exit 0");
    String branchId = redo.runId() != null ? redo.runId() : ProjectFiles.newRun(List.of(run.id()), project);
    check(branchId != null && !branchId.equals(run.id()), "a new run was made: " + branchId);
    ProjectFiles.Run branched = ProjectFiles.run(project, branchId);
    check(branched.step("align").status().equals("complete") && branched.step("inputs").done() && branched.step("match").done(), "align redone, earlier steps carried over");
    check(branched.remaining().equals(List.of("cleanup", "georeference", "lidar", "gcp", "dense", "ground", "finish")), "later steps stay pending: " + branched.remaining());
    check(String.join(" ", branched.arguments()).contains("--min-angle 2"), "the new option is saved with the new run");
    Map<String, String> sourceAfter = new LinkedHashMap<>();
    for (ProjectFiles.Step s : ProjectFiles.run(project, run.id()).steps()) sourceAfter.put(s.name(), s.status());
    check(sourceAfter.equals(sourceStatus), "the source run is not changed");
    java.util.concurrent.CompletableFuture<Integer> next = new java.util.concurrent.CompletableFuture<>();
    ChildRun.start(
        ProjectFiles.continueSteps(project, branchId, List.of("georeference"), List.of()),
        new ChildRun.Listener() {
          public void line(String line) {}

          public void finished(int code, boolean stopped) {
            next.complete(code);
          }
        });
    check(next.get(300, java.util.concurrent.TimeUnit.SECONDS) == 0 && ProjectFiles.run(project, branchId).step("georeference").status().equals("complete"), "the new run continues later, step by step");
    // Memory estimate as the launcher runs it: saved inputs, scratch project, verbose plan.
    Path scratch = Files.createTempDirectory(Path.of("target"), "gui-estimate-");
    List<String> args = new ArrayList<>(GuiOptions.withProject(List.of("-i", Path.of("..", "example_data", "forest", "forest_tiny").toString(), "-o", project.toString(), "--quality", "low"), scratch.resolve("p")));
    args.addAll(List.of("--estimate-memory", "--verbose"));
    java.util.concurrent.CompletableFuture<Integer> done = new java.util.concurrent.CompletableFuture<>();
    ChildRun estimate =
        ChildRun.start(
            args,
            new ChildRun.Listener() {
              public void line(String line) {}

              public void finished(int code, boolean stopped) {
                done.complete(code);
              }
            });
    check(done.get(300, java.util.concurrent.TimeUnit.SECONDS) == 0, "memory estimate runs");
    List<String> plan = GuiOptions.estimateLines(estimate.tail());
    check(plan.stream().anyMatch(l -> l.startsWith("Quality=")) && plan.stream().anyMatch(l -> l.contains("disparities")), "memory plan shown: " + plan);
    check(plan.stream().noneMatch(l -> l.startsWith("Pipeline step")), "step chatter left out");
    check(ProjectFiles.runIds(project).size() == 2, "the estimate leaves the project alone");
    PoseLauncherPanel.deleteTree(scratch);
    check(!Files.exists(scratch), "scratch project removed");
    return checks - before;
  }

  static void process() throws IOException {
    // Child command: same java, class path, entry point; arguments with spaces and Unicode kept whole.
    List<String> args = List.of("inputs", "-i", "C:/kuvat ja muut/Ääni", "-o", "/tmp/pro ject");
    List<String> c = PoseProcess.command(args, true);
    check(c.get(0).equals(PoseProcess.java().toString()) && Files.isRegularFile(PoseProcess.java()), "child uses this java");
    int cp = c.indexOf("-cp");
    check(cp > 0 && c.get(cp + 1).equals(System.getProperty("java.class.path")) && c.get(cp + 2).equals("photogrammetry"), "same class path, CLI entry point");
    check(c.subList(cp + 3, c.size()).equals(args), "arguments passed as they are");
    check(c.contains("-Dstdout.encoding=UTF-8") && !PoseProcess.command(args, false).contains("-Dstdout.encoding=UTF-8"), "UTF-8 console only when piped");
    check(c.stream().noneMatch(s -> s.startsWith("-Dsgm.heap.reexecuted")), "the re-exec flag is not inherited (the launcher may re-execute)");
    check(!PoseProcess.inherited("-agentlib:jdwp=transport=dt_socket") && !PoseProcess.inherited("-Dsgm.heap.reexecuted=true") && PoseProcess.inherited("-Xmx8g")
        && PoseProcess.inherited("--enable-native-access=ALL-UNNAMED"), "inherited JVM options");
    // Output lines: newline and carriage-return (progress redraw) both end a line.
    List<String> lines = new ArrayList<>();
    PoseProcess.lines(new StringReader("a\r\nb\rprogress 10%\rprogress 20%\nc"), lines::add);
    check(lines.equals(List.of("a", "b", "progress 10%", "progress 20%", "c")), "line splitting: " + lines);
    // Display quoting.
    check(PoseProcess.display(List.of("dense", "-o", "C:/a b"), true).equals("photogrammetry dense -o \"C:/a b\""), "Windows quoting");
    check(PoseProcess.display(List.of("-o", "/x/it's here"), false).equals("photogrammetry -o '/x/it'\\''s here'"), "POSIX quoting");
    check(PoseProcess.quote("", false).equals("''") && PoseProcess.quote("plain-arg_1.5", true).equals("plain-arg_1.5"), "empty and plain arguments");
    // gcp-edit's own flags are removed before the processing options are parsed.
    Main.Command cmd = GcpEditor.withoutEditorFlags(Main.command(new String[] {"gcp-edit", "-o", "p", "--browser", "--no-continue", "--run", "R1"}));
    check(cmd.editor() && Arrays.asList(cmd.rest()).equals(List.of("-o", "p")) && "R1".equals(cmd.runId()), "editor flags stripped");
    check(GuiSupport.HEADLESS.contains("photogrammetry --help"), "headless message points to the CLI");
    // Window icon: the repository's POSE logo in the usual sizes, transparent around the hexagon.
    List<java.awt.Image> icons = GuiSupport.icons();
    check(icons.size() == 9, "POSE icon in nine sizes");
    java.awt.image.BufferedImage small = (java.awt.image.BufferedImage) icons.get(0), large = (java.awt.image.BufferedImage) icons.get(icons.size() - 1);
    check(small.getWidth() == 16 && small.getHeight() == 16 && large.getWidth() == 256, "icon sizes 16 to 256 px");
    check((small.getRGB(0, 0) >>> 24) < 32 && (large.getRGB(128, 128) >>> 24) == 0 || (large.getRGB(60, 128) >>> 24) > 200, "icon keeps its transparency and shape");
    // 'photogrammetry gui' without a display: the message, exit code 2 (through the launcher).
    Process headless =
        new ProcessBuilder(PoseProcess.java().toString(), "-Djava.awt.headless=true", "--enable-native-access=ALL-UNNAMED", "-cp", System.getProperty("java.class.path"), "photogrammetry", "gui")
            .redirectErrorStream(true)
            .start();
    String text = new String(headless.getInputStream().readAllBytes(), java.nio.charset.StandardCharsets.UTF_8);
    try {
      check(headless.waitFor() == 2 && text.contains("No display"), "headless gui exits with the CLI pointer: " + text);
    } catch (InterruptedException e) {
      throw new IOException(e);
    }
  }

  /** A child that starts a grandchild: stopping the child stops both. */
  static void tree() throws Exception {
    List<String> command = List.of(PoseProcess.java().toString(), "-cp", System.getProperty("java.class.path"), "sgm.GuiRegression", "sleeper", "spawn");
    Process child = new ProcessBuilder(command).redirectErrorStream(true).redirectOutput(ProcessBuilder.Redirect.DISCARD).start();
    List<ProcessHandle> descendants = List.of();
    for (int i = 0; i < 200 && descendants.isEmpty(); i++) {
      Thread.sleep(100);
      descendants = child.toHandle().descendants().toList();
    }
    check(!descendants.isEmpty(), "test child started a grandchild");
    PoseProcess.terminate(child.toHandle(), 3000);
    child.waitFor(10, java.util.concurrent.TimeUnit.SECONDS);
    check(!child.isAlive(), "child stopped");
    for (ProcessHandle d : descendants) {
      d.onExit().get(10, java.util.concurrent.TimeUnit.SECONDS);
      check(!d.isAlive(), "grandchild stopped");
    }
  }

  static void sleeper(boolean spawn) throws Exception {
    if (spawn)
      new ProcessBuilder(PoseProcess.java().toString(), "-cp", System.getProperty("java.class.path"), "sgm.GuiRegression", "sleeper")
          .redirectErrorStream(true)
          .redirectOutput(ProcessBuilder.Redirect.DISCARD)
          .start();
    Thread.sleep(120_000);
  }

  static void editorModel() {
    Map<String, Object> state = new LinkedHashMap<>();
    state.put("images", List.of(image(0, "a.jpg", true), image(1, "b.jpg", true), image(2, "c.jpg", true), image(3, "d.jpg", false), image(4, "e.jpg", true)));
    state.put(
        "markers",
        List.of(
            marker("G1", 0, "predicted", 10, 10, false, null, null),
            marker("G1", 1, "refined", 11, 12, false, .9, null),
            marker("G1", 2, "pinned", 13, 14, true, null, 0.4),
            marker("G1", 4, "refined", 15, 16, true, .95, null),
            marker("G2", 0, "rejected", 1, 1, false, null, null),
            marker("G2", 1, "refined", 2, 2, false, .5, null)));
    state.put("gcps", List.of(Map.of("id", "G1", "type", "control", "pins", 1, "pin_rms", 0.4, "pin_max", 0.4), Map.of("id", "G2", "type", "check", "pins", 0)));
    state.put("ready", false);
    state.put("needed", "2 more control points");
    state.put("refining", true);
    state.put("accept", .85);
    GcpEditorModel m = new GcpEditorModel(state);
    check(m.gcps.size() == 2 && m.gcp("G1").markers() == 4 && m.gcp("G1").refined() == 2, "GCP rows count markers and refined ones");
    check(m.shown("G1").stream().map(GcpEditorModel.Marker::image).toList().equals(List.of(2, 1, 0)), "pinned (even if hidden), refined, predicted");
    check(m.hidden("G1").stream().map(GcpEditorModel.Marker::image).toList().equals(List.of(4)), "probably hidden refined marker in its own section");
    check(m.cards("G1").stream().map(GcpEditorModel.Marker::image).toList().equals(List.of(2, 1, 0, 4)), "browsing order");
    check(m.others("G1").isEmpty() && m.others("G2").stream().map(GcpEditorModel.Image::id).toList().equals(List.of(2, 4)), "other registered images without a marker");
    check(m.acceptable("G1") == 2 && m.acceptable("G2") == 0, "accept all counts refined markers at or above the threshold");
    check(m.refining && !m.ready && m.readiness().equals("Still needed: 2 more control points"), "status flags");
    check(GcpEditorModel.info(m.marker("G1", 1)).equals("refined 0.90"), "refined info: " + GcpEditorModel.info(m.marker("G1", 1)));
    check(GcpEditorModel.info(m.marker("G1", 2)).equals("pinned, 0.40 px"), "pinned info");
    check(GcpEditorModel.info(m.marker("G1", 4)).equals("refined 0.95, probably hidden"), "hidden info");
    check(GcpEditorModel.summary(m.gcp("G1")).equals("pin RMS 0.40 px, max 0.40 px · 2 refined") && GcpEditorModel.summary(m.gcp("G2")).equals("1 refined"), "GCP summary");
    check(GcpEditorWindow.infoHtml(m.marker("G1", 2)).contains("<font color='#16a34a'>0.40 px</font>"), "residual coloured by grade");
  }

  static Map<String, Object> image(int id, String name, boolean registered) {
    return Map.of("id", id, "name", name, "width", 400, "height", 300, "registered", registered);
  }

  static Map<String, Object> marker(String gcp, int image, String status, double u, double v, boolean hidden, Double score, Double residual) {
    Map<String, Object> m = new HashMap<>();
    m.put("gcp", gcp);
    m.put("image", image);
    m.put("status", status);
    m.put("u", u);
    m.put("v", v);
    m.put("predicted_u", u - 1);
    m.put("predicted_v", v - 1);
    m.put("hidden", hidden);
    m.put("score", score);
    m.put("residual", residual);
    return m;
  }

  static void files(Path root) throws IOException {
    Path dir = Files.createDirectories(root.resolve("kansio ä"));
    Path file = Files.writeString(dir.resolve("raportti ö.pdf"), "x");
    check(GuiSupport.openCommand(dir, "Windows 11").equals(List.of("explorer.exe", dir.toString())), "explorer for folders");
    check(GuiSupport.openCommand(file, "Windows 11").get(2).equals(file.toString()), "file opened by its default application on Windows");
    check(GuiSupport.openCommand(file, "Linux").equals(List.of("xdg-open", file.toString())), "xdg-open on Linux");
  }
}
